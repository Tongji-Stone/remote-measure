"""Compare calibrated bottom grafts and constrained denoising on exported meshes."""
import argparse
import csv
import json

import numpy as np
import open3d as o3d
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

from audit_board import overlap_checker
from experiment import HERE, metric, real_input, sample_frames, save, split
from reconstruct import envelope, footprint, grid_box, mesh, oriented, rays, scene, screened
from verify_contact import check

OUTPUT = HERE / '网格优化'
SOURCE = HERE / '场景/F1-ChArUco'
T = o3d.core.Tensor


def read(path):
    model = o3d.io.read_triangle_mesh(str(path))
    if not len(model.triangles):
        raise ValueError(f'Empty mesh: {path}')
    return model


def edges(faces):
    directed = faces[:, [[0, 1], [1, 2], [2, 0]]].reshape(-1, 2)
    keys, inverse, counts = np.unique(np.sort(directed, axis=1), axis=0,
                                      return_inverse=True, return_counts=True)
    return keys, counts, directed[counts[inverse] == 1]


def boundary_groups(model):
    v, f = np.asarray(model.vertices), np.asarray(model.triangles)
    _, counts, boundary = edges(f)
    if not len(boundary):
        return int(sum(counts > 2)), []
    used, inverse = np.unique(boundary, return_inverse=True)
    graph = coo_matrix((np.ones(len(boundary)), inverse.reshape(-1, 2).T),
                       shape=(len(used), len(used)))
    count, groups = connected_components(graph, directed=False)
    return int(sum(counts > 2)), [
        (len(verts), *np.percentile(v[verts, 2], [0, 50, 100]).tolist())
        for label in range(count) if len(verts := used[groups == label])
    ]


def single_boundary(model):
    _, counts, boundary = edges(np.asarray(model.triangles))
    successor = dict(boundary)
    if (not len(boundary) or np.any(counts > 2) or len(successor) != len(boundary)
            or len(np.unique(boundary[:, 1])) != len(boundary)):
        raise ValueError('Cut boundary is not an oriented ring.')
    start = int(boundary[0, 0])
    ring = [start]
    for _ in range(len(boundary) - 1):
        nxt = successor[ring[-1]]
        if nxt == start:
            raise ValueError('Cut has multiple rings.')
        ring.append(nxt)
    if successor[ring[-1]] != start:
        raise ValueError('Cut ring is open.')
    return np.array(ring)


def arc(points):
    length = np.linalg.norm(np.diff(np.r_[points, points[:1]], axis=0), axis=1)
    return np.r_[0., np.cumsum(length)] / length.sum()


def graft(surface, base, low, high):
    cuts = [o3d.t.geometry.TriangleMesh.from_legacy(m).clip_plane(
        T([0., 0., z]), T([0., 0., direction])).to_legacy()
        for m, z, direction in ((surface, high, 1.), (base, low, -1.))]
    upper, lower = cuts
    u, l = single_boundary(upper), single_boundary(lower)[::-1]
    a, b = np.asarray(upper.vertices), np.asarray(lower.vertices)
    # Pick the closest cyclic alignment over the whole contour, then zip by arc length.
    up_arc = arc(a[u])
    target = a[u[::max(1, len(u)//80)]]
    t = up_arc[:len(u):max(1, len(u)//80)]
    scores = []
    for start in range(len(l)):
        q = b[np.roll(l, -start)]
        q = np.r_[q, q[:1]]
        values = np.column_stack([np.interp(t, arc(q[:-1]), q[:, k]) for k in (0, 1)])
        scores.append(np.mean(np.sum((values - target[:, :2])**2, axis=1)))
    l = np.roll(l, -int(np.argmin(scores)))
    lower_arc = arc(b[l])
    l = l + len(a)
    i = j = 0
    stitch = []
    while i < len(u) or j < len(l):
        uu, ll = u[i % len(u)], l[j % len(l)]
        if j == len(l) or (i < len(u) and up_arc[i+1] <= lower_arc[j+1]):
            stitch.append((u[(i+1) % len(u)], uu, ll))
            i += 1
        else:
            stitch.append((uu, ll, l[(j+1) % len(l)]))
            j += 1
    model = mesh(np.r_[a, b], np.r_[np.asarray(upper.triangles),
                 np.asarray(lower.triangles) + len(a), np.array(stitch)])
    return model, float(np.sqrt(min(scores)))


def relax_graft(source, bound=.25):
    """Resolve local self-intersections above the seam with capped one-ring relaxation."""
    v0 = np.asarray(source.vertices)
    v = v0.copy()
    f = np.asarray(source.triangles)
    keys, _, _ = edges(f)
    i, j = np.r_[keys, keys[:, ::-1]].T
    for _ in range(4):
        model = mesh(v, f)
        pairs = np.asarray(model.get_self_intersecting_triangles())
        if not len(pairs):
            change = np.linalg.norm(v-v0, axis=1)
            return model, float(change.max()), int((change>0).sum())
        mask = np.zeros(len(v), bool)
        mask[f[pairs.ravel()]] = True
        mask[f[mask[f].any(axis=1)]] = True
        mask &= v0[:, 2] > 6.45
        e = v[j] - v[i]
        w = 1./np.maximum(np.linalg.norm(e, axis=1), .05)
        total = np.bincount(i, weights=w, minlength=len(v))
        dx = np.column_stack([np.bincount(i, weights=w*d, minlength=len(v))
                              for d in e.T]) / np.maximum(total[:, None], 1e-12)
        delta = v[mask]+.45*dx[mask]-v0[mask]
        norm = np.linalg.norm(delta, axis=1)
        v[mask] = v0[mask]+delta*np.minimum(1., bound/np.maximum(norm, 1e-12))[:, None]
    raise ValueError('Local relaxation did not eliminate all graft intersections.')


def denoise(source, method, iterations, bound, floor, scale=1.):
    v0 = np.asarray(source.vertices).copy()
    f = np.asarray(source.triangles)
    keys, _, _ = edges(f)
    i, j = np.r_[keys, keys[:, ::-1]].T
    count = len(v0)
    v = v0.copy()
    ramp = np.clip((v0[:, 2] - floor - 1.5)/2.5, 0., 1.)
    ramp = ramp**2 * (3.-2.*ramp)
    for multiplier in ([.5, -.53] * iterations if method == 'taubin' else [.8]*iterations):
        e = v[j] - v[i]
        spatial = np.linalg.norm(e, axis=1)
        if method == 'taubin':
            delta = e
            w = 1./np.maximum(spatial, .05)
        else:
            normal = np.asarray(mesh(v, f).compute_vertex_normals().vertex_normals).copy()
            height = np.einsum('ij,ij->i', e, normal[i])
            w = np.exp(-.5*(spatial/(1.18*scale))**2 - .5*(height/(.54*scale))**2)
            delta = height[:, None]*normal[i]
        total = np.bincount(i, weights=w, minlength=count)
        mean = np.column_stack([np.bincount(i, weights=w*d, minlength=count)
                                for d in delta.T]) / np.maximum(total[:, None], 1e-12)
        change = v + multiplier*ramp[:, None]*mean - v0
        norm = np.linalg.norm(change, axis=1)
        v = v0 + change*np.minimum(1., bound/np.maximum(norm, 1e-12))[:, None]
    return mesh(v, f)


def preserve_topology(source, candidate):
    """Freeze the neighborhoods of any new intersections and recheck the geometry."""
    v0 = np.asarray(source.vertices)
    v = np.asarray(candidate.vertices).copy()
    f = np.asarray(source.triangles)
    fixed = np.zeros(len(v), bool)
    for _ in range(5):
        m = mesh(v, f)
        pairs = np.asarray(m.get_self_intersecting_triangles())
        if not len(pairs):
            return m, int(fixed.sum())
        fixed[f[pairs.ravel()]] = True
        fixed[f[fixed[f].any(axis=1)]] = True
        v[fixed] = v0[fixed]
    raise ValueError('Local rollback could not eliminate candidate intersections.')


def fine_envelope(frames, cal):
    train, _, _, _ = split(frames, 'T0')
    p, n = oriented(train)
    if len(p) > 27000:
        i = np.random.default_rng(774).choice(len(p), 27000, replace=False)
        p, n = p[i], n[i]
    guide, gn = footprint(p)
    free, _, _ = rays(guide, train, cal)
    gn /= np.linalg.norm(gn, axis=1, keepdims=True)
    guide, gn = guide[free < 2], gn[free < 2]
    guided = screened(np.r_[p, guide], np.r_[n, gn])[0]
    pitch = .75
    lo, size, grid = grid_box(np.r_[p, guide], pitch)
    free, _, _ = rays(grid, train, cal)
    return envelope(guided, p, guide, lo, size, grid, free, pitch, exposed=cal['exposed_board'])


def fixed_samples(reference, lower, upper, radius=.8):
    sc = scene(reference)
    o3d.utility.random.seed(1225)
    s = np.asarray(reference.sample_points_uniformly(18000).points)
    s = s[(s[:, 2] >= lower) & (s[:, 2] <= upper)]
    rng = np.random.default_rng(170)
    d = rng.normal(size=s.shape)
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    return np.r_[s-d*radius/2, s+d*radius/2].astype(np.float32)


def normal_p50(model, probe):
    m = model.compute_vertex_normals()
    f = np.asarray(m.triangles)
    n = np.asarray(m.vertex_normals)
    out = scene(m).compute_closest_points(T(probe), nthreads=1)
    a = out['primitive_uvs'].numpy()
    interpolated = np.einsum('ij,ijk->ik', np.c_[1.-a.sum(1), a], n[f[out['primitive_ids'].numpy()]])
    interpolated /= np.maximum(np.linalg.norm(interpolated, axis=1, keepdims=True), 1e-12)
    left, right = np.split(interpolated, 2)
    angle = np.rad2deg(np.arccos(np.clip(np.einsum('ij,ij->i', left, right), -1., 1.)))
    return np.percentile(angle, [50, 95])


def fixed_surface_motion(base, other):
    v = np.asarray(base.vertices)
    w = np.asarray(other.vertices)
    d = np.linalg.norm(v-w, axis=1)
    return [float(np.percentile(d, 95)), float(d.max())]


def protected_minion():
    src = HERE.parent/'E003/results/E003-S01'
    base = read(src/'E003-S01-M05-T0.ply')
    probe = fixed_samples(base, 46., 73.)
    samples = np.asarray(o3d.io.read_point_cloud(str(src/'E003-S01-C00.ply')).points)
    if len(samples) > 20000:
        samples = samples[np.linspace(0, len(samples)-1, 20000).astype(int)]
    z = np.asarray(base.vertices)[:, 2].min()
    rows = []
    for iterations, bound in ((0, 0.), (3, .25), (8, .72)):
        method = 'M05' if not iterations else f'M05-B{iterations}'
        out = OUTPUT/f'E003-S01-{method}.ply'
        m, stopped = base, 0
        if iterations:
            m, stopped = preserve_topology(base, denoise(base, 'bilateral', iterations, bound, z, .65))
            save(out, m)
            m = read(out)
        q = scene(m).compute_distance(T(samples.astype('float32')), nthreads=1).numpy()
        closed = m.is_watertight()
        rows.append(dict(method=method, normal_p50_deg=normal_p50(m, probe)[0],
                         observed_fit_p95_mm=np.percentile(q, 95), closed=closed,
                         volume_ml=m.get_volume()/1000. if closed else None,
                         min_z_mm=np.asarray(m.vertices)[:, 2].min(), max_motion_limit_mm=bound, rollback_vertices=stopped))
    write_rows(OUTPUT/'E003-S01.csv', rows)
    best = min((r for r in rows[1:] if r['closed'] and
                r['observed_fit_p95_mm'] <= rows[0]['observed_fit_p95_mm'] + .1),
               key=lambda r:r['normal_p50_deg'], default=rows[0])
    return 'E003-S01-'+best['method']


def write_rows(path, rows):
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w', newline='') as out:
        f = csv.DictWriter(out, fields)
        f.writeheader()
        f.writerows(rows)


def main(fine=False):
    OUTPUT.mkdir(exist_ok=True)
    raw = read(SOURCE/'表面优选.ply')
    visible = read(SOURCE/'T0-S01-visible.ply')
    base = read(SOURCE/'封闭候选.ply')
    bounds = []
    for form, model in [('S01', raw), ('S01-visible', visible), ('S04', base)]:
        nonmanifold, groups = boundary_groups(model)
        pairs = np.asarray(model.get_self_intersecting_triangles())
        bounds.append(dict(form=form, intersecting_pairs=len(pairs), boundary_group_count=len(groups),
                           boundary_edge_count=len(edges(np.asarray(model.triangles))[2]),
                           nonmanifold_edges=nonmanifold, z_ranges_mm=groups))
    write_rows(OUTPUT/'边界.csv', bounds)
    frames, _, cal, plane, _, _, _ = real_input('F1', 'ChArUco', with_board_evidence=True)
    train, test, _, _ = split(frames, 'T0')
    train, _ = sample_frames(train, each=2600)
    probe = fixed_samples(base, 41., 60.)
    foot_probe = fixed_samples(base, 4., 13.)
    models = [('S01', raw, {}), ('S04', base, {})]
    for low,high in [(2.25, 3.45), (5.25, 6.45), (7.25, 8.45)]:
        name = f'接底-{low:g}'
        try:
            h, seam = graft(raw, base, low, high)
            models.append((name, h, {'seam_rms_mm': seam, 'upper_z':high}))
        except (RuntimeError, ValueError) as exc:
            models.append((name, None, {'error':str(exc)}))
    seam_model = next(model for name, model, _ in models if name=='接底-5.25')
    repaired, max_motion, modified = relax_graft(seam_model)
    models.append(('接底局部修复', repaired, {'motion_max_mm': max_motion, 'locally_moved_vertices': modified}))
    for name, method, passes, bound in [('Taubin', 'taubin', 12, .72),
                                        ('双边-8', 'bilateral', 8, .72),
                                        ('双边-20', 'bilateral', 20, .95)]:
        d = denoise(base, method, passes, bound, 0.)
        if method == 'bilateral':
            kept, stopped = preserve_topology(base, d)
            models.append((name+'（未回退）', d, {}))
            d = kept
        else:
            stopped = 0
        p95, mx = fixed_surface_motion(base, d)
        models.append((name, d, {'motion_p95_mm':p95, 'motion_max_mm':mx, 'rollback_vertices':stopped}))
    fine_path = OUTPUT/'细网格.ply'
    if fine or fine_path.exists():
        fine_model = fine_envelope(frames, cal) if fine else read(fine_path)
        models.append(('细网格', fine_model, {}))
        detail, stopped = preserve_topology(fine_model, denoise(fine_model, 'bilateral', 12, .60, 0., .68))
        a, b = fixed_surface_motion(fine_model, detail)
        models.append(('细网格双边', detail, {'motion_p95_mm':a, 'motion_max_mm':b, 'rollback_vertices':stopped}))
    rows = []
    for name, model, extra in models:
        row = dict(method=name, **extra)
        if model is not None:
            base_dir = {'S01':SOURCE/'表面优选.ply', 'S04':SOURCE/'封闭候选.ply'}.get(name)
            if base_dir is None and '未回退' not in name:
                path = OUTPUT/f'{name}.ply'
                save(path, model)
                model = read(path)
            row.update(metric(model, train, test, []))
            row['intersecting_pairs'] = len(model.get_self_intersecting_triangles())
            row['boundary_edges'] = len(edges(np.asarray(model.triangles))[2])
            row['head_normal_p50_deg'], row['head_normal_p95_deg'] = normal_p50(model, probe)
            row['shoe_normal_p50_deg'], _ = normal_p50(model, foot_probe)
            if row['watertight']:
                try:
                    check(model)
                    row['calibrated_cap_pass'] = True
                except AssertionError:
                    row['calibrated_cap_pass'] = False
        rows.append(row)
        print(name, {k:row.get(k) for k in ('status','watertight','calibrated_cap_pass',
               'observed_p95','head_normal_p50_deg','volume_mm3','seam_rms_mm')}, flush=True)
    write_rows(OUTPUT/'坐姿人偶.csv', rows)
    old_minion = protected_minion()
    candidates = {r['method']:r for r in rows}
    selected = '细网格双边'
    if selected not in candidates:
        selected = '双边-8'
    row = candidates[selected]
    reference = candidates['S04']
    assert candidates['接底局部修复']['calibrated_cap_pass']
    assert row['calibrated_cap_pass']
    assert row['observed_p95'] <= reference['observed_p95']
    assert row['head_normal_p50_deg'] < .60 * reference['head_normal_p50_deg']
    assert abs(row['volume_mm3']/reference['volume_mm3'] - 1.) <= .015
    basal_reference = OUTPUT/'细网格.ply' if selected=='细网格双边' else SOURCE/'封闭候选.ply'
    v1 = np.asarray(read(basal_reference).vertices)
    v2 = np.asarray(read(OUTPUT/f'{selected}.ply').vertices)
    assert np.array_equal(v1[v1[:, 2]<=1.5], v2[v1[:, 2]<=1.5])
    xy = np.vstack([r[::3, :2] for r in cal['exposed_board']])
    original = overlap_checker(SOURCE/'封闭候选.ply')(xy)
    candidate = overlap_checker(OUTPUT/f'{selected}.ply')(xy)
    print(selected, 'E102 exposed-board overlap', candidate.mean(), flush=True)
    audit = dict(selected=selected, minion_selected=old_minion, board_points=len(xy),
                 exposed_board_fraction_original=float(original.mean()),
                 exposed_board_fraction_selected=float(candidate.mean()),
                 cap_area_mm2=row['floor_cap_area_mm2'], board_fit_points=plane['deck_points'],
                 overlap_min_inset_mm=1.5, smoothing_min_frozen_z_mm=1.5)
    with (OUTPUT/'复核.json').open('w') as f:
        json.dump(audit, f, ensure_ascii=False, indent=2)


if __name__ == '__main__':
    cli = argparse.ArgumentParser()
    cli.add_argument('--fine', action='store_true')
    main(cli.parse_args().fine)
