"""Oriented mesh volume, topology and an independent Fubini column integral."""
import math
from dataclasses import dataclass

import numpy as np
import open3d as o3d


@dataclass
class Surface:
    vertices: np.ndarray
    faces: np.ndarray

    @classmethod
    def of(cls, mesh):
        return cls(np.asarray(mesh.vertices).copy(), np.asarray(mesh.triangles).copy())

    @classmethod
    def load(cls, path):
        m = o3d.io.read_triangle_mesh(str(path))
        if not m.has_triangles():
            raise ValueError(f'Expected a triangle mesh: {path}')
        return cls.of(m)

    def legacy(self):
        return o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(self.vertices),
                                         o3d.utility.Vector3iVector(self.faces.astype(np.int32)))

    def scene(self):
        context = o3d.t.geometry.RaycastingScene(nthreads=1)
        context.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(self.legacy()))
        return context

    def signed(self):
        a, b, c = (self.vertices[self.faces]-self.vertices.mean(axis=0)).transpose(1, 0, 2)
        return math.fsum(np.einsum('ij,ij->i', a, np.cross(b,c)))/6.

    def g(self):
        t = self.vertices[self.faces]
        cross = np.cross(t[:,1]-t[:,0], t[:,2]-t[:,0])
        area = np.linalg.norm(cross, axis=1)/2
        normal = np.divide(cross, 2*area[:,None], out=np.zeros_like(cross), where=area[:,None]>0)
        return t, area, normal

    def topology(self):
        e = self.faces[:, [[0,1],[1,2],[2,0]]].reshape(-1,2)
        keys, i, counts = np.unique(np.sort(e,axis=1),axis=0,return_inverse=True,return_counts=True)
        orient = np.bincount(i, weights=np.where(e[:,0]<e[:,1],1,-1), minlength=len(keys))
        m = self.legacy()
        labels, sizes, _ = m.cluster_connected_triangles()
        labels = np.asarray(labels)
        chis, volumes = [], []
        for k in range(len(sizes)):
            f = self.faces[labels==k]
            used, inverse = np.unique(f,return_inverse=True)
            part = Surface(self.vertices[used],inverse.reshape(f.shape))
            e0 = np.sort(f[:, [[0,1],[1,2],[2,0]]].reshape(-1,2),axis=1)
            chis.append(int(len(used)-len(np.unique(e0,axis=0))+len(f)))
            volumes.append(part.signed())
        return dict(boundary=int(sum(counts==1)), nonmanifold_edges=int(sum(counts>2)),
                    orientation_errors=int(sum((counts==2)&(orient!=0))),
                    vertex_manifold=m.is_vertex_manifold(), degenerate=int(sum(self.g()[1]<1e-12)),
                    components=len(sizes), chi=chis, component_signed_mm3=volumes)


def tetra_gradient(s):
    """Gradient of fixed-connectivity signed volume at every vertex."""
    t = s.vertices[s.faces]-s.vertices.mean(axis=0)
    out=np.zeros_like(s.vertices,dtype=float)
    for i in range(3):
        np.add.at(out,s.faces[:,i],np.cross(t[:,(i+1)%3],t[:,(i+2)%3])/6.)
    return out


def checked_volume(s):
    t=s.topology()
    if (t['boundary'] or t['orientation_errors'] or t['nonmanifold_edges'] or
            not t['vertex_manifold'] or t['degenerate'] or s.signed()<=0):
        raise ValueError(f'Unusable oriented surface: {t}')
    return s.signed()


def column_integral(surface, context, pitch, axis, offset):
    """Sum the intervals between alternating sorted ray crossings over a shifted XY grid."""
    other=[j for j in range(3) if j!=axis]
    base=surface.vertices.min(axis=0)
    end=surface.vertices.max(axis=0)
    coord=[np.arange(base[j]+pitch*o,end[j],pitch) for j,o in zip(other,offset)]
    grid=np.stack(np.meshgrid(*coord,indexing='ij'),axis=-1).reshape(-1,2)
    total=0.
    odd=0
    hits=0
    multiple=0
    for xy in np.array_split(grid,max(1,int(np.ceil(len(grid)/32000)))):
        if not len(xy):
            continue
        rays=np.zeros((len(xy),6),np.float32)
        rays[:,other]=xy
        rays[:,axis]=base[axis]-2.
        rays[:,3+axis]=1.
        result=context.list_intersections(o3d.core.Tensor(rays),nthreads=1)
        rid=result['ray_ids'].numpy().astype(int)
        dist=result['t_hit'].numpy().astype(float)
        order=np.lexsort((dist,rid))
        rid,dist=rid[order],dist[order]
        if len(rid)==0:
            continue
        keep=np.r_[True,(np.diff(rid)!=0)|(np.diff(dist)>2e-5)]
        rid,dist=rid[keep],dist[keep]
        count=np.bincount(rid,minlength=len(xy))
        hits+=int(sum(count>0))
        multiple+=int(sum(count>2))
        odd+=int(sum(count%2))
        start=(np.cumsum(count)-count)[rid]
        even=(count[rid]%2)==0
        sign=2*((np.arange(len(rid))-start)%2)-1
        total+=float(np.dot(sign[even],dist[even]))
    return dict(pitch_mm=pitch,axis='xyz'[axis],offset_a=offset[0],offset_b=offset[1],
                column_volume_mm3=total*pitch*pitch,odd_rays=odd,hit_rays=hits,multi_interval_rays=multiple)


def closed_candidate(s, *, self_intersections=True):
    try:
        v=checked_volume(s)
    except ValueError:
        return None
    if self_intersections and s.legacy().is_self_intersecting():
        return None
    return v
