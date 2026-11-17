"""Compare a full-turn repeat; distances describe agreement, not accuracy."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read_points(path):
    with path.open('rb') as f:
        header = b''
        while not header.endswith(b'end_header\n'):
            line = f.readline()
            if not line:
                raise ValueError('Incomplete PLY header')
            header += line
        if b'property float x' not in header or b'property uchar blue' not in header:
            raise ValueError('Expected binary float XYZ and RGB PLY')
        return np.fromfile(f, dtype=[('xyz', '<f4', 3), ('rgb', 'u1', 3)])['xyz'].astype(float)


def write_cloud(path, first, second):
    p = np.concatenate([first, second]); colors = np.repeat(np.array([[32, 112, 220], [244, 124, 24]], dtype=np.uint8), [len(first), len(second)], axis=0)
    r = np.empty(len(p), dtype=[('xyz', '<f4', 3), ('rgb', 'u1', 3)])
    r['xyz'], r['rgb'] = p, colors
    with path.open('wb') as f:
        f.write(('ply\nformat binary_little_endian 1.0\ncomment Experimental repeat comparison camera coordinates\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n' % len(p)).encode('ascii'))
        r.tofile(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scan', type=Path)
    ap.add_argument('calibration', type=Path)
    ap.add_argument('--roi', nargs=4, type=int, required=True, metavar=('X1','Y1','X2','Y2'))
    ap.add_argument('--depth', nargs=2, type=float, default=[345, 390])
    args = ap.parse_args()
    xml = args.calibration.read_text(encoding='utf-8')
    fs = cv2.FileStorage(xml, cv2.FILE_STORAGE_READ | cv2.FILE_STORAGE_MEMORY)
    K, dist = fs.getNode('cam_int').mat(), fs.getNode('cam_dist').mat()
    fs.release()
    def load(view):
        points = read_points(args.scan / view / 'box_view_10mm.ply')
        uv = cv2.projectPoints(points, np.zeros(3), np.zeros(3), K, dist)[0].reshape(-1, 2)
        x1, y1, x2, y2 = args.roi
        mask = ((uv[:,0]>=x1)&(uv[:,0]<x2)&(uv[:,1]>=y1)&(uv[:,1]<y2)&(points[:,2]>=args.depth[0])&(points[:,2]<=args.depth[1]))
        return points[mask]
    a,b=load('view_0000'),load('view_0004')
    if min(len(a),len(b)) < 200: raise ValueError('Too few points in selected face')
    rng=np.random.default_rng(732)
    # Fix one quarter of points as a holdout for evaluation; same spatial face, not an independent acquisition.
    a=a[rng.permutation(len(a))]; b=b[rng.permutation(len(b))]
    at,ae=np.split(a,[len(a)*3//4]); bt,be=np.split(b,[len(b)*3//4])
    at=at[:12000]; bt=bt[:12000]
    tree=cKDTree(at)
    def move(p,q):
        angle=q[0]; c,s=np.cos(angle),np.sin(angle)
        R=np.array([[c,0,s],[0,1,0],[-s,0,c]])
        center=np.array([0.,0.,365.])
        return (p-center)@R.T+center+q[1:]
    def residual(q):
        transformed=move(bt,q); d,idx=tree.query(transformed,workers=-1)
        mask=d<8
        if mask.sum()<100: return np.ones(300)*100
        # Scalar distances retain fixed output length when correspondences change.
        return np.minimum(d,8)
    q=np.zeros(4)
    result=least_squares(residual,q,bounds=([-0.1745,-15,-15,-15],[0.1745,15,15,15]),loss='soft_l1',f_scale=2,max_nfev=50,diff_step=1e-4)
    q=result.x
    def metrics(p,r):
        d=cKDTree(r).query(p,workers=-1)[0]
        return {'median_mm':float(np.median(d)),'p90_mm':float(np.percentile(d,90)), 'within_3mm_fraction':float(np.mean(d<3)), 'count':len(d)}
    before=metrics(be,ae); after=metrics(move(be,q),ae)
    reverse_before=metrics(ae,be); reverse_after=metrics(ae,move(be,q))
    output={'roi_pixels':args.roi,'depth_mm':args.depth,'face_point_counts':[len(a),len(b)],'nominal_square_size_mm':10,'holdout_0004_to_0000_before':before,'holdout_0004_to_0000_after':after,'holdout_0000_to_0004_before':reverse_before,'holdout_0000_to_0004_after':reverse_after,'fit_yaw_degrees':float(np.degrees(q[0])),'fit_translation_mm':q[1:].tolist(),'fit_rotation_center_camera_mm':[0,0,365], 'note':'The fit aligns one nearly planar face only. Rotation and in-plane translation are ambiguous; a held-out point subset is not an independent scan. Scale and calibration unverified; transform must not be interpreted as measured turntable error or applied to other views without independent evidence.'}
    out=args.scan/'loop_0000_0004_analysis.json'; out.write_text(json.dumps(output,ensure_ascii=False,indent=2),encoding='utf-8')
    write_cloud(args.scan/'loop_0000_0004_before.ply', a,b)
    write_cloud(args.scan/'loop_0000_0004_fitted.ply', a,move(b,q))
    fig,axes=plt.subplots(1,3,figsize=(16,5),constrained_layout=True)
    for ax, pts, title in [(axes[0],b,'Before fit'),(axes[1],move(b,q),'After constrained fit')]:
        ax.scatter(a[::4,0],a[::4,2],s=.5,c='#2472d4',label='0000',alpha=.45)
        ax.scatter(pts[::4,0],pts[::4,2],s=.5,c='#f08018',label='0004',alpha=.45)
        ax.set_xlabel('Camera X (mm)'); ax.set_ylabel('Camera Z (mm)'); ax.set_title(title+' — top view'); ax.legend(markerscale=8); ax.grid(alpha=.25); ax.set_aspect('equal')
    d0=cKDTree(ae).query(be)[0]; d1=cKDTree(ae).query(move(be,q))[0]
    axes[2].hist(d0,bins=np.linspace(0,15,51),alpha=.55,label='Before',density=True)
    axes[2].hist(d1,bins=np.linspace(0,15,51),alpha=.55,label='After',density=True)
    axes[2].set(xlabel='Nearest distance on held-out points (mm)',ylabel='Density',title='Repeat face agreement'); axes[2].legend(); axes[2].grid(alpha=.25)
    fig.suptitle('Experimental loop repeat comparison: 0000 and 0004 (not accuracy)')
    fig.savefig(args.scan/'loop_0000_0004_comparison.png',dpi=170); plt.close(fig)
    print(json.dumps(output,indent=2,ensure_ascii=False))

if __name__ == '__main__': main()
