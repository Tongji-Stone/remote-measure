"""Separate the square board, object, and background in the E101 captures."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import sys

import cv2 as cv
import numpy as np
from scipy.ndimage import label
from scipy.spatial import cKDTree
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'E101'))
from acquire import BRANCHES, SCANS, directory, load_frame
from geometry import transform
from calibrate import dump

CACHE = Path('/tmp/E102/masks')
DISTANCE = (150.,350.)
BOARD_BOX = (0.,100.)
CROP_MODES = ('E102','E103')
METHODS = ('M01','M02','M03','M04')


def ellipse(radius):
    return cv.getStructuringElement(cv.MORPH_ELLIPSE,(2*radius+1,)*2)


def local_and_range(data):
    q = transform(data['xyz'],np.linalg.inv(data['pose']))
    d = np.linalg.norm(data['xyz'],axis=1)
    return q,(d>=DISTANCE[0])&(d<=DISTANCE[1])


def square(q, margin=0):
    return ((q[:,:2] >= BOARD_BOX[0]-margin)&(q[:,:2]<=BOARD_BOX[1]+margin)).all(1)


def plane_design(q, model):
    x,y=(q[:,:2] / 50 - 1).T
    lin=np.c_[np.ones(len(q)),x,y]
    if model=='ChArUco':
        return lin
    return np.c_[lin,x*x,x*y,y*y]


def fit_board(q, use, model):
    guess=0. if model=='ChArUco' else 12.
    idx=np.flatnonzero(use & (abs(-q[:,2]-guess) < (4. if model=='ChArUco' else 9.)))
    if len(idx)<100:
        raise ValueError(f'Insufficient kit plane samples in {model}')
    a=plane_design(q,model)
    take=idx[::max(1,len(idx)//2300)]
    b=a[take]
    z=-q[take,2]
    prior=np.zeros(b.shape[1]);prior[0]=guess
    solve=least_squares(lambda beta: b@beta-z,prior,loss='soft_l1',f_scale=.45)
    h=-q[:,2]-a@solve.x
    inliers=h[idx]
    s=1.4826*np.median(abs(inliers-np.median(inliers)))
    return h,dict(coefficients=solve.x, support=int(len(idx)), noise_MAD_mm=float(s),
                  median_plane_height_mm=float(np.median(-q[idx,2])))


def connected(q, mask, uv):
    ids=np.flatnonzero(mask)
    if not len(ids):
        return mask.copy()
    grid=np.floor(q[ids]/1.45).astype(int)
    grid-=grid.min(0)
    size=tuple(grid.max(0)+1)
    volume=np.zeros(size,bool)
    volume[tuple(grid.T)]=True
    layers,_=label(volume,structure=np.ones((3,3,3),bool))
    at=layers[tuple(grid.T)]
    size=np.bincount(at)
    size[0]=0
    principal=np.argmax(size)
    main=np.flatnonzero(at==principal)
    nearby=cKDTree(q[ids[main][::3]]).query(q[ids])[0]
    out=np.zeros(len(q),bool)
    out[ids]=(at==principal)|((size[at]>=20)&(nearby<5.5))
    # Structured-light voids can split real parts, e.g. dark neck fabric between head and torso.
    # Admit a substantial component when both its geometry and its image boundary touch the main component.
    cx=cKDTree(uv[ids[main][::2]])
    for component in np.flatnonzero(size>=max(90,.018*len(main))):
        if component==principal:
            continue
        group=np.flatnonzero(at==component)
        image_gap=cx.query(uv[ids[group]])[0]
        if np.percentile(nearby[group],8)<17. and np.percentile(image_gap,8)<25.:
            out[ids[group]]=True
    return out


def small_mask(ref, seed, broad, img):
    """The 3 px grid matches the reconstruction. Work within the object bounding box."""
    x,y=ref['uv'].T.astype(int)//3
    if seed.sum()<80:
        raise ValueError('No stable foreground seed')
    lo=np.maximum([x[seed].min()-18,y[seed].min()-18],[0,0])
    hi=np.minimum([x[seed].max()+19,y[seed].max()+19],
                  [(img.shape[1]+2)//3,(img.shape[0]+2)//3])
    inside=(x>=lo[0])&(x<hi[0])&(y>=lo[1])&(y<hi[1])
    X=(x-lo[0]).clip(0,hi[0]-lo[0]-1)
    Y=(y-lo[1]).clip(0,hi[1]-lo[1]-1)
    small=(img[::3,::3])[lo[1]:hi[1],lo[0]:hi[0]]
    p=np.zeros(small.shape,np.uint8)
    p[Y[seed],X[seed]]=255
    close=cv.morphologyEx(p,cv.MORPH_CLOSE,ellipse(6))
    contours,_=cv.findContours(close,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv.contourArea(contour)>=14:
            cv.drawContours(close,[contour],0,255,cv.FILLED)
    exterior=cv.dilate(close,ellipse(10))
    eroded=cv.erode(close,ellipse(3))
    board=np.zeros(small.shape,np.uint8)
    ground=inside & ~broad
    board[Y[ground],X[ground]]=1
    grad=small.astype('float32')-cv.GaussianBlur(small.astype('float32'),(0,0),4)
    features=np.stack((small,cv.GaussianBlur(small,(0,0),3),
                       np.clip(grad+128,0,255).astype('uint8')),axis=-1)
    state=np.full(small.shape,cv.GC_BGD,np.uint8)
    state[exterior>0]=cv.GC_PR_BGD
    state[close>0]=cv.GC_PR_FGD
    state[(board>0)&(close==0)]=cv.GC_BGD
    state[(p>0)&(eroded>0)]=cv.GC_FGD
    if np.count_nonzero(state==cv.GC_FGD)>=25:
        cv.setRNGSeed(102)
        cv.grabCut(features,state,None,np.zeros((1,65)),np.zeros((1,65)),3,cv.GC_INIT_WITH_MASK)
        cut=((state==cv.GC_FGD)|(state==cv.GC_PR_FGD)).astype('uint8')
    else:
        cut=(close>0).astype('uint8')
    union=cv.morphologyEx(np.maximum(cut,(close>0).astype('uint8')),cv.MORPH_CLOSE,ellipse(2))
    # Reject nearby image regions reached solely via colour propagation.
    union &= cv.dilate(close,ellipse(6))>0
    return dict(silhouette=union.astype('uint8'), origin=lo, inside=inside, X=X, Y=Y)


def apply_image_mask(q, seg, mask):
    lo=seg['origin']
    x,y=(q['uv'].T.astype(int)//3)
    xx,yy=x-lo[0],y-lo[1]
    selected=(xx>=0)&(xx<mask.shape[1])&(yy>=0)&(yy<mask.shape[0])
    selected[selected]=mask[yy[selected],xx[selected]]!=0
    return selected


def reference_view(data, img):
    q,r=local_and_range(data)
    on=square(q)
    plane,fit=fit_board(q,r&on,'ChArUco')
    seeds=connected(q,r & square(q,18) & (plane>max(2.,4.*fit['noise_MAD_mm'])) & (plane<114.),data['uv'])
    broad=r & square(q,18) & (plane>-1.25) & (plane<114.)
    near_footprint=cKDTree(q[seeds]).query(q,k=1)[0]
    image=small_mask(data,seeds,broad & ((plane>.8)|(near_footprint<4.5)),img)
    foreground=apply_image_mask(data,image,image['silhouette'])
    near=cv.dilate(image['silhouette'],ellipse(2))
    h3=apply_image_mask(data,image,near)
    # Dense upper surface and any real near-board data within the image object boundary.
    body=broad & (seeds | (h3 & (near_footprint<3.35) & (plane>.72)))
    texture_confirmed=broad & foreground & (near_footprint<4.5) & (plane>=-.85)
    base=body | texture_confirmed
    # Add only geometrically backed overhangs. Squared footprint stays strict for the board.
    outside=base & (~on)
    board_plane=r & on & (abs(plane)<max(1.1,3.7*fit['noise_MAD_mm'])) & ~foreground & ~body
    slab=r & on & (plane>=-max(1.3,4.2*fit['noise_MAD_mm'])) & (plane<114.)
    # Point gate with a bounded prism, leave weak components for the E103 test.
    e102=slab | outside
    return dict(q=q, range=r, h=plane, fit=fit, e102=e102, board=board_plane, seeds=seeds,
                stable=body, base=base, foreground=foreground, distance=near_footprint,
                image=image, original=data)


def index_match(data, ref):
    _,a,b=np.intersect1d(data['grid_index'],ref['original']['grid_index'],
                         return_indices=True,assume_unique=True)
    return a,b


def board_from_reference(data,ref,branch):
    if branch=='ChArUco':
        return ref['h'],ref['fit']
    q,r=local_and_range(data)
    a,b=index_match(data,ref)
    use=np.zeros(len(q),bool)
    use[a]=ref['board'][b]
    return fit_board(q,r & use,branch)


def candidate(data, ref, branch):
    q,r=local_and_range(data)
    h,fit=board_from_reference(data,ref,branch)
    a,b=index_match(data,ref)
    matched=np.zeros(len(q),bool)
    matched[a]=True
    shared_board=np.zeros(len(q),bool);shared_board[a]=ref['board'][b]
    shared_stable=np.zeros(len(q),bool);shared_stable[a]=ref['stable'][b]
    shared_base=np.zeros(len(q),bool);shared_base[a]=ref['base'][b]
    shared_e102=np.zeros(len(q),bool);shared_e102[a]=ref['e102'][b]
    h_physical=h.copy()
    h_physical[a]=ref['h'][b]
    seg=ref['image']
    foreground=apply_image_mask(data,seg,seg['silhouette'])
    halo=apply_image_mask(data,seg,cv.dilate(seg['silhouette'],ellipse(3)))
    top=r & square(q,21) & (h>3.) & (h<114.)
    stable=connected(q,top & (shared_stable | (halo & ~matched)),data['uv'])
    # For common pixels use the consistently reconstructed foreground depth.
    upper=(shared_stable|stable) & r & ~shared_board
    tree=cKDTree(data['xyz'][upper][::2])
    dist=tree.query(data['xyz'])[0]
    # Retain single-view contact points if the silhouette and local surface support them.
    restore=r & ~shared_board & foreground & square(q,22) & (h_physical>-.85) & (h_physical<=5.0)
    low=restore & (shared_base | ((dist<3.6)&(h>=-max(1.4,2.*fit['noise_MAD_mm']))))
    if branch=='ChArUco':
        e102=ref['e102'].copy()
    else:
        # Valid Chessboard-only pixels may fill gaps where the comparison model did not decode.
        slab=square(q) & (h>=-max(2.6,3.*fit['noise_MAD_mm'])) & (h<114.)
        e102=r & (shared_e102 | (~matched & (slab | (stable&halo))) | shared_base)
    return dict(q=q, range=r, h=h, h_physical=h_physical, fit=fit, e102=e102,
                board=shared_board, stable=upper&e102, low=low&e102, near=dist,
                restore=restore&e102, foreground=foreground,
                M01=e102 & (h>3.) & square(q),
                M02=e102 & upper, M03=e102 & (upper|low))


def collect_scan(scan):
    bname=scan.name.split('_',4)[4]
    folders=sorted(scan.glob('view_*'))
    setup={}
    for branch in BRANCHES:
        cache=directory(scan)/branch
        setup[branch]=(json.loads((cache/'poses.json').read_text()),
                       [dict(np.load(p)) for p in sorted(cache.glob('*.npz'))])
    ref=[reference_view(d,load_frame(f,42)) for d,f in zip(setup['ChArUco'][1],folders)]
    cases={branch:[candidate(d,r,branch) for d,r in zip(v,ref)] for branch,(_,v) in setup.items()}
    for branch,(meta,data) in setup.items():
        cand=cases[branch]
        cycle=np.array(meta['pnp'])
        worlds=[transform(d['xyz'],T) for d,T in zip(data,cycle)]
        trees=[cKDTree(w[r['stable']][::4]) for w,r in zip(worlds,cand)]
        path=CACHE/scan.name/branch
        path.mkdir(parents=True,exist_ok=True)
        record=[]
        for i,(d,o,w) in enumerate(zip(data,cand,worlds)):
            pos=np.flatnonzero(o['restore'] & ~o['M03'])
            count=np.zeros(len(pos),np.uint8)
            if len(pos):
                for j,tree in enumerate(trees):
                    if i!=j:
                        count+=tree.query(w[pos])[0] < (2.7 if branch=='ChArUco' else 3.7)
            corroborated=np.zeros(len(w),bool)
            corroborated[pos]=(count>=2)&(o['near'][pos]<6.)
            o['M04']=o['M03'] | corroborated
            assert (o['M04']<=o['e102']).all()
            # Fit and evaluation arrays use the source indices. Point payloads remain in E101.
            np.savez_compressed(path/f'{i:02}.npz',height=o['h_physical'].astype('float32'),
                                branch_height=o['h'].astype('float32'), e102=o['e102'], board=o['board'],
                                **{m:o[m] for m in METHODS})
            record.append(dict(view=i, initial=len(w), distance_excluded=int((~o['range']).sum()),
                               board_plane=o['fit'], E102=int(o['e102'].sum()),
                               board_points=int(o['board'].sum()),
                               board_pixels_kept_E102=int((o['board']&o['e102']).sum()),
                               M04_bottom_count=int((o['M04'] & (o['h_physical']<=5.) &
                                                    (o['h_physical']>=-.85)).sum()),
                               M04_extra_support=int(corroborated.sum()),
                               **{m:int(o[m].sum()) for m in METHODS}))
        dump(path/'counts.json',dict(object=bname,camera_radial_mm=DISTANCE,board_xy_mm=BOARD_BOX,views=record))
        print(branch,bname,[(m,sum(row[m] for row in record)) for m in ('initial','E102','M01','M02','M03','M04','M04_bottom_count')],flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--scan')
    args=parser.parse_args()
    cv.ocl.setUseOpenCL(False)
    cv.setNumThreads(1)
    scans=[s for s in SCANS if args.scan is None or args.scan in s.name]
    with ThreadPoolExecutor(max_workers=3) as pool:
        list(pool.map(collect_scan,scans))
