"""Compare four object separation methods with independently annotated white frames."""
import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import least_squares

from common import BOX, HERE, load, table_basis, transform

LABELS=load(HERE/"labels.json")


def plane_for_view(s,key,prior,pose):
    c=np.array(prior["center"])
    n=np.array(prior["axis"])
    w=transform(s["xyz"],pose)
    a=(w-c)@table_basis(n)
    r=np.linalg.norm(a[:,:2],axis=1)
    # Broad ring is for plane fitting only. Per-view residual tracks depth bias at the physical deck.
    q=(r>25)&(r<76)&(abs(a[:,2])<15)&(s["error"]<7.)
    candidates=a[q]
    z=candidates[:,2]
    if len(z)<220:
        return n,c,0.,3.
    hist,edges=np.histogram(z,bins=np.linspace(-15,15,46))
    mode=(edges[np.argmax(hist)]+edges[np.argmax(hist)+1])/2.
    use=candidates[abs(z-mode)<3.6][::2]
    fit=least_squares(lambda v:np.r_[use[:,2]-use[:,:2]@v[:2]-v[2],v[:2]*15.],
        np.array([0.,0.,mode]),loss="soft_l1",f_scale=1.,bounds=([-.09,-.09,mode-2.3],[.09,.09,mode+2.3])).x
    B=table_basis(n)
    normal=n-B[:,:2]@fit[:2]
    normal/=np.linalg.norm(normal)
    h=(w-(c+n*fit[2]))@normal
    rough=np.median(abs(use[:,2]-use[:,:2]@fit[:2]-fit[2]))
    return normal,c+n*fit[2],float(fit[2]),float(rough)


def image_masks(s, h, xyz_world, prior, camera_mode):
    # Temporal + geometric candidate. Stage silhouette remains untouched in measured scene.
    C=np.array(prior["center"])
    up=np.array(prior["axis"])
    radial=np.linalg.norm((xyz_world-C)@table_basis(up)[:,:2],axis=1)
    seed=np.array(prior["object_center"])
    distance=np.linalg.norm((xyz_world-seed)@table_basis(up)[:,:2],axis=1)
    broad=(radial<108)&(distance<93)&(h>-1.)&(h<150)
    cut=(radial<86)&(h>4.)&(h<145)
    p3d=(h>max(2.2,min(5.,2.4*camera_mode)))&broad
    return cut,p3d,broad


def components(xyz,mask,seed):
    use=np.where(mask)[0]
    if len(use)<40: return mask.copy()
    vox=np.floor(xyz[use]/2.9).astype(np.int32)
    u,inv=np.unique(vox,axis=0,return_inverse=True)
    cloud={tuple(p):j for j,p in enumerate(u)}
    parent=np.arange(len(u))
    def root(i):
        while parent[i]!=i:
            parent[i]=parent[parent[i]];i=parent[i]
        return i
    for i,p in enumerate(u):
        for delta in ((1,0,0),(0,1,0),(0,0,1),(1,1,0),(1,-1,0),(1,0,1),(1,0,-1),(0,1,1),(0,1,-1)):
            q=cloud.get(tuple(p+delta))
            if q is not None: parent[root(i)]=root(q)
    index=np.array([root(i) for i in inv])
    counts=np.bincount(index,minlength=len(u))
    weights=np.exp(-np.linalg.norm(xyz[use]-seed,axis=1)**2/(2*48**2))
    scores=np.bincount(index,weights=weights,minlength=len(u))
    good=np.argmax(scores)
    member=index==good
    rest=u[np.unique(inv[member])]*2.9+1.45
    # Components that touch the main surface after a one-voxel dilation are retained.
    out=np.zeros(len(xyz),bool)
    out[use]=member | (cKDTree(rest).query(xyz[use])[0]<5.8)
    return out


def to_image(uv,mask,shape):
    x=np.rint(uv[:,0]*shape[1]/2448).astype(int).clip(0,shape[1]-1)
    y=np.rint(uv[:,1]*shape[0]/2048).astype(int).clip(0,shape[0]-1)
    out=np.zeros(shape,np.uint8)
    out[y[mask],x[mask]]=255
    return out,x,y


def grab(s,h,world,core,broad,cal,seed,other_support):
    # Seeded image segmentation can recover points close to the contact plane.
    shape=(410,490)
    p,x,y=to_image(s["uv"],core,shape)
    q,_,_=to_image(s["uv"],broad,shape)
    if core.sum()<28: return core.copy(),np.zeros(shape,np.uint8)
    p=cv2.morphologyEx(p,cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(13,13)))
    p=cv2.dilate(p,np.ones((3,3),np.uint8))
    n,label,stats,_=cv2.connectedComponentsWithStats(p)
    if n>1:
        substantial=[j for j in range(1,n) if stats[j,cv2.CC_STAT_AREA]>=max(7,stats[1:,cv2.CC_STAT_AREA].max()*.065)]
        p=np.isin(label,substantial).astype(np.uint8)*255
    outer=cv2.dilate(p,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(21,21)))
    inner=cv2.erode(p,np.ones((3,3),np.uint8))
    state=np.full(shape,cv2.GC_BGD,np.uint8)
    state[outer>0]=cv2.GC_PR_BGD
    state[p>0]=cv2.GC_PR_FGD
    state[inner>0]=cv2.GC_FGD
    image=cv2.resize(s["image"],shape[::-1],interpolation=cv2.INTER_AREA)
    # Stacking the image and two local residual/filter signals helps distinguish gray board and stage.
    low=cv2.GaussianBlur(image,(0,0),7)
    local=np.clip(image.astype(np.int16)-low.astype(np.int16)+128,0,255).astype(np.uint8)
    features=np.stack((image,low,local),axis=-1)
    try:
        cv2.grabCut(features,state,None,np.zeros((1,65)),np.zeros((1,65)),3,cv2.GC_INIT_WITH_MASK)
        mask=((state==cv2.GC_FGD)|(state==cv2.GC_PR_FGD)).astype(np.uint8)*255
    except cv2.error:
        mask=p
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
    # Keep supported geometric core through texture changes. Restore thin contact-area points only inside the image mask.
    seed_distance=cKDTree(world[core]).query(world)[0]
    final=(core|(broad&(h>-.3)&(mask[y,x]>0)&((seed_distance<5.)|other_support)))&(h<150)
    return final,mask


def separate(key,scenes,poses,prior,cal):
    results=[]
    seed=np.array(prior["object_center"])
    for s,T in zip(scenes,poses):
        p=transform(s["xyz"],T)
        n,c,offset,rough=plane_for_view(s,key,prior,T)
        h=(p-c)@n
        plane,above,broad=image_masks(s,h,p,prior,rough)
        comp=components(p,above,seed)
        results.append({"world":p,"h":h,"n":n,"c":c,"plane_offset":offset,"plane_noise":rough,
                        "M01":plane,"M02":above,"M03":comp,"broad":broad})
    trees=[cKDTree(r["world"][r["M03"]][::2]) for r in results if r["M03"].any()]
    for i,(s,r) in enumerate(zip(scenes,results)):
        own=r["M03"]
        support=np.zeros(len(own),np.int8)
        candidates=np.where(r["broad"])[0]
        if len(candidates):
            for tree in trees:
                support[candidates]+=(tree.query(r["world"][candidates])[0]<4.4)
        # Other-view support protects valid sparse surfaces without requiring that every point be repeatedly visible.
        coarse=r["M02"] & ((support>=2)|own)
        if not coarse.any(): coarse=own
        r["M04"]=coarse
        r["M05"],r["silhouette"]=grab(s,r["h"],r["world"],coarse,r["broad"],cal,seed,support>=2)
        _,x,y=to_image(s["uv"],coarse,r["silhouette"].shape)
        if coarse.any():
            frame=table_basis(r["n"])[:,:2]
            lateral=cKDTree(r["world"][coarse]@frame).query(r["world"]@frame)[0]
            mask=cv2.dilate(r["silhouette"],cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7)))
            restore=(r["h"]>=-1.)&(r["h"]<7.5)&r["broad"]&(lateral<3.6)&(mask[y,x]>0)
        else:
            restore=np.zeros(len(coarse),bool)
        r["M06"]=r["M05"]|restore
    # A near-rim object has no reliable contact margin for silhouette-based expansion.
    # Estimate offset without any annotation, using the previous calibrated axis and scene anchor.
    B=table_basis(np.array(prior["axis"]))
    offset=np.linalg.norm((seed-np.array(prior["center"]))@B[:,:2])
    for r in results:
        r["M07"]=r["M04"] if offset>24. else r["M06"]
    return results


def evaluate_masks(key,i,s,r):
    pts=LABELS[key].get(str(i))
    if pts is None:return None
    gt=np.zeros((205,245),np.uint8)
    cv2.fillPoly(gt,[np.asarray(pts,np.int32)],255)
    inner=cv2.erode(gt,np.ones((3,3),np.uint8))
    outer=cv2.dilate(gt,np.ones((5,5),np.uint8))
    _,x,y=to_image(s["uv"],np.ones(len(s["uv"]),bool),gt.shape)
    positive=inner[y,x]>0
    negative=outer[y,x]==0
    low=positive&(r["h"]<=8)&(r["h"]>=-4)
    idx=np.nonzero(gt)
    projected=np.count_nonzero(inner)/(245*205)*(2448*2048/16)
    tests={}
    for m in ("M01","M02","M03","M04","M05","M06","M07"):
        valid=r[m]
        tp=int((valid&positive).sum()); fp=int((valid&negative).sum()); fn=int((~valid&positive).sum())
        tests[m]=dict(tp=tp,fp=fp,fn=fn,f1=2*tp/max(1,2*tp+fp+fn),
                      low_tp=int((valid&low).sum()),low_n=int(low.sum()))
    outside=sum(int((r[m]&negative).sum()) for m in tests)
    return {"case":key,"view":i,"gt_preview":gt,"test":tests,
            "valid_depth_inside":int(positive.sum()),"silhouette_pixels_estimate":float(projected),
            "inside_valid_fraction":float(positive.sum()/max(1,projected))}
