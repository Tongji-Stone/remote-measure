"""Estimate observable normal displacement and propagate through the volume functional."""
import numpy as np
import open3d as o3d
from scipy import sparse
from scipy.spatial import cKDTree
from scipy.sparse.linalg import cg

from volume_math import Surface, closed_candidate, tetra_gradient


def unit(x):
    return x/np.maximum(np.linalg.norm(x,axis=1,keepdims=True),1e-12)


def closest(scene,p):
    result=scene.compute_closest_points(o3d.core.Tensor(p.astype(np.float32)),nthreads=1)
    q=result['points'].numpy()
    direction=result['primitive_normals'].numpy()
    d=np.linalg.norm(p-q,axis=1)
    sign=np.einsum('ij,ij->i',p-q,direction)
    return q,direction,result['primitive_ids'].numpy(),result['primitive_uvs'].numpy(),d,sign


def metrics(scene,p,cameras=None):
    if len(p)==0:
        return {}
    _,_,_,_,d,s=closest(scene,p)
    result=dict(p95_mm=float(np.percentile(d,95)),signed_mean_mm=float(np.mean(np.clip(s,-2.5,2.5))),
                within2=float(np.mean(d<=2.)),n=len(p))
    if cameras is not None:
        ray=unit(p-cameras)
        length=np.linalg.norm(p-cameras,axis=1)
        hit=scene.cast_rays(o3d.core.Tensor(np.c_[cameras,ray].astype(np.float32)),nthreads=1)['t_hit'].numpy()
        result['early3']=float(np.mean(hit<length-3.))
    return result


def fit_fields(surface,context,p,camera,floor=True):
    q,n,fi,uv,d,s=closest(context,p)
    incidence=np.einsum('ij,ij->i',unit(camera-q),n) if camera is not None else np.ones(len(p))
    good=(d<=2.5)&(incidence>=.32)
    if floor:
        good &= q[:,2]>=2.
    q,n,s,fi,uv,incidence=[x[good] for x in (q,n,s,fi,uv,incidence)]
    if len(q)<40:
        raise ValueError('Insufficient observations to fit a signed displacement.')
    _,inverse,count=np.unique(np.floor(q/1.25).astype(int),axis=0,return_inverse=True,return_counts=True)
    weight=np.sqrt(incidence)/count[inverse]
    v=surface.vertices
    f=surface.faces
    nv=len(v)
    # Per-sample scalar displacement interpolated in the nearest triangle.
    coords=np.c_[1-uv.sum(axis=1),uv].clip(0.,1.)
    basis=sparse.csr_matrix((coords.ravel(),(np.repeat(np.arange(len(fi)),3),f[fi].ravel())),
                            shape=(len(fi),nv))
    _,a,nf=surface.g()
    mass=np.zeros(nv)
    normals=np.zeros((nv,3))
    for i in range(3):
        np.add.at(mass,f[:,i],a/3.)
        np.add.at(normals,f[:,i],(a[:,None]*nf)/3.)
    normal=unit(normals.copy())
    edges=np.unique(np.sort(f[:,[[0,1],[1,2],[2,0]]].reshape(-1,2),axis=1),axis=0)
    i,j=np.r_[edges,edges[:,::-1]].T
    # Suppress diffusion across sharp folds and make unsupported areas a zero-displacement prior.
    alignment=np.maximum(np.einsum('ij,ij->i',normal[i],normal[j]),0.)
    conductance=(.02+.98*alignment**4)/np.maximum(np.linalg.norm(v[i]-v[j],axis=1),.22)
    adjacency=sparse.csr_matrix((conductance,(i,j)),shape=(nv,nv))
    lap=sparse.diags(np.asarray(adjacency.sum(axis=1)).ravel())-adjacency
    nearest_distance=cKDTree(q).query(v)[0]
    support=np.clip((5.-nearest_distance)/3.,0.,1.)
    if floor:
        support *= np.clip((v[:,2]-1.5)/3.,0.,1.)
    sort=np.argsort(s)
    median=s[sort[np.searchsorted(np.cumsum(weight[sort]),weight.sum()/2)]]
    enabled=np.flatnonzero(support>1e-6)
    basis=basis[:,enabled]
    ld=(.16*lap+sparse.diags(.045*mass+.01))[enabled][:,enabled]
    h=np.zeros(len(enabled))
    for _ in range(3):
        residual=basis@h-s
        influence=weight*np.minimum(1.,.30/np.maximum(abs(residual),1e-9))
        weighted=basis.multiply(influence[:,None])
        system=(ld+basis.T@weighted).tocsr()
        rhs=weighted.T@s
        precondition=sparse.diags(1./system.diagonal())
        h,info=cg(system,rhs,x0=h,M=precondition,rtol=1e-6,maxiter=160)
        if info!=0:
            raise RuntimeError(f'Normal displacement iteration did not converge ({info}).')
    d=np.zeros(nv)
    d[enabled]=h
    fields={'D1':support*np.clip(median,-.6,.6),'D2':support*np.clip(d,-.6,.6)}
    return {k:val[:,None]*normal for k,val in fields.items()},support,mass


def candidate(surface,step,floor=True):
    a0,n0=surface.g()[1:]
    for factor in (1.,.5,.25):
        result=Surface(surface.vertices+factor*step,surface.faces)
        a1,n1=result.g()[1:]
        same_dir=np.einsum('ij,ij->i',n0,n1)
        if np.any((same_dir<=.05)|(a1<.05*a0)):
            continue
        if floor and np.min(result.vertices[:,2])<-1e-7:
            continue
        if closed_candidate(result) is not None:
            return result,factor
    return None,0.


def split_more(p):
    q=np.floor(p/1.1).astype(np.int64)
    return (q[:,0]*47+q[:,1]*19+q[:,2]*37)%3


def quadrature(surface,context,p,cam,mass,floor):
    """Freeze the observed surface support and its vertex-area weights for each fold."""
    q,n,_,_,dist,_=closest(context,p)
    good=dist<=2.5
    if cam is not None:
        good &= np.einsum('ij,ij->i',unit(cam-q),n) >= .32
    if floor:
        good &= q[:,2]>=2.
    ids=np.flatnonzero(good)
    if not len(ids):
        return np.zeros(len(p)),0.
    distance,k=cKDTree(q[ids]).query(surface.vertices)
    normals=unit(np.asarray(surface.legacy().compute_vertex_normals().vertex_normals))
    compatible=np.einsum('ij,ij->i',normals,n[ids[k]])>.60
    supported=mass*np.clip((4.-distance)/2.,0.,1.)*compatible
    if floor:
        supported*=np.clip((surface.vertices[:,2]-1.5)/3.,0.,1.)
    weight=np.bincount(ids[k],weights=supported,minlength=len(p))
    return weight,float(weight.sum()/mass.sum())


def volume_defect(ctx,p,weights):
    *_,signed=closest(ctx,p)
    return float(np.dot(weights,np.clip(signed,-1.,1.))/1000.)


def evaluate(surface,p,cameras,truth,case,trial,category,hidden=None,floor=True,check_base=True):
    original=surface.signed()
    context=surface.scene()
    folds=split_more(p)
    a,b,c=(folds==k for k in range(3))
    base=metrics(context,p[b],None if cameras is None else cameras[b])
    grad=tetra_gradient(surface)
    rows=[]
    best='D0'
    best_volume='D0'
    best_p95=base['p95_mm']
    meshes={'D0':surface}
    fields,cover,mass=fit_fields(surface,context,p[a],None if cameras is None else cameras[a],floor)
    wb,cov=quadrature(surface,context,p[b],None if cameras is None else cameras[b],mass,floor)
    wc,_=quadrature(surface,context,p[c],None if cameras is None else cameras[c],mass,floor)
    raw=volume_defect(context,p[b],wb)
    best_defect=abs(raw)
    for name,step in {'D0':np.zeros_like(surface.vertices),**fields}.items():
        if name=='D0':
            out,applied=surface,0.
            if check_base and closed_candidate(surface) is None:
                raise ValueError(f'Non-closed base for {case}, {trial}')
        else:
            out,applied=candidate(surface,step,floor)
        row=dict(case=case,trial=trial,category=category,method=name,
                 physical_reference_ml=None if truth is None else truth/1000.,
                 coverage_area_fraction=np.dot(cover,mass)/mass.sum(),
                 tune_quadrature_area_fraction=cov,
                 fit_n=int(a.sum()), tune_n=int(b.sum()), test_n=int(c.sum()),
                 valid=out is not None,step_factor=applied)
        if out is not None:
            out_ctx=context if name=='D0' else out.scene()
            vm=out.signed()
            val=metrics(out_ctx,p[b],None if cameras is None else cameras[b])
            test=metrics(out_ctx,p[c],None if cameras is None else cameras[c])
            row.update(volume_ml=vm/1000.,delta_ml=(vm-original)/1000.,
                       first_order_delta_ml=(applied*np.sum(grad*step))/1000.,
                       relative_error=None if truth is None else vm/truth-1.,
                       max_normal_change_mm=float(applied*np.linalg.norm(step,axis=1).max()))
            row.update({f'tune_{k}':v for k,v in val.items()})
            row.update({f'test_{k}':v for k,v in test.items()})
            if hidden is not None and len(hidden):
                row['hidden_recall_2mm']=metrics(out_ctx,hidden)['within2']
            vol_residual=volume_defect(out_ctx,p[b],wb)
            row['tune_signed_defect_ml']=vol_residual
            row['test_signed_defect_ml']=volume_defect(out_ctx,p[c],wc)
            allowed=.10 if cov>=.80 else .03
            if (name!='D0' and abs(vol_residual)<min(best_defect,.9*abs(raw),abs(raw)-.02)
                    and (vm-original)*raw>0. and abs(vm/original-1.)<=allowed
                    and val['p95_mm']<=1.01*base['p95_mm']
                    and val.get('early3',0.)<=base.get('early3',0.)+.01):
                best_volume,best_defect=name,abs(vol_residual)
            if name!='D0' and (val['p95_mm']<min(.99*base['p95_mm'],best_p95)
                    and abs(val['signed_mean_mm'])<=abs(base['signed_mean_mm'])+.03
                    and val.get('early3',0.)<=base.get('early3',0.)+.01 and abs(vm/original-1.)<=.03):
                best=name
                best_p95=val['p95_mm']
            meshes[name]=out
        rows.append(row)
    selected=next(r for r in rows if r['method']==best)
    rows.append(dict(selected,method='D3',selected=best))
    selected_volume=next(r for r in rows if r['method']==best_volume)
    rows.append(dict(selected_volume,method='D4',selected=best_volume))
    return rows,meshes
