"""Produce point-cloud comparisons and shaded white-mesh images without a window."""
import csv
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter

import cv2
import numpy as np
import open3d as o3d
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial import cKDTree

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
ASSETS=ROOT/"algorithm/scene-fusion/E003/results"
SIM=HERE/"results"
FOLDER="figures"
FONT="/Library/Fonts/Arial Unicode.ttf"
BG="#ecf2f6"
DARK="#183246"
GREEN="#106d63"
CAPS={}


def font(s):
    return ImageFont.truetype(FONT,s)


def canvas(width,height,title,sub):
    image=Image.new("RGB",(width,height),"white")
    d=ImageDraw.Draw(image)
    d.text((22,12),title,fill=DARK,font=font(34))
    d.text((24,56),sub,fill="#456479",font=font(20))
    return image,d


def metadata(out,file,caption,sources):
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    CAPS.setdefault(out,{})[file.stem]={"sha256":digest(file),"source_sha256":digest(Path(__file__)),
        "caption":caption,"caption_sha256":hashlib.sha256(caption.encode()).hexdigest(),
        "inputs":{str(p.relative_to(ROOT)):digest(p) for p in set(sources)},
        "units":"mm* in geometry; panels use same scale where compared",
        "command":"algorithm/.venv/bin/python simulation/E003/render_e003.py"}


def save(canvas,base,number,caption,inputs):
    directory=base/FOLDER
    directory.mkdir(exist_ok=True)
    f=directory/f"E003-{number}.png"
    canvas.save(f,optimize=True)
    CAPS.setdefault(directory,{})
    metadata(directory,f,caption,inputs)


def read_points(path):
    cloud=o3d.io.read_point_cloud(str(path))
    p=np.asarray(cloud.points).copy()
    c=np.asarray(cloud.colors).copy()
    return p,c


def basis(azimuth,elevation):
    a,e=np.radians([azimuth,elevation])
    forward=np.array([np.cos(e)*np.cos(a),np.cos(e)*np.sin(a),np.sin(e)])
    right=np.array([-np.sin(a),np.cos(a),0.])
    return np.column_stack([right,np.cross(forward,right),forward])


def reference(p):
    lo,hi=np.percentile(p,[.5,99.5],axis=0)
    return (lo+hi)/2, max(np.ptp(np.array([lo,hi]),axis=0))*1.25


def project(size,points,center,scale,axes):
    pos=(points-center)@axes
    xy=pos[:,:2]*min(size)/scale
    return np.c_[size[0]/2+xy[:,0],size[1]/2-xy[:,1]],pos[:,2]


def dots(size,p,c,cen,sca,axes,limit=14000):
    a=np.full((size[1],size[0],3),[235,242,246],np.uint8)
    take=np.arange(0,len(p),max(1,len(p)//limit))
    xy,z=project(size,p[take],cen,sca,axes)
    take2=np.argsort(z)
    for (x,y),k in zip(np.rint(xy[take2]).astype(int),take[take2]):
        if 0<=x<size[0] and 0<=y<size[1]:
            cv2.circle(a,(x,y),1,tuple(map(int,c[k])),thickness=-1,lineType=cv2.LINE_AA)
    return Image.fromarray(a)


def mesh_preview(path,size,cent,scale,yaw,elevation):
    axes=basis(yaw,elevation)
    mesh=o3d.io.read_triangle_mesh(str(path))
    mesh.compute_vertex_normals()
    vertex=np.asarray(mesh.vertex_normals)
    triangles=np.asarray(mesh.triangles)
    scene=o3d.t.geometry.RaycastingScene(nthreads=2)
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    w,h=size
    yy,xx=np.indices((h,w))
    u=(xx-(w-1)/2)*scale/min(size)
    v=((h-1)/2-yy)*scale/min(size)
    origin=cent+u[:,:,None]*axes[:,0]+v[:,:,None]*axes[:,1]+3*scale*axes[:,2]
    rays=np.concatenate([origin,np.broadcast_to(-axes[:,2],origin.shape)],axis=-1).astype(np.float32)
    answer=scene.cast_rays(o3d.core.Tensor(rays),nthreads=2)
    valid=np.isfinite(answer["t_hit"].numpy())
    tri=answer["primitive_ids"].numpy()[valid]
    uv=answer["primitive_uvs"].numpy()[valid]
    bary=np.c_[1.-uv.sum(1),uv]
    normals=(answer["primitive_normals"].numpy()[valid] if "-M04-" in path.name
             else np.einsum("ij,ijk->ik",bary,vertex[triangles[tri]]))
    normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-8)
    normals*=np.where(normals@axes[:,2]<0,-1,1)[:,None]
    normal_view=normals@axes
    aa=normal_view@np.array([-.40,.57,.72])
    bb=normal_view@np.array([.71,.15,.64])
    color=(.33+.60*np.maximum(aa,0)+.14*np.maximum(bb,0))*255
    color=np.rint(np.clip(color,80,253)).astype(np.uint8)
    out=np.full((h,w,3),[231,240,247],dtype=np.uint8)
    out[valid]=np.c_[color,color,np.clip(color.astype(int)+1,0,255)]
    return Image.fromarray(out)


def general_points(index):
    out,draw=canvas(2065,1103,"E003  多视角去噪对照","同一组、同一位姿与取景。珊瑚色代表 D02 删除的点。")
    W,H=323,225
    sources=[]
    for i,(tag,row) in enumerate(index.items()):
        px=26+(i%2)*1020
        py=96+(i//2)*249
        folder=ASSETS/tag
        raw,clean=[folder/f"{tag}-D0{k}.ply" for k in (0,2)]
        paths=(raw,clean,folder/f"{tag}-01.json")
        sources.extend(paths)
        p,c=read_points(raw);q,d=read_points(clean)
        B=np.array(row["axis"])
        a=np.array([1.,0.,0.])-B[0]*B
        a/=np.linalg.norm(a)
        b=np.column_stack([a,np.cross(B,a),B])
        p=(p-row["center"])@b
        q=(q-row["center"])@b
        cent,sca=reference(p)
        xyz_axes=basis(-48,29)
        removed=cKDTree(q).query(p)[0]>.01
        changed=np.broadcast_to([156,176,187],(len(p),3)).copy()
        changed[removed]=[211,81,53]
        for j,(P,C) in enumerate(((p,c*220),(q,d*220),(p,changed))):
            panel=dots((W,H-30),P,C,cent,sca,xyz_axes,30000 if j==2 else 14000)
            out.paste(panel,(px+j*(W+9),py+29))
        draw.text((px,py),tag[-3:]+"  "+row["label"],fill=DARK,font=font(29))
        draw.text((px+332,py+3),"D02",fill=GREEN,font=font(24))
        draw.text((px+666,py+3),"删除 "+str(row["counts"][0]-row["counts"][2])+" 点",fill=DARK,font=font(24))
    save(out,ASSETS,10,"Same registered views, previous scene mask versus E003 denoising. Color marks affected samples.",sources)


def top_views(index,diag):
    out,d=canvas(1780,700,"E003  纸盒角部与轴心假设","实测俯视图使用固定训练盒面作参考。右侧是独立的 4 mm 理想模型干预。")
    W,H=324,176
    sources=[]
    boxids=("E003-S03","E003-S06","E003-S07","E003-S08")
    for slot,tag in enumerate(boxids):
        model=diag[tag]
        px=30+(slot%2)*380
        py=103+(slot//2)*259
        f=ASSETS/tag/f"{tag}-D02.ply"
        jp=SIM/tag/f"{tag}-02.json"
        sources.extend([f,jp])
        p,v=read_points(f)
        I=index[tag]
        z=np.array(I["axis"])
        x=np.array([1.,0.,0.])-z[0]*z
        x/=np.linalg.norm(x)
        B=np.column_stack((x,np.cross(z,x),z))
        p=(p-I["center"])@B
        yaw,cx,cy,cz,a,b,h=model["local_box_parameters"]
        R=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
        z_filter=(p[:,2]>cz+.12*h)&(p[:,2]<cz+.88*h)
        p=p[z_filter]
        c=v[z_filter]
        cent=np.array([cx,cy,0.]); scale=2.65*max(a,b)
        axes=np.eye(3)
        plane=dots((W,H),p,c*235,cent,scale,axes)
        A=np.array([[-a,-b],[a,-b],[a,b],[-a,b],[-a,-b]])@R.T+cent[:2]
        xy,_=project((W,H),np.c_[A,np.ones(len(A))],cent,scale,axes)
        dr=ImageDraw.Draw(plane)
        dr.line([tuple(t) for t in xy],fill="#253c4f",width=2)
        out.paste(plane,(px,py+32))
        d.text((px,py),tag[-3:]+"  "+I["label"],fill=DARK,font=font(23))
        score=model["heldout_E003"]["max_face_protrusion_P95"]
        if score is not None:
            d.text((px,py+215),f"最突出侧面 P95  {score:.2f} mm*",fill=DARK,font=font(19))
    for j,shift in enumerate([0,4.]):
        x0=815+j*455
        y0=150
        w,h=398,414
        image=Image.new("RGB",(w,h),BG)
        pen=ImageDraw.Draw(image)
        panel=(w,h)
        axes=np.eye(3)
        ori=np.zeros(3)
        scale=100.
        corners=np.array([[-34,-25,0],[34,-25,0],[34,25,0],[-34,25,0],[-34,-25,0]])
        quad,_=project(panel,corners,ori,scale,axes)
        pen.line([tuple(q) for q in quad],fill="#aebfcc",width=2)
        for p,u,clr in ((np.array([[-34,25,0],[34,25,0]]),[0,-1,0],"#4d9c6b"),
                         (np.array([[34,-25,0],[34,25,0]]),[-1,0,0],"#d07e43"),
                         (np.array([[-34,-25,0],[34,-25,0]]),[0,1,0],"#477fb5"),
                         (np.array([[-34,-25,0],[-34,25,0]]),[1,0,0],"#985899")):
            xy,_=project(panel,p+shift*np.array(u),ori,scale,axes)
            pen.line([tuple(q) for q in xy],fill=clr,width=6)
        out.paste(image,(x0,y0))
        d.text((x0,y0-34),["输入端点与面距相符","各侧面内移 4 mm，面宽固定"][j],fill=DARK,font=font(23))
        d.text((x0,y0+h+19),[ "超出 0 mm", "交点外各留 4 mm 的端部"][j],fill=GREEN,font=font(24))
    save(out,ASSETS,11,"Fixed-envelope measured box top view and synthetic four parallel inward displacement case.",sources)


def pair_steps(index):
    sys.path.insert(0,str(ASSETS.parent))
    from registration_pipeline import load_case,table_basis,transform,feature_pack,sac_ia,training_mask

    p=ASSETS/"E003-03.csv"
    with p.open(encoding="utf-8-sig") as file:
        rows=list(csv.DictReader(file))
    out,d=canvas(1790,610,"E003  粗配准与精配准实例","ISS 检测关键点，FPFH 配对并产生候选。最后一列按标定与已选全局位姿对齐。")
    sources=[p]
    for j,tag in enumerate(("E003-S01","E003-S03")):
        entry=index[tag]
        choices=[r for r in rows if r["id"]==tag and r["route"]=="R05" and r["sac_score"]]
        case=min(choices,key=lambda r:(r["feature_accepted"]!="True",float(r["sac_score"])))
        a,b=int(case["a"]),int(case["b"])
        _,_,_,scans,_=load_case(entry["key"])
        cloud=[]
        for i in (a,b):
            f=ASSETS/tag/f"{tag}-V{i:02d}.npz"
            sources.append(f)
            with np.load(f) as s:
                keep=s["mask"]&training_mask(scans[i]["uv"])
            cloud.append(scans[i]["xyz"][keep])
        features=list(map(feature_pack,cloud))
        coarse,_,_=sac_ia(*features,np.random.default_rng(904+a*11+b))
        poses=np.array(entry["poses"])
        C=np.array(entry["center"]);B=table_basis(np.array(entry["axis"]))
        D=[(transform(P,T)-C)@B for P,T in zip(cloud,poses[[a,b]])]
        cs,scale=reference(np.concatenate(D))
        axes=basis(-39,29)
        palettes=([55,116,170],[198,112,53])
        size=(410,175)
        output=[]
        for k in range(2):
            f=features[k][1]
            key=(transform(f,poses[[a,b][k]])-C)@B
            P=np.concatenate([D[k],key])
            colors=np.r_[np.tile([161,185,203],(len(D[k]),1)),np.tile([192,45,53],(len(key),1))]
            panel=dots(size,P,colors,cs,scale,axes,40000)
            od=ImageDraw.Draw(panel)
            xy,_=project(size,key,cs,scale,axes)
            for x,y in xy:
                od.ellipse((x-2,y-2,x+2,y+2),fill="#d8364b")
            output.append(panel)
        coarse_P=(transform(cloud[0],poses[b]@coarse)-C)@B
        cn=np.r_[np.tile(palettes[0],(len(coarse_P),1)),np.tile(palettes[1],(len(D[1]),1))]
        # The same viewport lets displaced coarse hypotheses leave the target area.
        output.append(dots(size,np.r_[coarse_P,D[1]],cn,cs,scale,axes))
        output.append(dots(size,np.concatenate(D),cn,cs,scale,axes))
        y=102+j*251
        d.text((23,y),tag[-3:]+"  "+entry["label"]+f"     V{a:02} → V{b:02}    "
               +("特征边通过门控" if case["feature_accepted"]=="True" else "特征粗配准偏离标定"),fill=DARK,font=font(22))
        for k,image in enumerate(output):
            x=24+441*k
            d.text((x,y+32),[f"源扫描 · ISS {features[0][-1]} 点",f"目标扫描 · ISS {features[1][-1]} 点","SAC-IA 式粗配准","本轮选用位姿"][k],
                   font=font(20),fill=GREEN)
            out.paste(image,(x,y+62))
    save(out,ASSETS,12,"Observed training clouds, ISS keypoints, raw sampled FPFH hypothesis, and chosen turntable registration in the same viewport.",sources)


def mesh_gallery(index,choices):
    out,d=canvas(2080,1018,"E003  白色三角网格","同一物体保持尺度与朝向。第一张是受实测支撑的 Poisson 面，之后是补全与底面。")
    W,H=329,204
    paths=[]
    for i,(tag,row) in enumerate(index.items()):
        x=24+(i%2)*1030
        y=92+(i//2)*227
        folder=SIM/tag
        pts=folder/f"{tag}-C00.ply"
        p,_=read_points(pts)
        cen,sc=reference(p)
        camera=np.array(row["poses"])[0,:3,3]
        norm=np.array(row["axis"])
        u=np.array([1.,0.,0.])-norm[0]*norm
        u/=np.linalg.norm(u)
        B=np.column_stack([u,np.cross(norm,u),norm])
        pos=(camera-row["center"])@B-cen
        azi=np.degrees(np.arctan2(pos[1],pos[0]))
        paths.append(pts)
        d.text((x,y),tag[-3:]+"  "+row["label"]+"     M01 实测  ·  "+choices[tag]+" 补全  ·  底面",font=font(22),fill=DARK)
        for j,(method,aa,ee) in enumerate((("M01",azi+13,22),(choices[tag],azi+13,22),(choices[tag],azi+45,-48))):
            path=folder/f"{tag}-{method}-T0.ply"
            paths.append(path)
            im=mesh_preview(path,(W,H-30),cen,sc,aa,ee)
            out.paste(im,(x+j*(W+8),y+34))
    save(out,SIM,10,"White shaded triangular models. Measured Poisson surface, selected closed surface and bottom view.",paths)


def flipper_detail(index):
    tag="E003-S01"
    folder=SIM/tag
    full=json.loads((folder/f"{tag}-02.json").read_text())
    hints=full["local_completion"]["0"].get("photo_support")
    if hints is None:
        return
    row=index[tag]
    annotations_path=HERE/"minion_footprints.json"
    annotations=json.loads(annotations_path.read_text())
    optical=ASSETS.parents[1]/"E001/optical_inputs.json"
    if not optical.exists():
        optical=ASSETS.parents[1]/"results/optical_inputs.json"
    meta=json.loads(optical.read_text())["minion"]
    out,d=canvas(1560,700,"E003  小黄人脚蹼","使用 V00、V01 白场图，保留 V07 对照。网格候选包含照片引导的脚蹼与局部底面。")
    paths=[annotations_path,optical,folder/f"{tag}-02.json"]
    x0,y0=940,1010
    for i,j in enumerate((0,1,7)):
        img=ROOT/meta["path"]/meta["views"][j]["frame"]/"graycode_42.png"
        paths.append(img)
        crop=Image.open(img).convert("RGB").crop((x0,y0,x0+540,y0+255))
        mark=ImageDraw.Draw(crop)
        for poly in annotations["polygons"][str(j)]:
            a=[(x-x0,y-y0) for x,y in poly]
            mark.line(a+[a[0]],fill="#50e3ac",width=3)
        crop.thumbnail((480,228))
        x=22+i*510
        out.paste(crop,(x,132))
        d.text((x,95),f"V{j:02}    "+("留出图" if j==7 else "轮廓输入"),font=font(24),fill=DARK)
    pts=folder/f"{tag}-C00.ply"
    guides=folder/f"{tag}-C02-T0.ply"
    paths.extend((pts,guides))
    p,_=read_points(pts)
    body,_=reference(p)
    q,_=read_points(guides)
    cent=np.r_[np.median(q[:,:2],axis=0),8.6]
    scale=max(np.ptp(q[:,:2],axis=0))*1.10
    norm=np.array(row["axis"])
    x=np.array([1.,0.,0.])-norm[0]*norm
    x/=np.linalg.norm(x)
    B=np.column_stack((x,np.cross(norm,x),norm))
    v=(np.array(row["poses"])[0,:3,3]-row["center"])@B-body
    yaw=np.degrees(np.arctan2(v[1],v[0]))
    for i,(method,de,elev) in enumerate((("M03",0,16),("M05",0,16),("M05",15,-43))):
        mesh=folder/f"{tag}-{method}-T0.ply"
        paths.append(mesh)
        x=22+i*510
        d.text((x,382),["M03 近盘细节","M05 照片引导","M05 底视放大"][i],font=font(25),fill=DARK)
        a=mesh_preview(mesh,(480,240),cent,scale,yaw+de,elev)
        out.paste(a,(x,416))
    save(out,SIM,11,"Raw white-field foot contours and generic versus photo-assisted foot reconstruction; view 07 was held out.",paths)


def main():
    index=json.loads((ASSETS/"E003-05.json").read_text())
    general_points(index)
    pair_steps(index)
    if (SIM/"E003-02.json").is_file():
        diag=json.loads((SIM/"E003-02.json").read_text())
        top_views(index,diag)
        wanted=SIM/"E003-03.json"
        choices=json.loads(wanted.read_text()) if wanted.exists() else {tag:"M04" if "box" in row["key"] else "M03" for tag,row in index.items()}
        mesh_gallery(index,choices)
        flipper_detail(index)
    for f,data in CAPS.items():
        (f/"E003.json").write_text(json.dumps(data,ensure_ascii=False,indent=2)+"\n")


if __name__=="__main__":
    main()
