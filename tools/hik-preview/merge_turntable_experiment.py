"""Experimental turntable alignment with a supplied approximate camera-frame axis."""
import argparse, json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial import cKDTree


def read_ply(p):
    with p.open('rb') as f:
        while f.readline()!=b'end_header\n': pass
        return np.fromfile(f,dtype=[('xyz','<f4',3),('rgb','u1',3)])['xyz'].astype(float)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('scan',type=Path);ap.add_argument('axis',type=Path);ap.add_argument('--bits',type=int,choices=[0,1,2],required=True);ap.add_argument('--radius',type=float,default=55);ap.add_argument('--base-clearance',type=float,default=3);ap.add_argument('--angles',type=Path,help='JSON mapping view names to corrected nominal angles');ap.add_argument('--suffix',default='',help='Output suffix');ap.add_argument('--no-crop',action='store_true',help='Keep all reconstructed points, including turntable and background');ap.add_argument('--input-pattern',default=None,help='Input filename pattern containing {bits}');args=ap.parse_args()
    session=json.loads((args.scan/'session.json').read_text(encoding='utf8'));axis=json.loads(args.axis.read_text(encoding='utf8'));o=np.array(axis['axis_point_camera_mm']);n=np.array(axis['axis_direction']);n/=np.linalg.norm(n)
    palette=np.array([[44,115,205],[232,133,45],[45,160,90],[195,66,60],[147,90,184],[22,162,170],[190,158,45],[213,100,162]],dtype=np.uint8)
    groups=[];colors=[];counts={};used_angles={};angle_step=session['nominal_step_degrees']; corrections=json.loads(args.angles.read_text(encoding='utf8')) if args.angles else None
    for v in session['views']:
        name=v['folder'];angle=corrections.get(name) if corrections is not None else v.get('nominal_angle_degrees');folder=args.scan/name
        if not folder.is_dir() or angle is None or abs(angle)>=360:continue
        fn=args.input_pattern.format(bits=args.bits) if args.input_pattern else ('20261120_all11_experimental.ply' if args.bits==0 else f'20261120_ignore{args.bits}_experimental.ply');p=read_ply(folder/fn)
        if not args.no_crop:
            z=(p-o)@n;rad=np.linalg.norm(p-o-z[:,None]*n,axis=1);p=p[(z < -args.base_clearance)&(z > -130)&(rad<args.radius)]
        R=cv2.Rodrigues(n*(-np.deg2rad(angle)))[0];p=(p-o)@R.T+o
        counts[name]=len(p);used_angles[name]=angle;groups.append(p);colors.append(np.repeat(palette[int(name[-4:])%8][None,:],len(p),axis=0))
    if not groups or not sum(map(len,groups)):raise ValueError('No object points after segmentation')
    a=np.concatenate(groups);c=np.concatenate(colors);records=np.empty(len(a),dtype=[('xyz','<f4',3),('rgb','u1',3)]);records['xyz']=a;records['rgb']=c
    out=args.scan/f'merged_ignore{args.bits}{args.suffix}_experimental.ply'
    with out.open('wb') as f:
        f.write((f'ply\nformat binary_little_endian 1.0\ncomment Experimental assumed turntable axis and nominal angles\nelement vertex {len(a)}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n').encode());records.tofile(f)
    overlap=[]
    for i in range(len(groups)):
        x=groups[i];y=groups[(i+1)%len(groups)]
        if len(x) and len(y):
            d=cKDTree(y).query(x,workers=-1)[0];overlap.append({'views':[i,(i+1)%len(groups)],'median_mm':float(np.median(d)),'fraction_within_3mm':float((d<3).mean())})
    result={'bits_ignored_per_axis':args.bits,'axis':axis,'views_and_point_counts':counts,'used_angles_degrees':used_angles,'points_total':len(a),'spatial_crop_enabled':not args.no_crop,'radius_mm':None if args.no_crop else args.radius,'clearance_mm':None if args.no_crop else args.base_clearance,'neighbor_distance':overlap,'note':('Approximate axis and nominal angles; no spatial crop; source reconstruction validity filters remain.' if args.no_crop else 'Approximate axis and nominal angles; fixed cylindrical crop; incomplete views and occlusion; nearest distances include nonoverlap and are not accuracy measurements.')}
    (args.scan/f'merged_ignore{args.bits}{args.suffix}_metadata.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8');print(args.scan.name,args.bits,counts,flush=True)

if __name__=='__main__':main()
