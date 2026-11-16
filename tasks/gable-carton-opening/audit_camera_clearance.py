"""Conservative actual-camera/paper clearance audit; never changes observations."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from trimesh.triangles import closest_point
from carton_probe import geometry as G

def triangles():
    A=G.A;lo=-G.HEIGHT
    surfaces=[
      [[-A,-A,lo],[-A,A,lo],[-A,A,0],[-A,-A,0]],
      [[A,A,lo],[A,-A,lo],[A,-A,0],[A,A,0]],
      [[-A,A,lo],[A,A,lo],[A,A,0],[-A,A,0]],
      [[A,-A,lo],[-A,-A,lo],[-A,-A,0],[A,-A,0]],
      [[-A,-A,lo],[A,-A,lo],[A,A,lo],[-A,A,lo]],
      [[A,-A,0],[A,A,0],[A,0,A]],
      [[-A,A,0],[A,A,0],[A,0,A],[0,0,A]],
      [[A,-A,0],[-A,-A,0],[0,0,A],[A,0,A]],
      [[0,0,A],[A,0,A],[A,0,A+G.FIN],[0,0,A+G.FIN]]]
    by_body={n:[v] for n,v in G.FACETS.items()};by_body['carton_body']=surfaces
    return {n:np.array([[v[0],v[i],v[i+1]] for v in vv for i in range(1,len(v)-1)])-G.CENTRES[n]
            for n,vv in by_body.items()}

def audit(run):
    c=np.load(run/'camera-poses.npz');p=np.load(run/'physics-substeps.npz');idx=c['physics_sample']
    assert idx.ndim==1 and len(idx)>0 and idx.min()>=0 and idx.max()<len(p['poses'])
    names=[str(x).split('/')[-1] for x in p['paths']];q=p['poses'][idx];rows={}
    for j,camera in enumerate(c['cameras']):
        eye=c['poses'][:,j,:3];minimum=np.full(len(eye),np.inf)
        for name,tris in triangles().items():
            b=names.index(name);local=Rotation.from_quat(q[:,b,3:]).inv().apply(eye-q[:,b,:3])
            for tri in tris:
                closest=closest_point(np.repeat(tri[None],len(eye),axis=0),local)
                d=np.linalg.norm(local-closest,axis=1)-G.THICKNESS/2
                minimum=np.minimum(minimum,d)
        b=names.index('carton_body');local=Rotation.from_quat(q[:,b,3:]).inv().apply(eye-q[:,b,:3])+G.CENTRES['carton_body']
        # Conservative central carton envelope: if inside this box, investigate
        # manually even if the nearest facet distance is positive.
        inside=(abs(local[:,0])<G.A)&(abs(local[:,1])<G.A)&(local[:,2]>-G.HEIGHT)&(local[:,2]<G.A+G.INNER_FIN)
        # Original station table: x[-.19,.91],y[-.65,.65],top.7475m.
        on_table=(eye[:,0]>=-.19)&(eye[:,0]<=.91)&(abs(eye[:,1])<=.65)
        table_clearance=float(np.min(eye[on_table,2]-.7475)) if on_table.any() else float('inf')
        rows[str(camera)]=dict(table_top_clearance_mm=table_clearance*1000,minimum_surface_clearance_mm=float(minimum.min()*1000),
          minimum_frame=int(minimum.argmin()),envelope_inside_frames=np.flatnonzero(inside).tolist(),
          passes_20mm_sphere=bool(minimum.min()>.020 and not inside.any() and table_clearance>.020))
    return dict(cameras=rows,frames=len(idx),passed=all(x['passes_20mm_sphere'] for x in rows.values()),
       criterion='Original camera origin stays outside central carton envelope and farther than20mm from every conservative full paper polygon. Sphere bound also excludes intersection of the10mm near plane with paper. The same20mm bound is checked above the original table.',
       geometry_note='Full untrimmed polygons enclose the rendered trimmed strips; subtract half paper thickness. Uses measured camera and physical body transforms for every recorded image; no commanded-pose substitution.')

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);a=p.parse_args()
    r=audit(a.run);(a.run/'camera-clearance.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2));return 0 if r['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
