"""CPU-only demo-path geometry audit; never changes simulator state.

Uses URDF convex collision parts, not a link-wide hull. AABBs only select pairs;
overlap is confirmed with convex SAT. Reported positive clearance lower bounds
are conservative; selected closest pairs also get constrained convex distance.
"""
from pathlib import Path
import argparse, hashlib, json, shutil, xml.etree.ElementTree as ET
import numpy as np
import trimesh
from scipy.spatial import ConvexHull
from scipy.optimize import minimize
from yam_arena import YAM_URDF
from yam_arena.urdf import forward_kinematics, rpy_matrix
from yam_arena.ik import tcp_pose, solve, matrix_to_quat_xyzw, LOWER, UPPER
from yam_arena.demo import ArmPlan, grip_rotation
from yam_arena.constants import ARM_JOINTS, READY
from yam_arena.station import BASES
import fixture_geometry as fg
import motion


def unique_axes(a):
    a=np.asarray(a).reshape(-1,3)
    a=a[np.linalg.norm(a,axis=1)>1e-9]
    a=a/np.linalg.norm(a,axis=1)[:,None]
    for i in range(3):
        mask=(np.abs(a[:,:i]).sum(1)<1e-8)&(a[:,i]<0)
        a[mask]*=-1
    return np.unique(np.round(a,8),axis=0)


class Shape:
    def __init__(self,name,vertices):
        self.name=name
        h=ConvexHull(vertices)
        self.v=np.asarray(vertices)[h.vertices]
        h=ConvexHull(self.v)
        self.normals=unique_axes(h.equations[:,:3])
        edges={}
        for face,n in zip(h.simplices,h.equations[:,:3]):
            for a,b in zip(face,np.roll(face,-1)):
                edges.setdefault(tuple(sorted((a,b))),[]).append(n)
        self.edges=unique_axes([self.v[b]-self.v[a] for (a,b),ns in edges.items()
                               if len(ns)<2 or abs(np.dot(ns[0],ns[1]))<1-1e-7])
        self.faces=h.simplices

    def at(self,r=np.eye(3),p=np.zeros(3)):
        return Placed(self,r,p)


class Placed:
    def __init__(self,s,r,p):
        self.shape=s;self.name=s.name;self.r=np.asarray(r);self.p=np.asarray(p)
        self.v=s.v@self.r.T+self.p
        self.lo=self.v.min(0);self.hi=self.v.max(0)


def box(name,size,center):
    return Shape(name,trimesh.creation.box(extents=size).vertices).at(p=center)


def aabb_distance(a,b):
    return float(np.linalg.norm(np.maximum(np.maximum(a.lo-b.hi,b.lo-a.hi),0)))


def sat(a,b):
    na=a.shape.normals@a.r.T;nb=b.shape.normals@b.r.T
    ea=a.shape.edges@a.r.T;eb=b.shape.edges@b.r.T
    axes=unique_axes(np.vstack((na,nb,np.cross(ea[:,None,:],eb[None,:,:]).reshape(-1,3))))
    av=a.v@axes.T;bv=b.v@axes.T
    gap=np.maximum(av.min(0)-bv.max(0),bv.min(0)-av.max(0))
    j=int(np.argmax(gap))
    return dict(overlap=bool(gap[j]<-1e-8),max_separation_mm=float(gap[j]*1000),axis=axes[j].tolist())


def convex_distance(a,b):
    # Six-variable closest-point QP with hull halfspace constraints.
    ha=ConvexHull(a.v).equations;hb=ConvexHull(b.v).equations
    def fun(x):return float(np.dot(x[:3]-x[3:],x[:3]-x[3:]))
    def jac(x):return np.r_[2*(x[:3]-x[3:]),-2*(x[:3]-x[3:])]
    cons=[{'type':'ineq','fun':lambda x:-(ha[:,:3]@x[:3]+ha[:,3])},
          {'type':'ineq','fun':lambda x:-(hb[:,:3]@x[3:]+hb[:,3])}]
    r=minimize(fun,np.r_[a.v.mean(0),b.v.mean(0)],jac=jac,constraints=cons,
               method='SLSQP',options={'ftol':1e-13,'maxiter':200})
    violation=max(0.,float((ha[:,:3]@r.x[:3]+ha[:,3]).max()),float((hb[:,:3]@r.x[3:]+hb[:,3]).max()))
    return dict(distance_mm=float(np.sqrt(max(0,r.fun))*1000),optimizer_success=bool(r.success),
                witness_a=r.x[:3].tolist(),witness_b=r.x[3:].tolist(),max_constraint_violation_m=violation)


def load_parts():
    root=ET.parse(YAM_URDF).getroot();parts={};files={str(YAM_URDF)}
    for link in root.findall('link'):
        entries=[]
        for i,c in enumerate(link.findall('collision')):
            m=c.find('geometry/mesh');path=YAM_URDF.parent/m.get('filename');files.add(str(path))
            mesh=trimesh.load_mesh(path,process=True)
            origin=c.find('origin');r=rpy_matrix(np.fromstring(origin.get('rpy','0 0 0'),sep=' ')) if origin is not None else np.eye(3)
            p=np.fromstring(origin.get('xyz','0 0 0'),sep=' ') if origin is not None else np.zeros(3)
            v=np.asarray(mesh.vertices)*np.fromstring(m.get('scale','1 1 1'),sep=' ')
            entries.append(Shape(f'{link.get("name")}/{i}',v@r.T+p))
        parts[link.get('name')]=entries
    return parts,files


def arm_parts(parts,q,finger,arm):
    angles=dict(zip(ARM_JOINTS,q));angles.update(joint7=finger,joint8=finger)
    fk=forward_kinematics(YAM_URDF,angles)
    return [s.at(fk[n][:3,:3],fk[n][:3,3]+BASES[arm]) for n,ss in parts.items() for s in ss]


class Check:
    def __init__(self):
        self.overlaps=[];self.minimum_lb=float('inf');self.closest=None;self.aabb_candidates=0
    def add(self,aa,bb,t,label):
        al=np.array([a.lo for a in aa]);ah=np.array([a.hi for a in aa])
        bl=np.array([b.lo for b in bb]);bh=np.array([b.hi for b in bb])
        d=np.linalg.norm(np.maximum(np.maximum(al[:,None,:]-bh[None,:,:],bl[None,:,:]-ah[:,None,:]),0),axis=2)
        ij=np.unravel_index(d.argmin(),d.shape)
        if d[ij]<self.minimum_lb:
            self.minimum_lb=float(d[ij]);self.closest=(aa[ij[0]],bb[ij[1]],t,label)
        for i,j in np.argwhere(d<1e-9):
            self.aabb_candidates+=1;a,b=aa[i],bb[j];s=sat(a,b)
            if s['overlap']:self.overlaps.append(dict(time=t,label=label,a=a.name,b=b.name,**s))
    def result(self):
        a,b,t,label=self.closest
        return dict(aabb_clearance_lower_bound_mm=self.minimum_lb*1000,aabb_candidate_count=self.aabb_candidates,
                    sat_overlap_count=len(self.overlaps),worst_overlaps=sorted(self.overlaps,key=lambda x:x['max_separation_mm'])[:20],
                    closest_aabb_candidate=dict(a=a.name,b=b.name,time=t,label=label,**sat(a,b),**convex_distance(a,b)))


def tool_parts_from_snapshot():
    # Read literal geometry from toolkit.py AST, without installing its scene hook.
    import ast
    tree=ast.parse(Path(__file__).with_name('toolkit.py').read_text())
    node=next(n for n in ast.walk(tree) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='parts' for t in n.targets))
    compiled=compile(ast.Expression(node.value),'<toolkit parts>','eval')
    from tool_geometry import TIP_SIZE  # the fine-tip pair is the one inserted
    return {sign:eval(compiled,{'__builtins__':{}},{'sign':sign,'blade':TIP_SIZE}) for sign in (-1,1)}


def static_checks(out):
    """Finite direct-entry poses and alternative withdrawal heights; not a proof that every strategy is blocked."""
    from yam_arena.urdf import axis_angle_matrix
    from copy import deepcopy
    out.mkdir(parents=True,exist_ok=True)
    parts,_=load_parts();carrier=[box(p.name,p.size,fg.CARRIER_CENTER+np.array(p.center)) for p in fg.carrier_parts()]
    cases=[]
    for finger in (0.,-.0067,-.020):
        angles=dict(zip(ARM_JOINTS,READY));angles.update(joint7=finger,joint8=finger)
        fk=forward_kinematics(YAM_URDF,angles);g=fk['gripper'];locals=[]
        for ln in ('gripper','tip_left','tip_right'):
            tf=np.linalg.inv(g)@fk[ln]
            for s in parts[ln]:locals.append(s.at(tf[:3,:3],tf[:3,3]))
        for yaw in (0,45,90):
            for tilt in (0,30,-30,60,-60):
                rot=axis_angle_matrix(np.array([1,0,0]),np.deg2rad(tilt))@grip_rotation(np.deg2rad(yaw))
                rotated=[s.shape.at(rot@s.r,rot@s.p) for s in locals]
                for anchor in ('both','tip_left','tip_right'):
                    pts=np.vstack([s.v for s in rotated if s.name.startswith('tip_') and (anchor=='both' or s.name.startswith(anchor+'/'))])
                    bottom=pts[:,2].min();end=pts[pts[:,2]<bottom+.003]
                    offset=np.array([.28-end[:,0].mean(),-.052-end[:,1].mean(),fg.FILM_Z-.007-bottom])
                    world=[s.shape.at(s.r,s.p+offset) for s in rotated]
                    hits=[]
                    for a in world:
                        for b in carrier:
                            if aabb_distance(a,b)>1e-9:continue
                            r=sat(a,b)
                            if r['overlap']:hits.append(dict(a=a.name,b=b.name,**r))
                    cases.append(dict(finger_joint_m=finger,yaw_deg=yaw,tilt_world_x_deg=tilt,anchor=anchor,
                                      tip_depth_below_film_mm=7,convex_overlap_count=len(hits),
                                      worst_overlap=min(hits,key=lambda x:x['max_separation_mm']) if hits else None))
    heights=[]
    for height in (.163,.173,.183,.193,.203):
        ws=deepcopy(motion.demo()['right'])
        for w in ws:
            if w.label in ('withdraw above tool','clear stand'):w.pos=(*w.pos[:2],fg.TABLE_Z+height)
        p=ArmPlan(ws,BASES['right']);worst=dict(error_mm=0.)
        # Warm-start coarse approach, then use 20Hz for the withdrawal under audit.
        for t in np.r_[np.arange(0,60,.2),np.arange(60,68.001,.05)]:
            q,label=p.action7(float(t));target,_,_,_=p.target(float(t));actual,_=tcp_pose(q[:6],BASES['right'])
            err=float(np.linalg.norm(actual-target)*1000)
            if t>=60 and err>worst['error_mm']:worst=dict(error_mm=err,time=float(t),label=label,actual=actual.tolist(),target=target.tolist(),joints=q[:6].tolist())
        heights.append(dict(tcp_height_above_table_mm=height*1000,worst_withdrawal=worst))
    result=dict(method='Ideal static gripper poses; full convex-part SAT against initial carrier; no claim all strategies blocked.',
                cases=cases,clear_cases=[c for c in cases if not c['convex_overlap_count']],withdrawal_height_sweep=heights,
                source_hashes={n:hashlib.sha256(Path(__file__).with_name(n).read_bytes()).hexdigest() for n in ('audit_geometry.py','motion.py','fixture_geometry.py')})
    (out/'static-checks.json').write_text(json.dumps(result,indent=2)+'\n')
    shutil.copyfile(__file__,out/'audit_geometry-static-source.py')
    print(json.dumps(dict(cases=len(cases),clear_cases=len(result['clear_cases']),heights=heights),indent=2),flush=True)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--hz',type=float,default=20);ap.add_argument('--static-only',action='store_true')
    ap.add_argument('--reference-trace',type=Path,required=True,help='probe.py --mode demo trace (e.g. <out>/0-trace.json)')
    args=ap.parse_args()
    if args.static_only:
        static_checks(args.out);return
    args.out.mkdir(parents=True,exist_ok=True);src=args.out/'source';src.mkdir(exist_ok=True)
    names=('motion.py','fixture_geometry.py','toolkit.py','audit_geometry.py')
    hashes={n:hashlib.sha256(Path(__file__).with_name(n).read_bytes()).hexdigest() for n in names}
    for n in names:shutil.copyfile(Path(__file__).with_name(n),src/n)
    parts,files=load_parts();carrier=[box(p.name,p.size,fg.CARRIER_CENTER+np.array(p.center)) for p in fg.carrier_parts()]
    tab=[box('tab/'+p.name,p.size,fg.TAB_ROOT+np.array(p.center)) for p in fg.tab_parts()]
    plans={s:ArmPlan(ws,BASES[s]) for s,ws in motion.demo().items()}
    checks={k:Check() for k in ('arm_arm','right_hand_carrier','left_hand_carrier','tool_carrier_insert','tool_tab_insert')}
    offset_checks={str(dx):Check() for dx in (-.0012,-.001,-.0008,-.0006)}
    ik=[];max_t=max(p.duration for p in plans.values());tool_geometry=tool_parts_from_snapshot()
    # Measured hand/tool transform from an integrated demo trace (see --reference-trace).
    trace=json.loads(args.reference_trace.read_text())
    # Measured open-tip grip just before insertion: the last 'settle above slot' step.
    ref_step=max(i for i,r in enumerate(trace) if r['right_phase']=='settle above slot');ref=trace[ref_step]
    tcp,hr=tcp_pose(np.array(ref['joint_pos'][7:13]),BASES['right'])
    from geometry import rotation
    relative={}
    for name in ('tweezer_a','tweezer_b'):
        root=np.array(ref['root_poses'][name]);relative[name]=(hr.T@rotation(root[3:]),hr.T@(root[:3]-tcp))
    for step in range(int(max_t*args.hz)+1):
        t=step/args.hz;placed={};labels={};qs={}
        for side,plan in plans.items():
            action,label=plan.action7(t);target,rot,finger,_=plan.target(t);p,r=tcp_pose(action[:6],BASES[side])
            ik.append(dict(time=t,arm=side,label=label,error_mm=float(np.linalg.norm(p-target)*1000),target=target.tolist(),actual=p.tolist(),joint_margin_rad=float(np.min(np.r_[action[:6]-LOWER,UPPER-action[:6]]))))
            placed[side]=arm_parts(parts,action[:6],finger,side);labels[side]=label;qs[side]=action[:6]
        checks['arm_arm'].add(placed['left'],placed['right'],t,labels['right'])
        for side in ('left','right'):
            hand=[p for p in placed[side] if p.name.split('/')[0] in ('gripper','tip_left','tip_right')]
            checks[side+'_hand_carrier'].add(hand,carrier,t,labels[side])
        if labels['right'] in ('insert tips','pinch tab','settle above slot'):
            tcp,hr=tcp_pose(qs['right'],BASES['right']);tool=[]
            for name,sign in [('tweezer_a',-1),('tweezer_b',1)]:
                rr,rp=relative[name];r=hr@rr;p=tcp+hr@rp
                for pn,size,offset in tool_geometry[sign]:
                    s=Shape(name+'/'+pn,trimesh.creation.box(extents=size).vertices)
                    tool.append(s.at(r,p+r@np.array(offset)))
            checks['tool_carrier_insert'].add(tool,carrier,t,labels['right'])
            checks['tool_tab_insert'].add(tool,tab,t,labels['right'])
            for dx,check in offset_checks.items():
                shifted=[s.shape.at(s.r,s.p+np.array([float(dx),0,0])) for s in tool]
                check.add(shifted,carrier+tab,t,labels['right'])
        if step%200==0:print(json.dumps(dict(step=step,time=t,overlaps={k:len(v.overlaps) for k,v in checks.items()})),flush=True)
    result=dict(method='20Hz FK + individual URDF convex-part SAT; AABB only broadphase; SLSQP closest-points on selected candidate',
                limitations=['Commanded joint geometry, not measured physics.', 'Tool insertion uses one measured open-tip hand/tool transform from the reference demo trace; pinch changes and reset-to-reset grip variation are not modeled.', 'Carrier held fixed in its initial pose; film dynamics excluded.', 'Discrete samples do not prove swept continuous clearance.', 'Visual-only yam.urdf has no collisions; yam_arm.urdf supplies 315 convex collision parts.'],
                source_hashes=hashes,source_unchanged=all(hashlib.sha256(Path(__file__).with_name(n).read_bytes()).hexdigest()==h for n,h in hashes.items()),
                reference_trace=str(args.reference_trace),reference_trace_sha256=hashlib.sha256(args.reference_trace.read_bytes()).hexdigest(),reference_step=ref_step,
                hypothetical_tool_x_offsets={dx:c.result() for dx,c in offset_checks.items()},
                offset_note='Rigid translation of predicted tool only; not a full revised robot-path proof. Recalibrate after grip physics changes.',
                collision_asset_hash=hashlib.sha256(b''.join(p.encode()+Path(p).read_bytes() for p in sorted(files))).hexdigest(),
                samples=len(ik)//2,duration_s=max_t,checks={k:c.result() for k,c in checks.items()},
                worst_ik=sorted(ik,key=lambda x:x['error_mm'],reverse=True)[:30],
                per_phase_ik={s:{label:max(x['error_mm'] for x in ik if x['arm']==s and x['label']==label) for label in sorted({x['label'] for x in ik if x['arm']==s})} for s in plans})
    (args.out/'summary.json').write_text(json.dumps(result,indent=2)+'\n');(args.out/'ik-trace.json').write_text(json.dumps(ik))
    print(json.dumps({k:dict(overlaps=v['sat_overlap_count'],lower_bound_mm=v['aabb_clearance_lower_bound_mm']) for k,v in result['checks'].items()},indent=2),flush=True)


if __name__=='__main__':main()
