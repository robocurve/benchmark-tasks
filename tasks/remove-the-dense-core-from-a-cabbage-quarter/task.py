"""A trimmed green cabbage half (a 140 x 76 x 60 mm centre strip) lies cut face up on a white polypropylene cutting
board, its pale triangular core at the root end toward the robot and green leaf layers around it. A black-handled
boning knife lies flat beside it, blade pointing away from the robot. A steel ruler lies near the board's left edge.
The bench and the board are the only supports.
"""
from __future__ import annotations
import json
import math
import os
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
import parts
import lots
import tilt
from fixture import cut_geometry
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import FINGER_OPEN, Waypoint

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
BOARD_T = .020
SURFACE = TOP + BOARD_T
SUCCESS_HOLD_S = 0.0  # score and success share the measured one-second hold
EPS = 1e-6
C = np.array([.33, .070, SURFACE])
GREEN = ((.38,.60,.20), (.57,.72,.32), (.73,.82,.46))
FOOD = ('leaf0','leaf1','leaf2','core')
# Every leaf layer is a horizontal nested group. Small boxes trace the curved envelope and the triangular stem.
# All coordinates are metres; estimated produce geometry, not a scan of a measured specimen.
def core_width(x):
    return max(0., .025 * (-.03-x)/.04)

def leaf_shapes(k):
    z = .010 + k*.020
    out=[]
    for i in range(14):
        x=-.065+i*.010
        w=.038 * math.sqrt(max(.12,1.-(x/.078)**2))
        overlap=[core_width(-.0675+j*.005) for j in range(8) if abs(x-(-.0675+j*.005))<.0049+.00245]
        gap=max(overlap)+.001 if overlap else 0.
        if gap:
            if w>gap:
                for sign in (-1,1):
                    out.append(parts.box((.0098,w-gap,.019), (x,sign*(w+gap)/2,z),GREEN[k]))
        else:
            out.append(parts.box((.0098,2*w,.019),(x,0.,z),GREEN[k]))
    return out
LEAVES = [parts.Part('leaf'+str(k),pos=tuple(float(v) for v in C),mass=.085,shapes=leaf_shapes(k),friction=(.55,.4)) for k in range(3)]
CORE = parts.Part('core',pos=tuple(float(v) for v in C),mass=.045,friction=(.55,.4),shapes=[
    parts.box((.0049,2*core_width(-.0675+i*.005),.058),(-.0675+i*.005,0.,.030),(.87,.87,.66)) for i in range(8)])
# Knife frame at the native handle centre. Blade from +64 to +216 mm, flat in x-y at reset.
# Polymer handle is 128 x 30 x 30 mm; mass is estimated from the published photograph and material volume.
HANDLE_Z=.016
KNIFE_HOME=(.330,-.135,SURFACE+HANDLE_Z)
KNIFE=parts.Part('knife',pos=KNIFE_HOME,mass=.140,friction=(.9,.7),torsion=.012,shapes=[
    parts.box((.116,.030,.030),(-.006,0,0),(.07,.07,.08)),
    parts.box((.012,.038,.034),(.058,0,0),(.10,.10,.11)),
    parts.box((.125,.021,.0018),(.1265,0,-.004),(.72,.74,.76)),
    *[parts.box((.0054,w,.0018),(.1917+i*.0054,-.0105+w/2,-.004),(.72,.74,.76)) for i,w in enumerate(np.linspace(.019,.002,5))],
])
BOARD=parts.Part('cutting_board',pos=(.52,0.,TOP),friction=(.65,.55),shapes=[
    parts.box((.53594,.36830,BOARD_T),(0,0,BOARD_T/2),(.92,.92,.89)),
    *[parts.box((.018,.340,.003),(x,0,.0015),(.25,.25,.25)) for x in (-.25,.25)]])
# Ruler: centimetre ticks, no color-coded cutting instruction.
RULER=parts.Part('ruler',pos=(.35,.175,SURFACE),shapes=[parts.box((.20,.012,.002),(0,0,.001),(.65,.66,.67)),
    *[parts.box((.001,.008 if i%5==0 else .004,.0003),(-.10+i*.01,-.002,.0022),(.05,.05,.05)) for i in range(21)]])
OBJECTS=[BOARD,RULER,*LEAVES,CORE,KNIFE]
# Each mirrored assembly remains a single intact bonded quarter at reset. The demo reads visible poses, not lot names.
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={n:tuple(C) for n in FOOD}),
    lots.Lot('B',place={**{n:(.32,.075,SURFACE) for n in FOOD},'knife':(.330,-.100,SURFACE+HANDLE_Z)},
        yaw={n:math.radians(15) for n in FOOD}))
DEMO_TOUCHES={'cutting_board'}

# Whole-object fiducials make the one-second hold relative to one anchor, not successive small movements.
_MARKS={p.name:np.array([np.asarray(sh.at)+np.array([x,y,z])*sh.half for sh in p.shapes
    for x in (-1,1) for y in (-1,1) for z in (-1,1)]) for p in (*LEAVES,CORE,KNIFE)}

# Declared cutting rule (fixture/cut_geometry.py). A boundary point is cut only when the MOVING, natively held
# blade edge sweeps through it between 2 mm inside the core outline and 10 mm outside it. Points cover both
# 47 mm x 58 mm boundary surfaces at 2 mm spacing. A tip touch or a hover covers almost nothing.
CUT_SAMPLES=cut_geometry.POINTS
# Local cutting edge: the sharp side, with the 27 mm tapered tip.
EDGE=np.array([[.064,-.0105,-.004],[.189,-.0105,-.004],[.216,-.0105,-.004]])

def fresh():
    return dict(known=False,bad=False,bonds=True,layer_bonds=True,cuts=[np.zeros(len(p),bool) for p in CUT_SAMPLES],
        incised=False,incision_groups=[[],[]],prev_world_edge=None,prev_leaf_edges=None,prev_hand_local=None,prev_native=None,prev_edge=None,edge_hist=[],prev_time=None,clear=False,hold=0.,anchor=None,previous=None,
        overcut=0.,strip=0.,residual=.04,peak_load=0.,load=0.,why='',knife_gripped=False)
REC=fresh()

def rot(st): return parts._quat_to_matrix(st.quat_xyzw)
def transform(st,points): return np.asarray(points)@rot(st).T+np.asarray(st.pos)
def local(st,points): return (np.asarray(points)-np.asarray(st.pos))@rot(st)

def segment_distance(points,a,b):
    d=b-a
    t=np.clip((points-a)@d/max(float(d@d),1e-15),0.,1.)
    return np.linalg.norm(points-a-t[:,None]*d,axis=1)

def sweep_distance(points,prev,edge):
    # Interpolate the moving edge at <=1 mm displacement, so a long step cannot skip the volume.
    n=max(1,int(math.ceil(np.linalg.norm(edge-prev,axis=1).max()/.001)))
    ds=np.full(len(points),np.inf)
    for w in np.linspace(0,1,n+1):
        e=prev+(edge-prev)*w
        for a,b in zip(e,e[1:]): ds=np.minimum(ds,segment_distance(points,a,b))
    return ds

def swept_points(prev,edge,spacing=.001):
    n=max(1,int(math.ceil(np.linalg.norm(edge-prev,axis=1).max()/spacing)))
    clouds=[]
    for w in np.linspace(0.,1.,n+1):
        e=prev+w*(edge-prev)
        for a,b in zip(e,e[1:]):
            count=max(2,int(math.ceil(np.linalg.norm(b-a)/spacing))+1)
            clouds.append(a+(b-a)*np.linspace(0.,1.,count)[:,None])
    return np.concatenate(clouds)

def swept_leaf_damage(objects,prev_world,edge,prev_leaf_edges=None):
    # Interpolate in EACH leaf's material frame: moving food past a stationary knife also cuts it.
    worst=0.
    for k in range(3):
        st=objects['leaf'+str(k)];now=local(st,edge)
        before=local(st,prev_world) if prev_leaf_edges is None else prev_leaf_edges['leaf'+str(k)]
        pts=swept_points(before,now);inside=np.zeros(len(pts),bool)
        for sh in LEAVES[k].shapes:
            inside|=np.all(np.abs(pts-np.asarray(sh.at))<=sh.half+.0009,axis=1)
        if not inside.any():continue
        p=pts[inside,:2]
        distance=np.minimum(segment_distance(p,np.array([-.07,-.025]),np.array([-.03,0.])),
                            segment_distance(p,np.array([-.07,.025]),np.array([-.03,0.])))
        worst=max(worst,float(distance.max())+.0009)
    return worst

def segment_box_gap(a,b,center,half,cap=.035):
    """Minimum segment-to-box distance, capped conservatively at35 mm; all coordinates in one body frame."""
    a=np.asarray(a)-center;b=np.asarray(b)-center;d=b-a
    # A segment outside the box expanded by cap cannot get closer than cap in Euclidean distance.
    safe=np.where(np.abs(d)>1e-15,d,1.)
    t0=(-half-cap-a)/safe;t1=(half+cap-a)/safe
    parallel=np.abs(d)<=1e-15
    enter=np.where(parallel,-np.inf,np.minimum(t0,t1));leave=np.where(parallel,np.inf,np.maximum(t0,t1))
    valid=(np.maximum(0.,enter.max(axis=1))<=np.minimum(1.,leave.min(axis=1))) & ~np.any(parallel & (np.abs(a)>half+cap),axis=1)
    if not valid.any():return cap
    a=a[valid];d=d[valid];safe=np.where(np.abs(d)>1e-15,d,1.)
    # Squared distance to an AABB is quadratic between crossings of its six face planes.
    crossings=np.concatenate((np.zeros((len(a),1)),np.ones((len(a),1)),(-half-a)/safe,(half-a)/safe),axis=1)
    crossings=np.sort(np.clip(crossings,0.,1.),axis=1)
    lo=crossings[:,:-1];hi=crossings[:,1:];mid=(lo+hi)/2
    pm=a[:,None,:]+mid[:,:,None]*d[:,None,:]
    outside=np.abs(pm)>half
    c=np.where(outside,a[:,None,:]-np.sign(pm)*half,0.)
    v=np.where(outside,d[:,None,:],0.)
    den=np.sum(v*v,axis=2)
    optimum=np.divide(-np.sum(c*v,axis=2),den,out=mid.copy(),where=den>1e-20)
    optimum=np.minimum(hi,np.maximum(lo,optimum))
    p=a[:,None,:]+optimum[:,:,None]*d[:,None,:]
    dist=np.linalg.norm(np.maximum(np.abs(p)-half,0.),axis=2)
    return min(cap,float(dist.min()))


def observe(objects,hands,t):
    global REC
    r=REC
    if r['prev_time'] is not None and t<=r['prev_time']: return
    dt=.05 if r['prev_time'] is None else min(.1,max(0.,t-r['prev_time']))
    r['prev_time']=float(t)
    if hands is None or not all(n in objects for n in (*FOOD,'knife')):
        r['known']=False;r['hold']=0.;r['bad']=True;r['why']='unknown hand or object state';return
    r['known']=True
    leaf,knife=objects['leaf0'],objects['knife']
    edge=transform(knife,EDGE)
    local_edge=local(leaf,edge)
    # The knife holder may be close to its own handle. Only the opposite/food-holding hand has a blade clearance gate.
    from yam_arena.analysis import box_signed_distance
    native=native_grasps(objects,hands)
    r['knife_gripped']=any(native.values())
    hand_local={arm:local(knife,points) for arm,points in hands.items()}
    for arm,points in hands.items():
        now=hand_local[arm]
        before=now if r['prev_hand_local'] is None else r['prev_hand_local'].get(arm)
        if before is None or before.shape!=now.shape:
            r['bad']=True;r['why']='hand correspondence unavailable';continue
        gap=min(segment_box_gap(before,now,np.asarray(sh.at),sh.half) for sh in KNIFE.shapes[2:])
        # Handle grasp exempts the holder from30 mm clearance, never from blade contact.
        # Full blade solids count, including the spine.3 mm is a conservative face-cloud contact guard.
        if native[arm] and (r['prev_native'] is None or r['prev_native'].get(arm,False)):
            if gap<.003-EPS:r['bad']=True;r['why']='knife holder contacted blade'
            continue
        r['blade_gap']=gap
        if gap<.0322-EPS:
            r['bad']=True;r['why']='blade within30 mm of a hand without a native handle grasp'
    r['prev_hand_local']={arm:p.copy() for arm,p in hand_local.items()};r['prev_native']=native.copy()
    moving=r['prev_edge'] is not None and np.linalg.norm(local_edge-r['prev_edge'])>.00015
    # The edge must lead: judged on the edge's motion over the last 0.5 s (EDGE_WINDOW), in the quarter frame.
    blade=rot(leaf).T@rot(knife)
    r['edge_hist']=[(tt,e) for tt,e in r['edge_hist'] if tt>=t-EDGE_WINDOW-1e-9]+[(float(t),local_edge.copy())]
    leads=edge_leads(local_edge.mean(axis=0)-r['edge_hist'][0][1].mean(axis=0),blade)
    active=False
    if moving and r['knife_gripped'] and leads:
        active=cut_geometry.sweep(r,r['prev_edge'],local_edge)

    if r['prev_world_edge'] is not None:
        damage=swept_leaf_damage(objects,r['prev_world_edge'],edge,r['prev_leaf_edges'])
        r['overcut']=max(r['overcut'],damage);r['strip']=r['overcut']
    r['prev_leaf_edges']={p.name:local(objects[p.name],edge) for p in LEAVES}
    r['prev_world_edge']=edge.copy()
    r['prev_edge']=local_edge.copy()
    # Coverage is evaluated over the whole boundary, not just a predetermined tip position.
    r['bonds']=not all(np.mean(v)>=.98 for v in r['cuts'])
    # Pulling cannot detach an intact core. A displacement while bonded is a broken constraint, not success.
    if r['bonds'] and np.linalg.norm(objects['core'].pos-leaf.pos)>.015:
        r['bad']=True;r['why']='core moved before complete cuts'
    # Full destructive cuts or gross separation permanently disallow useful credit.
    if r['overcut']>.025+EPS:
        r['bad']=True;r['why']='destructive edible overcut over 25 mm'
    r['active']=active
    r['clear']=True
    allpoints=np.concatenate(list(hands.values()))
    for name in (*FOOD,'knife'):
        st=objects[name]
        spec=next(p for p in OBJECTS if p.name==name)
        radius=max(np.linalg.norm(np.asarray(sh.at))+np.linalg.norm(sh.half) for sh in spec.shapes)+.008
        near=allpoints[np.linalg.norm(allpoints-st.pos,axis=1)<radius]
        if not len(near):continue
        for shape in next(p for p in OBJECTS if p.name==name).shapes:
            if box_signed_distance(near,transform(st,[shape.at])[0],rot(st),2*shape.half).min()<=.0072-EPS:
                r['clear']=False;break
        if not r['clear']:break
    current=np.concatenate([transform(objects[n],_MARKS[n]) for n in (*FOOD,'knife')])
    still=all(objects[n].resting(.02,.5) for n in (*FOOD,'knife'))
    supported=all(objects[n].low[2]>=SURFACE-.003 and objects[n].low[2]<=SURFACE+.003 for n in ('leaf0','core','knife'))
    if not (still and supported and r['clear'] and not r['bad']):
        r['anchor']=None;r['hold']=0.
    elif r['anchor'] is None or np.linalg.norm(current-r['anchor'],axis=1).max()>.001+EPS:
        r['anchor']=current.copy();r['hold']=0.
    else:r['hold']+=dt
    r['previous']=current

EDGE_WINDOW=.5
MAX_OFF_PLANE=math.radians(45)
def edge_leads(d,blade):
    """True if a blade (rotation in the quarter frame) moving by d cuts: edge first or drawn along its length, within
    45 degrees of its own plane, never spine first or flat side first. Under 1 mm of motion in 0.5 s does not cut."""
    d=np.asarray(d,float);size=np.linalg.norm(d)
    if size<.001:return False
    normal=blade[:,2];facing=-blade[:,1]
    inplane=d-(d@normal)*normal
    if math.atan2(abs(d@normal),np.linalg.norm(inplane))>MAX_OFF_PLANE+EPS:return False
    return d@facing>=-.2*size

def record_incisions(before,after):
    # Test helper: one held-blade step, recorded exactly as observe() records it.
    cut_geometry.sweep(REC,np.asarray(before),np.asarray(after))

def incision():
    return REC['incised']

def start_marks(name):
    lot=lots.current();R=parts._rot((0,0,1),lot.yaw_of(name))
    return _MARKS[name]@R.T+np.asarray(lot.pos(name))

def assembled(objects):
    # Each leaf layer stays within 10 mm of its place: relative to the bottom layer and to where it started.
    base=objects['leaf0']
    for k in (0,1,2):
        n='leaf'+str(k);st=objects[n];actual=transform(st,_MARKS[n])
        if np.linalg.norm(actual-start_marks(n),axis=1).max()>.010+EPS:return False
        if k and np.linalg.norm(actual-transform(base,_MARKS[n]),axis=1).max()>.010+EPS:return False
    return True

_CORE_SURFACE=[]
for sh in CORE.shapes:
    for axis in range(3):
        other=[i for i in range(3) if i!=axis]
        for sign in (-1,1):
            for aa in np.linspace(-sh.half[other[0]],sh.half[other[0]],max(2,int(math.ceil(2*sh.half[other[0]]/.0025))+1)):
                for bb in np.linspace(-sh.half[other[1]],sh.half[other[1]],max(2,int(math.ceil(2*sh.half[other[1]]/.0025))+1)):
                    pt=np.asarray(sh.at).copy();pt[axis]+=sign*sh.half[axis];pt[other[0]]+=aa;pt[other[1]]+=bb;_CORE_SURFACE.append(pt)
_CORE_SURFACE=np.unique(np.round(_CORE_SURFACE,7),axis=0)

def residual_span():
    """Largest connected uncut patch, including a conservative2 mm-grid cell diameter guard."""
    pts=np.concatenate([p[~mask] for p,mask in zip(CUT_SAMPLES,REC['cuts'])])
    if not len(pts):return 0.
    distance=np.linalg.norm(pts[:,None,:]-pts[None,:,:],axis=2)
    pending=set(range(len(pts)));worst=0.
    while pending:
        todo=[pending.pop()];component=[]
        while todo:
            i=todo.pop();component.append(i)
            neighbors=set(np.flatnonzero(distance[i]<=.003+EPS)) & pending
            pending.difference_update(neighbors);todo.extend(neighbors)
        worst=max(worst,float(distance[np.ix_(component,component)].max())+2*math.sqrt(2)*.001)
    return worst

def score(objects):
    r=REC
    if not r['known'] or r['bad'] or not r['clear'] or r['hold']<1.-EPS or not incision(): return 0.
    if not r['bonds'] and r['layer_bonds']:return 0.
    # Detached means at least 10 mm actual gap between core and all retained shapes.
    from yam_arena.analysis import box_signed_distance
    st=objects['core'];p=transform(st,_CORE_SURFACE)
    detached=not r['bonds'] and all(min(box_signed_distance(p,transform(objects[n],[s.at])[0],rot(objects[n]),2*s.half).min()
        for s in LEAVES[int(n[-1])].shapes)>.0118-EPS for n in ('leaf0','leaf1','leaf2'))
    if not detached or not assembled(objects) or r['overcut']>.010+EPS or r['strip']>.010+EPS: return .33
    # The whole dense core is one tracked body; both surfaces must have 98% coverage before it can leave.
    # The maximum contiguous uncut remnant must be <=10 mm for full credit.
    residual=residual_span()
    if r['overcut']<=.005+EPS and r['strip']<=.005+EPS and residual<=.010+EPS: return 1.
    return .67

def success(objects): return score(objects)==1.

# Face-sampled hand hulls. At 3 mm spacing, a planar hand face cannot slip through a vertex-only release test.
_FACE_CACHE={}
def face_points(name):
    if name in _FACE_CACHE:return _FACE_CACHE[name]
    from yam_arena import analysis
    pts=[]
    for h in analysis.link_hulls().get(name,[]):
        pts.append(h.vertices)
        for plane in np.unique(np.round(h.planes,7),axis=0):
            n,d=plane[:3],plane[3]
            on=h.vertices[np.abs(h.vertices@n-d)<1e-6]
            if len(on)<3:continue
            c=on.mean(axis=0);u=on[0]-c;u/=np.linalg.norm(u);v=np.cross(n,u)
            ring=on[np.argsort(np.arctan2((on-c)@v,(on-c)@u))]
            for a,b in zip(ring,np.roll(ring,-1,axis=0)):
                count=max(1,int(math.ceil(max(np.linalg.norm(a-c),np.linalg.norm(b-c),np.linalg.norm(a-b))/.003)))
                ii,jj=np.meshgrid(np.arange(count+1),np.arange(count+1));valid=ii+jj<=count
                pts.append(c+(a-c)*ii[valid,None]/count+(b-c)*jj[valid,None]/count)
    if not pts:raise ValueError('missing hand hull '+name)
    p=np.concatenate(pts);_,ix=np.unique(np.round(p/.0015).astype(int),axis=0,return_index=True)
    _FACE_CACHE[name]=p[np.sort(ix)];return _FACE_CACHE[name]

class HandClouds(dict):
    """Collision-face points plus native pad and finger observations; unavailable data never implies a grasp."""
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs);self.pads={};self.fingers={}

def native_grasps(objects,hands):
    from yam_arena.analysis import box_signed_distance
    knife=objects['knife'];handle=KNIFE.shapes[0]
    out={arm:False for arm in hands}
    for arm in hands:
        pads=getattr(hands,'pads',{}).get(arm);q=getattr(hands,'fingers',{}).get(arm)
        if pads is None or q is None or not np.isfinite(q).all() or float(np.sum(np.abs(q)))>=.070:continue
        sides=[]
        for pad in pads:
            distance=box_signed_distance(pad,transform(knife,[handle.at])[0],rot(knife),2*handle.half)
            contact=pad[distance<=.002+EPS]
            if not len(contact):sides=[];break
            y=float(np.median(local(knife,contact)[:,1]));sides.append(y)
        out[arm]=len(sides)==2 and sides[0]*sides[1]<0 and min(map(abs,sides))>=handle.half[1]-.004
    return out

def food_compressed(objects,hands):
    from yam_arena.analysis import box_signed_distance
    for arm in hands:
        pads=getattr(hands,'pads',{}).get(arm);q=getattr(hands,'fingers',{}).get(arm)
        if pads is None or q is None or not np.isfinite(q).all() or float(np.sum(np.abs(q)))>=.085:continue
        touch=[]
        for pad in pads:
            hit=False
            for p in LEAVES:
                st=objects[p.name];near=pad[np.all((pad>=st.low-.003)&(pad<=st.high+.003),axis=1)]
                if not len(near):continue
                for sh in p.shapes:
                    if box_signed_distance(near,transform(st,[sh.at])[0],rot(st),2*sh.half).min()<=.002+EPS:
                        hit=True;break
                if hit:break
            touch.append(hit)
        if all(touch):return True
    return False

def hand_clouds(sim):
    base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);out=HandClouds()
    for arm in ('left','right'):
        asset=base.scene[arm+'_robot'];poses=parts._np(asset.data.body_link_pose_w)[0];pts=[];named={}
        for n in ('gripper','tip_left','tip_right'):
            i=list(asset.body_names).index(n);pose=poses[i]
            named[n]=face_points(n)@parts._quat_to_matrix(pose[3:]).T+pose[:3]-origin;pts.append(named[n])
        out[arm]=np.concatenate(pts)
        out.pads[arm]=(named['tip_left'],named['tip_right'])
        q=parts._np(asset.data.joint_pos)[0];out.fingers[arm]=np.array([q[list(asset.joint_names).index(n)] for n in ('joint7','joint8')])
    if not all(np.isfinite(p).all() for p in out.values()):raise ValueError('nonfinite hands')
    return out

# Declare real tissue bonds as PhysX fixed joints, not welded grippers or fixed produce.
_PATHS={};_JOINTS=[];_original_author=parts._author

def author(part,root_path,translation,orientation):
    root=_original_author(part,root_path,translation,orientation)
    _PATHS[part.name]=root_path
    if not all(n in _PATHS for n in (*FOOD,'knife')):return root
    from pxr import Gf,Sdf,UsdPhysics
    from isaaclab.sim.utils import get_current_stage
    stage=get_current_stage()
    for n in FOOD[1:]:
        path=_PATHS[n]+'/tissue_bond'
        j=UsdPhysics.FixedJoint.Define(stage,path)
        j.CreateBody0Rel().SetTargets([Sdf.Path(_PATHS['leaf0'])]);j.CreateBody1Rel().SetTargets([Sdf.Path(_PATHS[n])])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(0));j.CreateLocalPos1Attr().Set(Gf.Vec3f(0))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(1));j.CreateLocalRot1Attr().Set(Gf.Quatf(1))
        # A disabled bond must not leave its former pair collision-filtered: free layers must stack naturally.
        j.CreateCollisionEnabledAttr().Set(True)
        j.CreateExcludeFromArticulationAttr(True);_JOINTS.append((n,path))
    # Knife-food response is the declared geometric material rule. Ordinary hand-food and hand-knife contacts remain.
    pairs=UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(_PATHS['knife']))
    pairs.CreateFilteredPairsRel().SetTargets([Sdf.Path(_PATHS[n]) for n in FOOD])
    return root
parts._author=author

def set_bonds(enabled,members=FOOD[1:]):
    from pxr import UsdPhysics
    from isaaclab.sim.utils import get_current_stage
    stage=get_current_stage()
    for name,path in _JOINTS:
        if name in members:UsdPhysics.Joint(stage.GetPrimAtPath(path)).CreateJointEnabledAttr().Set(bool(enabled))

@parts.on_pre_reset
def prepare_reset(sim):
    # Restore intact tissue BEFORE the stock body reset and settle, with no intervening physics step.
    # Otherwise a prior detached core/layer can settle independently and be yanked together at reset end.
    if _JOINTS:set_bonds(True)

@parts.on_reset
def reset(sim):
    global REC
    REC=fresh();set_bonds(True)

@parts.on_step
def step(sim):
    try:
        objects=sim.states();hands=hand_clouds(sim)
        old=REC['bonds'];observe(objects,hands,float(sim.t))
        if old and not REC['bonds']:set_bonds(False,('core',))
        if not REC['bonds'] and REC['layer_bonds'] and not food_compressed(objects,hands):
            set_bonds(False,('leaf1','leaf2'));REC['layer_bonds']=False
        # Geometry-only material response: no cutting force or bending moment is applied.
        REC['load']=0.
        if os.environ.get('TRACE_OUT') and round(sim.t*20)%5==0:
            row=dict(t=float(sim.t),cuts=[float(v.mean()) for v in REC['cuts']],bad=REC['bad'],why=REC['why'],
                overcut=REC['overcut'],bonds=REC['bonds'],layer_bonds=REC['layer_bonds'],hold=REC['hold'],clear=REC['clear'],score=score(objects),
                load=REC['load'],lot=lots.current().name,blade_gap=REC.get('blade_gap'),leaf_quat=[objects[n].quat_xyzw.tolist() for n in FOOD],knife_quat=objects['knife'].quat_xyzw.tolist(),knife=objects['knife'].pos.tolist(),leaves=[objects[n].pos.tolist() for n in FOOD])
            with Path(os.environ['TRACE_OUT']).with_name('cutting.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
    except Exception as e:
        REC['known']=False;REC['bad']=True;REC['why']=str(e)
        if os.environ.get('TRACE_OUT'):
            with Path(os.environ['TRACE_OUT']).with_name('cutting-errors.txt').open('a') as f:f.write(repr(e)+'\n')
        raise

AIM_OUT=.001
AIM_PAST_APEX=-.001
CALIBRATION=json.loads((Path(__file__).resolve().parent/'fixture'/'calibration.json').read_text())

# Open-loop motion derives from the published camera-visible starting poses. No state feedback during the trial.
def demo():
    lot=lots.current()
    # Position/yaw are visible reset geometry, not an invisible trial identity or physics parameter.
    cp=np.array(lot.pos('leaf0'));angle=lot.yaw_of('leaf0');R=parts._rot((0,0,1),angle)
    kp=np.array(lot.pos('knife'));ky=lot.yaw_of('knife')
    toolarm='right';holdarm='left'
    # The published visible root direction determines both cut approaches.
    def world(p):return tuple(float(v) for v in cp+R@np.array(p))
    def wp(d,p,f,label,yaw=0.,lean=0.,heading=0.,rotation=None):
        w=tilt.Waypoint(d,tuple(float(v) for v in p),f,label,yaw=yaw,tilt=lean,heading=heading)
        if rotation is not None:w.yaw=tilt._TiltedYaw(yaw,rotation)
        return w
    h=world((.055,0,.053))
    hp=world((.035,0,.110))
    holding=[wp(2.,hp,FINGER_OPEN,'approach quarter'),wp(2.,h,FINGER_OPEN,'descend to leaves'),
        wp(1.,h,-.020,'counterhold 54 mm quarter'),wp(50.,h,-.020,'stabilize during both cuts'),
        wp(1.,h,FINGER_OPEN,'release leaves'),wp(2.,hp,FINGER_OPEN,'clear leaves')]
    p=kp+np.array([0,0,.010]);hover=kp+np.array([0,0,.090])
    tool=[wp(2.,hover,FINGER_OPEN,'above native handle',ky),wp(2.,p,FINGER_OPEN,'around handle',ky),
        wp(1.,p,-.007,'grip native handle',ky),wp(2.,hover,-.007,'lift',ky),
        wp(2.,(float(cp[0]-.22),float(kp[1]),float(SURFACE+.108)),-.007,'move behind core before turning',ky)]
    # Published static calibration compensates the repeatable quarter tilt and native-arm tracking error.
    # It never reads an object pose or hidden cut state during execution.
    visible_yaw=math.degrees(angle)
    table=CALIBRATION['by_visible_yaw']
    cal=table[min(table,key=lambda key:abs(float(key)-visible_yaw))]
    workC=cp+R@np.asarray(cal['quarter_shift'])
    workR=R@np.asarray(cal['quarter_rotation'])
    down=tilt.rotation(0.)
    for cut_i,side in enumerate((-1,1)):
        theta_local=math.atan2(-side*.025,.04)
        desired=workR@parts._rot((0,0,1),theta_local)@parts._rot((1,0,0),math.pi/3)
        bias=R@np.asarray(cal['cut'][cut_i]['rotation_bias'])@R.T
        shift=R@np.asarray(cal['cut'][cut_i]['tcp_shift'])
        command=bias.T@desired@down
        # Aim the edge 1 mm outside the boundary (leaf side) and the tip 1 mm short of the apex: at the bottom of the
        # stroke the blade meets the board and slides a few mm forward and outward (measured in traces).
        along=np.array([math.cos(theta_local),math.sin(theta_local),0.])
        outward=side*np.array([-along[1],along[0],0.])
        tip=workC+workR@(np.array([-.030,0.,.060])+AIM_PAST_APEX*along+AIM_OUT*outward)
        high=tip-desired@EDGE[-1]+desired@np.array([0,0,.010])+workR@np.array([0,0,.030])-shift
        low=high-workR@np.array([0,0,.093]);back=high-workR@np.array([.075,0,0])
        yaw=math.atan2(desired[1,0],desired[0,0])
        tool.extend([wp(2.,back,-.007,f'cut{cut_i}: orient clear of holding hand',yaw,rotation=command),
            wp(2.,high,-.007,f'cut{cut_i}: approach core boundary',yaw,rotation=command),
            wp(5.,low,-.007,f'cut{cut_i}: sweep boundary',yaw,rotation=command),
            wp(1.,low-workR[:,0]*.020,-.007,f'cut{cut_i}: draw blade out before lifting',yaw,rotation=command),
            wp(2.,high-workR[:,0]*.020,-.007,f'cut{cut_i}: withdraw blade',yaw,rotation=command),
            wp(2.,back,-.007,f'cut{cut_i}: move clear before turning',yaw,rotation=command)])
    clear_side=np.array([float(cp[0]-.15),float(kp[1]),float(high[2])])
    tool.extend([wp(2.,clear_side,-.007,'return behind quarter before turning',yaw,rotation=command),
        wp(2.,clear_side,-.007,'flatten knife clear of holding hand',ky),
        wp(3.,hover,-.007,'return knife',ky),wp(2.,p+np.array([0,0,.007]),-.007,'lower knife',ky),
        wp(1.,p+np.array([0,0,.007]),FINGER_OPEN,'release knife',ky),wp(2.,hover,FINGER_OPEN,'clear knife',ky)])
    # Grasp the detached 30 mm core only after the holder has moved away.
    corepick=world((-.082,0,.045));coredrop=world((-.020,-.100,.045))
    holding.extend([wp(8.,hp,FINGER_OPEN,'wait knife clear'),wp(2.,world((-.082,0,.105)),FINGER_OPEN,'over core'),
        wp(2.,corepick,FINGER_OPEN,'down to freed core'),wp(1.,corepick,-.018,'grasp exposed root face'),
        wp(2.,world((-.132,0,.045)),-.018,'draw core out through the open root end'),
        wp(2.,world((-.132,0,.105)),-.018,'lift core clear'),
        wp(2.,world((-.132,-.100,.105)),-.018,'move sideways clear of leaves'),
        wp(2.,world((-.020,-.100,.085)),-.018,'carry core over the board'),
        wp(2.,coredrop,-.018,'lower core'),wp(1.,coredrop,FINGER_OPEN,'release core'),
        wp(2.,world((-.020,-.100,.085)),FINGER_OPEN,'clear core'),wp(4.,hp,FINGER_OPEN,'still hold')])
    tool.extend([wp(35.,hover,FINGER_OPEN,'remain clear',ky)])
    for waypoint in holding:waypoint.yaw=angle
    return {holdarm:holding,toolarm:tool}
