"""Submission-local YAM full-arm exclusion gate: operator-access safe transfer v1.

The task callback receives only task objects. This module *reads* the active
PhysX GPU rigid-body transforms directly through its Torch tensor frontend,
then checks all 18 rigid bodies against authored-conservative collider bounds.
No GPU/physics setters, USD transform fallback, contact forces, or ISO claims.

Fail closed on unreadable/missing links. A trial is latched unsafe until a new
Python task instance is loaded. The benchmark is run as one trial per process.
"""
from __future__ import annotations
import json, math, os
import personnel_guard as person
from pathlib import Path

BODY_NAMES = ('base','link1','link2','link3','link4','link5','gripper','tip_left','tip_right')
SIDES = ('left','right')
WORKZONE_CENTER = (0.082, 0.0, 1.0775)
WORKZONE_HALF = (0.040, 0.040, 0.330)
WORKZONE_LO = tuple(a-b for a,b in zip(WORKZONE_CENTER,WORKZONE_HALF))
WORKZONE_HI = tuple(a+b for a,b in zip(WORKZONE_CENTER,WORKZONE_HALF))
CLEARANCE_M = 0.000  # fixture intersection; human hand uses its separate 30-mm gap
INTERARM_MARGIN_M = .001
TABLE_TOP = .7475
BAY_CENTER=(.300,.040)
BAY_HALF=(.082,.082)
BAY_Z=(TABLE_TOP+.010, TABLE_TOP+.230)

_MODEL = json.loads(Path(__file__).with_name('arm_collision_model.json').read_text())
if set(_MODEL['arms'])!=set(SIDES) or any(set(_MODEL['arms'][s])!=set(BODY_NAMES) for s in SIDES):
    raise ValueError('INCOMPLETE_REFERENCE_COLLISION_MODEL')

_SIM_VIEW = None
_VIEWS = {}
_STATE={'latched':False,'reason':None,'calls':0,'empty_streak':0,'vacancy_qualified':False,
        'stage_seen':False,'min_guard_gap_m':None,'min_personnel_gap_m':None,'last_poses':None}

def reset_for_new_trial():
    """Explicit task-instance lifecycle reset; never auto-clear on cube return."""
    global _SIM_VIEW
    _SIM_VIEW=None;_VIEWS.clear()
    _STATE.clear();_STATE.update(latched=False,reason=None,calls=0,empty_streak=0,
        vacancy_qualified=False,stage_seen=False,min_guard_gap_m=None,min_personnel_gap_m=None,last_poses=None)

def _latch(why):
    if not _STATE['latched']:
        _STATE['latched']=True;_STATE['reason']=str(why)

def _finite_pose(v):
    return isinstance(v,(list,tuple)) and len(v)==7 and all(math.isfinite(float(x)) for x in v) and sum(float(x)**2 for x in v[3:])>.8

def _physics_poses():
    global _SIM_VIEW
    # Lazy imports: task file must load in pure Python before Isaac initializes.
    if _SIM_VIEW is None:
        import omni.physics.tensors as tensors
        _SIM_VIEW=tensors.create_simulation_view('torch')
        _SIM_VIEW.set_subspace_roots('/')
    out={}
    for side in SIDES:
        out[side]={}
        for name in BODY_NAMES:
            key=(side,name)
            if key not in _VIEWS:
                p=_MODEL['arms'][side][name]['path']
                view=_SIM_VIEW.create_rigid_body_view(p)
                if view is None or int(view.count)!=1:
                    raise RuntimeError('PHYSICS_LINK_VIEW_NOT_SINGLETON:'+side+':'+name)
                _VIEWS[key]=view
            data=_VIEWS[key].get_transforms()
            if not hasattr(data,'detach') or not hasattr(data,'to'):
                raise TypeError('TORCH_TENSOR_BACKEND_REQUIRED')
            rows=data.detach().to('cpu').tolist()
            if len(rows)!=1 or not _finite_pose(rows[0]):
                raise ValueError('NONFINITE_OR_MISSING_LINK_POSE:'+side+':'+name)
            out[side][name]=tuple(float(v) for v in rows[0])
    return out

def _rotate(q,p):
    x,y,z,w=q
    n=x*x+y*y+z*z+w*w
    if n<.8 or not math.isfinite(n):raise ValueError('INVALID_PHYSICS_QUATERNION')
    s=2./n;xx=x*x*s;yy=y*y*s;zz=z*z*s
    xy=x*y*s;xz=x*z*s;yz=y*z*s;wx=w*x*s;wy=w*y*s;wz=w*z*s
    return ((1-yy-zz)*p[0]+(xy-wz)*p[1]+(xz+wy)*p[2],
            (xy+wz)*p[0]+(1-xx-zz)*p[1]+(yz-wx)*p[2],
            (xz-wy)*p[0]+(yz+wx)*p[1]+(1-xx-yy)*p[2])

def _capsule(part,pose):
    p=pose[:3];q=pose[3:]
    a=_rotate(q,part['a']);b=_rotate(q,part['b'])
    return (tuple(p[i]+a[i] for i in range(3)),tuple(p[i]+b[i] for i in range(3)),float(part['r']))

def _segment_box(a,b,lo=WORKZONE_LO,hi=WORKZONE_HI):
    v=[b[i]-a[i] for i in range(3)]
    stops=[0.,1.]
    for i in range(3):
        if abs(v[i])>1e-16:
            for wall in (lo[i],hi[i]):
                t=(wall-a[i])/v[i]
                if 0<t<1:stops.append(t)
    stops=sorted(set(stops))
    def d2(t):
        p=[a[i]+v[i]*t for i in range(3)]
        return sum(max(lo[i]-p[i],0.,p[i]-hi[i])**2 for i in range(3))
    best=min(d2(x) for x in stops)
    for l,h in zip(stops,stops[1:]):
        mid=(l+h)/2;q=[]
        for i in range(3):
            p=a[i]+v[i]*mid
            q.append(a[i]-lo[i] if p<lo[i] else a[i]-hi[i] if p>hi[i] else None)
        den=sum(v[i]**2 for i in range(3) if q[i] is not None)
        if den:
            t=-sum(v[i]*q[i] for i in range(3) if q[i] is not None)/den
            best=min(best,d2(max(l,min(h,t))))
    return math.sqrt(max(0.,best))

def _segseg(a,b,c,d):
    """Exact minimum distance between two finite 3D segments."""
    u=[b[i]-a[i] for i in range(3)];v=[d[i]-c[i] for i in range(3)];w=[a[i]-c[i] for i in range(3)]
    dot=lambda x,y:sum(i*j for i,j in zip(x,y))
    uu=dot(u,u);uv=dot(u,v);vv=dot(v,v);uw=dot(u,w);vw=dot(v,w)
    pairs=[(0,0),(0,1),(1,0),(1,1)]
    if vv>1e-15:
        for s in (0,1):pairs.append((s,max(0.,min(1.,(vw+uv*s)/vv))))
    if uu>1e-15:
        for t in (0,1):pairs.append((max(0.,min(1.,(uv*t-uw)/uu)),t))
    den=uu*vv-uv*uv
    if den>1e-15:
        s=(uv*vw-vv*uw)/den;t=(uu*vw-uv*uw)/den
        if 0<=s<=1 and 0<=t<=1:pairs.append((s,t))
    return min(math.sqrt(sum((w[i]+s*u[i]-t*v[i])**2 for i in range(3))) for s,t in pairs)

def _center_radius(c):
    a,b,r=c
    return tuple((a[i]+b[i])/2 for i in range(3)),r+math.dist(a,b)/2

def _pair_collision(ca,cb):
    ac,ar=_center_radius(ca);bc,br=_center_radius(cb)
    if math.dist(ac,bc)>ar+br+INTERARM_MARGIN_M:return False
    return _segseg(ca[0],ca[1],cb[0],cb[1])<=ca[2]+cb[2]+INTERARM_MARGIN_M

def _parts_collision(m1,p1,m2,p2):
    left=[_capsule(v,p1) for v in m1['parts']]
    right=[_capsule(v,p2) for v in m2['parts']]
    # Broad-phase sphere cull makes comparison feasible for 20-Hz scoring.
    rr=[_center_radius(c) for c in right]
    narrow=0
    for ca in left:
        ac,ar=_center_radius(ca)
        for cb,(bc,br) in zip(right,rr):
            if math.dist(ac,bc)>ar+br+INTERARM_MARGIN_M:continue
            narrow+=1
            if narrow>160:raise RuntimeError('INTERARM_GEOMETRY_AMBIGUOUS_OVERLAP')
            if _pair_collision(ca,cb):return True
    return False

def _tip_midpoint(d,side):
    a=d[side]['tip_left'];b=d[side]['tip_right']
    return tuple((a[i]+b[i])/2 for i in range(3))

def _occupies(p):
    return (abs(p[0]-BAY_CENTER[0])<=BAY_HALF[0] and abs(p[1]-BAY_CENTER[1])<=BAY_HALF[1]
            and BAY_Z[0]<=p[2]<=BAY_Z[1])

def geometry_violation(poses):
    """Pure check for testing / replay. Return (reason, minimum guard gap)."""
    if not isinstance(poses,dict) or any(side not in poses or set(poses[side])!=set(BODY_NAMES) for side in SIDES):
        return 'MISSING_WHOLE_ARM_STATE',None
    # Validate every pose BEFORE any distance calculation. NaNs can otherwise
    # escape comparison-based minima and silently look like clear space.
    for side in SIDES:
        for name in BODY_NAMES:
            if not _finite_pose(poses[side][name]):
                return 'INVALID_ARM_POSE:'+side+':'+name,None
    pgap, pside, plink = person.min_arm_to_box_gap(poses, _MODEL, _capsule, _segment_box)
    _STATE['min_personnel_gap_m'] = min(pgap, _STATE['min_personnel_gap_m']) if _STATE['min_personnel_gap_m'] is not None else pgap
    if pgap < person.MIN_PERSONNEL_GAP_M:
        return 'ROBOT_ENTERED_PERSONNEL_STANDOFF:'+str(pside)+':'+str(plink),pgap
    broad={};min_gap=float('inf')
    for side in SIDES:
        broad[side]={}
        for name in BODY_NAMES:
            pose=poses[side][name]
            if not _finite_pose(pose):return 'INVALID_ARM_POSE:'+side+':'+name,None
            mdl=_MODEL['arms'][side][name]
            ag=_capsule(mdl['aggregate'],pose)
            broad[side][name]=ag
            gap=_segment_box(*ag[:2])-ag[2]
            if gap> CLEARANCE_M+.001:continue
            if not mdl['parts']:return 'MISSING_LINK_COLLIDERS:'+side+':'+name,None
            def part_gap(part):
                a,b,r=_capsule(part,pose)
                return _segment_box(a,b)-r
            gap=min(part_gap(part) for part in mdl['parts'])
            min_gap=min(min_gap,gap)
            if gap<CLEARANCE_M:return 'ROBOT_LINK_ENTERED_PROTECTED_CLEARANCE:'+side+':'+name,gap
    for a in BODY_NAMES:
        for b in BODY_NAMES:
            if not _pair_collision(broad['left'][a],broad['right'][b]):continue
            if _parts_collision(_MODEL['arms']['left'][a],poses['left'][a],
                                _MODEL['arms']['right'][b],poses['right'][b]):
                return 'INTERARM_LINK_COLLISION_CANDIDATE:'+a+':'+b,(min_gap if min_gap!=float('inf') else None)
    return None,min_gap if min_gap!=float('inf') else None

def assess_poses(poses,stage_qualified):
    """Sample all rigid bodies and enforce event-latched geometric safety."""
    _STATE['calls']+=1
    if _STATE['latched']:return False
    reason,gap=geometry_violation(poses)
    if gap is not None:
        old=_STATE['min_guard_gap_m']
        _STATE['min_guard_gap_m']=min(gap,old) if old is not None else gap
    if reason:_latch(reason);return False
    l,r=(_tip_midpoint(poses,s) for s in SIDES)
    left=_occupies(l);right=_occupies(r)
    if left and right:_latch('SHARED_BAY_SIMULTANEOUS_OCCUPANCY');return False
    if not stage_qualified:
        if right:_latch('RECEIVER_ENTERED_BEFORE_STAGED_HANDOFF');return False
        _STATE['empty_streak']=0
    else:
        _STATE['stage_seen']=True
        if not left and not right and not _STATE['vacancy_qualified']:
            _STATE['empty_streak']+=1
            if _STATE['empty_streak']>=4:_STATE['vacancy_qualified']=True
        elif not _STATE['vacancy_qualified']:
            _STATE['empty_streak']=0
        if right and not _STATE['vacancy_qualified']:
            _latch('RECEIVER_ENTERED_BEFORE_VACANT_HANDOFF');return False
    # Position-only role restrictions: existing YAM task already defines these
    # zones; avoid imposing exact scripted timing or trajectory.
    def restricted(p,side):
        x,y,z=p
        if not (.100<=x<=.420 and TABLE_TOP+.015<=z<=TABLE_TOP+.320):return False
        return y<=-.135 if side=='left' else y>=.105
    if restricted(l,'left') or restricted(r,'right'):
        _latch('ARM_ENTERED_OPPOSITE_ROLE_ZONE');return False
    _STATE['last_poses']=poses
    return True

def _write_score_trace(poses,score):
    """Opt-in local verification data. Disabled when run by the reviewer."""
    path=os.environ.get('SAFE_TRANSFER_SCORE_TRACE')
    if not path:return
    row={'schema':'safe_dual_transfer.v1.arm_score_step','control_callback_index':_STATE['calls'],
         'safety_latched':_STATE['latched'],'violation':_STATE['reason'],
         'final_score':float(score), 'poses':poses}
    with open(path,'a',encoding='utf-8') as f:
        f.write(json.dumps(row,separators=(',',':'))+'\n')

def current_poses():
    """Expose the same validated Torch pose reader to the object-release gate."""
    return _physics_poses()


def latch_external(why):
    _latch(why)


def enforce(object_score,history,poses=None):
    """Called by YAM's official score(objects), once per control step.

    A verified object score is necessary but not sufficient for full credit.
    Missing state/errors are penalized, not replaced by stale USD transforms.
    """
    # Respect the exact pose snapshot already read by the official callback.
    # Re-reading would mix two simulation instants and weaken evidence alignment.
    if _STATE['latched']:
        _STATE['calls']+=1
        result=0.0
    else:
        try:
            poses=_physics_poses() if poses is None else poses
            if assess_poses(poses,bool(history.get('stage_qualified',False))):
                # The staged rung is earned only AFTER both arms vacate the
                # exclusive handoff volume. This matches the task.json rule.
                qualified=_STATE['vacancy_qualified']
                result=float(object_score) if qualified else 0.0
            else:
                result=0.0
        except Exception as exc:
            _STATE['calls']+=1
            _latch('ARM_PHYSICS_UNVERIFIED:'+type(exc).__name__+':'+str(exc)[:120]);result=0.0
    try:
        _write_score_trace(poses,result)
    except OSError:
        # Debug logging does not affect the official score; all missing
        # physics poses have already been rejected in the safety gate.
        pass
    return result

def state():
    return {k:v for k,v in _STATE.items() if k!='last_poses'}
