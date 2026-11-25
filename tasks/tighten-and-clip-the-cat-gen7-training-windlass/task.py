"""A blue C-A-T Gen 7 training tourniquet surrounds a chocked and strapped training
thigh. Its rod and attached two-sided clip sit above the band. A sealed bladder
feeds an aneroid gauge with a green sector. Both grippers start open.
"""
from __future__ import annotations
import hashlib
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
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import FINGER_OPEN, Waypoint, grip_rotation

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
SCORE_OVER = 'end'
SUCCESS_HOLD_S = .05  # the independent scorer already enforces the full second
INSTRUCTION = 'Tighten the C-A-T trainer until its gauge is green, lock its rod in its attached clip, then move both hands clear.'
HERE = Path(__file__).resolve().parent
BLUE, DARK, FOAM = (.08,.30,.76), (.12,.13,.15), (.68,.53,.37)
PIVOT = np.array((.230, 0., TOP+.1505))
# ---------------------------------------------------------------------------
# TRAINER READINGS. Product figures obtained by phone from a technician who
# works with this product (Nov 2026); forces measured on a trainer using a scale.
# The phone report is preserved in evidence/TECHNICIAN-READINGS-NOV23.md.
# These are reported readings, not a raw instrument record or a full curve.
LOAD_SOURCE = 'technician_phone'
WINDING_LEVER_M = .065
WINDING_FORCE_AT_LOCK_N = 2.2*9.80665  # 2.2 kg-force at the stated lever
WINDING_LOCK_DEG = 270.
CLIP_INSERT_PEAK_N = .8*9.80665       # separate push reading, 0.8 kg-force
CLIP_EXTRACT_PEAK_N = .8*9.80665      # separate pull reading, 0.8 kg-force
CLIP_MOUTH_M = .010
ROD_DIAMETER_M = .012
CLIP_HOOK_AZIMUTH_DEG = (156., 204.) # estimate: fixed original hook locations; never inferred from lock angle
ROD_TILT_STIFFNESS_NM_RAD = .10     # estimate: woven-band bending (not in the measurement request)
# Model margin: the simulated insertion and extraction peaks are fitted 10%
# above the input peaks. This is an analytic fitting target, not a guarantee
# about native contact forces; insertion AND extraction need native validation.
# The credited snap needs at least the insertion input itself.
SIM_FORCE_MARGIN = 1.10
# Return to the rounded profile that gave native entry8.182 N. The flat
# underside fits failed to match both directions. Both analytical targets use
# the observed entry ratio as a starting point; extraction needs a fresh probe.
NATIVE_OVER_MODEL = 0.754
CLIP_EXIT_FIT_OFFSET = 0.0
# Unmeasured contact-model fit controls, separate from the trainer readings.
# Min-combine isolates the fitted clip pair from the rod/hand grip material.
CLIP_STATIC_FRICTION = .05
CLIP_DYNAMIC_FRICTION = .04
CLIP_FRICTION_COMBINE_MODE = 'min'
# Fixed compliance and lip height are model fits, not product measurements.
# Height3.38 mm did not improve on3.2 mm (complete pulls10.371/9.941 N).
# Weaken the extraction-side lip while increasing the backing lip to preserve
# their series stiffness approximately. Native validation is required; these
# are fixed elastic properties, never direction-dependent forces.
CLIP_LIP_HEIGHT_M = .0030
LIP_STIFFNESS_SCALES = (1.59,3.485)
# Native point data: entry uses partial deflection; pull reaches drive limits.
# Fit these two passive regimes separately. This fixed limit applies in both
# directions for the entire trial; it is not a measured material force curve.
# Force limits are independent of initial slope. These two scales preserve
# both previous native drive limits while the backing lip's initial slope rises.
LIP_DRIVE_FORCE_SCALES = (.50562,1.10823)
if len(LIP_DRIVE_FORCE_SCALES)!=2 or not all(math.isfinite(v) and v>0 for v in LIP_DRIVE_FORCE_SCALES):
    raise ValueError('invalid fixed spring-drive force scales')
CLIP_ENTRY_ALIGNMENT_LEAD_DEG = 0.  # fixed demo position compensation, not a product reading
LIP_ENTRY_FLAT_HEIGHT_M = .0010  # fixed upper entry face; model fit, not measured geometry
if len(LIP_STIFFNESS_SCALES)!=2 or not all(math.isfinite(v) and v>0 for v in LIP_STIFFNESS_SCALES):
    raise ValueError('invalid fixed lip compliance scales')
# ---------------------------------------------------------------------------

def validate_readings(c):
    """Reject unsupported measured labels; explicit constants must match evidence.

    The shipped spring is linear. A measured nonlinear torque curve requires a
    new mechanical fit and fresh runs; this guard never silently flattens it.
    Metadata validation cannot establish that a physical experiment occurred.
    """
    if c.get('measured') is not True or c.get('product')!='NAR C-A-T Gen 7 blue trainer 30-0033':
        raise ValueError('measured loads require the specified trainer')
    for key in ('date_utc','specimen_id','instrument','method','raw_file','raw_sha256','uncertainty'):
        if not c.get(key): raise ValueError('missing measurement provenance: '+key)
    raw=(HERE/'evidence'/c['raw_file']).resolve()
    if not raw.is_relative_to((HERE/'evidence').resolve()) or not raw.is_file() or hashlib.sha256(raw.read_bytes()).hexdigest()!=c['raw_sha256']:
        raise ValueError('missing raw readings or raw hash mismatch')
    expected={k:globals()[k] for k in ('WINDING_LEVER_M','WINDING_FORCE_AT_LOCK_N','WINDING_LOCK_DEG',
              'CLIP_INSERT_PEAK_N','CLIP_EXTRACT_PEAK_N','CLIP_MOUTH_M','ROD_DIAMETER_M','CLIP_HOOK_AZIMUTH_DEG')}
    given=c.get('constants',{})
    for key,want in expected.items():
        if key not in given or not np.allclose(given[key],want,rtol=0,atol=1e-9):
            raise ValueError('reading constant disagrees with evidence: '+key)
    rows=c.get('winding_readings',[])
    angles=sorted(set(float(r['angle_deg']) for r in rows))
    if len(angles)<3 or WINDING_LOCK_DEG not in angles:
        raise ValueError('winding readings require lock and two earlier angles')
    for angle in angles:
        group=[r for r in rows if float(r['angle_deg'])==angle]
        if len(group)<3 or any(not all(math.isfinite(float(r[k])) and float(r[k])>0
            for k in ('lever_m','tightening_force_n','holding_force_n','pressure_mmhg')) for r in group):
            raise ValueError('three valid winding repeats required at every angle')
        upper=max(float(r['lever_m'])*float(r['tightening_force_n']) for r in group)
        model=WINDING_FORCE_AT_LOCK_N*WINDING_LEVER_M*angle/WINDING_LOCK_DEG
        if not math.isclose(upper,model,rel_tol=.10):
            raise ValueError('measured torque curve needs a new mechanical fit')
    snaps=c.get('snap_readings',[])
    for hook in ('hip','knee'):
        group=[r for r in snaps if r.get('hook')==hook]
        if len(group)<3 or any(not all(math.isfinite(float(r[k])) and float(r[k])>0
            for k in ('insert_n','extract_n','travel_m')) for r in group):
            raise ValueError('three insertion/extraction repeats required per hook')
    for field,target in (('insert_n',CLIP_INSERT_PEAK_N),('extract_n',CLIP_EXTRACT_PEAK_N)):
        if not math.isclose(max(float(r[field]) for r in snaps),target,rel_tol=1e-6):
            raise ValueError('clip peak constant disagrees with readings: '+field)
    pp=np.asarray(c.get('pressure_points',[]),float)
    if pp.ndim!=2 or pp.shape[1]!=2 or len(pp)<5 or not np.isfinite(pp).all() or not np.all(np.diff(pp[:,0])>0) or np.any(np.diff(pp[:,1])<0):
        raise ValueError('invalid measured pressure curve')
    if pp[0,0]!=0 or abs(pp[0,1]-60)>2 or pp[-1,0]<WINDING_LOCK_DEG:
        raise ValueError('pressure curve must cover snug start through lock')
    return pp

if LOAD_SOURCE not in ('estimate','measured','technician_phone'):
    raise ValueError('unsupported LOAD_SOURCE')
for _name in ('WINDING_LEVER_M','WINDING_FORCE_AT_LOCK_N','WINDING_LOCK_DEG',
              'CLIP_INSERT_PEAK_N','CLIP_EXTRACT_PEAK_N','CLIP_MOUTH_M','ROD_DIAMETER_M','ROD_TILT_STIFFNESS_NM_RAD'):
    if not math.isfinite(globals()[_name]) or globals()[_name]<=0: raise ValueError('invalid load constant: '+_name)
_PRESSURE_POINTS=None
if LOAD_SOURCE=='measured':
    _PRESSURE_POINTS=validate_readings(json.loads((HERE/'evidence'/'trainer-measurements.json').read_text()))

ROD_LENGTH, ROD_RADIUS = .150, ROD_DIAMETER_M/2
CLIP_MOUTH = CLIP_MOUTH_M
FIXTURE_YAW = 0.
START_YAW = math.radians(336.-WINDING_LOCK_DEG)+FIXTURE_YAW  # enroll snug pose for unchanged original hip hook
SEAT_TILT = math.radians(21.)
HOOK_RADIUS = .050
CLEARANCE = .010
SAMPLE_MARGIN = .006  # >3mm face raster +1.5mm voxel coverage and0.8mm rod ribs
LIP_BEAD_R = .00075   # rounded lip edge radius (m), geometry estimate
LIP_BEAD_Z = CLIP_LIP_HEIGHT_M     # lip edge height above the seated rod axis (m)
LIP_CAP_DEFLECTION_M = .0008
WINDING_TORQUE_AT_LOCK_NM = WINDING_FORCE_AT_LOCK_N*WINDING_LEVER_M
WINDING_STIFFNESS = WINDING_TORQUE_AT_LOCK_NM/math.radians(WINDING_LOCK_DEG)
WINDING_LIMIT_DEG = WINDING_LOCK_DEG+30.
SNAP_GATE_N = CLIP_INSERT_PEAK_N

def lip_profile_peak(psi_max=math.pi/2):
    """Quasi-static vertical rod force per unit lip stiffness, both lips (m).

    A lip edge of radius LIP_BEAD_R is pushed outward by the rod; at contact
    angle t above horizontal the lip opens d(t)=(R+r)cos t-(mouth/2+r) and
    the vertical reaction is k*d*tan t on each lip. A lead-in face limits
    the contact angles met on the way in to psi_max.
    """
    R=ROD_RADIUS;r=LIP_BEAD_R;y=CLIP_MOUTH/2+r;best=0.
    for i in range(1,2000):
        t=min(psi_max,math.pi/2)*i/2000
        best=max(best,2*min(LIP_CAP_DEFLECTION_M,max(0.,(R+r)*math.cos(t)-y))*math.tan(t))
    return best

def exit_profile_peak(psi_max=math.pi/2):
    """Capped geometric prior with a native response-fit offset."""
    R=ROD_RADIUS;r=LIP_BEAD_R;y=CLIP_MOUTH/2+r;best=0.
    for i in range(1,2000):
        t=min(psi_max,math.pi/2)*i/2000
        d=min(LIP_CAP_DEFLECTION_M,max(0.,(R+r)*math.cos(t)-y))
        best=max(best,2*d*(math.tan(t)+CLIP_EXIT_FIT_OFFSET))
    return best

def fit_clip():
    """Fit physical spring and permanent faces to both reported peak targets.

    The entry ratio and exit response offset are empirical simulation fits.
    The actual forces must be validated natively; neither is hardware data.
    """
    full=lip_profile_peak();out_full=exit_profile_peak()
    if full<=0 or out_full<=0:raise ValueError('rod does not spread the lips')
    target_in=SIM_FORCE_MARGIN*CLIP_INSERT_PEAK_N/NATIVE_OVER_MODEL
    target_out=SIM_FORCE_MARGIN*CLIP_EXTRACT_PEAK_N/NATIVE_OVER_MODEL
    k=max(target_in/full,target_out/out_full)
    def face(target,profile):
        if target>=k*profile()-1e-12:return None
        lo,hi=1e-4,math.pi/2
        for _ in range(60):
            mid=(lo+hi)/2
            if k*profile(mid)<target:lo=mid
            else:hi=mid
        return hi
    return k,face(target_in,lip_profile_peak),face(target_out,exit_profile_peak)

LIP_STIFFNESS, LEAD_IN_ANGLE, LEAD_OUT_ANGLE = fit_clip()

def lip_shapes(sign):
    """Rounded retaining edge, flat upper entry face and fitted tangent slabs.

    The slab's inner plane is tangent to the bead. A continuous collider avoids
    the overlapping contact patches of the earlier cylinder-chain face.
    """
    y=CLIP_MOUTH/2+LIP_BEAD_R;shapes=[parts.cylinder(LIP_BEAD_R,.010,(0,sign*y,LIP_BEAD_Z),DARK,axis='x')]
    # Fill the bead's upper half with a permanent flat entry face. The lower
    # rounded retaining edge stays unchanged. This collider and its spring
    # react physically in both directions; nothing changes during a trial.
    shapes.append(parts.box((.010,2*LIP_BEAD_R,LIP_ENTRY_FLAT_HEIGHT_M),
        (0,sign*y,LIP_BEAD_Z+LIP_ENTRY_FLAT_HEIGHT_M/2),DARK))
    for direction,angle in ((1,LEAD_IN_ANGLE),(-1,LEAD_OUT_ANGLE)):
        if angle is None:continue
        length=.0055/math.cos(angle)
        shapes.append(parts.box((.010,2*LIP_BEAD_R,length),
            (0,sign*(y+length*math.sin(angle)/2),LIP_BEAD_Z+direction*.0055/2),
            DARK,roll=-direction*sign*angle))
    return tuple(shapes)

def pressure(angle):
    """Measured curve if validated, otherwise an explicitly estimated gauge proxy."""
    if _PRESSURE_POINTS is not None:
        return float(np.interp(abs(math.degrees(angle)),_PRESSURE_POINTS[:,0],_PRESSURE_POINTS[:,1]))
    return min(300.,60.+210.*abs(math.degrees(angle))/WINDING_LOCK_DEG)

def winding_load(angle):
    return WINDING_STIFFNESS*abs(angle)

def fixture_point(x,y,z):
    c,si=math.cos(FIXTURE_YAW),math.sin(FIXTURE_YAW)
    return (float(.23+c*(x-.23)-si*y),float(si*(x-.23)+c*y),float(z))

# The joint base is fixed because the full-size training cylinder is strapped
# into real chocks. The elastic winding torque reacts through that base.
THIGH=parts.Part('strapped_training_thigh',pos=(.23,0.,TOP+.0655),yaw=FIXTURE_YAW,shapes=[
    parts.cylinder(.0635,.400,color=FOAM,axis='y'),
    parts.cylinder(.0695,.038,color=BLUE,axis='y'),
    parts.cylinder(.066,.050,color=DARK,axis='y')],friction=(.6,.5))
CHOCKS=[]
for y in (-.15,.15):
    for x in (-.0707,.0707):
        CHOCKS.append(parts.Part('chock_'+str(len(CHOCKS)),pos=fixture_point(.23+x,y,TOP+.0175),yaw=FIXTURE_YAW,
                       shapes=[parts.box((.030,.040,.035),color=(.55,.36,.20))],friction=(.6,.5)))
# Securing straps at either end are separate from the scored central band.
STRAPS=[parts.Part('end_strap_'+str(i),pos=fixture_point(.23,y,TOP+.0655),yaw=FIXTURE_YAW,shapes=[
    parts.cylinder(.0645,.020,color=(.26,.27,.29),axis='y')],friction=(.6,.5))
    for i,y in enumerate((-.15,.15))]
WINDLASS=parts.Part('cat_windlass',pos=tuple(PIVOT),yaw=START_YAW,shapes=[
    parts.box((.030,.022,.005),(0,0,-.0175),BLUE)],joints=[
    parts.Hinge('winding',shapes=(parts.box((.020,.015,.005),(0,0,-.007),BLUE),),
                pivot=(0,0,0),axis=(0,0,1),lower=math.radians(-WINDING_LIMIT_DEG),upper=math.radians(WINDING_LIMIT_DEG),
                mass=.010,spring=WINDING_STIFFNESS,damping=.20,max_effort=max(5.,1.3*WINDING_STIFFNESS*math.radians(WINDING_LIMIT_DEG))),
    parts.Hinge('rod_tilt',shapes=(parts.cylinder(ROD_RADIUS,ROD_LENGTH,color=DARK,axis='x'),parts.cylinder(.0068,.004,(-.065,0,0),DARK,axis='x'),parts.cylinder(.0068,.004,(.065,0,0),DARK,axis='x')),
                pivot=(0,0,0),axis=(0,1,0),lower=-math.radians(30),upper=math.radians(30),
                mass=.030,spring=ROD_TILT_STIFFNESS_NM_RAD,damping=.025,max_effort=2.,parent='winding')],friction=(.8,.6))

# Attached bilateral clip: two physical channels. Each has two spring lips;
# compression, retention and release all come from contact geometry. There is
# no state-driven latch, joint target change, weld, or robot/object pose write.
CLIPS=[]
# Original hook locations are independent geometry inputs. A new lock angle
# can require a different demo; it never relocates the attached factory clip.
for side,azimuth in zip(('hip','knee'),CLIP_HOOK_AZIMUTH_DEG):
    psi=math.radians(azimuth)+FIXTURE_YAW
    floorz=-ROD_RADIUS-.0015
    r=parts._rot((0,0,1),psi)
    center=PIVOT+r@np.array((HOOK_RADIUS*math.cos(SEAT_TILT),0,-HOOK_RADIUS*math.sin(SEAT_TILT)))
    lips=[]
    # Lip stiffness and lead-in come from fit_clip(): the readings above.
    for sign,label,scale,force_scale in zip((-1,1),('a','b'),LIP_STIFFNESS_SCALES,LIP_DRIVE_FORCE_SCALES):
        klip=LIP_STIFFNESS*scale
        lips.append(parts.Slider('lip_'+label,shapes=lip_shapes(sign),
                    axis=(0,sign,0),travel=.002,mass=.008,spring=klip,
                    damping=2*math.sqrt(klip*.008),max_effort=LIP_STIFFNESS*LIP_CAP_DEFLECTION_M*force_scale))
    CLIPS.append(parts.Part('attached_clip_'+side,pos=tuple(center),yaw=psi,pitch=SEAT_TILT,
        shapes=[parts.box((.012,.016,.003),(0,0,floorz),DARK),
                parts.box((.012,.002,.012),(0,-.0075,floorz+.006),DARK),
                parts.box((.012,.002,.012),(0,.0075,floorz+.006),DARK)],joints=lips,friction=(CLIP_STATIC_FRICTION,CLIP_DYNAMIC_FRICTION),friction_combine_mode=CLIP_FRICTION_COMBINE_MODE))
GAUGE_POS=np.array((.360,-.180,TOP+.105))
GAUGE=parts.Part('aneroid_gauge',pos=tuple(GAUGE_POS),mass=.20,shapes=[
    parts.cylinder(.026,.010,color=(.92,.92,.87)),
    parts.box((.012,.012,.100),(0,0,-.055),DARK),
    parts.box((.060,.060,.008),(0,0,-.101),DARK)],friction=(.6,.5))
_DIAL_OP=None

def draw_gauge(stage,root):
    from pxr import Gf,Sdf,UsdGeom,UsdShade
    texture=str(HERE/'assets'/'dial.png');material=UsdShade.Material.Define(stage,root+'/dial_material')
    shader=UsdShade.Shader.Define(stage,root+'/dial_material/surface');shader.CreateIdAttr('UsdPreviewSurface')
    tex=UsdShade.Shader.Define(stage,root+'/dial_material/texture');tex.CreateIdAttr('UsdUVTexture')
    tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(texture)
    uv=UsdShade.Shader.Define(stage,root+'/dial_material/uv');uv.CreateIdAttr('UsdPrimvarReader_float2')
    uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
    tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result')
    shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(),'rgb')
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    face=UsdGeom.Mesh.Define(stage,root+'/dial_face');n=64
    pts=[(0.,0.,.0056)]+[(.0255*math.cos(i*2*math.pi/n),.0255*math.sin(i*2*math.pi/n),.0056) for i in range(n)]
    face.CreatePointsAttr([Gf.Vec3f(*p) for p in pts]);face.CreateFaceVertexCountsAttr([3]*n)
    face.CreateFaceVertexIndicesAttr([k for i in range(n) for k in (0,1+i,1+(i+1)%n)])
    face.CreateSubdivisionSchemeAttr('none');face.CreateDoubleSidedAttr(True)
    UsdGeom.PrimvarsAPI(face).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.vertex).Set(
        [Gf.Vec2f(.5+p[0]/.051,.5+p[1]/.051) for p in pts])
    UsdShade.MaterialBindingAPI.Apply(face.GetPrim()).Bind(material)
    indicator=UsdGeom.Xform.Define(stage,root+'/indicator')
    pointer=UsdGeom.Mesh.Define(stage,root+'/indicator/pointer')
    pointer.CreatePointsAttr([Gf.Vec3f(-.0014,-.003,.007),Gf.Vec3f(.0014,-.003,.007),Gf.Vec3f(0,.022,.007)])
    pointer.CreateFaceVertexCountsAttr([3]);pointer.CreateFaceVertexIndicesAttr([0,1,2]);pointer.CreateSubdivisionSchemeAttr('none')
    pointer.CreateDisplayColorAttr([Gf.Vec3f(.7,.02,.02)])
    indicator.AddRotateZOp().Set(81.)  # 60 mmHg at reset
parts.VISUAL_HOOKS[GAUGE.name]=draw_gauge
PLATE=[]
for i,x in enumerate((-.005,-.015,-.025,-.035,-.045)):
    h=math.sqrt(.0695**2-x*x)
    PLATE.append(parts.Part('cat_stabilization_plate_'+str(i),pos=fixture_point(float(PIVOT[0]+x),0.,TOP+.0655+h+.0013),yaw=FIXTURE_YAW,
                 pitch=math.atan2(x,h),shapes=[parts.box((.010,.038,.0025),color=DARK)],friction=(.4,.3)))
OBJECTS=[THIGH,*CHOCKS,*STRAPS,*PLATE,WINDLASS,*CLIPS,GAUGE]
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={GAUGE.name:GAUGE.pos}),
                  lots.Lot('B',place={GAUGE.name:(.400,.220,TOP+.105)}))
DEMO_TOUCHES={THIGH.name,*(c.name for c in CLIPS)}  # fingertip contact while grasping/seating; arm links still checked

def rod_pose(s):
    p,q=s.links['rod_tilt']
    return np.asarray(p),parts._quat_to_matrix(q)

def geometry(s):
    p,r=rod_pose(s)
    if not np.isfinite(np.r_[p,r.ravel(),list(s.joints.values())]).all():
        raise ValueError('nonfinite rod state')
    angle=float(s.joints['winding']); tipped=math.asin(float(np.clip(-r[2,0],-1,1)))
    ends=[p+sign*HOOK_RADIUS*r[:,0] for sign in (-1,1)]
    seated=False
    for clip in CLIPS:
        c=np.array(clip.pos); cr=clip.rotation
        for end in ends:
            d=(end-c)@cr
            if abs(d[0])<=.005 and abs(d[1])<=.0009 and -.0015<=d[2]<=.0025:
                seated=True
    return dict(angle=angle,pressure=pressure(angle),seated=bool(seated),tilt=tipped)

def value(snapshot,certified=True):
    if not certified or not snapshot.get('known') or snapshot.get('invalid'):
        return 0.
    p=snapshot['pressure']; locked=snapshot['seated'] and snapshot.get('snap_verified',False)
    if locked and p<250.-1e-6: return .25
    if 250.-1e-6<=p<=300.+1e-6 and not snapshot.get('body_touch'):
        if locked and snapshot.get('clear') and snapshot.get('still') and snapshot.get('hold_s',0)>=1.-1e-9:
            return 1.
        if locked or (snapshot.get('held') and snapshot.get('hand_contact_n',0)>=1. and abs(snapshot['tilt'])<=math.radians(5)+1e-9):
            return .5
    return 0.

_STATE={'known':False,'invalid':False}
_CERTIFICATE=None
_LAST=None
_CLOCK=0.
_FACE_CACHE={}

def stamp(s):
    return tuple(np.r_[s.pos,s.quat_xyzw,s.lin_vel,s.ang_vel,list(s.joints.values()),
                       *[np.r_[p,q] for p,q in s.links.values()]])

def hand_points(sim):
    from yam_arena import analysis
    global _FACE_CACHE
    if not _FACE_CACHE:
        from scipy.spatial import ConvexHull
        for name,hulls in analysis.link_hulls().items():
            samples=[]
            for h in hulls:
                v=np.asarray(h.vertices);samples.append(v)
                for face in ConvexHull(v).simplices:
                    a,b,c=v[face];n=max(1,int(math.ceil(max(np.linalg.norm(b-a),np.linalg.norm(c-a),np.linalg.norm(c-b))/.003)))
                    samples.append(np.array([a+(b-a)*i/n+(c-a)*j/n for i in range(n+1) for j in range(n+1-i)]))
            cloud=np.concatenate(samples);_,ix=np.unique(np.floor(cloud/.0015).astype(int),axis=0,return_index=True)
            _FACE_CACHE[name]=cloud[ix]
    base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);clouds=[]
    for side in ('left','right'):
        robot=base.scene[side+'_robot'];poses=parts._np(robot.data.body_link_pose_w)[0]
        for name,cloud in _FACE_CACHE.items():
            if name not in robot.body_names:continue
            p=poses[robot.body_names.index(name)]
            clouds.append(cloud@parts._quat_to_matrix(p[3:]).T+p[:3]-origin)
    if len(clouds)<12 or any(not np.isfinite(c).all() for c in clouds):raise ValueError('missing robot surfaces')
    return clouds

_VIEWS=None
_CLIP_HAND_VIEW=None
_SNAP={}

def contact_forces(sim):
    global _VIEWS
    from isaaclab_physx.physics import PhysxManager
    from yam_arena.robot import asset_link_paths
    if _VIEWS is None:
        paths=parts.ROOT_PATHS; links=asset_link_paths();envpath=sim.env.unwrapped.scene.env_prim_paths[0]
        handpaths=[envpath+'/'+arm+links[name] for arm in ('LeftArm','RightArm') for name in links]
        lips=[paths[c.name]+'/'+j.name for c in CLIPS for j in c.joints]
        floors=[paths[c.name]+'/base' for c in CLIPS]
        sv=PhysxManager.get_physics_sim_view()
        _VIEWS=(sv.create_rigid_contact_view([paths[WINDLASS.name]+'/rod_tilt'],filter_patterns=[handpaths+lips+floors],max_contact_data_count=512),len(handpaths))
    view,nh=_VIEWS
    f=parts._np(view.get_contact_force_matrix(dt=sim.env.unwrapped.sim.get_physics_dt())).reshape(-1,3)
    if len(f)!=nh+6 or not np.isfinite(f).all():raise ValueError('unknown native contacts')
    return f[:nh],f[nh:nh+4].reshape(2,2,3),f[nh+4:]

def friction_forces(sim):
    """Isaac Sim 6.1 returns friction separately from its normal-force matrix.

    Diagnostic telemetry only; this does not replace the snap's normal-force gate.
    """
    view,nh=_VIEWS
    data=view.get_friction_data(dt=sim.env.unwrapped.sim.get_physics_dt())
    vectors=parts._np(data[0]).reshape(-1,3)
    counts=parts._np(data[2]).reshape(-1);starts=parts._np(data[3]).reshape(-1)
    if len(counts)!=nh+6 or len(starts)!=nh+6:
        raise ValueError('unknown native friction data')
    f=np.zeros((nh+6,3))
    for i,(start,count) in enumerate(zip(starts,counts)):
        j,n=int(start),int(count)
        if n<0:raise ValueError('negative native friction count')
        if not n:continue
        if j<0 or j+n>len(vectors) or not np.isfinite(vectors[j:j+n]).all():
            raise ValueError('truncated native friction data')
        f[i]=vectors[j:j+n].sum(axis=0)
    return f[:nh],f[nh:nh+4].reshape(2,2,3),f[nh+4:]

def clip_normal_contacts(sim):
    """Read-only diagnostic contact points; never a scoring or force input."""
    view,nh=_VIEWS
    data=view.get_contact_data(dt=sim.env.unwrapped.sim.get_physics_dt())
    forces=parts._np(data[0]).reshape(-1)
    points=parts._np(data[1]).reshape(-1,3)
    normals=parts._np(data[2]).reshape(-1,3)
    separations=parts._np(data[3]).reshape(-1)
    counts=parts._np(data[4]).reshape(-1);starts=parts._np(data[5]).reshape(-1)
    if len(counts)!=nh+6 or len(starts)!=nh+6:raise ValueError('unknown clip point pairs')
    pairs=[]
    for i in range(nh,nh+6):
        at,n=int(starts[i]),int(counts[i])
        if n<0 or (n and (at<0 or at+n>min(len(forces),len(points),len(normals),len(separations)))):
            raise ValueError('truncated clip contact points')
        rows=[]
        for j in range(at,at+n):
            if not all(np.isfinite(v).all() for v in (forces[j],points[j],normals[j],separations[j])):
                raise ValueError('nonfinite clip contact point')
            rows.append(dict(normal_force_n=float(forces[j]),point_world_m=points[j].tolist(),
                             normal_world=normals[j].tolist(),separation_m=float(separations[j])))
        pairs.append(rows)
    return pairs

def clip_hand_contacts(sim):
    """Read-only calibration audit: detect direct robot/clip contact separately.

    Sum individual normal magnitudes, so opposed contacts cannot cancel.
    These values do not grant any score or change a mechanical parameter.
    """
    global _CLIP_HAND_VIEW
    from isaaclab_physx.physics import PhysxManager
    from yam_arena.robot import asset_link_paths
    paths=parts.ROOT_PATHS;links=asset_link_paths();envpath=sim.env.unwrapped.scene.env_prim_paths[0]
    hands=[envpath+'/'+arm+links[name] for arm in ('LeftArm','RightArm') for name in links]
    sources=[paths[c.name]+'/'+name for c in CLIPS for name in ('lip_a','lip_b','base')]
    if _CLIP_HAND_VIEW is None:
        _CLIP_HAND_VIEW=PhysxManager.get_physics_sim_view().create_rigid_contact_view(
            sources,filter_patterns=[hands for _ in sources],max_contact_data_count=1024)
    data=_CLIP_HAND_VIEW.get_contact_data(dt=sim.env.unwrapped.sim.get_physics_dt())
    forces=parts._np(data[0]).reshape(-1)
    counts=parts._np(data[4]).reshape(len(sources),len(hands))
    starts=parts._np(data[5]).reshape(len(sources),len(hands))
    f=np.zeros((len(sources),len(hands)))
    for i in range(len(sources)):
        for j in range(len(hands)):
            at,n=int(starts[i,j]),int(counts[i,j])
            if n<0:raise ValueError('negative clip/hand contact count')
            if not n:continue
            if at<0 or at+n>len(forces) or not np.isfinite(forces[at:at+n]).all():
                raise ValueError('truncated clip/hand contacts')
            f[i,j]=np.abs(forces[at:at+n]).sum()
    nh=len(hands)//2
    by_link=np.column_stack((f[:,:nh].sum(axis=1),f[:,nh:].sum(axis=1)))
    return by_link.reshape(2,3,2).sum(axis=1).tolist(),by_link.tolist()

def drive_readback(sim):
    """Read actual native joint settings and masses; never sets state/targets."""
    out={}
    for spec in [WINDLASS,*CLIPS]:
        asset=sim.env.unwrapped.scene[spec.name];view=asset.root_physx_view
        out[spec.name]=dict(joint_names=list(asset.joint_names),body_names=list(asset.body_names),
            stiffness=parts._np(view.get_dof_stiffnesses())[0].tolist(),
            damping=parts._np(view.get_dof_dampings())[0].tolist(),
            max_drive_force=parts._np(view.get_dof_max_forces())[0].tolist(),
            body_masses=parts._np(view.get_masses())[0].tolist())
    return out

def body_gap(clouds,states):
    v=np.concatenate(clouds);best=math.inf
    for radius,length in ((.0635,.400),(.0695,.038),(.066,.050)):
        d=(v-np.array(THIGH.pos))@THIGH.rotation;q=np.column_stack((np.hypot(d[:,0],d[:,2])-radius,np.abs(d[:,1])-length/2))
        signed=np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(np.maximum(q[:,0],q[:,1]),0)
        best=min(best,float(signed.min()))
    gs=states[GAUGE.name]
    for _,c,r,size in GAUGE.world_boxes({'base':(np.asarray(gs.pos),parts._quat_to_matrix(gs.quat_xyzw))}):
        best=min(best,float(np.linalg.norm(np.maximum(np.abs((v-c)@r)-size/2,0),axis=1).min()))
    return best

def clearance(clouds,s,states=None):
    p,r=rod_pose(s);v=np.concatenate(clouds);d=(v-p)@r
    gap=float(np.linalg.norm(np.maximum(np.abs(d)-(ROD_LENGTH/2,ROD_RADIUS,ROD_RADIUS),0),axis=1).min())
    for clip in CLIPS:
        cs=clip.state() if states is None else states[clip.name]
        if any(j.name not in cs.links for j in clip.joints):
            raise ValueError('missing live clip lip poses')
        poses={'base':(np.asarray(cs.pos),parts._quat_to_matrix(cs.quat_xyzw))}
        poses.update({name:(np.asarray(p),parts._quat_to_matrix(q)) for name,(p,q) in cs.links.items()})
        for _,c,r,size in clip.world_boxes(poses):
            gap=min(gap,float(np.linalg.norm(np.maximum(np.abs((v-c)@r)-size/2,0),axis=1).min()))
    return gap

def clip_retained(entered,joints):
    """Current lips must overlap the rod after a witnessed physical entry."""
    displacements=[float(joints[name]) for name in ('lip_a','lip_b')]
    if not all(math.isfinite(v) for v in displacements):raise ValueError('unknown clip lip pose')
    mouth=CLIP_MOUTH+sum(max(0.,v) for v in displacements)
    return bool(entered and mouth<=2*ROD_RADIUS-.0002),float(mouth)

def new_snap():
    return dict(armed=False,passed=False,verified=False,peak=0.,work=0.,down=0.,
                last_z=None,last_line=False,last_end=None,last_fz=0.,last_contact=False,
                seen_lips=0,low_z=None)

def snap_update(q,end_index,d,forces):
    """Measure one insertion, including sequential contact with beveled lips.

    The rod axis first approaches above the mouth. New depth contributes work
    only with consecutive real rod-lip contact; force-free gaps add nothing.
    Each lip must contact during that same entry. An upward reversal or lost
    alignment discards the record. Verification creates no constraint.
    """
    d=np.asarray(d,float);forces=np.asarray(forces,float)
    if d.shape!=(3,) or forces.shape!=(2,3) or not np.isfinite(np.r_[d,forces.ravel()]).all():
        raise ValueError('invalid snap observation')
    z=float(d[2]);line=bool(abs(d[0])<.006 and abs(d[1])<.0015)
    norms=np.linalg.norm(forces,axis=1);bits=sum(1<<i for i,n in enumerate(norms) if n>.1)
    fz=float(forces[:,2].sum());contact=bool(bits and fz>0)
    same=q['last_end']==int(end_index) and q['last_line']
    previous_z=q['last_z'];previous_fz=q['last_fz'];previous_contact=q['last_contact']
    if not line or not same:
        q.clear();q.update(new_snap())
    elif q['verified']:
        if z<-.002 or z>.003:q.clear();q.update(new_snap())
    elif q['armed']:
        if q['low_z'] is not None and z>float(q['low_z'])+.0002:
            q.clear();q.update(new_snap())
        elif not q['passed'] and z<=.0025 and not contact:
            # An under-force entry has already passed into the seat. Later
            # bottoming/pressing cannot replace the required insertion witness.
            q.clear();q.update(new_snap())
        else:
            # Record actual new contact distance, never free descent or repeated
            # motion over an already traversed depth. Asymmetric loading is real
            # clip entry; each bead still needs its own rod-contact witness.
            if q['low_z'] is None:q['low_z']=z
            advance=max(0.,float(q['low_z'])-z)
            if contact:
                q['peak']=max(q['peak'],fz)
                if previous_contact and previous_fz>0:
                    q['work']+=min(previous_fz,fz)*advance;q['down']+=advance
            q['low_z']=min(float(q['low_z']),z);q['seen_lips'] |= int(bits)
            if q['seen_lips']==3 and q['peak']>=SNAP_GATE_N and q['work']>=.001 and q['down']>=.001:
                q['passed']=True
            # The lip force normally reverses after passing the bead. Completed
            # physical work remains evidence while the same entry reaches seat.
            if -.0015<=z<.0015 and q['passed']:q['verified']=True
    if line and z>.008:q['armed']=True
    q['last_z']=z;q['last_line']=line;q['last_end']=int(end_index)
    q['last_fz']=fz;q['last_contact']=contact
    return bool(q['verified'] and line and abs(d[0])<=.005 and abs(d[1])<=.0009 and -.0015<=z<=.0025)

@parts.on_reset
def reset(sim=None):
    global _CERTIFICATE,_LAST,_CLOCK,_VIEWS,_CLIP_HAND_VIEW,_SNAP
    _STATE.clear();_STATE.update(known=False,invalid=False,snap_verified=False,hold_s=0.)
    _CERTIFICATE=None;_LAST=None;_CLOCK=0.;_VIEWS=None;_CLIP_HAND_VIEW=None
    _SNAP={c.name:new_snap() for c in CLIPS}
    if sim is not None:
        from isaaclab.app.settings_manager import get_settings_manager
        get_settings_manager().set_bool('/physics/disableContactProcessing',False)

@parts.on_step
def observe(sim):
    global _CERTIFICATE,_LAST,_CLOCK
    try:
        states=sim.states();s=states[WINDLASS.name];g=geometry(s);now=float(sim.t);dt=now-_CLOCK
        if not 0<dt<=.100001: raise ValueError('invalid observation interval')
        if g['seated'] or g['pressure']>=250.:
            clouds=hand_points(sim);gap=clearance(clouds,s,states);body_distance=body_gap(clouds,states);body_touch=body_distance<SAMPLE_MARGIN
        else:
            gap=0.;body_touch=False;body_distance=None
        hands,lip_forces,floors=contact_forces(sim)
        p,r=rod_pose(s);ends=[p+sign*HOOK_RADIUS*r[:,0] for sign in (-1,1)]
        verified=False;peaks=[];mouths=[]
        for i,clip in enumerate(CLIPS):
            q=_SNAP[clip.name];cr=clip.rotation;c=np.array(clip.pos)
            ends_local=[(e-c)@cr for e in ends];end_index=min(range(2),key=lambda i:np.linalg.norm(ends_local[i][:2]));d=ends_local[end_index]
            entered=snap_update(q,end_index,d,lip_forces[i]@cr)
            retained,mouth=clip_retained(entered,states[clip.name].joints);verified |= retained;mouths.append(mouth)
            peaks.append(q['peak'])
        speed=math.inf if _LAST is None else abs(g['angle']-_LAST['angle'])/dt
        ts=math.inf if _LAST is None else abs(g['tilt']-_LAST['tilt'])/dt
        still=speed<=math.radians(1) and ts<=math.radians(1)
        hand_n=float(np.linalg.norm(hands,axis=1).sum())
        eligible=g['seated'] and verified and 250<=g['pressure']<=300 and gap>=CLEARANCE+SAMPLE_MARGIN and still and not body_touch
        hold=_STATE.get('hold_s',0)+dt if eligible else 0.
        _STATE.update(g,known=True,clear=gap>=CLEARANCE+SAMPLE_MARGIN,held=hand_n>=1.,hand_contact_n=hand_n,
                      body_touch=body_touch,body_gap_sample_m=body_distance,sampled_clearance_m=gap,still=still,snap_verified=verified,hold_s=hold,clip_force_peaks=peaks,
                      lip_forces=lip_forces.tolist(),clip_base_forces=floors.tolist(),clip_mouths_mm=[float(v*1000) for v in mouths],hand_force_peak=float(np.linalg.norm(hands,axis=1).max()))
        _CLOCK=now;_LAST=g;_CERTIFICATE=stamp(s)
        _STATE['hand_left_n']=float(np.linalg.norm(hands[:len(hands)//2],axis=1).sum())
        _STATE['hand_right_n']=float(np.linalg.norm(hands[len(hands)//2:],axis=1).sum())
        if os.environ.get('TRACE_OUT'):
            _STATE['clip_joint_positions']=[dict(states[c.name].joints) for c in CLIPS]
            _STATE['clip_joint_velocities']=[parts._np(sim.env.unwrapped.scene[c.name].data.joint_vel)[0].tolist() for c in CLIPS]
            if 'native_drive_properties' not in _STATE:_STATE['native_drive_properties']=drive_readback(sim)
            clip_hand,clip_hand_links=clip_hand_contacts(sim)
            _STATE.update(clip_hand_normal_n=clip_hand,clip_hand_link_normal_n=clip_hand_links)
            hand_friction,lip_friction,base_friction=friction_forces(sim)
            _STATE.update(lip_frictions=lip_friction.tolist(),clip_base_frictions=base_friction.tolist(),hand_frictions=hand_friction.tolist(),clip_normal_contacts=clip_normal_contacts(sim))
        _STATE['grippers']={}
        from yam_arena.constants import TCP_OFFSET
        base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0])
        for side in ('left','right'):
            robot=base.scene[side+'_robot'];robot_poses=parts._np(robot.data.body_link_pose_w)[0];bp=robot_poses[robot.body_names.index('gripper')]
            jq=parts._np(robot.data.joint_pos)[0]
            _STATE['grippers'][side]=dict(quat=bp[3:].tolist(),tcp=(bp[:3]-origin+parts._quat_to_matrix(bp[3:])@np.array(TCP_OFFSET)).tolist(),finger_q=[float(jq[robot.joint_names.index(n)]) for n in ('joint7','joint8')])
            if os.environ.get('TRACE_OUT'):
                _STATE['grippers'][side].update(joint_names=list(robot.joint_names),joint_positions=jq.tolist(),joint_limits=parts._np(robot.data.joint_pos_limits)[0].tolist(),
                    bodies={n:(robot_poses[i,:3]-origin).tolist() for i,n in enumerate(robot.body_names)})
        from pxr import UsdGeom
        from isaaclab.sim.utils import get_current_stage
        gauge_root=parts.ROOT_PATHS[GAUGE.name]
        prim=get_current_stage().GetPrimAtPath(gauge_root+'/indicator')
        ops=UsdGeom.Xformable(prim).GetOrderedXformOps()
        ops[0].Set(135-270*min(g['pressure'],300)/300)
    except (KeyError,ValueError,AttributeError,IndexError,TypeError) as e:
        _CLOCK=float(sim.t)
        _STATE.update(known=False,invalid=True,hold_s=0.,error=type(e).__name__+': '+str(e))
    if os.environ.get('TRACE_OUT'):
        row=dict(t=float(sim.t),load_source=LOAD_SOURCE,state=_STATE,contacts=_SNAP)
        with Path(os.environ['TRACE_OUT']).with_name('windlass.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')

def score(objects):
    try:
        s=objects[WINDLASS.name]
        if stamp(s)!=_CERTIFICATE:return 0.
        # Native contact observations and physical retention gate every credited state.
        return value(_STATE,True)
    except (KeyError,TypeError,ValueError,AttributeError):return 0.

def success(objects):return score(objects)==1.


def demo():
    """Open-loop two-handed regrip and seating from the visible fixed layout."""
    from yam_arena import ik,station
    from yam_arena.constants import READY
    plans={'left':[],'right':[]};theta0=START_YAW
    # Compliance compensation was tuned at 1.94 N m; it scales with the lock
    # torque. Regrip angles scale with the lock angle.
    load=WINDING_TORQUE_AT_LOCK_NM/1.94;L=WINDING_LOCK_DEG
    # Grip squeeze: the jaw target sits 3.9 mm inside the 12 mm rod at the
    # estimated load (about 20 N per jaw). From 2% above it the target closes fully
    # (about 42 N, under the 50 N jaw cap) so the rod is not pried out.
    GRIP=-.0045+.0045*min(1.,max(0.,(load-1.02)/.3))
    gain=1.+.20*load*210./L;handoff=L-80.;retake=L-45.;lead=0.;seating_lead=CLIP_ENTRY_ALIGNMENT_LEAD_DEG  # retain right-hand load compensation; centre the seating end
    def point(theta,sign,dip=0.,radius=.065,above=0.,pad_height=0.):
        rot=parts._rot((0,0,1),theta)@parts._rot((0,1,0),dip)
        return PIVOT+rot@np.array((sign*radius,0.,pad_height))+np.array((0.,0.,above))
    def add(arm,t,p,f,label,yaw,actual,lean=None,dip=0.):
        if lean is None:lean=-45. if arm=='left' else 0.
        axis=(-math.sin(actual),math.cos(actual),0.)
        rot=parts._rot(axis,dip)@parts._rot((-math.sin(yaw),math.cos(yaw),0),math.radians(lean))@grip_rotation(yaw)
        plans[arm].append(Waypoint(t,tuple(float(v) for v in p),f,label,tilt._TiltedYaw(0,rot)))
        return plans[arm][-1]
    # Loaded strokes lead the rod by a compliance allowance. Near the edge of
    # the arm's reach, the allowance is trimmed in 1-degree steps to the
    # largest reachable lead; the rod angle the grip follows is unchanged.
    reposition={}
    def right(t,a,f,label,above=0.,dip=0.,branch=1,loaded=True,angle_lead=0.,z_lead=0.):
        cmd=theta0+math.radians(a*(gain if loaded else 1.)+angle_lead)
        w=add('right',t,point(cmd,1,dip,above=above+z_lead),f,label,theta0+math.radians(a)-branch*math.pi,theta0+math.radians(a),dip=dip,lean=lean)
        reposition[id(w)]=(cmd,theta0+math.radians(a),lambda c:point(c,1,dip,above=above+z_lead))
    def left(t,a,f,label,above=0.,dip=0.,radius=.065,loaded=False,angle_lead=0.,z_lead=0.,branch=0,lean=None):
        cmd=theta0+math.radians(a*(gain if loaded else 1.)+angle_lead)
        w=add('left',t,point(cmd,-1,dip,radius,above+z_lead),f,label,theta0+math.radians(a)-branch*math.pi,theta0+math.radians(a),dip=dip,lean=lean)
        reposition[id(w)]=(cmd,theta0+math.radians(a),lambda c:point(c,-1,dip,radius,above+z_lead))
    # The reported 270-degree stroke starts 60 degrees earlier than the old
    # 210-degree layout. The left hand winds this first section on the opposite
    # wrist branch, then hands the loaded rod to the right near the cylinder centreline.
    pre=L-95.
    ready_p,ready_r=ik.tcp_pose(np.asarray(READY),station.BASES['right'])
    add('right',2.,(.13,-.14,TOP+.18),FINGER_OPEN,'stage beside handoff',0.,0.,lean=0.)
    add('right',11.,(.13,-.14,TOP+.18),FINGER_OPEN,'wait beside handoff',0.,0.,lean=0.)
    left(2.,0,FINGER_OPEN,'approach initial winding end',above=.015,branch=1)
    left(1.,0,FINGER_OPEN,'lower initial winding hand',branch=1)
    left(2.,0,GRIP,'grasp initial winding end',branch=1)
    for a in np.linspace(0,pre,33)[1:]:
        lean=-45.*min(1.,max(0.,(pre-float(a))/30.))
        left(.25,float(a),GRIP,'wind initial section',branch=1,lean=lean)
    left(7.,pre,GRIP,'wait for turning grip',branch=1,lean=0.)
    # Approach above the observed held angle, then descend in short Cartesian
    # segments; a long joint interpolation swept an open finger into the rod.
    right(2.,pre-5.,FINGER_OPEN,'approach rod from above',above=.040,loaded=False)
    for z in np.linspace(.040,-.005,13)[1:]:
        right(.25,pre-5.,FINGER_OPEN,'lower to held rod',above=float(z),loaded=False)
    right(2.,pre-5.,GRIP,'grasp loaded rod',loaded=False,z_lead=-.005)
    right(5.,pre,GRIP,'wait initial winding hand clear')
    left(1.,pre,FINGER_OPEN,'release initial winding hand',branch=1,lean=0.)
    left(2.,pre,FINGER_OPEN,'lift initial winding hand',above=.015,branch=1,lean=0.)
    add('left',2.,(.13,.24,TOP+.18),FINGER_OPEN,'withdraw initial winding hand',0.,0.,lean=0.)
    for a in np.linspace(pre,handoff,27)[1:]:right(.25,float(a),GRIP,'wind first stroke')
    first_end=25.+26*.25
    ready_p,ready_r=ik.tcp_pose(np.asarray(READY),station.BASES['left'])
    plans['left'].append(Waypoint(first_end-25.,tuple(ready_p),FINGER_OPEN,'wait at ready',tilt._TiltedYaw(0,ready_r)))
    right(6.,handoff,GRIP,'wait counterhold')
    left(2.,handoff,FINGER_OPEN,'approach counterhold',above=.025)
    left(1.,handoff,FINGER_OPEN,'lower counterhold')
    left(2.,handoff,GRIP,'counterhold')
    left(1.,handoff,GRIP,'take winding load',loaded=True)
    right(1.,handoff,FINGER_OPEN,'release first grip')
    right(1.,handoff,FINGER_OPEN,'clear before wrist reset',above=.015)
    add('right',1.,(.13,-.22,TOP+.18),FINGER_OPEN,'reset wrist away',0.,0.,lean=0.)
    add('right',4.5,(.13,-.22,TOP+.18),FINGER_OPEN,'wait for counterhold turn',0.,0.,lean=0.)
    right(1.5,retake,FINGER_OPEN,'return after wrist reset',above=.025,branch=2,loaded=False)
    right(1.,retake,FINGER_OPEN,'retake rod',branch=2,loaded=False)
    right(2.,retake,GRIP,'close second grip',branch=2,loaded=False)
    right(1.,retake,GRIP,'take load',branch=2)
    right(3.,retake,GRIP,'wait counterhold release',branch=2)
    left(3.,handoff,GRIP,'maintain winding load',loaded=True)
    for a in np.linspace(handoff,retake,15)[1:]:left(.25,float(a),GRIP,'counterhold turns beyond wrist limit',loaded=True)
    left(6.5,retake,GRIP,'maintain winding load',loaded=True)
    left(1.,retake,FINGER_OPEN,'release counterhold',loaded=True)
    left(2.,retake,FINGER_OPEN,'withdraw counterhold',above=.06,loaded=True)
    for a in np.linspace(retake,L,16)[1:]:right(.25,float(a),GRIP,'wind to green',branch=2)
    right(5.,L,GRIP,'hold for seating grasp',branch=2)
    left(3.75,L,FINGER_OPEN,'approach attached clip',above=.025,radius=.073)
    left(2.,L,FINGER_OPEN,'lower seating grasp',radius=.073)
    left(3.,L,GRIP,'grasp seating end',radius=.073)
    # Native-simulation clip-phase compliance compensation; jaw angles retain intended
    # rod alignment. The level pause presents the rod above the real lip mouth.
    right(2.,L,GRIP,'align above clip',branch=2,angle_lead=lead)
    left(2.,L,GRIP,'align above clip',radius=.073,angle_lead=seating_lead)
    right(1.,L,GRIP,'pause above mouth',branch=2,angle_lead=lead)
    left(1.,L,GRIP,'pause above mouth',radius=.073,angle_lead=seating_lead)
    for u in np.linspace(0,1,8)[1:]:
        dip=-SEAT_TILT*float(u)
        right(5./7,L,GRIP,'allow rod tilt',dip=dip,branch=2,angle_lead=lead,z_lead=.015*u)
        left(5./7,L,GRIP,'press rod through lips',dip=dip,radius=.073,angle_lead=seating_lead,z_lead=-.015*u)
    right(6.,L,GRIP,'hold seated rod',dip=-SEAT_TILT,branch=2,angle_lead=lead,z_lead=.015)
    left(2.5,L,GRIP,'seat under load',dip=-SEAT_TILT,radius=.073,angle_lead=seating_lead,z_lead=-.015)
    left(1.,L,FINGER_OPEN,'release seating hand',dip=-SEAT_TILT,radius=.073,angle_lead=seating_lead,z_lead=-.015)
    left(1.5,L,FINGER_OPEN,'lift from attached clip',above=.10,dip=-SEAT_TILT,radius=.073,angle_lead=seating_lead,z_lead=-.015)
    add('left',2.,(.13,.24,TOP+.14),FINGER_OPEN,'withdraw seating hand',0.,0.,lean=0.)
    add('left',4.,(.13,.24,TOP+.14),FINGER_OPEN,'hands clear',0.,0.,lean=0.)
    right(1.,L,FINGER_OPEN,'release retained rod',dip=-SEAT_TILT,branch=2,angle_lead=lead,z_lead=.015)
    add('right',2.,(.13,-.22,TOP+.14),FINGER_OPEN,'withdraw turning hand',0.,0.,lean=0.)
    add('right',2.,(.13,-.22,TOP+.14),FINGER_OPEN,'released stillness',0.,0.,lean=0.)
    for side,waypoints in plans.items():
        q=None
        for i,w in enumerate(waypoints):
            q0=q
            q,pe,re=ik.solve(np.array(w.pos),ik.matrix_to_quat_xyzw(grip_rotation(w.yaw)),q_init=q0,base_pos=station.BASES[side],iters=160)
            if (pe>.001 or re>math.radians(.5)) and id(w) in reposition:
                cmd,actual,at=reposition[id(w)];step=math.copysign(math.radians(1.),cmd-actual)
                while (pe>.001 or re>math.radians(.5)) and abs(cmd-actual)>=math.radians(1.):
                    cmd-=step;w.pos=tuple(float(v) for v in at(cmd))
                    q,pe,re=ik.solve(np.array(w.pos),ik.matrix_to_quat_xyzw(grip_rotation(w.yaw)),q_init=q0,base_pos=station.BASES[side],iters=160)
                w.label+=' (lead trimmed to %.0f deg)'%math.degrees(cmd-actual)
            if pe>.001 or re>math.radians(.5):raise ValueError('unreachable fixed demo endpoint '+side+' '+w.label+' '+str((pe*1000,math.degrees(re),w.pos)))
            w.joint_target=tuple(float(v) for v in q);w.endpoint_ik_mm=float(pe*1000);w.endpoint_ik_deg=float(math.degrees(re))
    return plans
