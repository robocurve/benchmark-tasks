"""Dry-fit a replacement drain span using closed repair sleeves.

Park sleeves while the cut ends are accessible, seat the span on its support,
and choose the matching bore. All retention is passive rigid-body contact.
"""
import math
import pathlib
import sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, box, cyl, ball, PartState
from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.task_file import TABLE_TOP

SCENE = "station"

# Release-and-stillness block, verbatim from hang_holder_on_keyhole_studs.py (see release-stillness.md).
# The task module must define: import math; import numpy as np; from yam_arena.constants import FINGER_OPEN;
# and SCENE ("station" or "tabletop"). Paste after the imports.

# Release and stillness
# Released means every gripper TCP is at least RELEASE_CLEAR from the object's box.
# Still means below the task's linear and angular speed limits for STILL_S seconds.
# Evaluate every sticky condition and timer at the top of score() on every call,
# before any early return, so interrupted conditions cannot retain elapsed time.
RELEASE_CLEAR = 0.030
STILL_S = 1.0
TEST_GRIPPERS = None


def _run_context():
    context = {"env": None, "t": None, "step": 0, "episode": 0}
    try:
        from yam_arena import inspect_embodiment as _ie
    except ImportError:
        return context
    cls = _ie.YamArenaEmbodiment
    if hasattr(cls, "_rc_run"):
        return cls._rc_run
    original_step, original_reset = cls._task_step, cls.reset

    def task_step(self, *args, **kwargs):
        context["env"], context["t"] = self._env, self._t
        context["step"] += 1
        return original_step(self, *args, **kwargs)

    def reset(self, *args, **kwargs):
        result = original_reset(self, *args, **kwargs)
        context["episode"] += 1
        context["t"] = 0.0
        context["env"] = self._env
        return result

    task_step._rc_wrapped = reset._rc_wrapped = True
    cls._rc_run = context
    cls._task_step, cls.reset = task_step, reset
    return context


_RUN = _run_context()
_MEMORY_EPISODE = None
_TIMERS = {}
_EVER = {}


def grippers():
    if TEST_GRIPPERS is not None:
        return TEST_GRIPPERS
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, LEFT_BASE_POS, RIGHT_BASE_POS

    bases = station.BASES if SCENE == "station" else {"left": LEFT_BASE_POS, "right": RIGHT_BASE_POS}
    out = {}
    for arm in ("left", "right"):
        art = env.unwrapped.scene[arm + "_robot"]
        arm_ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        finger_ids = art.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        joints = art.data.joint_pos
        q = getattr(joints, "torch", joints)[0]
        if hasattr(q, "detach"):
            q = q.detach().cpu().numpy()
        q = np.asarray(q)
        tcp = ik.tcp_pose(q[arm_ids], bases[arm])[0]
        opening = float(np.clip(np.mean(q[finger_ids]) / FINGER_OPEN, 0.0, 1.0))
        out[arm] = (tcp, opening)
    return out


def released(state, clear=RELEASE_CLEAR):
    hands = grippers()
    if hands is None:
        return True  # Offline arm poses are unknown, so release is assumed.
    low, high = np.asarray(state.low), np.asarray(state.high)
    return all(np.linalg.norm(np.maximum(np.maximum(low - tcp, tcp - high), 0.0)) >= clear
               for tcp, _ in hands.values())


def _episode_memory():
    global _MEMORY_EPISODE
    if _MEMORY_EPISODE != _RUN["episode"]:
        _TIMERS.clear()
        _EVER.clear()
        _MEMORY_EPISODE = _RUN["episode"]


def held_for(key, cond):
    """Elapsed time over consecutive observed control steps; repeat calls are harmless."""
    _episode_memory()
    t, step = _RUN["t"], _RUN["step"]
    if t is None:
        return math.inf if cond else 0.0
    previous_step, start = _TIMERS.get(key, (None, None))
    if not cond:
        _TIMERS[key] = (step, None)
        return 0.0
    if start is None or step not in (previous_step, previous_step + 1) or t < start:
        start = t
    _TIMERS[key] = (step, start)
    return float(t - start)


def ever(key, cond):
    """Remember a satisfied condition until the next episode; offline each call stands alone."""
    if _RUN["t"] is None: return bool(cond)
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]


TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S, SCORE_OVER = 1.0, "end"
PIPE_X, AXIS_Z, PIPE_R = .200, TOP+.070, .016
STUB_END, STUB_OUTER, ELBOW_Y, ELBOW_R = .172, .300, .307, .019
SECTION_L, SECTION_MASS = .330, .063
CHAMFER_L, CHAMFER_R = .002, .014
COUP_L, BORE_R, OUTER_R, COUP_MASS = .080, .017, .026, .030
SMALL_L, SMALL_BORE, SMALL_OUTER, SMALL_MASS = .060, .0115, .020, .015
LARGE_L, LARGE_BORE, LARGE_OUTER, LARGE_MASS = .080, .02175, .0325, .040
CONTACT_OFFSET, START_GAP = .001, .0005
FRICTION = (.6, .5)
PVC_GREY, ORANGE, BLACK = (.32,.34,.36), (.95,.28,.025), (.035,.038,.042)
STEEL, TRAY_GREY = (.40,.44,.48), (.25,.28,.31)
CLAMP_Y, CLAMP_W, CLAMP_X, STRAP_T = .279, .008, .044, .004
POST_W, POST_D = .012, .008
START_X, SECTION_START_Y, SECTION_START_Z = .280, -.020, TOP+.031
COUP_X = .120
COUP_START_Y, SMALL_START_Y, LARGE_START_Y = .260, .380, -.367
COUP_START_Z = TOP+OUTER_R+.0085
SMALL_START_Z, LARGE_START_Z = TOP+SMALL_OUTER+.0085, TOP+LARGE_OUTER+.0085
TRAY_X, TRAY_HALF_X, TRAY_RIM_T, TRAY_FLOOR_T, TRAY_RIM_H = .120, .045, .004, .004, .008
TRAY_Y, TRAY_HALF_Y = .320, .115
CRADLE_YS, CRADLE_W, CRADLE_D = (-.060,.060), .005, .006
CRADLE_X = .007
CRADLE_L = max(CRADLE_YS)-min(CRADLE_YS)+CRADLE_D
COUP_SUPPORT_Y, COUP_SUPPORT_X, COUP_SUPPORT_W, COUP_SUPPORT_D = .030, .004, .004, .006
IN_LINE_DEG, IN_LINE_TOL, PEN_TOL = 4., .005, .002
COAX_DEG, COAX_TOL, MIN_OVERLAP = 5., .003, .025
OFF_BENCH, POSE_WINDOW_M, POSE_WINDOW_DEG, REST_SPEED, REST_SPIN = .010, .001, 1., .010, .5
SLIP_NAMES = ("coupling_a", "coupling_b")
SCORED_NAMES = ("section", *SLIP_NAMES)
COUPLING_DZ, CARRY_DZ = .001, .004
# Measured settle against the open +x finger: 46.95 - 26 mm, rounded by contact.
SLEEVE_GRIP_SHIFT, SECTION_GRIP_SHIFT = .0211, .002
SLEEVE_RELEASE_SHIFT = .014  # Re-centre 7.1 mm as the fingers open, then rise.
ENTRY_GAP, PARK_GAP = .005, .005
ENTRY_Y = STUB_END-ENTRY_GAP-COUP_L/2
PARK_Y = STUB_END+PARK_GAP+COUP_L/2
JOINT_Y = (STUB_END+SECTION_L/2)/2
RIGHT_GRASP = -.105
TRAVEL_Z, CLEAR_Z, PARK_Z, PARK_X, PARK_HAND_Y = TOP+.150, AXIS_Z+.075, TOP+.160, .090, .245
SECTION_LIFT_Z = TRAVEL_Z
# A steel support bed and round guides with PTFE slide liners. The crowns lie
# outboard of every allowed arrival axis, so gravity directs the pipe inward.
SUPPORT_L, SUPPORT_AXIS_DZ = .140, -.0005
SUPPORT_GUIDE_X, SUPPORT_SIDE_GAP = .0356, .0006
SUPPORT_GUIDE_R = SUPPORT_GUIDE_X-PIPE_R-SUPPORT_SIDE_GAP
SUPPORT_AXIS_Z = AXIS_Z+SUPPORT_AXIS_DZ
SUPPORT_CONTACT_X = .0045
SUPPORT_BED_Z = SUPPORT_AXIS_Z-math.sqrt(PIPE_R**2-SUPPORT_CONTACT_X**2)
SUPPORT_W = 2*(SUPPORT_GUIDE_X+SUPPORT_GUIDE_R)
SUPPORT_FRICTION = (.05,.04)
SECTION_RELEASE_GAP = .002
SECTION_PLACE_Z = SUPPORT_AXIS_Z+CARRY_DZ+SECTION_RELEASE_GAP
SLIDE_OUT, SLIDE_IN = PARK_Y-ENTRY_Y, PARK_Y-JOINT_Y
SLIDE_OUT_S, SLIDE_IN_S = 4.5, 2.5
RUN_SECONDS, FINAL_HOLD_S = 120., 3.5
# First-quadrant angle, rod radius, centre radius. Reflect in both axes.
# The tested sleeves retain their round-6 rods, wall webs and side grasps.
MATCHING_RODS = ((0.,.0045,.0215),(22.5,.0045,.0215),
                 (42.75,.00405,.021856881402606273),
                 (61.5,.00415,.021790654419545877),
                 (75.,.0045,.0215),(90.,.0045,.0215))
SMALL_ROD_COUNT, LARGE_ROD_COUNT = 12, 16
SLEEVE_WEB_X, SLEEVE_WEB_W, SLEEVE_WEB_H = .0215,.0064,.006
WEB_END = .010
SMALL_WEB_OVER_BORE, LARGE_WEB_OVER_BORE = .003,.00425
DECOY_WEB_T, DECOY_WEB_HALF_W = .001,.008


# The hand and both seats touch a true round middle. Only the two end
# portions use the faceted approximation; the closed sleeves slide there.
Y_ROUND = .125
SECTION_CORE_HALF_X, SECTION_CORE_HALF_Z = .0085, .0141
SECTION_STRIP_T, SECTION_EDGE_R = .0019, .00095
SECTION_CHAMFER_L = .0019
SECTION_PROFILE_INWARD_TOL, SECTION_PROFILE_OUTWARD_TOL = .0019, .0005
SECTION_STRIP_OUTERS = (.0104,.01225,.0141)


def chamfered_pipe(length):
    split=2*RIGHT_GRASP+Y_ROUND
    shapes=[cyl((0,(a+b)/2,0),PIPE_R,b-a,axis="y")
            for a,b in ((-Y_ROUND,split),(split,Y_ROUND))]
    for end in (-1,1):
        a,b=sorted((end*Y_ROUND,end*(length/2-SECTION_CHAMFER_L)))
        y=(a+b)/2;span=b-a
        shapes.append(box((0,y,0),(2*SECTION_CORE_HALF_X,span,2*SECTION_CORE_HALF_Z)))
        for sign in (-1,1):
            for outer in SECTION_STRIP_OUTERS:
                height=2*math.sqrt(PIPE_R**2-outer**2)
                shapes.append(box((sign*(outer-SECTION_STRIP_T/2),y,0),
                                  (SECTION_STRIP_T,span,height)))
            shapes.append(cyl((sign*(PIPE_R-SECTION_EDGE_R),y,0),SECTION_EDGE_R,span,axis="y"))
            shapes.append(cyl((0,y,sign*(PIPE_R-SECTION_EDGE_R)),SECTION_EDGE_R,span,axis="y"))
        shapes.append(cyl((0,end*(length/2-SECTION_CHAMFER_L/2),0),CHAMFER_R,SECTION_CHAMFER_L,axis="y"))
    return shapes


def ring_rods(bore=BORE_R, outer=OUTER_R):
    """A closed union of main circular rods; the bore has no centre stop."""
    if bore != BORE_R:
        count=SMALL_ROD_COUNT if bore==SMALL_BORE else LARGE_ROD_COUNT
        r=(outer-bore)/2;rho=bore+r
        return [(rho*math.cos(2*math.pi*i/count),rho*math.sin(2*math.pi*i/count),r)
                for i in range(count)]
    rods=[]
    for deg,r,rho in MATCHING_RODS:
        a=math.radians(deg)
        for sx in (-1,1):
            for sz in (-1,1):
                x,z=sx*rho*math.cos(a),sz*rho*math.sin(a)
                if not any(math.hypot(x-xx,z-zz)<1e-10 for xx,zz,_ in rods):
                    rods.append((x,z,r))
    return sorted(rods,key=lambda p:math.atan2(p[1],p[0])%(2*math.pi))


def coupling(name,pos,bore,outer,length,mass,yaw=0.):
    rods=ring_rods(bore,outer);sh=[]
    for x,z,r in rods:
        if bore==BORE_R:
            half=length/2-r
            sh.extend((cyl((x,0,z),r,2*half,axis="y"),ball((x,-half,z),r),ball((x,half,z),r)))
        else:
            sh.append(cyl((x,0,z),r,length,axis="y"))
    # A solid web inside the wall; real pads close on the outer rods.
    if bore==BORE_R:
        # The left sleeve presents local +x to the TCP, the right local -x.
        # Turning with the same hand keeps this side facing the TCP on the stub.
        web_sign=1 if name=="coupling_a" else -1
        sh.append(box((web_sign*SLEEVE_WEB_X,0,0),(SLEEVE_WEB_W,length-WEB_END,SLEEVE_WEB_H)))
    else:
        web_z=bore+(SMALL_WEB_OVER_BORE if bore==SMALL_BORE else LARGE_WEB_OVER_BORE)
        sh.append(box((0,0,web_z),(2*DECOY_WEB_HALF_W,length-WEB_END,DECOY_WEB_T)))
    return Part(name,pos,sh,mass=mass,color=BLACK,friction=FRICTION,contact_offset=CONTACT_OFFSET,grip=len(sh)-1,yaw=yaw)


def old_run():
    sh=[]
    for sign in (-1,1):
        sh.append(cyl((0,sign*((STUB_END+CHAMFER_L+STUB_OUTER)/2),0),PIPE_R,
                      STUB_OUTER-STUB_END-CHAMFER_L,axis="y"))
        sh.append(cyl((0,sign*(STUB_END+CHAMFER_L/2),0),CHAMFER_R,CHAMFER_L,axis="y"))
        sh.extend((ball((0,sign*ELBOW_Y,0),ELBOW_R),
                   cyl((0,sign*ELBOW_Y,-(AXIS_Z-TOP)/2),PIPE_R,AXIS_Z-TOP,axis="z")))
        sh.append(box((0,sign*CLAMP_Y,-(AXIS_Z-TOP+PIPE_R)/2),(POST_W,POST_D,AXIS_Z-TOP-PIPE_R),color=STEEL))
        sh.append(box((0,sign*CLAMP_Y,PIPE_R+STRAP_T/2),(CLAMP_X,CLAMP_W,STRAP_T),color=STEEL))
        for sx in (-1,1):
            sh.append(box((sx*(CLAMP_X/2-STRAP_T/2),sign*CLAMP_Y,0),(STRAP_T,CLAMP_W,2*PIPE_R),color=STEEL))
    return Part("drain_pipe",(PIPE_X,0,AXIS_Z),sh,color=PVC_GREY,friction=FRICTION,contact_offset=CONTACT_OFFSET)


DRAIN_PIPE = old_run()
SECTION = Part("section",(START_X,SECTION_START_Y,SECTION_START_Z),chamfered_pipe(SECTION_L),mass=SECTION_MASS,
               color=ORANGE,friction=FRICTION,contact_offset=CONTACT_OFFSET)
COUPLING_A = coupling("coupling_a",(COUP_X,COUP_START_Y,COUP_START_Z),BORE_R,OUTER_R,COUP_L,COUP_MASS,yaw=math.pi/2)
COUPLING_B = coupling("coupling_b",(COUP_X,-COUP_START_Y,COUP_START_Z),BORE_R,OUTER_R,COUP_L,COUP_MASS,yaw=-math.pi/2)
SMALL_COUPLING = coupling("small_coupling",(COUP_X,SMALL_START_Y,SMALL_START_Z),SMALL_BORE,SMALL_OUTER,SMALL_L,SMALL_MASS,yaw=math.pi/2)
LARGE_COUPLING = coupling("large_coupling",(COUP_X,LARGE_START_Y,LARGE_START_Z),LARGE_BORE,LARGE_OUTER,LARGE_L,LARGE_MASS,yaw=-math.pi/2)


def tray(side):
    yc=TRAY_Y if side=="left" else -TRAY_Y
    sh=[box((0,0,TRAY_FLOOR_T/2),(2*TRAY_HALF_X,2*TRAY_HALF_Y,TRAY_FLOOR_T))]
    for sx in (-1,1):sh.append(box((sx*(TRAY_HALF_X+TRAY_RIM_T/2),0,TRAY_RIM_H/2),
                                 (TRAY_RIM_T,2*TRAY_HALF_Y,TRAY_RIM_H)))
    # Open tray ends save four scene colliders; the four cradles retain each sleeve.
    for part,outer,bore in ((COUPLING_A,OUTER_R,BORE_R),(SMALL_COUPLING,SMALL_OUTER,SMALL_BORE)) if side=="left" else ((COUPLING_B,OUTER_R,BORE_R),(LARGE_COUPLING,LARGE_OUTER,LARGE_BORE)):
        radius=(outer-bore)/2
        inner=COUP_SUPPORT_X-COUP_SUPPORT_W/2
        height=part.pos[2]-TOP-(outer-radius)-math.sqrt(radius**2-inner**2)-START_GAP
        for sy in (-1,1):
            for sx in (-1,1):sh.append(box((sy*COUP_SUPPORT_Y,part.pos[1]-yc+sx*COUP_SUPPORT_X,height/2),
                                           (COUP_SUPPORT_D,COUP_SUPPORT_W,height)))
    return Part(side+"_tray",(TRAY_X,yc,TOP),sh,color=TRAY_GREY,contact_offset=CONTACT_OFFSET)


LEFT_TRAY, RIGHT_TRAY = tray("left"),tray("right")
CRADLE_CONTACT_X = CRADLE_X-CRADLE_W/2
CRADLE_AXIS_ABOVE_TOP = math.sqrt(PIPE_R**2-CRADLE_CONTACT_X**2)
SECTION_CRADLE_H = SECTION_START_Z-TOP-CRADLE_AXIS_ABOVE_TOP-START_GAP
# Both inner top edges carry the round pipe, forming a shallow V.
SECTION_CRADLES=Part("section_cradles",(START_X,SECTION_START_Y,TOP),[
    box((sign*CRADLE_X,0,SECTION_CRADLE_H/2),(CRADLE_W,CRADLE_L,SECTION_CRADLE_H))
    for sign in (-1,1)
],color=TRAY_GREY,friction=FRICTION,contact_offset=CONTACT_OFFSET)
PIPE_SUPPORT = Part("pipe_support",(PIPE_X,0,TOP),[
    *[box((sign*(SUPPORT_W/2+SUPPORT_CONTACT_X)/2,0,(SUPPORT_BED_Z-TOP)/2),
          (SUPPORT_W/2-SUPPORT_CONTACT_X,SUPPORT_L,SUPPORT_BED_Z-TOP)) for sign in (-1,1)],
    *[cyl((sign*SUPPORT_GUIDE_X,0,SUPPORT_AXIS_Z-TOP),SUPPORT_GUIDE_R,SUPPORT_L,axis="y")
      for sign in (-1,1)],
],color=STEEL,friction=SUPPORT_FRICTION,contact_offset=CONTACT_OFFSET)
OBJECTS = [DRAIN_PIPE,SECTION,COUPLING_A,COUPLING_B,SMALL_COUPLING,LARGE_COUPLING,LEFT_TRAY,RIGHT_TRAY,SECTION_CRADLES,PIPE_SUPPORT]


def axis_ends(state,length):
    axis=state.rotation()[:,1]
    return np.asarray(state.origin)-axis*length/2,np.asarray(state.origin)+axis*length/2


def line_metrics(state,length):
    ends=sorted(axis_ends(state,length),key=lambda p:p[1])
    angle=math.degrees(math.acos(float(np.clip(abs(state.rotation()[1,1]),0,1))))
    radial=[float(math.hypot(p[0]-PIPE_X,p[2]-AXIS_Z)) for p in ends]
    centre=math.hypot(state.origin[0]-PIPE_X,state.origin[2]-AXIS_Z)
    return dict(ends=ends,angle=angle,radial=radial,centre=centre,span=(ends[0][1],ends[1][1]))


def overlap(a,b):
    return max(0.,min(a[1],b[1])-max(a[0],b[0]))


def section_in_line(section):
    d=line_metrics(section,SECTION_L)
    return (d["angle"]<=IN_LINE_DEG and max(d["radial"])<=IN_LINE_TOL and
            d["span"][0]>=-STUB_END-PEN_TOL and d["span"][1]<=STUB_END+PEN_TOL)


def coupling_metrics(c,section):
    d=line_metrics(c,COUP_L)
    d["coax"]=d["angle"]<=COAX_DEG and d["centre"]<=COAX_TOL
    d["stub"]={"left":overlap(d["span"],(STUB_END,STUB_OUTER)),
               "right":overlap(d["span"],(-STUB_OUTER,-STUB_END))}
    d["section"]=overlap(d["span"],line_metrics(section,SECTION_L)["span"])
    return d


def coupled(objects,side):
    return any(d["coax"] and d["stub"][side]>=MIN_OVERLAP and d["section"]>=MIN_OVERLAP
               for d in (coupling_metrics(objects[n],objects["section"]) for n in SLIP_NAMES))


def on_run(c,section):
    d=coupling_metrics(c,section)
    # A section can be staged away from the old line: compare to its own line.
    R=section.rotation();local=R.T@(np.asarray(c.origin)-section.origin)
    rel_angle=math.degrees(math.acos(float(np.clip(abs(c.rotation()[:,1]@R[:,1]),0,1))))
    sec_ends=[float((R.T@(p-section.origin))[1]) for p in axis_ends(c,COUP_L)]
    on_section=(rel_angle<=COAX_DEG and math.hypot(local[0],local[2])<=COAX_TOL and
                overlap(sorted(sec_ends),(-SECTION_L/2,SECTION_L/2))>=MIN_OVERLAP)
    return bool((d["coax"] and max(d["stub"].values())>=MIN_OVERLAP) or on_section)


def lowest_point(state,spec):
    # Exact primitive extrema under the measured full rotation; no AABB tests.
    values=[]
    for kind,c,R,d in spec.world_shapes(state.origin,state.rotation()):
        extent=(abs(R[2])@ (np.asarray(d)/2) if kind=="box" else d[0] if kind=="ball" else
                abs(R[2,2])*d[1]/2+d[0]*math.sqrt(max(0.,1-R[2,2]**2)))
        values.append(c[2]-extent)
    return min(values)


_POSE_EPISODE, _POSE_TOKEN, _POSE_HISTORY, _QUIET = None,None,[],False


def at_rest(objects):
    """Centres and local-y axis directions over the last second; axial spin is free."""
    global _POSE_EPISODE,_POSE_TOKEN,_QUIET
    if _RUN["t"] is None:
        return all(objects[n].resting(REST_SPEED,REST_SPIN) for n in SCORED_NAMES)
    if _POSE_EPISODE!=_RUN["episode"]:
        _POSE_EPISODE=_RUN["episode"];_POSE_TOKEN=None;_POSE_HISTORY.clear();_QUIET=False
    token=(_RUN["step"],float(_RUN["t"]))
    if _POSE_TOKEN is None or token[0]!=_POSE_TOKEN[0]:
        if _POSE_TOKEN is not None and (token[0]!=_POSE_TOKEN[0]+1 or token[1]<=_POSE_TOKEN[1]):
            _POSE_HISTORY.clear()
        centres=np.array([objects[n].origin for n in SCORED_NAMES])
        axes=np.array([objects[n].rotation()[:,1] for n in SCORED_NAMES])
        _POSE_HISTORY.append((token[1],centres,axes))
        cutoff=token[1]-STILL_S
        # Retain the sample at/before the start of the window to prove coverage.
        while len(_POSE_HISTORY)>1 and _POSE_HISTORY[1][0]<=cutoff+1e-10:
            _POSE_HISTORY.pop(0)
        _QUIET=bool(_POSE_HISTORY[0][0]<=cutoff+1e-10 and all(
            np.all(np.linalg.norm(p-centres,axis=1)<=POSE_WINDOW_M+1e-12) and
            np.all(np.abs(np.sum(a*axes,axis=1))>=math.cos(math.radians(POSE_WINDOW_DEG))-1e-12)
            for _,p,a in _POSE_HISTORY))
        _POSE_TOKEN=token
    return _QUIET


def score(objects):
    # Setting a part down on the bench is allowed; falling off the bench is a sticky 0.
    fell=ever("fell off bench",any(lowest_point(objects[p.name],p)<TOP-OFF_BENCH for p in (SECTION,COUPLING_A,COUPLING_B)))
    quiet=at_rest(objects)
    release_ok=all(released(objects[n]) for n in SCORED_NAMES)
    in_line=section_in_line(objects["section"])
    joints=[coupled(objects,side) for side in ("left","right")]
    ready=not fell and in_line and all(joints) and release_ok
    # The geometric/release hold runs alongside the pose window, not after it.
    seconds=held_for("released aligned repair",ready)
    done=ready and quiet and seconds+1e-10>=STILL_S
    if fell:return 0.
    if in_line and all(joints):return 1. if done else .75
    staged=all(on_run(objects[n],objects["section"]) for n in SLIP_NAMES)
    if in_line and (sum(joints)==1 or staged):return .5
    return .25 if staged else 0.


def success(objects):
    return score(objects)>=1.


def grip_tcp(x,y,z,yaw,shift):
    """Place the gripped axis at (x,y,z), allowing its measured gripper-x offset."""
    return tuple(np.array([x,y,z])-shift*grip_rotation(yaw)[:,0])


def demo():
    # Durations and known start poses only.
    # Measured offsets: sleeves +21.1 mm and section +2 mm along gripper x.
    # Measured section hang: CARRY_DZ=4 mm, including drive sag.
    # Both fingers saturate independently on a 52 mm sleeve; its +x finger is open.
    # Lift before the yaw turn: the offset sleeve rotates about the TCP.
    out={}
    for side,sign,yaw in (("left",1,-math.pi/2),("right",-1,math.pi/2)):
        def w(dt,x,y,z,f,label,turn=None,shift=SLEEVE_GRIP_SHIFT):
            angle=yaw if turn is None else turn
            return Waypoint(dt,grip_tcp(x,y,z,angle,shift),f,label,angle)
        out[side]=[
            w(1.5,COUP_X,sign*COUP_START_Y,TRAVEL_Z,FINGER_OPEN,"tray approach",0.),
            w(1.5,COUP_X,sign*COUP_START_Y,COUP_START_Z,FINGER_OPEN,"tray descend",0.),
            w(1.,COUP_X,sign*COUP_START_Y,COUP_START_Z,FINGER_CLOSED,"tray close",0.),
            w(1.5,COUP_X,sign*COUP_START_Y,TRAVEL_Z,FINGER_CLOSED,"tray lift",0.),
            w(2.,PIPE_X,sign*ENTRY_Y,TRAVEL_Z,FINGER_CLOSED,"above open gap"),
            w(1.5,PIPE_X,sign*ENTRY_Y,AXIS_Z+COUPLING_DZ,FINGER_CLOSED,"gap descend"),
            w(SLIDE_OUT_S,PIPE_X,sign*PARK_Y,AXIS_Z+COUPLING_DZ,FINGER_CLOSED,"park sleeve"),
            w(1.,PIPE_X,sign*PARK_Y,AXIS_Z+COUPLING_DZ,FINGER_OPEN,"park open",shift=SLEEVE_RELEASE_SHIFT),
            w(1.2,PIPE_X,sign*PARK_Y,CLEAR_Z,FINGER_OPEN,"park rise",shift=SLEEVE_RELEASE_SHIFT),
        ]
    left,right=out["left"],out["right"]
    def add(plan,dt,x,y,z,f,label,yaw,shift):
        plan.append(Waypoint(dt,grip_tcp(x,y,z,yaw,shift),f,label,yaw))
    def L(dt,x,y,z,f,label,shift=SLEEVE_GRIP_SHIFT):add(left,dt,x,y,z,f,label,-math.pi/2,shift)
    def R(dt,x,y,z,f,label,shift=SLEEVE_GRIP_SHIFT):add(right,dt,x,y,z,f,label,math.pi/2,shift)
    def RS(dt,x,y,z,f,label):R(dt,x,y,z,f,label,SECTION_GRIP_SHIFT)
    # Both parking plans end at 15.7 s. Only the right hand handles the span.
    # Pre-offset its close as well as the carry, preserving the replay offset.
    RS(1.5,START_X,SECTION_START_Y+RIGHT_GRASP,CLEAR_Z,FINGER_OPEN,"section approach")
    RS(1.5,START_X,SECTION_START_Y+RIGHT_GRASP,SECTION_START_Z,FINGER_OPEN,"section descend")
    RS(1.,START_X,SECTION_START_Y+RIGHT_GRASP,SECTION_START_Z,FINGER_CLOSED,"section close")
    RS(2.,START_X,SECTION_START_Y+RIGHT_GRASP,SECTION_LIFT_Z,FINGER_CLOSED,"section lift")
    RS(2.,PIPE_X,RIGHT_GRASP,SECTION_LIFT_Z,FINGER_CLOSED,"section carry")
    RS(2.,PIPE_X,RIGHT_GRASP,SECTION_PLACE_Z,FINGER_CLOSED,"section lower")
    RS(1.,PIPE_X,RIGHT_GRASP,SECTION_PLACE_Z,FINGER_OPEN,"section release")
    RS(1.2,PIPE_X,RIGHT_GRASP,CLEAR_Z,FINGER_OPEN,"section release rise")
    R(1.2,PIPE_X,-PARK_Y,CLEAR_Z,FINGER_OPEN,"section move clear")
    wait=sum(w.duration for w in right)-sum(w.duration for w in left)
    last=left[-1]
    left.append(Waypoint(wait,last.pos,FINGER_OPEN,"wait clear of placement",last.yaw))
    # Both sleeves now slide together; the stand supports and aligns the span.
    # Keep the tested pre-offset side-wall re-grip and low slide height.
    for P,sign,label in ((L,1,"left"),(R,-1,"right")):
        P(1.2,PIPE_X,sign*PARK_Y,CLEAR_Z,FINGER_OPEN,label+" sleeve approach")
        P(1.2,PIPE_X,sign*PARK_Y,AXIS_Z+COUPLING_DZ,FINGER_OPEN,label+" sleeve descend")
        P(1.,PIPE_X,sign*PARK_Y,AXIS_Z+COUPLING_DZ,FINGER_CLOSED,label+" sleeve close")
        P(SLIDE_IN_S,PIPE_X,sign*JOINT_Y,AXIS_Z+COUPLING_DZ,FINGER_CLOSED,label+" joint slide")
        P(1.,PIPE_X,sign*JOINT_Y,AXIS_Z+COUPLING_DZ,FINGER_OPEN,label+" sleeve open",SLEEVE_RELEASE_SHIFT)
        P(1.2,PIPE_X,sign*JOINT_Y,CLEAR_Z,FINGER_OPEN,label+" sleeve rise",SLEEVE_RELEASE_SHIFT)
        P(1.2,PARK_X,sign*PARK_HAND_Y,PARK_Z,FINGER_OPEN,label+" park",0.)
    end=max(sum(w.duration for w in p) for p in out.values())
    for side,plan in out.items():
        last=plan[-1]
        plan.append(Waypoint(end-sum(w.duration for w in plan)+FINAL_HOLD_S,last.pos,FINGER_OPEN,"settle clear",last.yaw))
    return out
