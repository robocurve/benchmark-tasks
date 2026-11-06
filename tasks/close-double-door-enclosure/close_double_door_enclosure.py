"""Contact-enforced closing order for an overlapping pair of enclosure doors."""
import math
import pathlib
import sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import box, cyl, ball
from yam_joints import Mechanism, Link, Hinge, Slide
from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED
from yam_arena.demo import Waypoint
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
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
FRONT, HALF, DEPTH, HEIGHT = .215, .120, .140, .170
LEAF_W, LEAF_H, LEAF_T, LEAF_Z = .118, .152, .003, .085
PASSIVE_INITIAL, ACTIVE_INITIAL = 50., 70.
HINGE_DAMPING = .020
BOLT_TRAVEL, BOLT_MASS = .015, .020
BOLT_X, BOLT_Y = -.010, .040
ROD_R, ROD_BOTTOM, ROD_TIP = .004, .120, .158
POST_X, POST_Z, POST_H, POST_R = -.058, .1375, .025, .008
BORE_HALF, KEEPER_Z, KEEPER_H = .008, .174, .012
CATCH_R, CATCH_K, CATCH_REST, CATCH_TRAVEL = .003, 300., .0038, .004
# Front-side ball retracts toward -x; its spring pushes the rod toward +x,
# closing the passive hinge. The shallow neck provides a physical shoulder.
CATCH_CENTER = (BOLT_X-.0058, BOLT_Y, .167)
NECK_R, NECK_Z, NECK_H = .003, CATCH_CENTER[2]-BOLT_TRAVEL, .005
PEN_TOL = .002
CLOSED_DEG, BOLTED_Q = 1., .010
# Behind a shut astragal the passive sheet can sit ajar up to 2.92 deg (4.0 deg
# with the active leaf 1 deg open); it meets the astragal's front at 6.46 deg.
WRONG_LOW, WRONG_HIGH, WRONG_HOLD_S = 4.5, 8., .25
HINGE_SPEED, BOLT_SPEED = .05, .010
RUN_SECONDS = 60.
BOLT_TCP_Z = .145
BOLT_LIFT_OVERDRIVE = .005  # Vertical compliance compensation; tune with measured GPU lag.
PUSH_Z, PUSH_Y, ACTIVE_PUSH_Y, PUSH_DEPTH = .155, .085, .085, .0015
PUSH_PAST_DEG, PUSH_HOLD_S = 6., .8
PARK_Z = TOP+.180
BLUE, STEEL, BRASS = (.12,.32,.52), (.48,.52,.57), (.85,.56,.12)

# All coordinates below are relative to the front plane and tabletop.
shell = [box((DEPTH/2,0,.003), (DEPTH,.252,.006)),
         box(((DEPTH+.012)/2,0,HEIGHT-.003), (DEPTH-.012,.252,.006)),
         box((DEPTH-.003,0,HEIGHT/2), (.006,.252,HEIGHT)),
         box(((DEPTH+.008)/2,.123,HEIGHT/2), (DEPTH-.008,.006,HEIGHT)),
         box(((DEPTH+.008)/2,-.123,HEIGHT/2), (DEPTH-.008,.006,HEIGHT))]
# Front flange is behind the sheet; the hinge stop supplies the closed limit.
shell += [box((.013,s*.114,.085),(.006,.012,.158)) for s in (-1,1)]
shell += [box((.013,0,z),(.006,.216,.006)) for z in (.010,.160)]
# Four-box keeper with a 16 mm square vertical bore and 4 mm walls.
for s in (-1,1):
    shell += [box((BOLT_X+s*.010,BOLT_Y,KEEPER_Z),(.004,.024,KEEPER_H),color=STEEL),
              box((BOLT_X,BOLT_Y+s*.010,KEEPER_Z),(.016,.004,KEEPER_H),color=STEEL)]
# A rear bridge fastens the keeper to the steel header.
shell += [box((.007,BOLT_Y,.174),(.010,.024,.012),color=STEEL)]
ENCLOSURE = Mechanism("enclosure", (FRONT,0,TOP), shell,
    [Link("catch", [ball(CATCH_CENTER,CATCH_R,color=BRASS)],
          Slide("x",CATCH_CENTER,-CATCH_TRAVEL,0,spring=CATCH_K,rest=CATCH_REST,damping=.05),
          mass=.003,friction=(.8,.6))],color=BLUE,friction=(.8,.6),contact_offset=.0005)


def leaf_shapes(sign,active=False):
    # sign=+1 passive, sign=-1 active. Pull is a vertical bar, near the seam.
    shapes = [box((0,sign*(HALF-LEAF_W/2),LEAF_Z),(LEAF_T,LEAF_W,LEAF_H)),
              cyl((-.014,sign*.024,.092),.004,.050,color=BRASS),
              box((-.007,sign*.024,.070),(.014,.008,.006),color=BRASS),
              box((-.007,sign*.024,.114),(.014,.008,.006),color=BRASS)]
    if active:
        shapes += [box((-.009, .005,.085),(.003,.018,LEAF_H),color=STEEL),
                   box((-.0045,-.005,.085),(.006,.004,LEAF_H),color=STEEL)]
    else:
        # Two four-box guides; parent/child collision filtering is intentional.
        for z in (.125,.148):
            for s in (-1,1):
                shapes += [box((BOLT_X+s*.0075,BOLT_Y,z),(.003,.018,.005),color=STEEL),
                           box((BOLT_X,BOLT_Y+s*.0075,z),(.012,.003,.005),color=STEEL)]
    return shapes

# Three coaxial cylinders leave the retaining neck genuinely narrower; the
# original ball lead-in is above it and does not fill the groove.
BOLT_SHAPES = [cyl((BOLT_X,BOLT_Y,(lo+hi)/2),radius,hi-lo)
               for lo,hi,radius in ((ROD_BOTTOM,NECK_Z-NECK_H/2,ROD_R),
                                    (NECK_Z-NECK_H/2,NECK_Z+NECK_H/2,NECK_R),
                                    (NECK_Z+NECK_H/2,ROD_TIP,ROD_R))]
BOLT_SHAPES += [
               ball((BOLT_X,BOLT_Y,ROD_TIP),ROD_R),
               box(((POST_X+BOLT_X)/2,BOLT_Y,.122), (BOLT_X-POST_X,.006,.006)),
               cyl((POST_X,BOLT_Y,POST_Z),POST_R,POST_H,color=BRASS)]
DOOR_LEFT = Mechanism("door_left",(FRONT,0,TOP),
    [cyl((0,HALF,.085),.003,.140)],
    [Link("leaf",leaf_shapes(1),Hinge("z",(0,HALF,.085),-100,0,
          initial=-PASSIVE_INITIAL,damping=HINGE_DAMPING),mass=.180),
     Link("bolt",BOLT_SHAPES,Slide("z",(BOLT_X,BOLT_Y,.140),0,BOLT_TRAVEL,damping=.02),
          mass=BOLT_MASS,parent="leaf",color=STEEL,friction=(.8,.6))],
    color=BLUE,friction=(.8,.6),contact_offset=.0005)
DOOR_RIGHT = Mechanism("door_right",(FRONT,0,TOP),
    [cyl((0,-HALF,.085),.003,.140)],
    [Link("leaf",leaf_shapes(-1,True),Hinge("z",(0,-HALF,.085),0,100,
          initial=ACTIVE_INITIAL,damping=HINGE_DAMPING),mass=.200)],
    color=BLUE,friction=(.8,.6),contact_offset=.0005)
OBJECTS = [ENCLOSURE,DOOR_LEFT,DOOR_RIGHT]

_POSE_EPISODE = None
_POSE_TOKEN = None
_PREVIOUS_Q = None
_QUIET = False


def at_rest(objects):
    """Measured q change per control step online; reported velocity offline."""
    global _POSE_EPISODE, _POSE_TOKEN, _PREVIOUS_Q, _QUIET
    l,r=objects["door_left"],objects["door_right"]
    q=np.array([l.joints["leaf"],r.joints["leaf"],l.joints["bolt"]])
    limits=np.array([HINGE_SPEED,HINGE_SPEED,BOLT_SPEED])
    if _RUN["t"] is None:
        return bool(np.all(np.abs([l.joint_vel["leaf"],r.joint_vel["leaf"],l.joint_vel["bolt"]])<limits))
    if _POSE_EPISODE != _RUN["episode"]:
        _PREVIOUS_Q,_POSE_TOKEN,_QUIET=None,None,False
        _POSE_EPISODE=_RUN["episode"]
    token=(_RUN["step"],float(_RUN["t"]))
    if token != _POSE_TOKEN:
        _QUIET=False
        if _PREVIOUS_Q is not None:
            step,t,old=_PREVIOUS_Q
            dt=token[1]-t
            _QUIET=bool(token[0]==step+1 and 0<dt<=.050001 and np.all(np.abs(q-old)<limits*dt))
        _PREVIOUS_Q=(token[0],token[1],q.copy())
        _POSE_TOKEN=token
    return _QUIET


def score(objects):
    l,r=objects["door_left"],objects["door_right"]
    passive,active=-l.angle_deg("leaf"),r.angle_deg("leaf")
    pclosed,aclosed=abs(passive)<=CLOSED_DEG,abs(active)<=CLOSED_DEG
    bolted=pclosed and l.joints["bolt"]>=BOLTED_Q
    band=WRONG_LOW<=passive<=WRONG_HIGH and aclosed
    forced_s=held_for("astragal band",band)
    violation=ever("forced astragal",band and forced_s+1e-10>=WRONG_HOLD_S)
    quiet=at_rest(objects)
    free=all(released(s.link_view(link)) for s,link in ((l,"leaf"),(r,"leaf"),(l,"bolt")))
    done=pclosed and bolted and aclosed and quiet and free and not violation
    still_s=held_for("closed secured released still",done)
    if violation:return 0.
    if pclosed and bolted and aclosed:return 1. if done and still_s+1e-10>=STILL_S else .75
    if pclosed and aclosed:return .5
    if pclosed and bolted:return .25
    return 0.


def success(objects):
    return score(objects)>=1.


def push_target(side,degrees,backoff=0.):
    sign=1 if side=="left" else -1
    angle=math.radians(-sign*degrees)
    c,s=math.cos(angle),math.sin(angle)
    point=np.array([-.0015-.018+PUSH_DEPTH-backoff,sign*(PUSH_Y if side=="left" else ACTIVE_PUSH_Y),PUSH_Z])
    anchor=np.array([0,sign*HALF,.085])
    R=np.array([[c,-s,0],[s,c,0],[0,0,1]])
    return tuple(np.array([FRONT,0,TOP])+anchor+R@(point-anchor))


def demo():
    # Known layout and elapsed time only; never read task objects or scorer.
    plans={}
    for side,start in (("left",PASSIVE_INITIAL),("right",ACTIVE_INITIAL)):
        sign=1 if side=="left" else -1
        park=(.105,sign*.250,PARK_Z)
        def w(dt,p,f,label,yaw=0.):return Waypoint(dt,p,f,label,yaw)
        ws=[w(1.5,park,FINGER_CLOSED,"park before work")]
        if side=="right":ws += [w(17.,park,FINGER_CLOSED,"wait for passive bolt")]
        yaw=math.radians(-sign*start)
        ws += [w(.8,push_target(side,start,.055),FINGER_CLOSED,"face approach",yaw),
               w(.7,push_target(side,start),FINGER_CLOSED,"face touch",yaw)]
        for angle in np.linspace(start-5,-PUSH_PAST_DEG,int((start+PUSH_PAST_DEG)/5)+1):
            ws.append(w(.28,push_target(side,float(angle)),FINGER_CLOSED,f"push {angle:.2f}",math.radians(-sign*angle)))
        press=push_target(side,-PUSH_PAST_DEG)
        # The physical leaf is against its zero stop. Retreat along its world
        # -x normal with yaw fixed before changing y, z or yaw.
        retreat=(press[0]-.060,press[1],press[2])
        ws += [w(PUSH_HOLD_S,press,FINGER_CLOSED,"closed press",math.radians(sign*PUSH_PAST_DEG)),
               w(.6,retreat,FINGER_CLOSED,"face back off",math.radians(sign*PUSH_PAST_DEG))]
        if side=="left":
            p=(FRONT+POST_X,BOLT_Y,TOP+BOLT_TCP_Z)
            hover=(p[0],p[1],TOP+.195)
            up=(p[0],p[1],p[2]+BOLT_TRAVEL+BOLT_LIFT_OVERDRIVE)
            ws += [w(.4,(retreat[0],retreat[1],hover[2]),FINGER_CLOSED,"face rise",math.radians(sign*PUSH_PAST_DEG)),
                   w(.7,hover,FINGER_OPEN,"bolt hover"),w(.7,p,FINGER_OPEN,"bolt descend"),
                   w(.6,p,FINGER_CLOSED,"bolt pinch"),w(1.,up,FINGER_CLOSED,"bolt lift"),
                   w(.5,up,FINGER_CLOSED,"bolt hold"),w(.5,up,FINGER_OPEN,"bolt release"),
                   w(.6,hover,FINGER_OPEN,"bolt rise"),w(.8,park,FINGER_OPEN,"park after bolt")]
        else:ws += [w(.8,park,FINGER_CLOSED,"park after active")]
        plans[side]=ws
    # Left remains clear while right works, with at least three seconds at the end.
    duration=max(sum(w.duration for w in ws) for ws in plans.values())+4.0
    for side,ws in plans.items():
        park=(.105,(1 if side=="left" else -1)*.250,PARK_Z)
        ws.append(Waypoint(duration-sum(w.duration for w in ws),park,FINGER_OPEN,"hold clear"))
    return plans
