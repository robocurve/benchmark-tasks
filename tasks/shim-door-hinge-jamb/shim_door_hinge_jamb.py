"""Shim a prehung door's hinge jamb with two opposed tapered wood shims.
The gap changes by trial; the shims slide by real convex contact against a passive spring-backed jamb.
Drive a snug, centred pack without bowing the jamb, release both hands, and let it settle.
"""
import math
import pathlib
import sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, PartState, box, cyl
import yam_parts as parts
import yam_joints as joints
import yam_mesh as mesh
from yam_joints import Mechanism, Link, Slide
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP
SCENE = "station"
# Release-and-stillness block, verbatim from the parts kit's docs (release_stillness_block.py).
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


# Geometry in metres, z relative to the tabletop unless explicitly world.
TOP = TABLE_TOP[SCENE]
IN, XS, Z_HINGE, G0 = 0.0254, 0.260, 0.130, 0.0040
L, W, T_BUTT, T_TIP = 0.2032, 0.0349, 0.00635, 0.00079
SLOPE = (T_BUTT - T_TIP) / L
YJ = 0.057945
DEMO_PACE = 1.0
FORCE_DIAG = False
ACTIVE_GAP, TEST_GAP, FORCE_GAP = 0.0050, None, None
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
RUN_LIMIT_STEPS = 3600
STEEL, PINE, GYPSUM = (0.45, 0.47, 0.50), (0.86, 0.74, 0.52), (0.84, 0.84, 0.82)
WHITE, NICKEL, CEDAR, MDF = (0.93, 0.92, 0.88), (0.70, 0.70, 0.68), (0.62, 0.36, 0.22), (0.35, 0.42, 0.55)


def gap_for_seed(seed):
    return 0.0050 if seed is None else 0.0040 + 0.00005 * (int(seed) % 48)


def trial_gap():
    return float(next(g for g in (TEST_GAP, FORCE_GAP, ACTIVE_GAP) if g is not None))


def _b(x0, x1, y0, y1, z0, z1, color=None):
    return box(((x0+x1)/2, (y0+y1)/2, (z0+z1)/2), (x1-x0, y1-y0, z1-z0), color=color)


WALL = Part("wall", (0, 0, TOP), [
    _b(.150,.330,-.090,.090,0,.006,color=(.30,.31,.33)),
    _b(.184,.222,-.04445,.04445,.006,.296,color=PINE),
    _b(.222,.260,-.04445,.04445,.006,.296,color=PINE),
    _b(.170,.260,.04445,.05715,.006,.296,color=GYPSUM),
    _b(.170,.260,-.05715,-.04445,.006,.296,color=GYPSUM),
], friction=(.40,.30), contact_offset=.001)
JAMB = Mechanism("jamb", (XS,0,TOP), [
    _b(.002,.040,-.030,.030,.006,.014), _b(.040,.056,-.020,.020,.006,.300),
    _b(.002,.056,-.020,.020,.288,.300),
], [
    Link("setter", [_b(.020,.036,-.010,.010,.008,.012)],
         Slide("x", (.028,0,.010), -.0005,.0030,static_friction=200.,dynamic_friction=200.), .20),
    Link("jamb", [
        _b(.0040,.02146,-YJ,YJ,.016,.286),
        _b(.02146,.02176,-.05495,-.02295,.0855,.1745,color=NICKEL),
        cyl((.02746,-.0560,.130),.00635,.089,axis="z",color=NICKEL),
        _b(.02146,.0340,-.020,.020,.016,.028,color=STEEL),
        _b(.02146,.0340,-.020,.020,.274,.286,color=STEEL),
    ], Slide("x", (.004+.00873,0,.151),-.0035,.0060,spring=7000.,damping=50.,rest=0.),
         .35,parent="setter",color=WHITE,friction=(.40,.30)),
], color=STEEL,friction=(.40,.30),contact_offset=.001,self_collide=False)


def holder(name, y):
    return Part(name,(.190,y,TOP),[
        _b(-.025,.025,-.110,.110,0,.022),
        _b(-.025,-.0035,-.110,.110,.022,.030),
        _b(.0035,.025,-.110,.110,.022,.030),
    ],color=MDF,friction=(.40,.30),contact_offset=.001)


HOLDER_LEFT, HOLDER_RIGHT = holder("holder_left",.205), holder("holder_right",-.205)


def thickness(u):
    return T_TIP + SLOPE*u


PROXIES = tuple(_b(.00005,thickness(k*L/8)-.00005,-L/2+k*L/8,-L/2+(k+1)*L/8,
                   -W/2+.0002,W/2-.0002) for k in range(8))
WEDGE = mesh.prism([(0,-L/2),(T_TIP,-L/2),(T_BUTT,L/2),(0,L/2)],W,axis="z",
                   name="wedge",collision="convex",proxy=PROXIES)


class ShimPart(mesh.MeshPart):
    @property
    def free(self):
        return False  # Dynamic at runtime; exclude the narrow slots from seeded jitter.


HOME_LEFT = np.array([.1868,.325-L/2,TOP+.0221+W/2])
HOME_RIGHT = np.array([.1932,-(.325-L/2),TOP+.0221+W/2])
SHIM_LEFT = ShimPart("shim_left",HOME_LEFT,mass=.010,color=CEDAR,friction=(.40,.30),
                     contact_offset=.001,meshes=(WEDGE,))
SHIM_RIGHT = ShimPart("shim_right",HOME_RIGHT,mass=.010,color=CEDAR,yaw=math.pi,
                      friction=(.40,.30),contact_offset=.001,meshes=(WEDGE,))
OBJECTS = [WALL,JAMB,HOLDER_LEFT,HOLDER_RIGHT,SHIM_LEFT,SHIM_RIGHT]
# The shims are dynamic but never jittered, so the offline checker sees them as fixed: the pads' pinch on them
# is advisory there and the GPU demo decides.
PRESSED_PARTS = ("shim_left", "shim_right")


def _patch_shim_scene(scene):
    import functools
    mesh._patch_scene(scene)
    if getattr(scene,"_shim_token",None) is _SHIM_TOKEN:
        return
    old = scene.task_object
    @functools.wraps(old)
    def task_object(spec):
        if not isinstance(spec,ShimPart):
            return old(spec)
        # The plain MeshPart is free, hence Arena builds a RIGID object. The kit
        # registers it in parts._SPECS and _PART_NAMES for its normal state reader.
        temp = mesh.MeshPart(spec.name,spec.pos,shapes=(),meshes=spec.meshes,mass=spec.mass,
                             color=spec.color,yaw=spec.yaw,friction=spec.friction,
                             contact_offset=spec.contact_offset)
        obj = mesh._mesh_part_object(temp)
        from isaaclab.sim.utils import clone
        from shim_runtime import spawn_shim
        obj.spawner_cfg.func = clone(spawn_shim)
        return obj
    scene.task_object = task_object
    scene._shim_token = _SHIM_TOKEN


_SHIM_TOKEN = object()


def _install_shim_hooks():
    import importlib.machinery
    class Finder(importlib.machinery.PathFinder):
        _shim_scene_finder = True
        def find_spec(self,fullname,path=None,target=None):
            if fullname != "yam_arena.scene":
                return None
            spec = importlib.machinery.PathFinder.find_spec(fullname,path)
            if spec is not None and spec.loader is not None:
                spec.loader = parts._SceneLoader(spec.loader,_patch_shim_scene)
            return spec
    sys.meta_path[:] = [f for f in sys.meta_path if not getattr(f,"_shim_scene_finder",False)]
    sys.meta_path.insert(0,Finder())
    if "yam_arena.scene" in sys.modules:
        _patch_shim_scene(sys.modules["yam_arena.scene"])


_install_shim_hooks()

# Pose-only geometric measurements. Text reports lengths in mm.
def gap_now(jamb):
    return G0 + float(jamb.joints["setter"]) + float(jamb.joints["jamb"])


def bow(jamb):
    return abs(float(jamb.joints["jamb"]))


def shim_axes(shim):
    R = shim.rotation()
    return R[:,0],R[:,1],R[:,2]


def tip(shim):
    return np.asarray(shim.origin)-shim_axes(shim)[1]*L/2


def butt(shim):
    return np.asarray(shim.origin)+shim_axes(shim)[1]*L/2


def thickness_at(shim,y):
    el = shim_axes(shim)[1]
    return thickness((y-shim.origin[1])/el[1]+L/2) if abs(el[1])>1e-12 else math.inf


def side(shim):
    t,b = tip(shim)[1],butt(shim)[1]
    if b>YJ and t<b: return "left"
    if b<-YJ and t>b: return "right"
    return None


def wall_interval(shim):
    low,high = sorted((float(tip(shim)[1]),float(butt(shim)[1])))
    return max(low,-YJ),min(high,YJ)


def in_wall_points(shim):
    low,high = wall_interval(shim)
    if high<=low: return []
    et,el,ew = shim_axes(shim)
    o = np.asarray(shim.origin)
    return [o+el*((y-o[1])/el[1])+ew*z+et*x
            for y in (low,high) for z in (-.45*W,.45*W) for x in (0.,thickness_at(shim,y))]


def in_gap(shim,jamb):
    et,el,_ = shim_axes(shim)
    if abs(el[1])<math.cos(math.radians(3)) or abs(et[0])<math.cos(math.radians(3)):
        return False
    if abs(shim.origin[2]-(TOP+Z_HINGE))>.025+1e-12: return False
    points = in_wall_points(shim)
    return bool(points) and all(XS-.0005-1e-12<=p[0]<=XS+gap_now(jamb)+.0005+1e-12 for p in points)


def overlap_interval(a,b):
    left,right = (a,b) if side(a)=="left" else (b,a)
    return max(tip(left)[1],butt(right)[1]), min(butt(left)[1],tip(right)[1])


def pair(a,b,jamb):
    if {side(a),side(b)} != {"left","right"}: return False
    if not (in_gap(a,jamb) and in_gap(b,jamb)): return False
    et_a,_,_ = shim_axes(a)
    et_b,_,_ = shim_axes(b)
    if et_a[0]*et_b[0]>=0: return False
    lower,upper = (a,b) if et_a[0]>0 else (b,a)
    if lower.origin[0]>=upper.origin[0]: return False
    lo,hi = overlap_interval(a,b)
    return bool(hi-lo>=.070-1e-12 and lo<=-.020+1e-12 and hi>=.020-1e-12)


def clear0(a,b,jamb):
    return gap_now(jamb)-(thickness_at(a,0.)+thickness_at(b,0.))


def carries(shim,jamb):
    if not in_gap(shim,jamb) or side(shim) is None: return False
    lo,hi = wall_interval(shim)
    return min(gap_now(jamb)-thickness_at(shim,y) for y in (lo,hi))<=.0003+1e-12


SNUG_FREE = .00005  # snug: the pack bears on the jamb (free space at most 0.05 mm, the thinnest feeler gauge)
BOW_BANDS = ((.0004,1.),(.0008,.75),(.0016,.5))  # 1/64, 1/32 and 1/16 in


def terms(objects):
    a,b,j = objects["shim_left"],objects["shim_right"],objects["jamb"]
    p = pair(a,b,j)
    c = clear0(a,b,j) if p else math.inf
    snug = p and c<=SNUG_FREE+1e-12
    d = bow(j)
    if snug:
        tier = next((v for limit,v in BOW_BANDS if d<=limit+1e-12),.25)
    elif p and c<=.0015+1e-12: tier = .25  # a pair in place but not touching the jamb carries nothing
    elif not p and (carries(a,j) or carries(b,j)): tier = .25
    else: tier = 0.
    lo,hi = overlap_interval(a,b) if p else (0.,0.)
    return dict(pair=p,overlap=hi-lo,clear0=c,bow=d,tier=tier)


_WINDOW = {"episode":None,"samples":[]}


def _angle_deg(a,b):
    return math.degrees(math.acos(float(np.clip((np.trace(a.T@b)-1)/2,-1,1))))


def still(a,b,jamb):
    if _RUN["t"] is None:
        return bool(a.resting(.002,.05) and b.resting(.002,.05) and abs(jamb.joint_vel["jamb"])<.002)
    if _WINDOW["episode"]!=_RUN["episode"]:
        _WINDOW.update(episode=_RUN["episode"],samples=[])
    samples = _WINDOW["samples"]
    step,t = _RUN["step"],float(_RUN["t"])
    if not samples or samples[-1][0]!=step:
        if samples and (samples[-1][0]+1!=step or t<=samples[-1][1]): samples.clear()
        samples.append((step,t,np.array(a.origin),a.rotation(),np.array(b.origin),b.rotation(),float(jamb.joints["jamb"])))
        while len(samples)>1 and samples[1][1]<=t-STILL_S+1e-9: samples.pop(0)
    if samples[0][1]>t-STILL_S+1e-9: return False
    return all(np.linalg.norm(x[2]-y[2])<.0005 and _angle_deg(x[3],y[3])<1. and
               np.linalg.norm(x[4]-y[4])<.0005 and _angle_deg(x[5],y[5])<1. and abs(x[6]-y[6])<.00005
               for i,x in enumerate(samples) for y in samples[:i])


def score(objects):
    a,b,j = objects["shim_left"],objects["shim_right"],objects["jamb"]
    free = released(a) and released(b) and released(j.link_view("jamb"))
    settled_s = held_for("clear",free)
    calm = still(a,b,j)  # Sample on every call, including held and zero-score states.
    result = terms(objects)["tier"]
    return result if settled_s+1e-9>=2.0 and calm else min(result,.25)


def success(objects):
    return score(objects)>=1.0


# Fixtures only: never used by the demo.
def shim_origin(side,tip_y,flat_x,z=None,flip=False):
    sign = 1 if side=="left" else -1
    R = np.diag([sign,sign,1.])
    if flip: R = R@np.diag([-1.,1.,-1.])
    from yam_arena.ik import matrix_to_quat_xyzw
    return np.array([flat_x,tip_y+sign*L/2,TOP+Z_HINGE if z is None else z]), matrix_to_quat_xyzw(R)


def shim_state(spec,origin,quat,vel=(0.,0.,0.),spin=(0.,0.,0.)):
    from yam_arena.urdf import quat_xyzw_to_matrix
    o = np.asarray(origin,float)
    lo,hi = spec.aabb(o,quat_xyzw_to_matrix(np.asarray(quat)))
    return PartState((lo+hi)/2,np.asarray(quat),np.asarray(vel),np.asarray(spin),lo,hi,o)


def pair_layout(g,bow=.00015,centre=0.,loose=0.):
    actual = g+bow
    overlap = (actual-loose-2*T_TIP)/SLOPE
    a = shim_origin("left",centre-overlap/2,XS)
    b = shim_origin("right",centre+overlap/2,XS+actual)
    return {"shim_left":shim_state(SHIM_LEFT,*a),"shim_right":shim_state(SHIM_RIGHT,*b),
            "jamb":JAMB.offline_state(JAMB.pos,JAMB.yaw,{"setter":g-G0,"jamb":bow})}
# Scripted plan. Only read poses/joints; no runtime body or joint writes.
U_G, PRELOAD, N_STEP, BOW_STOP = .193,.00015,30,.00025
STEP_CAP = .035  # never command B's tip more than this past the depth at which the pair touches
# Above a gap of about 6.1 mm the shim held at the grip is thinner than the gap, so the far finger stands in the path
# of the other shim's tip, which meets it 11.3 mm from the hand's centre (URDF; ~12 mm on the GPU). Above WIDE_GAP the
# hands grip 3 mm nearer the butt (U_G_WIDE), and the drive stops before either tip comes within HAND_GUARD of the
# other hand. A grip at the very butt end (2.2 mm from it) let the shims roll, turn and tip in the hand (GPU).
U_G_WIDE, WIDE_GAP, HAND_GUARD = .196, .0060, .0135
ROLL_SQUARE = math.radians(.5)  # square a shim's roll in the hand (about its length) when it exceeds this
GRIP_PITCH = math.radians(-20)
PARK = {"left": (.25, .30, TOP+.18), "right": (.25, -.30, TOP+.18)}
GRIP_YAWS = {"left": -math.pi/2, "right": math.pi/2}
def grip_local(u):
    return np.array([thickness(u)/2,u-L/2,W/2-.0092])  # TCP 9.2 mm below the shim's top edge


GRIP_LOCAL = grip_local(U_G)


def grip_u(g):
    return U_G_WIDE if g>WIDE_GAP+1e-9 else U_G

def b_aim_x(g):
    """Where shim B's flat face is held while its tip enters: the tip centred in the free space at the right-hand mouth,
    between the jamb's back and shim A (A, already in to its depth, may reach that mouth), at least 1 mm off the jamb."""
    tip_a = -(g + PRELOAD - 2*T_TIP)/SLOPE/2
    t_a = thickness(-YJ - tip_a) if -YJ > tip_a else 0.
    return XS + g - max(.0010, (g - t_a - T_TIP)/2)

_DEMO = {}


def _objects_now(names):
    if _RUN["env"] is None: return None
    try:
        from yam_arena import scene
        return scene.object_states(_RUN["env"],names)
    except Exception as e:
        print(f"SHIM_READ failed: {e!r}",flush=True)
        return None


def _shim_now(arm):
    states = _objects_now(["shim_"+arm])
    return None if states is None else states["shim_"+arm]


def _jamb_now():
    states = _objects_now(["jamb"])
    return None if states is None else states["jamb"]


class Grip(float):
    """A grip yaw (rad) that also carries a pitch (rad) about the horizontal finger axis, through the TCP, and a roll
    (rad) about the wall-crossing y axis (the held shim's length), through the TCP."""

    def __new__(cls, yaw, pitch=0.0, roll=0.0):
        value = super().__new__(cls, yaw)
        value.grip_pitch = float(pitch)
        value.grip_roll = float(roll)
        return value

    def __repr__(self):
        return f"Grip({float(self)!r}, pitch={self.grip_pitch!r}, roll={self.grip_roll!r})"


def _install_grip_pitch():
    """Let a Grip tip the straight-down grip about its own finger axis; a plain float yaw is unchanged."""
    from yam_arena import demo as arena_demo
    from yam_arena.urdf import axis_angle_matrix

    original = arena_demo.grip_rotation
    if getattr(original, "_grip_pitch", False):
        return

    def grip_rotation(yaw=0.0):
        rotation = original(float(yaw))
        pitch = float(getattr(yaw, "grip_pitch", 0.0))
        roll = float(getattr(yaw, "grip_roll", 0.0))
        if pitch != 0.0:
            axis = np.array([-math.sin(float(yaw)), math.cos(float(yaw)), 0.0])  # the finger axis, horizontal
            rotation = axis_angle_matrix(axis, pitch) @ rotation
        if roll != 0.0:
            rotation = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), roll) @ rotation
        return rotation

    grip_rotation._grip_pitch = True
    grip_rotation.__wrapped__ = original
    arena_demo.grip_rotation = grip_rotation


_install_grip_pitch()


class LazyWaypoint(Waypoint):
    """Resolve each target once when first visited. Adaptive durations latch per step. The grip yaw adds the arm's
    squaring turn (the shim's measured yaw in the hand, taken out) as it stands when the target is resolved."""
    def __init__(self,duration,where,finger,label,keep=False,skip=None,yaw=math.pi/2,pitch=GRIP_PITCH,arm=None):
        self._duration = duration*(1. if keep else DEMO_PACE)
        self.where,self.finger,self.label,self._skip = where,finger,label,skip
        self._pos,self._skipped = None,False
        self._yaw,self._pitch,self._arm = yaw,pitch,arm
        self.grip_yaw = Grip(yaw,pitch)
    @property
    def pos(self):
        if self._pos is None:
            self._skipped = bool(self._skip and self._skip())
            self._pos = tuple(float(v) for v in self.where())
            turn = _DEMO.get(self._arm,{}).get("dyaw",0.) if self._arm else 0.
            roll = _DEMO.get(self._arm,{}).get("droll",0.) if self._arm and self._pitch else 0.
            self.grip_yaw = Grip(self._yaw+turn,self._pitch,roll)
        return self._pos
    @property
    def duration(self):
        return .05*DEMO_PACE if self._skipped else self._duration
    @property
    def yaw(self):
        _ = self.pos  # latch the target and its yaw together
        return self.grip_yaw


def demo():
    S = _DEMO
    S.clear()
    S.update(g=None,stop=False,command=None,tip_a=None,touch=None,a_cmd=None,a_cmd0=None)
    plans = {"left":[],"right":[]}
    for arm,sign,home in (("left",1,HOME_LEFT),("right",-1,HOME_RIGHT)):
        S[arm] = dict(v=-np.diag([sign,sign,1.])@GRIP_LOCAL,tcp=None,read=False,
                      home=home.copy(),sign=sign,origin=None,dyaw=0.,droll=0.)

    def read_gap():
        if S["g"] is None:
            j = _jamb_now()
            S["g"] = trial_gap() if j is None else gap_now(j)
        return S["g"]

    def target_overlap():
        return (read_gap()+PRELOAD-2*T_TIP)/SLOPE

    def start(arm):
        h = S[arm]
        if h.get("start") is None:
            shim = _shim_now(arm)
            h["start"] = h["home"] if shim is None else np.asarray(shim.origin)
            # The grip point along the shim is chosen for this trial's gap (see U_G_WIDE).
            h["v"] = -np.diag([h["sign"],h["sign"],1.])@grip_local(grip_u(read_gap()))
        return h["start"]-h["v"]

    def read_hand(arm):
        h = S[arm]
        if not h["read"]:
            nominal = h["v"].copy()
            shim = _shim_now(arm)
            if shim is not None:
                hands = grippers()
                tcp = np.asarray(hands[arm][0] if hands else h["tcp"])
                h["v"] = nominal+np.clip(np.asarray(shim.origin)-tcp-nominal,-.020,.020)
            h["read"] = True
        return h["v"]

    def lift(arm):
        read_hand(arm)
        return S[arm]["tcp"]+np.array([0,0,.045])

    def put(arm,tip_y,x,z=Z_HINGE):
        h = S[arm]
        o = np.array([x,tip_y+h["sign"]*L/2,TOP+z])
        h["origin"] = o
        return o-read_hand(arm)

    def square(arm):
        """Turn the hand about the vertical through its TCP by the shim's measured yaw in the grip, so the shim's
        length runs along y, and keep the shim's origin where it was planned (the in-hand offset turns with it)."""
        h = S[arm]
        shim = _shim_now(arm)
        hands = grippers()
        if shim is not None and hands:
            _,el,ew = shim_axes(shim)
            want = np.array([0.,h["sign"],0.])
            turn = float(np.clip(math.atan2(el[0]*want[1]-el[1]*want[0],el[0]*want[0]+el[1]*want[1]),
                                 -math.radians(6),math.radians(6)))
            c,s = math.cos(turn),math.sin(turn)
            v = np.asarray(shim.origin)-np.asarray(hands[arm][0])
            v = np.array([c*v[0]-s*v[1],s*v[0]+c*v[1],v[2]])
            # Roll about the shim's length (its width axis off vertical): a rolled shim cocks across the gap and bows
            # the jamb without filling it. Turn the wrist about y through the TCP to stand it upright.
            roll = math.atan2(ew[0],ew[2])
            fix = float(np.clip(-roll,-math.radians(4),math.radians(4))) if abs(roll)>ROLL_SQUARE else 0.
            c,s = math.cos(fix),math.sin(fix)
            h["v"] = np.array([c*v[0]+s*v[2],v[1],-s*v[0]+c*v[2]])
            h["dyaw"] += turn
            h["droll"] += fix
            h["read"] = True
            print(f"SHIM_SQUARE arm={arm} yaw_deg={-math.degrees(turn):.2f} roll_deg={math.degrees(roll):.2f} "
                  f"roll_fix_deg={math.degrees(fix):.2f} v_mm={np.round(h['v']*1000,2).tolist()}",flush=True)
        return h["origin"]-h["v"]

    def aim(arm,x):
        h = S[arm]
        shim = _shim_now(arm)
        correction = 0. if shim is None else float(np.clip(x-tip(shim)[0],-.008,.008))
        if shim is not None:
            print(f"SHIM_AIM arm={arm} tip_x_mm={tip(shim)[0]*1000:.2f} aim_mm={x*1000:.2f} "
                  f"correction_mm={correction*1000:.2f}",flush=True)
        # Preserve the observed lateral correction through entry and advance.
        h["v"] = h["v"]-np.array([correction,0,0])
        return h["tcp"]+np.array([correction,0,0])

    def trim(arm,x):
        """Once, after a press sideways: move the hand so the shim's flat face at its tip stands at x (the arm's
        steady error sideways is about 1 mm), keeping the correction for the moves that follow."""
        h = S[arm]
        shim = _shim_now(arm)
        if shim is None:
            return h["tcp"]
        correction = float(np.clip(x-tip(shim)[0],-.003,.003))
        h["v"] = h["v"]-np.array([correction,0,0])
        hands = grippers()
        if hands:
            h["offset"] = np.asarray(hands[arm][0])-h["tcp"]  # the arm's steady offset before any push
        print(f"SHIM_TRIM arm={arm} tip_x_mm={tip(shim)[0]*1000:.2f} want_mm={x*1000:.2f} "
              f"correction_mm={correction*1000:.2f}",flush=True)
        return h["tcp"]+np.array([correction,0,0])

    def ease(arm):
        """Take the push along the shim out (the hand runs ahead of the shim while it pushes). Sideways and up, the
        hand is held where it now actually is (less its steady offset from before the push): a pushing arm sags a few
        mm, and sent back to its old height it would lever the shims as it let go (GPU, 6.5 mm gap)."""
        shim = _shim_now(arm)
        if shim is None:
            return S[arm]["tcp"]
        held = np.asarray(shim.origin)-read_hand(arm)
        hands = grippers()
        now = S[arm]["tcp"] if not hands else np.asarray(hands[arm][0])-S[arm].get("offset",np.zeros(3))
        print(f"SHIM_EASE arm={arm} sag_mm={np.round((now-S[arm]['tcp'])*1000,2).tolist()} "
              f"push_mm={(S[arm]['tcp'][1]-held[1])*1000:.2f}",flush=True)
        return np.array([now[0],held[1],now[2]])

    def add(arm,dt,where,label,finger=FINGER_CLOSED,keep=False,skip=None,pitch=GRIP_PITCH):
        def target():
            p = np.asarray(where(),float)
            S[arm]["tcp"] = p
            return p
        plans[arm].append(LazyWaypoint(dt,target,finger,label,keep,skip,yaw=GRIP_YAWS[arm],pitch=pitch,arm=arm))

    for arm in plans:
        add(arm,1.2,lambda a=arm:start(a)+[0,0,.060],"over grip",FINGER_OPEN)
        add(arm,1.2,lambda a=arm:start(a),"down to grip",FINGER_OPEN)
        add(arm,.8,lambda a=arm:S[a]["tcp"],"close",keep=True)
        add(arm,1.2,lambda a=arm:lift(a),"lift")
    a="left"
    add(a,2.,lambda:put(a,YJ+.015,XS+(read_gap()-T_TIP)/2,Z_HINGE+.030),"carry")
    # From here on, every move at or in the wall keeps its duration at any pace; only the pick, lift, carries,
    # back-offs and park scale (at 0.8x, faster squaring and slides swung the shims' tips 3-4 mm off line: GPU).
    add(a,1.,lambda:put(a,YJ+.015,XS+(read_gap()-T_TIP)/2),"lower",keep=True)
    add(a,.8,lambda:square(a),"square",keep=True)
    add(a,.6,lambda:S[a]["tcp"],"settle",keep=True)
    add(a,.6,lambda:aim(a,XS+(read_gap()-T_TIP)/2),"aim",keep=True)
    # Moving fast along y, the arm swings the tip up to 3-5 mm sideways (GPU): close in slowly and aim again 3 mm out.
    # The moves near the mouths keep their durations at any pace (at 0.8x, B's faster slide swung its tip 3.6 mm).
    add(a,1.5,lambda:put(a,YJ+.003,XS+(read_gap()-T_TIP)/2),"approach",keep=True)
    add(a,.4,lambda:S[a]["tcp"],"settle at mouth",keep=True)
    add(a,.5,lambda:aim(a,XS+(read_gap()-T_TIP)/2),"aim at mouth",keep=True)
    add(a,1.5,lambda:put(a,YJ-.005,XS+(read_gap()-T_TIP)/2),"enter",keep=True)
    add(a,.6,lambda:put(a,YJ-.005,XS-.0003),"press onto stud",keep=True)
    add(a,.4,lambda:S[a]["tcp"],"settle on stud",keep=True)
    add(a,.5,lambda:trim(a,XS-.0003),"trim onto stud",keep=True)
    add(a,6.,lambda:put(a,-target_overlap()/2,XS-.0003),"advance A",keep=True)
    left_end=sum(w.duration for w in plans[a])
    b="right"
    stage=lambda:-(max(target_overlap()/2,YJ)+.020)
    add(b,2.,lambda:put(b,stage(),b_aim_x(read_gap()),Z_HINGE+.030),"carry staging")
    add(b,1.,lambda:put(b,stage(),b_aim_x(read_gap())),"lower",keep=True)
    add(b,.8,lambda:square(b),"square",keep=True)
    wait=left_end+.5-sum(w.duration for w in plans[b])
    add(b,wait,lambda:S[b]["tcp"],"wait for A",keep=True)
    # Square and aim again here, past A's tip (at large gaps A's tip stands out of this face), then slide in slowly.
    add(b,.8,lambda:square(b),"square again",keep=True)
    add(b,.4,lambda:S[b]["tcp"],"settle at staging",keep=True)
    add(b,.5,lambda:aim(b,b_aim_x(read_gap())),"aim at staging",keep=True)
    add(b,2.5,lambda:put(b,-(YJ+.015),b_aim_x(read_gap())),"slide to mouth",keep=True)
    add(b,.6,lambda:S[b]["tcp"],"settle",keep=True)
    add(b,.6,lambda:aim(b,b_aim_x(read_gap())),"settle aim",keep=True)
    add(b,1.5,lambda:put(b,-(YJ+.003),b_aim_x(read_gap())),"approach",keep=True)
    add(b,.4,lambda:S[b]["tcp"],"settle at mouth",keep=True)
    add(b,.5,lambda:aim(b,b_aim_x(read_gap())),"aim at mouth",keep=True)
    add(b,1.5,lambda:put(b,-(YJ-.005),b_aim_x(read_gap())),"enter",keep=True)

    def current_gap():
        j=_jamb_now()
        return read_gap() if j is None else gap_now(j)

    add(b,.6,lambda:put(b,-(YJ-.005),XS+current_gap()),"press onto jamb",keep=True)
    add(b,.4,lambda:S[b]["tcp"],"settle on jamb",keep=True)
    add(b,.5,lambda:trim(b,XS+current_gap()),"trim onto jamb",keep=True)

    def fast():
        shim=_shim_now(a)
        S["tip_a"]=-target_overlap()/2 if shim is None else float(tip(shim)[1])
        S["touch"]=(current_gap()-2*T_TIP)/SLOPE
        S["command"]=S["tip_a"]+S["touch"]-.015
        S["press_x"]=XS+current_gap()
        return put(b,S["command"],S["press_x"])
    add(b,2.,fast,"fast advance B",keep=True)

    def near_hand():
        """How close each shim's tip has come to the other hand's centre along the wall (A's tip to the right hand,
        B's tip to the left hand), from the simulator's state; None offline."""
        objects=_objects_now(["shim_left","shim_right"])
        hands=grippers()
        if objects is None or not hands: return None
        return min(float(tip(objects["shim_left"])[1]-hands["right"][0][1]),
                   float(hands["left"][0][1]-tip(objects["shim_right"])[1]))

    def stop():
        j=_jamb_now()
        if not S["stop"] and j is not None and j.joints["jamb"]>=BOW_STOP:
            # Only a bow the wedge itself makes counts: the pack's nominal thickness must fill the bowed gap. Shims
            # cocked in the hands also bow the jamb, and that bow vanishes when they are let go (GPU, 6.5 mm gap).
            objects=_objects_now(["shim_left","shim_right"])
            free=None if objects is None else clear0(objects["shim_left"],objects["shim_right"],j)
            if free is None or free<=.0001:
                S["stop"]=True
            else:
                print(f"SHIM_COCKED q_jamb_mm={j.joints['jamb']*1000:.3f} free_mm={free*1000:.3f}",flush=True)
        if not S["stop"]:
            d=near_hand()
            if d is not None and d<HAND_GUARD:
                S["stop"]=True
                q=math.nan if j is None else j.joints["jamb"]
                print(f"SHIM_GUARD tip_to_hand_mm={d*1000:.2f} q_jamb_mm={q*1000:.3f}",flush=True)
        return S["stop"]

    def step(k):
        j=_jamb_now()
        if not S["stop"]:
            S["command"]=min(S["command"]+.002,S["tip_a"]+S["touch"]+STEP_CAP)
        shim=_shim_now(b)
        actual=math.nan if shim is None else tip(shim)[1]
        q=math.nan if j is None else j.joints["jamb"]
        print(f"SHIM_STEP step={k} command_mm={S['command']*1000:.3f} tip_mm={actual*1000:.3f} "
              f"q_jamb_mm={q*1000:.3f} skipped={S['stop']}",flush=True)
        return S[b]["tcp"] if S["stop"] else put(b,S["command"],S["press_x"])
    def step_a(k):
        """While B steps in, A's hand also leans in, 1 mm a step (at most 20 mm past A's planned depth): B's push drags
        A back out through the friction between the shims, and a hand that only holds yields (GPU, 6.5 mm gap)."""
        if S.get("a_cmd") is None:
            S["a_cmd0"]=S["a_cmd"]=-target_overlap()/2
        if S["stop"]:
            return S[a]["tcp"]
        S["a_cmd"]=max(S["a_cmd"]-.001,S["a_cmd0"]-.020)
        return put(a,S["a_cmd"],XS-.0003)
    steps_start=sum(w.duration for w in plans[b])
    add(a,steps_start-sum(w.duration for w in plans[a]),lambda:S[a]["tcp"],"wait for B",keep=True)
    for k in range(N_STEP):
        add(b,.45,lambda k=k:step(k),"step B",keep=True,skip=stop)
        add(a,.45,lambda k=k:step_a(k),"step A",keep=True,skip=stop)
    # Both hands take out their pushes together (A's lean-in stores a push too: eased alone, either hand shoves the
    # pack), then B lets go and backs off while A holds still, then A lets go.
    add(b,.6,lambda:ease(b),"ease",keep=True)
    add(a,.6,lambda:ease(a),"ease",keep=True)
    add(b,.8,lambda:S[b]["tcp"],"open",FINGER_OPEN,keep=True)
    add(b,1.,lambda:S[b]["tcp"]+[0,-.050,.040],"back off",FINGER_OPEN)
    right_back_end=sum(w.duration for w in plans[b])
    add(a,right_back_end+.3-sum(w.duration for w in plans[a]),lambda:S[a]["tcp"],"hold for B release",keep=True)
    add(a,.8,lambda:S[a]["tcp"],"open",FINGER_OPEN,keep=True)
    add(a,1.,lambda:S[a]["tcp"]+[0,.050,.040],"back off",FINGER_OPEN)
    for arm,sign in ((a,1),(b,-1)):
        add(arm,1.4,lambda a=arm:np.array(PARK[a]),"park",FINGER_OPEN,pitch=0.)
        add(arm,4.,lambda a=arm:S[a]["tcp"],"stillness tail",FINGER_OPEN,keep=True,pitch=0.)
    return plans


import shim_runtime
shim_runtime.install(sys.modules.get(__name__) or globals())
