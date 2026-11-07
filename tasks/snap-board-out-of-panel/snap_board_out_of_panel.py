"""Passive V-score depaneling; all episode interaction is robot contact."""
import math
import pathlib
import sys
from types import SimpleNamespace
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import yam_parts as parts
from yam_parts import Part, PartState, box, cyl
from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED, PAD_TIP_PAST_TCP
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
SUCCESS_HOLD_S, SCORE_OVER = 1.0, "end"
RUN_SECONDS = 90.
X, START_SCORE_Y, SLIDE = .205, .025, .025
THICK, BOARD_W, BOARD_D, FRAME_D = .0016, .080, .090, .146
RAIL_Y, RAIL_D = .130, .032
SUPPORT_FRICTION = (.80, .65)
SLIDE_OVERSTROKE = .005
BLOCK_H, BLOCK_X, BLOCK_Y = .040, .130, .170
BLOCK_TOP = TOP+BLOCK_H
SCORE_Z = BLOCK_TOP+THICK/2+.001
FRAME_MASS, BOARD_MASS = .040, .030
BREAK_TORQUE, BREAK_FORCE = .480, 100.
BEND_LEVER, BEND_OVERTRAVEL, HOLD_OVERTRAVEL = .0707, .038, .024
VERTICAL_K, LIGHT_LAG = 290., .0014
HOLD_Y, PRESS_Y = .130, -.084
LAND_FLOOR, LAND_Y = TOP+.002, -.048
LAND_Z = LAND_FLOOR+THICK/2
TRAY_X, TRAY_Y, TRAY_W, TRAY_D = .205, -.215, .132, .136
TRAY_FLOOR, TRAY_WALL = TOP+.004, .012
TRAY_Z = TRAY_FLOOR+THICK/2
TERMINAL_Y, TERMINAL_H, TERMINAL_W, TERMINAL_D = -.005, .015, .032, .020
PICK_TCP_Z = LAND_Z+THICK/2+.016
TRAVEL_Z, PARK_Z = TOP+.135, TOP+.160
PICK_CLEAR_RISE, PICK_AWAY = .010, .018
EDGE_TOL, PULL_TOL, DROP_TOL = .002, .004, .045
BREAK_SHIFT, BREAK_ANGLE = .001, 2.
TWIST_DEG, WINDOW_S = 12., .050
# Attached samples, the last one included: only a visible wrong-sense deflection (1 deg =
# 1.6 mm at the 90 mm board's tip) is sticky; the 1 mm shift latch caps attached rotation
# near 1.27 deg. Loading direction is judged from the visible post-break window.
PRE_UP_MAX, PRE_TWIST_MAX = 1., 1.
EDGE_X0, EDGE_X1 = X-BLOCK_X/2, X+BLOCK_X/2
TABLE_X, TABLE_Y = (-.19, .91), (-.65, .65)
FALL_DEPTH = .050
DROP_MIN = .002
UP_TOL, FRAME_TILT, TRAY_TILT = .001, 5., 15.
PEN_TOL, FLOOR_TOL, STAGED_H = .002, .004, .030
FRAME_COM_MARGIN = .020
LINEAR_SPEED, ANGULAR_SPEED, CONTROL_DT = .010, .15, .050
TEST_HISTORY = None

GREEN, BLUE, STEEL, BLACK, GOLD = (.08,.40,.17), (.05,.36,.80), (.46,.50,.55), (.045,.06,.07), (.90,.67,.12)


class PanelPart(Part):
    @property
    def free(self):
        # Both bodies are dynamic at runtime, but must never jitter independently.
        return False


FRAME = PanelPart("frame", (X,START_SCORE_Y,SCORE_Z),
    [box((0,FRAME_D/2,0),(BOARD_W,FRAME_D,THICK)),
     box((0,RAIL_Y,.0088),(.080,RAIL_D,.016),color=STEEL),
     cyl((-.028,.023,.00095),.003,.0003,color=GOLD),
     cyl((.028,.023,.00095),.003,.0003,color=GOLD),
     cyl((-.028,.080,.00095),.002,.0003,color=BLACK),
     cyl((.028,.080,.00095),.002,.0003,color=BLACK)],
    FRAME_MASS, GREEN, friction=SUPPORT_FRICTION, contact_offset=.0002)
BOARD = PanelPart("board", (X,START_SCORE_Y-BOARD_D/2,SCORE_Z),
    [box((0,0,0),(BOARD_W,BOARD_D,THICK)),
     box((0,TERMINAL_Y,THICK/2+TERMINAL_H/2),(TERMINAL_W,TERMINAL_D,TERMINAL_H),color=BLUE),
     box((-.027,.025,.0048),(.012,.014,.008),color=BLACK),
     box((.027,.025,.0038),(.012,.014,.006),color=BLACK),
     box((0,.044,.00095),(.076,.001,.0003),color=GOLD)],
    BOARD_MASS,GREEN,friction=(.6,.45),contact_offset=.0002,grip=1)
BLOCK = Part("break_block",(X,BLOCK_Y/2,TOP),
    [box((0,0,BLOCK_H/2),(BLOCK_X,BLOCK_Y,BLOCK_H))],color=STEEL,friction=SUPPORT_FRICTION,contact_offset=.0002)
# Passive shallow ESD landing nest: 86 x 96 mm inside, block face is the fourth wall.
LANDING = Part("landing_nest",(X,LAND_Y,TOP),
    [box((0,0,.001),(.092,.104,.002)),
     box((-.045,0,.003),(.004,.104,.002)),
     box((.045,0,.003),(.004,.104,.002)),
     box((0,-.050,.003),(.086,.004,.002))],color=BLACK,friction=(.6,.45),contact_offset=.0002)
TRAY = Part("esd_tray",(TRAY_X,TRAY_Y,TOP),
    [box((0,0,.002),(TRAY_W+.008,TRAY_D+.008,.004)),
     box((-(TRAY_W+.004)/2,0,.010),(.004,TRAY_D+.008,TRAY_WALL)),
     box(((TRAY_W+.004)/2,0,.010),(.004,TRAY_D+.008,TRAY_WALL)),
     box((0,-(TRAY_D+.004)/2,.010),(TRAY_W,.004,TRAY_WALL)),
     box((0,(TRAY_D+.004)/2,.010),(TRAY_W,.004,TRAY_WALL))],color=(.20,.23,.30),friction=(.6,.45),contact_offset=.0002)
# Closed pads intentionally push and press these panel parts.
PRESSED_PARTS = ("frame", "board")
OBJECTS = [BLOCK,LANDING,TRAY,FRAME,BOARD]

# Spawn-only adapter, using only the unchanged compound-parts kit.
_PANEL_PATHS = {}
_JOINT_PATH = None
_PANEL_CFG = None
_PANEL_SPAWN = None
_PANEL_TOKEN = object()


def _joint(stage, paths, frame_pose=None, board_pose=None):
    from pxr import Gf, UsdPhysics
    path = paths["frame"]+"/v_score_joint"
    joint = UsdPhysics.FixedJoint.Define(stage,path)
    joint.CreateJointEnabledAttr().Set(False)
    joint.CreateBody0Rel().SetTargets([paths["frame"]])
    joint.CreateBody1Rel().SetTargets([paths["board"]])
    a0 = np.zeros(3)
    a1 = np.array([0.,BOARD_D/2,0.])
    q1 = np.array([0.,0.,0.,1.])
    if frame_pose is not None:
        f,b = frame_pose,board_pose
        anchor = f.to_world(a0)
        a1 = b.to_local(anchor)
        from yam_arena import ik
        q1 = ik.matrix_to_quat_xyzw(b.rotation().T@f.rotation())
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*map(float,a0)))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*map(float,a1)))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(float(q1[3]),Gf.Vec3f(*map(float,q1[:3]))))
    joint.CreateCollisionEnabledAttr().Set(False)
    joint.CreateExcludeFromArticulationAttr().Set(True)
    joint.CreateBreakForceAttr().Set(BREAK_FORCE)
    joint.CreateBreakTorqueAttr().Set(BREAK_TORQUE)
    # Last write, after both reset poses and all joint frames/limits are restored.
    joint.CreateJointEnabledAttr().Set(True)
    return path


def _spawn_panel(prim_path,cfg,*args,**kwargs):
    import isaaclab.sim as sim
    from pxr import PhysxSchema
    spec = FRAME if cfg.panel_name=="frame" else BOARD
    # A plain Part has free=True, so the kit authors a real dynamic rigid body.
    proxy = Part(spec.name,spec.pos,spec.shapes,spec.mass,spec.color,
                 friction=spec.friction,contact_offset=spec.contact_offset,grip=spec.grip)
    parts._SPECS[str(prim_path)] = proxy
    try:
        body = parts._spawn_part(prim_path,SimpleNamespace(part_name=str(prim_path)),*args,**kwargs)
    finally:
        parts._SPECS.pop(str(prim_path),None)
    rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(body)
    rigid.CreateSolverPositionIterationCountAttr().Set(32)
    rigid.CreateLinearDampingAttr().Set(.05)
    rigid.CreateAngularDampingAttr().Set(.05)
    _PANEL_PATHS[spec.name] = str(prim_path)
    global _JOINT_PATH
    if all(n in _PANEL_PATHS for n in ("frame","board")):
        _JOINT_PATH = _joint(kwargs.get("stage") or sim.get_current_stage(),_PANEL_PATHS)
    return body


def _panel_object(spec):
    global _PANEL_CFG,_PANEL_SPAWN
    from isaaclab.sim.utils import clone
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    if _PANEL_CFG is None:
        from panel_config import panel_config
        _PANEL_CFG,_PANEL_SPAWN = panel_config(),clone(_spawn_panel)
    _PANEL_PATHS.pop(spec.name,None)
    return Object(name=spec.name,object_type=ObjectType.RIGID,
                  spawner_cfg=_PANEL_CFG(func=_PANEL_SPAWN,panel_name=spec.name),
                  initial_pose=Pose(position_xyz=spec.pos,rotation_xyzw=(0,0,0,1)))


def _array(value):
    value = getattr(value,"torch",value)
    return value.detach().cpu().numpy().astype(float)


def _panel_states(env):
    origin = _array(env.unwrapped.scene.env_origins)[0]
    result = {}
    for spec in (FRAME,BOARD):
        data = env.unwrapped.scene.rigid_objects[spec.name].data
        pose = _array(data.root_pose_w)[0]
        p,q = pose[:3]-origin,pose[3:]
        lv,av = _array(data.root_lin_vel_w)[0],_array(data.root_ang_vel_w)[0]
        state = PartState(p,q,lv,av,p,p,p)
        lo,hi = spec.aabb(p,state.rotation())
        result[spec.name] = PartState((lo+hi)/2,q,lv,av,lo,hi,p)
    return result


def _patch_panel_scene(scene):
    import functools
    parts._patch_scene(scene)
    if getattr(scene,"_panel_token",None) is _PANEL_TOKEN:
        return
    old_object,old_states = scene.task_object,scene.object_states
    @functools.wraps(old_object)
    def task_object(spec):
        return _panel_object(spec) if isinstance(spec,PanelPart) else old_object(spec)
    @functools.wraps(old_states)
    def object_states(env,names):
        names = tuple(names)
        out = old_states(env,tuple(n for n in names if n not in ("frame","board")))
        if any(n in ("frame","board") for n in names):
            out.update({n:s for n,s in _panel_states(env).items() if n in names})
        return out
    scene.task_object,scene.object_states = task_object,object_states
    scene._panel_token = _PANEL_TOKEN


def _rearm_after_reset(env):
    import isaaclab.sim as sim
    stage = sim.get_current_stage()
    poses = _panel_states(env)
    # Keep the existing prim. _joint authors disabled, restores local frames from
    # the reset bodies, then enables last. PhysX reactivation needs GPU evidence;
    # authored jointEnabled is intent, not the solver's live broken-state flag.
    _joint(stage,_PANEL_PATHS,poses["frame"],poses["board"])


def _reset_panel_bodies(env,env_reset,*args,**kwargs):
    if _JOINT_PATH is not None:
        import isaaclab.sim as sim
        from pxr import UsdPhysics
        j = UsdPhysics.FixedJoint.Get(sim.get_current_stage(),_JOINT_PATH)
        j.CreateJointEnabledAttr().Set(False)
    result = env_reset(*args,**kwargs)
    _rearm_after_reset(env)
    return result


def _install_panel_hooks():
    import importlib.machinery
    class Finder(importlib.machinery.PathFinder):
        def find_spec(self,fullname,path=None,target=None):
            if fullname!="yam_arena.scene":return None
            spec = importlib.machinery.PathFinder.find_spec(fullname,path)
            if spec is not None and spec.loader is not None:
                spec.loader = parts._SceneLoader(spec.loader,_patch_panel_scene)
            return spec
    sys.meta_path.insert(0,Finder())
    if "yam_arena.scene" in sys.modules:
        _patch_panel_scene(sys.modules["yam_arena.scene"])
    try:
        from yam_arena.inspect_embodiment import YamArenaEmbodiment
    except ImportError:
        return
    cls = YamArenaEmbodiment
    original = cls.reset  # Includes the verbatim release/stillness wrapper above.
    def reset(self,*args,**kwargs):
        if self.task is None or not any(isinstance(o,PanelPart) for o in self.task.objects):
            return original(self,*args,**kwargs)
        self._ensure_sim(None)
        env = self._env
        # Arena returns gym.make(): hook the actual env, preserving Gym's outer
        # reset bookkeeping. _ensure_sim(None) is idempotent for a task env.
        base = env.unwrapped
        env_reset = base.reset
        saved_reset = vars(base).get("reset")
        had_reset = "reset" in vars(base)
        def reset_bodies_then_joint(*a,**kw):
            # Disable before body restoration; re-enable AFTER restoration and
            # BEFORE the harness's 0.5 s settling steps or first observation.
            return _reset_panel_bodies(env,env_reset,*a,**kw)
        base.reset = reset_bodies_then_joint
        try:
            return original(self,*args,**kwargs)
        finally:
            if had_reset:base.reset = saved_reset
            else:del base.reset
    cls.reset = reset


_install_panel_hooks()


def _angle(R):
    return math.degrees(math.acos(float(np.clip((np.trace(R)-1)/2,-1,1))))


def relative_terms(frame,board):
    R = frame.rotation().T@board.rotation()
    delta = frame.to_local(board.origin)-np.array([0.,-BOARD_D/2,0.])
    rx = math.degrees(math.atan2(R[2,1],R[2,2]))
    ry = math.degrees(math.asin(float(np.clip(-R[2,0],-1,1))))
    rz = math.degrees(math.atan2(R[1,0],R[0,0]))
    a = frame.to_local(board.to_world((0,BOARD_D/2,0)))
    ends = [frame.to_world((s*BOARD_W/2,0,0)) for s in (-1,1)]
    return dict(shift=float(np.linalg.norm(delta)),angle=_angle(R),bend=rx,twist=max(abs(ry),abs(rz)),
                pull=float(np.linalg.norm(a[:2])),drop=float(-a[2]),
                edge=max(abs(p[1]) for p in ends),
                edge_x=max(max(EDGE_X0-p[0],p[0]-EDGE_X1,0.) for p in ends),
                edge_z=max(abs(p[2]-(BLOCK_TOP+THICK/2)) for p in ends),
                centre_up=float(delta[2]),frame_tilt=frame.tilt_deg())


def _fresh_break():
    return dict(latch=None,valid=None,bad=False,max_bend=0.,seen_joined=False,last=None,
                last_joined=None,previous_z=None,downward=False,pre_twist=False,pre_up=False,
                max_drop=0.)


def _advance_break(memory,step,t,frame,board):
    _advance_break_terms(memory,step,t,relative_terms(frame,board),float(board.origin[2]))


def _advance_break_terms(memory,step,t,terms,centre_z,*,step_stride=1):
    """Shared pose-derived validity logic; replay may use recorded 10 Hz cadence.

    Runtime always uses stride 1 (20 Hz). Stride 2 replays only recorded
    observations, without inventing the unlogged intermediate control poses.
    """
    token = (step,float(t))
    if token==memory["last"]:return
    separated = terms["shift"]>BREAK_SHIFT or terms["angle"]>BREAK_ANGLE
    previous = memory["last"]
    contiguous = (previous is not None and step==previous[0]+step_stride
                  and 0<t-previous[1]<=CONTROL_DT*step_stride+1e-6)
    if not separated and memory["latch"] is None:
        memory["seen_joined"] = True
        memory["last_joined"] = terms.copy()
        # Significant wrong-sense loading is sticky even if later corrected.
        memory["pre_twist"] |= terms["twist"]>PRE_TWIST_MAX
        memory["pre_up"] |= terms["bend"] < -PRE_UP_MAX
    if memory["latch"] is None and separated:
        memory["latch"] = float(t)
        joined = memory["last_joined"]
        memory["bad"] = bool(not memory["seen_joined"] or not contiguous or memory["pre_twist"]
                             or memory["pre_up"] or joined is None
                             or joined["edge"]>EDGE_TOL or joined["edge_z"]>EDGE_TOL
                             or joined["edge_x"]>0.)
    if memory["latch"] is not None and memory["valid"] is None:
        memory["max_bend"] = max(memory["max_bend"],terms["bend"])
        memory["max_drop"] = max(memory["max_drop"],terms["drop"])
        memory["downward"] |= (contiguous and memory["previous_z"] is not None
                                and centre_z<memory["previous_z"])
        acceptable = (contiguous and terms["edge_z"]<=EDGE_TOL and terms["edge_x"]<=0.
                      and terms["twist"]<=TWIST_DEG
                      and terms["centre_up"]<=UP_TOL and terms["pull"]<=PULL_TOL
                      and -UP_TOL<=terms["drop"]<=DROP_TOL and terms["frame_tilt"]<=FRAME_TILT)
        memory["bad"] |= not acceptable
        if memory["bad"]:
            memory["valid"] = False
        elif t-memory["latch"]+1e-10>=WINDOW_S:
            memory["valid"] = memory["max_drop"]>=DROP_MIN and bool(memory["downward"])
            memory["bad"] = not memory["valid"]
    memory["last"] = token
    memory["previous_z"] = float(centre_z)


_BREAK_EPISODE = None
_BREAK_MEMORY = _fresh_break()


def _state(spec,p,q=(0,0,0,1)):
    p,q = np.asarray(p,float),np.asarray(q,float)
    s = PartState(p,q,np.zeros(3),np.zeros(3),p,p,p)
    lo,hi = spec.aabb(p,s.rotation())
    return PartState((lo+hi)/2,q,np.zeros(3),np.zeros(3),lo,hi,p)


def history_sample(step,t,frame,board):
    return (step,t,tuple(frame.origin),tuple(frame.quat_xyzw),tuple(board.origin),tuple(board.quat_xyzw))


def bend_states(degrees=0.,score_y=0.,pull=(0,0,0)):
    a = math.radians(degrees)
    q = (math.sin(a/2),0,0,math.cos(a/2))
    f = _state(FRAME,(X,score_y,BLOCK_TOP+THICK/2))
    temp = _state(BOARD,(0,0,0),q)
    p = f.origin+temp.rotation()@np.array([0.,-BOARD_D/2,0.])+np.asarray(pull)
    return f,_state(BOARD,p,q)


# Offline fixtures contain only measured poses, not a precomputed validity flag.
def fixture_history(kind="valid"):
    rows = []
    for i,a in enumerate((.3,8.,18.)):
        edge = (.005 if kind=="off edge" or (kind=="late registration" and not i)
                else -.0028 if kind=="2.8 mm registration" else -.007 if kind=="recoil" and i else 0.)
        f,b = bend_states(-a if kind=="up" else a,edge,
                          (.008,0,-.003*i) if i and kind=="pull" else (0,0,-.003*i))
        if i and kind in ("twist","bend and twist","twist cap"):
            from yam_arena import ik
            bend = 1. if kind=="twist" else 6. if kind=="bend and twist" else 8.
            twist = 13. if kind=="twist cap" else 6.
            from yam_arena.urdf import rpy_matrix
            R = rpy_matrix((math.radians(bend),math.radians(twist),0.))
            p = f.origin+R@np.array([0.,-BOARD_D/2,0.])+np.array([0,0,-.003*i])
            b = _state(BOARD,p,ik.matrix_to_quat_xyzw(R))
        if not i and kind in ("pre twist","twist","no pre bend","noise only"):
            from yam_arena import ik
            from yam_arena.urdf import rpy_matrix
            bend = .3 if kind=="pre twist" else .02 if kind=="twist" else .005 if kind=="noise only" else 0.
            twist = .3 if kind=="pre twist" else .2 if kind=="twist" else 0.
            R = rpy_matrix((math.radians(bend),math.radians(twist),0.))
            b = _state(BOARD,f.origin+R@np.array([0.,-BOARD_D/2,0.]),ik.matrix_to_quat_xyzw(R))
        if i and kind in ("flat drop","no anchor drop","upward departure"):
            f,b = bend_states(0.,edge,(0,0,-.008*i if kind!="no anchor drop" else -.0015))
            if kind=="upward departure" and i==2:
                f,b = bend_states(0.,edge,(0,0,-.007))
        if i and kind=="no downward motion":
            f,b = bend_states(2.1)
            lift = np.array([0.,0.,BLOCK_TOP+THICK/2-b.origin[2]])
            f,b = _state(FRAME,f.origin+lift),_state(BOARD,b.origin+lift,b.quat_xyzw)
        rows.append(history_sample(i,i*CONTROL_DT,f,b))
    return rows


VALID_HISTORY = fixture_history()


def break_memory(objects):
    global _BREAK_EPISODE,_BREAK_MEMORY
    if _RUN["t"] is None:
        memory = _fresh_break()
        for row in TEST_HISTORY or ():
            step,t,fp,fq,bp,bq = row
            _advance_break(memory,step,t,_state(FRAME,fp,fq),_state(BOARD,bp,bq))
        if TEST_HISTORY:
            _advance_break(memory,step+1,t+CONTROL_DT,objects["frame"],objects["board"])
        return memory
    if _BREAK_EPISODE!=_RUN["episode"]:
        _BREAK_MEMORY = _fresh_break()
        _BREAK_EPISODE = _RUN["episode"]
    _advance_break(_BREAK_MEMORY,_RUN["step"],_RUN["t"],objects["frame"],objects["board"])
    return _BREAK_MEMORY


_POSE_EPISODE,_POSE_TOKEN,_PREVIOUS_POSES,_QUIET = None,None,None,False


def at_rest(objects):
    global _POSE_EPISODE,_POSE_TOKEN,_PREVIOUS_POSES,_QUIET
    current = [objects[n] for n in ("frame","board")]
    if _RUN["t"] is None:
        return all(np.linalg.norm(s.lin_vel)<LINEAR_SPEED and np.linalg.norm(s.ang_vel)<ANGULAR_SPEED for s in current)
    if _POSE_EPISODE!=_RUN["episode"]:
        _PREVIOUS_POSES,_POSE_TOKEN,_QUIET = None,None,False
        _POSE_EPISODE = _RUN["episode"]
    token = (_RUN["step"],float(_RUN["t"]))
    if token!=_POSE_TOKEN:
        _QUIET = False
        if _PREVIOUS_POSES is not None:
            last,old = _PREVIOUS_POSES
            dt = token[1]-last[1]
            _QUIET = (token[0]==last[0]+1 and 0<dt<=CONTROL_DT+1e-6 and
                      all(np.linalg.norm(s.origin-p)<LINEAR_SPEED*dt and
                          math.radians(_angle(R.T@s.rotation()))<ANGULAR_SPEED*dt
                          for s,(p,R) in zip(current,old)))
        _PREVIOUS_POSES = (token,[(s.origin.copy(),s.rotation().copy()) for s in current])
        _POSE_TOKEN = token
    return bool(_QUIET)


def _collider_centroid(spec):
    # Matches the kit's uniform-density compound-collider mass approximation.
    volumes = [np.prod(s.size) if hasattr(s,"size") else math.pi*s.radius**2*s.height for s in spec.shapes]
    return np.average([s.center for s in spec.shapes],axis=0,weights=volumes)


FRAME_COM_LOCAL = _collider_centroid(FRAME)


def _supported_underside(corners):
    """Clip the substrate underside to the block top's XY rectangle."""
    polygon = list(corners)
    for axis,bound,sign in ((0,X-BLOCK_X/2,1),(0,X+BLOCK_X/2,-1),(1,0.,1),(1,BLOCK_Y,-1)):
        clipped = []
        for a,b in zip(polygon,polygon[1:]+polygon[:1]):
            da,db = sign*(a[axis]-bound),sign*(b[axis]-bound)
            if da>=0:clipped.append(a)
            if (da>=0)!=(db>=0):clipped.append(a+(b-a)*da/(da-db))
        polygon = clipped
    return polygon


TRAY_OUTLINE_HX, TRAY_OUTLINE_HY = TRAY_W/2+.004, TRAY_D/2+.004


def _overlaps_tray(corners):
    """Separating-axis test: frame substrate rectangle (yaw-aware) vs the tray's outer outline."""
    quad = np.asarray(corners)[:,:2]
    outline = np.array([(TRAY_X+sx*TRAY_OUTLINE_HX,TRAY_Y+sy*TRAY_OUTLINE_HY)
                        for sx,sy in ((-1,-1),(1,-1),(1,1),(-1,1))])
    axes = [np.array([1.,0.]),np.array([0.,1.]),quad[1]-quad[0],quad[3]-quad[0]]
    for axis in axes:
        n = float(np.linalg.norm(axis))
        if n<1e-12:continue
        a,b = quad@(axis/n),outline@(axis/n)
        if a.max()<=b.min() or b.max()<=a.min():
            return False
    return True


def placement_terms(objects):
    f,b = objects["frame"],objects["board"]
    corners = np.array([f.to_world((x,y,-THICK/2)) for x,y in
                        ((-BOARD_W/2,0),(BOARD_W/2,0),(BOARD_W/2,FRAME_D),(-BOARD_W/2,FRAME_D))])
    supported = _supported_underside(corners)
    com = f.to_world(FRAME_COM_LOCAL)
    com_margin = min(com[0]-(X-BLOCK_X/2),X+BLOCK_X/2-com[0],com[1],BLOCK_Y-com[1])
    # Signed support band: supported underside -PEN_TOL to +FLOOR_TOL from the block top.
    heights = [float(p[2]-BLOCK_TOP) for p in supported] if len(supported)>=3 else []
    support_gap = max(heights) if heights else math.inf
    supported_ok = bool(heights) and min(heights)>=-PEN_TOL and max(heights)<=FLOOR_TOL
    frame_on = com_margin>=FRAME_COM_MARGIN and supported_ok and f.tilt_deg()<=FRAME_TILT
    # Scrap frame may instead lie flat on the bench top, wholly on the table, clear of the tray outline.
    over_tray = _overlaps_tray(corners)
    on_bench = bool(f.tilt_deg()<=FRAME_TILT and not over_tray and
                    all(-PEN_TOL<=c[2]-TOP<=FLOOR_TOL and TABLE_X[0]<=c[0]<=TABLE_X[1]
                        and TABLE_Y[0]<=c[1]<=TABLE_Y[1] for c in corners))
    frame_on = bool(frame_on or on_bench)
    frame_lost = bool(f.low[2]<TOP-FALL_DEPTH)
    inside = (b.low[0]>=TRAY_X-TRAY_W/2 and b.high[0]<=TRAY_X+TRAY_W/2 and
              b.low[1]>=TRAY_Y-TRAY_D/2 and b.high[1]<=TRAY_Y+TRAY_D/2)
    gap = float(b.low[2]-TRAY_FLOOR)
    up = b.tilt_deg()<=TRAY_TILT
    return dict(frame_on=bool(frame_on),frame_lost=bool(frame_lost),frame_com_margin=float(com_margin),
                frame_support_gap=float(support_gap) if math.isfinite(support_gap) else None,
                inside=bool(inside),up=bool(up),gap=gap,
                staged=bool(inside and up and -PEN_TOL<=gap<=STAGED_H and frame_on),
                seated=bool(inside and up and -PEN_TOL<=gap<=FLOOR_TOL and frame_on))


def score(objects):
    memory = break_memory(objects)
    p = placement_terms(objects)
    floor = ever("board fell off bench",objects["board"].low[2]<TOP-FALL_DEPTH)
    frame_lost = ever("frame fell off bench",p["frame_lost"])
    quiet = at_rest(objects)
    free = released(objects["board"]) and released(objects["frame"])
    separated = relative_terms(objects["frame"],objects["board"])
    still_separated = separated["shift"]>BREAK_SHIFT or separated["angle"]>BREAK_ANGLE
    valid = memory["valid"] is True and not memory["bad"] and not floor and not frame_lost and still_separated
    done = bool(valid and p["seated"] and quiet and free)
    held = held_for("board trayed frame safe released still",done)
    if not valid:return 0.
    if p["staged"]:return 1. if done and held+1e-10>=STILL_S else .75
    return .5


def success(objects):
    return score(objects)>=1.


def demo():
    # Known initial geometry and time only; no sensing or scorer reads.
    C,O = FINGER_CLOSED,FINGER_OPEN
    park_l,park_r = (.100,.270,PARK_Z),(.100,-.300,PARK_Z)
    def w(dt,p,f,label,yaw=0.):return Waypoint(dt,tuple(p),f,label,yaw)
    push_z = SCORE_Z+.015
    push_y = START_SCORE_Y+FRAME_D+.0074
    slide_hover = (X,push_y,TRAVEL_Z)
    slide_start = (X,push_y,push_z)
    slide_end = (X,push_y-SLIDE-SLIDE_OVERSTROKE,push_z)
    hold = (X-.010,HOLD_Y,BLOCK_TOP+THICK+.016+PAD_TIP_PAST_TCP-HOLD_OVERTRAVEL)
    hold_hover = (X-.010,HOLD_Y,TRAVEL_Z)
    left = [w(1.5,park_l,C,"left park closed"),w(1.,slide_hover,C,"slide hover"),
            w(1.,slide_start,C,"slide approach"),w(1.5,slide_end,C,"slide panel 25 mm"),
            w(.5,slide_end,C,"slide settle"),
            w(.7,(X,push_y+.020,push_z),C,"slide back off"),
            w(.6,(X,push_y+.020,TRAVEL_Z),C,"slide rise"),
            w(.8,hold_hover,C,"hold hover"),w(1.,hold,C,"frame hold preload"),
            w(4.3,hold,C,"hold frame during bend and pickup"),
            w(.8,hold_hover,C,"frame release rise"),w(1.,park_l,O,"left park"),w(10.,park_l,O,"left clear")]
    pre_bend = sum(a.duration for a in left[:9])
    touch_z = BLOCK_TOP+THICK+PAD_TIP_PAST_TCP
    press = (X,PRESS_Y,touch_z)
    press_hover = (X,PRESS_Y,TRAVEL_Z)
    right = [w(1.5,park_r,C,"right park closed"),w(pre_bend-1.5,park_r,C,"wait for frame hold"),
             w(1.,press_hover,C,"board press hover"),w(1.,press,C,"board press touch")]
    for i,depth in enumerate((.008,.016,.024,BEND_OVERTRAVEL)):
        right.append(w(.55,(X,PRESS_Y,touch_z-depth),C,"progressive bend "+str(i+1)))
    right += [w(.5,(X,PRESS_Y,touch_z-BEND_OVERTRAVEL),C,"bend dwell"),
              w(.7,press_hover,C,"press rise closed"),w(.7,press_hover,O,"open above detached board",math.pi/2),
              w(1.,(X,LAND_Y+TERMINAL_Y,TRAVEL_Z),O,"terminal hover",math.pi/2),
              w(1.,(X,LAND_Y+TERMINAL_Y,PICK_TCP_Z),O,"terminal descend",math.pi/2),
              w(.7,(X,LAND_Y+TERMINAL_Y,PICK_TCP_Z),C,"terminal pinch after break",math.pi/2),
              w(.8,(X,LAND_Y+TERMINAL_Y,PICK_TCP_Z+PICK_CLEAR_RISE),C,"board clear nest lips",math.pi/2),
              w(.9,(X,LAND_Y+TERMINAL_Y-PICK_AWAY,PICK_TCP_Z+PICK_CLEAR_RISE),C,"board move away from frame",math.pi/2),
              w(1.,(X,LAND_Y+TERMINAL_Y-PICK_AWAY,TRAVEL_Z),C,"board lift",math.pi/2),
              w(1.6,(TRAY_X,TRAY_Y+TERMINAL_Y,TRAVEL_Z),C,"carry board to tray",math.pi/2),
              w(1.1,(TRAY_X,TRAY_Y+TERMINAL_Y,TRAY_Z+THICK/2+.016+.007),C,"board lower",math.pi/2),
              w(.7,(TRAY_X,TRAY_Y+TERMINAL_Y,TRAY_Z+THICK/2+.016+.007),O,"board release",math.pi/2),
              w(1.,(TRAY_X,TRAY_Y+TERMINAL_Y,TRAVEL_Z),O,"tray rise",math.pi/2),
              w(1.,park_r,O,"right park"),w(4.,park_r,O,"right clear")]
    duration = max(sum(a.duration for a in left),sum(a.duration for a in right))
    for plan,park in ((left,park_l),(right,park_r)):
        extra = duration-sum(a.duration for a in plan)
        if extra>1e-9:plan.append(w(extra,park,O,"hold clear to end"))
    return {"left":left,"right":right}
