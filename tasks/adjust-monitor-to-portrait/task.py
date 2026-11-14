"""Real-size desktop monitor: lift, tilt and pivot through passive stand joints."""
from pathlib import Path
import math
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from partkit import part, Hinge, Slider, on_reset, on_step, _np
from yam_arena.task_file import Box

INSTRUCTION = "Turn the monitor to portrait for reading a document. Leave its centre 32 cm above the desk, facing straight ahead and upright, with the base where it started. Let go and leave it stable."
TOP = .7475
START = np.array([.18, .025, TOP + .010])
WIDTH, HEIGHT, DEPTH = .5034, .3304, .0503
PIVOT = np.array([.015, 0., .1922])
PANEL_CENTRE = np.array([-.02515, 0., 0.])
TARGET_CENTRE = .320
HOLD_STEPS = 40
BLACK = (.055, .06, .065)
SILVER = (.37, .39, .41)

def box(c, size, color=BLACK):
    return (tuple(c), tuple(size), color)

# Panel chassis and visible glass. The pivot remains at the back centre. No
# enlarged grasp tabs or controls are added. Geometry dimensions come from the
# P2217 guide; unreported internal mass distribution/hinge efforts are estimates.
PANEL = (
    box(PANEL_CENTRE, (DEPTH, WIDTH, HEIGHT)),
    box((-.0508, 0., .002), (.001, .47376, .29610), (.12, .20, .27)),
    box((-.0515, -.16, .09), (.001, .07, .003), (.55, .62, .65)),
    box((-.0515, -.13, .07), (.001, .13, .003), (.55, .62, .65)),
    box((-.0515, -.13, .05), (.001, .13, .003), (.55, .62, .65)),
)
MONITOR = part(
    "monitor", tuple(float(v) for v in START),
    [box((.01, 0., 0.), (.18, .2426, .020), SILVER)], mass=1.90,
    friction=.8, patch_radius=.006,
    sliders=(Slider("lift", tuple(PIVOT),
                    (box(PIVOT + [.028, 0., -.04], (.035, .045, .11), SILVER),),
                    travel=.130, pressed=1., rate=10., damping=12., mass=.20,
                    parent="swivel", spring_zero=3.293, friction_effort=2.0),),
    hinges=(
        Hinge("swivel", (0., 0., 0.),
              (box((.075, 0., .13), (.055, .055, .25), SILVER),),
              lower=-math.pi/4, upper=math.pi/4, rate=0., damping=.15,
              mass=.10, friction_effort=.8),
        Hinge("tilt", tuple(PIVOT),
              (box((.014, 0., 0.), (.027, .05, .05), SILVER),),
              parent="lift", axis="Y", lower=math.radians(-5),
              upper=math.radians(21), rate=0., damping=.12, mass=.13,
              friction_effort=.95),
        Hinge("pivot", tuple(PIVOT), PANEL,
              parent="tilt", axis="X", lower=-math.pi/2, upper=math.pi/2,
              rate=0., damping=.12, mass=3.00, friction_effort=.18),
    ),
)
OBJECTS = [MONITOR, *[Box(name=f"{side}_robot_pedestal", pos=(0.,y,TOP+.175),
                         size=(.20,.20,.35), mass=None, color=SILVER)
                       for side,y in (("left",.305),("right",-.305))]]
_STATE = {}

def rx(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])

def ry(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])

def rz(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def panel_pose(obj):
    """Forward kinematics for observable panel pose, including free base motion."""
    yaw=obj.hinges['swivel']; tilt=obj.hinges['tilt']; pivot=obj.hinges['pivot']
    A=obj.rotation() @ rz(yaw)
    R=A @ ry(tilt) @ rx(pivot)
    centre=obj.pos + A @ (PIVOT + [0,0,obj.sliders['lift']]) + R @ PANEL_CENTRE
    return centre,R

@on_reset
def reset_state(environment=None):
    _STATE.clear()
    _STATE.update(step=0, cache=-1, last=(0.,False), recent=[], tips=[], collision=False, base_moved=False)

@on_step
def tick(environment):
    from yam_arena.constants import TCP_OFFSET
    base=environment.unwrapped
    origin=_np(base.scene.env_origins)[0]
    _STATE['step']+=1
    tips=[]
    for key in ('left_robot','right_robot'):
        robot=base.scene[key]
        p=_np(robot.data.body_link_pose_w)[0][list(robot.body_names).index('gripper')]
        from partkit import _quat_matrix
        x,y,z,w=p[3:7]
        tips.append(p[:3]+_quat_matrix((w,x,y,z)) @ np.asarray(TCP_OFFSET)-origin)
    _STATE['tips']=tips

def measure(obj):
    centre,R=panel_pose(obj)
    half=np.array([DEPTH,WIDTH,HEIGHT])/2
    corners=np.array([centre+R @ (half*np.array([a,b,c]))
                      for a in (-1,1) for b in (-1,1) for c in (-1,1)])
    held=any(np.all(np.abs(R.T @ (p-centre)) < half+.025) for p in _STATE['tips'])
    held |= any(np.all(np.abs(obj.to_local(p)-np.array([.01,0.,0.])) < np.array([.09,.1213,.01])+.025) for p in _STATE['tips'])
    held |= any(np.all(np.abs(obj.to_local(p)-np.array([.075,0.,.13])) < np.array([.0275,.0275,.125])+.025) for p in _STATE['tips'])
    low=float(corners[:,2].min()-TOP)
    base_tilt=math.degrees(math.acos(np.clip(obj.rotation()[2,2],-1,1)))
    return dict(centre=centre.tolist(), height=float(centre[2]-TOP),
                lift=obj.sliders['lift'], tilt=math.degrees(obj.hinges['tilt']),
                pivot=math.degrees(obj.hinges['pivot']), swivel=math.degrees(obj.hinges['swivel']),
                low=low, held=bool(held), base_shift=float(np.linalg.norm(obj.pos-START)),
                base_tilt=base_tilt, base_yaw=math.degrees(math.atan2(obj.rotation()[1,0], obj.rotation()[0,0])))

def update(objects):
    if _STATE['cache']==_STATE['step']:return _STATE['last']
    _STATE['cache']=_STATE['step']
    m=measure(objects['monitor'])
    # A pane must clear the desk. This geometric check is independent of
    # contact-force instrumentation and includes the rotation sweep history.
    _STATE['collision'] |= m['low'] < .001
    base_ok=m['base_shift']<.010 and m['base_tilt']<3 and abs(m['base_yaw'])<3
    _STATE['base_moved'] |= not base_ok
    base_ok = not _STATE['base_moved']
    portrait=abs(abs(m['pivot'])-90)<3
    target=portrait and abs(m['height']-TARGET_CENTRE)<.010 and abs(m['tilt'])<3 and abs(m['swivel'])<3
    qualified=target and base_ok and not m['held'] and not _STATE['collision']
    recent=_STATE['recent']
    recent.append(m if qualified else None)
    del recent[:-HOLD_STEPS]
    stable=(len(recent)==HOLD_STEPS and all(x is not None for x in recent)
            and all(np.linalg.norm(np.array(x['centre'])-m['centre'])<.002
                    and abs(x['pivot']-m['pivot'])<1 for x in recent))
    if not base_ok or _STATE['collision']:value=0.
    elif stable:value=1.
    elif portrait and not m['held']:value=.5
    elif m['lift']>.100 and abs(m['pivot'])>30:value=.25
    else:value=0.
    _STATE['last']=(value,bool(stable))
    return _STATE['last']

def score(objects):return update(objects)[0]
def success(objects):return update(objects)[1]
reset_state()
