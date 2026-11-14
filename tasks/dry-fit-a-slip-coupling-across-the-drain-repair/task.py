"""On the station table: a white 1-1/2 in PVC drain line runs straight away from the robot, between the arms, with a
damaged length cut out of it. Its two fixed ends rest in steel pipe clamps on posts. A loose white replacement length
fills the gap. Its far end is already dry-fitted: a grey repair coupling sits across that joint. Its near end is free.
A second grey repair coupling sits slid back onto the near fixed end, clear of the open joint. Black witness marks
are drawn round the pipes on both sides of each joint.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

PVC, GREY, BORE, MARK = (0.93, 0.93, 0.91), (0.62, 0.63, 0.64), (0.20, 0.20, 0.20), (0.05, 0.05, 0.05)
PVC_FRICTION = (0.40, 0.35)  # static, dynamic: dry PVC on PVC
STEEL, DARK = (0.55, 0.56, 0.58), (0.25, 0.26, 0.28)

# --- the drain line: axis along world x at y = 0 ------------------------------------------------------------------
LINE_Y = 0.0
AXIS_Z = TOP + 0.090  # axis 90 mm above the table
R_PIPE = 0.02415  # 1-1/2 in Schedule 40 PVC: 1.900 in (48.3 mm) outside diameter
R_IN = 0.02047  # 3.68 mm wall (shown at the cut ends only)
NEAR_END = 0.182  # x of the near fixed end's cut face
FAR_END = 0.402  # x of the far fixed end's cut face (a 220 mm gap)
NEW_LEN = 0.212  # the replacement length: 8 mm shorter than the gap
NEW_X0 = NEAR_END + 0.004  # its near end, 4 mm from the near fixed end
NEW_X1 = NEW_X0 + NEW_LEN  # its far end, 4 mm from the far fixed end
NEW_MASS = 0.153  # 0.72 kg/m of PVC (density 1.4)

# --- repair (slip) coupling: no centre stop, 80 mm long ----------------------------------------------------------
CPL_L = 0.080
R_BORE = 0.02435  # bore 48.7 mm across: 0.2 mm clearance per side on the pipe
R_OUT = 0.02875  # 57.5 mm outside
BEV_D, BEV_DEG = 0.0020, 30.0  # both mouths bevelled 30 degrees off the axis, 2 mm deep
BEV_L = BEV_D / math.tan(math.radians(BEV_DEG))
N_SIDES = 24
CPL_MASS = 0.080
MARK_IN = 0.038  # witness marks: 38 mm from each pipe end (half the coupling, less half the 4 mm gap)
MARK_W = 0.004
PEND_D, PEND_DEG = 0.0020, 30.0  # the replacement's cut ends: outside bevel 2 mm deep at 30 degrees

NEAR_CPL_X = NEAR_END - 0.006 - CPL_L / 2  # slid fully back onto the near fixed end, its mouth 6 mm from the cut
FAR_CPL_X = (NEW_X1 + FAR_END) / 2  # centred on the far joint (dry-fitted)


def _ring_slab(r0, r1, x0, x1, a, color, collide=True):
    """One flat of a ring about the x axis: radial depth r0..r1, axial x0..x1, at angle a round the axis."""
    w = 2 * math.tan(math.pi / N_SIDES) * r1
    rc = (r0 + r1) / 2
    return parts.box((x1 - x0, w, r1 - r0), at=((x0 + x1) / 2, -rc * math.sin(a), rc * math.cos(a)), color=color,
                     roll=a, collide=collide)


def _coupling_shapes():
    """In the coupling frame: origin at its middle, axis along x."""
    out = []
    xm = CPL_L / 2 - BEV_L  # where the bevels start
    slant = math.hypot(BEV_L, BEV_D)
    t = 0.0028  # bevel slab thickness
    th = math.radians(BEV_DEG)
    for i in range(N_SIDES):
        a = (i + 0.5) * 2 * math.pi / N_SIDES  # flats top and bottom are offset half a step: no flat at the very top
        out.append(_ring_slab(R_BORE, R_OUT, -xm, xm, a, GREY))
        for side in (1.0, -1.0):
            # inner face of the bevel: from (side*xm, R_BORE) to (side*L/2, R_BORE + BEV_D) in (axial, radial)
            mx, mr = side * (xm + CPL_L / 2) / 2, R_BORE + BEV_D / 2
            pitch = -side * th  # turns the slab's x toward the mouth's opening
            n_ax, n_r = math.sin(pitch), math.cos(pitch)  # the face's outward normal (axial, radial)
            cx, cr = mx + n_ax * t / 2, mr + n_r * t / 2
            out.append(parts.box((slant, 2 * math.tan(math.pi / N_SIDES) * R_OUT, t),
                                 at=(cx, -cr * math.sin(a), cr * math.cos(a)), color=GREY, roll=a, pitch=pitch))
    return out


def _pipe_shapes(x0, x1, marks=(), open_ends=(), origin=0.0, bevel=False):
    """A pipe from x0 to x1 (world x; shapes placed relative to ``origin``), with witness marks at the given x. With
    ``bevel`` both cut ends are bevelled on the outside, PEND_DEG off the axis and PEND_D deep."""
    if not bevel:
        out = [parts.cylinder(R_PIPE, x1 - x0, at=((x0 + x1) / 2 - origin, 0.0, 0.0), color=PVC, axis="x")]
    else:
        bl = PEND_D / math.tan(math.radians(PEND_DEG))
        out = [parts.cylinder(R_PIPE, x1 - x0 - 2 * bl, at=((x0 + x1) / 2 - origin, 0.0, 0.0), color=PVC, axis="x")]
        t, th, slant = 0.003, math.radians(PEND_DEG), math.hypot(bl, PEND_D)
        for side, xe in ((-1.0, x0), (1.0, x1)):
            out.append(parts.cylinder(R_PIPE - PEND_D - 0.0005, bl, at=(xe - side * bl / 2 - origin, 0.0, 0.0),
                                      color=PVC, axis="x"))
            pitch = side * th  # the outer face narrows toward the cut end
            mx, mr = xe - side * bl / 2 - origin, R_PIPE - PEND_D / 2 - 0.0003  # flat corners inside R_PIPE
            cx, cr = mx - math.sin(pitch) * t / 2, mr - math.cos(pitch) * t / 2
            for i in range(N_SIDES):
                a = (i + 0.5) * 2 * math.pi / N_SIDES
                out.append(parts.box((slant, 2 * math.tan(math.pi / N_SIDES) * R_PIPE, t),
                                     at=(cx, -cr * math.sin(a), cr * math.cos(a)), color=PVC, roll=a, pitch=pitch))
    for xm in marks:
        out.append(parts.cylinder(R_PIPE + 0.0001, MARK_W, at=(xm - origin, 0.0, 0.0), color=MARK, axis="x",
                                  collide=False))
    for xe in open_ends:  # the bore seen at a cut end
        out.append(parts.cylinder(R_IN, 0.0004, at=(xe - origin, 0.0, 0.0), color=BORE, axis="x", collide=False))
    return out


def _clamp_shapes(xc):
    """A steel post from the table and a pipe clamp round the pipe at x = xc (world, relative to the line's axis)."""
    zb = -(AXIS_Z - TOP)  # the table, relative to the axis
    post_h = (AXIS_Z - TOP) - R_PIPE - 0.004
    out = [parts.box((0.040, 0.060, 0.006), at=(xc, 0.0, zb + 0.003), color=DARK),  # foot plate
           parts.box((0.016, 0.016, post_h), at=(xc, 0.0, zb + post_h / 2), color=STEEL),
           parts.box((0.020, 0.040, 0.004), at=(xc, 0.0, -R_PIPE - 0.002), color=STEEL)]  # saddle under the pipe
    for i in range(N_SIDES):
        a = (i + 0.5) * 2 * math.pi / N_SIDES
        if math.cos(a) > -0.75:  # the strap round the pipe (open at the bottom over the saddle)
            out.append(_ring_slab(R_PIPE, R_PIPE + 0.0025, xc - 0.010, xc + 0.010, a, STEEL, collide=False))
    for s in (1.0, -1.0):  # strap ears and bolts
        out.append(parts.box((0.020, 0.012, 0.004), at=(xc, s * (R_PIPE + 0.008), -0.002), color=STEEL,
                             collide=False))
    return out


NEAR_X0, FAR_X1 = 0.045, 0.550
NEAR_PIPE = parts.Part("near_end", pos=(0.0, LINE_Y, AXIS_Z), friction=PVC_FRICTION, shapes=[
    *_pipe_shapes(NEAR_X0, NEAR_END, marks=(NEAR_END - MARK_IN,), open_ends=(NEAR_END, NEAR_X0)),
    *_clamp_shapes(0.072)])
FAR_PIPE = parts.Part("far_end", pos=(0.0, LINE_Y, AXIS_Z), friction=PVC_FRICTION, shapes=[
    *_pipe_shapes(FAR_END, FAR_X1, marks=(FAR_END + MARK_IN,), open_ends=(FAR_END, FAR_X1)),
    *_clamp_shapes(0.515)])
NEW_X = (NEW_X0 + NEW_X1) / 2
NEW_PIPE = parts.Part("new_pipe", pos=(NEW_X, LINE_Y, AXIS_Z), friction=PVC_FRICTION, mass=NEW_MASS, shapes=_pipe_shapes(
    NEW_X0, NEW_X1, marks=(NEW_X0 + MARK_IN, NEW_X1 - MARK_IN), open_ends=(NEW_X0, NEW_X1), origin=NEW_X, bevel=True))
COUPLING = parts.Part("coupling", pos=(NEAR_CPL_X, LINE_Y, AXIS_Z), friction=PVC_FRICTION, mass=CPL_MASS, shapes=_coupling_shapes())
FAR_COUPLING = parts.Part("far_coupling", pos=(FAR_CPL_X, LINE_Y, AXIS_Z), friction=PVC_FRICTION, mass=CPL_MASS, shapes=_coupling_shapes())

# The scripted fingers open to 80 mm and straddle the 48.3 mm pipe; the offline checker models them closed.
DEMO_TOUCHES = ("near_end",)

OBJECTS = [NEAR_PIPE, FAR_PIPE, NEW_PIPE, COUPLING, FAR_COUPLING]


# --- scoring -----------------------------------------------------------------------------------------------------
FULL_IN = 0.032  # each pipe at least 32 mm inside the coupling ...
EVEN = 0.006  # ... and the two depths within 6 mm of each other (the mouths on the witness marks)
PART_IN = 0.015  # 0.5: across the joint, each pipe at least 15 mm inside
FAR_IN = 0.030  # the far joint stays made: each pipe at least 30 mm inside the far coupling
ON_AXIS = 0.0015  # a coupling's middle and the replacement's ends within 1.5 mm of the line's axis
ALIGN_DEG = 1.0  # the replacement and both couplings within 1 degree of the line's direction
CLEAR = 0.005  # released: no finger or gripper surface within 5 mm of the replacement or either coupling

_grip = parts.__dict__.setdefault("_drain_coupling_task_grip", {"clear": None})
_AXIS = np.array([1.0, 0.0, 0.0])
_ON = np.array([0.0, LINE_Y, AXIS_Z])


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def _off_axis(p) -> float:
    d = np.asarray(p, float) - _ON
    return float(np.linalg.norm(d - (d @ _AXIS) * _AXIS))


def _angle_deg(q) -> float:
    u = _rot(q) @ _AXIS
    return math.degrees(math.acos(float(np.clip(abs(u @ _AXIS), -1.0, 1.0))))


def new_pipe_ends(objects):
    s = objects["new_pipe"]
    p, u = np.asarray(s.pos, float), _rot(s.quat_xyzw) @ _AXIS
    return p - u * NEW_LEN / 2, p + u * NEW_LEN / 2


def depths(objects, name: str, fixed_end: float, new_end) -> tuple[float, float]:
    """(depth of the fixed end inside the coupling, depth of the replacement's end inside it), measured along the
    coupling's own axis from each mouth to the pipe's cut face; negative when the pipe end is outside."""
    c = objects[name]
    p, u = np.asarray(c.pos, float), _rot(c.quat_xyzw) @ _AXIS
    fixed = np.array([fixed_end, LINE_Y, AXIS_Z])
    lo, hi = p - u * CPL_L / 2, p + u * CPL_L / 2
    if fixed_end < float(new_end[0]):  # near joint: the fixed end enters the low mouth
        return float((fixed - lo) @ u), float((hi - new_end) @ u)
    return float((hi - fixed) @ u), float((new_end - lo) @ u)


def _coupling_on(objects, name: str) -> bool:
    c = objects[name]
    return _off_axis(c.pos) <= ON_AXIS + EPS and _angle_deg(c.quat_xyzw) <= ALIGN_DEG + EPS


def in_line(objects) -> bool:
    a, b = new_pipe_ends(objects)
    return (_off_axis(a) <= ON_AXIS + EPS and _off_axis(b) <= ON_AXIS + EPS
            and _angle_deg(objects["new_pipe"].quat_xyzw) <= ALIGN_DEG + EPS)


def far_joint_made(objects) -> bool:
    _, b = new_pipe_ends(objects)
    d_fix, d_new = depths(objects, "far_coupling", FAR_END, b)
    return _coupling_on(objects, "far_coupling") and d_fix >= FAR_IN - EPS and d_new >= FAR_IN - EPS


def near_joint(objects) -> float:
    """1.0 on the marks, 0.5 across the joint, 0.0 otherwise (geometry only)."""
    if not _coupling_on(objects, "coupling"):
        return 0.0
    a, _ = new_pipe_ends(objects)
    d_fix, d_new = depths(objects, "coupling", NEAR_END, a)
    if d_fix >= FULL_IN - EPS and d_new >= FULL_IN - EPS and abs(d_fix - d_new) <= EVEN + EPS:
        return 1.0
    if d_fix >= PART_IN - EPS and d_new >= PART_IN - EPS:
        return 0.5
    return 0.0


def _done(objects) -> float:
    if _grip["clear"] is not True:  # unknown or not let go: no credit
        return 0.0
    if not all(objects[n].resting() for n in ("new_pipe", "coupling", "far_coupling")):
        return 0.0
    if not (in_line(objects) and far_joint_made(objects)):
        return 0.0
    return near_joint(objects)


def success(objects):
    """The loose coupling centred on the open joint's marks, the replacement in line, both joints made, let go and
    still."""
    return bool(_done(objects) == 1.0)


def score(objects):
    return _done(objects)


def _scored_boxes(objects):
    out = []
    for spec in (NEW_PIPE, COUPLING, FAR_COUPLING):
        st = objects[spec.name]
        out += [(c, r, s) for _, c, r, s in spec.world_boxes({"base": (np.asarray(st.pos), _rot(st.quat_xyzw))})]
    return out


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    boxes = _scored_boxes(sim.states())
    hulls = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in ("tip_left", "tip_right", "gripper"):
                m = np.eye(4)
                m[:3, :3] = _rot(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                hulls += analysis.world_hulls({n: m})
    _grip["clear"] = bool(hulls) and all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for c, r, s in boxes)


@parts.on_reset
def _reset_grip(sim) -> None:
    _grip["clear"] = None


# --- demo: open loop from the published start layout -------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
OPEN = -0.040  # 80 mm between the fingers
PIPE_GRIP = -0.02265  # sent to 45.3 mm on the 48.3 mm pipe: 1.5 mm past each side
CPL_GRIP = -0.02575  # sent to 51.5 mm on the 57.5 mm coupling
GRIP_Z = AXIS_Z + TIP - 0.003  # finger tips 3 mm below the axis: the pads pinch the pipe's widest part
HOVER = AXIS_Z + 0.060  # TCP height above the line; open fingers straddle the pipe and coupling
HOLD_X = 0.270  # the left hand holds the replacement here, 8.4 cm beyond the open joint
LIFT = 0.0036  # sent this far above in line: the soft arm settles low under the pipe's weight (calibrated once)
DROOP_AT_HOLD = 0.004  # the replacement's free end hangs about 6 mm low at the start, so about 4 mm where it is gripped
HOLD_DY = 0.003  # the left arm settles a few mm to the robot's right of its target here (calibrated once)
CPL_HOLD = -0.022  # the right hand holds the coupling 22 mm behind its middle, to keep clear of the left hand
SLIDE_TO = NEAR_END + 0.002 + 0.003  # the coupling's middle sent 3 mm past centred: the soft arm lags the drag


def demo():
    hx, hy = HOLD_X, LINE_Y + HOLD_DY
    left = [
        Waypoint(2.0, (hx, hy, HOVER), OPEN, "over the replacement"),
        Waypoint(1.5, (hx, hy, GRIP_Z - DROOP_AT_HOLD), OPEN, "down beside it"),
        Waypoint(1.0, (hx, hy, GRIP_Z - DROOP_AT_HOLD), PIPE_GRIP, "grip it"),
        Waypoint(1.5, (hx, hy, GRIP_Z + LIFT), PIPE_GRIP, "lift the free end into line"),
        Waypoint(11.0, (hx, hy, GRIP_Z + LIFT), PIPE_GRIP, "hold it in line"),
        Waypoint(1.0, (hx, hy, GRIP_Z + LIFT), OPEN, "let go"),
        Waypoint(1.5, (hx, hy, HOVER), OPEN, "up"),
        Waypoint(2.0, (0.26, 0.20, HOVER), OPEN, "away to the left"),
        Waypoint(2.0, (0.20, 0.25, TOP + 0.20), OPEN, "up and clear"),
        Waypoint(3.0, (0.20, 0.25, TOP + 0.20), OPEN, "wait clear"),
    ]
    cx, sx = NEAR_CPL_X + CPL_HOLD, SLIDE_TO + CPL_HOLD
    right = [
        Waypoint(4.0, (0.20, -0.25, TOP + 0.20), OPEN, "wait clear"),
        Waypoint(2.0, (cx, LINE_Y, HOVER), OPEN, "over the coupling"),
        Waypoint(1.5, (cx, LINE_Y, GRIP_Z), OPEN, "down beside it"),
        Waypoint(1.0, (cx, LINE_Y, GRIP_Z), CPL_GRIP, "grip it"),
        Waypoint(4.0, (sx, LINE_Y, GRIP_Z), CPL_GRIP, "slide it across the joint"),
        Waypoint(1.0, (sx, LINE_Y, GRIP_Z), OPEN, "let go"),
        Waypoint(1.5, (sx, LINE_Y, HOVER), OPEN, "up"),
        Waypoint(2.0, (0.20, -0.25, TOP + 0.20), OPEN, "away to the right"),
        Waypoint(6.0, (0.20, -0.25, TOP + 0.20), OPEN, "wait clear"),
    ]
    return {"left": left, "right": right}
