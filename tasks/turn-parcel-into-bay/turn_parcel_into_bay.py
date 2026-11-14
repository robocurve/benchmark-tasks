"""On the station table, in front of the two arms: a long parcel lies across the arms' view (its length side to side)
between two other parcels, one on each side, with only about 2 cm to spare at each end. Ahead of it, two yellow
bollards mark the mouth of a loading bay taped out in red on the table, which ends at a fixed stop bar. The long parcel
has to end up lying flat and lengthwise in the bay with its front end against the stop bar, and be left there,
without bumping either neighbour. The bollards are closer together (37 cm) than the parcel is long (42 cm), so it
cannot go forward lying across: it has to be turned a quarter turn first, right where it lies. Pushed at one end only,
a parcel swings about a point near its other end and slides sideways into a neighbour; turned by both ends at once in
opposite directions (a pure turn), its corners sweep a circle that just clears the neighbours and the bollards. Once
turned, it slides between the bollards into the bay.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- long parcel: brown corrugated cardboard box, 14 cm front to back x 42 cm side to side x 10 cm tall, 800 g, centred
  at (0.30, 0), its long sides parallel to the y axis. Turning about its centre, its corners sweep a circle 22.1 cm
  in radius.
- neighbours: two white cardboard boxes, 14 x 20 x 10 cm, 1.2 kg each, centred at (0.30, 0.333) and (0.30, -0.333):
  their inner faces are 2.3 cm beyond the long parcel's ends and 1.2 cm outside the circle its corners sweep.
- bollards: two fixed yellow posts 3 x 3 cm, 8 cm tall, centred at (0.48, 0.20) and (0.48, -0.20): 37 cm apart
  inside, 2.7 cm outside the circle the parcel's corners sweep.
- bay: red tape (1.5 cm wide, 0.5 mm thick) marking a rectangle 20 cm wide (y -0.10 to 0.10, inside the tape) from
  x 0.20 to x 0.66 on the table; it runs under the parcel's start position.
- stop bar: fixed dark grey bar, 2 cm front to back x 22 cm wide x 4 cm tall, its near face (the one toward the robot)
  at x 0.66, the bay's far end.

Score: 1.0 when success() holds for 2 s in a row; otherwise score() at the end of the trial.
- 0 for the whole trial once the parcel's centre has been more than 4 cm to either side of the centre line, or either
  neighbour has moved more than 1 cm (both latched, checked after every control step).
- 1.0 (success): the parcel released (no part of either arm within 1 cm of it: points no more than 1.5 cm apart
  covering the arms' collision meshes, placed by forward kinematics from their measured joint positions), lying flat
  on the table (each of the four corners of its underside within 3 mm of the table top), inside the bay (those four
  corners inside the tape's inner edges, or at most 5 mm onto the tape), its long sides within 5 degrees of front to
  back, both corners of its front end within 1 cm of the stop bar, and the parcel and both neighbours at rest (moving
  slower than 1 cm/s and turning slower than 0.1 rad/s), all of this without a break for 2 s.
- 0.5: at the end of the trial, the parcel released, lying flat on the table (as above) and at rest, with its long
  sides within 10 degrees of front to back (turned, but not placed as for 1.0).
- 0 otherwise.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 2.0
SCORE_OVER = "end"  # partial credit is the end state; a broken rule latches 0 inside the file

KRAFT = (0.62, 0.47, 0.30)
WHITE_BOX = (0.90, 0.89, 0.86)
YELLOW = (0.95, 0.78, 0.10)
TAPE_RED = (0.82, 0.10, 0.10)
DARK = (0.20, 0.21, 0.23)

PARCEL = (0.14, 0.42, 0.10)  # x, y, z at the start
PARCEL_KG = 0.8
CENTRE = np.array([0.30, 0.0])
CX = 0.30  # plain float for object positions (the simulator rejects NumPy scalars)
NEIGHBOUR = (0.14, 0.20, 0.10)
NEIGHBOUR_KG = 1.2
SWEEP_R = float(np.hypot(PARCEL[0], PARCEL[1]) / 2)  # 0.221
NEIGHBOUR_Y = SWEEP_R + 0.012 + NEIGHBOUR[1] / 2  # 0.333
BOLLARD, BOLLARD_XY = (0.03, 0.03, 0.08), (0.48, 0.20)
BAY_X, BAY_HALF_Y = (0.20, 0.66), 0.10  # inside of the taped rectangle; BAY_X[1] is the stop bar's near face
TAPE_W, TAPE_T = 0.015, 0.0005
STOP = (0.02, 0.22, 0.04)

# Debug and check runs (env vars; the normal task sets none of them):
# TP_ONE=1     the wrong way: one arm only (the right one pushes near the parcel's right end; the left stays clear).
# TP_HOVER=1   scorer check: the parcel held motionless (gravity off) in its finished place but 1 cm above the table;
#              the arms stay still.
# TP_SHORT=1   scorer check: the parcel starts turned, at rest on the table in the bay, its front end 3.5 cm short of
#              the stop bar; the arms stay still.
# TP_NO_LET_GO=1  scorer check: the demo, but the gripper keeps pushing the parcel against the stop bar to the end.
ONE_ARM = os.environ.get("TP_ONE") == "1"
HOVER_TEST = os.environ.get("TP_HOVER") == "1"
SHORT_TEST = os.environ.get("TP_SHORT") == "1"
NO_LET_GO = os.environ.get("TP_NO_LET_GO") == "1"
HOVER_UP = 0.01  # TP_HOVER: the parcel's underside this far above the table
SHORT_BY = 0.035  # TP_SHORT: its front end this far short of the stop bar
TURNED = -math.pi / 2  # the parcel's yaw once turned (as the demo turns it: clockwise seen from above)


def _tape():
    x0, x1 = BAY_X
    z = TOP + TAPE_T / 2
    side_len = x1 - x0
    return [
        Box("tape_back", pos=(x0 - TAPE_W / 2, 0.0, z), size=(TAPE_W, 2 * (BAY_HALF_Y + TAPE_W), TAPE_T),
            color=TAPE_RED),
        Box("tape_left", pos=((x0 + x1) / 2, BAY_HALF_Y + TAPE_W / 2, z), size=(side_len, TAPE_W, TAPE_T),
            color=TAPE_RED),
        Box("tape_right", pos=((x0 + x1) / 2, -(BAY_HALF_Y + TAPE_W / 2), z), size=(side_len, TAPE_W, TAPE_T),
            color=TAPE_RED),
    ]


def _parcel():
    if HOVER_TEST:  # a body without gravity stays where it is put: held motionless, as by an unseen hand
        x = BAY_X[1] - 0.002 - PARCEL[1] / 2
        return Body.build("parcel", pos=(float(x), 0.0, TOP + HOVER_UP + PARCEL[2] / 2), mass=PARCEL_KG,
                          shapes=[BoxShape((0.0, 0.0, 0.0), PARCEL, KRAFT)], yaw=TURNED, color=KRAFT, gravity=False,
                          friction=(1.2, 1.0))
    if SHORT_TEST:
        x = BAY_X[1] - SHORT_BY - PARCEL[1] / 2
        return Box("parcel", pos=(float(x), 0.0, TOP + 0.001 + PARCEL[2] / 2), size=PARCEL, mass=PARCEL_KG,
                   color=KRAFT, yaw=TURNED)
    return Box("parcel", pos=(CX, 0.0, TOP + 0.001 + PARCEL[2] / 2), size=PARCEL, mass=PARCEL_KG, color=KRAFT)


OBJECTS = _tape() + [
    Box("stop_bar", pos=(BAY_X[1] + STOP[0] / 2, 0.0, TOP + STOP[2] / 2), size=STOP, color=DARK),
    Box("bollard_left", pos=(BOLLARD_XY[0], BOLLARD_XY[1], TOP + BOLLARD[2] / 2), size=BOLLARD, color=YELLOW),
    Box("bollard_right", pos=(BOLLARD_XY[0], -BOLLARD_XY[1], TOP + BOLLARD[2] / 2), size=BOLLARD, color=YELLOW),
    _parcel(),
    Box("neighbour_left", pos=(CX, NEIGHBOUR_Y, TOP + 0.001 + NEIGHBOUR[2] / 2), size=NEIGHBOUR,
        mass=NEIGHBOUR_KG, color=WHITE_BOX),
    Box("neighbour_right", pos=(CX, -NEIGHBOUR_Y, TOP + 0.001 + NEIGHBOUR[2] / 2), size=NEIGHBOUR,
        mass=NEIGHBOUR_KG, color=WHITE_BOX),
]
NEIGHBOUR_START = {"neighbour_left": np.array([CENTRE[0], NEIGHBOUR_Y]),
                   "neighbour_right": np.array([CENTRE[0], -NEIGHBOUR_Y])}

# -- scoring ----------------------------------------------------------------------------------------------------------
# Every threshold is one a person can check on the real setup with a tape measure, a protractor and the video.
YAW_TOL, YAW_TOL_HALF = 5.0, 10.0  # degrees between the parcel's long sides and front to back: 1.0, 0.5
IN_BAY_SLACK = 0.005  # a corner counts as inside the bay when inside the tape's inner edge or at most 5 mm onto it
ON_TABLE = 0.003  # lying flat on the table: each corner of its underside within 3 mm of the table top
AT_BAR = 0.010  # pushed up to the stop bar: both corners of its front end within 1 cm of the bar's near face
RELEASED = 0.01  # let go: no part of either arm (gripper, fingers, wrist, forearm) within 1 cm of the parcel
SAMPLE = 0.015  # the arms' collision meshes are covered with points this close (any surface point within 8.7 mm)
STILL_V, STILL_W = 0.01, 0.1  # at rest: moving slower than 1 cm/s and turning slower than 0.1 rad/s (6 deg/s)
SIDE_MAX = 0.04  # "at any time": the parcel's centre more than 4 cm to either side of the centre line scores 0
BUMP_MAX = 0.01  # "at any time": a neighbour moved more than 1 cm scores 0
OLD_FLAT = 5.0  # the first version's flatness test (tilt, degrees), kept only to print what it would have said
ARM_LINKS = ("link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")  # what can reach the parcel

_DEBUG = os.environ.get("TP_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("TP_DEBUG_EVERY", "10"))
_HALF = np.array(PARCEL) / 2
_UNDER = np.array([[sx * _HALF[0], sy * _HALF[1], -_HALF[2]] for sx in (-1.0, 1.0) for sy in (-1.0, 1.0)])
_state: dict = {}
_TCP: dict = {}
_ARM_PTS: dict = {}  # points covering each arm's collision meshes (ARM_LINKS), world frame, after the last step
_ARM_IDS: dict = {}
_LINK_PTS: dict = {}  # the same points in each link's own frame (made once from the robot's URDF)


@on_reset
def _reset() -> None:
    _state.update(side=False, bump=False, max_side=0.0, max_bump=0.0, tick=0, measured=-1, m={}, step=0,
                  old_run=0, old_full_at=0, new_run=0)
    _TCP.clear()
    _ARM_PTS.clear()


_reset()


def _stl_triangles(path: pathlib.Path) -> np.ndarray:
    """The triangles of a binary STL file, (n, 3, 3)."""
    import struct

    data = path.read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    tri = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<3f4", (3,)), ("a", "<u2")]))
    return tri["v"].astype(np.float64)


def _surface_points(tris: np.ndarray) -> np.ndarray:
    """Points covering the triangles: each one cut into equal smaller triangles with sides of SAMPLE or less, and their
    corners kept. Every point of the surface is then within SAMPLE / sqrt(3) (8.7 mm) of a kept point."""
    sides = np.stack([np.linalg.norm(tris[:, (k + 1) % 3] - tris[:, k], axis=1) for k in range(3)], axis=1).max(1)
    cuts = np.maximum(1, np.ceil(sides / SAMPLE).astype(int))
    out = []
    for k in np.unique(cuts):
        ab = np.array([(i, j) for i in range(k + 1) for j in range(k + 1 - i)], float) / k
        weights = np.column_stack([ab, 1.0 - ab.sum(axis=1)])
        out.append(np.einsum("mk,nkd->nmd", weights, tris[cuts == k]).reshape(-1, 3))
    return np.vstack(out)


def _load_link_points() -> None:
    """Each arm link's collision meshes (convex pieces, from the URDF the simulator's arm is built from), as one point
    set per link in that link's frame, covering their surfaces (see _surface_points)."""
    import xml.etree.ElementTree as ET

    from yam_arena import YAM_URDF
    from yam_arena.urdf import rpy_matrix

    urdf = pathlib.Path(YAM_URDF)
    for link in ET.parse(urdf).getroot().iter("link"):
        name = link.get("name")
        if name not in ARM_LINKS:
            continue
        pieces = []
        for col in link.findall("collision"):
            origin, mesh = col.find("origin"), col.find("geometry/mesh")
            xyz = np.array([float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()])
            rpy = np.array([float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()])
            pieces.append(_surface_points(_stl_triangles(urdf.parent / mesh.get("filename"))) @ rpy_matrix(rpy).T + xyz)
        _LINK_PTS[name] = np.unique(np.round(np.vstack(pieces), 4), axis=0)
    missing = set(ARM_LINKS) - set(_LINK_PTS)
    if missing:
        raise RuntimeError(f"no collision meshes found for {sorted(missing)} in {urdf}")


@on_step
def _track(env) -> None:
    """After every control step: each arm's links placed by forward kinematics from its measured joint positions (arm
    and fingers), giving the points covering its collision meshes in the world, and each gripper's centre (TCP)."""
    try:
        from yam_arena import YAM_URDF, station
        from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, TCP_OFFSET
        from yam_arena.urdf import forward_kinematics

        if not _LINK_PTS:
            _load_link_points()
        _state["tick"] += 1
        base = getattr(env, "unwrapped", env)
        names = ARM_JOINTS + FINGER_JOINTS
        for side in ("left", "right"):
            robot = base.scene[f"{side}_robot"]
            if side not in _ARM_IDS:
                _ARM_IDS[side] = list(robot.find_joints(names, preserve_order=True)[0])
            q = as_numpy(robot.data.joint_pos)[0][_ARM_IDS[side]]
            poses = forward_kinematics(YAM_URDF, {n: float(v) for n, v in zip(names, q)})
            at = np.asarray(station.BASES[side], float)
            _ARM_PTS[side] = np.vstack([pts @ poses[k][:3, :3].T + poses[k][:3, 3] + at for k, pts in _LINK_PTS.items()])
            g = poses["gripper"]
            _TCP[side] = g[:3, :3] @ np.asarray(TCP_OFFSET, float) + g[:3, 3] + at
    except Exception:
        import traceback

        print("TP_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _turn_error(rot: np.ndarray) -> float:
    """Degrees between the parcel's long sides (its local y axis), seen from above, and the x axis (front to back): 0
    when turned a quarter turn either way, 90 at the start."""
    long = rot[:, 1]
    return float(math.degrees(math.atan2(abs(long[1]), abs(long[0]))))


def _gap_to_parcel(points: np.ndarray, pos: np.ndarray, rot: np.ndarray) -> float:
    """The smallest distance (m) from a set of points to the parcel (0 for a point on or inside it)."""
    local = (np.asarray(points, float) - pos) @ rot
    return float(np.linalg.norm(np.maximum(np.abs(local) - _HALF, 0.0), axis=1).min())


def _measure(objects) -> dict:
    """Everything the rubric looks at, once per control step; also latches the two rules."""
    if _state["measured"] == _state["tick"] and _state["m"]:
        return _state["m"]
    p = objects["parcel"]
    pos, rot = np.asarray(p.pos, float), rotation(np.asarray(p.quat_xyzw, float))
    under = pos + _UNDER @ rot.T  # the four corners of its underside
    heights = under[:, 2] - TOP
    on_table = bool(np.all(np.abs(heights) <= ON_TABLE))
    in_bay = bool(np.all(under[:, 0] >= BAY_X[0] - IN_BAY_SLACK) and np.all(under[:, 0] <= BAY_X[1] + IN_BAY_SLACK)
                  and np.all(np.abs(under[:, 1]) <= BAY_HALF_Y + IN_BAY_SLACK))
    front = np.sort(under[:, 0])[-2:]  # the two corners nearest the stop bar
    bar_gaps = BAY_X[1] - front  # their distances from its near face
    at_bar = bool(np.all(bar_gaps <= AT_BAR))
    err = _turn_error(rot)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(rot[2, 2])))))
    near = {k: _gap_to_parcel(v, pos, rot) for k, v in _ARM_PTS.items()}
    released = len(near) == 2 and min(near.values()) >= RELEASED
    parcel_still = p.resting(STILL_V, STILL_W)
    all_still = parcel_still and all(objects[n].resting(STILL_V, STILL_W) for n in NEIGHBOUR_START)
    # the two rules that hold for the whole trial
    side = abs(float(pos[1]))
    _state["max_side"] = max(_state["max_side"], side)
    if side > SIDE_MAX:
        _state["side"] = True
    for name, start in NEIGHBOUR_START.items():
        moved = float(np.linalg.norm(np.asarray(objects[name].pos[:2], float) - start))
        _state["max_bump"] = max(_state["max_bump"], moved)
        if moved > BUMP_MAX:
            _state["bump"] = True
    broken = _state["side"] or _state["bump"]
    full = bool(not broken and released and on_table and in_bay and err <= YAW_TOL and at_bar and all_still)
    half = bool(not broken and released and on_table and parcel_still and err <= YAW_TOL_HALF)
    # what the first version's rule (corners inside the bay in plan view, turned, tilted under 5 degrees, at rest)
    # would have said, printed with TP_DEBUG=1 only
    old = bool(not broken and in_bay and err <= YAW_TOL and tilt < OLD_FLAT and all_still)
    m = dict(pos=pos, err=err, tilt=tilt, heights=heights, on_table=on_table, in_bay=in_bay, bar_gaps=bar_gaps,
             at_bar=at_bar, near=near, released=released, parcel_still=parcel_still, all_still=all_still,
             broken=broken, full=full, half=half, old=old)
    _state.update(measured=_state["tick"], m=m)
    return m


def _debug(m) -> None:
    _state["step"] += 1
    _state["old_run"] = _state["old_run"] + 1 if m["old"] else 0
    _state["new_run"] = _state["new_run"] + 1 if m["full"] else 0
    hold = max(1, round(SUCCESS_HOLD_S * 20))
    if _state["old_run"] >= hold and not _state["old_full_at"]:
        _state["old_full_at"] = _state["step"]
        print(f"DBG step={_state['step']} OLD RULE would score 1.0 here (its conditions held for {SUCCESS_HOLD_S} s)",
              flush=True)
    if (_state["step"] - 1) % _DEBUG_EVERY:
        return
    p, h, g = m["pos"], m["heights"] * 1000, m["bar_gaps"] * 1000
    near = " ".join(f"{k}={v * 1000:.0f}mm" for k, v in m["near"].items())
    print(f"DBG step={_state['step']} pos=({p[0]:.4f},{p[1]:+.4f},{p[2] - TOP:.4f}) turn_err={m['err']:.1f}deg "
          f"tilt={m['tilt']:.2f}deg under_mm=[{h.min():.1f},{h.max():.1f}] on_table={m['on_table']} "
          f"in_bay={m['in_bay']} bar_gap_mm=[{g.min():.1f},{g.max():.1f}] at_bar={m['at_bar']} near({near}) "
          f"released={m['released']} still={m['parcel_still']}/{m['all_still']} "
          f"max_side={_state['max_side'] * 100:.1f}cm max_bump={_state['max_bump'] * 1000:.0f}mm "
          f"broken={'side' if _state['side'] else ''}{'bump' if _state['bump'] else ''}{'' if m['broken'] else '-'} "
          f"full={m['full']} run={_state['new_run']} half={m['half']} old_rule={m['old']} old_run={_state['old_run']}",
          flush=True)
    if len(_TCP) == 2:
        print("DBG   " + " | ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f})" for k, v in _TCP.items()),
              flush=True)


def success(objects) -> bool:
    """The parcel released, lying flat on the table inside the bay, turned to within 5 degrees of front to back, both
    corners of its front end within 1 cm of the stop bar, everything at rest, no rule broken (held for 2 s)."""
    return _measure(objects)["full"]


def distance(objects) -> float:
    m = _measure(objects)
    target = np.array([BAY_X[1] - PARCEL[1] / 2, 0.0])  # turned and pushed up to the stop bar
    return float(np.linalg.norm(m["pos"][:2] - target)) + 0.2 * math.radians(m["err"])


def score(objects) -> float:
    """Partial credit, read at the end of the trial (success scores 1.0): 0.5 when the parcel is released, lying flat
    on the table and at rest, turned to within 10 degrees of front to back; 0 if at any time its centre was more than
    4 cm to either side of the centre line or a neighbour moved more than 1 cm, or otherwise."""
    m = _measure(objects)
    if _DEBUG:
        _debug(m)
    return 0.5 if m["half"] else 0.0


# -- scripted demo -------------------------------------------------------------------------------------------------
# The turn: both grippers, closed and turned with the parcel, push it near its two ends in opposite directions: the
# left one pushes the back face forward near the left end, the right one pushes the front face back near the right
# end. Each contact point is moved along its circle about the parcel's centre, in small steps, so the parcel turns
# clockwise (seen from above) about its centre. The right wrist cannot turn a full quarter turn in one go, so halfway
# the left gripper eases off to a light touch while the right one backs off, turns its fingers half a turn (the closed
# fingers look the same) and comes back; then both finish the turn together, 90 degrees plus a small overshoot.
# The push: the left gripper moves out of the way; the right one comes round behind the parcel's back end and pushes
# it straight forward between the bollards until it meets the stop bar (the commanded point runs ahead of the parcel
# because the arm gives a little under the load), holds it there a moment, backs off and rises clear, leaving the
# parcel at rest against the bar.
FINGER_REACH = 0.034  # closed fingers reach this far ahead of the grip point along the push (turned square to it)
TIP = 0.013
PUSH_AT = float(os.environ.get("TP_PUSH_AT", "0.15"))  # contact this far from the centre along the long side
PUSH_IN = float(os.environ.get("TP_PUSH_IN", "0.008"))  # contacts commanded this far into the faces
STEPS = int(os.environ.get("TP_STEPS", "5"))  # steps per half turn
STEP_S = float(os.environ.get("TP_STEP_S", "1.0"))
OVERSHOOT = np.radians(float(os.environ.get("TP_OVERSHOOT", "1.5")))
FWD_LEAD = float(os.environ.get("TP_FWD_LEAD", "0.10"))  # forward push: commanded point this far past the stop
FWD_S = float(os.environ.get("TP_FWD_S", "5.0"))
PUSH_Z = TOP + 0.001 + 0.03 + TIP  # pad tips 3 cm above the parcel's underside
HOVER = 0.14
LOW_HOVER = 0.10


def _rot(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


def _stay(arm: str, seconds: float, label: str = "stay put") -> list:
    """The arm keeps its starting (READY) pose."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(seconds, tuple(float(v) for v in pos), FINGER_CLOSED, label, rot)]


def demo():
    if HOVER_TEST or SHORT_TEST:  # scorer checks: nothing moves
        return {"left": _stay("left", 12.0), "right": _stay("right", 12.0)}
    hx = PARCEL[0] / 2
    contacts = {"left": (np.array([-hx, PUSH_AT]), np.array([1.0, 0.0])),  # back face near the left end, push +x
                "right": (np.array([hx, -PUSH_AT]), np.array([-1.0, 0.0]))}  # front face near the right end, push -x

    def tcp(arm, yaw, depth):
        local, normal = contacts[arm]
        c = CENTRE + _rot(yaw) @ local
        n = _rot(yaw) @ normal
        return c - n * (FINGER_REACH - depth)

    mid_yaw = -np.pi / 4
    end_yaw = -np.pi / 2 - OVERSHOOT
    half1 = [mid_yaw * k / STEPS for k in range(1, STEPS + 1)]
    half2 = [mid_yaw + (end_yaw - mid_yaw) * k / STEPS for k in range(1, STEPS + 1)]
    regrasp = [0.8, 0.8, 1.2, 0.8, 0.8]  # right arm: back off, up, turn the fingers over, down, touch

    left = [
        Waypoint(2.0, (*tcp("left", 0.0, -0.02), TOP + HOVER), FINGER_CLOSED, "above the back face", 0.0),
        Waypoint(1.5, (*tcp("left", 0.0, -0.02), PUSH_Z), FINGER_CLOSED, "down behind the face", 0.0),
        Waypoint(1.0, (*tcp("left", 0.0, PUSH_IN), PUSH_Z), FINGER_CLOSED, "touch", 0.0),
    ]
    left += [Waypoint(STEP_S, (*tcp("left", y, PUSH_IN), PUSH_Z), FINGER_CLOSED, f"turn {k + 1}", float(y))
             for k, y in enumerate(half1)]
    left += [
        Waypoint(0.8, (*tcp("left", mid_yaw, 0.0), PUSH_Z), FINGER_CLOSED, "ease off to a light touch", float(mid_yaw)),
        Waypoint(sum(regrasp) - 0.8 - 0.8, (*tcp("left", mid_yaw, 0.0), PUSH_Z), FINGER_CLOSED, "wait", float(mid_yaw)),
        Waypoint(0.8, (*tcp("left", mid_yaw, PUSH_IN), PUSH_Z), FINGER_CLOSED, "touch again", float(mid_yaw)),
    ]
    left += [Waypoint(STEP_S, (*tcp("left", y, PUSH_IN), PUSH_Z), FINGER_CLOSED, f"turn {STEPS + k + 1}", float(y))
             for k, y in enumerate(half2)]
    back_l = tcp("left", end_yaw, -0.03) - np.array([0.03, 0.0])
    left += [
        Waypoint(1.0, (*back_l, PUSH_Z), FINGER_CLOSED, "back off", float(end_yaw)),
        Waypoint(1.5, (*back_l, TOP + LOW_HOVER), FINGER_CLOSED, "rise", float(end_yaw)),
        Waypoint(2.0, (0.20, 0.30, TOP + HOVER), FINGER_CLOSED, "out of the way", float(end_yaw)),
    ]
    if ONE_ARM:  # the wrong way: the left arm never helps
        left = _stay("left", sum(w.duration for w in left), "stay put (one arm only)")

    flip = np.pi  # the right gripper's fingers turned half a turn for the second half
    right = [
        Waypoint(2.0, (*tcp("right", 0.0, -0.02), TOP + HOVER), FINGER_CLOSED, "above the front face", 0.0),
        Waypoint(1.5, (*tcp("right", 0.0, -0.02), PUSH_Z), FINGER_CLOSED, "down in front of the face", 0.0),
        Waypoint(1.0, (*tcp("right", 0.0, PUSH_IN), PUSH_Z), FINGER_CLOSED, "touch", 0.0),
    ]
    right += [Waypoint(STEP_S, (*tcp("right", y, PUSH_IN), PUSH_Z), FINGER_CLOSED, f"turn {k + 1}", float(y))
              for k, y in enumerate(half1)]
    off = tcp("right", mid_yaw, -0.03)
    right += [
        Waypoint(regrasp[0], (*off, PUSH_Z), FINGER_CLOSED, "back off", float(mid_yaw)),
        Waypoint(regrasp[1], (*off, TOP + LOW_HOVER), FINGER_CLOSED, "up", float(mid_yaw)),
        Waypoint(regrasp[2], (*off, TOP + LOW_HOVER), FINGER_CLOSED, "turn the fingers over", float(mid_yaw + flip)),
        Waypoint(regrasp[3], (*off, PUSH_Z), FINGER_CLOSED, "down", float(mid_yaw + flip)),
        Waypoint(regrasp[4], (*tcp("right", mid_yaw, PUSH_IN), PUSH_Z), FINGER_CLOSED, "touch again", float(mid_yaw + flip)),
    ]
    right += [Waypoint(STEP_S, (*tcp("right", y, PUSH_IN), PUSH_Z), FINGER_CLOSED, f"turn {STEPS + k + 1}",
                       float(y + flip)) for k, y in enumerate(half2)]
    back_r = tcp("right", end_yaw, -0.03)
    right += [
        Waypoint(1.0, (*back_r, PUSH_Z), FINGER_CLOSED, "back off", float(end_yaw + flip)),
        Waypoint(1.5, (*back_r, TOP + LOW_HOVER), FINGER_CLOSED, "rise", float(end_yaw + flip)),
    ]
    # forward push, by the right gripper alone, fingers square to the parcel's back end (closed fingers 9.6 cm wide)
    back_face = CX - PARCEL[1] / 2  # the turned parcel's back end, x
    final_face = BAY_X[1] - PARCEL[1]  # where that end is once the parcel meets the stop bar
    behind = back_face - FINGER_REACH - 0.015
    pushed = (final_face - FINGER_REACH + FWD_LEAD, 0.0, PUSH_Z)
    right += [
        Waypoint(0.8, (*back_r, TOP + HOVER), FINGER_CLOSED, "up, clear of the parcel", float(end_yaw + flip)),
        Waypoint(2.0, (behind, 0.0, TOP + HOVER), FINGER_CLOSED, "round behind the parcel", 0.0),
        Waypoint(1.5, (behind, 0.0, PUSH_Z), FINGER_CLOSED, "down", 0.0),
        Waypoint(0.8, (back_face - FINGER_REACH + PUSH_IN, 0.0, PUSH_Z), FINGER_CLOSED, "touch the back end", 0.0),
        Waypoint(FWD_S, pushed, FINGER_CLOSED, "push it forward to the stop bar", 0.0),
        Waypoint(1.0, pushed, FINGER_CLOSED, "hold", 0.0),
    ]
    if NO_LET_GO:  # scorer check: never let go
        right += [Waypoint(6.0, pushed, FINGER_CLOSED, "keep pushing it against the bar (never let go)", 0.0)]
    else:
        right += [
            Waypoint(1.0, (final_face - FINGER_REACH - 0.03, 0.0, PUSH_Z), FINGER_CLOSED, "back off", 0.0),
            Waypoint(1.2, (final_face - FINGER_REACH - 0.03, 0.0, TOP + HOVER), FINGER_CLOSED, "rise", 0.0),
        ]
    end = max(sum(w.duration for w in right), sum(w.duration for w in left))
    for plan in (left, right):
        if sum(w.duration for w in plan) < end:
            last = plan[-1]
            plan.append(Waypoint(end - sum(w.duration for w in plan), last.pos, last.finger, "wait", last.yaw))
    return {"left": left, "right": right}
