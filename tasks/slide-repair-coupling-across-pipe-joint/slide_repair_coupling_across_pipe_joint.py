"""On the station table, a 1 1/2-inch PVC drain line under repair. On the right, an installed pipe held in a clamp; on the
left, a new length of pipe whose far end is pushed home in the socket of a fixed elbow and whose near end rests on a
wooden block, sagging about 1.5 cm below the installed pipe's end. A slip coupling (a repair coupling: a sleeve with
no stop inside) sits on the installed pipe, pushed back from its end.

Task: join the pipes. Lift the new pipe's loose end into line with the installed pipe, slide the coupling along the
installed pipe across the joint until it is centred on it, and let go: the coupling then holds the new pipe in line.
The new pipe pivots in the elbow and cannot slide along its length (it already reaches the installed pipe), so the
coupling cannot be put on it first and pushed on end-first; while its loose end sags, the coupling's mouth meets the
end of the new pipe instead of passing over it. One hand holds the new pipe in line while the other slides the
coupling across, the way a plumber closes a repair.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- installed pipe: fixed, 48.3 mm across, along y at x 0.30 and 10 cm above the table, its end at y -0.020; it rests in
  a clamp on a post at y -0.19.
- new pipe: loose, 48.3 mm across, 25 cm long, 0.19 kg; its near end 2 mm from the installed pipe's end, its far end
  in the elbow's socket (3 cm deep, 1.2 mm of play all round), the pipe tilted 3.5 degrees down to the block.
- block: fixed, pine, 6 x 5 cm, under the new pipe's near end.
- coupling: loose, 70 mm long, its bore 1.2 mm looser than the pipe all round, 5 mm walls, both mouths chamfered
  30 degrees over 6 mm, 0.05 kg; it starts on the installed pipe, centred 9 cm from the joint.

Full score: the coupling centred on the joint (each pipe at least 30 mm inside it), the new pipe in line (its near end
within 4 mm of the installed pipe's axis, its far end still home in the elbow), both hands clear (5 cm or more from the
pipes and the coupling), everything still for 1 s.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, plate  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
# The file keeps the best stage reached itself and a broken rule latches 0, so the score is read at the end of the run.
SCORE_OVER = "end"

# -- the pipe line (world frame) ----------------------------------------------------------------------------------------
PX, PZ = 0.30, TOP + 0.10  # the installed pipe's axis: along y at this x and height
R = 0.02415  # 1 1/2-inch PVC: 48.3 mm outside diameter
CLEAR = 0.0012  # radial play in the coupling's bore and in the elbow's socket
GAP = 0.002  # between the pipe ends
Y_END = -0.020  # the installed pipe's end
Y_JOINT = Y_END + GAP / 2  # the middle of the joint
L_NEW = 0.25
Y_NEAR = Y_END + GAP  # the new pipe's near end, in line
Y_FAR = Y_NEAR + L_NEW  # its far end, home in the elbow's socket
SOCKET = 0.030  # socket depth
Y_STOP = Y_FAR + 0.0005  # the socket's bottom (the new pipe pushed home sits 0.5 mm off it)
Y_PIVOT = Y_FAR - SOCKET / 2  # the middle of the socket: the new pipe tilts about it
DROOP = math.radians(3.5)  # the new pipe's starting tilt, its near end down on the block
NEW_KG, COUPLING_KG = 0.19, 0.05
DEBURR = 0.003  # the new pipe's cut end is deburred: its last 3 mm are 1.5 mm smaller in radius
PVC_MU = (0.45, 0.4)

LC, WALL, CHAMFER = 0.070, 0.005, 0.006  # coupling: length, wall, chamfer length (30 deg)
Y_COUPLING0 = Y_JOINT - 0.091  # the coupling's starting centre on the installed pipe

PVC_C, COUPLING_C, FITTING_C = (0.88, 0.88, 0.86), (0.62, 0.66, 0.72), (0.80, 0.80, 0.78)
CLAMP_C, BLOCK_C = (0.35, 0.36, 0.38), (0.74, 0.58, 0.36)


def _tube(y0: float, y1: float, bore: float, wall: float, color, n: int = 16, chamfer: float = 0.0,
          mouths: tuple[bool, bool] = (True, True)) -> list:
    """Staves of a tube along the body's y axis from y0 to y1: the bore a regular n-gon ``bore`` from the axis to its
    flats, the wall ``wall`` thick; a chamfered mouth (30 deg over ``chamfer``) flares the bore at the ends in
    ``mouths`` (at y0, at y1)."""
    shapes, mid = [], bore + wall / 2
    width = 2 * (bore + wall) * math.tan(math.pi / n) * 1.04
    rise = chamfer * math.tan(math.radians(30))
    a = y0 + (chamfer if mouths[0] else 0.0)
    b = y1 - (chamfer if mouths[1] else 0.0)

    def flare(rh, y_in, y_mouth):
        """A chamfer stave whose inner face runs from the bore at ``y_in`` out to the mouth: no step where it meets
        the straight bore."""
        p1, p2 = rh * bore + [0.0, y_in, 0.0], rh * (bore + rise) + [0.0, y_mouth, 0.0]
        d = (p2 - p1) / np.linalg.norm(p2 - p1)
        out = rh - np.dot(rh, d) * d
        out /= np.linalg.norm(out)
        return plate(p1 + out * wall / 2, p2 + out * wall / 2, width, wall, out, color)

    for k in range(n):
        th = 2 * math.pi * k / n
        rh = np.array([math.cos(th), 0.0, math.sin(th)])
        shapes.append(plate(rh * mid + [0.0, a, 0.0], rh * mid + [0.0, b, 0.0], width, wall, rh, color))
        if chamfer and mouths[0]:
            shapes.append(flare(rh, a, y0))
        if chamfer and mouths[1]:
            shapes.append(flare(rh, b, y1))
    return shapes


def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _quat_x(a: float) -> tuple[float, float, float, float]:
    return (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))


def _new_pipe_pose(tilt: float) -> np.ndarray:
    """The new pipe's centre when it is tilted ``tilt`` (near end down) about the middle of the elbow's socket."""
    pivot = np.array([PX, Y_PIVOT, PZ])
    return pivot - _rot_x(tilt) @ np.array([0.0, Y_PIVOT - (Y_NEAR + L_NEW / 2), 0.0])


def _axis_z(y: float, tilt: float = DROOP) -> float:
    """Height of the new pipe's axis at ``y`` when it is tilted ``tilt`` about the socket's middle."""
    return PZ - (Y_PIVOT - y) * math.tan(tilt)


BLOCK_Y = (0.005, 0.055)
BLOCK_TOP = _axis_z(BLOCK_Y[0]) - R / math.cos(DROOP) - 0.0003  # just under the sagging pipe's lowest point on it

INSTALLED = Body.build(
    "installed_pipe", pos=(PX, Y_END - 0.10, PZ), mass=1.0, fixed=True, friction=PVC_MU, color=PVC_C,
    shapes=[CylShape((0.0, 0.0, 0.0), R, 0.20, PVC_C, axis="Y"),
            BoxShape((0.0, -0.07, 0.0), (2 * R + 0.006, 0.022, 2 * R + 0.006), CLAMP_C, collide=False)])  # clamp band
NEW_PIPE = Body.build(
    "new_pipe", pos=tuple(_new_pipe_pose(DROOP)), mass=NEW_KG, friction=PVC_MU, color=PVC_C, init_quat=_quat_x(DROOP),
    damping=(0.0, 0.2), contact_offset=0.001,
    shapes=[CylShape((0.0, DEBURR / 2, 0.0), R, L_NEW - DEBURR, PVC_C, axis="Y"),  # deburred near end:
            CylShape((0.0, -(L_NEW - DEBURR) / 2, 0.0), R - 0.0015, DEBURR, PVC_C, axis="Y")])  # 1.5 mm smaller
# The elbow: its socket (origin at the socket's middle), the socket's bottom, and the elbow turning down to the floor.
_BEND = 2 * R + 0.012
_LEG_Y = (Y_STOP - Y_PIVOT) + 0.005 + _BEND / 2
ELBOW = Body.build(
    "elbow", pos=(PX, Y_PIVOT, PZ), mass=1.0, fixed=True, friction=PVC_MU, color=FITTING_C, contact_offset=0.001,
    shapes=[*_tube(-SOCKET / 2, Y_STOP - Y_PIVOT, R + CLEAR, 0.005, FITTING_C, mouths=(False, False)),
            CylShape((0.0, Y_STOP - Y_PIVOT + 0.0025, 0.0), R + CLEAR + 0.005, 0.005, FITTING_C, axis="Y"),
            BoxShape((0.0, _LEG_Y, 0.0), (_BEND, _BEND, _BEND), FITTING_C),
            CylShape((0.0, _LEG_Y, -(_BEND / 2 + (PZ - TOP - _BEND / 2) / 2)), R, PZ - TOP - _BEND / 2, PVC_C)])
COUPLING = Body.build(
    "coupling", pos=(PX, Y_COUPLING0, PZ - CLEAR + 0.0002), mass=COUPLING_KG, friction=PVC_MU, color=COUPLING_C,
    damping=(0.0, 0.2), contact_offset=0.001, solver_iterations=24,
    shapes=_tube(-LC / 2, LC / 2, R + CLEAR, WALL, COUPLING_C, chamfer=CHAMFER))
FIXED = [Box("clamp_post", pos=(PX, -0.19, TOP + (PZ - R - TOP) / 2), size=(0.03, 0.03, PZ - R - TOP), color=CLAMP_C),
         Box("block", pos=(PX, sum(BLOCK_Y) / 2, TOP + (BLOCK_TOP - TOP) / 2),
             size=(0.06, BLOCK_Y[1] - BLOCK_Y[0], BLOCK_TOP - TOP), color=BLOCK_C)]
OBJECTS = [*FIXED, INSTALLED, ELBOW, NEW_PIPE, COUPLING]

# -- scoring ------------------------------------------------------------------------------------------------------------
IN_LINE = 0.004  # the new pipe's near end within 4 mm of the installed pipe's axis
CENTRED, OVER = 0.030, 0.010  # each pipe this far inside the coupling: centred / across the joint
HOME, PULLED_OUT = 0.005, 0.020  # far end within 5 mm of the socket's bottom / pulled 2 cm out (a broken rule)
CLEAR_HANDS = 0.05  # each gripper's centre this far from the pipes' and the coupling's axes
STILL_STEPS, STILL_MM = 20, 1.0
_DEBUG = os.environ.get("PIPE_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("PIPE_DEBUG_EVERY", "10"))
_EP: dict = {}
_TCP: dict = {}
_ARM_IDS: dict = {}


@on_reset
def _reset() -> None:
    _EP.update(step=0, tick=0, counted=-1, best=0.0, failed="", recent=[])
    _TCP.clear()


_reset()


@on_step
def _track_grippers(env) -> None:
    """Each gripper's centre (TCP) from its arm's joint positions: "let go" needs it."""
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS

    _EP["tick"] += 1  # one control step: success() and score() both tally it, the history takes it once
    base = getattr(env, "unwrapped", env)
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        if side not in _ARM_IDS:
            _ARM_IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
        q = as_numpy(robot.data.joint_pos)[0]
        _TCP[side] = np.asarray(ik.tcp_pose(q[_ARM_IDS[side]], station.BASES[side])[0], float)


def _segment_distance(p, a, b) -> float:
    ab = b - a
    t = min(1.0, max(0.0, float(np.dot(p - a, ab) / np.dot(ab, ab))))
    return float(np.linalg.norm(p - (a + t * ab)))


def _tally(o) -> dict:
    s, c = o["new_pipe"], o["coupling"]
    axis = s.rotation()[:, 1]
    near, far = s.pos - axis * L_NEW / 2, s.pos + axis * L_NEW / 2
    c_axis = c.rotation()[:, 1]
    c0, c1 = c.pos - c_axis * LC / 2, c.pos + c_axis * LC / 2
    around = bool(math.hypot(c.pos[0] - PX, c.pos[2] - PZ) < 0.006 and abs(c_axis[1]) > math.cos(math.radians(5)))
    t = {
        "off": math.hypot(near[0] - PX, near[2] - PZ),  # near end from the installed pipe's axis
        "dx": near[0] - PX, "dz": near[2] - PZ,
        "tilt": math.degrees(math.acos(min(1.0, abs(axis[1])))),
        "installed_in": Y_END - (c.pos[1] - LC / 2),  # how far each pipe reaches into the coupling
        "new_in": (c.pos[1] + LC / 2) - near[1],
        "around": around,
        "far_out": Y_STOP - far[1],
        "dropped": min(s.low[2], c.low[2]) < TOP + 0.02,
    }
    hands = [_segment_distance(p, *seg) for p in _TCP.values()
             for seg in ((near, far), (c0, c1), (np.array([PX, Y_END - 0.2, PZ]), np.array([PX, Y_END, PZ])))]
    t["hands"] = min(hands) if len(_TCP) == 2 else 0.0
    if t["far_out"] > PULLED_OUT:
        _EP["failed"] = _EP["failed"] or "the new pipe was pulled out of the elbow"
    if t["dropped"]:
        _EP["failed"] = _EP["failed"] or "a pipe or the coupling fell on the table"
    in_line = t["off"] <= IN_LINE and t["tilt"] <= 2.0
    over = around and t["installed_in"] >= OVER and t["new_in"] >= OVER
    centred = around and t["installed_in"] >= CENTRED and t["new_in"] >= CENTRED
    home = t["far_out"] <= HOME
    stage = 0.75 if (centred and in_line and home) else 0.5 if (over and in_line) else 0.25 if in_line else 0.0
    if not _EP["failed"]:
        _EP["best"] = max(_EP["best"], stage)
    recent = _EP["recent"]
    if _EP["counted"] != _EP["tick"]:
        _EP["counted"] = _EP["tick"]
        recent.append(np.r_[near, c.pos])
        del recent[:-STILL_STEPS]
    still = len(recent) == STILL_STEPS and all(np.abs(r - recent[-1]).max() * 1000 < STILL_MM for r in recent)
    t.update(stage=stage, still=still, done=bool(stage >= 0.75 and t["hands"] >= CLEAR_HANDS and still))
    return t


def _debug(o, t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    c = o["coupling"]
    print(f"DBG step={_EP['step']} off={t['off'] * 1000:.1f}mm (dx {t['dx'] * 1000:+.1f}, dz {t['dz'] * 1000:+.1f}) tilt={t['tilt']:.2f} inst_in={t['installed_in'] * 1000:.0f}"
          f" new_in={t['new_in'] * 1000:.0f} around={t['around']} far_out={t['far_out'] * 1000:.1f}mm"
          f" coupling=({c.pos[0]:.3f},{c.pos[1]:.3f},{c.pos[2] - TOP:.3f}) hands={t['hands']:.3f} still={t['still']}"
          f" stage={t['stage']} best={_EP['best']} failed={_EP['failed'] or '-'}", flush=True)
    if len(_TCP) == 2:
        print("DBG   " + " | ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f})" for k, v in _TCP.items()),
              flush=True)


def score(objects) -> float:
    """The best stage reached (0 for the rest of the run once the new pipe is pulled out of the elbow, or a pipe or the
    coupling falls on the table): 0.25 the new pipe's near end held in line; 0.5 the coupling across the joint with
    each pipe at least 10 mm inside it, in line; 0.75 the coupling centred (each pipe at least 30 mm inside), in line,
    far end home. Full score (1.0) is success()."""
    t = _tally(objects)
    if _DEBUG:
        _debug(objects, t)
    return 0.0 if _EP["failed"] else _EP["best"]


def success(objects) -> bool:
    """Coupling centred, new pipe in line and home in the elbow, both hands at least 5 cm clear, still for 1 s."""
    t = _tally(objects)
    return bool(t["done"] and not _EP["failed"])


# -- scripted demo ------------------------------------------------------------------------------------------------------
# The left gripper comes down on the sagging new pipe 10 cm from its far end, closes on it and lifts it into line (the
# pipe turns about the elbow's socket). The right gripper comes down behind the coupling with its fingers astride the
# installed pipe and pushes the coupling's end along it, across the joint, until it is centred. The right gripper backs
# off, then the left lets go; both rise clear.

DOWN_X = {"left": grip_rotation(-math.pi / 2), "right": grip_rotation(math.pi / 2)}  # fingers closing along x
OPEN = -0.0325  # 65 mm between the fingers
CLAMP_PIPE = float(os.environ.get("PIPE_CLAMP", str(-(R - 0.001))))  # 2 mm squeeze on the 48.3 mm pipe
FORK = -0.0265  # 53 mm between the fingers: astride the pipe (48.3 mm), on the coupling's 60.6 mm end
Y_GRIP = 0.10  # where the left hand holds the new pipe
SAG = float(os.environ.get("PIPE_SAG", "0.004"))  # aim this much high: the wrist gives under the pipe's weight
TCP_DZ = 0.005  # TCP above the axis: the fingertips reach 8 mm below it
FORK_DZ = float(os.environ.get("PIPE_FORK_DZ", "0.002"))
PUSH_TO = Y_JOINT - LC / 2 - 0.014 + float(os.environ.get("PIPE_OVERSHOOT", "0.0"))  # fingers' middle at the end
START_BEHIND = Y_COUPLING0 - LC / 2 - 0.013


def _good_ik(arm: str, pos, rot):
    from yam_arena import ik, station

    best = None
    for sd in itertools.product((-0.4, 0.0, 0.4), (0.8, 1.3), (0.6, 1.2), (-0.8, 0.8), (-1.2, 0.0, 1.2),
                                (-1.5, 0.0, 1.5)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(sd, float),
                             base_pos=station.BASES[arm], iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} arm at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0, steps: int = 15) -> list[Waypoint]:
    """From READY along a straight line in joint space (the wrist's big turn stays on one IK branch)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, p, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in p), finger, label, rot)


def _left() -> list[Waypoint]:
    rot = DOWN_X["left"]
    sag = np.array([PX, Y_GRIP, _axis_z(Y_GRIP) + TCP_DZ])
    line = np.array([PX, Y_GRIP, PZ + TCP_DZ + SAG])
    out = _lead_in("left", sag + [0.0, 0.0, 0.08], rot, OPEN, "above the new pipe, fingers across it")
    out += [_w(1.5, sag, OPEN, "down astride the new pipe", rot),
            _w(1.0, sag, CLAMP_PIPE, "close on it", rot),
            _w(2.0, line, CLAMP_PIPE, "lift its loose end into line with the installed pipe", rot),
            _w(5.5, line, CLAMP_PIPE, "hold it in line while the coupling slides across", rot),
            _w(1.0, line, OPEN, "let go: the coupling holds it", rot),
            _w(1.5, line + [0.0, 0.0, 0.08], OPEN, "rise clear", rot),
            _w(3.0, line + [0.0, 0.0, 0.08], OPEN, "wait", rot)]
    return out


def _right() -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    rot = DOWN_X["right"]
    ready, ready_rot = ik.tcp_pose(np.array(READY, float), station.BASES["right"])
    behind = np.array([PX, START_BEHIND, PZ + FORK_DZ])
    pushed = np.array([PX, PUSH_TO, PZ + FORK_DZ])
    out = [Waypoint(3.0, tuple(float(v) for v in ready), FORK, "wait for the left hand", ready_rot)]
    out += _lead_in("right", behind + [0.0, 0.0, 0.08], rot, FORK, "above the installed pipe behind the coupling")
    out += [_w(1.5, behind, FORK, "down, fingers astride the installed pipe behind the coupling", rot),
            _w(3.0, pushed, FORK, "push the coupling across the joint until it is centred", rot),
            _w(0.7, pushed + [0.0, -0.012, 0.0], FORK, "back off", rot),
            _w(1.5, pushed + [0.0, -0.012, 0.08], FORK, "rise clear", rot),
            _w(4.8, pushed + [0.0, -0.012, 0.08], FORK, "wait", rot)]
    return out


def _idle(arm: str, seconds: float) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(seconds, tuple(float(v) for v in pos), OPEN, "stay put", rot)]


def demo():
    """PIPE_ONE_ARM=1 keeps the left arm still, to show that the coupling cannot cross the joint while the pipe sags."""
    if os.environ.get("PIPE_ONE_ARM") == "1":
        return {"left": _idle("left", 18.5), "right": _right()}
    return {"left": _left(), "right": _right()}
