"""On the station table, a gearbox plate, fixed, with two upright shafts 15 cm apart. On the left shaft a large spur
gear (36 teeth) is fitted; a detent holds it half a tooth out of line. On the right shaft, which is keyed (its gear
cannot turn on it), a smaller gear (24 teeth) waits 2.5 cm up, held there by a spring, its teeth on the large gear's.

Task: mesh the gears: turn the large gear by its knob until its teeth line up with the small gear's, hold it there,
and push the small gear down its shaft into mesh; then turn the large gear on, at least 30 degrees, to show the small
gear turning with it, and let go. Let go, the large gear springs back out of line; the
small gear only goes down while the teeth line up, and needs a push: one hand holds the large gear lined up while the
other pushes the small one in.

Run it:  inspect-robots run --embodiment yam_arena -E task=mesh_a_gear_turning_its_partner_to_line_up_the_teeth.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step, rotation,  # noqa: E402
                     set_hinge_drive, set_slider_drive, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the plate (fixed) and its two shafts --------------------------------------------------------------------------------
PX, PY = 0.30, 0.0
PLATE = (0.16, 0.32, 0.010)
PLATE_TOP = TOP + PLATE[2]
AX, AY = PX, PY + 0.075  # the large gear's shaft
BX, BY = PX, PY - 0.075  # the small gear's shaft (keyed)
GEAR_Z = 0.010  # the gears' underside above the plate, in mesh
GEAR_T = 0.012

# -- the large gear (A): 36 teeth, turned by a knob on its hub; a detent springs it to 0 (half a tooth out of line) ----
A_R, A_TEETH = 0.087, 36
A_KNOB_R, A_KNOB_H = 0.012, 0.040
DETENT = 0.03  # N m per degree toward 0
PITCH_A = 360.0 / A_TEETH
LINED_UP_AT = PITCH_A / 2  # its teeth line up with the small gear's at +-5 degrees (and every 10 from there)
LINE_UP_TOL = 3.0  # degrees (the tooth gap's play)

# -- the small gear (B): 24 teeth, keyed on its shaft; slides down it into mesh ----------------------------------------
B_R, B_TEETH = 0.057, 24
B_HUB_R, B_HUB_H = 0.012, 0.025  # a knob on its hub, to take it by
UP = 0.025  # it waits this far up its shaft, on the large gear's teeth
SPRING_K, SPRING_PRE = 150.0, 0.015  # N/m, pushing it up to UP + SPRING_PRE: about 2 N to start it down, 6 N at the bottom
BLOCK_K = 8000.0  # N/m: the teeth not lined up, it is held where it is
ENGAGED = UP - 0.003  # pushed below this, its teeth are in among the large gear's: the large gear can no longer turn
IN_MESH = 0.002  # down to within this of the bottom: in mesh (it clicks in and stays)
STEEL, BRASS, DARK, PLATE_C = (0.70, 0.71, 0.74), (0.78, 0.64, 0.30), (0.15, 0.15, 0.16), (0.30, 0.32, 0.36)

# -- scoring ----------------------------------------------------------------------------------------------------------
DRIVEN = 30.0  # the large gear turned at least this far once in mesh, the small one turning with it
HOME_WITHIN = 0.05
LET_GO = 0.03
HELD_S = 1.0
SUCCESS_HOLD_S = 1.0


def _teeth(r: float, n: int, z: float, color, phase: float = 0.0) -> list:
    out = []
    for k in range(n):
        a = math.radians(phase) + 2 * math.pi * k / n
        out.append(BoxShape((r * math.cos(a), r * math.sin(a), z), (0.010, 0.006, GEAR_T), color, yaw=a, collide=False))
    return out


def _plate() -> Body:
    shapes = [BoxShape((0.0, 0.0, PLATE[2] / 2), PLATE, PLATE_C),
              CylShape((AX - PX, AY - PY, PLATE[2] + GEAR_Z / 2), 0.010, GEAR_Z, STEEL),  # collars under the gears
              CylShape((BX - PX, BY - PY, PLATE[2] + GEAR_Z / 2), 0.010, GEAR_Z, STEEL),
              CylShape((BX - PX, BY - PY, PLATE[2] + (GEAR_Z + UP + GEAR_T + 0.012) / 2), 0.005, GEAR_Z + UP + GEAR_T + 0.012,
                       STEEL, collide=False)]  # the small gear's shaft, sticking up through it
    return Body.build("plate", pos=(PX, PY, TOP), mass=3.0, shapes=shapes, kinematic=True, friction=(0.5, 0.4),
                      color=PLATE_C)


def _gear_a() -> Body:
    z = GEAR_Z + GEAR_T / 2
    shapes = [CylShape((0.0, 0.0, z), A_R - 0.004, GEAR_T, BRASS), *_teeth(A_R + 0.001, A_TEETH, z, BRASS),
              CylShape((0.0, 0.0, GEAR_Z + GEAR_T + A_KNOB_H / 2), A_KNOB_R, A_KNOB_H, DARK),
              BoxShape((0.0, A_R * 0.55, GEAR_Z + GEAR_T + 0.0005), (0.006, A_R * 0.6, 0.001), DARK, collide=False)]  # a mark
    return Body.build("big_gear", pos=(AX, AY, PLATE_TOP), mass=0.4, shapes=shapes, friction=(1.2, 1.0), color=BRASS,
                      ignore=("plate",))


def _gear_b() -> Body:
    z = GEAR_Z + UP + GEAR_T / 2
    shapes = [CylShape((0.0, 0.0, z), B_R - 0.004, GEAR_T, STEEL),
              CylShape((0.0, 0.0, GEAR_Z + UP + GEAR_T + B_HUB_H / 2), B_HUB_R, B_HUB_H, STEEL)]
    return Body.build("small_gear", pos=(BX, BY, PLATE_TOP), mass=0.15, shapes=shapes, friction=(1.0, 0.9), color=STEEL,
                      ignore=("plate", "big_gear"))


def _small_teeth() -> Body:
    """The small gear's teeth and a mark on its face, shown only: the task file moves them with the small gear and,
    once it is in mesh, turns them with the large gear (36 to 24: half as fast again, the other way)."""
    z = GEAR_Z + UP + GEAR_T / 2
    shapes = [*_teeth(B_R + 0.001, B_TEETH, z, STEEL, phase=0.0),  # a tooth facing the large gear: out of line at its 0
              BoxShape((0.0, -B_R * 0.55, GEAR_Z + UP + GEAR_T + 0.0005), (0.006, B_R * 0.6, 0.001), DARK,
                       collide=False)]  # a mark
    return Body.build("small_teeth", pos=(BX, BY, PLATE_TOP), mass=0.01, shapes=shapes, kinematic=True, color=STEEL)


PLATE_B, GEAR_A, GEAR_B = _plate(), _gear_a(), _gear_b()
OBJECTS = [PLATE_B, GEAR_A, GEAR_B, _small_teeth()]
hinge("big_shaft", "plate", "big_gear", anchor=(AX, AY, PLATE_TOP), axis="Z", spring=DETENT, target_deg=0.0,
      damping=0.002)
slider("small_shaft", "plate", "small_gear", anchor=(BX, BY, PLATE_TOP), axis="Z", limits=(-UP, 0.0),
       stiffness=BLOCK_K, damping=20.0, target=0.0, max_force=300.0)  # 0 = waiting up; -UP = in mesh


# -- followed after every control step: lining up, the small gear's way down, the mesh --------------------------------------

def _fresh() -> dict:
    return {"state": "waiting", "lined_steps": 0, "held_lined": False, "tcps": None, "phi_mesh": None, "driven": 0.0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("GEAR_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _turn_deg(rot) -> float:
    return math.degrees(math.atan2(float(rot[1, 0]), float(rot[0, 0])))


def _off_line(phi: float) -> float:
    """How far (degrees) the large gear is from lining up its teeth with the small gear's."""
    return abs((phi - LINED_UP_AT + PITCH_A / 2) % PITCH_A - PITCH_A / 2)


def _place_teeth(base, origin, pos, turn_deg) -> None:
    import torch

    a = math.radians(turn_deg)
    asset = base.scene["small_teeth"]
    p = np.asarray(pos, float) + origin
    asset.write_root_pose_to_sim(torch.tensor([[*p.tolist(), 0.0, 0.0, math.sin(a / 2), math.cos(a / 2)]],
                                              dtype=torch.float32, device=asset.device))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    a = as_numpy(base.scene["big_gear"].data.root_link_pose_w)[0]
    b = as_numpy(base.scene["small_gear"].data.root_link_pose_w)[0]
    phi = _turn_deg(rotation(a[3:7]))
    down = float(PLATE_TOP - (b[2] - origin[2]))  # how far the small gear has been pushed down (0 waiting, UP in mesh)
    _EP["tcps"] = _tcps(base, origin)
    lined = _off_line(phi) <= LINE_UP_TOL
    _EP["lined_steps"] = _EP["lined_steps"] + 1 if lined else 0
    if _EP["lined_steps"] >= round(HELD_S * 20):
        _EP["held_lined"] = True
    state = _EP["state"]
    if state in ("waiting", "free") and down >= UP - IN_MESH:  # all the way down: it clicks in and stays
        set_slider_drive("small_shaft", target=-UP, stiffness=3000.0)
        set_hinge_drive("big_shaft", target_deg=phi, spring=0.0, damping=0.004)  # in mesh: the pair turns freely
        state = "meshed"
        _EP["phi_mesh"] = phi
    elif state == "waiting" and lined:  # the teeth line up: it can go down, against its spring
        set_slider_drive("small_shaft", target=SPRING_PRE, stiffness=SPRING_K)
        state = "free"
    elif state == "free" and not lined and down < UP - ENGAGED:  # out of line again before it got in: held again
        set_slider_drive("small_shaft", target=-down, stiffness=BLOCK_K)
        state = "waiting"
    if state == "free" and down >= UP - ENGAGED and not _EP.get("a_locked"):  # in among the teeth: the large gear holds
        set_hinge_drive("big_shaft", target_deg=phi, spring=5.0)
        _EP["a_locked"] = True
    _EP["state"] = state
    turn_b = 0.0
    if _EP["phi_mesh"] is not None:  # in mesh: the small gear turns with the large one
        d = (phi - _EP["phi_mesh"] + 180.0) % 360.0 - 180.0
        _EP["driven"] = max(_EP["driven"], abs(d))
        turn_b = -d * A_TEETH / B_TEETH
    _place_teeth(base, origin, b[:3] - origin, turn_b)
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status_from(down)
            tl, tr = _EP["tcps"]
            print(f"[gear] t={_DEBUG['steps'] / 20:5.1f}s big_gear={phi:6.2f}deg off_line={_off_line(phi):4.2f} "
                  f"small_down={down * 1000:5.1f}mm state={state} held_lined={_EP['held_lined']} let_go={st['let_go']} "
                  f"driven={_EP['driven']:5.1f}deg small_turn={turn_b:6.1f}deg "
                  f"left TCP-knob={np.round((tl - np.array([AX, AY, PLATE_TOP + GEAR_Z + GEAR_T + A_KNOB_H / 2])) * 1000).astype(int).tolist()} "
                  f"right TCP={np.round(tr * 1000).astype(int).tolist()} score={_points(st)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status_from(down: float) -> dict:
    let_go = True
    if _EP["tcps"] is not None:
        knob = np.array([AX, AY, PLATE_TOP + GEAR_Z + GEAR_T + A_KNOB_H / 2])
        hub = np.array([BX, BY, PLATE_TOP + GEAR_Z + UP - down + GEAR_T + B_HUB_H / 2])
        for t in _EP["tcps"]:
            let_go &= float(np.linalg.norm(t - knob)) >= LET_GO and float(np.linalg.norm(t - hub)) >= LET_GO
    return {"meshed": _EP["state"] == "meshed", "let_go": let_go,
            "tried": _EP["held_lined"] or down >= UP - ENGAGED, "driven": _EP["driven"] >= DRIVEN}


def _status(objects) -> dict:
    return _status_from(float(PLATE_TOP - objects["small_gear"].pos[2]))


def _points(st) -> float:
    if st["meshed"] and st["driven"] and st["let_go"]:
        return 1.0
    if st["meshed"]:
        return 0.5
    return 0.25 if st["tried"] else 0.0


def score(objects) -> float:
    """1.0 with the small gear pushed all the way down into mesh, the large gear then turned at least 30 degrees (the
    small one turning with it), both grippers let go (3 cm clear of the large gear's knob and the small gear's hub) and
    both arms back where they started; 0.5 with it in mesh; 0.25 if the large gear was held lined up for 1 s, or the
    small gear got in among its teeth, at some point; else 0.0."""
    pts = _points(_status(objects))
    if pts >= 1.0 and not _arms_home():
        return 0.5  # in mesh and driven, but the arms not back
    return pts


_HOME: dict = {}


def _arms_home() -> bool:
    """Both arms back where they started (each TCP within HOME_WITHIN of its start)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    tcps = _EP.get("tcps")
    if tcps is None:
        return False
    if "tcps" not in _HOME:
        _HOME["tcps"] = [np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[a])[0], float)
                         for a in ("left", "right")]
    return all(float(np.linalg.norm(t - h)) < HOME_WITHIN for t, h in zip(tcps, _HOME["tcps"]))


def success(objects) -> bool:
    """Full score with both gears still: the trial ends there."""
    return score(objects) >= 1.0 and objects["big_gear"].resting() and objects["small_gear"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the large gear's knob, grips it and turns it 5 degrees counterclockwise, lining
# its teeth up with the small gear's, and holds it. The right hand comes straight down on the small gear's knob, grips it
# and pushes it down its shaft into mesh, lets go and goes back to where it started. Then the left turns the large gear
# on 60 degrees, the small gear turning with it (the other way, half as fast again), lets go and goes back to where it
# started.


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


KNOB_GRIP = grip_rotation(-math.pi / 2)  # straight down, fingers closing along x
PUSH_GRIP = grip_rotation(math.pi / 2)  # straight down, fingers closing along x (the right hand's way round)
KNOB_IN_HAND = 0.008  # the left TCP this far above the knob's middle
TURN = math.radians(LINED_UP_AT - 1.5)  # the wrist turns the knob ~2.7 degrees further than it aims
PUSH = 0.020  # push the knob this far past the bottom (the shaft stops it; the arm gives against the spring)
LEAD_S, LEAD_STEPS = 3.0, 15


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed), base_pos=station.BASES[arm],
                             iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


TURNED_AT = LEAD_S + 1.0 + 0.8 + 1.2
DRIVE = math.radians(60.0)  # once in mesh, the left turns the large gear on this far


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    """From a pose back to the arm's start, straight through joint space (the lead-in, reversed)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = _good_ik(arm, pos, rot), np.array(READY, float)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, "back to where it started", r))
    return out


def _left(drive_at: float, total: float) -> list[Waypoint]:
    knob = np.array([AX, AY, PLATE_TOP + GEAR_Z + GEAR_T + A_KNOB_H / 2 + KNOB_IN_HAND])
    out = _lead_in("left", knob + [0.0, 0.0, 0.05], KNOB_GRIP, "over the large gear's knob", FINGER_OPEN)
    out += [_w(1.0, knob, FINGER_OPEN, "down round the knob", KNOB_GRIP),
            _w(0.8, knob, FINGER_CLOSED, "grip it", KNOB_GRIP),
            _w(1.2, knob, FINGER_CLOSED, "turn it 5 degrees: the teeth line up", _rz(TURN) @ KNOB_GRIP)]
    t = sum(w.duration for w in out)
    out.append(_w(max(drive_at - t, 0.3), knob, FINGER_CLOSED, "hold it there", _rz(TURN) @ KNOB_GRIP))
    for k in range(1, 5):
        out.append(_w(0.75, knob, FINGER_CLOSED, "turn it on: the small gear turns with it",
                      _rz(TURN + DRIVE * k / 4) @ KNOB_GRIP))
    end = _rz(TURN + DRIVE) @ KNOB_GRIP
    out += [_w(0.5, knob, FINGER_OPEN, "let go", end),
            _w(1.0, knob + [0.0, 0.0, 0.05], FINGER_OPEN, "rise", end)]
    out += _go_home("left", knob + [0.0, 0.0, 0.05], end, FINGER_OPEN)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out


def _right() -> tuple[list[Waypoint], float]:
    knob = np.array([BX, BY, PLATE_TOP + GEAR_Z + UP + GEAR_T + B_HUB_H / 2 + 0.005])
    down = knob - [0.0, 0.0, UP + PUSH]
    out = _lead_in("right", knob + [0.0, 0.0, 0.05], PUSH_GRIP, "over the small gear's knob", FINGER_OPEN)
    out += [_w(1.0, knob, FINGER_OPEN, "down round the knob", PUSH_GRIP),
            _w(0.8, knob, FINGER_CLOSED, "grip it", PUSH_GRIP)]
    t = sum(w.duration for w in out)
    out += [_w(max(TURNED_AT - t, 0.3) + 0.5, knob, FINGER_CLOSED, "wait for the teeth to be lined up", PUSH_GRIP),
            _w(2.0, down, FINGER_CLOSED, "push it down its shaft into mesh", PUSH_GRIP),
            _w(0.5, down, FINGER_CLOSED, "it clicks in", PUSH_GRIP),
            _w(0.5, down, FINGER_OPEN, "let go", PUSH_GRIP),
            _w(1.0, down + [0.0, 0.0, 0.05], FINGER_OPEN, "rise", PUSH_GRIP)]
    drive_at = sum(w.duration for w in out)
    out += _go_home("right", down + [0.0, 0.0, 0.05], PUSH_GRIP, FINGER_OPEN)
    return out, drive_at


def demo():
    right, drive_at = _right()
    total = max(sum(w.duration for w in right), drive_at + 3.0 + 1.5 + LEAD_S) + 3.0
    right.append(_w(total - sum(w.duration for w in right), right[-1].pos, FINGER_OPEN, "wait at the start",
                    right[-1].yaw))
    return {"left": _left(drive_at, total), "right": right}
