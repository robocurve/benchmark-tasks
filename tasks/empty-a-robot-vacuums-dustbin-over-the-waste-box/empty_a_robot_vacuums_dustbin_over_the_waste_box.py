"""A robot vacuum cleaner stands on the table. Its dustbin, a smoky grey box with a grip fin on its lid, sits in a
shallow seat on the robot's top, full of dust balls. The bin empties through two trap doors in its bottom, which swing
open while the orange release button on the bin's lid is held down and spring shut when it is let go. A waste box
stands on the table in front of the robot.

Task: take the dustbin off the robot and hold it over the waste box; while it is held there, hold its release button
down so the dust falls into the box, without spilling any outside it; then put the bin back in its seat on the robot,
doors shut, and take both arms back to where they started. One hand cannot hold the bin over the box and press its
button at once, and the doors are shut whenever the button is let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=empty_a_robot_vacuums_dustbin_over_the_waste_box.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, hinge,  # noqa: E402
                     on_reset, on_step, rotation, set_hinge_drive, slider)
from yam_arena.constants import FINGER_CLOSED, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the robot vacuum ---------------------------------------------------------------------------------------------------
RV = np.array([0.415, 0.0])  # its middle
RV_R = 0.170
RV_TOP = TOP + 0.060  # its top deck
BS = np.array([0.325, 0.0])  # the bin's seat (the bin's middle) on the deck
SEAT_RIM, SEAT_GAP = 0.010, 0.007  # the seat's rim height, and its clearance round the bin on each side

# -- the dustbin --------------------------------------------------------------------------------------------------------
BIN = (0.100, 0.180, 0.040)  # outside: front to back, side to side, tall
WALL = 0.004
INNER = (BIN[0] - 2 * WALL, BIN[1] - 2 * WALL)
FIN = (0.050, 0.010, 0.022)  # the grip fin on the lid, at y = FIN_Y
FIN_Y = -0.065
BTN_Y, BTN_R, BTN_H, BTN_TRAVEL = 0.072, 0.013, 0.008, 0.005
PRESSED = 0.0015  # the button pushed this far down: the doors open
DOOR = (INNER[0] / 2 - 0.0005, INNER[1] - 0.002, 0.004)  # each trap door
DOOR_SPRING, DOOR_DAMP, DOOR_OPEN = 0.010, 0.0003, 90.0  # N m/deg, N m s/deg, deg
BIN_MASS, DOOR_MASS = 0.15, 0.010
N_DUST, DUST_R, DUST_MASS = 10, 0.0065, 0.002

# -- the waste box ------------------------------------------------------------------------------------------------------
WB = np.array([0.145, 0.0])
WB_IN, WB_H = (0.180, 0.260), 0.060

WHITE, GREY, DARK = (0.90, 0.90, 0.91), (0.55, 0.56, 0.58), (0.10, 0.10, 0.11)
SMOKE, ORANGE, DUST = (0.30, 0.32, 0.35), (0.95, 0.45, 0.08), (0.45, 0.41, 0.36)
BLUE_GREY = (0.45, 0.52, 0.60)

# -- scoring ------------------------------------------------------------------------------------------------------------
IN_BOX_MIN, SPILL_MAX, HALF = 9, 1, 5
SEAT_XY, SEAT_Z, SEAT_TILT, SHUT_DEG = 0.006, 0.004, 5.0, 5.0
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0
BIN_Z = RV_TOP  # the bin's origin: the middle of its bottom


def _robot() -> Body:
    """The robot vacuum: a white disc with a dark bumper, a lidar turret, and the dustbin's seat on its deck; one
    fixed body, origin at its middle on the table."""
    h = RV_TOP - TOP
    s = [CylShape((0.0, 0.0, 0.004), RV_R - 0.006, 0.008, DARK, collide=False),
         CylShape((0.0, 0.0, 0.008 + (h - 0.008) / 2), RV_R, h - 0.008, WHITE),
         CylShape((0.0, 0.0, 0.020), RV_R + 0.0015, 0.022, DARK, collide=False),  # the bumper band
         CylShape((0.080, 0.0, h + 0.011), 0.040, 0.022, DARK),  # the lidar turret
         CylShape((0.080, 0.0, h + 0.0225), 0.036, 0.001, (0.25, 0.26, 0.28), collide=False),
         CylShape((0.135, 0.0, h + 0.0006), 0.009, 0.0012, GREY, collide=False),  # the clean button
         CylShape((0.135, 0.0, h + 0.0013), 0.004, 0.0006, (0.20, 0.70, 0.95), collide=False)]
    # the seat's rim round the bin
    bx, by = BIN[0] / 2 + SEAT_GAP, BIN[1] / 2 + SEAT_GAP
    off = BS - RV
    t = 0.005
    for sx in (-1.0, 1.0):
        s.append(BoxShape((off[0] + sx * (bx + t / 2), off[1], h + SEAT_RIM / 2), (t, 2 * by + 2 * t, SEAT_RIM), GREY))
    for sy in (-1.0, 1.0):
        s.append(BoxShape((off[0], off[1] + sy * (by + t / 2), h + SEAT_RIM / 2), (2 * bx, t, SEAT_RIM), GREY))
    s.append(BoxShape((off[0], off[1], h + 0.0003), (2 * bx, 2 * by, 0.0006), (0.70, 0.71, 0.73), collide=False))
    return Body.build("robot_vacuum", pos=(RV[0], RV[1], TOP), mass=3.0, kinematic=True, shapes=s, color=WHITE)


def _bin() -> Body:
    """The dustbin: four walls and a lid with a grip fin (its bottom is the two trap doors); origin at the middle of
    its bottom."""
    lx, ly, lz = BIN
    s = []
    for sx in (-1.0, 1.0):
        s.append(BoxShape((sx * (lx / 2 - WALL / 2), 0.0, lz / 2), (WALL, ly, lz), SMOKE))
    for sy in (-1.0, 1.0):
        s.append(BoxShape((0.0, sy * (ly / 2 - WALL / 2), lz / 2), (lx - 2 * WALL, WALL, lz), SMOKE))
    s += [BoxShape((0.0, 0.0, lz - WALL / 2), (lx, ly, WALL), SMOKE),
          BoxShape((0.0, FIN_Y, lz + FIN[2] / 2), FIN, DARK),
          BoxShape((0.0, -0.005, lz + 0.0003), (0.060, 0.070, 0.0006), BLUE_GREY, collide=False),  # the filter window
          CylShape((0.0, BTN_Y, lz + 0.0005), BTN_R + 0.003, 0.001, DARK, collide=False)]  # the button's collar
    for k in range(5):  # the filter's ribs
        s.append(BoxShape((-0.024 + k * 0.012, -0.005, lz + 0.0008), (0.003, 0.066, 0.0006), DARK, collide=False))
    return Body.build("dustbin", pos=(BS[0], BS[1], BIN_Z), mass=BIN_MASS, shapes=s, color=SMOKE,
                      com=(0.0, 0.0, lz / 2), friction=(1.0, 0.9))


def _button() -> Body:
    """The orange release button on the bin's lid; origin at its middle."""
    s = [CylShape((0.0, 0.0, 0.0), BTN_R, BTN_H, ORANGE)]
    return Body.build("release_button", pos=(BS[0], BS[1] + BTN_Y, BIN_Z + BIN[2] + BTN_H / 2), mass=0.005,
                      shapes=s, color=ORANGE, friction=(1.0, 0.9))


def _door(name: str, side: float) -> Body:
    """A trap door (side +1: the front one, hinged at the bin's front; -1: the back one); origin at its middle."""
    x = BS[0] + side * DOOR[0] / 2
    s = [BoxShape((0.0, 0.0, 0.0), DOOR, SMOKE)]
    return Body.build(name, pos=(x, BS[1], BIN_Z + DOOR[2] / 2), mass=DOOR_MASS, shapes=s, color=SMOKE,
                      friction=(0.4, 0.3), ignore=(("door_front",) if side < 0 else ()))


def _dust(k: int) -> Body:
    """A dust ball (fluff and grit) lying on the trap doors."""
    x = BS[0] + (0.022 if k % 2 == 0 else -0.022)
    y = BS[1] - 0.064 + (k // 2) * 0.032
    s = [SphereShape((0.0, 0.0, 0.0), DUST_R, DUST)]
    return Body.build(f"dust_{k}", pos=(x, y, BIN_Z + DOOR[2] + DUST_R + 0.001), mass=DUST_MASS, shapes=s,
                      color=DUST, friction=(0.6, 0.5), damping=(0.2, 0.2))


def _waste_box() -> Body:
    """The open waste box: a floor and four walls; origin at its floor's middle on the table."""
    w, (lx, ly), hgt = 0.004, WB_IN, WB_H
    s = [BoxShape((0.0, 0.0, w / 2), (lx + 2 * w, ly + 2 * w, w), (0.20, 0.21, 0.23))]
    for sx in (-1.0, 1.0):
        s.append(BoxShape((sx * (lx / 2 + w / 2), 0.0, hgt / 2), (w, ly + 2 * w, hgt), GREY))
    for sy in (-1.0, 1.0):
        s.append(BoxShape((0.0, sy * (ly / 2 + w / 2), hgt / 2), (lx, w, hgt), GREY))
    return Body.build("waste_box", pos=(WB[0], WB[1], TOP), mass=1.0, kinematic=True, shapes=s, color=GREY)


_HINGE_Z = BIN_Z + DOOR[2] / 2
hinge("door_front_hinge", "dustbin", "door_front", anchor=(float(BS[0] + INNER[0] / 2), float(BS[1]), _HINGE_Z),
      axis="Y", limits_deg=(-95.0, 0.0), spring=DOOR_SPRING, damping=DOOR_DAMP, target_deg=0.0)
hinge("door_back_hinge", "dustbin", "door_back", anchor=(float(BS[0] - INNER[0] / 2), float(BS[1]), _HINGE_Z),
      axis="Y", limits_deg=(0.0, 95.0), spring=DOOR_SPRING, damping=DOOR_DAMP, target_deg=0.0)
slider("button_slide", "dustbin", "release_button", anchor=(float(BS[0]), float(BS[1] + BTN_Y),
                                                            BIN_Z + BIN[2] + BTN_H / 2),
       axis="Z", limits=(-BTN_TRAVEL, 0.0), stiffness=300.0, damping=3.0, target=0.0, max_force=50.0)

OBJECTS = ([_robot(), _bin(), _button(), _door("door_front", 1.0), _door("door_back", -1.0)]
           + [_dust(k) for k in range(N_DUST)] + [_waste_box()])


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"open": False, "pressed_once": False, "tcps": None, "press": 0.0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("BIN_DEBUG")), "steps": 0}


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


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    bp, br = _pose(base, origin, "dustbin")
    up, _ = _pose(base, origin, "release_button")
    rest = bp + br @ np.array([0.0, BTN_Y, BIN[2] + BTN_H / 2])
    press = float((rest - up) @ (br @ np.array([0.0, 0.0, 1.0])))
    _EP["press"] = press
    want = press > PRESSED
    if want != _EP["open"]:
        set_hinge_drive("door_front_hinge", target_deg=-DOOR_OPEN if want else 0.0)
        set_hinge_drive("door_back_hinge", target_deg=DOOR_OPEN if want else 0.0)
        _EP["open"] = want
        _EP["pressed_once"] = _EP["pressed_once"] or want
    _EP["tcps"] = _tcps(base, origin)
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            dust = [_pose(base, origin, f"dust_{k}")[0] for k in range(N_DUST)]
            st = _count(dust, bp, br)
            df, dfr = _pose(base, origin, "door_front")
            tl, tr = _EP["tcps"]
            print(f"[bin] t={_DEBUG['steps'] / 20:5.1f}s press={press * 1000:4.1f}mm open={_EP['open']} "
                  f"bin={np.round(bp * 1000).astype(int).tolist()} tilt={_tilt(br):4.1f} "
                  f"door={_door_angle(br, dfr):5.1f} box={st['box']} inbin={st['bin']} spilled={st['spilled']} "
                  f"L={np.round(tl * 1000).astype(int).tolist()} R={np.round(tr * 1000).astype(int).tolist()} "
                  f"R-fin={np.round((tr - bp - br @ np.array([0.0, FIN_Y, BIN[2] + FIN[2]])) * 1000).astype(int).tolist()}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _tilt(rot) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(rot[2, 2])))))


def _door_angle(bin_rot, door_rot) -> float:
    """How far a door has swung from shut, in degrees."""
    rel = bin_rot.T @ door_rot
    return math.degrees(math.acos(max(-1.0, min(1.0, float(rel[2, 2])))))


def _count(dust, bp, br) -> dict:
    box = inbin = 0
    for p in dust:
        p = np.asarray(p, float)
        if abs(p[0] - WB[0]) < WB_IN[0] / 2 and abs(p[1] - WB[1]) < WB_IN[1] / 2 and p[2] < TOP + WB_H:
            box += 1
            continue
        q = br.T @ (p - np.asarray(bp, float))
        if abs(q[0]) < INNER[0] / 2 + 0.002 and abs(q[1]) < INNER[1] / 2 + 0.002 and -0.002 < q[2] < BIN[2]:
            inbin += 1
    return {"box": box, "bin": inbin, "spilled": len(dust) - box - inbin}


def _status(objects) -> dict:
    b = objects["dustbin"]
    br = rotation(b.quat_xyzw)
    st = _count([objects[f"dust_{k}"].pos for k in range(N_DUST)], b.pos, br)
    bp = np.asarray(b.pos, float)
    st["seated"] = (float(np.linalg.norm(bp[:2] - BS)) < SEAT_XY and abs(bp[2] - BIN_Z) < SEAT_Z
                    and _tilt(br) < SEAT_TILT)
    st["shut"] = all(_door_angle(br, rotation(objects[d].quat_xyzw)) < SHUT_DEG for d in ("door_front", "door_back"))
    st["home"] = _arms_home()
    return st


def _points(st) -> float:
    emptied = st["box"] >= IN_BOX_MIN and st["spilled"] <= SPILL_MAX
    if emptied and st["seated"] and st["shut"] and st["home"]:
        return 1.0
    if emptied:
        return 0.5
    return 0.25 if st["box"] >= HALF else 0.0


def score(objects) -> float:
    """Judged at the end. Emptied: at least 9 of the 10 dust balls inside the waste box and at most 1 anywhere else
    outside the bin (on the table or the robot). Seated: the bin back in its seat on the robot (its middle within 6 mm
    of the seat's, its bottom within 4 mm of the deck, level within 5 degrees) with both trap doors shut (within 5
    degrees). 1.0: emptied, the bin seated with its doors shut, and both arms back where they started (each gripper's
    fingertip centre within 5 cm of its start); 0.5: emptied; 0.25: at least 5 dust balls in the waste box; else
    0.0."""
    return _points(_status(objects))


_HOME: dict = {}


def _arms_home() -> bool:
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
    """Full score with the bin still."""
    return score(objects) >= 1.0 and objects["dustbin"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand comes straight down on the bin's grip fin, fingers closing across it, lifts the bin out of its seat
# and carries it back over the waste box, holding it there. The left hand comes down on the orange release button with
# its fingers closed, leaning outward, and holds it down: the trap doors swing open and the dust falls into the box.
# The left lets go and rises (the doors spring shut) and goes back to where it started; the right carries the bin back
# to the robot, lowers it into its seat, lets go and goes back to where it started.

def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


R_GRIP = grip_rotation(0.0)  # fingers closing along y, across the fin
LEAN = math.radians(20.0)
L_PRESS = _rx(-LEAN) @ grip_rotation(0.0)
FIN_IN_HAND = 0.006  # the right's TCP this far below the fin's top
PRE_FIN = -(FIN[1] / 2 + 0.006)
FINGER_REACH = 0.013  # the closed fingertips this far below the TCP
PRESS_DEPTH = 0.016  # aim this far below the button's top (the arm and the held bin give a little)
OVER_BOX = TOP + WB_H + 0.014  # the bin's bottom this high over the box
R_AIM = np.array([0.0, 0.0, 0.0])
SEAT_AIM = np.array([0.006, 0.006, 0.0])  # the loaded arm falls short when setting the bin back
L_AIM = np.array([-0.004, -0.008, -0.005])  # the held bin hangs low and left of plan
LEAD_S, LEAD_STEPS = 3.0, 15
_IK_CACHE: dict = {}


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    key = (arm, tuple(np.round(np.asarray(pos, float), 5)), tuple(np.round(np.asarray(rot, float), 5).ravel()))
    if key in _IK_CACHE:
        return _IK_CACHE[key]
    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    _IK_CACHE[key] = best[0]
    return best[0]


def _sweep(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _dur(ws) -> float:
    return sum(w.duration for w in ws)


def _fin_tcp(bin_xy, bin_z: float) -> np.ndarray:
    return np.array([bin_xy[0], bin_xy[1] + FIN_Y, bin_z + BIN[2] + FIN[2] - FIN_IN_HAND]) + R_AIM


def _right(release_at: float) -> tuple[list[Waypoint], dict]:
    from yam_arena.constants import READY

    ev = {}
    take = _fin_tcp(BS, BIN_Z)
    out = _sweep("right", np.array(READY, float), _good_ik("right", take + [0.0, 0.0, 0.04], R_GRIP),
                 "over the bin's grip fin", PRE_FIN)
    out += [_w(1.0, take, PRE_FIN, "down round the fin", R_GRIP),
            _w(0.6, take, FINGER_CLOSED, "grip it", R_GRIP),
            _w(1.0, take + [0.0, 0.0, 0.030], FINGER_CLOSED, "lift the bin out of its seat", R_GRIP)]
    over = _fin_tcp(WB, OVER_BOX)
    out += [_w(2.0, over + [0.0, 0.0, 0.01], FINGER_CLOSED, "carry it over the waste box", R_GRIP),
            _w(0.6, over, FINGER_CLOSED, "hold it low over the box", R_GRIP)]
    ev["over_box"] = _dur(out)
    if release_at > _dur(out):
        out.append(_w(release_at - _dur(out), over, FINGER_CLOSED, "hold it there while the dust falls out", R_GRIP))
    back = _fin_tcp(BS, BIN_Z) + SEAT_AIM
    out += [_w(0.8, over + [0.0, 0.0, 0.015], FINGER_CLOSED, "lift it", R_GRIP),
            _w(2.0, back + [0.0, 0.0, 0.025], FINGER_CLOSED, "carry it back over its seat", R_GRIP),
            _w(1.2, back + [0.0, 0.0, 0.002], FINGER_CLOSED, "lower it into its seat", R_GRIP),
            _w(0.4, back + [0.0, 0.0, 0.002], PRE_FIN, "let go", R_GRIP),
            _w(0.6, back + [0.0, 0.0, 0.04], PRE_FIN, "rise", R_GRIP)]
    out += _sweep("right", _good_ik("right", back + [0.0, 0.0, 0.04], R_GRIP), np.array(READY, float),
                  "back to where it started", PRE_FIN)
    return out, ev


def _left(start: float, total: float) -> tuple[list[Waypoint], float]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    top = np.array([WB[0], WB[1] + BTN_Y, OVER_BOX + BIN[2] + BTN_H]) + L_AIM
    touch = top + [0.0, 0.0, FINGER_REACH]
    above = touch + [0.0, 0.0, 0.04]
    p0, r0 = ik.tcp_pose(np.array(READY, float), station.BASES["left"])
    out = []
    if start > 0.0:
        out.append(_w(start, p0, FINGER_CLOSED, "wait at the start", r0))
    out += _sweep("left", np.array(READY, float), _good_ik("left", above, L_PRESS), "over the release button",
                  FINGER_CLOSED)
    out += [_w(0.8, touch + [0.0, 0.0, 0.004], FINGER_CLOSED, "down onto the button", L_PRESS),
            _w(0.6, touch - [0.0, 0.0, PRESS_DEPTH], FINGER_CLOSED, "press it", L_PRESS),
            _w(2.6, touch - [0.0, 0.0, PRESS_DEPTH], FINGER_CLOSED, "hold it down: the dust falls out", L_PRESS),
            _w(0.6, above, FINGER_CLOSED, "let go and rise", L_PRESS)]
    released = _dur(out)
    out += _sweep("left", _good_ik("left", above, L_PRESS), np.array(READY, float), "back to where it started",
                  FINGER_CLOSED)
    if total > _dur(out):
        out.append(_w(total - _dur(out), out[-1].pos, FINGER_CLOSED,
                      "hold still: the trial ends once the full score has held 1 s", out[-1].yaw))
    return out, released


def demo():
    _, ev = _right(0.0)
    lead = LEAD_S + 0.8  # the left arrives over the button as the bin settles over the box
    start = max(ev["over_box"] - lead + 0.4, 0.0)
    _, released = _left(start, 0.0)
    right, _ = _right(released + 0.3)
    total = _dur(right) + 3.0
    right.append(_w(3.0, right[-1].pos, PRE_FIN, "hold still: the trial ends once the full score has held 1 s",
                    right[-1].yaw))
    left, _ = _left(start, total)
    return {"left": left, "right": right}
