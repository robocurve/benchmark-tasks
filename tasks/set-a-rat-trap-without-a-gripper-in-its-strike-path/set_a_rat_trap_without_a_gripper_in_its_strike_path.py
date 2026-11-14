"""On the station table, a large wooden snap trap for rats is screwed to a pine plank for safe handling practice. Its
steel hammer lies sprung forward over the trap's far half. Its lock bar stands up at the near end, leaning back against
its stop.

Task: set the trap, its lock bar holding the hammer back, without ever letting the hammer snap and without a gripper
in the hammer's strike path while nothing holds it back (a hand on the hammer, or the latched lock bar). Both grippers
end clear of the set trap and both arms back where they started. A gripper over the trigger pedal of the set trap
springs it.

One way (the scripted demo): one hand pulls the hammer back over the top, against its strong spring, to the trap's
near end, and holds it there; the other swings the lock bar forward over the hammer and presses its tip down into the
catch; the hand on the hammer lets go and the hammer settles against the lock bar.

Why both hands: the hammer springs forward as soon as it is let go, and only a latched lock bar holds it.

Run it:  inspect-robots run --embodiment yam_arena -E task=set_a_rat_trap_without_a_gripper_in_its_strike_path.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step,  # noqa: E402
                     rotation, segment, set_hinge_drive)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the plank and the trap: the trap along x, its near end toward the arms ------------------------------------------
XC = 0.27  # the hammer's axle
PLANK_T = 0.015
BOARD = (0.140, 0.365, 0.090, 0.011)  # the trap's board: near end, far end (x), width, thickness
BASE_TOP = TOP + PLANK_T + BOARD[3]
AXLE_Z = BASE_TOP + 0.015  # the axle, on its brackets
HAMMER_R = 0.075  # the hammer's cross bar, from the axle
HAMMER_W = 0.080  # between its side arms
BAR_R = 0.005  # the cross bar's radius (10 mm steel)
SPRING = 0.0017  # N m per degree, toward sprung: about 0.3 N m with the hammer pulled right back (PhysX wraps a drive's
# error past 180 degrees, so the drive's target stays at 0 and the travel short of 180)
COCKED = 175.0  # the hammer's travel, from sprung forward (0) over the top to the near end

# -- the lock bar: on a staple post at the near end, slightly left of the middle ---------------------------------------
XH, YL = 0.150, 0.015  # its pin
ZH = BASE_TOP + 0.031  # at the height where it lies level over the pulled-back hammer
LOCK_LEN = XC + 0.005 - XH  # pin to tip: the tip reaches the catch, just past the axle
LOCK_T = 0.008  # 8 mm square steel
LOCK_REST = 110.0  # it starts standing up, leaning back against its stop (degrees up from lying forward)
CATCH = np.array([XH + LOCK_LEN, YL, ZH])  # where the catch takes the lock bar's tip
CATCH_WITHIN = 0.008  # the tip within this of the catch, the hammer pulled back: latched
LATCH_SPRING = 0.3  # N m per degree: a latched lock bar is held where it is

# -- scoring --------------------------------------------------------------------------------------------------------
PULLED_BACK = 165.0  # the hammer this far back: pulled back (and, with the lock bar latched, the trap set)
SET_AT = 160.0
SNAP_SPEED = 200.0  # deg/s: the hammer swinging forward this fast (pulled back past 30 degrees): it snapped
ZONE = (0.025, 0.095, 0.055)  # the strike path: this far from the axle (min, max) and across the trap (half-width)
ZONE_MARGIN = 15.0  # ... ahead of the hammer by more than this many degrees
HOLDING = 0.035  # a TCP within this of the cross bar holds the hammer back (as does the latched lock bar)
PEDAL = (0.295, 0.355, 0.030, BASE_TOP + 0.025)  # over the trigger pedal (x from, x to, half-width, up to z): a
# gripper there springs the set trap
LET_GO = 0.04
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

PINE, STEEL, BRASS, YELLOW, RUBBER, DARK = (0.86, 0.74, 0.52), (0.62, 0.64, 0.67), (0.80, 0.65, 0.25), \
    (0.95, 0.80, 0.15), (0.12, 0.12, 0.13), (0.25, 0.26, 0.28)


def _quat_y(deg: float) -> tuple[float, float, float, float]:
    a = math.radians(deg)
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


def _base() -> Body:
    """The plank, the trap's board, the axle brackets, the lock bar's staple post, the catch and the trigger pedal
    (fixed); origin on the table under the axle."""
    bx0, bx1, bw, bt = BOARD
    zb = BASE_TOP - TOP
    shapes = [BoxShape((0.0, 0.0, PLANK_T / 2), (0.32, 0.16, PLANK_T), (0.70, 0.55, 0.35)),
              BoxShape(((bx0 + bx1) / 2 - XC, 0.0, PLANK_T + bt / 2), (bx1 - bx0, bw, bt), PINE)]
    for s in (1.0, -1.0):  # the axle's brackets and coil springs
        shapes += [BoxShape((0.0, s * 0.047, zb + 0.008), (0.012, 0.003, 0.016), STEEL, collide=False),
                   CylShape((0.0, s * 0.030, AXLE_Z - TOP), 0.007, 0.018, STEEL, collide=False, axis="Y")]
    shapes += [BoxShape((XH - XC, YL, (zb + ZH - TOP) / 2), (0.008, 0.012, ZH - BASE_TOP), STEEL, collide=False),
               BoxShape((CATCH[0] - XC + 0.002, YL + 0.009, zb + 0.014), (0.010, 0.006, 0.028), BRASS, collide=False),
               BoxShape((CATCH[0] - XC - 0.001, YL + 0.004, ZH - TOP + 0.007), (0.012, 0.010, 0.003), BRASS,
                        collide=False),  # the catch's lip
               BoxShape((0.050, 0.0, zb + 0.002), (0.055, 0.050, 0.004), YELLOW, collide=False),  # the trigger pedal
               BoxShape((HAMMER_R, 0.0, zb + 0.001), (0.020, 0.07, 0.002), RUBBER, collide=False)]
    return Body.build("trap_base", pos=(XC, 0.0, TOP), mass=2.0, shapes=shapes, kinematic=True, color=PINE,
                      friction=(0.7, 0.6))


def _hammer() -> Body:
    """The U-shaped steel hammer, lying sprung forward; origin on the axle."""
    shapes = [segment((0.0, s * HAMMER_W / 2, 0.0), (HAMMER_R, s * HAMMER_W / 2, 0.0), 0.006, STEEL)
              for s in (1.0, -1.0)]
    shapes += [CylShape((HAMMER_R, 0.0, 0.0), BAR_R, HAMMER_W + 0.012, STEEL, axis="Y"),
               CylShape((0.0, 0.0, 0.0), 0.0025, HAMMER_W + 0.014, STEEL, collide=False, axis="Y")]
    return Body.build("hammer", pos=(XC, 0.0, AXLE_Z), mass=0.05, shapes=shapes, color=STEEL, friction=(0.3, 0.25),
                      com=(HAMMER_R * 0.6, 0.0, 0.0))


def _lock_bar() -> Body:
    """The lock bar, standing up leaning back against its stop; origin on its pin."""
    shapes = [BoxShape((LOCK_LEN / 2 + 0.002, 0.0, 0.0), (LOCK_LEN - 0.004, LOCK_T, LOCK_T), STEEL),
              BoxShape((LOCK_LEN - 0.002, 0.0, -0.004), (0.006, LOCK_T, 0.006), STEEL),  # the tip's hook
              CylShape((0.0, 0.0, 0.0), 0.006, 0.012, STEEL, collide=False, axis="Y")]  # its eye round the pin
    return Body.build("lock_bar", pos=(XH, YL, ZH), mass=0.02, shapes=shapes, color=STEEL, friction=(0.8, 0.7),
                      start_xyzw=_quat_y(-LOCK_REST), com=(LOCK_LEN / 2, 0.0, 0.0))


OBJECTS = [_base(), _hammer(), _lock_bar()]
# the hammer turns about +y the negative way as it is pulled back (its cross bar going up and over toward -x)
hinge("hammer_axle", "trap_base", "hammer", anchor=(XC, 0.0, AXLE_Z), axis="Y", limits_deg=(-COCKED, 1.0),
      spring=SPRING, damping=2e-6, target_deg=0.0)
hinge("lock_bar_pin", "trap_base", "lock_bar", anchor=(XH, YL, ZH), axis="Y", limits_deg=(-112.0, 4.0),
      spring=0.0, damping=1e-5, target_deg=0.0)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"latched": False, "snapped": False, "unsafe": False, "pulled": False, "tcps": None, "s": {}, "steps": 0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("TRAP_DEBUG"))}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    set_hinge_drive("lock_bar_pin", target_deg=0.0, spring=0.0, damping=1e-5)


def _grippers(base, origin) -> list[tuple[np.ndarray, np.ndarray]]:
    """Each arm's (TCP, fingertips' middle)."""
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        r = rotation(g[3:7])
        tcp = g[:3] - origin + r @ np.array(TCP_OFFSET)
        out.append((tcp, tcp + r[:, 2] * 0.013))
    return out


def _turn_about_y(r: np.ndarray) -> float:
    return math.degrees(math.atan2(float(r[0, 2]), float(r[0, 0])))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    h = base.scene["hammer"].data
    hp = as_numpy(h.root_link_pose_w)[0]
    back = -_turn_about_y(rotation(hp[3:7]))  # 0: sprung forward; 180: pulled right back
    if back < -20.0:
        back += 360.0
    speed = float(as_numpy(h.root_ang_vel_w)[0][1])  # deg/s toward sprung is positive about +y
    speed = math.degrees(speed)
    lp = as_numpy(base.scene["lock_bar"].data.root_link_pose_w)[0]
    lrot = rotation(lp[3:7])
    up = -_turn_about_y(lrot)  # the lock bar: 0 lying forward, 110 leaning back
    tip = lp[:3] - origin + lrot @ np.array([LOCK_LEN, 0.0, 0.0])
    bar = hp[:3] - origin + rotation(hp[3:7]) @ np.array([HAMMER_R, 0.0, 0.0])  # the cross bar's middle
    if back >= PULLED_BACK:
        _EP["pulled"] = True
    if not _EP["latched"] and back >= PULLED_BACK and float(np.linalg.norm(tip - CATCH)) <= CATCH_WITHIN:
        _EP["latched"] = True  # the catch takes the tip: the lock bar holds the hammer back
        set_hinge_drive("lock_bar_pin", target_deg=-up, spring=LATCH_SPRING, damping=0.002)
    if back >= 30.0 and speed > SNAP_SPEED:
        _EP["snapped"] = True
    grips = _grippers(base, origin)
    _EP["tcps"] = [g[0] for g in grips]
    held = _EP["latched"] or any(abs(float(t[1] - bar[1])) <= HAMMER_W / 2 + 0.01 and math.hypot(
        float(t[0] - bar[0]), float(t[2] - bar[2])) <= HOLDING for t, _ in grips)
    if _EP["latched"] and any(PEDAL[0] <= float(p[0]) <= PEDAL[1] and abs(float(p[1])) <= PEDAL[2]
                              and float(p[2]) <= PEDAL[3] for pts in grips for p in pts):
        _EP["snapped"] = True  # a gripper on the trigger pedal: the catch lets go and the hammer snaps forward
        _EP["latched"] = False
        set_hinge_drive("lock_bar_pin", spring=0.0, damping=1e-5)
    if back >= 30.0 and not held:  # nothing holds the hammer back: no gripper may be in its strike path
        for pts in grips:
            for p in pts:
                dx, dy, dz = float(p[0] - XC), float(p[1]), float(p[2] - AXLE_Z)
                rho = math.hypot(dx, dz)
                ahead = math.degrees(math.atan2(dz, dx))
                if ZONE[0] <= rho <= ZONE[1] and abs(dy) <= ZONE[2] and -5.0 <= ahead <= back - ZONE_MARGIN:
                    _EP["unsafe"] = True
    clear = all(float(np.linalg.norm(t - q)) >= LET_GO for t in _EP["tcps"] for q in (bar, tip))
    _EP["s"] = {"back": back, "up": up, "clear": clear, "held": held}
    _EP["steps"] += 1
    if _DEBUG["on"] and _EP["steps"] % 10 == 0:
        (tl, _), (tr, _) = grips
        print(f"[trap] t={_EP['steps'] / 20:5.1f}s hammer={back:6.1f}deg speed={speed:7.1f} lock={up:6.1f}deg "
              f"tip-catch={np.round((tip - CATCH) * 1000).astype(int).tolist()} latched={_EP['latched']} "
              f"held={held} snapped={_EP['snapped']} unsafe={_EP['unsafe']} clear={clear} "
              f"rTCP-bar={np.round((tr - bar) * 1000).astype(int).tolist()} "
              f"lTCP-tip={np.round((tl - tip) * 1000).astype(int).tolist()} score={score(None)}",
              file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 if the hammer ever snapped forward (swinging forward faster than 200 degrees a second once pulled back past
    30 degrees; a gripper over the trigger pedal of the set trap springs it), or if a gripper's TCP or fingertips were
    ever in its strike path (2.5 to 9.5 cm from the axle, within 5.5 cm of the trap's middle line, more than 15
    degrees ahead of the hammer) while nothing held the hammer back (no TCP within 3.5 cm of its cross bar, and the
    lock bar not yet latched). Else 1.0 with the trap set (the lock bar's tip latched in the catch, the hammer held at
    least 160 degrees back), both hands let go (each TCP at least 4 cm from the hammer's cross bar and the lock bar's
    tip) and both arms back where they started; 0.5 with the trap set; 0.25 if the hammer was pulled back 165 degrees
    at some point; else 0.0."""
    s = _EP["s"]
    if _EP["snapped"] or _EP["unsafe"] or not s:
        return 0.0
    if _EP["latched"] and s["back"] >= SET_AT:
        if s["clear"] and _arms_home():
            return 1.0
        return 0.5
    return 0.25 if _EP["pulled"] else 0.0


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
    """Full score with the hammer and the lock bar still: the trial ends there."""
    return score(objects) >= 1.0 and objects["hammer"].resting() and objects["lock_bar"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand, leaning out to the right, comes down round the hammer's cross bar near its right end and pinches it
# lightly. It carries the cross bar up and over the top to the trap's near end, the bar turning in its fingers, and
# holds it down there. The left, leaning out to the left, pinches the lock bar near its tip and swings it forward over
# the hammer, pressing its tip down into the catch. It lets go and withdraws to the left, out of the trap. The right
# lets go, the hammer settling against the lock bar, and rises. Both arms go back to where they started.

BAR_Y = -0.030  # the right hand holds the cross bar here
LEAN = 30.0
RIGHT_GRIP = None  # set below (needs numpy rotations)
PINCH = -0.003  # a light pinch: holds the bar, lets it turn
FORK = -(BAR_R + 0.006)
LOCK_AT = LOCK_LEN - 0.012  # the left hand holds the lock bar here, along it
LOCK_PINCH = -0.002
LEAD_S, LEAD_STEPS = 3.0, 15


def _rx(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])


RIGHT_GRIP = _rx(LEAN) @ grip_rotation(math.pi / 2)  # fingers closing along x, across the bar; leaning out to the right
LEFT_GRIP = _rx(-LEAN) @ grip_rotation(0.0)  # fingers closing along y, across the lock bar; leaning out to the left


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

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
    return best[0]


def _joint_path(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, _good_ik(arm, pos, rot), np.array(READY, float), "back to where it started", finger)


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _on_bar(back: float, r: float = HAMMER_R) -> np.ndarray:
    a = math.radians(back)
    return np.array([XC + r * math.cos(a), BAR_Y, AXLE_Z + r * math.sin(a)])


def _on_lock(up: float, along: float = LOCK_AT, press: float = 0.0) -> np.ndarray:
    a = math.radians(up)
    return np.array([XH + along * math.cos(a), YL, ZH + along * math.sin(a) - press])


def _right(left_out: float) -> tuple[list[Waypoint], float]:
    out = _lead_in("right", _on_bar(0.0) + [0.0, 0.0, 0.05], RIGHT_GRIP, "over the hammer's cross bar", FORK)
    out += [_w(1.0, _on_bar(0.0), FORK, "down round the cross bar", RIGHT_GRIP),
            _w(0.5, _on_bar(0.0), PINCH, "pinch it lightly", RIGHT_GRIP)]
    for k in range(1, 13):
        back = COCKED * k / 12
        out.append(_w(0.45, _on_bar(back, HAMMER_R - (0.004 if k == 12 else 0.0)) + [0.0, 0.0, -0.006 * (k == 12)],
                      PINCH, "pull the hammer up and over to the near end", RIGHT_GRIP))
    cocked_at = sum(w.duration for w in out)
    held = _on_bar(COCKED, HAMMER_R - 0.004) + [0.0, 0.0, -0.006]
    out.append(_w(max(left_out + 0.5 - cocked_at, 0.5), held, PINCH, "hold the hammer back", RIGHT_GRIP))
    away = held + [0.0, 0.0, 0.07]
    out += [_w(0.6, held, FORK, "let go: the hammer settles against the lock bar", RIGHT_GRIP),
            _w(1.0, away, FORK, "rise, away from the trigger pedal", RIGHT_GRIP)]
    out += _go_home("right", away, RIGHT_GRIP, FORK)
    return out, cocked_at


def _left(cocked_at: float) -> tuple[list[Waypoint], float]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    p0, r0 = ik.tcp_pose(np.array(READY, float), station.BASES["left"])
    out = [_w(cocked_at + 0.3, p0, FINGER_OPEN, "wait for the hammer to be pulled back", r0)]
    lead = _lead_in("left", _on_lock(LOCK_REST) + [0.0, 0.03, 0.04], LEFT_GRIP, "over the lock bar", FINGER_OPEN)
    out += lead
    out += [_w(1.0, _on_lock(LOCK_REST), -0.012, "round the lock bar near its tip", LEFT_GRIP),
            _w(0.5, _on_lock(LOCK_REST), LOCK_PINCH, "hold it", LEFT_GRIP)]
    for k in range(1, 9):
        up = LOCK_REST * (1 - k / 8)
        out.append(_w(0.45, _on_lock(up), LOCK_PINCH, "swing it forward over the hammer", LEFT_GRIP))
    out += [_w(0.6, _on_lock(0.0, press=0.006), LOCK_PINCH, "press its tip down into the catch", LEFT_GRIP),
            _w(0.5, _on_lock(0.0, press=0.006), -0.012, "let go", LEFT_GRIP)]
    out_at = _on_lock(0.0) + [0.0, 0.07, 0.06]
    out += [_w(1.0, out_at, -0.012, "withdraw to the left, out of the trap", LEFT_GRIP)]
    left_out = sum(w.duration for w in out)
    out += _go_home("left", out_at, LEFT_GRIP, -0.012)
    return out, left_out


def demo():
    _, cocked_at = _right(0.0)
    left, left_out = _left(cocked_at)
    right, _ = _right(left_out)
    total = max(sum(w.duration for w in right), sum(w.duration for w in left)) + 3.0
    for arm in (left, right):
        arm.append(_w(total - sum(w.duration for w in arm), arm[-1].pos, arm[-1].finger,
                      "hold still: the trial ends once the full score has held 1 s with both arms home", arm[-1].yaw))
    return {"left": left, "right": right}
