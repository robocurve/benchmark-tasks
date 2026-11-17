"""A distillation set-up stands on a lab stand: a round-bottom flask in a cork ring with a still head on it, the still
head held by a clamp, its side arm reaching horizontally toward the right and ending in a ground-glass cone. A glass
condenser lies on two cork cradles in front of it; a yellow plastic joint clip (a Keck clip) stands on the table on
the left.

Task: fit the condenser to the still head. One hand lifts the condenser and pushes its ground-glass socket onto the
side arm's cone, and keeps holding it there: an unclipped horizontal joint does not carry the condenser, which slides
off the cone and falls. The other hand takes the clip and pushes it down over the joint until it snaps on. Then both
hands let go, the condenser hanging on the clipped joint, and both arms go back to where they started. The glass must
not be dropped.

Run it:  inspect-robots run --embodiment yam_arena -E task=clip_a_condenser_onto_a_still_head.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, glue, on_reset,  # noqa: E402
                     on_step, rotation, set_glue_on)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the fixed set-up: stand, flask, still head and its side arm ------------------------------------------------------
CX = 0.28  # everything is on this forward line
AXIS_Z = TOP + 0.085  # the side arm's (and the condenser's) axis height
HEAD_Y = 0.140  # the still head's vertical tube
JOINT_Y = 0.040  # the side arm's shoulder: where the condenser's socket rim seats
CONE = (0.011, 0.030)  # the cone: radius, length (toward -y)
ARM_R = 0.010  # the side arm tube
ANCHOR_AT = np.array([CX, 0.0, TOP - 0.05])  # a hidden fixed body the joint locks to

# -- the condenser (borosilicate glass): origin at its socket rim on its axis, running toward -y ----------------------
SOCK_IN, SOCK_OUT, SOCK_LEN = 0.0145, 0.0175, 0.040
NOSE = (0.0075, 0.006)  # the cone's narrower nose: radius, length
FLARE = (0.008, 34.0)  # the socket's flared mouth: length, angle out
JACKET_R, JACKET = 0.017, (-0.043, -0.255)  # its water jacket: radius, y span from the rim
COND_MASS, COND_COM = 0.15, (0.0, -0.150, 0.0)
START = np.array([0.190, -0.030, TOP + 0.035])  # its rim at the start, lying on the cradles
CRADLE_Y = (-0.090, -0.240)  # the cradles' middles (y)

# -- the joint clip (yellow acetal): origin under its spine's middle ----------------------------------------------------
CLIP_SPINE = (0.051, 0.012, 0.006)
CLIP_ARM = (0.004, 0.012, 0.018)
CLIP_ARM_X = 0.0215
CLIP_TAB = (0.012, 0.010, 0.018)
CLIP_START = np.array([0.190, 0.120])

GLASS, FROST = (0.80, 0.90, 0.95), (0.93, 0.95, 0.96)
IRON, CHROME, BLACK, CORK = (0.18, 0.18, 0.20), (0.80, 0.81, 0.83), (0.08, 0.08, 0.09), (0.72, 0.55, 0.36)
YELLOW, WATER = (0.95, 0.80, 0.10), (0.55, 0.72, 0.85)

# -- scoring ----------------------------------------------------------------------------------------------------------
SEAT_Y = (JOINT_Y - 0.006, JOINT_Y + 0.010)  # the condenser's rim here: seated (pushed onto the cone)
SEAT_OFF, SEAT_DEG = 0.005, 6.0  # its axis this close to the cone's, and within this angle
CLIP_IN = (0.010, 0.014, 0.006)  # the clip over the joint: across, along, its spine this far above the socket's top
CLIP_TILT = 20.0  # the clip upright within this
DROP_SPEED, DROP_DEG = 0.7, 35.0
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _rig() -> Body:
    """Stand, cork ring, flask, still head (with its clamp) and the side arm with its cone; origin on the table under
    the still head."""
    o = np.array([CX, HEAD_Y, TOP])

    def at(x, y, z):
        return tuple((np.array([x, y, z]) - o).tolist())

    rod_x = CX + 0.125  # the rod stands behind the still head, away from the arms
    shapes = [BoxShape(at(CX + 0.05, HEAD_Y, TOP + 0.006), (0.20, 0.12, 0.012), IRON),  # the base plate
              CylShape(at(rod_x, HEAD_Y, TOP + 0.012 + 0.09), 0.006, 0.18, CHROME),  # the rod
              BoxShape(at(rod_x, HEAD_Y, TOP + 0.150), (0.028, 0.028, 0.028), BLACK),  # boss head
              BoxShape(at((rod_x + CX) / 2 + 0.01, HEAD_Y, TOP + 0.150), (rod_x - CX - 0.01, 0.010, 0.010), CHROME),
              BoxShape(at(CX + 0.006, HEAD_Y + 0.018, TOP + 0.150), (0.034, 0.008, 0.016), CORK),  # its jaws
              BoxShape(at(CX + 0.006, HEAD_Y - 0.018, TOP + 0.150), (0.034, 0.008, 0.016), CORK)]
    for k in range(10):  # the cork ring under the flask
        a = k * math.pi / 5
        shapes.append(BoxShape(at(CX + 0.032 * math.cos(a), HEAD_Y + 0.032 * math.sin(a), TOP + 0.012 + 0.0075),
                               (0.016, 0.022, 0.015), CORK, yaw=a))
    bulb_z = TOP + 0.012 + 0.015 + 0.02
    shapes += [SphereShape(at(CX, HEAD_Y, bulb_z), 0.030, GLASS),  # the flask's bulb
               CylShape(at(CX, HEAD_Y, bulb_z + 0.04), 0.0125, 0.03, GLASS),  # its neck
               CylShape(at(CX, HEAD_Y, (bulb_z + 0.055 + TOP + 0.175) / 2), 0.013, TOP + 0.175 - bulb_z - 0.055,
                        GLASS),  # the still head's tube
               CylShape(at(CX, (HEAD_Y + JOINT_Y) / 2, AXIS_Z), ARM_R, HEAD_Y - JOINT_Y, GLASS, axis="Y"),  # side arm
               CylShape(at(CX, JOINT_Y - CONE[1] / 2, AXIS_Z), CONE[0], CONE[1], FROST, axis="Y"),  # the cone
               CylShape(at(CX, JOINT_Y - CONE[1] - NOSE[1] / 2, AXIS_Z), NOSE[0], NOSE[1], FROST, axis="Y")]
    return Body.build("rig", pos=tuple(o.tolist()), mass=5.0, kinematic=True, shapes=shapes, color=GLASS,
                      friction=(0.12, 0.10), slick=True)


def _anchor() -> Body:
    return Body.build("lock_anchor", pos=tuple(ANCHOR_AT.tolist()), mass=0.01, kinematic=True, color=BLACK,
                      shapes=[SphereShape((0.0, 0.0, 0.0), 0.003, BLACK, collide=False)])


def _cradles() -> Body:
    """Two cork cradles with low ridges either side of the condenser; origin between them on the table."""
    o = np.array([START[0], (CRADLE_Y[0] + CRADLE_Y[1]) / 2, TOP])
    shapes = []
    floor = START[2] - JACKET_R - TOP
    for y in CRADLE_Y:
        shapes.append(BoxShape((0.0, y - o[1], floor / 2), (0.050, 0.030, floor), CORK))
        for s in (-1.0, 1.0):
            shapes.append(BoxShape((s * (JACKET_R + 0.004), y - o[1], floor + 0.004), (0.008, 0.030, 0.008), CORK))
    return Body.build("cradles", pos=tuple(o.tolist()), mass=0.1, kinematic=True, shapes=shapes, color=CORK)


def _qmul(p, q) -> tuple[float, float, float, float]:
    """The rotation q, then p (quaternions as x, y, z, w)."""
    x1, y1, z1, w1 = p
    x2, y2, z2, w2 = q
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def _condenser() -> Body:
    shapes = []
    rm = (SOCK_IN + SOCK_OUT) / 2
    for k in range(8):  # the socket: a ring open toward +y
        a = k * math.pi / 4
        q = (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))  # turned a about y
        shapes.append(BoxShape((rm * math.sin(a), -SOCK_LEN / 2, rm * math.cos(a)),
                               (2 * rm * math.tan(math.pi / 8) + 0.001, SOCK_LEN, SOCK_OUT - SOCK_IN), FROST,
                               quat_xyzw=q))
        # its flared mouth: a plate tilted FLARE[1] degrees out, from the bore at the rim
        th = math.radians(FLARE[1])
        rc = SOCK_IN + FLARE[0] / 2 * math.tan(th) + (SOCK_OUT - SOCK_IN) / 2
        shapes.append(BoxShape((rc * math.sin(a), FLARE[0] / 2, rc * math.cos(a)),
                               (2 * rc * math.tan(math.pi / 8) + 0.002, FLARE[0] / math.cos(th), SOCK_OUT - SOCK_IN),
                               FROST, quat_xyzw=_qmul(q, (math.sin(th / 2), 0.0, 0.0, math.cos(th / 2)))))
    shapes += [CylShape((0.0, (JACKET[0] + JACKET[1]) / 2, 0.0), JACKET_R, JACKET[0] - JACKET[1], GLASS, axis="Y"),
               CylShape((0.0, (-SOCK_LEN + JACKET[0]) / 2, 0.0), 0.012, -SOCK_LEN - JACKET[0] + 0.002, GLASS, axis="Y"),  # neck
               CylShape((0.0, JACKET[1] - 0.012, 0.0), 0.008, 0.024, GLASS, axis="Y"),  # its drip end
               CylShape((0.0, JACKET[1] + 0.020, JACKET_R + 0.006), 0.004, 0.014, WATER, collide=False),  # hose barbs
               CylShape((0.0, JACKET[0] - 0.020, -JACKET_R - 0.006), 0.004, 0.014, WATER, collide=False)]
    return Body.build("condenser", pos=tuple(START.tolist()), mass=COND_MASS, shapes=shapes, color=GLASS,
                      friction=(0.9, 0.8), com=COND_COM)


def _clip() -> Body:
    shapes = [BoxShape((0.0, 0.0, CLIP_SPINE[2] / 2), CLIP_SPINE, YELLOW),
              BoxShape((0.0, 0.0, CLIP_SPINE[2] + CLIP_TAB[2] / 2), CLIP_TAB, YELLOW)]
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((s * CLIP_ARM_X, 0.0, -CLIP_ARM[2] / 2 + 0.001), CLIP_ARM, YELLOW))
    return Body.build("clip", pos=(CLIP_START[0], CLIP_START[1], TOP + CLIP_ARM[2] - 0.001), mass=0.008,
                      shapes=shapes, color=YELLOW, friction=(1.0, 0.9))


OBJECTS = [_rig(), _anchor(), _cradles(), _condenser(), _clip()]
glue("condenser_lock", "lock_anchor", "condenser", anchor=tuple(ANCHOR_AT.tolist()))
glue("clip_lock", "lock_anchor", "clip", anchor=tuple(ANCHOR_AT.tolist()))
CLIP_Y = JOINT_Y - 0.010  # the clip sits on the socket, just behind its flared mouth
SEAT = {"condenser": np.array([CX, JOINT_Y, AXIS_Z]), "clip": np.array([CX, CLIP_Y, AXIS_Z + SOCK_OUT])}


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"dropped": False, "seated_ever": False, "clipped": False, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("CLIP_DEBUG")), "steps": 0}


def _lock_frames() -> None:
    """Each lock holds its part at its seat, square (the clip draws the joint home)."""
    import bodykit
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    for name, lock in (("condenser", "condenser_lock"), ("clip", "clip_lock")):
        joint = UsdPhysics.FixedJoint(get_current_stage().GetPrimAtPath(bodykit._GLUE_PATHS[lock]))
        joint.GetLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in SEAT[name] - ANCHOR_AT]))
        joint.GetLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.GetLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    _lock_frames()
    for lock in ("condenser_lock", "clip_lock"):
        set_glue_on(lock, False)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _seated(cp, cr) -> bool:
    off = math.hypot(float(cp[0] - CX), float(cp[2] - AXIS_Z))
    angle = math.degrees(math.acos(max(-1.0, min(1.0, float(cr[1, 1])))))
    return SEAT_Y[0] < float(cp[1]) < SEAT_Y[1] and off < SEAT_OFF and angle < SEAT_DEG


def _clip_over(kp, kr) -> bool:
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(kr[2, 2])))))
    dz = float(kp[2]) - (AXIS_Z + SOCK_OUT)
    return (abs(float(kp[0]) - CX) < CLIP_IN[0] and abs(float(kp[1]) - CLIP_Y) < CLIP_IN[1]
            and -0.004 < dz < CLIP_IN[2] and tilt < CLIP_TILT)


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    c = as_numpy(base.scene["condenser"].data.root_link_pose_w)[0]
    k = as_numpy(base.scene["clip"].data.root_link_pose_w)[0]
    cp, cr, kp, kr = c[:3] - origin, rotation(c[3:7]), k[:3] - origin, rotation(k[3:7])
    vel = as_numpy(base.scene["condenser"].data.root_lin_vel_w)[0]
    droop = math.degrees(math.asin(max(-1.0, min(1.0, abs(float(cr[2, 1]))))))
    if not _EP["clipped"] and (float(vel[2]) < -DROP_SPEED or droop > DROP_DEG):
        _EP["dropped"] = True
    seated = _seated(cp, cr)
    if seated:
        _EP["seated_ever"] = True
    if not _EP["clipped"] and seated and _clip_over(kp, kr):  # the clip snaps on: the joint is held
        set_glue_on("condenser_lock", True)
        set_glue_on("clip_lock", True)
        _EP["clipped"] = True
    _EP["tcps"] = tcps = _tcps(base, origin)
    near = min(float(np.linalg.norm(t - p)) for t in tcps for p in (kp, cp + cr @ np.array([0.0, -0.12, 0.0])))
    _EP["s"] = {"seated": seated, "near": near}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status()
            tl, tr = tcps
            print(f"[clip] t={_DEBUG['steps'] / 20:5.1f}s cond_rim={np.round(cp * 1000).astype(int).tolist()} "
                  f"droop={droop:4.1f} seated={seated} ever={_EP['seated_ever']} "
                  f"clip-seat={np.round((kp - SEAT['clip']) * 1000).astype(int).tolist()} "
                  f"clip_tilt={math.degrees(math.acos(max(-1.0, min(1.0, float(kr[2, 2]))))):4.1f} "
                  f"cond_deg={math.degrees(math.acos(max(-1.0, min(1.0, float(cr[1, 1]))))):4.1f} clipped={_EP['clipped']} "
                  f"dropped={_EP['dropped']} L-tab={np.round((tl - kp - [0, 0, CLIP_SPINE[2] + CLIP_TAB[2]]) * 1000).astype(int).tolist()} "
                  f"R-cond={np.round((tr - cp - cr @ [0, -0.15, JACKET_R]) * 1000).astype(int).tolist()} "
                  f"L={np.round(tl * 1000).astype(int).tolist()} R={np.round(tr * 1000).astype(int).tolist()} "
                  f"home={st['home']} score={_points(st)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    return {"clipped": _EP["clipped"], "seated_ever": _EP["seated_ever"], "dropped": _EP["dropped"],
            "home": _arms_home()}


def _points(s) -> float:
    if s["dropped"]:
        return 0.0
    if s["clipped"] and s["home"]:
        return 1.0
    if s["clipped"]:
        return 0.5
    return 0.25 if s["seated_ever"] else 0.0


def score(objects) -> float:
    """0.0 if the condenser was ever dropped (falling faster than 0.7 m/s, or tipped more than 35 degrees off level)
    before the clip was on. Seated: the condenser's socket pushed onto the side arm's cone (its rim within 6 mm short
    of, or 10 mm past, the cone's shoulder; its axis within 5 mm and 6 degrees of the cone's). Clipped: the clip snapped
    on over the joint while the condenser was seated. Otherwise 1.0: clipped, and both arms back where they started
    (each gripper's fingertip centre (TCP) within 5 cm of its start) at the end; 0.5: clipped; 0.25: the condenser
    seated at some point; else 0.0."""
    return _points(_status())


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
    """Full score with the condenser and the clip still."""
    return score(objects) >= 1.0 and objects["condenser"].resting() and objects["clip"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand comes straight down on the condenser 15.5 cm from its socket (at its balance point), fingers closing across it, lifts it off
# the cradles, carries it up in line with the side arm and pushes its socket onto the cone, then holds it there. The
# left hand comes down on the clip's tab, fingers closing across it, lifts it, carries it over the side arm to above
# the joint and pushes it down over the joint until it snaps on. Both hands let go and both arms go back to where they
# started.

GRIP_AT = 0.155  # the right takes the condenser this far from its socket's rim
PAD_TWIST_DEG = 4.0  # the slanted pads turn the held condenser this much about z: the grip is turned back by it
RIGHT_G = grip_rotation(math.pi / 2 - math.radians(PAD_TWIST_DEG))  # fingers closing (about) along x, across it
LEFT_G = grip_rotation(0.0)  # fingers closing along y, across the clip's tab
IN = 0.003  # the right's TCP this far below the condenser's top
TAB_ABOVE = 0.004  # the left's TCP this far above the tab's top
R_PRE = -(JACKET_R + 0.006)
L_PRE = -(CLIP_TAB[1] / 2 + 0.006)
PUSH_PAST = 0.006  # push the socket this far past the shoulder
AIM_UP = 0.008  # aim the condenser this much high (the arm sags and the condenser droops in the grip)
CLIP_PUSH = 0.010  # aim the clip this far below its seat
CLIP_AIM = np.array([0.0, -0.008, 0.0])  # and this far off it (the left arm lags)
LEAD_S, LEAD_STEPS = 3.0, 15


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


def _sweep(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _sweep(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _sweep(arm, _good_ik(arm, pos, rot), np.array(READY, float), "back to where it started", finger)


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _right(let_go_at: float) -> tuple[list[Waypoint], float]:
    g = RIGHT_G
    grip = np.array([0.0, -GRIP_AT, JACKET_R - IN])
    take = START + grip
    line = np.array([CX, JOINT_Y - CONE[1] - 0.018, AXIS_Z + AIM_UP]) + grip  # its rim 1.8 cm short of the cone's tip
    seat = np.array([CX, JOINT_Y + PUSH_PAST, AXIS_Z + AIM_UP]) + grip
    out = _lead_in("right", take + [0.0, 0.0, 0.05], g, "over the condenser", R_PRE)
    out += [_w(1.0, take, R_PRE, "down round it", g),
            _w(0.6, take, FINGER_CLOSED, "take it", g),
            _w(1.0, take + [0.0, 0.0, 0.06], FINGER_CLOSED, "lift it off the cradles", g),
            _w(2.0, line + [0.0, 0.0, 0.01], FINGER_CLOSED, "carry it in line with the side arm", g),
            _w(0.6, line, FINGER_CLOSED, "line its socket up with the cone", g),
            _w(2.0, seat, FINGER_CLOSED, "push the socket onto the cone", g)]
    seated_at = sum(w.duration for w in out)
    if let_go_at > seated_at:
        out.append(_w(let_go_at - seated_at, seat, FINGER_CLOSED, "hold it there while the clip goes on", g))
    out += [_w(0.5, seat, R_PRE, "let go: the condenser hangs on the clipped joint", g),
            _w(0.8, seat + [0.0, 0.0, 0.04], R_PRE, "rise", g)]
    out += _go_home("right", seat + [0.0, 0.0, 0.04], g, FINGER_OPEN)
    return out, seated_at


def _left(start: float, total: float) -> tuple[list[Waypoint], float]:
    g = LEFT_G
    tab_top = CLIP_ARM[2] - 0.001 + CLIP_SPINE[2] + CLIP_TAB[2]
    take = np.array([CLIP_START[0], CLIP_START[1], TOP + tab_top + TAB_ABOVE])
    seat_tcp = SEAT["clip"] + CLIP_AIM + [0.0, 0.0, CLIP_SPINE[2] + CLIP_TAB[2] + TAB_ABOVE]
    over = seat_tcp + [0.0, 0.0, 0.025]
    via = np.array([CX, 0.075, over[2]])  # over the side arm, between the still head and the joint
    out = _lead_in("left", take + [0.0, 0.0, 0.05], g, "over the clip", L_PRE)
    t = sum(w.duration for w in out)
    if start > t:
        out.append(_w(start - t, take + [0.0, 0.0, 0.05], L_PRE, "wait for the condenser to be on", g))
    out += [_w(1.0, take, L_PRE, "down round its tab", g),
            _w(0.6, take, FINGER_CLOSED, "take the clip", g),
            _w(1.0, take + [0.0, 0.0, over[2] - take[2]], FINGER_CLOSED, "lift it", g),
            _w(1.4, via, FINGER_CLOSED, "carry it over the side arm", g),
            _w(1.2, over, FINGER_CLOSED, "along to above the joint", g),
            _w(1.4, seat_tcp - [0.0, 0.0, CLIP_PUSH], FINGER_CLOSED, "push it down over the joint: it snaps on", g),
            _w(0.5, seat_tcp - [0.0, 0.0, CLIP_PUSH], FINGER_CLOSED, "hold", g)]
    clipped_at = sum(w.duration for w in out)
    out += [_w(0.5, seat_tcp, L_PRE, "let go", g),
            _w(0.8, seat_tcp + [0.0, 0.0, 0.03], L_PRE, "rise", g)]
    out += _go_home("left", seat_tcp + [0.0, 0.0, 0.03], g, FINGER_OPEN)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s", out[-1].yaw))
    return out, clipped_at


def demo():
    _, seated_at = _right(0.0)
    _, clipped_at = _left(seated_at - 4.0, 0.0)
    right, _ = _right(clipped_at + 0.8)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_OPEN, "wait at the start", right[-1].yaw))
    left, _ = _left(seated_at - 4.0, total)
    return {"left": left, "right": right}
