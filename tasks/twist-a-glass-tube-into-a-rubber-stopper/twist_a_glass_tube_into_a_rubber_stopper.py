"""On the station table, a short glass tube stands upright, clamped by its lower end in a stand; a rubber stopper lies
beside it.

Task: fit the stopper onto the tube's top end, the way it is done in a lab: one hand holds the tube still, close below
its top end (by the cloth wrapped round it there); the other sets the stopper's hole on the tube's end and twists it down, gently, a little at a time.
Twisting the stopper while the tube is held only far below it (or only by the stand's clamp) levers the glass, and it
snaps:
the classic lab injury. So the hand on the tube must hold it within 3.5 cm below the stopper while the stopper turns.

Run it:  inspect-robots run --embodiment yam_arena -E task=twist_a_glass_tube_into_a_rubber_stopper.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation,  # noqa: E402
                     set_slide_target, set_thread_on, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the stand (fixed): a block whose socket clamps the tube's lower end ---------------------------------------------------
AXIS = np.array([0.26, 0.0])
RACK_H, SOCKET_IN, SOCKET_D = 0.050, 0.0053, 0.040  # a snug, deep socket: the tube stands straight

# -- the glass tube, clamped upright in the stand (fixed); shown as two pieces: where they meet, it snaps ------------------
TUBE_R = 0.005
TUBE_Z0 = TOP + RACK_H - SOCKET_D  # its lower end, on the socket's floor
TUBE_TOP = TOP + 0.137  # its upper end
TOP_PIECE = 0.045  # the upper piece (the one the stopper goes on): this long
CLOTH = (0.017, 0.018, 0.040)  # a cloth wrapped round the tube near its top end: across, from, to (down from the end)

# -- the stopper: octagonal rubber with a rim on top, a bore through it narrower than the tube, a countersink at its
# underside's mouth ----------------------------------------------------------------------------------------------------
STOP_R, STOP_H = 0.009, 0.030  # across the corners / 2, tall
FLANGE_R, FLANGE_H = 0.013, 0.004  # a wider rim on top: held just below it, it cannot slip down through the fingers
SINK_D, SINK_R = 0.003, 0.007  # the tube's end sits in the countersink ...
BORE_R = 0.0045  # ... above a bore narrower than the tube: it goes on only by twisting (the thread; the stopper and
# the tube's top piece do not collide: before it catches, a stopper let go slides on down the tube, not caught)
STOP_AT = (0.20, -0.10)  # where it lies at the start, on the table, underside down

# -- the fit: seated on the tube's end and turned TO_CATCH clockwise (from above) against it, it catches; then PITCH per
# turn draws it down the tube ----------------------------------------------------------------------------------------
PITCH = 0.016
TO_CATCH = math.radians(45.0)
DEPTH_MAX = 0.016
THREAD_DRAG = 0.02  # N m s/rad: the rubber's grip on the glass
SEATED = (0.0015, 0.008)  # the tube's end this far up into the stopper: seated (it can catch)
ALIGN_R, ALIGN_TILT = 0.003, math.radians(10.0)

RUBBER, GLASS, GLASS_END, DARK, STEEL, CLOTH_C = (0.45, 0.12, 0.10), (0.72, 0.86, 0.90), (0.55, 0.75, 0.80), \
    (0.20, 0.21, 0.23), (0.62, 0.64, 0.66), (0.92, 0.90, 0.84)

# -- scoring ----------------------------------------------------------------------------------------------------------
HOLD_NEAR = 0.035  # the tube held no further than this below the stopper while the stopper turns, or it snaps
ON_TUBE = 0.015  # a TCP within this of the tube's axis (below the stopper, above the rack) holds the tube
TURNING = math.radians(1.0)  # a step in which the stopper turns at least this much against the tube loads the glass
FULL, PART = 0.012, 0.006
LET_GO = 0.03
SUCCESS_HOLD_S = 1.0


def _ring(r_in, r_out, z0, z1, color, n=16) -> list:
    """A ring of n boxes about the z axis (n=8: an octagon); neighbours overlap (no slots)."""
    out = []
    r_mid, thick = (r_in + r_out) / 2, r_out - r_in
    chord = 2 * r_out * math.tan(math.pi / n) + 0.0005
    for k in range(n):
        a = 2 * math.pi * k / n
        out.append(BoxShape((r_mid * math.cos(a), r_mid * math.sin(a), (z0 + z1) / 2), (thick, chord, z1 - z0), color,
                            yaw=a))
    return out


def _rack() -> Body:
    shapes = [BoxShape((0.0, 0.0, (RACK_H - SOCKET_D) / 2), (0.05, 0.05, RACK_H - SOCKET_D), DARK),
              *_ring(SOCKET_IN, 0.025, RACK_H - SOCKET_D, RACK_H, DARK, n=12)]
    shapes.append(BoxShape((0.0, 0.0, RACK_H + 0.006), (0.020, 0.020, 0.012), STEEL, collide=False))  # its clamp
    return Body.build("rack", pos=(*AXIS, TOP), mass=1.0, shapes=shapes, kinematic=True, friction=(0.5, 0.4),
                      color=DARK)


def _tube_low() -> Body:
    """The tube's lower piece; origin at its lower end."""
    length = TUBE_TOP - TOP_PIECE - TUBE_Z0
    return Body.build("tube_low", pos=(*AXIS, TUBE_Z0), mass=0.006, friction=(1.2, 1.1), color=GLASS, kinematic=True,
                      shapes=[CylShape((0.0, 0.0, length / 2), TUBE_R, length, GLASS)])


def _tube_top() -> Body:
    """The tube's upper piece; origin at its upper end (the one the stopper goes on)."""
    return Body.build("tube_top", pos=(*AXIS, TUBE_TOP), mass=0.008, friction=(1.2, 1.1), color=GLASS, kinematic=True,
                      shapes=[CylShape((0.0, 0.0, -TOP_PIECE / 2), TUBE_R, TOP_PIECE, GLASS),
                              CylShape((0.0, 0.0, -0.0005), TUBE_R - 0.0015, 0.0012, GLASS_END, collide=False),
                              BoxShape((0.0, 0.0, -(CLOTH[1] + CLOTH[2]) / 2), (CLOTH[0], CLOTH[0], CLOTH[2] - CLOTH[1]),
                                       CLOTH_C)])


def _stopper() -> Body:
    """Origin at the middle of its underside (where the hole's mouth is)."""
    shapes = [*_ring(SINK_R, STOP_R, 0.0, SINK_D, RUBBER, n=8), *_ring(BORE_R, STOP_R, SINK_D, STOP_H, RUBBER, n=8),
              *_ring(BORE_R, FLANGE_R, STOP_H, STOP_H + FLANGE_H, RUBBER, n=8)]
    return Body.build("stopper", pos=(*STOP_AT, TOP), mass=0.02, shapes=shapes, friction=(1.2, 1.1), color=RUBBER,
                      ignore=("tube_top",))  # the fit (a thread switched on while running) does not stop them colliding


OBJECTS = [_rack(), _tube_low(), _tube_top(), _stopper()]
SNAPPED = (np.array([AXIS[0] - 0.02, AXIS[1] + 0.07, TOP + CLOTH[0] / 2]),  # the snapped-off piece, lying on the table
           np.array([math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)]))
thread("fit", "tube_top", "stopper", anchor0=(0.0, 0.0, 0.0), axis0=(0.0, 0.0, -1.0), anchor1=(0.0, 0.0, 0.0),
       axis1=(0.0, 0.0, -1.0), slide_limits=(0.0, DEPTH_MAX), turn_damping=THREAD_DRAG, slide_stiffness=2000.0,
       slide_damping=10.0, slide_max_force=20.0)


# -- followed after every control step: the fit and the glass -----------------------------------------------------------

def _fresh() -> dict:
    return {"pre": 0.0, "caught": False, "ever_caught": False, "x0": 0.0, "turns": 0.0, "phi": None, "broken": False,
            "tcps": None, "s": {}, "broke_at": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("TUBE_DEBUG")), "steps": 0}


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


def _pose(base, origin, name) -> tuple[np.ndarray, np.ndarray]:
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _geometry(t_pos, t_rot, s_pos, s_rot) -> dict:
    """The stopper's hole against the tube's end: how far the end is up into it, how far off its axis, how lined up,
    and the stopper's turn against the tube about the tube's axis."""
    up = t_rot @ np.array([0.0, 0.0, 1.0])
    v = s_pos - t_pos
    depth = -float(v @ up)  # the stopper's underside this far below the tube's end
    u, w = t_rot @ np.array([1.0, 0.0, 0.0]), s_rot @ np.array([1.0, 0.0, 0.0])
    return {"depth": depth, "radial": float(np.linalg.norm(v + depth * up)), "up": up,
            "cos": float((s_rot @ np.array([0.0, 0.0, 1.0])) @ up),
            "phi": math.atan2(float(np.cross(u, w) @ up), float(u @ w))}


def _snap(base, origin) -> None:
    """The glass snaps where the pieces meet: the top piece (with the stopper, which comes free) drops on the table."""
    import torch

    set_thread_on("fit", False)
    asset = base.scene["tube_top"]
    p = SNAPPED[0] + origin
    asset.write_root_pose_to_sim(torch.tensor([[*p.tolist(), *SNAPPED[1].tolist()]], dtype=torch.float32,
                                              device=asset.device))


def _hold_below(tcps, s_pos, t_pos, up) -> float | None:
    """How far below the stopper's underside the nearest hand on the tube holds it (None: no hand on the tube)."""
    best = None
    for p in tcps:
        v = p - t_pos
        along = float(v @ up)
        below = float((s_pos - p) @ up)
        if np.linalg.norm(v - along * up) <= ON_TUBE and below > 0.0 and float(p[2] - TOP - RACK_H) > 0.0:
            if best is None or below < best:
                best = below
    return best


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    t_pos, t_rot = _pose(base, origin, "tube_top")
    s_pos, s_rot = _pose(base, origin, "stopper")
    g = _geometry(t_pos, t_rot, s_pos, s_rot)
    _EP["tcps"] = _tcps(base, origin)
    tighten = 0.0  # the stopper clockwise seen from above, against the tube
    if _EP["phi"] is not None:
        tighten = -((g["phi"] - _EP["phi"] + math.pi) % (2 * math.pi) - math.pi)
    _EP["phi"] = g["phi"]
    seated = (SEATED[0] <= g["depth"] <= SEATED[1] + DEPTH_MAX and g["radial"] <= ALIGN_R
              and g["cos"] >= math.cos(ALIGN_TILT))
    hold = _hold_below(_EP["tcps"], s_pos, t_pos, g["up"])
    # turning the stopper against the tube while the tube is held only far below it levers the glass: it snaps
    twisting = any(float(np.linalg.norm(p - (s_pos + s_rot @ np.array([0.0, 0.0, STOP_H / 2])))) < LET_GO
                   for p in _EP["tcps"])  # a hand on the stopper, turning it
    if not _EP["broken"] and (seated or _EP["caught"]) and twisting and abs(tighten) >= TURNING and \
            (hold is None or hold > HOLD_NEAR):
        _EP["broken"] = True
        _EP["broke_at"] = hold
        _EP["caught"] = False
        _snap(base, origin)
    if _EP["broken"]:
        pass
    elif _EP["caught"]:
        _EP["turns"] += tighten / (2 * math.pi)
        set_slide_target("fit", min(max(_EP["x0"] + PITCH * _EP["turns"], 0.0), DEPTH_MAX))
    elif seated and g["depth"] <= SEATED[1]:
        _EP["pre"] = max(_EP["pre"] + tighten, 0.0)
        if _EP["pre"] >= TO_CATCH and not _EP["broken"]:
            set_slide_target("fit", g["depth"])
            set_thread_on("fit", True)
            _EP.update(caught=True, ever_caught=True, x0=g["depth"], turns=0.0)
    else:
        _EP["pre"] = 0.0
    held = any(float(np.linalg.norm(p - (s_pos + s_rot @ np.array([0.0, 0.0, STOP_H / 2])))) < LET_GO
               for p in _EP["tcps"])
    _EP["s"] = {"depth": g["depth"], "stopper_held": held}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            print(f"[tube] t={_DEBUG['steps'] / 20:5.1f}s depth={g['depth'] * 1000:5.1f}mm radial={g['radial'] * 1000:4.1f}mm "
                  f"tilt={math.degrees(math.acos(max(-1.0, min(1.0, g['cos'])))):4.1f} seated={seated} pre={math.degrees(_EP['pre']):5.1f} "
                  f"caught={_EP['caught']} turns={_EP['turns']:5.2f} hold={None if hold is None else round(hold * 1000)}mm "
                  f"broken={_EP['broken']} tube_yaw={math.degrees(math.atan2(t_rot[1, 0], t_rot[0, 0])):6.1f} "
                  f"stopper={np.round(s_pos * 1000).astype(int).tolist()} "
                  f"leftTCP-tube_end={np.round((tl - t_pos) * 1000).astype(int).tolist()} "
                  f"rightTCP-stopper={np.round((tr - s_pos) * 1000).astype(int).tolist()} score={_points(_status())}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    return {"broken": _EP["broken"], "caught": _EP["caught"], "ever_caught": _EP["ever_caught"],
            "depth": st.get("depth", 0.0), "stopper_held": bool(st.get("stopper_held"))}


def _points(s) -> float:
    if s["broken"]:
        return 0.0
    if s["caught"] and s["depth"] >= FULL and not s["stopper_held"]:
        return 1.0
    if s["caught"] and s["depth"] >= PART:
        return 0.5
    return 0.25 if s["ever_caught"] else 0.0


def score(objects) -> float:
    """0.0 if the tube snapped (the stopper turned against it while it was held only further than 3.5 cm below the
    stopper, or not at all). Else 1.0 with the stopper at least 12 mm down the tube and let go; 0.5 at least 6 mm down;
    0.25 with the stopper caught on the tube at some point; else 0.0."""
    return _points(_status())


def success(objects) -> bool:
    """Full score with the stopper still: the trial ends once it has held 1 s."""
    return score(objects) >= 1.0 and objects["stopper"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes level from the left onto the tube's cloth, 2.9 cm below its top end, fingers closing across it,
# and holds it still. The right hand comes straight down on the stopper, fingers closing across its flats, picks it up, sets its
# hole on the tube's end and twists it down clockwise in strokes of a quarter turn (grip, roll the wrist, let go, roll
# back, grip again), following it down. Then both hands let go and draw back.


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


SIDE_L = _rx(-math.pi / 2) @ grip_rotation(-math.pi / 2)  # level, pointing -y (from the left), closing along x
STOP_GRIP = _rz(math.pi / 4) @ grip_rotation(0.0)  # straight down, at 45 degrees (across two flats): strokes roll to -45
HOLD_BELOW_TOP = 0.026  # the left holds the tube this far below its top end: on the cloth
IN_HAND = 0.024  # the right TCP this far up the stopper from its underside, just below its rim
STROKE = math.radians(90.0)
STROKES = 4
TURN_S, RELEASE_S, BACK_S, REGRIP_S = 1.6, 0.4, 1.0, 0.5
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


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    side = np.array([*AXIS, TUBE_TOP - HOLD_BELOW_TOP])
    out = _lead_in("left", side + [0.0, 0.05, 0.0], SIDE_L, "level, left of the tube's top", FINGER_OPEN)
    out += [_w(1.0, side, FINGER_OPEN, "onto it, close below its end", SIDE_L),
            _w(0.6, side, FINGER_CLOSED, "hold it still", SIDE_L)]
    held_at = sum(w.duration for w in out)
    out += [_w(max(release_at - held_at, 0.3), side, FINGER_CLOSED, "hold it while the stopper is twisted on", SIDE_L),
            _w(0.5, side, FINGER_OPEN, "let go", SIDE_L),
            _w(1.0, side + [0.0, 0.06, 0.0], FINGER_OPEN, "draw back", SIDE_L)]
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), side + [0.0, 0.06, 0.0], FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s", SIDE_L))
    return out, held_at


def _depth_after(turned: float) -> float:
    """How far down the tube the stopper's underside is after the hand has turned it ``turned`` (rad) from seated."""
    return min(SEATED[0] + 0.0015 + max(turned - TO_CATCH, 0.0) * PITCH / (2 * math.pi), DEPTH_MAX)


def _at(depth: float) -> np.ndarray:
    return np.array([*AXIS, TUBE_TOP - depth + IN_HAND])


def _right(start: float) -> list[Waypoint]:
    pick = np.array([*STOP_AT, TOP + IN_HAND])
    out = _lead_in("right", pick + [0.0, 0.0, 0.05], STOP_GRIP, "over the stopper", FINGER_OPEN)
    out += [_w(1.0, pick, FINGER_OPEN, "down round it", STOP_GRIP),
            _w(0.6, pick, FINGER_CLOSED, "take it", STOP_GRIP),
            _w(1.5, pick + [0.0, 0.0, TUBE_TOP + 0.006 - TOP], FINGER_CLOSED, "lift it", STOP_GRIP)]
    t = sum(w.duration for w in out)
    out += [_w(max(start - t, 0.3) + 1.5, _at(-0.006), FINGER_CLOSED, "its hole over the tube's end", STOP_GRIP),
            _w(1.0, _at(SEATED[0] + 0.0015), FINGER_CLOSED, "set it on the end", STOP_GRIP)]
    turned = 0.0
    for k in range(STROKES):
        for j in range(1, 5):
            out.append(_w(TURN_S / 4, _at(_depth_after(turned + STROKE * j / 4)), FINGER_CLOSED,
                          f"twist it down, gently ({k + 1})", _rz(-STROKE * j / 4) @ STOP_GRIP))
        turned += STROKE
        at = _at(_depth_after(turned))
        out.append(_w(RELEASE_S, at, FINGER_OPEN, "let go", _rz(-STROKE) @ STOP_GRIP))
        if k < STROKES - 1:
            for j in range(1, 5):
                out.append(_w(BACK_S / 4, at, FINGER_OPEN, "turn back, open", _rz(-STROKE * (1 - j / 4)) @ STOP_GRIP))
            out.append(_w(REGRIP_S, at, FINGER_CLOSED, "take it again", STOP_GRIP))
    out.append(_w(1.0, _at(_depth_after(turned)) + [-0.03, -0.05, 0.0], FINGER_OPEN, "back off",
                  _rz(-STROKE) @ STOP_GRIP))
    return out


def demo():
    _, held_at = _left(0.0, 0.0)
    right = _right(held_at)
    t_r = sum(w.duration for w in right)
    total = t_r + 4.5
    right.append(_w(4.5, right[-1].pos, FINGER_OPEN, "hold still", right[-1].yaw))
    left, _ = _left(t_r + 0.3, total + 1.5)
    return {"left": left, "right": right}
