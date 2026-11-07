"""On the station table, in front of the arms: a garden tub with two wooden battens laid across it, and on the battens a
garden sieve full of potting soil with a few pebbles in it.

Task: sift the soil. Lift the sieve off the battens by its two side handles, shake it level over the tub until the soil
has gone through the mesh, keeping the pebbles in the sieve, then set it back on the battens. The sieve is wider than a
gripper can span, with a handle on each side; held by one handle it tips and pours pebbles into the tub, and it sits
too heavily on the battens to be shaken by sliding it there. It is held up by both handles and shaken, the way gardeners
riddle soil over a barrow.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- tub: fixed, 28 x 28 cm inside, 6 cm tall, centred at (0.40, 0).
- battens: fixed, 2 x 32 x 1.2 cm, across the tub's rims under the sieve's front and back walls.
- sieve: loose, 0.6 kg; a wooden frame 20 cm square inside and 7 cm tall (front and back walls 2.4 cm thick, side
  walls 8 mm) over a steel mesh of 4 mm bars with 14 mm square holes; a flat handle 6 x 2.5 x 1 cm out of the middle of
  its left and right walls, 2.5 cm above the mesh. It starts on the battens, centred over the tub.
- soil: 72 crumbs, 2 x 1 x 1 cm, 3 g each, in two layers on the mesh. Each lies across the mesh until shaking tips it
  end-first through a hole (see the fidelity note in task.json).
- pebbles: 6, 2.6 cm across, 30 g each, on top of the soil.

Full score: at least 50 of the 72 crumbs (about 70 %) in the tub, no pebble ever in the tub, and the sieve resting back
on the battens (within 2 cm of where it started, at most 1.5 cm higher, tilted less than 6 degrees); still for 2 s.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, SphereShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 2.0
SCORE_OVER = "best"  # partial credit: the best reached during the run (a pebble in the tub latches 0)

# -- layout (world frame) -----------------------------------------------------------------------------------------------
TUB_XY, TUB_IN, TUB_H, TUB_T = (0.40, 0.0), 0.28, 0.06, 0.01
BATTEN = (0.020, TUB_IN + 2 * TUB_T + 0.02, 0.012)  # across the tub's left and right rims
TUB_C, BATTEN_C = (0.22, 0.42, 0.28), (0.66, 0.52, 0.36)

# -- the sieve (origin: the middle of the mesh's underside) -------------------------------------------------------------
INNER, WALL_T, WALL_H = 0.200, 0.008, 0.070
FB_T = 0.024  # front and back walls 2.4 cm thick (they rest on the battens), side walls 8 mm
PITCH, BAR, BAR_H = 0.018, 0.004, 0.006  # 14 mm holes between 4 mm bars
LUG = (0.060, 0.025, 0.010)  # x, y, z: a flat handle out of each side wall, at the loaded sieve's centre of mass
LUG_Z = 0.025  # at the loaded sieve's centre of mass: 4 cm up, it swung about the handles when shaken
LUG_Y = INNER / 2 + WALL_T + LUG[1] / 2
SIEVE_KG = 0.6
FRAME_C, MESH_C, LUG_C = (0.55, 0.40, 0.25), (0.30, 0.30, 0.32), (0.18, 0.18, 0.18)
BATTEN_X = INNER / 2 + FB_T / 2  # the battens under the front and back walls (+-1.5 cm of play), clear of the mesh
START = np.array([TUB_XY[0], TUB_XY[1], TOP + TUB_H + BATTEN[2]])  # the sieve's origin resting on the battens
SPAWN_DZ = 0.001  # spawned 1 mm up: the thick walls flush on the battens were shoved 6.7 cm sideways at reset

# -- soil and pebbles ---------------------------------------------------------------------------------------------------
CRUMB, CRUMB_KG, CRUMB_MU = (0.020, 0.010, 0.010), 0.003, (0.35, 0.25)
N_SIDE, LAYERS = 6, 2  # 72 crumbs
STONE_R, STONE_KG, N_STONES = 0.013, 0.030, 6
SOIL_C, STONE_C = (0.35, 0.25, 0.16), (0.62, 0.62, 0.64)


def _sieve() -> Body:
    shapes = []
    n = int(round(INNER / PITCH))
    for k in range(n + 1):
        c = -INNER / 2 + k * INNER / n
        shapes.append(BoxShape((c, 0.0, BAR_H / 2), (BAR, INNER, BAR_H), MESH_C))
        shapes.append(BoxShape((0.0, c, BAR_H / 2), (INNER, BAR, BAR_H), MESH_C))
    a, fb, long = INNER / 2 + WALL_T / 2, INNER / 2 + FB_T / 2, INNER + 2 * WALL_T
    shapes += [BoxShape((fb, 0.0, WALL_H / 2), (FB_T, long, WALL_H), FRAME_C),
               BoxShape((-fb, 0.0, WALL_H / 2), (FB_T, long, WALL_H), FRAME_C),
               BoxShape((0.0, a, WALL_H / 2), (INNER, WALL_T, WALL_H), FRAME_C),
               BoxShape((0.0, -a, WALL_H / 2), (INNER, WALL_T, WALL_H), FRAME_C)]
    shapes += [BoxShape((0.0, s * LUG_Y, LUG_Z), LUG, LUG_C) for s in (1, -1)]
    return Body.build("sieve", pos=tuple(START + [0.0, 0.0, SPAWN_DZ]), mass=SIEVE_KG, shapes=shapes, friction=(1.0, 0.9),
                      color=FRAME_C)


def _fixed() -> list:
    (tx, ty), h, t, w = TUB_XY, TUB_IN / 2 + TUB_T / 2, TUB_T, TUB_IN + 2 * TUB_T
    z = TOP + TUB_H / 2
    tub = [Box("tub_floor", pos=(tx, ty, TOP + 0.002), size=(w, w, 0.004), color=TUB_C),
           Box("tub_front", pos=(tx - h, ty, z), size=(t, w, TUB_H), color=TUB_C),
           Box("tub_back", pos=(tx + h, ty, z), size=(t, w, TUB_H), color=TUB_C),
           Box("tub_left", pos=(tx, ty + h, z), size=(TUB_IN, t, TUB_H), color=TUB_C),
           Box("tub_right", pos=(tx, ty - h, z), size=(TUB_IN, t, TUB_H), color=TUB_C)]
    battens = [Box(f"batten_{name}", pos=(tx + s * BATTEN_X, ty, TOP + TUB_H + BATTEN[2] / 2), size=BATTEN,
                   color=BATTEN_C) for name, s in (("front", -1), ("back", 1))]
    return tub + battens


def _soil() -> list[Body]:
    out, k = [], 0
    pitch = 0.026
    z0 = START[2] + SPAWN_DZ + BAR_H + CRUMB[2] / 2 + 0.0005
    for layer in range(LAYERS):
        for i in range(N_SIDE):
            for j in range(N_SIDE):
                x = START[0] + (i - (N_SIDE - 1) / 2) * pitch + (0.006 if layer else 0.0)
                y = START[1] + (j - (N_SIDE - 1) / 2) * pitch + (0.006 if layer else 0.0)
                yaw = (0.0, 0.5, 1.1, 1.57, 2.2)[k % 5]
                out.append(Body.build(f"crumb_{k:02d}", pos=(x, y, z0 + layer * (CRUMB[2] + 0.0025)), yaw=yaw,
                                      mass=CRUMB_KG, shapes=[BoxShape((0.0, 0.0, 0.0), CRUMB, SOIL_C)],
                                      friction=CRUMB_MU, damping=(0.05, 0.3), color=SOIL_C))
                k += 1
    return out


def _stones() -> list[Body]:
    z = START[2] + SPAWN_DZ + BAR_H + LAYERS * (CRUMB[2] + 0.0025) + STONE_R + 0.002
    spots = [(-0.05, -0.05), (0.05, -0.04), (0.0, 0.0), (-0.04, 0.05), (0.05, 0.05), (0.0, -0.075)]
    return [Body.build(f"pebble_{k}", pos=(START[0] + dx, START[1] + dy, z), mass=STONE_KG,
                       shapes=[SphereShape((0.0, 0.0, 0.0), STONE_R, STONE_C)], friction=(0.8, 0.7), color=STONE_C)
            for k, (dx, dy) in enumerate(spots[:N_STONES])]


SIEVE = _sieve()
SOIL, STONES = _soil(), _stones()
CRUMB_NAMES, STONE_NAMES = [c.name for c in SOIL], [s.name for s in STONES]
OBJECTS = [*_fixed(), SIEVE, *SOIL, *STONES]

# -- scoring --------------------------------------------------------------------------------------------------------
SIFTED, HALF, SOME = 50, 36, 18  # crumbs in the tub for 1.0 (~70 %; the demo sifts 56), 0.5 and 0.25
BACK_XY, BACK_Z, LEVEL = 0.02, 0.015, math.radians(6)  # a crumb under a wall is fine
_DEBUG = os.environ.get("SIEVE_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("SIEVE_DEBUG_EVERY", "10"))  # steps between debug lines
_EP = {"step": 0, "stone_in": False}


_TCP: dict = {}
_ARM_IDS: dict = {}


@on_reset
def _reset():
    _EP.update(step=0, stone_in=False)
    _TCP.clear()


@on_step
def _track_grippers(env) -> None:
    """Each gripper's TCP and finger opening from its arm's joint positions (debug output only)."""
    if not _DEBUG:
        return
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS

    base = getattr(env, "unwrapped", env)
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        if side not in _ARM_IDS:
            _ARM_IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
        q = as_numpy(robot.data.joint_pos)[0]
        _TCP[side] = (np.asarray(ik.tcp_pose(q[_ARM_IDS[side]], station.BASES[side])[0], float), q[-2:])


def _in_tub(p) -> bool:
    x, y, z = p
    return bool(abs(x - TUB_XY[0]) < TUB_IN / 2 and abs(y - TUB_XY[1]) < TUB_IN / 2 and TOP < z < TOP + TUB_H)


def _tally(o) -> dict:
    s = o["sieve"]
    tilt = float(math.acos(max(-1.0, min(1.0, rotation(s.quat_xyzw)[2, 2]))))
    back = bool(math.hypot(s.pos[0] - START[0], s.pos[1] - START[1]) <= BACK_XY and abs(s.pos[2] - START[2]) < BACK_Z
                and tilt < LEVEL)
    if any(_in_tub(o[n].pos) for n in STONE_NAMES):
        _EP["stone_in"] = True  # latched: a pebble in the tub spoils the sifted soil for the rest of the run
    return {"soil": sum(_in_tub(o[n].pos) for n in CRUMB_NAMES), "stone_in": _EP["stone_in"], "back": back,
            "tilt": tilt}


def _points(t) -> float:
    if t["stone_in"]:
        return 0.0
    if t["soil"] >= SIFTED:
        return 1.0 if t["back"] else 0.5
    return 0.5 if t["soil"] >= HALF else 0.25 if t["soil"] >= SOME else 0.0


def _debug(o, t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    s = o["sieve"]
    print(f"DBG step={_EP['step']} sieve=({s.pos[0]:.3f},{s.pos[1]:.3f},{s.pos[2] - TOP:.3f}) "
          f"tilt={math.degrees(t['tilt']):.0f} soil_in_tub={t['soil']}/{len(CRUMB_NAMES)} stone_in={t['stone_in']} "
          f"back={t['back']} pts={_points(t)}", flush=True)
    if len(_TCP) == 2:
        r = rotation(s.quat_xyzw)
        roll, pitch = math.degrees(math.atan2(r[2, 1], r[2, 2])), math.degrees(-math.asin(max(-1.0, min(1.0, r[2, 0]))))
        out = []
        for side, sy in (("left", 1.0), ("right", -1.0)):
            lug = np.asarray(s.pos) + r @ np.array([0.0, sy * LUG_Y, LUG_Z])
            tcp, fingers = _TCP[side]
            out.append(f"{side} tcp_z={tcp[2] - TOP:.3f} lug_z={lug[2] - TOP:.3f} d={np.round(tcp - lug, 3).tolist()} "
                       f"f={np.round(fingers, 4).tolist()}")
        print(f"DBG   roll={roll:.1f} pitch={pitch:.1f} " + " | ".join(out), flush=True)


def score(objects) -> float:
    """Partial credit (success scores 1.0; the best over the run counts, but a pebble in the tub latches 0): 0.5 with at
    least 36 of the 72 crumbs in the tub (or 50 before the sieve is back on the battens); 0.25 with at least 18; 0 with
    fewer, or once any pebble has gone into the tub."""
    return min(_points(_tally(objects)), 0.5)


def success(objects) -> bool:
    t = _tally(objects)
    if _DEBUG:
        _debug(objects, t)
    return _points(t) >= 1.0 and objects["sieve"].resting()


# -- scripted demo --------------------------------------------------------------------------------------------------
# Each gripper comes in level from its own side, pointing inwards with its fingers upright, slides onto its handle from
# the front and clamps it from above and below (the lower finger carries the load). Together they lift the sieve off the
# battens, shake it round in small circles, level, until the soil has gone through, set it back on the battens and let
# go, sliding off the handles to the front.


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


SIDE_ROT = {"left": _ry(-math.pi / 2) @ _rx(-math.pi / 2) @ grip_rotation(-math.pi / 2),  # level, pointing -y,
            "right": _ry(-math.pi / 2) @ _rx(math.pi / 2) @ grip_rotation(math.pi / 2)}  # fingers closing along z
GRIP_IN = 0.007  # the TCP this far in from the handle's outer edge: the fingertips (1.3 cm on) stop 5 mm short of the wall
CLAMP = float(os.environ.get("SIEVE_CLAMP", "-0.001"))  # 2 mm between the fingers round the 1 cm handle: a firm clamp
OPEN = -0.020  # 4 cm apart
SLIDE = 0.055  # start this far in front of the handle's middle, then slide on
LIFT = float(os.environ.get("SIEVE_LIFT", "0.035"))  # lift the sieve this far off the battens to shake it
AMP = float(os.environ.get("SIEVE_AMP", "0.03"))  # radius (m) of the shaking circles
HZ = float(os.environ.get("SIEVE_HZ", "2.0"))
SHAKE_S = float(os.environ.get("SIEVE_SHAKE_S", "30"))
PITCH_COMP = float(os.environ.get("SIEVE_PITCH_COMP", "4.5"))  # deg, see _arm()
REVERSE_S = float(os.environ.get("SIEVE_REVERSE_S", "2"))  # reverse the circles every 2 s: 56 of 72 through, vs 52


def _grip_point(arm: str, origin) -> np.ndarray:
    s = 1.0 if arm == "left" else -1.0
    return np.asarray(origin, float) + np.array([0.0, s * (LUG_Y + LUG[1] / 2 - GRIP_IN), LUG_Z])


def _good_ik(arm: str, pos, rot):
    from yam_arena import ik, station

    best = None
    for sd in itertools.product((0.0,), (0.8, 1.3), (0.6, 1.2), (-0.8, 0.8), (-1.2, 0.0, 1.2), (-1.5, 0.0, 1.5)):
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
    """From READY along a straight line in joint space (the big turn of the wrist stays on one IK branch)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _arm(arm: str) -> list[Waypoint]:
    # Loaded, each wrist droops and the sieve pitches back ~4.5 deg about the line between the handles (debug runs
    # sfE): while it is held up, the grips are tipped forward by as much so that it hangs level.
    level = _ry(-math.radians(PITCH_COMP)) @ SIDE_ROT[arm]

    def w(t, origin, finger, label, dx=0.0, rot=SIDE_ROT[arm]):
        p = _grip_point(arm, origin) + np.array([dx, 0.0, 0.0])
        return Waypoint(t, tuple(float(v) for v in p), finger, label, rot)

    rot = SIDE_ROT[arm]

    lifted = START + [0.0, 0.0, LIFT]
    # Come down in front of the handle, never over it: an open lower finger passing over the handle lands on it.
    high = _grip_point(arm, START) + [-SLIDE, 0.0, 0.08]
    out = _lead_in(arm, high, rot, OPEN, "level, above the front of its handle, fingers upright")
    out += [w(1.5, START, OPEN, "down beside the handle's front end", dx=-SLIDE),
            w(1.5, START, OPEN, "slide onto the handle"),
            w(1.2, START, CLAMP, "clamp it from above and below"),
            w(0.5, START, CLAMP, "steady"),
            w(2.0, lifted, CLAMP, "lift the sieve off the battens, level", rot=level)]
    quarter = 1.0 / (4.0 * HZ)
    for cycle in range(int(SHAKE_S * HZ)):
        turn = -1.0 if REVERSE_S > 0 and int(cycle / HZ / REVERSE_S) % 2 else 1.0  # reverse the circles now and then
        for ph in (1, 2, 3, 4):  # quarter periods round a small circle
            a = ph * math.pi / 2
            out.append(w(quarter, lifted + [AMP * math.cos(a), turn * AMP * math.sin(a), 0.0], CLAMP,
                         "shake it round in small circles", rot=level))
    out += [w(1.0, lifted, CLAMP, "hold it still: the last soil drops", rot=level),
            w(2.0, START + [0.0, 0.0, 0.002], CLAMP, "set it back on the battens"),
            w(1.0, START + [0.0, 0.0, 0.002], OPEN, "let go"),
            w(1.5, START, OPEN, "slide off the handle to the front", dx=-SLIDE),
            w(3.0, START, OPEN, "wait", dx=-SLIDE)]
    return out


def demo():
    return {"left": _arm("left"), "right": _arm("right")}
