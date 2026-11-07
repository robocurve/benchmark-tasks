"""On the station table stands a benchtop wire-stripping station: a block with a hole in its top, whose blades close
round whatever is in the hole while its lever is held down. A short cable with a plug on one end stands upright in a
holder beside it.

Task: strip 10 mm of insulation off the wire's end. One hand puts the wire's end down into the hole, 10 mm deep (as deep
as it goes in, so much is stripped); the other presses the lever down, closing the blades, and holds it down while the
first pulls the wire straight back up: the cut insulation stays behind in the station, and the copper is bare. Let the
lever up before the wire is out and the blades let go: the cut insulation comes back out on the wire.

Run it:  inspect-robots run --embodiment yam_arena -E task=strip_a_wire_end_in_a_stripping_station.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, glue, hinge, on_reset,  # noqa: E402
                     on_step, plate, rotation, set_glue_on)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the stripping station (fixed): a block with a hole down into it; origin on the table under the hole ---------------
SX, SY = 0.24, 0.12
BLOCK = (0.05, 0.05, 0.04)  # x, y, tall: the blades are at its top
HOLE_RI = 0.008  # the hole: 16 mm across, down to 5 mm above the table
FUNNEL_R, FUNNEL_L = 0.022, 0.010  # a wide lead-in at its mouth: 4.4 cm across
PIVOT = np.array([0.0, -BLOCK[1] / 2 - 0.005, BLOCK[2] + 0.012])  # the lever's hinge, along x, at the block's right side
ARM_X, ARM_Z = 0.024, 0.016
ARM_END = -0.225  # the lever's arm reaches this far along y from the hinge (to the right)
HANDLE_Y = -0.20  # the middle of its rubber grip, from the hinge: 23 cm right of the hole
OPEN_DEG = 12.0
SPRING_TO, SPRING = -20.0, 0.06  # pulled toward -20 degrees, so it rests on its open stop (about 5 N to hold it down)
CLOSED_WITHIN = 1.5  # degrees: the blades are closed
OPENED = 4.0  # degrees: let up this far, they let go

# -- the cable: stiff, 12 mm across; the insulation round its last 24 mm is 12 rings, each held to it until cut --------
WIRE_L, WIRE_R, CORE_R = 0.12, 0.006, 0.004
RING_H, N_RINGS = 0.002, 12
WIRE_MASS = 0.04
PLUG, PLUG_H = 0.016, 0.03  # a square plug moulded on the cable's other end (on top), 16 mm across
HOLDER_AT = (0.30, 0.21)
HOLDER_R, HOLDER_H = 0.0055, 0.018  # snug round the copper end: the cable stands straight
BLACK, COPPER, STEEL, DARK, RED = (0.08, 0.08, 0.09), (0.80, 0.50, 0.25), (0.62, 0.63, 0.66), (0.20, 0.21, 0.23), \
    (0.75, 0.10, 0.10)

# -- scoring ----------------------------------------------------------------------------------------------------------
STRIP = (0.008, 0.012)  # a good strip: this much insulation off
INSERTED = 0.005
CLEAR = 0.002  # pulled off: the wire's end this far above the blades
RELEASED_DEG = 10.0
LET_GO = 0.04
SUCCESS_HOLD_S = 1.0


def _station() -> Body:
    bx, by, bz = BLOCK
    s = HOLE_RI + 0.002  # half the square shaft the hole's tube sits in
    zw = bz - FUNNEL_L  # the block's walls stop under the funnel
    shapes = [BoxShape((0.0, 0.0, 0.0025), (bx, by, 0.005), DARK),  # base, under the hole
              BoxShape(((bx / 2 + s) / 2, 0.0, zw / 2), (bx / 2 - s, by, zw), STEEL),
              BoxShape((-(bx / 2 + s) / 2, 0.0, zw / 2), (bx / 2 - s, by, zw), STEEL),
              BoxShape((0.0, (by / 2 + s) / 2, zw / 2), (2 * s, by / 2 - s, zw), STEEL),
              BoxShape((0.0, -(by / 2 + s) / 2, zw / 2), (2 * s, by / 2 - s, zw), STEEL),
              BoxShape((PIVOT[0], PIVOT[1], (bz + PIVOT[2] + 0.008) / 2), (0.02, 0.01, PIVOT[2] + 0.008 - bz), DARK)]
    n = 10
    for k in range(n):  # the hole's tube and its funnel
        a = 2 * math.pi * k / n
        u = (math.cos(a), math.sin(a), 0.0)
        r = HOLE_RI + 0.0008
        shapes.append(plate((r * u[0], r * u[1], 0.005), (r * u[0], r * u[1], bz - FUNNEL_L),
                            2 * r * math.tan(math.pi / n) + 0.001, 0.0016, u, STEEL))
        r1 = FUNNEL_R + 0.0008
        shapes.append(plate((r * u[0], r * u[1], bz - FUNNEL_L), (r1 * u[0], r1 * u[1], bz), 2 * r1 * math.tan(math.pi / n) + 0.001,
                            0.0016, u, STEEL))
    return Body.build("stripper", pos=(SX, SY, TOP), mass=1.0, shapes=shapes, kinematic=True, friction=(0.3, 0.25),
                      color=STEEL)


def _lever() -> Body:
    arm = -ARM_END
    shapes = [BoxShape((0.0, -arm / 2 + 0.01, 0.0), (ARM_X, arm + 0.02, ARM_Z), DARK),
              BoxShape((0.0, HANDLE_Y, 0.0), (ARM_X + 0.006, 0.05, ARM_Z + 0.006), RED)]
    a = math.radians(-OPEN_DEG) / 2
    return Body.build("lever", pos=(SX + PIVOT[0], SY + PIVOT[1], TOP + PIVOT[2]), mass=0.15, shapes=shapes,
                      friction=(1.0, 0.9), color=DARK, start_xyzw=(math.sin(a), 0.0, 0.0, math.cos(a)))


def _holder() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.002), HOLDER_R + 0.004, 0.004, DARK)]
    n = 10
    for k in range(n):
        a = 2 * math.pi * k / n
        u = (math.cos(a), math.sin(a), 0.0)
        r = HOLDER_R + 0.001
        shapes.append(plate((r * u[0], r * u[1], 0.004), (r * u[0], r * u[1], HOLDER_H), 2 * r * math.tan(math.pi / n) + 0.001,
                            0.002, u, DARK))
    return Body.build("holder", pos=(HOLDER_AT[0], HOLDER_AT[1], TOP), mass=0.1, shapes=shapes, kinematic=True,
                      friction=(0.3, 0.25), color=DARK)


WIRE_Z0 = TOP + 0.0045  # the wire's end, standing in its holder


def _wire() -> Body:
    """Origin at the wire's end (its bottom, standing upright); the copper runs up through the rings."""
    strip = RING_H * N_RINGS
    cable = WIRE_L - PLUG_H
    shapes = [CylShape((0.0, 0.0, strip / 2), CORE_R, strip, COPPER),
              CylShape((0.0, 0.0, strip + (cable - strip) / 2), WIRE_R, cable - strip, BLACK),
              BoxShape((0.0, 0.0, cable + PLUG_H / 2), (PLUG, PLUG, PLUG_H), (0.55, 0.56, 0.58))]
    return Body.build("wire", pos=(HOLDER_AT[0], HOLDER_AT[1], WIRE_Z0), mass=WIRE_MASS, shapes=shapes,
                      friction=(1.0, 0.9), color=BLACK, com=(0.0, 0.0, WIRE_L / 2))


def _rings() -> list[Body]:
    """Each ring is just to be seen (it does not collide): held to the wire, or once cut and the wire pulled out, held
    in the station's blades."""
    return [Body.build(f"ring_{k:02d}", pos=(HOLDER_AT[0], HOLDER_AT[1], WIRE_Z0 + (k + 0.5) * RING_H), mass=0.0005,
                       shapes=[CylShape((0.0, 0.0, 0.0), WIRE_R, RING_H * 0.98, BLACK, collide=False)], color=BLACK)
            for k in range(N_RINGS)]


STATION, LEVER, HOLDER, WIRE = _station(), _lever(), _holder(), _wire()
RINGS = _rings()
RING_NAMES = [r.name for r in RINGS]
OBJECTS = [STATION, LEVER, HOLDER, WIRE, *RINGS]
hinge("lever_pin", "stripper", "lever", anchor=(SX + PIVOT[0], SY + PIVOT[1], TOP + PIVOT[2]), axis="X",
      limits_deg=(-OPEN_DEG, 0.0), spring=SPRING, damping=0.002, target_deg=SPRING_TO)
for _r in RINGS:
    glue(f"{_r.name}_on_wire", "wire", _r.name, anchor=_r.pos)
    glue(f"{_r.name}_in_blades", "stripper", _r.name, anchor=_r.pos)  # off until cut


# -- followed after every control step: the cut and the pull ------------------------------------------------------------

def _fresh() -> dict:
    return {"cut": 0, "depth": None, "stripped": False, "best_depth": -1.0, "tcps": None, "lever_deg": -OPEN_DEG,
            "handle": None, "wire_moved": False}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("STRIP_DEBUG")), "steps": 0}


def _joint(name: str):
    import bodykit
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    return UsdPhysics.FixedJoint(get_current_stage().GetPrimAtPath(bodykit._GLUE_PATHS[name]))


def _quat_wxyz(m: np.ndarray) -> tuple[float, float, float, float]:
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    if w > 1e-6:
        return w, (m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w)
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0.0, 1.0 + m[i, i] - m[j, j] - m[k, k])) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[0] = (m[k, j] - m[j, k]) / s
    q[1 + i] = s / 4
    q[1 + j] = (m[j, i] + m[i, j]) / s
    q[1 + k] = (m[k, i] + m[i, k]) / s
    return tuple(q)


def _hold(name: str, b0_pos, b0_rot, r_pos, r_rot) -> None:
    """Switch a ring's glue on where the ring is now, against its body0 as it is now."""
    from pxr import Gf

    j = _joint(name)
    j.GetLocalPos0Attr().Set(Gf.Vec3f(*(b0_rot.T @ (np.asarray(r_pos) - np.asarray(b0_pos))).tolist()))
    j.GetLocalRot0Attr().Set(Gf.Quatf(*_quat_wxyz(b0_rot.T @ r_rot)))
    j.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    j.GetLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    set_glue_on(name, True)


_START_FRAMES: dict = {}


@on_reset
def _new_episode() -> None:
    from pxr import Gf

    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    for k, r in enumerate(RINGS):  # every ring back on the wire as it started, none in the blades
        name = f"{r.name}_on_wire"
        j = _joint(name)
        if name not in _START_FRAMES:
            _START_FRAMES[name] = (j.GetLocalPos0Attr().Get(), j.GetLocalRot0Attr().Get(), j.GetLocalPos1Attr().Get(),
                                   j.GetLocalRot1Attr().Get())
        p0, q0, p1, q1 = _START_FRAMES[name]
        j.GetLocalPos0Attr().Set(Gf.Vec3f(p0))
        j.GetLocalRot0Attr().Set(Gf.Quatf(q0))
        j.GetLocalPos1Attr().Set(Gf.Vec3f(p1))
        j.GetLocalRot1Attr().Set(Gf.Quatf(q1))
        set_glue_on(name, False)
        set_glue_on(name, True)
        set_glue_on(f"{r.name}_in_blades", False)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _lever_deg(l_rot) -> float:
    return math.degrees(math.atan2(l_rot[2, 1], l_rot[1, 1]))


def _depth(w_pos, w_rot) -> tuple[float, float]:
    """(how far the wire's end is below the blades, in the hole; how far it is off the hole's axis)."""
    end = np.asarray(w_pos, float) - np.array([SX, SY, TOP])
    return float(BLOCK[2] - end[2]), math.hypot(end[0], end[1])


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])

    def pose(name):
        p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
        return p[:3] - origin, rotation(p[3:7])

    w_pos, w_rot = pose("wire")
    l_pos, l_rot = pose("lever")
    s_pos, s_rot = np.array([SX, SY, TOP]), np.eye(3)
    depth, off = _depth(w_pos, w_rot)
    if off < HOLE_RI:
        _EP["best_depth"] = max(_EP["best_depth"], depth)
    _EP["lever_deg"] = _lever_deg(l_rot)
    _EP["handle"] = l_pos + l_rot @ np.array([0.0, HANDLE_Y, 0.0])
    _EP["tcps"] = _tcps(base, origin)
    closed = _EP["lever_deg"] > -CLOSED_WITHIN
    if closed and _EP["cut"] == 0 and not _EP["stripped"] and off < HOLE_RI and depth > RING_H / 2:
        n = min(N_RINGS, int(round(depth / RING_H)))  # the rings below the blades are cut off: the blades hold them
        _EP["cut"], _EP["depth"] = n, depth
        for r in RINGS[:n]:
            r_pos, r_rot = pose(r.name)
            set_glue_on(f"{r.name}_on_wire", False)
            _hold(f"{r.name}_in_blades", s_pos, s_rot, r_pos, r_rot)
    if _EP["cut"] and not _EP["stripped"]:
        if depth < -CLEAR:  # pulled clear while the blades held the cut insulation: stripped
            _EP["stripped"] = True
        elif _EP["lever_deg"] < -OPENED:  # let up too soon: the cut insulation stays on the wire
            for r in RINGS[:_EP["cut"]]:
                r_pos, r_rot = pose(r.name)
                set_glue_on(f"{r.name}_in_blades", False)
                _hold(f"{r.name}_on_wire", w_pos, w_rot, r_pos, r_rot)
            _EP["cut"] = 0
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status()
            tl, tr = _EP["tcps"]
            print(f"[strip] t={_DEBUG['steps'] / 20:5.1f}s lever={_EP['lever_deg']:6.1f} depth={depth * 1000:6.1f}mm "
                  f"off={off * 1000:4.1f}mm cut={_EP['cut']} stripped={_EP['stripped']} "
                  f"leftTCP-end={np.round((tl - w_pos) * 1000).astype(int).tolist()} "
                  f"rightTCP-grip={np.round((tr - _EP['handle']) * 1000).astype(int).tolist()} score={_points(st)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    tcps = _EP["tcps"]
    let_go = tcps is None or _EP["handle"] is None or all(float(np.linalg.norm(t - _EP["handle"])) >= LET_GO
                                                           for t in tcps)
    strip = _EP["cut"] * RING_H
    return {"stripped": _EP["stripped"], "good": STRIP[0] - 1e-9 <= strip <= STRIP[1] + 1e-9, "strip": strip,
            "released": _EP["lever_deg"] <= -RELEASED_DEG and let_go, "inserted": _EP["best_depth"] >= INSERTED}


def _points(st) -> float:
    if st["stripped"] and st["good"] and st["released"]:
        return 1.0
    if st["stripped"] or (st["good"] and _EP["cut"]):
        return 0.5
    return 0.25 if st["inserted"] else 0.0


def score(objects) -> float:
    """1.0 with 8 to 12 mm of insulation stripped off (cut by the closed blades and left in the station as the cable was
    pulled out) and the lever let go (no TCP within 4 cm of its grip) and back up; 0.5 for any strip, or for a cut of the right length with the blades
    still closed on it; 0.25 if the cable's end went at least 5 mm into the hole at some point; else 0.0."""
    return _points(_status())


def success(objects) -> bool:
    """Full score with the lever and the wire still."""
    return score(objects) >= 1.0 and objects["lever"].resting() and objects["wire"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the cable's plug, fingers closing across it, lifts it out of its holder,
# carries it over the station and lowers its end 10 mm into the hole. The right hand comes straight down on the lever's
# rubber grip, fingers closed, and presses the lever down, closing the blades, and holds it there while the left pulls
# the wire straight up out of the hole. Then the right rises and the lever springs back up.

GRIP = grip_rotation(0.0)  # straight down: the left's fingers close along y, across the wire; the right's are closed
HOLD_DOWN = 0.015  # the left TCP this far below the plug's top (the plug's flat sides hold)
DEPTH = 0.009  # aim 9 mm: the end goes in about 1 mm further
PRESS = 0.015
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


def _grip_top(deg: float) -> np.ndarray:
    a = math.radians(deg)
    p = np.array([0.0, HANDLE_Y * math.cos(a), HANDLE_Y * math.sin(a)])
    return np.array([SX, SY, TOP]) + PIVOT + p + [0.0, 0.0, (ARM_Z + 0.006) / 2]


HAND_UP = WIRE_L - HOLD_DOWN  # the left TCP above the wire's end
HANG = np.array([0.0, 0.0, 0.0])  # held by its plug, the cable hangs straight under the left TCP


def _left() -> tuple[list[Waypoint], float, np.ndarray, float]:
    pick = np.array([HOLDER_AT[0], HOLDER_AT[1], WIRE_Z0 + HAND_UP])
    over = np.array([SX, SY, TOP + BLOCK[2] + 0.03 + HAND_UP]) - HANG  # its end 3 cm above the station's top
    down = np.array([SX, SY, TOP + BLOCK[2] - DEPTH + HAND_UP]) - HANG
    out = _lead_in("left", pick + [0.0, 0.0, 0.03], GRIP, "over the wire", FINGER_OPEN)
    out += [_w(1.0, pick, FINGER_OPEN, "down round it", GRIP),
            _w(0.8, pick, FINGER_CLOSED, "take it", GRIP),
            _w(2.5, np.array([pick[0], pick[1], over[2]]), FINGER_CLOSED, "lift it straight out of its holder, high", GRIP),
            _w(2.5, over, FINGER_CLOSED, "over the station's hole (slowly: it hangs in the hand)", GRIP),
            _w(0.8, over, FINGER_CLOSED, "let it settle", GRIP),
            _w(2.0, down, FINGER_CLOSED, "its end 10 mm down into the hole", GRIP)]
    return out, sum(w.duration for w in out), down, 0.0


def demo():
    left, t_in, down, _ = _left()
    above = _grip_top(-OPEN_DEG) + [0.0, 0.0, 0.04]
    right = _lead_in("right", above, GRIP, "over the lever's grip", FINGER_CLOSED)
    t = sum(w.duration for w in right)
    press = _grip_top(0.0) - [0.0, 0.0, PRESS]
    right += [_w(max(t_in - t, 0.2) + 0.3, above, FINGER_CLOSED, "wait for the wire to be in the hole", GRIP),
              _w(0.8, _grip_top(-OPEN_DEG) + [0.0, 0.0, 0.005], FINGER_CLOSED, "onto the grip", GRIP),
              _w(1.5, press, FINGER_CLOSED, "press the lever down: the blades close", GRIP)]
    t_closed = sum(w.duration for w in right)
    right.append(_w(3.0, press, FINGER_CLOSED, "hold it down while the wire is pulled out", GRIP))
    t_pull = t_closed + 0.6
    right += [_w(1.5, above, FINGER_CLOSED, "rise: the lever springs back up", GRIP)]
    left += [_w(max(t_pull - t_in, 0.2), down, FINGER_CLOSED, "hold it while the blades close", GRIP),
             _w(1.5, down + [0.0, 0.0, 0.035], FINGER_CLOSED, "pull it straight up out of the hole", GRIP)]
    t_l, t_r = sum(w.duration for w in left), sum(w.duration for w in right)
    hold = "hold still: the trial ends once the full score has held 1 s"
    left.append(_w(max(t_r - t_l, 0.2) + 3.0, left[-1].pos, FINGER_CLOSED, hold, GRIP))
    t_l = sum(w.duration for w in left)
    right.append(_w(max(t_l - sum(w.duration for w in right), 0.5), above, FINGER_CLOSED, "wait", GRIP))
    return {"left": left, "right": right}
