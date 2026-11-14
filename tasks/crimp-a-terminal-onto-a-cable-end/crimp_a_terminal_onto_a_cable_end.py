"""On the station table stands a bench crimper with a ring terminal seated in its die, the terminal's barrel pointing
toward the arms. A short cable with a stripped end lies on a rest to its left.

Task: crimp the terminal onto the cable's end. One hand picks up the cable and pushes its stripped end into the
terminal's barrel, straight along it, and holds it there; the other presses the crimper's lever down until it closes,
then lets it spring open again. Then the first hand pulls the cable out of the crimper with the terminal crimped on its
end, holds it up to show it, and lays it back on its rest, and both arms go back to where they started. Let go of, the
cable tips in the barrel (it is loose in it until crimped), and a crimp on a tipped or wrongly inserted cable is a bad
crimp.

Run it:  inspect-robots run --embodiment yam_arena -E task=crimp_a_terminal_onto_a_cable_end.py \\
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
                     plate, rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the crimper (fixed); its frame: origin on the table under the barrel's mouth, x along the barrel (away from the
# arms), y left --------------------------------------------------------------------------------------------------------
BX, BY = 0.30, 0.0
AXIS_Z = 0.045  # the barrel's axis above the table
BAR_RI, BAR_T, BAR_L = 0.003, 0.0015, 0.016  # a 16 mm2 tube lug's barrel: 6 mm inside, 16 mm long, mouth at x = 0
FLARE_L, FLARE_R = 0.008, 0.010  # a lead-in in front of the mouth, 2 cm wide at its front
PIVOT = np.array([0.005, 0.065, AXIS_Z + 0.019])  # the lever's hinge, along x, left of the barrel
ARM_X, ARM_Z = 0.024, 0.016  # the lever arm's cross-section
ARM_END = -0.235  # the arm reaches this far along y from the hinge (to the right)
HANDLE_Y = -0.21  # the middle of its rubber grip, from the hinge: 15 cm right of the barrel
OPEN_DEG = 12.0  # the lever's stop: open this far (handle up)
SPRING_TO, SPRING = -20.0, 0.06  # its spring pulls it toward -20 degrees, so it rests on the stop: about 1.2 N m
# at closed, less the lever's weight (about 5 N on the grip)
CLOSED_WITHIN = 1.5  # degrees: closed this far, the lever crimps

# -- the cable; its frame: origin at the tip of the stripped end, the cable along -x --------------------------------
STRIP_L, STRIP_R = 0.016, 0.0025  # the bare end: 16 mm2 stranded copper, 5 mm across, stripped to the barrel's length
CABLE_L, CABLE_R = 0.16, 0.004
CABLE_MASS = 0.03
REST_Y = 0.12  # the cable's rest, left of the crimper
TIP0 = np.array([0.265, REST_Y])  # where the cable's tip lies on the rest
REST_AT = (-0.075, -0.145)  # the two V-rests, along the cable from its tip

COPPER, BLACK, RED, STEEL, DARK = (0.80, 0.50, 0.25), (0.08, 0.08, 0.09), (0.75, 0.10, 0.10), (0.62, 0.63, 0.66), \
    (0.20, 0.21, 0.23)

# -- scoring ----------------------------------------------------------------------------------------------------------
# a good crimp: the bare end this far into the barrel (at least 13 of its 16 mm in: the insulation, too wide for the
# barrel, meets the lead-in within 3 mm of its mouth) and the cable within this angle of the barrel's axis
DEPTH = (0.013, 0.0165)
ALIGN_DEG = 8.0
INSERTED = 0.008
RELEASED_DEG = 10.0  # the lever back open at least this far
LET_GO = 0.04
OUT = 0.04  # the cable's tip this far from the barrel's mouth: taken out of the crimper
HOME_WITHIN = 0.05  # each TCP this close to where it started: the arms are back home
ON_REST = (0.03, 0.015, 20.0)  # laid back on its rest: the tip this close to where it lay (across, up) and the cable
# within this many degrees of lying along its rests
SUCCESS_HOLD_S = 1.0


def _crimper() -> Body:
    shapes = [BoxShape((0.01, 0.02, 0.006), (0.09, 0.15, 0.012), DARK),  # base plate
              BoxShape((0.01, 0.0, 0.012 + (AXIS_Z - BAR_RI - BAR_T - 0.012) / 2),
                       (0.034, 0.03, AXIS_Z - BAR_RI - BAR_T - 0.012), STEEL),  # lower die block, under the barrel
              BoxShape((PIVOT[0], PIVOT[1], (0.012 + PIVOT[2] + 0.012) / 2), (0.03, 0.02, PIVOT[2]), DARK)]  # bracket
    n = 10
    for k in range(n):  # the die's flared lead-in in front of the terminal's mouth, panels round the x axis
        a = 2 * math.pi * k / n
        u = (0.0, math.cos(a), math.sin(a))
        r = BAR_RI + BAR_T / 2
        r1 = FLARE_R + BAR_T / 2
        shapes.append(plate((0.0, r * u[1], AXIS_Z + r * u[2]), (-FLARE_L, r1 * u[1], AXIS_Z + r1 * u[2]),
                            2 * r1 * math.tan(math.pi / n) + 0.001, BAR_T, u, STEEL))
    return Body.build("crimper", pos=(BX, BY, TOP), mass=1.0, shapes=shapes, kinematic=True, friction=(0.25, 0.2), slick=True,
                      color=DARK)


def _terminal_shapes(r_in: float, collide: bool) -> list:
    """A ring terminal on the barrel's axis, its mouth at x = 0: the barrel (panels round the x axis, ``r_in`` inside)
    and its flat tongue with a ring."""
    n = 10
    shapes = [BoxShape((BAR_L + 0.012, 0.0, -BAR_RI - BAR_T + 0.001), (0.024, 0.014, 0.002), COPPER, collide=collide),
              CylShape((BAR_L + 0.018, 0.0, -BAR_RI - BAR_T + 0.0021), 0.0035, 0.0003, DARK, collide=False)]  # its hole
    for k in range(n):
        a = 2 * math.pi * k / n
        u = (0.0, math.cos(a), math.sin(a))
        r = r_in + BAR_T / 2
        shapes.append(plate((0.0, r * u[1], r * u[2]), (BAR_L, r * u[1], r * u[2]), 2 * r * math.tan(math.pi / n) + 0.001,
                            BAR_T, u, COPPER, collide=collide))
    return shapes


TERM_AT = np.array([BX, BY, TOP + AXIS_Z])  # the terminal's origin (its mouth, on the axis) as it sits in the die
HIDE = np.array([BX, BY + 0.05, TOP - 0.03])  # under the table


def _terminal() -> Body:
    return Body.build("terminal", pos=tuple(TERM_AT.tolist()), mass=0.01, shapes=_terminal_shapes(BAR_RI, True),
                      kinematic=True, friction=(0.3, 0.25), slick=True, color=COPPER)  # tinned copper, smooth inside


def _crimped() -> Body:
    """The terminal crimped (its barrel squeezed onto the copper): shown once crimped, following the cable."""
    return Body.build("crimped_terminal", pos=tuple(HIDE.tolist()), mass=0.01, kinematic=True, color=COPPER,
                      shapes=_terminal_shapes(STRIP_R + 0.0008, False))


def _lever() -> Body:
    """Origin on the hinge; the arm along -y, level when closed."""
    arm_len = -ARM_END
    shapes = [BoxShape((0.0, -arm_len / 2 + 0.01, 0.0), (ARM_X, arm_len + 0.02, ARM_Z), DARK),
              BoxShape((0.0, HANDLE_Y, 0.0), (ARM_X + 0.006, 0.05, ARM_Z + 0.006), RED),  # rubber grip
              BoxShape((BAR_L / 2 - PIVOT[0], -PIVOT[1], -(PIVOT[2] - AXIS_Z - BAR_RI - BAR_T) / 2),
                       (0.02, 0.014, PIVOT[2] - AXIS_Z - BAR_RI - BAR_T), STEEL, collide=False)]  # upper die
    a = math.radians(-OPEN_DEG) / 2
    return Body.build("lever", pos=(BX + PIVOT[0], BY + PIVOT[1], TOP + PIVOT[2]), mass=0.15, shapes=shapes,
                      friction=(1.0, 0.9), color=DARK, start_xyzw=(math.sin(a), 0.0, 0.0, math.cos(a)))


def _rest() -> Body:
    shapes = []
    for x in REST_AT:
        h = AXIS_Z - CABLE_R
        shapes += [BoxShape((x, 0.0, h / 2), (0.016, 0.03, h), DARK),
                   BoxShape((x, 0.0075, h + 0.003), (0.016, 0.006, 0.006), DARK),  # lips either side of the cable
                   BoxShape((x, -0.0075, h + 0.003), (0.016, 0.006, 0.006), DARK)]
    return Body.build("rest", pos=(TIP0[0], TIP0[1], TOP), mass=0.5, shapes=shapes, kinematic=True,
                      friction=(0.6, 0.5), color=DARK)


def _cable() -> Body:
    shapes = [CylShape((-STRIP_L / 2, 0.0, 0.0), STRIP_R, STRIP_L, COPPER, axis="X"),
              CylShape((-STRIP_L - (CABLE_L - STRIP_L) / 2, 0.0, 0.0), CABLE_R, CABLE_L - STRIP_L, BLACK, axis="X")]
    return Body.build("cable", pos=(TIP0[0], TIP0[1], TOP + AXIS_Z + 0.0005), mass=CABLE_MASS, shapes=shapes,
                      friction=(1.0, 0.9), color=BLACK, com=(-CABLE_L / 2, 0.0, 0.0))


CRIMPER, LEVER, REST, CABLE = _crimper(), _lever(), _rest(), _cable()
OBJECTS = [CRIMPER, LEVER, REST, CABLE, _terminal(), _crimped()]
hinge("lever_pin", "crimper", "lever", anchor=(BX + PIVOT[0], BY + PIVOT[1], TOP + PIVOT[2]), axis="X",
      limits_deg=(-OPEN_DEG, 0.0), spring=SPRING, damping=0.002, target_deg=SPRING_TO)


# -- followed after every control step: the crimp ------------------------------------------------------------------

def _fresh() -> dict:
    return {"crimped": False, "good": False, "depth": None, "align": None, "best_depth": -1.0, "tcps": None,
            "lever_deg": -OPEN_DEG, "handle": None, "on_cable": None, "tip": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("CRIMP_DEBUG")), "steps": 0}


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


def _lever_deg(l_rot) -> float:
    """The lever's turn about x from closed (negative: open)."""
    return math.degrees(math.atan2(l_rot[2, 1], l_rot[1, 1]))


def _insertion(c_pos, c_rot) -> tuple[float, float, float]:
    """(how far the cable's tip is past the barrel's mouth, along the barrel; the cable's angle from the barrel's axis,
    degrees; the tip's distance from that axis)."""
    tip = np.asarray(c_pos, float) - np.array([BX, BY, TOP + AXIS_Z])
    axis = c_rot @ np.array([1.0, 0.0, 0.0])
    return float(tip[0]), math.degrees(math.acos(max(-1.0, min(1.0, float(axis[0]))))), math.hypot(tip[1], tip[2])


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


def _place(base, origin, name, pos, rot) -> None:
    import torch

    asset = base.scene[name]
    w, x, y, z = _quat_wxyz(np.asarray(rot, float))
    p = np.asarray(pos, float) + origin
    asset.write_root_pose_to_sim(torch.tensor([[*p.tolist(), x, y, z, w]], dtype=torch.float32, device=asset.device))


def _crimp(base, origin, c_pos, c_rot) -> None:
    """The crimp: the terminal is squeezed onto the cable's end; from now on it goes where the cable goes."""
    _EP["on_cable"] = (c_rot.T @ (TERM_AT - c_pos), c_rot.T)  # the terminal (unrotated) in the cable's frame
    _place(base, origin, "terminal", HIDE, np.eye(3))
    _place(base, origin, "crimped_terminal", TERM_AT, np.eye(3))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    c = as_numpy(base.scene["cable"].data.root_link_pose_w)[0]
    lv = as_numpy(base.scene["lever"].data.root_link_pose_w)[0]
    c_pos, c_rot, l_pos, l_rot = c[:3] - origin, rotation(c[3:7]), lv[:3] - origin, rotation(lv[3:7])
    depth, align, off = _insertion(c_pos, c_rot)
    if off < BAR_RI:
        _EP["best_depth"] = max(_EP["best_depth"], depth)
    _EP["lever_deg"] = _lever_deg(l_rot)
    _EP["handle"] = l_pos + l_rot @ np.array([0.0, HANDLE_Y, 0.0])
    _EP["tcps"] = _tcps(base, origin)
    if not _EP["crimped"] and _EP["lever_deg"] > -CLOSED_WITHIN:
        _EP["crimped"] = True
        _EP["depth"], _EP["align"] = depth, align
        _EP["good"] = DEPTH[0] <= depth <= DEPTH[1] and align <= ALIGN_DEG and off < BAR_RI
        if depth > 0.0 and off < BAR_RI:
            _crimp(base, origin, c_pos, c_rot)
    elif _EP["on_cable"] is not None:  # the crimped terminal goes with the cable
        p_rel, r_rel = _EP["on_cable"]
        _place(base, origin, "crimped_terminal", c_pos + c_rot @ p_rel, c_rot @ r_rel)
    _EP["tip"] = c_pos
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status()
            tl, tr = _EP["tcps"]
            print(f"[crimp] t={_DEBUG['steps'] / 20:5.1f}s lever={_EP['lever_deg']:6.1f} depth={depth * 1000:6.1f}mm "
                  f"align={align:5.1f} axis={np.round(c_rot[:, 0], 3).tolist()} off={off * 1000:5.1f}mm crimped={_EP['crimped']} good={_EP['good']} "
                  f"at_crimp=({None if _EP['depth'] is None else round(_EP['depth'] * 1000, 1)}mm, "
                  f"{None if _EP['align'] is None else round(_EP['align'], 1)}deg) "
                  f"leftTCP-tip={np.round((tl - c_pos) * 1000).astype(int).tolist()} "
                  f"rightTCP-grip={np.round((tr - _EP['handle']) * 1000).astype(int).tolist()} score={_points(st)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    tcps = _EP["tcps"]
    let_go = tcps is None or _EP["handle"] is None or all(float(np.linalg.norm(t - _EP["handle"])) >= LET_GO
                                                           for t in tcps)
    tip = _EP["tip"]
    out = tip is not None and float(np.linalg.norm(tip - TERM_AT)) >= OUT
    return {"crimped": _EP["crimped"] and _EP["depth"] is not None and _EP["depth"] > 0.0, "good": _EP["good"],
            "released": _EP["lever_deg"] <= -RELEASED_DEG and let_go, "inserted": _EP["best_depth"] >= INSERTED,
            "out": out and _EP["on_cable"] is not None}


def _points(st) -> float:
    if st["good"] and st["released"] and st["out"]:
        return 1.0
    if st["crimped"]:
        return 0.5
    return 0.25 if st["inserted"] else 0.0


def score(objects) -> float:
    """1.0 for a good crimp (the bare end 13 to 16.5 mm into the barrel and the cable within 8 degrees of its axis when
    the lever closed) with the lever let go and back open, the cable taken out of the crimper with the terminal on it
    (its tip at least 4 cm from where the barrel's mouth was) and laid back on its rest, and both arms back where they
    started; 0.5 for any crimp with the bare end in the barrel; 0.25 if the end was pushed at least 8 mm into the barrel
    at some point; else 0.0."""
    pts = _points(_status())
    if pts >= 1.0 and not (_on_rest(objects) and _arms_home()):
        return 0.5  # crimped and taken out, but not laid back on its rest or the arms not back
    return pts


def _on_rest(objects) -> bool:
    """The cable laid back on its rest: its tip within 3 cm (across) and 1.5 cm (up or down) of where it lay, and the
    cable within 20 degrees of lying along the rests."""
    c = objects["cable"]
    tip, axis = np.asarray(c.pos, float), rotation(c.quat_xyzw)[:, 0]
    return (float(np.linalg.norm(tip[:2] - TIP0)) <= ON_REST[0] and abs(float(tip[2]) - (TOP + AXIS_Z)) <= ON_REST[1]
            and abs(float(axis[0])) >= math.cos(math.radians(ON_REST[2])))


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
    """Full score with the lever and the cable still: the trial ends there."""
    return score(objects) >= 1.0 and objects["lever"].resting() and objects["cable"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the cable 5.5 cm from its tip, fingers closing across it, lifts it off its rest,
# carries it to the barrel's axis 2 cm in front of the mouth and pushes the bare end straight in (aiming 3 mm past full
# depth: the insulation stops on the lead-in with about 14 mm of the copper in the barrel), and holds it. The right hand
# comes straight down on the lever's rubber grip, fingers closed, and presses the lever down until it closes and crimps;
# then it rises, the lever springs open, and the right goes back to where it started. The left pulls the cable straight
# back out with the terminal crimped on its end, holds it up to show it, lays it back on its rest and goes back to where
# it started.

GRIP_L = grip_rotation(0.0)  # straight down, fingers closing along y (across the cable, which runs along x)
GRIP_R = grip_rotation(0.0)  # straight down, fingers closed, pressing the grip
HOLD_AT = 0.055  # the left TCP this far back from the cable's tip
IN_DEPTH = 0.019  # aim 3 mm past full depth: the insulation stops the cable at the barrel's mouth
PRESS = 0.015  # aim this far below the closed lever's grip (the arm gives; the spring pushes back about 5 N)
SAG = 0.006  # the held cable hangs this far below the left TCP (the jaw pushes it toward the fingertips): aim above
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
    """The top of the lever's rubber grip (world) with the lever at ``deg``."""
    a = math.radians(deg)
    p = np.array([0.0, HANDLE_Y * math.cos(a), HANDLE_Y * math.sin(a)])
    return np.array([BX, BY, TOP]) + PIVOT + p + [0.0, 0.0, (ARM_Z + 0.006) / 2]


def _left() -> tuple[list[Waypoint], float]:
    pick = np.array([TIP0[0] - HOLD_AT, TIP0[1], TOP + AXIS_Z + 0.0005])
    front = np.array([BX - 0.02 - HOLD_AT, BY, TOP + AXIS_Z + SAG])
    held = np.array([BX + IN_DEPTH - HOLD_AT, BY, TOP + AXIS_Z + SAG])
    out = _lead_in("left", pick + [0.0, 0.0, 0.05], GRIP_L, "over the cable", FINGER_OPEN)
    out += [_w(1.0, pick, FINGER_OPEN, "down round it", GRIP_L),
            _w(0.8, pick, FINGER_CLOSED, "grip it", GRIP_L),
            _w(0.8, pick + [0.0, 0.0, 0.02], FINGER_CLOSED, "lift it off the rest", GRIP_L),
            _w(1.5, front, FINGER_CLOSED, "line it up with the barrel, 2 cm in front of the mouth", GRIP_L),
            _w(1.5, held, FINGER_CLOSED, "push the bare end straight in until the insulation meets the barrel", GRIP_L)]
    return out, sum(w.duration for w in out)


def _right(start: float) -> list[Waypoint]:
    above = _grip_top(-OPEN_DEG) + [0.0, 0.0, 0.04]
    out = _lead_in("right", above, GRIP_R, "over the lever's grip", FINGER_CLOSED)
    t = sum(w.duration for w in out)
    out.append(_w(max(start - t, 0.2), above, FINGER_CLOSED, "wait for the cable to be held in the barrel", GRIP_R))
    press = _grip_top(0.0) - [0.0, 0.0, PRESS]
    out += [_w(0.8, _grip_top(-OPEN_DEG) + [0.0, 0.0, 0.005], FINGER_CLOSED, "onto the grip", GRIP_R),
            _w(1.5, press, FINGER_CLOSED, "press the lever down until it closes: the crimp", GRIP_R),
            _w(0.8, press, FINGER_CLOSED, "hold it closed", GRIP_R),
            _w(1.5, above, FINGER_CLOSED, "rise: the lever springs open", GRIP_R)]
    return out


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


PULL_OUT = 0.06  # the left pulls the cable this far straight back out of the crimper
SHOW_UP = 0.06  # ... and lifts it this much, to show the terminal on its end


def demo():
    left, t_held = _left()
    right = _right(t_held + 0.3)
    t_r = sum(w.duration for w in right)
    right += _go_home("right", right[-1].pos, GRIP_R, FINGER_CLOSED)
    held = np.asarray(left[-1].pos)
    pulled = held - [PULL_OUT, 0.0, 0.0]
    shown = pulled + [0.0, 0.0, SHOW_UP]
    rest = np.array([TIP0[0] - HOLD_AT, TIP0[1], TOP + AXIS_Z + SAG])  # the cable back on its rest
    left += [_w(max(t_r - t_held, 0.2) + 0.5, held, FINGER_CLOSED, "hold the cable in the barrel", GRIP_L),
             _w(1.5, pulled, FINGER_CLOSED, "pull it straight out: the terminal comes with it, crimped on", GRIP_L),
             _w(1.0, shown, FINGER_CLOSED, "hold it up to show the terminal", GRIP_L),
             _w(1.5, shown, FINGER_CLOSED, "hold it up to show the terminal", GRIP_L),
             _w(1.5, rest + [0.0, 0.0, 0.03], FINGER_CLOSED, "over its rest", GRIP_L),
             _w(0.8, rest, FINGER_CLOSED, "lay it on its rest", GRIP_L),
             _w(0.5, rest, FINGER_OPEN, "let go", GRIP_L),
             _w(0.8, rest + [0.0, 0.0, 0.05], FINGER_OPEN, "rise", GRIP_L)]
    left += _go_home("left", rest + [0.0, 0.0, 0.05], GRIP_L, FINGER_OPEN)
    t_l = sum(w.duration for w in left)
    left.append(_w(3.0, left[-1].pos, FINGER_OPEN,
                   "hold still: the trial ends once the full score has held 1 s with both arms home", left[-1].yaw))
    t_r = sum(w.duration for w in right)
    right.append(_w(max(t_l - t_r, 0.5) + 3.0, right[-1].pos, FINGER_CLOSED, "wait at the start", right[-1].yaw))
    return {"left": left, "right": right}
