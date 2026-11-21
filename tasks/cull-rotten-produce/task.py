"""Take the rotten vegetables (two tomatoes) out of a produce container and put them in the waste bin, leaving the fresh produce.

On the station table stand two fixed open containers: a produce container (a Rubbermaid FreshWorks 3-gallon produce
saver, 18 x 12 x 6 5/16 in, its lid off) and, to the right of it, a waste bin (an OXO Good Grips Easy-Clean 1.75
gallon compost bin, 8.3 x 10.15 x 12 in, its lid open). In the produce container, packed with about 10 mm between
neighbours, lie two red bell peppers, a zucchini and two rotten tomatoes (a dark patch beside the stem, facing
the front of the table). One rotten
tomato is wedged between the two peppers, the other between a pepper and the container's right wall.

What makes it hard: each tomato is 86-88 mm across and the gripper opens 94 mm, so the fingers have about 3 mm either
side of it; and the tomato stands 10 mm from its neighbours along the row, which keeps the fingers out of the row: they
have to go down in front of and behind it, closing across the row, and the tomato must be lifted clear without
pushing the peppers. The waste bin's rim is 305 mm above the table, higher than a straight-down grip reaches (about
0.17 m here), so the hand has to tilt to carry the tomato over it.

Simulation: every piece of produce is one rigid body with a convex-hull collider (marso SimReady assets by M-XR,
CC BY 4.0: rotten_tomato, red_bell_pepper, zucchini; no squashing, no juice). The containers are fixed boxes.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ...
"""

from __future__ import annotations

import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from scenekit import Shape, UsdBody, arm_joints, on_reset, on_step, robot_links  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Take the rotten vegetables out of the produce container and put them in the waste bin."

MM = 0.001
TOP = TABLE_TOP[SCENE]

# -- the produce container: Rubbermaid FreshWorks 3 gal, 18 x 12 x 6 5/16 in outside (457 x 305 x 160 mm; Webstaurant-
# Store listing 6902052935), its long side along y. Wall and floor 3 mm (not published: an estimate). Fixed, lid off.
PC_XY = (0.360, -0.030)
PC_X, PC_Y, PC_H = 305 * MM, 457 * MM, 160 * MM
PC_WALL, PC_FLOOR = 3 * MM, 3 * MM
PC_IN_X, PC_IN_Y = PC_X - 2 * PC_WALL, PC_Y - 2 * PC_WALL
PC_RIM = TOP + PC_H
PC_FLOOR_TOP = TOP + PC_FLOOR
PC_COLOR = (0.80, 0.85, 0.82)
PRODUCE_CONTAINER = [
    Shape("pc_floor", pos=(PC_XY[0], PC_XY[1], TOP + PC_FLOOR / 2), size=(PC_X, PC_Y, PC_FLOOR), color=PC_COLOR),
    Shape("pc_wall_far", pos=(PC_XY[0] + (PC_X - PC_WALL) / 2, PC_XY[1], TOP + PC_H / 2), size=(PC_WALL, PC_Y, PC_H),
          color=PC_COLOR),
    Shape("pc_wall_near", pos=(PC_XY[0] - (PC_X - PC_WALL) / 2, PC_XY[1], TOP + PC_H / 2), size=(PC_WALL, PC_Y, PC_H),
          color=PC_COLOR),
    Shape("pc_wall_left", pos=(PC_XY[0], PC_XY[1] + (PC_Y - PC_WALL) / 2, TOP + PC_H / 2), size=(PC_X, PC_WALL, PC_H),
          color=PC_COLOR),
    Shape("pc_wall_right", pos=(PC_XY[0], PC_XY[1] - (PC_Y - PC_WALL) / 2, TOP + PC_H / 2), size=(PC_X, PC_WALL, PC_H),
          color=PC_COLOR),
]

# -- the waste bin: OXO Good Grips Easy-Clean 1.75 gal compost bin, 10.15 in wide (x) x 8.3 in deep (y) x 12 in high
# (Walmart and Target listings), a straight-walled box here with 3 mm walls and floor (the real one tapers); fixed.
WB_XY = (0.400, -0.490)
WB_X, WB_Y, WB_H = 258 * MM, 211 * MM, 305 * MM
WB_WALL, WB_FLOOR = 3 * MM, 3 * MM
WB_IN_X, WB_IN_Y = WB_X - 2 * WB_WALL, WB_Y - 2 * WB_WALL
WB_RIM = TOP + WB_H
WB_FLOOR_TOP = TOP + WB_FLOOR
WB_COLOR = (0.17, 0.17, 0.18)
WASTE_BIN = [
    Shape("wb_floor", pos=(WB_XY[0], WB_XY[1], TOP + WB_FLOOR / 2), size=(WB_X, WB_Y, WB_FLOOR), color=WB_COLOR),
    Shape("wb_wall_far", pos=(WB_XY[0] + (WB_X - WB_WALL) / 2, WB_XY[1], TOP + WB_H / 2), size=(WB_WALL, WB_Y, WB_H),
          color=WB_COLOR),
    Shape("wb_wall_near", pos=(WB_XY[0] - (WB_X - WB_WALL) / 2, WB_XY[1], TOP + WB_H / 2), size=(WB_WALL, WB_Y, WB_H),
          color=WB_COLOR),
    Shape("wb_wall_left", pos=(WB_XY[0], WB_XY[1] + (WB_Y - WB_WALL) / 2, TOP + WB_H / 2), size=(WB_X, WB_WALL, WB_H),
          color=WB_COLOR),
    Shape("wb_wall_right", pos=(WB_XY[0], WB_XY[1] - (WB_Y - WB_WALL) / 2, TOP + WB_H / 2), size=(WB_X, WB_WALL, WB_H),
          color=WB_COLOR),
]

# -- the produce (marso SimReady assets; size = the package's bounds, mass = its estimate; origin at the bounds' centre)
# name: (file, x, y, z extents, mass kg, (static, dynamic) friction)
PIECES = {
    "tomato": ("rotten_tomato_a.usdc", (88.4 * MM, 86.0 * MM, 58.7 * MM), 0.220, (0.6, 0.5)),
    "pepper": ("red_pepper.usdc", (84.9 * MM, 101.6 * MM, 101.8 * MM), 0.170, (0.5, 0.4)),
    "zucchini": ("zucchini.usdc", (197.2 * MM, 53.6 * MM, 52.0 * MM), 0.297, (0.5, 0.4)),
}
GAP = float(os.environ.get("GAP", "10")) * MM  # between neighbours in a row, and to the walls (at most 15 mm fits the near row)
ROTTEN = ["tomato_1", "tomato_2"]
GOOD = ["pepper_1", "pepper_2", "zucchini"]


def _yaw(deg: float) -> tuple[float, float, float, float]:
    h = np.radians(deg) / 2
    return (0.0, 0.0, float(np.sin(h)), float(np.cos(h)))


def _footprint(kind: str, yaw: float) -> tuple[float, float]:
    x, y, _ = PIECES[kind][1]
    return (y, x) if int(round(yaw)) % 180 == 90 else (x, y)


# name: (kind, x, yaw degrees); y follows from the row's packing, from the container's right wall (-y) leftwards
ROWS = {
    0.295: [("tomato_2", "tomato", 90), ("pepper_1", "pepper", 0), ("tomato_1", "tomato", 90),
            ("pepper_2", "pepper", 0)],  # near row, from the right wall
    0.425: [("zucchini", "zucchini", 90)],  # far row
}


def _layout() -> dict[str, tuple[str, float, float, float]]:
    out = {}
    for x, row in ROWS.items():
        y = PC_XY[1] - PC_IN_Y / 2 + GAP
        for name, kind, yaw in row:
            w = _footprint(kind, yaw)[1]
            out[name] = (kind, x, y + w / 2, float(yaw))
            y += w + GAP
    return out


LAYOUT = _layout()


def _body(name: str) -> UsdBody:
    kind, x, y, yaw = LAYOUT[name]
    f, ext, mass, friction = PIECES[kind]
    return UsdBody(name, usd=f"assets/{f}", pos=(x, y, PC_FLOOR_TOP + ext[2] / 2 + 0.5 * MM), size=ext, mass=mass,
                   rot_xyzw=_yaw(yaw), friction=friction, solver_iters=32)


OBJECTS = [*PRODUCE_CONTAINER, *WASTE_BIN, *(_body(n) for n in LAYOUT)]

# -- scoring -------------------------------------------------------------------------------------------------------
FINGER_REACH = 88 * MM  # the fingertips' ends below the tip links' origins
RELEASE_DIST = 70 * MM  # a fingertip this close to a tomato's centre is still holding it (a held tomato: ~45 mm)
REST_SPEED = 0.05  # m/s: a tomato in the bin is resting below this
MOVE_MAX = 20 * MM  # a good piece may end at most this far (seen from above) from where it started
_STATE: dict = {"released": True, "t": 0.0, "last": -1.0}


def in_box(pos, centre, inner_x, inner_y, floor_top, rim) -> bool:
    """The body's origin (its centre) within the container's inner walls and between its floor and its rim."""
    p = np.asarray(pos, float)
    return bool(abs(p[0] - centre[0]) < inner_x / 2 and abs(p[1] - centre[1]) < inner_y / 2
                and floor_top < p[2] < rim)


def in_waste_bin(pos) -> bool:
    return in_box(pos, WB_XY, WB_IN_X, WB_IN_Y, WB_FLOOR_TOP, WB_RIM)


def in_produce_container(pos) -> bool:
    return in_box(pos, PC_XY, PC_IN_X, PC_IN_Y, PC_FLOOR_TOP, PC_RIM)


def undisturbed(objects) -> bool:
    """Every good piece ended within MOVE_MAX of its starting centre (seen from above)."""
    return all(np.linalg.norm(np.asarray(objects[n].pos)[:2] - np.array(LAYOUT[n][1:3])) <= MOVE_MAX for n in GOOD)


def score(objects, released: bool | None = None) -> float:
    """0 unless every good piece (both peppers and the zucchini) still lies in the produce container, each within 2 cm
    of where it started.
    Then 0.5 for each rotten tomato in the waste bin (one: 0.5; both: 0.75), and 1.0 for both when the arms have let
    go (no fingertip within 70 mm of a tomato) and both tomatoes are at rest (under 5 cm/s)."""
    if not all(in_produce_container(objects[n].pos) for n in GOOD) or not undisturbed(objects):
        return 0.0
    out = [n for n in ROTTEN if in_waste_bin(objects[n].pos)]
    if not out:
        return 0.0
    if len(out) == 1:
        return 0.5
    free = _STATE["released"] if released is None else released
    resting = all(np.linalg.norm(objects[n].lin_vel) < REST_SPEED for n in out)
    return 1.0 if free and resting else 0.75


def success(objects) -> bool:
    return score(objects) == 1.0


def fingertips(side: dict) -> np.ndarray:
    return (side["tip_left"] + side["tip_right"]) / 2 - np.array([0.0, 0.0, FINGER_REACH])


@on_reset
def _reset(environment) -> None:
    _STATE.update(released=True, t=0.0, last=-1.0)


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states

    links = robot_links(environment)
    states = object_states(environment, ROTTEN)
    tips = [fingertips(side) for side in links.values()]
    _STATE["released"] = not any(np.linalg.norm(tip - np.asarray(states[n].pos)) < RELEASE_DIST
                                 for tip in tips for n in ROTTEN)
    _STATE["t"] += environment.unwrapped.step_dt
    if os.environ.get("CULL_LOG") and _STATE["t"] - _STATE["last"] >= 0.5:
        _STATE["last"] = _STATE["t"]
        allst = object_states(environment, [*ROTTEN, *GOOD])
        print(f"LOG t={_STATE['t']:.1f} " + " ".join(
            f"{n}=({(allst[n].pos[0]):.3f},{(allst[n].pos[1]):.3f},{(allst[n].pos[2] - TOP) * 1000:.0f}mm"
            f"{',BIN' if in_waste_bin(allst[n].pos) else ''}{',PC' if in_produce_container(allst[n].pos) else ''})"
            for n in [*ROTTEN, *GOOD]) + f" released={_STATE['released']} score={score(allst)}"
            + f" right_tips_sep={np.linalg.norm(links['right']['tip_left'] - links['right']['tip_right']) * 1000:.0f}mm"
            f" right_tips_z={(fingertips(links['right'])[2] - TOP) * 1000:.0f}mm"
            f" fingers={np.round(arm_joints(environment, 'right', ['joint7', 'joint8']) * 1000, 1).tolist()}mm"
            f" right_tips_xy=({fingertips(links['right'])[0]:.3f},{fingertips(links['right'])[1]:.3f})", flush=True)


# -- the scripted demo: the right arm -----------------------------------------------------------------------------
HOVER = TOP + 0.150  # over the peppers' tops (0.102 m) with the fingertips clear; a straight-down grip reaches 0.20 m
LIFT = TOP + 0.160  # a held tomato's lowest point (TCP - 0.0293) then clears the peppers (0.102 m); the container's 0.160 m walls
# are cleared by the tilting carry that follows (the straight-down grip reaches 0.17 m at most here)
PARK = (0.20, -0.30, TOP + 0.15)
CARRY_Z = TOP + 0.380  # TCP height over the waste bin: the held tomato's lowest point clears the 305 mm rim
SQUEEZE = float(os.environ.get("SQUEEZE", "-0.036"))  # finger target while holding a tomato (gap 72 mm: it stops
# the fingers at its 86-88 mm width, and they press with the drive's force)


GRIP_YAW = float(os.environ.get("GRIP_YAW", "90"))  # degrees: 0 closes the fingers along y, 90 along x (across the row)


def demo():
    """For each tomato: down in front of and behind it with the fingers open (they close along x, across the row, so
    the neighbouring peppers stand clear of the fingers), close on it, lift straight out, tilt the hand 45 degrees
    forward (a joint move), carry it over the bin, open, and go back to the park pose."""
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.demo import Waypoint as W

    import tiltkit

    tiltkit.install()
    yaw = float(np.radians(GRIP_YAW))
    c = s = float(np.cos(np.radians(45)))
    closing = (s, 0, c) if int(round(GRIP_YAW)) % 180 == 90 else (0, -1, 0)  # the fingers' closing line, tilted with the hand
    closing = tuple(float(os.environ.get("CLOSE_SIGN", "1")) * v for v in closing)
    tilt = tiltkit.grip(approach=(c, 0, -s), closing=closing, joint=True)
    carry = tiltkit.Tilt(tilt.rot)
    right = [W(1.5, PARK, O, "start", 0.0)]
    for name in ("tomato_1", "tomato_2"):
        kind, x, y, _ = LAYOUT[name]
        z = PC_FLOOR_TOP + PIECES[kind][1][2] / 2  # the TCP at the tomato's middle
        right += [
            W(2.0, (x, y, HOVER), O, f"over {name}", yaw),
            W(1.5, (x, y, z), O, "down round it", yaw),
            W(0.8, (x, y, z), SQUEEZE, "close on it", yaw),
            W(1.5, (x, y, LIFT), SQUEEZE, "lift out", yaw),
            W(3.0, (WB_XY[0], WB_XY[1], CARRY_Z), SQUEEZE, "tilt and carry over the bin", tilt),
            W(0.8, (WB_XY[0], WB_XY[1], CARRY_Z), O, "let go", carry),
            W(2.5, PARK, O, "back", tiltkit.Tilt(None, joint=True)),
        ]
    t = sum(w.duration for w in right) + 3.0
    return {"left": [W(t, (0.20, 0.30, HOVER), O, "wait", 0.0)], "right": right + [W(3.0, PARK, O, "wait", 0.0)]}


# -- offline checks ------------------------------------------------------------------------------------------------
def _aabb(name: str):
    kind, x, y, yaw = LAYOUT[name]
    fx, fy = _footprint(kind, yaw)
    return x - fx / 2, x + fx / 2, y - fy / 2, y + fy / 2


def check_layout() -> dict:
    """Every piece inside the container's walls, no two overlapping, the gaps beside each tomato at least 9 mm."""
    gaps = {}
    names = list(LAYOUT)
    for n in names:
        x0, x1, y0, y1 = _aabb(n)
        assert PC_XY[0] - PC_IN_X / 2 <= x0 and x1 <= PC_XY[0] + PC_IN_X / 2, f"{n} outside in x"
        assert PC_XY[1] - PC_IN_Y / 2 <= y0 and y1 <= PC_XY[1] + PC_IN_Y / 2, f"{n} outside in y"
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            ax0, ax1, ay0, ay1 = _aabb(a)
            bx0, bx1, by0, by1 = _aabb(b)
            gx = max(bx0 - ax1, ax0 - bx1)
            gy = max(by0 - ay1, ay0 - by1)
            gap = max(gx, gy)
            assert gap > 0, f"{a} and {b} overlap"
            gaps[(a, b)] = gap
    for t in ROTTEN:
        x0, x1, y0, y1 = _aabb(t)
        # the fingers close along y: what matters is the free space either side of the tomato along y, in its x-span
        left = PC_XY[1] + PC_IN_Y / 2 - y1
        right = y0 - (PC_XY[1] - PC_IN_Y / 2)
        for o in LAYOUT:
            if o == t:
                continue
            ox0, ox1, oy0, oy1 = _aabb(o)
            if ox1 > x0 and ox0 < x1:  # shares the tomato's x-span
                if oy0 >= y1:
                    left = min(left, oy0 - y1)
                elif oy1 <= y0:
                    right = min(right, y0 - oy1)
        gaps[t] = (right, left)
        assert min(right, left) >= 9 * MM, (t, right, left)
    return gaps


def self_test(ObjectState=None):
    """The scoring rules on hand-built states."""
    from scenekit import BodyState

    def st(pos, speed=0.0):
        p = np.asarray(pos, float)
        return BodyState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.array([speed, 0, 0]),
                         ang_vel=np.zeros(3), low=p, high=p)

    in_bin = (WB_XY[0] + 0.03, WB_XY[1] - 0.02, WB_FLOOR_TOP + 0.03)
    start = {n: st((x, y, PC_FLOOR_TOP + PIECES[k][1][2] / 2)) for n, (k, x, y, _) in LAYOUT.items()}

    def with_(**moves):
        s = dict(start)
        s.update({n: (v if isinstance(v, BodyState) else st(v)) for n, v in moves.items()})
        return s

    on_table = (0.55, 0.45, TOP + 0.03)
    cases = {
        "start": (start, True, 0.0),
        "one tomato in the bin": (with_(tomato_1=in_bin), True, 0.5),
        "both, arms clear": (with_(tomato_1=in_bin, tomato_2=(WB_XY[0] - 0.04, WB_XY[1] + 0.03, WB_FLOOR_TOP + 0.03)),
                             True, 1.0),
        "both, a fingertip still at a tomato": (with_(tomato_1=in_bin, tomato_2=(WB_XY[0] - 0.04, WB_XY[1] + 0.03,
                                                                                WB_FLOOR_TOP + 0.03)), False, 0.75),
        "both, one still rolling": (with_(tomato_1=in_bin, tomato_2=st((WB_XY[0] - 0.04, WB_XY[1] + 0.03,
                                                                       WB_FLOOR_TOP + 0.03), speed=0.2)), True, 0.75),
        "tomato on the table": (with_(tomato_1=on_table), True, 0.0),
        "tomato held above the bin": (with_(tomato_1=(WB_XY[0], WB_XY[1], WB_RIM + 0.05)), True, 0.0),
        "tomato in the bin, a pepper dropped on the table": (with_(tomato_1=in_bin, pepper_1=on_table), True, 0.0),
        "both in the bin, a pepper in the bin too": (with_(tomato_1=in_bin, tomato_2=(WB_XY[0] - 0.04, WB_XY[1] + 0.03,
                                                      WB_FLOOR_TOP + 0.03), pepper_2=(WB_XY[0], WB_XY[1] - 0.05,
                                                      WB_FLOOR_TOP + 0.05)), True, 0.0),
        "zucchini thrown out, one tomato in the bin": (with_(tomato_2=in_bin, zucchini=on_table), True, 0.0),
        "a pepper moved 30 mm along the row": (with_(tomato_1=in_bin, pepper_1=(LAYOUT["pepper_1"][1],
                                               LAYOUT["pepper_1"][2] + 0.030, PC_FLOOR_TOP + 0.05)), True, 0.0),
        "a pepper nudged 10 mm": (with_(tomato_1=in_bin, pepper_1=(LAYOUT["pepper_1"][1],
                                  LAYOUT["pepper_1"][2] + 0.010, PC_FLOOR_TOP + 0.05)), True, 0.5),
    }
    for name, (objs, released, want) in cases.items():
        got = score(objs, released)
        assert abs(got - want) < 1e-9, (name, got, want)
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules, the packing, then each arm's plan through the kinematics at 20 Hz.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    for k, v in check_layout().items():
        print("gap", k, tuple(round(g * 1000, 1) for g in v) if isinstance(v, tuple) else round(v * 1000, 1), "mm")
    for name, (_, ext, mass, _) in PIECES.items():
        print(f"{name}: {np.round(np.array(ext) * 1000, 1)} mm, {mass * 1000:.0f} g")
    assert min(PIECES["tomato"][1][:2]) + 4 * MM < 94 * MM, "a tomato must fit the gripper's 94 mm opening"
    task = load(__file__)
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / 20)[0] for k in range(round(plan.duration * 20) + 5)])
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
