"""Serve roast potatoes with a serving spoon and fork.

Two roast potatoes lie in a white serving dish, with a serving spoon and a two-tine serving fork resting in the dish. A
dinner plate stands to the right of the dish. The robot serves both potatoes onto the plate the way people do at the table,
one tool in each hand:

- the spoon is held tipped forward, its tip on the dish's floor, and the fork pushes a potato up into its bowl;
- the spoon lifts the potato, tipped back so it rests against the bowl's back rim, and carries it over the dish's rim;
- over the plate the spoon tips forward and the potato slides off its open tip onto the plate.

A potato rides out of a spoon held level or tipped the wrong way while it is carried, and a fork that pushes too low goes
under it. A gripper must not touch the food.

Run it:  inspect-robots run --embodiment yam_arena -E task=serve_roast_potatoes_with_a_serving_spoon_and_fork.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402,F401
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, plate,  # noqa: E402
                     rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
env = os.environ.get

# -- where things are (world: x forward from the arm bases' line, y left, z up) --------------------------------------------
DISH = np.array([0.338, 0.070, TOP])  # the serving dish's middle, on the table
DISH_X, DISH_Y, DISH_H, WALL, BASE = 0.180, 0.260, 0.025, 0.008, 0.006  # outside sizes, wall and base thickness
FLOOR = TOP + BASE  # the dish's inside floor
PLATE = np.array([0.338, -0.185, TOP])  # the dinner plate's middle, on the table
PLATE_R, PLATE_T, WELL_R = 0.120, 0.008, 0.095  # its radius, the thickness of its flat middle, the flat middle's radius
POTATOES = [np.array([0.290, 0.075, FLOOR]), np.array([0.350, 0.075, FLOOR])]  # each potato's middle, on the floor;
# each lies crosswise, its 40 mm length along x
SPOON0 = np.array([0.338, 0.0104, FLOOR + 0.0003])  # the middle of the underside of the spoon's bowl, at rest on the floor
FORK0 = np.array([0.338, 0.0946, FLOOR + 0.0203])  # the tips of the fork's tines (underside), at rest: raised, its handle
# resting on the dish's rim and its end on the table
# how far each tool is tipped back at rest (degrees about x; its handle rests on the rim or the table), as measured
SPOON_PITCH0 = float(env("SRV_SP0", "-0.5"))
FORK_PITCH0 = float(env("SRV_FP0", "-14.3"))
# -- a roast potato: a chunk 40 mm long, its 30 x 26 mm cross-section rounded at the corners (r 5 mm) -------------------------
PL, PW, PH, PR = 0.040, 0.030, 0.026, 0.005
HY = PW / 2  # half the potato's extent along y, as it lies
POTATO_MASS = 0.030
# -- the rules ----------------------------------------------------------------------------------------------------------
ON_PLATE_R = 0.110  # a potato is on the plate when its middle is within this of the plate's middle, resting on it
HANDS_OFF = 0.030  # a gripper "touches" the food when its fingertips' midpoint comes this close to a potato's middle
SUCCESS_HOLD_S = 1.0

WHITE, STEEL, GOLD, DARK = (0.93, 0.93, 0.91), (0.72, 0.73, 0.75), (0.86, 0.62, 0.26), (0.20, 0.20, 0.22)


# -- the bodies -------------------------------------------------------------------------------------------------------------

def _dish() -> Body:
    hx, hy = DISH_X / 2, DISH_Y / 2
    sh = [BoxShape((0.0, 0.0, BASE / 2), (DISH_X, DISH_Y, BASE), WHITE),
          BoxShape((hx - WALL / 2, 0.0, DISH_H / 2), (WALL, DISH_Y, DISH_H), WHITE),
          BoxShape((-hx + WALL / 2, 0.0, DISH_H / 2), (WALL, DISH_Y, DISH_H), WHITE),
          BoxShape((0.0, hy - WALL / 2, DISH_H / 2), (DISH_X - 2 * WALL, WALL, DISH_H), WHITE),
          BoxShape((0.0, -hy + WALL / 2, DISH_H / 2), (DISH_X - 2 * WALL, WALL, DISH_H), WHITE)]
    return Body.build("dish", pos=tuple(DISH.tolist()), mass=1.50, shapes=sh, color=WHITE, friction=(0.7, 0.6),
                      contact_offset=0.001)


def _plate() -> Body:
    sh = [CylShape((0.0, 0.0, PLATE_T / 2), PLATE_R - 0.006, PLATE_T, WHITE)]
    n = 28
    for k in range(n):  # the raised rim round the flat middle
        a = 2 * math.pi * k / n
        r = (WELL_R + PLATE_R) / 2
        sh.append(BoxShape((r * math.cos(a), r * math.sin(a), PLATE_T + 0.003), (PLATE_R - WELL_R, 2 * r * math.pi / n + 0.002,
                                                                                    0.006), WHITE, yaw=a))
    return Body.build("plate", pos=tuple(PLATE.tolist()), mass=0.55, shapes=sh, color=WHITE, friction=(0.5, 0.4),
                      contact_offset=0.001)


def _potato(name: str, pos) -> Body:
    sh = [BoxShape((0.0, 0.0, PH / 2), (PL, PW - 2 * PR, PH), GOLD),
          BoxShape((0.0, 0.0, PH / 2), (PL, PW, PH - 2 * PR), GOLD)]
    for y in (-(PW / 2 - PR), PW / 2 - PR):
        for z in (PR, PH - PR):
            sh.append(CylShape((0.0, y, z), PR, PL, GOLD, axis="X"))
    return Body.build(name, pos=tuple(np.asarray(pos).tolist()), mass=POTATO_MASS, shapes=sh, color=GOLD,
                      friction=(0.45, 0.35), contact_offset=0.001, solver_iterations=24)


# the spoon, in its own frame: the bowl's middle at the origin on the underside of its floor, its open tip toward +y, the
# handle rising toward -y. A 26 cm stainless serving spoon, about 110 g.
BOWL_W, BOWL_L, BOWL_T, RIM_H = 0.052, 0.070, 0.0010, 0.009
SPOON_GRIP = np.array([0.0, float(env("SRV_SG", "-0.115")), 0.0])  # where the hand takes the handle (y; z is filled in from the handle's line)
NECK_A, NECK_B, HANDLE_END = np.array([0.0, -0.036, 0.006]), np.array([0.0, -0.080, 0.030]), np.array([0.0, -0.225, 0.040])


def _qx(deg: float) -> tuple:
    """The quaternion (x, y, z, w) of a turn of ``deg`` about x."""
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def _rx(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])


def _handle_z(a, b, y) -> float:
    t = (y - a[1]) / (b[1] - a[1])
    return float(a[2] + t * (b[2] - a[2]))


SPOON_GRIP[2] = _handle_z(NECK_B, HANDLE_END, SPOON_GRIP[1])


def _spoon() -> Body:
    sh = [BoxShape((0.0, 0.0, BOWL_T / 2), (BOWL_W, BOWL_L, BOWL_T), STEEL),
          BoxShape((BOWL_W / 2 - 0.0015, -0.006, RIM_H / 2), (0.003, BOWL_L - 0.012, RIM_H), STEEL),
          BoxShape((-BOWL_W / 2 + 0.0015, -0.006, RIM_H / 2), (0.003, BOWL_L - 0.012, RIM_H), STEEL),
          BoxShape((0.0, -BOWL_L / 2 + 0.0015, (RIM_H + 0.003) / 2), (BOWL_W, 0.003, RIM_H + 0.003), STEEL),
          plate(NECK_A, NECK_B, 0.012, 0.004, (0.0, 0.0, 1.0), STEEL),
          plate(NECK_B, HANDLE_END, 0.015, 0.003, (0.0, 0.0, 1.0), STEEL)]
    return Body.build("spoon", pos=tuple(SPOON0.tolist()), mass=0.110, shapes=sh, color=STEEL, friction=(0.9, 0.8),
                      contact_offset=0.001, solver_iterations=24, start_xyzw=_qx(SPOON_PITCH0),
                      damping=(0.05, 0.4), com=(0.0, -0.046, 0.010),  # it balances at the root of its handle
                      slick=True)  # polished steel: a potato slides on it


# the fork, in its own frame: the tips of its two tines at the origin, on their underside; the tines run toward +y to
# their root, and the handle rises beyond. A 27 cm stainless two-tine serving fork, about 90 g.
TINE_L, TINE_GAP, TINE_W, TINE_T = 0.070, 0.018, 0.004, 0.003
FORK_GRIP = np.array([0.0, float(env("SRV_FG", "0.150")), 0.0])
FNECK_A, FNECK_B, FHANDLE_END = np.array([0.0, 0.074, 0.002]), np.array([0.0, 0.100, 0.032]), np.array([0.0, 0.265, 0.042])
FORK_GRIP[2] = _handle_z(FNECK_B, FHANDLE_END, FORK_GRIP[1])


def _fork() -> Body:
    sh = [BoxShape((x, TINE_L / 2, TINE_T / 2), (TINE_W, TINE_L, TINE_T), STEEL) for x in (-TINE_GAP / 2, TINE_GAP / 2)]
    sh += [BoxShape((0.0, TINE_L + 0.002, TINE_T / 2), (TINE_GAP + TINE_W, 0.006, TINE_T), STEEL),
           plate(FNECK_A, FNECK_B, 0.010, 0.004, (0.0, 0.0, 1.0), STEEL),
           plate(FNECK_B, FHANDLE_END, 0.015, 0.003, (0.0, 0.0, 1.0), STEEL)]
    return Body.build("fork", pos=tuple(FORK0.tolist()), mass=0.090, shapes=sh, color=STEEL, friction=(0.9, 0.8),
                      contact_offset=0.001, solver_iterations=24, start_xyzw=_qx(FORK_PITCH0),
                      damping=(0.05, 0.4))


OBJECTS = [_dish(), _plate(), _potato("potato_a", POTATOES[0]), _potato("potato_b", POTATOES[1]), _spoon(), _fork()]
POTATO_NAMES = ("potato_a", "potato_b")


# -- followed after every control step ------------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"s": {}, "steps": 0, "last": None, "touched": False, "dropped": False}


_EP = _fresh()


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())


def _tcps(base, origin):
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _middle(p, r) -> np.ndarray:
    """A potato's middle (its body origin is on its underside)."""
    return p + r @ np.array([0.0, 0.0, PH / 2])


def measure(potatoes, plate_pos, dish_pos, tcps) -> dict:
    """``potatoes``: (position, rotation) of each; ``plate_pos``, ``dish_pos``: the plate's and the dish's origins."""
    served, on_table = 0, 0
    for p, r in potatoes:
        m = _middle(p, r)
        d = float(np.hypot(*(m - plate_pos)[:2]))
        if d <= ON_PLATE_R and m[2] - plate_pos[2] <= PLATE_T + PH:  # resting on the plate, not held above it
            served += 1
        elif m[2] - TOP <= PH * 0.75 and not (abs(m[0] - dish_pos[0]) < DISH_X / 2 and abs(m[1] - dish_pos[1]) < DISH_Y / 2) \
                and d > PLATE_R:
            on_table += 1  # on the bare table: dropped
    touched = any(float(np.linalg.norm(t - _middle(p, r))) < HANDS_OFF for t in tcps for p, r in potatoes)
    return {"served": served, "on_table": on_table, "touched": bool(touched)}


@on_step
def _follow(env_) -> None:
    base = env_.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pots = []
    for n in POTATO_NAMES:
        p = as_numpy(base.scene[n].data.root_link_pose_w)[0]
        pots.append((p[:3] - origin, rotation(p[3:7])))
    pl = as_numpy(base.scene["plate"].data.root_link_pose_w)[0][:3] - origin
    dp = as_numpy(base.scene["dish"].data.root_link_pose_w)[0][:3] - origin
    tcps = _tcps(base, origin)
    s = measure(pots, pl, dp, tcps)
    tools = {}
    for n in ("spoon", "fork"):
        q = as_numpy(base.scene[n].data.root_link_pose_w)[0]
        rr = rotation(q[3:7])
        tools[n] = (q[:3] - origin, math.degrees(math.atan2(rr[2, 1], rr[1, 1])))
    _EP["touched"] = _EP["touched"] or s["touched"]
    _EP["dropped"] = _EP["dropped"] or s["on_table"] > 0
    now = np.concatenate([p for p, _ in pots])
    last = _EP.get("last")
    s["still"] = last is not None and float(np.abs(now - last).max()) < 0.0003
    _EP["last"] = now
    _EP["s"] = s
    _EP["steps"] += 1
    if env("SRV_DEBUG") and _EP["steps"] % int(env("SRV_DEBUG_N", "20")) == 0:
        rel = [np.round((_middle(p, r) - PLATE) * 1000, 1) for p, r in pots]
        print(f"[srv] step {_EP['steps']} served {s['served']} table {s['on_table']} touched {_EP['touched']} "
              f"dropped {_EP['dropped']} potatoes-plate {rel[0]} {rel[1]} R {np.round(tcps[1] * 1000, 1)} "
              f"L {np.round(tcps[0] * 1000, 1)}", flush=True)
        print(f"[srv]   spoon {np.round(tools['spoon'][0] * 1000, 1)} pitch {tools['spoon'][1]:.1f}  "
              f"fork {np.round(tools['fork'][0] * 1000, 1)} pitch {tools['fork'][1]:.1f}", flush=True)


# -- scoring ------------------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.5 for each potato resting on the plate (its middle within 11 cm of the plate's middle), so 1.0 for both. 0.0 if a
    gripper touched a potato (its fingertips' midpoint within 3 cm of the potato's middle) or a potato fell on the bare
    table at any time."""
    s = _EP["s"]
    if not s or _EP["touched"] or _EP["dropped"]:
        return 0.0
    return 0.5 * min(2, s["served"])


def success(objects) -> bool:
    return score(objects) >= 1.0 and _EP["s"].get("still", False)


# -- scripted demo --------------------------------------------------------------------------------------------------------

LEAD_S, LEAD_STEPS = 3.0, 15


def _frame(approach, close) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - np.dot(close, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _good_ik(arm: str, pos, rot, near=None) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    sols = []
    for seed in itertools.product((-0.5, 0.0, 0.5), (1.3, 1.9), (1.0, 1.7), (-1.0, 0.0), (0.0,), (-1.0, 0.0, 1.0)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            sols.append((q, float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())))
    if not sols:
        raise ValueError(f"no IK solution for the {arm} hand at {np.round(pos, 4)}")
    if near is not None:
        roomy = [s for s in sols if s[1] > 0.15] or sols
        return min(roomy, key=lambda s: float(np.linalg.norm(s[0] - near)))[0]
    return max(sols, key=lambda s: s[1])[0]


def _joint_path(arm: str, q0, q1, label: str, finger, secs=LEAD_S, steps=LEAD_STEPS) -> list:
    from yam_arena import ik, station

    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(secs / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def demo():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur = {}
    for arm in ("left", "right"):
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
        cur[arm] = (np.asarray(p, float), FINGER_OPEN, r)

    def go(arm, t, pos, finger, label, rot=None, n=1):
        p0, _, r0 = cur[arm]
        rot = r0 if rot is None else rot
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label, rot))
        cur[arm] = (p1, finger, rot)

    def wait(arm, t, label="wait"):
        p, f, r = cur[arm]
        if t > 0:
            plan[arm].append(Waypoint(t, tuple(float(v) for v in p), f, label, r))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    def sync():  # both arms wait for each other
        end = max(dur("left"), dur("right"))
        for arm in ("left", "right"):
            wait(arm, end - dur(arm), "wait for the other hand")

    def lead(arm, pos, rot, finger, label, secs=LEAD_S):
        p0, _, r0 = cur[arm]
        q0 = np.array(READY, float) if not plan[arm] else _good_ik(arm, p0, r0, near=np.array(READY, float))
        plan[arm] += _joint_path(arm, q0, _good_ik(arm, pos, rot, near=q0), label, finger, secs)
        cur[arm] = (np.asarray(pos, float), finger, rot)

    def home(arm, label="back to where it started"):
        p, f, r = cur[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p, r, near=np.array(READY, float)), np.array(READY, float), label, f)
        cur[arm] = (np.asarray(plan[arm][-1].pos, float), f, plan[arm][-1].yaw)

    up = np.array([0.0, 0.0, 1.0])
    open_ = float(env("SRV_OPEN", "-0.030"))
    hold = FINGER_CLOSED
    gdz = float(env("SRV_GDZ", "0.006"))  # the TCP this far above the handle's middle line when taking it
    # straight down, the fingers closing across each handle (along x); the left hand's close is mirrored
    r_r = _frame((0.0, 0.0, -1.0), (1.0, 0.0, 0.0))
    r_l = _frame((0.0, 0.0, -1.0), (float(env("SRV_LCLOSE", "-1.0")), 0.0, 0.0))
    tool = {"right": (SPOON_GRIP, SPOON_PITCH0, r_r, np.array([0.0, 0.0, float(env("SRV_SZ", "0.0"))])),
            "left": (FORK_GRIP, FORK_PITCH0, r_l, np.array([0.0, 0.0, float(env("SRV_FZ", "0.0"))]))}

    aim_pitch = {"right": float(env("SRV_SPA", "15.0")), "left": float(env("SRV_FPA", "0.0"))}

    def hand(arm, origin, pitch=0.0, comp=None):
        """The TCP and rotation that put the tool in ``arm``'s hand with its origin at ``origin``, turned ``pitch``
        degrees about x (0: bowl or tines level), the tool held as it was taken at rest. ``comp`` adds the aim pitch that
        makes up for the tool sagging in the hand once it is carried."""
        grip, p0, r0, aim = tool[arm]
        if comp is None:
            comp = False
        if comp:
            pitch = pitch + aim_pitch[arm]
        # the hand takes the handle square to it (tilted with it at rest), the TCP ``gdz`` above its middle line
        g = grip + np.array([0.0, 0.0, gdz])
        return np.asarray(origin, float) + _rx(pitch) @ g + aim, _rx(pitch) @ r0

    last = {}  # each tool's last commanded origin and total pitch, to turn it gradually

    def tgo(arm, t, origin, label, pitch=0.0, finger=None, n=1, dz=0.0, comp=None):
        if comp is None:
            comp = False
        total = pitch + (aim_pitch[arm] if comp else 0.0)
        o1 = np.asarray(origin, float) + up * dz
        o0, p0 = last.get(arm, (o1, total))
        for j in range(1, n + 1):
            f = j / n
            p, r = hand(arm, o0 + (o1 - o0) * f, p0 + (total - p0) * f, comp=False)
            go(arm, t / n, p, hold if finger is None else finger, label, rot=r)
        last[arm] = (o1, total)

    # take the tools where they rest
    for arm, o0 in (("right", SPOON0), ("left", FORK0)):
        p, r = hand(arm, o0, tool[arm][1], comp=False)
        lead(arm, p + up * 0.05, r, open_, "above the " + ("spoon's" if arm == "right" else "fork's") + " handle")
    for arm, o0 in (("right", SPOON0), ("left", FORK0)):
        tgo(arm, 1.0, o0, "fingers either side of the handle", pitch=tool[arm][1], finger=open_, n=2, comp=False)
        tgo(arm, 0.6, o0, "take it", pitch=tool[arm][1], comp=False)
    sync()
    # lift both a little and level them
    tgo("right", 0.8, SPOON0, "spoon up, its bowl level", dz=0.02)
    tgo("left", 0.8, FORK0, "fork up, its tines level", dz=0.02)
    sync()

    push = float(env("SRV_PUSH", "0.055"))  # how far the fork pushes a potato toward the spoon
    lift = float(env("SRV_LIFT", "0.060"))  # how high the spoon is carried over the dish's rim
    drop_y = float(env("SRV_DROP_Y", "-0.200"))  # where the bowl's middle is set down on the plate (y)
    out = float(env("SRV_OUT", "0.085"))  # how far the spoon is drawn out from under the potato
    meet = float(env("SRV_MEET", "0.010"))  # how far the spoon slides toward the potato as the fork pushes it
    press = float(env("SRV_PRESS", "0.0"))  # how far below the floor (or the potato's top) the tools are aimed
    for k, pot in enumerate(POTATOES):
        x = pot[0]
        # the spoon's bowl flat on the floor, its tip just short of the potato; the fork's tines behind the potato
        bowl = np.array([x, pot[1] - HY - BOWL_L / 2 - 0.006, FLOOR])
        tips = np.array([x, pot[1] + HY + 0.006, FLOOR])
        scoop = -float(env("SRV_SCOOP", "8"))  # the spoon tipped forward to scoop: its tip on the floor, the bowl a ramp
        lift_tip = (BOWL_L / 2) * math.sin(math.radians(-scoop))  # the bowl's middle this far up when its tip is down
        tgo("right", 1.2, bowl, "the spoon's bowl in front of the potato", dz=0.02 + lift_tip, n=2, pitch=scoop)
        tgo("left", 1.2, tips, "the fork's tines behind the potato", dz=0.02, n=2)
        tgo("right", 0.6, bowl, "the spoon's tip down on the dish's floor", dz=lift_tip - press, pitch=scoop)
        fh = float(env("SRV_FH", "0.012"))  # the tines this far above the floor as they push: on the potato's flat side
        tgo("left", 0.6, tips, "tines down just over the floor", dz=fh)
        sync()
        # push the potato onto the bowl, the spoon meeting it a little
        tgo("left", 1.6, tips + np.array([0, -push, 0]), "push the potato onto the spoon with the fork", dz=fh, n=4)
        tgo("right", 1.6, bowl + np.array([0, meet, 0]), "slide the bowl under it", dz=lift_tip - press, n=4, pitch=scoop)
        wait("left", float(env("SRV_PUSH_HOLD", "0.6")), "keep pushing until it is on the spoon")
        sync()
        bowl = bowl + np.array([0, meet, 0])
        tips_end = tips + np.array([0, -push, 0])
        # the fork up out of the way; the spoon lifts the potato, tipped back a little so it rests against the back rim
        tgo("left", 0.8, tips_end, "fork up out of the way", dz=float(env("SRV_FUP", "0.045")), n=2)
        b1 = bowl + up * lift
        tgo("right", 1.6, b1, "lift the potato on the spoon, tipping it back so the potato rests on the back rim", n=6,
            comp=True)
        sync()
        dy = drop_y - bowl[1]
        dx = float(env("SRV_DX_A", "0.0")) if k == 0 else 0.0  # the first potato a little toward the plate's middle
        b2 = b1 + np.array([dx, dy, 0])
        tgo("right", float(env("SRV_CARRY_S", "3.0")), b2, "carry it over the rim to the plate", n=8, comp=True)
        b3 = np.array([b2[0], b2[1], PLATE[2] + PLATE_T + float(env("SRV_TIPH", "0.020"))])
        tgo("right", 0.8, b3, "lower it over the plate", n=2, comp=True)
        tip = float(env("SRV_TIP", "-35"))  # tip the spoon forward: the potato slides off its open tip onto the plate
        tgo("right", 1.4, b3, "tip the spoon so the potato slides off onto the plate", pitch=tip, n=4)
        wait("right", 0.6, "let it slide off")
        tgo("right", 1.0, b3 + up * 0.06, "spoon up and level again", n=2, comp=True)
        sync()
        if k + 1 < len(POTATOES):  # back over the dish for the next potato
            nxt = POTATOES[k + 1]
            tgo("right", 1.6, np.array([nxt[0], nxt[1] - HY - BOWL_L / 2 - 0.006, FLOOR]), "spoon back over the dish",
                dz=0.05, n=3)
            tgo("left", 1.6, np.array([nxt[0], nxt[1] + HY + 0.006, FLOOR]), "fork back over the dish", dz=0.05, n=3)
            sync()
    # put the tools back in the dish and let go
    tgo("right", 1.6, SPOON0, "spoon back to the dish", dz=0.03, n=3)
    tgo("left", 1.6, FORK0, "fork back to the dish", dz=0.03, n=3)
    tgo("right", 0.8, SPOON0, "lay the spoon in the dish", dz=0.004)
    tgo("left", 0.8, FORK0, "lay the fork in the dish", dz=0.004)
    go("right", 0.5, cur["right"][0], open_, "let go of the spoon")
    go("left", 0.5, cur["left"][0], open_, "let go of the fork")
    go("right", 0.6, cur["right"][0] + up * 0.05, open_, "hand up")
    go("left", 0.6, cur["left"][0] + up * 0.05, open_, "hand up")
    home("right")
    home("left")
    end = max(dur("left"), dur("right")) + 2.0
    for arm in ("left", "right"):
        wait(arm, end - dur(arm), "hold still")
    return plan
