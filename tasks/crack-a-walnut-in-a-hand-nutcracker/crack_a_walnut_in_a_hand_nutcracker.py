"""On the station table lies a steel hand nutcracker, its two long handles open in a V toward the arms, on a smooth
worktop, with a walnut between its jaws. A small white bowl waits to the right and a saucer to the left.

Task: crack the walnut, put the kernel in the bowl and every piece of shell on the saucer, and bring both arms back.
The walnut cracks once both jaws press it with CRACK_N (PhysX contact forces). Pushed from one side only, the light
nutcracker just slides across the smooth worktop with the walnut, and nothing cracks.

Run it:  inspect-robots run --embodiment yam_arena -E task=crack_a_walnut_in_a_hand_nutcracker.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, contact_forces,  # noqa: E402
                     hinge, on_reset, on_step, rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the nutcracker: two steel levers on a pivot, open in a V toward the arms -------------------------------------------
PX, PY = 0.44, 0.0  # the pivot
OPEN_DEG = 30.0  # each lever's angle from the V's middle line when open (the spring holds them there)
SHUT_LIMIT = 20.0  # the levers can close no further than this (each lever's angle)
BEND = 0.20  # each lever runs straight out from the pivot to here, then bends into its handle ...
PARALLEL_DEG = 25.0  # ... which runs parallel to the V's middle line when the lever is this far from it
GRIP_LEN, GRIP_W, GRIP_H = 0.090, 0.022, 0.022  # the red handle grip
BAR_W, BAR_H = 0.014, 0.012  # the steel bar
JAW = (0.040, 0.098, 0.018, 0.034)  # the jaw: from, to (along the lever), width, height
LEVER_MASS = 0.16
SPRING = 0.04  # N m per degree, opening the levers ...
PRELOAD = 8.0  # ... toward 8 degrees past fully open, so they rest open against the stop
JAW_IN = JAW[2] - BAR_W / 2  # a jaw's inner face, this far in from its lever's middle line

# -- the walnut: 36 x 34 x 33 mm, 12 g, on the worktop between the jaws, its middle 7 cm out from the pivot -------------
NUT_S = 0.070
NUT = np.array([PX - NUT_S, PY, TOP])  # its middle, on the worktop
W_A, W_B, W_C = 0.018, 0.017, 0.0165  # half its length (along the V), width and height
NUT_MASS = 0.012
CRACK_N = 40.0  # N: the shell gives once each jaw presses it this hard ...
CRACK_STEPS = 2  # ... for this many control steps running (a steady squeeze, not a knock)
# once it gives: a 6 g kernel, half the shell (3 g) and two quarters of it (1.5 g each), laid out between the levers
# along the V's middle line, this far out from the pivot; each piece (length along the V, width, height)
KERNEL = (0.028, 0.025, 0.024)
SHELL = {"shell_a": (0.034, 0.032, 0.017), "shell_b": (0.030, 0.017, 0.016), "shell_c": (0.026, 0.017, 0.015)}
MASS = {"kernel": 0.0060, "shell_a": 0.0030, "shell_b": 0.0015, "shell_c": 0.0015}
LAYOUT = {"kernel": 0.100, "shell_a": 0.145, "shell_b": 0.188, "shell_c": 0.226}
SHELLS = tuple(SHELL)
PIECES = ("kernel",) + SHELLS
PARK = (0.30, 0.0, TOP + 1.5)  # pieces wait out of sight in a closed box up here

BOWL_AT, BOWL_R, BOWL_H = np.array([0.27, -0.25]), 0.050, 0.040
DISH_AT, DISH_R = np.array([0.27, 0.25]), 0.062

STEEL, RED, BROWN, DARK, KERNEL_C, KERNEL_D, WHITE, BLUE, HOLE, INNER = (0.72, 0.73, 0.75), (0.80, 0.12, 0.10), \
    (0.55, 0.40, 0.24), (0.44, 0.31, 0.18), (0.86, 0.72, 0.50), (0.74, 0.58, 0.36), (0.95, 0.95, 0.93), \
    (0.30, 0.50, 0.85), (0.10, 0.10, 0.10), (0.78, 0.64, 0.44)

# -- scoring ----------------------------------------------------------------------------------------------------------
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0
ON_DISH_Z = 0.04  # a piece of shell on the saucer: its middle at most this far above the saucer's base


def _u(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([math.cos(a), math.sin(a), 0.0])


def _lever(name: str, side: float) -> Body:
    """A lever in its own frame: origin on the pivot, unrotated; its shapes laid out along the open angle."""
    phi = 180.0 - side * OPEN_DEG  # the left lever (+1) points back and left, the right (-1) back and right
    u, yaw = _u(phi), math.radians(phi)
    shapes = [BoxShape(tuple((u * (BEND + 0.006) / 2 + [0, 0, BAR_H / 2]).tolist()), (BEND + 0.006, BAR_W, BAR_H),
                       STEEL, yaw=yaw)]
    g_phi = 180.0 - side * (OPEN_DEG - PARALLEL_DEG)  # the handle's direction
    g = _u(g_phi)
    mid = u * BEND + g * GRIP_LEN / 2
    shapes.append(BoxShape(tuple((mid + [0, 0, GRIP_H / 2]).tolist()), (GRIP_LEN, GRIP_W, GRIP_H), RED,
                           yaw=math.radians(g_phi)))
    shapes.append(CylShape(tuple((u * BEND + [0, 0, BAR_H / 2]).tolist()), BAR_W / 2, BAR_H, STEEL))  # the bend
    a, b, w, h = JAW
    n = np.array([-u[1], u[0], 0.0]) * side  # toward the V's middle
    shapes.append(BoxShape(tuple((u * (a + b) / 2 + n * (w - BAR_W) / 2 + [0, 0, h / 2]).tolist()), (b - a, w, h),
                           STEEL, yaw=yaw))
    for k in range(5):  # the jaw's teeth (seen only)
        r = a + 0.006 + k * (b - a - 0.012) / 4
        shapes.append(BoxShape(tuple((u * r + n * (w - BAR_W / 2 + 0.0005) + [0, 0, h / 2]).tolist()),
                               (0.003, 0.002, h * 0.8), (0.55, 0.56, 0.58), yaw=yaw, collide=False))
    if side > 0:  # the pivot pin (seen only)
        shapes.append(CylShape((0.0, 0.0, 0.016), 0.009, 0.032, (0.60, 0.61, 0.63), collide=False))
    return Body.build(name, pos=(PX, PY, TOP), mass=LEVER_MASS, shapes=shapes, friction=(0.30, 0.25), color=STEEL,
                      damping=(0.0, 0.05))


def _walnut() -> Body:
    """One rigid body: a 34 mm upright cylinder (what the jaws touch; unseen) inside a lumpy shell (seen only)."""
    shapes = [CylShape((0.0, 0.0, W_C), W_B, 2 * W_C, None)]
    r = 0.0062
    rows = [(-28.0, 12), (0.0, 14), (28.0, 12), (56.0, 8), (80.0, 3)]
    for j, (lat, n) in enumerate(rows):
        for i in range(n):
            lon = 2 * math.pi * (i + 0.5 * (j % 2)) / n
            c = math.cos(math.radians(lat))
            p = ((W_A - r) * c * math.cos(lon), (W_B - r) * c * math.sin(lon),
                 W_C + (W_C - r) * math.sin(math.radians(lat)))
            shapes.append(SphereShape(p, r, BROWN if (i + j) % 2 else DARK, collide=False))
    shapes.append(BoxShape((0.0, 0.0, W_C), (2 * W_A - 0.004, 0.0016, 2 * W_C - 0.003), DARK, collide=False))  # seam
    return Body.build("walnut", pos=tuple(NUT.tolist()), mass=NUT_MASS, shapes=shapes, friction=(1.2, 1.1),
                      color=BROWN, damping=(0.2, 2.0), report_contacts=True)


def _store(k: int) -> np.ndarray:
    return np.array([PARK[0] - 0.03 + 0.02 * k, PARK[1] + (0.02 if k % 2 else -0.02), PARK[2] + 0.005])


def _kernel() -> Body:
    lx, ly, lz = KERNEL
    shapes = [BoxShape((0.0, 0.0, lz / 2), (lx, ly, lz), None)]
    shapes.append(BoxShape((0.0, 0.0, lz / 2), (lx - 0.005, ly - 0.005, lz - 0.004), KERNEL_C, collide=False))
    for sx in (-1.0, 1.0):  # its four lobes, top and bottom
        for sy in (-1.0, 1.0):
            for z, rr in ((0.0058, 0.0055), (0.0155, 0.0062)):
                shapes.append(SphereShape((sx * (lx / 2 - rr), sy * (ly / 2 - rr * 0.8), z), rr,
                                          KERNEL_C if z > 0.01 else KERNEL_D, collide=False))
    shapes.append(BoxShape((0.0, 0.0, lz / 2), (lx - 0.002, 0.0012, lz - 0.002), KERNEL_D, collide=False))
    return Body.build("kernel", pos=tuple(_store(0).tolist()), mass=MASS["kernel"], shapes=shapes,
                      friction=(1.0, 0.9), color=KERNEL_C, damping=(0.2, 1.0), solver_iterations=32)


def _shell(name: str, k: int) -> Body:
    """A piece of shell lying dome up: a box (what is touched; unseen) under a lumpy brown dome (seen only)."""
    lx, ly, lz = SHELL[name]
    shapes = [BoxShape((0.0, 0.0, lz / 2), (lx, ly, lz), None)]
    shapes.append(BoxShape((0.0, 0.0, 0.0015), (lx - 0.002, ly - 0.002, 0.003), INNER, collide=False))  # broken rim
    r = lz * 0.55
    for i, (fx, fy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1), (0, 0))):
        rr = r * (1.15 if (fx, fy) == (0, 0) else 1.0)
        shapes.append(SphereShape((fx * (lx / 2 - rr), fy * (ly / 2 - rr), lz - rr), rr, BROWN if i % 2 else DARK,
                                  collide=False))
    shapes.append(BoxShape((0.0, 0.0, lz * 0.4), (lx - 0.004, ly - 0.004, lz * 0.8), BROWN, collide=False))
    return Body.build(name, pos=tuple(_store(k).tolist()), mass=MASS[name], shapes=shapes, friction=(1.2, 1.1),
                      color=BROWN, damping=(0.2, 1.0), solver_iterations=32)


def _store_box() -> Body:
    w, t = 0.12, 0.004
    shapes = [BoxShape((0.0, 0.0, t / 2), (w, w, t), HOLE), BoxShape((0.0, 0.0, w - t / 2), (w, w, t), HOLE)]
    shapes += [BoxShape((s * (w / 2 - t / 2), 0.0, w / 2), (t, w, w), HOLE) for s in (-1.0, 1.0)]
    shapes += [BoxShape((0.0, s * (w / 2 - t / 2), w / 2), (w, t, w), HOLE) for s in (-1.0, 1.0)]
    return Body.build("piece_store", pos=PARK, mass=1.0, shapes=shapes, kinematic=True, color=HOLE)


def _bowl() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.003), BOWL_R * 0.6, 0.006, WHITE)]
    n = 20
    for i in range(n):
        a = 2 * math.pi * i / n
        shapes.append(BoxShape(((BOWL_R - 0.003) * math.cos(a), (BOWL_R - 0.003) * math.sin(a), BOWL_H / 2),
                               (0.006, 2 * math.pi * BOWL_R / n * 1.12, BOWL_H), WHITE, yaw=a))
    shapes.append(CylShape((0.0, 0.0, 0.0065), BOWL_R - 0.004, 0.001, (0.88, 0.88, 0.86), collide=False))
    return Body.build("bowl", pos=(BOWL_AT[0], BOWL_AT[1], TOP), mass=0.30, shapes=shapes, color=WHITE,
                      friction=(0.6, 0.5))


def _dish() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.002), DISH_R, 0.004, BLUE)]
    n = 22
    for i in range(n):
        a = 2 * math.pi * i / n
        shapes.append(BoxShape(((DISH_R - 0.003) * math.cos(a), (DISH_R - 0.003) * math.sin(a), 0.007),
                               (0.006, 2 * math.pi * DISH_R / n * 1.12, 0.010), BLUE, yaw=a))
    return Body.build("saucer", pos=(DISH_AT[0], DISH_AT[1], TOP), mass=0.30, shapes=shapes, color=BLUE,
                      friction=(0.6, 0.5))


OBJECTS = [_lever("lever_left", 1.0), _lever("lever_right", -1.0), _walnut(), _store_box(), _kernel(),
           *[_shell(n, k + 1) for k, n in enumerate(SHELLS)], _bowl(), _dish()]
hinge("pivot", "lever_left", "lever_right", anchor=(PX, PY, TOP + 0.01), axis="Z",
      limits_deg=(-2.0 * (OPEN_DEG - SHUT_LIMIT), 0.0), spring=SPRING, damping=0.0005, target_deg=PRELOAD)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"cracked": False, "crack_t": None, "pressed": 0, "peak": 0.0, "crack_f": None, "tcps": None, "s": {},
            "min_half": OPEN_DEG}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("NUT_DEBUG")), "steps": 0}


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


def _place(base, origin, name, pos, yaw: float = 0.0) -> None:
    import torch

    asset = base.scene[name]
    pose = torch.tensor([[*(np.asarray(pos, float) + origin).tolist(), 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]],
                        dtype=torch.float32, device=asset.device)
    asset.write_root_pose_to_sim(pose)
    asset.write_root_velocity_to_sim(torch.zeros(1, 6, dtype=torch.float32, device=asset.device))


def _yaw(rot) -> float:
    return math.degrees(math.atan2(float(rot[1, 0]), float(rot[0, 0])))


def _crack(base, origin, pivot, d) -> None:
    """The shell gives: the walnut goes to the store; the kernel and the three pieces of shell fall out along the V."""
    _place(base, origin, "walnut", _store(4))
    yaw = math.atan2(float(d[1]), float(d[0]))
    for name in PIECES:
        _place(base, origin, name, pivot + d * LAYOUT[name] + [0.0, 0.0, 0.002], yaw)


def _on_dish(p, s_pos) -> bool:
    return float(np.linalg.norm(p[:2] - s_pos[:2])) < DISH_R - 0.006 and s_pos[2] - 0.005 < p[2] < s_pos[2] + ON_DISH_Z


def _in_bowl(p, b_pos) -> bool:
    return float(np.linalg.norm(p[:2] - b_pos[:2])) < BOWL_R - 0.008 and b_pos[2] - 0.005 < p[2] < b_pos[2] + BOWL_H


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    l_pos, l_rot = _pose(base, origin, "lever_left")
    r_pos, r_rot = _pose(base, origin, "lever_right")
    rel = (_yaw(r_rot) - _yaw(l_rot) + 180.0) % 360.0 - 180.0  # 0 open; negative as they close
    half = OPEN_DEG + rel / 2.0  # each lever's angle from the V's middle line
    _EP["min_half"] = min(_EP["min_half"], half)
    d = _u((_yaw(l_rot) + 180.0 - OPEN_DEG + _yaw(r_rot) + 180.0 + OPEN_DEG) / 2.0)  # along the V's middle line
    n_pos, _ = _pose(base, origin, "walnut")
    v = n_pos - l_pos
    along, across = float(v @ d), float(abs(v[0] * d[1] - v[1] * d[0]))
    between = JAW[0] < along < JAW[1] and across < 0.015 and n_pos[2] < TOP + 0.01
    f_left, f_right = (float(np.linalg.norm(f)) for f in contact_forces("walnut", ("lever_left", "lever_right")))
    squeeze = min(f_left, f_right)  # what both jaws press it with
    if not _EP["cracked"]:
        _EP["peak"] = max(_EP["peak"], squeeze)
        pressed = between and half <= OPEN_DEG - 1.0 and squeeze >= CRACK_N
        _EP["pressed"] = _EP["pressed"] + 1 if pressed else 0
        if _EP["pressed"] >= CRACK_STEPS:
            _EP["cracked"] = True
            _EP["crack_t"] = _DEBUG["steps"] / 20.0
            _EP["crack_f"] = (round(f_left, 1), round(f_right, 1))
            _crack(base, origin, l_pos, d)
    pose = {n: _pose(base, origin, n)[0] for n in PIECES}
    b_pos, _ = _pose(base, origin, "bowl")
    s_pos, _ = _pose(base, origin, "saucer")
    cracked = _EP["cracked"]
    in_bowl = cracked and _in_bowl(pose["kernel"], b_pos)
    on_dish = sum(cracked and _on_dish(pose[n], s_pos) for n in SHELLS)
    shell_in_bowl = cracked and any(_in_bowl(pose[n], b_pos) for n in SHELLS)
    _EP["tcps"] = tcps = _tcps(base, origin)
    _EP["s"] = {"cracked": cracked, "in_bowl": in_bowl, "on_dish": on_dish, "shell_in_bowl": shell_in_bowl}
    _DEBUG["steps"] += 1
    if _DEBUG["on"] and (_DEBUG["steps"] % 10 == 0 or (squeeze > 1.0 and not cracked) or
                         _EP["crack_t"] == (_DEBUG["steps"] - 1) / 20.0):
        tl, tr = tcps
        mm = lambda p: np.round(p * 1000).astype(int).tolist()  # noqa: E731
        print(f"[nut] t={_DEBUG['steps'] / 20:5.2f}s half={half:5.1f} min={_EP['min_half']:5.1f} "
              f"between={between} along={along * 1000:5.1f} across={across * 1000:5.1f} "
              f"jaw_N=({f_left:5.1f},{f_right:5.1f}) peak={_EP['peak']:5.1f} cracked={cracked} crack_t={_EP['crack_t']} "
              f"crack_f={_EP['crack_f']} pivot={mm(l_pos)} walnut={mm(n_pos)} pieces={ {n: mm(pose[n]) for n in PIECES} } "
              f"in_bowl={in_bowl} on_dish={on_dish} shell_in_bowl={shell_in_bowl} home={_arms_home()} L={mm(tl)} "
              f"R={mm(tr)} score={_points(_status())}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    return {"cracked": bool(st.get("cracked")), "in_bowl": bool(st.get("in_bowl")), "on_dish": st.get("on_dish", 0),
            "shell_in_bowl": bool(st.get("shell_in_bowl")), "home": _arms_home()}


def _points(s) -> float:
    if not s["cracked"]:
        return 0.0
    if s["in_bowl"] and s["on_dish"] == len(SHELLS):
        return 1.0 if s["home"] else 0.75
    if s["in_bowl"] and not s["shell_in_bowl"]:
        return 0.5
    return 0.25


def score(objects) -> float:
    """Cracked: both jaws pressed the walnut with at least 40 N each (PhysX contact forces) for two control steps
    running (0.1 s), while it lay between them (4-9.8 cm out from the pivot, within 1.5 cm of the V's middle line, on
    the worktop) and the levers were pressed in at least 1 degree from fully open; the shell then gives into a kernel
    and three pieces of shell. Judged at the end: 1.0 when it cracked, the kernel is in the bowl (its middle within
    4.2 cm of the bowl's middle, inside it), all three pieces of shell are on the saucer (each middle within 5.6 cm of
    its middle and at most 4 cm above its base) and both arms are back where they started (each TCP within 5 cm of its
    start); 0.75 for the same with an arm not back; 0.5 when it cracked and the kernel is in the bowl with no shell in
    it; 0.25 when it cracked; else 0.0."""
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
    """Full score with the kernel, the three pieces of shell, the bowl and the saucer still."""
    return (score(objects) >= 1.0 and all(objects[n].resting() for n in PIECES)
            and objects["bowl"].resting() and objects["saucer"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Each hand comes down beside the outside of one handle's red grip, fingers closed and leaning in, and the two push the
# grips toward each other. The soft arms push only in proportion to how far they are held short of where they aim, so
# they aim well past the grips (about 5 mm per newton) until the shell cracks. They draw back and the spring opens the
# levers. The right hand takes the kernel to the bowl and goes back to where it started; the left takes the three
# pieces of shell to the saucer, one by one, and goes back to where it started too.

FACE = 0.0125  # a closed gripper's side, this far from the TCP across its closing axis
PUSH_F = 0.7  # the hands push each grip this far along it from the bend
PUSH_Z = TOP + 0.017  # the TCP: the fingertips just above the worktop, on the grip's side
TILT = 20.0  # degrees the fingertips lean in (the wrists out); the push straightens them
SQUEEZE_N = 25.0  # what each hand aims to push with (N)
SQUEEZE_STEPS, SQUEEZE_STEP_S = 12, 0.5
LEAD_S, LEAD_STEPS = 3.0, 15
LIFT = 0.06
PICK_Z = 0.0145  # the TCP over a piece of shell: the fingertips 1 mm above the worktop
KERNEL_Z = 0.020  # ... over the kernel: its top no more than 4 mm above the TCP (the fingers narrow above it)
SQUEEZE_MM = 0.002  # the fingers aim this far inside what they hold: a gentle hold (about 10 N) for light pieces
KP = np.diag([80.0, 80.0, 80.0, 10.0, 10.0, 10.0])  # the arm joints' stiffness (N m/rad), yam_arena's actuators
ACROSS = grip_rotation(0.0)  # top-down, fingers closing along y (across the V)


def _along_x(side: float) -> np.ndarray:
    """Top-down, fingers closing along x (along the V); the wrist turned the way each arm reaches easily."""
    return grip_rotation(-math.pi / 2 if side > 0 else math.pi / 2)


def _rx(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])


def _grip_dir(side: float, half: float) -> float:
    return 180.0 - side * (half - PARALLEL_DEG)


def _push_at(side: float, half: float, gap: float = 0.0) -> np.ndarray:
    """The TCP beside the outside of a handle's grip (side +1 left, -1 right) with the lever ``half`` degrees from the
    V's middle line, ``gap`` m off it."""
    g = _u(_grip_dir(side, half))
    out = np.array([g[1], -g[0], 0.0]) * side  # away from the V's middle
    c = np.array([PX, PY, 0.0]) + _u(180.0 - side * half) * BEND + g * GRIP_LEN * PUSH_F + out * (
        GRIP_W / 2 + FACE + gap)
    return np.array([c[0], c[1], PUSH_Z])


def _push_rot(side: float) -> np.ndarray:
    return _rx(-side * TILT) @ _along_x(side)


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


def _give(arm: str, pos, rot, force) -> np.ndarray:
    """How far the TCP gives (m) under ``force`` (N) at ``pos``, the joints held by their stiffness, the hand free to
    turn: J K^-1 J^T f."""
    from yam_arena import ik, station

    q = _good_ik(arm, pos, rot)
    p0, r0 = ik.tcp_pose(q, station.BASES[arm])
    jac = np.zeros((6, 6))
    for i in range(6):
        dq = q.copy()
        dq[i] += 1e-6
        p, r = ik.tcp_pose(dq, station.BASES[arm])
        dr = r @ r0.T
        jac[:3, i] = (p - p0) / 1e-6
        jac[3:, i] = np.array([dr[2, 1] - dr[1, 2], dr[0, 2] - dr[2, 0], dr[1, 0] - dr[0, 1]]) / 2e-6
    return (jac @ np.linalg.inv(KP) @ jac.T)[:3, :3] @ np.asarray(force, float)


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = _good_ik(arm, pos, rot), np.array(READY, float)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, "back to where it started", r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _squeeze(side: float, arm: str) -> list[Waypoint]:
    name = "left" if side > 0 else "right"
    rot = _push_rot(side)
    start, touch = _push_at(side, OPEN_DEG), _push_at(side, OPEN_DEG - 6.0)
    # then aim past the grip so that the arm's give leaves it pushing about SQUEEZE_N across the V; half the give
    # along x (the nutcracker slides freely: the hands must not drag it) and 80 % of it in z (so that the hand presses
    # down a little rather than riding up over the grip)
    aim = _give(arm, touch, rot, [0.0, -side * SQUEEZE_N, 0.0]) * [0.5, 1.0, 0.8]
    out = _lead_in(arm, start + [0.0, side * 0.02, 0.05], rot, f"over the outside of the {name} handle's grip",
                   FINGER_CLOSED)
    out += [_w(1.0, start + [0.0, side * 0.02, 0.0], FINGER_CLOSED, "down beside it, fingers closed", rot),
            _w(0.6, start, FINGER_CLOSED, "against the grip", rot)]
    for k in range(1, 4):  # close the jaws onto the walnut
        out.append(_w(0.5, _push_at(side, OPEN_DEG - 6.0 * k / 3), FINGER_CLOSED,
                      "push the handles together: the jaws close on the walnut", rot))
    for k in range(1, SQUEEZE_STEPS + 1):
        out.append(_w(SQUEEZE_STEP_S, touch + aim * k / SQUEEZE_STEPS, FINGER_CLOSED,
                      "push harder: the jaws crack the shell", rot))
    back = _push_at(side, SHUT_LIMIT, 0.06) + [0.0, 0.0, 0.08]
    out += [_w(1.0, touch + aim, FINGER_CLOSED, "keep pushing", rot),
            _w(1.2, _push_at(side, SHUT_LIMIT, 0.01), FINGER_CLOSED, "ease off", rot),
            _w(0.8, back, FINGER_CLOSED, "draw back and up: the spring opens the levers", rot)]
    return out


def _pick_place(src, dst, rot, width, name, dst_name, drop_z) -> list[Waypoint]:
    """Take a piece ``width`` m across (along the fingers' closing axis) from ``src`` and drop it over ``dst``."""
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    open_w, hold = -(width / 2 + 0.008), -(width / 2 - SQUEEZE_MM)
    return [_w(2.0, src + [0.0, 0.0, LIFT], open_w, f"over the {name}", rot),
            _w(1.2, src, open_w, "down round it", rot),
            _w(0.4, src, open_w, "settle", rot),
            _w(0.6, src, hold, f"take the {name}", rot),
            _w(0.9, src + [0.0, 0.0, LIFT], hold, "lift it", rot),
            _w(1.6, [dst[0], dst[1], drop_z], hold, f"over the {dst_name}", rot),
            _w(0.5, [dst[0], dst[1], drop_z], open_w, "let it drop", rot),
            _w(0.6, [dst[0], dst[1], drop_z + 0.03], open_w, "rise", rot)]


def _laid(name: str, z: float) -> np.ndarray:
    """Where a piece lies once the shell has given (on the V's middle line, the nutcracker where it started)."""
    return np.array([PX - LAYOUT[name], PY, TOP + z])


def _wait(arm: list, until: float, label: str) -> None:
    t = sum(w.duration for w in arm)
    arm.append(_w(max(until - t, 0.3), arm[-1].pos, arm[-1].finger, label, arm[-1].yaw))


def demo():
    left = _squeeze(1.0, "left")
    right = _squeeze(-1.0, "right")
    right += _pick_place(_laid("kernel", KERNEL_Z), BOWL_AT, ACROSS, KERNEL[1], "kernel", "bowl",
                         TOP + BOWL_H + 0.045)
    kernel_away = sum(w.duration for w in right[:-3])  # the kernel is up and on its way to the bowl
    right += _go_home("right", right[-1].pos, ACROSS, FINGER_OPEN)
    _wait(left, kernel_away - 2.0, "wait for the right hand to take the kernel away")
    for name, at, which in (("shell_c", (-0.025, -0.02), "first"), ("shell_b", (0.025, -0.02), "next"),
                            ("shell_a", (0.0, 0.025), "last")):
        lx, ly, lz = SHELL[name]
        left += _pick_place(_laid(name, PICK_Z), DISH_AT + at, ACROSS, ly,
                            f"{which} piece of shell", "saucer", TOP + 0.045)
    left += _go_home("left", left[-1].pos, ACROSS, FINGER_OPEN)
    total = max(sum(w.duration for w in left), sum(w.duration for w in right)) + 3.0
    for arm in (left, right):
        arm.append(_w(max(total - sum(w.duration for w in arm), 0.5), arm[-1].pos, FINGER_OPEN,
                      "hold still: the trial ends once the full score has held 1 s", arm[-1].yaw))
    return {"left": left, "right": right}
