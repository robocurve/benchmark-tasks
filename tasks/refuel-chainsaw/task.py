"""On the station table, a vintage chainsaw lies on its flat base, its bar pointing away from the robot to the right, its
fuel cap off. Beside it stands a 5 L two-stroke mix can holding fuel, its spout open. The task: pour the fuel from the
can into the chainsaw's tank without spilling, and stand the can back on the table.

The fuel is about a thousand small rigid beads (soilkit) with damping and friction. The chainsaw is a fixed prop
(USDCraft-10K ds_R951_08 visuals) with our own hollow tank and a 4 cm filler opening; the premix bottle is our own hollow shell with a
60 mm spout. See README.md.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import scenekit  # noqa: E402
from scenekit import UsdBody, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import capkit  # noqa: E402
import soilkit  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Top up the chainsaw's fuel tank from the premix bottle: pour the fuel into the tank without spilling."

MM = 0.001
TOP = TABLE_TOP[SCENE]

# -- chainsaw (its own frame: origin on the table under the housing, bar toward +x, z up), turned so the bar points -y
SAW_XY = (0.45, 0.005)
SAW_YAW = -np.pi / 2
SAW_QUAT = (0.0, 0.0, float(np.sin(SAW_YAW / 2)), float(np.cos(SAW_YAW / 2)))
NECK_LOC = (-0.195, -0.05)  # fuel neck centre in the saw's frame
NECK_HOLE = 0.040
TANK = ((-0.280, -0.170), (-0.095, -0.005), (0.100, 0.190))  # inner x, y, z of the hollow tank (saw frame)
LID_TOP = 0.200

# -- can (origin on the table at the centre of its base; long axis x, spout toward +x)
CAN_L, CAN_W, CAN_H = 0.085, 0.070, 0.200
WALL = 4 * MM
SPOUT_X, SPOUT_Y, SPOUT_LEN, SPOUT_ID = 0.020, 0.019, 0.060, 0.024
GRIP_LOC = (0.0, 0.0, 0.110)  # where the pads hold the bottle: its body at mid height (the centre of mass), closing on its 70 mm width
CAN_XY = (0.35, 0.55)
CAN_MASS = 0.12

SAW_ORIGIN = (SAW_XY[0], SAW_XY[1], TOP)


def saw_to_world(p):
    """A point in the saw's frame in the station frame."""
    x, y, z = p
    c, s = np.cos(SAW_YAW), np.sin(SAW_YAW)
    return np.array([SAW_XY[0] + c * x - s * y, SAW_XY[1] + s * x + c * y, TOP + z])


HOLE = saw_to_world((*NECK_LOC, LID_TOP))  # the neck hole's centre in the lid, station frame

SAW = UsdBody("chainsaw", usd="assets/chainsaw.usdc", pos=SAW_ORIGIN, rot_xyzw=SAW_QUAT, mass=None, material=False,
              size=(0.9, 0.28, 0.34))
CAN = UsdBody("can", usd="assets/can.usda", pos=(*CAN_XY, TOP), mass=CAN_MASS, material=False, solver_iters=32, 
              size=(CAN_L, CAN_W, CAN_H + SPOUT_LEN))

# -- the fuel: beads of 2.0-2.8 mm radius lying on the can's floor (a scaled-down dose of the 5 L can's contents)
R_MIN, R_MAX = float(os.environ.get("BEAD_RMIN") or 2.0) * MM, float(os.environ.get("BEAD_RMAX") or 2.8) * MM
FILL_H = float(os.environ.get("FILL_H") or 0.045)
FUEL = soilkit.Soil("fuel", pos=(*CAN_XY, TOP + WALL + FILL_H / 2),
                    size=(CAN_L - 2 * WALL - 1 * MM, CAN_W - 2 * WALL - 1 * MM, FILL_H),
                    r_min=R_MIN, r_max=R_MAX, density=1000.0, friction=(0.5, 0.4),
                    angular_damping=float(os.environ.get("BEAD_ANGDAMP") or 10.0),
                    linear_damping=float(os.environ.get("BEAD_LINDAMP") or 2.0),
                    colors=((0.95, 0.80, 0.15), (0.98, 0.86, 0.25), (0.90, 0.72, 0.10)),
                    start_file=os.environ.get("FUEL_START") or ("assets/fuel_start.npz"
                                                                 if (pathlib.Path(__file__).parent / "assets/fuel_start.npz").is_file() else None))
OBJECTS = [SAW, CAN, FUEL]  # the tank's neck and the can's spout are open: neither cap is part of the task

# -- caps -----------------------------------------------------------------------------------------------------------
_SEATS: dict = {}


def cap_status() -> dict:
    return {}


_n_log = [0]


@on_step
def _log(environment) -> None:
    _n_log[0] += 1
    if os.environ.get("FUEL_LOG") and _n_log[0] % 5 == 0:
        p, r = soilkit.grains("fuel")
        print(f"LOG t={_n_log[0] / 20:5.1f} caps {cap_status()} beads-z-min {p[:, 2].min() - TOP:.3f}", flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------
SPILL_MAX = 0.10  # more than this share of the fuel outside both the can and the tank: 0
FULL = float(os.environ.get("FULL") or 0.80)  # share of the fuel in the tank, for the last step


def _local(body, p):
    return np.array([capkit.qrot(capkit.qinv(body.quat_xyzw), q - body.pos) for q in p])


def in_tank(body, p) -> np.ndarray:
    """Bead centres inside the tank's cavity or in the neck hole above it (the saw's frame)."""
    loc = _local(body, p)
    (x0, x1), (y0, y1), (z0, z1) = TANK
    cavity = (loc[:, 0] > x0) & (loc[:, 0] < x1) & (loc[:, 1] > y0) & (loc[:, 1] < y1) & (loc[:, 2] > z0) & (loc[:, 2] < z1)
    nx, ny = NECK_LOC
    hole = (np.abs(loc[:, 0] - nx) < NECK_HOLE / 2) & (np.abs(loc[:, 1] - ny) < NECK_HOLE / 2) & \
           (loc[:, 2] >= z1) & (loc[:, 2] < LID_TOP + 0.03)
    return cavity | hole


def in_can(body, p) -> np.ndarray:
    """Bead centres inside the can's shell or its spout bore (the can's frame)."""
    loc = _local(body, p)
    shell = (np.abs(loc[:, 0]) < CAN_L / 2) & (np.abs(loc[:, 1]) < CAN_W / 2) & (loc[:, 2] > -0.005) & (loc[:, 2] < CAN_H)
    spout = (np.hypot(loc[:, 0] - SPOUT_X, loc[:, 1] - SPOUT_Y) < SPOUT_ID / 2 + 0.002) & (loc[:, 2] >= CAN_H - 0.01) & \
            (loc[:, 2] < CAN_H + SPOUT_LEN + 0.003)
    return shell | spout


def shares(objects, grains=None) -> tuple[float, float]:
    """(share of the fuel's mass in the tank, share outside both the can and the tank)."""
    p, r = grains if grains is not None else soilkit.grains("fuel")
    m = r ** 3
    tank = in_tank(objects["chainsaw"], p)
    can = in_can(objects["can"], p)
    return float(m[tank].sum() / m.sum()), float(m[~tank & ~can].sum() / m.sum())


def in_tank(body, p) -> np.ndarray:
    """Bead centres inside the tank's cavity or in the neck hole above it (the saw's frame)."""
    loc = _local(body, p)
    (x0, x1), (y0, y1), (z0, z1) = TANK
    cavity = (loc[:, 0] > x0) & (loc[:, 0] < x1) & (loc[:, 1] > y0) & (loc[:, 1] < y1) & (loc[:, 2] > z0) & (loc[:, 2] < z1)
    nx, ny = NECK_LOC
    hole = (np.abs(loc[:, 0] - nx) < NECK_HOLE / 2) & (np.abs(loc[:, 1] - ny) < NECK_HOLE / 2) & \
           (loc[:, 2] >= z1) & (loc[:, 2] < LID_TOP + 0.03)
    return cavity | hole


def in_can(body, p) -> np.ndarray:
    """Bead centres inside the can's shell or its spout bore (the can's frame)."""
    loc = _local(body, p)
    shell = (np.abs(loc[:, 0]) < CAN_L / 2) & (np.abs(loc[:, 1]) < CAN_W / 2) & (loc[:, 2] > -0.005) & (loc[:, 2] < CAN_H)
    spout = (np.hypot(loc[:, 0] - SPOUT_X, loc[:, 1] - SPOUT_Y) < SPOUT_ID / 2 + 0.002) & (loc[:, 2] >= CAN_H - 0.01) & \
            (loc[:, 2] < CAN_H + SPOUT_LEN + 0.003)
    return shell | spout


def shares(objects, grains=None) -> tuple[float, float]:
    """(share of the fuel's mass in the tank, share outside both the can and the tank)."""
    p, r = grains if grains is not None else soilkit.grains("fuel")
    m = r ** 3
    tank = in_tank(objects["chainsaw"], p)
    can = in_can(objects["can"], p)
    return float(m[tank].sum() / m.sum()), float(m[~tank & ~can].sum() / m.sum())


UPRIGHT_DEG = 20.0  # the can ends standing: its up axis within this of vertical


def _can_upright(can) -> bool:
    up = capkit.qrot(can.quat_xyzw, [0, 0, 1.0])
    return bool(np.degrees(np.arccos(np.clip(up[2], -1, 1))) < UPRIGHT_DEG)


def score(objects, grains=None) -> float:
    """0 if over SPILL_MAX of the fuel is outside both the can and the tank; else 0.25, 0.5, 0.75 for 25, 50, 75 % of
    its mass in the tank, and 1.0 for FULL of it (80 %)."""
    got, spilled = shares(objects, grains)
    if spilled > SPILL_MAX:
        return 0.0
    if got >= FULL:
        return 1.0
    for need, value in ((0.75, 0.75), (0.5, 0.5), (0.25, 0.25)):
        if got >= need:
            return value
    return 0.0


# -- scripted demo ----------------------------------------------------------------------------------------------------
def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_about(axis, angle):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * (k @ k)


PHI_90 = np.radians(float(os.environ.get("PHI_T") or 60.0))  # the roll while carrying: beads must not reach the spout yet
PHI_POUR = np.radians(float(os.environ.get("PHI") or 123.0))  # how far the can is rolled to pour (the wrist's reach ends near here)
T_POUR = float(os.environ.get("T_POUR") or 14.0)  # s held rolled
# grip point (the grip tab's centre) to the spout tip, in the can's frame
V_TIP = np.array([SPOUT_X, SPOUT_Y, CAN_H + SPOUT_LEN]) - np.array(GRIP_LOC)


def demo():
    """The left arm takes the can's cap off (straight down, turning the wrist) and the saw's cap off, and sets both on
    the table. It then takes the can by its carry bar from behind, hand pointing forward, fingers closing across the
    bar, lifts it, rolls it about the bar's axis (the wrist's roll) to lie on its side, carries it over the tank neck,
    rolls it on until the fuel runs, holds, and brings it back. Last it screws the saw's cap back on."""
    from yam_arena.constants import FINGER_CLOSED as C
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.demo import Waypoint as W

    import tiltkit

    tiltkit.install()
    Y0, Y1 = float(os.environ.get("YAW0") or -0.9), float(os.environ.get("YAW1") or 0.9)
    park_l, park_r = (0.20, 0.30, TOP + 0.20), (0.20, -0.30, TOP + 0.22)
    hx, hy = float(HOLE[0]), float(HOLE[1])

    left = []
    # -- the can: hand forward, fingers closing across the carry bar; the can follows the hand rigidly
    psi_g = np.radians(float(os.environ.get("PSI_G") or 5))
    r_grip = _rx(psi_g) @ tiltkit.rotation((1, 0, 0), (0, -1, 0))
    bar0 = np.array([CAN_XY[0] + GRIP_LOC[0], CAN_XY[1] + GRIP_LOC[1], TOP + GRIP_LOC[2]])

    def hand_for_tip(tip, phi):
        return np.asarray(tip, float) - _rx(-phi) @ V_TIP

    lift_z = float(os.environ.get("LIFT_Z") or 0.36)
    hover_z = float(os.environ.get("HOVER_Z") or 0.31)
    pour_z = float(os.environ.get("POUR_Z") or 0.195)
    up0 = bar0 + [0, 0, lift_z - GRIP_LOC[2]]
    tip_dy = float(os.environ.get("TIP_DY") or -0.016)
    tip_hover = np.array([hx, hy + tip_dy, TOP + hover_z])
    tip_pour = np.array([hx - float(os.environ.get("TIP_DX") or 0.0), hy + tip_dy, TOP + pour_z])
    hover = hand_for_tip(tip_hover, PHI_90)
    # keyframes (end time, hand position or None, tip position or None, roll): between two keyframes that both give the tip,
    # the TIP moves on a straight line while the roll changes (the hand's position follows), so the spout stays over the neck
    t = 0.0
    keys = [(0.0, bar0, None, 0.0)]
    seq = (os.environ.get("SEQ") or "0") == "1"  # lower the spout to the neck first (carry roll), then roll on to pour
    plan = [(1.5, up0, None, 0.0), (2.5, up0, None, PHI_90), (3.0, hover, tip_hover, PHI_90)]
    if seq:
        plan += [(2.5, None, tip_pour, PHI_90), (4.0, None, tip_pour, PHI_POUR), (T_POUR, None, tip_pour, PHI_POUR),
                 (3.0, None, tip_pour, PHI_90), (2.5, None, tip_hover, PHI_90)]
    else:
        roll_s = float(os.environ.get("ROLL_S") or 7.0)  # a slow roll while the tip is brought onto the opening
        plan += [(roll_s, None, tip_pour, PHI_POUR), (T_POUR, None, tip_pour, PHI_POUR), (3.0, None, tip_hover, PHI_90)]
    plan += [(3.0, up0, None, PHI_90), (2.5, up0, None, 0.0), (1.5, bar0 + [0, 0, 0.004], None, 0.0)]
    for dur, a, tp, ph in plan:
        t += dur
        keys.append((t, None if a is None else np.asarray(a, float), tp, ph))

    def at(tt):
        for (ta, pa, ta_tip, fa), (tb, pb, tb_tip, fb) in zip(keys, keys[1:]):
            if tt <= tb:
                u = min(1.0, max(0.0, (tt - ta) / (tb - ta)))
                u = u * u * (3 - 2 * u)
                ph = fa + (fb - fa) * u
                if ta_tip is not None and tb_tip is not None:
                    return hand_for_tip(ta_tip + (tb_tip - ta_tip) * u, ph), ph
                pa = pa if pa is not None else hand_for_tip(ta_tip, fa)
                pb = pb if pb is not None else hand_for_tip(tb_tip, fb)
                return pa + (pb - pa) * u, ph
        last = keys[-1]
        return (last[1] if last[1] is not None else hand_for_tip(last[2], last[3])), last[3]

    pre = bar0 - np.array([0.08, 0, 0])
    left += [
        W(3.0, tuple(pre + [0, 0, 0.15]), O, "above and behind the grip tab", tiltkit.Tilt(r_grip, joint=True)),
        W(2.0, tuple(pre), O, "down behind the tab", tiltkit.Tilt(r_grip)),
        W(1.5, tuple(bar0), O, "round the tab", tiltkit.Tilt(r_grip)),
        W(1.0, tuple(bar0), C, "grip", tiltkit.Tilt(r_grip)),
    ]
    step = 0.5
    for (ta, pa, tipa, fa), (tb, pb, tipb, fb) in zip(keys, keys[1:]):
        dur = tb - ta
        held = abs(fa - fb) < 1e-9 and ((tipa is not None and tipb is not None and np.allclose(tipa, tipb))
                                         or (pa is not None and pb is not None and np.allclose(pa, pb)))
        n = 1 if held else max(1, int(round(dur / step)))  # a hold is one waypoint: no jerk from re-planning every half second
        for k in range(1, n + 1):
            p_, ph = at(ta + dur * k / n)
            left.append(W(dur / n, tuple(p_), C, "hold" if held else "pour",
                          tiltkit.Tilt(_rx(-ph) @ r_grip, load=(CAN_MASS + 0.13, V_TIP * 0))))
    left += [
        W(1.0, tuple(bar0 + [0, 0, 0.004]), O, "let go", tiltkit.Tilt(r_grip)),
        W(2.0, tuple(pre + [0, 0, 0.05]), O, "back off", tiltkit.Tilt(r_grip)),
    ]
    left.append(W(2.0, park_l, O, "back", tiltkit.Tilt(None, joint=True)))
    total = sum(w.duration for w in left)
    right = [W(total, park_r, O, "wait", 0.0)]
    return {"left": left, "right": right}


def success(objects) -> bool:
    return score(objects) == 1.0


def self_test(BodyState=None):
    """The scoring rules on hand-built states: beads where we put them, the cap flag forced by hand."""
    if BodyState is None:
        from scenekit import BodyState

    def st(pos, quat=(0.0, 0.0, 0.0, 1.0)):
        z = np.zeros(3)
        return BodyState(pos=np.array(pos, float), quat_xyzw=np.array(quat, float), lin_vel=z, ang_vel=z, low=z, high=z,
                         joints={})

    saw, can = st(SAW_ORIGIN, SAW_QUAT), st((*CAN_XY, TOP))
    objs = {"chainsaw": saw, "can": can}
    n = 100
    r = np.full(n, 2.4 * MM)
    tank_pt = saw_to_world((-0.225, -0.05, 0.15))  # inside the cavity
    neck_pt = saw_to_world((*NECK_LOC, 0.21))  # in the hole
    can_pt = (CAN_XY[0], CAN_XY[1], TOP + 0.01)
    table_pt = (0.65, 0.30, TOP + 0.002)
    lid_pt = saw_to_world((-0.20, -0.02, 0.203))  # on the lid beside the hole: outside both

    def beads(n_tank, n_can=None, n_table=0, n_lid=0):
        n_can = n - n_tank - n_table - n_lid if n_can is None else n_can
        pts = [tuple(tank_pt)] * n_tank + [can_pt] * n_can + [table_pt] * n_table + [tuple(lid_pt)] * n_lid
        arr = np.array(pts, float).reshape(-1, 3)
        return arr, np.full(len(arr), 2.4 * MM)

    cases = {
        "nothing poured": (0.0, beads(0), False),
        "30 % poured": (0.25, beads(30), False),
        "60 % poured": (0.5, beads(60), False),
        "76 % poured": (0.75, beads(76), False),
        "all poured": (1.0, beads(100), True),
        "90 % poured": (1.0, beads(90), True),
        "81 % poured": (1.0, beads(81), True),
        "only 40 % poured": (0.25, beads(40), True),
        "60 % poured but 32 % on the table": (0.0, beads(60, 8, 32), True),
        "78 % poured, 5 % on the table, rest in the bottle": (0.75, beads(78, 17, 5), True),
        "60 % poured, 32 % left on the lid": (0.0, beads(60, 8, 0, 32), True),
        "beads in the neck count": (1.0, (np.array([neck_pt] * 90 + [can_pt] * 10, float), r), True),
    }
    for name, (want, (p, rr), _) in cases.items():
        got = score(objs, (p, rr))
        assert abs(got - want) < 1e-9, (name, got, want)
    return len(cases)


if __name__ == "__main__":
    print(f"scoring: {self_test()} cases pass")
    p, r = soilkit.start_layout(FUEL)
    print(f"fuel: {len(p)} beads, {np.sum(1000 * 4 / 3 * np.pi * r ** 3) * 1000:.0f} g")
    print("hole", np.round(HOLE, 3))
