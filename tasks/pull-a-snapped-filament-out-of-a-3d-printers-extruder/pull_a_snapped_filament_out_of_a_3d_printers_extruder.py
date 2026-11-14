"""On the station table stands the corner of a 3D printer: a black aluminium frame beam, the stepper motor and the
black extruder on it, its red tension lever lying on top, a spool of orange filament on its holder behind. A piece of
orange filament has snapped off inside the extruder: it sticks out of the extruder's inlet to the left, its other end
held between the drive gear and the idler bearing that the lever's spring presses on it. A grey scrap tray stands in
front.

Task: one hand holds the red lever pressed down (that lifts the idler off the filament) while the other pulls the
broken piece out of the extruder; drop the piece in the scrap tray, let the lever go and take both arms back to where
they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=pull_a_snapped_filament_out_of_a_3d_printers_extruder.py \\
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
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step,  # noqa: E402
                     rotation, set_slide_target, set_thread_on, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the extruder: its filament path runs along y at PATH_Z through (EX, EY) ------------------------------------------
EX, EY = 0.30, 0.0
PATH_Z = TOP + 0.085
HOUSE = (0.040, 0.045, 0.040)  # the housing: deep (x), long (y, along the path), tall
INLET_Y = EY + HOUSE[1] / 2 + 0.012  # the inlet fitting's mouth (the filament comes out here)
GEAR_Y = EY  # the drive gear and the idler bearing pinch the filament here
# -- the tension lever: hinged about x on top of the housing, its arm reaching toward -y to a pad -----------------------
LEVER_PIVOT = np.array([EX, EY + 0.012, TOP + 0.108])
LEVER_LEN = 0.095  # pivot to the pad's middle
PAD = (0.026, 0.026, 0.008)
OPEN_DEG = 8.0  # pressed down past 8 degrees, the idler is off the filament
LEVER_MAX_DEG = 20.0
RELEASED_DEG = 4.0
# -- the snapped piece of filament: along y, its broken tip inside the extruder past the drive gear --------------------
FIL_R, FIL_L = 0.000875, 0.130  # standard 1.75 mm PLA filament, 13 cm
FIL_MASS = 0.0004  # 13 cm of 1.75 mm PLA (1.24 g/cm^3): 0.39 g
TIP_Y0 = EY - 0.035
FIL_MID0 = np.array([EX, TIP_Y0 + FIL_L / 2, PATH_Z])
FREE_AT = (GEAR_Y + 0.008) - TIP_Y0  # pulled this far, the tip is past the drive gear: nothing grips it any more
OUT_AT = (INLET_Y + 0.004) - TIP_Y0  # pulled this far, the tip is out of the inlet: the piece is loose
PULLED_SOME = 0.03
# -- the scrap tray ---------------------------------------------------------------------------------------------------
TX0, TX1, TY0, TY1, TRAY_H, TRAY_T = 0.150, 0.230, 0.050, 0.250, 0.022, 0.004

BLACK, ALU, RED, BRASS, ORANGE, PTFE, TRAY_GREY, MOTOR = (0.09, 0.09, 0.10), (0.20, 0.20, 0.22), (0.80, 0.12, 0.10), \
    (0.80, 0.66, 0.30), (0.98, 0.50, 0.08), (0.95, 0.95, 0.93), (0.45, 0.47, 0.50), (0.16, 0.16, 0.18)

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _printer() -> Body:
    """The printer's frame beam with the stepper motor and the extruder housing on it, the PTFE tube from the outlet,
    and the spool on its holder behind. Origin on the table under the extruder."""
    hx, hy, hz = HOUSE
    pz = PATH_Z - TOP
    shapes = [
        BoxShape((0.0, 0.0, pz), (hx, hy, hz), BLACK),  # the extruder housing
        BoxShape((hx / 2 + 0.021, 0.0, pz), (0.042, 0.042, 0.042), MOTOR),  # the stepper motor behind it
        BoxShape((hx / 2 + 0.021, 0.0, pz + 0.0215), (0.030, 0.030, 0.002), ALU, collide=False),
        BoxShape((hx / 2 + 0.052, 0.0, pz - 0.010), (0.020, 0.34, 0.020), ALU),  # the frame beam (2020 extrusion)
        CylShape((0.0, hy / 2 + 0.006, pz), 0.006, 0.012, BLACK, axis="Y", collide=False),  # the inlet fitting
        CylShape((0.0, hy / 2 + 0.0115, pz), 0.0045, 0.002, BRASS, axis="Y", collide=False),
        CylShape((-hx / 2 - 0.001, GEAR_Y - EY, pz), 0.0055, 0.002, BRASS, axis="X", collide=False),  # MK8 drive gear
        CylShape((0.0, -hy / 2 - 0.005, pz), 0.004, 0.010, PTFE, axis="Y", collide=False),  # PTFE tube out ...
        CylShape((0.065, -hy / 2 - 0.010, pz), 0.004, 0.130, PTFE, axis="X", collide=False),  # ... back ...
        CylShape((0.130, -hy / 2 - 0.010, (pz + 0.25) / 2), 0.004, 0.25 - pz, PTFE, collide=False),  # ... up
    ]
    # the printer's frame behind: two uprights, the top beam, the print head on it, the bed
    for y in (-0.17, 0.17):
        shapes.append(BoxShape((0.14, y, 0.15), (0.020, 0.020, 0.30), ALU, collide=False))
    shapes += [BoxShape((0.14, 0.0, 0.29), (0.020, 0.36, 0.020), ALU, collide=False),
               BoxShape((0.125, -hy / 2 - 0.010, 0.255), (0.040, 0.050, 0.050), BLACK, collide=False),  # print head
               BoxShape((0.105, -hy / 2 - 0.010, 0.255), (0.002, 0.030, 0.030), (0.25, 0.45, 0.80), collide=False),
               CylShape((0.125, -hy / 2 - 0.010, 0.222), 0.004, 0.016, BRASS, collide=False),  # the nozzle
               BoxShape((0.25, 0.0, 0.035), (0.20, 0.30, 0.006), (0.12, 0.12, 0.13), collide=False),  # the bed
               BoxShape((0.25, 0.0, 0.0385), (0.19, 0.29, 0.001), (0.85, 0.85, 0.80), collide=False),
               BoxShape((0.14, -0.17, 0.08), (0.006, 0.07, 0.05), (0.10, 0.20, 0.45), collide=False)]  # its screen
    for y in (-0.15, 0.15):  # the beam's legs
        shapes.append(BoxShape((hx / 2 + 0.052, y, (pz - 0.020) / 2), (0.020, 0.020, pz - 0.020), ALU))
    # the spool of orange filament on its holder, behind and to the left
    sx, sy, sz = hx / 2 + 0.115, 0.10, 0.115
    shapes += [CylShape((sx, sy, sz), 0.060, 0.050, ORANGE, axis="Y", collide=False),
               CylShape((sx, sy - 0.027, sz), 0.068, 0.004, BLACK, axis="Y", collide=False),
               CylShape((sx, sy + 0.027, sz), 0.068, 0.004, BLACK, axis="Y", collide=False),
               CylShape((sx, sy + 0.040, sz), 0.006, 0.050, ALU, axis="Y", collide=False),
               BoxShape((sx, sy + 0.065, sz / 2), (0.020, 0.010, sz), ALU, collide=False)]
    return Body.build("printer", pos=(EX, EY, TOP), mass=5.0, kinematic=True, color=BLACK, shapes=shapes)


def _lever() -> Body:
    """The red aluminium tension lever: an arm from the pivot toward -y, a ridged pad at its end; under it near the
    pivot the idler bearing. Origin on the pivot."""
    arm0, arm1 = 0.006, -(LEVER_LEN + PAD[1] / 2)
    shapes = [BoxShape((0.0, (arm0 + arm1) / 2, 0.0), (0.014, arm0 - arm1, 0.006), RED),
              BoxShape((0.0, -LEVER_LEN, 0.004), PAD, RED),
              CylShape((0.0, -0.012, -0.012), 0.005, 0.004, (0.75, 0.76, 0.78), axis="X", collide=False),  # 623 idler
              BoxShape((0.0, -0.012, -0.007), (0.008, 0.008, 0.010), RED, collide=False),
              CylShape((0.0, -0.030, -0.010), 0.004, 0.014, (0.70, 0.70, 0.72), collide=False)]  # the spring
    for k in range(3):  # ridges on the pad
        shapes.append(BoxShape((0.0, -LEVER_LEN - 0.008 + 0.008 * k, 0.0085), (PAD[0], 0.002, 0.001), BLACK,
                               collide=False))
    return Body.build("lever", pos=tuple(LEVER_PIVOT), mass=0.04, shapes=shapes, friction=(1.0, 0.9), color=RED,
                      com=(0.0, -0.05, 0.0), damping=(0.0, 1.0))


def _piece() -> Body:
    """The snapped piece of orange PLA filament (standard 1.75 mm, 13 cm, 0.39 g), its broken tip inside."""
    shapes = [CylShape((0.0, 0.0, 0.0), FIL_R, FIL_L, ORANGE, axis="Y"),
              CylShape((0.0, -FIL_L / 2 + 0.002, 0.0), FIL_R * 0.8, 0.004, (0.90, 0.42, 0.05), axis="Y",
                       collide=False)]
    return Body.build("filament", pos=tuple(FIL_MID0), mass=FIL_MASS, shapes=shapes, friction=(1.2, 1.1), color=ORANGE,
                      damping=(0.3, 6.0))


def _tray() -> Body:
    """A grey plastic scrap tray."""
    lx, ly = TX1 - TX0, TY1 - TY0
    shapes = [BoxShape((0.0, 0.0, TRAY_T / 2), (lx, ly, TRAY_T), TRAY_GREY)]
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((s * (lx - TRAY_T) / 2, 0.0, TRAY_H / 2), (TRAY_T, ly, TRAY_H), TRAY_GREY))
        shapes.append(BoxShape((0.0, s * (ly - TRAY_T) / 2, TRAY_H / 2), (lx, TRAY_T, TRAY_H), TRAY_GREY))
    return Body.build("tray", pos=((TX0 + TX1) / 2, (TY0 + TY1) / 2, TOP), mass=0.3, kinematic=True, color=TRAY_GREY,
                      shapes=shapes)


OBJECTS = [_printer(), _tray(), _lever(), _piece()]
_PRINTER = OBJECTS[0]
# The lever's hinge: sprung down onto its stop (pressed shut on the filament), pressing its pad down lifts the idler.
hinge("lever_pivot", "printer", "lever", anchor=tuple(LEVER_PIVOT), axis="X", limits_deg=(0.0, LEVER_MAX_DEG),
      spring=0.02, damping=0.0004, target_deg=-5.0)
# The filament in the extruder's path: a joint authored switched on that keeps it on the path's axis and slides it
# along it, pulled by a stiff spring toward a target the task file moves. Its slide coordinate runs along -y (into
# the extruder) so that the authored target (the upper limit) is where the piece starts.
ANCHOR0 = (float(FIL_MID0[0] - EX), float(FIL_MID0[1] - EY), float(PATH_Z - TOP))
thread("feed", "printer", "filament", anchor0=ANCHOR0, axis0=(0.0, -1.0, 0.0), anchor1=(0.0, 0.0, 0.0),
       axis1=(0.0, -1.0, 0.0), slide_limits=(-0.20, 0.0), turn_damping=0.0, slide_stiffness=4000.0,
       slide_damping=20.0, slide_max_force=1.0e4, play=0.0015, tilt_deg=2.0, always_on=True)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"on": True, "hold": 0.0, "was_open": False, "prev": 0.0, "pulled": 0.0, "max_pulled": 0.0, "loose": False,
            "lever": 0.0, "opened": False, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("FILAMENT_DEBUG")), "steps": 0}
DT = 0.05


@on_reset
def _new_episode() -> None:
    was_off = not _EP.get("on", True)
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    try:
        set_slide_target("feed", 0.0)
        if was_off:  # off and on again: rebuilt from the reset poses
            set_thread_on("feed", False)
            set_thread_on("feed", True)
    except Exception:  # not authored yet (the first reset comes before the scene exists)
        pass


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


def _tilt(r) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    lp, lr = _pose(base, origin, "lever")
    fp, fr = _pose(base, origin, "filament")
    lever = math.degrees(math.atan2(lr[2, 1], lr[1, 1]))  # its turn about x: pad down is positive
    is_open = lever >= OPEN_DEG
    _EP["lever"] = lever
    _EP["opened"] = _EP["opened"] or is_open
    pulled = float(fp[1] - FIL_MID0[1])  # how far the piece has come out of the extruder
    _EP["max_pulled"] = max(_EP["max_pulled"], pulled)
    if _EP["on"]:
        if pulled >= OUT_AT:
            _EP["on"] = False  # the tip is out of the inlet: the piece is loose
            _EP["loose"] = True
            set_thread_on("feed", False)
        elif is_open or pulled >= FREE_AT:
            v = max(-0.15, min(0.15, (pulled - _EP["prev"]) / DT))
            set_slide_target("feed", -(pulled + v * DT))  # the idler is up (or the tip past the gear): it slides freely
            _EP["was_open"] = True
        else:
            if _EP["was_open"]:  # the lever has just come back down: the drive gear grips the piece where it is
                _EP["hold"] = pulled
                _EP["was_open"] = False
            set_slide_target("feed", -_EP["hold"])
    _EP["prev"] = pulled
    _EP["pulled"] = pulled
    in_tray = (not _EP["on"]) and TX0 + 0.005 < fp[0] < TX1 - 0.005 and TY0 < fp[1] < TY1 \
        and fp[2] < TOP + TRAY_H + 0.01 and abs(fr[2, 1]) < 0.5  # lying in the tray, roughly along y
    ends = [fp + fr @ np.array([0.0, s * FIL_L / 2, 0.0]) for s in (-1.0, 1.0)]
    in_tray = in_tray and all(TX0 < e[0] < TX1 and TY0 < e[1] < TY1 for e in ends)
    _EP["tcps"] = _tcps(base, origin)
    _EP["s"] = {"in_tray": in_tray, "released": lever <= RELEASED_DEG}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, trc = _EP["tcps"]
            jl = _fingers(base)
            print(f"[filament] t={_DEBUG['steps'] / 20:5.1f}s lever={lever:5.1f} open={is_open} pulled={pulled * 1000:6.1f}mm "
                  f"max={_EP['max_pulled'] * 1000:6.1f}mm on={_EP['on']} hold={_EP['hold'] * 1000:5.1f} "
                  f"fil={np.round(fp * 1000).astype(int).tolist()} fil_tilt={_tilt(fr):4.1f} in_tray={in_tray} "
                  f"leftTCP={np.round(tl * 1000).astype(int).tolist()} "
                  f"rightTCP={np.round(trc * 1000).astype(int).tolist()} fingers={jl} score={score(None)}",
                  file=sys.stderr, flush=True)


def _fingers(base) -> list:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        q = as_numpy(robot.data.joint_pos)[0]
        names = list(robot.joint_names)
        out.append([round(float(q[i]) * 1000, 1) for i, n in enumerate(names) if "joint7" in n or "joint8" in n])
    return out


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0 with the piece out of the extruder, lying in the scrap tray, the lever let go (back down within 4 degrees)
    and both arms back where they started; 0.5 with the piece out of the extruder at the end; 0.25 with the piece pulled
    at least 3 cm out of the extruder at some point; else 0.0."""
    s = _EP["s"]
    if _EP["loose"]:
        if s.get("in_tray") and s.get("released") and _arms_home():
            return 1.0
        return 0.5
    if _EP["max_pulled"] >= PULLED_SOME:
        return 0.25
    return 0.0


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
    """Full score with the piece still in the tray: the trial ends there."""
    return score(objects) >= 1.0 and objects["filament"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand comes down on the lever's pad with its fingers closed and presses it down, lifting the idler off the
# filament, and holds it there. The left hand comes down over the piece where it sticks out of the inlet, takes it
# across, pulls it straight out of the extruder along its length, lifts it, carries it over the scrap tray, lays it in
# and lets go. The right lets the lever back up; both arms go back to where they started.

PRESS = 0.034  # the pad pressed this far down (the lever stops at 20 degrees, about 3.2 cm)
GRIP_Y = FIL_MID0[1] + FIL_L / 2 - 0.022  # the left takes the piece 2.2 cm from its outer end
PULL = 0.105
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


FIL_FINGER = -0.020
G_ACROSS = grip_rotation(-math.pi / 2)  # down, fingers closing along x: across the piece
G_PAD = grip_rotation(math.pi / 2)


def _right(release_at: float) -> tuple[list[Waypoint], float]:
    pad = LEVER_PIVOT + [0.0, -LEVER_LEN, PAD[2]]  # the pad's top middle
    over = pad + [0.0, 0.0, 0.04]
    out = _lead_in("right", over, G_PAD, "over the lever's pad, fingers closed", FINGER_CLOSED)
    out += [_w(1.0, pad + [0.0, 0.0, 0.002], FINGER_CLOSED, "down onto the pad", G_PAD),
            _w(1.0, pad + [0.0, 0.003, -PRESS], FINGER_CLOSED, "press the lever down: the idler lifts", G_PAD)]
    pressed_at = sum(w.duration for w in out)
    out += [_w(max(release_at - pressed_at, 0.3), pad + [0.0, 0.003, -PRESS], FINGER_CLOSED,
               "hold it down while the other hand pulls the piece out", G_PAD),
            _w(1.0, over, FINGER_CLOSED, "let the lever back up", G_PAD)]
    out += _go_home("right", over, G_PAD, FINGER_CLOSED)
    return out, pressed_at


def _left(start: float, total: float) -> tuple[list[Waypoint], float]:
    take = np.array([EX, GRIP_Y, PATH_Z])
    out = _lead_in("left", take + [0.0, 0.0, 0.05], G_ACROSS, "over the piece sticking out of the inlet", FIL_FINGER)
    out.append(_w(max(start - LEAD_S, 0.3), take + [0.0, 0.0, 0.05], FIL_FINGER, "wait for the lever to be held down",
                  G_ACROSS))
    out += [_w(1.0, take, FIL_FINGER, "down round the piece", G_ACROSS),
            _w(0.6, take, FINGER_CLOSED, "take it", G_ACROSS),
            _w(3.0, take + [0.0, PULL, 0.0], FINGER_CLOSED, "pull it straight out of the extruder", G_ACROSS)]
    out_at = sum(w.duration for w in out)
    drop_y = (TY0 + TY1) / 2 + (GRIP_Y - FIL_MID0[1])  # the piece centred in the tray
    drop = np.array([(TX0 + TX1) / 2, drop_y, TOP + TRAY_T + FIL_R + 0.008])
    out += [_w(0.8, take + [0.0, PULL, 0.05], FINGER_CLOSED, "lift it", G_ACROSS),
            _w(1.5, drop + [0.0, 0.0, 0.05], FINGER_CLOSED, "carry it over the scrap tray", G_ACROSS),
            _w(1.0, drop, FINGER_CLOSED, "lay it in the tray", G_ACROSS),
            _w(0.5, drop, FIL_FINGER, "let go", G_ACROSS),
            _w(0.8, drop + [0.0, 0.0, 0.05], FIL_FINGER, "rise", G_ACROSS)]
    out += _go_home("left", drop + [0.0, 0.0, 0.05], G_ACROSS, FIL_FINGER)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FIL_FINGER,
                  "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out, out_at


def demo():
    _, pressed_at = _right(0.0)
    _, out_at = _left(pressed_at + 0.3, 0.0)
    right, _ = _right(out_at + 0.4)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_CLOSED, "wait at the start", right[-1].yaw))
    left, _ = _left(pressed_at + 0.3, total)
    return {"left": left, "right": right}
