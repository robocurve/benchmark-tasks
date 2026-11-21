"""On the station table: an 8-inch cast-iron tortilla press (like the Victoria 8-inch press: two round 20 cm plates,
the top plate on a hinge at one side, a pressing lever pivoted on the opposite side), the press open, its plates lined
with two squares of plastic cut from a freezer bag, and a 35 g ball of corn masa on a small plate beside it. Task
(instruction): press the masa ball into a corn tortilla at least 14 cm across, then open the press.

How it is done (Victoria's use notes; any tortilla recipe): put the ball of masa in the middle of the bottom plate,
between the plastic sheets; lower the top plate onto it; swing the lever over onto the top plate and press it down
firmly; lift the lever, open the press and peel off the tortilla. A ball put down off-centre comes out lopsided and
runs past the plates' edge; a light press leaves it small and thick.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- press: fixed (3.9 kg cast iron on rubber feet). Bottom plate 20 cm across, 1.2 cm thick, centred at (0.22, 0.00),
  its top 3.0 cm above the table. Top plate 20 cm across, 1.2 cm thick, 1.2 kg, hinged along x at y +0.115, 3.6 cm
  above the table; on its top a round pressing boss 2.4 cm across and 4.0 cm long (along x) at its centre. It starts
  open, swung up 100 deg and resting on its hinge stop (leaning 10 deg past upright).
- lever: 0.45 kg, pivoted along x at y -0.118, 7.4 cm above the table, 2.2 cm wide (x) and 1.4 cm deep; 28.8 cm from
  the pivot to its end (y +0.17 when it lies over the plates). It starts swung up 95 deg, resting on its stop. Lying
  on the top plate it bears on the boss 11.8 cm from its pivot: 2.4 : 1 leverage at its end.
- masa ball: 4.0 cm across, 35 g, a flattened spot where it rests; on a small plate 11 cm across at (0.12, -0.21).
The masa (a rule; simulators cannot model dough): once the top plate comes down onto the ball between the plates, the
ball becomes a disc of the same volume (33.5 cm3) under the plate, centred where the ball lay. Its thickness h only
goes down: the dough's resistance to the plates is a stiff spring (30 N/mm at the ball's centre) against h, and the
masa yields (h gets thinner) while that force exceeds F(h) = F14 * (2.18 mm / h), F14 = 60 N, the force that presses
it to 2.18 mm, 14 cm across. The top plate's own weight (and the lever's) takes it to about 7.5 cm across; a hand on
the top plate (no lever) to about 9 cm (26 N); the lever pressed down at its end with about 20 N reaches 14 cm (the
demo: 15 cm, 69 N).

Score (the best stage reached; d = the tortilla's diameter):
- 0.25: the masa has been pressed between the plates (the top plate came down onto the ball on the bottom plate).
- 0.5: d >= 10 cm.
- 0.75: d >= 14 cm.
- 1.0 (success, held 1 s): d >= 14 cm and the press open again (the lever and the top plate both swung back 90 deg
  or more, past upright, onto their stops), so the tortilla can be taken off.
0 for the whole run once the tortilla runs past the plates' edge (its centre's distance from the plates' centre plus
its radius over 10 cm: a ball put down off-centre).
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, Link, Linkage, SphereShape, allow_matrix_grips, as_numpy  # noqa: E402
from bodykit import on_reset, on_step, rotation  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
_E = os.environ.get

# -- the layout (world, m; heights above the table) ------------------------------------------------------------------
C = np.array([0.22, 0.0])  # the plates' centre
R_PLATE = 0.100  # 8-inch (20 cm) plates
PLATE_T = 0.012
Z_B = 0.030  # the bottom plate's top
HY, HZ = 0.115, 0.036  # the top plate's hinge axis (along x): y, height
BOSS_R, BOSS_L = 0.012, 0.040
BOSS_Z = Z_B + PLATE_T + BOSS_R  # the boss's axis when closed (its top 0.066)
PY, PZ = -0.118, 0.074  # the lever's pivot axis (along x): y, height
LEV_W, LEV_T, LEV_L = 0.022, 0.014, 0.288  # the lever's width (x), depth (z) and length from the pivot
TOP_OPEN, LEV_OPEN = -100.0, 95.0  # start angles (deg) of the top plate (about +x; opening is negative) and the lever
CLOSE_STOP = 3.0  # deg: the top plate's joint stop, past touching the bottom plate (the dough's spring takes the load;
# the plates do not collide with each other: they are one articulation)
R_BALL = 0.020
BALL_M = 0.035
V_MASA = 4.0 / 3.0 * math.pi * R_BALL ** 3  # 33.5 cm3
SAUCER = np.array([0.12, -0.21])
SAUCER_R, SAUCER_T = 0.055, 0.012
# the masa rule
F14 = float(_E("TORT_F14", "60.0"))  # N on the dough at 14 cm across
H14 = 4.0 * V_MASA / (math.pi * 0.14 ** 2)  # 2.18 mm
K_LIN = float(_E("TORT_K", "30000.0"))  # N/m: the dough's stiffness against the plates, at the dough's centre
D_PLATE = float(_E("TORT_DPLATE", "0.8"))  # N m s/rad on the top plate's hinge
D_FULL, D_PART = 0.14, 0.10
OPEN_DEG = 90.0  # the lever and the top plate back past upright, on their stops
HIDE = np.array([0.22, 0.0, -0.30])  # under the table

IRON, IRON_HI, MASA, SHEET, CERAMIC = ((0.11, 0.11, 0.12), (0.22, 0.22, 0.24), (0.93, 0.84, 0.55),
                                       (0.88, 0.90, 0.92), (0.95, 0.95, 0.93))


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _press() -> Linkage:
    o = np.array([C[0], C[1], TOP])
    base = [CylShape((0.0, 0.0, Z_B - PLATE_T / 2), R_PLATE, PLATE_T, IRON),
            CylShape((0.0, 0.0, Z_B - PLATE_T - 0.003), R_PLATE - 0.012, 0.006, IRON),  # ribs under the plate
            BoxShape((0.0, 0.0, Z_B + 0.00015), (0.17, 0.17, 0.0003), SHEET, collide=False)]  # plastic sheet
    for a in (90.0, 210.0, 330.0):  # three feet
        t = math.radians(a)
        base.append(CylShape((0.075 * math.cos(t), 0.075 * math.sin(t), (Z_B - PLATE_T - 0.006) / 2), 0.012,
                             Z_B - PLATE_T - 0.006, IRON))
    for sx in (-0.055, 0.055):  # the hinge's lugs on the base
        base.append(BoxShape((sx, HY - C[1], HZ - 0.008), (0.016, 0.022, 0.022), IRON))
    top = [CylShape((0.0, 0.0, Z_B + PLATE_T / 2), R_PLATE, PLATE_T, IRON),
           CylShape((0.0, 0.0, Z_B + PLATE_T + 0.002), R_PLATE - 0.012, 0.004, IRON_HI, collide=False),
           CylShape((0.0, 0.0, BOSS_Z), BOSS_R, BOSS_L, IRON_HI, axis="X"),
           BoxShape((0.0, 0.0, Z_B - 0.00015), (0.17, 0.17, 0.0003), SHEET, collide=False),
           BoxShape((0.0, HY - C[1] - 0.008, HZ), (0.090, 0.018, 0.014), IRON)]  # the hinge's knuckle
    links = [Link("top", tuple(top), mass=1.2, anchor=_v([C[0], HY, TOP + HZ]), axis="X",
                  limits=(TOP_OPEN, CLOSE_STOP), init=TOP_OPEN, stiffness=1.0e-4, spring_damping=D_PLATE)]
    return Linkage.build("press", pos=_v(o), mass=2.3, color=IRON, floating=False, friction=(0.6, 0.5),
                         shapes=base, links=links, contact_offset=0.002, solver_iterations=48)


def _lever() -> Linkage:
    o = np.array([C[0], PY, TOP])
    base = [BoxShape((0.0, 0.0, (PZ + 0.010) / 2), (0.050, 0.016, PZ + 0.010), IRON),  # the pivot's bracket
            BoxShape((0.0, -0.004, 0.006), (0.060, 0.040, 0.012), IRON)]
    lev = [BoxShape((0.0, LEV_L / 2 - 0.010, PZ), (LEV_W, LEV_L - 0.020, LEV_T), IRON_HI),
           CylShape((0.0, LEV_L - 0.030, PZ), 0.011, 0.024, IRON_HI, axis="X"),  # the rounded end
           CylShape((0.0, 0.0, PZ), 0.010, LEV_W + 0.012, IRON, axis="X", collide=False)]  # the pivot pin
    links = [Link("arm", tuple(lev), mass=0.45, anchor=_v([C[0], PY, TOP + PZ]), axis="X", limits=(-5.0, LEV_OPEN),
                  init=LEV_OPEN, stiffness=1.0e-4, spring_damping=0.15)]
    return Linkage.build("lever", pos=_v(o), mass=1.0, color=IRON, floating=False, friction=(0.6, 0.5),
                         shapes=base, links=links, contact_offset=0.002, solver_iterations=48)


def _masa() -> Body:
    sh = [SphereShape((0.0, 0.0, R_BALL), R_BALL, MASA),
          CylShape((0.0, 0.0, 0.0015), 0.012, 0.003, MASA)]  # the flattened spot it rests on
    return Body.build("masa", pos=(float(SAUCER[0]), float(SAUCER[1]), TOP + SAUCER_T + 0.001), mass=BALL_M,
                      color=MASA, friction=(1.0, 0.9), damping=(0.5, 2.0), shapes=sh)


def _saucer() -> Body:
    sh = [CylShape((0.0, 0.0, SAUCER_T / 2), SAUCER_R, SAUCER_T, CERAMIC)]
    return Body.build("saucer", pos=(float(SAUCER[0]), float(SAUCER[1]), TOP + 0.001), mass=0.30, color=CERAMIC,
                      friction=(0.8, 0.7), shapes=sh)


SLOPE = 0.0016 / 0.070  # the drawn tortilla's thickness above the plate per metre of radius (1.6 mm at 7 cm)


def _tortilla() -> Body:
    """A flat cone, apex up, drawn only: set so the bottom plate cuts it at the tortilla's radius, it shows as a thin
    disc of that radius on the plate (the rest hides inside the plate)."""
    sh = []
    for i in range(45):  # radii 1.0 to 9.8 cm: none reach past the plate's edge
        r = 0.010 + 0.002 * i
        sh.append(CylShape((0.0, 0.0, -SLOPE * r - 0.0002), r, 0.0004, MASA, collide=False))
    return Body.build("tortilla", pos=_v(HIDE), mass=0.02, color=MASA, gravity=False, damping=(5.0, 5.0), shapes=sh)


PRESS = _press()
LEVER = _lever()
MASA_BALL = _masa()
SAUCER_BODY = _saucer()
TORTILLA = _tortilla()
OBJECTS = [PRESS, LEVER, SAUCER_BODY, MASA_BALL, TORTILLA]

# -- tracking and scoring --------------------------------------------------------------------------------------------
_DEBUG = _E("TORT_DEBUG") == "1"
_DEBUG_EVERY = int(_E("TORT_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}


@on_reset
def _reset() -> None:
    _EP.update(tick=0, step=0, formed=False, c=np.array(C), h=None, d=0.0, force=0.0, fmax=0.0, top=TOP_OPEN,
               lev=LEV_OPEN, ball=np.zeros(3), overrun=False, k_on=None, tcp={}, gaps={})


_reset()


def gap_at(theta: float, y: float) -> float:
    """The gap (m) between the plates at world y, with the top plate at ``theta`` (rad; 0 closed, opening < 0)."""
    ly, dz = y - HY, Z_B - HZ
    return HZ + ly * math.sin(theta) + dz * math.cos(theta) - Z_B


def theta_for_gap(g: float, y: float) -> float:
    lo, hi = -math.pi / 2, math.radians(CLOSE_STOP)  # gap_at falls as theta rises
    for _ in range(40):
        mid = (lo + hi) / 2
        if gap_at(mid, y) > g:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def diameter(h: float) -> float:
    return math.sqrt(4.0 * V_MASA / (math.pi * h))


def _place(base, name: str, pos) -> None:
    import torch

    origin = as_numpy(base.scene.env_origins[0])
    pose = np.array([[pos[0] + origin[0], pos[1] + origin[1], pos[2] + origin[2], 0.0, 0.0, 0.0, 1.0]])
    obj = base.scene[name]
    obj.write_root_link_pose_to_sim_index(root_pose=torch.tensor(pose, dtype=torch.float32, device=base.device))
    obj.write_root_com_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=torch.float32,
                                                                       device=base.device))


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("TORT_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _set_drive(press, k_ang: float, target: float) -> None:
    import torch

    j = _IDS["top"]
    if _EP["k_on"] != k_ang:
        press.write_joint_stiffness_to_sim_index(
            stiffness=torch.tensor([[k_ang]], dtype=torch.float32, device=press.device), joint_ids=[j])
        _EP["k_on"] = k_ang
    press.set_joint_position_target_index(target=torch.tensor([[target]], dtype=torch.float32, device=press.device),
                                          joint_ids=[j])
    press.write_data_to_sim()


def _track_inner(env) -> None:
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    origin = as_numpy(base.scene.env_origins[0])
    if _DEBUG:
        for side in ("left", "right"):
            robot = base.scene[f"{side}_robot"]
            if side not in _IDS:
                _IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
                _IDS[side + "_f"] = list(robot.find_joints(FINGER_JOINTS, preserve_order=True)[0])
            q = as_numpy(robot.data.joint_pos)[0]
            _EP["tcp"][side] = np.asarray(ik.tcp_pose(q[_IDS[side]], station.BASES[side])[0], float)
            _EP["gaps"][side] = float(-q[_IDS[side + "_f"]].sum())
    press, lever = base.scene["press"], base.scene["lever"]
    if "top" not in _IDS:
        _IDS["top"] = list(press.joint_names).index("top_turn")
        _IDS["arm"] = list(lever.joint_names).index("arm_turn")
    th = float(as_numpy(press.data.joint_pos)[0][_IDS["top"]])
    lv = float(as_numpy(lever.data.joint_pos)[0][_IDS["arm"]])
    _EP.update(top=math.degrees(th), lev=math.degrees(lv))
    if not _EP["formed"]:
        if _EP["k_on"] is None:
            _set_drive(press, 0.0, th)
        bp = as_numpy(base.scene["masa"].data.root_link_pose_w)[0][:3] - origin
        _EP["ball"] = bp
        on_plate = (float(np.hypot(*(bp[:2] - C))) <= R_PLATE and abs(bp[2] - (TOP + Z_B)) < 0.006)
        if on_plate and gap_at(th, float(bp[1])) <= 2 * R_BALL + 0.003:
            _EP.update(formed=True, c=np.array(bp[:2]), h=gap_at(th, float(bp[1])))
    if _EP["formed"]:
        _place(base, "masa", HIDE + [0.1, 0.0, 0.0])
        cy = float(_EP["c"][1])
        ly = abs(cy - HY)
        g = gap_at(th, cy)
        h = _EP["h"]
        force = K_LIN * max(0.0, h - g)
        if force > F14 * H14 / h:  # the masa yields: thinner until its resistance meets the force
            h = min(h, (g + math.sqrt(g * g + 4.0 * F14 * H14 / K_LIN)) / 2.0)
            force = K_LIN * max(0.0, h - g)
        _EP.update(h=h, d=diameter(h), force=force, fmax=max(_EP["fmax"], force))
        if g < h:  # the plate on the dough: its spring pushes the plate back to the dough's thickness (one-sided)
            _set_drive(press, K_LIN * ly * ly, theta_for_gap(h, cy))
        else:
            _set_drive(press, 0.0, th)
        r = _EP["d"] / 2
        if float(np.hypot(*(_EP["c"] - C))) + r > R_PLATE:
            _EP["overrun"] = True
        _place(base, "tortilla", (float(_EP["c"][0]), cy, TOP + Z_B + 0.0003 + SLOPE * r))


def _tally() -> dict:
    opened = _EP["lev"] >= OPEN_DEG and _EP["top"] <= -OPEN_DEG
    done = bool(_EP["formed"] and not _EP["overrun"] and _EP["d"] >= D_FULL and opened)
    return dict(opened=opened, done=done)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    b = _EP["ball"]
    print(f"DBG step={_EP['step']} top={_EP['top']:+.1f} lev={_EP['lev']:+.1f} formed={int(_EP['formed'])} "
          f"h={(_EP['h'] or 0) * 1000:.2f}mm d={_EP['d'] * 100:.1f}cm F={_EP['force']:.1f}N Fmax={_EP['fmax']:.1f}N "
          f"c=({_EP['c'][0]:.3f},{_EP['c'][1]:.3f}) over={int(_EP['overrun'])} open={int(t['opened'])} "
          f"done={int(t['done'])} ball=({b[0]:.3f},{b[1]:.3f},{(b[2] - TOP) * 1000:.1f}mm) | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{(v[2] - TOP) * 1000:.1f}mm) gap={_EP['gaps'][k] * 1000:.0f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    t = _tally()
    if _DEBUG:
        _debug(t)
    if _EP["overrun"] or not _EP["formed"]:
        return 0.0
    best = 0.25
    if _EP["d"] >= D_PART:
        best = 0.5
    if _EP["d"] >= D_FULL:
        best = 0.75
        if t["done"]:
            best = 1.0
    return best


def success(objects) -> bool:
    return bool(_tally()["done"])


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Open loop, from the known layout. The right gripper takes the masa ball off its plate and sets it down in the middle
# of the bottom plate. The left gripper pinches the top plate's boss by its ends and lowers the plate onto the ball.
# The right gripper takes the lever near its end, swings it over and lays it on the boss. The left gripper, closed,
# pushes the lever's end down (aimed below its lowest point: the arm gives under load) and holds it. Then the right
# gripper lifts the lever back to its stop and the left lifts the top plate back to its stop.
# TORT_OFFCENTRE=1 (the wrong way): the ball is put down 5 cm off-centre (toward the arms); pressed, the tortilla runs
# past the plates' edge.
# TORT_NOLEVER=1 (a second wrong way): the left gripper pushes the top plate's boss down by hand instead of the lever.
OPEN_C = -0.047
OFFCENTRE = _E("TORT_OFFCENTRE") == "1"
NOLEVER = _E("TORT_NOLEVER") == "1"
P = {k: float(_E("TORT_" + k.upper(), v)) for k, v in dict(
    ball_grip="-0.016", boss_grip="-0.012", bar_grip="-0.006", r_hold="0.17", press_over="0.035",
    press_s="3.0", hold_s="2.0", tip_r="0.265", off="0.05").items()}


def _R(c1, c3) -> np.ndarray:
    c1, c3 = np.asarray(c1, float), np.asarray(c3, float)
    c1, c3 = c1 / np.linalg.norm(c1), c3 / np.linalg.norm(c3)
    return np.column_stack([c1, np.cross(c3, c1), c3])


def _lean(sgn: float, deg: float) -> np.ndarray:
    """Pointing down, fingers closing along ``sgn`` * x, the approach leaning ``deg`` toward +y."""
    b = math.radians(deg)
    return _R((sgn, 0.0, 0.0), (0.0, math.sin(b), -math.cos(b)))


DOWN_L = _lean(-1.0, 0.0)  # left arm, straight down, fingers along x
DOWN_R = _lean(1.0, 0.0)  # right arm
DOWN_RY = _R((0.0, -1.0, 0.0), (0.0, 0.0, -1.0))  # right arm, straight down, fingers along y


def _good_ik(arm: str, pos, rot):
    from yam_arena import ik, station

    best = None
    b = station.BASES[arm]
    j1 = math.atan2(float(pos[1]) - b[1], float(pos[0]) - b[0])
    for sd in itertools.product((j1,), (0.8, 1.3), (0.6, 1.2), (-0.8, 0.8), (0.0,), (-1.8, -0.9, 0.0, 0.9, 1.8)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(sd, float),
                             base_pos=station.BASES[arm], iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
                if room > 0.3:
                    break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} arm at {pos}")
    return best[0]


def _joint_path(arm: str, q0, q1, finger: float, label: str, seconds: float, steps: int = 15) -> list:
    from yam_arena import ik, station

    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0) -> list:
    from yam_arena.constants import READY

    return _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), finger, label, seconds)


def _lead_out(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0) -> list:
    from yam_arena.constants import READY

    return _joint_path(arm, _good_ik(arm, pos, rot), np.array(READY, float), finger, label, seconds)


def _w(t, p, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in p), finger, label, rot)


def _ready(arm: str):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def _dur(wps) -> float:
    return float(sum(w.duration for w in wps))


def _wait(wps, t, label="wait") -> None:
    if t > 1e-6:
        last = wps[-1]
        wps.append(Waypoint(t, last.pos, last.finger, label, last.yaw))


def _sync(L, R) -> None:
    dl, dr = _dur(L), _dur(R)
    _wait(L, dr - dl)
    _wait(R, dl - dr)


def _boss_point(th_deg: float) -> np.ndarray:
    """The left TCP pinching the boss by its ends (4 mm above its axis: the pads clear the plate), plate at th_deg."""
    a = math.radians(th_deg)
    y0, z0 = -(HY - C[1]), BOSS_Z - HZ
    y1, z1 = y0 * math.cos(a) - z0 * math.sin(a), y0 * math.sin(a) + z0 * math.cos(a)
    return np.array([C[0], HY + y1, TOP + HZ + z1 + 0.004])


def _boss_rot(th_deg: float) -> np.ndarray:
    return _lean(-1.0, max(th_deg, -60.0))


def _bar_point(lv_deg: float, r: float) -> np.ndarray:
    """The right TCP pinching the lever's bar ``r`` from the pivot, at lever angle ``lv_deg``."""
    a = math.radians(lv_deg)
    u = np.array([0.0, math.cos(a), math.sin(a)])
    n = np.array([0.0, -math.sin(a), math.cos(a)])
    return np.array([C[0], PY, TOP + PZ]) + r * u + n * 0.004


BAR_TILT_MAX = 75.0


def _bar_rot(lv_deg: float) -> np.ndarray:
    return _lean(1.0, min(lv_deg, BAR_TILT_MAX))  # the pinch along x lets the bar turn in it past this


def _timeline():
    L, R = [], []
    ball = np.array([SAUCER[0], SAUCER[1], TOP + SAUCER_T + 0.001 + R_BALL + 0.004])
    put = np.array([C[0] - (P["off"] if OFFCENTRE else 0.0), C[1], TOP + Z_B + R_BALL + 0.006])
    # 1. the right gripper (its fingers closing along y, so its body stays narrow along x, clear of the raised lever)
    # takes the ball and sets it down in the middle of the bottom plate
    R += _lead_in("right", ball + [0, 0, 0.08], DOWN_RY, OPEN_C, "above the masa ball")
    R += [_w(1.5, ball, OPEN_C, "fingers round the ball", DOWN_RY),
          _w(1.0, ball, P["ball_grip"], "close on the ball", DOWN_RY),
          _w(0.3, ball, P["ball_grip"], "settle", DOWN_RY),
          _w(1.2, ball + [0, 0, 0.08], P["ball_grip"], "lift", DOWN_RY),
          _w(1.5, np.array([ball[0], put[1], put[2] + 0.06]), P["ball_grip"], "carry, in front of the press",
             DOWN_RY),
          _w(1.5, put + [0, 0, 0.06], P["ball_grip"], "over the middle of the plate", DOWN_RY),
          _w(1.2, put, P["ball_grip"], "set it down", DOWN_RY),
          _w(0.8, put, OPEN_C, "let go", DOWN_RY),
          _w(1.0, put + [0, 0, 0.08], OPEN_C, "rise", DOWN_RY),
          _w(1.5, np.array([ball[0], put[1] - 0.04, put[2] + 0.12]), OPEN_C, "back out", DOWN_RY)]
    R += _lead_out("right", np.array([ball[0], put[1] - 0.04, put[2] + 0.12]), DOWN_RY, OPEN_C, "to the ready pose")
    lb0 = _boss_point(TOP_OPEN)
    L += _lead_in("left", lb0 + [0.0, 0.03, 0.05], _boss_rot(TOP_OPEN), OPEN_C, "beside the top plate's boss")
    _sync(L, R)
    # 2. the left pinches the boss and lowers the top plate onto the ball
    L += [_w(1.5, lb0, OPEN_C, "fingers either end of the boss", _boss_rot(TOP_OPEN)),
          _w(1.0, lb0, P["boss_grip"], "pinch the boss", _boss_rot(TOP_OPEN)),
          _w(0.3, lb0, P["boss_grip"], "settle", _boss_rot(TOP_OPEN))]
    th_land = math.degrees(theta_for_gap(2 * R_BALL, C[1])) - 1.0
    n = 16
    for k in range(1, n + 1):
        th = TOP_OPEN + (th_land - TOP_OPEN) * k / n
        L.append(_w(4.0 / n, _boss_point(th), P["boss_grip"], "lower the top plate", _boss_rot(th)))
    L += [_w(0.8, _boss_point(th_land), OPEN_C, "let go", _boss_rot(th_land)),
          _w(1.0, _boss_point(th_land) + [0, 0, 0.06], OPEN_C, "rise", _boss_rot(th_land))]
    tip = np.array([C[0], PY + P["tip_r"], TOP + PZ + LEV_T / 2])  # the lever's end lying level
    if NOLEVER:  # push the top plate down by hand, on its boss
        bt = _boss_point(0.0) + [0, 0, 0.003]
        L += [_w(1.0, bt + [0, 0, 0.03], 0.0, "close the fingers over the boss", DOWN_L),
              _w(P["press_s"], bt - [0, 0, P["press_over"]], 0.0, "press the top plate down", DOWN_L),
              _w(P["hold_s"], bt - [0, 0, P["press_over"]], 0.0, "hold", DOWN_L),
              _w(1.0, bt + [0, 0, 0.06], 0.0, "rise", DOWN_L)]
        _sync(L, R)
    else:
        L += [_w(1.5, tip + [0, 0, 0.12], 0.0, "over the lever's end, fingers closed", DOWN_L)]
        _sync(L, R)
        # 3. the right takes the lever and lays it over onto the boss
        R += [_w(1.5, _bar_point(LEV_OPEN - 5, P["r_hold"]) + [0, -0.05, 0.0], OPEN_C, "beside the lever",
                 _bar_rot(LEV_OPEN - 5)),
              _w(1.2, _bar_point(LEV_OPEN, P["r_hold"]), OPEN_C, "fingers either side of the bar", _bar_rot(LEV_OPEN)),
              _w(1.0, _bar_point(LEV_OPEN, P["r_hold"]), P["bar_grip"], "pinch the bar", _bar_rot(LEV_OPEN)),
              _w(0.3, _bar_point(LEV_OPEN, P["r_hold"]), P["bar_grip"], "settle", _bar_rot(LEV_OPEN))]
        lv_land = 20.0
        for k in range(1, n + 1):
            lv = LEV_OPEN + (lv_land - LEV_OPEN) * k / n
            R.append(_w(4.0 / n, _bar_point(lv, P["r_hold"]), P["bar_grip"], "swing the lever over", _bar_rot(lv)))
        R += [_w(0.8, _bar_point(lv_land, P["r_hold"]), OPEN_C, "let go", _bar_rot(lv_land)),
              _w(1.2, _bar_point(lv_land, P["r_hold"]) + [0, -0.03, 0.06], OPEN_C, "rise", _bar_rot(lv_land))]
        _sync(L, R)
        # 4. the left pushes the lever's end down and holds it
        L += [_w(1.5, tip + [0, 0, 0.012 + 0.10], 0.0, "onto the lever's end", DOWN_L),
              _w(P["press_s"], tip + [0, 0, 0.012 - P["press_over"]], 0.0, "press the lever down", DOWN_L),
              _w(P["hold_s"], tip + [0, 0, 0.012 - P["press_over"]], 0.0, "hold", DOWN_L),
              _w(1.2, tip + [0, 0, 0.012 + 0.08], 0.0, "rise", DOWN_L),
              _w(1.5, np.array([0.16, 0.26, TOP + 0.20]), 0.0, "out of the right arm's way", DOWN_L)]
        _sync(L, R)
        # 5. the right lifts the lever back to its stop
        R += [_w(1.5, _bar_point(0.0, P["r_hold"]) + [0, 0, 0.04], OPEN_C, "over the bar", _bar_rot(0.0)),
              _w(1.0, _bar_point(0.0, P["r_hold"]), OPEN_C, "fingers either side of the bar", _bar_rot(0.0)),
              _w(1.0, _bar_point(0.0, P["r_hold"]), P["bar_grip"], "pinch the bar", _bar_rot(0.0)),
              _w(0.3, _bar_point(0.0, P["r_hold"]), P["bar_grip"], "settle", _bar_rot(0.0))]
        for k in range(1, n + 1):
            lv = (LEV_OPEN - 3.0) * k / n
            R.append(_w(4.0 / n, _bar_point(lv, P["r_hold"]), P["bar_grip"], "lift the lever back", _bar_rot(lv)))
        R += [_w(0.8, _bar_point(LEV_OPEN - 3.0, P["r_hold"]), OPEN_C, "let go", _bar_rot(LEV_OPEN - 3.0)),
              _w(1.2, _bar_point(LEV_OPEN - 3.0, P["r_hold"]) + [0, -0.03, -0.02], OPEN_C, "back off",
                 _bar_rot(LEV_OPEN - 3.0))]
        _sync(L, R)
    # 6. the left lifts the top plate back to its stop
    lb = _boss_point(0.0)
    L += [_w(1.5, lb + [0, 0, 0.05], OPEN_C, "over the boss", _boss_rot(0.0)),
          _w(1.0, lb, OPEN_C, "fingers either end of the boss", _boss_rot(0.0)),
          _w(1.0, lb, P["boss_grip"], "pinch the boss", _boss_rot(0.0)),
          _w(0.3, lb, P["boss_grip"], "settle", _boss_rot(0.0))]
    for k in range(1, n + 1):
        th = TOP_OPEN * k / n
        L.append(_w(4.0 / n, _boss_point(th), P["boss_grip"], "open the press", _boss_rot(th)))
    L += [_w(0.8, _boss_point(TOP_OPEN), OPEN_C, "let go", _boss_rot(TOP_OPEN)),
          _w(1.2, _boss_point(TOP_OPEN) + [0, 0.04, 0.04], OPEN_C, "back off", _boss_rot(TOP_OPEN))]
    L += _lead_out("left", _boss_point(TOP_OPEN) + [0, 0.04, 0.04], _boss_rot(TOP_OPEN), OPEN_C, "back to ready")
    R += _lead_out("right", np.asarray(R[-1].pos), R[-1].yaw, OPEN_C, "back to ready")
    return L, R


def demo():
    left, right = _timeline()
    lp, lq = _ready("left")
    rp, rq = _ready("right")
    end = max(_dur(left), _dur(right)) + 2.0
    left += [_w(end - _dur(left), lp, OPEN_C, "stay clear", lq)]
    right += [_w(end - _dur(right), rp, OPEN_C, "stay clear", rq)]
    return {"left": left, "right": right}
