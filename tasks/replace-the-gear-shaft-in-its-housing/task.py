"""On the station table, in front of the left arm: a closed grey gear housing with a bore through its front and
rear walls, running away from the robot. Inside the housing a brass gear hangs on a dark steel shaft that is flush
with both faces of the housing; a narrow slit in the housing roof shows the gear. Behind the housing a grooved rail
continues the bore line to a tall grey bulkhead. In front of the housing a grooved steel trough runs in line with
the bore. To its left, on a grooved rest, lies a new orange shaft, longer than the old one; a slim steel drift lies
on a second rest to the right.
"""

import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6
SUCCESS_HOLD_S = 3.0  # the end state must hold for 3 s (the arm is clear of the housing by then)

# World frame: x away from the arm bases, y to the robot's left, z up (m).
HX0, HL = 0.35, 0.102  # housing front face, length along the bore
HX1 = HX0 + HL  # rear face
HY = 0.10  # bore line
WALL = 0.020  # front and rear wall thickness
FZ = TOP + 0.010  # cavity floor (top of the housing's base)
AZ = FZ + 0.0325  # shaft axis when a shaft rests on the bore bottom
R = 0.007  # both shafts: 14 mm
BORE_HW = 0.0095  # bore 19 mm wide
BORE_LO, BORE_HI = AZ - R, AZ + R + 0.0003  # front bore bottom and top (14.3 mm tall)
REAR_DROP = 0.001  # the rear bore's bottom, and the rail behind it, sit 1 mm lower (rear bore 15.3 mm tall)
REAR_HW = 0.0105  # rear bore 21 mm wide
CAV_HW = 0.028  # cavity 56 mm wide
SIDE = 0.010
H_HW = CAV_HW + SIDE  # housing 76 mm wide
CAV_TOP = FZ + 0.060
H_TOP = CAV_TOP + 0.010  # housing 80 mm tall
SLIT_HW = 0.004  # 8 mm viewing slit in the roof, over the cavity

GREY, DARK, ORANGE, BRASS, STEEL, PALE = (0.45, 0.47, 0.50), (0.14, 0.14, 0.15), (0.95, 0.50, 0.10), \
    (0.80, 0.62, 0.25), (0.72, 0.73, 0.75), (0.90, 0.90, 0.86)


def _b(x0, x1, y0, y1, z0, z1, color=GREY):
    """A box from world extents (m)."""
    return parts.box((x1 - x0, y1 - y0, z1 - z0), at=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), color=color)


def _wall(x0, x1, lo=BORE_LO, hw=BORE_HW):
    """A wall across the housing with the bore through it."""
    return [_b(x0, x1, -CAV_HW, CAV_HW, FZ, lo), _b(x0, x1, -CAV_HW, CAV_HW, BORE_HI, CAV_TOP),
            _b(x0, x1, hw, CAV_HW, lo, BORE_HI), _b(x0, x1, -CAV_HW, -hw, lo, BORE_HI)]


# The housing as a fixed part whose origin is the world origin at y = HY (shapes in world x and z, y about the bore).
_HOUSING_SHAPES = [
    _b(HX0, HX1, -H_HW, H_HW, TOP, FZ),  # base
    _b(HX0, HX1, CAV_HW, H_HW, FZ, CAV_TOP), _b(HX0, HX1, -H_HW, -CAV_HW, FZ, CAV_TOP),  # side walls
    *_wall(HX0, HX0 + WALL), *_wall(HX1 - WALL, HX1, BORE_LO - REAR_DROP, REAR_HW),
    _b(HX0, HX1, SLIT_HW, H_HW, CAV_TOP, H_TOP), _b(HX0, HX1, -H_HW, -SLIT_HW, CAV_TOP, H_TOP),  # roof either side
    _b(HX0, HX0 + WALL, -SLIT_HW, SLIT_HW, CAV_TOP, H_TOP), _b(HX1 - WALL, HX1, -SLIT_HW, SLIT_HW, CAV_TOP, H_TOP),
    # Shaft guide across the cavity, wall to wall, through the gear's bore: two strips 3 mm wide with a 4 mm gap,
    # their tops 0.3 mm below the front bore's bottom, whose edges carry a shaft on the bore line. It keeps a shaft
    # end from drooping between the walls; it is well below the gear's bore top, so it does not carry the gear.
    _b(HX0 + WALL, HX1 - WALL, 0.002, 0.005, AZ - 0.012, BORE_LO - 0.0003, STEEL),
    _b(HX0 + WALL, HX1 - WALL, -0.005, -0.002, AZ - 0.012, BORE_LO - 0.0003, STEEL),
]

# Grooved trough in front of the housing, in line with the bore: floor level with the bore bottom, a 14.5 mm wide,
# 4 mm deep groove that keeps a 14 mm shaft lying in it within 1 mm of the bore line. The rests and the rail behind
# the housing have the same groove, 2 mm deep.
TR_X0 = 0.155
GROOVE_HW, GROOVE_D, GROOVE_OUT = 0.00725, 0.002, 0.020
TROUGH_D = 0.004


def _groove(x0, x1, floor, y0=0.0, color=STEEL, depth=GROOVE_D):
    return [_b(x0, x1, y0 - GROOVE_OUT, y0 + GROOVE_OUT, TOP, floor, color),
            _b(x0, x1, y0 + GROOVE_HW, y0 + GROOVE_OUT, floor, floor + depth, color),
            _b(x0, x1, y0 - GROOVE_OUT, y0 - GROOVE_HW, floor, floor + depth, color)]


# Behind the housing: an outboard support, a grooved rail in line with the bore that the old shaft slides out onto,
# and a bulkhead 135 mm behind the rear face: room for the old 102 mm shaft to come out, none to feed a 170 mm shaft
# in from the back.
BH_X = HX1 + 0.135
_BACK = [*_groove(HX1, BH_X, BORE_LO - REAR_DROP, color=PALE), _b(BH_X, BH_X + 0.010, -0.07, 0.07, TOP, TOP + 0.12, GREY)]

HOUSING = parts.Part("housing", pos=(0.0, HY, 0.0), friction=(0.4, 0.3), shapes=[*_HOUSING_SHAPES, *_groove(TR_X0, HX0, BORE_LO, depth=TROUGH_D), *_BACK])

# Gear: a brass block 40 x 50 x 44 mm with a 25 mm wide bore. It hangs on the shaft by two 6 mm steel needle rollers
# across the top of its bore, one at each end, so it rides 10 mm above the cavity floor. Origin at the block's centre.
GL, GW, GH = 0.040, 0.050, 0.044
G_BOT = FZ + 0.010
GZ = G_BOT + GH / 2
GX = HX0 + HL / 2
GB_TOP, GB_BOT, GB_HW = AZ + R, AZ + R - 0.020, 0.0125  # bore: rollers' underside, bottom, half-width
ROLLER_R = 0.003
GEAR_MASS = 0.25


def _gb(y0, y1, z0, z1):
    return parts.box((GL, y1 - y0, z1 - z0), at=(0.0, (y0 + y1) / 2, (z0 + z1) / 2 - GZ), color=BRASS)


GEAR = parts.Part("gear", pos=(GX, HY, GZ), mass=GEAR_MASS, friction=(0.4, 0.3), shapes=[
    _gb(-GW / 2, GW / 2, GB_TOP + ROLLER_R, G_BOT + GH), _gb(-GW / 2, GW / 2, G_BOT, GB_BOT),
    _gb(GB_HW, GW / 2, GB_BOT, GB_TOP + ROLLER_R), _gb(-GW / 2, -GB_HW, GB_BOT, GB_TOP + ROLLER_R),
    *[parts.cylinder(ROLLER_R, 2 * GB_HW, at=(sx * (GL / 2 - ROLLER_R), 0.0, GB_TOP + ROLLER_R - GZ), color=STEEL,
                     axis="y") for sx in (-1, 1)]])

# Old shaft: 102 mm, flush with both faces. Origin at its centre.
OLD_L = HL
OLD = parts.Part("old_shaft", pos=(HX0 + OLD_L / 2, HY, AZ), mass=0.12, friction=(0.5, 0.4),
                 shapes=[parts.cylinder(R, OLD_L, color=DARK, axis="x")])

# New shaft: 170 mm, flat ends. Origin at its centre.
NEW_L = 0.170
NOSE_TIP, NEAR_END = NEW_L / 2, NEW_L / 2
REST_Y = 0.20
REST_X0 = 0.17  # near end of the new shaft on its rest
NEW_START = (REST_X0 + NEAR_END, REST_Y, AZ)
NEW = parts.Part("new_shaft", pos=NEW_START, mass=0.17, friction=(0.5, 0.4), shapes=[
    parts.cylinder(R, NEW_L, color=ORANGE, axis="x")])

# Drift: 10 mm x 150 mm steel rod on its own grooved rest.
DRIFT_R, DRIFT_L = 0.005, 0.150
DRIFT_Y = -0.05
DRIFT_START = (REST_X0 + DRIFT_L / 2, DRIFT_Y, BORE_LO + DRIFT_R)
DRIFT = parts.Part("drift", pos=DRIFT_START, mass=0.09, friction=(0.5, 0.4),
                   shapes=[parts.cylinder(DRIFT_R, DRIFT_L, color=STEEL, axis="x")])

RESTS = parts.Part("rests", pos=(0.0, 0.0, 0.0), shapes=[*_groove(REST_X0 + 0.02, REST_X0 + 0.16, BORE_LO, REST_Y),
                                                         *_groove(REST_X0 + 0.02, REST_X0 + 0.16, BORE_LO, DRIFT_Y)])

OBJECTS = [HOUSING, RESTS, GEAR, OLD, NEW, DRIFT]

# ---- scoring ---------------------------------------------------------------------------------------------------
REAR_TOL = 0.002  # the new shaft's lead end within 2 mm of the rear face, or past it
LINE_TOL = 0.004  # the new shaft's axis within 4 mm of the bore line (it lies in the bore)
TILT_MAX = np.radians(3.0)
GEAR_TILT = np.radians(8.0)
GEAR_DZ = 0.002  # gear centre within 2 mm of its starting height
SHAFT_SPIN = 2.0  # a shaft lying still may rock about its own axis (the simulated cylinder is faceted)
CLEAR = 0.005  # released: no finger or gripper surface within 5 mm of either shaft


def _rot(q):
    return parts._quat_to_matrix(q)


def _tilt_x(state) -> float:
    """Angle between the object's own x axis and the bore line (world x)."""
    return float(np.arccos(np.clip(abs(_rot(state.quat_xyzw)[0, 0]), -1.0, 1.0)))


def _ends(state, ahead, behind):
    r = _rot(state.quat_xyzw)[:, 0]
    p = np.asarray(state.pos, float)
    return p + ahead * r, p - behind * r


def new_through(objects) -> bool:
    """The orange shaft lies along the bore through both walls: its lead end within 2 mm of the rear face or past it,
    its near end at or in front of the front face."""
    s = objects["new_shaft"]
    tip, near = _ends(s, NOSE_TIP, NEAR_END)
    p = np.asarray(s.pos)
    return bool(
        tip[0] >= HX1 - REAR_TOL - EPS and near[0] <= HX0 + EPS
        and abs(p[1] - HY) <= LINE_TOL + EPS and abs(p[2] - AZ) <= LINE_TOL + EPS
        and _tilt_x(s) <= TILT_MAX + EPS
    )


def old_out(objects) -> bool:
    """No part of the dark shaft is inside the housing's outline."""
    o = objects["old_shaft"]
    lo, hi = np.asarray(o.low), np.asarray(o.high)
    inside = (lo[0] < HX1 - EPS and hi[0] > HX0 + EPS and lo[1] < HY + H_HW - EPS and hi[1] > HY - H_HW + EPS
              and lo[2] < H_TOP - EPS)
    return not bool(inside)


def _pitch(state) -> float:
    """How far the object's own x axis (front to back) dips or rises out of the horizontal."""
    return float(np.arcsin(np.clip(abs(_rot(state.quat_xyzw)[2, 0]), 0.0, 1.0)))


def gear_in_line(objects) -> bool:
    """The gear's centre within 2 mm of its starting height and its front-to-back axis within 8 degrees of level.
    Swinging sideways on the shaft (about the bore line) is allowed: the gear hangs from it."""
    g = objects["gear"]
    return bool(abs(float(g.pos[2]) - GZ) <= GEAR_DZ + EPS and _pitch(g) <= GEAR_TILT + EPS)


# Released: measured in the simulator after every control step, from the robot's finger and gripper collision hulls
# against points on both shafts' surfaces. Offline (near_misses.py, no simulator) it is taken as true.
_GRIP = {"sim": False, "clear": True}
_LINKS = ("gripper", "tip_left", "tip_right")


def _shaft_points(state, ahead, behind, r):
    p, rot = np.asarray(state.pos, float), _rot(state.quat_xyzw)
    xs = np.linspace(-behind, ahead, max(2, int((ahead + behind) / 0.005) + 1))
    a = np.linspace(0, 2 * np.pi, 16, endpoint=False)
    local = np.stack([np.repeat(xs, len(a)), np.tile(r * np.cos(a), len(xs)), np.tile(r * np.sin(a), len(xs))], 1)
    return p + local @ rot.T


def _robot_hulls(sim):
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        link_poses = {}
        for i, n in enumerate(asset.body_names):
            if n in _LINKS:
                t = np.eye(4)
                t[:3, :3], t[:3, 3] = _rot(poses[i, 3:]), poses[i, :3] - origin
                link_poses[n] = t
        out += analysis.world_hulls(link_poses)
    return out


def _clearance(hulls, pts) -> float:
    lo, hi = pts.min(0) - 0.02, pts.max(0) + 0.02
    best = np.inf
    for _, h in hulls:
        if np.any(h.vertices.max(0) < lo) or np.any(h.vertices.min(0) > hi):
            continue
        d = (pts @ h.planes[:, :3].T - h.planes[:, 3]).max(axis=1)  # a lower bound on each point's distance
        best = min(best, float(d.min()))
    return best


@parts.on_step
def _check_grip(sim):
    _GRIP["sim"] = True
    st = sim.states()
    hulls = _robot_hulls(sim)
    pts = np.concatenate([_shaft_points(st["new_shaft"], NOSE_TIP, NEAR_END, R),
                          _shaft_points(st["old_shaft"], OLD_L / 2, OLD_L / 2, R)])
    c = _clearance(hulls, pts)
    _GRIP["clear"] = c >= CLEAR - EPS


def released(objects) -> bool:
    return bool(_GRIP["clear"])


def success(objects):
    """Orange shaft through both walls, dark shaft wholly out of the housing, gear still carried in line, neither
    shaft held, and the shafts and gear still."""
    return bool(
        new_through(objects) and old_out(objects) and gear_in_line(objects) and released(objects)
        and objects["gear"].resting()
        and all(objects[n].resting(max_spin=SHAFT_SPIN) for n in ("new_shaft", "old_shaft"))
    )


def distance(objects):
    tip, _ = _ends(objects["new_shaft"], NOSE_TIP, NEAR_END)
    return float(np.linalg.norm(tip - np.array((HX1, HY, AZ))))


def score(objects):
    """Pass/fail: 1.0 when success() holds, else 0."""
    return 1.0 if success(objects) else 0.0


# ---- demo: fixed waypoints, straight-down grip, fingers closing across the shaft ------------------------------
TIP = 0.013  # finger tips below the TCP
SAG_Z = 0.004  # measured: carrying a light part the left arm settles about 4 mm below its target
SAG_Y = 0.0024  # ... and about 2.4 mm to the robot's left of it
GRIP_Z = AZ - 0.003 + TIP  # tips 3 mm below the shaft axis, 2 mm above the groove's lips
ABOVE = TOP + 0.13
PUSH = HX1 + 0.018 - (REST_X0 + NEW_L)  # lead end from 10 mm short of the front face to 18 mm past the rear face


def _lay_in_trough(start=(REST_X0 + NEAR_END, REST_Y), grip_z=GRIP_Z, dx=0.0):
    """Grip a rod at its middle, carry it across and let it down into the trough (the new shaft: 10 mm short of the
    face)."""
    mx, sy = start
    return [Waypoint(1.5, (mx, sy, ABOVE), FINGER_OPEN, "over the rod"),
            Waypoint(1.5, (mx, sy, grip_z), FINGER_OPEN, "down"),
            Waypoint(0.8, (mx, sy, grip_z), FINGER_CLOSED, "grip at the middle"),
            Waypoint(1.0, (mx, sy, grip_z + 0.025), FINGER_CLOSED, "lift"),
            Waypoint(1.5, (mx + dx, HY, grip_z + 0.025), FINGER_CLOSED, "over the trough"),
            Waypoint(1.0, (mx + dx, HY, grip_z + SAG_Z + 0.002), FINGER_CLOSED, "lower to 2 mm over the groove"),
            Waypoint(0.8, (mx + dx, HY, grip_z + SAG_Z + 0.002), FINGER_OPEN, "let go"),
            Waypoint(0.8, (mx + dx, HY, grip_z + 0.03), FINGER_OPEN, "up")]


TURN = -np.pi / 2  # the grip turned a quarter turn about the vertical: the closed fingers lie one behind the other
FINGER_X = 0.012  # ... and the front finger's face is about this far ahead of the TCP (set from the trace)
PUSH_Z = AZ + TIP  # finger tips level with the shaft axis, 3 mm above the trough's lips
LAG = 0.016  # the soft arm lags behind its target against about 3 N of friction


def _push_from_behind(push, near=REST_X0):
    """Closed fingers, turned a quarter turn, come down behind the rod's near end and push it along the groove in
    one stroke."""
    x0 = near - FINGER_X - 0.008
    y = HY - SAG_Y
    return [Waypoint(1.2, (x0, y, PUSH_Z + 0.03), FINGER_CLOSED, "behind the near end", TURN),
            Waypoint(1.0, (x0, y, PUSH_Z), FINGER_CLOSED, "down", TURN),
            Waypoint(5.0, (x0 + 0.008 + push + LAG, y, PUSH_Z), FINGER_CLOSED, "push end to end", TURN),
            Waypoint(1.5, (x0 + 0.008 + push + LAG, y, PUSH_Z), FINGER_CLOSED, "hold while the arm catches up", TURN)]


def _finish(w):
    end, yaw = w[-1].pos, w[-1].yaw
    return w + [Waypoint(0.8, (end[0] - LAG - 0.02, end[1], end[2]), FINGER_CLOSED, "back off", yaw),
                Waypoint(0.8, (end[0] - LAG - 0.02, end[1], ABOVE), FINGER_CLOSED, "up", yaw),
                Waypoint(1.2, (0.15, 0.28, ABOVE), FINGER_OPEN, "away, clear of the housing"),
                Waypoint(3.0, (0.15, 0.28, ABOVE), FINGER_OPEN, "wait")]


def _carry_and_push(push=PUSH, extra=()):
    return _finish(_lay_in_trough() + _push_from_behind(push) + list(extra))


DRIFT_GRIP_Z = BORE_LO + 0.003 + TIP  # tips 2 mm below the 10 mm drift's axis (on its rest)
DRAW_Z = AZ - 0.0023 + TIP  # in the trough: tips just below the drift's axis, 0.7 mm above the trough's lips


def _shortcut(mode):
    """Strategies a reviewer would try, for GPU traces; each should score 0."""
    if mode == "part":  # push the new shaft only 45 mm in and leave it
        return _carry_and_push(push=0.045)
    if mode == "hang":  # stop with the old shaft's front end still 6 mm inside the rear wall
        return _carry_and_push(push=HX1 - 0.006 - (REST_X0 + NEW_L))
    if mode == "hold":  # the full push, but the fingers stay pressed on the shaft's end
        w = _lay_in_trough() + _push_from_behind(PUSH)
        return w + [Waypoint(4.0, w[-1].pos, FINGER_CLOSED, "keep pressing", w[-1].yaw)]
    if mode == "fingers":  # push the old shaft's flush front end with the closed fingers
        y = HY - SAG_Y
        x0 = HX0 - FINGER_X - 0.010
        return _finish([Waypoint(1.5, (x0, y, ABOVE), FINGER_CLOSED, "in front of the old shaft", TURN),
                        Waypoint(1.2, (x0, y, PUSH_Z), FINGER_CLOSED, "down", TURN),
                        Waypoint(3.0, (HX0 - FINGER_X + 0.008, y, PUSH_Z), FINGER_CLOSED, "push the end", TURN),
                        Waypoint(1.5, (HX0 - FINGER_X + 0.008, y, PUSH_Z), FINGER_CLOSED, "keep pushing", TURN)])
    if mode == "drift":  # drive the old shaft out with the drift, take the drift out, then fit the new shaft
        d0 = DRIFT_START[0]
        dx = 0.02  # lay the drift 10 mm short of the face
        push = HX1 + 0.010 - (d0 + dx + DRIFT_L / 2)
        near = d0 + dx - DRIFT_L / 2
        w = _lay_in_trough(start=(d0, DRIFT_Y), grip_z=DRIFT_GRIP_Z, dx=dx)
        w += _push_from_behind(push, near=near)
        w += [Waypoint(1.0, (w[-1].pos[0] - 0.03, w[-1].pos[1], ABOVE), FINGER_CLOSED, "up", TURN)]
        far = near + push
        gx, y = far + 0.012, HY - SAG_Y
        w += [Waypoint(1.0, (gx, y, ABOVE), FINGER_OPEN, "over the drift's end"),
              Waypoint(1.2, (gx, y, DRAW_Z), FINGER_OPEN, "down"),
              Waypoint(0.8, (gx, y, DRAW_Z), FINGER_CLOSED, "grip"),
              Waypoint(3.5, (gx - push - 0.01, y, DRAW_Z), FINGER_CLOSED, "draw the drift out"),
              Waypoint(1.2, (gx - push - 0.01, y, ABOVE), FINGER_CLOSED, "lift it clear"),
              Waypoint(1.5, (gx - push - 0.01, 0.0, ABOVE), FINGER_CLOSED, "aside"),
              Waypoint(1.2, (gx - push - 0.01, 0.0, TOP + TIP + 0.006), FINGER_CLOSED, "down to the table"),
              Waypoint(0.8, (gx - push - 0.01, 0.0, TOP + TIP + 0.006), FINGER_OPEN, "let go"),
              Waypoint(1.0, (gx - push - 0.01, 0.0, ABOVE), FINGER_OPEN, "up")]
        return _finish(w + _lay_in_trough() + _push_from_behind(PUSH))
    raise ValueError(f"unknown SHAFT_DEMO {mode!r}")


def demo():
    """The new shaft laid in the trough and pushed through in one stroke, driving the old one out ahead of it.
    SHAFT_DEMO=static waits; part, hang, hold, fingers and drift replay the shortcuts listed in the
    README, each of which should score 0."""
    mode = os.environ.get("SHAFT_DEMO", "")
    if mode == "static":
        return {"left": [Waypoint(8.0, (0.20, 0.25, ABOVE), FINGER_OPEN, "wait")]}
    if mode:
        return {"left": _shortcut(mode)}
    return {"left": _carry_and_push()}
