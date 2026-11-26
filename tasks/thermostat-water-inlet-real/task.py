"""Seat an engine thermostat square in its counterbore on a vertical housing face with the jiggle valve at the top, then slide the water inlet over its two studs to capture it.

An engine front section stands on the bench with its thermostat housing face vertical, facing the robot. The housing has a 61 mm counterbore 6 mm deep, a
50 mm passage behind it, and two M8 studs 92 mm apart standing 30 mm out of the face. The thermostat (60.0 mm across its seal, 42.9 mm from the seal's mid-plane to the
spring end, 23.9 mm from it to the bridge top) stands on its spring end on a shop mat; the cast water inlet lies beside it on its flange edge and hose end. All rigid.

    python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = ("Replace the engine's thermostat: seat the new thermostat in the housing with its spring end toward the engine and its jiggle valve at the top, and let go of it, "
               "then push the water inlet home over both studs until it sits flush on the housing, and leave it on the studs over the thermostat.")
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # the tracker counts its own 1 s hold, so score() and success() agree
HOLD_STEPS = 20  # 1 s at 20 Hz

# the housing face (vertical, normal -x) and the bore's axis
XF = 0.420
ZC = 0.150  # bore axis above the table
R_CB, D_CB = 0.0305, 0.006  # counterbore: 61.0 mm across (0.5 mm radial clearance on the 60.0 mm seal), 6 mm deep
CHAMFER = 0.0015  # 45-degree lead-in chamfer round the counterbore's mouth
R_PASS = 0.025  # the coolant passage behind the seat
STUD_Y, R_STUD, STUD_L = 0.046, 0.004, 0.030  # two M8 studs, 92 mm apart, 30 mm proud
TIP_L = 0.0025  # the studs' chamfered ends: 8 mm down to 3.6 mm over 2.5 mm
HOLE = 0.005  # stud holes 10 mm across: the coarse clearance hole for M8 (ISO 273)
MAT = 0.004

# thermostat (Stant 46128 / Gates 33868 class): its frame has x along the axis toward the spring end, origin at the seal's mid-plane
R_SEAL, T_SEAL = 0.030, 0.006
CAGE_L, R_CAGE = 0.0429, 0.017
BRIDGE_TOP = 0.0239
BAR_W, BAR_D = 0.012, 0.001  # the bridge's crossbar, stamped sheet: 12 mm across the jaws, 1 mm thick (Gates 33868 photo, scaled to its 56 mm flange)
LEG_Y = 0.0235
LEG_T = 0.003  # the bridge's legs
R_VALVE = 0.021  # the jiggle valve sits on the flange on local +z, between the bridge's legs
TH_MASS = 0.11

# water inlet (2GR-FE class, cast aluminium): its frame has x along the face normal toward the engine, origin at the face's centre
IN_MASS = 0.32
R_OPEN, R_FL = 0.028, 0.036
EAR_Y, EAR_HZ = 0.058, 0.011
FL_T = 0.010
R_DOME, DOME_X, CAP_X = 0.032, -0.030, -0.036
R_NECK, NECK_X = 0.016, -0.100
BEND = math.radians(25.0)
BEND_L = 0.085
GRIP_X = -0.073  # the hand takes the straight neck here

LOOKS = {"iron": m.Look(color=m.rgb(0.36, 0.37, 0.38), roughness=0.6, metallic=0.3), "alu": m.Look(color=m.rgb(0.74, 0.75, 0.76), roughness=0.45, metallic=0.6),
         "steel": m.Look(color=m.rgb(0.80, 0.80, 0.78), roughness=0.3, metallic=0.85), "rubber": m.Look(color=m.rgb(0.08, 0.08, 0.09), roughness=0.85),
         "brass": m.Look(color=m.rgb(0.80, 0.55, 0.30), roughness=0.35, metallic=0.8), "mat": m.Look(color=m.rgb(0.15, 0.16, 0.17), roughness=0.95),
         "stud": m.Look(color=m.rgb(0.55, 0.56, 0.58), roughness=0.4, metallic=0.8), "block": m.Look(color=m.rgb(0.62, 0.63, 0.64), roughness=0.55, metallic=0.4)}


def _hull(points) -> m.Mesh:
    """A convex piece from 8 corners in box order (4 front, 4 back)."""
    v = np.asarray(points, float)
    f = [(0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6), (0, 4, 5), (0, 5, 1), (1, 5, 6), (1, 6, 2), (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0)]
    return m.Mesh(v, np.array(f))


def _ring(r_in, r_out, x0, x1, n, centre=(0.0, 0.0, 0.0), r_in1=None, phase=0.0):
    """A ring about an x-parallel axis through `centre`, as n convex segments; r_in is the apothem, so a round part of radius r_in passes.
    `r_in1`, if given, is the bore at x1, for a conical lead-in; `phase` turns the segments about the axis."""
    cx, cy, cz = centre
    out = []
    for k in range(n):
        a0, a1 = phase + 2 * math.pi * k / n, phase + 2 * math.pi * (k + 1) / n
        pts = []
        for x, ri in ((x0, r_in), (x1, r_in if r_in1 is None else r_in1)):
            rv = ri / math.cos(math.pi / n)
            for r, a in ((rv, a0), (r_out, a0), (r_out, a1), (rv, a1)):
                pts.append((cx + x, cy + r * math.cos(a), cz + r * math.sin(a)))
        out.append(_hull(pts))
    return out


def _add(parts, cols, name, mesh, look, collide=True):
    parts.append((name, mesh, look))
    if collide:
        cols.append(m.col_hull(f"{name}_c", mesh))


def _engine() -> m.Part:
    parts, cols = [], []
    c = (0.0, 0.0, ZC)
    for k, seg in enumerate(_ring(R_CB + CHAMFER, 0.045, XF, XF + CHAMFER, 16, c, r_in1=R_CB)):
        _add(parts, cols, f"lead{k}", seg, "alu")
    for k, seg in enumerate(_ring(R_CB, 0.045, XF + CHAMFER, XF + D_CB, 16, c)):
        _add(parts, cols, f"cb{k}", seg, "alu")
    for k, seg in enumerate(_ring(R_PASS, 0.045, XF + D_CB, XF + 0.020, 16, c)):
        _add(parts, cols, f"seat{k}", seg, "alu")
    for k, (y0, y1, z0, z1) in enumerate([(0.040, 0.075, -0.050, 0.050), (-0.075, -0.040, -0.050, 0.050), (-0.040, 0.040, 0.040, 0.050), (-0.040, 0.040, -0.050, -0.040)]):
        _add(parts, cols, f"boss{k}", m.box((0.020, y1 - y0, z1 - z0), (XF + 0.010, (y0 + y1) / 2, ZC + (z0 + z1) / 2)), "alu")
    for k, (y0, y1, z0, z1) in enumerate([(0.025, 0.050, -0.050, 0.050), (-0.050, -0.025, -0.050, 0.050), (-0.025, 0.025, 0.025, 0.050), (-0.025, 0.025, -0.050, -0.025)]):
        _add(parts, cols, f"neck{k}", m.box((0.040, y1 - y0, z1 - z0), (XF + 0.040, (y0 + y1) / 2, ZC + (z0 + z1) / 2)), "alu")
    _add(parts, cols, "block", m.box((0.19, 0.40, 0.30), (XF + 0.155, 0.0, 0.150)), "block")
    _add(parts, cols, "cover", m.box((0.03, 0.30, 0.10), (XF + 0.045, 0.0, 0.250)), "iron")  # timing-cover lip above the housing
    parts.append(("bore_wall", m.cylinder(R_PASS - 0.0005, 0.040, (XF + 0.040, 0.0, ZC), axis="x", segments=32), "iron"))  # dark passage, seen down the bore
    for k, sy in enumerate((-1, 1)):
        _add(parts, cols, f"stud{k}", m.cylinder(R_STUD, STUD_L - TIP_L, (XF - (STUD_L - TIP_L) / 2, sy * STUD_Y, ZC), axis="x", segments=16), "stud")
        tip = m.lathe([(0.0, 0.0), (R_STUD, 0.0), (0.0018, TIP_L), (0.0, TIP_L)], 16).aimed("-X").moved((XF - STUD_L + TIP_L, sy * STUD_Y, ZC))
        _add(parts, cols, f"tip{k}", tip, "stud")  # chamfered stud end
    _add(parts, cols, "mat", m.box((0.20, 0.50, MAT), (0.29, 0.0, MAT / 2)), "mat")
    return m.fixed("engine", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.6, 0.5))


def _thermostat() -> m.Part:
    parts, cols = [], []
    _add(parts, cols, "seal", m.cylinder(R_SEAL, T_SEAL, (0.0, 0.0, 0.0), axis="x", segments=32), "rubber")
    parts.append(("flange", m.cylinder(0.028, 0.0015, (-0.0035, 0.0, 0.0), axis="x", segments=32), "steel"))
    _add(parts, cols, "cage", m.cylinder(R_CAGE, CAGE_L - T_SEAL / 2, ((CAGE_L + T_SEAL / 2) / 2, 0.0, 0.0), axis="x", segments=20), "brass")
    parts.append(("spring", m.cylinder(R_CAGE + 0.0005, 0.024, (0.018, 0.0, 0.0), axis="x", segments=20), "steel"))
    parts.append(("foot", m.cylinder(0.014, 0.0015, (CAGE_L - 0.00075, 0.0, 0.0), axis="x", segments=20), "steel"))
    for k, sy in enumerate((-1, 1)):
        _add(parts, cols, f"leg{k}", m.box((BRIDGE_TOP - T_SEAL / 2, LEG_T, BAR_W), (-(BRIDGE_TOP + T_SEAL / 2) / 2, sy * LEG_Y, 0.0)), "steel")
    _add(parts, cols, "bar", m.box((BAR_D, 2 * LEG_Y + LEG_T, BAR_W), (-(BRIDGE_TOP - BAR_D / 2), 0.0, 0.0)), "steel")
    parts.append(("valve", m.cylinder(0.0018, 0.008, (-0.002, 0.0, R_VALVE), axis="x", segments=10), "brass"))
    parts.append(("valve_head", m.cylinder(0.003, 0.0015, (-0.0062, 0.0, R_VALVE), axis="x", segments=10), "brass"))
    yaw = math.radians(40.0)  # the bridge starts turned 40 degrees about the vertical, the valve pointing sideways
    q = _quat(_rz(yaw) @ _ry(math.pi / 2))
    return m.free("thermostat", (0.300, -0.120, TOP + MAT + CAGE_L + 0.0005), parts, cols, LOOKS, mass=TH_MASS, friction=(0.8, 0.7), start_quat=q)


def _inlet() -> m.Part:
    parts, cols = [], []
    for k, seg in enumerate(_ring(R_OPEN, R_FL, -FL_T, 0.0, 12, phase=math.pi / 12)):  # a flat of the rim at the bottom
        _add(parts, cols, f"fl{k}", seg, "alu")
    for k, sy in enumerate((-1, 1)):
        for j, (y0, y1, z0, z1) in enumerate([(0.034, STUD_Y - HOLE, -EAR_HZ, EAR_HZ), (STUD_Y + HOLE, EAR_Y, -EAR_HZ, EAR_HZ),
                                               (STUD_Y - HOLE, STUD_Y + HOLE, HOLE, EAR_HZ), (STUD_Y - HOLE, STUD_Y + HOLE, -EAR_HZ, -HOLE)]):
            ya, yb = sorted((sy * y0, sy * y1))
            _add(parts, cols, f"ear{k}{j}", m.box((FL_T, yb - ya, z1 - z0), (-FL_T / 2, (ya + yb) / 2, (z0 + z1) / 2)), "alu")
    for k, seg in enumerate(_ring(R_OPEN, R_DOME, DOME_X, -FL_T, 12)):
        _add(parts, cols, f"dome{k}", seg, "alu")
    _add(parts, cols, "cap", m.cylinder(R_DOME, DOME_X - CAP_X, ((DOME_X + CAP_X) / 2, 0.0, 0.0), axis="x", segments=24), "alu")
    _add(parts, cols, "neck", m.cylinder(R_NECK, CAP_X - NECK_X, ((CAP_X + NECK_X) / 2, 0.0, 0.0), axis="x", segments=20), "alu")
    u = np.array([-math.cos(BEND), 0.0, -math.sin(BEND)])  # the second leg of the neck, bent toward local -z
    mid = np.array([NECK_X, 0.0, 0.0]) + u * BEND_L / 2
    leg = m.cylinder(R_NECK, BEND_L, (0.0, 0.0, 0.0), axis="x", segments=20).turned(-math.degrees(BEND), "y").moved(mid)
    _add(parts, cols, "hose_leg", leg, "alu")
    bead = m.cylinder(R_NECK + 0.002, 0.004, (0.0, 0.0, 0.0), axis="x", segments=20).turned(-math.degrees(BEND), "y").moved(mid + u * (BEND_L / 2 - 0.012))
    parts.append(("bead", bead, "alu"))
    parts.append(("elbow", m.sphere(R_NECK, (NECK_X, 0.0, 0.0), rings=6, segments=20), "alu"))
    return m.free("inlet", IN_POS, parts, cols, LOOKS, mass=IN_MASS, friction=(0.8, 0.7), start_quat=_quat(IN_START))


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _quat(R) -> tuple:
    """Rotation matrix to (x, y, z, w)."""
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return (x, y, z, w)


IN_TILT = math.atan2(0.0504 - R_FL * math.cos(math.pi / 12), 0.1702)  # 5.2 degrees: resting on the rim's lowest flat and the hose end
IN_START = np.array([[math.cos(IN_TILT), 0.0, -math.sin(IN_TILT)], [0.0, 1.0, 0.0], [math.sin(IN_TILT), 0.0, math.cos(IN_TILT)]]).T  # laid on the bench face toward the engine, the bent neck down
IN_POS = (0.380, 0.120, TOP + MAT + R_FL * math.cos(math.pi / 12) * math.cos(IN_TILT) + 0.0005)
IN_FINAL = np.eye(3)  # face toward the engine (+x), ears along y, the bent neck dropping toward the hose

OBJECTS = [_engine(), _thermostat(), _inlet()]


# -- scoring ------------------------------------------------------------------------------------

def gripper_points(environment) -> np.ndarray:
    """Points over both whole grippers (station frame): both fingers through their thickness, width and length, and the gripper housing (palm) behind them."""
    base = environment.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = []
    fy, fz = np.linspace(-0.018, 0.018, 5), np.linspace(0.090, 0.150, 7)
    hx, hy, hz = np.linspace(-0.045, 0.045, 7), np.linspace(-0.030, 0.030, 5), np.linspace(0.0, 0.090, 6)
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = m._np(robot.data.body_link_pose_w)[0, i]
        R = m.quat_matrix(pose[3:7])
        opening = abs(float(m._np(robot.data.joint_pos)[0, robot.find_joints("joint7")[0][0]]))
        local = [(sx * (opening + 0.0045 + t), y, z) for sx in (-1.0, 1.0) for t in (-0.0045, 0.0, 0.0045) for y in fy for z in fz]
        local += [(x, y, z) for x in hx for y in hy for z in hz]
        out.append(pose[:3] - origin + np.asarray(local) @ R.T)
    return np.vstack(out)


def _cyl_dist(p, r, x0, x1):
    """Distance from points p (in a part's frame) to a solid cylinder about x from x0 to x1."""
    dr = np.maximum(np.hypot(p[:, 1], p[:, 2]) - r, 0.0)
    dx = np.maximum(np.maximum(x0 - p[:, 0], p[:, 0] - x1), 0.0)
    return np.hypot(dr, dx)


def _box_dist(p, lo, hi):
    return np.linalg.norm(np.maximum(np.maximum(np.asarray(lo) - p, p - np.asarray(hi)), 0.0), axis=1)


def thermostat_gap(pts, th) -> float:
    """Nearest gripper point to the thermostat's outer surface (its whole envelope, seal to spring end)."""
    p = (pts - th.pos) @ th.rotation()
    return float(np.min(_cyl_dist(p, R_SEAL, -BRIDGE_TOP, CAGE_L)))


def inlet_gap(pts, inl) -> float:
    """Nearest gripper point to the water inlet: flange with ears, dome, straight neck and the bent hose leg."""
    p = (pts - inl.pos) @ inl.rotation()
    dists = [_box_dist(p, (-FL_T, -EAR_Y, -R_FL), (0.0, EAR_Y, R_FL)), _cyl_dist(p, R_DOME, CAP_X, -FL_T), _cyl_dist(p, R_NECK, NECK_X, CAP_X)]
    a = np.array([NECK_X, 0.0, 0.0])
    u = np.array([-math.cos(BEND), 0.0, -math.sin(BEND)])
    t = np.clip((p - a) @ u, 0.0, BEND_L)
    dists.append(np.maximum(np.linalg.norm(p - (a + t[:, None] * u), axis=1) - R_NECK, 0.0))
    return float(np.min(np.minimum.reduce(dists)))


def measure(th, inl, pts) -> dict:
    R = th.rotation()
    ax = R[:, 0]
    tilt = math.degrees(math.acos(float(np.clip(ax[0], -1.0, 1.0))))  # spring-end axis against the bore's axis (+x, into the engine)
    p = np.asarray(th.pos, float)
    depth = (p[0] - XF) * 1000  # the seal's mid-plane behind the face: 3.0 mm when the seal is home on its seat
    radial = math.hypot(p[1], p[2] - (TOP + ZC)) * 1000
    v = R[:, 2]
    valve = abs(math.degrees(math.atan2(v[1], v[2])))  # the jiggle valve's angle from top dead centre, about the bore axis
    Ri = inl.rotation()
    face = [inl.pos + Ri @ np.array(q) for q in ((0.0, EAR_Y, 0.0), (0.0, -EAR_Y, 0.0), (0.0, 0.0, R_FL), (0.0, 0.0, -R_FL))]
    gaps = [(XF - f[0]) * 1000 for f in face]  # each extreme of the inlet's face, out from the housing face
    holes = [inl.pos + Ri @ np.array((0.0, sy * STUD_Y, 0.0)) for sy in (-1, 1)]
    studs = []
    for sy in (-1, 1):
        studs.append(min(math.hypot(h[1] - sy * STUD_Y, h[2] - (TOP + ZC)) for h in holes) * 1000)
    return {"tilt": round(tilt, 2), "depth_mm": round(depth, 2), "radial_mm": round(radial, 2), "valve": round(valve, 1),
            "face_gap_mm": [round(g, 2) for g in gaps], "stud_mm": [round(s, 2) for s in studs], "th_hand_mm": round(thermostat_gap(pts, th) * 1000, 1),
            "in_hand_mm": round(inlet_gap(pts, inl) * 1000, 1), "th_z_mm": round((p[2] - TOP) * 1000, 1)}


def seated(r) -> bool:
    """Seal home on its seat (seal mid-plane 3 mm behind the face, within 1 mm), centred, spring end in, square within 3 degrees."""
    return 2.0 <= r["depth_mm"] <= 4.0 and r["radial_mm"] <= 1.5 and r["tilt"] <= 3.0


def flush(r) -> bool:
    """Water inlet home over both studs, its face within 1 mm of the housing face all round."""
    return max(r["stud_mm"]) <= 2.0 and all(-0.5 <= g <= 1.0 for g in r["face_gap_mm"])


ON_STUDS_MM = 20.0  # the thermostat's bridge stands 20.9 mm proud of the face (23.9 mm from the seal's mid-plane, seated 3.0 mm deep): an inlet closer than this still encloses it


def on_studs(r) -> bool:
    """Water inlet resting on both studs over the thermostat: holes on the studs, no point of its face more than 20 mm out."""
    return max(r["stud_mm"]) <= 3.0 and all(-0.5 <= g <= ON_STUDS_MM for g in r["face_gap_mm"])


CLEAR_MM = 10.0  # no part of either gripper within 10 mm of a part counts as let go


class Tracker:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.steps, self.last = 0, {}
        self.alone = self.alone_valve = 0  # steps in a row with the thermostat seated, let go, and the inlet not yet on
        self.t1 = self.t2 = self.t3 = False
        self.home = 0  # steps in a row with the inlet pushed flush over the seated thermostat
        self.hold = 0

    def step(self, env):
        self.steps += 1
        th, inl = m.state(env, "thermostat"), m.state(env, "inlet")
        r = measure(th, inl, gripper_points(env))
        cover_off = min(r["face_gap_mm"]) > 10.0
        alone = seated(r) and r["th_hand_mm"] > CLEAR_MM and cover_off
        self.alone = self.alone + 1 if alone else 0
        self.alone_valve = self.alone_valve + 1 if alone and r["valve"] <= 10.0 else 0
        self.t1 = self.t1 or self.alone >= HOLD_STEPS
        self.t2 = self.t2 or self.alone_valve >= HOLD_STEPS
        good = self.t2 and seated(r) and r["valve"] <= 10.0
        self.home = self.home + 1 if good and flush(r) else 0
        self.t3 = self.t3 or self.home >= HOLD_STEPS
        done = self.t3 and good and on_studs(r) and r["th_hand_mm"] > CLEAR_MM and r["in_hand_mm"] > CLEAR_MM
        self.hold = self.hold + 1 if done else 0
        self.last = {**r, "t1": self.t1, "t2": self.t2, "t3": self.t3, "hold": self.hold}
        if self.steps % 10 == 0:
            print("[thermo] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), "t": round(self.steps * float(env.unwrapped.step_dt), 2), **self.last}), flush=True)


TRACK = Tracker()
m.on_step(TRACK.step)
m.on_reset(TRACK.reset)


def _tier(t1, t2, last) -> float:
    if not last:
        return 0.0
    if t1 and not seated(last):  # seated once, then knocked cocked or out (a pinched seal): the run is void
        return 0.0
    return 0.6 if t2 and last["valve"] <= 10.0 else 0.3 if t1 else 0.0  # the valve must still be up at the end


def score(objects) -> float:
    """1.0 once the inlet, pushed flush for 1 s, has rested on the studs with both hands clear for HOLD_STEPS steps; 0.6 for the thermostat seated alone with its valve up, and still up; 0.3 seated alone; 0 if it was seated and is now cocked or out."""
    if TRACK.hold >= HOLD_STEPS:
        return 1.0
    return _tier(TRACK.t1, TRACK.t2, TRACK.last)


def success(objects) -> bool:
    return TRACK.hold >= HOLD_STEPS


# -- the scripted demo --------------------------------------------------------------------------

AXIS = np.array([1.0, 0.0, 0.0])
BORE = np.array([XF, 0.0, TOP + ZC])
PINCH = d._as_grip(d.tilted(90.0, 0.0).rot @ d.axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.pi / 2))  # fingers into the bore, closing up and down on the crossbar
TH_TCP = np.array([-0.030, 0.0, 0.0])  # the TCP in the thermostat's frame when the crossbar is pinched
IN_G = d._as_grip(d.axis_angle_matrix(AXIS, math.radians(30.0)) @ d.axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(20.0))
                  @ d.level(-90.0).rot @ d.axis_angle_matrix(np.array([0.0, 0.0, 1.0]), -math.pi / 2))
# the left hand from the robot's side of the neck: fingers 20 degrees toward the engine and 30 degrees below level, closing across the level neck
IN_TCP = np.array([GRIP_X, 0.0, 0.0])


def _pose(ctx, name):
    st = ctx.state(name)
    return np.asarray(st.pos, float), m.quat_matrix(st.quat_xyzw)


def _square(ctx, arm, name, goal_R, t=0.6):
    """Turn the held part about its own origin until its frame matches goal_R (read from its pose)."""
    p, R = _pose(ctx, name)
    E = goal_R @ R.T
    ang = math.acos(float(np.clip((np.trace(E) - 1) / 2, -1.0, 1.0)))
    if ang < math.radians(0.3):
        return
    ax = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])
    yield from arm.arc(p, ax / np.linalg.norm(ax), math.degrees(ang), t=t, label=f"square the {name}")


def _right(ctx, goal=np.eye(3)):
    """Seat the thermostat; `goal` is its final orientation (the identity: spring end in, valve at the top)."""
    r = ctx.right
    p, R = _pose(ctx, "thermostat")
    g = d._as_grip(R @ PINCH.rot)  # a down grip on the crossbar, turned so that the valve comes up when the hand turns into the bore
    tcp = p + R @ TH_TCP
    yield from r.move(tcp + np.array([0.0, 0.0, 0.08]), grip=g, finger=d.FINGER_OPEN, t=2.5, label="over the thermostat")
    yield from r.move(tcp, t=1.2, label="down round the crossbar")
    yield from r.close(BAR_W, force=25.0, label="pinch the crossbar")
    yield from r.move(r.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="lift")
    stand = BORE + np.array([-0.060, 0.0, 0.0])
    yield from r.move(stand + goal @ TH_TCP, grip=d._as_grip(goal @ PINCH.rot), t=2.5, label="turn it spring end first to the bore")
    for _ in range(2):
        yield from _square(ctx, r, "thermostat", goal)
        yield from r.servo(stand, tol=0.0005, timeout=1.5, read=lambda: _pose(ctx, "thermostat")[0], label="on the bore's axis")
    near = BORE + np.array([-0.009, 0.0, 0.0])
    yield from r.servo(near, tol=0.0005, timeout=2.0, read=lambda: _pose(ctx, "thermostat")[0], label="spring end into the passage, seal at the mouth")
    yield from _square(ctx, r, "thermostat", goal, t=0.4)
    home = BORE + np.array([0.0032, 0.0, 0.0])  # the seal's mid-plane with the seal on its seat
    yield from r.servo(lambda t: near + (home - near) * min(1.0, t / 1.5), tol=0.0004, timeout=3.0, read=lambda: _pose(ctx, "thermostat")[0], label="push the seal home, square")
    yield from r.wait(0.3)
    yield from r.open(label="let go")
    yield from r.move(r.at() + np.array([-0.08, 0.0, 0.0]), t=1.2, label="back out")
    yield from r.move(np.array([0.25, -0.25, TOP + 0.25]), t=1.5, label="clear")


def _left(ctx):
    l = ctx.left
    p, R = _pose(ctx, "inlet")
    g = d._as_grip(R @ IN_FINAL.T @ IN_G.rot)  # the inlet's grip at its bench pose
    tcp = p + R @ IN_TCP
    a = l.approach_axis(g)
    yield from l.move(tcp - 0.08 * a, grip=g, finger=-0.024, t=2.5, label="beside the inlet's neck")
    yield from l.move(tcp, t=1.5, label="fingers over and under the neck")
    yield from l.close(2 * R_NECK, force=30.0, label="grip the neck")
    yield from l.move(l.at() + np.array([0.0, 0.0, 0.12]), t=1.2, label="lift")
    ready = np.array([XF - 0.15, 0.13, TOP + ZC + 0.06])
    yield from l.move(ready + IN_FINAL @ IN_TCP, grip=IN_G, t=2.0, label="wait to the side")


def _inlet_on(ctx, release=True):
    l = ctx.left
    face = BORE + np.array([-0.045, 0.0, 0.0])
    yield from l.move(face + IN_FINAL @ IN_TCP, t=2.0, label="in front of the studs")
    for _ in range(2):
        yield from _square(ctx, l, "inlet", IN_FINAL)
        yield from l.servo(face, tol=0.0004, timeout=3.0, read=lambda: _pose(ctx, "inlet")[0], label="holes on the studs' axes")
    start = BORE + np.array([-0.033, 0.0, 0.0])
    yield from l.servo(start, tol=0.0004, timeout=2.5, read=lambda: _pose(ctx, "inlet")[0], label="holes at the stud tips")
    yield from _square(ctx, l, "inlet", IN_FINAL, t=0.4)
    yield from l.servo(start, tol=0.0004, timeout=2.0, read=lambda: _pose(ctx, "inlet")[0], label="holes at the stud tips, square")
    mid = BORE + np.array([-0.022, 0.0, 0.0])  # 8 mm onto the studs, before the thermostat's bridge (20.9 mm proud) enters the inlet
    yield from l.servo(lambda t: start + (mid - start) * min(1.0, t / 1.5), tol=0.0004, timeout=2.5, read=lambda: _pose(ctx, "inlet")[0], label="onto the studs")
    yield from _square(ctx, l, "inlet", IN_FINAL, t=0.4)
    yield from l.servo(mid, tol=0.0004, timeout=1.5, read=lambda: _pose(ctx, "inlet")[0], label="onto the studs, square")
    home = BORE + np.array([0.0002, 0.0, 0.0])
    yield from l.servo(lambda t: mid + (home - mid) * min(1.0, t / 2.5), tol=0.0004, timeout=4.0, read=lambda: _pose(ctx, "inlet")[0], label="slide it home over the thermostat")
    yield from l.wait(1.5)  # held flush for the 1 s check before letting go
    if not release:
        yield from l.wait(4.0)
        return
    yield from l.move(l.at(), finger=-0.024, t=0.5, label="let go")
    yield from l.move(l.at() - 0.08 * l.approach_axis(), t=1.2, label="fingers out from the neck")
    yield from l.move(l.at() + np.array([-0.05, 0.08, 0.08]), t=1.5, label="clear")


def demo_script(ctx):
    """full: the task. Lazy routes: none (nothing moves), no_thermostat (the inlet closed over an empty housing), valve_side (thermostat seated with its
    valve at the side), cocked (thermostat pushed in 6 degrees off square), held (the inlet held flush, never let go)."""
    mode = ctx.mode
    if mode == "none":
        yield from ctx.left.wait(10.0)
    else:
        if mode == "no_thermostat":
            yield from _left(ctx)
        else:
            goal = {"valve_side": d.axis_angle_matrix(AXIS, math.pi / 2), "cocked": d.axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(6.0))}.get(mode, np.eye(3))
            yield from ctx.together(_right(ctx, goal), _left(ctx))
        yield from _inlet_on(ctx, release=mode != "held")
    yield from ctx.left.wait(2.0)
    print("[thermo] END " + json.dumps({"mode": ctx.mode, **TRACK.last}), flush=True)
