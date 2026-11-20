"""The scene's parts, built procedurally in this file: a paver patio (the ground) laid in a drip tray, a 6.3 L plastic
pail, a garden tap on a brick pillar with a digital water meter screwed onto it, a 5/8 inch garden hose, a pistol-grip
spray nozzle with a sprung trigger, and clear plastic splash sleeves over the arms and their bases. NumPy only at import
(the kit authors them in Isaac Sim later).

World frame: x forward from the arm bases, y left, z up, metres. The arm bases stand at (0, +-0.305, 0.75).
"""

from __future__ import annotations

import math

import numpy as np

import kit
import meshes as m
import water as w
from kit import Body, BoxShape, CylShape, MeshShape
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- materials ------------------------------------------------------------------------------------------------------

PAVER = (0.56, 0.55, 0.52)
JOINT_SAND = (0.36, 0.33, 0.28)
PAIL_BLUE = (0.07, 0.30, 0.62)
WIRE = (0.62, 0.63, 0.64)
GRIP_BLACK = (0.05, 0.05, 0.055)
HOSE_GREEN = (0.10, 0.42, 0.16)
HOSE_STRIPE = (0.78, 0.66, 0.16)
NOZZLE_BODY = (0.80, 0.81, 0.82)  # die-cast zinc, satin
NOZZLE_RUBBER = (0.09, 0.30, 0.13)  # the green rubber overmould
NOZZLE_DIAL = (0.12, 0.12, 0.13)
TRIGGER_METAL = (0.70, 0.71, 0.72)
BRASS = (0.78, 0.60, 0.26)
VALVE_RED = (0.62, 0.07, 0.06)
BRICK = (0.55, 0.24, 0.16)
MORTAR = (0.70, 0.68, 0.62)
WATER = (0.80, 0.90, 0.93)  # a GLASS colour: rendered as clear water
STREAM = (0.81, 0.90, 0.93)

kit.FINISH.update({
    PAVER: (0.92, 0.0), JOINT_SAND: (1.0, 0.0), PAIL_BLUE: (0.32, 0.0), WIRE: (0.30, 1.0), GRIP_BLACK: (0.6, 0.0),
    HOSE_GREEN: (0.28, 0.0), HOSE_STRIPE: (0.30, 0.0), NOZZLE_BODY: (0.30, 1.0), NOZZLE_RUBBER: (0.65, 0.0),
    NOZZLE_DIAL: (0.55, 0.0), TRIGGER_METAL: (0.25, 1.0), BRASS: (0.32, 1.0), VALVE_RED: (0.45, 0.0),
    BRICK: (0.85, 0.0), MORTAR: (0.95, 0.0),
})
TRAY_BLACK = (0.07, 0.07, 0.075)  # black HDPE drip tray
METER_GREY = (0.20, 0.21, 0.22)  # the meter's housing
METER_LCD = (0.55, 0.62, 0.52)  # its LCD window
METER_BLUE = (0.10, 0.28, 0.55)  # its swivel nut and buttons
SLEEVE = (0.88, 0.91, 0.93)  # clear polyethylene splash sleeves
kit.FINISH.update({TRAY_BLACK: (0.55, 0.0), METER_GREY: (0.40, 0.0), METER_LCD: (0.15, 0.0), METER_BLUE: (0.40, 0.0),
                   SLEEVE: (0.20, 0.0)})
kit.OPACITY.update({SLEEVE: 0.28})
FOAM = (0.90, 0.93, 0.94)
WET = (0.30, 0.30, 0.29)  # wet concrete
kit.FINISH.update({FOAM: (0.55, 0.0), WET: (0.12, 0.0)})
# water renders as clear glass (refractive index 1.333); the flood stream is partly aerated, so slightly frosted
kit.GLASS.update({WATER: ((0.86, 0.95, 0.96), 1.333, 0.0), STREAM: ((0.95, 0.98, 0.99), 1.333, 0.18)})

# -- the drip tray: a shallow black HDPE tray on the table that the patio is laid in -------------------------------

TRAY_X = (0.092, 0.908)  # outside
TRAY_Y = (-0.612, 0.612)
TRAY_FLOOR = 0.002
TRAY_WALL = 0.004
TRAY_H = 0.035  # the walls' height above the table


def _tray_shapes():
    cx, cy = (TRAY_X[0] + TRAY_X[1]) / 2, (TRAY_Y[0] + TRAY_Y[1]) / 2
    sx, sy = TRAY_X[1] - TRAY_X[0], TRAY_Y[1] - TRAY_Y[0]
    t, h = TRAY_WALL, TRAY_H
    return [
        BoxShape((cx, cy, TRAY_FLOOR / 2), (sx, sy, TRAY_FLOOR), TRAY_BLACK),
        BoxShape((TRAY_X[0] + t / 2, cy, h / 2), (t, sy, h), TRAY_BLACK),
        BoxShape((TRAY_X[1] - t / 2, cy, h / 2), (t, sy, h), TRAY_BLACK),
        BoxShape((cx, TRAY_Y[0] + t / 2, h / 2), (sx - 2 * t, t, h), TRAY_BLACK),
        BoxShape((cx, TRAY_Y[1] - t / 2, h / 2), (sx - 2 * t, t, h), TRAY_BLACK),
        # the rolled-over top edge of the walls
        *[CylShape((x, cy, h), 0.003, sy, TRAY_BLACK, collide=False, axis="Y") for x in (TRAY_X[0] + t / 2,
                                                                                          TRAY_X[1] - t / 2)],
        *[CylShape((cx, y, h), 0.003, sx, TRAY_BLACK, collide=False, axis="X") for y in (TRAY_Y[0] + t / 2,
                                                                                          TRAY_Y[1] - t / 2)],
    ]


TRAY = Body.build("drip_tray", pos=(0.0, 0.0, TOP), mass=4.0, kinematic=True, friction=(0.8, 0.7),
                  shapes=_tray_shapes())

# -- the ground: a concrete paver patio laid in the tray ---------------------------------------------------------------

PATIO_X = (0.10, 0.90)
PATIO_Y = (-0.60, 0.60)
PAVER_T = 0.012
GROUND = TOP + TRAY_FLOOR + PAVER_T  # the ground's surface


def _patio_shapes():
    cx, cy = (PATIO_X[0] + PATIO_X[1]) / 2, (PATIO_Y[0] + PATIO_Y[1]) / 2
    sx, sy = PATIO_X[1] - PATIO_X[0], PATIO_Y[1] - PATIO_Y[0]
    shapes = [  # one smooth collider; the pavers on it are visual
        BoxShape((cx, cy, PAVER_T / 2), (sx, sy, PAVER_T), None),
        BoxShape((cx, cy, PAVER_T / 2 - 0.001), (sx, sy, PAVER_T - 0.002), JOINT_SAND, collide=False),
    ]
    rng = np.random.default_rng(7)
    tile, gap = 0.200, 0.004
    tones = [tuple(float(np.clip(c * k, 0, 1)) for c in PAVER) for k in (0.92, 0.97, 1.0, 1.04)]
    for row, x0 in enumerate(np.arange(PATIO_X[0], PATIO_X[1] - 1e-6, tile)):
        offset = tile / 2 if row % 2 else 0.0  # running bond
        for y0 in np.arange(PATIO_Y[0] - offset, PATIO_Y[1] - 1e-6, tile):
            ya, yb = max(y0, PATIO_Y[0]), min(y0 + tile, PATIO_Y[1])
            if yb - ya < 0.02:
                continue
            shapes.append(BoxShape(((x0 + x0 + tile) / 2, (ya + yb) / 2, PAVER_T / 2),
                                   (tile - gap, yb - ya - gap, PAVER_T), tones[int(rng.integers(len(tones)))],
                                   collide=False))
    return shapes


PATIO = Body.build("patio", pos=(0.0, 0.0, TOP + TRAY_FLOOR), mass=50.0, kinematic=True, friction=(0.9, 0.8),
                   shapes=_patio_shapes())

# -- the pail -----------------------------------------------------------------------------------------------------------

BUCKET_XY = (0.46, -0.04)
FLOOR_Z = 0.008  # the inner floor above the pail's base (bucket frame)
BRIM_Z = FLOOR_Z + w.DEPTH  # the brim: the top of the inside wall
BUCKET_KG = 0.38


def r_in(z):
    return w.radius_at(z - FLOOR_Z)


def r_out(z):
    return r_in(z) + w.WALL


def _pail_profile():
    zb = FLOOR_Z
    ro0 = r_out(zb)
    prof = [(0.0, 0.004), (ro0 - 0.014, 0.004), (ro0 - 0.012, 0.0), (ro0 - 0.003, 0.0), (ro0, 0.004)]
    for z in (0.02, 0.05, 0.10, 0.150):
        prof.append((r_out(z), z))
    for dz, dr in ((0.0, 0.0), (0.002, 0.0035), (0.008, 0.0035), (0.010, 0.0)):  # a stiffening rib
        prof.append((r_out(0.155 + dz) + dr, 0.155 + dz))
    prof.append((r_out(BRIM_Z - 0.010), BRIM_Z - 0.010))
    top = BRIM_Z
    ro = r_out(top)
    # the rolled rim: out, round and back in over the brim
    for a in np.linspace(-math.pi / 2, math.pi * 0.95, 12):
        prof.append((ro + 0.0045 + 0.0055 * math.cos(a), top - 0.0045 + 0.0055 * math.sin(a)))
    ri = r_in(top)
    prof += [(ri + 0.0012, top + 0.0005), (ri, top - 0.0015)]
    for z in (top - 0.02, 0.10, zb + 0.004):
        prof.append((r_in(z), z))
    prof += [(r_in(zb) - 0.003, zb), (0.0, zb)]
    return prof


def _bail():
    """The wire bail handle, hinged in two ears and lying down along the pail's side toward -x."""
    ear_z = BRIM_Z - 0.022
    ear_r = r_out(ear_z) + 0.006
    tilt = math.radians(108.0)  # the bail lies down over the -x side, resting against the wall
    pts = []
    for a in np.linspace(0.0, math.pi, 41):  # a half-circle from one ear to the other
        rise = (ear_r + 0.022) * math.sin(a)
        pts.append((-rise * math.sin(tilt), ear_r * math.cos(a), ear_z + rise * math.cos(tilt)))
    pts = np.array(pts)
    out = [MeshShape(*_np(m.tube(pts, 0.0022, 10)), WIRE, collide=False, smooth=True)]
    mid = len(pts) // 2
    grip = pts[mid - 6:mid + 7]
    out.append(MeshShape(*_np(m.tube(grip, 0.0085, 16)), GRIP_BLACK, collide=False, smooth=True))
    for s in (-1, 1):  # the ears
        out.append(BoxShape((0.0, s * (r_out(ear_z) + 0.003), ear_z), (0.022, 0.008, 0.026), PAIL_BLUE, collide=False))
    return out


def _np(mesh):
    p, f = mesh
    return tuple(map(tuple, np.round(p, 6).tolist())), tuple(map(tuple, f.tolist()))


def _pail_colliders(n: int = 20):
    out = [CylShape((0.0, 0.0, FLOOR_Z / 2), r_out(FLOOR_Z), FLOOR_Z, None)]
    z0, z1 = FLOOR_Z, BRIM_Z + 0.004
    for k in range(n):
        a = 2 * math.pi * (k + 0.5) / n
        c, s = math.cos(a), math.sin(a)
        rb, rt = r_in(z0) + w.WALL / 2 + 0.002, r_in(z1) + w.WALL / 2 + 0.002
        width = 2 * math.pi * (rt + 0.004) / n * 1.06
        out.append(kit.plate((rb * c, rb * s, z0), (rt * c, rt * s, z1), width, w.WALL + 0.004, (c, s, 0.0), None))
    return out


BUCKET = Body.build(
    "bucket", pos=(*BUCKET_XY, GROUND + 0.0005), mass=BUCKET_KG, friction=(0.7, 0.6),
    shapes=[MeshShape(*_np(m.lathe(_pail_profile(), 96)), PAIL_BLUE, collide=False, smooth=True),
            *_bail(), *_pail_colliders()],
    com=(0.0, 0.0, 0.07),
)
kit.dynamic_mesh("water_body", "bucket", WATER)

# -- the garden tap on its brick pillar ----------------------------------------------------------------------------------

TAP_XY = (0.66, 0.50)
HOSE_IN_Z = 0.095  # where the hose's coupling screws on (the meter's outlet), above the ground
# A digital water meter screws onto the tap's nose, the hose onto the meter (top to bottom: the meter's blue swivel
# nut, holding a 12 L/min flow-regulator washer, its housing with the LCD toward the robot, its male outlet).
METER_NUT = 0.016  # the swivel nut's height
METER_BODY = 0.058  # the housing's height
METER_OUT = 0.010  # the male outlet's height
METER_R = 0.024  # the housing's radius
METER_STACK = METER_NUT + METER_BODY + METER_OUT
SPOUT_Z = HOSE_IN_Z + METER_STACK  # the tap's own outlet above the ground
PILLAR = (0.115, 0.115, SPOUT_Z + 0.065)  # x, y, height above the ground
SPOUT_X = -PILLAR[0] / 2 - 0.055  # the outlet, in front of the pillar's -x face (tap frame)


def _pillar_shapes():
    shapes = [BoxShape((0, 0, PILLAR[2] / 2), PILLAR, MORTAR)]
    rng = np.random.default_rng(3)
    course, joint = 0.040, 0.005
    half = PILLAR[0] / 2
    for i, z in enumerate(np.arange(0.0, PILLAR[2] - 1e-6, course)):
        cuts = [-half, 0.0, half] if i % 2 == 0 else [-half, -half / 2, half / 2, half]  # running bond
        for face in range(4):
            yaw = face * math.pi / 2
            n = np.array([math.cos(yaw), math.sin(yaw), 0.0])
            t = np.array([-n[1], n[0], 0.0])
            for u0, u1 in zip(cuts[:-1], cuts[1:]):
                c = n * (half + 0.002) + t * (u0 + u1) / 2 + np.array([0, 0, z + course / 2])
                tone = tuple(float(np.clip(v * rng.uniform(0.86, 1.10), 0, 1)) for v in BRICK)
                shapes.append(BoxShape(tuple(c), (0.004, u1 - u0 - joint, course - joint), tone, yaw=yaw,
                                       collide=False))
    shapes.append(BoxShape((0, 0, PILLAR[2] + 0.008), (PILLAR[0] + 0.02, PILLAR[1] + 0.02, 0.016), MORTAR))  # cap
    # the brass hose bib: a body out of the front face, bent down to a threaded nose
    body = [(-PILLAR[0] / 2 + 0.005, 0, SPOUT_Z + 0.050), (SPOUT_X - 0.004, 0, SPOUT_Z + 0.050)]
    bend = [(SPOUT_X + 0.016 * (1 - math.sin(a)) - 0.004 * 0, 0, SPOUT_Z + 0.034 + 0.016 * math.cos(a))
            for a in np.linspace(0, math.pi / 2, 8)]
    path = np.array(body + bend[1:] + [(SPOUT_X + 0.0, 0, SPOUT_Z + 0.010)])
    path[len(body):, 0] = np.linspace(SPOUT_X - 0.004, SPOUT_X, len(path) - len(body))
    shapes.append(MeshShape(*_np(m.tube(path, 0.0115, 20)), BRASS, collide=False, smooth=True))
    shapes.append(CylShape((-PILLAR[0] / 2 - 0.006, 0, SPOUT_Z + 0.050), 0.022, 0.012, BRASS, collide=False,
                           axis="X"))  # the flange
    shapes.append(CylShape((SPOUT_X, 0, SPOUT_Z + 0.004), 0.0135, 0.018, BRASS, collide=False))  # threaded nose
    shapes.append(CylShape((SPOUT_X + 0.022, 0, SPOUT_Z + 0.072), 0.004, 0.030, BRASS, collide=False))  # stem
    wheel = []
    for k in range(5):  # the cast handwheel
        a = 2 * math.pi * k / 5
        wheel.append(BoxShape((SPOUT_X + 0.022 + 0.012 * math.cos(a), 0.012 * math.sin(a), SPOUT_Z + 0.088),
                              (0.024, 0.005, 0.004), VALVE_RED, yaw=a, collide=False))
    rim = m.lathe([(0.025, -0.003), (0.028, 0.0), (0.025, 0.003), (0.022, 0.0), (0.025, -0.003)], 40)
    shapes += wheel
    shapes.append(MeshShape(*_np(m.transform(rim, None, (SPOUT_X + 0.022, 0, SPOUT_Z + 0.088))), VALVE_RED,
                            collide=False, smooth=True))
    return shapes + _meter_shapes()


def _meter_shapes():
    """The digital water meter (tap frame), hanging from the tap's nose: swivel nut, housing, LCD, outlet."""
    x = SPOUT_X
    z_nut = SPOUT_Z + 0.004 - METER_NUT  # the nut's foot (the tap's nose ends at SPOUT_Z + 0.004 - 0.009)
    z_body = z_nut - METER_BODY
    z_out = HOSE_IN_Z - 0.005  # the outlet reaches into the hose's coupling and up into the housing
    nut = m.lathe([(0.0, 0.0), (0.0165, 0.0), (0.0175, 0.002), (0.0175, METER_NUT - 0.002), (0.0165, METER_NUT),
                   (0.0, METER_NUT)], 12)  # a 12-sided grip
    housing = m.lathe([(0.0, 0.0), (0.012, 0.0), (METER_R - 0.004, 0.004), (METER_R, 0.012),
                       (METER_R, METER_BODY - 0.012), (METER_R - 0.004, METER_BODY - 0.004), (0.014, METER_BODY),
                       (0.0, METER_BODY)], 40)
    out_h = METER_OUT + 0.011
    outlet = m.lathe([(0.0, 0.0), (0.0120, 0.0), (0.0125, 0.002), (0.0125, out_h), (0.0, out_h)], 24)
    return [
        MeshShape(*_np(m.transform(nut, None, (x, 0, z_nut))), METER_BLUE, collide=False, smooth=False),
        MeshShape(*_np(m.transform(housing, None, (x, 0, z_body))), METER_GREY, collide=False, smooth=True),
        MeshShape(*_np(m.transform(outlet, None, (x, 0, z_out))), BRASS, collide=False, smooth=True),
        # the LCD window and its button on the housing's -x face (toward the robot)
        BoxShape((x - METER_R + 0.0005, 0.0, z_body + 0.034), (0.003, 0.026, 0.016), METER_LCD, collide=False),
        BoxShape((x - METER_R + 0.0002, 0.0, z_body + 0.034), (0.003, 0.030, 0.020), (0.03, 0.03, 0.03),
                 collide=False),
        CylShape((x - METER_R + 0.0005, 0.0, z_body + 0.016), 0.004, 0.003, METER_BLUE, collide=False, axis="X"),
        CylShape((x, 0, z_body + METER_BODY / 2), METER_R, METER_BODY, None),  # its collider
    ]


TAP = Body.build("tap", pos=(*TAP_XY, GROUND), mass=20.0, kinematic=True, shapes=_pillar_shapes())

# -- the nozzle --------------------------------------------------------------------------------------------------------
# Nozzle frame N: origin on the barrel's axis above the handle, +x out along the barrel to the spray head, +z up from
# the handle to the barrel, so the handle hangs toward -z. The trigger hangs in front of the handle and swings about
# +y (positive: squeezed toward the handle).

HEAD_X = (0.105, 0.140)
HEAD_R = 0.026
OUTLET = np.array([HEAD_X[1] + 0.001, 0.0, 0.0])  # the flood outlet, centre of the dial's face
HANDLE_TOP = np.array([-0.004, 0.0, -0.008])
HANDLE_BOTTOM = np.array([-0.030, 0.0, -0.105])
COUPLING_END = HANDLE_BOTTOM + 0.026 * (HANDLE_BOTTOM - HANDLE_TOP) / np.linalg.norm(HANDLE_BOTTOM - HANDLE_TOP)
TRIGGER_HINGE = np.array([0.022, 0.0, -0.020])
TRIGGER_TIP = np.array([0.033, 0.0, -0.094])
TRIGGER_STOP_DEG = 22.0
TRIGGER_PRELOAD_DEG = 12.0  # the return spring's preload, in degrees of its rate
TRIGGER_RATE = 0.022  # N m per degree: 0.26 N m at rest, 0.75 N m (about 11 N at the lever's middle) at the stop
NOZZLE_KG = 0.24
TRIGGER_KG = 0.025
GRASP_X = 0.072  # where the holding hand takes the barrel (nozzle frame): the middle of its flat-sided grip
GRIP_LEN = 0.044
GRIP_HALF = (GRIP_LEN / 2, 0.0150, 0.0140)  # half sizes: along the barrel, across (y_N), and between the flats (z_N)


def _nozzle_shapes():
    barrel = m.lathe([(0.0, -0.022), (0.0125, -0.022), (0.0140, -0.012), (0.0130, 0.020), (0.0122, 0.070),
                      (0.0135, 0.095), (0.0175, HEAD_X[0]), (0.0, HEAD_X[0])], 40)
    barrel = m.transform(barrel, m.Z_TO_X)
    dial = m.lathe([(0.0, HEAD_X[0]), (HEAD_R - 0.002, HEAD_X[0]), (HEAD_R, HEAD_X[0] + 0.003),
                    (HEAD_R, HEAD_X[1] - 0.004), (HEAD_R - 0.003, HEAD_X[1]), (0.0, HEAD_X[1])], 48)
    dial = m.transform(dial, m.Z_TO_X)
    axis = (HANDLE_BOTTOM - HANDLE_TOP) / np.linalg.norm(HANDLE_BOTTOM - HANDLE_TOP)
    handle_path = np.array([HANDLE_TOP + axis * t for t in np.linspace(0, np.linalg.norm(HANDLE_BOTTOM - HANDLE_TOP),
                                                                       12)])
    swell = 1.0 + 0.10 * np.sin(np.linspace(0, math.pi, 12))
    handle = m.superellipse_sweep(handle_path, 0.0150 * swell, 0.0125 * swell, 3.0, 28, up_hint=(0, 1, 0))
    coupling = m.lathe([(0.0, 0.0), (0.0150, 0.0), (0.0165, 0.003), (0.0165, 0.022), (0.0150, 0.026),
                        (0.0, 0.026)], 32)
    rot_c = m.rot_y(math.atan2(-axis[0], -axis[2]) + math.pi)  # its z along the handle's axis, downward
    coupling = m.transform(coupling, rot_c, HANDLE_BOTTOM)
    # the rubber-covered, flat-sided grip on the barrel where the holding hand takes it (flats up and down in N)
    grip_path = np.array([(x, 0.0, 0.0) for x in np.linspace(GRASP_X - GRIP_LEN / 2, GRASP_X + GRIP_LEN / 2, 6)])
    grip = m.superellipse_sweep(grip_path, GRIP_HALF[1], GRIP_HALF[2], 4.0, 32, up_hint=(0, 0, 1))
    shapes = [
        MeshShape(*_np(barrel), NOZZLE_BODY, collide=False, smooth=True),
        MeshShape(*_np(grip), NOZZLE_RUBBER, collide=False, smooth=True),
        BoxShape((GRASP_X, 0.0, 0.0), (GRIP_LEN, 2 * GRIP_HALF[1], 2 * GRIP_HALF[2]), None),
        MeshShape(*_np(dial), NOZZLE_DIAL, collide=False, smooth=True),
        MeshShape(*_np(handle), NOZZLE_RUBBER, collide=False, smooth=True),
        MeshShape(*_np(coupling), BRASS, collide=False, smooth=True),
        # the dial's knurled ribs and the pattern window
        *[BoxShape((HEAD_X[0] + 0.0175, (HEAD_R + 0.0005) * math.cos(a), (HEAD_R + 0.0005) * math.sin(a)),
                   (0.022, 0.003, 0.003), NOZZLE_RUBBER, collide=False,
                   quat_xyzw=(math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))) for a in np.linspace(0, 2 * math.pi, 24,
                                                                                               endpoint=False)],
        CylShape((HEAD_X[1] + 0.0005, 0.0, 0.0), 0.0050, 0.001, (0.02, 0.02, 0.02), collide=False, axis="X"),
        # colliders
        CylShape((0.040, 0.0, 0.0), 0.0135, 0.125, None, axis="X"),
        CylShape(((HEAD_X[0] + HEAD_X[1]) / 2, 0.0, 0.0), HEAD_R, HEAD_X[1] - HEAD_X[0], None, axis="X"),
        BoxShape(tuple(((HANDLE_TOP + HANDLE_BOTTOM) / 2).tolist()), (0.030, 0.026, 0.100), None,
                 quat_xyzw=(0.0, math.sin(-math.atan2(axis[0], -axis[2]) / 2), 0.0,
                            math.cos(-math.atan2(axis[0], -axis[2]) / 2))),
        CylShape(tuple((HANDLE_BOTTOM + axis * 0.013).tolist()), 0.0165, 0.026, None,
                 quat_xyzw=(0.0, math.sin(-math.atan2(axis[0], -axis[2]) / 2), 0.0,
                            math.cos(-math.atan2(axis[0], -axis[2]) / 2))),
    ]
    return shapes


def _trigger_shapes():
    """The trigger in the nozzle's frame, relative to its hinge (the trigger body's origin is on the hinge)."""
    tip = TRIGGER_TIP - TRIGGER_HINGE
    path = np.array([tip * t + np.array([0.004 * math.sin(math.pi * t), 0, 0]) for t in np.linspace(0, 1, 10)])
    lever = m.superellipse_sweep(path, 0.0055, 0.0075, 3.0, 20, up_hint=(0, 1, 0))
    length = float(np.linalg.norm(tip))
    a = math.atan2(tip[0], -tip[2])
    return [
        MeshShape(*_np(lever), TRIGGER_METAL, collide=False, smooth=True),
        CylShape((0, 0, 0), 0.004, 0.018, TRIGGER_METAL, collide=False, axis="Y"),
        BoxShape(tuple((tip / 2).tolist()), (0.011, 0.015, length), None,
                 quat_xyzw=(0.0, math.sin(-a / 2), 0.0, math.cos(-a / 2))),
    ]


# The nozzle starts lying on its left side on the patio (its y_N up), turned NOZZLE_YAW_DEG about the vertical from
# pointing straight ahead: the barrel points back and to the left, the handle back and to the right.
NOZZLE_YAW_DEG = 120.0
GRASP_XY0 = (0.300, 0.150)  # the barrel where the holding hand takes it, at the start
NOZZLE_ROT0 = m.rot_z(math.radians(NOZZLE_YAW_DEG)) @ np.column_stack([(1.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                                                                        (0.0, -1.0, 0.0)])
NOZZLE_POS0 = np.array([*GRASP_XY0, GROUND + HEAD_R + 0.0005]) - NOZZLE_ROT0 @ np.array([0.075, 0.0, 0.0])
NOZZLE = Body.build("nozzle", pos=tuple(NOZZLE_POS0), mass=NOZZLE_KG, friction=(1.0, 0.9), shapes=_nozzle_shapes(),
                    start_xyzw=kit.quat_from_matrix(NOZZLE_ROT0), com=(0.035, 0.0, -0.030), solver_iterations=32)
TRIGGER = Body.build("trigger", pos=tuple(NOZZLE_POS0 + NOZZLE_ROT0 @ TRIGGER_HINGE), mass=TRIGGER_KG,
                     friction=(1.0, 0.9), shapes=_trigger_shapes(), start_xyzw=kit.quat_from_matrix(NOZZLE_ROT0),
                     ignore=("nozzle",), solver_iterations=32)
kit.joint("trigger_pivot", "nozzle", "trigger", anchor=tuple(TRIGGER.pos), frame=tuple(map(tuple, NOZZLE_ROT0)),
          free=("rotY",), limits_deg=(("rotY", 0.0, TRIGGER_STOP_DEG),),
          stiffness=(("rotY", TRIGGER_RATE),), damping=(("rotY", 0.0004),),
          target_deg=(("rotY", -TRIGGER_PRELOAD_DEG),))

# -- the hose --------------------------------------------------------------------------------------------------------

HOSE_R = 0.0110  # 5/8 inch garden hose, 22 mm outside
LINK = 0.040
HOSE_KG_PER_M = 0.35  # vinyl hose full of water
HOSE_BEND = 0.010  # N m per degree at each link joint: the hose's bending stiffness (EI about 0.02 N m^2)


def _hose_path() -> np.ndarray:
    """The hose's centre line as it lies at the start: down from the tap's nose, a loop over the patio and into the
    nozzle's coupling."""
    nose = np.array([TAP_XY[0] + SPOUT_X, TAP_XY[1], GROUND + HOSE_IN_Z - 0.006])  # the meter's outlet
    low = GROUND + HOSE_R
    start = NOZZLE_POS0 + NOZZLE_ROT0 @ COUPLING_END
    start[2] = max(start[2], low)
    out = NOZZLE_ROT0 @ (COUPLING_END - HANDLE_BOTTOM)
    out = np.array([out[0], out[1], 0.0]) / np.linalg.norm(out[:2])
    ctrl = [
        nose,
        nose + np.array([0.0, 0.0, -0.045]),
        np.array([nose[0] + 0.030, nose[1] - 0.030, low]),
        np.array([0.650, 0.420, low]),
        np.array([0.700, 0.340, low]),
        np.array([0.620, 0.300, low]),
        np.array([0.480, 0.320, low]),
        np.array([0.340, 0.340, low]),
        np.array([0.210, 0.300, low]),
        np.array([0.150, 0.180, low]),
        np.array([0.150, 0.040, low]),
        np.array([*(start[:2] + 0.060 * out[:2]), low]),
        start,
    ]
    ctrl = np.array(ctrl)
    # a smooth curve through the control points (Catmull-Rom), resampled at equal arc length
    pts = []
    for i in range(len(ctrl) - 1):
        p0, p1, p2, p3 = ctrl[max(i - 1, 0)], ctrl[i], ctrl[i + 1], ctrl[min(i + 2, len(ctrl) - 1)]
        for t in np.linspace(0, 1, 30, endpoint=False):
            pts.append(0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t
                              + (-p0 + 3 * p1 - 3 * p2 + p3) * t ** 3))
    pts.append(ctrl[-1])
    pts = np.array(pts)
    pts[2:, 2] = np.maximum(pts[2:, 2], low)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.r_[0, np.cumsum(seg)]
    n = int(round(s[-1] / LINK))
    return np.column_stack([np.interp(np.linspace(0, s[-1], n + 1), s, pts[:, k]) for k in range(3)])


def _frame_along(d) -> np.ndarray:
    x = np.asarray(d, float) / np.linalg.norm(d)
    up = np.array([0.0, 0.0, 1.0]) if abs(x[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    y = np.cross(up, x)
    y /= np.linalg.norm(y)
    return np.column_stack([x, y, np.cross(x, y)])


HOSE_NODES = _hose_path()
HOSE_LINKS: list[Body] = []


def _build_hose():
    nodes = HOSE_NODES
    for i in range(len(nodes) - 1):
        a, b = nodes[i], nodes[i + 1]
        length = float(np.linalg.norm(b - a))
        rot = _frame_along(b - a)
        shapes = [CylShape((0, 0, 0), HOSE_R, length, None, axis="X")]  # drawn by the skin (hoseskin.py)
        HOSE_LINKS.append(Body.build(f"hose_{i:02d}", pos=tuple((a + b) / 2), mass=HOSE_KG_PER_M * length,
                                     shapes=shapes, start_xyzw=kit.quat_from_matrix(rot), friction=(0.9, 0.8),
                                     damping=(0.05, 0.3), solver_iterations=24))
    bend = (("rotY", HOSE_BEND), ("rotZ", HOSE_BEND), ("rotX", HOSE_BEND * 2))
    damp = (("rotY", 0.0006), ("rotZ", 0.0006), ("rotX", 0.0006))
    kit.joint("hose_tap", "tap", HOSE_LINKS[0].name, anchor=tuple(nodes[0]),
              frame=tuple(map(tuple, _frame_along(nodes[1] - nodes[0]))), stiffness=bend, damping=damp)
    for i in range(1, len(HOSE_LINKS)):
        kit.joint(f"hose_j{i:02d}", HOSE_LINKS[i - 1].name, HOSE_LINKS[i].name, anchor=tuple(nodes[i]),
                  frame=tuple(map(tuple, _frame_along(nodes[i + 1] - nodes[i - 1]))), stiffness=bend, damping=damp)
    kit.joint("hose_nozzle", "nozzle", HOSE_LINKS[-1].name, anchor=tuple(nodes[-1]),
              frame=tuple(map(tuple, _frame_along(nodes[-1] - nodes[-2]))),
              stiffness=(("rotY", HOSE_BEND), ("rotZ", HOSE_BEND)), damping=damp)  # a swivel: free to spin


_build_hose()

# -- effects: the stream (in world coordinates, under a fixed holder at the origin) ------------------------------------

FX = Body.build("fx", pos=(0.0, 0.0, 0.0), mass=1.0, kinematic=True,
                shapes=[BoxShape((0.0, 0.0, -0.5), (0.01, 0.01, 0.01), None, collide=False)])
def _hose_start_skin():
    import hoseskin

    pts, side = hoseskin.centre_line([lk.pos for lk in HOSE_LINKS], [kit.rotation(lk.start_xyzw) for lk in HOSE_LINKS],
                                     np.linalg.norm(np.diff(HOSE_NODES, axis=0), axis=1))
    return hoseskin.skin(pts, side, HOSE_R)


kit.dynamic_mesh("hose_skin", "fx", HOSE_GREEN, initial=lambda: _hose_start_skin()[0])
kit.dynamic_mesh("hose_stripe", "fx", HOSE_STRIPE, initial=lambda: _hose_start_skin()[1])
kit.dynamic_mesh("stream", "fx", STREAM)
kit.dynamic_mesh("foam", "fx", FOAM)  # where the stream strikes the water
kit.dynamic_mesh("puddles", "fx", WET)  # wet patches where water has landed on the ground

# -- splash protection on the arms: a clear polyethylene sleeve over each link and the base ---------------------------
# Visual only (no collision, no mass): each sleeve is a rounded box SLEEVE_GAP larger than the link's own visual
# geometry, authored under the link so it moves with it. The fingers, the wrist cameras' lenses and the cable
# exits are left uncovered, as on the real arms.

SLEEVE_LINKS = ("base", "link1", "link2", "link3", "link4", "link5", "gripper")
SLEEVE_GAP = 0.006


def _sleeve_mesh(lo, hi):
    lo, hi = np.asarray(lo, float) - SLEEVE_GAP, np.asarray(hi, float) + SLEEVE_GAP
    size = hi - lo
    axis = int(np.argmax(size))
    a, b = [k for k in range(3) if k != axis]
    unit = np.eye(3)
    centre = (lo + hi) / 2
    path = np.array([centre + unit[axis] * t for t in np.linspace(-size[axis] / 2, size[axis] / 2, 9)])
    # loose plastic: the section swells a little between the ends
    swell = 1.0 + 0.04 * np.sin(np.linspace(0, math.pi, len(path)))
    return m.superellipse_sweep(path, size[a] / 2 * swell, size[b] / 2 * swell, 3.0, 28, up_hint=unit[a])


def _author_sleeves(stage, robot_path, links):
    import isaaclab.sim as sim_utils
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt

    from yam_arena.station import linear_rgb

    look = f"{robot_path}/SleeveLook"
    surface = kit.material_cfg(SLEEVE, linear_rgb)
    surface.func(look, surface)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    for name in SLEEVE_LINKS:
        if name not in links:
            continue
        link = stage.GetPrimAtPath(robot_path + links[name])
        box = None
        for child in link.GetChildren():
            if child.HasAPI(UsdPhysics.RigidBodyAPI) or child.GetName().startswith("sleeve"):
                continue
            r = cache.ComputeRelativeBound(child, link).ComputeAlignedRange()
            if not r.IsEmpty():
                box = r if box is None else box.UnionWith(r)
        if box is None:
            continue
        pts, faces = _sleeve_mesh(box.GetMin(), box.GetMax())
        geom = UsdGeom.Mesh.Define(stage, f"{link.GetPath()}/sleeve")
        geom.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
        geom.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
        geom.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.reshape(-1).astype(np.int32)))
        geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        geom.CreateDoubleSidedAttr(True)
        geom.CreateExtentAttr([Gf.Vec3f(*map(float, pts.min(0))), Gf.Vec3f(*map(float, pts.max(0)))])
        sim_utils.bind_visual_material(str(geom.GetPath()), look, stage=stage)


kit.ROBOT_AUTHORS.append(_author_sleeves)

OBJECTS = [TRAY, PATIO, BUCKET, TAP, NOZZLE, TRIGGER, *HOSE_LINKS, FX]
