"""Set a probe positioner on a wafer probe station: slide it until its needle points at the die, hold it still as the other hand throws its magnet lever to ON, then trim the tip onto the mark with its X and Y knobs.

A TS150-class platen (33 x 23.5 cm, R125 opening) 12 cm above the bench, a chuck with a 150 mm wafer 28 mm below its top, a microscope lens above the die mark. Positioner 1 (an MP40-class
unit, 65 x 40 mm base, clamped) holds a needle whose tip is 2 mm past the mark. Positioner 2 stands loose, its magnet lever OFF. Its base is a free body on the platen; its needle sits on an X and a Y stage, each
driven by a knob (0.3 mm of tip travel a turn, estimate). The scorer reads the tip against the mark the way a technician reads it on the microscope camera.

    python demokit.py task.py OUT --no-record --mode full,none,one_hand,unheld,no_trim,touch
    python demokit.py task.py OUT          (final: 640 x 360, the GPU to yourself)
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = ("Set the loose probe positioner up on the marked die. Its magnet is off. Slide it along the platen until its needle points at the die with the tip near the mark, never touching the other needle, "
               "the wafer or the microscope. Keep it from shifting while you push its magnet lever to ON. Then turn its X and Y knobs to bring the tip over the mark, short of the other probe's tip. Let go.")
SUCCESS_HOLD_S = 0.0
SCORE_OVER = "end"
TOP = d.TABLE_TOP
MM = 1e-3

DX = float(os.environ.get("PP_DX", "0.06"))  # the whole station is set this far out in x (reach)
SX = 0.36 + DX  # the die mark and the chuck's centre, x (y = 0)
PZ = TOP + 0.12  # platen top
PLATEN_T = 0.012
CZ = PZ - 0.028  # chuck top
WAFER_T = 0.0007
WZ = CZ + WAFER_T  # wafer top
TIPZ = WZ + 0.001  # needle tips 1 mm above the wafer
PL_X, PL_Y, R_OPEN = 0.166, 0.1175, 0.125  # platen half length and half width, opening radius
BASE = (0.040, 0.065, 0.100)  # MP40 footprint: 40 along x, 65 along y; 100 mm tall (estimate)
REACH = 0.144  # base centre to needle tip, stages centred (estimate)
ARM_Z, BEND_X = 0.040, 0.080  # the arm's height above the platen, and where the needle leaves it (frame, x)
TIP_Z_REL = TIPZ - PZ
BASE_M, NEEDLE_M = 0.6, 0.05  # kg (estimates)
MU = float(os.environ.get("PP_MU", "0.3"))  # base on platen (estimate; nickel on mild steel 0.178 to 0.64, Engineering Toolbox)
MAGNET_N = float(os.environ.get("PP_MAGNET_N", "343.0"))  # downward pull of the magnet at ON: first4magnets 19206 rated 70 kg (686 N) pull-off on thick steel, taken at half for the 12.7 mm bar
MM_PER_TURN = 0.3  # tip travel per knob turn
TRAVEL = 0.0065  # each stage: 13 mm in all
KSIGN_X = float(os.environ.get("PP_KSX", "1"))  # + knob degrees advance the tip along +x (X stage), and along +y (Y stage)
KSIGN_Y = float(os.environ.get("PP_KSY", "1"))
LEVER_ON_DEG = 62.0  # the lever swings 70 degrees to its stop (estimate); past 62 it is ON
LEVER_PIVOT = (-0.012, BASE[1] / 2 + 0.004, 0.045)  # frame coordinates, on the +y face
LEVER_LEN = 0.030
LEVER_N = float(os.environ.get("PP_LEVER_N", "3.0"))  # push at the lever knob, N (estimate; MIL-STD-1472H finger band 2.8 to 11 N)
TAB = (0.008, 0.020, 0.016)
STUB = 0.014  # the knob stands this far out of the arm (so 18 mm of closed fingers clear the face)
KNOB_R, KNOB_H, KNOB_Z = 0.008, 0.012, 0.060
KNOB_INERTIA = (1.669e-6,) * 3  # kg m^2: what the 12 mm hub and its stripe gave (mechkit guesses a link's from its mesh, so the long thimble would raise it five-fold)
MARK = np.array([SX, 0.0])
P1_X = SX + 0.002 + REACH  # positioner 1's base centre; its tip stands 2 mm past the mark
P1_TIP = np.array([SX + 0.002, 0.0])
START_Y = 0.065
START_X = SX - REACH  # base centre x: the tip then stands on the mark's line in x
LENS_R, LENS_Z0 = 0.022, WZ + 0.04

LOOKS = {"steel": m.Look(m.rgb(0.70, 0.71, 0.74), metallic=0.85, roughness=0.35), "nickel": m.Look(m.rgb(0.80, 0.81, 0.83), metallic=0.45, roughness=0.3), "mark": m.Look(m.rgb(0.90, 0.10, 0.10), roughness=0.5),
         "lever": m.Look(m.rgb(0.80, 0.15, 0.12), roughness=0.4), "knob": m.Look(m.rgb(0.95, 0.62, 0.10), metallic=0.2, roughness=0.4), "needle": m.Look(m.rgb(0.90, 0.90, 0.88), metallic=0.5, roughness=0.2),
         "tip": m.Look(m.rgb(0.30, 0.30, 0.33), metallic=1.0, roughness=0.25), "wafer": m.Look(m.rgb(0.86, 0.88, 0.92), metallic=0.4, roughness=0.15), "grid": m.Look(m.rgb(0.42, 0.45, 0.52), metallic=0.6, roughness=0.3),
         "chuck": m.Look(m.rgb(0.66, 0.67, 0.70), metallic=0.5, roughness=0.35), "lens": m.Look(m.rgb(0.06, 0.07, 0.10), metallic=0.3, roughness=0.05), "bench": m.Look(m.rgb(0.22, 0.23, 0.25), metallic=0.5, roughness=0.5),
         "alu": m.Look(m.rgb(0.84, 0.85, 0.87), metallic=0.5, roughness=0.3), "anod": m.Look(m.rgb(0.06, 0.06, 0.07), metallic=0.1, roughness=0.55), "blue": m.Look(m.rgb(0.07, 0.15, 0.42), metallic=0.6, roughness=0.35),
         "dark": m.Look(m.rgb(0.02, 0.02, 0.025), roughness=0.7), "pointer": m.Look(m.rgb(0.95, 0.95, 0.95), roughness=0.5), "green": m.Look(m.rgb(0.10, 0.65, 0.20), roughness=0.5),
         "scope": m.Look(m.rgb(0.10, 0.10, 0.11), metallic=0.4, roughness=0.5), "scope2": m.Look(m.rgb(0.42, 0.43, 0.46), metallic=0.6, roughness=0.45)}


def _box(size, c):
    return m.box(size, c)


def _cone(a, b, r0: float, r1: float, segments: int = 12) -> m.Mesh:
    """A solid cone from a (radius r0) to b (radius r1), for a needle's taper."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    L = float(np.linalg.norm(b - a))
    mesh = m.lathe([(0.0, 0.0), (r0, 0.0), (r1, L), (0.0, L)], segments)
    z, dvec = np.array([0.0, 0.0, 1.0]), (b - a) / L
    ax = np.cross(z, dvec)
    s, c = float(np.linalg.norm(ax)), float(z @ dvec)
    if s > 1e-9:
        k = ax / s
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        R = np.eye(3) + s * K + (1 - c) * K @ K
        mesh = m.Mesh(mesh.v @ R.T, mesh.f)
    return mesh.moved(a)


def _ribbed(r: float, length: float, axis: str, n: int = 18) -> m.Mesh:
    """A knurled thimble: a core with n thin ribs round it, centred on the origin, along `axis`."""
    parts = [m.cylinder(r * 0.93, length, segments=2 * n)]
    for k in range(n):
        parts.append(_box((r * 0.14, 0.0009, length), (r * 0.93, 0.0, 0.0)).turned(k * 360.0 / n, "z"))
    mesh = m.join(*parts)
    return mesh.turned(90.0, "y") if axis == "x" else mesh.turned(-90.0, "x") if axis == "y" else mesh


def _platen_half(side: int, z1: float, t: float) -> m.Mesh:
    """One end of the platen: a plate t thick, top at z1, from its outer end to the R125 opening's arc, which is cut smooth."""
    ys = np.linspace(-PL_Y, PL_Y, 61)
    xi = side * np.sqrt(np.maximum(R_OPEN**2 - ys**2, 0.0))
    xo, z0 = side * PL_X, z1 - t
    v, f = [], []

    def quad(pts, hint):
        ids = []
        for p in pts:
            v.append(p)
            ids.append(len(v) - 1)
        n = np.cross(np.subtract(pts[1], pts[0]), np.subtract(pts[2], pts[0]))
        if n @ np.asarray(hint, float) < 0:
            ids = ids[::-1]
        f.extend([(ids[0], ids[1], ids[2]), (ids[0], ids[2], ids[3])])

    for i in range(len(ys) - 1):
        ya, yb, xa, xb = ys[i], ys[i + 1], xi[i], xi[i + 1]
        quad([(xo, ya, z1), (xa, ya, z1), (xb, yb, z1), (xo, yb, z1)], (0, 0, 1))
        quad([(xo, ya, z0), (xa, ya, z0), (xb, yb, z0), (xo, yb, z0)], (0, 0, -1))
        quad([(xo, ya, z0), (xo, yb, z0), (xo, yb, z1), (xo, ya, z1)], (side, 0, 0))
        quad([(xa, ya, z0), (xb, yb, z0), (xb, yb, z1), (xa, ya, z1)], (-(xa + xb), -(ya + yb), 0))
    for y, sgn in ((-PL_Y, -1), (PL_Y, 1)):
        quad([(xo, y, z0), (xi[0 if sgn < 0 else -1], y, z0), (xi[0 if sgn < 0 else -1], y, z1), (xo, y, z1)], (0, sgn, 0))
    return m.Mesh(np.array(v, float), np.array(f))


# -- the station ----------------------------------------------------------------------------------------------------------------------------------------------------------------


def _station() -> m.Part:
    parts, cols = [], []
    # the platen: a 10 mm nickel-plated steel plate (visual), with the R125 opening cut round and smooth; its colliders stay 12 mm, in 10 mm steps
    zt = PZ - TOP
    for k, side in enumerate((-1, 1)):
        parts.append((f"platen{k}", _platen_half(side, zt, 0.010), "nickel"))
    step_c = 0.01
    ys = np.arange(-PL_Y + step_c / 2, PL_Y, step_c)
    for yc in ys:
        hc = math.sqrt(max(R_OPEN**2 - yc**2, 0.0))
        for side in (-1, 1):
            x0, x1 = (-PL_X, -hc) if side < 0 else (hc, PL_X)
            cols.append(m.col_box(f"pc{len(cols)}", (x1 - x0, step_c, PLATEN_T), ((x0 + x1) / 2, yc, zt - PLATEN_T / 2)))
    for side in (-1, 1):  # colliders of the platen's end blocks (as before)
        sz, c = (0.025, 2 * PL_Y, zt - PLATEN_T), (side * (PL_X - 0.0125), 0.0, (zt - PLATEN_T) / 2)
        cols.append(m.col_box(f"legc{(side+1)//2}", sz, c))
    # the platen's frame: four legs and an apron under the plate
    lz = zt - 0.010
    for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        parts.append((f"leg{i}", _box((0.022, 0.022, lz), (sx * (PL_X - 0.011), sy * (PL_Y - 0.011), lz / 2)), "bench"))
    for sy in (-1, 1):
        parts.append((f"rail{(sy+1)//2}", _box((2 * PL_X - 0.044, 0.012, 0.018), (0.0, sy * (PL_Y - 0.011), lz - 0.009)), "bench"))
    for sx in (-1, 1):
        parts.append((f"rail_e{(sx+1)//2}", _box((0.012, 2 * PL_Y - 0.044, 0.018), (sx * (PL_X - 0.011), 0.0, lz - 0.009)), "bench"))
    # the chuck column, the chuck and the wafer
    col_h = CZ - TOP - 0.02
    parts.append(("column", m.cylinder(0.05, col_h, (0, 0, col_h / 2)), "bench"))
    cols.append(m.col_cylinder("column_c", 0.05, col_h, (0, 0, col_h / 2)))
    parts.append(("chuck", m.cylinder(0.08, 0.02, (0, 0, CZ - TOP - 0.01)), "chuck"))
    cols.append(m.col_cylinder("chuck_c", 0.08, 0.02 + WAFER_T, (0, 0, CZ - TOP - 0.01 + WAFER_T / 2)))
    wt = CZ - TOP + WAFER_T  # the wafer's top
    parts.append(("wafer", m.cylinder(0.075, WAFER_T, (0, 0, CZ - TOP + WAFER_T / 2), segments=64), "wafer"))
    # a faint die grid (10 mm dies, the marked die centred on the mark) and the mark
    n = 0
    for c in range(-7, 8):
        off = 0.005 + 0.010 * c if c >= 0 else -0.005 + 0.010 * (c + 1)
        half = math.sqrt(max(0.074**2 - off**2, 0.0))
        if half > 0.004:
            parts.append((f"gx{n}", _box((0.00015, 2 * half, 0.0002), (off, 0.0, wt + 0.0001)), "grid"))
            parts.append((f"gy{n}", _box((2 * half, 0.00015, 0.0002), (0.0, off, wt + 0.0001)), "grid"))
            n += 1
    parts.append(("cross0", _box((0.008, 0.0006, 0.0004), (0, 0, wt + 0.0002)), "mark"))
    parts.append(("cross1", _box((0.0006, 0.008, 0.0004), (0, 0, wt + 0.0002)), "mark"))
    parts.append(("dot", m.cylinder(0.0009, 0.0004, (0, 0, wt + 0.0002)), "mark"))
    # the microscope: a bridge on two posts at y = +/-0.20, a zoom body over it with a camera head, and the objective 4 cm above the wafer
    top_b = PZ - TOP + 0.10
    for side in (-1, 1):
        parts.append((f"post{(side+1)//2}", m.cylinder(0.015, top_b, (0, side * 0.20, top_b / 2)), "scope2"))
        cols.append(m.col_cylinder(f"postc{(side+1)//2}", 0.015, top_b, (0, side * 0.20, top_b / 2)))
        parts.append((f"foot{(side+1)//2}", _box((0.07, 0.07, 0.008), (0, side * 0.20, 0.004)), "scope2"))
    bsz, bc = (0.04, 0.43, 0.02), (0.0, 0.0, top_b + 0.01)
    parts.append(("bridge", _box(bsz, bc), "scope2"))
    cols.append(m.col_box("bridge_c", bsz, bc))
    lz0 = LENS_Z0 - TOP
    lh = top_b - lz0
    parts.append(("objective", m.cylinder(0.018, 0.026, (0, 0, lz0 + 0.013)), "scope"))  # the objective's barrel
    parts.append(("obj_ring", m.cylinder(0.0215, 0.006, (0, 0, lz0 + 0.029)), "alu"))
    parts.append(("obj_glass", m.cylinder(0.0125, 0.0006, (0, 0, lz0 + 0.0003)), "lens"))
    parts.append(("nosepiece", m.cylinder(0.022, lh - 0.032, (0, 0, lz0 + 0.032 + (lh - 0.032) / 2)), "scope"))
    cols.append(m.col_cylinder("lens_c", LENS_R, lh, (0, 0, lz0 + lh / 2)))
    z0 = top_b + 0.02  # the zoom body stands on the bridge: a body, a ribbed zoom ring, a focus knob, and the camera head on top
    parts.append(("zoom_body", m.cylinder(0.019, 0.05, (0, 0, z0 + 0.025)), "scope"))
    parts.append(("zoom_ring", _ribbed(0.023, 0.012, "z").moved((0, 0, z0 + 0.018)), "alu"))
    parts.append(("focus_knob", _ribbed(0.010, 0.008, "y").moved((0.0, 0.028, z0 + 0.034)), "alu"))
    parts.append(("cam_mount", m.cylinder(0.014, 0.008, (0, 0, z0 + 0.054)), "alu"))
    parts.append(("cam_head", _box((0.032, 0.032, 0.035), (0, 0, z0 + 0.0795)), "scope2"))
    parts.append(("cam_lens_ring", m.cylinder(0.008, 0.004, (0.0, -0.018, z0 + 0.0795), axis="y"), "lens"))
    return m.fixed("station", (SX, 0.0, TOP), parts, cols, LOOKS, friction=(MU, MU))


# -- a positioner, as visuals: an MP40-class unit (65 x 40 mm base plate, three stacked slide blocks, two micrometer heads, a magnet lever, a probe arm and a tungsten needle) -------------------------------


def _needle_parts(prefix: str, look: str):
    """The arm, the needle holder and the needle of a positioner, in its frame (x along the needle, z above the platen): visual parts and colliders. The colliders are the arm's and needle's lines."""
    head = (BEND_X - 0.046, 0.0, ARM_Z)
    bend = (BEND_X, 0.0, ARM_Z)
    tip = (REACH, 0.0, TIP_Z_REL)
    arm_mesh = m.tube([head, bend], 0.003)
    nd = np.array(tip) - np.array(bend)
    nd /= np.linalg.norm(nd)
    shank_end = np.array(tip) - 0.007 * nd
    parts = [(f"{prefix}arm", _box((0.046, 0.010, 0.010), (BEND_X - 0.023 + 0.005, 0.0, ARM_Z)), "alu"),  # the 10 x 10 mm probe arm, 10 cm from the base's rear face to the holder
             (f"{prefix}clamp", _box((0.014, 0.016, 0.022), (BASE[0] / 2 + 0.027, 0.0, ARM_Z)), "anod"),  # the clamp where the needle arm joins the head
             (f"{prefix}clamp_screw", m.cylinder(0.003, 0.004, (BASE[0] / 2 + 0.027, 0.0, ARM_Z + 0.013)), "alu"),
             (f"{prefix}holder", _box((0.012, 0.010, 0.014), (BEND_X + 0.004, 0.0, ARM_Z - 0.003)), "anod"),
             (f"{prefix}screw", m.cylinder(0.003, 0.005, (BEND_X + 0.004, 0.0, ARM_Z + 0.0095)), "alu"),
             (f"{prefix}shank", m.tube([np.array(bend) + np.array([0.003, 0.0, -0.004]), shank_end], 0.0007, sides=8), "needle"),
             (f"{prefix}tip", _cone(shank_end, tip, 0.0007, 0.00008, 8), "tip")]
    cols = [m.col_hull(f"{prefix}armc", arm_mesh)]
    n = 4
    pts = [np.array(bend) + (np.array(tip) - np.array(bend)) * k / n for k in range(n + 1)]
    for k in range(n):
        cols.append(m.col_hull(f"{prefix}nc{k}", m.tube([pts[k], pts[k + 1]], 0.0006, sides=8)))
    return parts, cols


def _stack() -> list:
    """The base plate and the three slide blocks, with their dovetail lines and reveals, the lever's pivot and its ON and OFF marks."""
    bx, by = BASE[0], BASE[1]
    parts = [("core", _box((0.036, 0.060, 0.090), (0, 0, 0.055)), "dark"), ("baseplate", _box((bx, by, 0.010), (0, 0, 0.005)), "anod"),
             ("block_a", _box((bx, by, 0.027), (0, 0, 0.025)), "alu"), ("block_b", _box((bx, by, 0.030), (0, 0, 0.055)), "blue"), ("block_c", _box((0.036, 0.058, 0.0285), (0, 0, 0.08575)), "anod")]
    # dovetail lines: two dark rails along each block's +x and +y faces (and -y), so the blocks read as slides
    for nm, (z0, z1, fx, fy) in {"a": (0.0115, 0.0385, bx / 2, by / 2), "b": (0.040, 0.070, bx / 2, by / 2), "c": (0.0715, 0.100, 0.018, 0.029)}.items():
        for k, zf in enumerate((0.3, 0.7)):
            z = z0 + zf * (z1 - z0)
            parts.append((f"dv{nm}x{k}", _box((0.0004, 2 * fy - 0.004, 0.0009), (fx + 0.0001, 0.0, z)), "dark"))
            parts.append((f"dv{nm}y{k}", _box((2 * fx - 0.004, 0.0004, 0.0009), (0.0, fy + 0.0001, z)), "dark"))
            parts.append((f"dv{nm}n{k}", _box((2 * fx - 0.004, 0.0004, 0.0009), (0.0, -fy - 0.0001, z)), "dark"))
    parts.append(("z_barrel", m.cylinder(0.0065, 0.016, (0.0, 0.0, 0.108)), "alu"))  # the Z micrometer on top: barrel, graduation rings, ribbed thimble
    parts.append(("z_grad0", m.cylinder(0.00665, 0.0003, (0.0, 0.0, 0.106)), "dark"))
    parts.append(("z_grad1", m.cylinder(0.00665, 0.0003, (0.0, 0.0, 0.110)), "dark"))
    parts.append(("z_thimble", _ribbed(0.0075, 0.010, "z").moved((0.0, 0.0, 0.121)), "anod"))
    pv = np.array(LEVER_PIVOT)
    parts.append(("pivot", m.cylinder(0.006, 0.004, (pv[0], by / 2 + 0.002, pv[2]), axis="y"), "alu"))
    parts.append(("on_mark", _box((0.005, 0.0006, 0.005), (pv[0] + LEVER_LEN - 0.006, by / 2 + 0.0002, pv[2] - 0.010)), "green"))
    parts.append(("off_mark", _box((0.005, 0.0006, 0.005), (pv[0] - 0.009, by / 2 + 0.0002, pv[2] + 0.018)), "pointer"))
    return parts


def _micrometers() -> list:
    """The static barrels of the X (on the rear face) and Y (on the -y face) micrometer heads: silver, 15 mm across, with graduation rings and a collar."""
    out = []
    kx = np.array([-BASE[0] / 2, 0.0, KNOB_Z])
    ky = np.array([0.0, -BASE[1] / 2, KNOB_Z])
    out.append(("collar_x", m.cylinder(0.0095, 0.002, (-0.001, 0, 0), axis="x").moved(kx), "anod"))
    out.append(("barrel_x", m.cylinder(0.0072, 0.0262, (-0.0131, 0, 0), axis="x").moved(kx), "alu"))
    out.append(("collar_y", m.cylinder(0.0095, 0.002, (0, -0.001, 0), axis="y").moved(ky), "anod"))
    out.append(("barrel_y", m.cylinder(0.0072, 0.0262, (0, -0.0131, 0), axis="y").moved(ky), "alu"))
    for k, off in enumerate((0.006, 0.0095, 0.013, 0.0165, 0.020)):
        out.append((f"grad_x{k}", m.cylinder(0.00735, 0.0003, (-off, 0, 0), axis="x").moved(kx), "dark"))
        out.append((f"grad_y{k}", m.cylinder(0.00735, 0.0003, (0, -off, 0), axis="y").moved(ky), "dark"))
    return out


def _lever_meshes(alpha_deg: float):
    """The magnet lever (bar and knob tab) at angle alpha above +x: 90 is OFF (up), 0 is ON (forward)."""
    pv = np.array(LEVER_PIVOT)
    dirv = np.array([math.cos(math.radians(alpha_deg)), 0.0, math.sin(math.radians(alpha_deg))])
    end = pv + LEVER_LEN * dirv + np.array([0.0, STUB, 0.0])
    bar = m.tube([pv, pv + LEVER_LEN * dirv, end], 0.003)
    return bar, end


def _carriage() -> list:
    """The Y stage's carriage and the X stage's head, in front of the blocks (static on positioner 1)."""
    return [("plate", _box((0.006, 0.05, 0.02), (BASE[0] / 2 + 0.003, 0, ARM_Z)), "blue")], ("head", _box((0.014, 0.03, 0.025), (BASE[0] / 2 + 0.013, 0, ARM_Z)), "alu")


def _p1() -> m.Part:
    plate, head = _carriage()
    bar, end = _lever_meshes(0.0)
    tab = _box((TAB[2], TAB[1], TAB[0]), tuple(end))  # lever ON: pointing forward, the knob tab turned with it
    kx = np.array([-BASE[0] / 2, 0.0, KNOB_Z])
    ky = np.array([0.0, -BASE[1] / 2, KNOB_Z])
    thx = ("thimble_x", _ribbed(0.0080, 0.0070, "x").moved(kx + np.array([-0.0290, 0, 0])), "anod")
    thy = ("thimble_y", _ribbed(0.0080, 0.0070, "y").moved(ky + np.array([0, -0.0290, 0])), "anod")
    p, c = _needle_parts("n_", "needle")
    parts = _stack() + _micrometers() + plate + [head, ("lever", bar, "lever"), ("lever_tab", tab, "lever"), thx, thy] + p
    cols = [m.col_box("body_c", BASE, (0, 0, BASE[2] / 2))] + c
    return m.free("pos1", (P1_X, 0.0, PZ + 0.0003), parts, cols, LOOKS, mass=BASE_M, friction=(MU, MU), torsional_patch=None, solver_iters=64, yaw_deg=180.0)


def _p2() -> m.Part:
    body = _stack() + _micrometers()
    body_c = [m.col_box("body_c", BASE, (0, 0, BASE[2] / 2))]
    # the stages: Y (across) carries X (along the needle), which carries the head, the arm and the needle
    plate, head = _carriage()
    ystage = m.slider("stagey", "Y", -TRAVEL, TRAVEL, plate, [m.col_box("plate_c", (0.006, 0.05, 0.02), (BASE[0] / 2 + 0.003, 0, ARM_Z))],
                      mass=0.03, spring=2.0e4, damping=60.0, effort=200.0, friction=0.0)
    np_parts, np_cols = _needle_parts("n_", "needle")
    xstage = m.slider("stagex", "X", -TRAVEL, TRAVEL, [head] + np_parts, [m.col_box("head_c", (0.014, 0.03, 0.025), (BASE[0] / 2 + 0.013, 0, ARM_Z))] + np_cols, mass=NEEDLE_M, spring=2.0e4, damping=60.0,
                      effort=200.0, friction=0.0)
    xstage = m.Link(**{**xstage.__dict__, "parent": "stagey"})
    # the lever: on the +y face, 45 mm up, a 30 mm arm with a knob tab; OFF points up (+z), ON points forward (+x): a quarter turn about y
    pv = np.array(LEVER_PIVOT)
    bar, end = _lever_meshes(90.0)
    tab = _box(TAB, tuple(end))  # the knob: a flat tab 8 mm thick along the push, 20 mm across, 16 mm along the arm
    lv = m.Link("lever", "hinge", "Y", tuple(pv), -5.0, 72.0, (("bar", bar, "lever"), ("tab", tab, "lever")), (m.col_hull("lever_bar_c", bar), m.col_box("lever_tab_c", TAB, tuple(end))), mass=0.02, spring=3.0,
                damping=0.02, friction=0.0, effort=LEVER_N * LEVER_LEN)
    # the X knob on the rear face (-x), the Y knob on the -y face: a ribbed 16 mm thimble on a silver barrel, with a pointer stripe on its end
    kx_at = np.array([-BASE[0] / 2, 0.0, KNOB_Z])
    ky_at = np.array([0.0, -BASE[1] / 2, KNOB_Z])
    hub_x = m.cylinder(KNOB_R, KNOB_H, (-KNOB_H / 2, 0, 0), axis="x").moved(kx_at)  # the collider's hull
    hub_y = m.cylinder(KNOB_R, KNOB_H, (0, -KNOB_H / 2, 0), axis="y").moved(ky_at)
    thim_x = _ribbed(0.0080, 0.0070, "x").moved(kx_at + np.array([-0.0290, 0, 0]))
    thim_y = _ribbed(0.0080, 0.0070, "y").moved(ky_at + np.array([0, -0.0290, 0]))
    stripe_x = _box((0.0008, 0.0035, 0.015), (-KNOB_H - 0.0004, 0, 0)).moved(kx_at)
    stripe_y = _box((0.0035, 0.0008, 0.015), (0, -KNOB_H - 0.0004, 0)).moved(ky_at)
    knobx = m.Link("knobx", "hinge", "X", tuple(kx_at), -3600.0, 3600.0, (("hub", thim_x, "anod"), ("stripe", stripe_x, "pointer")), (m.col_hull("knobx_c", hub_x),), mass=0.03, spring=0.0, damping=0.05, friction=0.02, inertia=KNOB_INERTIA)
    knoby = m.Link("knoby", "hinge", "Y", tuple(ky_at), -3600.0, 3600.0, (("hub", thim_y, "anod"), ("stripe", stripe_y, "pointer")), (m.col_hull("knoby_c", hub_y),), mass=0.03, spring=0.0, damping=0.05, friction=0.02, inertia=KNOB_INERTIA)
    return m.mechanism("pos2", (START_X, START_Y, PZ + 0.0003), [ystage, xstage, lv, knobx, knoby], LOOKS, frame_parts=body, frame_colliders=body_c, friction=(MU, MU), solver_iters=64,
                       free_mass=BASE_M, frame_inertia=(BASE_M * 0.0025, BASE_M * 0.0030, BASE_M * 0.0016))


OBJECTS = [_station(), _p1(), _p2()]

# -- geometry readings ----------------------------------------------------------------------------------------------------------------------------------------------------------

TIP_REL = np.array([REACH, 0.0, TIP_Z_REL])
POLY_REL = np.array([(BEND_X - 0.046, 0.0, ARM_Z), (BEND_X, 0.0, ARM_Z), (REACH, 0.0, TIP_Z_REL)])  # head, bend, tip


def poly_world(st, sx: float = 0.0, sy: float = 0.0) -> np.ndarray:
    """The arm and needle of a positioner as a polyline in the world (a stage moves all of it)."""
    return st.to_world(POLY_REL + np.array([sx, sy, 0.0]))


def tip_world(st, sx: float = 0.0, sy: float = 0.0) -> np.ndarray:
    return poly_world(st, sx, sy)[-1]


def seg_dist(p0, p1, q0, q1) -> float:
    """The least distance between two segments."""
    u, v, w = p1 - p0, q1 - q0, p0 - q0
    a, b, c, dd, e = u @ u, u @ v, v @ v, u @ w, v @ w
    den = a * c - b * b
    s = np.clip((b * e - c * dd) / den, 0.0, 1.0) if den > 1e-12 else 0.0
    t = (b * s + e) / c if c > 1e-12 else 0.0
    if t < 0.0:
        t, s = 0.0, np.clip(-dd / a, 0.0, 1.0)
    elif t > 1.0:
        t, s = 1.0, np.clip((b - dd) / a, 0.0, 1.0)
    return float(np.linalg.norm((p0 + s * u) - (q0 + t * v)))


def poly_dist(A, B) -> float:
    return min(seg_dist(A[i], A[i + 1], B[j], B[j + 1]) for i in range(len(A) - 1) for j in range(len(B) - 1))


def pt_poly_dist(P, A) -> np.ndarray:
    out = np.full(len(P), 1e9)
    for i in range(len(A) - 1):
        a, b = A[i], A[i + 1]
        t = np.clip(((P - a) @ (b - a)) / ((b - a) @ (b - a)), 0.0, 1.0)
        out = np.minimum(out, np.linalg.norm(P - (a + t[:, None] * (b - a)), axis=1))
    return out


def box_dist(p, centre, half) -> np.ndarray:
    q = np.abs(np.atleast_2d(p) - centre) - half
    return np.linalg.norm(np.maximum(q, 0.0), axis=1)


def heading_deg(st) -> float:
    """The needle's heading in plan (degrees from +x), and the base centre to mark's direction."""
    R = st.rotation()
    return math.degrees(math.atan2(R[1, 0], R[0, 0]))


def tier(g: dict) -> float:
    """0.25: upright (under 2 deg), the needle within 5 deg of the line to the mark, the tip within 4 mm of it in plan. 0.5: lever ON, the tip within 4 mm of the mark when it latched, and the tip moved under 0.5 mm across the throw.
    0.75: the tip within 0.3 mm of the mark in plan, the knobs turned at least 90 degrees in all after the clamp, the base moved under 0.5 mm from the clamp to the end. 1.0: grippers clear of it, all still for 1 s, positioner 1 unmoved, the lever still ON. 0: a touch of a needle, the wafer or the lens."""
    if g["touch"]:
        return 0.0
    t = 0.0
    if g["tilt"] < 2.0 and abs(g["head_err"]) < 5.0 and g["tip_err"] < 4 * MM:
        t = 0.25
        if g["lever_on"] and g["throw_shift"] < 0.5 * MM and g["tip_err_clamp"] < 4 * MM:
            t = 0.5
            if g["tip_err"] < 0.3 * MM and g["knob_turn"] >= 90.0 and g["base_move"] < 0.5 * MM:
                t = 0.75
                if g["clear"] and g["still"] and g["p1_moved"] < 1.0 * MM:
                    t = 1.0
    return t


class Trial:
    def reset(self, env=None):
        self.n = 0
        self.touch = ""
        self.final = 0.0
        self.last = {}
        self.tip_before = None  # the tip while the lever is still OFF
        self.throw_shift = 0.0
        self.p1_start = None
        self.quiet = 0
        self.prev = None
        self.tgt = 0.0
        self.clamped = False
        self.sx = 0.0
        self.sy = 0.0
        self.thrown = False
        self.clamp_pose = None
        self.tip_err_clamp = 1e9
        self.k0 = (0.0, 0.0)

    def step(self, env):
        st = m.state(env, "pos2")
        p1 = m.state(env, "pos1")
        self.n += 1
        q_l = float(st.joint("lever"))  # radians, positive toward ON
        # the lever's friction: a drive (spring 3.0 N m/rad) that trails the joint by a slack, so it holds LEVER_N at the knob (3 N: 0.09 N m)
        slack = LEVER_N * LEVER_LEN / 3.0
        if q_l - self.tgt > slack:
            self.tgt = q_l - slack
        elif self.tgt - q_l > slack:
            self.tgt = q_l + slack
        m.set_target(env, "pos2", "lever", self.tgt)
        # the knobs drive the stages: 0.3 mm of tip travel a turn
        kx, ky = st.deg("knobx"), st.deg("knoby")
        self.sx = float(np.clip(KSIGN_X * kx / 360.0 * MM_PER_TURN * MM, -TRAVEL, TRAVEL))
        self.sy = float(np.clip(KSIGN_Y * ky / 360.0 * MM_PER_TURN * MM, -TRAVEL, TRAVEL))
        m.set_target(env, "pos2", "stagex", self.sx)
        m.set_target(env, "pos2", "stagey", self.sy)
        lever_deg = math.degrees(q_l)
        on = self.clamped or lever_deg >= LEVER_ON_DEG  # the lever latches at ON (its lock reads a hair under 62 degrees)
        if on and not self.clamped:
            self.clamped = True
            m.lock(env, "pos2", "lever")
            self.clamp_pose = np.asarray(st.pos, float).copy()
            self.k0 = (float(kx), float(ky))
            self.tip_err_clamp = float(np.linalg.norm(tip_world(st, float(st.joint("stagex")), float(st.joint("stagey")))[:2] - MARK))
        # the magnet: a downward pull at ON, so that the base's friction holds it
        m.push(env, "pos2", (0.0, 0.0, -MAGNET_N if on else 0.0), body="frame")
        if self.n == 1:
            m.push(env, "pos1", (0.0, 0.0, -MAGNET_N), body=None)
            self.p1_start = np.asarray(p1.pos, float).copy()
        sxr = float(st.joint("stagex"))
        syr = float(st.joint("stagey"))
        poly2 = poly_world(st, sxr, syr)
        poly1 = poly_world(p1)
        tip = poly2[-1]
        # the tip across the lever's throw
        if lever_deg < 3.0:
            self.tip_before = tip.copy()
        elif self.tip_before is not None and not self.thrown:
            self.throw_shift = max(self.throw_shift, float(np.linalg.norm((tip - self.tip_before)[:2])))
            self.thrown = on  # the throw ends as the lever reaches ON
        # touches: needle on needle, needle or tip on the wafer, on the lens, a fingertip on a needle or arm, the wafer or the lens
        tips = m.fingertips(env)
        why = ""
        if poly_dist(poly2, poly1) < 0.0006:
            why = "needle on needle"
        elif min(poly2[-1][2], p1.to_world(POLY_REL)[-1][2]) < WZ + 0.0002:
            why = "needle on the wafer"
        lens_c = np.array([SX, 0.0])
        for P in (poly2, poly1):
            for a, b in zip(P[:-1], P[1:]):
                for tt in np.linspace(0.0, 1.0, 40):
                    q = a + tt * (b - a)
                    if np.hypot(*(q[:2] - lens_c)) < LENS_R + 0.0005 and q[2] > LENS_Z0 - 0.0005:
                        why = why or "needle on the lens"
        near = min(float(pt_poly_dist(tips, poly2).min()), float(pt_poly_dist(tips, poly1).min()))
        if near < 0.006:
            why = why or "gripper on a needle"
        if (np.hypot(tips[:, 0] - SX, tips[:, 1]) < 0.09).any() and (tips[:, 2] < WZ + 0.006).any():
            why = why or "gripper on the wafer"
        if (np.hypot(tips[:, 0] - SX, tips[:, 1]) < LENS_R + 0.004)[tips[:, 2] > LENS_Z0 - 0.004].any():
            why = why or "gripper on the lens"
        if why and not self.touch:
            self.touch = why
            print(f"[pp] TOUCH {why} at step {self.n}", flush=True)
        # the readings
        tip_err = float(np.linalg.norm(tip[:2] - MARK))
        gap1 = float(np.linalg.norm(tip[:2] - tip_world(p1)[:2]))
        to_mark = MARK - np.asarray(st.pos, float)[:2]
        head_err = heading_deg(st) - math.degrees(math.atan2(to_mark[1], to_mark[0]))
        head_err = (head_err + 180.0) % 360.0 - 180.0
        # clear: every fingertip 3 cm from the positioner's box and from its needle
        c = st.to_local(tips)
        bx = box_dist(c, np.array([-0.005, 0.0125, 0.055]), np.array([0.030, 0.050, 0.055]))
        clear = bool((bx >= 0.03).all() and pt_poly_dist(tips, poly2).min() >= 0.03)
        cur = np.r_[np.asarray(st.pos, float), kx, ky, lever_deg]
        moved = 1e9 if self.prev is None else float(np.linalg.norm(cur[:3] - self.prev[:3]))
        dang = 1e9 if self.prev is None else float(np.abs(cur[3:] - self.prev[3:]).max())
        self.prev = cur
        still = bool(moved < 0.0002 and dang < 0.3)
        self.quiet = self.quiet + 1 if still else 0
        g = {"touch": self.touch, "tilt": st.tilt_deg(), "head_err": head_err, "tip_err": tip_err, "lever_on": bool(on), "throw_shift": self.throw_shift, "gap1": gap1, "clear": clear,
             "still": self.quiet >= 20, "tip_err_clamp": self.tip_err_clamp, "knob_turn": (abs(kx - self.k0[0]) + abs(ky - self.k0[1])) if self.clamped else 0.0,
             "base_move": float(np.linalg.norm(np.asarray(st.pos, float) - self.clamp_pose)) if self.clamp_pose is not None else 1e9, "p1_moved": float(np.linalg.norm(np.asarray(p1.pos, float) - self.p1_start)) if self.p1_start is not None else 0.0}
        self.final = tier(g)
        self.last = {"tier": self.final, **{k: (round(v, 5) if isinstance(v, float) else v) for k, v in g.items()}, "tip": [round(float(v), 5) for v in tip], "lever_deg": round(lever_deg, 1), "dbox": round(float(bx.min()), 3), "dneedle": round(float(pt_poly_dist(tips, poly2).min()), 3), "kx": round(kx, 1), "ky": round(ky, 1)}
        if self.n % 10 == 0:
            print("[pp] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), "n": self.n, **self.last}), flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return float(TRIAL.final)


def success(objects) -> bool:
    return bool(TRIAL.final >= 1.0)


# -- the scripted demo ------------------------------------------------------------------------------------------------------------------------------------------------------------

def _frame(ctx):
    s = ctx.state("pos2")
    return np.asarray(s.pos, float), s


def _tip(ctx):
    s = ctx.state("pos2")
    return tip_world(s, float(s.joint("stagex")), float(s.joint("stagey")))


def env_of(ctx):
    return ctx.emb._env


def demo_script(ctx):
    """Modes: full. none: the arms wait. one_hand: the left hand alone throws the lever, the base unslid and unheld. unheld: the base slid to the die, then let go before the lever
    is thrown, nobody holding it. no_trim: slide, hold and clamp, the knobs untouched, the tip 1.5 mm short. touch: the whole act, then the right hand's fingers brush the needle.

    full: the right hand slides the base by a pinch on its top, closing on the needle tip's reading. HOLD "press": the right lets go and presses the base's top down onto the platen;
    the left throws the magnet lever, stopping the moment it reads ON, then turns the X knob in part turns, reading the tip after each; the left then presses the top while the right
    turns the Y knob. HOLD "pinch": the right keeps its pinch through the throw. Each knob is turned with the base held down by the other hand (or, with "pinch", by the magnet alone).
    The lever is on the base's +y face and the top press of the left hand would stand in the right hand's way behind it, so the left throws the lever (reach, README)."""
    mode = ctx.mode
    if mode not in ("full", "none", "one_hand", "no_trim", "touch", "unheld"):
        raise ValueError(f"unknown mode {mode!r}")
    L, R = ctx.left, ctx.right
    if mode == "none":
        yield from L.wait(3.0)
        return
    hold_z = PZ + float(os.environ.get("PP_HZ", "0.092"))
    hold_grip = d.tilted(float(os.environ.get("PP_HT", "45")), 90.0)  # right hand, leaning toward +y, fingers closing along x
    hold_off = -float(os.environ.get("PP_HB", "0.012")) * hold_grip.rot[:, 2]
    press_n = float(os.environ.get("PP_PN", "8"))
    press = HOLD == "press" and mode not in ("one_hand", "unheld")

    # 1. slide: the right hand pinches the base's top and slides it along the platen, closing on the tip's reading
    if mode != "one_hand":
        b = ctx.pos("pos2")
        pin = np.array([b[0], b[1], hold_z]) + hold_off
        yield from R.move(pin + np.array([0.0, -0.03, 0.10]), grip=hold_grip, finger=d.FINGER_OPEN, label="right over the base")
        yield from R.move(pin, t=1.4, label="right down on the base")
        yield from R.close(BASE[0], float(os.environ.get("PP_GRIP", "40")), label="right grips the base")
        off = R.tcp()[:2] - ctx.pos("pos2")[:2]
        yield from R.move(np.r_[np.array([SX - REACH, 0.0]) + off, hold_z + hold_off[2]], t=3.0, label="slide the base to the die")
        tgt_tip = MARK + np.array([{"no_trim": -0.0015}.get(mode, -0.0003), -0.0003])  # a hair short of the mark on both axes: the trim then ends on forward turns
        herr = 0.0
        for k in range(10):  # half steps on the tip's own reading (the microscope view), each let settle: a base on friction lags the hand, then creeps on
            s = ctx.state("pos2")
            e = tgt_tip - _tip(ctx)[:2]
            to_mark = MARK - np.asarray(s.pos, float)[:2]
            herr = (heading_deg(s) - math.degrees(math.atan2(to_mark[1], to_mark[0])) + 180.0) % 360.0 - 180.0
            if np.linalg.norm(e) < 0.0003 and abs(herr) < 2.0:
                break
            yield from R.move(R.at() + np.r_[e * SLIDE_GAIN, 0.0], t=0.4, label="trim the slide")
            yield from R.wait(1.0)
        print(f"[pp] slid: base {np.round(ctx.pos('pos2')[:2] * 1000, 2)} mm, tip {np.round(_tip(ctx)[:2] * 1000, 2)} mm, heading err {herr:.2f} deg", flush=True)
        if mode == "unheld" or press:
            t0 = _tip(ctx)[:2].copy()
            yield from R.open(t=0.4)
            yield from R.move(R.at() + np.array([0.0, -0.02, 0.03]), t=0.6, label="right lets go")
            print(f"[pp] let go: tip moved {np.linalg.norm(_tip(ctx)[:2] - t0) * 1000:.2f} mm", flush=True)

    # 2. hold: the right hand presses the base's top down (not a pinch)
    def press_on(arm, grip, at, label):
        top = lambda: ctx.pos("pos2") + np.array([at[0], at[1], BASE[2] + 0.014])
        yield from arm.move(top() + np.array([0.0, 0.0, 0.03]) - 0.03 * grip.rot[:, 2], grip=grip, finger=d.FINGER_CLOSED, t=1.0, label=f"{label}: over the top")
        yield from arm.move(top() + np.array([0.0, 0.0, 0.012]), t=0.8, label=f"{label}: down")
        t0 = _tip(ctx)[:2].copy()
        yield from arm.press((0.0, 0.0, -1.0), force=press_n, max_travel=0.03, speed=0.03, settle=0.4, timeout=3.0, label=f"{label} presses the base")
        print(f"[pp] {label} pressing: tip {np.round(_tip(ctx)[:2] * 1000, 2)} mm, moved {np.linalg.norm(_tip(ctx)[:2] - t0) * 1000:.2f} mm", flush=True)

    if press:
        yield from press_on(R, hold_grip, (float(os.environ.get("PP_PAX", "0.0")), float(os.environ.get("PP_PAY", "-0.010"))), "right")

    # 3. the left hand throws the lever, from behind, and stops the moment it reads ON
    pivot = ctx.pos("pos2") + np.array(LEVER_PIVOT)
    r0 = float(os.environ.get("PP_LR", "0.022"))
    th = float(os.environ.get("PP_LTH", "45"))
    g_l = d.tilted(th, 0.0)
    ax_l = np.array([math.sin(math.radians(th)), 0.0, -math.cos(math.radians(th))])

    def lever_hand(alpha_deg: float):
        ball = pivot + np.array([0.0, STUB, 0.0]) + LEVER_LEN * np.array([math.cos(math.radians(alpha_deg)), 0.0, math.sin(math.radians(alpha_deg))])
        return ball - r0 * ax_l + np.array([0.0, float(os.environ.get("PP_LYB", "0")), float(os.environ.get("PP_LZB", "0"))])

    p0 = lever_hand(90.0)
    yield from L.move(p0 - 0.06 * ax_l, grip=g_l, finger=d.FINGER_CLOSED, label="left behind the lever")
    yield from L.servo(p0 - 0.06 * ax_l, timeout=1.5, label="left behind the lever")
    yield from L.move(p0, t=1.0, label="left to the lever's knob")
    yield from L.servo(p0, tol=0.002, timeout=2.0, label="left at the knob")
    lead = float(os.environ.get("PP_LLEAD", "8"))
    for k in range(60):  # push the knob round its arc, a little ahead of where it reads; stop the moment it latches ON
        if TRIAL.clamped:
            break
        yield from L.follow(lambda: lever_hand(90.0 - min(ctx.deg("pos2", "lever") + lead + 2.0 * k, 100.0)), t=0.2, speed=0.12, until=lambda: TRIAL.clamped, label="left pushes the lever")
    print(f"[pp] lever {ctx.deg('pos2', 'lever'):.1f} deg ON {TRIAL.clamped}; tip {np.round(_tip(ctx)[:2] * 1000, 2)} mm, throw shift {TRIAL.throw_shift * 1000:.2f} mm", flush=True)
    yield from L.move(L.at() - 0.03 * ax_l, t=0.4, label="left off the lever")
    yield from L.move(L.at() + np.array([0.0, 0.07, 0.04]), t=0.8, label="left backs off")
    yield from L.move(L.at() + np.array([-0.05, 0.0, 0.0]), t=0.6)
    if mode == "one_hand":
        yield from L.wait(3.0)
        print("[pp] END " + json.dumps({"mode": mode, **TRIAL.last}), flush=True)
        return
    if not press and mode != "unheld":  # the pinch lets go once the magnet holds
        yield from R.wait(0.3)
        yield from R.open(t=0.4)
        yield from R.move(R.at() + np.array([0.0, -0.02, 0.10]), t=0.8, label="right lifts away")
    if mode in ("unheld", "no_trim"):
        if press:
            yield from R.move(R.at() + np.array([0.0, -0.03, 0.10]), t=0.8, label="right lifts away")
        yield from R.wait(3.0)
        print("[pp] END " + json.dumps({"mode": mode, **TRIAL.last}), flush=True)
        return

    # 4. trim: the left turns the X knob (the right pressing, with "press"), then the right the Y knob (the left pressing)
    kx_pos = lambda: ctx.pos("pos2") + np.array([-BASE[0] / 2 - KNOB_H + 0.002 - 0.005, 0.0, KNOB_Z])
    ky_pos = lambda: ctx.pos("pos2") + np.array([0.0, -BASE[1] / 2 - KNOB_H / 2 - 0.004, KNOB_Z])
    yield from _trim(ctx, L, "x", kx_pos, mode)
    if True:
        if press:
            yield from press_on(L, d.tilted(float(os.environ.get("PP_LPT", "45")), 0.0), (float(os.environ.get("PP_LPX", "-0.008")), 0.012), "left")
            yield from R.move(R.at() + np.array([0.0, -0.05, 0.08]), t=0.8, label="right lifts off the top")
        rp, rr = d.ik.tcp_pose(np.array(d.READY, dtype=float), R.base)  # via the ready pose: a solve seeded from the top press takes the wrong branch to the Y knob
        yield from R.move(rp, grip=d._as_grip(rr), finger=d.FINGER_OPEN, t=1.4, label="right through its ready pose")
        yield from R.move(ky_pos() + np.array([0.0, -0.10, 0.04]), grip=d.level(90.0), finger=d.FINGER_OPEN, t=1.2, label="right round to the Y knob")
        yield from _trim(ctx, R, "y", ky_pos, mode)
        if abs(MARK[0] - _tip(ctx)[0]) > 0.00018 and not press:  # the Y turns may have nudged x: once more on X
            yield from _trim(ctx, L, "x", kx_pos, mode)
    if mode == "touch":  # the act done, then the right hand's fingers brush the positioner's needle
        mid = lambda: (poly_world(ctx.state("pos2"), float(ctx.state("pos2").joint("stagex")), float(ctx.state("pos2").joint("stagey")))[-2:].mean(axis=0))
        yield from R.move(mid() + np.array([0.0, -0.03, 0.05]), grip=d.level(90.0), finger=d.FINGER_OPEN, t=1.5, label="right over the needle")
        yield from R.move(mid() + np.array([0.0, 0.0, 0.004]), t=1.0, label="right brushes the needle")
        yield from R.wait(0.5)
    # 5. let go: every hand clear
    rp, rr = d.ik.tcp_pose(np.array(d.READY, dtype=float), R.base)
    yield from R.move(rp, grip=d._as_grip(rr), finger=d.FINGER_OPEN, t=1.4, label="right clear")
    yield from L.move(L.at() + np.array([-0.06, 0.03, 0.10]), t=1.0, label="left clear")
    yield from L.wait(1.5)
    print("[pp] END " + json.dumps({"mode": mode, **TRIAL.last}), flush=True)


def _trim(ctx, arm, which, knob_pos, mode):
    """Turn one knob in part turns of at most KDEG degrees until the tip's error along that axis is under 0.1 mm, reading the tip after each."""
    ax = 0 if which == "x" else 1
    if which == "x":
        grip, axis, ksign, appr = d.level(0.0, KFLIP["x"]), (1.0, 0.0, 0.0), KSIGN_X, np.array([-0.07, 0.0, 0.0])
    else:
        flip = KFLIP["y"] if os.environ.get("PP_YFLIP", "") != "auto" else bool((MARK[1] - _tip(ctx)[1]) * KSIGN_Y > 0)
        grip, axis, ksign, appr = d.level(90.0, flip), (0.0, 1.0, 0.0), KSIGN_Y, np.array([0.0, -0.07, 0.0])
    tgt = MARK[ax]
    yield from arm.move(knob_pos() + appr, grip=grip, finger=d.FINGER_OPEN, label=f"{which} knob: over")
    yield from arm.move(knob_pos(), t=1.0, label=f"{which} knob: in")
    first = 0.0
    for it in range(int(os.environ.get("PP_KIT", "24"))):
        e = tgt - _tip(ctx)[ax]
        if abs(e) < 0.0001:
            break
        if first and e * first < 0 and abs(e) < 0.00025:  # overshot but inside the 0.3 mm tier: a turn back drags the base, so stop
            break
        first = first or e
        deg = float(np.clip(KGAIN * e / (MM_PER_TURN * MM) * 360.0, -KDEG, KDEG))
        if abs(deg) < 20.0:
            deg = math.copysign(20.0, deg)
        pre = -0.5 * deg * ksign * KTURN[which] * PREROLL  # wind the open wrist back half a turn first, so the turn runs either side of its rest (the wrist's roll limit)
        if which == "y" and deg * ksign > 0:  # a +Y turn starts from rest: wound back first, the open fingers lose the knob
            pre = 0.0
        if pre:
            yield from arm.turn(pre, axis=axis, t=max(0.4, abs(pre) / 120.0), label=f"{which} knob: wind back")
        yield from arm.close(KNOB_R * 2, 20.0, t=0.3, label=f"{which} knob: grip")
        yield from arm.turn(deg * ksign * KTURN[which], axis=axis, t=max(0.5, abs(deg) / 120.0), label=f"{which} knob: turn")
        yield from arm.wait(0.2)
        print(f"[pp] {which} knob turn {deg:.0f}: tip {np.round(_tip(ctx)[:2] * 1000, 2)} mm, base {np.round(ctx.pos('pos2')[:2] * 1000, 2)}", flush=True)
        yield from arm.open(t=0.25)
        yield from arm.turn(-deg * ksign * KTURN[which] - pre, axis=axis, t=max(0.5, abs(deg) / 120.0), label=f"{which} knob: wrist back")
    yield from arm.open(t=0.25)
    yield from arm.move(arm.at() + appr, t=0.8, label=f"{which} knob: away")


KTURN = {"x": float(os.environ.get("PP_KTX", "1")), "y": float(os.environ.get("PP_KTY", "1"))}
KFLIP = {"x": False, "y": False}
HOLD = os.environ.get("PP_HOLD", "press")
KDEG = float(os.environ.get("PP_KDEG", "150"))
KGAIN = float(os.environ.get("PP_KGAIN", "1.0"))
SLIDE_GAIN = float(os.environ.get("PP_SG", "0.5"))
PREROLL = float(os.environ.get("PP_PRE", "1"))
