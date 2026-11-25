"""Lobby dustpan front v1e (carry_dump1e): preloaded bimanual handoff at clear-of-bin station + raised rim-clear transit +
steep rear-recess dump (and LD_MODE='handoff_repinch_coupon' no-dump 2-core qualification coupon per Codex 2308/2310/2322/2324).

Fixes over carry_dump1d (35de177a, preserved untouched):
  1. SEAM 1a — Station moved clear of wastebasket walls: HANDOFF_R_TARGET shifted from [0.355, -0.120, 0.920] to
     [0.365, -0.175, 0.960] (-55 mm Y away from wb_wall_minusY=+0.055 m, +10 mm X away from wb_wall_minusX=+0.2075 m,
     +40 mm Z), and p_l_high_prebin shifted in -Y from [0.360, -0.065, 0.942] to [0.360, -0.160, 0.945] so both inbound
     lift and outbound reverse exit clear the rim before/after any Z change.
  2. SEAM 1b — Measured Left load BEFORE Right release: after Left pinches L_GRIP_S=0.365 m (step 230..240), Left lifts
     by PRELOAD_DELTA_L = [+0.026, -0.042, +0.068] m while Right synchronously unloads its arm PD spring to pt_r_held +
     [0, 0, +0.020] m with jaws STILL CLOSED (steps 240..270), transferring ~100% of the 0.90 kg weight onto Left's
     stiffness=10 wrist PD BEFORE Right opens (steps 270..282). Target release drop <= 10 mm, pan->bin >= 20 mm, bin_mv <= 0.5 mm.
  3. Exact front duration preserved: left == right == 37.8 s (756 steps) and transit gate at step 436 (21.8 s), keeping
     repinch_latch_step = 836, tail_latch_step = 974, and integrated total = 62.2 s unchanged for claude-cake's int3.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import labkit  # noqa: E402
from labkit import VISUAL_ONLY, Joint, on_reset, on_step, part  # noqa: E402
from core_contacts import TensorCoreContactMonitor  # noqa: E402
from transfer_evidence import CoreTransfer, TransferSample  # noqa: E402
from yam_arena import YAM_URDF  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.station import FLOOR_Z  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
labkit.floor_bay()
FLOOR = FLOOR_Z  # -0.0125 m

BLACK_PP = (0.08, 0.08, 0.09)
DARK_GREY = (0.18, 0.19, 0.21)
VINYL_HANDLE = (0.11, 0.11, 0.13)
BIN_LLDPE = (0.12, 0.13, 0.15)
KRAFT = (0.63, 0.49, 0.33)

TAPE_YELLOW = (0.86, 0.72, 0.12)
TAPE_WHITE = (0.88, 0.88, 0.89)
SCALE_BLACK = (0.09, 0.09, 0.10)
SCALE_WHITE = (0.93, 0.93, 0.94)
for _vc in (TAPE_YELLOW, TAPE_WHITE, SCALE_BLACK, SCALE_WHITE):
    VISUAL_ONLY.add(_vc)

PAN_W, PAN_D, PAN_H = 0.3239, 0.2858, 0.1270
PAN_T = 0.0035
TRUN_Z = 0.1050
REAR_Y = -0.1650
FRONT_Y = PAN_D + REAR_Y
ASSEMBLED_H = 0.9144
HANDLE_R = 0.0110

RECESS_Y = -0.1860
RECESS_Z = 0.1050
RECESS_W, RECESS_THICK, RECESS_H = 0.110, 0.016, 0.018

YOKE_LOWER = -math.radians(90.0)
YOKE_UPPER = math.radians(8.0)
YOKE_INIT = math.radians(7.0)

PARK_XY = (0.340, -0.180)
BIN_XY = (0.390, +0.185)

BIN_L, BIN_W, BIN_H = 0.3650, 0.2600, 0.3810
BIN_WALL_T, BIN_FLOOR_T = 0.0040, 0.0050

CORE_ID, CORE_OD, CORE_H = 0.0762, 0.0826, 0.0508
CORE_MASS = 0.029

R_GRIP_S = 0.680
L_GRIP_S = 0.365

HANDOFF_R_TARGET = (0.365, -0.180, 0.960)
PRELOAD_DELTA_L = np.array([+0.028, -0.044, +0.086], float)   # claude-cake hr2
UNLOAD_LIFT_R = np.array([0.0, 0.0, +0.008], float)   # claude-cake hr2
HIGH_PREBIN_TARGET = (0.360, -0.160, 0.945)

PAN_PIECE_NAMES = (
    "pan_floor", "pan_rear_wall", "pan_left_wall", "pan_right_wall",
    "pan_wheel_left", "pan_wheel_right",
    "pan_recess_gusset_left", "pan_recess_gusset_right", "pan_recess_bar",
)
HANDLE_PIECE_NAMES = (
    "yoke_arm_left", "yoke_arm_right", "yoke_crossbar", "handle_top_cap", "handle_shaft",
)
WB_PIECE_NAMES = (
    "wb_floor", "wb_wall_minusX", "wb_wall_plusX", "wb_wall_minusY", "wb_wall_plusY",
)


def _pan_boxes():
    wt = PAN_T
    yc = (REAR_Y + FRONT_Y) * 0.5
    side_h = PAN_H
    return [
        ((0.0, yc, 0.0020), (PAN_W, PAN_D, 0.0040), BLACK_PP),
        ((0.0, REAR_Y + 0.0050, side_h * 0.5), (PAN_W, 0.0100, side_h), BLACK_PP),
        ((-PAN_W * 0.5 + wt * 0.5, yc, side_h * 0.5), (wt, PAN_D, side_h), BLACK_PP),
        ((+PAN_W * 0.5 - wt * 0.5, yc, side_h * 0.5), (wt, PAN_D, side_h), BLACK_PP),
        ((-PAN_W * 0.5 + 0.014, REAR_Y + 0.014, 0.014), (0.012, 0.028, 0.028), DARK_GREY),
        ((+PAN_W * 0.5 - 0.014, REAR_Y + 0.014, 0.014), (0.012, 0.028, 0.028), DARK_GREY),
        ((-RECESS_W * 0.5 + 0.008, (REAR_Y + RECESS_Y) * 0.5, RECESS_Z), (0.016, abs(RECESS_Y - REAR_Y), RECESS_H), DARK_GREY),
        ((+RECESS_W * 0.5 - 0.008, (REAR_Y + RECESS_Y) * 0.5, RECESS_Z), (0.016, abs(RECESS_Y - REAR_Y), RECESS_H), DARK_GREY),
        ((0.0, RECESS_Y, RECESS_Z), (RECESS_W, RECESS_THICK, RECESS_H), DARK_GREY),
    ]


def _handle_boxes_and_cyls():
    yoke_x = PAN_W * 0.5 + 0.009
    yoke_top_z = 0.250
    shaft_bot_z = yoke_top_z
    shaft_top_z = ASSEMBLED_H - 0.036
    shaft_len = shaft_top_z - shaft_bot_z
    boxes = [
        ((-yoke_x, 0.0, (TRUN_Z + yoke_top_z) * 0.5), (0.010, 0.020, yoke_top_z - TRUN_Z + 0.020), VINYL_HANDLE),
        ((+yoke_x, 0.0, (TRUN_Z + yoke_top_z) * 0.5), (0.010, 0.020, yoke_top_z - TRUN_Z + 0.020), VINYL_HANDLE),
        ((0.0, 0.0, yoke_top_z), (2 * yoke_x + 0.010, 0.020, 0.016), VINYL_HANDLE),
        ((0.0, 0.0, ASSEMBLED_H - 0.018), (0.032, 0.026, 0.036), DARK_GREY),
    ]
    cyls = [
        ((0.0, 0.0, (shaft_bot_z + shaft_top_z) * 0.5), HANDLE_R, shaft_len, VINYL_HANDLE, "Z"),
    ]
    return boxes, cyls


def _handle_joint() -> Joint:
    boxes, cyls = _handle_boxes_and_cyls()
    return Joint(
        "handle", kind="hinge", axis="X", anchor=(0.0, 0.0, TRUN_Z),
        lower=YOKE_LOWER, upper=YOKE_UPPER, initial=YOKE_INIT, target=YOKE_INIT,
        stiffness=0.0, damping=0.05, friction=0.015, mass=0.30,
        boxes=boxes, cylinders=cyls,
    )


def _wastebasket_boxes():
    wt, ft = BIN_WALL_T, BIN_FLOOR_T
    return [
        ((0.0, 0.0, ft * 0.5), (BIN_L, BIN_W, ft), BIN_LLDPE),
        ((-BIN_L * 0.5 + wt * 0.5, 0.0, BIN_H * 0.5), (wt, BIN_W, BIN_H), BIN_LLDPE),
        ((+BIN_L * 0.5 - wt * 0.5, 0.0, BIN_H * 0.5), (wt, BIN_W, BIN_H), BIN_LLDPE),
        ((0.0, -BIN_W * 0.5 + wt * 0.5, BIN_H * 0.5), (BIN_L - 2 * wt, wt, BIN_H), BIN_LLDPE),
        ((0.0, +BIN_W * 0.5 - wt * 0.5, BIN_H * 0.5), (BIN_L - 2 * wt, wt, BIN_H), BIN_LLDPE),
    ]


def _kraft_core_boxes(n_seg: int = 12):
    r_mid = 0.25 * (CORE_ID + CORE_OD)
    wall = (CORE_OD - CORE_ID) * 0.5
    chord = 2.0 * (CORE_OD * 0.5) * math.sin(math.pi / n_seg)
    out = []
    for k in range(n_seg):
        ang = 2.0 * math.pi * k / n_seg
        cx, cy = r_mid * math.cos(ang), r_mid * math.sin(ang)
        out.append(((cx, cy, 0.0), (wall, chord, CORE_H), KRAFT, ang))
    return out


def _floor_markings():
    px, py = PARK_XY
    bx, by = BIN_XY
    pw, pd, bw, bd = 0.400, 0.360, 0.420, 0.310
    tw, th = 0.014, 0.001
    return [
        ((px, py - pd * 0.5, th * 0.5), (pw, tw, th), TAPE_YELLOW),
        ((px, py + pd * 0.5, th * 0.5), (pw, tw, th), TAPE_YELLOW),
        ((px - pw * 0.5, py, th * 0.5), (tw, pd, th), TAPE_YELLOW),
        ((px + pw * 0.5, py, th * 0.5), (tw, pd, th), TAPE_YELLOW),
        ((bx, by - bd * 0.5, th * 0.5), (bw, tw, th), TAPE_WHITE),
        ((bx, by + bd * 0.5, th * 0.5), (bw, tw, th), TAPE_WHITE),
        ((bx - bw * 0.5, by, th * 0.5), (tw, bd, th), TAPE_WHITE),
        ((bx + bw * 0.5, by, th * 0.5), (tw, bd, th), TAPE_WHITE),
        ((0.220 - 0.025, 0.000, th * 0.6), (0.050, 0.018, th), SCALE_BLACK),
        ((0.220 + 0.025, 0.000, th * 0.6), (0.050, 0.018, th), SCALE_WHITE),
    ]


FLOOR_MARKS = part("floor_marks", pos=(0.0, 0.0, FLOOR), fixed=True, boxes=_floor_markings())
# rev3 tape rule (Jingyi 1610 "yes, tape rule"): parked = all four hopper base corners inside the yellow tape's OUTER edge
TAPE_OUT_HX, TAPE_OUT_HY = 0.400 * 0.5 + 0.014 * 0.5, 0.360 * 0.5 + 0.014 * 0.5     # rendered rectangle centreline + half tape width
BASE_CORNERS = ((-PAN_W * 0.5, REAR_Y, 0.0), (+PAN_W * 0.5, REAR_Y, 0.0), (-PAN_W * 0.5, FRONT_Y, 0.0), (+PAN_W * 0.5, FRONT_Y, 0.0))


def _parked_now(dp_st) -> bool:
    """one sample of the 1.0-tier parking predicate (finite pose; tape footprint, base height, tilt, handle)"""
    try:
        if not np.isfinite(np.asarray(dp_st.pos, float)).all():
            return False
        hdir = _handle_dir_world(dp_st)
        h_vert = math.degrees(math.acos(float(np.clip(hdir[2], -1.0, 1.0))))
        return bool(_tape_margin(dp_st) >= 0.0 and (float(dp_st.pos[2]) - FLOOR) <= 0.015 and dp_st.tilt_deg() <= 10.0 and h_vert <= 15.0)
    except Exception:  # noqa: BLE001
        return False


def _park_update(dp_st):
    """codex 1614: record the parking predicate on every observation; 1.0 needs it on all of the last STILL_N samples"""
    h = _OS.setdefault("park_hist", [])
    h.append(_parked_now(dp_st)); del h[:-STILL_N]


def _tape_margin(dp_st) -> float:
    """smallest margin (m) of the four hopper base corners inside the tape's outer edge (negative = a corner outside)"""
    m = 9.0
    for c in BASE_CORNERS:
        w = np.asarray(dp_st.to_world(c), float)
        m = min(m, TAPE_OUT_HX - abs(float(w[0]) - PARK_XY[0]), TAPE_OUT_HY - abs(float(w[1]) - PARK_XY[1]))
    return m
DUSTPAN = part("dustpan", pos=(PARK_XY[0], PARK_XY[1], FLOOR + 0.001), mass=0.54, friction=0.55,
               boxes=_pan_boxes(), joints=[_handle_joint()])
WASTEBASKET = part("wastebasket", pos=(BIN_XY[0], BIN_XY[1], FLOOR + 0.001), mass=0.74, friction=0.65,
                   boxes=_wastebasket_boxes())
CORE_Z0 = FLOOR + 0.001 + 0.0040 + CORE_H * 0.5 + 0.001
CORE_1 = part("core_1", pos=(PARK_XY[0] - 0.055, PARK_XY[1] - 0.098, CORE_Z0), mass=CORE_MASS, friction=0.35,
              boxes=_kraft_core_boxes())
# rev3 (Jingyi 1525 "mixed debris"): realistic lobby sweepings, rigid colliders, real sizes and masses, distinct colours
BALL_R, BALL_MASS = 0.030, 0.005            # crumpled paper ball, about 60 mm across, 5 g
CAP_R, CAP_H, CAP_MASS = 0.015, 0.012, 0.002  # plastic bottle cap 30 mm x 12 mm, 2 g
CARD_L, CARD_W, CARD_T, CARD_MASS = 0.080, 0.050, 0.003, 0.003   # folded card / receipt 80 x 50 x 3 mm, 3 g
PAPER_WHITE, CAP_BLUE, CARD_GREEN = (0.90, 0.90, 0.87), (0.10, 0.33, 0.78), (0.22, 0.62, 0.38)
PAN_FLOOR_Z = FLOOR + 0.001 + 0.0040


def _paper_ball_mesh():
    """lumpy icosphere (42 vertices, seeded radial jitter +-12 %) collided as its convex hull"""
    t = (1.0 + 5 ** 0.5) / 2.0
    v = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t),
         (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    f = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6),
         (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10),
         (8, 6, 7), (9, 8, 1)]
    pts = [np.asarray(x, float) / np.linalg.norm(x) for x in v]
    mid = {}
    def m(a, b):
        k = (min(a, b), max(a, b))
        if k not in mid:
            q = pts[a] + pts[b]; pts.append(q / np.linalg.norm(q)); mid[k] = len(pts) - 1
        return mid[k]
    tris = []
    for a, b, c in f:
        ab, bc, ca = m(a, b), m(b, c), m(c, a)
        tris += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
    rng = np.random.default_rng(1525)
    P = np.asarray(pts) * (BALL_R * (1.0 + rng.uniform(-0.12, 0.12, (len(pts), 1))))
    return labkit.Mesh(points=P, tris=np.asarray(tris, int), color=PAPER_WHITE, collide="hull")


BALL_LOCAL_LOW = float(-_paper_ball_mesh().points[:, 2].min())
PAPER_BALL = part("paper_ball", pos=(PARK_XY[0] + 0.060, PARK_XY[1] - 0.110, PAN_FLOOR_Z + BALL_LOCAL_LOW + 0.001),
                  mass=BALL_MASS, friction=0.45, meshes=[_paper_ball_mesh()], color=PAPER_WHITE)
BOTTLE_CAP = part("bottle_cap", pos=(PARK_XY[0] + 0.060, PARK_XY[1] - 0.040, PAN_FLOOR_Z + CAP_H * 0.5 + 0.001),
                  mass=CAP_MASS, friction=0.35, cylinders=[((0.0, 0.0, 0.0), CAP_R, CAP_H, CAP_BLUE, "Z")], color=CAP_BLUE)
CARD = part("card", pos=(PARK_XY[0] - 0.050, PARK_XY[1] - 0.020, PAN_FLOOR_Z + CARD_T * 0.5 + 0.001),
            mass=CARD_MASS, friction=0.40, boxes=[((0.0, 0.0, 0.0), (CARD_L, CARD_W, CARD_T), CARD_GREEN)], color=CARD_GREEN)
ITEMS = [CORE_1, PAPER_BALL, BOTTLE_CAP, CARD]
ITEM_NAMES = tuple(it.name for it in ITEMS)
# shape for containment / clearance: ("cyl", r, half_h) along body z, ("sphere", r), ("box", hx, hy, hz)
ITEM_SHAPE = {"core_1": ("cyl", CORE_OD * 0.5, CORE_H * 0.5),
              "paper_ball": ("hull", _paper_ball_mesh().points),
              "bottle_cap": ("cyl", CAP_R, CAP_H * 0.5),
              "card": ("box", CARD_L * 0.5, CARD_W * 0.5, CARD_T * 0.5)}


def _item_ext(name, st, frame_st):
    """half-extents of the item along the axes of frame_st (orientation-aware)"""
    shp = ITEM_SHAPE[name]
    o = np.asarray(frame_st.to_local(st.to_world((0.0, 0.0, 0.0))), float)
    axes = [np.asarray(frame_st.to_local(st.to_world(e)), float) - o for e in ((1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0))]
    if shp[0] == "sphere":
        return np.full(3, shp[1])
    if shp[0] == "hull":
        A = np.stack(axes)
        return np.abs(np.asarray(shp[1], float) @ A).max(axis=0)
    if shp[0] == "cyl":
        ax = axes[2] / max(float(np.linalg.norm(axes[2])), 1e-9)
        return shp[2] * np.abs(ax) + shp[1] * np.sqrt(np.clip(1.0 - ax * ax, 0.0, 1.0))
    A = np.stack([a / max(float(np.linalg.norm(a)), 1e-9) for a in axes])       # rows: body axes in frame coords
    return np.abs(A).T @ np.asarray(shp[1:], float)


def _item_lohi(name, st, frame_st):
    """(lo, hi): signed extents of the item about its centre along the axes of frame_st (asymmetric for the hull)"""
    shp = ITEM_SHAPE[name]
    if shp[0] != "hull":
        e = _item_ext(name, st, frame_st)
        return -e, e
    o = np.asarray(frame_st.to_local(st.to_world((0.0, 0.0, 0.0))), float)
    W = np.asarray([frame_st.to_local(st.to_world(q)) for q in shp[1]], float) - o
    return W.min(axis=0), W.max(axis=0)


def _item_bound_r(name):
    shp = ITEM_SHAPE[name]
    if shp[0] == "hull":
        return float(np.linalg.norm(shp[1], axis=1).max())
    if shp[0] == "sphere":
        return float(shp[1])
    if shp[0] == "cyl":
        return float(math.hypot(shp[1], shp[2]))
    return float(np.linalg.norm(shp[1:]))


def _item_marks(name):
    """two opposite body-frame marks per item for the 5 mm stillness check"""
    shp = ITEM_SHAPE[name]
    if shp[0] == "hull":
        P = np.asarray(shp[1], float)
        return (tuple(P[int(np.argmax(P[:, 0]))]), tuple(P[int(np.argmin(P[:, 0]))]))
    if shp[0] == "sphere":
        return ((shp[1], 0.0, 0.0), (-shp[1], 0.0, 0.0))
    if shp[0] == "cyl":
        return ((shp[1], 0.0, shp[2]), (-shp[1], 0.0, -shp[2]))
    return ((shp[1], shp[2], shp[3]), (-shp[1], -shp[2], -shp[3]))

OBJECTS = [FLOOR_MARKS, DUSTPAN, WASTEBASKET] + ITEMS

_state = {}
_LINK_LOCAL_BOXES: dict[str, tuple[np.ndarray, np.ndarray]] | None = None
_SIGNS_9 = np.array([
    [0.0, 0.0, 0.0],
    [-1, -1, -1], [-1, -1, +1], [-1, +1, -1], [-1, +1, +1],
    [+1, -1, -1], [+1, -1, +1], [+1, +1, -1], [+1, +1, +1],
], float)


def dustpan_to_hollow_bin_nearest_pair(dp_st, wb_st) -> tuple[float, str, str]:
    """Vectorized 70-pair exact 15-axis SAT + analytical support-plane certified lower bound LB [mm] (<0.4 ms/step)."""
    R_wb = wb_st.rotation()
    p_wb = np.asarray(wb_st.pos, float)
    R_dp_in_wb = R_wb.T @ dp_st.rotation()
    p_dp_in_wb = R_wb.T @ (np.asarray(dp_st.pos, float) - p_wb)

    q_h = float(dp_st.joints["handle"])
    Rh = _rx_mat(q_h)
    trun_local = np.array([0.0, 0.0, TRUN_Z])
    R_sh_in_wb = R_dp_in_wb @ Rh
    p_trun_in_wb = p_dp_in_wb + R_dp_in_wb @ trun_local

    hbox, hcyl = _handle_boxes_and_cyls()
    box_names = PAN_PIECE_NAMES + HANDLE_PIECE_NAMES[:4]
    c1_list, R1_list, h1_list = [], [], []
    for c0, sz, _ in _pan_boxes():
        c1_list.append(p_dp_in_wb + R_dp_in_wb @ np.asarray(c0, float))
        R1_list.append(R_dp_in_wb)
        h1_list.append(np.asarray(sz, float) * 0.5)
    for c0, sz, _ in hbox:
        c1_list.append(p_trun_in_wb + R_sh_in_wb @ (np.asarray(c0, float) - trun_local))
        R1_list.append(R_sh_in_wb)
        h1_list.append(np.asarray(sz, float) * 0.5)

    C1_13 = np.asarray(c1_list, float)      # (13, 3)
    R1_13 = np.asarray(R1_list, float)      # (13, 3, 3)
    H1_13 = np.asarray(h1_list, float)      # (13, 3)

    wb_raw = _wastebasket_boxes()
    C2_5 = np.asarray([c0 for c0, _, _ in wb_raw], float)          # (5, 3)
    H2_5 = np.asarray([sz for _, sz, _ in wb_raw], float) * 0.5    # (5, 3)

    C1 = np.repeat(C1_13, 5, axis=0)        # (65, 3)
    R1 = np.repeat(R1_13, 5, axis=0)        # (65, 3, 3)
    H1 = np.repeat(H1_13, 5, axis=0)        # (65, 3)
    C2 = np.tile(C2_5, (13, 1))             # (65, 3)
    H2 = np.tile(H2_5, (13, 1))             # (65, 3)

    T = C1 - C2                             # (65, 3)
    absR = np.abs(R1) + 1e-12               # (65, 3, 3)
    sep = np.any(np.abs(T) > H2 + np.einsum("nij,nj->ni", absR, H1), axis=1)
    TR = np.einsum("ni,nij->nj", T, R1)
    sep |= np.any(np.abs(TR) > H1 + np.einsum("nji,nj->ni", absR, H2), axis=1)
    for i in range(3):
        i1, i2 = (i + 1) % 3, (i + 2) % 3
        for j in range(3):
            j1, j2 = (j + 1) % 3, (j + 2) % 3
            lhs = np.abs(T[:, i2] * R1[:, i1, j] - T[:, i1] * R1[:, i2, j])
            rhs = H2[:, i1] * absR[:, i2, j] + H2[:, i2] * absR[:, i1, j] + H1[:, j1] * absR[:, i, j2] + H1[:, j2] * absR[:, i, j1]
            sep |= lhs > rhs
    if not np.all(sep):
        idx0 = int(np.flatnonzero(~sep)[0])
        return 0.0, box_names[idx0 // 5], WB_PIECE_NAMES[idx0 % 5]

    LO2, HI2 = C2 - H2, C2 + H2
    U0 = _SIGNS_9[None, :, :] * H1[:, None, :]
    X0 = C1[:, None, :] + np.einsum("nkj,nij->nki", U0, R1)
    RES0 = np.maximum(0.0, X0 - HI2[:, None, :]) + np.minimum(0.0, X0 - LO2[:, None, :])
    best_k = np.argmin(np.einsum("nki,nki->nk", RES0, RES0), axis=1)
    U = U0[np.arange(65), best_k].copy()
    Y = U.copy()
    tn = 1.0
    for _ in range(28):
        XY = C1 + np.einsum("nij,nj->ni", R1, Y)
        RES = np.maximum(0.0, XY - HI2) + np.minimum(0.0, XY - LO2)
        GRAD = np.einsum("nij,ni->nj", R1, RES)
        UN = np.clip(Y - 0.99 * GRAD, -H1, +H1)
        tn2 = 0.5 * (1.0 + math.sqrt(1.0 + 4.0 * tn * tn))
        Y = UN + ((tn - 1.0) / tn2) * (UN - U)
        U, tn = UN, tn2
    XF = C1 + np.einsum("nij,nj->ni", R1, U)
    RF = np.maximum(0.0, XF - HI2) + np.minimum(0.0, XF - LO2)
    UB = np.linalg.norm(RF, axis=1)
    N = RF / np.maximum(UB[:, None], 1e-12)
    M = np.einsum("nij,ni->nj", R1, N)
    LB_box = np.where(
        UB < 1e-9,
        0.0,
        np.maximum(0.0, np.einsum("ni,ni->n", C1, N) - np.einsum("ni,ni->n", H1, np.abs(M)) - (np.einsum("ni,ni->n", C2, N) + np.einsum("ni,ni->n", H2, np.abs(N)))),
    ) * 1000.0
    i_box = int(np.argmin(LB_box))
    best_mm = float(LB_box[i_box])
    best_dp = box_names[i_box // 5]
    best_wb = WB_PIECE_NAMES[i_box % 5]
    if best_mm == 0.0:
        return 0.0, best_dp, best_wb

    c_cyl, r_cyl, l_cyl, _, _ = hcyl[0]
    r_f, hl_f = float(r_cyl), float(l_cyl) * 0.5
    c_cyl_wb = p_trun_in_wb + R_sh_in_wb @ (np.asarray(c_cyl, float) - trun_local)
    LO5, HI5 = C2_5 - H2_5, C2_5 + H2_5
    Uc = np.zeros((5, 3), float)
    Yc = np.zeros((5, 3), float)
    tnc = 1.0
    for _ in range(36):
        XYc = c_cyl_wb[None, :] + Yc @ R_sh_in_wb.T
        RESc = np.maximum(0.0, XYc - HI5) + np.minimum(0.0, XYc - LO5)
        UNc = Yc - 0.99 * (RESc @ R_sh_in_wb)
        r_xy = np.hypot(UNc[:, 0], UNc[:, 1])
        scale = np.where(r_xy > r_f, r_f / np.maximum(r_xy, 1e-12), 1.0)
        UNc[:, 0] *= scale
        UNc[:, 1] *= scale
        UNc[:, 2] = np.clip(UNc[:, 2], -hl_f, +hl_f)
        tnc2 = 0.5 * (1.0 + math.sqrt(1.0 + 4.0 * tnc * tnc))
        Yc = UNc + ((tnc - 1.0) / tnc2) * (UNc - Uc)
        Uc, tnc = UNc, tnc2
    XFc = c_cyl_wb[None, :] + Uc @ R_sh_in_wb.T
    RFc = np.maximum(0.0, XFc - HI5) + np.minimum(0.0, XFc - LO5)
    UBc = np.linalg.norm(RFc, axis=1)
    Nc = RFc / np.maximum(UBc[:, None], 1e-12)
    Mc = Nc @ R_sh_in_wb
    LB_cyl = np.where(
        UBc < 1e-7,
        0.0,
        np.maximum(0.0, Nc @ c_cyl_wb - r_f * np.hypot(Mc[:, 0], Mc[:, 1]) - hl_f * np.abs(Mc[:, 2]) - (np.einsum("ni,ni->n", C2_5, Nc) + np.einsum("ni,ni->n", H2_5, np.abs(Nc)))),
    ) * 1000.0
    i_cyl = int(np.argmin(LB_cyl))
    if float(LB_cyl[i_cyl]) < best_mm:
        best_mm = float(LB_cyl[i_cyl])
        best_dp = HANDLE_PIECE_NAMES[4]
        best_wb = WB_PIECE_NAMES[i_cyl]
    return best_mm, best_dp, best_wb


def _get_link_local_boxes() -> dict[str, tuple[np.ndarray, np.ndarray]]:
    global _LINK_LOCAL_BOXES
    if _LINK_LOCAL_BOXES is not None:
        return _LINK_LOCAL_BOXES
    urdf_p = pathlib.Path(YAM_URDF)
    root = ET.parse(urdf_p).getroot()

    def _vec(s, d):
        return np.array([float(x) for x in s.split()], float) if s else np.array(d, float)

    def _rpy(r):
        cr, sr = math.cos(r[0]), math.sin(r[0])
        cp, sp = math.cos(r[1]), math.sin(r[1])
        cy, sy = math.cos(r[2]), math.sin(r[2])
        return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
                @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
                @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))

    out = {}
    for link in root.findall("link"):
        nm = link.attrib["name"]
        if nm in ("base", "world"):
            continue
        los, his = [], []
        for c in link.findall("collision"):
            m = c.find("geometry/mesh")
            if m is None:
                continue
            p = (urdf_p.parent / m.get("filename")).resolve()
            b = p.read_bytes()
            cnt = struct.unpack_from("<I", b, 80)[0]
            v = np.frombuffer(b, dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]), count=cnt, offset=84)["v"].reshape(-1, 3).astype(float)
            o = c.find("origin")
            R = _rpy(_vec(o.get("rpy") if o is not None else None, [0, 0, 0]))
            t = _vec(o.get("xyz") if o is not None else None, [0, 0, 0])
            vv = (v * _vec(m.get("scale"), [1, 1, 1])) @ R.T + t
            los.append(vv.min(0))
            his.append(vv.max(0))
        if los:
            lo_a, hi_a = np.asarray(los), np.asarray(his)
            corners = np.stack([
                np.stack([lo_a[:, 0], lo_a[:, 1], lo_a[:, 2]], -1),
                np.stack([lo_a[:, 0], lo_a[:, 1], hi_a[:, 2]], -1),
                np.stack([lo_a[:, 0], hi_a[:, 1], lo_a[:, 2]], -1),
                np.stack([lo_a[:, 0], hi_a[:, 1], hi_a[:, 2]], -1),
                np.stack([hi_a[:, 0], lo_a[:, 1], lo_a[:, 2]], -1),
                np.stack([hi_a[:, 0], lo_a[:, 1], hi_a[:, 2]], -1),
                np.stack([hi_a[:, 0], hi_a[:, 1], lo_a[:, 2]], -1),
                np.stack([hi_a[:, 0], hi_a[:, 1], hi_a[:, 2]], -1),
            ], axis=1)
            out[nm] = (lo_a, corners)
    _LINK_LOCAL_BOXES = out
    return out


def _rx_mat(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], float)


def _align_z_to_dir(hdir_w: np.ndarray) -> np.ndarray:
    u = np.asarray(hdir_w, float)
    u = u / max(np.linalg.norm(u), 1e-12)
    z = np.array([0.0, 0.0, 1.0])
    ax = np.cross(z, u)
    s = float(np.linalg.norm(ax))
    c = float(np.clip(np.dot(z, u), -1.0, 1.0))
    if s < 1e-8:
        return np.eye(3)
    return axis_angle_matrix(ax / s, math.atan2(s, c))


def _handle_point_world(dp_st, z_pan_rest: float) -> np.ndarray:
    q = float(dp_st.joints["handle"])
    p_local = np.array([0.0, 0.0, TRUN_Z]) + _rx_mat(q) @ np.array([0.0, 0.0, z_pan_rest - TRUN_Z])
    return np.asarray(dp_st.to_world(tuple(p_local)), float)


def _handle_dir_world(dp_st) -> np.ndarray:
    q = float(dp_st.joints["handle"])
    d_local = _rx_mat(q) @ np.array([0.0, 0.0, 1.0])
    return dp_st.rotation() @ d_local


def _pan_corners_world(dp_st) -> np.ndarray:
    pts = [
        (-PAN_W * 0.5, REAR_Y, 0.0), (+PAN_W * 0.5, REAR_Y, 0.0),
        (-PAN_W * 0.5, FRONT_Y, 0.0), (+PAN_W * 0.5, FRONT_Y, 0.0),
        (-PAN_W * 0.5, REAR_Y, PAN_H), (+PAN_W * 0.5, REAR_Y, PAN_H),
        (-PAN_W * 0.5, FRONT_Y, PAN_H), (+PAN_W * 0.5, FRONT_Y, PAN_H),
        (-RECESS_W * 0.5, RECESS_Y - RECESS_THICK * 0.5, RECESS_Z - RECESS_H * 0.5),
        (+RECESS_W * 0.5, RECESS_Y - RECESS_THICK * 0.5, RECESS_Z + RECESS_H * 0.5),
    ]
    return np.array([dp_st.to_world(p) for p in pts], float)


_IA_BOX = {}


def _live_arm_telemetry(env, dp_st, wb_st):
    from yam_arena import ik as _ik
    from yam_arena import runtime as _rt
    from yam_arena import station as _st

    lboxes = _get_link_local_boxes()
    b = env.unwrapped
    o = labkit._np(b.scene.env_origins)[0]
    R_wb = wb_st.rotation()
    p_wb = np.asarray(wb_st.pos, float)
    wb_lo = np.array([-BIN_L * 0.5, -BIN_W * 0.5, 0.0])
    wb_hi = np.array([+BIN_L * 0.5, +BIN_W * 0.5, BIN_H])

    R_dp = dp_st.rotation()
    p_dp = np.asarray(dp_st.pos, float)
    q_h = float(dp_st.joints["handle"])
    Rh = _rx_mat(q_h)
    trun_local = np.array([0.0, 0.0, TRUN_Z])
    pan_lo = np.array([-PAN_W * 0.5, RECESS_Y - 0.010, 0.0])
    pan_hi = np.array([+PAN_W * 0.5, FRONT_Y, PAN_H])
    yoke_x = PAN_W * 0.5 + 0.014
    yoke_lo = np.array([-yoke_x, -0.014, TRUN_Z - 0.010])
    yoke_hi = np.array([+yoke_x, +0.014, 0.258])

    arm_world_aabbs = {}
    tcp_info = {}
    min_arm_bin = 9999.0
    min_nongrip_pan_yoke = 9999.0

    for side, rkey in (("left", "left_robot"), ("right", "right_robot")):
        robot = b.scene[rkey]
        lposes = _rt.link_poses(robot)
        g = lposes["gripper"]
        R_tcp = g[:3, :3]
        p_tcp = R_tcp @ np.asarray(TCP_OFFSET, float) + g[:3, 3] - o
        f_ids = robot.find_joints(["joint7", "joint8"], preserve_order=True)[0]
        q7, q8 = (float(v) for v in labkit._np(robot.data.joint_pos)[0, f_ids])
        a_ids = robot.find_joints(ARM_JOINTS, preserve_order=True)[0]
        q_act = labkit._np(robot.data.joint_pos)[0, a_ids]
        q_tgt = labkit._np(robot.data.joint_pos_target)[0, a_ids]
        p_cmd, R_cmd = _ik.tcp_pose(q_tgt, _st.BASES[side])
        track_mm = float(np.linalg.norm(p_tcp - p_cmd) * 1000.0)
        cos_ang = float(np.clip((np.trace(R_cmd.T @ R_tcp) - 1.0) * 0.5, -1.0, 1.0))
        track_deg = math.degrees(math.acos(cos_ang))
        kp_6 = np.array([80.0, 80.0, 80.0, 10.0, 10.0, 10.0], float)
        pd_tau_nm = float(np.linalg.norm(kp_6 * (q_tgt - q_act)))
        tcp_info[side] = (p_tcp, R_tcp, q7, q8, track_mm, track_deg, q_act, pd_tau_nm)

        side_los, side_his = [], []
        _IA_BOX.setdefault(side, [])
        _IA_BOX[side] = []
        for lnm, (_, corners) in lboxes.items():
            tf = lposes[lnm]
            cw = corners @ tf[:3, :3].T + (tf[:3, 3] - o)
            wlo, whi = cw.min(axis=1), cw.max(axis=1)
            _IA_BOX[side].extend((f"{lnm}[{k}]", cw[k]) for k in range(len(cw)))
            side_los.append(wlo)
            side_his.append(whi)

            c_wb = (cw - p_wb) @ R_wb
            blo, bhi = c_wb.min(axis=1), c_wb.max(axis=1)
            db = float(np.linalg.norm(np.maximum(0.0, np.maximum(blo - wb_hi, wb_lo - bhi)), axis=1).min() * 1000.0)
            min_arm_bin = min(min_arm_bin, db)

            if not (side == "right" and lnm in ("gripper", "tip_left", "tip_right")):
                c_dp = (cw - p_dp) @ R_dp
                plo, phi = c_dp.min(axis=1), c_dp.max(axis=1)
                dp_d = float(np.linalg.norm(np.maximum(0.0, np.maximum(plo - pan_hi, pan_lo - phi)), axis=1).min() * 1000.0)
                c_sh = (c_dp - trun_local) @ Rh + trun_local
                slo, shi = c_sh.min(axis=1), c_sh.max(axis=1)
                dy_d = float(np.linalg.norm(np.maximum(0.0, np.maximum(slo - yoke_hi, yoke_lo - shi)), axis=1).min() * 1000.0)
                min_nongrip_pan_yoke = min(min_nongrip_pan_yoke, dp_d, dy_d)

        arm_world_aabbs[side] = (np.vstack(side_los), np.vstack(side_his))

    llo, lhi = arm_world_aabbs["left"]
    rlo, rhi = arm_world_aabbs["right"]
    _dm = np.linalg.norm(np.maximum(0.0, np.maximum(llo[:, None, :] - rhi[None, :, :], rlo[None, :, :] - lhi[:, None, :])), axis=2)
    d_ia = float(_dm.min() * 1000.0)
    try:  # claude-cake (codex 0033/0039): conservative margin over EVERY implicated box pair
        if d_ia < 60.0:
            def _ax(c):
                e = [c[1] - c[0], c[2] - c[0], c[4] - c[0]]
                return [v / np.linalg.norm(v) for v in e if np.linalg.norm(v) > 1e-9]
            def _sat(A, B):
                axes = _ax(A) + _ax(B)
                axes += [np.cross(u, v) / np.linalg.norm(np.cross(u, v)) for u in _ax(A) for v in _ax(B) if np.linalg.norm(np.cross(u, v)) > 1e-6]
                return max(max((A @ u).min() - (B @ u).max(), (B @ u).min() - (A @ u).max()) for u in axes)
            near = list(zip(*np.nonzero(_dm < 0.020)))          # every pair whose AABB gap is under 20 mm
            cons, worst, n_neg, n_zero = 9.9, None, 0, int((_dm <= 1e-9).sum())
            for i, j in near:
                nl, cl = _IA_BOX["left"][i]; nr, cr = _IA_BOX["right"][j]
                g = _sat(np.asarray(cl, float), np.asarray(cr, float))
                n_neg += int(g <= 0.0)
                m = max(float(_dm[i, j]), g)                      # true gap >= both bounds
                if m < cons:
                    cons, worst = m, (nl, nr, float(_dm[i, j]), g)
            far_min = float(_dm[_dm >= 0.020].min()) if (_dm >= 0.020).any() else 9.9
            cons = min(cons, far_min)                             # pairs not examined are >= their AABB gap
            print(f"[ia_pair] step={_state.get('n', -1)} aabb_mm={d_ia:.1f} n_near={len(near)} n_aabb_zero={n_zero} "
                  f"n_sat_overlap={n_neg} conservative_mm={cons * 1000.0:.1f} worst=" +
                  (f"L.{worst[0]}~R.{worst[1]}(aabb={worst[2] * 1e3:.1f},sat={worst[3] * 1e3:.1f})" if worst else "none"), flush=True)
    except Exception as _e:  # noqa: BLE001
        print(f"[ia_pair] error {_e!r}"[:200], flush=True)
    return tcp_info, d_ia, min_arm_bin, min_nongrip_pan_yoke


def _core_status(cs, wb_st, dp_st, name="core_1") -> str:
    pw = np.asarray(wb_st.to_local(cs.pos), float)
    lo, hi = _item_lohi(name, cs, wb_st)
    if (max(abs(pw[0] + lo[0]), abs(pw[0] + hi[0])) <= BIN_L * 0.5 - BIN_WALL_T + 0.002) and (max(abs(pw[1] + lo[1]), abs(pw[1] + hi[1])) <= BIN_W * 0.5 - BIN_WALL_T + 0.002) and (BIN_FLOOR_T - 0.005 <= pw[2] + lo[2]) and (pw[2] + hi[2] <= BIN_H - 0.050):
        return "in_bin"
    pp = np.asarray(dp_st.to_local(cs.pos), float)
    if abs(pp[0]) <= PAN_W * 0.5 and (REAR_Y <= pp[1] <= FRONT_Y + 0.015) and (0.0 <= pp[2] <= PAN_H + 0.025):
        return "in_pan"
    return "outside"


@on_reset
def _reset(env):
    _state.clear()
    _state.update(
        n=0, latched={}, trun_z0=None, wb_xy_15s=None, abort_dump=False, abort_reason="",
        grip_ref_R=None, grip_ref_L=None, grip_ref_rec_R=None,
        max_slip_R_mm=0.0, max_slip_L_mm=0.0, max_slip_rec_R_mm=0.0,
        min_pan_floor_airborne_mm=9999.0, min_pan_rim_transit_mm=9999.0,
        winning_handoff_pair={"dist_mm": 9999.0, "dp_piece": "", "wb_piece": "", "step": 0, "t_s": 0.0},
        winning_transit_pair={"dist_mm": 9999.0, "dp_piece": "", "wb_piece": "", "step": 0, "t_s": 0.0},
        winning_repinch_pair={"dist_mm": 9999.0, "dp_piece": "", "wb_piece": "", "step": 0, "t_s": 0.0},
        winning_overall_pair={"dist_mm": 9999.0, "dp_piece": "", "wb_piece": "", "step": 0, "t_s": 0.0},
        trun_before_preload=None, trun_pre_release=None,
        tau_L_before_preload_nm=None, tau_L_pre_release_nm=None, tau_R_before_preload_nm=None, tau_R_pre_release_nm=None,
        max_release_drop_z_mm=0.0, max_release_xy_lurch_mm=0.0,
        max_bin_handoff_move_mm=0.0, max_bin_transit_move_mm=0.0, max_bin_repinch_move_mm=0.0,
        min_handoff_inter_arm_mm=9999.0, min_handoff_arm_bin_mm=9999.0,
        min_repinch_inter_arm_mm=9999.0, min_repinch_arm_bin_mm=9999.0,
        repinch_hold_sample={}, solo_right_r0_sample={},
        min_pan_chute_pitch_deg=999.0,
        min_inter_arm_mm=9999.0, min_arm_bin_mm=9999.0, min_nongrip_pan_yoke_mm=9999.0,
        dump_peak_pose={}, release_pose={},
    )


@on_step
def _step(env):
    dp_st = labkit._part_state(env, DUSTPAN)
    wb_st = labkit._part_state(env, WASTEBASKET)
    it_st = [labkit._part_state(env, it) for it in ITEMS]
    _state["live"] = (dp_st, wb_st, *it_st)
    _state["n"] += 1
    n = _state["n"]

    step_dt = float(getattr(env.unwrapped, "step_dt", 0.05))
    t_sim = n * step_dt
    is_coupon = os.environ.get("LD_MODE") == "handoff_repinch_coupon"

    trun_w = np.asarray(dp_st.to_world((0.0, 0.0, TRUN_Z)), float)
    rec_w = np.asarray(dp_st.to_world((0.0, RECESS_Y, RECESS_Z)), float)
    if _state["trun_z0"] is None and n >= 10:
        _state["trun_z0"] = float(trun_w[2])
    if _state["wb_xy_15s"] is None and t_sim >= 15.00:
        _state["wb_xy_15s"] = np.asarray(wb_st.pos[:2], float).copy()

    pan_pts = _pan_corners_world(dp_st)
    pan_floor_mm = float((pan_pts[:, 2].min() - FLOOR) * 1000.0)
    wb_rim_z = float(wb_st.pos[2]) + BIN_H
    pan_above_rim_mm = float((pan_pts[:, 2].min() - wb_rim_z) * 1000.0)
    b_mv = float(np.linalg.norm(np.asarray(wb_st.pos[:2]) - np.asarray(BIN_XY)) * 1000.0)

    pair_mm, win_dp, win_wb = dustpan_to_hollow_bin_nearest_pair(dp_st, wb_st)
    if pair_mm < _state["winning_overall_pair"]["dist_mm"]:
        _state["winning_overall_pair"] = {"dist_mm": round(pair_mm, 2), "dp_piece": win_dp, "wb_piece": win_wb, "step": n, "t_s": round(t_sim, 2)}

    R_dp = dp_st.rotation()
    pan_pitch_deg = math.degrees(math.atan2(float(R_dp[2, 1]), float(R_dp[1, 1])))
    if pan_pitch_deg < _state["min_pan_chute_pitch_deg"]:
        _state["min_pan_chute_pitch_deg"] = pan_pitch_deg

    statuses = [_core_status(st_, wb_st, dp_st, nm) for st_, nm in zip(it_st, ITEM_NAMES)]
    c_pan = statuses.count("in_pan")
    c_bin = statuses.count("in_bin")

    tcp_info, d_ia, d_ab, d_py = _live_arm_telemetry(env, dp_st, wb_st)
    p_tcp_l, R_tcp_l, q7_l, q8_l, trk_l_mm, _, q_act_l, tau_l_nm = tcp_info["left"]
    p_tcp_r, R_tcp_r, q7_r, q8_r, trk_r_mm, _, q_act_r, tau_r_nm = tcp_info["right"]

    _state["min_inter_arm_mm"] = min(_state["min_inter_arm_mm"], d_ia)
    _state["min_arm_bin_mm"] = min(_state["min_arm_bin_mm"], d_ab)
    _state["min_nongrip_pan_yoke_mm"] = min(_state["min_nongrip_pan_yoke_mm"], d_py)
    if t_sim >= 6.00:
        _state["min_pan_floor_airborne_mm"] = min(_state["min_pan_floor_airborne_mm"], pan_floor_mm)

    # Phase (a) Handoff preload + Right release + solo-Left hold metrics (steps 1..306, t <= 15.30 s):
    if n == 240:
        _state["trun_before_preload"] = trun_w.copy()
        _state["tau_L_before_preload_nm"] = round(tau_l_nm, 3)
        _state["tau_R_before_preload_nm"] = round(tau_r_nm, 3)
    if n == 270:
        _state["trun_pre_release"] = trun_w.copy()
        _state["tau_L_pre_release_nm"] = round(tau_l_nm, 3)
        _state["tau_R_pre_release_nm"] = round(tau_r_nm, 3)
    if 270 <= n <= 306 and _state["trun_pre_release"] is not None:
        drop_z = float((_state["trun_pre_release"][2] - trun_w[2]) * 1000.0)
        lurch_xy = float(np.linalg.norm(trun_w[:2] - _state["trun_pre_release"][:2]) * 1000.0)
        _state["max_release_drop_z_mm"] = max(_state["max_release_drop_z_mm"], drop_z)
        _state["max_release_xy_lurch_mm"] = max(_state["max_release_xy_lurch_mm"], lurch_xy)
    if n <= 306:
        if pair_mm < _state["winning_handoff_pair"]["dist_mm"]:
            _state["winning_handoff_pair"] = {"dist_mm": round(pair_mm, 2), "dp_piece": win_dp, "wb_piece": win_wb, "step": n, "t_s": round(t_sim, 2)}
        _state["max_bin_handoff_move_mm"] = max(_state["max_bin_handoff_move_mm"], b_mv)
        _state["min_handoff_inter_arm_mm"] = min(_state["min_handoff_inter_arm_mm"], d_ia)
        _state["min_handoff_arm_bin_mm"] = min(_state["min_handoff_arm_bin_mm"], d_ab)

    if t_sim <= 21.80:
        if pair_mm < _state["winning_transit_pair"]["dist_mm"]:
            _state["winning_transit_pair"] = {"dist_mm": round(pair_mm, 2), "dp_piece": win_dp, "wb_piece": win_wb, "step": n, "t_s": round(t_sim, 2)}
        if float(pan_pts[:, 1].max()) >= BIN_XY[1] - BIN_W * 0.5 - 0.010:
            _state["min_pan_rim_transit_mm"] = min(_state["min_pan_rim_transit_mm"], pan_above_rim_mm)
        _state["max_bin_transit_move_mm"] = max(_state["max_bin_transit_move_mm"], b_mv)

    # Tightened Phase (a) + Transit Gate at step 436 (t=21.80 s) per Codex 2308/2310:
    if n == 436:
        wt = _state["winning_transit_pair"]
        wh = _state["winning_handoff_pair"]
        preload_lift_mm = float((_state["trun_pre_release"][2] - _state["trun_before_preload"][2]) * 1000.0) if (_state["trun_pre_release"] is not None and _state["trun_before_preload"] is not None) else 0.0
        drop_mm = _state["max_release_drop_z_mm"]
        if wh["dist_mm"] < 20.0 or wt["dist_mm"] < 8.0 or drop_mm > 10.0 or _state["max_bin_transit_move_mm"] > 0.5 or c_pan < len(ITEMS):
            _state["abort_dump"] = True
            _state["abort_reason"] = (
                f"handoff_win={wh['dp_piece']}->{wh['wb_piece']}:{wh['dist_mm']:.2f}mm "
                f"transit_win={wt['dp_piece']}->{wt['wb_piece']}:{wt['dist_mm']:.2f}mm "
                f"preload_lift={preload_lift_mm:+.2f}mm drop={drop_mm:.2f}mm "
                f"bin_mv={_state['max_bin_transit_move_mm']:.2f}mm c_pan={c_pan}"
            )
            print(f"[cd1c_gate] ABORT_BEFORE_TIPPING: {_state['abort_reason']}", file=sys.stderr, flush=True)
        else:
            print(
                f"[cd1c_gate] TRANSIT_CLEAR_PROCEED: preload_lift={preload_lift_mm:+.2f}mm drop={drop_mm:.2f}mm "
                f"lurch_xy={_state['max_release_xy_lurch_mm']:.2f}mm "
                f"handoff_win={wh['dp_piece']}->{wh['wb_piece']}:{wh['dist_mm']:.2f}mm@t={wh['t_s']}s "
                f"transit_win={wt['dp_piece']}->{wt['wb_piece']}:{wt['dist_mm']:.2f}mm@t={wt['t_s']}s "
                f"min_rim_z={_state['min_pan_rim_transit_mm']:+.2f}mm bin_mv={_state['max_bin_transit_move_mm']:.2f}mm c_pan={c_pan}/{len(ITEMS)}",
                file=sys.stderr, flush=True,
            )

    p_r_sh_in_tcp = R_tcp_r.T @ (_handle_point_world(dp_st, R_GRIP_S) - p_tcp_r)
    if _state["grip_ref_R"] is None and t_sim >= 3.85:
        _state["grip_ref_R"] = p_r_sh_in_tcp.copy()
    if _state["grip_ref_R"] is not None and 3.85 <= t_sim <= 13.45:
        _state["max_slip_R_mm"] = max(_state["max_slip_R_mm"], float(np.linalg.norm(p_r_sh_in_tcp - _state["grip_ref_R"]) * 1000.0))

    p_l_sh_in_tcp = R_tcp_l.T @ (_handle_point_world(dp_st, L_GRIP_S) - p_tcp_l)
    if _state["grip_ref_L"] is None and t_sim >= 11.95:
        _state["grip_ref_L"] = p_l_sh_in_tcp.copy()
    slip_l_mm = 0.0
    if _state["grip_ref_L"] is not None and t_sim >= 11.95:
        slip_l_mm = float(np.linalg.norm(p_l_sh_in_tcp - _state["grip_ref_L"]) * 1000.0)
        if not (is_coupon and n > 584):
            _state["max_slip_L_mm"] = max(_state["max_slip_L_mm"], slip_l_mm)

    p_rec_in_rtcp = R_tcp_r.T @ (rec_w - p_tcp_r)
    if not is_coupon and _state["grip_ref_rec_R"] is None and t_sim >= 26.85 and not _state["abort_dump"]:
        _state["grip_ref_rec_R"] = p_rec_in_rtcp.copy()
    slip_rec_mm = 0.0
    if not is_coupon and _state["grip_ref_rec_R"] is not None and 26.85 <= t_sim <= 33.95:
        slip_rec_mm = float(np.linalg.norm(p_rec_in_rtcp - _state["grip_ref_rec_R"]) * 1000.0)
        _state["max_slip_rec_R_mm"] = max(_state["max_slip_rec_R_mm"], slip_rec_mm)

    # Phase (b) Coupon re-pinch hold + live Right re-pinch + Left release + solo-Right R_r0 metrics (steps 437..674):
    if is_coupon and 437 <= n <= 674:
        if pair_mm < _state["winning_repinch_pair"]["dist_mm"]:
            _state["winning_repinch_pair"] = {"dist_mm": round(pair_mm, 2), "dp_piece": win_dp, "wb_piece": win_wb, "step": n, "t_s": round(t_sim, 2)}
        _state["max_bin_repinch_move_mm"] = max(_state["max_bin_repinch_move_mm"], b_mv)
        _state["min_repinch_inter_arm_mm"] = min(_state["min_repinch_inter_arm_mm"], d_ia)
        _state["min_repinch_arm_bin_mm"] = min(_state["min_repinch_arm_bin_mm"], d_ab)
        if n == 550:
            _state["repinch_hold_sample"] = {
                "step": n, "t_s": round(t_sim, 2),
                "trun_w_m": [round(float(x), 4) for x in trun_w],
                "pan_pitch_deg": round(pan_pitch_deg, 2),
                "q_hdl_deg": round(math.degrees(float(dp_st.joints["handle"])), 2),
                "yaw_deg": round(math.degrees(math.atan2(float(R_dp[1, 0]), float(R_dp[0, 0]))), 2),
                "pair_mm": round(pair_mm, 2), "win_pair": f"{win_dp}->{win_wb}",
                "L_tcp_pos_m": [round(float(x), 4) for x in p_tcp_l],
                "R_tcp_pos_m": [round(float(x), 4) for x in p_tcp_r],
                "L_q78_mm": [round(q7_l * 1000, 2), round(q8_l * 1000, 2)],
                "R_q78_mm": [round(q7_r * 1000, 2), round(q8_r * 1000, 2)],
                "cores_pan_bin": [c_pan, c_bin],
            }
        if n == 674:
            _state["solo_right_r0_sample"] = {
                "step": n, "t_s": round(t_sim, 2),
                "trun_w_m": [round(float(x), 4) for x in trun_w],
                "pan_pitch_deg": round(pan_pitch_deg, 2),
                "q_hdl_deg": round(math.degrees(float(dp_st.joints["handle"])), 2),
                "yaw_deg": round(math.degrees(math.atan2(float(R_dp[1, 0]), float(R_dp[0, 0]))), 2),
                "pair_mm": round(pair_mm, 2), "win_pair": f"{win_dp}->{win_wb}",
                "R_tcp_pos_m": [round(float(x), 4) for x in p_tcp_r],
                "R_q78_mm": [round(q7_r * 1000, 2), round(q8_r * 1000, 2)],
                "clr_inter_arm_mm": round(d_ia, 2),
                "clr_arm_bin_mm": round(d_ab, 2),
                "cores_pan_bin": [c_pan, c_bin],
            }

    if not is_coupon and n == 620:
        _state["dump_peak_pose"] = {
            "t_s": round(t_sim, 2),
            "dp_pos_m": [round(float(x), 4) for x in dp_st.pos],
            "dp_rot_3x3": [[round(float(v), 5) for v in row] for row in R_dp],
            "trun_w_m": [round(float(x), 4) for x in trun_w],
            "recess_w_m": [round(float(x), 4) for x in rec_w],
            "pan_pitch_deg": round(pan_pitch_deg, 2),
            "q_hdl_deg": round(math.degrees(float(dp_st.joints["handle"])), 2),
            "R_tcp_pos_m": [round(float(x), 4) for x in p_tcp_r],
            "R_tcp_rot_3x3": [[round(float(v), 5) for v in row] for row in R_tcp_r],
            "L_tcp_pos_m": [round(float(x), 4) for x in p_tcp_l],
            "L_tcp_rot_3x3": [[round(float(v), 5) for v in row] for row in R_tcp_l],
            "L_joint_pos_deg": [round(math.degrees(float(x)), 2) for x in q_act_l],
            "R_joint_pos_deg": [round(math.degrees(float(x)), 2) for x in q_act_r],
            "cores_pan_bin": [c_pan, c_bin],
        }
    if not is_coupon and n == 710:
        _state["release_pose"] = {
            "t_s": round(t_sim, 2),
            "dp_pos_m": [round(float(x), 4) for x in dp_st.pos],
            "dp_rot_3x3": [[round(float(v), 5) for v in row] for row in R_dp],
            "trun_w_m": [round(float(x), 4) for x in trun_w],
            "recess_w_m": [round(float(x), 4) for x in rec_w],
            "hdir_w": [round(float(x), 4) for x in _handle_dir_world(dp_st)],
            "pan_pitch_deg": round(pan_pitch_deg, 2),
            "q_hdl_deg": round(math.degrees(float(dp_st.joints["handle"])), 2),
            "L_tcp_pos_m": [round(float(x), 4) for x in p_tcp_l],
            "L_tcp_rot_3x3": [[round(float(v), 5) for v in row] for row in R_tcp_l],
            "R_tcp_pos_m": [round(float(x), 4) for x in p_tcp_r],
            "R_tcp_rot_3x3": [[round(float(v), 5) for v in row] for row in R_tcp_r],
            "L_joint_pos_deg": [round(math.degrees(float(x)), 2) for x in q_act_l],
            "R_joint_pos_deg": [round(math.degrees(float(x)), 2) for x in q_act_r],
        }

    fine_window = (235 <= n <= 310) or (is_coupon and 475 <= n <= 674 and n % 2 == 0)
    if os.environ.get("LD_LOG", "1") == "1" and (n == 1 or n % 10 == 0 or fine_window):
        q_deg = math.degrees(float(dp_st.joints["handle"]))
        dp_yaw = math.degrees(math.atan2(float(R_dp[1, 0]), float(R_dp[0, 0])))
        print(
            f"[cd1e] step={n:3d} t={t_sim:5.2f}s "
            f"trun=({trun_w[0]:.3f},{trun_w[1]:+.3f},{trun_w[2]:.3f}) "
            f"pan_rim={pan_above_rim_mm:+6.1f}mm pair={win_dp}->{win_wb}:{pair_mm:5.1f}mm "
            f"pitch={pan_pitch_deg:+5.1f}deg q_hdl={q_deg:+5.1f}deg yaw={dp_yaw:+4.1f}deg "
            f"pan/bin={c_pan}/{c_bin} bin_err={b_mv:4.2f}mm "
            f"L_q78=({q7_l*1000:+.1f},{q8_l*1000:+.1f}) R_q78=({q7_r*1000:+.1f},{q8_r*1000:+.1f}) "
            f"tauL={tau_l_nm:4.2f}Nm tauR={tau_r_nm:4.2f}Nm trkL={trk_l_mm:4.1f}mm slipL={slip_l_mm:4.2f}mm "
            f"clr(ia={d_ia:4.1f},bin={d_ab:4.1f},py={d_py:4.1f})mm",
            file=sys.stderr, flush=True,
        )

    if is_coupon and n == 674:
        preload_lift_mm = float((_state["trun_pre_release"][2] - _state["trun_before_preload"][2]) * 1000.0) if (_state["trun_pre_release"] is not None and _state["trun_before_preload"] is not None) else 0.0
        wh = _state["winning_handoff_pair"]
        wt = _state["winning_transit_pair"]
        wr = _state["winning_repinch_pair"]
        handoff_pass = (
            preload_lift_mm > 5.0
            and _state["max_release_drop_z_mm"] <= 10.0
            and wh["dist_mm"] >= 20.0
            and _state["max_bin_handoff_move_mm"] <= 0.5
            and c_pan == len(ITEMS)
            and _state["min_handoff_inter_arm_mm"] > 5.0
        )
        repinch_pass = (
            wr["dist_mm"] >= 20.0
            and _state["max_bin_repinch_move_mm"] <= 0.5
            and c_pan == len(ITEMS)
            and _state["min_repinch_inter_arm_mm"] > 5.0
        )
        coupon_summary = {
            "mode": "handoff_repinch_coupon_2core_no_dump",
            "load_disclosure": "No-dump coupon carries original 2-core payload (0.898 kg total vs 0.840 kg empty) through both handoff and lowered re-pinch hold per Codex 2322.",
            "phase_a_handoff_pass": handoff_pass,
            "phase_b_repinch_pass": repinch_pass,
            "all_gates_pass": bool(handoff_pass and repinch_pass and not _state["abort_dump"]),
            "trun_before_preload_m": [round(float(x), 4) for x in _state["trun_before_preload"]] if _state["trun_before_preload"] is not None else None,
            "trun_pre_release_m": [round(float(x), 4) for x in _state["trun_pre_release"]] if _state["trun_pre_release"] is not None else None,
            "tau_L_before_preload_nm": _state["tau_L_before_preload_nm"],
            "tau_L_pre_release_nm": _state["tau_L_pre_release_nm"],
            "tau_R_before_preload_nm": _state["tau_R_before_preload_nm"],
            "tau_R_pre_release_nm": _state["tau_R_pre_release_nm"],
            "preload_lift_z_mm": round(preload_lift_mm, 2),
            "max_release_drop_z_mm": round(_state["max_release_drop_z_mm"], 2),
            "max_release_xy_lurch_mm": round(_state["max_release_xy_lurch_mm"], 2),
            "winning_handoff_pair": wh,
            "winning_transit_pair": wt,
            "winning_repinch_pair": wr,
            "handoff_max_bin_move_mm": round(_state["max_bin_handoff_move_mm"], 2),
            "transit_max_bin_move_mm": round(_state["max_bin_transit_move_mm"], 2),
            "repinch_max_bin_move_mm": round(_state["max_bin_repinch_move_mm"], 2),
            "max_slip_R_lift_mm": round(_state["max_slip_R_mm"], 2),
            "max_slip_L_mm": round(_state["max_slip_L_mm"], 2),
            "handoff_min_inter_arm_mm": round(_state["min_handoff_inter_arm_mm"], 2),
            "handoff_min_arm_bin_mm": round(_state["min_handoff_arm_bin_mm"], 2),
            "repinch_min_inter_arm_mm": round(_state["min_repinch_inter_arm_mm"], 2),
            "repinch_min_arm_bin_mm": round(_state["min_repinch_arm_bin_mm"], 2),
            "repinch_hold_sample": _state["repinch_hold_sample"],
            "solo_right_r0_sample": _state["solo_right_r0_sample"],
            "final_cores_in_pan": c_pan,
        }
        print(f"[repinch_coupon_summary] {json.dumps(coupon_summary)}", file=sys.stderr, flush=True)

    if not is_coupon and n == 756:
        summary = {
            "final_t_s": round(t_sim, 2),
            "abort_dump": _state["abort_dump"],
            "abort_reason": _state["abort_reason"],
            "max_release_drop_z_mm": round(_state["max_release_drop_z_mm"], 2),
            "winning_handoff_pair": _state["winning_handoff_pair"],
            "transit_min_pan_above_rim_mm": round(_state["min_pan_rim_transit_mm"], 2),
            "winning_transit_pair": _state["winning_transit_pair"],
            "winning_overall_pair": _state["winning_overall_pair"],
            "transit_max_bin_move_mm": round(_state["max_bin_transit_move_mm"], 2),
            "peak_dump_chute_pitch_deg": round(_state["min_pan_chute_pitch_deg"], 2),
            "max_slip_R_lift_mm": round(_state["max_slip_R_mm"], 2),
            "max_slip_L_total_mm": round(_state["max_slip_L_mm"], 2),
            "max_slip_rec_R_dump_mm": round(_state["max_slip_rec_R_mm"], 2),
            "final_cores_in_bin": c_bin,
            "final_cores_in_pan": c_pan,
            "final_core_statuses": statuses,
            "final_item_pos_m": {nm: [round(float(x), 4) for x in st_.pos] for nm, st_ in zip(ITEM_NAMES, it_st)},
            "final_bin_tilt_deg": round(wb_st.tilt_deg(), 2),
            "final_bin_xy_disp_mm": round(b_mv, 2),
            "min_pan_floor_airborne_mm": round(_state["min_pan_floor_airborne_mm"], 2),
            "min_live_inter_arm_mm": round(_state["min_inter_arm_mm"], 2),
            "min_live_arm_bin_mm": round(_state["min_arm_bin_mm"], 2),
            "min_live_nongrip_pan_yoke_mm": round(_state["min_nongrip_pan_yoke_mm"], 2),
            "dump_peak_pose": _state["dump_peak_pose"],
            "post_dump_release_pose": _state["release_pose"],
        }
        print(f"[cd1b_summary] {json.dumps(summary)}", file=sys.stderr, flush=True)


def _front_stub_score(objects) -> float:
    return 0.0


SUCCESS_HOLD_S = 1.0


def _front_stub_success(objects) -> bool:
    return False


def _front_stub_distance(objects) -> float:
    return 1.0


import yam_arena.demo as _demo  # noqa: E402


class Orient(float):
    def __new__(cls, rot):
        obj = float.__new__(cls, 0.0)
        obj._rot = rot
        return obj

    @property
    def rot(self):
        r = self._rot() if callable(self._rot) else self._rot
        return np.asarray(r, float)


# ---- claude-cake: joint-space waypoint (escapes the post-dump IK trap; see make_joint_unwind_patch.py) ----
Q_STAGE = (-0.9281278958267719, 0.3583055002965652, 0.3072288028121247, -0.811738384563976, -0.7150331090928882, -0.6536632916913919)


class _JointGoal:
    """A waypoint position given as arm joints. np.asarray() gives the goal's FK TCP (world), so Cartesian waypoints before and
    after it interpolate from/to a consistent pose; ArmPlan.action7 blends joints while it is the current waypoint."""
    def __init__(self, q, base):
        self.q = np.asarray(q, float); self.base = np.asarray(base, float)
        from yam_arena import ik as _ik
        self.p, self.R = _ik.tcp_pose(self.q, self.base)

    def __array__(self, dtype=None, copy=None):
        return np.asarray(self.p, dtype=dtype or float)


if not getattr(_demo.ArmPlan.action7, "_joint_goal", False):
    _action7_ik = _demo.ArmPlan.action7

    def _action7(self, t):
        tt = t
        for w in self.waypoints:
            if tt < w.duration:
                break
            tt -= w.duration
        else:
            w = None
        if w is not None and isinstance(w.pos, _JointGoal):
            if getattr(self, "_jg_for", None) is not w:
                self._jg_for, self._jg_q0 = w, np.array(self.q, float)
            s_ = _demo.ik.smoothstep(tt / w.duration)
            self.q = self._jg_q0 + s_ * (w.pos.q - self._jg_q0)
            _, _, finger, label = self.target(t)
            grip = float(np.clip(finger / _demo.FINGER_OPEN, 0.0, 1.0)) if _demo.FINGER_OPEN != _demo.FINGER_CLOSED else 1.0
            return np.r_[self.q, grip], label
        return _action7_ik(self, t)

    _action7._joint_goal = True
    _demo.ArmPlan.action7 = _action7

_down_grip = _demo.grip_rotation
if not getattr(_down_grip, "_orient", False):
    def _grip_rotation(yaw=0.0):
        return yaw.rot if isinstance(yaw, Orient) else _down_grip(yaw)

    _grip_rotation._orient = True
    _demo.grip_rotation = _grip_rotation


def _rx(deg: float) -> np.ndarray:
    return axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(deg))


def _ry(deg: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(deg))


def _rz(deg: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(deg))


def _vert_shaft_rot(yaw_deg: float, pitch_deg: float = 52.0) -> np.ndarray:
    return _rz(yaw_deg) @ _ry(-pitch_deg) @ _down_grip(0.0)


class _LivePos:
    def __init__(self, fn):
        self.fn = fn

    def __array__(self, dtype=None, copy=None):
        return np.asarray(self.fn(), dtype=dtype or float)


LEFT_PARK = (0.24, +0.26, 0.88)
RIGHT_PARK = (0.18, -0.36, 0.85)   # claude-cake hr2: far park, clear of the left wrist at HIGH_PREBIN
SHAFT_OPEN = -0.038
SHAFT_PINCH = FINGER_CLOSED
RECESS_OPEN = -0.038
RECESS_PINCH = FINGER_CLOSED


def _pad_tcp(target_world, rot_mat: np.ndarray, back: float = 0.0, depth: float = -0.004) -> np.ndarray:
    gz = rot_mat[:, 2]
    return np.asarray(target_world, float) - (depth + back) * gz


def _latched(key: str, min_step: int, compute_fn):
    lat = _state.setdefault("latched", {})
    if key not in lat and _state.get("live") is not None and _state.get("n", 0) >= min_step:
        lat[key] = compute_fn(_state["live"][0])
    return lat.get(key)


def demo():
    R_park = _ry(-55.0) @ _down_grip(0.0)
    R_r_hi = _vert_shaft_rot(+20.0, 50.0)
    R_r0 = _rx(+8.0) @ R_r_hi
    R_l_base = _vert_shaft_rot(-16.0, 52.0)
    R_l_tilt = _rx(-30.0) @ R_l_base

    def p_r_pick(back=0.0):
        pt = _latched("r_pick", 15, lambda dp: _handle_point_world(dp, R_GRIP_S))
        if pt is None:
            pt = np.array([PARK_XY[0], PARK_XY[1], FLOOR + 0.001 + TRUN_Z]) + _rx_mat(math.radians(8.0)) @ np.array([0.0, 0.0, R_GRIP_S - TRUN_Z])
        return _pad_tcp(pt, R_r0, back=back)

    def p_r_vlift():
        pt = _latched("r_pick", 15, lambda dp: _handle_point_world(dp, R_GRIP_S))
        if pt is None:
            pt = np.array([PARK_XY[0], PARK_XY[1], FLOOR + 0.001 + TRUN_Z]) + _rx_mat(math.radians(8.0)) @ np.array([0.0, 0.0, R_GRIP_S - TRUN_Z])
        return _pad_tcp([pt[0], pt[1], pt[2] + 0.210], R_r0)

    p_r_lift_hi = _pad_tcp(HANDOFF_R_TARGET, R_r_hi)

    def _handoff_pose():
        def _measure_held(dp):
            pt_l = _handle_point_world(dp, L_GRIP_S)
            pt_r_held = _handle_point_world(dp, R_GRIP_S)
            hdir_h = _handle_dir_world(dp)
            R_l = _align_z_to_dir(hdir_h) @ R_l_base
            return pt_l, R_l, pt_r_held

        res = _latched("l_handoff_pose", 174, _measure_held)
        if res is None:
            pt_r_nom = np.array([HANDOFF_R_TARGET[0] - 0.031, HANDOFF_R_TARGET[1] - 0.031, HANDOFF_R_TARGET[2] - 0.037], float)
            pt_l_nom = pt_r_nom - np.array([0.0, 0.0, R_GRIP_S - L_GRIP_S], float)
            return pt_l_nom, R_l_base, pt_r_nom
        return res

    def p_l_handoff(back=0.0):
        pt_l, R_l, _ = _handoff_pose()
        return _pad_tcp(pt_l, R_l, back=back)

    def p_l_preloaded():
        pt_l, R_l, _ = _handoff_pose()
        return _pad_tcp(pt_l + PRELOAD_DELTA_L, R_l, back=0.0)

    def R_l_handoff():
        return _handoff_pose()[1]

    def p_r_unloaded(back=0.0):
        _, _, pt_r_held = _handoff_pose()
        return _pad_tcp(pt_r_held + UNLOAD_LIFT_R, R_r_hi, back=back)

    p_l_high_prebin = _pad_tcp(HIGH_PREBIN_TARGET, R_l_base)
    p_l_over_bin = _pad_tcp([0.390, +0.245, 0.902], R_l_tilt)
    R_l_dump = _rx(-42.0) @ R_l_base
    p_l_dump_dip = _pad_tcp([0.390, +0.245, 0.872], R_l_dump)
    RECESS_MID_Y = -0.1745

    def _dump_rebind():
        def _measure_ob(dp):
            trun_ob = np.asarray(dp.to_world((0.0, 0.0, TRUN_Z)), float)
            rec_ob = np.asarray(dp.to_world((0.0, RECESS_MID_Y, RECESS_Z)), float)
            R_dp_ob = dp.rotation()
            x_trun = R_dp_ob[:, 0]
            R_rec_0 = axis_angle_matrix(x_trun, math.radians(+14.0)) @ np.column_stack([-R_dp_ob[:, 1], -R_dp_ob[:, 0], -R_dp_ob[:, 2]])
            hang_deg = math.degrees(math.atan2(float(R_dp_ob[2, 1]), float(R_dp_ob[1, 1])))
            arc_dump = float(np.clip(hang_deg + 49.0, 96.0, 102.0))
            print(
                f"[dump_rebind] step={_state.get('n')} trun_ob={trun_ob.round(4).tolist()} "
                f"rec_ob={rec_ob.round(4).tolist()} hang_deg={hang_deg:.2f} arc_dump={arc_dump:.2f}",
                file=sys.stderr, flush=True,
            )
            return trun_ob, rec_ob, x_trun, R_rec_0, arc_dump

        res = _latched("dump_rebind", 436, _measure_ob)
        if res is None:
            R_nom = _rz(7.1) @ _rx(53.6)
            xt = R_nom[:, 0]
            R0 = axis_angle_matrix(xt, math.radians(+14.0)) @ np.column_stack([-R_nom[:, 1], -R_nom[:, 0], -R_nom[:, 2]])
            tr = np.array([0.350, +0.1402, 0.6147])
            rc = tr + R_nom @ np.array([0.0, RECESS_MID_Y, 0.0])
            return tr, rc, xt, R0, 102.0
        return res

    def _rec_pose(frac: float, back: float = 0.0):
        if _state.get("abort_dump", False):
            return np.asarray(RIGHT_PARK, float), R_park
        trun_ob, rec_ob, x_trun, R_rec_0, arc_dump = _dump_rebind()
        arc = frac * arc_dump
        trun_t = trun_ob + np.array([0.0, 0.0, -0.025 * frac])
        R_arc_pos = axis_angle_matrix(x_trun, -math.radians(arc))
        R_arc_rot = axis_angle_matrix(x_trun, -math.radians(arc * 0.80))
        R_a = R_arc_rot @ R_rec_0
        p_a = _pad_tcp(trun_t + R_arc_pos @ (rec_ob - trun_ob), R_a, back=back)
        return p_a, R_a

    def _stage_rec_pos():
        return np.asarray(RIGHT_PARK if _state.get("abort_dump", False) else (0.320, -0.130, 0.590), float)

    def _stage_rec_rot():
        return R_park if _state.get("abort_dump", False) else _dump_rebind()[3]

    def _l_dump_pos():
        return p_l_over_bin if _state.get("abort_dump", False) else p_l_dump_dip

    def _l_dump_rot():
        return R_l_tilt if _state.get("abort_dump", False) else R_l_dump

    # Pre-dump transit + handoff (exact 21.8 s = 436 steps on both arms):
    left_transit = [
        Waypoint(8.8, LEFT_PARK, FINGER_OPEN, "wait_for_right_lift_and_settle", Orient(R_park)),
        Waypoint(1.2, _LivePos(lambda: p_l_handoff(0.08)), SHAFT_OPEN, "approach_lower_shaft", Orient(R_l_handoff)),
        Waypoint(0.9, _LivePos(lambda: p_l_handoff(0.00)), SHAFT_OPEN, "seat_on_lower_shaft", Orient(R_l_handoff)),
        Waypoint(0.6, _LivePos(lambda: p_l_handoff(0.00)), SHAFT_PINCH, "pinch_lower_shaft", Orient(R_l_handoff)),
        Waypoint(0.5, _LivePos(lambda: p_l_handoff(0.00)), SHAFT_PINCH, "settle_bimanual_handoff", Orient(R_l_handoff)),
        Waypoint(1.0, _LivePos(p_l_preloaded), SHAFT_PINCH, "preload_left_lift_and_unload_right", Orient(R_l_handoff)),
        Waypoint(1.0, _LivePos(p_l_preloaded), SHAFT_PINCH, "settle_preloaded_before_release", Orient(R_l_handoff)),
        Waypoint(0.6, _LivePos(p_l_preloaded), SHAFT_PINCH, "hold_while_right_releases", Orient(R_l_handoff)),
        Waypoint(0.6, _LivePos(p_l_preloaded), SHAFT_PINCH, "hold_while_right_clears", Orient(R_l_handoff)),
        Waypoint(0.1, _LivePos(p_l_preloaded), SHAFT_PINCH, "solo_left_support_hold", Orient(R_l_handoff)),
        Waypoint(2.2, p_l_high_prebin, SHAFT_PINCH, "lift_pan_above_bin_rim_first", Orient(R_l_base)),
        Waypoint(2.8, p_l_over_bin, SHAFT_PINCH, "carry_tilt_over_wastebasket", Orient(R_l_tilt)),
        Waypoint(1.5, p_l_over_bin, SHAFT_PINCH, "settle_over_wastebasket_before_dump", Orient(R_l_tilt)),
    ]

    right_transit = [
        Waypoint(1.2, _LivePos(lambda: p_r_pick(0.08)), SHAFT_OPEN, "approach_upright_shaft", Orient(R_r0)),
        Waypoint(1.0, _LivePos(lambda: p_r_pick(0.00)), SHAFT_OPEN, "seat_on_upright_shaft", Orient(R_r0)),
        Waypoint(0.8, _LivePos(lambda: p_r_pick(0.00)), SHAFT_PINCH, "pinch_upright_shaft", Orient(R_r0)),
        Waypoint(0.9, _LivePos(lambda: p_r_pick(0.00)), SHAFT_PINCH, "settle_grip_before_lift", Orient(R_r0)),
        Waypoint(2.0, _LivePos(p_r_vlift), SHAFT_PINCH, "pure_vertical_lift", Orient(R_r0)),
        Waypoint(1.4, p_r_lift_hi, SHAFT_PINCH, "carry_inboard_closed", Orient(R_r_hi)),
        Waypoint(1.5, p_r_lift_hi, SHAFT_PINCH, "settle_held_before_transfer", Orient(R_r_hi)),
        Waypoint(3.2, p_r_lift_hi, SHAFT_PINCH, "hold_for_left_pinch", Orient(R_r_hi)),
        Waypoint(1.0, _LivePos(lambda: p_r_unloaded(0.00)), SHAFT_PINCH, "unload_right_spring_onto_left", Orient(R_r_hi)),
        Waypoint(1.0, _LivePos(lambda: p_r_unloaded(0.00)), SHAFT_PINCH, "settle_unloaded_before_release", Orient(R_r_hi)),
        Waypoint(0.6, _LivePos(lambda: p_r_unloaded(0.00)), SHAFT_OPEN, "release_upper_shaft", Orient(R_r_hi)),
        Waypoint(0.8, _LivePos(lambda: p_r_unloaded(0.11)), SHAFT_OPEN, "clear_upper_shaft", Orient(R_r_hi)),
        Waypoint(1.8, RIGHT_PARK, FINGER_OPEN, "park_right_clear", Orient(R_park)),
        Waypoint(4.6, RIGHT_PARK, FINGER_OPEN, "wait_until_transit_gate", Orient(R_park)),
    ]

    if os.environ.get("LD_MODE") == "handoff_repinch_coupon":
        import end_to_end_splice_helper_for_cake as _SP
        _lp, _rp, _ = _SP.make_post_dump_to_return3_waypoints(
            Waypoint=Waypoint, Orient=Orient, _LivePos=_LivePos, _latched=_latched, _pad_tcp=_pad_tcp,
            _handle_point_world=_handle_point_world, _handle_dir_world=_handle_dir_world, _align_z_to_dir=_align_z_to_dir,
            p_l_high_prebin=_pad_tcp((0.365, -0.220, 0.945), R_l_base), R_l_base=R_l_base, R_r0=R_r0, R_park=R_park, LEFT_PARK=LEFT_PARK,
            RIGHT_REPINCH_STAGE=(0.18, -0.36, 0.85),
            SHAFT_OPEN=SHAFT_OPEN, SHAFT_PINCH=SHAFT_PINCH, FINGER_OPEN=FINGER_OPEN, repinch_latch_step=436 + 80,
            **({"REPINCH_HOLD_TARGET": tuple(float(v) for v in os.environ["LD_REPINCH_HOLD"].split(","))}
               if os.environ.get("LD_REPINCH_HOLD") else {}),
        )
        # Truncate splice right after solo-Right R_r0 settle + 1.0 s hold (total coupon = 21.8 + 10.9 + 1.0 = 33.7 s = 674 steps):
        left = left_transit + _lp[:-1] + [Waypoint(2.0, LEFT_PARK, FINGER_OPEN, "hold_left_parked_coupon_end", Orient(R_park))]
        right = right_transit + _rp + [Waypoint(1.0, right_transit[0].pos if False else _rp[-1].pos, SHAFT_PINCH, "hold_solo_right_r0_coupon_end", _rp[-1].yaw)]
    else:
        left = left_transit + [
            Waypoint(5.1, p_l_over_bin, SHAFT_PINCH, "steady_while_right_pinches_recess", Orient(R_l_tilt)),
            Waypoint(3.0, _LivePos(_l_dump_pos), SHAFT_PINCH, "dip_and_tilt_shaft_for_steep_chute", Orient(_l_dump_rot)),
            Waypoint(2.5, _LivePos(_l_dump_pos), SHAFT_PINCH, "hold_steep_chute_for_cores", Orient(_l_dump_rot)),
            Waypoint(1.6, p_l_over_bin, SHAFT_PINCH, "restore_over_bin_shaft_pose", Orient(R_l_tilt)),
            Waypoint(3.8, p_l_over_bin, SHAFT_PINCH, "steady_shaft_during_dump_and_hold", Orient(R_l_tilt)),
        ]
        right = right_transit + [
            Waypoint(1.4, _LivePos(_stage_rec_pos), RECESS_OPEN, "stage_behind_recess", Orient(_stage_rec_rot)),
            Waypoint(1.2, _LivePos(lambda: _rec_pose(0.0, 0.08)[0]), RECESS_OPEN, "approach_rear_recess", Orient(lambda: _rec_pose(0.0, 0.08)[1])),
            Waypoint(1.0, _LivePos(lambda: _rec_pose(0.0, 0.00)[0]), RECESS_OPEN, "seat_in_rear_recess", Orient(lambda: _rec_pose(0.0, 0.00)[1])),
            Waypoint(0.7, _LivePos(lambda: _rec_pose(0.0, 0.00)[0]), RECESS_PINCH, "pinch_rear_recess", Orient(lambda: _rec_pose(0.0, 0.00)[1])),
            Waypoint(0.8, _LivePos(lambda: _rec_pose(0.0, 0.00)[0]), RECESS_PINCH, "settle_rear_recess", Orient(lambda: _rec_pose(0.0, 0.00)[1])),
            Waypoint(1.5, _LivePos(lambda: _rec_pose(0.5, 0.00)[0]), RECESS_PINCH, "lift_rear_recess_mid", Orient(lambda: _rec_pose(0.5, 0.00)[1])),
            Waypoint(1.5, _LivePos(lambda: _rec_pose(1.0, 0.00)[0]), RECESS_PINCH, "tilt_pan_to_dump_chute", Orient(lambda: _rec_pose(1.0, 0.00)[1])),
            Waypoint(2.5, _LivePos(lambda: _rec_pose(1.0, 0.00)[0]), RECESS_PINCH, "hold_dump_chute", Orient(lambda: _rec_pose(1.0, 0.00)[1])),
            Waypoint(1.6, _LivePos(lambda: _rec_pose(0.18, 0.00)[0]), RECESS_PINCH, "ease_pan_closed", Orient(lambda: _rec_pose(0.18, 0.00)[1])),
            Waypoint(0.7, _LivePos(lambda: _rec_pose(0.18, 0.00)[0]), RECESS_OPEN, "release_rear_recess", Orient(lambda: _rec_pose(0.18, 0.00)[1])),
            Waypoint(1.1, _LivePos(lambda: _rec_pose(0.18, 0.09)[0]), FINGER_OPEN, "back_off_rear_recess", Orient(lambda: _rec_pose(0.18, 0.09)[1])),
            Waypoint(2.0, _LivePos(lambda: _rec_pose(0.18, 0.09)[0]), FINGER_OPEN, "hold_post_dump_clear", Orient(lambda: _rec_pose(0.18, 0.09)[1])),   # claude-cake: hold, do not walk into the IK trap
        ]
    if os.environ.get("LD_MODE") == "integrated":        # claude-cake integration (make_integrated_patch.py)
        import end_to_end_splice_helper_for_cake as _SP
        import return_tail_interface as _RT
        _fl, _fr = sum(w.duration for w in left), sum(w.duration for w in right)
        if abs(_fl - _fr) > 1e-6:                            # align the two front halves before the splice
            (left if _fl < _fr else right).append(Waypoint(abs(_fl - _fr), (left if _fl < _fr else right)[-1].pos,
                (left if _fl < _fr else right)[-1].finger, "align_front_halves", (left if _fl < _fr else right)[-1].yaw))
        _front_steps = int(round(max(_fl, _fr) * 20))
        _lp, _rp, _tail_latch = _SP.make_post_dump_to_return3_waypoints(
            Waypoint=Waypoint, Orient=Orient, _LivePos=_LivePos, _latched=_latched, _pad_tcp=_pad_tcp,
            _handle_point_world=_handle_point_world, _handle_dir_world=_handle_dir_world, _align_z_to_dir=_align_z_to_dir,
            p_l_high_prebin=_pad_tcp((0.365, -0.220, 0.945), R_l_base), R_l_base=R_l_base, R_r0=R_r0, R_park=R_park, LEFT_PARK=LEFT_PARK,   # claude-cake int3: hr2 exit
            RIGHT_REPINCH_STAGE=(0.18, -0.36, 0.85),   # claude-cake int3: hr2 far stage
            SHAFT_OPEN=SHAFT_OPEN, SHAFT_PINCH=SHAFT_PINCH, FINGER_OPEN=FINGER_OPEN, repinch_latch_step=_front_steps + 80,
            **({"REPINCH_HOLD_TARGET": tuple(float(v) for v in os.environ["LD_REPINCH_HOLD"].split(","))}
               if os.environ.get("LD_REPINCH_HOLD") else {}))          # SEAM 2: corrected re-pinch hold (codex 2328)
        _tail = _RT.make_return_tail(Waypoint=Waypoint, Orient=Orient, _LivePos=_LivePos, _latched=_latched, _pad_tcp=_pad_tcp,
            _handle_dir_world=_handle_dir_world, PARK_XY=PARK_XY, FLOOR=FLOOR, TRUN_Z=TRUN_Z, R_r0=R_r0, R_park=R_park,
            SHAFT_PINCH=SHAFT_PINCH, SHAFT_OPEN=SHAFT_OPEN, FINGER_OPEN=FINGER_OPEN, latch_step=_tail_latch)
        _jg = _JointGoal(Q_STAGE, _demo.RIGHT_BASE_POS)   # the planner's own right base
        assert _rp[0].label == "stage_clear_while_left_exits_bin", _rp[0].label
        _rp = [Waypoint(_rp[0].duration, _jg, SHAFT_OPEN, "joint_unwind_to_repinch_stage", Orient(_jg.R))] + list(_rp[1:])   # claude-cake
        left += _lp; right += _rp + _tail
        _dl, _dr = sum(w.duration for w in left), sum(w.duration for w in right)
        left.append(Waypoint(_dr - _dl, LEFT_PARK, FINGER_OPEN, "left_hold_parked_to_end", Orient(R_park)))
        _state["int_meta"] = {"front_steps": _front_steps, "repinch_latch_step": _front_steps + 80, "tail_latch_step": _tail_latch,
                              "total_s": round(_dr, 2)}
    return {"left": left, "right": right}


# ===================== OWNER SCORER (claude-cake; byte-exact from the reargrip2-lineage task, make_owner_scorer_patch.py) ====
RELEASE = 0.080
STILL_N = 21
STILL_TOL = 0.005  # 5.0 mm over 1.0 s (21 samples at 20 Hz)

_OS = {}


def _tips(env) -> np.ndarray:
    from yam_arena import runtime as _rt

    b = env.unwrapped
    o = b.scene.env_origins[0].cpu().numpy()
    out = []
    for n in ("left_robot", "right_robot"):
        robot = b.scene[n]
        mid = _rt.pad_tips_world(robot) - o
        g = _rt.link_poses(robot)["gripper"]
        ids = robot.find_joints(["joint7", "joint8"], preserve_order=True)[0]
        q7, q8 = (abs(float(v)) for v in labkit._np(robot.data.joint_pos)[0, ids])
        out += [mid, mid - q7 * g[:3, 0], mid + q8 * g[:3, 0]]
    return np.asarray(out, float)


def _tracked_points(dp_st, wb_st, items) -> np.ndarray:
    pts = [
        dp_st.to_world((-0.16, REAR_Y, 0.0)),
        dp_st.to_world((+0.16, FRONT_Y, PAN_H)),
        dp_st.to_world((0.0, RECESS_Y, RECESS_Z)),
        _handle_point_world(dp_st, ASSEMBLED_H),
        wb_st.to_world((-BIN_L * 0.5, -BIN_W * 0.5, 0.0)),
        wb_st.to_world((+BIN_L * 0.5, +BIN_W * 0.5, BIN_H)),
    ] + [st_.to_world(mk) for st_, nm in zip(items, ITEM_NAMES) for mk in _item_marks(nm)]
    return np.asarray(pts, float)


def _min_hand_clearance(tips: np.ndarray, dp_st, wb_st, items) -> float:
    if tips.ndim != 2 or tips.shape != (6, 3) or not np.isfinite(tips).all():
        return 0.0
    best = 9.0
    q_h = float(dp_st.joints["handle"])
    Rh = _rx_mat(q_h)
    for t in tips:
        # 1. Pan hopper + rear recess AABB in pan frame
        ql = np.asarray(dp_st.to_local(t), float)
        if not np.isfinite(ql).all():
            return 0.0
        d_pan = np.maximum(np.abs(ql - np.array([0.0, -0.032, 0.064])) - np.array([0.176, 0.156, 0.066]), 0.0)
        best = min(best, float(np.linalg.norm(d_pan)))
        # 2. Handle + yoke in handle frame
        qh_pt = Rh.T @ (ql - np.array([0.0, 0.0, TRUN_Z])) + np.array([0.0, 0.0, TRUN_Z])
        for c0, h0 in (((0.0, 0.0, 0.180), (0.180, 0.015, 0.080)), ((0.0, 0.0, 0.580), (0.020, 0.020, 0.340))):
            dh = np.maximum(np.abs(qh_pt - np.asarray(c0)) - np.asarray(h0), 0.0)
            best = min(best, float(np.linalg.norm(dh)))
        # 3. Wastebasket AABB in wastebasket frame
        qw = np.asarray(wb_st.to_local(t), float)
        dw = np.maximum(np.abs(qw - np.array([0.0, 0.0, BIN_H * 0.5])) - np.array([BIN_L * 0.5, BIN_W * 0.5, BIN_H * 0.5]), 0.0)
        best = min(best, float(np.linalg.norm(dw)))
        # 4. Every debris item (the tape core exactly; the others by bounding sphere, conservative)
        for cs, nm in zip(items, ITEM_NAMES):
            qc = np.asarray(cs.to_local(t), float)
            if nm == "core_1":
                dr = max(0.0, math.hypot(qc[0], qc[1]) - CORE_OD * 0.5)
                dz = max(0.0, abs(qc[2]) - CORE_H * 0.5)
                best = min(best, math.hypot(dr, dz))
            else:
                best = min(best, max(0.0, float(np.linalg.norm(qc)) - _item_bound_r(nm)))
    return best


def _owner_core_status(cs, wb_st, dp_st, name="core_1") -> str:
    # Check inside wastebasket in wastebasket's own local frame (allows harmless bin slide!)
    pw = np.asarray(wb_st.to_local(cs.pos), float)
    # rev3: orientation-aware extents per item shape (core / cap cylinders, ball sphere, card box)
    lo, hi = _item_lohi(name, cs, wb_st)
    in_bin_xy = (max(abs(pw[0] + lo[0]), abs(pw[0] + hi[0])) <= BIN_L * 0.5 - BIN_WALL_T + 0.002) and (max(abs(pw[1] + lo[1]), abs(pw[1] + hi[1])) <= BIN_W * 0.5 - BIN_WALL_T + 0.002)
    in_bin_z = (BIN_FLOOR_T - 0.005 <= pw[2] + lo[2]) and (pw[2] + hi[2] <= BIN_H - 0.050)
    if in_bin_xy and in_bin_z:
        return "in_bin"
    pp = np.asarray(dp_st.to_local(cs.pos), float)
    lp, hp = _item_lohi(name, cs, dp_st)
    if (max(abs(pp[0] + lp[0]), abs(pp[0] + hp[0])) <= PAN_W * 0.5 - PAN_T + 0.002 and REAR_Y + 0.010 - 0.002 <= pp[1] + lp[1]
            and pp[1] + hp[1] <= FRONT_Y + 0.002 and 0.004 - 0.005 <= pp[2] + lp[2] and pp[2] + hp[2] <= PAN_H + 0.025):   # claude-cake (codex 0029/0039): whole core inside the authored inner faces and the mouth line
        return "in_pan"
    return "outside"


def _new_transfer_tracker():
    return CoreTransfer(
        PAN_W * 0.5 - PAN_T, REAR_Y + 0.010, FRONT_Y, PAN_H,
        BIN_L * 0.5 - BIN_WALL_T, BIN_W * 0.5 - BIN_WALL_T, BIN_H,
    )


def _advance_transfers(dp_st, wb_st, cores):
    rear = dp_st.to_world((0.0, REAR_Y, 0.004))
    lip = dp_st.to_world((0.0, FRONT_Y, 0.004))
    slope = float((rear[2] - lip[2]) / PAN_D)
    lip_in_bin = tuple(wb_st.to_local(lip))
    for tracker, core in zip(_OS["transfers"], cores):
        tracker.update(
            _OS["n"] + 1,
            TransferSample(
                tuple(dp_st.to_local(core.pos)),
                tuple(wb_st.to_local(core.pos)),
                lip_in_bin,
                slope,
            ),
        )


def score(objects) -> float:
    dp_st = objects["dustpan"]
    wb_st = objects["wastebasket"]
    items = [objects[nm] for nm in ITEM_NAMES]

    if not (_OS.get("far", False) and _OS.get("still", False)):
        return 0.0
    contacts = _OS.get("contacts")
    if contacts is None or not contacts.verified or contacts.first_hand_contact is not None:
        return 0.0
    if wb_st.tilt_deg() > 10.0 or (float(wb_st.pos[2]) - FLOOR) > 0.015:
        return 0.0

    statuses = [_owner_core_status(st_, wb_st, dp_st, nm) for st_, nm in zip(items, ITEM_NAMES)]
    if "outside" in statuses or "in_bin" not in statuses:
        return 0.0
    if any(status == "in_bin" and tracker.entered_step is None
           for status, tracker in zip(statuses, _OS["transfers"])):
        return 0.0

    hdir = _handle_dir_world(dp_st)
    h_vert_deg = math.degrees(math.acos(float(np.clip(hdir[2], -1.0, 1.0))))
    ph = _OS.get("park_hist") or []                      # rev3 tape rule over the whole final 1 s window (codex 1614)
    pan_parked = len(ph) >= STILL_N and all(ph[-STILL_N:])
    n_in = statuses.count("in_bin")                   # every in-bin item is tip-credited (checked above)
    if n_in == len(ITEMS) and pan_parked:
        return 1.0
    return 0.5 if n_in >= 2 else 0.0                   # 0.5: >= 2 of 4 tipped in; parking on the mark is NOT needed (Jingyi 1610)


def success(objects) -> bool:
    return score(objects) == 1.0


def distance(objects) -> float:
    return 1.0 - score(objects)


@on_reset
def _owner_reset(env):
    previous_contacts = _OS.get("contacts")
    if previous_contacts is not None:
        previous_contacts.close()
    _OS.clear()
    _OS.update(
        n=0, clear_hist=[], pts_hist=[], park_hist=[], far=False, still=False,
        contacts=TensorCoreContactMonitor.attach(env),
        transfers=[_new_transfer_tracker() for _ in ITEMS],
    )


@on_step
def _owner_step(env):
    dp_st = labkit._part_state(env, DUSTPAN)
    wb_st = labkit._part_state(env, WASTEBASKET)
    items = [labkit._part_state(env, it) for it in ITEMS]
    _advance_transfers(dp_st, wb_st, items)
    _park_update(dp_st)
    clr = _min_hand_clearance(_tips(env), dp_st, wb_st, items)
    pts = _tracked_points(dp_st, wb_st, items)
    _OS["clear_hist"].append(clr); del _OS["clear_hist"][:-STILL_N]
    _OS["far"] = (len(_OS["clear_hist"]) == STILL_N and all(math.isfinite(c) for c in _OS["clear_hist"])
                  and min(_OS["clear_hist"]) >= RELEASE)
    _OS["pts_hist"].append(pts); del _OS["pts_hist"][:-STILL_N]
    h = np.asarray(_OS["pts_hist"], float)
    mm = float(np.linalg.norm(h[:, None] - h[None, :], axis=3).max()) if len(h) == STILL_N and np.isfinite(h).all() else 9.9
    _OS["still"] = (len(h) == STILL_N and np.isfinite(h).all() and mm <= STILL_TOL)
    _OS["n"] += 1
    if _OS["n"] == 1 or _OS["n"] % 10 == 0:
        st = [_owner_core_status(st_, wb_st, dp_st, nm) for st_, nm in zip(items, ITEM_NAMES)]
        hd = _handle_dir_world(dp_st)
        hv = math.degrees(math.acos(float(np.clip(hd[2], -1.0, 1.0))))
        sc = score({"dustpan": dp_st, "wastebasket": wb_st, **{nm: st_ for nm, st_ in zip(ITEM_NAMES, items)}})
        print(f"[score] step={_OS['n']} hand_obj_clr_mm={clr * 1e3:.1f} far={_OS['far']} still={_OS['still']} move_mm={mm * 1e3:.2f} "
              f"items={dict(zip(ITEM_NAMES, st))} pan=({dp_st.pos[0]:.3f},{dp_st.pos[1]:+.3f},{dp_st.pos[2]:.3f}) tilt={dp_st.tilt_deg():.1f} "
              f"h_vert={hv:.1f} bin_tilt={wb_st.tilt_deg():.1f} tape_margin_mm={_tape_margin(dp_st) * 1e3:.1f} score_now={sc} "
              f"mouth_steps={[tr.departure_step for tr in _OS['transfers']]} "
              f"rim_steps={[tr.entered_step for tr in _OS['transfers']]} "
              f"core_reporter_verified={_OS['contacts'].verified} " f"physics_samples={_OS['contacts'].sample_count} " f"support_pairs={_OS['contacts'].support_events} "
              f"arm_core={_OS['contacts'].first_hand_contact}", flush=True)
