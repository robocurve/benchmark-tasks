"""Aspirate the spent medium from a tilted T75 culture flask, the pipette tip resting at its back..."""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import ourkit as k  # noqa: E402
from ourkit import demo as D  # noqa: E402
from ourkit.scene import rpy  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

P = D.P
Grip = D.Grip
SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0
TOP = TABLE_TOP[SCENE]
DT = 0.05
INSTRUCTION = ("Aspirate the spent medium out of the open flask with the aspirator without touching the cell layer, "
               "then lay the flask flat on its outline, put the aspirator back on its stand and let go.")

# == the scoring numbers
V0_ML = 15.0
RATE_ML_S = 2.0
FILM_LEVEL, FILM_DRAINED, DRAIN_DEG = 0.25e-3, 0.02e-3, 20.0
GROWTH_AREA = 75e-4
STRIP = 0.010
TOUCH_MM = 1.02
V_FULL, V_075, V_050, V_025 = 0.50, 1.0, 3.0, 10.0
FLAT_DEG = 5.0
OUTLINE = 0.020
RETURN_MM, RETURN_DEG = 5.0, 5.0
CLEAR_MM = 20.0
STILL_MM, STILL_S = 1.0, 2.0
DROP, DROP_T = 0.020, 0.25

# == the props (positions in m from the arm bases' midpoint, x forward, y left; heights above the table)
CLEAR, GREY, PLA, STEEL, RED, TAPE = ((0.86, 0.90, 0.93), (0.30, 0.32, 0.36), (0.95, 0.95, 0.92), (0.55, 0.57, 0.60),
                                      (0.80, 0.15, 0.15), (0.25, 0.55, 0.88))
GAP = 0.0003

FLASK_L, FLASK_W, FLASK_H, WALL = 0.1241, 0.0842, 0.0396, 0.0015
L, W, H = FLASK_L, FLASK_W, FLASK_H
THICK = 0.0045
NECK_D, NECK_L, NECK_PROUD = 0.0215, 0.022, 0.008
BORE_R = NECK_D / 2
NECK_N = 12
R_IN = BORE_R * math.cos(math.pi / NECK_N)
NECK_T = 0.005
NECK_X0, NECK_X1 = L + NECK_PROUD - NECK_L, L + NECK_PROUD
NECK_Z = H / 2
IN_LO = np.array([WALL, -W / 2 + WALL, WALL])
IN_HI = np.array([L - WALL, W / 2 - WALL, H - WALL])
G_LOC = np.array([0.055, 0.0, H / 2])
MOUTH = np.array([NECK_X1, 0.0, NECK_Z])
NECK_MID = np.array([(NECK_X0 + NECK_X1) / 2, 0.0, NECK_Z])
FLASK_M, MEDIUM_M = 0.060, 0.015
FRICTION = 0.5
BACK_Y, CENTRE_X = 0.100, 0.290

def _fb(centre, size, color=CLEAR, label="", turn=(0.0, 0.0, 0.0), ghost=False):
    return k.box(np.asarray(centre, float) - G_LOC, size, color, label=label, turn=turn, ghost=ghost)


_HOLE = 0.011
FLASK_BOXES = [
    _fb(((L - THICK) / 2, 0, (WALL - THICK) / 2), (L + THICK, W, WALL + THICK), label="floor"),
    _fb(((WALL - THICK) / 2, 0, (WALL + H) / 2), (WALL + THICK, W, H - WALL), label="backwall"),
    _fb((L / 2, 0, H - WALL / 2), (L, W, WALL), label="topwall"),
    _fb((L / 2, W / 2 - WALL / 2, H / 2), (L, WALL, H - 2 * WALL), label="side_a"),
    _fb((L / 2, -W / 2 + WALL / 2, H / 2), (L, WALL, H - 2 * WALL), label="side_b"),
    _fb((L / 2, 0, WALL + 0.0005), (L - 2 * WALL - 0.002, W - 2 * WALL - 0.002, 0.001), RED, label="medium",
        ghost=True),
]
_FY = (W / 2 - WALL + _HOLE) / 2
FLASK_BOXES += [_fb((L - WALL / 2, s * _FY, H / 2), (WALL, W / 2 - WALL - _HOLE, H - 2 * WALL), label=f"front_{i}")
                for i, s in enumerate((1, -1))]
FLASK_BOXES += [_fb((L - WALL / 2, 0, (WALL + NECK_Z - _HOLE) / 2), (WALL, 2 * _HOLE, NECK_Z - _HOLE - WALL),
                    label="front_lo"),
                _fb((L - WALL / 2, 0, (H - WALL + NECK_Z + _HOLE) / 2), (WALL, 2 * _HOLE, H - WALL - NECK_Z - _HOLE),
                    label="front_hi")]
_NW = 2 * (R_IN + NECK_T) * math.tan(math.pi / NECK_N)
for _i in range(NECK_N):
    _f = 2 * math.pi * _i / NECK_N
    _r = R_IN + NECK_T / 2
    FLASK_BOXES.append(_fb(((NECK_X0 + NECK_X1) / 2, -_r * math.sin(_f), NECK_Z + _r * math.cos(_f)),
                           (NECK_L, _NW, NECK_T), label=f"neck_{_i}", turn=(_f, 0.0, 0.0)))

R0 = rpy((0.0, 0.0, -math.pi / 2))
O_FLAT = np.array([CENTRE_X, BACK_Y, TOP + THICK + GAP])
PIVOT = np.array([-THICK, W / 2, -THICK])
C0 = O_FLAT + R0 @ PIVOT
FLASK = k.mech("flask", tuple(O_FLAT + R0 @ G_LOC), FLASK_BOXES, FLASK_M + MEDIUM_M, yaw=-math.pi / 2,
               color=CLEAR, surface=FRICTION)

LEVER, PIP_D, PIP_L, HP_D, HP_L = 0.270, 0.00618, 0.275, 0.025, 0.130
_SP = HP_D * math.tan(math.pi / 8)
ASP_BOXES = [k.box((0.010 - HP_L / 2, 0, 0), (HP_L, HP_D, _SP), GREY, turn=(i * math.pi / 4, 0, 0),
                   label=f"handpiece_{i}") for i in range(4)]
ASP_BOXES += [k.box((0.015, 0, 0), (0.010, 0.012, 0.012), GREY, label="adapter"),
              k.box((LEVER - PIP_L / 2, 0, 0), (PIP_L, PIP_D, PIP_D), CLEAR, label="pipette")]
ASP_M = 0.065
UP = np.array([0.0, 0.0, 1.0])


def tool_rot(d) -> np.ndarray:
    """The tool's rotation with its axis along d (tip-forward) and its y axis horizontal (up x d)."""
    d = np.asarray(d, float) / np.linalg.norm(d)
    y = np.cross(UP, d)
    y /= np.linalg.norm(y)
    return np.column_stack([d, y, np.cross(d, y)])


def heading_dip(h_deg, dip_deg):
    h, p = math.radians(h_deg), math.radians(dip_deg)
    return np.array([math.cos(p) * math.cos(h), math.cos(p) * math.sin(h), -math.sin(p)])


STAND_X, STAND_Y, STAND_Z, STAND_HEAD, STAND_DIP = 0.200, -0.180, 0.220, 90.0, 35.0
STAND_G = np.array([STAND_X, STAND_Y, TOP + STAND_Z])
R_STAND = tool_rot(heading_dip(STAND_HEAD, STAND_DIP))
ASPIRATOR = k.mech("aspirator", tuple(STAND_G + 0.0006 * UP), ASP_BOXES, ASP_M, yaw=math.radians(STAND_HEAD),
                   pitch=math.radians(STAND_DIP), color=GREY, surface=FRICTION)
STAND_M = 2.0
BASE = (0.100, 0.200, 0.006)


def _stand_boxes() -> list:
    """Printed cradle in the tool's rest frame:"""
    g, out = GAP, []
    out += [k.box((-0.095, 0, -(HP_D / 2 + 0.003 + g)), (0.030, HP_D + 0.016, 0.006), PLA),
            k.box((0.045, 0, -(PIP_D / 2 + 0.003 + g)), (0.020, PIP_D + 0.016, 0.006), PLA),
            k.box((0.0235 + g, 0, -0.010), (0.023, 0.012, 0.006), PLA, label="stop")]
    for s in (1, -1):
        out.append(k.box((-0.095, s * (HP_D / 2 + 0.006 + g), -0.005), (0.030, 0.004, 0.014), PLA))
        out.append(k.box((0.045, s * (PIP_D / 2 + 0.006 + g), -0.001), (0.020, 0.004, 0.010), PLA))
    turn = (0.0, -math.radians(STAND_DIP), 0.0)
    for x_t, below in ((-0.095, HP_D / 2 + 0.006), (0.040, 0.013)):
        top = STAND_G + R_STAND @ np.array([x_t, 0.0, -below])
        h = float(top[2] - TOP) - BASE[2]
        c = np.array([top[0], top[1], TOP + BASE[2] + h / 2])
        out.append(k.box(R_STAND.T @ (c - STAND_G), (0.012, 0.012, h - 0.001), PLA, turn=turn))
    c = np.array([STAND_X, STAND_Y + 0.010, TOP + BASE[2] / 2 + g])
    out.append(k.box(R_STAND.T @ (c - STAND_G), (BASE[1], BASE[0], BASE[2]), STEEL, turn=turn, label="base"))
    return out


STAND_BOXES = _stand_boxes()
STAND = k.mech("stand", tuple(STAND_G), STAND_BOXES, STAND_M, yaw=math.radians(STAND_HEAD),
               pitch=math.radians(STAND_DIP), color=PLA, surface=FRICTION)


TAPE_W, TAPE_OFF = 0.012, 0.004


def _tape() -> list:
    """The outline the flask is laid back on."""
    tw, off, t = TAPE_W, TAPE_OFF, 0.0005
    x0, x1 = CENTRE_X - W / 2 - off, CENTRE_X + W / 2 + off
    y0, y1 = BACK_Y - L - off, BACK_Y + THICK + off
    z = TOP + t / 2
    return [Box("tape_a", pos=(x0 - tw / 2, (y0 + y1) / 2, z), size=(tw, y1 - y0 + 2 * tw, t), color=TAPE),
            Box("tape_b", pos=(x1 + tw / 2, (y0 + y1) / 2, z), size=(tw, y1 - y0 + 2 * tw, t), color=TAPE),
            Box("tape_c", pos=(CENTRE_X, y1 + tw / 2, z), size=(x1 - x0, tw, t), color=TAPE),
            Box("tape_d", pos=(CENTRE_X, y0 - tw / 2, z), size=(x1 - x0, tw, t), color=TAPE)]


# == the biological safety cabinet the robot works in
BSC_W, BSC_H, BSC_D = 1.199, 0.780, 0.630
BSC_FRONT = -0.150
BSC_BACK = BSC_FRONT + BSC_D
FRONT_GRILLE, REAR_GRILLE = 0.100, 0.050
SASH_OPEN = 0.254
SS, GLASS, GRILLE = (0.88, 0.89, 0.90), (0.80, 0.90, 0.95), (0.22, 0.23, 0.25)


def _slats(name: str, x0: float, depth: float, pitch: float = 0.010, bar: float = 0.005) -> list:
    """A grille's perforated look:"""
    n = int(round(depth / pitch))
    return [Box(f"{name}_slat_{i}", pos=(x0 + (i + 0.5) * pitch, 0.0, TOP + 0.00325), size=(bar, BSC_W, 0.0005),
                color=SS) for i in range(n)]


def _cabinet() -> list:
    """Work surface, back wall, side walls, the front and rear grilles, and the sash frame (glass and..."""
    w2, t, rail = BSC_W / 2, 0.010, 0.030
    cz = TOP + BSC_H / 2
    sx = BSC_FRONT - FRONT_GRILLE
    return [Box("bsc_work_surface", pos=(sx + (BSC_BACK - sx) / 2, 0.0, TOP + 0.0001), size=(BSC_BACK - sx, BSC_W, 0.0004),
                color=SS),
            Box("bsc_back_wall", pos=(BSC_BACK + t / 2, 0.0, cz), size=(t, BSC_W, BSC_H), color=SS),
            Box("bsc_left_wall", pos=(BSC_FRONT + BSC_D / 2, w2 + t / 2, cz), size=(BSC_D, t, BSC_H), color=SS),
            Box("bsc_right_wall", pos=(BSC_FRONT + BSC_D / 2, -w2 - t / 2, cz), size=(BSC_D, t, BSC_H), color=SS),
            Box("bsc_rear_grille", pos=(BSC_BACK - REAR_GRILLE / 2, 0.0, TOP + 0.0015), size=(REAR_GRILLE, BSC_W, 0.003),
                color=GRILLE),
            Box("bsc_front_grille", pos=(BSC_FRONT - FRONT_GRILLE / 2, 0.0, TOP + 0.0015),
                size=(FRONT_GRILLE, BSC_W, 0.003), color=GRILLE),
            Box("bsc_left_front", pos=(sx + FRONT_GRILLE / 2, w2 + t / 2, cz), size=(FRONT_GRILLE, t, BSC_H), color=SS),
            Box("bsc_right_front", pos=(sx + FRONT_GRILLE / 2, -w2 - t / 2, cz), size=(FRONT_GRILLE, t, BSC_H), color=SS),
            Box("bsc_sash_rail", pos=(sx - rail / 2, 0.0, TOP + SASH_OPEN + rail / 2), size=(rail, BSC_W, rail), color=SS),
            Box("bsc_sash_header", pos=(sx - rail / 2, 0.0, TOP + BSC_H - rail / 2), size=(rail, BSC_W, rail), color=SS),
            *_slats("bsc_rear", BSC_BACK - REAR_GRILLE, REAR_GRILLE), *_slats("bsc_front", sx, FRONT_GRILLE),
            ]


OBJECTS = [FLASK, ASPIRATOR, STAND, *_tape(), *_cabinet()]
WORK_IN = 0.1016
GRILLE_Z = 0.030
SASH_X = BSC_FRONT - FRONT_GRILLE
ARM_EVERY = 4

# == the medium: a volume rule on the flask's real interior
_GRID = 48
_G = (np.arange(_GRID) + 0.5) / _GRID


def vol_below(rot, origin, h) -> float:
    """m3 of the flask interior below the world plane z = h (columns along the steepest axis)."""
    n = rot[2, :]
    e = h - origin[2] - float(n @ IN_LO)
    size = IN_HI - IN_LO
    a = int(np.argmax(np.abs(n)))
    i, j = [b for b in range(3) if b != a]
    qi, qj = np.meshgrid(_G * size[i], _G * size[j], indexing="ij")
    col = np.clip((e - n[i] * qi - n[j] * qj) / n[a], 0.0, size[a])
    if n[a] < 0:
        col = size[a] - col
    return float(col.sum() * size[i] * size[j] / _GRID ** 2)


def surface(rot, origin, vol) -> float:
    """World height of the level surface holding ``vol`` m3 in the interior."""
    corners = np.array([[x, y, z] for x in (IN_LO[0], IN_HI[0]) for y in (IN_LO[1], IN_HI[1])
                        for z in (IN_LO[2], IN_HI[2])])
    zs = origin[2] + corners @ rot[2, :]
    lo, hi = float(zs.min()), float(zs.max())
    for _ in range(28):
        mid = (lo + hi) / 2
        if vol_below(rot, origin, mid) < vol:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def film(slope_deg: float) -> float:
    """m3 kept as a film on the floor at this floor slope."""
    s = min(max(slope_deg, 0.0), DRAIN_DEG) / DRAIN_DEG
    return GROWTH_AREA * (FILM_LEVEL + (FILM_DRAINED - FILM_LEVEL) * s)


def flask_frame(st):
    """(rotation, local-frame origin) of the flask as it is."""
    R = st.rotation()
    return R, np.asarray(st.pos, float) - R @ G_LOC


def floor_slope(R) -> float:
    return math.degrees(math.acos(float(np.clip(R[2, 2], -1.0, 1.0))))


def lip_z(R, o) -> float:
    """World height of the mouth's lowest point."""
    return float((o + R @ MOUTH)[2] - BORE_R * math.hypot(R[2, 1], R[2, 2]))


_XS = np.append(np.arange(LEVER - PIP_L, LEVER - 0.001, 0.004), LEVER)
_HP = PIP_D / 2
PIP_PTS = np.array([[x, b * _HP, c * _HP] for x in _XS for b in (-1, 1) for c in (-1, 1)])
TIP_PTS = np.array([[LEVER, b * _HP, c * _HP] for b in (-1, 1) for c in (-1, 1)])


def in_flask(pts_world, R, o) -> np.ndarray:
    return (np.asarray(pts_world) - o) @ R


def medium_step(V, R, o, tip_world, dt=DT):
    """One step of the volume rule:"""
    fv = min(film(floor_slope(R)), V)
    pool = V - fv
    if pool <= 1e-10:
        return V, None, False
    h = surface(R, o, pool)
    face_l = in_flask(tip_world, R, o)
    inside = bool(np.all((face_l > IN_LO - 1e-3) & (face_l < IN_HI + 1e-3)))
    rim = float(np.asarray(tip_world)[:, 2].min())
    if inside and rim < h:
        pool = max(vol_below(R, o, rim), pool - RATE_ML_S * 1e-6 * dt, 0.0)
        return pool + fv, (surface(R, o, pool) if pool > 1e-10 else rim), True
    return V, h, False


def cell_touch(pip_world, R, o) -> bool:
    """Any pipette point within TOUCH_MM of the floor's inside more than STRIP from the back wall."""
    q = in_flask(pip_world, R, o)
    over = (q[:, 0] > WALL + STRIP) & (q[:, 0] < IN_HI[0]) & (np.abs(q[:, 1]) < IN_HI[1])
    return bool(np.any(q[over, 2] < IN_LO[2] + TOUCH_MM / 1000.0))


def near_wall(pip_world, R, o, mm=1.5) -> bool:
    """Any pipette point within ``mm`` of the inside of a wall or the neck (the servo stops reading the..."""
    q = in_flask(pip_world, R, o)
    m = mm / 1000.0
    inside = (q[:, 0] > IN_LO[0] - m) & (q[:, 0] < IN_HI[0]) & (np.abs(q[:, 1]) < IN_HI[1] + m) & (q[:, 2] < IN_HI[2] + m)
    d = np.minimum.reduce([q[:, 0] - IN_LO[0], IN_HI[1] - np.abs(q[:, 1]), q[:, 2] - IN_LO[2], IN_HI[2] - q[:, 2]])
    neck = (q[:, 0] > NECK_X0 - WALL) & (q[:, 0] < NECK_X1 + m)
    rad = np.hypot(q[:, 1], q[:, 2] - NECK_Z)
    return bool(np.any(inside & (d < m)) or np.any(neck & (rad > R_IN - m) & (rad < R_IN + NECK_T)))


def world_pts(st, local) -> np.ndarray:
    return np.asarray(st.pos, float) + np.asarray(local) @ st.rotation().T


_FOOT = np.array([[x, y, -THICK] for x in (-THICK, L) for y in (-W / 2, W / 2)])
_FL_PTS = np.array([[x, y, z] for x in np.linspace(-THICK, NECK_X1, 8) for y in np.linspace(-W / 2, W / 2, 5)
                    for z in np.linspace(-THICK, H, 4)]) - G_LOC
_ASP_PTS = np.array([[x, b * HP_D / 2, c * HP_D / 2] for x in np.linspace(0.010 - HP_L, 0.010, 7) for b in (-1, 1)
                     for c in (-1, 1)] + [[x, 0, 0] for x in np.linspace(0.02, LEVER, 12)])
STAND_ROT = rpy((0.0, math.radians(STAND_DIP), math.radians(STAND_HEAD)))

# == the scorer
RUN: dict = {}
SPILL, DROPPED, TOUCHED = k.Sticky(), k.Sticky(), k.Sticky()
CABINET = k.Sticky()
END = k.Settle(STILL_S, STILL_MM / 1000.0)
REC = k.run_record.RunRecord(__file__, "flask", "aspirator")


def _reset_run(states=None) -> None:
    RUN.clear()
    RUN.update(V=V0_ML * 1e-6, ml=V0_ML, k=0, hist={"flask": [], "aspirator": []}, asp0=None, flat=False,
               returned=False, outline_mm=None, tilt=0.0, ret_mm=None, ret_deg=None, h=None, drawing=False,
               near=False, tint=10)
    if states and "aspirator" in states:
        a = states["aspirator"]
        RUN["asp0"] = (np.asarray(a.pos, float).copy(), a.rotation().copy())


_reset_run()


def put_back(R, o, asp) -> tuple[bool, bool]:
    """(flask flat on its outline, hand piece back in its saddles)."""
    foot = o + _FOOT @ R.T
    foot0 = O_FLAT + _FOOT @ R0.T
    RUN["outline_mm"] = round(1000 * float(np.linalg.norm((foot - foot0)[:, :2], axis=1).max()), 1)
    RUN["tilt"] = round(floor_slope(R), 2)
    flat = RUN["tilt"] <= FLAT_DEG and RUN["outline_mm"] <= OUTLINE * 1000
    p0, r0 = RUN["asp0"] if RUN.get("asp0") else (STAND_G + 0.0006 * UP, STAND_ROT)
    RUN["ret_mm"] = round(1000 * float(np.linalg.norm(np.asarray(asp.pos) - p0)), 2)
    RUN["ret_deg"] = round(math.degrees(math.acos(float(np.clip((np.trace(r0.T @ asp.rotation()) - 1) / 2, -1, 1)))), 2)
    return flat, RUN["ret_mm"] <= RETURN_MM and RUN["ret_deg"] <= RETURN_DEG


def grade(ml: float, touched: bool, spill: bool, dropped: bool, put_back_ok: bool) -> float:
    """The ladder:"""
    if spill or dropped or ml > V_025 + 1e-9:
        return 0.0
    if touched:
        return 0.25
    if ml <= V_FULL + 1e-9 and put_back_ok:
        return 1.0
    if (ml <= V_075 + 1e-9 and put_back_ok) or ml <= V_FULL + 1e-9:
        return 0.75
    if ml <= V_050 + 1e-9:
        return 0.5
    return 0.25


def score(states) -> float:
    if "flask" not in states or "aspirator" not in states:
        return 0.0
    R, o = flask_frame(states["flask"])
    flat, ret = put_back(R, o, states["aspirator"])
    RUN["flat"], RUN["returned"] = flat, ret
    return grade(RUN["ml"], bool(TOUCHED), bool(SPILL) or bool(CABINET), bool(DROPPED), flat and ret and END.ok)


def success(states) -> bool:
    return score(states) >= 1.0


def distance(states) -> float:
    return float(RUN.get("ml", V0_ML))


@k.on_reset
def _reset(states) -> None:
    SPILL.reset(), DROPPED.reset(), TOUCHED.reset(), END.reset(), CABINET.reset()
    _reset_run(states)
    REC.reset()


@k.on_step
def _watch(states) -> None:
    if "flask" not in states or "aspirator" not in states:
        return
    fl, asp = states["flask"], states["aspirator"]
    R, o = flask_frame(fl)
    RUN["k"] += 1
    V, h, drawing = medium_step(RUN["V"], R, o, world_pts(asp, TIP_PTS))
    RUN.update(V=V, ml=round(V * 1e6, 3), h=h, drawing=drawing)
    if h is not None and h > lip_z(R, o):
        SPILL.trip("the medium reached the mouth's lip", RUN["k"] * DT)
    pip = world_pts(asp, PIP_PTS)
    if cell_touch(pip, R, o):
        TOUCHED.trip("pipette on the cell layer", RUN["k"] * DT)
    RUN["near"] = near_wall(pip, R, o)
    for name, st in (("flask", fl), ("aspirator", asp)):
        hist = RUN["hist"][name]
        hist.append(float(st.pos[2]))
        del hist[:-int(round(DROP_T / DT)) - 1]
        if max(hist) - hist[-1] > DROP:
            DROPPED.trip(f"{name} dropped", RUN["k"] * DT)
    still_pts = np.concatenate([world_pts(fl, _FOOT - G_LOC), world_pts(asp, TIP_PTS[:1]), [np.asarray(asp.pos)]])
    clear = k.hands_clear(np.concatenate([world_pts(fl, _FL_PTS), world_pts(asp, _ASP_PTS)]), CLEAR_MM)
    END.push(still_pts.ravel(), clear)
    tint = int(math.ceil(10 * RUN["ml"] / V0_ML))
    if tint != RUN["tint"]:
        RUN["tint"] = tint
        f = tint / 10
        k.recolor("medium", tuple(float(c) for c in f * np.array(RED) + (1 - f) * np.array(CLEAR)))
    REC.tick(states, lambda: f"ml {RUN['ml']:.2f} drawing {int(drawing)} near {int(RUN['near'])} "
                             f"tilt {floor_slope(R):.1f} score {score(states):g}")


# == the demo: two servoed arms (the crux prototype's) on ourkit
from servo import FEED, SRV, Servo, Way, _WayPos, way_wp  # noqa: E402

k.on_step(FEED.update)
k.on_reset(FEED.reset)

PITCH, ROLL = math.radians(35.0), math.radians(15.0)
TARGET = np.array([IN_LO[0] + 0.0045, IN_HI[1] - 0.014, IN_LO[2] + 0.0050])
HOVER = TARGET + np.array([0.0, 0.0, 0.010])
MIDDLE = np.array([IN_LO[0] + 0.030, IN_HI[1] - 0.014, IN_LO[2] + 0.0040])


XR0 = np.column_stack([[0, 0, -1.0], [1.0, 0, 0], [0, -1.0, 0]])
_A = np.array([0.0, -math.sin(math.radians(20)), -math.cos(math.radians(20))])
_C = np.array([-1.0, 0.0, 0.0])
ML_WORLD = np.column_stack([_C, np.cross(_A, _C), _A])
ML0 = R0.T @ ML_WORLD
RS = Servo("right", "aspirator", XR0, np.zeros(3), gain=0.08, cap_p=0.015, cap_r=math.radians(4.0))
LS = Servo("left", "flask", ML0, np.zeros(3), gain=0.05, cap_p=0.008, cap_r=math.radians(2.5))
LS.read_once = True
TIMES = {"press": [1e9, 1e9], "left_open": 1e9, "right_held": 1e9}
_PLAN: dict = {}


def flask_pose(theta: float, phi: float):
    """(rotation, local origin) of the flask pitched ``theta`` neck-up and rolled ``phi`` (+y side low)..."""
    R = R0 @ rpy((0.0, -theta, 0.0)) @ rpy((-phi, 0.0, 0.0))
    return R, C0 - R @ PIVOT


def line_of(target):
    d = (target - NECK_MID) / np.linalg.norm(target - NECK_MID)
    return d, float((MOUTH[0] - target[0]) / -d[0])


def tool_on_line(target, depth=None, extra=None, live=True):
    """fn(states) (or the planned pose) of the tool putting the tip ``depth`` past the mouth plane on..."""
    d_loc, s_mouth = line_of(target)

    def pose(R, o):
        tipl = extra if extra is not None else target - (s_mouth - depth) * d_loc
        d = R @ d_loc
        return tool_rot(d), o + R @ tipl - LEVER * d

    if live:
        return lambda st: pose(*flask_frame(st["flask"]))
    return pose(*flask_pose(PITCH, ROLL))


@k.on_reset
def _servo_reset(states) -> None:
    RS.reset()
    LS.reset()
    SRV.clear()
    _set_times()


@k.on_step
def _servo_step(states) -> None:
    t = RS.steps * DT
    p0, p1 = TIMES["press"]
    hold_r = bool(RUN.get("near")) or (p0 <= t + DT <= p1)
    RS.update(states, t + DT, hold_x=hold_r, touching=bool(RUN.get("near")))
    LS.update(states, t + DT)


def _set_times() -> None:
    """Plan times of every servoed move (sync's padding reuses the last way's target:"""
    seen = set()
    for arm in ("right", "left"):
        t = 0.0
        for w in _PLAN.get(arm, []):
            if isinstance(w.pos, _WayPos) and w.pos.way not in seen:
                seen.add(w.pos.way)
                w.pos.way.t0, w.pos.way.t1 = t, t + w.duration
            t += w.duration


def flask_way(theta, phi):
    Rf, of = flask_pose(theta, phi)
    pose = (Rf, of + Rf @ G_LOC)
    return (lambda _st: pose), pose


def _fixed(pose):
    return (lambda _st: pose), pose


def plan(how: str = "demo") -> dict:
    """The two-arm plan:"""
    RS.ways.clear()
    LS.ways.clear()
    target = {"hover": HOVER, "middle": MIDDLE}.get(how, TARGET)
    d_loc, s_mouth = line_of(target)
    rest = target if how == "hover" else target + 0.005 * d_loc + np.array([0.0, 0.0, -0.0015])
    if how == "middle":
        rest = target + np.array([0.0, 0.0, -0.0015])
    line = lambda depth: (tool_on_line(target, depth), tool_on_line(target, depth, live=False))  # noqa: E731
    at_rest = (tool_on_line(target, extra=rest), tool_on_line(target, extra=rest, live=False))

    g_flat = Grip.from_matrix(ML_WORLD)
    take = P.grasp_tilted(O_FLAT + R0 @ G_LOC, W, grip=g_flat, squeeze=0.0035, retreat=None, label="the flask body",
                          unproven=True)
    hold_l = take[-1].finger
    take = take[:2] + [Waypoint(0.4, take[1].pos, take[1].finger, "settle before closing", g_flat)] + take[2:]
    left = list(take) + [Waypoint(0.5, take[-1].pos, hold_l, "settle the grip", g_flat)]
    LS.held_from = P.duration(take) + 0.4
    n = 6
    for i in range(1, n + 1):
        left.append(way_wp(LS, math.degrees(PITCH / n) / P.LID_DEG_PER_S, *flask_way(PITCH * i / n, 0.0), hold_l,
                           f"tilt neck-up {math.degrees(PITCH * i / n):.0f}"))
    for i in (1, 2):
        left.append(way_wp(LS, 1.0, *flask_way(PITCH, ROLL * i / 2), hold_l, f"roll {math.degrees(ROLL * i / 2):.0f}"))
    fn_t, nom_t = flask_way(PITCH, ROLL)
    left.append(way_wp(LS, 1.0, fn_t, nom_t, hold_l, "settle the tilt"))
    t_tilted = P.duration(left)

    g_stand = Grip.from_matrix(R_STAND @ XR0)
    take_r = P.grasp_tilted(STAND_G, HP_D, grip=g_stand, squeeze=0.0045, approach=0.030, retreat=None,
                            label="the hand piece", unproven=True)
    hold_r = take_r[-1].finger
    inl = np.asarray(take_r[0].pos)
    vias = [Waypoint(2.0, P._p(inl + np.array([0.06, -0.06, 0.20])), take_r[0].finger, "over the stand", g_stand),
            Waypoint(1.2, P._p(inl + np.array([0.0, 0.0, 0.09])), take_r[0].finger, "above the hand piece", g_stand),
            Waypoint(1.2, P._p(inl), take_r[0].finger, "down behind the hand piece", g_stand)]
    take_r = vias + take_r[1:2] + [Waypoint(0.4, take_r[1].pos, take_r[1].finger, "settle before closing",
                                            g_stand)] + take_r[2:]
    right = list(take_r) + [Waypoint(0.5, take_r[-1].pos, hold_r, "settle the grip", g_stand)]
    TIMES["right_held"] = RS.held_from = P.duration(take_r) + 0.4
    lift = (R_STAND, STAND_G + 0.025 * UP)
    up2 = (R_STAND, STAND_G + np.array([0.0, -0.030, 0.080]))
    right.append(way_wp(RS, 1.2, *_fixed(lift), hold_r, "lift off the stand", integrate=False))
    right.append(way_wp(RS, 1.5, *_fixed(up2), hold_r, "raise clear", integrate=False))
    stage = tool_on_line(target, -0.060, live=False)
    right.append(way_wp(RS, 5.0, *_fixed(stage), hold_r, "carry to the line", integrate=False))
    right.append(way_wp(RS, max(0.5, t_tilted + P.STEADY_SETTLE - P.duration(right)), *_fixed(stage), hold_r,
                        "wait for the tilt"))
    right.append(way_wp(RS, 1.2, *line(-0.040), hold_r, "line up 40 mm out"))
    right.append(way_wp(RS, 1.2, *line(-0.020), hold_r, "line up 20 mm out"))
    right.append(way_wp(RS, 1.0, *line(-0.020), hold_r, "hold on the line"))
    prev = -0.020
    for dpt, v in ((0.0, 0.020), (0.045, 0.020), (0.090, 0.020), (s_mouth - 0.020, 0.010)):
        right.append(way_wp(RS, max(1.0, (dpt - prev) / v), *line(dpt), hold_r, f"in to {1000 * dpt:.0f} mm"))
        prev = dpt
    right.append(way_wp(RS, 2.0, *line(s_mouth), hold_r, "to the back wall"))
    right.append(way_wp(RS, 0.8, *line(s_mouth), hold_r, "settle at the wall"))
    t_press0 = P.duration(right)
    right.append(way_wp(RS, 1.5, *at_rest, hold_r, "rest the tip at the wall, on the floor", integrate=False))
    right.append(way_wp(RS, 7.5, *at_rest, hold_r, "hold while the pool is drawn", integrate=False))
    TIMES["press"] = [t_press0, P.duration(right)]
    right.append(way_wp(RS, (s_mouth - 0.060) / 0.010, *line(0.060), hold_r, "out to 60 mm"))
    right.append(way_wp(RS, 0.100 / 0.010, *line(-0.040), hold_r, "out of the neck"))
    up_fn = tool_on_line(target, -0.040)
    nom_c = tool_on_line(target, -0.040, live=False)
    lift_off = np.array([0.0, -0.040, 0.060])
    clear_way = ((lambda st: (up_fn(st)[0], up_fn(st)[1] + lift_off)), (nom_c[0], nom_c[1] + lift_off))
    right.append(way_wp(RS, 1.5, *clear_way, hold_r, "lift clear"))
    t_out = P.duration(right)

    if how != "still_held":
        right.append(way_wp(RS, 4.0, *_fixed(up2), hold_r, "back over the stand"))
        right.append(way_wp(RS, 2.0, *_fixed(lift), hold_r, "above the saddles"))
        set_down = (R_STAND, STAND_G + 0.001 * UP)
        right.append(way_wp(RS, 2.5, *_fixed(set_down), hold_r, "into the saddles", integrate=False))
        right.append(way_wp(RS, 0.5, *_fixed(set_down), hold_r, "settle in the saddles", integrate=False))
        last_r = RS.ways[-1]
        rel = P.LivePoint(lambda st: tuple(float(v) for v in last_r.resolve()[1]), FEED,
                          tuple(float(v) for v in RS.tcp(*set_down, bias=False)[1]))
        a_r = P.approach_axis(g_stand)
        back_r = P.LivePoint(lambda st: tuple(float(v) for v in np.asarray(rel.resolve()) - 0.030 * a_r), FEED,
                             tuple(float(v) for v in RS.tcp(*set_down, bias=False)[1] - 0.030 * a_r))
        up_r = P.LivePoint(lambda st: tuple(float(v) for v in np.asarray(back_r.resolve()) + 0.090 * UP), FEED,
                           tuple(float(v) for v in RS.tcp(*set_down, bias=False)[1] - 0.030 * a_r + 0.090 * UP))
        opened_r = P.finger_for_gap(HP_D + 0.020)
        right += [Waypoint(P.T_RELEASE, rel, opened_r, "let go of the hand piece", g_stand),
                  Waypoint(1.2, back_r, opened_r, "back off the hand piece", g_stand),
                  Waypoint(1.2, up_r, opened_r, "up clear of the stand", g_stand)]
        hp, hg = P.home("right")
        right += [Waypoint(2.0, (0.320, -0.240, TOP + 0.340), P.FINGER_OPEN, "up and over the tool", hg),
                  Waypoint(P.HOME_S, hp, P.FINGER_OPEN, "home", hg)]

    left.append(way_wp(LS, max(0.5, t_out + 0.5 - P.duration(left)), fn_t, nom_t, hold_l, "hold the tilt"))
    if how != "not_flat":
        for i in (1, 0):
            left.append(way_wp(LS, 1.0, *flask_way(PITCH, ROLL * i / 2), hold_l, f"unroll {math.degrees(ROLL * i / 2):.0f}"))
        for i in range(n - 1, -1, -1):
            left.append(way_wp(LS, math.degrees(PITCH / n) / P.LID_DEG_PER_S, *flask_way(PITCH * i / n, 0.0), hold_l,
                               f"lay down {math.degrees(PITCH * i / n):.0f}"))
        fn0, nom0 = flask_way(0.0, 0.0)
        left.append(way_wp(LS, 0.8, fn0, nom0, hold_l, "set it flat", integrate=False))
        TIMES["left_open"] = P.duration(left)
        last_way = LS.ways[-1]
        opened = P.LivePoint(lambda st: tuple(float(v) for v in last_way.resolve()[1]), FEED,
                             tuple(float(v) for v in LS.tcp(*nom0, bias=False)[1]))
        a = ML_WORLD[:, 2]
        back_pt = P.LivePoint(lambda st: tuple(float(v) for v in np.asarray(opened.resolve()) - 0.060 * a), FEED,
                              tuple(float(v) for v in LS.tcp(*nom0, bias=False)[1] - 0.060 * a))
        left += [Waypoint(P.T_RELEASE, opened, P.FINGER_OPEN, "let go of the flask", g_flat),
                 Waypoint(P.T_LIFT, back_pt, P.FINGER_OPEN, "back out of the grip", g_flat),
                 Waypoint(0.5, back_pt, P.FINGER_OPEN, "clear of the flask", g_flat)]
        hp, hg = P.home("left")
        left += [Waypoint(P.HOME_S, hp, P.FINGER_OPEN, "home", hg)]
    out = P.sync({"left": left, "right": right})
    for ws in out.values():
        e = ws[-1]
        ws.append(Waypoint(P.FINAL_HOLD_S, e.pos, e.finger, "hold still", e.yaw))
    _PLAN.clear()
    _PLAN.update(out)
    _set_times()
    return out


def demo():
    return D.choose(demo=lambda: plan("demo"), hover=lambda: plan("hover"), middle=lambda: plan("middle"),
                    still_held=lambda: plan("still_held"), not_flat=lambda: plan("not_flat"))


# == what the template tests read
TEXT_NUMBERS = [("39.6 mm", FLASK_H), ("1.5 mm walls", WALL), ("21.5 mm bore", NECK_D), ("22 mm long", NECK_L),
                ("60 g empty", FLASK_M), ("y 10.0 cm", BACK_Y), ("x 29.0 cm", CENTRE_X),
                ("10 mm from the back wall", STRIP), ("12 mm wide", TAPE_W), ("4 mm outside", TAPE_OFF),
                ("25 mm across", HP_D), ("130 mm long", HP_L), ("275 mm long", PIP_L), ("6.18 mm across", PIP_D),
                ("65 g in all", ASP_M), ("7.5 s", V0_ML / RATE_ML_S), ("6 mm steel", BASE[2]), ("2.0 kg", STAND_M),
                ("35 deg below", STAND_DIP), ("270 mm in front", LEVER), ("x 20.0 cm", STAND_X),
                ("y -18.0 cm", STAND_Y), ("22.0 cm up", STAND_Z), ("1.02 mm above the floor", TOUCH_MM / 1000),
                ("within 5 deg of level", FLAT_DEG), ("within 20 mm of its taped", OUTLINE),
                ("within 5 mm and 5 deg", RETURN_MM / 1000), ("no gripper within 20 mm", CLEAR_MM / 1000),
                ("more than 1 mm", STILL_MM / 1000), ("last 2 s", STILL_S), ("more than 20 mm within", DROP),
                ("within 0.25 s", DROP_T), ("0.25 mm on a level", FILM_LEVEL), ("0.02 mm once", FILM_DRAINED),
                ("slopes 20 deg", DRAIN_DEG), ("6 mm thick", WALL + THICK), ("4.5 mm higher", THICK)]
CHEATS = {"none": 0.0, "hover": 0.5, "middle": 0.25, "still_held": 0.75, "not_flat": 0.75}
HOLD: dict = {}


# == the cabinet's own rules (sticky zeros): the work zone, the grilles and the sash
def arm_points():
    """Every link hull vertex of both arms in the world (forward kinematics from the joints), or None..."""
    e = k.scene.env()
    if e is None:
        return None
    from yam_arena import analysis, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    out = []
    for arm in ("left", "right"):
        robot = e.unwrapped.scene[f"{arm}_robot"]
        names = list(robot.joint_names)
        j = k.scene._np(robot.data.joint_pos)[0]
        q = {n: float(j[names.index(n)]) for n in ARM_JOINTS} | {f: float(j[names.index(f)]) for f in FINGER_JOINTS}
        for _, h in analysis.world_hulls(analysis.fk_link_poses_world(q, station.BASES[arm]), None):
            out.append(np.asarray(getattr(h, "vertices", getattr(h, "points", None)), dtype=float))
    return np.concatenate(out) if out else None


@k.on_step
def _cabinet_rules(states) -> None:
    if CABINET or "flask" not in states:
        return
    t = RUN.get("k", 0) * DT
    for name, boxes in (("flask", FLASK_BOXES), ("aspirator", ASP_BOXES), ("stand", STAND_BOXES)):
        if name not in states:
            continue
        pts = k.score.surface_points(states[name], "", boxes, 0.02)
        if np.any(pts[:, 0] < BSC_FRONT + WORK_IN):
            CABINET.trip(f"{name} within 4 in of the front grille", t)
            return
        low = pts[:, 2] < TOP + GRILLE_Z
        if np.any(low & (pts[:, 0] > BSC_BACK - REAR_GRILLE)):
            CABINET.trip(f"{name} on the rear grille", t)
            return
    if RUN.get("k", 0) % ARM_EVERY:
        return
    a = arm_points()
    if a is None:
        return
    out = (a[:, 0] < SASH_X) | (a[:, 0] > BSC_BACK) | (np.abs(a[:, 1]) > BSC_W / 2) | (a[:, 2] > TOP + BSC_H)
    if np.any(out):
        CABINET.trip("an arm left the cabinet's interior (sash plane, back or side wall, ceiling)", t)
