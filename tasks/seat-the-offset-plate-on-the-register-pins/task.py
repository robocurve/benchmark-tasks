"""PROBE v1 (stream D): lower a punched A5-scale offset plate the last 25 mm
onto two register pins on a proof-press bed until it sits flat.

The plate is a Z-slider, already on both pins, 25 mm short of the bed.
Joint 0 = seated, start = TRAVEL. on_reset writes the start (mechkit start=
bug). PhysX contactOffset is 4 mm, so a free plate on real pin diameters is
a later stretch (SPEC / RISK).

Env: TASK_DEMO = demo | idle | short | naive
"""
from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import hhkit  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "Put the plate on the pins."

V = hhkit.demo_variant("demo")
PAD = 0.013

# A5-scale work plate on a short proof-press bed in the cue zone.
CX = 0.32
CY = 0.22
PXL = 0.150          # plate length (x)
PW = 0.110           # plate width (y)
PT = 0.002           # plate thickness
TRAVEL = 0.025
BED_Z = 0.040        # bed top above the table
PIN_R = 0.005
PIN_H = 0.022
PIN_DX = 0.045       # pins split along x, centred in y

LOOKS = {
    "bed": m.Look(m.rgb(0.18, 0.18, 0.20), metallic=0.4, roughness=0.45),
    "iron": m.Look(m.rgb(0.25, 0.22, 0.20), metallic=0.5, roughness=0.5),
    "pin": m.Look(m.rgb(0.72, 0.73, 0.70), metallic=0.85, roughness=0.25),
    "alu": m.Look(m.rgb(0.70, 0.71, 0.72), metallic=0.8, roughness=0.3),
    "image": m.Look(m.rgb(0.15, 0.42, 0.55), roughness=0.4),
    "mark": m.Look(m.rgb(0.85, 0.15, 0.12), roughness=0.4),
    "wood": m.Look(m.rgb(0.45, 0.32, 0.18), roughness=0.8),
    "yellow": m.Look(m.rgb(0.75, 0.62, 0.12), roughness=0.4),
}

_cn = [0]


def col(mesh):
    _cn[0] += 1
    return m.col_hull(f"c{_cn[0]}", mesh)


# ---- press bed, origin at (CX, CY, TOP). Pins at (±PIN_DX, 0).
bed = m.box((0.22, 0.16, BED_Z), (0.0, 0.0, BED_Z / 2))
# a short cylinder-segment look: two bearer rings at the ±y ends of the bed
bearer_l = m.cylinder(0.018, 0.16, (0.095, 0.0, BED_Z - 0.002), axis="y", segments=14)
bearer_r = m.cylinder(0.018, 0.16, (-0.095, 0.0, BED_Z - 0.002), axis="y", segments=14)
# visual clamp bar at the +x lead edge
clamp = m.box((0.016, 0.14, 0.010), (0.095, 0.0, BED_Z + 0.005))
# two register pins (visual; the joint is the real stop)
pin_a = m.cylinder(PIN_R, PIN_H, (PIN_DX, 0.0, BED_Z + PIN_H / 2), axis="z", segments=12)
pin_b = m.cylinder(PIN_R, PIN_H, (-PIN_DX, 0.0, BED_Z + PIN_H / 2), axis="z", segments=12)
# pin collars / bases
col_a = m.cylinder(0.009, 0.004, (PIN_DX, 0.0, BED_Z + 0.002), axis="z", segments=10)
col_b = m.cylinder(0.009, 0.004, (-PIN_DX, 0.0, BED_Z + 0.002), axis="z", segments=10)

fx_parts = [
    ("bed", bed, "bed"),
    ("bl", bearer_l, "iron"),
    ("br", bearer_r, "iron"),
    ("clamp", clamp, "iron"),
    ("pina", pin_a, "pin"),
    ("pinb", pin_b, "pin"),
    ("cola", col_a, "pin"),
    ("colb", col_b, "pin"),
]
# No pin colliders on the travel path — the joint is the stop.
fx_cols = [col(bed), col(bearer_l), col(bearer_r), col(clamp)]

# ---- plate as a Z-slider. Pivot at the seated centre of the sheet.
AT = (0.0, 0.0, BED_Z + PT / 2)


def at_mesh(mesh):
    return mesh.moved(AT)


sheet = at_mesh(m.box((PXL, PW, PT), (0.0, 0.0, 0.0)))
# printed image on the plate (a cyan panel + a red register mark) so it
# reads as a printing plate, not a blank sheet.
image = at_mesh(m.box((PXL - 0.030, PW - 0.030, 0.0012), (0.0, 0.0, PT / 2 + 0.0004)))
# four tiny register marks at the mid-edges
mk1 = at_mesh(m.box((0.010, 0.003, 0.0012), (0.0, PW / 2 - 0.006, PT / 2 + 0.0005)))
mk2 = at_mesh(m.box((0.010, 0.003, 0.0012), (0.0, -PW / 2 + 0.006, PT / 2 + 0.0005)))
mk3 = at_mesh(m.box((0.003, 0.010, 0.0012), (PXL / 2 - 0.006, 0.0, PT / 2 + 0.0005)))
mk4 = at_mesh(m.box((0.003, 0.010, 0.0012), (-PXL / 2 + 0.006, 0.0, PT / 2 + 0.0005)))
col_sheet = at_mesh(m.box((PXL, PW, 0.008), (0.0, 0.0, 0.003)))

plate_link = m.slider(
    "plate",
    "Z",
    lower=-0.001,
    upper=TRAVEL + 0.002,
    parts=[
        ("sheet", sheet, "alu"),
        ("image", image, "image"),
        ("mk1", mk1, "mark"),
        ("mk2", mk2, "mark"),
        ("mk3", mk3, "mark"),
        ("mk4", mk4, "mark"),
    ],
    colliders=(col(col_sheet),),
    at=AT,
    mass=0.18,
    friction=0.03,
    damping=0.4,
    spring=0.0,
    gravity=False,
)

PRESS = m.mechanism(
    "press",
    (CX, CY, TOP),
    [plate_link],
    LOOKS,
    frame_parts=fx_parts,
    frame_colliders=fx_cols,
    friction=(1.0, 0.8),
    solver_iters=48,
)

# Distractor plate on a wooden board, +y, not on any pins.
BOX = (CX - 0.02, CY + 0.16, TOP)
BOARD = m.fixed(
    "board",
    BOX,
    [("board", m.box((0.16, 0.13, 0.008), (0.0, 0.0, 0.004)), "wood")],
    [m.col_box("board_c", (0.16, 0.13, 0.008), (0.0, 0.0, 0.004))],
    LOOKS,
)
DIST = m.free(
    "spare",
    (BOX[0], BOX[1], TOP + 0.008 + PT / 2),
    [
        ("sheet", m.box((PXL, PW, PT), (0.0, 0.0, 0.0)), "alu"),
        ("image", m.box((PXL - 0.030, PW - 0.030, 0.0012), (0.0, 0.0, PT / 2 + 0.0004)), "yellow"),
    ],
    [col(m.box((PXL, PW, 0.008), (0.0, 0.0, 0.003)))],
    LOOKS,
    mass=0.18,
    friction=(0.6, 0.5),
    yaw_deg=20.0,
)

OBJECTS = [PRESS, BOARD, DIST]


def _clear(mem):
    mem.update(seated=False, rest=0, z=9, dz=9, vel=9, clear=False)


TRIAL = hhkit.Trial(on_clear=_clear)


def _write_joint(env, name, val):
    import torch
    asset = env.unwrapped.scene["press"]
    j = asset.find_joints(f"{name}_joint")[0][0]
    asset.write_joint_state_to_sim(
        torch.tensor([[val]], device=asset.device),
        torch.zeros((1, 1), device=asset.device),
        joint_ids=[j],
    )


@m.on_reset
def _reset(env):
    _write_joint(env, "plate", TRAVEL)
    print(
        f"[D] reset plate v1 CX={CX} CY={CY} bed_z={BED_Z} travel={TRAVEL} variant={V}",
        flush=True,
    )


@m.on_step
def _log(env):
    if not TRIAL.once():
        return
    st = m.state(env, "press")
    z_joint = float(st.joint("plate"))
    if TRIAL.step <= 2:
        print(f"[D] joints={getattr(st, 'joints', None)}", flush=True)

    dz_home = z_joint
    vel = 0.0
    try:
        vel = float(np.linalg.norm(st.lin_vel))
    except Exception:
        pass

    seated = abs(dz_home) <= 0.003
    face_w = np.array([CX, CY, TOP + BED_Z + PT + 0.001 + z_joint])
    grips = hhkit.grippers(env)
    if grips:
        dmin = min(float(np.linalg.norm(np.asarray(g["tcp"], float) - face_w)) for g in grips)
        clear = dmin >= 0.03
    else:
        dmin, clear = 9.0, False

    rest_ok = vel < 0.03 and seated and clear
    TRIAL.mem["rest"] = TRIAL.mem.get("rest", 0) + 1 if rest_ok else 0
    TRIAL.mem.update(
        seated=seated,
        z=round(z_joint, 4),
        dz=round(dz_home, 4),
        vel=round(vel, 4),
        clear=clear,
        dmin=round(dmin, 3),
        rest=TRIAL.mem["rest"],
    )
    n = TRIAL.step
    if n in (1, 2, 5) or n % 15 == 0 or seated:
        g0 = grips[0]["tcp"] if grips else (0, 0, 0)
        print(
            f"[D] n {n} plate z={z_joint*1000:.1f}mm  dz={dz_home*1000:.1f}mm "
            f"seat={seated} rest={TRIAL.mem['rest']} "
            f"tcpL ({g0[0]:.3f},{g0[1]:.3f},{g0[2]-TOP:.3f})",
            flush=True,
        )


def score(objects):
    mm = TRIAL.mem or {}
    if mm.get("seated") and mm.get("clear") and mm.get("rest", 0) >= 20:
        return 1.0
    if mm.get("seated"):
        return 0.5
    dz = mm.get("dz", 9)
    if isinstance(dz, (int, float)) and 0.005 < dz <= TRAVEL - 0.008:
        return 0.5
    return 0.0


def success(objects):
    return score(objects) >= 1.0


def distance(objects):
    mm = TRIAL.mem or {}
    return float(abs(mm.get("dz", 9.0)))


REST_R = (0.20, -0.30, TOP + 0.18)
REST_L = (0.20, 0.30, TOP + 0.18)
YAW = 0.0
PX = CX
PY = CY
Z_START = TOP + BED_Z + PT + TRAVEL
Z_SEAT = Z_START - TRAVEL
Z_HIGH = TOP + 0.18
ZG_PRESS_START = Z_START + PAD
ZG_PRESS_SEAT = Z_SEAT + PAD


def plan_demo():
    return [
        Waypoint(2.0, (PX, PY, Z_HIGH), FINGER_CLOSED, "high over the plate", yaw=YAW),
        Waypoint(1.2, (PX, PY, ZG_PRESS_START + 0.004), FINGER_CLOSED, "down onto the plate", yaw=YAW),
        Waypoint(0.5, (PX, PY, ZG_PRESS_START - 0.001), FINGER_CLOSED, "press", yaw=YAW),
        Waypoint(2.0, (PX, PY, ZG_PRESS_SEAT - 0.001), FINGER_CLOSED, "push to the bed", yaw=YAW),
        Waypoint(1.0, (PX, PY, ZG_PRESS_SEAT - 0.001), FINGER_CLOSED, "hold seated", yaw=YAW),
        Waypoint(1.0, (PX, PY, Z_HIGH), FINGER_OPEN, "up clear", yaw=YAW),
        Waypoint(1.4, REST_L, FINGER_OPEN, "rest", yaw=YAW),
    ]


def plan_short():
    mid = ZG_PRESS_START - 0.012
    return [
        Waypoint(2.0, (PX, PY, Z_HIGH), FINGER_CLOSED, "high over the plate", yaw=YAW),
        Waypoint(1.2, (PX, PY, ZG_PRESS_START + 0.004), FINGER_CLOSED, "down onto the plate", yaw=YAW),
        Waypoint(0.5, (PX, PY, ZG_PRESS_START - 0.001), FINGER_CLOSED, "press", yaw=YAW),
        Waypoint(1.2, (PX, PY, mid), FINGER_CLOSED, "push only 12 mm", yaw=YAW),
        Waypoint(0.8, (PX, PY, Z_HIGH), FINGER_OPEN, "up", yaw=YAW),
        Waypoint(1.4, REST_L, FINGER_OPEN, "rest", yaw=YAW),
    ]


def plan_naive():
    side = (0.22, 0.30, TOP + 0.18)
    return [
        Waypoint(2.0, side, FINGER_OPEN, "aside, not over the plate", yaw=YAW),
        Waypoint(1.4, REST_L, FINGER_OPEN, "rest", yaw=YAW),
    ]


def demo():
    hold_r = [Waypoint(1.0, REST_R, FINGER_OPEN, "stay", yaw=0.0)]
    if V == "idle":
        return {"left": [Waypoint(8.0, REST_L, FINGER_OPEN, "stay", yaw=0.0)], "right": hold_r}
    if V == "short":
        return {"left": plan_short(), "right": hold_r}
    if V == "naive":
        return {"left": plan_naive(), "right": hold_r}
    return {"left": plan_demo(), "right": hold_r}
