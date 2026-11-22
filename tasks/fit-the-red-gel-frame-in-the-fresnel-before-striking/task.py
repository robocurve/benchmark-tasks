"""PROBE v6 (stream D): slide a 3.75 in red gel frame the last 30 mm along the
colour-frame holders of an Altman 100 3 in Inky fresnel until it sits on the stop.

Sourced from the Altman Lighting CAT #100 3" Fresnel ("Inky"): die-cast aluminium,
black epoxy sandtex, 3 in Pyrex stepped fresnel lens, rigid plated-steel yoke,
suggested 510-B C-clamp, 3.75 x 3.75 in colour frame (100-CFB), focus thumbscrew
on the underside, hinged door with a knurled side knob. See
https://www.altmanlighting.com/wp-content/uploads/2016/04/100Specs.pdf
and the 100 data sheet (7.5 x 5.25 x 4.25 in, 2.2 lb).

The real fixture is 7.5 in (190 mm) along its axis. Sitting it on its back at
true length would put the lens 19 cm above the bench, out of a straight-down
grip, so the barrel is shortened to a 70 mm lens height. That REACH trade-off
is flagged; the lens diameter, frame size, yoke, C-clamp, finish and colours
are true. No false colour: v5's red tilt and focus knobs are gone (real hardware
is black or plated).

v5 sat the lamp on its back and pushed the frame down in Z, which hid the
stepped lens under the 95 mm gel in the top camera. v6 keeps the lamp on its
back (pre-hang practice) but slides the frame in X in the real colour-frame
holders, so a crescent of the stepped lens is visible at the start. The frame
is an X-slider; joint 0 = seated; start= is written in on_reset (mechkit bug).
PhysX contactOffset is 4 mm, so a free body in real 3–5 mm runners is a later
stretch (RISK.md).

Env: TASK_DEMO = demo | idle | short | naive
"""
from __future__ import annotations

import math
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
INSTRUCTION = "Put the red gel in the light."

V = hhkit.demo_variant("demo")
PAD = 0.013

# Altman 100 / 100-CFB
FRAME = 0.0952          # 3.75 in colour frame
LENS_R = 0.0381         # 3 in Pyrex fresnel
TRAVEL = 0.030          # start short of the end stop, along X
FRAME_T = 0.004
GEL_T = 0.002
LENS_Z = 0.070          # REACH trade-off: real axis length is 7.5 in / 190 mm

# World: lamp in the cue zone, lens pointing +Z, red frame face-up, sliding in X.
CX = 0.32
CY = 0.22

LOOKS = {
    "body": m.Look(m.rgb(0.07, 0.07, 0.07), roughness=0.85),          # black epoxy sandtex
    "lens": m.Look(m.rgb(0.72, 0.80, 0.75), roughness=0.12),          # slightly green Pyrex
    "ring": m.Look(m.rgb(0.62, 0.70, 0.66), roughness=0.20),
    "yoke": m.Look(m.rgb(0.62, 0.63, 0.61), metallic=0.7, roughness=0.35),  # plated flat steel
    "clamp": m.Look(m.rgb(0.28, 0.28, 0.29), metallic=0.5, roughness=0.4),
    "knob": m.Look(m.rgb(0.10, 0.10, 0.10), roughness=0.5),            # black thumbscrews, not red
    "run": m.Look(m.rgb(0.10, 0.10, 0.10), roughness=0.7),             # integral cast holders
    "alu": m.Look(m.rgb(0.72, 0.73, 0.74), metallic=0.8, roughness=0.3),
    "red": m.Look(m.rgb(0.82, 0.05, 0.06), roughness=0.35),
    "blue": m.Look(m.rgb(0.08, 0.22, 0.70), roughness=0.35),
    "wood": m.Look(m.rgb(0.45, 0.32, 0.18), roughness=0.8),
    "cable": m.Look(m.rgb(0.06, 0.06, 0.06), roughness=0.7),
}

_cn = [0]


def col(mesh):
    _cn[0] += 1
    return m.col_hull(f"c{_cn[0]}", mesh)


# ---- fixture body (mechanism frame), origin at (CX, CY, TOP) ----
# Local: barrel along +Z, rear cap on the table, lens face at z = LENS_Z.
# Shortened vs the real 190 mm axis (REACH); diameter ~ the real 4.25 in width.
body = m.cylinder(0.052, 0.048, (0.0, 0.0, 0.032), axis="z", segments=22)
rear = m.cylinder(0.056, 0.016, (0.0, 0.0, 0.008), axis="z", segments=18)
front = m.cylinder(0.050, 0.016, (0.0, 0.0, LENS_Z - 0.014), axis="z", segments=22)
# baffled vents on the barrel (visual only)
vents = []
for i, ang in enumerate((-0.9, -0.5, 0.5, 0.9)):
    vx, vy = 0.053 * math.cos(ang), 0.053 * math.sin(ang)
    vents.append((f"vent{i}", m.box((0.004, 0.010, 0.018), (vx, vy, 0.034)), "knob"))

# underside focus thumbscrew (the real one is on the bottom; here it sticks out on −x)
focus = m.cylinder(0.009, 0.024, (-0.064, 0.0, 0.022), axis="x", segments=12)
focus_tip = m.cylinder(0.014, 0.008, (-0.078, 0.0, 0.022), axis="x", segments=10)
# hinged-door knurled knob on +y
door = m.cylinder(0.010, 0.016, (0.0, 0.062, 0.036), axis="y", segments=12)

# 3 in stepped fresnel: a clear disc and concentric ridges on the face
lens = m.cylinder(LENS_R, 0.008, (0.0, 0.0, LENS_Z - 0.004), axis="z", segments=32)
rings = []
for i, r in enumerate((0.034, 0.028, 0.022, 0.016, 0.010, 0.005)):
    rings.append((
        f"ring{i}",
        m.cylinder(r, 0.0016, (0.0, 0.0, LENS_Z + 0.0004 + 0.0005 * i), axis="z", segments=24),
        "ring",
    ))

# Plated flat-steel yoke: a U around the barrel, crossbar on the table at +x
yoke_bar = m.box((0.016, 0.168, 0.010), (0.082, 0.0, 0.007))
arm_l = m.box((0.078, 0.012, 0.010), (0.043, 0.078, 0.007))
arm_r = m.box((0.078, 0.012, 0.010), (0.043, -0.078, 0.007))
boss_l = m.cylinder(0.012, 0.016, (0.0, 0.066, 0.030), axis="y", segments=12)
boss_r = m.cylinder(0.012, 0.016, (0.0, -0.066, 0.030), axis="y", segments=12)

# Altman 510-B style C-clamp on the yoke, opening drawn in XY so it reads from above
clamp_back = m.box((0.016, 0.072, 0.028), (0.112, 0.0, 0.016))
clamp_jaw = m.box((0.042, 0.016, 0.024), (0.133, 0.028, 0.016))
clamp_anvil = m.box((0.030, 0.016, 0.024), (0.127, -0.028, 0.016))
clamp_screw = m.cylinder(0.006, 0.040, (0.136, 0.000, 0.016), axis="y", segments=10)
clamp_tee = m.cylinder(0.006, 0.036, (0.136, -0.040, 0.016), axis="x", segments=10)

# Colour-frame holders (integral, two sides and an end stop): low C-channels
# in the lens plane, long enough to hold the frame at the start and at the stop.
half = FRAME / 2
run_len = FRAME + TRAVEL + 0.016
run_cx = TRAVEL / 2
run_h = 0.010
run_z = LENS_Z + FRAME_T / 2 + 0.001
run_l_back = m.box((run_len, 0.007, run_h), (run_cx,  half + 0.007, run_z))
run_r_back = m.box((run_len, 0.007, run_h), (run_cx, -half - 0.007, run_z))
run_l_lip = m.box((run_len, 0.006, 0.003), (run_cx,  half + 0.002, run_z + 0.005))
run_r_lip = m.box((run_len, 0.006, 0.003), (run_cx, -half - 0.002, run_z + 0.005))
run_end = m.box((0.007, FRAME + 0.020, run_h + 0.004), (-half - 0.007, 0.0, run_z))

# trailing fibreglass leads (visual), off the rear cap toward −x
cable_a = m.cylinder(0.004, 0.040, (-0.070, 0.010, 0.006), axis="x", segments=8)
cable_b = m.cylinder(0.004, 0.050, (-0.110, 0.030, 0.006), axis="x", segments=8)

fx_parts = [
    ("body", body, "body"),
    ("rear", rear, "body"),
    ("front", front, "body"),
    *vents,
    ("focus", focus, "knob"),
    ("focust", focus_tip, "knob"),
    ("door", door, "knob"),
    ("lens", lens, "lens"),
    *rings,
    ("yokeb", yoke_bar, "yoke"),
    ("arml", arm_l, "yoke"),
    ("armr", arm_r, "yoke"),
    ("bossl", boss_l, "yoke"),
    ("bossr", boss_r, "yoke"),
    ("clback", clamp_back, "clamp"),
    ("cljaw", clamp_jaw, "clamp"),
    ("clanv", clamp_anvil, "clamp"),
    ("clscrew", clamp_screw, "clamp"),
    ("cltee", clamp_tee, "clamp"),
    ("rlb", run_l_back, "run"),
    ("rrb", run_r_back, "run"),
    ("rll", run_l_lip, "run"),
    ("rrl", run_r_lip, "run"),
    ("rend", run_end, "run"),
    ("caba", cable_a, "cable"),
    ("cabb", cable_b, "cable"),
]
# Colliders: body / yoke / clamp / knobs only. No runner colliders on the travel path.
fx_cols = [
    col(body),
    col(rear),
    col(front),
    col(yoke_bar),
    col(arm_l),
    col(arm_r),
    col(clamp_back),
    col(clamp_jaw),
    col(clamp_anvil),
    col(focus),
    col(door),
]

# ---- red frame as an X-slider. Joint 0 = SEATED (centred on the lens).
# start= is ignored by this mechkit, so on_reset writes joint = TRAVEL.
# Meshes live at the joint pivot (seated centre of the sheet).
AT = (0.0, 0.0, LENS_Z + FRAME_T / 2)


def at_mesh(mesh):
    return mesh.moved(AT)


sheet = at_mesh(m.box((FRAME, FRAME, FRAME_T), (0.0, 0.0, 0.0)))
gel = at_mesh(m.box((FRAME - 0.014, FRAME - 0.014, GEL_T), (0.0, 0.0, FRAME_T / 2 + GEL_T / 2 - 0.0005)))
# folded tab on +x: the press lip, visible at the start (further +x by TRAVEL)
tab = at_mesh(m.box((0.012, 0.030, 0.006), (half + 0.004, 0.0, 0.002)))
col_sheet = at_mesh(m.box((FRAME, FRAME, 0.010), (0.0, 0.0, 0.003)))

gel_link = m.slider(
    "gel",
    "X",
    lower=-0.001,
    upper=TRAVEL + 0.002,
    parts=[("sheet", sheet, "alu"), ("gel", gel, "red"), ("tab", tab, "alu")],
    colliders=(col(col_sheet),),
    at=AT,
    mass=0.10,
    friction=0.03,
    damping=0.4,
    spring=0.0,
    gravity=False,
)

LIGHT = m.mechanism(
    "light",
    (CX, CY, TOP),
    [gel_link],
    LOOKS,
    frame_parts=fx_parts,
    frame_colliders=fx_cols,
    friction=(1.0, 0.8),
    solver_iters=48,
)

# Blue distractor lying face-up in a shallow gel box, +y of the lamp (clear of the clamp at +x).
BOX_POS = (CX - 0.02, CY + 0.16, TOP)
TRAY = m.fixed(
    "tray",
    BOX_POS,
    [("tray", m.box((0.12, 0.12, 0.008), (0.0, 0.0, 0.004)), "wood")],
    [m.col_box("tray_c", (0.12, 0.12, 0.008), (0.0, 0.0, 0.004))],
    LOOKS,
)
BLUE = m.free(
    "blue",
    (BOX_POS[0], BOX_POS[1], TOP + 0.008 + FRAME_T / 2),
    [
        ("sheet", m.box((FRAME, FRAME, FRAME_T), (0.0, 0.0, 0.0)), "alu"),
        ("gel", m.box((FRAME - 0.014, FRAME - 0.014, GEL_T), (0.0, 0.0, FRAME_T / 2 + GEL_T / 2 - 0.0005)), "blue"),
    ],
    [col(m.box((FRAME, FRAME, 0.010), (0.0, 0.0, 0.003)))],
    LOOKS,
    mass=0.10,
    friction=(0.6, 0.5),
    yaw_deg=0.0,
)

OBJECTS = [LIGHT, TRAY, BLUE]


def _clear(mem):
    mem.update(seated=False, in_runners=False, rest=0, z=9, dz=9, tilt=99, vel=9, clear=False)


TRIAL = hhkit.Trial(on_clear=_clear)


def _write_joint(env, name, val):
    import torch
    asset = env.unwrapped.scene["light"]
    j = asset.find_joints(f"{name}_joint")[0][0]
    asset.write_joint_state_to_sim(
        torch.tensor([[val]], device=asset.device),
        torch.zeros((1, 1), device=asset.device),
        joint_ids=[j],
    )


@m.on_reset
def _reset(env):
    # start= is not applied; write the 30 mm-short pose here (see coupler / drain-plug).
    _write_joint(env, "gel", TRAVEL)
    print(
        f"[D] reset gel v6 CX={CX} CY={CY} lens_z={LENS_Z} travel={TRAVEL} axis=X variant={V}",
        flush=True,
    )


@m.on_step
def _log(env):
    if not TRIAL.once():
        return
    st = m.state(env, "light")
    x_joint = float(st.joint("gel"))
    if TRIAL.step <= 2:
        print(f"[D] joints={getattr(st, 'joints', None)} pos={getattr(st, 'pos', None)}", flush=True)

    dz_home = x_joint  # 0 at the stop; +TRAVEL at start
    tilt = 0.0
    vel = 0.0
    try:
        vel = float(np.linalg.norm(st.lin_vel))
        tilt = hhkit.tilt_deg(st)
    except Exception:
        pass

    in_runners = -0.002 <= x_joint <= TRAVEL + 0.005
    seated = abs(dz_home) <= 0.003 and in_runners

    grips = hhkit.grippers(env)
    # press target: centre of the red face at the current X
    face_w = np.array([CX + x_joint, CY, TOP + LENS_Z + FRAME_T + GEL_T])
    if grips:
        dmin = min(float(np.linalg.norm(np.asarray(g["tcp"], float) - face_w)) for g in grips)
        clear = dmin >= 0.03
    else:
        dmin, clear = 9.0, False

    rest_ok = vel < 0.03 and seated and clear
    TRIAL.mem["rest"] = TRIAL.mem.get("rest", 0) + 1 if rest_ok else 0
    TRIAL.mem.update(
        seated=seated,
        in_runners=in_runners,
        z=round(x_joint, 4),
        dz=round(dz_home, 4),
        tilt=round(tilt, 1),
        vel=round(vel, 4),
        clear=clear,
        dmin=round(dmin, 3),
        rest=TRIAL.mem["rest"],
    )
    n = TRIAL.step
    if n in (1, 2, 5) or n % 15 == 0 or seated:
        g0 = grips[0]["tcp"] if grips else (0, 0, 0)
        print(
            f"[D] n {n} gel x={x_joint*1000:.1f}mm  dz_home={dz_home*1000:.1f}mm "
            f"seat={seated} in={in_runners} rest={TRIAL.mem['rest']} "
            f"tcpL ({g0[0]:.3f},{g0[1]:.3f},{g0[2]-TOP:.3f})",
            flush=True,
        )


def score(objects):
    mm = TRIAL.mem or {}
    if mm.get("seated") and mm.get("clear") and mm.get("rest", 0) >= 20:
        return 1.0
    if mm.get("seated"):
        return 0.5
    # 0.5 only if the frame has actually been pushed at least 10 mm from the
    # start, but is still short of the stop. The start pose itself must score 0.
    dz = mm.get("dz", 9)
    if mm.get("in_runners") and isinstance(dz, (int, float)) and 0.005 < dz <= TRAVEL - 0.010:
        return 0.5
    return 0.0


def success(objects):
    return score(objects) >= 1.0


def distance(objects):
    mm = TRIAL.mem or {}
    return float(abs(mm.get("dz", 9.0)))


# Straight-down on the red face, then a slide along −X to the stop. Left arm.
REST_R = (0.20, -0.30, TOP + 0.18)
REST_L = (0.20, 0.30, TOP + 0.18)
YAW = 0.0
PY = CY
Z_FACE = TOP + LENS_Z + FRAME_T + GEL_T
Z_HIGH = TOP + 0.18
ZG = Z_FACE + PAD
PX_START = CX + TRAVEL
PX_SEAT = CX
# v6 demo stopped 3.7 mm short of the 3 mm seated window (arm settled ~2 mm past the
# commanded seat x). Overdrive a few millimetres toward the stop, and press a touch
# deeper so the fingers don't slip on the gel face.
PX_PUSH = CX - 0.008


def plan_demo():
    return [
        Waypoint(2.0, (PX_START, PY, Z_HIGH), FINGER_CLOSED, "high over the red gel", yaw=YAW),
        Waypoint(1.2, (PX_START, PY, ZG + 0.004), FINGER_CLOSED, "down onto the gel", yaw=YAW),
        Waypoint(0.5, (PX_START, PY, ZG - 0.003), FINGER_CLOSED, "press", yaw=YAW),
        Waypoint(2.0, (PX_PUSH, PY, ZG - 0.003), FINGER_CLOSED, "slide to the stop", yaw=YAW),
        Waypoint(1.2, (PX_PUSH, PY, ZG - 0.003), FINGER_CLOSED, "hold seated", yaw=YAW),
        Waypoint(1.0, (PX_SEAT, PY, Z_HIGH), FINGER_OPEN, "up clear", yaw=YAW),
        Waypoint(1.4, REST_L, FINGER_OPEN, "rest", yaw=YAW),
    ]


def plan_short():
    mid = PX_START - 0.018
    return [
        Waypoint(2.0, (PX_START, PY, Z_HIGH), FINGER_CLOSED, "high over the red gel", yaw=YAW),
        Waypoint(1.2, (PX_START, PY, ZG + 0.004), FINGER_CLOSED, "down onto the gel", yaw=YAW),
        Waypoint(0.5, (PX_START, PY, ZG - 0.003), FINGER_CLOSED, "press", yaw=YAW),
        Waypoint(1.2, (mid, PY, ZG - 0.003), FINGER_CLOSED, "slide only about 12 mm", yaw=YAW),
        Waypoint(0.8, (mid, PY, Z_HIGH), FINGER_OPEN, "up", yaw=YAW),
        Waypoint(1.4, REST_L, FINGER_OPEN, "rest", yaw=YAW),
    ]


def plan_naive():
    side = (0.22, 0.30, TOP + 0.18)
    return [
        Waypoint(2.0, side, FINGER_OPEN, "aside, not over the lamp", yaw=YAW),
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
