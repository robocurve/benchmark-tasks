"""Retail / cafe counter on the station table (110 x 130 cm matte oak-laminate worktop centered at (36.0, 0.0) cm,
x forward -19..+91 cm, y left -65..+65 cm, z = 0 at table top = 0.7475 m; origin at the midpoint between the left and
right arm bases at (0, +30.5, 0.25) cm and (0, -30.5, 0.25) cm).

At (19.0, 0.0) cm stands a fixed dark-grey ABS U-slot counter rack (11.0 cm along x, 6.8 cm along y, 6.5 cm tall, with a
4.4 cm-wide vertical slot along x and 0.8 cm slot floor). Standing upright inside the slot is a flat-folded Southern
Champion Tray (SCT) 2795 kraft paperboard 2-cup handled drink carrier (42 g; folded 9.5 cm along x, 4.2 cm thick along y,
22.5 cm tall; pops open to 9.5 cm wide along x by 18.8..19.4 cm long along y by 22.5 cm tall, with two 9.5 x 9.1 cm
pockets, 8.4 cm pocket walls with 5.6 cm upper collar flaps, wrap-around corner cheeks, 2-flap interlocking auto-bottom
floors, and a central 2-ply 6.6 cm top handle arch).

On the right-side drink pass stand two sealed 245 g Solo 378W 8 oz paper hot cups with white Solo TL38R2 traveler dome
sip lids (8.0 cm rim OD, 5.1 cm base OD, 10.8 cm total height) at (26.0, -24.0) cm and (37.0, -24.0) cm. On the left-side
customer counter lies a fixed blue non-slip silicone pickup mat (16.0 cm along x by 24.0 cm along y by 0.4 cm thick)
centered at (32.0, +22.0) cm. A continuous 1.0 cm white tape line runs across the counter at y = +8.0 cm (x 20..90 cm,
2 cm short of the mat's y = +10 cm edge), between the work area and the mat: the carrier must hold both seated cups before
its spine centre first crosses that line toward the mat.

Run it:
    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
        -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1500 \
        --instruction "Open the flat 2-cup drink carrier and seat both lidded hot cups upright in its pockets before it crosses the white tape line, then leave the loaded carrier standing on the blue pickup mat."
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import grip_limited as gl  # noqa: E402
import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # 0.7475 m
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = (
    "Open the flat 2-cup drink carrier and seat both lidded hot cups upright in its pockets before it crosses "
    "the white tape line, then leave the loaded carrier standing on the blue pickup mat."
)

# -- Colours ----------------------------------------------------------------------------------------------------------
KRAFT = (0.66, 0.47, 0.28)
KRAFT_DARK = (0.52, 0.36, 0.20)
CUP_CREAM = (0.93, 0.90, 0.83)
SLEEVE_BROWN = (0.48, 0.32, 0.18)
LID_WHITE = (0.96, 0.96, 0.97)
RACK_GREY = (0.28, 0.29, 0.31)
MAT_BLUE = (0.18, 0.44, 0.76)

# -- Table bounds (station table: 110 cm deep x 130 cm wide centered at (36, 0) cm) -----------------------------------
TABLE_DEPTH_X, TABLE_WIDTH_Y = 1.10, 1.30
TABLE_X0 = -0.19
TABLE_X1 = TABLE_X0 + TABLE_DEPTH_X   # 0.91
TABLE_HALF_Y = TABLE_WIDTH_Y / 2.0    # 0.65
DROP_LOW_Z = 0.030      # lowest point < -3.0 cm below tabletop

# -- Fixed counter U-slot rack at (19.0, 0.0) cm (slot along x, holding folded carrier upright) -----------------------
RACK_X, RACK_Y = 0.19, 0.00
RACK_LEN_X = 0.110      # 11.0 cm along x (x 13.5..24.5 cm)
SLOT_W_Y = 0.044        # 4.4 cm inner slot width along y
RAIL_T = 0.012          # 1.2 cm left/right rail thickness (total rack y = 6.8 cm)
RACK_FLOOR_H = 0.008    # 0.8 cm slot floor height
RACK_WALL_H = 0.065     # 6.5 cm total rack height
RACK_Y_SIZE = SLOT_W_Y + 2 * RAIL_T  # 0.068 m

RACK_PARTS = [
    Box("rack_floor", pos=(RACK_X, RACK_Y, TOP + RACK_FLOOR_H / 2),
        size=(RACK_LEN_X, RACK_Y_SIZE, RACK_FLOOR_H), color=RACK_GREY),
    Box("rack_rail_left", pos=(RACK_X, RACK_Y + (SLOT_W_Y + RAIL_T) / 2, TOP + RACK_WALL_H / 2),
        size=(RACK_LEN_X, RAIL_T, RACK_WALL_H), color=RACK_GREY),
    Box("rack_rail_right", pos=(RACK_X, RACK_Y - (SLOT_W_Y + RAIL_T) / 2, TOP + RACK_WALL_H / 2),
        size=(RACK_LEN_X, RAIL_T, RACK_WALL_H), color=RACK_GREY),
]

# -- Fixed blue silicone customer pickup mat at (32.0, +22.0) cm ------------------------------------------------------
MAT_X, MAT_Y = 0.32, 0.22
MAT_SX, MAT_SY, MAT_SZ = 0.160, 0.240, 0.004  # 16.0 x 24.0 x 0.4 cm (x 24..40 cm, y 10..34 cm)
PICKUP_MAT = Box("pickup_mat", pos=(MAT_X, MAT_Y, TOP + MAT_SZ / 2), size=(MAT_SX, MAT_SY, MAT_SZ), color=MAT_BLUE)

# -- Loading-zone line: continuous white tape line at y = +8.0 cm across the counter (x 20..90 cm, just off the mat) ---
# The carrier must be loaded with both cups on the counter side of this line (carrier spine centre y < +8.0 cm) before
# its spine centre first crosses it toward the mat; crossing earlier (empty or 1-cup carrier) latches the score at 0.70.
LOAD_LINE_Y = 0.080
TAPE_X0, TAPE_X1, TAPE_W, TAPE_H = 0.20, 0.90, 0.010, 0.0003
LOAD_LINE_TAPE = Box("load_line_tape", pos=((TAPE_X0 + TAPE_X1) / 2, LOAD_LINE_Y, TOP + TAPE_H / 2),
                     size=(TAPE_X1 - TAPE_X0, TAPE_W, TAPE_H), color=(0.93, 0.93, 0.90))

# -- SCT 2795 2-Cup Handled Kraft Carrier (9.5 x 19.4 x 22.5 cm open, 42 g total; all 5 bodies spawn at identity rot) -
CAR_W = 0.095           # 3-3/4 in = 9.5 cm pocket width along x
CAR_L = 0.194           # 7-5/8 in = 19.4 cm nominal open length along y
WALL_H = 0.084          # 8.4 cm pocket collar wall height (3.3 in)
SPINE_H = 0.225         # 8-7/8 in = 22.5 cm total handle height
PANEL_T = 0.004         # 4 mm kraft end-wall thickness
SPINE_T = 0.004         # 4 mm 2-ply central divider thickness
HANDLE_BAR_X = 0.066    # 6.6 cm tapered top handle arch width along x
HANDLE_BAR_R = 0.0045   # 9 mm diameter (4.5 mm radius) rolled top handle rim along x
END_COLLAR_X = 0.056    # 5.6 cm upper pocket collar tab width along x (1.9 cm jaw clearance per side)
END_COLLAR_H = 0.048    # 4.8 cm collar tab height above the 3.6 cm full-width lower pocket wall

FLOOR_X = 0.090
FLOOR_T = 0.003
FLOOR_LEN = 0.080       # 8.0 cm inner auto-bottom floor flap (hinged at y=+-0.6 cm, z=+0.55 cm; spans to +-8.6 cm flat)
FLOOR_HINGE_Y = 0.0060
FLOOR_HINGE_Z = 0.0055
END_Y_FOLD = 0.0190     # +-1.90 cm folded end-wall center -> 4.2 cm outer folded width inside 4.4 cm rack slot
OPEN_TRAVEL = 0.0830    # allows pulling end wall to +-10.2 cm so 8.0 cm floor drops past 9.5 mm ledge & locks at +-9.15 cm

# Exact 42 g (0.042 kg) published carrier mass across the 5 colliding panels
M_SPINE = 0.016
M_END = 0.006
M_FLOOR = 0.007
# Total: 16 + 6 + 6 + 7 + 7 = 42 g

CAR_Z0 = TOP + RACK_FLOOR_H + 0.001

CAR_SPINE = Box("carrier_spine", pos=(RACK_X, RACK_Y, CAR_Z0 + SPINE_H / 2),
                size=(CAR_W, SPINE_T, SPINE_H), mass=M_SPINE, color=KRAFT)
CAR_FLOORL = Box("carrier_floorl", pos=(RACK_X, RACK_Y + FLOOR_HINGE_Y, CAR_Z0 + FLOOR_HINGE_Z + FLOOR_LEN / 2),
                 size=(FLOOR_X, FLOOR_T, FLOOR_LEN), mass=M_FLOOR, color=KRAFT_DARK)
CAR_FLOORR = Box("carrier_floorr", pos=(RACK_X, RACK_Y - FLOOR_HINGE_Y, CAR_Z0 + FLOOR_HINGE_Z + FLOOR_LEN / 2),
                 size=(FLOOR_X, FLOOR_T, FLOOR_LEN), mass=M_FLOOR, color=KRAFT_DARK)
CAR_ENDL = Box("carrier_endl", pos=(RACK_X, RACK_Y + END_Y_FOLD, CAR_Z0 + WALL_H / 2),
               size=(CAR_W, PANEL_T, WALL_H), mass=M_END, color=KRAFT_DARK)
CAR_ENDR = Box("carrier_endr", pos=(RACK_X, RACK_Y - END_Y_FOLD, CAR_Z0 + WALL_H / 2),
               size=(CAR_W, PANEL_T, WALL_H), mass=M_END, color=KRAFT_DARK)

CARRIER_PARTS = (CAR_SPINE, CAR_FLOORL, CAR_FLOORR, CAR_ENDL, CAR_ENDR)
CARRIER_NAMES = tuple(p.name for p in CARRIER_PARTS)

# -- Two Solo 378W 8 oz paper hot cups with Solo TL38R2 traveler dome lids (245 g each, sealed dry weighted) ----------
CUP_BASE_R = 0.0255     # 5.1 cm base OD (2.02 in)
CUP_RIM_R = 0.0400      # 8.0 cm rim OD (3.15 in)
CUP_BODY_H = 0.094      # 9.4 cm cup wall height (3.7 in)
CUP_TOTAL_H = 0.108     # 10.8 cm total height including TL38R2 traveler dome lid
CUP_MASS = 0.245        # 245 g (8 fl oz beverage equivalent; CoM 4.2 cm above base)
CUP_COM_Z = -CUP_TOTAL_H / 2 + 0.042

CUP1_START = (0.26, -0.24)
CUP2_START = (0.37, -0.24)

CUP1 = Cylinder("cup_1", pos=(CUP1_START[0], CUP1_START[1], TOP + CUP_TOTAL_H / 2 + 0.0005),
                radius=CUP_BASE_R, height=CUP_TOTAL_H, mass=CUP_MASS, color=CUP_CREAM)
CUP2 = Cylinder("cup_2", pos=(CUP2_START[0], CUP2_START[1], TOP + CUP_TOTAL_H / 2 + 0.0005),
                radius=CUP_BASE_R, height=CUP_TOTAL_H, mass=CUP_MASS, color=CUP_CREAM)
CUPS = (CUP1, CUP2)
CUP_NAMES = ("cup_1", "cup_2")

OBJECTS = [*RACK_PARTS, PICKUP_MAT, LOAD_LINE_TAPE, *CARRIER_PARTS, CUP1, CUP2]

VARIATION = {
    "obj:cup_1": {"dx": 0.010, "dy": 0.010, "yaw": 0.20},
    "obj:cup_2": {"dx": 0.010, "dy": 0.010, "yaw": 0.20},
}

TWIN_EXTRA = [
    {"name": "carrier_creases", "kind": "joint", "real": "SCT 2795 accordion side gussets & 2-flap auto-bottom creases",
     "note": "two Y-axis pocket expansion joints (0..8.3 cm, 1.20 N crease return spring) + two X-hinged 9.0x8.0 cm "
             "auto-bottom floors that drop flat against +-90 deg stops and mechanically lock against end-wall bottom tabs"},
    {"name": "cups_taper_and_lid", "kind": "body", "real": "Solo 378W 8 oz cup + TL38R2 dome lid, 235 g sealed steel shot",
     "note": "5-slice tapered collider (5.1 cm base to 8.0 cm rim) + white dome sip lid, 245 g total, CoM 4.2 cm up"},
]

# -- No-damage contact limits (latched score 0 when exceeded) ---------------------------------------------------------
# Paper hot cup + lid: a conventional paper cup sidewall yields 6.35 mm (0.25 in) under only ~500 g = 4.9 N (US 5,772,111
# rigidity test); 20.0 N on one contact (~4x that, far past the elastic flex range) creases the wall / unseats the lid.
# Kraft carrier board: ~6.5 kN/m short-span compression (SCT, CD) for ~310 g/m2 carton board (Nord. Pulp Pap. Res. J.
# 2023, doi 10.1515/npprj-2023-0021); a ~5 mm edge contact therefore buckles it at ~30 N -> 30.0 N limit per contact.
# Measured as the PhysX normal contact force between any single robot link (pads, fingers, wrist, forearm...) and a cup or
# carrier body, every control step.
CUP_SIDEWALL_REF_N = 4.9          # ~500 g sidewall rigidity of a conventional paper cup (US 5,772,111) ...
CUP_SIDEWALL_REF_DEFL = 0.00635   # ... at 6.35 mm (0.25 in) deflection
CUP_CRUSH_N = 20.0
CARRIER_CRUSH_N = 30.0

# -- Calibrated force-limited pad grip constants (matching README.md and task.json 1:1) --------------------------------
# The YAM gripper can squeeze up to 50.0 N per finger (yam_arena GRIPPER effort, i2rt linear_4310), but the no-damage
# limits above bound the usable pad squeeze to 20.0 N on cups/collars/arch and 30.0 N on the rolled handle rim, so the
# friction caps are derived from those squeezes. Rubber pad mu = 0.5 on matte kraft / cup sleeve (~20 x 30 mm pad patch)
# and mu_rim = 0.10 on the smooth 9 mm (4.5 mm radius) rolled handle:
#   pull cap  = 2 pads x mu 0.5 x 20.0 N = 20.0 N            (rim: 2 x 0.5 x 30.0 N = 30.0 N)
#   twist cap = 20.0 N x 10 mm mean pad-patch radius = 0.20 N m (about pad normal)
#   tip cap   = 2 x 20.0 N x 15 mm half pad length = 0.60 N m (in pad plane on cups, collars & 6.6 cm arch)
#   rim roll  = 2 x mu_rim 0.10 x 30.0 N x 4.5 mm bar radius = 0.027 N m (pinching the 9 mm central handle rim along Y)
GRIP_SQUEEZE_N = CUP_CRUSH_N
RIM_SQUEEZE_N = CARRIER_CRUSH_N
GRIP_MU = 0.5
RIM_MU = 0.10
GRIP_TWIST_R = 0.010
GRIP_TIP_HALF = 0.015
GRIP_HOLD_N = 2.0 * GRIP_MU * GRIP_SQUEEZE_N                 # 20.0 N
GRIP_TWIST_NM = GRIP_HOLD_N * GRIP_TWIST_R                   # 0.20 N m
GRIP_TIP_NM = 2.0 * GRIP_SQUEEZE_N * GRIP_TIP_HALF           # 0.60 N m
RIM_HOLD_N = 2.0 * GRIP_MU * RIM_SQUEEZE_N                   # 30.0 N
HANDLE_ROLL_NM = 2.0 * RIM_MU * RIM_SQUEEZE_N * HANDLE_BAR_R  # 0.027 N m
PAD_MIN_CUP_N = 3.0     # >= 3.0 N normal squeeze on EACH pad (2 x 0.5 x 3.0 N = 3.0 N > 2.40 N cup & 1.20 N gusset)
PAD_MIN_RIM_N = 5.5     # >= 5.5 N normal squeeze on EACH pad (2 x 0.5 x 5.5 N = 5.5 N > 5.22 N loaded 532 g carrier)
GRIP_LOSE_STEPS = 5     # release if two-pad squeeze drops below threshold for 5 steps (0.25 s) or jaws open

# -- Runtime state & USD handles --------------------------------------------------------------------------------------
_P: dict = {}
_ENV: dict = {"env": None}
_H: dict = {"fingers_bound": False}


def _stage():
    try:
        from isaaclab.sim.utils.stage import get_current_stage

        return get_current_stage()
    except Exception:  # noqa: BLE001
        return None


def _Rq(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _env():
    env = _ENV["env"]
    if env is None:
        try:
            from yam_arena import inspect_embodiment as ie

            env = getattr(ie.YamArenaEmbodiment, "_carrier_env", None)
        except Exception:  # noqa: BLE001
            env = None
    return env


def _link_pose_w(env, name: str) -> np.ndarray:
    """Exact USD link-frame world pose (root_link_pose_w, NOT CoM-shifted pose) for zero-snap joint attachment."""
    ro = env.unwrapped.scene.rigid_objects[name]
    t = getattr(ro.data.root_link_pose_w, "torch", ro.data.root_link_pose_w)[0].detach().cpu().numpy().astype(np.float64)
    q_aw = env.unwrapped.arena_world.get_pose_w(name)[0, 3:].detach().cpu().numpy().astype(np.float64)
    T = np.eye(4)
    T[:3, :3] = _Rq(q_aw)
    T[:3, 3] = t[:3]
    return T


def _grip_pose_fn(body_path: str) -> tuple[np.ndarray, np.ndarray]:
    env = _env()
    T = _link_pose_w(env, body_path.rsplit("/", 1)[-1])
    return T[:3, 3], T[:3, :3]


def _jaw_axis(env, side: str) -> np.ndarray:
    from yam_arena.runtime import link_poses

    rob = env.unwrapped.scene[f"{side}_robot"]
    return link_poses(rob)["gripper"][:3, 0]


def _body_grip_paths() -> list[str]:
    r = _P.get("root")
    return [f"{r}/{n}" for n in (*CUP_NAMES, CAR_ENDL.name, CAR_ENDR.name, CAR_SPINE.name)] if r else []


def _rim_grip_paths() -> list[str]:
    r = _P.get("root")
    return [f"{r}/{CAR_SPINE.name}"] if r else []


def _engage_body_ok(side: str, mid_w: np.ndarray, body_path: str) -> bool:
    env = _env()
    if env is None or GRIP_RIM.holding(side):
        return False
    bname = body_path.rsplit("/", 1)[-1]
    if bname in CUP_NAMES:
        Tc = _link_pose_w(env, bname)
        loc = Tc[:3, :3].T @ (np.asarray(mid_w, float) - Tc[:3, 3])
        return bool(-0.030 <= float(loc[2]) <= CUP_TOTAL_H / 2 + 0.010
                    and math.hypot(float(loc[0]), float(loc[1])) <= CUP_RIM_R + 0.010)
    if bname in (CAR_ENDL.name, CAR_ENDR.name):
        Te = _link_pose_w(env, bname)
        col_c = Te[:3, :3] @ np.array([0.0, 0.0, WALL_H / 2 - END_COLLAR_H / 2]) + Te[:3, 3]
        loc = Te[:3, :3].T @ (np.asarray(mid_w, float) - col_c)
        return bool(abs(float(loc[0])) <= END_COLLAR_X / 2 + 0.025
                    and abs(float(loc[1])) <= 0.028 and abs(float(loc[2])) <= END_COLLAR_H / 2 + 0.012)
    if bname == CAR_SPINE.name:
        Ts = _link_pose_w(env, CAR_SPINE.name)
        hc = Ts[:3, :3] @ np.array([0.0, 0.0, SPINE_H / 2 - 0.016]) + Ts[:3, 3]
        loc_s = Ts[:3, :3].T @ (np.asarray(mid_w, float) - hc)
        if not (abs(float(loc_s[0])) <= CAR_W / 2 + 0.012
                and abs(float(loc_s[1])) <= 0.028 and abs(float(loc_s[2])) <= 0.038):
            return False
        jaw_ax = _jaw_axis(env, side)
        # X-straddle across the 6.6 cm outer handle pillars (Y-pinch on the 9 mm central bar uses GRIP_RIM)
        return abs(float(np.dot(jaw_ax, Ts[:3, 0]))) >= 0.55
    return False


def _engage_rim_ok(side: str, mid_w: np.ndarray, body_path: str) -> bool:
    env = _env()
    if env is None or GRIP_BODY.holding(side):
        return False
    Ts = _link_pose_w(env, CAR_SPINE.name)
    hc = Ts[:3, :3] @ np.array([0.0, 0.0, SPINE_H / 2 - 0.010]) + Ts[:3, 3]
    loc_s = Ts[:3, :3].T @ (np.asarray(mid_w, float) - hc)
    if not (abs(float(loc_s[0])) <= HANDLE_BAR_X / 2 + 0.014
            and abs(float(loc_s[1])) <= 0.028 and abs(float(loc_s[2])) <= 0.034):
        return False
    jaw_ax = _jaw_axis(env, side)
    return abs(float(np.dot(jaw_ax, Ts[:3, 0]))) < 0.55


GRIP_BODY = gl.LimitedGrip(
    _body_grip_paths,
    label="cup or carrier collar",
    name="grip_body",
    joint_root=lambda: _P.get("jr"),
    squeeze_n=GRIP_SQUEEZE_N,
    mu=GRIP_MU,
    twist_radius=GRIP_TWIST_R,
    tip_half_len=GRIP_TIP_HALF,
    contact="pads",
    contact_min_n=PAD_MIN_CUP_N,
    engage_ok=_engage_body_ok,
    pose_fn=_grip_pose_fn,
    lose_steps=GRIP_LOSE_STEPS,
    log_prefix="[grip_body]",
)

GRIP_RIM = gl.LimitedGrip(
    _rim_grip_paths,
    label="carrier handle rim",
    name="grip_rim",
    joint_root=lambda: _P.get("jr"),
    squeeze_n=RIM_SQUEEZE_N,
    mu=GRIP_MU,
    twist_radius=GRIP_TWIST_R,
    tip_nm=HANDLE_ROLL_NM,
    contact="pads",
    contact_min_n=PAD_MIN_RIM_N,
    engage_ok=_engage_rim_ok,
    pose_fn=_grip_pose_fn,
    lose_steps=GRIP_LOSE_STEPS,
    log_prefix="[grip_rim]",
)


_CV: dict = {}


def _robot_contact_max() -> dict | None:
    """{object name: max PhysX normal contact force [N] from any single robot link this step}, or None if unavailable."""
    root = _P.get("root")
    st = _stage()
    if root is None or st is None:
        return None
    sv = GRIP_BODY._sim_view()
    if sv is None:
        return None
    names = (*CARRIER_NAMES, *CUP_NAMES)
    if _CV.get("sv") is not sv:
        from pxr import Usd, UsdPhysics

        links = []
        for arm in ("LeftArm", "RightArm"):
            ap = st.GetPrimAtPath(f"{root}/{arm}")
            if not ap.IsValid():
                continue
            for pr in Usd.PrimRange(ap, Usd.TraverseInstanceProxies()):
                if pr.HasAPI(UsdPhysics.RigidBodyAPI):
                    links.append(str(pr.GetPath()))
        if not links:
            return None
        gl.enable_contacts()
        bodies = [f"{root}/{n}" for n in names]
        _CV.clear()
        _CV.update(sv=sv, n=len(bodies), views=[
            sv.create_rigid_contact_view([lp], filter_patterns=[bodies], max_contact_data_count=256) for lp in links])
        print(f"[carrier] no-damage contact monitor on {len(links)} robot links x {len(bodies)} bodies", flush=True)
    from yam_arena.constants import PHYSICS_DT

    out = dict.fromkeys(names, 0.0)
    for v in _CV["views"]:
        m = gl._np(v.get_contact_force_matrix(dt=PHYSICS_DT)).reshape(_CV["n"], 3)
        for i, n in enumerate(names):
            out[n] = max(out[n], float(np.linalg.norm(m[i])))
    return out


def holding_any() -> bool:
    return GRIP_BODY.holders() > 0 or GRIP_RIM.holders() > 0


def is_seated_grip(side: str) -> bool:
    return bool(
        (GRIP_BODY.holding(side) and GRIP_BODY.seat.get(side, 0) == 0)
        or (GRIP_RIM.holding(side) and GRIP_RIM.seat.get(side, 0) == 0)
    )


def held_summary() -> str:
    items = []
    for s in ("left", "right"):
        b = GRIP_BODY.held_body(s) or GRIP_RIM.held_body(s)
        if b:
            items.append(f"{s[0]}:{b.rsplit('/', 1)[-1]}")
    return ",".join(items) or "-"


def _author(root: str) -> None:
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    _P["root"] = root
    jr = f"{root}/carrier_joints"
    UsdGeom.Xform.Define(stage, jr)
    _P["jr"] = jr

    # Elastomer physics material for robot finger pads (multiply mode so smooth 9 mm handle_bar rolls under overload)
    grip_mat = f"{jr}/Looks/grip_pad_mat"
    g_cfg = sim_utils.RigidBodyMaterialCfg(
        static_friction=1.00, dynamic_friction=0.90, friction_combine_mode="multiply", restitution=0.0
    )
    g_cfg.func(grip_mat, g_cfg)
    _P["grip_mat"] = grip_mat

    looks: dict = {}

    def look(rgb, rough=0.65):
        key = (tuple(rgb), rough)
        if key not in looks:
            lp = f"{jr}/Looks/mat_{len(looks)}"
            cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(rgb), roughness=rough)
            cfg.func(lp, cfg)
            looks[key] = lp
        return looks[key]

    pmats: dict = {}

    def pmat(mu, rest=0.0):
        if mu not in pmats:
            mp = f"{jr}/Looks/pmat_{len(pmats)}"
            cfg = sim_utils.RigidBodyMaterialCfg(
                static_friction=mu, dynamic_friction=0.9 * mu, friction_combine_mode="min", restitution=rest
            )
            cfg.func(mp, cfg)
            pmats[mu] = mp
        return pmats[mu]

    def hide_default_geom(path: str, disable_col: bool = True) -> None:
        for p in Usd.PrimRange(stage.GetPrimAtPath(path)):
            if disable_col and p.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
            if p.IsA(UsdGeom.Gprim):
                UsdGeom.Imageable(p).MakeInvisible()

    def add_box(parent: str, name: str, pos, size, rgb, mu=0.4, collide=True):
        path = f"{parent}/{name}"
        g = UsdGeom.Cube.Define(stage, path)
        g.CreateSizeAttr(1.0)
        xf = UsdGeom.Xformable(g)
        xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
        xf.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        bind_visual_material(path, look(rgb), stage=stage)
        if collide:
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
            cp.CreateContactOffsetAttr(0.0012)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(path, pmat(mu), stage=stage)
        return path

    def add_cyl(parent: str, name: str, pos, radius: float, height: float, rgb, mu=0.6, collide=True, rough=0.5,
                axis: str = "Z"):
        path = f"{parent}/{name}"
        g = UsdGeom.Cylinder.Define(stage, path)
        g.CreateRadiusAttr(float(radius))
        g.CreateHeightAttr(float(height))
        g.CreateAxisAttr(axis)
        UsdGeom.Xformable(g).AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
        bind_visual_material(path, look(rgb, rough=rough), stage=stage)
        if collide:
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
            cp.CreateContactOffsetAttr(0.0012)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(path, pmat(mu), stage=stage)
        return path

    def nocollide(a: str, b: str) -> None:
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(a)).GetFilteredPairsRel().AddTarget(Sdf.Path(b))

    # 1. Detail carrier_spine: lower divider, 6.6 cm outer pillars & central 9 mm handle rod
    sp_path = f"{root}/{CAR_SPINE.name}"
    hide_default_geom(sp_path, disable_col=True)
    sp_ma = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(sp_path))
    sp_ma.CreateMassAttr(M_SPINE)
    sp_ma.CreateDiagonalInertiaAttr(Gf.Vec3f(9.0e-5, 9.0e-5, 4.0e-5))
    h_low = 0.165
    add_box(sp_path, "div_lower", (0.0, 0.0, -SPINE_H / 2 + h_low / 2), (CAR_W, SPINE_T, h_low), KRAFT, mu=0.22)
    h_cut = SPINE_H - h_low  # 0.060 m
    z_cut = -SPINE_H / 2 + h_low + h_cut / 2
    pillar_w = 0.010
    for sgn, tag in ((-1.0, "l"), (1.0, "r")):
        add_box(sp_path, f"pillar_{tag}", (sgn * (HANDLE_BAR_X / 2 - pillar_w / 2), 0.0, z_cut),
                (pillar_w, 0.006, h_cut), KRAFT, mu=0.90)
        add_box(sp_path, f"wing_{tag}", (sgn * (CAR_W / 2 - 0.0015), 0.0, -SPINE_H / 2 + 0.024),
                (0.003, 0.010, 0.048), KRAFT, mu=0.25)
    add_cyl(sp_path, "handle_bar", (0.0, 0.0, SPINE_H / 2 - 0.006),
            HANDLE_BAR_R, HANDLE_BAR_X - 2 * pillar_w, KRAFT_DARK, mu=0.01, axis="X")

    # 2. Side cup-centering curbs at outer X edges of floor_l / floor_r (8.5 cm clear X span so 5.1 cm cup sits flat)
    for fspec, curb_y in ((CAR_FLOORL, -0.0035), (CAR_FLOORR, +0.0035)):
        fp_root = f"{root}/{fspec.name}"
        fp_ma = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(fp_root))
        fp_ma.CreateMassAttr(M_FLOOR)
        fp_ma.CreateDiagonalInertiaAttr(Gf.Vec3f(3.0e-5, 3.0e-5, 3.0e-5))
        fp_mesh = f"{fp_root}/geometry/mesh"
        if stage.GetPrimAtPath(fp_mesh).IsValid():
            bind_physics_material(fp_mesh, pmat(0.85), stage=stage)
        for sgn_x, ctag in ((-1.0, "front"), (+1.0, "back")):
            add_box(fp_root, f"curb_{ctag}", (sgn_x * 0.044, curb_y, 0.0), (0.003, 0.005, 0.060), KRAFT_DARK, mu=0.45)

    # 3. Die-cut end walls carrier_endl / carrier_endr: 9.5 cm lower pocket + 5.6 x 4.8 cm upper collar + 9.5 mm ledge
    for espec, sgn_in in ((CAR_ENDL, -1.0), (CAR_ENDR, +1.0)):
        ep_root = f"{root}/{espec.name}"
        hide_default_geom(ep_root, disable_col=True)
        ep_ma = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(ep_root))
        ep_ma.CreateMassAttr(M_END)
        ep_ma.CreateDiagonalInertiaAttr(Gf.Vec3f(4.0e-5, 4.0e-5, 4.0e-5))
        low_h = WALL_H - END_COLLAR_H
        add_box(ep_root, "lower_wall", (0.0, 0.0, -WALL_H / 2 + low_h / 2), (CAR_W, PANEL_T, low_h), KRAFT_DARK, mu=0.24)
        add_box(ep_root, "upper_collar", (0.0, 0.0, WALL_H / 2 - END_COLLAR_H / 2),
                (END_COLLAR_X, 0.005, END_COLLAR_H), KRAFT_DARK, mu=0.85)
        add_box(ep_root, "lock_ledge", (0.0, sgn_in * 0.00475, -WALL_H / 2 + FLOOR_HINGE_Z),
                (0.080, 0.0095, 0.004), KRAFT_DARK, mu=0.55)
        for sgn_x, ctag in ((-1.0, "front"), (+1.0, "back")):
            add_box(ep_root, f"cheek_{ctag}", (sgn_x * (CAR_W / 2 - 0.0015), sgn_in * 0.005, -WALL_H / 2 + 0.017),
                    (0.003, 0.010, 0.034), KRAFT, mu=0.25)

    pm_mesh = f"{root}/{PICKUP_MAT.name}/geometry/mesh"
    if stage.GetPrimAtPath(pm_mesh).IsValid():
        bind_physics_material(pm_mesh, pmat(0.85), stage=stage)

    # 4. Build tapered Solo 378W 8 oz cups + white TL38R2 traveler dome sip lids (true profile, 245 g, CoM at 4.2 cm)
    for cspec in CUPS:
        cp_path = f"{root}/{cspec.name}"
        hide_default_geom(cp_path, disable_col=True)
        ma = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(cp_path))
        ma.CreateMassAttr(CUP_MASS)
        ma.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, float(CUP_COM_Z)))
        ma.CreateDiagonalInertiaAttr(Gf.Vec3f(2.4e-4, 2.4e-4, 1.4e-4))
        z_bot = -CUP_TOTAL_H / 2
        n_slices = 5
        sh = CUP_BODY_H / n_slices
        for k in range(n_slices):
            frac = (k + 0.5) / n_slices
            r_k = CUP_BASE_R + (CUP_RIM_R - CUP_BASE_R) * frac
            col = SLEEVE_BROWN if k in (2, 3) else CUP_CREAM
            add_cyl(cp_path, f"body_{k}", (0.0, 0.0, z_bot + (k + 0.5) * sh), r_k, sh, col, mu=0.75)
        lid_h = CUP_TOTAL_H - CUP_BODY_H
        add_cyl(cp_path, "lid_skirt", (0.0, 0.0, z_bot + CUP_BODY_H + 0.003), CUP_RIM_R + 0.0012, 0.006,
                LID_WHITE, mu=0.55, rough=0.35)
        add_cyl(cp_path, "lid_dome", (0.0, 0.0, z_bot + CUP_BODY_H + 0.006 + (lid_h - 0.006) / 2),
                CUP_RIM_R - 0.0025, lid_h - 0.006, LID_WHITE, mu=0.55, rough=0.35)
        add_box(cp_path, "sip_spout", (CUP_RIM_R - 0.009, 0.0, z_bot + CUP_TOTAL_H - 0.0015),
                (0.010, 0.016, 0.004), LID_WHITE, collide=False)

    # Keep physical collision active between (CAR_FLOORL, CAR_ENDL) and (CAR_FLOORR, CAR_ENDR) so that when
    # CAR_ENDL/R is pulled past 8.95 cm, CAR_FLOORL/R drops flat to +-90 deg and props lock_ledge open mechanically!
    nocollide_pairs = [
        (CAR_SPINE, CAR_FLOORL), (CAR_SPINE, CAR_FLOORR), (CAR_SPINE, CAR_ENDL), (CAR_SPINE, CAR_ENDR),
        (CAR_FLOORL, CAR_FLOORR), (CAR_FLOORL, CAR_ENDR), (CAR_FLOORR, CAR_ENDL), (CAR_ENDL, CAR_ENDR),
    ]
    for a_sp, b_sp in nocollide_pairs:
        pa = stage.GetPrimAtPath(f"{root}/{a_sp.name}")
        pb = stage.GetPrimAtPath(f"{root}/{b_sp.name}")
        a_cols = [pa.GetPath()] + [p.GetPath() for p in Usd.PrimRange(pa) if p.HasAPI(UsdPhysics.CollisionAPI)]
        b_cols = [pb.GetPath()] + [p.GetPath() for p in Usd.PrimRange(pb) if p.HasAPI(UsdPhysics.CollisionAPI)]
        for ac in dict.fromkeys(a_cols):
            for bc in dict.fromkeys(b_cols):
                nocollide(str(ac), str(bc))

    print(f"[carrier] build-time finger/floor filter on {_filter_fingers_vs_floors(stage)} finger links", flush=True)
    for p in (*CARRIER_PARTS, *CUPS):
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(f"{root}/{p.name}"))
        rb.CreateSolverPositionIterationCountAttr(24)
        rb.CreateSolverVelocityIterationCountAttr(4)
        rb.CreateMaxDepenetrationVelocityAttr(0.8)
        rb.CreateLinearDampingAttr(0.35)
        rb.CreateAngularDampingAttr(0.60)

    # 5. Author pocket expansion joints (Y-axis PrismaticJoint) & auto-bottom floor hinges (X-axis RevoluteJoint)
    dz_end = -(SPINE_H - WALL_H) / 2

    def open_joint(name: str, b_end: Box, y0_spine: float, lo: float, hi: float):
        pj = UsdPhysics.PrismaticJoint.Define(stage, f"{jr}/{name}")
        pj.CreateBody0Rel().SetTargets([Sdf.Path(f"{root}/{CAR_SPINE.name}")])
        pj.CreateBody1Rel().SetTargets([Sdf.Path(f"{root}/{b_end.name}")])
        pj.CreateAxisAttr("Y")
        pj.CreateLocalPos0Attr(Gf.Vec3f(0.0, float(y0_spine), float(dz_end)))
        pj.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        pj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        pj.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        pj.CreateLowerLimitAttr(float(lo))
        pj.CreateUpperLimitAttr(float(hi))
        pj.CreateExcludeFromArticulationAttr(True)
        dr = UsdPhysics.DriveAPI.Apply(pj.GetPrim(), "linear")
        dr.CreateTypeAttr("force")
        dr.CreateTargetPositionAttr(0.0)
        dr.CreateStiffnessAttr(30.0)
        dr.CreateDampingAttr(2.5)
        dr.CreateMaxForceAttr(1.20)  # 1.20 N paperboard gusset return >> 0.15 N floor wedge & 0.10 N counter friction
        return str(pj.GetPath())

    _P["j_open_l"] = open_joint("j_open_l", CAR_ENDL, +END_Y_FOLD, 0.0, OPEN_TRAVEL)
    _P["j_open_r"] = open_joint("j_open_r", CAR_ENDR, -END_Y_FOLD, -OPEN_TRAVEL, 0.0)

    hz_sp = -SPINE_H / 2 + FLOOR_HINGE_Z
    for fname, fspec, fy_sp, lo_deg, hi_deg, tgt_deg in (
        ("j_floor_l", CAR_FLOORL, +FLOOR_HINGE_Y, -90.0, 0.0, -90.0),
        ("j_floor_r", CAR_FLOORR, -FLOOR_HINGE_Y, 0.0, +90.0, +90.0),
    ):
        fj = UsdPhysics.RevoluteJoint.Define(stage, f"{jr}/{fname}")
        fj.CreateBody0Rel().SetTargets([Sdf.Path(f"{root}/{CAR_SPINE.name}")])
        fj.CreateBody1Rel().SetTargets([Sdf.Path(f"{root}/{fspec.name}")])
        fj.CreateAxisAttr("X")
        fj.CreateLocalPos0Attr(Gf.Vec3f(0.0, float(fy_sp), float(hz_sp)))
        fj.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        fj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, float(-FLOOR_LEN / 2)))
        fj.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        fj.CreateLowerLimitAttr(float(lo_deg))
        fj.CreateUpperLimitAttr(float(hi_deg))
        fj.CreateExcludeFromArticulationAttr(True)
        fdr = UsdPhysics.DriveAPI.Apply(fj.GetPrim(), "angular")
        fdr.CreateTypeAttr("force")
        fdr.CreateTargetPositionAttr(float(tgt_deg))
        fdr.CreateStiffnessAttr(0.0025)
        fdr.CreateDampingAttr(0.0005)
        fdr.CreateMaxForceAttr(0.018)
        _P[fname] = str(fj.GetPath())


def _wrap_cfg(spec, obj):
    from isaaclab.sim.utils import clone

    cfg = obj.spawner_cfg
    if spec.name == OBJECTS[-1].name:
        orig = cfg.func
        base = getattr(orig, "__wrapped__", orig)

        def spawn(prim_path, c, *a, **k):
            prim = base(prim_path, c, *a, **k)
            _author(str(prim_path).rsplit("/", 1)[0])
            return prim

        cfg.func = clone(spawn)
    obj.object_cfg = obj._init_object_cfg()
    return obj


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_carrier_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        obj = orig(spec)
        if getattr(spec, "name", None) == OBJECTS[-1].name:
            _wrap_cfg(spec, obj)
        return obj

    task_object._carrier_patched = True
    scene.task_object = task_object


class _SceneFinder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        orig_exec = spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_module
        return spec


if "yam_arena.scene" in sys.modules:
    _patch_scene(sys.modules["yam_arena.scene"])
elif not any(isinstance(f, _SceneFinder) for f in sys.meta_path):
    sys.meta_path.insert(0, _SceneFinder())


def _filter_fingers_vs_floors(st) -> int:
    """Thin folded paperboard floor flaps deflect under a finger pad; the rigid sim flaps would jam the jaws instead,
    so the two finger links do not collide with the two floor-flap bodies (they still collide with every other part)."""
    from pxr import Sdf, Usd, UsdPhysics

    root = _P.get("root")
    if st is None or root is None:
        return 0
    tips = []
    for arm in ("LeftArm", "RightArm"):
        ap = st.GetPrimAtPath(f"{root}/{arm}")
        if not ap.IsValid():
            continue
        for p in Usd.PrimRange(ap, Usd.TraverseInstanceProxies()):
            ps = str(p.GetPath())
            if ps.endswith(("/tip_left", "/tip_right")) and p.HasAPI(UsdPhysics.RigidBodyAPI):
                tips.append(ps)
    for fl in (CAR_FLOORL.name, CAR_FLOORR.name):
        fp = st.GetPrimAtPath(f"{root}/{fl}")
        if fp.IsValid() and tips:
            rel = UsdPhysics.FilteredPairsAPI.Apply(fp).GetFilteredPairsRel()
            for t in tips:
                rel.AddTarget(Sdf.Path(t))
    return len(tips)


def _bind_finger_friction_once() -> None:
    if _H.get("fingers_bound") or _P.get("root") is None or _P.get("grip_mat") is None:
        return
    st = _stage()
    if st is None or not st.GetPrimAtPath(_P["grip_mat"]).IsValid():
        return
    from isaaclab.sim.utils import bind_physics_material
    from pxr import Usd, UsdPhysics

    if not _H.get("floors_filtered"):
        _H["floors_filtered"] = True
        print(f"[carrier] runtime finger/floor filter on {_filter_fingers_vs_floors(st)} finger links", flush=True)
    cnt = 0
    for arm in ("LeftArm", "RightArm"):
        ap = st.GetPrimAtPath(f"{_P['root']}/{arm}")
        if not ap.IsValid():
            continue
        for p in Usd.PrimRange(ap, Usd.TraverseInstanceProxies()):
            ps = str(p.GetPath())
            if ("tip_left" in ps or "tip_right" in ps) and p.HasAPI(UsdPhysics.CollisionAPI):
                bind_physics_material(ps, _P["grip_mat"], stage=st)
                cnt += 1
    if cnt > 0:
        _H["fingers_bound"] = True


# -- Geometry & Kinematics Helpers ------------------------------------------------------------------------------------
def tilt_deg(quat_xyzw) -> float:
    R = _Rq(quat_xyzw)
    return float(math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2]))))))


def floor_flat_deg(quat_xyzw) -> float:
    R = _Rq(quat_xyzw)
    nz = min(1.0, max(0.0, abs(float(R[2, 1]))))
    return float(math.degrees(math.acos(nz)))


def pocket_spans_m(objects) -> tuple[float, float]:
    sp = objects[CAR_SPINE.name]
    Rs = _Rq(sp.quat_xyzw)
    ys = Rs[:3, 1]
    dl = float(np.dot(np.asarray(objects[CAR_ENDL.name].pos, float) - np.asarray(sp.pos, float), ys))
    dr = float(-np.dot(np.asarray(objects[CAR_ENDR.name].pos, float) - np.asarray(sp.pos, float), ys))
    return max(0.0, dl), max(0.0, dr)


def carrier_clear_of_rack(objects, margin: float = 0.020) -> bool:
    for n in CARRIER_NAMES:
        o = objects[n]
        low, high = np.asarray(o.low, float), np.asarray(o.high, float)
        dx = max(0.0, (RACK_X - RACK_LEN_X / 2) - high[0], low[0] - (RACK_X + RACK_LEN_X / 2))
        dy = max(0.0, (RACK_Y - RACK_Y_SIZE / 2) - high[1], low[1] - (RACK_Y + RACK_Y_SIZE / 2))
        dz = max(0.0, low[2] - (TOP + RACK_WALL_H))
        if math.sqrt(dx * dx + dy * dy + dz * dz) < margin:
            return False
    return True


def pocket_centers_world(objects) -> tuple[np.ndarray, np.ndarray]:
    sp = objects[CAR_SPINE.name]
    Rs = _Rq(sp.quat_xyzw)
    ys = Rs[:3, 1]
    span_l, span_r = pocket_spans_m(objects)
    c_mid = np.array([sp.pos[0], sp.pos[1], sp.low[2]], dtype=float)
    return c_mid + (0.5 * span_l + 0.002) * ys, c_mid - (0.5 * span_r + 0.002) * ys


# -- Episode tracking & scoring ---------------------------------------------------------------------------------------
class _Episode:
    def __init__(self):
        self.step = 0
        self.time = 0.0
        self.violated = False
        self.why = ""
        self.loaded = False          # both cups seated while carrier spine centre y < LOAD_LINE_Y (latched)
        self.crossed_unloaded = False  # spine centre crossed LOAD_LINE_Y before `loaded` (latched; caps score at 0.70)
        self.peak_cup_n = 0.0          # max single robot-link contact force on a cup / lid over the trial
        self.peak_carrier_n = 0.0      # max single robot-link contact force on a carrier body over the trial
        self.tcp_hist: list[tuple[float, list[np.ndarray]]] = []
        self.mock_grippers: list[np.ndarray] | None = None


_EP = _Episode()


def reset_episode() -> None:
    global _EP
    GRIP_BODY.reset()
    GRIP_RIM.reset()
    _H["fingers_bound"] = False
    _EP = _Episode()


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_carrier_step_hooked", False):
        base_step = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            type(self)._carrier_env = self._env
            try:
                _bind_finger_friction_once()
                GRIP_BODY.update(self._env)
                GRIP_RIM.update(self._env)
                if __import__("os").environ.get("CARRIER_DBG") == "1" and _EP.step % 10 == 0:
                    for _sd in ("left", "right"):
                        _pf = {k.rsplit("/", 1)[-1]: (round(v[0], 2), round(v[1], 2))
                               for k, v in GRIP_BODY.pad_forces(_sd).items() if max(v) > 0.01}
                        if _pf:
                            print(f"[dbg step={_EP.step} {_sd}] pf={_pf}", flush=True)
                from yam_arena.scene import object_states

                _observe_step(object_states(self._env, self.task.names))
            except Exception as e:  # noqa: BLE001
                print(f"[carrier] step hook error: {e!r}", flush=True)
            return base_step(self, *a, **k)

        _task_step._carrier_step_hooked = True
        cls._task_step = _task_step

    if not getattr(cls.reset, "_carrier_reset_hooked", False):
        base_reset = cls.reset

        def _reset(self, *a, **k):
            reset_episode()
            obs = base_reset(self, *a, **k)
            _ENV["env"] = self._env
            type(self)._carrier_env = self._env
            return obs

        _reset._carrier_reset_hooked = True
        cls.reset = _reset


_install_hooks()


# -- Scoring Constants (identical to README.md and task.json) ---------------------------------------------------------
RUN_LIMIT_STEPS = 1500
CUP_SPILL_DEG = 25.0           # Score 0 if at any time either lidded cup tilts > 25.0 deg from vertical (spill)
RACK_CLEAR_MARGIN = 0.020      # >= 2.0 cm clear of counter rack for popped-open carrier
OPEN_MIN_SPAN = 0.082          # >= 8.2 cm pocket opening span on BOTH pockets for popped-open carrier
FLOOR_FLAT_MAX_DEG = 12.0      # both auto-bottom floors <= 12.0 deg from horizontal
UPRIGHT_END_DEG = 8.0          # <= 8.0 deg tilt at end for open carrier and seated cups
CARRIER_BASE_MAX_Z = 0.010     # carrier bottom <= 1.0 cm above mat/table
CUP_BASE_ABOVE_FLOOR = 0.015   # cup base <= 1.5 cm above carrier bottom
POCKET_XY_TOL = 0.022          # cup center within 2.2 cm of pocket center in XY
MAT_FOOTPRINT_TOL = 0.002      # 2.0 mm boundary tolerance on 16.0 x 24.0 cm pickup mat
RELEASE_MIN_DIST = 0.020       # clear release: neither gripper holding, TCPs >= 2.0 cm from carrier and cups
PARK_CLEAR_DIST = 0.100        # 1.00 requires both gripper TCPs >= 10.0 cm clear of carrier and cups
PARK_MAX_SPEED = 0.020         # < 2.0 cm/s continuously over final 1.0 s
PARK_HOLD_S = 1.0


def grippers() -> list[np.ndarray] | None:
    if _EP.mock_grippers is not None:
        return _EP.mock_grippers
    env = _env()
    if env is None:
        return None
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


def _box_dist(pt: np.ndarray, low: np.ndarray, high: np.ndarray) -> float:
    d = np.maximum(np.maximum(low - pt, pt - high), 0.0)
    return float(np.linalg.norm(d))


def _observe_step(objects) -> None:
    _EP.step += 1
    _EP.time = _EP.step / 20.0
    gs = grippers()
    if gs is not None and len(gs) == 2 and all(np.isfinite(g).all() for g in gs):
        _EP.tcp_hist.append((_EP.time, [g.copy() for g in gs]))
        cutoff = _EP.time - (PARK_HOLD_S + 0.5)
        while len(_EP.tcp_hist) > 2 and _EP.tcp_hist[0][0] < cutoff:
            _EP.tcp_hist.pop(0)

    if _EP.violated:
        return

    # 1. Off-table drop (center drops below z=0 or outside 110 x 130 cm station tabletop, or lowest point < -3.0 cm)
    for n in (*CARRIER_NAMES, *CUP_NAMES):
        o = objects[n]
        p = np.asarray(o.pos, float)
        if not np.isfinite(p).all():
            _EP.violated, _EP.why = True, f"{n}_nonfinite"
            return
        if (float(p[2]) < TOP or float(o.low[2]) < TOP - DROP_LOW_Z
                or not (TABLE_X0 <= p[0] <= TABLE_X1 and abs(p[1]) <= TABLE_HALF_Y)):
            _EP.violated, _EP.why = True, f"{n}_off_table"
            return

    # 2. No damage: any single robot link pressing a cup/lid > 20.0 N or a carrier body > 30.0 N (crushed / torn)
    cf = _robot_contact_max() if _EP.mock_grippers is None else None
    if cf is not None:
        _EP.peak_cup_n = max(_EP.peak_cup_n, *(cf[n] for n in CUP_NAMES))
        _EP.peak_carrier_n = max(_EP.peak_carrier_n, *(cf[n] for n in CARRIER_NAMES))
        if __import__("os").environ.get("CARRIER_DBG") == "1" and max(cf.values()) > 12.0:
            print(f"[dmg step={_EP.step}] " + " ".join(f"{k}={v:.1f}" for k, v in cf.items() if v > 3.0), flush=True)
        for n in CUP_NAMES:
            if cf[n] > CUP_CRUSH_N:
                _EP.violated, _EP.why = True, f"{n}_crushed_{cf[n]:.0f}N"
                return
        for n in CARRIER_NAMES:
            if cf[n] > CARRIER_CRUSH_N:
                _EP.violated, _EP.why = True, f"{n}_crushed_{cf[n]:.0f}N"
                return

    # 3. Lidded hot cup spill (> 25.0 deg from vertical; empty carrier tilt during handling is not penalized)
    for cname in CUP_NAMES:
        if tilt_deg(objects[cname].quat_xyzw) > CUP_SPILL_DEG:
            _EP.violated, _EP.why = True, f"{cname}_spilled"
            return

    # 4. Loading-zone order: both cups seated on the counter side of the y = +8.0 cm tape line before the carrier
    #    spine centre first crosses it toward the mat (no carrying the empty carrier to the mat and loading it there).
    sp_y = float(objects[CAR_SPINE.name].pos[1])
    if not _EP.loaded and sp_y < LOAD_LINE_Y:
        seated = {_cup_seated_in_pocket(objects, c) for c in CUP_NAMES}
        if None not in seated and len(seated) == 2:
            _EP.loaded = True
    if not _EP.loaded and sp_y >= LOAD_LINE_Y:
        _EP.crossed_unloaded = True


def _carrier_open_upright(objects) -> bool:
    if not carrier_clear_of_rack(objects, margin=RACK_CLEAR_MARGIN):
        return False
    span_l, span_r = pocket_spans_m(objects)
    if min(span_l, span_r) < OPEN_MIN_SPAN:
        return False
    for fn in (CAR_FLOORL.name, CAR_FLOORR.name):
        if floor_flat_deg(objects[fn].quat_xyzw) > FLOOR_FLAT_MAX_DEG:
            return False
    sp = objects[CAR_SPINE.name]
    if tilt_deg(sp.quat_xyzw) > UPRIGHT_END_DEG:
        return False
    return float(sp.low[2]) <= TOP + MAT_SZ + CARRIER_BASE_MAX_Z


def _cup_seated_in_pocket(objects, cname: str) -> str | None:
    c = objects[cname]
    if tilt_deg(c.quat_xyzw) > UPRIGHT_END_DEG:
        return None
    sp = objects[CAR_SPINE.name]
    if float(c.low[2] - sp.low[2]) > CUP_BASE_ABOVE_FLOOR or float(c.low[2] - sp.low[2]) < -0.006:
        return None
    pl, pr = pocket_centers_world(objects)
    if math.hypot(c.pos[0] - pl[0], c.pos[1] - pl[1]) <= POCKET_XY_TOL:
        return "L"
    if math.hypot(c.pos[0] - pr[0], c.pos[1] - pr[1]) <= POCKET_XY_TOL:
        return "R"
    return None


def _carrier_on_mat(objects) -> bool:
    sp = objects[CAR_SPINE.name]
    if float(sp.low[2]) > TOP + MAT_SZ + CARRIER_BASE_MAX_Z:
        return False
    x0, x1 = MAT_X - MAT_SX / 2, MAT_X + MAT_SX / 2
    y0, y1 = MAT_Y - MAT_SY / 2, MAT_Y + MAT_SY / 2
    for n in (CAR_SPINE.name, CAR_ENDL.name, CAR_ENDR.name):
        o = objects[n]
        if float(o.low[0]) < x0 - MAT_FOOTPRINT_TOL or float(o.high[0]) > x1 + MAT_FOOTPRINT_TOL:
            return False
        if float(o.low[1]) < y0 - MAT_FOOTPRINT_TOL or float(o.high[1]) > y1 + MAT_FOOTPRINT_TOL:
            return False
    return True


def _parked_and_still(objects, gs: list[np.ndarray]) -> bool:
    for g in gs:
        for n in (*CARRIER_NAMES, *CUP_NAMES):
            o = objects[n]
            if _box_dist(g, np.asarray(o.low, float), np.asarray(o.high, float)) < PARK_CLEAR_DIST:
                return False
    if not _EP.tcp_hist:
        return False
    t_now = _EP.tcp_hist[-1][0]
    window = [(t, pts) for t, pts in _EP.tcp_hist if t >= t_now - PARK_HOLD_S - 1e-6]
    if len(window) < 2 or (t_now - window[0][0]) < PARK_HOLD_S - 0.08:
        return False
    for k in range(1, len(window)):
        dt = max(window[k][0] - window[k - 1][0], 1e-4)
        for arm_i in (0, 1):
            spd = float(np.linalg.norm(window[k][1][arm_i] - window[k - 1][1][arm_i])) / dt
            if spd >= PARK_MAX_SPEED:
                return False
    return True


def score(objects) -> float:
    if _EP.violated:
        return 0.0
    for n in (*CARRIER_NAMES, *CUP_NAMES):
        o = objects[n]
        p = np.asarray(o.pos, float)
        if not np.isfinite(p).all():
            return 0.0
        if (float(p[2]) < TOP or float(o.low[2]) < TOP - DROP_LOW_Z
                or not (TABLE_X0 <= p[0] <= TABLE_X1 and abs(p[1]) <= TABLE_HALF_Y)):
            return 0.0
    for cname in CUP_NAMES:
        if tilt_deg(objects[cname].quat_xyzw) > CUP_SPILL_DEG:
            return 0.0

    gs = grippers()
    if gs is None or len(gs) != 2 or not all(np.isfinite(g).all() for g in gs):
        return 0.0
    if holding_any():
        return 0.0
    for g in gs:
        for n in (*CARRIER_NAMES, *CUP_NAMES):
            o = objects[n]
            if _box_dist(g, np.asarray(o.low, float), np.asarray(o.high, float)) < RELEASE_MIN_DIST:
                return 0.0

    if not _carrier_open_upright(objects):
        return 0.0

    p1 = _cup_seated_in_pocket(objects, CUP1.name)
    p2 = _cup_seated_in_pocket(objects, CUP2.name)
    pockets = {p for p in (p1, p2) if p is not None}

    if len(pockets) == 0:
        return 0.25
    if len(pockets) == 1:
        return 0.50
    if _EP.crossed_unloaded or not _carrier_on_mat(objects):
        return 0.70
    if not _parked_and_still(objects, gs):
        return 0.85
    return 1.00


def success(objects) -> bool:
    return score(objects) >= 1.0 and all(objects[n].resting(0.015, 0.2) for n in (CAR_SPINE.name, *CUP_NAMES))


def distance(objects) -> float:
    return 1.0 - score(objects)


def progress(objects) -> dict:
    p1 = _cup_seated_in_pocket(objects, CUP1.name)
    p2 = _cup_seated_in_pocket(objects, CUP2.name)
    pockets = {p for p in (p1, p2) if p is not None}
    span_l, span_r = pocket_spans_m(objects)
    gs = grippers()
    return {
        "violated": int(_EP.violated),
        "why": _EP.why,
        "span_l_cm": round(100.0 * span_l, 2),
        "span_r_cm": round(100.0 * span_r, 2),
        "floor_l_deg": round(floor_flat_deg(objects[CAR_FLOORL.name].quat_xyzw), 1),
        "floor_r_deg": round(floor_flat_deg(objects[CAR_FLOORR.name].quat_xyzw), 1),
        "carrier_open_upright": int(_carrier_open_upright(objects)),
        "cups_seated": len(pockets),
        "loaded_before_line": int(_EP.loaded),
        "crossed_line_unloaded": int(_EP.crossed_unloaded),
        "on_pickup_mat": int(_carrier_on_mat(objects)),
        "parked_still": int(bool(gs and len(gs) == 2 and _parked_and_still(objects, gs))),
        "peak_cup_n": round(_EP.peak_cup_n, 1),
        "peak_carrier_n": round(_EP.peak_carrier_n, 1),
        "slips_body": len(GRIP_BODY.slips),
        "slips_rim": len(GRIP_RIM.slips),
    }
