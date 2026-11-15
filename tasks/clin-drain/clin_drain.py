"""Recharge a 400 mL 3-spring wound-drain reservoir: press it flat on the non-slip mat, seat its tethered spout plug
home, and release without losing compression.

Bench layout in station coordinates (TOP = station.TABLE_TOP, +X forward, +Y left; arm bases at y = +-0.305 m):
- mat (fixed): grey dimpled absorbent bench pad (260 x 220 x 3 mm) centred at (0.285, +0.040, TOP + 0.0015),
  top surface at MAT_TOP = TOP + 0.003 m.
- drain (loose, 145 g): 400 mL 3-spring closed-wound suction evacuator (Ø112 mm, 76 mm expanded -> 20 mm compressed)
  resting on the mat at (0.280, +0.050, MAT_TOP). White polypropylene top/bottom discs, clear collapsible film
  side wall, three internal stainless-steel springs, a +Y inlet port with a short tubing stub, a -X finger-press lug,
  a flared pouring spout (15.6 mm ID, 14 mm tall) on the top disc at local (+0.062, -0.002), and a plug-holder
  cradle reaching local (+0.077, -0.040).
- plug (loose, 10 g): tethered pouring-spout stopper (Ø10.4 mm tapered stem x 12.5 mm, Ø27 mm flange, 20 x 6 x 15 mm
  pull-tab) standing open in the cradle at (0.357, +0.010), 40.8 mm from the spout. The tether is not simulated.
- visual-only bench props (assets/props, labkit.SCENERY): no collider, no mass, all at x >= 0.42 m.

Run it:
  inspect-robots run --embodiment yam_arena -E task=clin_drain.py -E object_jitter=0 -E control_hz=20 \\
    -E cam_width=320 -E cam_height=176 --instruction "<see task.json>" --scorer task_score ...
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import numpy as np

_HERE = str(pathlib.Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import arena_assets_fix  # noqa: E402, F401
import labkit  # noqa: E402
from labkit import GLASS_COLORS, Joint, PartState, on_reset, on_step, part  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402

# -- geometry & scoring constants (single source of truth for score(), success(), demo()) ------------------

SCENE = "station"
TOP = station.TABLE_TOP

MAT_SIZE = (0.260, 0.220, 0.003)
MAT_POS = (0.285, 0.040, TOP + MAT_SIZE[2] / 2.0)
MAT_TOP = TOP + MAT_SIZE[2]

DRAIN_XY = (0.280, 0.050)
DRAIN_RADIUS = 0.056  # 112 mm OD 400 mL round spring reservoir
H_BOTTOM_DISC = 0.0045
H_TOP_DISC = 0.0045
H_EXPANDED = 0.0760  # 76 mm expanded disc-to-disc height
STROKE_EACH = 0.0280  # 28 mm per slide joint -> 56 mm total stroke -> 20 mm compressed
H_COMPRESSED = H_EXPANDED - 2.0 * STROKE_EACH  # 20.0 mm (26.3% of expanded)

PRESS_LOCAL = (-0.044, -0.005)  # world (0.236, +0.045), inside 56 mm top disc on -X side
SPOUT_LOCAL = (0.062, -0.002)   # world (0.342, +0.048), moulded +X pouring-spout boss (106 mm from press)
SPOUT_THROAT_R = 0.0078  # 15.6 mm ID throat
SPOUT_WALL_T = 0.0030
SPOUT_THROAT_H = 0.0070
SPOUT_FLARE_H = 0.0070
SPOUT_H = SPOUT_THROAT_H + SPOUT_FLARE_H  # 14.0 mm above top disc

HOLDER_LOCAL = (0.077, -0.040)  # world (0.357, +0.010), 40.8 mm from spout (< 45 mm tether)
HOLDER_FLOOR_Z = 0.0040

PLUG_TIP_R = 0.0030
PLUG_TIP_H = 0.0055
PLUG_STEM_R = 0.0052
PLUG_STEM_H = 0.0125  # 5.5 mm tapered tip + 7 mm sealing cylinder
PLUG_FLANGE_R = 0.0135  # 27.0 mm OD cap flange (>= 20 px at 1280x704)
PLUG_FLANGE_H = 0.0035
PLUG_TAB_SIZE = (0.020, 0.006, 0.015)
PLUG_SPAWN_Z = MAT_TOP + HOLDER_FLOOR_Z + 0.0005

SPRING_K_EFF = 240.0  # N/m total -> 13.4 N at full 56 mm compression (~1.4 kgf)
SPRING_K_JOINT = 2.0 * SPRING_K_EFF  # two equal springs in series: 480 N/m each
SPRING_DAMPING = 24.0
SPRING_FRICTION = 0.04

TETHER_MAX_M = 0.045  # 40 mm nominal tether + 5 mm margin

# Scoring thresholds (shared by on_step, score, success)
COMPRESSED_FRAC = 0.30  # <= 30% of expanded height (22.8 mm)
HELD_FRAC = 0.30  # still <= 30% of expanded height (22.8 mm) after release: stays flat
COMPRESSED_MAX_H = COMPRESSED_FRAC * H_EXPANDED
HELD_MAX_H = HELD_FRAC * H_EXPANDED
PLUG_SEAT_XY_TOL = 0.0065
PLUG_FLANGE_TOL_Z = 0.0045
PLUG_TILT_MAX_DEG = 22.0
DRAIN_TILT_MAX_DEG = 18.0
HAND_CLEAR_M = 0.050
CLEAR_HOLD_S = 5.0  # must stay <= 30% for 5 s after both grippers are clear

SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0  # clear_s >= 4.0 s + SUCCESS_HOLD_S 1.0 s = 5.0 s clear before termination
CLEAR_REQ_S = CLEAR_HOLD_S - SUCCESS_HOLD_S
PASS_SCORE = 1.0 - 1e-9

# -- colours & materials -----------------------------------------------------------------------------------

MAT_TEAL = (0.20, 0.40, 0.54)
PP_WHITE = (0.91, 0.92, 0.90)
PP_CREAM = (0.94, 0.94, 0.92)
STEEL = (0.75, 0.77, 0.80)
PORT_BLUE = (0.14, 0.46, 0.80)
TICK_NAVY = (0.12, 0.25, 0.50)
TUBE_CLEAR = (0.82, 0.91, 0.95)
BELLOWS_GLASS = (0.84, 0.92, 0.96)
GLASS_COLORS.add(BELLOWS_GLASS)
GLASS_COLORS.add(TUBE_CLEAR)


# -- multi-box / multi-cylinder assembly helpers -----------------------------------------------------------

def _oct_ring_boxes(cx: float, cy: float, z0: float, h: float, r_in: float, thick: float, color, pitch: float = 0.0):
    """8-sided hollow octagonal ring made of thin wall boxes around (cx, cy)."""
    boxes = []
    # Side length of octagon circumscribing r_in: 2 * r_in * tan(pi/8)
    side_w = 2.0 * (r_in + thick * 0.5) * math.tan(math.pi / 8.0) + 0.0012
    r_mid = r_in + thick * 0.5 + (0.5 * h * math.sin(pitch) if pitch else 0.0)
    zc = z0 + 0.5 * h
    for k in range(8):
        ang = k * math.pi / 4.0
        bx = cx + r_mid * math.cos(ang)
        by = cy + r_mid * math.sin(ang)
        boxes.append(((bx, by, zc), (thick, side_w, h), color, (ang, pitch)))
    return tuple(boxes)


def _spring_positions():
    r = 0.026
    return [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a in (60.0, 180.0, 300.0)]


def _drain_body_geometry():
    sx, sy = SPOUT_LOCAL
    hx, hy = HOLDER_LOCAL
    boxes = [
        # Stabilizing base foot under +X pouring-spout boss
        ((0.056, sy, H_BOTTOM_DISC / 2.0), (0.024, 0.024, H_BOTTOM_DISC), PP_WHITE),
        # Plug-holder bridge & open-Y cradle connecting lower rim to (hx, hy)
        ((0.064, -0.026, HOLDER_FLOOR_Z / 2.0), (0.042, 0.028, HOLDER_FLOOR_Z), PP_WHITE),
        ((hx, hy, HOLDER_FLOOR_Z / 2.0), (0.030, 0.028, HOLDER_FLOOR_Z), PP_WHITE),
        # Low +-X cradle ribs around (hx, hy) leaving +-Y completely open for gripper fingers
        ((hx - 0.012, hy, HOLDER_FLOOR_Z + 0.0020), (0.004, 0.024, 0.004), PP_WHITE),
        ((hx + 0.012, hy, HOLDER_FLOOR_Z + 0.0020), (0.004, 0.024, 0.004), PP_WHITE),
        # Graduation tick marks on front (+X/-Y) lower wall
        ((0.0, -0.0545, 0.014), (0.022, 0.0015, 0.0015), TICK_NAVY),
        # Visual tether anchor lug on rim between spout and socket
        ((0.066, -0.018, 0.006), (0.006, 0.006, 0.004), PORT_BLUE),
    ]
    cyls = [
        # Bottom polypropylene disc
        ((0.0, 0.0, H_BOTTOM_DISC / 2.0), DRAIN_RADIUS, H_BOTTOM_DISC, PP_WHITE),
        # Lower translucent PVC bellows skirt (z: 4.5..19.5 mm)
        ((0.0, 0.0, H_BOTTOM_DISC + 0.0075), 0.053, 0.015, BELLOWS_GLASS),
    ]
    # 3 lower steel spring seats
    for x, y in _spring_positions():
        cyls.append(((x, y, H_BOTTOM_DISC + 0.007), 0.0045, 0.014, STEEL))
        cyls.append(((x, y, H_BOTTOM_DISC + 0.0075), 0.0068, 0.003, STEEL))
    return tuple(boxes), tuple(cyls)


def _drain_joints():
    sx, sy = SPOUT_LOCAL
    # Joint 1: bellows_mid (middle translucent ring + 3 middle spring coils + 200 mL graduation stripe)
    mid_cyls = [
        ((0.0, 0.0, 0.040), 0.0515, 0.015, BELLOWS_GLASS),
    ] + [((x, y, 0.040), 0.0070, 0.010, STEEL) for x, y in _spring_positions()]
    mid_boxes = (
        ((0.0, -0.052, 0.040), (0.024, 0.0015, 0.0018), TICK_NAVY),
    )
    bellows_mid = Joint(
        "bellows_mid", kind="slide", axis="Z", anchor=(0.0, 0.0, 0.0),
        boxes=mid_boxes, cylinders=tuple(mid_cyls),
        lower=-STROKE_EACH, upper=0.0,
        stiffness=SPRING_K_JOINT, damping=SPRING_DAMPING, target=0.0, friction=SPRING_FRICTION,
        initial=0.0, mass=0.025,
    )

    # Joint 2: top_plate (upper skirt + top disc + 3 upper spring rods + inlet port & tube stub + hollow pouring spout)
    top_cyls = [
        ((0.0, 0.0, 0.066), 0.050, 0.011, BELLOWS_GLASS),  # upper skirt (60.5..71.5 mm)
        ((0.0, 0.0, H_EXPANDED - H_TOP_DISC / 2.0), DRAIN_RADIUS, H_TOP_DISC, PP_CREAM),  # top disc (71.5..76 mm)
        # +Y anti-reflux inlet port & short clear tubing stub (clear of -X press and +X spout)
        ((0.0, +0.036, H_EXPANDED + 0.004), 0.0060, 0.008, PORT_BLUE),
        ((0.0, +0.036, H_EXPANDED + 0.011), 0.0032, 0.010, TUBE_CLEAR),
        ((+0.008, +0.042, H_EXPANDED + 0.015), 0.0028, 0.018, TUBE_CLEAR, "X"),
    ] + [((x, y, 0.066), 0.0035, 0.011, STEEL) for x, y in _spring_positions()]

    spout_throat = _oct_ring_boxes(
        sx, sy, H_EXPANDED, SPOUT_THROAT_H, SPOUT_THROAT_R, SPOUT_WALL_T, PP_CREAM, pitch=0.0
    )
    spout_flare = _oct_ring_boxes(
        sx, sy, H_EXPANDED + SPOUT_THROAT_H, SPOUT_FLARE_H, SPOUT_THROAT_R, 0.0025, PP_CREAM, pitch=0.72
    )
    top_boxes = (
        # Moulded pouring-spout shoulder joining top disc to +X spout rim
        ((0.054, sy, H_EXPANDED - H_TOP_DISC / 2.0), (0.024, 0.020, H_TOP_DISC), PP_CREAM),
        # Moulded -X finger-press pad rim lug (matches top disc flush)
        ((-0.052, -0.005, H_EXPANDED - H_TOP_DISC / 2.0), (0.016, 0.024, H_TOP_DISC), PP_CREAM),
        *spout_throat,
        *spout_flare,
        ((0.0, -0.0505, 0.066), (0.022, 0.0015, 0.0016), TICK_NAVY),
        ((0.056, -0.016, H_EXPANDED - 0.002), (0.006, 0.005, 0.004), PORT_BLUE),  # upper tether lug
    )
    top_plate = Joint(
        "top_plate", kind="slide", axis="Z", parent="bellows_mid", anchor=(0.0, 0.0, 0.0),
        boxes=top_boxes, cylinders=tuple(top_cyls),
        lower=-STROKE_EACH, upper=0.0,
        stiffness=SPRING_K_JOINT, damping=SPRING_DAMPING, target=0.0, friction=SPRING_FRICTION,
        initial=0.0, mass=0.045,
    )
    return (bellows_mid, top_plate)


_BODY_BOXES, _BODY_CYLS = _drain_body_geometry()

MAT = part(
    "mat", MAT_POS, fixed=True, friction=1.4,
    boxes=(((0.0, 0.0, 0.0), MAT_SIZE, MAT_TEAL),),
)

DRAIN = part(
    "drain", (*DRAIN_XY, MAT_TOP), mass=0.075, friction=1.3,
    boxes=_BODY_BOXES, cylinders=_BODY_CYLS, joints=_drain_joints(),
)

PLUG = part(
    "plug", (DRAIN_XY[0] + HOLDER_LOCAL[0], DRAIN_XY[1] + HOLDER_LOCAL[1], PLUG_SPAWN_Z),
    mass=0.010, friction=1.3,
    cylinders=(
        ((0.0, 0.0, PLUG_TIP_H / 2.0), PLUG_TIP_R, PLUG_TIP_H, PP_CREAM),
        ((0.0, 0.0, PLUG_TIP_H + (PLUG_STEM_H - PLUG_TIP_H) / 2.0), PLUG_STEM_R, PLUG_STEM_H - PLUG_TIP_H, PP_CREAM),
        ((0.0, 0.0, PLUG_STEM_H + PLUG_FLANGE_H / 2.0), PLUG_FLANGE_R, PLUG_FLANGE_H, PORT_BLUE),
    ),
    boxes=(
        ((0.0, 0.0, PLUG_STEM_H + PLUG_FLANGE_H + PLUG_TAB_SIZE[2] / 2.0), PLUG_TAB_SIZE, PORT_BLUE),
    ),
)

OBJECTS = (MAT, DRAIN, PLUG)

# Photoreal visual meshes (assets/visual, built by assets_build/); the shapes above stay as invisible colliders.
_VIS = pathlib.Path(_HERE) / "assets" / "visual"
labkit.VISUALS.update({
    "mat/body": str(_VIS / "mat.usda"),
    "drain/body": str(_VIS / "drain_body.usda"),
    "drain/bellows_mid": str(_VIS / "drain_mid.usda"),
    "drain/top_plate": str(_VIS / "drain_top.usda"),
    "plug/body": str(_VIS / "plug.usda"),
})
# Visual-only bench props (SimReady, physics stripped; assets_build/build_props.py), referenced beside the mat:
# kidney dish, folded drape, glove box, disinfectant spray, specimen bag, syringe, all at x >= 0.42 m (no robot path).
labkit.SCENERY["mat"] = [(str(pathlib.Path(_HERE) / "assets" / "props" / "bench_props.usda"), (0.0, 0.0, TOP))]


# -- trial state (reset per episode; fail-closed unless simulator or test provides TCPs) ------------------

_trial: dict = {
    "compressed_ever": False,
    "lifted_off_mat": False,
    "sealed": False,
    "sealed_while_expanded": False,
    "sealed_q": (0.0, 0.0),
    "clear_s": 0.0,
    "tcps": None,
    "step": -1,
}
_ids: dict = {}


def _set_test_trial(*, compressed_ever=None, lifted_off_mat=None, sealed=None, sealed_while_expanded=None, clear_s=None, tcps=None) -> None:
    """Explicit test hook for offline scoring tests (never called by demo())."""
    if compressed_ever is not None:
        _trial["compressed_ever"] = bool(compressed_ever)
    if lifted_off_mat is not None:
        _trial["lifted_off_mat"] = bool(lifted_off_mat)
    if sealed is not None:
        _trial["sealed"] = bool(sealed)
    if sealed_while_expanded is not None:
        _trial["sealed_while_expanded"] = bool(sealed_while_expanded)
    if clear_s is not None:
        _trial["clear_s"] = float(clear_s)
    if tcps is not None:
        _trial["tcps"] = tuple(np.asarray(p, float) for p in tcps)


@on_reset
def _reset(_env) -> None:
    _trial.update(
        compressed_ever=False,
        lifted_off_mat=False,
        sealed=False,
        sealed_while_expanded=False,
        sealed_q=(0.0, 0.0),
        clear_s=0.0,
        tcps=None,
        step=-1,
    )
    _ids.clear()


def _arm_tcps(env) -> tuple[np.ndarray, np.ndarray] | None:
    try:
        base = env.unwrapped
        out = []
        for side in ("left", "right"):
            asset = base.scene[f"{side}_robot"]
            key = f"arm_{side}"
            if key not in _ids:
                _ids[key] = asset.find_joints(ARM_JOINTS, preserve_order=True)[0]
            q = labkit._np(asset.data.joint_pos)[0, _ids[key]]
            tcp_p, _ = ik.tcp_pose(q, station.BASES[side])
            out.append(tcp_p)
        return (out[0], out[1])
    except Exception:
        return None


def _drain_height_m(drain: PartState) -> float:
    q_mid = float(drain.joints.get("bellows_mid", 0.0))
    q_top = float(drain.joints.get("top_plate", 0.0))
    return H_EXPANDED + q_mid + q_top


def _tilt_deg(st: PartState) -> float:
    z_axis = st.rotation()[:, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(z_axis[2])))))


def _plug_in_spout(drain: PartState, plug: PartState) -> bool:
    """True when plug stem is seated upright inside the pouring spout with its flange flush on the spout rim."""
    q_sum = float(drain.joints.get("bellows_mid", 0.0)) + float(drain.joints.get("top_plate", 0.0))
    p_loc = drain.to_local(plug.pos)
    r_xy = math.hypot(float(p_loc[0] - SPOUT_LOCAL[0]), float(p_loc[1] - SPOUT_LOCAL[1]))
    if r_xy > PLUG_SEAT_XY_TOL:
        return False
    flange_z_loc = float(p_loc[2]) + PLUG_STEM_H
    rim_z_loc = H_EXPANDED + SPOUT_H + q_sum
    if abs(flange_z_loc - rim_z_loc) > PLUG_FLANGE_TOL_Z:
        return False
    dot_z = float(np.dot(plug.rotation()[:, 2], drain.rotation()[:, 2]))
    return dot_z >= math.cos(math.radians(PLUG_TILT_MAX_DEG))


def _plug_tether_intact(drain: PartState, plug: PartState) -> bool:
    q_sum = float(drain.joints.get("bellows_mid", 0.0)) + float(drain.joints.get("top_plate", 0.0))
    rim_w = drain.to_world((SPOUT_LOCAL[0], SPOUT_LOCAL[1], H_EXPANDED + SPOUT_H + q_sum))
    flange_w = plug.to_world((0.0, 0.0, PLUG_STEM_H))
    # Tether attaches at the spout collar / rim; if carried far away or knocked off bench, fail
    return float(np.linalg.norm(flange_w - rim_w)) <= TETHER_MAX_M + 0.045


def _hands_clear(drain: PartState, plug: PartState, tcps) -> bool:
    if tcps is None or len(tcps) < 2:
        return False
    d_lo, d_hi = np.asarray(drain.low, float) - HAND_CLEAR_M, np.asarray(drain.high, float) + HAND_CLEAR_M
    p_lo, p_hi = np.asarray(plug.low, float) - HAND_CLEAR_M, np.asarray(plug.high, float) + HAND_CLEAR_M
    for tp in tcps:
        t = np.asarray(tp, float)
        if np.all(t >= d_lo) and np.all(t <= d_hi):
            return False
        if np.all(t >= p_lo) and np.all(t <= p_hi):
            return False
    return True


def _on_mat(drain: PartState) -> bool:
    cx, cy = float(drain.pos[0]), float(drain.pos[1])
    mx, my, _ = MAT_POS
    hx, hy, _ = MAT_SIZE
    return (
        abs(cx - mx) <= hx / 2.0 + 0.02
        and abs(cy - my) <= hy / 2.0 + 0.02
        and float(drain.low[2]) >= TOP - 0.005
    )


# (name, joint_or_body) of each scored sub-action, as printed to the run log at its end value
SUBACTIONS = (
    ("compress middle bellows slide", "drain.bellows_mid"),
    ("compress upper plate slide", "drain.top_plate"),
    ("seat spout plug at compressed-state rim", "plug"),
)


def _log_subaction_end(drain: PartState, plug: PartState, step: int, run_end: bool) -> None:
    """Print one SUBACTION_END JSON line per sub-action: slide joints in m, plug origin in world m, the 0-based step,
    and run_end (the success hold is complete, so the runtime ends the run on this step).
    Called on every step of the full-score hold, so a successful run's last lines are its end state. Observe-only."""
    ends = (
        round(float(drain.joints.get("bellows_mid", 0.0)), 6),
        round(float(drain.joints.get("top_plate", 0.0)), 6),
        [round(float(v), 6) for v in plug.pos],
    )
    for (name, key), end in zip(SUBACTIONS, ends):
        rec = {"name": name, "joint_or_body": key, "end_value": end, "step": step, "run_end": run_end}
        print("SUBACTION_END " + json.dumps(rec), flush=True)


@on_step
def _watch(env) -> None:
    base = env.unwrapped
    drain = labkit._part_state(env, labkit._SPECS["drain"])
    plug = labkit._part_state(env, labkit._SPECS["plug"])
    _trial["step"] += 1

    _trial["tcps"] = _arm_tcps(env)
    h = _drain_height_m(drain)
    if float(drain.pos[2]) > MAT_TOP + 0.025:
        _trial["lifted_off_mat"] = True
        _trial["compressed_ever"] = False
    elif (
        h <= COMPRESSED_MAX_H
        and _tilt_deg(drain) <= DRAIN_TILT_MAX_DEG
        and _on_mat(drain)
        and float(drain.pos[2]) <= MAT_TOP + 0.012
        and not _trial["lifted_off_mat"]
    ):
        _trial["compressed_ever"] = True

    seated_now = _plug_in_spout(drain, plug)
    if seated_now:
        if not _trial["sealed"]:
            _trial["sealed"] = True
            q_mid = float(np.clip(drain.joints.get("bellows_mid", 0.0), -STROKE_EACH, 0.0))
            q_top = float(np.clip(drain.joints.get("top_plate", 0.0), -STROKE_EACH, 0.0))
            _trial["sealed_q"] = (q_mid, q_top)
            if h > COMPRESSED_MAX_H:
                _trial["sealed_while_expanded"] = True
    else:
        _trial["sealed"] = False
        _trial["sealed_q"] = (0.0, 0.0)

    if (
        seated_now
        and not _trial["sealed_while_expanded"]
        and h <= HELD_MAX_H
        and _tilt_deg(drain) <= DRAIN_TILT_MAX_DEG
        and _on_mat(drain)
        and _hands_clear(drain, plug, _trial["tcps"])
        and drain.resting()
        and plug.resting()
    ):
        _trial["clear_s"] += float(getattr(base, "step_dt", 0.05))
    else:
        _trial["clear_s"] = 0.0
    if _trial["clear_s"] >= CLEAR_REQ_S:
        dt = float(getattr(base, "step_dt", 0.05))
        _log_subaction_end(drain, plug, _trial["step"], _trial["clear_s"] >= CLEAR_HOLD_S - 0.5 * dt)

    # Trapped vacuum lock: when the plug seals the spout, lock joint targets at sealed_q;
    # if unsealed, springs expand back to (0.0, 0.0).
    import torch

    asset = base.scene["drain"]
    if "drain_j" not in _ids:
        names = list(asset.joint_names)
        _ids["drain_j"] = [names.index(j.joint_name) for j in DRAIN.joints]
    tgt = list(_trial["sealed_q"]) if _trial["sealed"] else [0.0, 0.0]
    asset.set_joint_position_target(torch.tensor([tgt], device=asset.device), joint_ids=_ids["drain_j"])


# -- scoring & task hooks ----------------------------------------------------------------------------------

def score(objects: dict[str, PartState]) -> float:
    """Partial credit:
    - 0.0: plug seated while expanded, reservoir tipped/off-mat, or not yet compressed.
    - 0.3: top and bottom brought into contact (<= 30% expanded height) at any time.
    - 0.7: plug seated in spout while reservoir <= 30% expanded height (still held or < 4 s clear).
    - 1.0: reservoir still <= 30% expanded height, plug seated, resting, and both grippers clear >= 4 s
      (the run then ends as a success after 1 s more).
    """
    drain = objects["drain"]
    plug = objects["plug"]

    if _tilt_deg(drain) > DRAIN_TILT_MAX_DEG or not _on_mat(drain) or _trial["lifted_off_mat"]:
        return 0.0
    if float(plug.low[2]) < TOP - 0.005:
        return 0.0

    h = _drain_height_m(drain)
    seated = _plug_in_spout(drain, plug)
    compressed_now = h <= COMPRESSED_MAX_H
    compressed_ever = _trial["compressed_ever"] or compressed_now

    if _trial["sealed_while_expanded"] or (seated and not compressed_ever and h > COMPRESSED_MAX_H):
        return 0.0

    if not compressed_ever:
        return 0.0

    if not seated:
        return 0.3

    # Seated after valid compression
    if (
        h <= HELD_MAX_H
        and _hands_clear(drain, plug, _trial["tcps"])
        and drain.resting()
        and plug.resting()
        and _trial["clear_s"] >= CLEAR_REQ_S
    ):
        return 1.0

    return 0.7


def success(objects: dict[str, PartState]) -> bool:
    """Reservoir compressed, plug seated, both grippers clear, and held <= 30% height for 5 s total."""
    return score(objects) >= PASS_SCORE


def distance(objects: dict[str, PartState]) -> float:
    """Remaining compression + plug-to-spout rim distance (m)."""
    drain = objects["drain"]
    plug = objects["plug"]
    h = _drain_height_m(drain)
    q_sum = float(drain.joints.get("bellows_mid", 0.0)) + float(drain.joints.get("top_plate", 0.0))
    rim_w = drain.to_world((SPOUT_LOCAL[0], SPOUT_LOCAL[1], H_EXPANDED + SPOUT_H + q_sum))
    flange_w = plug.to_world((0.0, 0.0, PLUG_STEM_H))
    return float(max(0.0, h - H_COMPRESSED) + np.linalg.norm(flange_w - rim_w))


# -- scripted demo (from fixed scene geometry only) --------------------------------------------------------

def demo():
    """Bimanual recharge:
    - Left arm closes its fingers, presses straight down on the centre of the reservoir top disc until flat (20 mm),
      holds steady while the right arm inserts the plug, then withdraws clear after the right arm clears.
    - Right arm picks the plug by its pull-tab from the holder socket, carries it over the pouring spout, pushes
      straight down to seat the flange flush on the spout rim, opens to release, and parks clear.
    - Both arms hold parked clear for > 5 s so the post-release hold check passes.
    """
    W = _demo.Waypoint
    c = FINGER_CLOSED
    wide = 0.45 * FINGER_OPEN  # ~42 mm open jaw around the 16 x 5 mm pull-tab

    dx, dy = DRAIN_XY
    lx, ly = dx + PRESS_LOCAL[0], dy + PRESS_LOCAL[1]
    sx, sy = dx + SPOUT_LOCAL[0], dy + SPOUT_LOCAL[1]
    hx, hy = dx + HOLDER_LOCAL[0], dy + HOLDER_LOCAL[1]

    # Left gripper TCP heights (pad tips are PAD_TIP_PAST_TCP = 13 mm below TCP)
    l_hover_z = MAT_TOP + H_EXPANDED + PAD_TIP_PAST_TCP + 0.030
    l_touch_z = MAT_TOP + H_EXPANDED + PAD_TIP_PAST_TCP + 0.002
    # Overtravel below MAT_TOP + H_COMPRESSED + PAD_TIP_PAST_TCP (MAT_TOP + 0.033) to overcome 13.4 N spring
    l_press_z = MAT_TOP + H_COMPRESSED + PAD_TIP_PAST_TCP - 0.022
    l_park = (0.20, +0.26, MAT_TOP + 0.140)

    # Right gripper TCP heights: pick pull-tab (tab spans z = PLUG_SPAWN_Z + 0.016..0.031)
    # Pad tips are PAD_TIP_PAST_TCP = 13 mm below TCP -> r_pick_z places tips just above flange top
    r_pick_z = PLUG_SPAWN_Z + PLUG_STEM_H + PLUG_FLANGE_H + PAD_TIP_PAST_TCP + 0.001
    r_lift_z = MAT_TOP + H_COMPRESSED + SPOUT_H + 0.042
    r_clear_z = MAT_TOP + H_COMPRESSED + SPOUT_H + 0.040
    # When seated on compressed spout rim (MAT_TOP + 0.034), flange bottom is at MAT_TOP + 0.034,
    # so plug origin is at MAT_TOP + 0.0215; TCP offset above plug origin = r_pick_z - PLUG_SPAWN_Z.
    r_seat_z = MAT_TOP + H_COMPRESSED + SPOUT_H - PLUG_STEM_H + (r_pick_z - PLUG_SPAWN_Z) - 0.003
    r_park = (0.20, -0.26, MAT_TOP + 0.140)

    left = [
        W(1.0, (lx, ly, l_hover_z), c, "over reservoir press zone"),
        W(1.3, (lx, ly, l_press_z), c, "compress springs flat"),
        W(6.8, (lx, ly, l_press_z), c, "hold flat while right arm seats plug"),
        W(0.8, (lx, ly, l_hover_z), c, "lift clear after plug release"),
        W(0.8, l_park, c, "park left arm clear"),
        W(5.1, l_park, c, "wait 5s post-release hold"),
    ]

    right = [
        W(2.1, (0.36, -0.18, MAT_TOP + 0.100), wide, "wait on -Y until left finishes press"),
        W(0.9, (hx, hy, r_lift_z), wide, "over plug pull-tab"),
        W(0.8, (hx, hy, r_pick_z), wide, "lower onto pull-tab"),
        W(0.5, (hx, hy, r_pick_z), c, "pinch pull-tab"),
        W(0.7, (hx, hy, r_lift_z), c, "lift plug out of holder"),
        W(1.0, (sx, sy, r_lift_z), c, "move plug over spout"),
        W(0.4, (sx, sy, r_lift_z), c, "settle over spout funnel"),
        W(0.9, (sx, sy, r_seat_z), c, "push plug flush into spout"),
        W(0.4, (sx, sy, r_seat_z), wide, "open fingers to release plug"),
        W(0.7, (sx, sy, r_clear_z), wide, "lift clear of spout"),
        W(0.6, (0.375, -0.05, MAT_TOP + 0.085), wide, "retract clear on +X/-Y"),
        W(0.7, (0.34, -0.18, MAT_TOP + 0.110), wide, "swing clear to -Y"),
        W(0.8, r_park, wide, "park right arm clear"),
        W(5.3, r_park, wide, "wait 5s post-release hold"),
    ]

    return {"left": left, "right": right}
