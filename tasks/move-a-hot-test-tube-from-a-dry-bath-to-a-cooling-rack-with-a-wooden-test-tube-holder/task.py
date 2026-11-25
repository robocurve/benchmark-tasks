"""Transferring a glass reaction tube out of a 95.0 C benchtop dry-bath incubator into a stainless Z-shape cooling rack
using a self-closing (cross-action) wooden clothespin test-tube holder, then setting the holder back on its blue
silicone work mat.

How it is built (scene.py, every figure from parts.py):
- ``bath``: FOUR E'S SCIENTIFIC TC0401001 dry bath (1.50 kg) with a sloped front ``95.0`` LED readout and a recessed
  96 x 76 mm aluminium block carrying a chamfered 45.0 mm deep bore (inscribed octagon 18.6 mm).
- ``tube``: DWK Life Sciences PYREX 1620/09M 16.0 x 100.0 mm borosilicate rimmed test tube standing in the heated well.
- ``long_arm`` and ``short_lever``: Eisco Labs CH0726B 260.4 mm wooden clothespin holder on a preloaded torsion-spring
  hinge ``jaw_h`` with force-gated anti-creep barrel weld ``w_grip`` (engaged where the tube stands only after >= 10 deg
  opening and bilateral >= 6 N jaw load, cut at >= 11.5 deg opening or > 18 N jam; no computed push). At rest the spring
  closes the four 30-deg V-cheeks to 10.0 mm; squeezing the rear tail opens the V-jaws up to 31.8 mm; releasing around
  the 16.0 mm glass neck clamps the tube under ~15.6 N normal load.
- ``rack``: BOCHEM 8291 18/10 stainless Z-shape rack (220 x 110 x 50 mm) with 10 x 5 chamfered 18.4 mm apertures across
  upper and middle guide tiers over a solid floor.
- ``mat``: TRU COMPONENTS 300 x 200 x 2 mm blue silicone work mat where the wooden holder starts and ends.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
import scene  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 3.0  # both arms finish returning to ready before the trial ends so the last frame shows the scene clear
INSTRUCTION = (
    "Move the glass test tube from the dry bath into the stainless cooling rack with the wooden test-tube holder, "
    "without touching the tube or the heating block with the grippers, then lay the holder back on the blue mat."
)
RUN_LIMIT_STEPS = 2000  # 100 s at 20 Hz
TOP = TABLE_TOP[SCENE]
SIDES = ("left", "right")
SCORED_BODIES = ("tube", "bath", "rack", "long_arm", "short_lever")

scene.build(TOP)
OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    mk.mend_all()
    STATE.clear()
    STATE.update(
        seed=seed,
        zero=None,
        hold=0,
        hold_tube=None,
        hold_clamp=None,
        hands=None,
        was_opened=False,
        grip_ready=0,
        max_jaw_deg=0.0,
        peak_jaw_force=0.0,
        first_lifted=None,
        first_racked=None,
    )
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)


# --- geometry and contact queries -----------------------------------------------------------------------------------------
def body_tilt_deg(name: str) -> float:
    """Angle (deg) of body ``name``'s local +z axis from world vertical."""
    z_up = float(mk.pose(name)[1][2, 2])
    return float(math.degrees(math.acos(max(-1.0, min(1.0, z_up)))))


def holder_flat_deg(name: str) -> float:
    """Angle (deg) of symmetric wooden bar ``name``'s flat side from horizontal (either side up counts)."""
    z_abs = min(1.0, abs(float(mk.pose(name)[1][2, 2])))
    return float(math.degrees(math.acos(z_abs)))


def tube_tilt_deg() -> float:
    return body_tilt_deg("tube")


def tube_bottom_world() -> np.ndarray:
    """World coordinates of the centre of the test tube's bottom face."""
    p, r = mk.pose("tube")
    return p - r[:, 2] * (P.TUBE_L / 2.0)


def tube_bottom_above_bench() -> float:
    return float(tube_bottom_world()[2] - TOP)


def jaw_angle() -> float:
    """rad ``short_lever`` stands open from its built closed pose."""
    return float(mk.joint("jaw_h"))


def jaw_forces_on_tube() -> tuple[float, float]:
    """Normal clamping forces (N) of ``long_arm`` and ``short_lever`` on ``tube``."""
    return float(mk.contact_force("tube", "long_arm")), float(mk.contact_force("tube", "short_lever"))


def fixture_displaced(name: str) -> bool:
    p, _ = mk.pose(name)
    b, _ = mk.built_pose(name)
    slide = float(math.hypot(p[0] - b[0], p[1] - b[1]))
    return slide > P.FIXTURE_SLIDE_MAX or body_tilt_deg(name) > P.FIXTURE_TILT_DEG


# --- the hands ------------------------------------------------------------------------------------------------------------
def read_hands() -> None:
    scene.read_hands_into(STATE, SIDES)


def hands_clear() -> float:
    """m from the nearest hull corner of either hand to the tube, dry bath, cooling rack or wooden holder."""
    hands = STATE.get("hands")
    if not hands:
        return 0.0
    points = np.vstack([hands[s] for s in SIDES if s in hands])
    return min(scene.box_distance(points, name) for name in SCORED_BODIES)


# --- whole-trial zero rules and end levels --------------------------------------------------------------------------------
def zero_now() -> str | None:
    """Whole-trial rules: the reason the trial is permanently scored 0, or None."""
    for side in SIDES:
        robot = f"{side} robot"
        if mk.touching("tube", robot):
            return f"{side} gripper touched the glass tube directly"
        if mk.touching("bath", robot, shapes=scene.HOT_SHAPES):
            return f"{side} gripper touched the 95 C heating block"
    if mk.touching("tube", "scene") or mk.touching("tube", "mat"):
        return "glass tube touched the bench or mat"
    if tube_tilt_deg() > P.TUBE_DROP_TILT_DEG:
        return "glass tube tilted more than 35 degrees"
    if fixture_displaced("bath") or fixture_displaced("rack"):
        return "dry bath or cooling rack displaced more than 20 mm or tilted more than 10 degrees"
    return None


def tube_in_vpocket() -> bool:
    """Whether the tube's axis sits inside long_arm's V-cradle within 4.5 mm horizontally and along the barrel."""
    pl, rl = mk.pose("long_arm")
    pt, rt = mk.pose("tube")
    rel = rl.T @ (pt - pl)
    axis_l = rl.T @ rt[:, 2]
    if abs(float(axis_l[2])) < 0.97:
        return False
    # Point of the tube's centreline at the height of long_arm's midplane (rel_z = 0):
    at_jaw = rel - axis_l * (float(rel[2]) / float(axis_l[2]))
    radial = math.hypot(float(at_jaw[0]) - P.X_JAW, float(at_jaw[1]) - P.tube_y_in_clamp(P.TUBE_R))
    # Clamp sits on the exposed glass barrel below the rolled rim:
    along = float(rt[:, 2] @ (pl - pt))
    return radial <= 0.0045 and (-P.TUBE_L / 2.0 + 0.015 <= along <= P.TUBE_L / 2.0 - P.RIM_H)


def update_clamp_hold() -> None:
    """Engage or release w_grip strictly from physical jaw opening and bilateral spring clamping forces."""
    deg = math.degrees(jaw_angle())
    if deg >= P.WELD_OPEN_ARM_DEG:
        STATE["was_opened"] = True
    fl, fs = jaw_forces_on_tube()
    jam = max(float(mk.contact_force("tube", "bath")), float(mk.contact_force("tube", "rack")))
    if not mk.is_cut("w_grip"):
        if deg >= P.WELD_RELEASE_DEG or fs < 1.5 or jam > P.WELD_SLIP_N:
            mk.cut("w_grip")
            STATE["grip_ready"] = 0
        return
    if (
        STATE.get("was_opened", False)
        and deg < P.WELD_RELEASE_DEG
        and fl >= P.WELD_MAKE_N
        and fs >= P.WELD_MAKE_N
        and tube_in_vpocket()
    ):
        STATE["grip_ready"] = STATE.get("grip_ready", 0) + 1
        if STATE["grip_ready"] >= 2:
            mk.make("w_grip")
    else:
        STATE["grip_ready"] = 0


def tube_clamped_and_lifted() -> bool:
    """0.25 condition: holder clamps the tube (>= 4 N spring load) and the tube's underside is >= 90 mm over the bench."""
    fl, fs = jaw_forces_on_tube()
    held = (not mk.is_cut("w_grip") or fl >= P.CLAMP_SEATED_N) and fs >= P.CLAMP_SEATED_N
    return held and tube_bottom_above_bench() >= P.LIFT_CLEAR_Z and tube_tilt_deg() <= P.LIFT_TILT_MAX_DEG


def rack_hole_match() -> tuple[int, int] | None:
    """(col, row) of the BOCHEM 8291 hole the upright tube passes through at both guide tiers, or None."""
    if tube_tilt_deg() > P.RACK_UPRIGHT_DEG:
        return None
    pr, rr = mk.pose("rack")
    _, rt = mk.pose("tube")
    bot_r = rr.T @ (tube_bottom_world() - pr)
    if not (P.RACK_SHEET_T - 0.002 <= bot_r[2] <= P.RACK_SHEET_T + P.RACK_FLOOR_TOL):
        return None
    axis_r = rr.T @ rt[:, 2]
    if axis_r[2] < 0.98:
        return None
    # Where the tube's centreline crosses the middle (24 mm) and upper (50 mm) perforated tiers in rack coordinates:
    p_mid = bot_r + axis_r * ((P.RACK_MID_Z - bot_r[2]) / axis_r[2])
    p_top = bot_r + axis_r * ((P.RACK_H - bot_r[2]) / axis_r[2])
    for col in range(P.RACK_COLS):
        hx = (col - (P.RACK_COLS - 1) / 2.0) * P.RACK_PITCH
        for row in range(P.RACK_ROWS):
            hy = (row - (P.RACK_ROWS - 1) / 2.0) * P.RACK_PITCH
            if (
                math.hypot(p_mid[0] - hx, p_mid[1] - hy) <= P.RACK_HOLE_RADIAL_TOL
                and math.hypot(p_top[0] - hx, p_top[1] - hy) <= P.RACK_HOLE_RADIAL_TOL
            ):
                return (col, row)
    return None


def tube_in_rack() -> bool:
    return rack_hole_match() is not None


def holder_on_mat() -> bool:
    """Wooden holder lies flat inside the 300 x 200 mm blue mat, >= 20 mm clear of tube, bath and rack."""
    edge_tol = 0.005
    for part in ("long_arm", "short_lever"):
        p, r = mk.pose(part)
        if holder_flat_deg(part) > P.HOLDER_FLAT_DEG or (p[2] - TOP) > P.HOLDER_MAX_Z:
            return False
        lo, hi = mk.MECH.bodies[part].bounds()
        corners = np.array([[x, y, 0.0] for x in (lo[0], hi[0]) for y in (lo[1], hi[1])]) @ r.T + p
        if np.any(np.abs(corners[:, 0] - P.MAT_XY[0]) > P.MAT_L / 2.0 + edge_tol):
            return False
        if np.any(np.abs(corners[:, 1] - P.MAT_XY[1]) > P.MAT_W / 2.0 + edge_tol):
            return False
        if min(scene.bodies_gap(part, other) for other in ("tube", "bath", "rack")) < P.HOLDER_CLEAR:
            return False
    return True


def full_now() -> bool:
    return tube_in_rack() and holder_on_mat() and hands_clear() >= P.HANDS_CLEAR


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        read_hands()
    fl, fs = jaw_forces_on_tube()
    STATE["max_jaw_deg"] = max(STATE.get("max_jaw_deg", 0.0), math.degrees(jaw_angle()))
    STATE["peak_jaw_force"] = max(STATE.get("peak_jaw_force", 0.0), fs if not mk.is_cut("w_grip") else min(fl, fs))
    update_clamp_hold()
    if STATE.get("zero") is None:
        STATE["zero"] = zero_now()
    t_now = mk.now() if mk.MECH.environment is not None else 0.0
    if STATE.get("first_lifted") is None and STATE.get("zero") is None and tube_clamped_and_lifted():
        STATE["first_lifted"] = t_now
    if STATE.get("first_racked") is None and STATE.get("zero") is None and tube_in_rack():
        STATE["first_racked"] = t_now
    if STATE.get("zero") is None and full_now():
        pt, pc = mk.pose("tube")[0].copy(), mk.pose("long_arm")[0].copy()
        if STATE.get("hold", 0) == 0:
            STATE["hold_tube"], STATE["hold_clamp"] = pt, pc
        STATE["hold"] = STATE.get("hold", 0) + 1
        drift = max(float(np.linalg.norm(pt - STATE["hold_tube"])), float(np.linalg.norm(pc - STATE["hold_clamp"])))
        if drift > P.HOLD_DRIFT:
            STATE["hold"], STATE["hold_tube"], STATE["hold_clamp"] = 0, None, None
    else:
        STATE["hold"], STATE["hold_tube"], STATE["hold_clamp"] = 0, None, None


def level() -> float:
    if STATE.get("zero") is not None or zero_now() is not None:
        return 0.0
    if STATE.get("hold", 0) >= P.HOLD_STEPS and full_now():
        return 1.0
    if tube_in_rack():
        return 0.5
    if tube_clamped_and_lifted():
        return 0.25
    return 0.0


def score(objects) -> float:
    """0 for good if either gripper touches the glass tube or the 96 x 76 mm heating block, the tube touches the bench or
    mat or tilts past 35 degrees, or the dry bath or rack slides past 20 mm or tilts past 10 degrees. Otherwise at the end:
    1.0 when the tube stands upright (<= 8 deg) through both tiers of any hole of the cooling rack within 6 mm of its 2 mm
    floor, the wooden holder lies flat (<= 12 deg, centre <= 26 mm over bench) inside the 300 x 200 mm blue mat (within 5 mm
    of its edge) and >= 20 mm clear of tube, bath and rack, and both hands stand >= 30 mm clear for 1.0 s (moving <= 3 mm);
    0.5 when the tube stands upright in a rack hole as above, the other 1.0 conditions not all met; 0.25 when the wooden
    holder's jaws hold the tube by its barrel clear of the dry bath (underside >= 90 mm over the bench, <= 15 deg tilt;
    simulator stand-in: >= 4 N spring jaw contact); 0 otherwise."""
    return level()


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {
        "zero": STATE.get("zero"),
        "hold steps": STATE.get("hold", 0),
        "max jaw open (deg)": round(STATE.get("max_jaw_deg", 0.0), 2),
        "peak jaw force (N)": round(STATE.get("peak_jaw_force", 0.0), 2),
        "first lifted (s)": STATE.get("first_lifted"),
        "first racked (s)": STATE.get("first_racked"),
    }
    if mk.MECH.poses:
        fl, fs = jaw_forces_on_tube()
        out.update(
            {
                "tube bottom over bench (mm)": round(1000 * tube_bottom_above_bench(), 2),
                "tube tilt (deg)": round(tube_tilt_deg(), 2),
                "jaw open (deg)": round(math.degrees(jaw_angle()), 2),
                "jaw gap (mm)": round(1000 * P.jaw_gap(jaw_angle()), 2),
                "jaw forces on tube (N)": [round(fl, 2), round(fs, 2)],
                "rack hole": rack_hole_match(),
                "holder on mat": holder_on_mat(),
                "hands clear (mm)": round(1000 * hands_clear(), 1),
                "score": score(None),
            }
        )
    return out
