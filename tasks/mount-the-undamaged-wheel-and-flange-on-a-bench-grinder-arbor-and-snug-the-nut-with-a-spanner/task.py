"""Mount the undamaged 150 mm wheel and relieved steel outer flange onto the bare right arbor of an unplugged Einhell
TC-BG 150 bench grinder, snug the 19 mm arbor nut with a GEDORE 894 19 spanner while counter-holding the opposite
wheel, and set the spanner down on the bench away from the grinder. Figures live in ``parts.py``; bodies and joints live in ``scene.py``."""

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
SUCCESS_HOLD_S = 2.5
INSTRUCTION = (
    "Mount a 150 mm grinding wheel and the steel outer flange onto the bare right arbor of the unplugged bench "
    "grinder, snug the 19 mm arbor nut with the open-end spanner, and set the spanner down on the bench away from the "
    "grinder, then let go "
    "and move both hands clear."
)
RUN_LIMIT_STEPS = 1600  # 80 s at 20 Hz
TOP = TABLE_TOP[SCENE]
SIDES = ("left", "right")
FREE_PARTS = ("wheel_sound", "wheel_cracked", "outer_flange", "arbor_nut", "wrench")
CLEAR_BODIES = ("grinder", "spindle", *FREE_PARTS)

scene.build(TOP)
OBJECTS = list(mk.MECH.bodies.values())
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    STATE.clear()
    STATE.update(
        seed=seed,
        zero=None,
        hold=0,
        hold_from=None,
        hands=None,
        snug_ref=0.0,
        nut_held=0.0,
        nut_entry_roll=None,
        nut_last_roll=None,
        nut_turned_deg=0.0,
        peak_angle_deg=0.0,
        peak_torque_nm=0.0,
        peak_wrench_n=0.0,
        first_half=None,
    )
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)
    if "nut_turn" in mk.MECH.joint_prims:
        mk.set_spring("nut_turn", rest=0.0, most=P.TAU_RUN)


def radial_error(pos: np.ndarray) -> float:
    return float(math.hypot(float(pos[0]) - P.GRINDER_X, float(pos[2]) - (TOP + P.ARBOR_Z)))


def wheel_tilt_deg() -> float:
    _, rw = mk.pose("wheel_sound")
    return float(math.degrees(math.acos(min(1.0, abs(float(rw[1, 1]))))))


def wheel_inner_gap() -> float:
    pw, _ = mk.pose("wheel_sound")
    return abs(P.Y_INNER_FACE - (float(pw[1]) + P.WHEEL_T / 2.0))


def wheel_seated() -> bool:
    pw, _ = mk.pose("wheel_sound")
    return radial_error(pw) <= P.RADIAL_TOL and wheel_tilt_deg() <= P.TILT_TOL_DEG and wheel_inner_gap() <= P.AXIAL_GAP_TOL


def flange_tilt_deg() -> float:
    _, rf = mk.pose("outer_flange")
    return float(math.degrees(math.acos(max(-1.0, min(1.0, float(rf[1, 1]))))))


def flange_rim_gap() -> float:
    """m axial gap from outer_flange's Ø 52 mm bearing rim (local +y) to wheel_sound's -y blotter face."""
    pw, _ = mk.pose("wheel_sound")
    pf, rf = mk.pose("outer_flange")
    rim_y = float((pf + rf @ np.array([0.0, P.FLANGE_DEPTH / 2.0, 0.0]))[1])
    return abs((float(pw[1]) - P.WHEEL_T / 2.0) - rim_y)


def flange_seated() -> bool:
    pf, _ = mk.pose("outer_flange")
    return (
        wheel_seated()
        and radial_error(pf) <= P.RADIAL_TOL
        and flange_tilt_deg() <= P.TILT_TOL_DEG
        and flange_rim_gap() <= P.AXIAL_GAP_TOL
    )


def nut_engaged() -> bool:
    return not mk.is_cut("nut_catch")


def witness_angle_deg() -> float:
    if not nut_engaged():
        return 0.0
    return float(math.degrees(mk.joint("nut_turn") - float(STATE.get("snug_ref", 0.0))))


def stack_clamped() -> bool:
    return wheel_seated() and flange_seated() and nut_engaged()


def current_torque_nm() -> float:
    if not nut_engaged():
        return 0.0
    if not (wheel_seated() and flange_seated()):
        return P.TAU_RUN
    return P.thread_torque_for_angle_deg(max(0.0, witness_angle_deg()), stack_clamped=True)


def clamp_preload_n() -> float:
    return P.clamp_force_for_angle_deg(max(0.0, witness_angle_deg())) if stack_clamped() else 0.0


def cracked_shift_m() -> float:
    pc, _ = mk.pose("wheel_cracked")
    bc, _ = mk.built_pose("wheel_cracked")
    dz = max(0.0, abs(float(pc[2] - bc[2])) - 0.002)
    return float(math.sqrt((pc[0] - bc[0]) ** 2 + (pc[1] - bc[1]) ** 2 + dz * dz))


def wrench_offset_m() -> float:
    return float(np.linalg.norm(mk.pose("wrench")[0] - mk.built_pose("wrench")[0]))


def wrench_from_nut_m() -> float:
    return float(np.linalg.norm(mk.pose("wrench")[0] - mk.pose("arbor_nut")[0]))


def wrench_set_down() -> bool:
    """The spanner is off the nut and down low: its jaw at least WRENCH_CLEAR_NUT from the nut and at most WRENCH_DOWN_Z
    over the bench (on the bench or in its rack; not perched on the grinder, not held up)."""
    return wrench_from_nut_m() >= P.WRENCH_CLEAR_NUT and float(mk.pose("wrench")[0][2]) - TOP <= P.WRENCH_DOWN_Z


def bodies_at_rest() -> bool:
    return all(float(np.linalg.norm(mk.velocity(n)[0])) <= P.REST_LIN_MAX for n in ("spindle", *FREE_PARTS))


def zero_now() -> str | None:
    if cracked_shift_m() > P.CRACKED_MOVE_ZERO:
        return "cracked wheel moved more than 10 mm from its rack slot"
    if stack_clamped() and witness_angle_deg() > P.ANGLE_CRACK_DEG:
        return "arbor nut over-tightened past 195 degrees (27.1 N m), the maker limit for this wheel size"
    for name in FREE_PARTS:
        if float(mk.pose(name)[0][2]) < TOP + P.DROP_Z_BELOW_TOP:
            return f"{name} dropped off the bench"
    return None


_HULLS: dict = {}
_LINKS: dict = {}


def read_hands() -> None:
    from yam_arena import analysis

    if not _HULLS:
        for link in ("gripper", "tip_left", "tip_right"):
            _HULLS[link] = np.vstack([h.vertices for h in analysis.link_hulls().get(link, [])])
    base = mk.MECH.environment.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    hands = {}
    for side in SIDES:
        robot = base.scene[f"{side}_robot"]
        pts = []
        for link, local in _HULLS.items():
            key = (id(base), side, link)
            if key not in _LINKS:
                _LINKS[key] = robot.find_bodies(link)[0][0]
            pw = robot.data.body_link_pose_w.torch[0, _LINKS[key]].detach().cpu().numpy().astype(float)
            pts.append(local @ mk.quat_to_matrix(pw[3:7]).T + pw[:3] - origin)
        hands[side] = np.vstack(pts)
    STATE["hands"] = hands


def _box_distance(points: np.ndarray, name: str) -> float:
    position, rotation = mk.pose(name)
    low, high = mk.MECH.bodies[name].bounds()
    local = (points - position) @ rotation
    gap = np.maximum(np.maximum(np.asarray(low) - local, local - np.asarray(high)), 0.0)
    return float(np.linalg.norm(gap, axis=1).min())


def hands_clear() -> float:
    hands = STATE.get("hands")
    if not hands:
        return 0.0
    points = np.vstack([hands[s] for s in SIDES if s in hands])
    return min(_box_distance(points, name) for name in CLEAR_BODIES)


def _nut_roll_on_spindle() -> float:
    rel = mk.pose("spindle")[1].T @ mk.pose("arbor_nut")[1]
    return float(math.atan2(-float(rel[2, 0]), float(rel[0, 0])))


def _nut_touches_seat() -> bool:
    """The nut's collar bears on the outer flange: rubbing the bare thread alone is not the snug point."""
    if mk.contact_force("outer_flange", "arbor_nut") >= 0.25:
        return True
    if mk.MECH.bodies["arbor_nut"].contacts and mk.MECH.contact_points:
        return any(abs(c.force) >= 0.15 for c in mk.touching("arbor_nut", "outer_flange", within=0.0018))
    return False


def _carry_threaded_nut() -> None:
    """Threaded, the nut takes the solver's stand-in mass and spin inertia (parts.NUT_SOLVER_MASS, SOLVER_SPIN_INERTIA),
    written the way ``mk.set_mass`` writes them; back to the built ones at the next reset."""
    mk.set_mass("arbor_nut", P.NUT_SOLVER_MASS)
    obj = mk.MECH.environment.unwrapped.scene.rigid_objects["arbor_nut"]
    inertia = obj.data.body_inertia.torch[:1].clone() * 0.0
    for i, v in zip((0, 4, 8), P.SOLVER_SPIN_INERTIA):
        inertia[..., i] = float(v)
    obj.set_inertias_index(inertias=inertia)


def _step_nut_thread() -> None:
    if not nut_engaged():
        pn, rn = mk.pose("arbor_nut")
        # square to the arbor within 2 deg (cocked further, a nut cross-threads); a catch cocked 2.5 deg let the spanner's
        # jaw roll round the hex at 2.1 N m instead of 3.4 N m (probe gw_p22)
        coaxial = radial_error(pn) <= 0.0040 and float(rn[1, 1]) >= math.cos(math.radians(P.CATCH_TILT_DEG))
        in_zone = (P.Y_ARBOR_TIP - 0.003) <= float(pn[1]) <= (P.Y_INNER_FACE - 0.005)
        if coaxial and in_zone and _nut_touches_seat():
            roll = _nut_roll_on_spindle()
            if STATE.get("nut_entry_roll") is None:
                STATE["nut_entry_roll"] = roll
                STATE["nut_last_roll"] = roll
            unwrapped_roll = mk.unwrapped(roll, STATE["nut_last_roll"])
            STATE["nut_last_roll"] = unwrapped_roll
            turned = unwrapped_roll - float(STATE["nut_entry_roll"])
            STATE["nut_turned_deg"] = float(math.degrees(turned))
            if turned >= math.radians(16.0):
                ref = mk.joint("nut_turn") if "nut_turn" in mk.MECH.travel else 0.0
                STATE["snug_ref"] = ref
                STATE["nut_held"] = ref
                mk.make("nut_catch")
                if "nut_turn" in mk.MECH.joint_prims:
                    mk.set_spring("nut_turn", rest=ref, most=P.TAU_RUN)
                    _carry_threaded_nut()
        else:
            STATE["nut_entry_roll"] = None
            STATE["nut_last_roll"] = None
        return

    theta = mk.joint("nut_turn")
    ref = float(STATE.get("snug_ref", 0.0))
    phi_deg = math.degrees(theta - ref)
    tau_cap = P.thread_torque_for_angle_deg(max(0.0, phi_deg), stack_clamped=wheel_seated() and flange_seated())
    max_lag = min(tau_cap / P.NUT_DRIVE_K, math.radians(30.0))
    held = float(STATE.get("nut_held", ref))
    if theta - held > max_lag:
        held = theta - max_lag
    elif held - theta > max_lag:
        held = theta + max_lag
    STATE["nut_held"] = held
    if "nut_turn" in mk.MECH.joint_prims:
        mk.set_spring("nut_turn", rest=held, most=tau_cap)


def half_now() -> bool:
    return wheel_seated() and flange_seated()


def snug_in_window() -> bool:
    return stack_clamped() and (P.ANGLE_MIN_DEG <= witness_angle_deg() <= P.ANGLE_MAX_DEG)


def full_now() -> bool:
    return snug_in_window() and wrench_set_down() and hands_clear() >= P.HANDS_CLEAR and bodies_at_rest()


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        read_hands()
    _step_nut_thread()
    STATE["peak_angle_deg"] = max(STATE.get("peak_angle_deg", 0.0), witness_angle_deg())
    STATE["peak_torque_nm"] = max(STATE.get("peak_torque_nm", 0.0), current_torque_nm())
    STATE["peak_wrench_n"] = max(STATE.get("peak_wrench_n", 0.0), mk.contact_force("wrench", "arbor_nut"))
    if STATE.get("zero") is None:
        STATE["zero"] = zero_now()
    if STATE.get("first_half") is None and STATE.get("zero") is None and half_now():
        STATE["first_half"] = mk.now() if mk.MECH.environment is not None else 0.0
    if STATE.get("zero") is None and full_now():
        if STATE.get("hold", 0) == 0:
            STATE["hold_from"] = {n: mk.pose(n)[0].copy() for n in ("wheel_sound", "outer_flange", "wrench")}
        STATE["hold"] = STATE.get("hold", 0) + 1
        drift = max(float(np.linalg.norm(mk.pose(n)[0] - STATE["hold_from"][n])) for n in STATE["hold_from"])
        if drift > P.HOLD_DRIFT:
            STATE["hold"], STATE["hold_from"] = 0, None
    else:
        STATE["hold"], STATE["hold_from"] = 0, None


def level() -> float:
    if STATE.get("zero") is not None or zero_now() is not None:
        return 0.0
    if STATE.get("hold", 0) >= P.HOLD_STEPS and full_now():
        return 1.0
    if half_now():
        return 0.5
    return 0.0


def score(objects) -> float:
    """0 for good if the cracked wheel moves > 10 mm, the 19 mm nut is turned > 195 deg past snug (> 27.1 N m), or any
    part falls off the bench. 0.5: intact wheel on 12.7 mm right arbor within 2.5 mm of inner flange and 52 mm outer
    flange seated concave-recess-inward within 2.5 mm of blotter (tilt <= 3.5 deg). 1.0: 0.5 met, nut tightened 17-195
    deg past snug (2.5-27.1 N m), spanner down >= 100 mm from the nut, <= 80 mm up, hands >= 30 mm clear, at rest (< 25 mm/s, <= 3 mm
    drift) for 1.0 s. 0 otherwise."""
    return level()


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {
        "zero": STATE.get("zero"),
        "hold steps": STATE.get("hold", 0),
        "first 0.5 at (s)": STATE.get("first_half"),
        "peak witness angle (deg)": round(STATE.get("peak_angle_deg", 0.0), 2),
        "peak thread torque (N m)": round(STATE.get("peak_torque_nm", 0.0), 2),
        "peak wrench-on-nut contact (N)": round(STATE.get("peak_wrench_n", 0.0), 2),
    }
    if mk.MECH.poses:
        out.update(
            {
                "wheel seated": wheel_seated(),
                "wheel inner gap (mm)": round(1000.0 * wheel_inner_gap(), 2),
                "wheel tilt (deg)": round(wheel_tilt_deg(), 2),
                "flange seated (recess inward)": flange_seated(),
                "flange rim gap (mm)": round(1000.0 * flange_rim_gap(), 2),
                "flange tilt from +y (deg)": round(flange_tilt_deg(), 2),
                "nut engaged": nut_engaged(),
                "witness angle past snug (deg)": round(witness_angle_deg(), 2),
                "thread torque (N m)": round(current_torque_nm(), 2),
                "blotter clamp preload (N)": round(clamp_preload_n(), 1),
                "spindle turn (deg)": round(math.degrees(mk.joint("spindle")), 2),
                "cracked wheel shift (mm)": round(1000.0 * cracked_shift_m(), 2),
                "wrench from cradle (mm)": round(1000.0 * wrench_offset_m(), 1),
                "wrench from nut (mm)": round(1000.0 * wrench_from_nut_m(), 1),
                "hands clear (mm)": round(1000.0 * hands_clear(), 1),
                "score": score(None),
            }
        )
    return out
