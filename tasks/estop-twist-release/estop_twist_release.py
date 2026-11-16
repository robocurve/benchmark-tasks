"""Machine safety: emergency-stop function test on a benchtop grinder-polisher's control station.

On the bench: a running single-platen metallographic grinder-polisher (200 mm platen turning at 60 rpm, green RUN
lamp lit) wired to two fixed control stations: a yellow 1-hole emergency-stop station with a 16 mm-panel, 40 mm red
mushroom push-to-lock / turn-to-reset operator, and a grey 2-hole station with a green START push button and the green
RUN pilot lamp. The simplified simulated function test (not an ISO/IEC compliance claim): press the
mushroom until it latches -> the machine stops and the RUN lamp goes out; twist the head clockwise until it unlatches
and springs back out -> the machine must stay stopped; only then press START -> the machine restarts.

Mechanics (labkit joints, real contact; values of a cited 16 mm-panel XA-class operator, see README):
- head_press: slide, 4.5 mm maximum operator stroke, return spring 2625 N/m (10.5 N at the 4.0 mm latch point).
- head_twist (rides on head_press): hinge about the vertical, torsion spring 0.16 N*m at 30 deg, stop at 40 deg.
- Latch = the detent at the pressed position: once the head reaches the 4.0 mm latch point its spring holds it at
  4.2 mm (the hook moves the spring's rest point, as the latch pawl does); pulling the head does not unlatch it. The
  release cam unlatches only past RELEASE_DEG of clockwise twist, then the spring returns the head to its rest.
- reset: momentary slide, 4 mm stroke, spring return; it re-arms the machine only while the e-stop is released.
- The machine's contactor is self-holding: it drops out when the e-stop NC contact opens and pulls in only on a START
  press with the e-stop released. Lamp and platen are physical target-driven outputs observed by the scorer.
  Twist-release only resets the e-stop device; a separate fresh START press is required for motion.
  This simplified functional model is not a safety-rated electrical implementation.
Scoring observes the measured output trajectory as well as contact history and end state (see score()).
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import numpy as np
from yam_arena import demo as _demo, ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN
from yam_arena.task_file import TABLE_TOP

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import arena_assets_fix  # noqa: E402,F401
import labkit  # noqa: E402
from labkit import Joint, PartState, on_reset, on_step, part  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
# The episode ends once the full score has held this long: the arm is parked and the restarted machine stays in view.
SUCCESS_HOLD_S = 2.0

# -- layout (metres, station frame: origin mid-bases, +x forward, +y left, +z up) -----------------------------------
ESTOP_BOX_POS = (0.300, -0.110, TOP + 0.035)  # yellow 1-hole station, 80 x 80 x 70 mm, centre
STATION_POS = (0.300, 0.000, TOP + 0.035)  # grey 2-hole station, 80 x 120 x 70 mm, centre
MACHINE_POS = (0.520, 0.120, TOP + 0.060)  # grinder-polisher base centre (330 x 290 x 120 mm)
PANEL_Z = 0.035  # station cover (panel) height in the station part frames: top of the 70 mm box
RESET_LOCAL = (0.0, -0.028, 0.0)
LAMP_LOCAL = (0.0, 0.030, 0.0)
PLATEN_LOCAL = (0.0, 0.0, 0.072)  # platen hub on the machine (top of the drive boss)

ESTOP_XY = (ESTOP_BOX_POS[0], ESTOP_BOX_POS[1])
RESET_XY = (STATION_POS[0] + RESET_LOCAL[0], STATION_POS[1] + RESET_LOCAL[1])

# -- operator data (cited 16 mm-panel, 40 mm mushroom push-to-lock / turn-to-reset e-stop; README) ------------------------
HEAD_R = 0.020  # 40 mm mushroom
HEAD_H = 0.014  # mushroom cap thickness
HEAD_TOP_ABOVE_PANEL = 0.0304  # catalogue drawing (40 mm button)
STROKE_MAX_M = 0.0045  # maximum operator stroke
LATCH_AT_M = 0.0040  # minimum stroke for direct opening action: NC contact open + latched
LATCH_HOLD_M = 0.0042  # where the latch holds the head
PRESS_SPRING = 2625.0  # N/m: 10.5 N push-to-lock force at the 4.0 mm latch point
RELEASE_DEG = 30.0  # clockwise turn that lifts the latch (declared value; catalogue gives only the torque)
TWIST_STOP_DEG = 40.0
TWIST_SPRING = 0.16 / math.radians(RELEASE_DEG)  # N*m/rad: 0.16 N*m turn-to-reset torque at the release angle
HEAD_OUT_TOL_M = 0.0005  # head counts as back out within 0.5 mm of its rest
# Legacy RESET_* constants and "reset" joint/state keys identify the dedicated START button.
# Kept for asset/test compatibility; there is no separate blue safety-reset button.
RESET_STROKE_M = 0.004
RESET_MAKE_M = 0.0025  # START contact closes at 2.5 mm
RESET_SPRING = 1250.0  # N/m: 5 N at full stroke
RESET_CAP_R = 0.012
RESET_TOP_ABOVE_PANEL = 0.014
LAMP_OFF_DROP_M = 0.010  # display only: the lit lens sinks into the bezel when the lamp is off
PLATEN_RPM = 60.0
PLATEN_STOP_RAD_S = 0.20  # observed stop tolerance, not commanded speed
PLATEN_RUN_RAD_S = 0.50  # observed motion required for running credit
LAMP_STATE_TOL_M = 0.002  # observed lens position tolerance
PLATEN_WRAP_RAD = math.pi / 2  # the platen's clamp ring has 4 lugs: its look repeats every 90 deg

# -- scoring (single source of truth for score(), success(), the hook and the SUBACTION_END log) --------------------
W_LATCH_STOP = 0.35  # e-stop latched (NC open) while the machine ran -> machine stopped, lamp out
W_RELEASED = 0.20  # released by the twist: latch lifted, head back out at the end
W_NO_RESTART = 0.10  # head out and the machine still stopped for >= NO_RESTART_HOLD_S before START
W_RESET = 0.25  # START pressed only after the release, machine running at the end
W_ARMS_CLEAR = 0.10  # every gripper centre point >= HAND_CLEAR_M from both buttons at the end, test complete
NO_RESTART_HOLD_S = 1.0
HAND_CLEAR_M = 0.100
SCORE_TIERS = (0.0, 0.35, 0.55, 0.65, 0.8, 0.9, 1.0)

# -- colours (stand-in looks for the collider shapes; the visuals come from assets/estop_visuals.usda) ---------------
YELLOW = (0.96, 0.78, 0.10)
RED = (0.78, 0.06, 0.05)
GREY = (0.62, 0.64, 0.66)
BLUE = (0.10, 0.32, 0.75)
GREEN = (0.20, 0.85, 0.30)
STEEL = (0.70, 0.71, 0.72)
DARK = (0.12, 0.12, 0.13)

VISUALS = str(pathlib.Path(__file__).resolve().parent / "assets" / "estop_visuals.usdc")
_VIS = VISUALS if pathlib.Path(VISUALS).is_file() else None

HEAD_PRESS = Joint(
    "head_press",
    kind="slide",
    axis="Z",
    anchor=(0.0, 0.0, PANEL_Z),
    lower=-STROKE_MAX_M,
    upper=0.0,
    initial=0.0,
    target=0.0,
    stiffness=PRESS_SPRING,
    damping=40.0,
    mass=0.012,
    # plunger + collar under the mushroom (operator neck)
    cylinders=[((0.0, 0.0, PANEL_Z + 0.009), 0.011, 0.018, RED, "Z")],
)

HEAD_TWIST = Joint(
    "head_twist",
    kind="hinge",
    axis="Z",
    anchor=(0.0, 0.0, PANEL_Z),
    lower=-math.radians(TWIST_STOP_DEG),
    upper=0.0,
    initial=0.0,
    target=0.0,
    stiffness=TWIST_SPRING,
    damping=0.02,
    mass=0.016,
    parent="head_press",
    # the 40 mm mushroom cap, 14 mm thick, top 30.4 mm above the panel
    cylinders=[((0.0, 0.0, PANEL_Z + HEAD_TOP_ABOVE_PANEL - HEAD_H / 2), HEAD_R, HEAD_H, RED, "Z")],
)

ESTOP = part(
    "estop_station",
    pos=ESTOP_BOX_POS,
    fixed=True,
    friction=0.9,
    boxes=[
        ((0.0, 0.0, 0.0), (0.080, 0.080, 0.070), YELLOW),  # 1-hole enclosure
        ((0.0, 0.0, PANEL_Z + 0.001), (0.060, 0.060, 0.002), YELLOW),  # 60 mm EMERGENCY STOP legend plate
    ],
    joints=[HEAD_PRESS, HEAD_TWIST],
    visual_usd=_VIS,
)

RESET_BTN = Joint(
    "reset",
    kind="slide",
    axis="Z",
    anchor=(RESET_LOCAL[0], RESET_LOCAL[1], PANEL_Z),
    lower=-RESET_STROKE_M,
    upper=0.0,
    initial=0.0,
    target=0.0,
    stiffness=RESET_SPRING,
    damping=8.0,
    mass=0.010,
    cylinders=[((RESET_LOCAL[0], RESET_LOCAL[1], PANEL_Z + RESET_TOP_ABOVE_PANEL / 2), RESET_CAP_R,
                RESET_TOP_ABOVE_PANEL, GREEN, "Z")],
)

LAMP = Joint(
    "lamp",
    kind="slide",
    axis="Z",
    anchor=(LAMP_LOCAL[0], LAMP_LOCAL[1], PANEL_Z),
    lower=-LAMP_OFF_DROP_M,
    upper=0.0,
    initial=0.0,
    target=0.0,
    stiffness=200.0,
    damping=4.0,
    mass=0.005,
    cylinders=[((LAMP_LOCAL[0], LAMP_LOCAL[1], PANEL_Z - 0.004), 0.010, 0.008, GREEN, "Z")],
)

STATION = part(
    "control_station",
    pos=STATION_POS,
    fixed=True,
    friction=0.9,
    boxes=[((0.0, 0.0, 0.0), (0.080, 0.120, 0.070), GREY)],
    cylinders=[
        ((RESET_LOCAL[0], RESET_LOCAL[1], PANEL_Z + 0.001), 0.0145, 0.002, STEEL, "Z"),  # reset bezel ring
        ((LAMP_LOCAL[0], LAMP_LOCAL[1], PANEL_Z + 0.004), 0.0145, 0.008, STEEL, "Z"),  # lamp bezel
    ],
    joints=[RESET_BTN, LAMP],
    visual_usd=_VIS,
)

PLATEN = Joint(
    "platen",
    kind="hinge",
    axis="Z",
    anchor=PLATEN_LOCAL,
    lower=-0.05,
    upper=2 * math.pi * PLATEN_RPM / 60.0 * 30.0 + 0.05,  # full 30 s trial, no visual-angle wrap
    initial=0.0,
    target=0.0,
    stiffness=50.0,
    damping=2.0,
    mass=0.8,
    cylinders=[((PLATEN_LOCAL[0], PLATEN_LOCAL[1], PLATEN_LOCAL[2] + 0.006), 0.100, 0.012, DARK, "Z")],
)

MACHINE = part(
    "grinder_polisher",
    pos=MACHINE_POS,
    fixed=True,
    boxes=[((0.0, 0.0, 0.0), (0.330, 0.290, 0.120), GREY)],
    cylinders=[((PLATEN_LOCAL[0], PLATEN_LOCAL[1], 0.066), 0.030, 0.012, STEEL, "Z")],  # drive boss
    joints=[PLATEN],
    visual_usd=_VIS,
)

OBJECTS = [ESTOP, STATION, MACHINE]
PARTS = OBJECTS


# -- per-trial state, updated by the observe hook from the joint states (never read by demo()) ----------------------
def _fresh() -> dict:
    return dict(running=True, latched=False, latched_once=False, released_by_twist=False, release_step=None,
                stopped_out_steps=0, no_restart_ok=False, reset_after_release=False, reset_while_latched=False,
                outputs_valid=False, output_running=False, output_stopped=False, seen_running=False,
                stop_verified=False, last_platen=None, measured_platen_rate=None,
                reset_down=False, steps=0, held=0, tcps=None, platen=0.0, dt=0.05)


_trial: dict = _fresh()
_ids: dict = {}


def _joint(st: PartState, name: str) -> float:
    return float(st.joints.get(name, 0.0))


def head_out(estop: PartState) -> bool:
    return _joint(estop, "head_press") >= -HEAD_OUT_TOL_M


def _arm_tcps(env):
    try:
        base = env.unwrapped
        out = []
        for side in ("left", "right"):
            asset = base.scene[f"{side}_robot"]
            key = f"arm_{side}"
            if key not in _ids:
                _ids[key] = asset.find_joints(ARM_JOINTS, preserve_order=True)[0]
            q = labkit._np(asset.data.joint_pos)[0, _ids[key]]
            out.append(ik.tcp_pose(q, station.BASES[side])[0])
        return (out[0], out[1])
    except Exception:
        return None


def _button_clearance(tcps) -> float:
    """Smallest distance (m) from either gripper centre point to the e-stop head or the START button; 0 without TCPs."""
    if tcps is None or len(tcps) < 2:
        return 0.0
    pts = [np.array([*ESTOP_XY, TOP + 0.035 + PANEL_Z + HEAD_TOP_ABOVE_PANEL]),
           np.array([*RESET_XY, TOP + 0.035 + PANEL_Z + RESET_TOP_ABOVE_PANEL])]
    return min(float(np.linalg.norm(np.asarray(t, float) - p)) for t in tcps for p in pts)


def _observe_outputs(station_st: PartState, machine_st: PartState, dt: float) -> None:
    """Observe measured joints before changing targets. A commanded stop is not stop evidence.

    The platen joint is continuous over the trial, so use its unwrapped displacement.
    Visual fourfold symmetry must not alias fast rotation into a measured stop.
    No observation here writes joint state, fabricates motion or changes the mechanism.
    """
    t = _trial
    t["outputs_valid"] = t["output_running"] = t["output_stopped"] = False
    try:
        angle = float(machine_st.joints["platen"])
        lamp = float(station_st.joints["lamp"])
        if not all(math.isfinite(v) for v in (angle, lamp, dt)) or not (0 < dt < 0.1):
            raise ValueError("missing or invalid machine observation")
        previous = t["last_platen"]
        t["last_platen"] = angle
        if previous is None:
            return
        delta = angle - previous
        rate = abs(delta / dt)
        t["measured_platen_rate"] = rate
        t["outputs_valid"] = True
        t["output_running"] = lamp >= -LAMP_STATE_TOL_M and rate >= PLATEN_RUN_RAD_S
        t["output_stopped"] = lamp <= -LAMP_OFF_DROP_M + LAMP_STATE_TOL_M and rate <= PLATEN_STOP_RAD_S
        t["seen_running"] = t["seen_running"] or t["output_running"]
    except (AttributeError, KeyError, TypeError, ValueError):
        t["last_platen"] = None
        t["measured_platen_rate"] = None


def update_machine(estop: PartState, station_st: PartState, dt: float) -> None:
    """One control step of the e-stop latch, the release cam and the self-holding contactor, from the joint states."""
    t = _trial
    press, twist = _joint(estop, "head_press"), _joint(estop, "head_twist")
    reset_down = _joint(station_st, "reset") <= -RESET_MAKE_M
    home = twist > -math.radians(RELEASE_DEG) / 2  # the pawl can only catch with the head turned back home
    if not t["latched"] and press <= -LATCH_AT_M and home:  # latch point reached: NC opens, pawl catches the plunger
        t["latched"] = True
        if t["running"] and t["seen_running"]:
            t["latched_once"] = True
        t["stop_verified"] = False
        t["running"] = False
        t["released_by_twist"], t["release_step"], t["no_restart_ok"], t["stopped_out_steps"] = False, None, False, 0
        t["reset_after_release"] = False
    elif t["latched"] and twist <= -math.radians(RELEASE_DEG):  # release cam lifts the pawl
        t["latched"] = False
        t["released_by_twist"], t["release_step"] = True, t["steps"]
    if reset_down and not t["reset_down"]:  # a START press (rising edge of its contact)
        if t["latched"]:
            t["reset_while_latched"] = True  # the interlock ignores it; the test order was broken
        elif not t["running"] and t["released_by_twist"] and head_out(estop):
            t["running"] = True
            t["reset_after_release"] = True
    t["reset_down"] = reset_down
    if t["latched_once"] and t["latched"] and t["output_stopped"]:
        t["stop_verified"] = True
    if t["released_by_twist"] and not t["running"] and not t["latched"] and press >= -HEAD_OUT_TOL_M and t["output_stopped"]:
        t["stopped_out_steps"] += 1
        if t["stopped_out_steps"] * dt >= NO_RESTART_HOLD_S - 1e-9:
            t["no_restart_ok"] = True
    elif not t["no_restart_ok"]:
        t["stopped_out_steps"] = 0  # the hold must be continuous


def _components(objects: dict) -> dict:
    estop = objects["estop_station"]
    station_st = objects["control_station"]
    machine_st = objects["grinder_polisher"]
    for observed, names in ((estop, ("head_press", "head_twist")),
                            (station_st, ("reset", "lamp")), (machine_st, ("platen",))):
        if not all(math.isfinite(float(observed.joints[name])) for name in names):
            raise ValueError("non-finite machine observation")
    t = _trial
    if not t["outputs_valid"]:
        return dict(score=0.0, latch=False, released=False, no_restart=False, reset=False, clear=False)
    latch = bool(t["latched_once"] and t["stop_verified"])
    released = latch and t["released_by_twist"] and not t["latched"] and head_out(estop)
    no_restart = released and t["no_restart_ok"]
    reset = released and t["running"] and t["reset_after_release"] and not t["reset_while_latched"] and t["output_running"]
    clear = latch and released and no_restart and reset and _button_clearance(t["tcps"]) >= HAND_CLEAR_M
    total = (W_LATCH_STOP * latch + W_RELEASED * released + W_NO_RESTART * no_restart + W_RESET * reset
             + W_ARMS_CLEAR * clear)
    return dict(score=round(float(total), 6), latch=latch, released=released, no_restart=no_restart, reset=reset,
                clear=clear)


def score(objects: dict) -> float:
    """Partial credit in SCORE_TIERS; see task.json scoring."""
    try:
        return _components(objects)["score"]
    except (AttributeError, KeyError, TypeError, ValueError):
        return 0.0


def success(objects: dict) -> bool:
    return score(objects) >= 1.0 - 1e-6


def distance(objects: dict) -> float:
    return float(1.0 - score(objects))


# -- hooks -----------------------------------------------------------------------------------------------------------
def _set_targets(env, asset_name: str, values: dict):
    asset = env.unwrapped.scene[asset_name]
    try:
        import torch

        names = list(asset.joint_names)
        ids = [names.index(f"{k}_joint") for k in values]
        q = torch.tensor([list(values.values())], device=asset.device, dtype=torch.float32)
    except Exception:
        return
    asset.set_joint_position_target(q, joint_ids=ids)
    return asset, ids, q


@on_reset
def _reset(env) -> None:
    _trial.clear()
    _trial.update(_fresh())
    _ids.clear()


@on_step
def _watch(env) -> None:
    estop = labkit._part_state(env, ESTOP)
    st = labkit._part_state(env, STATION)
    dt = float(getattr(env.unwrapped, "step_dt", 0.05))
    _trial["steps"] += 1
    _trial["dt"] = dt
    machine = labkit._part_state(env, MACHINE)
    _observe_outputs(st, machine, dt)
    update_machine(estop, st, dt)
    _trial["tcps"] = _arm_tcps(env)
    # Latch pawl: the head's spring rest point is the latched depth while latched, its free rest otherwise.
    _set_targets(env, "estop_station", {"head_press": -LATCH_HOLD_M if _trial["latched"] else 0.0})
    _set_targets(env, "control_station", {"lamp": 0.0 if _trial["running"] else -LAMP_OFF_DROP_M})
    if _trial["running"]:  # display: the platen turns at PLATEN_RPM (its look repeats every 90 deg)
        _trial["platen"] = _trial["platen"] + 2 * math.pi * PLATEN_RPM / 60.0 * dt
        _set_targets(env, "grinder_polisher", {"platen": _trial["platen"]})
    # A motor target drives the platen through physics; never teleport its joint position.
    objs = {"estop_station": estop, "control_station": st, "grinder_polisher": machine}
    full = _components(objs)
    if _trial["steps"] % 10 == 0:  # observe-only state trace (mm, deg, contactor flags)
        t = _trial
        print(f"ESTOP_TRACE step={t['steps']} press_mm={_joint(estop, 'head_press') * 1e3:.2f} "
              f"twist_deg={math.degrees(_joint(estop, 'head_twist')):.1f} start_mm={_joint(st, 'reset') * 1e3:.2f} "
              f"latched={int(t['latched'])} running={int(t['running'])} released={int(t['released_by_twist'])} "
              f"score={full['score']} observed_rate_rad_s={t['measured_platen_rate']} "
              f"observed_running={int(t['output_running'])} observed_stopped={int(t['output_stopped'])}", flush=True)
    _trial["held"] = _trial["held"] + 1 if full["score"] >= 1.0 - 1e-6 else 0
    if _trial["held"]:
        _log_subaction_end(estop, st, full, _trial["held"] >= max(1, round(SUCCESS_HOLD_S / dt)))


SUBACTIONS = (
    ("press the e-stop head to its latch: machine stopped, RUN lamp out", "estop_station.head_press"),
    ("twist the head clockwise to release it: head back out", "estop_station.head_twist"),
    ("confirm the machine stays stopped after the release", "machine.no_restart_s"),
    ("press START after the release: machine running again", "control_station.reset"),
    ("arm clear of both buttons", "arms.min_tcp_button_distance"),
)


def _log_subaction_end(estop: PartState, st: PartState, comp: dict, run_end: bool) -> None:
    t = _trial
    press_mm = _joint(estop, "head_press") * 1e3
    twist_deg = math.degrees(_joint(estop, "head_twist"))
    clear = _button_clearance(t["tcps"])
    dt_rel = t["stopped_out_steps"] * t["dt"]
    ends = (int(t["latched_once"]), round(press_mm, 3), round(dt_rel, 2), int(t["running"]), round(clear, 4))
    for (name, key), end in zip(SUBACTIONS, ends):
        rec = {"name": name, "joint_or_body": key, "end_value": end, "step": t["steps"], "run_end": run_end}
        print("SUBACTION_END " + json.dumps(rec), flush=True)
    if run_end:
        print(f"SUBACTION_END latched_once={t['latched_once']} released_by_twist={t['released_by_twist']} "
              f"head_press_mm={press_mm:.3f} head_twist_deg={twist_deg:.2f} stopped_after_release_s={dt_rel:.2f} "
              f"reset_after_release={t['reset_after_release']} reset_while_latched={t['reset_while_latched']} "
              f"running={t['running']} button_clear_mm={clear * 1e3:.1f} score={comp['score']:.2f} "
              f"step={t['steps']} run_end=True", flush=True)


# -- scripted demo: open loop, from the fixed station layout only ----------------------------------------------------
def demo() -> dict:
    """Right arm: press the mushroom to its latch, grip and twist it clockwise to release, let it spring out, wait
    with the machine stopped, press START, park. The left arm stays at READY."""
    W = _demo.Waypoint
    ex, ey = ESTOP_XY
    rx, ry = RESET_XY
    panel = TOP + 0.035 + PANEL_Z
    head_top = panel + HEAD_TOP_ABOVE_PANEL
    tip = 0.013  # pad tips below the TCP (PAD_TIP_PAST_TCP)
    press_z = head_top + tip - STROKE_MAX_M - 0.040  # command past the 4.5 mm stop: the arm's compliance presses
    cap_mid_latched = head_top - HEAD_H / 2 - LATCH_HOLD_M
    grip_z = cap_mid_latched + 0.006
    reset_top = panel + RESET_TOP_ABOVE_PANEL
    reset_press_z = reset_top + tip - RESET_STROKE_M - 0.040
    hover = panel + 0.085
    twist = -math.radians(TWIST_STOP_DEG + 6.0)
    park = (0.150, -0.330, TOP + 0.200)
    right = [
        W(1.20, (ex, ey, hover), FINGER_CLOSED, "over the running machine's e-stop"),
        W(0.70, (ex, ey, head_top + tip + 0.002), FINGER_CLOSED, "onto the mushroom"),
        W(0.60, (ex, ey, press_z), FINGER_CLOSED, "press through the latch point"),
        W(0.60, (ex, ey, press_z), FINGER_CLOSED, "hold: machine stops, lamp out"),
        W(0.50, (ex, ey, hover), FINGER_CLOSED, "lift off: head stays latched"),
        W(0.50, (ex, ey, hover), FINGER_OPEN, "open"),
        W(0.70, (ex, ey, grip_z), FINGER_OPEN, "fingers around the latched cap"),
        W(0.70, (ex, ey, grip_z), FINGER_CLOSED, "grip the cap"),
        W(0.90, (ex, ey, grip_z), FINGER_CLOSED, "twist clockwise past the release angle", twist),
        W(0.60, (ex, ey, grip_z), FINGER_OPEN, "let go: head springs out", twist),
        W(0.60, (ex, ey, hover), FINGER_OPEN, "lift clear, machine stays stopped", twist),
        W(1.10, (rx, ry, hover), FINGER_CLOSED, "over START (machine still stopped)"),
        W(0.60, (rx, ry, reset_top + tip + 0.002), FINGER_CLOSED, "onto START"),
        W(0.40, (rx, ry, reset_press_z), FINGER_CLOSED, "press START"),
        W(0.40, (rx, ry, reset_top + tip + 0.010), FINGER_CLOSED, "let START spring back"),
        W(0.60, (rx, ry, hover), FINGER_CLOSED, "lift"),
        W(1.20, park, FINGER_OPEN, "park clear of the stations"),
    ]
    return {"right": right}
