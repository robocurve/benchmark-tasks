"""Scripted demo and ablation motion plans for pack-cold-chain-shipper-with-thermal-buffer.

Constructs open-loop ArmPlan waypoints for dual YAM arms.

Modes:
- demo: Safe packing sequence: gel brick -> thermal buffer barrier -> medication carton -> withdraw clear, hold 3.0 s.
- no_buffer_violation: Skips buffer pad, placing carton directly on sub-zero gel brick (freezing hazard, score 0.25).
- brick_only: Places only the refrigerant gel brick, leaving shipper unbuffered and uncartoned (score 0.25).
- buffer_without_carton: Places gel brick and thermal buffer pad, but does not place medication carton (score 0.50).
- settle: Both arms idle at standby (score 0.00).
- dream: Unscripted baseline policy (score 0.00).

Every part is pinched across its short side (fingers close along world X). Pick and place heights come from
geometry.py: the commanded TCP z is the pad-tip z plus 0.013 m, with the pad tips 4.5 mm above the surface the part
rests on. Every stop above a part pauses before the arm descends.
"""

from __future__ import annotations

from dataclasses import replace
import os
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN
import geometry as g

FINGER_WIDE = FINGER_OPEN
FINGER_GRIP = g.FINGER_GRIP
FINGER_RELEASE = g.FINGER_RELEASE

HOME_L = g.HOME_L
HOME_R = g.HOME_R
TRANSIT_Z = g.TRANSIT_Z


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def _pad_sync(wps_a: list[Waypoint], wps_b: list[Waypoint], label: str = "sync_wait") -> None:
    t_a = sum(w.duration for w in wps_a)
    t_b = sum(w.duration for w in wps_b)
    if t_a < t_b:
        wps_a.append(Waypoint(pos=wps_a[-1].pos, yaw=wps_a[-1].yaw, finger=wps_a[-1].finger, duration=t_b - t_a, label=label))
    elif t_b < t_a:
        wps_b.append(Waypoint(pos=wps_b[-1].pos, yaw=wps_b[-1].yaw, finger=wps_b[-1].finger, duration=t_a - t_b, label=label))


def mode_from_env() -> str:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return "settle"
    if os.environ.get("DEMO_NO_BUFFER_VIOLATION") == "1":
        return "no_buffer_violation"
    if os.environ.get("DEMO_BRICK_ONLY") == "1":
        return "brick_only"
    if os.environ.get("DEMO_BUFFER_WITHOUT_CARTON") == "1":
        return "buffer_without_carton"
    if os.environ.get("DEMO_DREAM") == "1":
        return "dream"
    return os.environ.get("TASK_MODE", "demo")


# Fingers close along world X (across the short side of every part). Yaw -pi/2 and +pi/2 pinch the same
# way; each arm uses the one that keeps its joint6 clear of its limit (left arm -pi/2, right arm +pi/2).
ARM_YAW = {"left": g.GRIP_YAW_LEFT, "right": g.GRIP_YAW_RIGHT}
YAW = g.GRIP_YAW_RIGHT  # placeholder while building; _plans() sets each arm's own yaw
PART_NAMES = ("brick", "buffer", "carton")


def part_of(label: str) -> str | None:
    """The part a waypoint label works on (labels end in _brick / _buffer / _carton), else None."""
    tail = label.rsplit("_", 1)[-1]
    return tail if tail in PART_NAMES else None


def layout(variant: str) -> dict:
    """Which arm moves which part, and where each part starts (world XY on its staging pad)."""
    if variant.upper() == "B":
        return {
            "arms": {"brick": "left", "buffer": "right", "carton": "right"},
            "staging": {
                "brick": (g.PAD_LEFT_X, g.PAD_LEFT_Y),
                "buffer": (g.PAD_FRONT_X, g.PAD_FRONT_Y),
                "carton": (g.PAD_RIGHT_X, g.PAD_RIGHT_Y),
            },
        }
    return {
        "arms": {"brick": "right", "buffer": "left", "carton": "left"},
        "staging": {
            "brick": (g.PAD_RIGHT_X, g.PAD_RIGHT_Y),
            "buffer": (g.PAD_FRONT_X, g.PAD_FRONT_Y),
            "carton": (g.PAD_LEFT_X, g.PAD_LEFT_Y),
        },
    }


def _wp(pos, finger, duration, label) -> Waypoint:
    return Waypoint(pos=tuple(pos), yaw=YAW, finger=finger, duration=duration, label=label)


def _dwell(wps: list[Waypoint], seconds: float, label: str) -> None:
    """Hold the last pose for ``seconds`` so the soft arm catches up with the commanded point."""
    last = wps[-1]
    wps.append(Waypoint(pos=last.pos, yaw=last.yaw, finger=last.finger, duration=seconds, label=label))


def _transfer(wps: list[Waypoint], name: str, pick_xy, place_xy, place_z: float, home) -> None:
    """Pinch ``name`` across its short side on its staging pad, carry it, seat it, release it, go home.

    Every stop above a part pauses before the arm descends; pad tips stay 4.5 mm above the surface.
    """
    px, py = pick_xy
    qx, qy = place_xy
    pick_z = g.PICK_TCP_Z
    wps.append(_wp((px, py, TRANSIT_Z), FINGER_WIDE, 0.8, f"hover_pick_{name}"))
    _dwell(wps, 0.6, f"pause_hover_pick_{name}")
    wps.append(_wp((px, py, pick_z), FINGER_WIDE, 0.8, f"descend_{name}"))
    _dwell(wps, 0.4, f"pause_descend_{name}")
    wps.append(_wp((px, py, pick_z), FINGER_GRIP, 0.8, f"grasp_{name}"))
    wps.append(_wp((px, py, TRANSIT_Z), FINGER_GRIP, 0.8, f"lift_{name}"))
    wps.append(_wp((qx, qy, TRANSIT_Z), FINGER_GRIP, 1.0, f"approach_shipper_{name}"))
    _dwell(wps, 0.6, f"pause_above_shipper_{name}")
    wps.append(_wp((qx, qy, place_z), FINGER_GRIP, 0.8, f"place_{name}"))
    _dwell(wps, 0.3, f"pause_place_{name}")
    wps.append(_wp((qx, qy, place_z), FINGER_RELEASE, 0.6, f"release_{name}"))
    wps.append(_wp((qx, qy, TRANSIT_Z), FINGER_RELEASE, 0.8, f"lift_clear_{name}"))
    wps.append(_wp(home, FINGER_WIDE, 0.8, f"return_home_{name}"))


def _plans(mode: str | None = None, variant: str | None = None) -> dict[str, list[Waypoint]]:
    raw = _build(mode, variant)
    return {arm: [replace(w, yaw=ARM_YAW[arm]) for w in wps] for arm, wps in raw.items()}


def _build(mode: str | None = None, variant: str | None = None) -> dict[str, list[Waypoint]]:
    if mode is None:
        mode = mode_from_env()

    if variant is None:
        variant = os.environ.get("TASK_VARIANT", "A").upper()
    else:
        variant = variant.upper()
    if variant not in ("A", "B"):
        variant = "A"

    left_wps: list[Waypoint] = [_wp(HOME_L, FINGER_WIDE, 1.0, "home_ready")]
    right_wps: list[Waypoint] = [_wp(HOME_R, FINGER_WIDE, 1.0, "home_ready")]

    if mode in ("settle", "dream"):
        left_wps.append(_wp(HOME_L, FINGER_WIDE, 12.0, "settle_home"))
        right_wps.append(_wp(HOME_R, FINGER_WIDE, 12.0, "settle_home"))
        return {"left": left_wps, "right": right_wps}

    # Staging layout:
    # In Variant A:
    # - Gel brick staged on Right pad (x=0.220, y=-0.220) -> Right arm picks brick
    # - Buffer pad staged on Front pad (x=0.115, y=0.000)  -> Left arm picks buffer
    # - Carton staged on Left pad (x=0.220, y=+0.220)     -> Left arm picks carton
    # In Variant B:
    # - Gel brick staged on Left pad (x=0.220, y=+0.220)  -> Left arm picks brick
    # - Buffer pad staged on Front pad (x=0.115, y=0.000) -> Right arm picks buffer
    # - Carton staged on Right pad (x=0.220, y=-0.220)   -> Right arm picks carton

    lay = layout(variant)
    brick_arm, buffer_arm, carton_arm = (lay["arms"][n] for n in PART_NAMES)
    brick_staging_xy = lay["staging"]["brick"]
    carton_staging_xy = lay["staging"]["carton"]

    front_staging_xy = (g.PAD_FRONT_X, g.PAD_FRONT_Y)
    shipper_xy = (g.SHIPPER_X, g.SHIPPER_Y)

    def arm_wps(arm: str) -> list[Waypoint]:
        return left_wps if arm == "left" else right_wps

    def arm_home(arm: str):
        return HOME_L if arm == "left" else HOME_R

    def hold_and_finish() -> dict[str, list[Waypoint]]:
        _pad_sync(left_wps, right_wps)
        left_wps.append(_wp(HOME_L, FINGER_WIDE, 5.0, "hold_settle"))
        right_wps.append(_wp(HOME_R, FINGER_WIDE, 5.0, "hold_settle"))
        return {"left": left_wps, "right": right_wps}

    # Phase 1: refrigerant gel brick (Layer 1)
    _transfer(arm_wps(brick_arm), "brick", brick_staging_xy, shipper_xy, g.BRICK_PLACE_TCP_Z, arm_home(brick_arm))
    if mode == "brick_only":
        return hold_and_finish()

    # Synchronize before second arm acts
    _pad_sync(left_wps, right_wps, label="wait_for_brick_arm")

    # Phase 2: thermal buffer barrier pad (Layer 2), skipped in no_buffer_violation
    if mode != "no_buffer_violation":
        _transfer(arm_wps(buffer_arm), "buffer", front_staging_xy, shipper_xy, g.BUFFER_PLACE_TCP_Z, arm_home(buffer_arm))
        if mode == "buffer_without_carton":
            return hold_and_finish()

    # Phase 3: medication carton (Layer 3). In no_buffer_violation it is set straight onto the brick.
    carton_z = g.CARTON_DIRECT_PLACE_TCP_Z if mode == "no_buffer_violation" else g.CARTON_PLACE_TCP_Z
    _transfer(arm_wps(carton_arm), "carton", carton_staging_xy, shipper_xy, carton_z, arm_home(carton_arm))

    # Phase 4: synchronized standby hold
    return hold_and_finish()


def plans(mode: str | None = None, variant: str | None = None) -> dict[str, list[Waypoint]]:
    """The plan for ``mode`` with every waypoint duration stretched by DEMO_TIME_SCALE (default 1.8) so the arms keep up."""
    if mode is None:
        mode = mode_from_env()
    result = _plans(mode, variant=variant)
    if mode in ("settle", "dream"):
        return result
    scale = _f("DEMO_TIME_SCALE", 1.8)
    return {arm: [replace(w, duration=w.duration * scale) for w in wps] for arm, wps in result.items()}
