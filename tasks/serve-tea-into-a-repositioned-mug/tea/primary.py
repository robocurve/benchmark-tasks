"""Shared cold-water score. Worksheet JSON and simulator observations use identical units."""

import argparse
import json
import math
from pathlib import Path

DWELL_S = 2.0
POSITION_MM = 3.0
ROTATION_DEG = 3.0
UPRIGHT_DEG = 6.0
CLEARANCE_MM = 35.0
WATER_G_PER_ML = 1.000   # Declared simulated water density
SCORE_ID = "tea-moving-receiver-v4"
QUANTITIES = (
    "mug_water_g", "resident_water_g", "other_spill_g", "window_s",
    "position_span_mm", "orientation_span_deg", "max_tilt_deg",
)
BOOLEANS = (
    "measurements_valid", "fault_latched", "visible_discharge", "supported_throughout",
    "counter_supported_throughout", "released_throughout", "clear_35mm_throughout",
    "receiver_motion_completed",
)


def grade(observation, variant="moving"):
    """Missing/ambiguous measurements have no numeric score. Only the static control waives motion."""
    def result(value, reason):
        return {"score": value, "status": "invalid" if value is None else "scored", "reason": reason}

    if variant not in ("moving", "static_control"):
        raise ValueError("Unrecognized receiver condition")
    if not isinstance(observation, dict):
        return result(None, "Expected measurements object")
    for key in BOOLEANS:
        if type(observation.get(key)) is not bool:
            return result(None, f"Missing or ambiguous {key}")
    for key in QUANTITIES:
        value = observation.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            return result(None, f"Missing or invalid {key}")
    if any(observation[k] > 180 for k in ("max_tilt_deg", "orientation_span_deg")):
        return result(None, "Angle outside physical range")
    if not observation["measurements_valid"]:
        return result(None, "Measurement or examiner-stage commissioning check failed")
    if observation["counter_supported_throughout"] and not observation["supported_throughout"]:
        return result(None, "Counter support contradicts unsupported sample")
    if observation["fault_latched"]:
        return result(0.0, "Persistent fault")
    if observation["resident_water_g"] > 2.0 or observation["other_spill_g"] > 10.0:
        return result(0.0, "Collected spill exceeds allowance")
    delivered = observation["mug_water_g"]
    if not 30.0 <= delivered <= 130.0:
        return result(0.0, "Delivered mass outside 30–130 g")
    if (observation["window_s"] < DWELL_S or observation["visible_discharge"]
            or not observation["supported_throughout"]
            or observation["position_span_mm"] > POSITION_MM
            or observation["orientation_span_deg"] > ROTATION_DEG):
        return result(0.0, "Stopped, stable, supported two-second dwell incomplete")
    # At the end of a settled, otherwise gradable official pour, an unshifted
    # receiver above the examiner's upper trigger mark has no valid disturbance.
    if variant == "moving" and delivered > 55.0 and not observation["receiver_motion_completed"]:
        return result(None, "Missing required examiner stroke after trigger")
    complete = (
        delivered >= 100 and (observation["receiver_motion_completed"] or variant == "static_control")
        and observation["max_tilt_deg"] <= UPRIGHT_DEG and observation["counter_supported_throughout"]
        and observation["released_throughout"] and observation["clear_35mm_throughout"]
    )
    return result(1.0, "Completed serving") if complete else result(0.5, "Stable partial serving")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("worksheet", type=Path)
    parser.add_argument("--variant", choices=("moving", "static_control"), default="moving")
    args = parser.parse_args()
    print(json.dumps(grade(json.loads(args.worksheet.read_text()), args.variant), indent=2))
