"""Reduce a cold-water trial worksheet to the same outcome used by the simulation."""

import argparse
import json
from math import isfinite
from pathlib import Path

from .primary import grade


class Unscorable(ValueError):
    """An examiner must resolve the observation or repeat the trial."""


def number(value, label):
    if type(value) not in (float, int) or not isfinite(value) or value < 0:
        raise Unscorable(f"Missing or invalid {label}")
    return float(value)


def flag(record, key):
    value = record[key]
    if type(value) is not bool:
        raise Unscorable(f"Missing video or equipment observation: {key}")
    return value


def interval(value, error, minimum=None, maximum=None, label="measurement"):
    """Classify an inclusive interval. Boundary-straddling readings are ambiguous."""
    if minimum is not None:
        if value + error < minimum:
            return False
        if value - error < minimum:
            raise Unscorable(f"Uncertainty straddles the {label} minimum ({minimum})")
    if maximum is not None:
        if value - error > maximum:
            return False
        if value + error > maximum:
            raise Unscorable(f"Uncertainty straddles the {label} maximum ({maximum})")
    return True


def evaluate(sheet):
    for k in ("video_ref", "calibration_ref"):
        if not isinstance(sheet[k], str) or not sheet[k].strip():
            raise Unscorable(f"Missing {k}")
    if not flag(sheet, "equipment_ok"):
        raise Unscorable("Containment, camera, collection or fixture check failed")
    calibration = sheet["calibration"]
    reading_error = number(calibration["scale_reading_error_g"], "scale reading error")
    if reading_error <= 0:
        raise Unscorable("Enter the calibrated uncertainty of a scale reading")
    residual_limit = number(calibration["balance_limit_g"], "mass balance tolerance")
    if not 0 < residual_limit <= 0.5:
        raise Unscorable("Record a commissioned mass balance tolerance up to 0.5 g")
    masses, errors = {}, {}
    for channel in ("pitcher", "mug", "resident", "other"):
        raw = sheet["weighings"][channel]
        before = number(raw["before_g"], f"{channel} before")
        after = number(raw["after_g"], f"{channel} after")
        recovery = number(raw["collection_error_g"], f"{channel} collection uncertainty")
        net = (before - after) if channel == "pitcher" else (after - before)
        errors[channel] = 2 * reading_error + recovery
        if channel == "resident" and errors[channel] > 0.2 + 1e-9:
            raise Unscorable("Resident mass uncertainty exceeds 0.2 g")
        if net < -errors[channel]:
            raise Unscorable(f"Unexplained negative transfer for {channel}")
        masses[channel] = max(0.0, net)
    residual = masses["pitcher"] - masses["mug"] - masses["resident"] - masses["other"]
    if abs(residual) > residual_limit + 1e-9:
        raise Unscorable(f"Unaccounted transfer {residual:.2f} g exceeds commissioned recovery tolerance")
    # Unlocated water could belong to any collector. Expand each interval in the
    # direction of the residual before classifying a threshold.
    def bucket(name, lo=None, hi=None):
        low = masses[name] - errors[name] + min(0.0, residual)
        high = masses[name] + errors[name] + max(0.0, residual)
        return interval((low + high) / 2, (high - low) / 2, lo, hi, name)

    bucket("resident", hi=2)
    bucket("other", hi=10)
    bucket("mug", 30, 130)
    bucket("mug", 100)

    stage = sheet["stage"]
    movement_due = flag(stage, "trigger_level_reached")
    obstructed = flag(stage, "robot_interference_observed")
    if not movement_due and masses["mug"] - errors["mug"] > 55:
        raise Unscorable("Delivered mass contradicts an unreached trigger")
    moved = False
    if movement_due and not obstructed:
        for key, low, high in (("onset_g", 45, 55), ("travel_mm", 81, 85), ("duration_s", 0.85, 1.15)):
            measured = number(stage[key], key)
            error = number(stage[key + "_error"], key + " uncertainty")
            if not interval(measured, error, low, high, key):
                raise Unscorable(f"Examiner stage {key} outside [{low}, {high}]")
        moved = True
    video = sheet["video"]
    obs = dict(measurements_valid=True, mug_water_g=masses["mug"],
               resident_water_g=masses["resident"], other_spill_g=masses["other"],
               receiver_motion_completed=moved)
    trip = flag(video, "witness_latched")
    drop = flag(video, "drop_observed")
    obs["fault_latched"] = trip or drop or obstructed
    for key in ("visible_discharge", "supported_throughout", "counter_supported_throughout",
                "released_throughout"):
        obs[key] = flag(video, key)
    bounds = (("window_s", 2, None), ("position_span_mm", None, 3),
              ("orientation_span_deg", None, 3), ("max_tilt_deg", None, 6),
              ("min_clearance_mm", 35, None))
    for key, lo, hi in bounds:
        value = number(video[key], key)
        passed = interval(value, number(video[key + "_error"], key + " uncertainty"), lo, hi, key)
        if key == "min_clearance_mm":
            obs["clear_35mm_throughout"] = passed
        else:
            obs[key] = value
    return dict(**grade(obs), observations=obs, net_mass_g=masses,
                mass_balance_residual_g=round(residual, 6), mass_uncertainty_g=errors)


def grade_sheet(sheet):
    try:
        return evaluate(sheet)
    except (KeyError, TypeError, Unscorable) as exc:
        return {"score": None, "status": "invalid", "reason": str(exc)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sheet", type=Path)
    args = parser.parse_args()
    try:
        result = grade_sheet(json.loads(args.sheet.read_text()))
    except (OSError, ValueError) as exc:
        result = {"score": None, "status": "invalid", "reason": str(exc)}
    print(json.dumps(result, indent=2))
    return 2 if result["status"] == "invalid" else 0


if __name__ == "__main__":
    raise SystemExit(main())
