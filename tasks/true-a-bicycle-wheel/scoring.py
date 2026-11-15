"""Score a completed physical judging record. Thresholds are provisional.

This does not read simulator hidden state or grant a submission certificate.
Measurements must come from the whole-wheel released check in JUDGING.md.
"""
import argparse
import json
import math
from pathlib import Path

CHECKS = ("correct_start", "whole_revolution_visible", "hands_and_tools_clear",
          "wheel_and_stand_intact", "nipples_only_adjusted",
          "spoke_tension_check_complete", "evidence_reviewed")


def score(record):
    result = {"score": 0., "record_valid": False, "submission_certified": False,
              "thresholds_calibrated": False, "reasons": []}
    try:
        if record.get("schema") != "wheel-judging-v2":
            raise ValueError("Unsupported judging record")
        checks = record["checks"]
        if any(type(checks.get(k)) is not bool for k in CHECKS):
            raise ValueError("All observable checks must be reviewed as true or false")
        names = ("lateral_runout_mm", "radial_runout_mm", "dish_offset_mm")
        values = {k: record[k] for k in names}
        if any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in values.values()):
            raise ValueError("All measurements must be finite nonnegative millimetres")
        tensions = record["spoke_tensions_n"]
        if not isinstance(tensions, list) or len(tensions) != 32 or any(
                type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in tensions):
            raise ValueError("All 32 spoke tensions must be measured in newtons")
        result["record_valid"] = True
        failures = [k for k in CHECKS if not checks[k]]
        if values["radial_runout_mm"] > 1.: failures.append("radial runout exceeds 1.0 mm")
        if values["dish_offset_mm"] > .5: failures.append("dish offset exceeds 0.5 mm")
        if min(tensions) < 400. or max(tensions) > 1000.:
            failures.append("spoke tension outside specified fixture range 400 to 1000 N")
        result["reasons"] = failures
        if not failures:
            r = values["lateral_runout_mm"]
            result["score"] = 1. if r <= .5 else .5 if r <= 1.5 else 0.
    except (KeyError, TypeError, ValueError) as e:
        result["reasons"].append(str(e))
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("record", type=Path)
    args = p.parse_args()
    print(json.dumps(score(json.loads(args.record.read_text())), indent=2))
