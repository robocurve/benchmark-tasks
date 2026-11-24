"""Grade either the detailed 280-frame worksheet or the interval worksheet for one bench trial."""

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import median


def detailed_grade(reader):
    from task.scoring import score

    rows = []
    for row in reader:
        if row["readable"] != "1":
            raise ValueError(f"Unscorable evidence at frame {row['frame']}")
        w = {
            k: float(row[k])
            for k in (
                "heel_channel_n",
                "heel_clear_mm",
                "protected_clear_mm",
                "sole_bed_gap_mm",
                "sole_up",
                "person_clear_mm",
                "park_gap_mm",
            )
        }
        for k in ("lift_slack", "bed_touch", "both_jaws_on_limb"):
            if row[k] not in ("1", "0"):
                raise ValueError("Expected 0/1 in " + k)
            w[k] = row[k] == "1"
        for k in (
            "heel_xyz_mm",
            "knee_xyz_mm",
            "sole_marks_mm",
            "stripe_compression_mm",
            "calf_rows",
        ):
            w[k] = json.loads(row[k])
        occupied = sorted(set(w["calf_rows"][0] + w["calf_rows"][1]))
        w["bearing_span_mm"] = (occupied[-1] - occupied[0]) * 43 if occupied else 0
        rows.append(dict(tick=int(row["frame"]) - 1, witness=w))
    return score(rows)


RESET_LOADS = tuple(f"reset_heel_n_frame_{i}" for i in range(1, 6))
NUMERIC = (
    *RESET_LOADS,
    "reset_max_heel_gap_mm",
    "dwell_max_marker_axis_excursion_mm",
    "dwell_max_heel_n",
    "dwell_max_sole_bed_gap_mm",
    "dwell_min_sole_up",
    "dwell_min_protected_gap_mm",
    "dwell_min_person_gap_mm",
    "dwell_min_heel_gap_mm",
    "dwell_min_park_gap_mm",
)
DISCRETE = {
    "reset_max_bearing_regions": {0, 1, 2},
    "dwell_all_slack": {0, 1},
    "dwell_min_bearing_regions": {0, 1, 2},
    "dwell_min_bearing_span_mm": set(range(0, 216, 43)),
}
WINDOWS = ("opposing_grip_windows", "extra_heel_load_with_arm_contact_windows")
SHORT_FIELDS = ("check", "value", "uncertainty", "evidence")


def latch_frame(windows):
    """Return the 11th observation of the first qualifying run, on the 1-based video clock."""
    if windows == "none":
        return math.inf
    first = math.inf
    previous_end = -1
    for interval in windows.split():
        limits = interval.split("-")
        if len(limits) != 2 or not all(x.isdigit() for x in limits):
            raise ValueError("Expected inclusive first-last frame numbers or 'none'.")
        start, end = map(int, limits)
        if not 1 <= start <= end <= 280 or start <= previous_end + 1:
            raise ValueError(
                "Event windows must be ordered, disjoint maximal runs within frames 1–280."
            )
        if end - start >= 10:
            first = min(first, start + 10)
        previous_end = end
    return first


def certain(low_test, high_test, name):
    if low_test != high_test:
        raise ValueError("Unscorable measurement uncertainty in " + name)
    return low_test


def interval_grade(reader):
    """Exact extrema for universal gates; each joint predicate must hold in the same frame."""
    expected = {"coverage", *NUMERIC, *DISCRETE, *WINDOWS}
    cells = {}
    for row in reader:
        if None in row or any(v is None for v in row.values()):
            raise ValueError("Malformed interval worksheet row.")
        key, value, uncertainty, evidence = (row[k].strip() for k in SHORT_FIELDS)
        if (
            key not in expected
            or key in cells
            or not value
            or value == "?"
            or not evidence
            or evidence == "?"
        ):
            raise ValueError(
                "Unknown, duplicate, incomplete or unreadable interval check: " + key
            )
        if key in NUMERIC:
            v, u = float(value), float(uncertainty)
            if not math.isfinite(v) or not math.isfinite(u) or u < 0:
                raise ValueError("Invalid value or uncertainty in " + key)
            if (
                key in RESET_LOADS
                or key in ("dwell_max_heel_n", "dwell_max_marker_axis_excursion_mm")
            ) and v < 0:
                raise ValueError("Negative load or excursion in " + key)
            if key == "dwell_min_sole_up" and not -1 <= v <= 1:
                raise ValueError("Sole vertical component outside [-1, 1].")
            cells[key] = (v - u, v + u)
        else:
            if uncertainty:
                raise ValueError(
                    "Use '?' for an uncertain categorical or interval observation: "
                    + key
                )
            if key in DISCRETE:
                if value not in {str(n) for n in DISCRETE[key]}:
                    raise ValueError("Invalid count or category in " + key)
                cells[key] = int(value)
            else:
                cells[key] = value
    if set(cells) != expected:
        raise ValueError(
            "Missing interval checks: " + ", ".join(sorted(expected - set(cells)))
        )
    if cells["coverage"] != "1-280":
        raise ValueError(
            "Unscorable: record 280 consecutive readable and synchronized observations."
        )

    def decide(key, predicate):
        low, high = cells[key]
        return certain(predicate(low), predicate(high), key)

    # Medians are monotone coordinate-wise, so these endpoints bound the true reset median.
    base_lo, base_hi = (median(cells[key][i] for key in RESET_LOADS) for i in (0, 1))
    reset_ok = (
        certain(base_lo >= 5, base_hi >= 5, "reset median load"),
        decide("reset_max_heel_gap_mm", lambda v: v <= 1),
        cells["reset_max_bearing_regions"] == 0,
    )
    if not all(reset_ok):
        raise ValueError(
            "Reset requires an initially loaded heel and an unloaded cushion."
        )
    pinch, excess = (latch_frame(cells[k]) for k in WINDOWS)
    if min(pinch, excess) != math.inf:
        return (
            0.0,
            "sustained_opposing_grip"
            if pinch <= excess
            else "sustained_extra_heel_load",
        )
    if not decide("dwell_max_marker_axis_excursion_mm", lambda v: v <= 5):
        return 0.0, "moving_at_end"
    heel_lo, heel_hi = cells["dwell_max_heel_n"]
    relief = certain(
        heel_hi <= 0.5 * base_lo, heel_lo <= 0.5 * base_hi, "half-baseline heel relief"
    )
    partial = (
        relief,
        cells["dwell_all_slack"] == 1,
        cells["dwell_min_bearing_regions"] >= 1,
        decide("dwell_max_sole_bed_gap_mm", lambda v: v <= 2),
        decide("dwell_min_sole_up", lambda v: v >= 0.9),
        decide("dwell_min_protected_gap_mm", lambda v: v >= 3),
        decide("dwell_min_person_gap_mm", lambda v: v > 1),
    )
    if not all(partial):
        return 0.0, "no_sustained_relief"
    full = (
        decide("dwell_max_heel_n", lambda v: v < 2),
        decide("dwell_min_heel_gap_mm", lambda v: v >= 10),
        cells["dwell_min_bearing_regions"] == 2,
        cells["dwell_min_bearing_span_mm"] >= 86,
        decide("dwell_min_park_gap_mm", lambda v: v >= 25),
    )
    return (1.0, "heel_floated") if all(full) else (0.5, "relief_incomplete")


def grade(path):
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        if tuple(reader.fieldnames or ()) == SHORT_FIELDS:
            return interval_grade(reader)
        if "frame" not in (reader.fieldnames or ()):
            raise ValueError("Expected the interval sheet or detailed frame sheet.")
        return detailed_grade(reader)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("worksheet", type=Path)
    try:
        print(*grade(p.parse_args().worksheet))
    except (KeyError, ValueError, TypeError, IndexError) as exc:
        p.exit(2, f"Unscorable worksheet: {exc}\n")
