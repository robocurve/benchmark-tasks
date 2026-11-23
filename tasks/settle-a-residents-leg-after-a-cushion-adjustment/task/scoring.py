"""One calibrated heel channel and frame-visible bench witnesses. Evaluate the final 2 s."""

import math
from statistics import median
from .station import HZ, STEPS


def _valid(w):
    for k in (
        "heel_channel_n",
        "sole_up",
        "heel_clear_mm",
        "protected_clear_mm",
        "sole_bed_gap_mm",
        "person_clear_mm",
        "park_gap_mm",
        "bearing_span_mm",
    ):
        if not math.isfinite(w[k]):
            raise ValueError("Nonfinite " + k)
    for name, n in (
        ("heel_xyz_mm", 3),
        ("knee_xyz_mm", 3),
        ("stripe_compression_mm", 6),
    ):
        if len(w[name]) != n or not all(map(math.isfinite, w[name])):
            raise ValueError("Incomplete " + name)
    if len(w["sole_marks_mm"]) != 4 or any(
        len(p) != 3 or not all(map(math.isfinite, p)) for p in w["sole_marks_mm"]
    ):
        raise ValueError("Missing cushion corner")
    if w["heel_channel_n"] < 0 or any(
        type(w[k]) is not bool for k in ("both_jaws_on_limb", "lift_slack", "bed_touch")
    ):
        raise ValueError("Invalid heel or marker reading")
    regions = w["calf_rows"]
    if len(regions) != 2 or any(
        any(
            type(i) is not int or i not in range(6) or w["stripe_compression_mm"][i] < 3
            for i in region
        )
        for region in regions
    ):
        raise ValueError("Invalid calf stripe")
    union = sorted(set(regions[0] + regions[1]))
    if w["bearing_span_mm"] != ((union[-1] - union[0]) * 43 if union else 0):
        raise ValueError("Inconsistent stripe span")


def score(rows):
    if len(rows) != STEPS or any(r["tick"] != i for i, r in enumerate(rows)):
        raise ValueError("Incomplete episode")
    frames = [r["witness"] for r in rows]
    for w in frames:
        _valid(w)
    base = median(w["heel_channel_n"] for w in frames[:5])
    if base < 5 or any(
        w["heel_clear_mm"] > 1 or any(w["calf_rows"]) for w in frames[:5]
    ):
        raise ValueError("Reset requires an initially loaded heel and unloaded cushion")
    pinch = excess = 0
    for w in frames:
        pinch = pinch + 1 if w["both_jaws_on_limb"] else 0
        excess = (
            excess + 1
            if w["person_clear_mm"] <= 1 and w["heel_channel_n"] > 1.5 * base
            else 0
        )
        if pinch >= 11:
            return 0.0, "sustained_opposing_grip"
        if excess >= 11:
            return 0.0, "sustained_extra_heel_load"
    end = frames[-(2 * HZ + 1) :]
    xyz = lambda w: [w["heel_xyz_mm"], w["knee_xyz_mm"], *w["sole_marks_mm"]]
    for marker in zip(*(xyz(w) for w in end)):
        if any(max(axis) - min(axis) > 5 for axis in zip(*marker)):
            return 0.0, "moving_at_end"
    if not all(
        w["lift_slack"]
        and w["heel_channel_n"] <= 0.5 * base
        and any(w["calf_rows"])
        and w["sole_bed_gap_mm"] <= 2
        and w["sole_up"] >= 0.9
        and w["protected_clear_mm"] >= 3
        and w["person_clear_mm"] > 1
        for w in end
    ):
        return 0.0, "no_sustained_relief"
    if all(
        w["heel_channel_n"] < 2
        and w["heel_clear_mm"] >= 10
        and all(w["calf_rows"])
        and w["bearing_span_mm"] >= 86
        and w["park_gap_mm"] >= 25
        for w in end
    ):
        return 1.0, "heel_floated"
    return 0.5, "relief_incomplete"
