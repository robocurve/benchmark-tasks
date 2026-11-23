"""Score terminal, load-bearing support from measured samples. All constants are bench criteria."""

import math
from statistics import median

from .station import HZ, STEPS

FIELDS = (
    "heel_n",
    "prox_n",
    "dist_n",
    "achilles_pad_n",
    "bed_pad_n",
    "free_gap_mm",
    "arm_clear_mm",
    "knee_pad_n",
    "support_span_mm",
    "robot_person_mean_n",
    "opposed_grip_mean_n",
    "robot_pad_n",
    "assist_nm",
    "knee_velocity",
    "cushion_speed",
)
FORCES = FIELDS[:5] + (
    "knee_pad_n",
    "support_span_mm",
    "robot_person_mean_n",
    "opposed_grip_mean_n",
    "robot_pad_n",
)


def score(rows):
    if len(rows) != STEPS:
        raise ValueError("Incomplete trial")
    for i, r in enumerate(rows):
        if (
            r["tick"] != i
            or not all(math.isfinite(r[k]) for k in FIELDS)
            or any(r[k] < 0 for k in FORCES)
        ):
            raise ValueError(f"Invalid sensor record at {i}")
    initial = rows[:5]
    base = median(r["heel_n"] for r in initial)
    if base < 5 or any(
        r["free_gap_mm"] > 1 or r["prox_n"] + r["dist_n"] > 1 for r in initial
    ):
        raise ValueError("Reset must show loaded heel and an empty cushion")
    # Sustained loading of the heel above its own relaxed baseline during direct robot contact.
    excess = 0
    grip = 0
    for r in rows:
        excess = (
            excess + 1
            if r["robot_person_mean_n"] > 1 and r["heel_n"] > 1.5 * base
            else 0
        )
        grip = grip + 1 if r["opposed_grip_mean_n"] > 0.25 * base else 0
        if grip >= HZ // 2:
            return 0.0, "sustained_opposing_grip"
        if excess >= HZ // 2:
            return 0.0, "sustained_extra_heel_load"
    end = rows[-(2 * HZ + 1) :]

    def relief(r):
        return (
            abs(r["assist_nm"]) < 0.01
            and abs(r["knee_velocity"]) < 0.08
            and r["cushion_speed"] < 0.012
            and r["heel_n"] <= 0.5 * base
            and r["prox_n"] + r["dist_n"] >= 0.5 * base
            and r["bed_pad_n"] >= 0.5 * base
            and r["achilles_pad_n"] < 1
            and r["knee_pad_n"] < 1
            and r["robot_person_mean_n"] < 1
        )

    if not all(relief(r) for r in end):
        return 0.0, "no_sustained_relief"

    def complete(r):
        return (
            r["heel_n"] <= 0.3
            and r["free_gap_mm"] >= 10
            and min(r["prox_n"], r["dist_n"]) >= 0.15 * base
            and r["support_span_mm"] >= 86
            and r["arm_clear_mm"] >= 25
            and r["robot_pad_n"] < 1
        )

    if all(complete(r) for r in end):
        return 1.0, "heel_floated"
    return 0.5, "relief_incomplete"
