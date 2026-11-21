"""Measure the retained pack-down result and its constituent physical operations."""

import numpy as np

from scene import TABLE_CENTER, TABLE_SIZE, TABLE_TOP

TABLE_CORNERS = (TABLE_CENTER[:2] - TABLE_SIZE[:2] / 2.,
                 TABLE_CENTER[:2] + TABLE_SIZE[:2] / 2.)


def geometry(row):
    positions = row["positions"]
    axis = np.array(row["axes"]["cup_base"])
    sleeve_axis = np.array(row["axes"]["heat_sleeve"])
    base = np.array(positions["cup_base"])
    sleeve = np.array(positions["heat_sleeve"])
    displacement = sleeve-base
    axial = float(displacement@axis)
    radial = float(np.linalg.norm(displacement-axial*axis))
    height = .107 - np.sum(row["folds_mm"])*.001
    sleeve_alignment = float(axis@sleeve_axis)
    sleeve_top = axial + max(0., .022*sleeve_alignment)
    sleeve_top += .0375*np.sqrt(max(0., 1.-sleeve_alignment**2))
    removed = sleeve_top < -.002 or radial > .085
    edge = axial if sleeve_alignment >= 0 else axial-.022
    stowed = abs(edge-.0005) < .004 and radial <= .003 and abs(sleeve_alignment) >= .985
    upright = axis[2] > .984 and abs(base[2]-TABLE_TOP) < .007
    return {"height_mm":float(height*1000), "sleeve_axial_mm":axial*1000,
            "sleeve_radial_mm":radial*1000, "removed":removed,
            "stowed":bool(stowed), "upright_on_table":bool(upright)}


def grade(history):
    if not history:
        return {"task_score":0., "reason":"empty_run"}
    flags = [geometry(row) for row in history]
    detached = False
    started_compressing = False
    invalid = []
    for row, geometry_values in zip(history,flags):
        vented_and_removed = geometry_values["removed"] and row["stopper_deg"] >= 45
        if vented_and_removed:
            detached = True
        if geometry_values["height_mm"] < 94.0 and not started_compressing:
            if not vented_and_removed:
                invalid.append("compression_before_vent_and_sleeve_removed")
            started_compressing = True
        if row.get("arms_touched", False):
            invalid.append("arm_collision")
        if min(row["folds_mm"]) < -2. or max(row["folds_mm"]) > 22.:
            invalid.append("fold_travel_exceeded")
        for obj in ("cup_base","heat_sleeve"):
            pos = np.asarray(row["positions"][obj])
            low, high = TABLE_CORNERS
            if np.any(pos[:2] < low) or np.any(pos[:2] > high) or pos[2] < TABLE_TOP - .009:
                invalid.append("part_left_table")
    stable_samples = history[-40:]
    hands_released = len(stable_samples) == 40 and all(
        not row.get("cup_touched", any(row["contact_n"].values())) for row in stable_samples
    )
    folded = hands_released and all(45. <= g["height_mm"] <= 52. and g["upright_on_table"] for g in flags[-40:])
    stowed = folded and all(g["stowed"] for g in flags[-40:])
    shut = stowed and all(abs(r["stopper_deg"]) <= 5. for r in stable_samples)
    task_score = float(sum((detached, folded, stowed, shut)))/4.
    if invalid:
        task_score = 0.
    return {"task_score":task_score, "vented_and_sleeve_removed":detached,
            "folded_and_stable":folded,"sleeve_stowed_and_stable":stowed,
            "closed_tab_and_stable":shut,"violations":sorted(set(invalid)), **flags[-1]}
