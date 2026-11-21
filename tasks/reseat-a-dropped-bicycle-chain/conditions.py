"""Measured bench parameters for the deterministic sensitivity trials."""

from fixtures import Chain

# Lateral reset offset (mm), catcher radius reduction (mm), rear drag (mN m).
CASES = {
    "reference": (13, 0, 2),
    "offset_11": (11, 0, 2),
    "offset_15": (15, 0, 2),
    "drag_1": (13, 0, 1),
    "drag_3": (13, 0, 3),
    "sleeve_2": (13, 2, 2),
    "sleeve_5": (13, 5, 2),
    "sleeve_10": (13, 10, 2),
    "deep_near_light": (11, 10, 1),
    "deep_far_heavy": (15, 10, 3),
}


def build(case, tool, variant=0):
    drop, reduction, drag = CASES[case]
    return Chain(
        variant,
        drop=drop / 1000,
        sleeve_reduction=reduction / 1000,
        rear_drag=drag / 1000,
        lift_pick=tool == "pick",
    )
