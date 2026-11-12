from __future__ import annotations

"""Scoring logic for keyboard_tray_latch (T22)."""

import math
from typing import Mapping, Sequence

from geometry import DESK_POS_WORLD, GATE_DESK_DRIFT_MAX_M


def evaluate_state(
    desk_pos: Sequence[float],
    tray_slide_m: float,
    latch_engaged: bool,
    resting: bool = True,
) -> tuple[float, bool, str]:
    drift = math.sqrt(
        (desk_pos[0] - DESK_POS_WORLD[0]) ** 2
        + (desk_pos[1] - DESK_POS_WORLD[1]) ** 2
        + (desk_pos[2] - DESK_POS_WORLD[2]) ** 2
    )
    if drift > GATE_DESK_DRIFT_MAX_M:
        return 0.0, False, f"Gate: desk moved {drift*1000:.1f} mm > 5.0 mm"

    if tray_slide_m < 0.050:
        return 0.0, False, f"Tray moved {tray_slide_m*100:.1f} cm < 5 cm"
    if tray_slide_m < 0.150:
        return 0.25, False, f"Tray moved {tray_slide_m*100:.1f} cm >= 5 cm"
    if tray_slide_m < 0.260:
        return 0.50, False, f"Tray moved {tray_slide_m*100:.1f} cm >= 15 cm"
    if not latch_engaged or tray_slide_m < 0.278:
        return 0.75, False, f"Tray moved {tray_slide_m*100:.1f} cm >= 26 cm, not yet latched"

    if resting:
        return 1.00, True, "Tray slid in 27.8 cm, purple latch engaged, settled"
    return 0.75, False, "Tray latched, settling..."
