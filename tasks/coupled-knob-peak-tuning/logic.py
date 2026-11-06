"""Hidden-state logic for coupled knob peak tuning. Pure Python, no simulator imports.

The two knob settings (x = left knob, y = right knob, both in [0, 100] mm) are coupled along a diagonal ridge:
  r(x, y) = exp(-(u^2 / (2 su^2) + v^2 / (2 sv^2)))     (r is the meter reading, 0..1)
  u = ((x - x0) + (y - y0)) / sqrt(2)    (along the ridge, 45 degrees)
  v = (-(x - x0) + (y - y0)) / sqrt(2)   (across the ridge)
  su = 45 mm, sv = 9 mm; the peak (x0, y0) is one of three documented VARIANTS selected by the TASK_VARIANT
  environment variable (default 0 = (80, 50) mm; the trial list in README.md says which variant each trial uses).

READ button press (rising edge with hysteresis at 20 Hz) evaluates r(x, y) and moves the meter needle to r * 160 mm along the
scale bar, keeping it there until the next press. Only the first 8 presses count towards the budget. To score 0.75 the counted
presses must include at least 2 separated settings, and to score 1.0 at least 3 separated settings (separated = at least
10 mm apart, in the larger of the two knob differences, from every earlier separated press).
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

# Peak variants (x0, y0) in mm. Variant 0 is the default and the one in the demo video.
VARIANTS = ((80.0, 50.0), (88.0, 72.0), (95.0, 55.0))
SCALE_SU = 45.0  # mm
SCALE_SV = 9.0   # mm

# Button hysteresis thresholds for 8 mm travel (slider axis Z, negative downward displacement)
BUTTON_PRESSED_THRESHOLD = -0.0064  # m (80% of 8 mm travel)
BUTTON_RELEASED_THRESHOLD = -0.0035  # m; GPU run 1: the unpressed cap rests 2.0 mm down, so -1.6 mm never re-armed

MAX_COUNTED_PRESSES = 8
SEPARATION_MM = 10.0  # presses count as separated when they differ by >= 10 mm in max(|dx|, |dy|)
DISTINCT_READING_MIN_DIFF_MM = SEPARATION_MM  # old name, kept for the tests
MIN_SEPARATED_FOR_0_75 = 2
MIN_SEPARATED_FOR_1_00 = 3


def _variant_from_env() -> int:
    try:
        v = int(os.environ.get("TASK_VARIANT", "0"))
    except ValueError:
        v = 0
    return v if 0 <= v < len(VARIANTS) else 0


VARIANT = _variant_from_env()
PEAK_X0, PEAK_Y0 = VARIANTS[VARIANT]


def set_variant(index: int) -> None:
    """Select the hidden peak variant (tests and demo planning; the simulator reads TASK_VARIANT at import)."""
    global VARIANT, PEAK_X0, PEAK_Y0
    if not 0 <= index < len(VARIANTS):
        raise ValueError(f"variant must be 0..{len(VARIANTS) - 1}")
    VARIANT = index
    PEAK_X0, PEAK_Y0 = VARIANTS[index]


def reading(x_mm: float, y_mm: float, variant: int | None = None) -> float:
    """Evaluate continuous meter reading r in [0, 1] at knob settings (x, y) in mm."""
    x0, y0 = (PEAK_X0, PEAK_Y0) if variant is None else VARIANTS[variant]
    dx = float(x_mm) - x0
    dy = float(y_mm) - y0
    sqrt2 = math.sqrt(2.0)
    u = (dx + dy) / sqrt2
    v = (-dx + dy) / sqrt2
    exponent = -((u * u) / (2.0 * SCALE_SU * SCALE_SU) + (v * v) / (2.0 * SCALE_SV * SCALE_SV))
    return float(math.exp(exponent))


def count_separated(history: list[tuple[float, float]]) -> int:
    """Number of presses (in order) that are >= SEPARATION_MM from every earlier counted-as-separated press."""
    kept: list[tuple[float, float]] = []
    for x, y in history:
        if all(max(abs(x - a), abs(y - b)) >= SEPARATION_MM for a, b in kept):
            kept.append((x, y))
    return len(kept)


@dataclass
class LogicState:
    press_count: int = 0
    counted_presses: int = 0
    needle_r: float = 0.0
    last_read_r: float = 0.0
    button_pressed: bool = False
    rising_edge: bool = False
    is_counted: bool = False
    has_distinct_readings: bool = False  # >= 2 separated counted presses (needed for 0.75)
    has_three_separated: bool = False  # >= 3 separated counted presses (needed for 1.0)
    separated_presses: int = 0
    press_history: list[tuple[float, float]] = field(default_factory=list)


class KnobTuningLogic:
    """Stateful logic tracker for button hysteresis, press budget, and needle position."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.press_count = 0
        self.counted_presses = 0
        self.needle_r = 0.0
        self.last_read_r = 0.0
        self.button_pressed = False
        self.press_history: list[tuple[float, float]] = []

    def _check_distinct_readings(self) -> bool:
        return count_separated(self.press_history) >= MIN_SEPARATED_FOR_0_75

    def step(self, btn_slider_z: float, knob_x_mm: float, knob_y_mm: float) -> LogicState:
        """Update button state and needle display given current button slider position and knob settings."""
        rising_edge = False
        is_counted = False

        if not self.button_pressed:
            if btn_slider_z <= BUTTON_PRESSED_THRESHOLD:
                self.button_pressed = True
                rising_edge = True
                self.press_count += 1
                r = reading(knob_x_mm, knob_y_mm)
                self.needle_r = r
                self.last_read_r = r
                if self.press_count <= MAX_COUNTED_PRESSES:
                    self.counted_presses += 1
                    is_counted = True
                    self.press_history.append((float(knob_x_mm), float(knob_y_mm)))  # only counted presses can qualify
        else:
            if btn_slider_z >= BUTTON_RELEASED_THRESHOLD:
                self.button_pressed = False

        return LogicState(
            press_count=self.press_count,
            counted_presses=self.counted_presses,
            needle_r=self.needle_r,
            last_read_r=self.last_read_r,
            button_pressed=self.button_pressed,
            rising_edge=rising_edge,
            is_counted=is_counted,
            has_distinct_readings=self._check_distinct_readings(),
            has_three_separated=count_separated(self.press_history) >= MIN_SEPARATED_FOR_1_00,
            separated_presses=count_separated(self.press_history),
            press_history=list(self.press_history),
        )


def coordinate_search_path(variant: int = 0, start=(30.0, 30.0), readings: int = 8) -> list[tuple[float, float, float]]:
    """Oracle single-axis search: alternately set one knob to its best value given the other (exact line search with
    the true function), taking one reading after each move. Returns [(x, y, r)] for the start plus ``readings`` moves."""
    x, y = start
    out = [(x, y, reading(x, y, variant))]
    x0, y0 = VARIANTS[variant]
    inv_su2, inv_sv2 = 1.0 / SCALE_SU**2, 1.0 / SCALE_SV**2
    ratio = (inv_sv2 - inv_su2) / (inv_sv2 + inv_su2)
    for k in range(readings):
        if k % 2 == 0:
            x = max(0.0, min(100.0, x0 + (y - y0) * ratio))
        else:
            y = max(0.0, min(100.0, y0 + (x - x0) * ratio))
        out.append((x, y, reading(x, y, variant)))
    return out


def ridge_path(variant: int = 0, start=(30.0, 30.0)) -> list[tuple[float, float]]:
    """Reading points of the scripted demo: two single-axis probes, then jumps along the ridge to the peak."""
    x0, y0 = VARIANTS[variant]
    sx, sy = start
    fx = (0.0, 0.5, 0.5, 0.85, 1.0)
    fy = (0.0, 0.0, 0.6, 0.9, 1.0)
    return [(round(sx + a * (x0 - sx), 1), round(sy + b * (y0 - sy), 1)) for a, b in zip(fx, fy, strict=True)]
