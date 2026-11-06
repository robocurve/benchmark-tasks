"""Pure Python hidden-state logic for backlash dial setpoint.

Story:
A real calibration dial has backlash: after you reverse direction it takes a few mm of travel
before the indicator moves.

Hidden logic:
- displayed = true position, except after a direction reversal the display holds still until the knob
  has moved 6 mm further (then follows with a 6 mm lag that persists while moving in that direction).
- TARGET band: setpoint +- 2.5 mm displayed. Left setpoint = 65 mm on rail, Right setpoint = 40 mm on rail.
- Press LOCK when display is in band; value locked is the displayed one.
- Zero-gate: fixture moved > 5 mm; a lock outside band is allowed once per dial (can unlock by pressing LOCK again),
  but a third lock attempt on a dial latches 0.0.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# Dimensions and tolerances in mm
BACKLASH_DEADBAND_MM = 6.0
TARGET_BAND_TOL_MM = 2.5
SETPOINTS_MM = {"left": 65.0, "right": 40.0}
START_SETTINGS_MM = {"left": 30.0, "right": 70.0}

# Button hysteresis thresholds for 8 mm travel (slider axis Z, negative downward displacement)
BUTTON_PRESSED_THRESHOLD = -0.0064  # m (80% of 8 mm travel)
BUTTON_RELEASED_THRESHOLD = -0.0035  # m (re-arm level)

MAX_ALLOWED_LOCK_ATTEMPTS = 2  # 3rd attempt latches zero-gate
MAX_ALLOWED_OUTSIDE_LOCKS = 1  # 2nd lock outside band latches zero-gate


def target_band(side: str) -> tuple[float, float]:
    """Return (min_val, max_val) displayed in mm for target band."""
    sp = SETPOINTS_MM[side]
    return (sp - TARGET_BAND_TOL_MM, sp + TARGET_BAND_TOL_MM)


def is_in_band(side: str, displayed_mm: float) -> bool:
    """Return whether displayed_mm is inside the target band for side."""
    sp = SETPOINTS_MM[side]
    return abs(float(displayed_mm) - sp) <= TARGET_BAND_TOL_MM + 1e-6


@dataclass
class BacklashDial:
    """Hysteresis backlash tracker for a single dial."""

    side: str
    true_x: float = 0.0
    displayed: float = 0.0
    prev_true_x: float = 0.0

    # Direction and reversal tracking
    # 1 for forward (increasing x), -1 for backward (decreasing x)
    current_dir: int = 1
    # Tracking significant motion for scoring
    min_x_seen: float = 100.0
    max_x_seen: float = 0.0
    downward_travel_mm: float = 0.0
    upward_travel_mm: float = 0.0
    has_reversed: bool = False

    # LOCK state
    locked: bool = False
    locked_val: float | None = None
    locked_in_band: bool = False
    lock_attempts: int = 0
    outside_band_locks: int = 0

    # Button hysteresis
    btn_pressed: bool = False

    def __post_init__(self):
        start = START_SETTINGS_MM[self.side]
        self.reset(start)

    def reset(self, start_x: float | None = None) -> None:
        if start_x is None:
            start_x = START_SETTINGS_MM[self.side]
        self.true_x = float(start_x)
        self.prev_true_x = float(start_x)
        self.displayed = float(start_x)  # starts calibrated
        self.current_dir = 1
        self.min_x_seen = float(start_x)
        self.max_x_seen = float(start_x)
        self.downward_travel_mm = 0.0
        self.upward_travel_mm = 0.0
        self.has_reversed = False

        self.locked = False
        self.locked_val = None
        self.locked_in_band = False
        self.lock_attempts = 0
        self.outside_band_locks = 0
        self.btn_pressed = False

    def update_position(self, x_mm: float) -> float:
        """Update true position and return current displayed value."""
        x = float(x_mm)
        dx = x - self.prev_true_x

        # Track motion ranges
        if dx < -0.01:
            self.downward_travel_mm += abs(dx)
        elif dx > 0.01:
            self.upward_travel_mm += dx

        # A direction reversal required traversing backlash deadband:
        # e.g., significant downward travel (>= 6 mm) followed by upward travel (>= 6 mm)
        if self.downward_travel_mm >= BACKLASH_DEADBAND_MM and self.upward_travel_mm >= BACKLASH_DEADBAND_MM:
            self.has_reversed = True

        # Backlash hysteresis model:
        # If knob moves in + direction beyond display: display follows true position (y = x)
        # If knob moves in - direction beyond display - B: display follows with B lag (y = x + B)
        # In deadband: display holds still
        y = self.displayed
        if x > y:
            y = x
        elif x + BACKLASH_DEADBAND_MM < y:
            y = x + BACKLASH_DEADBAND_MM
        else:
            pass  # holds still inside deadband

        self.true_x = x
        self.prev_true_x = x
        self.displayed = y
        return y

    def update_button(self, btn_z: float) -> tuple[bool, bool]:
        """Update LOCK button state given slider z displacement.

        Returns (is_pressed, rising_edge).
        """
        rising_edge = False
        if not self.btn_pressed and btn_z <= BUTTON_PRESSED_THRESHOLD:
            self.btn_pressed = True
            rising_edge = True
            # Rising edge triggers LOCK / UNLOCK
            self._handle_lock_toggle()
        elif self.btn_pressed and btn_z >= BUTTON_RELEASED_THRESHOLD:
            self.btn_pressed = False

        return self.btn_pressed, rising_edge

    def _handle_lock_toggle(self) -> None:
        """Handle rising edge press on LOCK button."""
        if not self.locked:
            # Attempting to lock
            self.lock_attempts += 1
            self.locked = True
            self.locked_val = self.displayed
            self.locked_in_band = is_in_band(self.side, self.displayed)
            if not self.locked_in_band:
                self.outside_band_locks += 1
        else:
            # Unlocking: can unlock by pressing LOCK again
            self.locked = False
            self.locked_in_band = False
            self.locked_val = None


@dataclass
class BacklashLogicState:
    displayed: dict[str, float] = field(default_factory=dict)
    locked: dict[str, bool] = field(default_factory=dict)
    locked_val: dict[str, float | None] = field(default_factory=dict)
    locked_in_band: dict[str, bool] = field(default_factory=dict)
    lock_attempts: dict[str, int] = field(default_factory=dict)
    outside_band_locks: dict[str, int] = field(default_factory=dict)
    has_reversed: dict[str, bool] = field(default_factory=dict)
    btn_pressed: dict[str, bool] = field(default_factory=dict)
    rising_edge: dict[str, bool] = field(default_factory=dict)
    zero_gate: bool = False
    zero_gate_reason: str = ""


class BacklashDialLogic:
    """Stateful logic tracker for both dials and their LOCK buttons."""

    def __init__(self) -> None:
        self.dials = {
            "left": BacklashDial("left"),
            "right": BacklashDial("right"),
        }
        self.zero_gate_latched = False
        self.zero_gate_reason = ""

    def reset(self) -> None:
        for d in self.dials.values():
            d.reset()
        self.zero_gate_latched = False
        self.zero_gate_reason = ""

    def step(
        self,
        setting_l: float,
        setting_r: float,
        btn_l_z: float,
        btn_r_z: float,
    ) -> BacklashLogicState:
        """Step both dials and return BacklashLogicState."""
        state = BacklashLogicState()

        # Update positions
        disp_l = self.dials["left"].update_position(setting_l)
        disp_r = self.dials["right"].update_position(setting_r)

        # Update buttons
        btn_p_l, rise_l = self.dials["left"].update_button(btn_l_z)
        btn_p_r, rise_r = self.dials["right"].update_button(btn_r_z)

        # Check zero-gates on lock attempts
        for side, dial in self.dials.items():
            if dial.lock_attempts > MAX_ALLOWED_LOCK_ATTEMPTS:
                self.zero_gate_latched = True
                self.zero_gate_reason = f"Third lock attempt on {side} dial"
            if dial.outside_band_locks > MAX_ALLOWED_OUTSIDE_LOCKS:
                self.zero_gate_latched = True
                self.zero_gate_reason = f"Second lock outside band on {side} dial"

        state.displayed = {"left": disp_l, "right": disp_r}
        state.locked = {"left": self.dials["left"].locked, "right": self.dials["right"].locked}
        state.locked_val = {"left": self.dials["left"].locked_val, "right": self.dials["right"].locked_val}
        state.locked_in_band = {"left": self.dials["left"].locked_in_band, "right": self.dials["right"].locked_in_band}
        state.lock_attempts = {"left": self.dials["left"].lock_attempts, "right": self.dials["right"].lock_attempts}
        state.outside_band_locks = {"left": self.dials["left"].outside_band_locks, "right": self.dials["right"].outside_band_locks}
        state.has_reversed = {"left": self.dials["left"].has_reversed, "right": self.dials["right"].has_reversed}
        state.btn_pressed = {"left": btn_p_l, "right": btn_p_r}
        state.rising_edge = {"left": rise_l, "right": rise_r}
        state.zero_gate = self.zero_gate_latched
        state.zero_gate_reason = self.zero_gate_reason

        return state
