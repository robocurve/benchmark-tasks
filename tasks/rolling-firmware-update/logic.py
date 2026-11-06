"""Hidden state machine for the rolling firmware update task. Pure Python.

Tracks:
- Unit A and Unit B versions (0 or 1), active flag, updating flag, and progress (0..80 steps at 20 Hz).
- Switch button toggle between units.
- Service door deflections (>= 12 mm required to update, < 2 mm considered released).
- Hysteresis on button presses (press <= -6.4 mm, re-arm >= -3.5 mm).
- OUTAGE invariant: latched if the active unit is updating (offline) at any time.
- Status lamps: ACTIVE (blue), VERSION (grey/amber/white), PROGRESS (0..4 cyan lamps), LOAD (green/red).
"""

from __future__ import annotations

import geometry as g

UPDATE_DURATION_S = 4.0
CONTROL_HZ = 20.0
REQUIRED_PROGRESS_STEPS = int(round(UPDATE_DURATION_S * CONTROL_HZ))  # 80 steps


class UnitState:
    """State of a single redundant unit."""

    def __init__(self, name: str, active: bool = False):
        self.name = name
        self.version = 0  # 0 (old) or 1 (new)
        self.updating = False
        self.progress_steps = 0  # 0..80
        self.active = active

    def reset(self, active: bool = False) -> None:
        self.version = 0
        self.updating = False
        self.progress_steps = 0
        self.active = active

    @property
    def progress_s(self) -> float:
        return self.progress_steps / CONTROL_HZ

    @property
    def is_offline(self) -> bool:
        """A unit is offline while updating."""
        return self.updating


class RollingFirmwareLogic:
    """Hidden state machine evaluated at every control step (20 Hz)."""

    def __init__(self, hz: float = CONTROL_HZ, update_duration_s: float = UPDATE_DURATION_S) -> None:
        self.hz = hz
        self.required_steps = int(round(update_duration_s * hz))
        self.unit_a = UnitState("A", active=True)
        self.unit_b = UnitState("B", active=False)
        self.outage = False

        # Button press tracking (hysteresis)
        self._btn_a_pressed = False
        self._btn_b_pressed = False
        self._btn_sw_pressed = False

        self.counted_update_a_presses = 0
        self.counted_update_b_presses = 0
        self.counted_switch_presses = 0
        self.raw_button_press_count = 0

    def reset(self) -> None:
        self.unit_a.reset(active=True)
        self.unit_b.reset(active=False)
        self.outage = False
        self._btn_a_pressed = False
        self._btn_b_pressed = False
        self._btn_sw_pressed = False
        self.counted_update_a_presses = 0
        self.counted_update_b_presses = 0
        self.counted_switch_presses = 0
        self.raw_button_press_count = 0

    @property
    def active_unit(self) -> str:
        return "A" if self.unit_a.active else "B"

    @property
    def total_counted_presses(self) -> int:
        return self.counted_update_a_presses + self.counted_update_b_presses + self.counted_switch_presses

    def step(
        self,
        door_a_deflection: float,
        door_b_deflection: float,
        btn_update_a_z: float,
        btn_update_b_z: float,
        btn_switch_z: float,
    ) -> dict:
        """Evaluate rules at one control step.

        Parameters:
            door_a_deflection: positive distance door A is open (metres).
            door_b_deflection: positive distance door B is open (metres).
            btn_update_a_z: joint pos of UPDATE A slider (negative when pressed).
            btn_update_b_z: joint pos of UPDATE B slider.
            btn_switch_z: joint pos of SWITCH slider.

        Returns state dictionary.
        """
        # 1. Rising edge detection on buttons
        # UPDATE A
        if btn_update_a_z <= g.BUTTON_PRESS_THRESHOLD and not self._btn_a_pressed:
            self._btn_a_pressed = True
            self.raw_button_press_count += 1
            # UPDATE counted press while door deflection >= 12 mm starts update
            if door_a_deflection >= g.DOOR_DEFLECTION_THRESHOLD:
                self.counted_update_a_presses += 1
                if self.unit_a.version == 0 and not self.unit_a.updating:
                    self.unit_a.updating = True
                    self.unit_a.progress_steps = 0
        elif btn_update_a_z >= g.BUTTON_REARM_THRESHOLD:
            self._btn_a_pressed = False

        # UPDATE B
        if btn_update_b_z <= g.BUTTON_PRESS_THRESHOLD and not self._btn_b_pressed:
            self._btn_b_pressed = True
            self.raw_button_press_count += 1
            if door_b_deflection >= g.DOOR_DEFLECTION_THRESHOLD:
                self.counted_update_b_presses += 1
                if self.unit_b.version == 0 and not self.unit_b.updating:
                    self.unit_b.updating = True
                    self.unit_b.progress_steps = 0
        elif btn_update_b_z >= g.BUTTON_REARM_THRESHOLD:
            self._btn_b_pressed = False

        # SWITCH
        if btn_switch_z <= g.BUTTON_PRESS_THRESHOLD and not self._btn_sw_pressed:
            self._btn_sw_pressed = True
            self.raw_button_press_count += 1
            self.counted_switch_presses += 1
            # Move active to the other unit instantly
            if self.unit_a.active:
                self.unit_a.active = False
                self.unit_b.active = True
            else:
                self.unit_a.active = True
                self.unit_b.active = False
        elif btn_switch_z >= g.BUTTON_REARM_THRESHOLD:
            self._btn_sw_pressed = False

        # 2. Updating progress advances only while door deflection >= 18 mm
        if self.unit_a.updating:
            if door_a_deflection >= g.DOOR_DEFLECTION_THRESHOLD:
                self.unit_a.progress_steps += 1
                if self.unit_a.progress_steps >= self.required_steps:
                    self.unit_a.version = 1
                    self.unit_a.updating = False
            else:
                # Releasing door resets progress and cancels update
                self.unit_a.updating = False
                self.unit_a.progress_steps = 0

        if self.unit_b.updating:
            if door_b_deflection >= g.DOOR_DEFLECTION_THRESHOLD:
                self.unit_b.progress_steps += 1
                if self.unit_b.progress_steps >= self.required_steps:
                    self.unit_b.version = 1
                    self.unit_b.updating = False
            else:
                self.unit_b.updating = False
                self.unit_b.progress_steps = 0

        # 3. OUTAGE invariant: active unit is offline (updating) at any time
        if (self.unit_a.active and self.unit_a.is_offline) or (self.unit_b.active and self.unit_b.is_offline):
            self.outage = True

        # 4. Compute visual indicator lamp states
        load_lamp = "red" if self.outage else "green"

        def _version_lamp(unit: UnitState) -> str:
            if unit.updating:
                return "amber"
            return "white" if unit.version == 1 else "grey"

        def _progress_count(unit: UnitState) -> int:
            if unit.version == 1:
                return 4
            if unit.updating:
                # 20 steps per lamp (1.0 s)
                return min(4, self.unit_b.progress_steps // 20 if unit.name == "B" else self.unit_a.progress_steps // 20)
            return 0

        return {
            "outage": self.outage,
            "active_unit": self.active_unit,
            "unit_a_version": self.unit_a.version,
            "unit_b_version": self.unit_b.version,
            "unit_a_updating": self.unit_a.updating,
            "unit_b_updating": self.unit_b.updating,
            "unit_a_progress_steps": self.unit_a.progress_steps,
            "unit_b_progress_steps": self.unit_b.progress_steps,
            "load_lamp": load_lamp,
            "version_lamp_a": _version_lamp(self.unit_a),
            "version_lamp_b": _version_lamp(self.unit_b),
            "progress_lamps_a": _progress_count(self.unit_a),
            "progress_lamps_b": _progress_count(self.unit_b),
        }
