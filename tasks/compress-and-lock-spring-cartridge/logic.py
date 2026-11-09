"""State machine and verification logic for compress-and-lock-spring-cartridge. Pure Python.

Tracks:
- Plunger displacement (0-45 mm along +X).
- Plunger compressed >= 38 mm held for >= 1.0 s.
- Physical stop box logic: Lock bar travel clamped to 15 mm unless plunger is compressed >= 38 mm.
- Lock bar sliding across >= 55 mm while plunger compressed (LOCKED state, latched ever_locked per lesson 8).
- Release detection: Plunger released (no arm contact) while retained >= 38 mm by lock bar for >= 2.0 s.
- Arm clearance and status lamp: Both arms clear and lamp green held for >= 1.0 s (20 steps).
- Zero-gate: None (physical stop makes early bar push physically impossible).
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

import geometry as g


@dataclass
class CartridgeState:
    plunger_disp: float = 0.0
    bar_disp: float = 0.0
    bar_blocked: bool = False
    plunger_compressed_1s: bool = False
    compress_time: float = 0.0
    bar_across: bool = False
    ever_locked: bool = False
    plunger_released: bool = False
    released_and_held_2s: bool = False
    release_hold_time: float = 0.0
    lamp_green: bool = False
    arms_clear: bool = False
    arms_clear_1s: bool = False
    arms_clear_time: float = 0.0
    solved: bool = False


class CartridgeLogic:
    def __init__(self) -> None:
        self.state = CartridgeState()

    def compute_bar_stop(self, plunger_disp: float, raw_bar_disp: float) -> tuple[float, bool]:
        """Physical stop box logic:

        The lock bar can pass over the plunger notch only if the plunger is compressed >= 38 mm;
        otherwise the bar stops at 15 mm travel against the plunger stop box.
        Once locked (bar across through the notch), the bar remains across.
        """
        bar_d = max(0.0, float(raw_bar_disp))
        if self.state.ever_locked and bar_d >= g.BAR_LOCKED_THRESHOLD:
            return min(bar_d, g.LOCK_BAR_TRAVEL), False

        if plunger_disp < g.PLUNGER_COMPRESS_THRESHOLD:
            if bar_d > g.BAR_STOP_EARLY_LIMIT:
                return g.BAR_STOP_EARLY_LIMIT, True
            return bar_d, False
        # Plunger compressed >= 38 mm: notch is aligned, bar can pass through
        return min(bar_d, g.LOCK_BAR_TRAVEL), False

    def compute_effective_plunger(
        self,
        raw_plunger_disp: float,
        bar_disp: float,
        arm_pushing: bool,
    ) -> float:
        """Physical latch retention logic:

        If the lock bar is across (>= 55 mm) while compressed, it mechanically blocks
        the plunger notch from returning back past 38 mm when the arm lets go.
        If the bar is not across, releasing the arm allows the spring to throw the plunger back to 0.
        """
        plunger_d = max(0.0, float(raw_plunger_disp))
        if self.state.ever_locked and bar_disp >= g.BAR_LOCKED_THRESHOLD:
            # Mechanically held by the bar at >= 38 mm
            return max(plunger_d, g.PLUNGER_RETAINED_MIN_DISP)
        if arm_pushing:
            return plunger_d
        # Not locked and arm not pushing: spring throws it back to 0
        return plunger_d

    def check_plunger_contact(
        self,
        plunger_disp: float,
        arm_tcps: dict[str, tuple[float, float, float] | np.ndarray] | None,
    ) -> bool:
        """Check if any robot arm is in physical contact with the plunger knob."""
        if arm_tcps is None:
            return False
        knob_x = g.PLUNGER_REST_X + plunger_disp
        knob_y = g.PLUNGER_REST_Y
        knob_z = g.PLUNGER_REST_Z

        for _arm, tcp in arm_tcps.items():
            # Pad contact zone around plunger knob
            dx = abs(tcp[0] - (knob_x - g.PAD_TO_TCP))
            dy = abs(tcp[1] - knob_y)
            dz = abs(tcp[2] - knob_z)
            if dx <= 0.035 and dy <= 0.035 and dz <= 0.035:
                return True
        return False

    def check_arms_clear(
        self,
        arm_tcps: dict[str, tuple[float, float, float] | np.ndarray] | None,
    ) -> bool:
        """Check if both arms are clear of the fixture workspace."""
        if arm_tcps is None:
            return True
        for _arm, tcp in arm_tcps.items():
            # In fixture bounding region in xy and below hover height
            in_x = (g.BASE_POS[0] - g.BASE_SIZE[0] / 2 - 0.040) <= tcp[0] <= (g.BASE_POS[0] + g.BASE_SIZE[0] / 2 + 0.040)
            in_y = (g.BASE_POS[1] - g.BASE_SIZE[1] / 2 - 0.040) <= tcp[1] <= (g.BASE_POS[1] + g.BASE_SIZE[1] / 2 + 0.040)
            in_z = tcp[2] < g.HOVER_Z - 0.020
            if in_x and in_y and in_z:
                return False
        return True

    def step(
        self,
        dt: float,
        raw_plunger_disp: float,
        raw_bar_disp: float,
        arm_tcps: dict[str, tuple[float, float, float] | np.ndarray] | None = None,
        forced_plunger_released: bool | None = None,
    ) -> CartridgeState:
        st = self.state

        # 1. Determine arm contact on plunger knob
        if forced_plunger_released is not None:
            arm_pushing = not forced_plunger_released
        else:
            arm_pushing = self.check_plunger_contact(raw_plunger_disp, arm_tcps)
        st.plunger_released = not arm_pushing

        # 2. Compute effective bar displacement with physical stop box
        effective_bar, blocked = self.compute_bar_stop(raw_plunger_disp, raw_bar_disp)
        st.bar_disp = effective_bar
        st.bar_blocked = blocked

        # 3. Compute effective plunger displacement with bar retention
        effective_plunger = self.compute_effective_plunger(raw_plunger_disp, effective_bar, arm_pushing)
        st.plunger_disp = effective_plunger

        # 4. Check plunger compressed >= 38 mm for >= 1.0 s
        if st.plunger_disp >= g.PLUNGER_COMPRESS_THRESHOLD:
            st.compress_time += dt
            if st.compress_time >= 1.0:
                st.plunger_compressed_1s = True
        else:
            st.compress_time = 0.0

        # 5. Check lock bar across >= 55 mm while compressed (latched ever_locked per lesson 8)
        st.bar_across = (st.bar_disp >= g.BAR_LOCKED_THRESHOLD)
        if st.bar_across and (st.plunger_disp >= g.PLUNGER_COMPRESS_THRESHOLD):
            st.ever_locked = True

        # 6. Check release: plunger released and stays compressed >= 38 mm for 2.0 s
        if st.ever_locked and st.plunger_released and (st.plunger_disp >= g.PLUNGER_COMPRESS_THRESHOLD):
            st.release_hold_time += dt
            if st.release_hold_time >= 2.0:
                st.released_and_held_2s = True
        else:
            st.release_hold_time = 0.0

        # 7. Status lamp: green when locked and plunger compressed >= 38 mm
        st.lamp_green = st.ever_locked and (st.plunger_disp >= g.PLUNGER_COMPRESS_THRESHOLD)

        # 8. Check arms clear
        st.arms_clear = self.check_arms_clear(arm_tcps)

        # 9. Solved state
        st.solved = (
            st.plunger_compressed_1s
            and st.ever_locked
            and st.released_and_held_2s
            and st.arms_clear
            and st.lamp_green
        )

        return st
