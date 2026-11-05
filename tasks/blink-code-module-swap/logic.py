"""Hidden-state logic for the blink-code module swap task.

Pure Python, no simulator dependencies. Fully unit-tested.

Rules:
- Four sockets: 0, 1, 2, 3 (marked by 1, 2, 3, 4 white dots).
- Failed socket: index 2 (socket 3, 3 dots).
- Modules: 'module_1', 'module_2', 'module_3' (failed), 'module_4', 'spare' (teal).
- Sockets start with module_1 in 0, module_2 in 1, module_3 in 2, module_4 in 3.
- DIAG button (8 mm travel):
  Held down when joint position <= -0.0064 m (-6.4 mm, 80% travel).
  Released when joint position >= -0.0035 m (-3.5 mm, hysteresis).
- Status lamp behavior:
  - If DIAG is released (not held): lamp is 'off', blink sequence resets to step 0.
  - While DIAG is held:
    - If any healthy module ('module_1' in 0, 'module_2' in 1, 'module_4' in 3) is NOT seated:
      steady 'red' lamp.
    - Else if failed module swap is complete ('spare' seated in socket 2 AND 'module_3' not in any socket):
      steady 'green' lamp (verified).
    - Else (healthy modules seated, swap not yet complete):
      blink pattern: N = 3 pulses (0.3 s on = 6 steps, 0.3 s off = 6 steps at 20 Hz),
      followed by 1.5 s gap (30 steps off). Repeating period = 66 steps (3.3 s).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

ALL_MODULES: tuple[str, ...] = ("module_1", "module_2", "module_3", "module_4", "spare")
HEALTHY_MODULES: tuple[str, ...] = ("module_1", "module_2", "module_4")
FAILED_MODULE: str = "module_3"
SPARE_MODULE: str = "spare"
FAILED_SOCKET: int = 2  # socket 3 (0-indexed 2, 3 dots)

PRESS_THRESHOLD = -0.0064    # <= -6.4 mm
RELEASE_THRESHOLD = -0.0035  # >= -3.5 mm

# Blink timing at 20 Hz:
# 3 pulses: each 6 steps ON (0.3 s), 6 steps OFF (0.3 s)
# 1.5 s gap: 30 steps OFF
# Period = 3 * 12 + 30 = 66 steps (3.3 s)
PULSE_ON_STEPS = 6
PULSE_OFF_STEPS = 6
GAP_OFF_STEPS = 30
PULSE_COUNT = 3
BLINK_PERIOD = PULSE_COUNT * (PULSE_ON_STEPS + PULSE_OFF_STEPS) + GAP_OFF_STEPS  # 66


@dataclass(frozen=True)
class LogicState:
    diag_held: bool
    held_steps: int
    lamp_state: str  # "off", "blink", "green", "red"
    swap_completed: bool
    healthy_seated: bool
    diag_held_ge_3s: bool
    green_verified: bool


class BlinkCodeLogic:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._diag_held: bool = False
        self._held_steps: int = 0
        self._blink_step: int = 0
        self._green_held_steps: int = 0
        self._diag_held_ge_3s: bool = False
        self._green_verified: bool = False

    def update(
        self,
        seated_in_socket: Mapping[str, int | None],
        diag_slider_q: float,
    ) -> LogicState:
        """Update logic with current module socket assignments and DIAG button slider position (m, <= 0)."""
        # 1. Update button hold state with hysteresis
        if diag_slider_q <= PRESS_THRESHOLD:
            self._diag_held = True
        elif diag_slider_q >= RELEASE_THRESHOLD:
            self._diag_held = False

        # 2. Check healthy modules and swap completion
        healthy_seated = (
            seated_in_socket.get("module_1") == 0
            and seated_in_socket.get("module_2") == 1
            and seated_in_socket.get("module_4") == 3
        )

        swap_completed = (
            healthy_seated
            and seated_in_socket.get(SPARE_MODULE) == FAILED_SOCKET
            and seated_in_socket.get(FAILED_MODULE) is None
        )

        # 3. Determine lamp state and counters
        if not self._diag_held:
            lamp_state = "off"
            self._held_steps = 0
            self._blink_step = 0
            self._green_held_steps = 0
        else:
            self._held_steps += 1
            if self._held_steps >= 60:  # 3.0 s at 20 Hz
                self._diag_held_ge_3s = True

            if not healthy_seated:
                lamp_state = "red"
                self._green_held_steps = 0
            elif swap_completed:
                lamp_state = "green"
                self._green_held_steps += 1
                if self._green_held_steps >= 60:  # 3.0 s showing green
                    self._green_verified = True
            else:
                self._green_held_steps = 0
                phase = self._blink_step % BLINK_PERIOD
                self._blink_step += 1
                # Pulses: 0..5 on, 6..11 off, 12..17 on, 18..23 off, 24..29 on, 30..35 off, 36..65 gap off
                if phase in range(0, 6) or phase in range(12, 18) or phase in range(24, 30):
                    lamp_state = "blink"
                else:
                    lamp_state = "off"

        return LogicState(
            diag_held=self._diag_held,
            held_steps=self._held_steps,
            lamp_state=lamp_state,
            swap_completed=swap_completed,
            healthy_seated=healthy_seated,
            diag_held_ge_3s=self._diag_held_ge_3s,
            green_verified=self._green_verified,
        )
