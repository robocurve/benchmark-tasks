"""Hidden-state logic for the buffer socket module permutation task.

Pure Python, no simulator dependencies. Fully unit-tested.

Rules:
- Four sockets: 0, 1, 2, 3 (sockets 1..4).
  - Socket 0 (Socket 1): requires 'module_red'
  - Socket 1 (Socket 2): requires 'module_green'
  - Socket 2 (Socket 3): requires 'module_blue'
  - Socket 3 (Socket 4): buffer (no required module)
- Three modules: 'module_red', 'module_green', 'module_blue'.
  - Socket 0 starts with 'module_green' (mis-racked)
  - Socket 1 starts with 'module_red' (mis-racked)
  - Socket 2 starts with 'module_blue' (matching)
  - Socket 3 starts empty (buffer)
- COMMIT button (8 mm travel):
  - Pressed when slider displacement <= -0.0064 m (-6.4 mm, 80% travel)
  - Released when slider displacement >= -0.0035 m (-3.5 mm, hysteresis)
- STATUS lamp:
  - 'green' when all 3 coloured sockets hold matching module AND all modules are seated (none held/displaced)
  - 'red' otherwise (or while any module is being held/unseated/mismatched)
- Valid COMMIT:
  - Registered when COMMIT button is pressed while STATUS lamp is 'green'.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

MODULE_NAMES: tuple[str, ...] = ("module_red", "module_green", "module_blue")
TARGET_SOCKETS: dict[str, int] = {
    "module_red": 0,
    "module_green": 1,
    "module_blue": 2,
}

PRESS_THRESHOLD = -0.0064    # <= -6.4 mm
RELEASE_THRESHOLD = -0.0035  # >= -3.5 mm


@dataclass(frozen=True)
class LogicState:
    commit_held: bool
    lamp_state: str  # "green" or "red"
    all_three_matching: bool
    matching_swapped_count: int  # 0, 1, or 2 (red in 0, green in 1)
    matching_total_count: int    # 0..3 (including blue in 2)
    commit_pressed_valid: bool   # latched: COMMIT was pressed while STATUS lamp was green


class PermutationLogic:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._commit_held: bool = False
        self._commit_pressed_valid: bool = False

    def update(
        self,
        seated_in_socket: Mapping[str, int | None],
        commit_slider_q: float,
        any_displaced: bool = False,
    ) -> LogicState:
        """Update logic state with module socket locations and COMMIT slider position."""
        # 1. Update button hold state with hysteresis
        if commit_slider_q <= PRESS_THRESHOLD:
            self._commit_held = True
        elif commit_slider_q >= RELEASE_THRESHOLD:
            self._commit_held = False

        # 2. Check matching modules
        red_matching = seated_in_socket.get("module_red") == TARGET_SOCKETS["module_red"]
        green_matching = seated_in_socket.get("module_green") == TARGET_SOCKETS["module_green"]
        blue_matching = seated_in_socket.get("module_blue") == TARGET_SOCKETS["module_blue"]

        matching_swapped_count = (1 if red_matching else 0) + (1 if green_matching else 0)
        matching_total_count = matching_swapped_count + (1 if blue_matching else 0)

        all_three_matching = (
            red_matching
            and green_matching
            and blue_matching
            and not any_displaced
        )

        # 3. Determine STATUS lamp state:
        # Green when all 3 coloured sockets hold matching module AND no other module displaced > 5 mm.
        # Red otherwise (or while any module is being held/unseated).
        lamp_state = "green" if all_three_matching else "red"

        # 4. Check for valid COMMIT press:
        # Counted if the button is currently pressed and the status lamp is green
        if self._commit_held and lamp_state == "green":
            self._commit_pressed_valid = True

        return LogicState(
            commit_held=self._commit_held,
            lamp_state=lamp_state,
            all_three_matching=all_three_matching,
            matching_swapped_count=matching_swapped_count,
            matching_total_count=matching_total_count,
            commit_pressed_valid=self._commit_pressed_valid,
        )
