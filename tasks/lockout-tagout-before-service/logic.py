"""State machine and verification logic for lockout-tagout-before-service. Pure Python.

Tracks:
- Breaker lever displacement (0 to 40 mm).
- Lock pin position and seating in hole (within 4 mm).
- TEST button press and DEAD lamp (3.0 s timer).
- Cover sliding displacement (0 to 90 mm).
- SERVICE button press under cover.
- Safety sequence enforcement:
  FAULT triggered if cover opened >= 20 mm before:
    1. Breaker is OFF
    2. Pin is seated
    3. TEST showed DEAD
- Zero-gate: FAULT latches score to 0.0.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
import numpy as np

import geometry as g


@dataclass
class LotoState:
    breaker_off: bool = False
    pin_seated: bool = False
    test_pressed_dead: bool = False
    dead_lamp_timer: float = 0.0
    dead_lamp_lit: bool = False
    live_lamp_lit: bool = True
    cover_open_70mm: bool = False
    service_pressed: bool = False
    service_hold_time: float = 0.0
    fault: bool = False
    fault_reason: str = ""
    solved: bool = False


class LotoLogic:
    def __init__(self) -> None:
        self.state = LotoState()

    def step(
        self,
        dt: float,
        breaker_disp: float,    # 0.0 = ON, 0.040 = OFF
        pin_pos: tuple[float, float, float] | np.ndarray,
        test_btn_disp: float,   # negative when pressed, <= -0.0064 m
        cover_disp: float,      # 0.0 = closed, >= 0.070 = open
        service_btn_disp: float,# negative when pressed, <= -0.0064 m
        arm_tcps: dict[str, tuple[float, float, float] | np.ndarray] | None = None,
    ) -> LotoState:
        st = self.state
        if st.fault:
            return st

        # 1. Breaker status and LIVE lamp
        if breaker_disp >= g.BREAKER_OFF_THRESHOLD:
            st.breaker_off = True
            st.live_lamp_lit = False
        else:
            # If pin is seated, breaker cannot be switched back ON
            if st.pin_seated:
                # Physically blocked by pin
                pass
            else:
                st.breaker_off = False
                st.live_lamp_lit = True

        # 2. Red lock pin seating
        # Pin hole is at (g.HOLE_POS[0], g.HOLE_POS[1]). Pin seated when within 4 mm of bottom.
        # Pin cannot be seated if breaker is still ON (hole is covered by lever)
        d_hole_xy = math.hypot(pin_pos[0] - g.HOLE_POS[0], pin_pos[1] - g.HOLE_POS[1])
        if d_hole_xy <= 0.010 and pin_pos[2] <= g.PIN_SEATED_Z + g.PIN_SEATED_THRESHOLD_M:
            if st.breaker_off:
                st.pin_seated = True
            else:
                # Attempted to insert pin while breaker ON -> hole blocked
                pass

        # 3. TEST button and DEAD lamp
        if test_btn_disp <= -0.0064:
            if st.breaker_off:
                st.test_pressed_dead = True
                st.dead_lamp_timer = 3.0  # lights for 3.0 s
            else:
                # TEST pressed while breaker ON -> FAULT (power live)
                st.fault = True
                st.fault_reason = "test_pressed_live"
                return st

        if st.dead_lamp_timer > 0.0:
            st.dead_lamp_timer = max(0.0, st.dead_lamp_timer - dt)
            st.dead_lamp_lit = True
        else:
            st.dead_lamp_lit = False

        # 4. Service cover and FAULT checking
        st.cover_open_70mm = (cover_disp >= g.COVER_OPEN_MIN)

        # FAULT check: cover opened >= 20 mm while prerequisites not met
        if cover_disp >= g.COVER_FAULT_OPEN:
            if not st.breaker_off:
                st.fault = True
                st.fault_reason = "cover_opened_breaker_on"
                return st
            if not st.pin_seated:
                st.fault = True
                st.fault_reason = "cover_opened_pin_not_seated"
                return st
            if not st.test_pressed_dead:
                st.fault = True
                st.fault_reason = "cover_opened_before_test"
                return st

        # 5. SERVICE button under cover
        if service_btn_disp <= -0.0064 and st.cover_open_70mm and st.pin_seated and not st.fault:
            st.service_hold_time += dt
            if st.service_hold_time >= 1.0:
                st.service_pressed = True

        st.solved = (
            st.breaker_off
            and st.pin_seated
            and st.test_pressed_dead
            and st.service_pressed
            and not st.fault
        )
        return st

