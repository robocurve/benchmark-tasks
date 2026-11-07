"""State machine and safety verification logic for estop-before-clearing-jam. Pure Python.

Tracks:
- Pusher sweep motion (code-driven non-colliding bar sweeping over 90 mm, period 4.0 s).
- E-STOP button press and pusher arrest (within 0.3 s, latched ever_stopped).
- Block status (stuck in lane danger zone vs extracted into green tray / parked pad).
- RESET button press (arms restart, ignored if pressed before E-STOP).
- START button press (restarts pusher if block cleared; FAULT if block still in lane; ignored if RESET skipped).
- Safety sequence enforcement:
  - FAULT (zero-gate): any arm TCP entering the danger zone while pusher is moving.
  - FAULT (zero-gate): START pressed while block is still in lane.
- Lamp states:
  - RUN (green): lit while pusher is moving and no fault.
  - STOP (red): lit when pusher has stopped safely after E-STOP.
  - FAULT (amber): lit when a safety fault has been triggered.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping, Sequence
import numpy as np

import geometry as g


@dataclass
class JamClearState:
    pusher_moving: bool = True
    ever_stopped: bool = False
    pusher_stopping: bool = False
    stop_timer: float = 0.0
    pusher_time: float = 0.0
    pusher_x: float = g.PUSHER_X_MIN

    estop_pressed: bool = False
    reset_pressed: bool = False
    start_pressed: bool = False

    block_in_lane: bool = True
    block_in_tray: bool = False
    block_on_pad: bool = False
    block_cleared: bool = False

    run_lamp_lit: bool = True
    stop_lamp_lit: bool = False
    fault_lamp_lit: bool = False

    fault: bool = False
    fault_reason: str = ""
    restarted: bool = False
    solved: bool = False


class JamClearLogic:
    def __init__(self) -> None:
        self.state = JamClearState()

    def step(
        self,
        dt: float,
        estop_disp: float,    # negative when pressed, <= -0.0064 m
        reset_disp: float,    # negative when pressed, <= -0.0064 m
        start_disp: float,    # negative when pressed, <= -0.0064 m
        block_pos: tuple[float, float, float] | np.ndarray,
        arm_tcps: Mapping[str, tuple[float, float, float] | np.ndarray] | None = None,
    ) -> JamClearState:
        st = self.state
        if st.fault:
            st.run_lamp_lit = False
            st.stop_lamp_lit = False
            st.fault_lamp_lit = True
            return st

        # 1. Update block location
        b_pos = tuple(float(v) for v in block_pos[:3])
        st.block_in_lane = g.in_danger_zone(b_pos)
        st.block_in_tray = g.in_tray(b_pos)
        st.block_on_pad = g.in_parked_pad(b_pos)
        st.block_cleared = (not st.block_in_lane) and (st.block_in_tray or st.block_on_pad)

        # 2. Check E-STOP actuation
        if estop_disp <= g.BUTTON_PRESS_THRESHOLD:
            st.estop_pressed = True
            if st.pusher_moving and not st.pusher_stopping:
                st.pusher_stopping = True
                st.stop_timer = 0.0

        # 3. Pusher stopping / sweeping update
        if st.pusher_stopping and st.pusher_moving:
            st.stop_timer += dt
            if st.stop_timer >= 0.30:  # stops within 0.3 s
                st.pusher_moving = False
                st.ever_stopped = True
                st.pusher_stopping = False
                st.run_lamp_lit = False
                st.stop_lamp_lit = True

        if st.pusher_moving:
            st.pusher_time += dt
            st.pusher_x = g.pusher_x_at_time(st.pusher_time)

        # 4. Danger zone hazard check: TCP in danger zone while pusher is moving = FAULT
        if st.pusher_moving and arm_tcps is not None:
            for arm_name, tcp in arm_tcps.items():
                tcp_pt = tuple(float(v) for v in tcp[:3])
                if g.in_danger_zone(tcp_pt):
                    st.fault = True
                    st.fault_reason = f"tcp_in_danger_zone_while_moving_{arm_name}"
                    st.run_lamp_lit = False
                    st.stop_lamp_lit = False
                    st.fault_lamp_lit = True
                    return st

        # 5. RESET button actuation
        # RESET before E-STOP has no effect. Can only arm reset after E-STOP has stopped the pusher.
        if reset_disp <= g.BUTTON_PRESS_THRESHOLD:
            if st.ever_stopped and not st.pusher_moving and not st.fault:
                st.reset_pressed = True

        # 6. START button actuation
        if start_disp <= g.BUTTON_PRESS_THRESHOLD:
            if not st.reset_pressed:
                # START without RESET has no effect
                pass
            else:
                # RESET was pressed; check if lane is clear
                if st.block_in_lane:
                    # START pressed with block still in lane = FAULT
                    st.fault = True
                    st.fault_reason = "start_pressed_with_block_in_lane"
                    st.run_lamp_lit = False
                    st.stop_lamp_lit = False
                    st.fault_lamp_lit = True
                    return st
                else:
                    # Safe restart!
                    st.start_pressed = True
                    st.pusher_moving = True
                    st.restarted = True
                    st.run_lamp_lit = True
                    st.stop_lamp_lit = False
                    st.fault_lamp_lit = False

        # 7. Check arm clearance for solved status
        arms_clear = True
        if arm_tcps is not None:
            for arm_name, tcp in arm_tcps.items():
                tcp_pt = tuple(float(v) for v in tcp[:3])
                if g.in_danger_zone(tcp_pt):
                    arms_clear = False

        st.solved = (
            st.ever_stopped
            and st.block_cleared
            and st.restarted
            and not st.fault
            and arms_clear
        )
        return st
