"""Logic and state machine for two-key-authorization-hold task. Pure Python.

Hidden logic:
- Guards must be pushed outward >= 60 mm to expose keys.
- Both keys must be pressed at the same time to advance progress (1 lamp per 0.75 s).
- 3.0 s of simultaneous hold latches ARMED (green lamp).
- Releasing either key before 3.0 s resets progress to 0 and adds 1 strike (2 strikes = LOCKED, red lamp).
- After ARMED, CONFIRM must be pressed within 10.0 s before timeout locks the console.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import geometry as g


@dataclass
class TwoKeyState:
    guard_left_open: bool = False
    guard_right_open: bool = False
    both_guards_open: bool = False
    key_left_pressed: bool = False
    key_right_pressed: bool = False
    both_keys_held: bool = False
    simultaneous_hold_time: float = 0.0
    max_simultaneous_hold: float = 0.0
    progress_count: int = 0
    armed: bool = False
    armed_time: float | None = None
    confirm_pressed: bool = False
    strikes: int = 0
    locked: bool = False
    solved: bool = False
    fault_lamp: bool = False
    re_armed: dict[str, bool] = field(default_factory=lambda: {b: True for b in g.BUTTONS})


class TwoKeyLogic:
    def __init__(self) -> None:
        self.state = TwoKeyState()
        self.sim_time = 0.0

    def step(
        self,
        dt: float,
        button_displacements: dict[str, float],
        guard_displacements: dict[str, float],
    ) -> TwoKeyState:
        self.sim_time += dt
        cur_t = self.sim_time
        st = self.state

        # 1. Update guards
        gl_disp = guard_displacements.get("guard_left", 0.0)
        gr_disp = guard_displacements.get("guard_right", 0.0)
        st.guard_left_open = (abs(gl_disp) >= g.GUARD_OPEN_THRESHOLD)
        st.guard_right_open = (abs(gr_disp) >= g.GUARD_OPEN_THRESHOLD)
        st.both_guards_open = (st.guard_left_open and st.guard_right_open)

        # 2. Check key pressing
        kl_disp = button_displacements.get("key_left", 0.0)
        kr_disp = button_displacements.get("key_right", 0.0)
        st.key_left_pressed = (kl_disp <= g.BUTTON_PRESS_THRESHOLD and st.guard_left_open)
        st.key_right_pressed = (kr_disp <= g.BUTTON_PRESS_THRESHOLD and st.guard_right_open)
        st.both_keys_held = (st.key_left_pressed and st.key_right_pressed)

        # 3. Check lockouts and arming
        if not st.locked and not st.confirm_pressed:
            if st.armed:
                # Check 10 s timeout
                if st.armed_time is not None and (cur_t - st.armed_time) > 10.0:
                    st.armed = False
                    st.locked = True
                    st.fault_lamp = True
            else:
                # Pre-armed state
                if st.both_keys_held:
                    st.simultaneous_hold_time += dt
                    st.max_simultaneous_hold = max(st.max_simultaneous_hold, st.simultaneous_hold_time)
                    st.progress_count = min(4, int((st.simultaneous_hold_time + 1e-4) // 0.75))
                    if st.simultaneous_hold_time >= 3.0 - 1e-4:
                        st.armed = True
                        st.armed_time = cur_t
                        st.progress_count = 4
                else:
                    if st.simultaneous_hold_time > 0.0:
                        # Released before 3.0 s -> strike and reset
                        st.strikes += 1
                        st.simultaneous_hold_time = 0.0
                        st.progress_count = 0
                        if st.strikes >= 2:
                            st.locked = True
                            st.fault_lamp = True

        # 4. Check CONFIRM button press
        for b in g.BUTTONS:
            z_disp = button_displacements.get(b, 0.0)
            if z_disp >= g.BUTTON_RELEASE_THRESHOLD:
                st.re_armed[b] = True

            if z_disp <= g.BUTTON_PRESS_THRESHOLD and st.re_armed[b]:
                st.re_armed[b] = False
                if b == "confirm":
                    if st.armed and not st.locked:
                        st.confirm_pressed = True
                        st.solved = True

        return st
