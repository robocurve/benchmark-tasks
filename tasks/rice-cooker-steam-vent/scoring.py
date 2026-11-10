"""Measured-state start sequence; a blocked running vent permanently fails."""
import math


class TaskScorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.press_steps = self.clear_steps = self.rest_steps = 0
        self.started = self.unsafe_start_ever = self.invalid_state_ever = False
        self.max_button_travel_m = 0.
        self.last_step = self.last_result = None

    def update(self, *, step, vent_clear, towel_supported, released, stable,
               button_travel_m, valid=True):
        if step == self.last_step:
            return self.last_result
        valid = bool(valid and math.isfinite(button_travel_m)
                     and (self.last_step is None or step > self.last_step))
        self.last_step = step
        self.invalid_state_ever |= not valid
        if math.isfinite(button_travel_m):
            self.max_button_travel_m = max(self.max_button_travel_m, button_travel_m)
        self.press_steps = self.press_steps + 1 if valid and button_travel_m >= .005 else 0
        # A spring-return pushbutton does not physically prevent an unsafe start.
        # Its electronic running state latches after a sustained 5 mm press.
        self.started |= self.press_steps >= 3
        self.unsafe_start_ever |= self.started and not vent_clear
        unsafe = self.unsafe_start_ever or self.invalid_state_ever
        self.clear_steps = self.clear_steps + 1 if vent_clear and not unsafe else 0
        button_released = button_travel_m <= .002
        good = (self.started and vent_clear and towel_supported and released
                and stable and button_released and not unsafe)
        self.rest_steps = self.rest_steps + 1 if good else 0
        success = self.rest_steps >= 20
        score = (0. if unsafe else 1. if success else .5 if self.started and vent_clear
                 else .25 if self.clear_steps >= 6 else 0.)
        self.last_result = dict(score=score, success=success, started=self.started,
            vent_clear=bool(vent_clear), towel_supported=bool(towel_supported),
            released=bool(released), stable=bool(stable), button_released=bool(button_released),
            button_travel_m=float(button_travel_m), max_button_travel_m=self.max_button_travel_m,
            unsafe_start_ever=self.unsafe_start_ever, invalid_state_ever=self.invalid_state_ever,
            press_steps=self.press_steps, clear_steps=self.clear_steps, safe_hold_steps=self.rest_steps)
        return self.last_result

    @staticmethod
    def configuration():
        return dict(control_hz=20, button_press_m=.005, button_press_steps=3,
            button_released_m=.002, clear_dwell_steps=6, final_dwell_steps=20,
            vent_opening_radius_m=.016, horizontal_vent_clearance_margin_m=.010,
            maximum_rest_speed_m_s=.020, maximum_rest_angular_speed_rad_s=.20,
            tcp_towel_clearance_m=.055, tcp_button_clearance_m=.050,
            scores={"clear_vent":.25,"started_with_clear_vent":.5,
                    "started_supported_released_stable":1.,"unsafe_or_invalid_or_no_progress":0.},
            fidelity="Rigid folded-cloth surrogate; start is a logical latch after measured plunger travel. No heat, pressure, steam or cooking simulation.")
