"""Numeric safe-method score sampled at 20 Hz from physical state."""
import math


class TaskScorer:
    def __init__(self):self.reset()

    def reset(self):
        self.closed_steps=self.press_steps=self.lift_steps=self.bath_steps=self.rest_steps=0
        self.microwave_started=self.contaminated_ever=self.invalid_ever=self.press_consumed=False
        self.start_events=0

    def update(self,*,door_angle_deg,button_travel_m,bottle_in_bath,bottle_supported,released,
               lifted,neck_submerged,stable,valid=True):
        valid=bool(valid and math.isfinite(door_angle_deg) and math.isfinite(button_travel_m))
        self.invalid_ever |= not valid
        self.contaminated_ever |= bool(neck_submerged)
        closed=valid and abs(door_angle_deg)<=3
        self.closed_steps=self.closed_steps+1 if closed else 0
        if button_travel_m<.002:self.press_consumed=False
        pressing=closed and self.closed_steps>=10 and button_travel_m>=.004
        self.press_steps=self.press_steps+1 if pressing else 0
        if self.press_steps>=3 and not self.press_consumed:
            self.microwave_started=True;self.press_consumed=True;self.start_events+=1
        unsafe=self.microwave_started or self.contaminated_ever or self.invalid_ever
        self.lift_steps=self.lift_steps+1 if lifted and valid and not unsafe else 0
        self.bath_steps=self.bath_steps+1 if bottle_in_bath and valid and not unsafe else 0
        good=bottle_in_bath and bottle_supported and released and stable and valid and not unsafe
        self.rest_steps=self.rest_steps+1 if good else 0
        success=self.rest_steps>=20
        score=0. if unsafe else 1. if success else .5 if self.bath_steps>=10 else .25 if self.lift_steps>=6 else 0.
        return dict(score=score,success=success,microwave_started=self.microwave_started,
            contaminated_ever=self.contaminated_ever,invalid_state_ever=self.invalid_ever,
            bottle_in_bath=bool(bottle_in_bath),bottle_supported=bool(bottle_supported),released=bool(released),
            stable=bool(stable),door_angle_deg=float(door_angle_deg),button_travel_m=float(button_travel_m),
            start_events=self.start_events,lift_steps=self.lift_steps,bath_steps=self.bath_steps,safe_hold_steps=self.rest_steps)

    @staticmethod
    def configuration():
        return dict(control_hz=20,lift_height_m=.030,lift_dwell_steps=6,
            bath_xy_radius_m=.050,max_bath_tilt_deg=15,min_immersion_m=.030,
            min_neck_water_clearance_m=.015,bath_dwell_steps=10,
            supported_floor_tolerance_m=[-.003,.006],max_rest_speed_m_s=.020,
            max_rest_angular_speed_rad_s=.15,tcp_axis_clearance_m=.100,full_hold_steps=20,
            microwave_closed_angle_deg=3,microwave_closed_dwell_steps=10,start_press_m=.004,start_press_steps=3,
            scores={'safe_lift':.25,'upright_in_bath':.5,'supported_released_stable_in_bath':1.,'otherwise_or_unsafe':0.})
