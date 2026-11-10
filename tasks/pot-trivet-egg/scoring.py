"""Measured-state task score. Egg/pot contact is a conservative damage proxy."""
import math


class TaskScorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.clear_steps = self.place_steps = self.rest_steps = 0
        self.contact_ever = self.invalid_ever = False
        self.max_pot_egg_force_n = 0.
        self.last_step = None
        self.last_result = None

    def update(self, *, step, egg_clear, egg_supported, pot_on_trivet, released,
               stable, pot_egg_force_n, valid=True):
        if step == self.last_step:
            return self.last_result
        valid = bool(valid and math.isfinite(pot_egg_force_n)
                     and (self.last_step is None or step > self.last_step))
        self.last_step = step
        self.invalid_ever |= not valid
        if math.isfinite(pot_egg_force_n):
            self.max_pot_egg_force_n = max(self.max_pot_egg_force_n, pot_egg_force_n)
            self.contact_ever |= pot_egg_force_n > .20
        unsafe = self.contact_ever or self.invalid_ever
        clear = egg_clear and not unsafe
        placed = clear and pot_on_trivet
        good = placed and egg_supported and released and stable
        self.clear_steps = self.clear_steps + 1 if clear else 0
        self.place_steps = self.place_steps + 1 if placed else 0
        self.rest_steps = self.rest_steps + 1 if good else 0
        success = self.rest_steps >= 20
        score = (0. if unsafe else 1. if success else .5 if self.place_steps >= 10
                 else .25 if self.clear_steps >= 6 else 0.)
        self.last_result = dict(
            score=score, success=success, egg_clear=bool(egg_clear),
            egg_supported=bool(egg_supported), pot_on_trivet=bool(pot_on_trivet),
            released=bool(released), stable=bool(stable),
            pot_contact_ever=self.contact_ever, invalid_state_ever=self.invalid_ever,
            pot_egg_force_n=float(pot_egg_force_n),
            max_pot_egg_force_n=self.max_pot_egg_force_n,
            clear_steps=self.clear_steps, place_steps=self.place_steps,
            safe_hold_steps=self.rest_steps,
        )
        return self.last_result

    @staticmethod
    def configuration():
        return dict(
            control_hz=20, cleared_dwell_steps=6, placed_dwell_steps=10,
            success_dwell_steps=20, trivet_full_base_support_radius_m=.1058,
            maximum_pot_tilt_deg=12, supported_height_tolerance_m=[-.003, .006],
            maximum_rest_speed_m_s=.020, maximum_rest_angular_speed_rad_s=.20,
            tcp_object_bounds_clearance_m=.055, pot_egg_contact_threshold_n=.20,
            contact_history_physics_steps=12,
            scores={"safe_egg_clearance": .25, "pot_supported_on_cleared_trivet": .5,
                    "egg_safe_and_both_objects_released_stable": 1.,
                    "otherwise_or_latched_contact_or_invalid": 0.},
            fidelity="Pot/egg contact is a conservative damage proxy; shell fracture and gripper crush are not simulated.",
        )
