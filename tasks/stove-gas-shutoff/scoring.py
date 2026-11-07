"""History-aware numeric scoring; no simulator imports or policy-visible state."""
import math

PRESS_M = .008
PRESS_STEPS = 3                  # 0.15 s at 20 Hz
IGNITION_DEG = 20.0
IGNITION_STEPS = 3
SUCCESS_STEPS = 20               # 1 s after latching


class TaskScorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.presses = self.safe_steps = 0
        self.knob_counts = [0, 0]
        self.latched = self.ignition_ever = self.invalid_ever = False

    def update(self, travel_m, knob_degrees, *, valid=True):
        finite = math.isfinite(travel_m) and len(knob_degrees) == 2 and all(map(math.isfinite, knob_degrees))
        self.invalid_ever |= not valid or not finite
        if finite and valid:
            self.presses = self.presses + 1 if travel_m >= PRESS_M else 0
            self.latched |= self.presses >= PRESS_STEPS
            for i, angle in enumerate(knob_degrees):
                self.knob_counts[i] = self.knob_counts[i] + 1 if abs(angle) >= IGNITION_DEG else 0
            self.ignition_ever |= max(self.knob_counts) >= IGNITION_STEPS
        self.safe_steps = self.safe_steps + 1 if self.latched and not self.ignition_ever and not self.invalid_ever else 0
        success = self.safe_steps >= SUCCESS_STEPS
        score = 0.0 if self.invalid_ever else (0.2 if self.ignition_ever else (1.0 if success else 0.0))
        return dict(score=score, success=success, gas_shutoff_latched=self.latched,
                    ignition_ever=self.ignition_ever, invalid_state_ever=self.invalid_ever,
                    button_travel_m=float(travel_m), knob_angles_deg=list(map(float,knob_degrees)),
                    safe_hold_steps=self.safe_steps)

    @staticmethod
    def configuration():
        return dict(press_threshold_m=PRESS_M,press_dwell_steps=PRESS_STEPS,
                    ignition_angle_deg=IGNITION_DEG,ignition_dwell_steps=IGNITION_STEPS,
                    safe_hold_steps=SUCCESS_STEPS,control_hz=20,
                    safe_shutoff_score=1.0,ignition_score=.2,otherwise_score=0.0,
                    ignition_history_caps_score=True,invalid_state_score=0.0)
