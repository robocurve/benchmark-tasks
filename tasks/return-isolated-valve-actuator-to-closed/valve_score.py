"""Shared causal scorer. All inputs are measured; no policy name is an input."""

from collections import deque
from math import isfinite

from valve_layout import PIN_CLEAR


class Trial:
    def __init__(self):
        self.tick = 0
        self.reason = None
        self.loaded = False
        self.reset_load = deque(maxlen=24)
        self.wheel = None
        self.relief_ticks = 0
        self.started = False
        self.extracted = False
        self.closed = False
        self.slow = True
        self.unsupported_ticks = 0
        self.controlled = True
        self.dwell = 0
        self.score = 0.0
        self.peak_lift = 0.0
        self.peak_wrist = 0.0
        self.saw_relief = False
        self.saw_guidance = False

    def observe(self, r):
        self.tick += 1
        numeric = (
            "angle",
            "speed",
            "lift",
            "pin_offset",
            "pin_tilt",
            "pin_force",
            "human_force",
            "collision",
            "wrist_mm",
            "memory_mm",
            "pin_speed",
            "pin_spin",
        )
        if (
            r["tick"] != self.tick
            or not all(isfinite(r[k]) for k in numeric)
            or not all(isfinite(v) for v in r["jaw"])
        ):
            raise ValueError("Missing, reordered or nonfinite measurement")
        angle = r["angle"]
        speed = abs(r["speed"])
        lift = r["lift"]
        self.peak_lift = max(self.peak_lift, lift)
        self.peak_wrist = max(self.peak_wrist, r["wrist_mm"])

        def fault(reason):
            if self.reason is None:
                self.reason = reason

        if self.tick <= 24:
            self.reset_load.append(r["pin_force"])
            if (
                abs(angle - 45) > 3
                or abs(lift) > 0.003
                or abs(r["wrist_mm"]) > 1
                or abs(r["memory_mm"]) > 1
            ):
                fault("invalid_reset_pose")
        if self.tick == 24:
            self.loaded = sum(self.reset_load) / 24 > 8
            if not self.loaded:
                raise ValueError("Missing loaded pin contact at reset")
        if r["human_force"] > 1 or r["wrist_mm"] >= 10:
            fault("surrogate_contact")
        if r["collision"] > 5:
            fault("unintended_collision")
        if speed > 45:
            fault("shaft_overspeed")
        if angle > 50:
            fault("override_overwind")
        if r["pin_lost"]:
            fault("pin_outside_tray")
        bilateral = [i for i, ok in enumerate(r["wheel_grip"]) if ok]
        if not self.started:
            valid = (
                self.loaded
                and bool(bilateral)
                and r["pin_force"] < 0.5
                and abs(angle - 45) <= 3
                and lift < 0.003
            )
            if valid:
                if self.wheel != bilateral[0]:
                    self.relief_ticks = 0
                self.wheel = bilateral[0]
                self.relief_ticks += 1
                self.saw_relief |= self.relief_ticks >= 12
            elif lift < 0.003:
                self.relief_ticks = 0
            if lift >= 0.003:
                self.started = True
                if self.relief_ticks < 12:
                    fault("pin_moved_under_load")
        if self.started and not self.extracted:
            if (
                abs(angle - 45) > 3
                or r["pin_tilt"] > 12
                or (lift < PIN_CLEAR and r["pin_offset"] > 0.006)
            ):
                fault("uncontrolled_extraction")
            if (
                self.wheel is None
                or not r["wheel_grip"][self.wheel]
                or not r["pin_grip"][1 - self.wheel]
            ):
                fault("unsupported_extraction")
            if lift >= PIN_CLEAR:
                self.extracted = True
        if self.extracted:
            self.slow &= speed <= 30
            held = self.wheel is not None and r["wheel_grip"][self.wheel]
            if 4 < angle < 43 and not held:
                self.unsupported_ticks += 1
                if self.unsupported_ticks >= 12:
                    self.controlled = False
            else:
                self.unsupported_ticks = 0
            if (
                self.wheel is not None
                and r["wheel_grip"][self.wheel]
                and 8 < angle < 35
            ):
                self.saw_guidance = True
            self.closed = (
                self.saw_guidance
                and self.controlled
                and self.slow
                and abs(angle) <= 3
                and speed < 4
            )
        ready = (
            self.closed
            and r["on_tray"]
            and r["pin_speed"] < 0.015
            and r["pin_spin"] < 0.2
            and min(r["jaw"]) >= 0.025
            and r["clear"]
            and not r["touching"]
        )
        self.dwell = self.dwell + 1 if ready else 0
        if self.reason:
            self.score = 0.0
        elif self.dwell >= 240:
            self.score = 1.0
        elif self.closed:
            self.score = 0.75
        elif self.extracted:
            self.score = 0.50
        elif self.relief_ticks >= 12:
            self.score = 0.25
        else:
            self.score = 0.0
        return self.score
