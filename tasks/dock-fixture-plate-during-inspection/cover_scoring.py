"""Docking a staged plate at an occupied receiver. Sample once per physics substep."""

from cover_geometry import REST_Z, PIN_HEIGHT, K


class Trial:
    def __init__(self):
        self.tick = self.dwell = self.hover_ticks = 0
        self.flags = [False, False, False, False]
        self.reason = ""
        self.score = 0.0
        self.extracted = self.reset_valid = self.lifted = False
        self.reached_hover = False
        self.loaded_ticks = 0
        self.stage_seen = [False, False]

    def observe(self, r):
        if r["tick"] != self.tick + 1:
            raise ValueError("Lost substep")
        self.tick += 1
        occupied = min(r["limb_gap"], r["gauge_gap"]) < 0
        clear = min(r["limb_gap"], r["gauge_gap"]) >= 0.010
        if self.tick <= 24:
            if (
                r["stage_error"] > 0.010
                or not occupied
                or max(r["finger_load"]) > 1.0
                or max(r["stroke"]) >= 0.001
            ):
                raise ValueError(
                    "Invalid reset: plate must start staged, hand unloaded and receiver occupied"
                )
            for i, f in enumerate(r["stage_loads"]):
                self.stage_seen[i] |= f > 1.0
            if self.tick == 24:
                if not all(self.stage_seen):
                    raise ValueError("No bilateral staging support at reset")
                self.reset_valid = True
        for i, stroke in enumerate(r["stroke"]):
            self.flags[i] |= stroke >= 15 / K
        self.flags[2] |= r["skin_N"] >= 4
        self.lifted |= (
            self.reset_valid
            and r["bottom"] >= REST_Z + PIN_HEIGHT + 0.035
            and max(r["stage_loads"]) < 1
        )
        self.extracted |= clear
        registered = [
            depth >= 0.004 and margin >= -0.00030
            for depth, margin in zip(r["pin_depth"], r["pin_margin"])
        ]
        # An initially unloaded hand taking sustained plate weight is an observable
        # premature handoff even when neither independent overload switch trips.
        loading_hand = any(
            d >= 0.003 and f >= 1 for d, f in zip(r["stroke"], r["finger_load"])
        )
        self.loaded_ticks = self.loaded_ticks + 1 if loading_hand else 0
        self.flags[3] |= self.loaded_ticks >= 12 or (occupied and any(registered))
        if not self.reason:
            if any(self.flags[:2]):
                self.reason = "digit_overload"
            elif self.flags[2]:
                self.reason = "direct_surrogate_contact"
            elif self.loaded_ticks >= 12:
                self.reason = "plate_loaded_hand"
            elif occupied and any(registered):
                self.reason = "engaged_while_occupied"
            elif r["bottom"] < 0.037:
                self.reason = "plate_dropped"
        held = all(
            any(k.startswith(side + "/") for k in r["handles"])
            for side in ("left", "right")
        )
        held_off_supports = held and not r["other_support"] and max(r["rests"]) < 1
        hovering = (
            self.lifted
            and r["xy_error"] <= 0.045
            and r["bottom"] >= REST_Z + PIN_HEIGHT + 0.025
            and held_off_supports
            and r["tilt"] <= 6
            and r["speed"] < 0.035
        )
        self.hover_ticks = self.hover_ticks + 1 if hovering else 0
        if self.hover_ticks >= 120:
            self.reached_hover = True
        seated = (
            self.lifted
            and clear
            and all(registered)
            and min(r["rests"]) > 1
            and r["tilt"] <= 4
            and r["xy_error"] <= 0.025
            and abs(r["bottom"] - REST_Z) <= 0.003
            and not r["contact"]
            and not r["other_support"]
            and r["offrest"] < 0.1
            and r["clearance"] >= 0.035
            and r["speed"] < 0.008
            and r["spin"] < 0.06
        )
        self.dwell = self.dwell + 1 if seated else 0
        partial = self.reached_hover and (
            hovering
            or (
                self.lifted
                and held_off_supports
                and clear
                and r["tilt"] <= 6
                and r["speed"] < 0.035
                and min(r["pin_depth"]) < 0.004
                and r["bottom"] >= REST_Z + PIN_HEIGHT
                and r["xy_error"] <= 0.045
            )
        )
        self.score = (
            0.0
            if self.reason
            else 1.0
            if self.dwell >= 360
            else 0.5
            if partial
            else 0.0
        )
        return self.score
