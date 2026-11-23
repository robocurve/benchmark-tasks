"""Grade the task on the 20 Hz observation grid. All fault inputs are visible positions."""

from collections import deque

from cover_geometry import PIN_HEIGHT, REST_Z


def visual_faults(r):
    # Use the two views of the *same* frame. A low pickup at the staging bay is valid.
    occupied = min(r["gauge_gap"], r["limb_gap"]) < 0.035
    low = min(p[2] for p in r["corners"]) < REST_Z + PIN_HEIGHT + 0.020
    return (
        r["stroke"][0] >= 0.003,
        r["stroke"][1] >= 0.003,
        abs(r["drive_travel"] - r["travel"]) >= 0.010,
        occupied and r["over_receiver"] and low,
        r["bottom"] < 0.037,
    )


def still(window, limit):
    """Peak-to-peak motion of each marked corner along each axis, in metres."""
    if len(window) != window.maxlen:
        return False
    for corner in zip(*window):
        for axis in zip(*corner):
            if max(axis) - min(axis) > limit:
                return False
    return True


class Trial:
    def __init__(self):
        self.tick = 0
        self.flags = [False] * 4
        self.reason = ""
        self.score = 0.0
        self.reset_valid = self.lifted = self.extracted = self.reached_hover = False
        self.hold = deque(maxlen=11)  # Ten elapsed 50 ms intervals.
        self.seat = deque(maxlen=31)  # Thirty elapsed intervals.
        self.dwell = 0

    def observe(self, r):
        if r["tick"] != self.tick + 1:
            raise ValueError("Missing or reordered physics sample")
        self.tick += 1
        if self.tick % 12 == 0:
            self.grade_frame(r)
        return self.score

    def grade_frame(self, r):
        gap = min(r["limb_gap"], r["gauge_gap"])
        clear = gap >= 0.035
        if not self.reset_valid:
            if (
                r["stage_error"] > 0.010
                or gap >= 0
                or max(r["stroke"]) >= 0.001
                or min(r["stage_loads"]) <= 0
                or abs(r["bottom"] - 0.092) > 0.003
            ):
                raise ValueError(
                    "Invalid setup on first frame: verify staging and unloaded fingers"
                )
            self.reset_valid = True

        faults = visual_faults(r)
        self.flags = [old or active for old, active in zip(self.flags, faults[:4])]
        if not self.reason:
            names = (
                "left_digit_travel",
                "right_digit_travel",
                "wrist_offset",
                "low_plate_at_occupied_receiver",
                "plate_dropped",
            )
            self.reason = next((name for name, hit in zip(names, faults) if hit), "")

        self.lifted |= (
            r["bottom"] >= REST_Z + PIN_HEIGHT + 0.035 and max(r["stage_loads"]) == 0
        )
        self.extracted |= clear
        registered = [
            d >= 0.004 and m >= -0.00030
            for d, m in zip(r["pin_depth"], r["pin_margin"])
        ]
        held = all(
            any(k.startswith(side + "/") for k in r["handles"])
            for side in ("left", "right")
        )
        carrying = held and not r["other_support"] and max(r["rests"]) == 0
        level_hold = (
            self.lifted and carrying and r["tilt"] <= 6 and r["xy_error"] <= 0.045
        )
        high = level_hold and r["bottom"] >= REST_Z + PIN_HEIGHT + 0.025
        incomplete = (
            level_hold
            and clear
            and min(r["pin_depth"]) < 0.004
            and r["bottom"] >= REST_Z + PIN_HEIGHT
        )
        if high or (self.reached_hover and incomplete):
            self.hold.append(r["corners"])
        else:
            self.hold.clear()
        stable = still(self.hold, 0.010)
        self.reached_hover |= high and stable

        seated = (
            self.lifted
            and clear
            and all(registered)
            and min(r["rests"]) > 0
            and r["tilt"] <= 4
            and r["xy_error"] <= 0.025
            and abs(r["bottom"] - REST_Z) <= 0.003
            and not r["contact"]
            and not r["other_support"]
            and r["clearance"] >= 0.035
        )
        if seated:
            self.seat.append(r["corners"])
            self.dwell += 12
        else:
            self.seat.clear()
            self.dwell = 0

        if self.reason:
            self.score = 0.0
        elif still(self.seat, 0.002):
            self.score = 1.0
        elif self.reached_hover and stable:
            self.score = 0.5
        else:
            self.score = 0.0
