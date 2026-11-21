"""Scorer: where the tablets are, how they got there, and whether everything was left standing. NumPy only.

All positions are in the tabletop frame (z = 0 on the table). A body's pose is (origin, rotation matrix); the
bottle's, cap's and cup's origins are the centres of their undersides as they start (the cap starts upside down,
so its origin is the centre of its outer top face).
"""

from dataclasses import dataclass

import numpy as np

from . import layout as L

CONTROL_HZ = 20
N = L.N_TABLETS
LIE_TICKS = 5                 # 0.25 s inside the cap: the tablet has lain in the cap
TOUCH_TICKS = 5               # 0.25 s of gripper contact in total: the tablet was handled
STILL_TICKS = CONTROL_HZ      # 1 s
STILL_M = .001
CLEAR_M = .030                # every gripper centre this far from the bottle, the cap and the cup
UPRIGHT_COS = float(np.cos(np.radians(10.)))
STANDING_M = .005
RESTING_M = .005              # the cap counts as resting on the table when its lowest point is under 5 mm up
HELD_N = 1.0                  # a fingertip pressing on the cap with at least this force (the demo's pinch: about 8 N each)
SUPPORT_N = .02               # the bottle or the cup pushing on the cap with this much is holding it up (cap and two
                              # tablets weigh 0.066 N)
UNHELD_TICKS = 5              # 0.25 s with a tablet in a cap that is not held, or is held up by something else
MAX_STRAYS = 3
CAP_NOT_DONE = .2             # the most a run earns while something is still held or moving
TIERS = {"dose_given_and_tidy": 1., "dose_given": .7, "one_off": .4, "dose_in_cap": .2}


def _local(pose, points):
    origin, rotation = pose
    return (np.asarray(points, dtype=float) - np.asarray(origin)) @ np.asarray(rotation)


def in_bottle(pose, points):
    p = _local(pose, points)
    return (np.hypot(p[:, 0], p[:, 1]) < L.BOTTLE_R) & (p[:, 2] > 0.) & (p[:, 2] < L.BOTTLE_H)


def in_cap(pose, points):
    p = _local(pose, points)
    return (np.hypot(p[:, 0], p[:, 1]) < L.CAP_R_IN + .001) & (p[:, 2] > L.CAP_FLOOR - .001) & (p[:, 2] < L.CAP_H + .003)


def in_cup(pose, points):
    p = _local(pose, points)
    radius = (L.CUP_R_BOTTOM - L.CUP_WALL) + (L.CUP_R_TOP - L.CUP_R_BOTTOM) * np.clip(p[:, 2], 0., L.CUP_H) / L.CUP_H
    return (np.hypot(p[:, 0], p[:, 1]) < radius + .001) & (p[:, 2] > 0.) & (p[:, 2] < L.CUP_H)


def regions(bottle, cap, cup, tablets):
    """(in bottle, in cap, in cup) for each tablet. Where two containers' regions overlap (the cap held just over
    the bottle's mouth or the cup), the cup counts first, then the cap, then the bottle."""
    cupped = in_cup(cup, tablets)
    capped = in_cap(cap, tablets) & ~cupped
    bottled = in_bottle(bottle, tablets) & ~capped & ~cupped
    return bottled, capped, cupped


def cap_lowest(cap):
    """Height above the table of the lowest point of the cap (a cylinder 44.6 mm across and 17.2 mm tall)."""
    origin, rotation = cap
    axis_z = float(np.asarray(rotation)[2, 2])
    spread = L.CAP_R * float(np.sqrt(max(0., 1. - axis_z ** 2)))
    return float(origin[2]) + min(0., L.CAP_H * axis_z) - spread


def cap_resting(cap):
    """The cap is on the table (not held up off it)."""
    return cap_lowest(cap) < RESTING_M


def gap_to_cylinder(pose, radius, height, point):
    """Distance from a point to an upright-in-its-own-frame cylinder (0 inside it)."""
    p = _local(pose, np.asarray(point)[None])[0]
    return float(np.hypot(max(np.hypot(p[0], p[1]) - radius, 0.), max(-p[2], p[2] - height, 0.)))


def standing(pose):
    origin, rotation = pose
    return bool(np.asarray(rotation)[2, 2] > UPRIGHT_COS and abs(float(origin[2])) < STANDING_M)


class SafetyCapOpening:
    """Track the cap relative to the bottle. Both quantities are invariant to arm or bottle motion."""
    def __init__(self):
        self.stage = "locked"

    def observe(self, bottle, cap, bimanual):
        p = _local(bottle, np.asarray(cap[0])[None])[0]
        r = np.asarray(bottle[1]).T @ np.asarray(cap[1])
        press = L.CLOSED_Z - p[2]
        yaw = float(np.degrees(np.arctan2(r[1, 0], r[0, 0])))
        coaxial = np.hypot(*p[:2]) <= .006 and r[2, 2] < -np.cos(np.radians(15.))
        if self.stage != "opened":
            if self.stage == "turned":
                if bimanual and coaxial and -press >= .015 and 30 <= yaw <= 69:
                    self.stage = "opened"
                elif not bimanual or abs(yaw) < 4:
                    self.stage = "locked"
            elif not bimanual or not coaxial:
                self.stage = "locked"
            elif self.stage == "locked":
                if press >= .0015 and abs(yaw) <= 8:
                    self.stage = "pressed"
            elif self.stage == "pressed":
                if press < .001 or yaw < -8:
                    self.stage = "locked"
                elif yaw >= 20:
                    self.stage = "past_stop"
            elif self.stage == "past_stop" and L.WINDOW_START + L.LUG_HALF_ANGLE <= yaw <= 68:
                self.stage = "turned"
        return {"opening_stage": self.stage, "cap_press_mm": round(1000 * float(press), 2),
                "cap_lift_mm": round(-1000 * float(press), 2), "cap_turn_deg": round(yaw, 2),
                "opening_bimanual": bool(bimanual)}


class TabletGrade:
    def __init__(self):
        self.opening = SafetyCapOpening()
        self.cap_ticks = np.zeros(N, dtype=int)
        self.lay_in_cap = np.zeros(N, dtype=bool)
        self.touch_ticks = np.zeros(N, dtype=int)
        self.unheld_ticks = 0
        self.void = None
        self.history = []
        self.done_ticks = 0

    def update(self, tablets, bottle, cap, cup, touching, grippers, cap_held=None, cap_supported=False, opening_bimanual=False):
        """tablets: (N, 3) centres; bottle, cap, cup: (origin, rotation); touching: N booleans, a gripper is in
        contact with that tablet now; grippers: the two gripper centre points, or None if they cannot be read;
        cap_held: both fingertips of one gripper press on the cap (None when it cannot be read: counts as not
        held); cap_supported: the bottle or the cup pushes on the cap (it is resting on one of them)."""
        tablets = np.asarray(tablets, dtype=float)
        bottled, capped, cupped = regions(bottle, cap, cup, tablets)
        closure = self.opening.observe(bottle, cap, opening_bimanual)
        if self.opening.stage != "opened":
            # While closed the inverted cap overlaps the upper neck. Tablets
            # inside that neck belong to the bottle, even if the bottle is tilted.
            bottled = in_bottle(bottle, tablets)
            capped &= ~bottled
            if self.void is None and not bottled.all():
                self.void = "tablet_left_before_robot_opened_safety_cap"
        # sticky conditions first, on every call
        self.cap_ticks = np.where(capped, self.cap_ticks + 1, 0)
        self.lay_in_cap |= self.cap_ticks >= LIE_TICKS
        self.lay_in_cap &= ~bottled                       # back in the bottle: it has to pass through the cap again
        self.touch_ticks += np.asarray(touching, dtype=bool)
        if self.void is None and bool((cupped & ~self.lay_in_cap).any()):
            self.void = "tablet_reached_cup_without_lying_in_cap"
        # The dose goes into a cap held in a hand: with a tablet in the cap, the cap must be pinched by both
        # fingertips of one gripper and rest on nothing else (not the table, the bottle or the cup).
        cause = None
        if bool(capped.any()):
            if cap_resting(cap):
                cause = "tablet_in_cap_resting_on_table"
            elif cap_supported:
                cause = "tablet_in_cap_resting_on_bottle_or_cup"
            elif not cap_held:
                cause = "tablet_in_cap_not_held"
        self.unheld_ticks = self.unheld_ticks + 1 if cause else 0
        if self.void is None and self.unheld_ticks >= UNHELD_TICKS:
            self.void = cause
        if self.void is None and bool((self.touch_ticks >= TOUCH_TICKS).any()):
            self.void = "gripper_handled_a_tablet"
        n_cup, n_cap, n_bottle = int(cupped.sum()), int(capped.sum()), int(bottled.sum())
        strays = N - n_cup - n_cap - n_bottle             # on the table, on the floor or anywhere else
        not_back = N - n_cup - n_bottle                   # neither given nor back in the bottle
        cup_up, bottle_up = standing(cup), standing(bottle)
        if grippers is None:
            released = False                              # unreadable grippers count as still holding
        else:
            released = all(gap_to_cylinder(pose, radius, height, point) >= CLEAR_M for point in grippers
                           for pose, radius, height in ((bottle, L.BOTTLE_R, L.BOTTLE_H), (cap, L.CAP_R, L.CAP_H),
                                                        (cup, L.CUP_R_TOP, L.CUP_H)))
        snapshot = np.vstack((tablets, bottle[0], cap[0], cup[0], np.asarray(bottle[1])[:, 2] * .05,
                              np.asarray(cap[1])[:, 2] * .05))
        self.history = (self.history + [snapshot])[-STILL_TICKS:]
        still = len(self.history) == STILL_TICKS and float(np.abs(self.history[-1] - self.history[0]).max()) < STILL_M \
            and all(float(np.abs(self.history[-1] - h).max()) < STILL_M for h in self.history)
        done = released and still
        self.done_ticks = self.done_ticks + 1 if done else 0
        if self.void is not None:
            outcome, tier = "void", 0.
        elif self.opening.stage != "opened":
            outcome = "safety_cap_closed"
        elif n_cup == L.DOSE and cup_up and not_back == 0 and bottle_up:
            outcome = "dose_given_and_tidy"
        elif n_cup == L.DOSE and cup_up and not_back <= MAX_STRAYS:
            outcome = "dose_given"
        elif n_cup in (L.DOSE - 1, L.DOSE + 1) and cup_up and not_back <= MAX_STRAYS:
            outcome = "one_off"
        elif n_cup == 0 and n_cap == L.DOSE and strays == 0:
            outcome = "dose_in_cap"
        else:
            outcome = "nothing"
        if self.void is None:
            tier = TIERS.get(outcome, 0.)
        score = tier if done else min(tier, CAP_NOT_DONE)
        return {**closure, "score": float(score), "outcome": outcome, "void": self.void, "in_cup": n_cup, "in_cap": n_cap,
                "in_bottle": n_bottle, "strays": strays, "lay_in_cap": int(self.lay_in_cap.sum()),
                "cup_standing": cup_up, "bottle_standing": bottle_up, "released": released, "still": bool(still),
                "max_touch_ticks": int(self.touch_ticks.max()),
                "cap_lowest_mm": round(1000 * cap_lowest(cap), 1), "cap_held": cap_held,
                "cap_supported": bool(cap_supported)}

    @property
    def finished(self):
        return self.void is not None


@dataclass(frozen=True)
class TabletScorer:
    name: str = "tablets_cap_score"

    def __call__(self, record, target):
        from inspect_robots.scorer import Score

        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        grade = record.steps[-1].result.info["tablet_grade"]
        return Score(value=grade["score"],
                     explanation=f"{grade['outcome']}: {grade['in_cup']} in the cup, {grade['in_cap']} in the cap, "
                                 f"{grade['in_bottle']} in the bottle, {grade['strays']} elsewhere; void: {grade['void']}")


def make_scorer():
    return TabletScorer()
