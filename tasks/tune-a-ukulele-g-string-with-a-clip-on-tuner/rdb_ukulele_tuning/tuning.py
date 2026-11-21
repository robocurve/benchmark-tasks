"""The rules that stand in for sound, written out. No sound or string vibration is simulated:

- ``Pitch``: each string's pitch, in cents from its target note, follows its button's angle through the worm gear's
  free play.
- ``PluckDetector``: what counts as plucking the G string.
- ``TunerDisplay`` / ``reading``: what the clip-on tuner shows (for 3 s after a pluck).
- ``FinalCheck``: whether the G string has been plucked since its button last turned (full marks need it).
- ``button_turn_for``, ``equivalent_roll``, ``roll_change``: arithmetic the scripted demonstration uses.

Pure Python (``math`` only). The adapter feeds these the simulator's readings once per 20 Hz control step; the
tests feed them hand-made ones.
"""

from __future__ import annotations

import math

from . import layout as L

CONTROL_HZ = 20


def unwrap(previous, wrapped):
    """Continue an angle reading that the simulator reports in (-pi, pi] (a revolute joint without end stops)."""
    return previous + (wrapped - previous + math.pi) % (2 * math.pi) - math.pi


def clip(value, low, high):
    return min(max(value, low), high)


class Pitch:
    """One string's pitch from its button's joint angle.

    The joint angle (radians, about the instrument's +x) is turned into "tightening degrees" (``layout.tighten_sign``).
    The string's post follows the button through a dead band FREE_PLAY_DEG wide (the gear's free play), and the
    pitch moves CENTS_PER_DEG for every degree the post is turned by the button (that is, per degree of the button
    once the free play is taken up). At reset the free play is taken up in the tightening direction (the reset
    notes bring the G string up to F#4 from below), so tightening moves the post at once and loosening first turns
    the button through the free play.
    """

    def __init__(self, name, start_cents=None, angle=0.):
        self.name = name
        self.sign = L.tighten_sign(name)
        self.start = float(L.START_CENTS[name] if start_cents is None else start_cents)
        self.button = self.sign * math.degrees(angle)
        self.post = self.button - L.FREE_PLAY_DEG / 2
        self.post0 = self.post
        self.cents = self.start

    def update(self, angle):
        """``angle``: the button's unwrapped joint angle, radians. Returns cents from the target note."""
        self.button = self.sign * math.degrees(angle)
        half = L.FREE_PLAY_DEG / 2
        self.post = clip(self.post, self.button - half, self.button + half)
        self.cents = self.start + L.CENTS_PER_DEG * (self.post - self.post0)
        return self.cents

    @property
    def play(self):
        """Where the button sits in the free play, tightening degrees: +FREE_PLAY_DEG/2 when the next tightening
        turn moves the post at once, -FREE_PLAY_DEG/2 when the next loosening turn does."""
        return self.button - self.post

    @property
    def turned(self):
        """Tightening degrees the button has turned since reset."""
        return self.button - (self.post0 + L.FREE_PLAY_DEG / 2)


class PluckDetector:
    """A pluck: the pluckable stretch of the G string is pushed at least PLUCK_OUT across the strings (either way) and, with
    no part of either arm touching it at any control step after the last step on which it was that far out, it is
    back within PLUCK_BACK no later than PLUCK_RETURN_S after that step. Plucks closer together than PLUCK_GAP_S
    count once. A string pushed out and eased back by the finger, or held, or only brushed, is not a pluck."""

    def __init__(self, hz=CONTROL_HZ):
        self.return_ticks = int(round(L.PLUCK_RETURN_S * hz))
        self.gap_ticks = int(round(L.PLUCK_GAP_S * hz))
        self.phase = "rest"            # rest, out, returning, damped
        self.out_tick = None           # last step on which the string was PLUCK_OUT or more out
        self.peak = 0.                 # largest displacement of the current pluck, metres
        self.last = None               # step of the last counted pluck
        self.count = 0
        self.ignored = 0               # plucks too soon after the last one
        self.damped = 0                # pushes out that were not plucks (touched or eased back)
        self.last_peak = 0.

    def update(self, tick, displacement, touched):
        """One control step. ``displacement``: the stretch's sideways position, metres (0 at rest);
        ``touched``: any part of either arm is in contact with it. Returns True on the step a pluck counts."""
        d = abs(float(displacement))
        if d >= L.PLUCK_OUT:
            if self.phase != "out":
                self.peak = 0.
            self.phase, self.out_tick = "out", tick
            self.peak = max(self.peak, d)
            return False
        if self.phase == "out":
            self.phase = "returning"
        if self.phase == "returning":
            if touched or tick - self.out_tick > self.return_ticks:
                self.phase = "damped"
                self.damped += 1
            elif d <= L.PLUCK_BACK:
                self.phase = "rest"
                if self.last is None or tick - self.last >= self.gap_ticks:
                    self.last, self.last_peak = tick, self.peak
                    self.count += 1
                    return True
                self.ignored += 1
            return False
        if self.phase == "damped" and d <= L.PLUCK_BACK:
            self.phase = "rest"
        return False


def blank():
    """The tuner not reading: needle at its left rest, every lamp off."""
    return {"active": False, "note": None, "cents_from_note": None, "needle_deg": L.NEEDLE_REST_DEG,
            "lamps": {name: False for name in (*L.NOTE_LAMP.values(), L.IN_TUNE_LAMP[0])}}


def reading(cents):
    """What a chromatic clip-on tuner shows for a string ``cents`` from G4: the nearest note, the needle 0.9 degrees
    per cent from it (45 degrees at 50 cents), green within IN_TUNE_CENTS of it. The simulated display has lamps for
    F#, G and G# only; further than 50 cents beyond them it shows no note and its needle sits at that end."""
    semitone = math.floor(cents / 100. + .5)
    out = blank()
    out["active"] = True
    if abs(semitone) > 1:
        out["needle_deg"] = L.NEEDLE_MAX_DEG if cents > 0 else -L.NEEDLE_MAX_DEG
        return out
    off = cents - 100. * semitone
    out["note"] = L.TUNER_NOTES[semitone + 1]
    out["cents_from_note"] = round(off, 3)
    out["needle_deg"] = clip(off * L.NEEDLE_MAX_DEG / L.NEEDLE_RANGE_CENTS, -L.NEEDLE_MAX_DEG, L.NEEDLE_MAX_DEG)
    out["lamps"][L.NOTE_LAMP[semitone]] = True
    out["lamps"][L.IN_TUNE_LAMP[0]] = abs(off) <= L.IN_TUNE_CENTS
    return out


class TunerDisplay:
    """The clip-on tuner reads the G string from the step a pluck counts until TUNER_ACTIVE_S later (a new pluck
    starts the time again); it shows the pitch live, so turning the button while it reads moves the needle. The rest
    of the time it is blank."""

    def __init__(self, hz=CONTROL_HZ):
        self.active_ticks = int(round(L.TUNER_ACTIVE_S * hz))
        self.since = None

    def update(self, tick, plucked, cents):
        if plucked:
            self.since = tick
        if self.since is not None and tick - self.since < self.active_ticks:
            return reading(cents)
        return blank()


class FinalCheck:
    """Whether the G string has been plucked since its button last turned, so that the tuner has shown the final pitch.

    The last counted pluck stands as the final check until the G button turns more than ``limit_deg`` (the gear's free
    play, FREE_PLAY_DEG) away from where it was at that pluck: any rotation larger than the free play is a movement,
    and a pluck before the button's last movement does not count. (A turn within the free play can still move the
    pitch by up to FREE_PLAY_DEG x CENTS_PER_DEG, under 1 cent.) On the bench the judge decides it from the trial
    video: a pluck visible after the last turn of the button."""

    def __init__(self, limit_deg=L.FREE_PLAY_DEG):
        self.limit = float(limit_deg)
        self.checked = False
        self.pluck_tick = None       # step of the last counted pluck
        self.at_deg = None           # the G button's angle at that pluck, degrees
        self.moved_tick = None       # the step on which the button last moved after a pluck

    def update(self, tick, plucked, button_deg):
        """One control step. ``plucked``: a pluck counted on this step; ``button_deg``: the G button's angle, degrees
        (unwrapped, any fixed zero). Returns whether the final check stands."""
        if plucked:
            self.checked, self.pluck_tick, self.at_deg = True, tick, float(button_deg)
        elif self.checked and abs(float(button_deg) - self.at_deg) > self.limit:
            self.checked, self.moved_tick = False, tick
        return self.checked

    def turned_since_pluck(self, button_deg):
        """How far the G button has turned since the last pluck, degrees (None before any pluck)."""
        return None if self.at_deg is None else float(button_deg) - self.at_deg


# -- arithmetic for the scripted demonstration ----------------------------------------------------------------
def button_turn_for(error_cents, play_deg, cents_per_deg=L.CENTS_PER_DEG, free_play_deg=L.FREE_PLAY_DEG):
    """Tightening degrees to turn a button (negative: loosen) so that a string ``error_cents`` from its note comes
    to it: the free play still to take up in that direction, then error / cents_per_deg."""
    if error_cents == 0:
        return 0.
    direction = -1. if error_cents > 0 else 1.
    slack = max(0., free_play_deg / 2 - direction * play_deg)
    return direction * (slack + abs(error_cents) / cents_per_deg)


def roll_change(tighten_deg, tighten_sign, roll_sign):
    """Change of the wrist's roll joint (radians) that turns a button held by it ``tighten_deg``.

    ``tighten_sign``: +1 if the button's joint angle rising tightens it (``layout.tighten_sign``). ``roll_sign``: +1
    if the roll joint rising turns the gripper's fingers positively about the button's joint axis (+x)."""
    return math.radians(tighten_deg) * tighten_sign * roll_sign


def equivalent_roll(roll, low, high, heading, period=math.pi):
    """Among the rolls ``roll + k * period`` (a flat button looks the same every half turn), the one inside
    [low, high] that leaves the most room to turn toward ``heading`` (+1: the roll will rise). None if none fits."""
    k_low = math.ceil((low - roll) / period - 1e-9)
    k_high = math.floor((high - roll) / period + 1e-9)
    if k_low > k_high:
        return None
    return roll + (k_low if heading > 0 else k_high) * period
