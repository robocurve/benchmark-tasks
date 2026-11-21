"""Score "raise the loaded desk converter to its ninth setting". NumPy only, so the rules are unit-tested offline.

What is measured each control step (in a real trial: by eye, a ruler and a photo):

* the setting the top is at: the arm angle read from the lift joint, converted with layout.setting_of (real trial:
  count the notches, or measure the work surface's height against the green card and the published table);
* whether each side's lock is seated: its latch fully down, which happens only with its pin inside a notch
  (real trial: after the trial press the top down at each side by hand; a seated lock does not give);
* the monitor's and the keyboard's displacement and tilt relative to the top (photo against start marks);
* whether the robot is clear: no arm link touching the converter or its load, each fingertip centre at least
  CLEAR_M from its handle (real trial: both grippers visibly off the converter).

Tiers (the score at the end of the trial; 0.2 is earned once and kept):
  0.2  the top has been raised at least one setting above setting 4 at some moment (both locks were open together);
  0.5  at the end the top is at setting 7 or higher (locked or not, held or not), or 0.2 if lower;
  0.8  at the end both locks are seated in the same notch, setting 9, and the top is level (sides within 5 mm);
  1.0  in addition the monitor and the keyboard have each moved at most 30 mm on the top and tilted at most 10
       degrees, and both grippers have been clear for the last HOLD_S.
  0 for the whole trial, latched, if the monitor or the keyboard falls (tilts past 45 degrees or drops off its
       surface).
A trial ends early once 1.0 has been held for HOLD_S.
"""

from dataclasses import dataclass, field
import math

import numpy as np

from . import layout as L

CONTROL_HZ = 20
HOLD_S = 1.0
SEATED_M = .002          # a latch this close to its lowest position is seated (its pin is down in a notch)
AT_NOTCH = .30           # within this many settings of a notch's centre. A loaded top seated in notch k reads about
                         # k - 0.089 (slot play, layout.SEAT_SETTINGS); seated latches already prove a pin is in a slot,
                         # this only names the notch, so the band is wide (but under 0.5, so the notch is unique)
assert L.SEAT_SETTINGS + .1 < AT_NOTCH < .5
RAISED_ONE = L.START_SETTING + 1 - AT_NOTCH         # 4.7: beyond what the slot play allows with a pin in (4.09)
HIGH_ENOUGH = 7 - AT_NOTCH
LEVEL_M = .005           # the two sides' heights may differ by this much (real trial: ruler at both side edges)
LOAD_MOVE_M = .030
LOAD_TILT = math.radians(10.)
FALLEN_TILT = math.radians(45.)
FALLEN_DROP_M = .040     # a load more than this below where it rests (relative to the top) has fallen off
CLEAR_M = .050
TOUCH_N = .3


def tilt(rotation):
    """Angle between a rotation's z axis and the reference z axis (callers pass the load's rotation in the top frame)."""
    return float(math.acos(np.clip(np.asarray(rotation)[2, 2], -1., 1.)))


@dataclass
class ConverterGrade:
    monitor_start: tuple                  # monitor position in the top frame at the start (from layout)
    keyboard_start: tuple
    hold_ticks: int = 0
    clear_ticks: int = 0
    raised_once: bool = False
    fallen: str | None = None
    peak_setting: float = 0.
    history: list = field(default_factory=list)

    def update(self, angle, latches, top_pos, top_rot, monitor, keyboard, tcps, touching):
        """One control step.

        angle: lift arm angle above level (rad); latches: {"left": m, "right": m} latch heights above their lowest
        position; top_pos, top_rot: the top frame's origin and rotation (station frame); monitor, keyboard:
        (position, rotation) of each load's frame; tcps: {"left": point, "right": point} fingertip centres;
        touching: True if any arm link touches the converter or its load this step.
        """
        setting = float(L.setting_of(angle))
        self.peak_setting = max(self.peak_setting, setting)
        nearest = int(round(setting))
        at_notch = abs(setting - nearest) <= AT_NOTCH
        seated = {side: float(latches[side]) <= SEATED_M for side in ("left", "right")}
        locked_at = nearest if at_notch and all(seated.values()) else None
        top_pos, top_rot = np.asarray(top_pos, dtype=float), np.asarray(top_rot, dtype=float)
        # level: height difference of the two side edges of the work surface
        side_dz = abs(float((top_rot @ np.array((0., L.WIDTH, 0.)))[2]))
        level = side_dz <= LEVEL_M

        def load_state(pose, start):
            pos, rot = np.asarray(pose[0], dtype=float), np.asarray(pose[1], dtype=float)
            local = top_rot.T @ (pos - top_pos)
            moved = float(np.linalg.norm(local[:2] - np.asarray(start)[:2]))
            drop = float(np.asarray(start)[2] - local[2])
            return moved, drop, tilt(top_rot.T @ rot)

        m_moved, m_drop, m_tilt = load_state(monitor, self.monitor_start)
        k_moved, k_drop, k_tilt = load_state(keyboard, self.keyboard_start)
        if self.fallen is None:
            if m_tilt > FALLEN_TILT or m_drop > FALLEN_DROP_M:
                self.fallen = "monitor"
            elif k_tilt > FALLEN_TILT or k_drop > FALLEN_DROP_M:
                self.fallen = "keyboard"
        loads_ok = (m_moved <= LOAD_MOVE_M and k_moved <= LOAD_MOVE_M and m_tilt <= LOAD_TILT and k_tilt <= LOAD_TILT)
        grasps = {side: L.top_origin(angle) + L.GRASP_IN_TOP[side] for side in ("left", "right")}
        away = {side: float(np.linalg.norm(np.asarray(tcps[side]) - grasps[side])) for side in ("left", "right")}
        clear_now = (not touching) and min(away.values()) >= CLEAR_M
        self.clear_ticks = self.clear_ticks + 1 if clear_now else 0
        clear = self.clear_ticks >= round(HOLD_S * CONTROL_HZ)
        if setting >= RAISED_ONE:
            self.raised_once = True

        tier = .2 if self.raised_once else 0.
        if setting >= HIGH_ENOUGH:
            tier = .5
        if locked_at == L.GOAL_SETTING and level:
            tier = .8
            if loads_ok and clear:
                tier = 1.
        score = 0. if self.fallen else tier
        self.hold_ticks = self.hold_ticks + 1 if score == 1. else 0
        grade = {
            "score": score, "tier": tier, "setting": round(setting, 3), "peak_setting": round(self.peak_setting, 3),
            "nearest_notch": nearest, "at_notch": at_notch, "seated": seated, "locked_at": locked_at,
            "latch_mm": {s: round(1000 * float(latches[s]), 2) for s in ("left", "right")},
            "level": level, "side_dz_mm": round(1000 * side_dz, 2), "raised_once": self.raised_once,
            "monitor_moved_mm": round(1000 * m_moved, 1), "monitor_tilt_deg": round(math.degrees(m_tilt), 2),
            "keyboard_moved_mm": round(1000 * k_moved, 1), "keyboard_tilt_deg": round(math.degrees(k_tilt), 2),
            "loads_ok": loads_ok, "fallen": self.fallen, "touching": bool(touching),
            "hand_from_handle_mm": {s: round(1000 * v, 1) for s, v in away.items()}, "clear": clear,
            "held_s": round(self.hold_ticks / CONTROL_HZ, 2),
        }
        self.history.append(score)
        return grade

    @property
    def finished(self):
        return self.hold_ticks >= round(HOLD_S * CONTROL_HZ)


def make_grade():
    monitor = (float(L.MONITOR_AT[0]), float(L.MONITOR_AT[1]), 0.)
    keyboard = (float(L.KEYBOARD_AT[0]), float(L.KEYBOARD_AT[1]), -L.TRAY_DROP)
    return ConverterGrade(monitor_start=monitor, keyboard_start=keyboard)


@dataclass(frozen=True)
class ConverterScorer:
    name: str = "desk_converter_score"

    def __call__(self, record, target):
        from inspect_robots.scorer import Score

        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        g = record.steps[-1].result.info["desk_converter_grade"]
        return Score(value=g["score"],
                     explanation=f"Setting {g['setting']} (peak {g['peak_setting']}); locks seated {g['seated']}; "
                                 f"locked at {g['locked_at']}; loads ok {g['loads_ok']}; fallen {g['fallen']}; "
                                 f"robot clear {g['clear']}")


def make_scorer():
    return ConverterScorer()
