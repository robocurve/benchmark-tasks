"""The busybox's internal state, tracked from its joint positions once per control step. NumPy only.

:class:`BusyboxState` reads the controls (switches, sliders, dial) and follows every push button through
press and release events with hold times. It also returns the drive targets that give the mechanics their
feel: each rocker is pulled to the side it is on (bistable), the dial to its nearest number (detents), and
every button up against its rest limit (springs).

Button presses are sampled at the control rate (20 Hz on Dream's settings): a press shorter than one control
period can go unnoticed.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict, dataclass

import numpy as np

from busybox import design as d


@dataclass(frozen=True)
class ButtonEvent:
    time: float  # episode time [s]
    button: str
    kind: str  # "press" or "release"
    held: float = 0.0  # for releases: how long the button was held down [s]


def switch_on(q: float) -> bool:
    """A rocker is ON while its far end is down (positive angle)."""
    return q > 0.0


def slider_value(q: float) -> float:
    """A slider's reading on the continuous 1..5 scale."""
    return 1.0 + (q - d.slider_x(0)) / d.SLIDER_TRAVEL * (d.SLIDER_POSITIONS - 1)


def slider_mark(q: float) -> int:
    """The nearest slider mark, 1..5."""
    return int(np.clip(round(slider_value(q)), 1, d.SLIDER_POSITIONS))


def dial_number(q: float) -> int:
    """The number the dial points at (nearest detent), 1..8; numbers increase clockwise seen from above."""
    return round(-q / d.DIAL_STEP) % d.DIAL_POSITIONS + 1


def dial_detent(q: float) -> float:
    """The dial's nearest detent angle (unwrapped, so the knob never spins back a full turn)."""
    return round(q / d.DIAL_STEP) * d.DIAL_STEP


class BusyboxState:
    """Tracks one busybox over an episode. Call :meth:`reset`, then :meth:`update` after every control step."""

    def __init__(self, cfg: d.BusyboxConfig):
        self.cfg = cfg
        self._depth = {b: d.travel(b) for b in d.BUTTONS}
        self.reset()

    def reset(self, q: Mapping[str, float] | None = None, time: float = 0.0) -> None:
        self.q = dict(q if q is not None else self.cfg.initial_joint_pos())
        self.time = time
        self.pressed = dict.fromkeys(d.BUTTONS, False)
        self.press_count = dict.fromkeys(d.BUTTONS, 0)
        self.press_start = dict.fromkeys(d.BUTTONS, 0.0)
        self.last_hold = dict.fromkeys(d.BUTTONS, 0.0)
        self.events: list[ButtonEvent] = []
        self._update_buttons()

    def update(self, q: Mapping[str, float], time: float) -> list[ButtonEvent]:
        """Take the joint positions after a control step at episode ``time``; returns the new button events."""
        missing = set(d.JOINTS) - set(q)
        if missing:
            raise KeyError(f"joint positions missing for {sorted(missing)}")
        self.q, self.time = dict(q), float(time)
        return self._update_buttons()

    def _update_buttons(self) -> list[ButtonEvent]:
        new = []
        for b in d.BUTTONS:
            depth = -self.q[b]  # buttons rest at 0 and go down to -travel
            was = self.pressed[b]
            # Pressed beyond PRESS_FRACTION of the travel; released once it rises above RELEASE_FRACTION.
            now = depth > (d.RELEASE_FRACTION if was else d.PRESS_FRACTION) * self._depth[b]
            if now and not was:
                self.press_start[b] = self.time
                self.press_count[b] += 1
                new.append(ButtonEvent(round(self.time, 3), b, "press"))
            elif was and not now:
                self.last_hold[b] = self.time - self.press_start[b]
                new.append(ButtonEvent(round(self.time, 3), b, "release", round(self.last_hold[b], 3)))
            self.pressed[b] = now
        self.events.extend(new)
        return new

    # -- readouts ------------------------------------------------------------------------

    @property
    def switches(self) -> tuple[bool, ...]:
        return tuple(switch_on(self.q[s]) for s in d.SWITCHES)

    @property
    def slider_values(self) -> tuple[float, ...]:
        return tuple(slider_value(self.q[s]) for s in d.SLIDERS)

    @property
    def slider_marks(self) -> tuple[int, ...]:
        return tuple(slider_mark(self.q[s]) for s in d.SLIDERS)

    @property
    def dial(self) -> int:
        return dial_number(self.q["dial"])

    def hold_time(self, button: str) -> float:
        """How long ``button`` has been held down so far (0 if it is up) [s]."""
        return self.time - self.press_start[button] if self.pressed[button] else 0.0

    def targets(self) -> dict[str, float]:
        """Drive targets for the next control step: rockers to their side, the dial to its detent, buttons up."""
        out = {s: d.SWITCH_ANGLE if switch_on(self.q[s]) else -d.SWITCH_ANGLE for s in d.SWITCHES}
        out["dial"] = dial_detent(self.q["dial"])
        out.update(dict.fromkeys(d.BUTTONS, d.BUTTON_PRELOAD))
        return out

    def snapshot(self) -> dict:
        """The configuration and the state of every control, as plain Python values."""
        return {
            "time": round(self.time, 3),
            "config": asdict(self.cfg),
            "switches": ["on" if s else "off" for s in self.switches],
            "sliders": [round(v, 2) for v in self.slider_values],
            "slider_marks": list(self.slider_marks),
            "dial": self.dial,
            "buttons": {
                b: {
                    "pressed": self.pressed[b],
                    "count": self.press_count[b],
                    "held": round(self.hold_time(b), 3),
                    "last_hold": round(self.last_hold[b], 3),
                }
                for b in d.BUTTONS
            },
            "events": [asdict(e) for e in self.events],
        }
