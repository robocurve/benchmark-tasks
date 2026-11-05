"""The busybox: dimensions, layout and the configuration it shows. Pure Python, no Isaac Sim.

A flat, heavy case rests loose on the table in front of the arms. Its top face holds six 11 cm modules in a
2 x 3 grid; one of its two long side faces (+y "left" or -y "right") carries a serial-number sticker.

Box frame: origin at the bottom centre of the case, x away from the robots, y to the robots' left, z up.
"Up" on a module's label (where its text starts) is +x.

Modules (``MODULES``) and their joints:

- ``switches``: 4 rocker switches ``switch_0..3`` (hinges about y, bistable), far end down (positive angle) = ON;
  a red lamp above each switch is lit while it is on. Numbered 1-4 from left to right.
- ``sliders``: 2 friction sliders ``slider_0`` (left) and ``slider_1`` (right) along x, scale 1 (near) to 5 (far).
- ``buttons``: 4 spring push buttons ``button_red``, ``button_green`` (far row), ``button_yellow``,
  ``button_blue`` (near row).
- ``dial``: a bar knob ``dial`` (hinge about z) with 8 detents; 1 points to +x, numbers increase clockwise seen
  from above.
- ``big_button``: one large spring button ``big_button`` with a coloured cap and a PRESS or HOLD label.
- ``keypad``: a display showing a 5-letter word above six spring keys ``key_0..5`` (row-major from the far left),
  each with a Greek capital.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

MODULES = ("switches", "sliders", "buttons", "dial", "big_button", "keypad")

# sRGB colours.
BUTTON_COLORS: dict[str, tuple[float, float, float]] = {
    "red": (0.85, 0.12, 0.10),
    "green": (0.10, 0.65, 0.20),
    "yellow": (0.95, 0.80, 0.10),
    "blue": (0.12, 0.30, 0.85),
}
BIG_BUTTON_COLORS: dict[str, tuple[float, float, float]] = {**BUTTON_COLORS, "white": (0.92, 0.92, 0.90)}
BIG_BUTTON_LABELS = ("PRESS", "HOLD", "START")
DISPLAY_WORDS = ("APPLE", "HOUSE", "WATER", "LIGHT", "PLANT", "CHAIR", "BREAD", "STONE", "RIVER", "CLOUD")
# Greek capitals that don't look like Latin letters.
GREEK_LETTERS: dict[str, str] = {
    "gamma": "Γ",
    "delta": "Δ",
    "theta": "Θ",
    "lambda": "Λ",
    "xi": "Ξ",
    "pi": "Π",
    "sigma": "Σ",
    "phi": "Φ",
    "psi": "Ψ",
    "omega": "Ω",
}
SERIAL_LETTERS = "ABCDEFGHIJKLMNPQRSTUVWXZ"  # no O or Y (too close to 0 and V)

CASE_COLOR = (0.22, 0.24, 0.22)
PLATE_COLOR = (0.55, 0.56, 0.54)
KNOB_COLOR = (0.08, 0.08, 0.08)
FRAME_COLOR = (0.10, 0.10, 0.10)
ROCKER_COLOR = (0.90, 0.90, 0.88)
ROCKER_MARK_COLOR = (0.85, 0.10, 0.10)
KEY_COLOR = (0.90, 0.89, 0.83)

NUM_SWITCHES = 4
NUM_SLIDERS = 2
NUM_KEYS = 6
DIAL_POSITIONS = 8
SLIDER_POSITIONS = 5

CASE_SIZE = (0.25, 0.37, 0.05)  # x (depth), y (width), z (height) [m]
CASE_MASS = 2.5  # [kg]
MODULE_SIZE = 0.11
MODULE_PITCH = 0.12
LABEL_T = 0.0005  # lift of a label above the surface it is printed on [m]

SWITCH_ANGLE = 0.35  # rocker throw to either side [rad]
SLIDER_TRAVEL = 0.072  # between slider marks 1 and 5 [m]
BUTTON_TRAVEL = 0.004  # [m]
BIG_BUTTON_TRAVEL = 0.005
PRESS_FRACTION = 0.6  # a button counts as pressed beyond this fraction of its travel
RELEASE_FRACTION = 0.3  # ... and as released again above this fraction (hysteresis)
DIAL_STEP = 2 * math.pi / DIAL_POSITIONS

LAMP_X, LAMP_RADIUS = 0.036, 0.0055  # switch lamps: x above the switches (module frame) and radius [m]

# Joint mechanics (SI units; angular in rad). Detents (switches, dial) and button springs are joint position
# drives whose targets :mod:`busybox.state` sets every control step; the arms easily overpower them.
SWITCH_DRIVE = {"stiffness": 0.15, "damping": 0.002, "armature": 1e-5}  # N·m/rad, N·m·s/rad
DIAL_DRIVE = {"stiffness": 0.08, "damping": 0.002, "armature": 1e-5, "friction": 0.002}
BUTTON_DRIVE = {"stiffness": 300.0, "damping": 1.0, "armature": 0.005}  # N/m, N·s/m
BIG_BUTTON_DRIVE = {"stiffness": 400.0, "damping": 1.0, "armature": 0.005}
BUTTON_PRELOAD = 0.003  # springs pull towards this far above the rest (upper) limit [m]
SLIDER_DRIVE = {"stiffness": 0.0, "damping": 2.0, "armature": 0.01, "friction": 1.0}  # friction: N
PART_MASS = {"switch": 0.005, "slider": 0.01, "button": 0.005, "big_button": 0.01, "key": 0.005, "dial": 0.02}

BUTTONS = (*(f"button_{c}" for c in BUTTON_COLORS), "big_button", *(f"key_{i}" for i in range(NUM_KEYS)))
SWITCHES = tuple(f"switch_{i}" for i in range(NUM_SWITCHES))
SLIDERS = tuple(f"slider_{i}" for i in range(NUM_SLIDERS))
JOINTS = (*SWITCHES, *SLIDERS, "dial", *BUTTONS)


def slot_centers() -> list[tuple[float, float, float]]:
    """Module slots on the top face (box frame), row-major: far row left to right, then near row."""
    p, z = MODULE_PITCH, CASE_SIZE[2]
    return [(x, y, z) for x in (p / 2, -p / 2) for y in (p, 0.0, -p)]


def switch_ys() -> list[float]:
    """Switch y in the module frame, left (+y) to right."""
    return [0.0375 - 0.025 * i for i in range(NUM_SWITCHES)]


def slider_ys() -> list[float]:
    return [0.028, -0.028]


def slider_x(k: float) -> float:
    """Slider mark k (0-based, may be fractional) along x in the module frame; mark 1 is near (-x), 5 far."""
    return -SLIDER_TRAVEL / 2 + k * SLIDER_TRAVEL / (SLIDER_POSITIONS - 1)


def button_xy() -> dict[str, tuple[float, float]]:
    return {"red": (0.025, 0.025), "green": (0.025, -0.025), "yellow": (-0.025, 0.025), "blue": (-0.025, -0.025)}


def key_xy() -> list[tuple[float, float]]:
    return [(x, y) for x in (-0.004, -0.036) for y in (0.034, 0.0, -0.034)]


def travel(button: str) -> float:
    return BIG_BUTTON_TRAVEL if button == "big_button" else BUTTON_TRAVEL


@dataclass(frozen=True)
class BusyboxConfig:
    """What one busybox shows and how its controls start. A submission uses one fixed configuration."""

    serial: str = "KD4XR7"
    serial_side: str = "left"  # "left" (+y face) or "right" (-y face)
    word: str = "WATER"
    letters: tuple[str, ...] = ("psi", "omega", "delta", "xi", "lambda", "phi")  # keys 0..5
    big_button_color: str = "blue"
    big_button_label: str = "HOLD"
    layout: tuple[str, ...] = ("dial", "keypad", "sliders", "switches", "big_button", "buttons")  # per slot
    switches: tuple[bool, ...] = (True, False, True, False)  # starting state, True = ON
    sliders: tuple[int, ...] = (2, 4)  # starting marks, 1..5
    dial: int = 3  # starting number, 1..8

    def __post_init__(self) -> None:
        s = self.serial
        if not (
            len(s) == 6 and all(s[i] in SERIAL_LETTERS for i in (0, 1, 3, 4)) and s[2].isdigit() and s[5].isdigit()
        ):
            raise ValueError(f"serial must look like KD4XR7 (letters without O and Y), got {s!r}")
        if self.serial_side not in ("left", "right"):
            raise ValueError(f"serial_side must be 'left' or 'right', got {self.serial_side!r}")
        if not (len(self.word) == 5 and self.word.isalpha() and self.word.isupper()):
            raise ValueError(f"word must be 5 capital letters, got {self.word!r}")
        if (
            len(self.letters) != NUM_KEYS
            or len(set(self.letters)) != NUM_KEYS
            or set(self.letters) - set(GREEK_LETTERS)
        ):
            raise ValueError(f"letters must be {NUM_KEYS} distinct names from {tuple(GREEK_LETTERS)}")
        if self.big_button_color not in BIG_BUTTON_COLORS or self.big_button_label not in BIG_BUTTON_LABELS:
            raise ValueError(f"big button: color in {tuple(BIG_BUTTON_COLORS)}, label in {BIG_BUTTON_LABELS}")
        if sorted(self.layout) != sorted(MODULES):
            raise ValueError(f"layout must order all of {MODULES}, got {self.layout}")
        if len(self.switches) != NUM_SWITCHES or len(self.sliders) != NUM_SLIDERS:
            raise ValueError("switches needs 4 entries and sliders 2")
        if not all(1 <= v <= SLIDER_POSITIONS for v in self.sliders) or not 1 <= self.dial <= DIAL_POSITIONS:
            raise ValueError("sliders start at a mark 1..5 and the dial at a number 1..8")

    @classmethod
    def sample(cls, seed: int) -> BusyboxConfig:
        """A random configuration (labels, layout and starting controls) from ``seed``."""
        rng = random.Random(seed)
        serial = "".join(rng.choice(SERIAL_LETTERS if i in (0, 1, 3, 4) else "0123456789") for i in range(6))
        return cls(
            serial=serial,
            serial_side=rng.choice(("left", "right")),
            word=rng.choice(DISPLAY_WORDS),
            letters=tuple(rng.sample(tuple(GREEK_LETTERS), NUM_KEYS)),
            big_button_color=rng.choice(tuple(BIG_BUTTON_COLORS)),
            big_button_label=rng.choice(BIG_BUTTON_LABELS),
            layout=tuple(rng.sample(MODULES, len(MODULES))),
            switches=tuple(rng.random() < 0.5 for _ in range(NUM_SWITCHES)),
            sliders=tuple(rng.randint(1, SLIDER_POSITIONS) for _ in range(NUM_SLIDERS)),
            dial=rng.randint(1, DIAL_POSITIONS),
        )

    def module_center(self, module: str) -> tuple[float, float, float]:
        """Centre of a module's printed plate, just above the case's top face (box frame)."""
        x, y, z = slot_centers()[self.layout.index(module)]
        return (x, y, z + LABEL_T)

    def initial_joint_pos(self) -> dict[str, float]:
        """Every joint's starting position: the configured switches, sliders and dial; buttons up."""
        q = {name: 0.0 for name in JOINTS}
        for i, on in enumerate(self.switches):
            q[f"switch_{i}"] = SWITCH_ANGLE if on else -SWITCH_ANGLE
        for i, mark in enumerate(self.sliders):
            q[f"slider_{i}"] = slider_x(mark - 1)
        q["dial"] = -DIAL_STEP * (self.dial - 1)
        return q

    def key(self) -> str:
        """A short id of this configuration, for caching built assets."""
        import hashlib

        return hashlib.sha1(repr(self).encode()).hexdigest()[:12]
