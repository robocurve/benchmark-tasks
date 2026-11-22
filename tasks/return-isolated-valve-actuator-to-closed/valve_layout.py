"""Station dimensions in metres, fixture coordinates relative to the table."""

from math import cos, radians, sin

ORIGIN = (0.285, 0.0, 0.7475)
PIN = (-0.108, 0.0109, 0.047)
TRAY = (-0.168, 0.0, 0.012)
OPEN = 45.0
PIN_CLEAR = 0.060
KNOB_Z = 0.128
PROMPT = "Remove the mechanical lock pin from the valve actuator and return the indicator to its closed mark."
MODES = (
    "idle",
    "loaded_pull",
    "unload_only",
    "abandon_release",
    "reference",
    "alternate",
)
BODIES = ("frame", "rotor", "pin", "wrist", "memory")
SIDES = ("left", "right")


def knob(side, turn):
    x, y = (0.010, 0.151 if side == "left" else -0.151)
    a = radians(turn)
    return x * cos(a) - y * sin(a), x * sin(a) + y * cos(a), KNOB_Z
