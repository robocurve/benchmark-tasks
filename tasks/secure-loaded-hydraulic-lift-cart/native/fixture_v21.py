"""Geometry of the service cart in meters and degrees."""

from pathlib import Path
import math
from native.entry_layout import CART_ROOT

ROOT = Path(__file__).resolve().parents[1]
CASES = ("open", "both", "left", "right")
BASES = {"left": (0.0, 0.305, 0.75), "right": (0.0, -0.305, 0.75)}
X, Y, Z = 0.19, 0.34, 0.37
HEIGHT = 0.186
GRIP_Y, GRIP_Z = 0.125, 0.143
SEAT = -1.15
OFFSET, HALF = -0.145, 0.012
VALVE = (0.29, -0.605, 0.80)
INSTRUCTION = "Make this loaded cart safe to leave unattended overnight. Leave the crate on the cart."


def grip(side, angle):
    a = math.radians(angle)
    return (
        CART_ROOT[0] + X + GRIP_Z * math.sin(a),
        (Y + GRIP_Y) * (1 if side == "left" else -1),
        Z + GRIP_Z * math.cos(a),
    )


def handle(angle):
    a = math.radians(angle)
    return (
        CART_ROOT[0] + VALVE[0] + 0.074 * math.cos(a),
        VALVE[1] + 0.074 * math.sin(a),
        VALVE[2] + 0.080,
    )
