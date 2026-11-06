"""USD bodies of the jack-and-stands trainer (original simplified geometry). Import after Isaac Sim starts.

Bodies (origins in design.start_poses):
    load         steel beam with its two end blocks (one rigid body, 23 kg), marks on top and on the near face
    saddle       the jack's saddle plate, on a vertical prismatic joint (its ram is visual, inside the jack body)
    lever        the pump lever with its green pad, on a revolute joint about y at the pivot (spring-returned)
    knob         the yellow release knob (hub and wing), on a revolute joint about z
    stand_left,  red fixed-height jack stands: skid, column, saddle plate, grip block at the skid's near end
    stand_right
    wrench       the orange wrench under the beam: grip block and bar

The jack body, the lever's pivot cheeks and the knob's post are fixed task-file objects (jack_stands_task.py).
"""

from __future__ import annotations

from pathlib import Path

from . import design as d
from .usd_bodies import Body

STEEL, DARK = (0.30, 0.33, 0.38), (0.12, 0.12, 0.13)
YELLOW, CYAN, RED = (0.98, 0.85, 0.05), (0.0, 0.80, 0.90), (0.85, 0.08, 0.08)
GREEN, ORANGE, BLACK = (0.10, 0.75, 0.20), (0.98, 0.45, 0.05), (0.03, 0.03, 0.03)


def load() -> Path:
    b = Body("load", mass=d.LOAD_MASS, friction=(0.6, 0.5))
    bx, by, bz = d.BEAM
    b.box((0.0, 0.0, 0.0), d.BEAM, STEEL)
    fx, fy, fz = d.FOOT
    for s in (-1, 1):
        b.box((0.0, s * d.FOOT_Y, -bz / 2 - fz / 2), d.FOOT, DARK)
    top = bz / 2 + 0.0005
    b.box((0.0, d.LIFT_Y, top), (bx, d.MARK_W["lift"], 0.001), YELLOW, collide=False)
    for s in (-1, 1):
        b.box((0.0, s * d.SUPPORT_Y, top), (bx, d.MARK_W["support"], 0.001), CYAN, collide=False)
        b.box((-bx / 2 - 0.0005, s * d.SUPPORT_Y, 0.0), (0.001, d.MARK_W["support"], bz), CYAN, collide=False)
    b.box((-bx / 2 - 0.0005, d.LIFT_Y, 0.0), (0.001, d.MARK_W["lift"], bz), YELLOW, collide=False)
    return b.save()


def saddle() -> Path:
    b = Body("saddle", mass=d.SADDLE_MASS, friction=(1.0, 0.9))
    sx, sy, sz = d.SADDLE
    b.box((0.0, 0.0, 0.0), d.SADDLE, BLACK)
    ram = d.JACK_TRAVEL + 0.02
    b.cylinder((0.0, 0.0, -sz / 2 - ram / 2), d.RAM_R, ram, (0.75, 0.75, 0.78), collide=False)
    return b.save()


def lever() -> Path:
    b = Body("lever", mass=d.LEVER_MASS, friction=(1.0, 0.9))
    lx, ly, lz = d.LEVER_BAR
    b.box((-lx / 2, 0.0, 0.0), d.LEVER_BAR, DARK)
    b.box((d.PAD_X, 0.0, d.PAD_Z), d.PAD, GREEN)
    b.cylinder((0.0, 0.0, 0.0), 0.006, 2 * d.CHEEK_Y + d.CHEEK[1], (0.75, 0.75, 0.78), axis="Y", collide=False)
    return b.save()


def knob() -> Path:
    b = Body("knob", mass=d.KNOB_MASS, friction=(1.0, 0.9))
    r, h = d.KNOB_HUB
    b.cylinder((0.0, 0.0, h / 2), r, h, YELLOW)
    wx, wy, wz = d.KNOB_WING
    b.box((0.0, 0.0, h + wz / 2), d.KNOB_WING, YELLOW)
    b.box((wx / 2 - 0.004, 0.0, h + wz + 0.0005), (0.008, wy, 0.001), BLACK, collide=False)  # pointer tip
    return b.save()


def stand(name: str) -> Path:
    b = Body(name, mass=d.STAND_MASS, friction=(0.4, 0.35))  # steel skid on the table
    (sx, sy), skid = d.STAND_SKID
    b.box((sx, sy, skid[2] / 2), skid, RED)
    cx, cy = d.STAND_COL
    top = d.STAND_TOP
    col_h = d.STAND_H - top[2] - skid[2]
    b.box((0.0, 0.0, skid[2] + col_h / 2), (cx, cy, col_h), RED)
    b.box((0.0, 0.0, d.STAND_H - top[2] / 2), top, RED)
    (gx, gy), grip = d.STAND_GRIP
    b.box((gx, gy, skid[2] + grip[2] / 2), grip, (0.55, 0.05, 0.05))
    return b.save()


def wrench() -> Path:
    b = Body("wrench", mass=d.WRENCH_MASS, friction=(0.6, 0.5))
    gx, gy, gz = d.WRENCH_GRIP
    b.box((0.0, 0.0, gz / 2), d.WRENCH_GRIP, ORANGE)
    bx, by, bz = d.WRENCH_BAR
    b.box((gx / 2 + bx / 2, 0.0, bz / 2), d.WRENCH_BAR, ORANGE)
    return b.save()


BUILDERS = {"load": load, "saddle": saddle, "lever": lever, "knob": knob,
            "stand_left": lambda: stand("stand_left"), "stand_right": lambda: stand("stand_right"),
            "wrench": wrench}
