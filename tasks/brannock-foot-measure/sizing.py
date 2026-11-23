"""Brannock sizing rules and the device's modelled layout. Pure Python, no simulator.

Published (Wikipedia, "Brannock Device", oldid 1356918829, "Sizing system" section and its table, which follows
these formulas row for row; the maker states the same 1/3-inch size step and 3/16-inch width step on
https://brannock.com/pages/about-us):

    heel-to-toe length  = 7 1/3 in + s * 1/3 in
    heel-to-ball (arch) = 5 18/50 in + s * 11/50 in
    width               = 1 10/16 in + s * 1/8 in + k * 3/16 in      (k = steps up from AAAA)

with s the men's Brannock size. Everything in the DEVICE dict below is modelled, not published by the maker:
check it on the purchased unit (README, "Modelled dimensions").

Device frame: origin on the base plate's top surface where the device's centre line meets the inside of the heel
cup's back wall; u runs from the heel toward the toes, v toward the medial (big-toe) side of a right foot, z up.
Units in this module: millimetres unless a name ends in _m.
"""

from __future__ import annotations

import math

IN = 25.4
WIDTHS = ("AAAA", "AAA", "AA", "A", "B", "C", "D", "E", "EE", "EEE", "EEEE")


def foot_length(s: float) -> float:
    return (7 + 1 / 3 + s / 3) * IN


def arch_length(s: float) -> float:
    return (5 + 18 / 50 + s * 11 / 50) * IN


def width(s: float, letter: str) -> float:
    return (1 + 10 / 16 + s / 8 + WIDTHS.index(letter) * 3 / 16) * IN


SIZE_STEP_ARCH = 11 / 50 * IN  # mm of arch length per full size (5.588)
WIDTH_STEP = 3 / 16 * IN  # mm per width letter (4.7625)


def arch_reading(u_index: float) -> float:
    """Arch size read at the pointer's index line, u_index mm from the heel cup's back wall."""
    return (u_index / IN - 5 - 18 / 50) / (11 / 50)


def length_reading(u_toe: float) -> float:
    return (u_toe / IN - 7 - 1 / 3) * 3


def width_steps(d: float, s: float) -> float:
    """Width read on the size-s row for a bar face d mm from the medial guide face, in letter steps from AAAA."""
    return (d / IN - 1 - 10 / 16 - s / 8) / (3 / 16)


def width_letter(d: float, s: float) -> str:
    k = round(width_steps(d, s))
    return WIDTHS[min(max(k, 0), len(WIDTHS) - 1)]


# The three foot forms (right feet); trial k uses FORMS[k % 3].
FORMS = (
    dict(name="m8C", size=8.0, letter="C"),
    dict(name="m9D", size=9.0, letter="D"),
    dict(name="m10_5E", size=10.5, letter="E"),
)
for f in FORMS:
    f.update(L=foot_length(f["size"]), A=arch_length(f["size"]), W=width(f["size"], f["letter"]))

# -- modelled device (mm; not published by the maker) ---------------------------------------------------------------
DEVICE = dict(
    base_u=(-14.0, 326.0),  # base plate extent along the centre line
    base_v=(-150.0, 72.0),  # lateral (-) to medial (+)
    base_t=10.0,  # plate thickness
    cup_r=37.0,  # heel cup: inside radius of its rounded back
    cup_wall=5.0,
    cup_h=28.0,  # cup wall height above the plate
    cup_side_u=58.0,  # the cup's straight side walls run to this u
    guide_v=53.0,  # medial guide face (the width scale's zero)
    guide_u=(80.0, 300.0),
    guide_w=12.0,  # guide thickness (v), its top carries the arch scale
    guide_h=26.0,
    ptr_travel=(140.0, 236.0),  # pointer index travel (u)
    ptr_reach=0.0,  # the pointer's curved nose is flush with the guide face (it does not push the foot)
    ptr_nose_r=5.0,
    ptr_nose_z=(7.0, 21.0),
    bar_travel=(60.0, 132.0),  # bar face distance d from the guide face
    bar_u=(112.0, 222.0),  # the bar's face spans this u range
    bar_z=(3.0, 25.0),
    bar_t=12.0,  # bar beam thickness (v)
)
PTR_START = 146.0  # pointer index slid back (maker: prepare the device with the pointer back)
BAR_START = 128.0  # width bar at its widest


def check() -> None:
    """The scales cover the three forms."""
    for f in FORMS:
        assert DEVICE["ptr_travel"][0] + 10 < f["A"] < DEVICE["ptr_travel"][1] - 10, f
        assert DEVICE["bar_travel"][0] + 10 < f["W"] < DEVICE["bar_travel"][1] - 10, f
        assert abs(arch_reading(f["A"]) - f["size"]) < 1e-9 and width_letter(f["W"], f["size"]) == f["letter"]
        assert math.isclose(length_reading(f["L"]), f["size"])


check()
NOSE_TO_INDEX = 0.0  # the index line runs through the centre of the pointer's curve
