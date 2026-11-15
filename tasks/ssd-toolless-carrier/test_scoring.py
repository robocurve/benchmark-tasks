"""Offline checks of task.py (no GPU, no Isaac Sim): every scoring outcome from fabricated object states, and the
geometry of the drive, its holes and the carrier's pins.

    python -m pytest test_scoring.py
"""

from __future__ import annotations

import importlib
import itertools
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from ssdkit import PartState  # noqa: E402

T = importlib.import_module("task")

CAR = np.array([*T.CARRIER_XY, T.TOP])
SEATED = CAR + T.SEAT


def _quat(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.array([*(a * math.sin(angle / 2)), math.cos(angle / 2)])


def _state(pos, quat=(0, 0, 0, 1), sliders=None, vz=0.0):
    p = np.asarray(pos, float)
    return PartState(pos=p, quat_xyzw=np.asarray(quat, float), lin_vel=np.array([0, 0, vz]), ang_vel=np.zeros(3),
                     low=p, high=p, sliders=sliders or {})


def _objs(ssd=(0, 0, 0), rail=0.0, quat=(0, 0, 0, 1), vz=0.0, car=(0, 0, 0)):
    """``ssd`` is the drive's offset from its seated position; ``car`` moves the carrier (and the seat with it)."""
    car = np.asarray(car, float)
    return {"carrier": _state(CAR + car, sliders={"rail": rail}),
            "ssd": _state(SEATED + car + np.asarray(ssd, float), quat, vz=vz)}


def _run(frames):
    T._reset_state()
    out = (0.0, False)
    for objs in frames:
        out = (T.score(objs), T.success(objs))
    return out


def _start():
    return {"carrier": _state(CAR, _quat((0, 0, 1), T.CARRIER_YAW), sliders={"rail": 0.0}),
            "ssd": _state(np.array([*T.SSD_XY, T.TOP]), _quat((0, 0, 1), T.SSD_YAW))}


def test_start_scores_zero():
    assert _run([_start()]) == (0.0, False)


def test_seated_succeeds_after_one_second():
    assert _run([_objs() for _ in range(20)]) == (1.0, True)
    assert _run([_objs(car=(0.03, 0.02, 0)) for _ in range(20)]) == (1.0, True)   # carrier moved: still seated


def test_seated_but_not_yet_held_scores_three_quarters():
    assert _run([_objs() for _ in range(5)]) == (0.75, False)


def test_pin_counts():
    assert T.pins_in(_objs()["ssd"], _objs()["carrier"]) == (2, 2)
    # Drive against the open rail (3 mm toward it), rail held open: no pins in.
    o = _objs(ssd=(0, 0.003, 0), rail=0.0065)
    assert T.pins_in(o["ssd"], o["carrier"]) == (0, 0)
    # Drive on the fixed pins, rail still held open: only the right side.
    o = _objs(rail=0.0065)
    assert T.pins_in(o["ssd"], o["carrier"]) == (2, 0)
    # Rail let go but its pins missed the holes (drive 2.5 mm too far forward): the rail rests on the drive's side.
    o = _objs(ssd=(-0.0025, 0, 0), rail=0.0025)
    assert T.pins_in(o["ssd"], o["carrier"]) == (0, 0)


def test_partial_scores():
    assert _run([_objs(ssd=(0, 0.003, 0), rail=0.0065)])[0] == 0.25        # lying in the carrier, no pins
    assert _run([_objs(rail=0.0065)])[0] == 0.5                             # fixed-side pins in
    assert _run([_objs(rail=0.0009)] * 1 + [_objs(rail=0.0009) for _ in range(25)])[0] == 1.0   # rail within 1 mm
    o = _objs(ssd=(0, 0, 0), rail=0.0012)                                   # pins 0.8 mm deep: not counted
    assert T.pins_in(o["ssd"], o["carrier"]) == (2, 0)


def test_not_flat_or_off_seat():
    tilt = _quat((0, 1, 0), math.radians(1.5))                             # one end 2.6 mm up
    assert _run([_objs(ssd=(0, 0, 0.0013), quat=tilt) for _ in range(20)])[0] == 0.0
    assert _run([_objs(ssd=(0, 0, 0.0015)) for _ in range(20)])[0] == 0.0   # held 1.5 mm above the ribs
    assert _run([_objs(ssd=(0, 0.006, 0)) for _ in range(20)])[0] == 0.0    # 6 mm off to the side


def test_moving_drive_does_not_succeed():
    frames = [_objs(ssd=(0.0002 * (k % 2) * 5, 0, 0)) for k in range(25)]  # jiggling 1 mm
    assert _run(frames) == (0.75, False)


def test_wrong_way_or_dropped_on_the_way_does_not_count_against_a_seated_drive():
    # Judged on the final state: a drive briefly held the wrong way over the carrier, or dropped into it, then seated
    # correctly, scores 1.0.
    end_for_end = _quat((0, 0, 1), math.pi)
    upside = _quat((1, 0, 0), math.pi)
    for first in (_objs(ssd=(0, 0, 0.003), quat=end_for_end), _objs(ssd=(0, 0, 0.003), quat=upside),
                  _objs(ssd=(0, 0, 0.02), vz=-0.6)):
        assert _run([first] + [_objs() for _ in range(25)]) == (1.0, True)


def test_wrong_way_in_the_carrier_scores_zero():
    for q in (_quat((0, 0, 1), math.pi), _quat((1, 0, 0), math.pi)):
        assert _run([_objs(quat=q) for _ in range(25)])[0] == 0.0


# -- geometry ---------------------------------------------------------------------------------------------------------

def _boxes(boxes, base, prefix):
    return [(prefix, np.asarray(base) + np.array(b[0]), np.array(b[1])) for b in boxes]


def _carrier_boxes(rail):
    return (_boxes(T.CARRIER_BOXES, (0, 0, 0), "carrier")
            + _boxes(T._latch_rail().boxes, (0, rail, 0), "rail"))


def _overlaps(a_list, b_list, tol=1e-5):
    hits = []
    for a, b in itertools.product(a_list, b_list):
        depth = np.minimum(a[1] + a[2] / 2, b[1] + b[2] / 2) - np.maximum(a[1] - a[2] / 2, b[1] - b[2] / 2)
        if (depth > tol).all():
            hits.append((a[0], tuple(np.round(a[1] * 1000, 2)), tuple(np.round(b[1] * 1000, 2))))
    return hits


def test_seated_drive_clears_every_carrier_part():
    ssd = _boxes(T.SSD_BOXES, T.SEAT, "ssd")
    assert _overlaps(_carrier_boxes(0.0), ssd) == []


def test_pins_clear_their_holes():
    # 0.75 mm round each pin inside its hole, and its tip 1 mm short of the hole's bottom.
    assert (T.HOLE - T.PIN) / 2 >= 0.75 - 1e-9
    assert T.HOLE_DEPTH - T.PIN_IN >= 1.0 - 1e-9
    ssd = _boxes(T.SSD_BOXES, T.SEAT, "ssd")
    for dx in (-0.0007, 0.0007):
        for dz in (0.0, 0.0007):          # resting on its ribs, or lifted 0.7 mm
            moved = [(n, c + np.array([dx, 0, dz]), s) for n, c, s in ssd]
            assert _overlaps(_carrier_boxes(0.0), moved) == []


def test_turned_drive_does_not_fit_the_pins():
    # End for end, the holes are 4.6 mm off the pins: the pins hit the drive's side wall.
    ssd = [(n, T.SEAT + np.array([-c[0], -c[1], c[2]]), s) for n, c, s in _boxes(T.SSD_BOXES, (0, 0, 0), "ssd")]
    assert _overlaps(_carrier_boxes(0.0), ssd) != []


def test_open_rail_lets_the_drive_down_beside_the_fixed_pins():
    # Rail held 6.5 mm open, drive lowered 3 mm toward it: 1 mm clear of the fixed pins and 1.5 mm of the sprung ones.
    ssd = _boxes(T.SSD_BOXES, T.SEAT + np.array([0, 0.003, 0]), "ssd")
    cb = _carrier_boxes(0.0065)
    assert _overlaps(cb, ssd) == []
    fixed_tip = -(T.HALF_W - T.PIN_IN) / 1000
    assert -T.HALF_W / 1000 + 0.003 - fixed_tip >= 0.001 - 1e-9
    sprung_tip = (T.HALF_W - T.PIN_IN) / 1000 + 0.0065
    assert sprung_tip - (T.HALF_W / 1000 + 0.003) >= 0.0015 - 1e-9
    assert 0.0065 < T.RAIL_TRAVEL


def test_finger_pads_clear_the_carrier():
    # The drive is pinched across its width at its centre, the pads' tips 2 mm above
    # its bottom face. Pads: 27 mm along x, 6.4 mm thick, flat 24 mm up; checked with the drive 3 mm toward the open
    # rail (rail 6.5 mm open) and seated (rail open, and closed).
    for dy, rail in ((0.003, 0.0065), (0.0, 0.0065), (0.0, 0.0)):
        cx = T.SEAT[0]
        z0 = T.SEAT[2] + 0.002
        pads = []
        for side in (-1, 1):
            y_in = side * T.SSD_W / 2 + dy
            pads.append(("pad", np.array([cx, y_in + side * 0.0032, z0 + 0.012]), np.array([0.027 + 0.004, 0.0064,
                                                                                               0.024])))
        assert _overlaps(_carrier_boxes(rail), pads) == [], (dy, rail)


def test_tab_pads_clear_the_rest():
    # The other arm pinches the tab, pad tips 14 mm above the table; pads 27 mm along x, 6.4 mm thick, 24 mm up.
    # Nothing but the tab may be in the pads' way, and the pads stay outboard of the drive (even 3 mm toward the rail).
    tx = sum(T.TAB_X) / 2000
    y0, y1 = T.TAB_Y[0] / 1000, T.TAB_Y[1] / 1000
    for rail in (0.0, 0.0065):
        pads = [("pad", np.array([tx, y0 - 0.0032 + rail, 0.017 + 0.012]), np.array([0.027 + 0.004, 0.0064, 0.024])),
                ("pad", np.array([tx, y1 + 0.0032 + rail, 0.017 + 0.012]), np.array([0.027 + 0.004, 0.0064, 0.024]))]
        others = [b for b in _carrier_boxes(rail) if not (b[0] == "rail" and b[2][2] > 0.02)]
        assert _overlaps(others, pads) == []
        assert y0 - 0.0064 > (T.HALF_W + 3.0 + 2.0) / 1000


def _turned(boxes, xy, yaw):
    """Boxes turned by a multiple of 90 degrees about z and moved to ``xy`` (world)."""
    c, s = round(math.cos(yaw)), round(math.sin(yaw))
    rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return [(n, rot @ cen + np.array([*xy, 0]), np.abs(rot) @ size) for n, cen, size in boxes]


def test_start_layout_has_nothing_overlapping():
    ssd = _turned(_boxes(T.SSD_BOXES, (0, 0, 0), "ssd"), T.SSD_XY, T.SSD_YAW)
    car = _turned(_carrier_boxes(0.0), T.CARRIER_XY, T.CARRIER_YAW)
    assert _overlaps(car, ssd) == []
    ssd_hi = max(c[1] + s[1] / 2 for _, c, s in ssd)
    car_lo = min(c[1] - s[1] / 2 for _, c, s in car)
    assert ssd_hi < car_lo - 0.05      # 5 cm apart
