"""Beside the fixture area stands a single-handle faucet: a dark body with a forward spout and one
blue knob on a stem, guided by four posts so it turns but cannot slide off. A small light-blue tub
rests on the table on the right arm's side. NSPIRE D161 (Sink): a faucet that cannot activate or
deactivate water fails the inspection — and the check that matters is operational: water runs while
open, stops when shut, and the fixture is used, not just photographed.

Filling a vessel is that check, end to end: one arm positions the tub under the spout, the other
opens the knob and lets it run for a target dwell (8 seconds stands in for the fill volume — no
fluids are simulated), shuts the knob off, and the tub is returned upright without spilling. Score
components: the fill dwell, the shut-off, and a clean return.

Run it:  inspect-robots run --embodiment yam_arena -E task=vessel_fill.py \
           --instruction "fill the tub under the spout, then shut the water off and return the tub" \
           --scorer task_score ...
"""

import numpy as np

from yam_arena.demo import TIPS_ON_TABLE, Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BODY = Box("body", pos=(0.30, 0.0, TOP + 0.035), size=(0.05, 0.17, 0.07), color=(0.05, 0.06, 0.07))
SPOUT = Box("spout", pos=(0.34, 0.0, TOP + 0.20), size=(0.08, 0.024, 0.025), color=(0.05, 0.06, 0.07))
RISER = Box("riser", pos=(0.30, 0.0, TOP + 0.14), size=(0.03, 0.03, 0.14), color=(0.05, 0.06, 0.07))
STEM = Cylinder("stem", pos=(0.30, 0.065, TOP + 0.12), radius=0.014, height=0.03, color=(0.55, 0.57, 0.60))
KNOB = Box("knob", pos=(0.30, 0.065, TOP + 0.155), size=(0.05, 0.05, 0.04), mass=0.15,
           color=(0.15, 0.35, 0.75))
_POST_OFF = 0.0311
POSTS = [Cylinder(f"post{i}", pos=(0.30 + sx, 0.065 + sy, TOP + 0.091), radius=0.006, height=0.042,
                  color=(0.17, 0.19, 0.21))
         for i, (sx, sy) in enumerate([(_POST_OFF, _POST_OFF), (_POST_OFF, -_POST_OFF),
                                       (-_POST_OFF, _POST_OFF), (-_POST_OFF, -_POST_OFF)])]
# The tub: a square 5.2 cm cup — flat faces let the parallel jaw carry it without tipping (a smooth
# cylinder pinched at its sides pivots freely); spawned 2 mm proud and settled.
TUB_REST = (0.44, -0.18, TOP + 0.03)
TUB = Box("tub", pos=(0.44, -0.18, TOP + 0.032), size=(0.052, 0.052, 0.06), mass=0.08,
          color=(0.55, 0.75, 0.90))

OBJECTS = [BODY, SPOUT, RISER, STEM, KNOB] + POSTS + [TUB]

FILL_XY = (0.41, 0.0)                        # the tub's seat under the spout's outlet
FILL_TOL = 0.04                              # tub centre within this of the seat counts as under it
DWELL_TARGET = 8.0                           # seconds of open-tap dwell that stands for the volume
FLOW_MARGIN = np.radians(25.0)               # tap runs when the square knob sits >= this off either
                                             # closed orientation (0/180 deg; a square knob is
                                             # 180-deg symmetric, so "closed" repeats every 180)
CLOSED_YAW = np.radians(15.0)                # knob within this of a closed orientation counts as shut
SPILL_TILT = np.radians(45.0)                # tub tilt beyond this is a spill (dumped, not carried)
RETURN_XY_TOL = 0.04                         # the tub's home tolerance
RETURN_TILT = np.radians(15.0)               # upright at the end

SUCCESS_HOLD_S = 0.4

_state = {"dwell": 0.0, "spilled": False}


def _knob_yaw(objects):
    x, y, z, w = objects["knob"].quat_xyzw
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + x * x))


def _tilt(tub):
    x, y, _, _ = tub.quat_xyzw
    return np.arccos(np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0))


def _update(objects):
    """Idempotent per-call state: the spill flag. (The dwell accumulates in success(), which the
    embodiment calls exactly once per control step; distance()/score() also land here.)"""
    if _tilt(objects["tub"]) > SPILL_TILT:
        _state["spilled"] = True


def _valve_off_angle(objects):
    """Angle between the square knob and the nearest closed orientation (0 or 180 deg)."""
    a = _knob_yaw(objects) % np.pi
    return min(a, np.pi - a)


def _dwell_step(objects):
    under = np.hypot(objects["tub"].pos[0] - FILL_XY[0], objects["tub"].pos[1] - FILL_XY[1]) < FILL_TOL
    if (under and _valve_off_angle(objects) >= FLOW_MARGIN
            and _tilt(objects["tub"]) <= SPILL_TILT):
        _state["dwell"] += 1.0 / 20.0               # one control step at 20 Hz


def _shut_off(objects):
    return _valve_off_angle(objects) <= CLOSED_YAW


def _returned(objects):
    t = objects["tub"]
    home = np.hypot(t.pos[0] - TUB_REST[0], t.pos[1] - TUB_REST[1]) < RETURN_XY_TOL
    return home and _tilt(t) <= RETURN_TILT

def success(objects):
    """The tub was held under the open spout for the target dwell, the knob is shut off, and the tub
    is back home upright without ever having tipped past the spill angle."""
    _update(objects)
    _dwell_step(objects)
    return (_state["dwell"] >= DWELL_TARGET and _shut_off(objects)
            and _returned(objects) and not _state["spilled"])


def distance(objects):
    """Seconds of dwell still missing (informational; the scorer is task_score)."""
    _update(objects)
    return max(0.0, DWELL_TARGET - _state["dwell"])


def score(objects):
    """0.5 for the fill dwell, 0.2 for shutting the knob off, 0.3 for an upright, unspilled return."""
    _update(objects)
    fill = min(1.0, _state["dwell"] / DWELL_TARGET)
    clean = _returned(objects) and not _state["spilled"]
    return 0.5 * fill + 0.2 * _shut_off(objects) + 0.3 * clean


SCORE_OVER = "end"      # the dwell accumulates and the end state (shut off, returned) is what counts


def demo():
    """The right arm carries the tub to the seat under the spout and sets it down; the left arm then
    opens the knob with the proven 12-step incremental yaw sweep (a single fast sweep slips), holds
    the fill dwell, sweeps it shut, and the right arm returns the tub."""
    z_carry = TOP + 0.04
    z_grip = TOP + 0.03
    grip, open_ = -0.021, -0.04695
    grasp_aperture, close_target, release_aperture = -0.040, -0.023, -0.030
    grasp_z = TIPS_ON_TABLE + 0.124             # pad tips on the knob's flats, per the faucet recipe
    hover_z = grasp_z + 0.07
    kx, ky = 0.30, 0.065
    grasp_yaw = -np.pi / 2
    sweep = np.deg2rad(95.0)    # the knob's yaw tracks ~1.4x the commanded wrist yaw: 95 deg
                                # commanded lands the knob ~134 deg off closed, safely mid-open
    left = [Waypoint(10.5, (kx, ky, hover_z), grasp_aperture, "wait, then hover over the knob", yaw=grasp_yaw),
            Waypoint(1.5, (kx, ky, grasp_z), grasp_aperture, "descend", yaw=grasp_yaw),
            Waypoint(0.8, (kx, ky, grasp_z), close_target, "grasp", yaw=grasp_yaw)]
    left += [Waypoint(1.0, (kx, ky, grasp_z), close_target, f"turn to full open {i}/12",
                      yaw=grasp_yaw + sweep * i / 12) for i in range(1, 13)]
    left += [Waypoint(15.0, (kx, ky, grasp_z), close_target, "hold the fill dwell", yaw=grasp_yaw + sweep)]
    left += [Waypoint(0.75, (kx, ky, grasp_z), close_target, f"shut the tap {i}/8",
                      yaw=grasp_yaw + sweep * (1 - i / 8)) for i in range(1, 9)]
    left += [Waypoint(0.8, (kx, ky, grasp_z), release_aperture, "release", yaw=grasp_yaw),
             Waypoint(1.0, (kx, ky, hover_z + 0.05), release_aperture, "retreat", yaw=grasp_yaw)]
    return {
        "right": [
            Waypoint(2.0, (0.44, -0.18, z_grip), open_, "descend open around the tub"),
            Waypoint(0.8, (0.44, -0.18, z_grip), grip, "close on the tub"),
            Waypoint(2.5, (0.44, -0.18, z_carry), grip, "lift the tub"),
            Waypoint(3.0, (0.41, 0.005, z_carry), grip, "carry the tub in"),
            Waypoint(2.0, (0.41, 0.005, z_grip), grip, "lower the tub under the spout"),
            Waypoint(0.5, (0.41, 0.005, z_grip), open_, "release"),
            Waypoint(19.2, (0.41, 0.005, z_grip), open_, "wait while the tap fills"),
            Waypoint(1.5, (0.41, 0.005, z_carry), open_, "approach the tub"),
            Waypoint(0.8, (0.41, 0.005, z_grip), open_, "descend open around the tub"),
            Waypoint(0.6, (0.41, 0.005, z_grip), grip, "close on the tub"),
            Waypoint(2.5, (0.41, 0.005, z_carry), grip, "lift the tub"),
            Waypoint(3.0, (0.42, -0.06, z_carry), grip, "carry the tub back"),
            Waypoint(2.0, (0.44, -0.18, z_carry), grip, "over home"),
            Waypoint(1.5, (0.44, -0.18, z_grip), grip, "set the tub home"),
            Waypoint(0.5, (0.44, -0.18, z_grip), open_, "release"),
            Waypoint(1.0, (0.44, -0.18, z_carry), open_, "retreat"),
        ],
        "left": left,
    }
