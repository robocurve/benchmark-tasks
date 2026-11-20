"""The cap on the inhaler's mouthpiece, declared into mechkit for a task file (task.py) and for its known-load check
(cap_task.py). A mechkit helper module: it may declare (MECHKIT_HELPER below).

The cap's walls really go round the mouthpiece (0.3 mm play, a 1 mm lead-in at its rim: parts.py), so the hand slides it
on and its end drives a coin that sticks out of the mouth into the mouthpiece. When the cap's end has come within
SEAT_GAP of the mouth's end, its axes lined up with the mouthpiece's, the cap is welded to the inhaler: the seat. The
weld stands in for the push fit's hold (about 5 N to pull it off, (a)); the push itself meets only the walls' contact.
"""

from __future__ import annotations

import dataclasses
import math

import mechkit as mk
import numpy as np
import parts as P
from mechkit import matrix_to_quat, quat_to_matrix, weld

MECHKIT_HELPER = True  # mechkit counts this module's declarations as the task file's own

# The inhaler's body frame B, turned from the world about the vertical only (a weld on a body built rolled held about
# ten times looser in spike_make, run w9k6); the inhaler's own frame I (parts.py) is B turned by I_IN_B and moved.
BODY_IN_I = np.array([0.010, 0.0, 0.030])  # B's origin in I: inside the boot's solid, 30 mm up the barrel
I_IN_B = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])  # I's axes in B: lying on its side, the long axis of the opening up
SEAT_GAP = 0.001  # m: seated, the cap's end within 1 mm of the mouth's end
SEAT_AXIS, SEAT_OFF = math.radians(10.0), 0.001  # and its axis within 10 deg, its middle within 1 mm of the mouthpiece's


def body_pose(i_pos, i_rot) -> tuple[np.ndarray, np.ndarray]:
    """The inhaler body's frame B in the world for its frame I at ``i_pos``, ``i_rot``."""
    i_pos, i_rot = np.asarray(i_pos, float), np.asarray(i_rot, float)
    return i_pos + i_rot @ BODY_IN_I, i_rot @ I_IN_B.T


def inhaler_body_shapes() -> list:
    """parts.inhaler_shapes() (in I) put into the body frame B."""
    out = []
    for s in P.inhaler_shapes():
        pos = I_IN_B @ (np.asarray(s.pos, float) - BODY_IN_I)
        out.append(dataclasses.replace(s, pos=tuple(float(v) for v in pos), quat=tuple(matrix_to_quat(I_IN_B @ quat_to_matrix(s.quat)))))
    return out


def i_pose() -> tuple[np.ndarray, np.ndarray]:
    """The inhaler's frame I in the world, from its body's pose."""
    p, r = mk.pose("inhaler")
    r_i = r @ I_IN_B
    return p - r_i @ BODY_IN_I, r_i


def declare() -> None:
    """The seat: the cap welded to the inhaler when it is pushed home (the cap and the inhaler declared before)."""
    weld("cap_seat", "cap", "inhaler", made=False)


def rel(name: str) -> tuple[np.ndarray, np.ndarray]:
    """Body ``name``'s position and rotation in the inhaler's frame I."""
    p, r = i_pose()
    q, s = mk.pose(name)
    return r.T @ (q - p), r.T @ s


def seated() -> bool:
    """The cap is welded at its seat."""
    return not mk.is_cut("cap_seat")



def gap() -> float:
    """m from the mouth's end to the inside of the cap's end along the mouthpiece (0: pushed home)."""
    return float(rel("cap")[0][0] + 0.5 * P.CAP_DEPTH - P.CAP_END - P.MOUTH_X)


def lined_up() -> bool:
    """The cap's axis along the mouthpiece's, its opening towards the inhaler, its middle on the mouthpiece's axis."""
    p, r = rel("cap")
    return bool(-r[0, 0] >= math.cos(SEAT_AXIS) and math.hypot(p[1], p[2]) <= SEAT_OFF)


def coin_in(name: str = "coin") -> bool:
    """Coin ``name`` reaches into the mouthpiece's bore: its middle inside the bore's oval across, on the mouthpiece's
    side of the barrel's axis, and less than its own reach along the mouthpiece past the mouth's end (its radius for a
    coin lying across the bore, half its thickness for one standing face on to the mouth), so that some of it is in."""
    radius, thick = (P.LODGED_R, P.LODGED_T) if name == "coin" else (P.DIME_R, P.DIME_T)
    c, r = rel(name)
    along = min(1.0, abs(float(r[0, 2])))  # the coin's axis (its body's z) along the mouthpiece
    reach = radius * math.sqrt(1.0 - along**2) + 0.5 * thick * along
    return bool(P.BORE_BACK - 0.005 <= c[0] < P.MOUTH_X + reach and (c[1] / P.BORE_A) ** 2 + (c[2] / P.BORE_B) ** 2 <= 1.0)


def reset() -> None:
    """From the task's @on_reset function (mechkit's reset takes the seat's weld away by itself)."""


def watch(events: list) -> None:
    """Once per control step: a cap pushed home, lined up, is welded at its seat."""
    if not seated() and gap() <= SEAT_GAP and lined_up():
        mk.make("cap_seat")
        events.append((mk.now(), f"cap pushed home ({1000 * gap():.2f} mm from the mouth's end): seated"))
