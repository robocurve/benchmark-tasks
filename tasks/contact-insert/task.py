"""On the station table, in front of the arms: a black sealed 4-way connector housing (24 x 24 x 40 mm, glass-filled
nylon) lies on its side, its rear face (the wire side, four square cavities numbered 1-4 by moulded dots) toward the
right arm. Beside it, on a two-block rest (20 mm high, a 38 mm gap behind the crimp), lies a crimped size-16 socket contact
(21 mm, gold) on a 140 mm red 1.0 mm2 wire, its nose pointing at the housing. An orange secondary lock (a comb: a 24 x 8 mm bar with two prongs) stands in a grey holder,
and a white pinout card shows the four cavities as seen from the rear, cavity 3 marked red.

The contact goes into a cavity from the rear; pushed in about 11 mm past the rear face, its shoulder passes the
cavity's sprung locking finger, which snaps in behind it (the click). The secondary lock is pressed down into the slot
across the top of the housing; it only goes fully in when every locking finger is closed.

Task: push the crimped contact into cavity 3 of the connector, as on the pinout card, from the wire side, until it
clicks; hold the housing with one hand and push the wire close behind the crimp with the other. Tug the wire lightly to
check it is locked, then press the secondary lock fully into the slot on top of the housing. Then let go of
everything.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ct_parts as ct  # noqa: E402

from yam_arena.task_file import TABLE_TOP  # noqa: E402

ct.install()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

MM = 1e-3
HOUSING_POS = (0.30, 0.02, TOP + ct.H_H / 2 + 0.1 * MM)  # the rear face's centre; the cavities along +y
CONTACT_YAW = math.radians(-90.0)  # the contact's nose toward +y (the housing), its wire along -y
REST_POS = (0.305, -0.09, TOP + ct.REST_H)  # the lead's rest, under the crimp end
CONTACT_POS = (REST_POS[0], REST_POS[1], REST_POS[2] + ct.SHOULDER_R + 0.1 * MM)  # the crimp end
HOLDER_POS = (0.20, -0.06, TOP + ct.HOLDER_SIZE[2])  # the holder's top face's centre
LOCK_POS = (HOLDER_POS[0], HOLDER_POS[1], HOLDER_POS[2] + 0.1 * MM)  # the lock's bar on the holder
CARD_POS = (0.19, 0.10, TOP)

OBJECTS = [
    ct.part("housing", "housing", HOUSING_POS),
    ct.part("contact", "contact", CONTACT_POS, yaw=CONTACT_YAW),
    ct.part("holder", "holder", HOLDER_POS),
    ct.part("lock", "lock", LOCK_POS),
    ct.part("card", "card", CARD_POS),
]
if not os.environ.get("CT_BENCH_NO_REST"):  # (the bench puts the contact straight into a cavity: its wire would run
    OBJECTS.append(ct.part("rest", "rest", REST_POS, yaw=CONTACT_YAW))  # through the rest)

# -- scoring ---------------------------------------------------------------------------------------------------------
CREDIT = {"in": 0.25, "locked": 0.5, "lock_in": 0.75, "done": 1.0}


def in_cavity(objects) -> bool:
    """The contact in cavity 3 from the rear: its nose at least 5 mm in, on the cavity's axis (1 mm), pointing in."""
    return objects["housing"].cavity == ct.TARGET_CAVITY


def locked(objects) -> bool:
    """Seated with its locking finger closed behind the shoulder, and it held the tug (30 N for 0.5 s after a 1 s
    ramp, between wire and housing; moved under 0.5 mm)."""
    h = objects["housing"]
    return bool(h.locked and h.tug == "passed")


def hands_off(objects) -> bool:
    dt = 1.0 / 20.0
    return objects["housing"].hands_off_steps * dt >= ct.HANDS_OFF_S


def score(objects) -> float:
    """0 if the secondary lock was ever forced onto a finger held open by a half-seated contact (a prong loaded 20 N
    for 0.25 s: the housing damaged), if the contact was ever locked in a wrong cavity, or it isn't in cavity 3;
    0.25 in cavity 3; 0.5 also locked (held the tug); 0.75 also the secondary lock fully in; 1.0 also nothing held
    (no fingertip within 20 mm of the housing, contact, wire or lock for 1 s)."""
    h = objects["housing"]
    if h.forced or h.wrong_locked or not in_cavity(objects):
        return 0.0
    if not locked(objects):
        return CREDIT["in"]
    if not h.lock_in:
        return CREDIT["locked"]
    if not hands_off(objects):
        return CREDIT["lock_in"]
    return CREDIT["done"]


def success(objects) -> bool:
    """Full score and the housing, contact and lock still (each moving under 5 mm/s, 0.1 rad/s)."""
    h = objects["housing"]
    return score(objects) >= 1.0 - 1e-9 and h.lin_vel[0] < 5 * MM and h.ang_vel[0] < 0.1


def distance(objects) -> float:
    """Metres the contact's nose still has to go to be in cavity 3 (a progress measure, not the score)."""
    h = objects["housing"]
    return max(0.0, ct.IN_DEPTH - h.nose_depth) if h.cavity == ct.TARGET_CAVITY else 0.05
