"""Blister-card medication pass: push the tablets due now out of a resident's blister cards into a medicine cup.

Three 30-day blister cards (``blister/``: design, USD, foil mechanics) are spawned through ``yampatch.py`` as
articulations, one foil flap under each filled cell; the tablets are task-file cylinders resting on the flaps.
``on_step`` tears foil that a tablet has been pushed through, ``on_reset`` restores the cards and tablets. The
medication record is a printed sheet on the table. The docstring an agent reads is set at the end of this file.

Run the scripted demo:  inspect-robots run --embodiment yam_arena --policy yam_scripted \\
    -E task=tasks/blister_med_pass/blister_med_pass.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 \\
    -E cam_height=176 --scorer task_score --max-steps 2400 \\
    --instruction "It is 08:00. Give the resident's 08:00 medications into the medicine cup, as the medication record says."
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

# Task files are imported by path, so this folder is not on sys.path: add it for yampatch.py and the blister package.
_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

import yampatch  # noqa: E402
from blister import design as d  # noqa: E402  (pure Python; usd and sim are imported after Isaac Sim starts)

SCENE = "station"
TOP = TABLE_TOP[SCENE]
REST_GAP = 0.0005  # cards and cup spawn this far above their support and settle onto it
STAND = (0.20, 0.05, 0.015)  # each card rests on a wooden stand narrower than itself (x, y, z) [m]: its long edges
# overhang by 1.5 cm, so fingers can close on them
CARD_Z = TOP + STAND[2] + REST_GAP  # a card's underside at the start
CARD_XY = {"furosemide": (0.27, 0.10), "metoprolol": (0.27, 0.20), "atorvastatin": (0.27, 0.30)}
CUP_XY = (0.30, -0.08)
MAR_XY = (0.12, -0.07)  # the record's centre: in front of the cup, clear of the stands
CARDS = {c.name: c for c in d.CARDS}
_DEBUG = bool(os.environ.get("BLISTER_DEBUG"))


def _make_card(card: d.Card):
    def make():
        from blister.sim import CardObject

        return CardObject(card, (*CARD_XY[card.name], CARD_Z))

    return make


def _build(what: str):
    def build():
        from blister import usd

        return getattr(usd, f"build_{what}")()

    return build


def _pills() -> list[Cylinder]:
    out = []
    for card in d.CARDS:
        cx, cy = CARD_XY[card.name]
        for cell in card.full_cells():
            x, y = d.cell_xy(cell)
            out.append(Cylinder(card.pill_name(cell), (cx + x, cy + y, CARD_Z + d.PILL_Z), card.pill_color,
                                d.PILL_MASS, radius=d.PILL_RADIUS, height=d.PILL_HEIGHT))  # fmt: skip
    return out


OBJECTS = [
    *(Box(f"{c.name}_stand", (*CARD_XY[c.name], TOP + STAND[2] / 2), (0.62, 0.48, 0.32), size=STAND) for c in d.CARDS),
    *(yampatch.usd_object(c.name, (*CARD_XY[c.name], CARD_Z), kind="articulation", make=_make_card(c),
                          extent=d.CARD_SIZE[0]) for c in d.CARDS),
    yampatch.usd_object("cup", (*CUP_XY, TOP + REST_GAP), kind="rigid", build=_build("cup"), mass=d.CUP_MASS,
                        extent=2 * d.CUP_RADIUS),
    yampatch.usd_object("mar", (*MAR_XY, TOP), kind="fixed", build=_build("mar"), extent=0.18),
    *_pills(),
]  # fmt: skip


# -- state -----------------------------------------------------------------------------------------------------------

_CARDS = None  # blister.sim.Cards, once the simulator runs


def _cards(env):
    global _CARDS
    if _CARDS is None:
        from blister.sim import Cards

        _CARDS = Cards(env)
    return _CARDS


_FELL: set[str] = set()  # tablets that came to rest outside the cup: dropped for good (discarded, not given)


def on_reset(env) -> None:
    _cards(env).reset()
    _FELL.clear()


def on_step(env, states) -> None:
    for name, cell in _cards(env).after_step():
        if _DEBUG:
            print(f"[blister] foil torn: {name} cell {cell}", flush=True)
    for pill, out, inside in _whereabouts(states):
        if out and not inside and pill not in _FELL and states[pill].resting():
            if states[pill].pos[2] < states["cup"].low[2] + d.CUP_HEIGHT:  # on the table, a block or beside the cup
                _FELL.add(pill)
                if _DEBUG:
                    print(f"[blister] dropped: {pill}", flush=True)


yampatch.install(on_reset=on_reset, on_step=on_step)


# -- scoring ---------------------------------------------------------------------------------------------------------

DUE = d.due()  # {"furosemide": 2, "metoprolol": 1, "atorvastatin": 0}
N_DUE = sum(DUE.values())
OUT_OF_CELL = 0.008  # m: a tablet this far from its cell's centre has left the card


def _rotate(quat_xyzw, v) -> np.ndarray:
    x, y, z, w = quat_xyzw
    u, v = np.array([x, y, z]), np.asarray(v, dtype=float)
    return v + 2 * w * np.cross(u, v) + 2 * np.cross(u, np.cross(u, v))


def _in_cup(pill, cup) -> bool:
    """Inside the cup: within its inner radius of its axis, and between its floor and rim."""
    floor = cup.low[2]
    return bool(np.linalg.norm(pill.pos[:2] - cup.pos[:2]) < d.CUP_RADIUS and floor < pill.pos[2] < floor + d.CUP_HEIGHT)


def _whereabouts(objects):
    """(tablet, out of its cell, in the cup) for every tablet."""
    for card in d.CARDS:
        st = objects[card.name]
        for cell in card.full_cells():
            pill = card.pill_name(cell)
            x, y = d.cell_xy(cell)
            home = st.pos + _rotate(st.quat_xyzw, (x, y, d.PILL_Z))
            out = bool(np.linalg.norm(objects[pill].pos - home) >= OUT_OF_CELL)
            yield pill, out, out and _in_cup(objects[pill], objects["cup"])


def tally(objects) -> tuple[dict[str, int], int]:
    """Tablets in the cup per card, and tablets dropped: out of their card but not in the cup, or that ever came to
    rest outside the cup (put in the cup afterwards, they still count as dropped, not given)."""
    in_cup, dropped = dict.fromkeys(CARDS, 0), 0
    card_of = {c.pill_name(cell): c.name for c in d.CARDS for cell in c.full_cells()}
    for pill, out, inside in _whereabouts(objects):
        if pill in _FELL:
            dropped += 1
        elif inside:
            in_cup[card_of[pill]] += 1
        elif out:
            dropped += 1
    return in_cup, dropped


def _wrong(in_cup) -> bool:
    return any(n > DUE[name] for name, n in in_cup.items())


def success(objects):
    """Exactly the due tablets in the cup, none dropped, and all of them at rest."""
    in_cup, dropped = tally(objects)
    return in_cup == DUE and dropped == 0 and all(objects[p].resting() for p in _pill_names())


def score(objects):
    """1/3 per due tablet in the cup, minus 1/3 per tablet dropped outside it (at least 0); 0 if the cup holds any
    tablet not due now (a wrong drug, or more of a drug than the dose)."""
    in_cup, dropped = tally(objects)
    if _wrong(in_cup):
        return 0.0
    return round(max(0.0, (sum(in_cup.values()) - dropped) / N_DUE), 6)


def _pill_names():
    return [c.pill_name(cell) for c in d.CARDS for cell in c.full_cells()]


# -- scripted demo ---------------------------------------------------------------------------------------------------
# First the right arm holds its wrist camera over the medication record to read it, as a solving run must. Then the
# left arm pinches a card by its two long edges near the label, lifts it off its stand, turns it a quarter turn so
# its cells point at the right arm and sets the card on the cup's rim, the cell to give over the cup and the rim under
# both of the card's halves; the right arm presses the bubble with its closed fingers until the tablet drops through
# the foil into the cup (the rim takes the push). The arms work 18 cm apart, so their wrists stay clear of each other.
# Each card goes back on its stand.

L, W, T = d.CARD_SIZE
GRIP_BACK = 0.18  # the pinch is this far from the first pressed cell, towards the label
GRIP_Z = CARD_Z - 0.012 + PAD_TIP_PAST_TCP  # pad tips 12 mm below the card's underside (the flat of the pads), beside its stand
PINCH = -(W / 2 - 0.002)  # finger target: pads 4 mm narrower than the card, a gentle squeeze (about 10 N)
CARRY_Z = GRIP_Z + (TOP + 0.06 - CARD_Z)  # the card's underside 6 cm above the table: clear of the cup and stands
ON_CUP_Z = GRIP_Z + (TOP + REST_GAP + d.CUP_HEIGHT + 0.001 - CARD_Z)  # the card's underside 1 mm above the rim
CARD_TOP_ON_CUP = TOP + REST_GAP + d.CUP_HEIGHT + 0.001 + T
PUSH_HOVER = CARD_TOP_ON_CUP + 0.03 + PAD_TIP_PAST_TCP
# A firm press: the arm's position control is soft, so the press aims well below the card's top face (a teleoperator
# pushing firmly does the same with the leader arm), and the fingertip drifts while it presses, so it aims beside the
# bubble. Measured on the bubble: 11 to 16 N.
PUSH_IN = {"furosemide": 0.04, "metoprolol": 0.025}  # m below the card's top face
PUSH_AIM = np.array([0.006, 0.011])  # m (world x, y): where the press aims, from the bubble
CUP_SHIFT = 0.6  # the card point over the cup's centre: this fraction of the way from the card's centre line to the
# cell, so the cell is over the cup and the rim is under both halves of the card
HOVER = TOP + 0.12
PLAN = (("furosemide", (25, 26)), ("metoprolol", (27,)))
TURN = -np.pi / 2  # the held card's yaw: its cells towards the right arm
PRESS_YAW = np.pi / 2  # the pressing gripper's finger rails along the card
WAIT_XY = {"left": (0.20, 0.20), "right": (0.20, -0.20)}
READ_Z = TOP + 0.22  # the right wrist camera over the record, as high as the arm reaches it
READ_YAW = 0.0  # its image rows along the sheet's lines
READ_XY = (MAR_XY[0] + 0.06, MAR_XY[1])  # the camera looks back towards the arm: this centres the sheet in its view


def _wp(t, xyz, finger, label, yaw=0.0):
    return Waypoint(t, tuple(float(v) for v in xyz), finger, label, float(yaw))


def _turned(yaw: float, v) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def _duration(plan) -> float:
    return sum(w.duration for w in plan)


def _wait_until(plan, t, label="wait") -> None:
    if _duration(plan) < t:
        last = plan[-1]
        plan.append(Waypoint(t - _duration(plan), last.pos, last.finger, label, last.yaw))


def demo():
    hold = [_wp(1.5, (*WAIT_XY["left"], HOVER), FINGER_CLOSED, "ready")]
    push = [_wp(2.0, (*READ_XY, READ_Z), FINGER_CLOSED, "over the medication record", READ_YAW),
            _wp(2.0, (*READ_XY, READ_Z), FINGER_CLOSED, "read it", READ_YAW),
            _wp(1.5, (*WAIT_XY["right"], HOVER), FINGER_CLOSED, "ready")]
    _wait_until(hold, _duration(push) - 1.5, "wait for the record")
    for name, cells in PLAN:
        grip = np.array([d.cell_xy(cells[0])[0] - GRIP_BACK, 0.0])  # the pinch, in the card's frame
        home = np.array(CARD_XY[name]) + grip
        hold += [_wp(1.5, (*home, HOVER), FINGER_OPEN, f"above {name}"),
                 _wp(1.2, (*home, GRIP_Z), FINGER_OPEN, "down beside its edges"),
                 _wp(0.8, (*home, GRIP_Z), PINCH, "pinch the card"),
                 _wp(1.0, (*home, CARRY_Z), PINCH, "lift")]  # fmt: skip
        for cell in cells:
            cx, cy = d.cell_xy(cell)
            at = np.array(CUP_XY) - _turned(TURN, np.array([cx, cy * CUP_SHIFT]) - grip)
            bubble = np.array(CUP_XY) + _turned(TURN, np.array([0.0, cy * (1 - CUP_SHIFT)]))
            push_z = CARD_TOP_ON_CUP - PUSH_IN[name] + PAD_TIP_PAST_TCP
            hold += [_wp(2.0, (*at, CARRY_Z), PINCH, f"cell {cell} over the cup", TURN),
                     _wp(1.0, (*at, ON_CUP_Z), PINCH, "onto the rim", TURN),
                     _wp(0.3, (*at, ON_CUP_Z), PINCH, "settle", TURN)]  # fmt: skip
            _wait_until(push, _duration(hold) - 1.5, "wait for the card")
            push += [_wp(1.5, (*bubble, PUSH_HOVER), FINGER_CLOSED, "over the bubble", PRESS_YAW),
                     _wp(1.0, (*bubble + PUSH_AIM, push_z), FINGER_CLOSED, f"press cell {cell} through", PRESS_YAW),
                     _wp(0.8, (*bubble + PUSH_AIM, push_z), FINGER_CLOSED, "hold: the tablet drops", PRESS_YAW),
                     _wp(0.6, (*bubble, PUSH_HOVER), FINGER_CLOSED, "up", PRESS_YAW)]  # fmt: skip
            _wait_until(hold, _duration(push), "hold the card")
            hold.append(_wp(0.8, (*at, CARRY_Z), PINCH, "off the rim", TURN))
        push.append(_wp(1.2, (*WAIT_XY["right"], HOVER), FINGER_CLOSED, "out of the way"))
        hold += [_wp(2.0, (*home, CARRY_Z), PINCH, f"{name} back", 0.0),
                 _wp(1.0, (*home, GRIP_Z + 0.001), PINCH, "onto its stand"),
                 _wp(0.6, (*home, GRIP_Z + 0.001), FINGER_OPEN, "let go"),
                 _wp(1.0, (*home, HOVER), FINGER_OPEN, "up")]  # fmt: skip
    hold.append(_wp(1.2, (*WAIT_XY["left"], HOVER), FINGER_CLOSED, "done"))
    t = max(_duration(hold), _duration(push))
    _wait_until(hold, t)
    _wait_until(push, t)
    return {"left": hold, "right": push}


# -- what an agent reads ---------------------------------------------------------------------------------------------


def docs() -> str:
    cards = "; ".join(
        f"{c.drug.lower()} {c.strength} at ({CARD_XY[c.name][0]:.2f}, {CARD_XY[c.name][1]:+.2f}) m, cells "
        f"{c.first_full} to 30 still full" for c in d.CARDS)  # fmt: skip
    return (
        f"A medication pass at a care home. On the table lie three of one resident's 30-day blister cards, each "
        f"{L * 100:.0f} x {W * 100:.0f} cm, long side pointing away from the arms, the pharmacy label at the near end: "
        f"{cards}. Each cell holds one tablet under a clear bubble and is sealed with foil underneath; pressing the "
        f"tablet through the bubble tears the foil and the tablet drops out below. The cells are numbered 1 to 30. "
        f"A white 1 oz paper medicine cup ({2 * d.CUP_RADIUS * 100:.1f} cm across, {d.CUP_HEIGHT * 100:.0f} cm tall) "
        f"stands at ({CUP_XY[0]:.2f}, {CUP_XY[1]:+.2f}) m. The resident's printed medication administration record "
        f"(a US letter page, {d.MAR_SIZE[1] * 100:.1f} x {d.MAR_SIZE[0] * 100:.1f} cm) lies flat at "
        f"({MAR_XY[0]:.2f}, {MAR_XY[1]:+.2f}) m, readable from the arms' side: each drug with its dose and the times "
        f"it is given. Each card lies on a wooden stand {STAND[1] * 100:.0f} cm wide and {STAND[2] * 100:.1f} cm tall, so "
        f"its long edges overhang."
    )


__doc__ = docs()
