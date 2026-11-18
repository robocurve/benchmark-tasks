"""At a theatre's ticket check, a patron's ticket stands on edge in the slot of the usher's ticket stand on the
counter: a card ticket with a perforated stub at its right-hand end. Beside the stand is the stub box.

The robot must tear the stub off along the perforation, put the stub in the stub box and leave the ticket in its stand
for the patron to take. The perforation holds the two parts together until it is pulled or bent hard enough; torn
anywhere but along it, the card would not part.

Everything is a real product at true size: a 2 x 8 in (51 x 203 mm) card event ticket with a 2 in perforated stub
(3.3 g), a
slotted wooden ticket stand and a stub box.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import dataclasses
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import PartState, finger_points, on_reset, on_step, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Tear the stub off this ticket and put it in the stub box."

CARD = (0.93, 0.90, 0.80)
RED = (0.72, 0.12, 0.12)
WOOD = (0.55, 0.38, 0.22)
BOX = (0.20, 0.20, 0.22)

# -- the ticket stand (fixed): a block with a slot the ticket stands in on edge, along y
STAND_X = 0.300
BLOCK_H, SLOT_W, SLOT_D = 0.020, 0.003, 0.008       # block height, slot width and depth
BLOCK = (0.040, 0.240)                              # block depth (x) and length (y)
STAND = [Box(f"stand_{k}", pos=(STAND_X + s * (SLOT_W / 2 + (BLOCK[0] / 2 - SLOT_W / 2) / 2), 0.0, TOP + BLOCK_H / 2),
             size=(BLOCK[0] / 2 - SLOT_W / 2, BLOCK[1], BLOCK_H), color=WOOD) for k, s in enumerate((1, -1))]
STAND.append(Box("stand_slot_floor", pos=(STAND_X, 0.0, TOP + (BLOCK_H - SLOT_D) / 2), size=(SLOT_W, BLOCK[1],
                                                                                           BLOCK_H - SLOT_D), color=WOOD))

# -- the ticket: two loose card bodies joined along the perforation by a breakable joint
CARD_T = 0.0008                                     # 0.8 mm (the real card is 0.3 mm; thinner is not reliable here)
CARD_H = 0.051
TICKET_Y = (-0.1015, 0.1015)                        # the whole 8 in ticket along y; the stub at the -y (right) end
PERF_Y = -0.0505                                    # the perforation: a 2 in stub
CARD_Z0 = BLOCK_H - SLOT_D + 0.0005                 # its lower edge, in the slot
TEAR_FORCE, TEAR_TORQUE = 8.0, 0.04                 # pulled apart with 8 N, or bent with 0.04 N m, it tears
MAIN_MASS, STUB_MASS = 0.0025, 0.0008


def _card(name, y0, y1, mass, stripe=None):
    h = CARD_H
    boxes = [((0.0, 0.0, h / 2), (CARD_T, y1 - y0, h), CARD)]
    if stripe:
        boxes.append(((0.0, 0.0, h * 0.8), (CARD_T + 0.0002, y1 - y0, h * 0.14), stripe))
    return part(name, pos=(STAND_X, (y0 + y1) / 2, TOP + CARD_Z0), mass=mass, color=CARD, friction=0.9, boxes=boxes)


TICKET = _card("ticket", PERF_Y, TICKET_Y[1], MAIN_MASS, stripe=RED)
STUB = dataclasses.replace(_card("stub", TICKET_Y[0], PERF_Y, STUB_MASS, stripe=RED),
                           meta={"weld": {"to": "ticket", "force": TEAR_FORCE, "torque": TEAR_TORQUE}})

# -- the stub box: an open-topped box on the counter to the right
STUB_BOX_XY, STUB_BOX = (0.250, -0.200), (0.090, 0.090, 0.050)
_bx, _by = STUB_BOX_XY
_w = 0.004
BOXES = [Box("stub_box_floor", pos=(_bx, _by, TOP + _w / 2), size=(STUB_BOX[0], STUB_BOX[1], _w), color=BOX)]
for k, (dx, dy, sx, sy) in enumerate(((STUB_BOX[0] / 2, 0, _w, STUB_BOX[1]), (-STUB_BOX[0] / 2, 0, _w, STUB_BOX[1]),
                                      (0, STUB_BOX[1] / 2, STUB_BOX[0], _w), (0, -STUB_BOX[1] / 2, STUB_BOX[0], _w))):
    BOXES.append(Box(f"stub_box_wall_{k}", pos=(_bx + dx, _by + dy, TOP + STUB_BOX[2] / 2), size=(sx, sy, STUB_BOX[2]),
                     color=BOX))
OBJECTS = [*STAND, TICKET, STUB, *BOXES]

# -- scoring ------------------------------------------------------------------------------------------------------------
TORN_GAP = 0.004                                    # the two parts' perforation edges this far apart: torn
UPRIGHT = math.radians(25.0)
CLEAR = 0.030
STILL_STEPS = 20
STILL_TOL = 0.001
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(torn=False, clear=False, recent=[], steps=0)


_reset_state()


def tilt(state: PartState) -> float:
    return math.acos(max(-1.0, min(1.0, float(state.rotation()[2, 2]))))


def perf_gap(ticket: PartState, stub: PartState) -> float:
    """Distance between the stub's and the ticket's halves of the perforation (its middle point on each part)."""
    a = ticket.to_world((0.0, PERF_Y - (PERF_Y + TICKET_Y[1]) / 2, CARD_H / 2))
    b = stub.to_world((0.0, PERF_Y - (TICKET_Y[0] + PERF_Y) / 2, CARD_H / 2))
    return float(np.linalg.norm(a - b))


def stub_in_box(stub: PartState) -> bool:
    p = stub.to_world((0.0, 0.0, CARD_H / 2))
    return (abs(float(p[0]) - _bx) < STUB_BOX[0] / 2 - _w and abs(float(p[1]) - _by) < STUB_BOX[1] / 2 - _w
            and float(p[2]) < TOP + STUB_BOX[2])


def ticket_in_stand(ticket: PartState) -> bool:
    """The ticket still standing in the stand's slot: its lower edge in the slot and leaning less than 25 degrees."""
    p = ticket.to_world((0.0, 0.0, 0.0))
    return abs(float(p[0]) - STAND_X) < SLOT_W and float(p[2]) < TOP + BLOCK_H and tilt(ticket) < UPRIGHT


def box_distance(point, state: PartState, y0, y1) -> float:
    p = state.to_local(point) - np.array([0.0, 0.0, CARD_H / 2])
    half = np.array([CARD_T / 2, (y1 - y0) / 2, CARD_H / 2])
    return float(np.linalg.norm(np.maximum(0.0, np.abs(p) - half)))


def observe(ticket: PartState, stub: PartState, fingers) -> None:
    s = _STATE
    if perf_gap(ticket, stub) > TORN_GAP:
        s["torn"] = True                            # a torn perforation stays torn
    s["clear"] = fingers is not None and all(
        box_distance(f, ticket, PERF_Y, TICKET_Y[1]) >= CLEAR and box_distance(f, stub, TICKET_Y[0], PERF_Y) >= CLEAR
        for f in fingers)
    s["recent"].append(np.concatenate([np.asarray(ticket.pos, float), np.asarray(stub.pos, float)]))
    del s["recent"][:-STILL_STEPS]
    s["steps"] += 1


@on_step
def _track(environment) -> None:
    from partkit import _part_state

    ticket, stub = _part_state(environment, TICKET), _part_state(environment, STUB)
    try:
        fingers = [np.asarray(p, float) for p in finger_points(environment)]
    except Exception as exc:  # noqa: BLE001 - an unreadable gripper never counts as clear
        if not _STATE.get("warned"):
            print(f"ticket-stub-tear: gripper check unavailable: {exc!r}", flush=True)
            _STATE["warned"] = True
        fingers = None
    observe(ticket, stub, fingers)


def _still() -> bool:
    recent = _STATE["recent"]
    return len(recent) == STILL_STEPS and float(np.ptp(np.array(recent), axis=0).max()) < STILL_TOL


def distance(objects) -> float:
    """Metres from the stub to the middle of the stub box."""
    p = objects["stub"].to_world((0.0, 0.0, CARD_H / 2))
    return float(np.linalg.norm(p - np.array([_bx, _by, TOP + STUB_BOX[2] / 2])))


def _level(objects) -> tuple[float, bool]:
    ticket, stub = objects["ticket"], objects["stub"]
    torn = _STATE["torn"]
    if torn and stub_in_box(stub) and ticket_in_stand(ticket):
        if _still() and _STATE["clear"]:
            return 1.0, True
        return 0.75, False
    if torn and stub_in_box(stub):
        return 0.5, False
    if torn:
        return 0.25, False
    return 0.0, False


def success(objects) -> bool:
    """The stub torn off along the perforation and lying in the stub box, the ticket still standing in its stand (its
    lower edge in the slot, leaning under 25 degrees), both grippers 3 cm clear of both and nothing moving for 1 s."""
    return _level(objects)[1]


def score(objects) -> float:
    """1.0 success; 0.75 torn, stub in the box and ticket in its stand, but a gripper is near or something moves; 0.5
    torn with the stub in the box but the ticket out of its stand; 0.25 the stub was torn off at some point; else 0."""
    return _level(objects)[0]
