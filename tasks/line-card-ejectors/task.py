"""Seat a modular switch's line card with its two ejector levers.

The lowest slot of a modular data-center switch chassis is fixed to the table, its mouth facing the arms. A line card
(400 mm wide, 460 mm deep, 4 kg) stands in the slot's guides, pulled out 60 mm, with both ejector levers swung fully
open. Over the last 8 mm the card's connectors press onto the backplane's against 80 N, more than the arms can push.
The levers give the leverage: each pivots on the faceplate, and swung shut its claw comes round behind a steel pin on
the chassis and draws its end of the card in, 8.5 times the force on its handle. Both have to be closed together: an
end drawn in alone cocks the card in its guides, and connectors driven in more than 1.5 mm unevenly bend backplane pins.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import cardkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Install the line card: seat it fully in its slot and lock both of its ejector levers shut."

LOOKS = {
    "steel": m.Look(m.rgb(0.62, 0.63, 0.65), metallic=0.6, roughness=0.45),
    "dark": m.Look(m.rgb(0.16, 0.17, 0.19), roughness=0.6),
    "face": m.Look(m.rgb(0.20, 0.21, 0.23), metallic=0.3, roughness=0.5),
    "pcb": m.Look(m.rgb(0.10, 0.35, 0.18), roughness=0.7),
    "sink": m.Look(m.rgb(0.70, 0.71, 0.73), metallic=0.8, roughness=0.35),
    "lever": m.Look(m.rgb(0.95, 0.75, 0.10), roughness=0.5),
    "pin": m.Look(m.rgb(0.85, 0.86, 0.88), metallic=0.9, roughness=0.25),
    "conn": m.Look(m.rgb(0.80, 0.70, 0.30), metallic=0.5, roughness=0.4),
    "port": m.Look(m.rgb(0.05, 0.05, 0.06), roughness=0.8),
}


def _box(name, lo, hi, look, collide=True):
    """A box from corner ``lo`` to corner ``hi`` (m): its visual mesh and, unless ``collide`` is False, its collider."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c, s = (lo + hi) / 2, hi - lo
    parts = [(name, m.box(tuple(s), tuple(c)), look)]
    cols = [m.col_box(f"{name}_c", tuple(s), tuple(c))] if collide else []
    return parts, cols


def _join(*pieces):
    parts, cols = [], []
    for p, c in pieces:
        parts += p
        cols += c
    return tuple(parts), tuple(cols)


# -- the chassis slot, in its own frame: origin on the table at the middle of the slot's mouth; x into the slot -----
X_FACE = 0.33              # world x of the chassis face (the slot's mouth)
SLOT_W = 0.403             # between the guide walls: 1.5 mm clear each side of the 400 mm card
FLOOR = 0.010              # the slot's floor, above the table
SLOT_DEPTH = 0.52
PIN_R = 0.004              # the ejector pins: 8 mm steel pins standing on tabs in front of the guide walls
PIN_X = -0.0283            # (chassis frame) 28.3 mm in front of the face
PIN_Y = 0.205
CONN_TRAVEL = 0.008        # the connectors mate over the last 8 mm ...
CONN_RATE = 5000.0         # ... against 5000 N/m each: 40 N each, 80 N together, at home
CONN_Y = 0.15
SEAT_X = X_FACE - 0.005    # world x of the card's origin at home (the faceplate 1 mm proud of the chassis face)
CARD_D = 0.460
CONN_X0 = SEAT_X - X_FACE + CARD_D - CONN_TRAVEL   # (chassis frame) a connector's face at rest: 8 mm in front of the card's rear at home

_w = SLOT_W / 2
CHASSIS_PARTS, CHASSIS_COLS = _join(
    _box("floor", (0.0, -0.235, 0.0), (SLOT_DEPTH, 0.235, FLOOR), "steel"),
    _box("tab_l", (-0.040, _w, 0.0), (0.0, 0.235, FLOOR), "steel"),
    _box("tab_r", (-0.040, -0.235, 0.0), (0.0, -_w, FLOOR), "steel"),
    _box("wall_l", (0.0, _w, FLOOR), (SLOT_DEPTH, 0.235, 0.060), "steel"),
    _box("wall_r", (0.0, -0.235, FLOOR), (SLOT_DEPTH, -_w, 0.060), "steel"),
    _box("roof", (0.0, -0.235, 0.060), (SLOT_DEPTH, 0.235, 0.064), "steel"),
    _box("backplane", (SLOT_DEPTH, -0.235, 0.0), (SLOT_DEPTH + 0.008, 0.235, 0.120), "dark"),
    _box("conn_base_l", (CONN_X0 + 0.028, CONN_Y - 0.03, 0.012), (SLOT_DEPTH, CONN_Y + 0.03, 0.045), "dark"),
    _box("ear_l", (0.0, 0.235, 0.0), (0.004, 0.262, 0.064), "steel", collide=False),       # the chassis' front flanges
    _box("ear_r", (0.0, -0.262, 0.0), (0.004, -0.235, 0.064), "steel", collide=False),
    *[_box(f"vent{i}", (0.06 + 0.04 * i, -0.17, 0.064), (0.075 + 0.04 * i, 0.17, 0.0645), "dark", collide=False)
      for i in range(10)],
    _box("conn_base_r", (CONN_X0 + 0.028, -CONN_Y - 0.03, 0.012), (SLOT_DEPTH, -CONN_Y + 0.03, 0.045), "dark"),
)
_pins = []
for _s in (1, -1):
    mesh = m.cylinder(PIN_R, 0.040, (PIN_X, _s * PIN_Y, FLOOR + 0.020))
    _pins.append(((f"pin_{'l' if _s > 0 else 'r'}", mesh, "pin"),))
CHASSIS_PARTS += tuple(p for ps in _pins for p in ps)
CHASSIS_COLS += tuple(m.col_cylinder(f"pin_{'l' if s > 0 else 'r'}_c", PIN_R, 0.040, (PIN_X, s * PIN_Y, FLOOR + 0.020))
                      for s in (1, -1))


def _connector(side: int):
    """A backplane connector's mating resistance: a sprung plunger (20 x 40 x 25 mm) whose face stands 8 mm in front of
    where the card's rear edge sits at home. The card's rear presses it back against its spring; at home it bottoms out."""
    name = "conn_l" if side > 0 else "conn_r"
    p, c = _box(name, (CONN_X0, side * CONN_Y - 0.02, 0.015), (CONN_X0 + 0.020, side * CONN_Y + 0.02, 0.040), "conn")
    return m.slider(name, "X", 0.0, CONN_TRAVEL, p, c, at=(CONN_X0, side * CONN_Y, 0.03), spring=CONN_RATE,
                    damping=40.0, friction=0.0, mass=0.3, effort=5000.0)


CHASSIS = m.mechanism("chassis", (X_FACE, 0.0, TOP), [_connector(1), _connector(-1)], LOOKS, frame_parts=CHASSIS_PARTS,
                      frame_colliders=CHASSIS_COLS, friction=(0.12, 0.12))

# -- the line card, in its own frame: origin at the bottom of the faceplate's front at the middle; x into the slot ----
CARD_W = 0.400
CARD_H = 0.035
CARD_MASS = 4.0
FACE_HALF_W = 0.186
PIVOT_X = -0.016           # each lever's pivot, 16 mm in front of the faceplate, 8 mm inside the card's edge
PIVOT_Y = 0.192
CLAW_R = 0.013             # the pin sits 13 mm out from the pivot when the lever is shut
HANDLE_L = 0.110           # pivot to the handle's finger tab
OPEN_DEG = 85.0            # the levers start swung fully open, the claws pointing into the slot past the pins
OPEN_LIMIT = 85.0
CLAW_L = 0.030             # pivot to the claw's end
START_OUT = 0.060          # the card starts pulled out this far
LATCH_DEG = 2.5            # a lever within this of shut, with its end of the card home, snaps into its catch
CATCH_GAP = 0.0015
CATCH_RATE = 200.0         # the catch holds a lever shut: N m/rad, up to 10 N m
CATCH_EFFORT = 10.0


def _card_frame():
    body = _join(
        _box("carrier", (0.004, -CARD_W / 2, 0.0), (CARD_D, CARD_W / 2, 0.004), "steel"),
        _box("pcb", (0.004, -CARD_W / 2 + 0.003, 0.004), (CARD_D - 0.002, CARD_W / 2 - 0.003, 0.0056), "pcb",
             collide=False),
        _box("sinks", (0.10, -0.15, 0.0056), (0.36, 0.15, CARD_H), "sink"),
        _box("faceplate", (0.0, -FACE_HALF_W, 0.002), (0.004, FACE_HALF_W, 0.042), "face"),
        _box("plug_l", (CARD_D - 0.020, CONN_Y - 0.025, 0.004), (CARD_D, CONN_Y + 0.025, 0.030), "conn"),
        _box("plug_r", (CARD_D - 0.020, -CONN_Y - 0.025, 0.004), (CARD_D, -CONN_Y + 0.025, 0.030), "conn"),
    )
    ports = []
    for i in range(9):
        y = -0.150 + i * 0.0375
        ports.append(_box(f"port{i}", (-0.0005, y - 0.012, 0.012), (0.0, y + 0.012, 0.032), "port", collide=False))
    brackets = []
    for s in (1, -1):    # each lever's pivot bracket, above the lever
        brackets.append(_box(f"bracket_{'l' if s > 0 else 'r'}", (PIVOT_X - 0.007, s * PIVOT_Y - 0.007, 0.032),
                             (0.0, s * PIVOT_Y + 0.007, 0.040), "face"))
    # the card's real extent as one collider for its weight on the floor and its edges in the guides
    parts, cols = _join(body, *ports, *brackets)
    cols = tuple(c for c in cols if "pcb" not in c)
    return parts, cols


def _lever(side: int):
    """An ejector lever, shut (joint zero) lying along the faceplate: a hub on its pivot, a handle reaching 110 mm toward
    the card's middle with a finger tab standing up at its end, and a claw reaching 30 mm outward past the card's edge.
    Swung fully open (85 degrees) the claw points into the slot, inside the chassis pin, so the card slides in past it.
    Swung shut, the claw comes round behind the pin and draws the card in: the tab is 8.5 times as far from the pivot
    as the pin (13 mm) is when shut, so 5 N on the tab gives 40 N at the pin."""
    s = side
    name = "lever_l" if s > 0 else "lever_r"
    py = s * PIVOT_Y
    tip = py - s * HANDLE_L

    def yspan(a, b):
        return (min(a, b), max(a, b))

    hub = _box(f"{name}_hub", (PIVOT_X - 0.006, py - 0.006, 0.012), (PIVOT_X + 0.006, py + 0.006, 0.030), "lever")
    y0, y1 = yspan(py - s * 0.006, tip)
    handle = _box(f"{name}_handle", (PIVOT_X - 0.004, y0, 0.014), (PIVOT_X + 0.004, y1, 0.028), "lever")
    y0, y1 = yspan(tip - s * 0.006, tip + s * 0.008)
    tab = _box(f"{name}_tab", (PIVOT_X - 0.004, y0, 0.014), (PIVOT_X + 0.004, y1, 0.075), "lever")
    y0, y1 = yspan(py + s * 0.006, py + s * CLAW_L)
    claw = _box(f"{name}_claw", (PIVOT_X - 0.003, y0, 0.016), (PIVOT_X + 0.003, y1, 0.026), "lever")
    parts, cols = _join(hub, handle, tab, claw)
    lo, hi = (-2.0, OPEN_LIMIT) if s < 0 else (-OPEN_LIMIT, 2.0)
    return m.hinge(name, "Z", (PIVOT_X, py, 0.021), lo, hi, parts, cols, mass=0.08, damping=0.05, friction=0.01,
                   effort=200.0)


_CARD_PARTS, _CARD_COLS = _card_frame()
CARD = m.mechanism("card", (SEAT_X - START_OUT, 0.0, TOP + FLOOR), [_lever(1), _lever(-1)], LOOKS,
                   frame_parts=_CARD_PARTS, frame_colliders=_CARD_COLS, friction=(0.12, 0.12), free_mass=CARD_MASS,
                   frame_inertia=(0.054, 0.071, 0.124), frame_com=(0.23, 0.0, 0.015),
                   start=(("lever_l", -math.radians(OPEN_DEG)), ("lever_r", math.radians(OPEN_DEG))))

OBJECTS = [CHASSIS, CARD]

# -- scoring -----------------------------------------------------------------------------------------------------------
END_Y = 0.180              # the faceplate's ends are measured 180 mm either side of its middle
AT_PINS = 0.025            # both ends within 25 mm of home: in far enough for the levers to catch their pins
ENGAGING = 0.008           # both ends within 8 mm: the connectors are mating
HOME = 0.001               # both ends within 1 mm: seated
COCK_LIMIT = 0.0015        # ends more than 1.5 mm apart while the connectors mate: bent backplane pins
SHUT_DEG = 3.0             # a lever counts as shut within 3 degrees of shut (in its catch)
HANDS_CLEAR = 0.03         # and let go: no gripper finger within 3 cm of either finger tab
_STATE: dict = {}


@m.on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(clear=False, at_pins=False, engaging=False, seated=False, bent=False, latched={"lever_l": False, "lever_r": False})
    _STATE["_held"], _STATE["_step"] = 0, None


_reset_state()


@m.on_reset
def _open_levers(environment) -> None:
    """Start both levers swung open (the articulation's own initial joint positions are not applied on every reset)."""
    import torch

    asset = environment.unwrapped.scene["card"]
    for link, deg in (("lever_l", -OPEN_DEG), ("lever_r", OPEN_DEG)):
        j = asset.find_joints(f"{link}_joint")[0][0]
        pos = torch.tensor([[math.radians(deg)]], device=asset.device)
        asset.write_joint_state_to_sim(pos, torch.zeros_like(pos), joint_ids=[j])


@m.on_step
def _latch(environment) -> None:
    """Each lever's spring catch: a lever swung to within 2.5 degrees of shut, with its end of the card within 1.5 mm of
    home (its claw bearing on its pin), snaps into the catch, which pulls it the rest of the way shut and holds it there
    against the connectors' push."""
    st = m.state(environment, "card")
    _STATE["clear"] = bool(np.min([np.linalg.norm(m.fingertips(environment) - t, axis=1).min()
                                   for t in tab_points(st)]) > HANDS_CLEAR)
    gaps = end_gaps(st)
    for i, link in enumerate(("lever_l", "lever_r")):
        if not _STATE["latched"][link] and abs(st.deg(link)) <= LATCH_DEG and gaps[i] <= CATCH_GAP:
            m.lock(environment, "card", link, stiffness=CATCH_RATE, effort=CATCH_EFFORT)
            m.set_target(environment, "card", link, 0.0)     # the catch's spring snaps the lever fully shut
            _STATE["latched"][link] = True


def tab_points(card) -> np.ndarray:
    """The middle of each lever's finger tab (left, right), world, from the card's pose and the levers' angles."""
    out = []
    for s, link in ((1, "lever_l"), (-1, "lever_r")):
        a = card.joint(link)
        pivot = np.array([PIVOT_X, s * PIVOT_Y])
        rel = np.array([0.0, -s * (HANDLE_L + 0.001)])
        rot = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        x, y = pivot + rot @ rel
        out.append(card.to_world(np.array([[x, y, 0.045]]))[0])
    return np.array(out)


def end_gaps(card) -> np.ndarray:
    """How far each end of the faceplate (left, right) is short of home (m), from the card's pose."""
    pts = card.to_world(np.array([[0.0, END_Y, 0.0], [0.0, -END_Y, 0.0]]))
    return SEAT_X - pts[:, 0]


def lever_deg(card, link: str) -> float:
    """How far a lever is open (degrees, 0 shut)."""
    return abs(card.deg(link))


def _assess(objects) -> tuple[float, bool]:
    card = objects["card"]
    g = end_gaps(card)
    worst = float(g.max())
    if worst <= ENGAGING and abs(g[0] - g[1]) > COCK_LIMIT:
        _STATE["bent"] = True
    if _STATE["bent"]:
        return 0.0, False
    _STATE["at_pins"] |= worst <= AT_PINS
    _STATE["engaging"] |= worst <= ENGAGING
    seated = worst <= HOME
    _STATE["seated"] |= seated
    shut = all(lever_deg(card, k) <= SHUT_DEG for k in ("lever_l", "lever_r"))
    if seated and shut and _STATE["clear"]:
        return 1.0, True
    if _STATE["seated"]:
        return 0.75, False
    if _STATE["engaging"]:
        return 0.5, False
    return (0.25 if _STATE["at_pins"] else 0.0), False


FULL_HOLD_STEPS = 20        # the success condition must hold this many control steps in a row (1 s) for 1.0
UNHELD = 0.75                # the score while it has held for less


def _update(objects) -> tuple[float, bool]:
    """This step's (score, success). success() and score() are both called on each step's states; the step is
    assessed once. The score is 1.0 only once the success condition has held for 1 s; until then it is UNHELD."""
    last = _STATE.get("_step")
    if last is not None and last[0] is objects and last[2] < 2:
        _STATE["_step"] = (last[0], last[1], last[2] + 1)
        return last[1]
    raw, ok = _assess(objects)
    _STATE["_held"] = _STATE.get("_held", 0) + 1 if ok else 0
    out = (raw if raw < 1.0 else 1.0 if _STATE["_held"] >= FULL_HOLD_STEPS else UNHELD, ok)
    _STATE["_step"] = (objects, out, 1)
    return out


def success(objects) -> bool:
    """Seated (both ends of the faceplate within 1 mm of home) with both levers shut and let go; the harness requires it
    for 1 s."""
    return _update(objects)[0] >= 1.0   # true only once the full score's 1 s hold is complete


def score(objects) -> float:
    """0.25 pushed in to the pins; 0.5 connectors mating; 0.75 seated; 1.0 seated with both levers shut and let go.
    0 if the connectors were ever driven in more than 1.5 mm unevenly."""
    return _update(objects)[0]
