"""Seat a switch line card by swinging dual ejector levers closed into chassis catches.

Task file for yam_arena embodiment:
- Full-scale commercial 1U modular switch chassis with IEEE 1101.10 guide card tracks and flange pins.
- 4.0 kg rigid line card blade with dual pivoting cam ejector levers and spring detent catches.
- Dual high-density backplane connector blocks with 80 N total mating resistance over final 8 mm.
- Skew tolerance: <= 1.5 mm differential left vs right during connector engagement.
- Automated 20 N retention proof pull along withdrawal axis.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Seat the switch line card: swing both ejector levers closed into their catches and release."

LOOKS = {
    "steel": m.Look(m.rgb(0.62, 0.63, 0.65), metallic=0.6, roughness=0.45),
    "rack_post": m.Look(m.rgb(0.18, 0.19, 0.21), metallic=0.7, roughness=0.4),
    "neighbor": m.Look(m.rgb(0.25, 0.26, 0.28), metallic=0.5, roughness=0.5),
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
    """A box from corner ``lo`` to corner ``hi`` (m): visual mesh and collider."""
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


# -- Chassis geometry (IEEE 1101.10 / 19" 1U rack subrack standard) ----------------------------------------------------
X_FACE = 0.33              # world x of chassis face aperture
SLOT_W = 0.403             # 403 mm between guide tracks (1.5 mm clearance each side for 400 mm card)
FLOOR = 0.010              # slot floor above table top
SLOT_DEPTH = 0.520         # 520 mm depth
PIN_R = 0.004              # 8 mm diameter steel ejector stop pins
PIN_X = -0.0283            # 28.3 mm in front of chassis face aperture
PIN_Y = 0.205              # 205 mm from centerline
CONN_TRAVEL = 0.008        # 8 mm connector mating travel
CONN_RATE = 5000.0         # 5000 N/m per connector block (40 N each, 80 N total at home)
CONN_Y = 0.150             # connector positions at y = +-150 mm
SEAT_X = X_FACE - 0.005    # card origin at home (faceplate 1 mm proud of chassis face)
CARD_D = 0.460             # card length 460 mm
CONN_X0 = SEAT_X - X_FACE + CARD_D - CONN_TRAVEL

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
    _box("conn_base_r", (CONN_X0 + 0.028, -CONN_Y - 0.03, 0.012), (SLOT_DEPTH, -CONN_Y + 0.03, 0.045), "dark"),
    _box("ear_l", (0.0, 0.235, 0.0), (0.004, 0.262, 0.064), "steel", collide=False),
    _box("ear_r", (0.0, -0.262, 0.0), (0.004, -0.235, 0.064), "steel", collide=False),
    # 19-inch 4-post rack bay vertical mounting uprights (EIA-310-D standard, 482.6 mm outer mounting span)
    _box("rack_upright_l", (-0.010, 0.235, -0.050), (0.030, 0.265, 0.160), "rack_post", collide=False),
    _box("rack_upright_r", (-0.010, -0.265, -0.050), (0.030, -0.235, 0.160), "rack_post", collide=False),
    # Adjacent 1U modular chassis unit above in the rack bay (constraining top approach)
    _box("neighbor_above", (0.010, -0.235, 0.066), (SLOT_DEPTH, 0.235, 0.110), "neighbor", collide=False),
    *[_box(f"vent{i}", (0.06 + 0.04 * i, -0.17, 0.064), (0.075 + 0.04 * i, 0.17, 0.0645), "dark", collide=False)
      for i in range(10)],
)
_pins = []
for _s in (1, -1):
    mesh = m.cylinder(PIN_R, 0.040, (PIN_X, _s * PIN_Y, FLOOR + 0.020))
    _pins.append(((f"pin_{'l' if _s > 0 else 'r'}", mesh, "pin"),))
CHASSIS_PARTS += tuple(p for ps in _pins for p in ps)
CHASSIS_COLS += tuple(m.col_cylinder(f"pin_{'l' if s > 0 else 'r'}_c", PIN_R, 0.040, (PIN_X, s * PIN_Y, FLOOR + 0.020))
                      for s in (1, -1))


def _connector(side: int):
    name = "conn_l" if side > 0 else "conn_r"
    p, c = _box(name, (CONN_X0, side * CONN_Y - 0.02, 0.015), (CONN_X0 + 0.020, side * CONN_Y + 0.02, 0.040), "conn")
    return m.slider(name, "X", 0.0, CONN_TRAVEL, p, c, at=(CONN_X0, side * CONN_Y, 0.03), spring=CONN_RATE,
                    damping=40.0, friction=0.0, mass=0.3, effort=5000.0)


CHASSIS = m.mechanism("chassis", (X_FACE, 0.0, TOP), [_connector(1), _connector(-1)], LOOKS, frame_parts=CHASSIS_PARTS,
                      frame_colliders=CHASSIS_COLS, friction=(0.12, 0.12))

# -- Line card geometry ------------------------------------------------------------------------------------------------
CARD_W = 0.400
CARD_H = 0.035
CARD_MASS = 4.0
FACE_HALF_W = 0.186
PIVOT_X = -0.016
PIVOT_Y = 0.192
CLAW_R = 0.013
HANDLE_L = 0.110
OPEN_DEG = 85.0
OPEN_LIMIT = 85.0
CLAW_L = 0.030
START_OUT = 0.060
LATCH_DEG = 2.5
CATCH_GAP = 0.0015
CATCH_RATE = 200.0
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
    for s in (1, -1):
        brackets.append(_box(f"bracket_{'l' if s > 0 else 'r'}", (PIVOT_X - 0.007, s * PIVOT_Y - 0.007, 0.032),
                             (0.0, s * PIVOT_Y + 0.007, 0.040), "face"))
    parts, cols = _join(body, *ports, *brackets)
    cols = tuple(c for c in cols if "pcb" not in c)
    return parts, cols


def _lever(side: int):
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

# -- Scoring -----------------------------------------------------------------------------------------------------------
END_Y = 0.180
AT_PINS = 0.025            # Stage 0.25 threshold: card pushed within 25 mm of home
ENGAGING = 0.008           # Stage 0.50 threshold: card within 8 mm connector mating zone
HOME = 0.0010              # Stage 0.75 threshold: card seated within 1.0 mm
COCK_LIMIT = 0.0015        # Defect latch: skew > 1.5 mm while mating permanently bends pins
SHUT_DEG = 2.5             # Levers swung shut within 2.5 degrees (per review)
HANDS_CLEAR = 0.030        # Grippers released > 30 mm clear of finger tabs
PROOF_PULL_N = 20.0        # 20 N automated retention proof pull along -X withdrawal axis
FULL_HOLD_STEPS = 20       # 1.0 s held (20 steps at 20 Hz)
UNHELD = 0.75

_STATE: dict = {}


@m.on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(
        clear=False,
        at_pins=False,
        engaging=False,
        seated=False,
        bent=False,
        latched={"lever_l": False, "lever_r": False},
        _held=0,
        _step=None,
    )


_reset_state()


@m.on_reset
def _open_levers(environment) -> None:
    import torch

    asset = environment.unwrapped.scene["card"]
    for link, deg in (("lever_l", -OPEN_DEG), ("lever_r", OPEN_DEG)):
        j = asset.find_joints(f"{link}_joint")[0][0]
        pos = torch.tensor([[math.radians(deg)]], device=asset.device)
        asset.write_joint_state_to_sim(pos, torch.zeros_like(pos), joint_ids=[j])


@m.on_step
def _latch(environment) -> None:
    st = m.state(environment, "card")
    _STATE["clear"] = bool(np.min([np.linalg.norm(m.fingertips(environment) - t, axis=1).min()
                                  for t in tab_points(st)]) > HANDS_CLEAR)
    gaps = end_gaps(st)
    for i, link in enumerate(("lever_l", "lever_r")):
        if not _STATE["latched"][link] and abs(st.deg(link)) <= LATCH_DEG and gaps[i] <= CATCH_GAP:
            m.lock(environment, "card", link, stiffness=CATCH_RATE, effort=CATCH_EFFORT)
            m.set_target(environment, "card", link, 0.0)
            _STATE["latched"][link] = True

    # Automated 20 N proof pull along -X withdrawal axis once seated, shut, and clear
    shut = all(lever_deg(st, k) <= SHUT_DEG for k in ("lever_l", "lever_r"))
    worst = float(gaps.max())
    if worst <= HOME and shut and _STATE["clear"]:
        m.push(environment, "card", (-PROOF_PULL_N, 0.0, 0.0), body="frame")
    else:
        m.push(environment, "card", (0.0, 0.0, 0.0), body="frame")


def tab_points(card) -> np.ndarray:
    """The middle of each lever's finger tab (left, right), world coordinates."""
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
    """Distance each end of the faceplate (left, right) is short of home (m)."""
    pts = card.to_world(np.array([[0.0, END_Y, 0.0], [0.0, -END_Y, 0.0]]))
    return SEAT_X - pts[:, 0]


def lever_deg(card, link: str) -> float:
    return abs(card.deg(link))


def _assess(objects) -> tuple[float, bool]:
    card = objects["card"]
    g = end_gaps(card)
    worst = float(g.max())

    # Skew defect check: once either connector enters mating zone (<= 8 mm), skew > 1.5 mm trips bent pins latch
    if worst <= ENGAGING and abs(g[0] - g[1]) > COCK_LIMIT:
        _STATE["bent"] = True
    if _STATE["bent"]:
        return 0.0, False

    # Check excessive tilt / card dropped defect latch (>15 deg tilt)
    if hasattr(card, "tilt_deg") and card.tilt_deg() > 15.0:
        return 0.0, False

    _STATE["at_pins"] |= worst <= AT_PINS
    _STATE["engaging"] |= worst <= ENGAGING
    seated = worst <= HOME
    shut = all(lever_deg(card, k) <= SHUT_DEG for k in ("lever_l", "lever_r"))
    _STATE["seated"] |= (seated and shut)

    if seated and shut and _STATE["clear"]:
        return 1.0, True
    if _STATE["seated"]:
        return 0.75, False
    if _STATE["engaging"]:
        return 0.50, False
    return (0.25 if _STATE["at_pins"] else 0.0), False


def _update(objects) -> tuple[float, bool]:
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
    """Seated with both levers shut and let go, held for 1.0 s (20 control steps)."""
    return _update(objects)[0] >= 1.0


def score(objects) -> float:
    """Stage scoring: 0.25 at pins; 0.50 mating connectors; 0.75 seated; 1.0 seated, shut, released, held.
    Permanent 0.0 if skew > 1.5 mm while mating connectors."""
    return _update(objects)[0]
