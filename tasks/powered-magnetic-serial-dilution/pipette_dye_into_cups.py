"""On the station table, in front of the left arm: four clear sample cups on an operating four-position magnetic
stirrer, and a transfer pipette standing tip down in a grey holder to their left.

- The cup nearest the arms holds dark blue stock. The three behind it are cups 1, 2 and 3, front to back; each holds
  one pipetteful of clear water.
- Every cup has a magnetic stir bar. The four stations run continuously at 600 rpm. After dispensing, wait at least
  one second for powered mixing before drawing from that cup.
- The pipette is a squeeze-bulb transfer pipette that takes exactly one pipetteful. Its bulb's squeeze faces point
  to the robot's left and right. Grip the bulb lightly to hold it; squeeze it hard to push its contents out; ease
  back to a light grip to draw liquid up, which works only while the tip is under the liquid. So: squeeze before
  dipping, ease off under the liquid to draw, lift, then squeeze over or inside the next cup to dispense.

Run it:  inspect-robots run --embodiment yam_arena -E task=pipette_dye_into_cups.py --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from benchkit import Slider, ToolState, on_reset, on_step, recolor, tool  # noqa: E402
from ledger import Ledger, Vessel  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "With the four-position magnetic stirrer already running, use the transfer pipette to dilute the dark stock "
    "by half in each cup. Move one pipetteful from the stock into cup 1, allow it to mix, then one from cup 1 into "
    "cup 2, allow it to mix, then one from cup 2 into cup 3 and allow it to mix. Stand the empty pipette back in "
    "its holder and let go."
)

# -- the powered magnetic stirring station and cups ----------------------------------------------------------------
ROW_Y = 0.12
CUP_X = {"stock": 0.17, "cup1": 0.215, "cup2": 0.26, "cup3": 0.305}
CUP_INNER = 0.030  # square opening (m)
WALL = 0.003
CUP_HEIGHT = 0.040
STIRRER_SIZE = (0.180, 0.070, 0.012)
STIRRER_CENTRE = ((CUP_X["stock"] + CUP_X["cup3"]) / 2, ROW_Y)
STIRRER_TOP = TOP + STIRRER_SIZE[2]
STIR_PAD = (0.034, 0.034, 0.002)
STIR_BAR = (0.014, 0.003, 0.002)
STIR_SPEED_RPM = 600
MIX_SECONDS = 1.0
CONTROL_HZ = 20
MIX_STEPS = round(MIX_SECONDS * CONTROL_HZ)
PIPETTEFUL_ML = 5.0
LIQUID_PER_ALIQUOT = PIPETTEFUL_ML * 1e-6 / CUP_INNER**2  # m of liquid one pipetteful adds to a cup (5.6 mm)
FILM = 0.002  # the coloured layer drawn on each cup's floor
ORDER = ["stock", "cup1", "cup2", "cup3"]
START = {"stock": {"dye": 2.0}, "cup1": {"water": 1.0}, "cup2": {"water": 1.0}, "cup3": {"water": 1.0}}

CLEAR = (0.86, 0.89, 0.92)
STOCK_BLUE = (0.10, 0.16, 0.62)
WATER = (0.78, 0.87, 0.96)
GREY = (0.55, 0.56, 0.58)

# -- the holder ----------------------------------------------------------------------------------------------------
HOLDER_XY = (0.24, 0.18)
HOLDER_INNER = 0.022
HOLDER_HEIGHT = 0.050

# -- the pipette (tool frame: origin at the bulb's centre, z up) ----------------------------------------------------
BULB = (0.020, 0.008, 0.030)  # the core, thin enough that the fingers reach full squeeze before they meet it
STEM = 0.008
STEM_LEN = 0.090
PAD = (0.020, 0.010, 0.026)
SQUEEZE_TRAVEL = 0.009  # each side of the bulb moves in this far (a real bulb gives about a centimetre)
TIP_LOCAL = (0.0, 0.0, -BULB[2] / 2 - STEM_LEN)
REST_WIDTH = BULB[1] + 2 * PAD[1]  # across the bulb's sides at rest
PIPETTE_MASS = 0.03  # the body; each side adds 4 g
PIPETTE_Z = TOP + 0.003 - TIP_LOCAL[2]  # standing in the holder, tip just above the table

SIDE_RATE = 350.0  # N/m per side: a light grip (1.5 mm in) holds about 1 N by friction, three times the weight
WHITE = (0.95, 0.95, 0.95)
STEM_WHITE = (0.90, 0.91, 0.93)

PIPETTE = tool(
    "pipette",
    pos=(*HOLDER_XY, PIPETTE_Z),
    boxes=[
        ((0.0, 0.0, 0.0), BULB, WHITE),
        ((0.0, 0.0, -BULB[2] / 2 - STEM_LEN / 2), (STEM, STEM, STEM_LEN), STEM_WHITE),
    ],
    sliders=[
        Slider("side_l", rest=(0.0, BULB[1] / 2 + PAD[1] / 2, 0.0), size=PAD, axis="Y",
               lower=-SQUEEZE_TRAVEL, upper=0.0, color=WHITE, rate=SIDE_RATE),
        Slider("side_r", rest=(0.0, -BULB[1] / 2 - PAD[1] / 2, 0.0), size=PAD, axis="Y",
               lower=0.0, upper=SQUEEZE_TRAVEL, color=WHITE, rate=SIDE_RATE),
    ],
    mass=PIPETTE_MASS,
    points=[("tip", TIP_LOCAL)],
)


def _walls(name: str, xy, inner: float, height: float, color, floor_z: float = TOP) -> list[Box]:
    """Four fixed walls around a square opening, standing on the table."""
    x, y = xy
    off = inner / 2 + WALL / 2
    z = floor_z + height / 2
    long = inner + 2 * WALL
    return [
        Box(f"{name}_wall_f", pos=(x + off, y, z), size=(WALL, long, height), color=color),
        Box(f"{name}_wall_b", pos=(x - off, y, z), size=(WALL, long, height), color=color),
        Box(f"{name}_wall_l", pos=(x, y + off, z), size=(inner, WALL, height), color=color),
        Box(f"{name}_wall_r", pos=(x, y - off, z), size=(inner, WALL, height), color=color),
    ]


def _film(name: str, color) -> Box:
    return Box(f"{name}_liquid", pos=(CUP_X[name], ROW_Y, STIRRER_TOP + FILM / 2),
               size=(CUP_INNER - 0.002,) * 2 + (FILM,),
               color=color)


OBJECTS = [
    Box("powered_stirrer_base", pos=(*STIRRER_CENTRE, TOP + STIRRER_SIZE[2] / 2), size=STIRRER_SIZE,
        color=(0.22, 0.24, 0.27)),
    *(Box(f"{n}_stir_pad", pos=(CUP_X[n], ROW_Y, STIRRER_TOP + STIR_PAD[2] / 2), size=STIR_PAD,
          color=(0.12, 0.13, 0.15)) for n in ORDER),
    Box("stirrer_power_light", pos=(STIRRER_CENTRE[0], ROW_Y - 0.032, STIRRER_TOP + 0.003),
        size=(0.012, 0.003, 0.004), color=(0.12, 0.90, 0.24)),
    *(Box(f"{n}_stir_bar", pos=(CUP_X[n], ROW_Y, STIRRER_TOP + STIR_BAR[2] / 2), size=STIR_BAR,
          color=(0.94, 0.94, 0.96)) for n in ORDER),
    *(w for n in ORDER for w in _walls(n, (CUP_X[n], ROW_Y), CUP_INNER, CUP_HEIGHT, CLEAR, STIRRER_TOP)),
    *(_film(n, STOCK_BLUE if n == "stock" else WATER) for n in ORDER),
    *_walls("holder", HOLDER_XY, HOLDER_INNER, HOLDER_HEIGHT, GREY),
    PIPETTE,
]

# -- the liquid ledger -----------------------------------------------------------------------------------------------


def _vessels() -> list[Vessel]:
    return [Vessel(n, (CUP_X[n], ROW_Y), (CUP_INNER, CUP_INNER), floor_z=STIRRER_TOP,
                   rim_z=STIRRER_TOP + CUP_HEIGHT,
                   height_per_aliquot=LIQUID_PER_ALIQUOT, contents=dict(START[n]), catch_above=CATCH_ABOVE)
            for n in ORDER]


CATCH_ABOVE = 0.03  # squeezed out up to this far above a cup's rim, over its opening, the liquid lands in the cup
LEDGER = Ledger(_vessels(), travel=SQUEEZE_TRAVEL, order=ORDER,
                powered_mixers=set(ORDER), mix_steps=MIX_STEPS)


def squeeze(state: ToolState) -> float:
    """How far the bulb's sides are pushed in, on average (m)."""
    return float(np.mean([abs(v) for v in state.sliders.values()])) if state.sliders else 0.0


def liquid_colour(cup: Vessel) -> tuple[float, float, float]:
    """The colour a cup's liquid shows, clear if empty. Dye absorbs light exponentially with strength (Beer-Lambert),
    so each halving is a visible step and even an eighth is plainly tinted."""
    if cup.volume < 1e-6:
        return CLEAR
    f = cup.fraction("dye")
    return tuple(w * (b / w) ** f for w, b in zip(WATER, STOCK_BLUE))


_SHOWN: dict = {}


def _show_liquids() -> None:
    for name, cup in LEDGER.vessels.items():
        colour = tuple(round(c, 3) for c in liquid_colour(cup))
        if _SHOWN.get(name) != colour:
            _SHOWN[name] = colour
            recolor(f"{name}_liquid", colour)


@on_reset
def _reset(states) -> None:
    LEDGER.reset()
    _SHOWN.clear()
    _show_liquids()


@on_step
def _track(states) -> None:
    pipette = states["pipette"]
    LEDGER.update(pipette.point("tip"), squeeze(pipette))
    _show_liquids()


# -- scoring -------------------------------------------------------------------------------------------------------
TARGET = {"cup1": 0.5, "cup2": 0.25, "cup3": 0.125}  # fraction of stock strength
VOLUME_RANGE = (0.75, 2.25)  # pipettefuls: one or two (a cup keeps two until its pipetteful moves on)
# A cup's strength counts as its target when it is closer (by ratio) to it than to half or double it: the same
# call as matching it against reference cups made two-fold apart.
CONC_BAND = (2**-0.5, 2**0.5)
PER_CUP = 0.3
SPILL_CAP = 0.5
HOLDER_MAX_TILT = 20.0  # degrees
HOLDER_TIP_BELOW = TOP + 0.02


def cup_right(ledger: Ledger, name: str) -> bool:
    cup, conc = ledger.vessels[name], TARGET[name]
    low, high = VOLUME_RANGE
    return (cup.mixed and low <= cup.volume <= high
            and CONC_BAND[0] * conc <= cup.fraction("dye") < CONC_BAND[1] * conc)


def stock_draws(ledger: Ledger) -> int:
    """How many pipettefuls left the stock: the protocol takes exactly one."""
    return sum(e.kind == "aspirate" and e.vessel == "stock" for e in ledger.events)


def cups_in_order(ledger: Ledger) -> int:
    """How many of cups 1, 2, 3 are right, counting from cup 1 and stopping at the first that isn't."""
    n = 0
    for name in ("cup1", "cup2", "cup3"):
        if not cup_right(ledger, name):
            break
        n += 1
    return n


LET_GO = 0.0005  # m: both sides of the bulb back within this of rest means nothing is gripping it


def in_holder(state: ToolState) -> bool:
    tip = state.point("tip")
    inside = abs(tip[0] - HOLDER_XY[0]) <= HOLDER_INNER / 2 and abs(tip[1] - HOLDER_XY[1]) <= HOLDER_INNER / 2
    return bool(inside and tip[2] < HOLDER_TIP_BELOW and state.tilt_deg() <= HOLDER_MAX_TILT)


def let_go(state: ToolState) -> bool:
    return all(abs(v) <= LET_GO for v in state.sliders.values())


def put_away(ledger: Ledger, pipette: ToolState) -> bool:
    """Standing in its holder, empty, and not held."""
    return in_holder(pipette) and ledger.tip_volume < 1e-9 and let_go(pipette)


def grade(ledger: Ledger, pipette: ToolState) -> float:
    """The score as the outcome table states it."""
    if ledger.went_back or ledger.unmixed_draw or stock_draws(ledger) > 1:
        return 0.0
    n = cups_in_order(ledger)
    score = PER_CUP * n + (0.1 if n == 3 and put_away(ledger, pipette) else 0.0)
    if ledger.spilled > 1e-9:
        score = min(score, SPILL_CAP)
    return round(score, 6)


def success(objects) -> bool:
    """All three cups right, nothing spilled, no going back, and the pipette standing still in its holder."""
    pipette = objects["pipette"]
    return grade(LEDGER, pipette) >= 1.0 and pipette.resting()


def score(objects) -> float:
    return grade(LEDGER, objects["pipette"])


def distance(objects) -> float:
    """Metres from the pipette's tip to the cup the protocol needs next (or the holder once all three are made)."""
    tip = objects["pipette"].point("tip")
    n = cups_in_order(LEDGER)
    goal = (*HOLDER_XY, TOP) if n == 3 else (CUP_X[ORDER[n + 1]], ROW_Y, TOP)
    return float(np.linalg.norm(tip - np.array(goal)))


# -- the scripted demo (left arm) -----------------------------------------------------------------------------------
GRIP_ABOVE_CENTRE = 0.004  # the TCP sits this far above the bulb's centre when holding it
HOLD = -(REST_WIDTH - 2 * 0.0015) / 2  # finger target: sides pushed in 1.5 mm, enough to hold, short of a press
SQUEEZE = -(REST_WIDTH - 2 * SQUEEZE_TRAVEL - 0.002) / 2  # past full travel, so the sides bottom out
TIP_TRAVEL = STIRRER_TOP + 0.065  # tip height while moving: clears the cups' rims
TIP_PRIME = STIRRER_TOP + 0.030  # inside a cup, above its liquid: squeeze here before dipping
TIP_DIP = STIRRER_TOP + 0.004  # under even one pipetteful (5.6 mm), above the stir bar
TIP_DISPENSE = STIRRER_TOP + 0.022  # inside a cup, above its liquid
TIP_PARK = TOP + 0.004  # back in the holder


# The arm's wrist settles a few degrees off vertical, which a 12 cm pipette turns into centimetres at the tip, and
# it differs by reach. The demo is open loop, so it aims off by the error measured at each spot (m, in x and y).
TRIM = {"stock": (0.004, -0.002), "cup1": (0.006, -0.001), "cup2": (0.008, -0.001), "cup3": (0.011, -0.001),
        "holder": (0.009, 0.0)}


def tcp_for_tip(x: float, y: float, tip_z: float) -> tuple[float, float, float]:
    return (x, y, tip_z - TIP_LOCAL[2] + GRIP_ABOVE_CENTRE)


def _aim(name: str) -> tuple[float, float]:
    """Where to send the TCP so the tip arrives over ``name``."""
    x, y = HOLDER_XY if name == "holder" else (CUP_X[name], ROW_Y)
    return (x + TRIM[name][0], y + TRIM[name][1])


def _transfer(src: str, dst: str) -> list[Waypoint]:
    sx, sy = _aim(src)
    dx, dy = _aim(dst)
    return [
        Waypoint(2.0, tcp_for_tip(sx, sy, TIP_TRAVEL), HOLD, f"over {src}"),
        Waypoint(0.8, tcp_for_tip(sx, sy, TIP_TRAVEL), HOLD, "settle"),
        Waypoint(1.5, tcp_for_tip(sx, sy, TIP_PRIME), HOLD, "into the cup"),
        Waypoint(0.8, tcp_for_tip(sx, sy, TIP_PRIME), SQUEEZE, "squeeze in air"),
        Waypoint(1.5, tcp_for_tip(sx, sy, TIP_DIP), SQUEEZE, "dip"),
        Waypoint(1.0, tcp_for_tip(sx, sy, TIP_DIP), HOLD, "release: draw"),
        Waypoint(1.5, tcp_for_tip(sx, sy, TIP_TRAVEL), HOLD, "lift"),
        Waypoint(2.0, tcp_for_tip(dx, dy, TIP_TRAVEL), HOLD, f"over {dst}"),
        Waypoint(0.8, tcp_for_tip(dx, dy, TIP_TRAVEL), HOLD, "settle"),
        Waypoint(1.5, tcp_for_tip(dx, dy, TIP_DISPENSE), HOLD, "into the cup"),
        Waypoint(0.8, tcp_for_tip(dx, dy, TIP_DISPENSE), SQUEEZE, "squeeze: dispense"),
        Waypoint(1.5, tcp_for_tip(dx, dy, TIP_TRAVEL), SQUEEZE, "lift, still squeezed"),
        Waypoint(0.8, tcp_for_tip(dx, dy, TIP_TRAVEL), HOLD, "release in air"),
        Waypoint(MIX_SECONDS, tcp_for_tip(dx, dy, TIP_TRAVEL), HOLD, "powered stirring dwell"),
    ]


def demo():
    """Left arm: take the pipette from its holder, make the three transfers, stand it back and let go."""
    hx, hy = HOLDER_XY
    start_tip = PIPETTE_Z + TIP_LOCAL[2]
    plan = [
        Waypoint(1.5, tcp_for_tip(hx, hy, TIP_TRAVEL), FINGER_OPEN, "over the pipette"),
        Waypoint(1.5, tcp_for_tip(hx, hy, start_tip), FINGER_OPEN, "down to the bulb"),
        Waypoint(0.8, tcp_for_tip(hx, hy, start_tip), HOLD, "grip"),
        Waypoint(1.2, tcp_for_tip(hx, hy, TIP_TRAVEL), HOLD, "lift out"),
    ]
    plan += _transfer("stock", "cup1") + _transfer("cup1", "cup2") + _transfer("cup2", "cup3")
    plan += [
        Waypoint(2.0, tcp_for_tip(*_aim("holder"), TIP_TRAVEL), HOLD, "over the holder"),
        Waypoint(0.8, tcp_for_tip(*_aim("holder"), TIP_TRAVEL), HOLD, "settle"),
        Waypoint(2.0, tcp_for_tip(*_aim("holder"), TIP_PARK), HOLD, "into the holder"),
        Waypoint(1.0, tcp_for_tip(*_aim("holder"), TIP_PARK), FINGER_OPEN, "let go"),
        Waypoint(1.5, tcp_for_tip(*_aim("holder"), TIP_TRAVEL), FINGER_OPEN, "up"),
    ]
    return {"left": plan}
