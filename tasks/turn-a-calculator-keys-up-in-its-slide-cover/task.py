"""A school calculator (Texas Instruments TI-30XIIS) lies KEYS DOWN in its hard slide cover on a closed A5 hardback
notebook: the cover underneath, channel up, over the keys (its front position), the calculator's back up. The cover is
a shallow channel: a plate, two side walls whose lips run in grooves along the calculator's sides at mid-thickness, an
end wall at the ON-key end, open at the display end. The pair lies along the arms' line (y), the open display end to
the right, 60 mm past the notebook's right edge. Above the cover's walls the calculator's sides are bare (8.9 mm): a
shallow pinch from above takes the calculator there, a deep one takes the cover by its walls. The task: take the
calculator out of the cover, turn it over and slide it back in, keys up, so that the cover is on its back.

The cover on the calculator (mechkit 1.0 parts only):
- the rails: the cover's lips are real boxes in real grooves (0.6 mm of play over and under a lip, 0.5 mm beside the
  walls): a cover offered from above lands on the calculator's edges; the calculator enters only through the cover's
  open end, in line (the lips' and the walls' first 3 mm are half as thick: a lead-in of about 1 mm).
- once a lip is RAILS_IN into its groove, in the front or the back position and within the rails' real play of the
  line (0.5 mm across, 0.6 mm up or down, read at the calculator's leading end, where the walls and lips act; 1
  degree), the calculator is set exactly on the cover's rail line (that is
  the largest correction the rule makes; each one is recorded in the report) and the cover welded to a hidden carrier
  that slides along the calculator on a slide ("rails"): from then on it moves along the rails only; beyond the play it
  is not taken. Its resistance is a spring capped at RAIL_FORCE, whose rest follows it (no creep under a hand's
  jerks), and within DETENT_FROM of home capped at DETENT_FORCE: the end's click; within CLICK_IN of home the spring
  draws it home and holds it there. Slid RELEASE_PAST its rails' length the weld is cut: the cover is free.
- home is the end wall against the calculator's end (the slide's stop).

Every size of the calculator and the cover is (a): retailers' published figures (155 x 81-82 x 19 mm, 22 mm
with the cover) and the rest assumed until measured on the bought pair (README, table of sizes).
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
from mechkit import body, box, cylinder, slide, weld  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
RUN_LIMIT_STEPS = 1500  # 75 s at 20 Hz: the scripted full run (1,176 steps, 58.8 s, both arms 2 s at ready at its end) plus about 28 %
SUCCESS_HOLD_S = 5.0  # s the full score must hold before the trial ends: long enough to see both arms back at their ready pose (the default 1 s ended the reference run with the hands still over the calculator: w12c19)
INSTRUCTION = "Take the slide cover off the front of the calculator and slide it onto the back, so that the calculator lies keys up in its cover."
TOP = TABLE_TOP[SCENE]
GRAVITY = 9.81

# --- colours ---
NAVY, COVER_C, KEY_C, KEY_DARK, KEY_YELLOW, DISPLAY_C = (0.08, 0.13, 0.33), (0.10, 0.10, 0.13), (0.78, 0.79, 0.80), (0.20, 0.21, 0.24), (0.86, 0.74, 0.18), (0.36, 0.40, 0.33)
BOOK_C, PAGES_C, PENCIL_C, LEAD_C = (0.82, 0.82, 0.80), (0.93, 0.92, 0.88), (0.95, 0.75, 0.12), (0.2, 0.2, 0.2)

# --- the notebook: a closed A5 hardback of any make, light grey (a ground lighter than the navy calculator and the
# bench), long side along y (a: 210 x 148 x 15 mm; fixed: it never moves) ---
BOOK = (0.148, 0.210, 0.015)  # m: x, y, thickness
BOOK_XY = (0.24, 0.04)
BOOK_TOP = TOP + BOOK[2]
BOOK_RIGHT = BOOK_XY[1] - BOOK[1] / 2  # y of its right edge: -0.065

# --- the calculator ("calc"), keys up; its frame: the middle of its box, x across, y along (the display at +y), z out of
# the keys. Sizes (a): 155 x 81 x 19 mm, 100 g ---
CALC = (0.081, 0.155, 0.019)  # m: width (x), length (y), thickness (z)
CALC_MASS = 0.100  # kg (a)
GROOVE_H, GROOVE_D = 0.0024, 0.0015  # m: a groove's height and depth, at mid-thickness on both sides (a)
OVERHANG = 0.060  # m the calculator's right end lies past the notebook's right edge
CALC_Y = BOOK_RIGHT - OVERHANG + CALC[1] / 2  # the middle: -0.0475
# --- the cover; its frame is the calculator's own when it is home on the front (and turned half a turn about y when it
# is home on the back): plate over the keys, walls down to the grooves, lips in them, the end wall at -y (a) ---
PLATE_T, KEY_GAP = 0.0015, 0.0015  # m: the plate's thickness and its height over the keys' face (22 - 19 mm = 3 mm)
WALL_T, WALL_GAP = 0.0012, 0.0005  # m: a wall's thickness and its play beside the calculator's side
LIP_T, LIP_IN = 0.0012, 0.0014  # m: a lip's thickness (z: 0.6 mm of play over and under it) and its reach in from its wall (0.6 mm short of the groove's floor)
END_T, END_GAP = 0.0012, 0.0003  # m: the end wall's thickness, and its play to the calculator's end at home (the slide's stop is home)
COVER_MASS = 0.030  # kg (a)
NOSE = 0.003  # m: the lead-in at the open end: the lips' first NOSE are half as thick and reach half as far in, the walls' first NOSE half as thick
HALF_W = CALC[0] / 2  # 0.0405
WALL_IN = HALF_W + WALL_GAP  # the walls' inner faces at x = +-0.041
COVER_W = 2 * (WALL_IN + WALL_T)  # outside: 84.4 mm
PLATE_LOW = CALC[2] / 2 + KEY_GAP  # the plate's underside, z in the cover's frame: 0.011
COVER_TOP_Z = PLATE_LOW + PLATE_T  # 0.0125: 22 mm over the calculator's back with it
LIP_LOW = -LIP_T / 2  # the lips sit across the groove's middle (z = 0)
COVER_Y0, COVER_Y1 = -CALC[1] / 2 - END_GAP - END_T, CALC[1] / 2  # the cover's ends along y, in its frame: the end wall at the ON-key end, the open end flush with the display end
RAILS = CALC[1]  # m: the cover's travel from home to where its lips leave the grooves


def calc_shapes() -> list:
    """The calculator: two slabs and a narrower core between them make the two grooves (open at both ends); its face
    drawn: display, solar cell and 40 keys."""
    w, l, t = CALC
    slab = (t - GROOVE_H) / 2
    shapes = [
        box((w, l, slab), pos=(0.0, 0.0, -t / 2 + slab / 2)),
        box((w, l, slab), pos=(0.0, 0.0, t / 2 - slab / 2)),
        box((w - 2 * GROOVE_D, l, GROOVE_H), pos=(0.0, 0.0, 0.0)),
    ]
    face = t / 2 + 0.0003
    shapes.append(box((0.060, 0.026, 0.0006), pos=(0.0, l / 2 - 0.024, face), color=DISPLAY_C, solid=False))
    shapes.append(box((0.034, 0.010, 0.0006), pos=(0.0, l / 2 - 0.006, face), color=KEY_DARK, solid=False))
    for row in range(8):
        for col in range(5):
            colour = KEY_YELLOW if (row, col) == (0, 0) else (KEY_DARK if row < 2 else KEY_C)
            shapes.append(box((0.0115, 0.0068, 0.0012), pos=(-0.028 + 0.014 * col, l / 2 - 0.047 - 0.0115 * row, face + 0.0003), color=colour, solid=False))
    back = -t / 2 - 0.0003  # the back, drawn: the battery door and four feet
    shapes.append(box((0.046, 0.032, 0.0006), pos=(0.0, -l / 2 + 0.030, back), color=KEY_DARK, solid=False))
    for sx in (1.0, -1.0):
        for sy in (1.0, -1.0):
            shapes.append(box((0.006, 0.010, 0.0008), pos=(sx * 0.031, sy * 0.064, back - 0.0001), color=KEY_DARK, solid=False))
    return shapes


def cover_shapes() -> list:
    """The cover in its frame (home on the front): plate, two walls, two lips, the end wall at -y; the lips' and the
    walls' first NOSE at the open end (+y) thinner: the lead-in."""
    length = COVER_Y1 - COVER_Y0
    mid = (COVER_Y0 + COVER_Y1) / 2
    plate = box((COVER_W, length, PLATE_T), pos=(0.0, mid, PLATE_LOW + PLATE_T / 2))
    wall_h = PLATE_LOW - LIP_LOW
    body_l = length - NOSE
    body_mid = COVER_Y0 + body_l / 2
    shapes = [plate]
    for s in (1.0, -1.0):
        shapes.append(box((WALL_T, body_l, wall_h), pos=(s * (WALL_IN + WALL_T / 2), body_mid, LIP_LOW + wall_h / 2)))
        shapes.append(box((WALL_T / 2, NOSE, wall_h), pos=(s * (WALL_IN + WALL_T * 0.75), COVER_Y1 - NOSE / 2, LIP_LOW + wall_h / 2)))
        shapes.append(box((LIP_IN, body_l, LIP_T), pos=(s * (WALL_IN - LIP_IN / 2), body_mid, 0.0)))
        shapes.append(box((LIP_IN / 2, NOSE, LIP_T / 2), pos=(s * (WALL_IN - LIP_IN / 4), COVER_Y1 - NOSE / 2, 0.0)))
    shapes.append(box((COVER_W, END_T, wall_h + PLATE_T), pos=(0.0, COVER_Y0 + END_T / 2, LIP_LOW + (wall_h + PLATE_T) / 2)))
    return shapes


# The start: both turned half a turn about x (keys down, the display end to -y); the cover's plate 1 mm over the notebook
# (a loose body is drawn from frame 0).
START_QUAT = (1.0, 0.0, 0.0, 0.0)  # x, y, z, w
START_ROT = mk.quat_to_matrix(START_QUAT)
CALC_AT = (BOOK_XY[0], CALC_Y, BOOK_TOP + 0.001 + COVER_TOP_Z)
body("notebook", [box(BOOK, color=BOOK_C), box((BOOK[0] - 0.006, BOOK[1] - 0.004, BOOK[2] - 0.004), pos=(0.003, 0.0, 0.0), color=PAGES_C, solid=False)],
     pos=(*BOOK_XY, TOP + BOOK[2] / 2), color=BOOK_C)  # fmt: skip
body("pencil", [cylinder(0.0036, 0.168, axis="y", color=PENCIL_C), cylinder(0.0012, 0.006, pos=(0.0, -0.087, 0.0), axis="y", color=LEAD_C)],
     pos=(BOOK_XY[0] + BOOK[0] / 2 + 0.03, 0.05, TOP + 0.0036), color=PENCIL_C)  # fmt: skip
body("calc", calc_shapes(), pos=CALC_AT, quat=START_QUAT, mass=CALC_MASS, color=NAVY)
body("cover", cover_shapes(), pos=CALC_AT, quat=START_QUAT, mass=COVER_MASS, color=COVER_C)

# --- the rails: a hidden carrier slides along the calculator; the cover is welded to it while it is on ---
CARRIER_MASS = 0.030  # kg: as heavy as the cover (a carrier of a few grams is not held steady by a spring this stiff)
_ghosts = [box((0.003, 0.003, 0.003), pos=(sx * 0.025, sy * 0.05, 0.0), passes=("calc", "cover", "notebook")) for sx in (1, -1) for sy in (1, -1)]
body("carrier", _ghosts, pos=CALC_AT, quat=START_QUAT, mass=CARRIER_MASS, color=COVER_C)
RAIL_FORCE = 1.5  # N (a): what slides the cover along the rails; bench: a spring scale on the cover, pulled along, read while it moves
DETENT_FORCE = 6.0  # N (a, the spec: "about 6 N"): the end click; bench: the same scale, the reading as it leaves home
DETENT_FROM, CLICK_IN = 0.003, 0.001  # m from home: the click's bump, and where it draws the cover home
GIVE = 0.0005  # m: the spring's stretch at DETENT_FORCE (its stiffness: 12 kN/m)
STIFFNESS = DETENT_FORCE / GIVE
REACH = 0.010  # m the slide runs past the rails' end before it is let go (one control step of a fast pull)
OUT = (0.0, -1.0, 0.0)  # the way the cover slides off, in the calculator's frame (towards its ON-key end: the calculator leaves through the open end)
slide("rails", "carrier", "calc", axis=tuple(START_ROT @ np.array(OUT)), limits=(0.0, RAILS + REACH), spring=STIFFNESS, rest=0.0, damping=0.01 * STIFFNESS)
weld("on_front", "cover", "carrier", made=True)  # the cover starts home on the front
weld("on_again", "cover", "carrier", made=False)  # made each time the cover is taken onto the rails again, front or back
weld("cover_held", "cover", None, made=False)  # check.py only: the cover held where it lies while known loads work on the calculator
RELEASE_PAST = 0.001  # m past RAILS: let go
RAILS_IN = 0.006  # m: a lip this far into its groove holds the cover in line: taken onto the rails
# The rails' fit is a joint made when the calculator enters within the play. When it is taken, the calculator is set
# exactly on the cover's rail line, which its leading end stands off by no more than the real play: LATERAL across (the
# walls' play), ACROSS up or down (the lips' play), and its direction by no more than LINE_TILT (about a degree). Beyond
# these it is not taken. The offsets are read at the leading end, where the walls and the lips act on an entering
# calculator: read at the cover's origin, 70 mm past that end at the entry, a 0.5 degree yaw alone counted 0.6 mm and
# an entry within the walls was not taken (w12c21). (A weld made as it stood would carry the tilt along 155 mm of
# rails, through the walls.)
LATERAL = WALL_GAP  # 0.5 mm
ACROSS = (GROOVE_H - LIP_T) / 2  # 0.6 mm
LINE_TILT = math.radians(1.0)
LEAD = np.array([0.0, -CALC[1] / 2, 0.0])  # the calculator's leading end's middle (its ON-key end), in its frame
HOLD_STEPS = 2  # control steps in line before it is taken

OBJECTS = list(mk.MECH.bodies.values())
SIDES = ("left", "right")
FLIP = np.diag([-1.0, 1.0, -1.0])  # half a turn about y: the cover home on the back, in the calculator's frame

# --- state, rules and score ---
STATE: dict = {}
FELL = 0.02  # m under the bench top: a part has left the bench
LEFT_FRONT = "left_front"


def begin(seed: int | None = None) -> dict:
    """A new trial's state (no simulator needed)."""
    STATE.clear()
    STATE.update(seed=seed, on="front", rest=0.0, offers=0, fell=False, left_front=False, events=[], most_in_back=0.0, corrections=[])
    for k in range(len(mk.MECH.bodies["carrier"].shapes)):
        mk.show("carrier", k, False)
    return STATE


def cap(travel: float) -> float:
    """The rails' resistance (N) where the cover stands: the click near home, the rails elsewhere."""
    return DETENT_FORCE if travel <= DETENT_FROM else RAIL_FORCE


def follow(rest: float, travel: float) -> float:
    """Where the spring rests from now on (m of travel): home within CLICK_IN; else where it rested, unless the cover
    has gone further from it than its spring stretches at the cap there (+ a 0.05 mm slack); then that far behind it."""
    if travel <= CLICK_IN:
        return 0.0
    lag = cap(travel) / STIFFNESS
    if abs(travel - rest) <= lag + 0.00005:
        return rest
    return travel - lag if travel > rest else travel + lag


def in_calc_frame(name: str) -> tuple[np.ndarray, np.ndarray]:
    """(position, rotation) of body ``name`` in the calculator's frame."""
    (cp, cr), (bp, br) = mk.pose("calc"), mk.pose(name)
    return cr.T @ (bp - cp), cr.T @ br


def angle(r: np.ndarray) -> float:
    return float(math.acos(float(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0))))


def offer() -> dict:
    """How the cover stands to the calculator: its travel off home along the rails (m); where the calculator's leading
    end's middle (its ON-key end, which enters the cover first in either position) lies off the cover's rail line,
    across (the cover's x) and up or down (its z), in the cover's frame: the walls' and the lips' play acts there; the
    turn off the front and the back positions; and whether the calculator is within the rails' play of the line."""
    p, r = in_calc_frame("cover")
    travel = float(-p[1])
    front, back = angle(r), angle(r @ FLIP.T)
    which = "front" if front <= back else "back"
    tilt = min(front, back)
    lead = r.T @ (LEAD - p)
    across, up = float(lead[0]), float(lead[2])
    # Taken only between home and RAILS_IN onto the rails: in line at a travel of 0 or more the calculator overlaps the
    # cover's channel, which it can reach only through the open end (the end wall closes the other); held in line
    # beyond the closed end, out in the air, its travel is negative, and setting it home would pass it through the end
    # wall. (No higher lower bound: a fast entry first in line after HOLD_STEPS must still be taken, or it jams.)
    in_line = abs(across) <= LATERAL and abs(up) <= ACROSS and tilt <= LINE_TILT and 0.0 <= travel <= RAILS - RAILS_IN
    return {"travel": travel, "across": across, "up": up, "which": which, "tilt": tilt, "in_line": bool(in_line)}


def on_line(which: str, travel: float) -> tuple[np.ndarray, np.ndarray]:
    """(position, rotation) in the world of the calculator exactly on the cover's rails, the cover on its ``which``
    side ("front" or "back"), ``travel`` m off home: the calculator is the part that was brought to the cover."""
    vp, vr = mk.pose("cover")
    rotation = vr if which == "front" else vr @ FLIP  # FLIP is its own inverse
    return vp - rotation @ np.array([0.0, -travel, 0.0]), rotation


def _sim() -> bool:
    return mk.MECH.environment is not None


@mk.on_reset
def start(seed) -> None:
    begin(seed)
    if _sim():  # the simulator keeps stops, a spring's rest and its cap through a reset
        mk.set_limits("rails", 0.0, RAILS + REACH)
        mk.set_spring("rails", rest=0.0, most=DETENT_FORCE)
        mk.push("carrier", force=(0.0, 0.0, CARRIER_MASS * GRAVITY))  # its weight carried for it, from the first step


def rails() -> None:
    """The cover on the rails, once per control step: its resistance, let go past the rails' end; taken back in line."""
    now = round(float(mk.now()), 3)
    if STATE["on"] is not None:
        travel = float(mk.joint("rails"))
        if travel >= RAILS + RELEASE_PAST:
            mk.cut("on_front" if not mk.is_cut("on_front") else "on_again")
            STATE["events"].append((now, f"off the {STATE['on']}"))
            if STATE["on"] == "front":
                STATE["left_front"] = True
            STATE["on"], STATE["rest"], STATE["offers"] = None, 0.0, 0
            if _sim():  # the carrier back home inside the calculator
                mk.set_joint("rails", 0.0)
                mk.set_spring("rails", rest=0.0, most=DETENT_FORCE)
            return
        rest = follow(STATE["rest"], travel)
        before = cap(STATE["rest"]) if STATE["rest"] > CLICK_IN else DETENT_FORCE
        now_cap = cap(rest) if rest > CLICK_IN else DETENT_FORCE
        if _sim() and (rest != STATE["rest"] or now_cap != before):
            mk.set_spring("rails", rest=rest, most=now_cap)
        STATE["rest"] = rest
        if STATE["on"] == "back":
            STATE["most_in_back"] = max(STATE["most_in_back"], RAILS - travel)
        return
    o = offer()
    STATE["offers"] = STATE["offers"] + 1 if o["in_line"] else 0
    if STATE["offers"] >= HOLD_STEPS:
        travel = min(max(o["travel"], 0.0), RAILS)
        if _sim():
            position, rotation = on_line(o["which"], travel)
            mk.write_pose("calc", position, mk.matrix_to_quat(rotation), lin_vel=tuple(mk.velocity("calc")[0]))
            mk.set_joint("rails", travel)
            mk.set_spring("rails", rest=travel, most=cap(travel))
        mk.make("on_again")
        STATE["on"], STATE["rest"], STATE["offers"] = o["which"], travel, 0
        STATE["events"].append((now, f"on the {o['which']}"))
        # The correction the take makes (the calculator set on the cover's line): within the play by the rule above.
        STATE["corrections"].append({"t": now, "across (mm)": round(1000 * o["across"], 3), "up (mm)": round(1000 * o["up"], 3), "tilt (deg)": round(math.degrees(o["tilt"]), 3)})
        if o["which"] == "back":
            STATE["most_in_back"] = max(STATE["most_in_back"], RAILS - travel)


@mk.on_step
def step() -> None:
    rails()
    for name in ("calc", "cover"):
        if mk.pose(name)[0][2] < TOP - FELL:
            STATE["fell"] = True


# --- the score ---
HOME_WITHIN = 0.003  # m from the end stop: home (the spec)
IN_BACK = 0.040  # m into the back rails: 0.5
KEYS_UP = math.radians(20.0)  # the calculator's face within this of up: keys up
FLAT = math.radians(10.0)  # lying flat
RESTING = 0.02  # m/s
CLEAR = 0.03  # m every grasp point keeps from the calculator and the cover at the end


def keys_up() -> bool:
    return float(mk.pose("calc")[1][:, 2] @ np.array([0.0, 0.0, 1.0])) >= math.cos(KEYS_UP)


def lying_flat() -> bool:
    return float(mk.pose("calc")[1][:, 2] @ np.array([0.0, 0.0, 1.0])) >= math.cos(FLAT)


def at_rest() -> bool:
    return all(float(np.linalg.norm(mk.velocity(name)[0])) < RESTING for name in ("calc", "cover"))


def distance_to(point, name: str) -> float:
    """From ``point`` (world) to the box of body ``name`` (m; 0 inside)."""
    p, r = mk.pose(name)
    spec = mk.MECH.bodies[name]
    low, high = spec.bounds()
    local = r.T @ (np.asarray(point, dtype=float) - p)
    return float(np.linalg.norm(local - np.clip(local, low, high)))


def let_go() -> bool:
    return all(min(distance_to(mk.gripper(side)[0], name) for name in ("calc", "cover")) >= CLEAR for side in SIDES if side in mk.MECH.grippers)


def score(objects) -> float:
    """0 if the calculator or the cover left the bench at any moment, or the cover is over the keys at the end (on the
    front, whichever way up the pair lies: the cover on the back with the keys down IS the cover over the keys). 1.0:
    on the back, home within 3 mm, the calculator keys up and lying flat, both at rest, every grasp point 3 cm clear.
    0.5: on the back at least 40 mm in, the calculator keys up, not home or not let go. 0.25: the cover came off the
    front and is not on the front at the end. Else 0."""
    if STATE.get("fell"):
        return 0.0
    on = STATE.get("on")
    if on == "front":
        return 0.0
    if on == "back" and keys_up():
        travel = float(mk.joint("rails"))
        if travel <= HOME_WITHIN and lying_flat() and at_rest() and let_go():
            return 1.0
        if RAILS - travel >= IN_BACK:
            return 0.5
    return 0.25 if STATE.get("left_front") else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def report() -> dict:
    o = offer()
    return {"on": STATE.get("on"), "rails travel (mm)": round(1000 * float(mk.joint("rails")), 2), "cover off home (mm)": round(1000 * o["travel"], 2),
            "cover across, up (mm)": [round(1000 * o["across"], 2), round(1000 * o["up"], 2)], "cover tilt (deg)": round(math.degrees(o["tilt"]), 2),
            "keys up": keys_up(), "left the front": STATE.get("left_front"), "events": STATE.get("events"), "corrections at the takes": STATE.get("corrections")}  # fmt: skip
