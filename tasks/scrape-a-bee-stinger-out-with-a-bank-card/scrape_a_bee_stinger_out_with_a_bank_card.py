"""Scrape a bee stinger out of the back of a hand with the edge of a bank card, never pinching it.

On the station table: a life-size silicone hand lies palm down, strapped to a pine block, fingers pointing at the
robot. A bee stinger (a stand-in: 0.5 mm black nylon with a 1.5 mm black bead as the venom sac) stands upright in the
back of the hand inside a 12 mm ink ring, 3 mm out and 3 mm in its track. To the right, a blank bank card (CR80,
0.76 mm PVC) stands on its long edge in a kerf in a small pine block.

- MedlinePlus: scrape the back of a credit card or other straight edge across the stinger; do not use tweezers, which
  may squeeze the venom sac. NHS: scrape it sideways with a fingernail or the edge of a bank card.
- The stinger leans on two hinges at the skin and slides along its track against friction. A card edge drawn
  sideways along the skin catches its base, tips it over and drags it out; pressed down on it, it goes in; a gripper
  finger on it is the pinch the sources forbid.

The skin is a pad on a sprung slider (declared 1 N/mm). The scorer only reads poses and joint positions every control
step; nothing is moved, latched or held by code.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import ourkit as k  # noqa: E402
from ourkit import demo as D  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1.0 state already includes 1 s released and still
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Get the bee stinger out of the back of the hand with the bank card standing in the block, then lay the "
               "card down on the table and let go. Only the card may touch the stinger.")
SWEEP = json.loads(os.environ.get("TASK_SWEEP", "") or "{}")
R = math.radians
DT = 1.0 / 20.0

# -- the props (cm in the text: x forward, y left, from the arm bases' midpoint) ------------------------------------
BLOCK_XY, BLOCK = (0.32, -0.06), (0.15, 0.15, 0.02)  # pine block, screwed down
STING = np.array([0.31, -0.06, TOP + 0.055])  # the sting site on the back of the hand, at the skin
PAD_T = 0.004  # the skin pad's travel: it dents up to 4 mm
PAD_X, PAD_Y = 0.100, 0.070  # the soft skin over the back of the hand
K_SKIN_N_PER_MM_DECLARED = 1.0  # the skin pad: 1 N per mm
K_SKIN_N_PER_MM = K_SKIN_N_PER_MM_DECLARED * SWEEP.get("skin", 1.0)
PROUD = 0.003  # the stinger stands 3 mm out
BURIED = 0.003  # and 3 mm in its track
STUB = 0.0015  # it collides as a 1.5 mm square stick (drawn 0.5 mm with a 1.5 mm bead)
GAP_UNDER = 0.0002  # the stick's foot sits 0.2 mm above the pad (no resting contact with its own skin)
F_HOLD_N = 0.5  # N: the track's grip along the stinger
F_HOLD = F_HOLD_N * SWEEP.get("hold", 1.0)
LEAN_RATE = 0.003 * SWEEP.get("lean", 1.0)  # N m/rad: the silicone holding it upright
HAND_KG = 0.5  # the real hand's mass; it lies loose on the block in the sim (strapped in the real setup)
LINK_KG = 0.008  # each moving part of the skin and stinger (PhysX: link masses within ~100:1 of the hand)
CARD = (0.0856, 0.00076, 0.054)  # CR80 card: 85.6 x 0.76 x 54 mm standing on its long edge, face normal along y
CARD_T = CARD[1]
CARD_KG = 0.0055 * SWEEP.get("card_mass", 1.0)
KERF_XY, KERF = (0.24, -0.22), (0.08, 0.03, 0.02)  # pine block with a 1 mm kerf 10 mm deep along its top
KERF_W, KERF_D = 0.001, 0.010
CARD_START = (KERF_XY[0], KERF_XY[1], TOP + KERF[2] - KERF_D + CARD[2] / 2 + 0.0003)

SKIN_C, PINE, INK, CARD_C, BLACK = (0.86, 0.68, 0.58), (0.80, 0.66, 0.44), (0.05, 0.05, 0.05), (0.96, 0.96, 0.94), (0.03, 0.03, 0.03)


def _fixed() -> list:
    kx, ky = KERF_XY
    side = (KERF[1] - KERF_W) / 2
    return [Box("block", pos=(*BLOCK_XY, TOP + BLOCK[2] / 2), size=BLOCK, color=PINE),
            Box("kerf_base", pos=(kx, ky, TOP + (KERF[2] - KERF_D) / 2), size=(KERF[0], KERF[1], KERF[2] - KERF_D),
                color=PINE),
            Box("kerf_l", pos=(kx, ky + KERF_W / 2 + side / 2, TOP + KERF[2] - KERF_D / 2), size=(KERF[0], side, KERF_D),
                color=PINE),
            Box("kerf_r", pos=(kx, ky - KERF_W / 2 - side / 2, TOP + KERF[2] - KERF_D / 2), size=(KERF[0], side, KERF_D),
                color=PINE)]


# The hand: a heavy body (frame origin at the sting site, on the skin) of boxes, the skin pad on a slider, and the
# stinger on two lean hinges and a draw slider along its track.
HAND_BASE_Z = TOP + BLOCK[2] - STING[2]  # the block's top in the hand frame (-35 mm)
HAND_BOXES = [
    k.span((-0.05, -0.04, HAND_BASE_Z), (0.06, 0.04, -PAD_T - 0.0003), SKIN_C, label="back"),  # palm and back
    k.span((-0.12, -0.035, HAND_BASE_Z), (-0.05, 0.035, -0.017), SKIN_C, label="fingers"),
    k.span((0.06, -0.03, HAND_BASE_Z), (0.10, 0.03, -0.008), SKIN_C, label="wrist"),
]
PAD_BOXES = [k.box((0.005, 0.0, -PAD_T / 2), (PAD_X, PAD_Y, PAD_T), SKIN_C, label="skin")]
PAD_BOXES += [k.box((0.006 * math.cos(a), 0.006 * math.sin(a), 0.00002), (0.0012, 0.0016, 0.00004), INK,
                    turn=(0.0, 0.0, a), ghost=True, label=f"ring{i}") for i, a in enumerate(np.linspace(0, 2 * math.pi, 16, endpoint=False))]
# each lean link carries a 0.6 mm hidden collider at the hinge, its top flush with the skin (PhysX takes a link's
# inertia from its colliders; parts of one mech never collide with each other)
LINK_BOX = ((0.0, 0.0, -0.0003), (0.0006, 0.0006, 0.0006))
LEAN_ARMATURE = 1e-6  # kg m^2 on each lean hinge: a steady joint for the light links
STING_BOXES = [k.box((0.0, 0.0, GAP_UNDER + PROUD / 2), (STUB, STUB, PROUD), BLACK, label="stinger"),
               k.box((0.0, 0.0, GAP_UNDER + PROUD - 0.0006), (0.002, 0.002, 0.0012), BLACK, ghost=True, label="sac")]
SKIN = k.slider("skin", (0.0, 0.0, 0.0), "Z", -PAD_T, 0.0, PAD_BOXES, rate=K_SKIN_N_PER_MM * 1000.0, target=0.0,
                damping=2.0, mass=LINK_KG, weightless=True)
LEAN_X = k.hinge("lean_x", (0.0, 0.0, 0.0), "X", -R(90), R(90), [k.box(*LINK_BOX, SKIN_C, hidden=True, label="link1")],
                 parent="skin", rate=LEAN_RATE, target=0.0, damping=0.0005, mass=LINK_KG, weightless=True,
                 armature=LEAN_ARMATURE)
LEAN_Y = k.hinge("lean_y", (0.0, 0.0, 0.0), "Y", -R(90), R(90), [k.box(*LINK_BOX, SKIN_C, hidden=True, label="link2")],
                 parent="lean_x", rate=LEAN_RATE, target=0.0, damping=0.0005, mass=LINK_KG, weightless=True,
                 armature=LEAN_ARMATURE)
DRAW = k.slider("draw", (0.0, 0.0, 0.0), "Z", -BURIED, 0.008, STING_BOXES, parent="lean_y", friction=F_HOLD,
                damping=0.5, mass=LINK_KG, weightless=True)
HAND = k.mech("hand", tuple(STING), HAND_BOXES, HAND_KG, joints=[SKIN, LEAN_X, LEAN_Y, DRAW], color=SKIN_C, surface=0.6)
CARD_MECH = k.mech("card", CARD_START, [k.box((0, 0, 0), CARD, CARD_C, label="card")], CARD_KG, color=CARD_C, surface=0.5,
                   contact_offset=0.0005)
OBJECTS = [HAND, CARD_MECH, *_fixed()]

# -- scoring -----------------------------------------------------------------------------------------------------------
TIPPED_DEG = 30.0  # 0.25: the card has tipped the stinger 30 degrees
OUT = BURIED  # 0.5: drawn 3 mm, no part in its track
PUSHED_IN = 0.002  # pushed 2 mm further in: pressed down on the sac (zero)
TOUCH = 0.00002  # a gripper finger within 0.02 mm of the stinger: contact
HAND_MOVED = 0.005
HAND_OFF, HAND_TIP_DEG = 0.020, 10.0
CLEAR_MM = 30.0
STILL_S, STILL_TOL = 1.0, 0.002
ON_TABLE = 0.002  # the card's lowest corner within 2 mm of the table top
FLAT_MAX = 0.005  # and its highest corner under 5 mm: lying flat


def lean_deg(h) -> float:
    """The stinger's lean from upright (deg)."""
    ax = h.spec.frames(h.q)["draw"][0] @ np.array([0.0, 0.0, 1.0])
    return math.degrees(math.acos(float(np.clip(ax[2], -1.0, 1.0))))


def drawn(h) -> float:
    return float(h.q["draw"])


def stinger_points(h) -> np.ndarray:
    return k.score.surface_points(h, "draw", STING_BOXES[:1], 0.0005)


def card_corners(c) -> np.ndarray:
    half = np.array(CARD) / 2
    rot = c.rotation()
    return np.array([np.asarray(c.pos, float) + rot @ (half * s) for s in
                     [(a, b, d) for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)]])


def card_on_table(c) -> bool:
    """Resting on the table, off the hand, its block and the card block."""
    pts = card_corners(c)
    x, y = float(c.pos[0]), float(c.pos[1])
    on_block = abs(x - BLOCK_XY[0]) < BLOCK[0] / 2 + 0.005 and abs(y - BLOCK_XY[1]) < BLOCK[1] / 2 + 0.005
    on_kerf = abs(x - KERF_XY[0]) < KERF[0] / 2 + 0.005 and abs(y - KERF_XY[1]) < KERF[1] / 2 + 0.005
    return float(pts[:, 2].min()) < TOP + ON_TABLE and float(pts[:, 2].max()) < TOP + FLAT_MAX and not on_block and not on_kerf


PINCH, PUSHED, OFF, TIPPED, END = k.Sticky(), k.Sticky(), k.Sticky(), k.Ever(), k.Settle(STILL_S, STILL_TOL)
RUN: dict = {"moved": 0.0, "start": None}
REC = k.run_record.RunRecord(__file__, "hand", "card")


def grade(states) -> float:
    if PINCH or PUSHED or OFF:
        return 0.0
    h, c = states["hand"], states["card"]
    out = drawn(h) >= OUT - 1e-6
    laid = out and card_on_table(c) and k.hands_clear(card_corners(c), 1.0)
    return k.ladder((0.25, bool(TIPPED)), (0.5, out), (0.75, laid),
                    (1.0, laid and RUN["moved"] <= HAND_MOVED and END.ok))


def score(states) -> float:
    return grade(states)


def success(states) -> bool:
    return grade(states) >= 1.0


def distance(states) -> float:
    return max(0.0, OUT - drawn(states["hand"]))


@k.on_reset
def _reset(states) -> None:
    PINCH.reset(), PUSHED.reset(), OFF.reset(), TIPPED.reset(), END.reset()
    RUN.update(moved=0.0, start=np.asarray(states["hand"].pos, float).copy(), n=0)
    REC.reset()


@k.on_step
def _watch(states) -> None:
    h, c = states["hand"], states["card"]
    if RUN["start"] is None:
        RUN["start"] = np.asarray(h.pos, float).copy()
    grips = k.score.grippers()
    if grips and k.score.clearance(stinger_points(h), grips, beyond=0.01) <= TOUCH:
        PINCH.trip("a gripper finger touched the stinger")
    if drawn(h) <= -PUSHED_IN:
        PUSHED.trip("the stinger was pushed into the skin")
    TIPPED.update(lean_deg(h) >= TIPPED_DEG)
    RUN["moved"] = max(RUN["moved"], float(np.linalg.norm(np.asarray(h.pos, float) - RUN["start"])))
    if RUN["moved"] > HAND_OFF or h.tilt_deg() > HAND_TIP_DEG:
        OFF.trip("the hand came off its block")
    pts = np.concatenate([k.score.surface_points(h, "", HAND_BOXES, 0.02), card_corners(c)])
    END.push(np.concatenate([np.asarray(c.pos, float), [drawn(h), lean_deg(h) / 1000.0]]),
             k.hands_clear(pts, CLEAR_MM, grips))
    REC.tick(states, lambda: f"skin {float(h.q['skin']) * 1000:.2f} mm lean {lean_deg(h):.1f} deg drawn "
                             f"{drawn(h) * 1000:.2f} mm card {np.round(np.asarray(c.pos) * 1000, 1)} pinch {bool(PINCH)} "
                             f"pushed {bool(PUSHED)} score {grade(states):g}")
    if os.environ.get("TASK_DEBUG"):
        RUN["n"] = RUN.get("n", 0) + 1
        if RUN["n"] % 10 == 0:
            print(f"DBG t {RUN['n'] * DT:.1f} skin {float(h.q['skin']) * 1000:.2f} lean {lean_deg(h):.1f} "
                  f"drawn {drawn(h) * 1000:.2f} card {np.round(np.asarray(c.pos) * 1000, 1)} tilt {c.tilt_deg():.1f} "
                  f"hand {RUN['moved'] * 1000:.1f} pinch {bool(PINCH)} pushed {bool(PUSHED)} g {grade(states)}",
                  flush=True)


# -- the demo ---------------------------------------------------------------------------------------------------------
G_PICK = D.Grip.deg(yaw=0)  # straight down, fingers closing along y (the card's face normal)
ROLL = 20.0
G_SCRAPE = D.Grip.deg(roll=ROLL)  # the card leans 20 deg: its lower edge at +y of the grip, trailing a drag toward -y
GRIP_BELOW_TOP = 0.020  # the TCP 20 mm below the card's top edge (the pads cover its top 33 mm)
TCP_TO_EDGE = CARD[2] - GRIP_BELOW_TOP  # along the approach, from the TCP to the card's lower edge
DENT_T = 0.0005  # the edge 0.5 mm into the skin pad (0.5 N)
START_DY, END_DY = 0.015, -0.045  # the drag runs from 15 mm left of the stinger to 45 mm right of it
V_DRAG = 0.010  # m/s
V_DOWN = 0.004
HOVER = 0.004  # the edge this far above the skin before the descent
KI, KD = 2.0, 4.0
LAY_XY = (0.17, -0.10)  # where the card is laid on the table (clear of both blocks)


def card_top(st) -> np.ndarray:
    c = st["card"]
    return np.asarray(c.pos, float) + c.rotation() @ np.array([0.0, 0.0, CARD[2] / 2])


def edge_mid(st) -> np.ndarray:
    c = st["card"]
    return np.asarray(c.pos, float) + c.rotation() @ np.array([0.0, 0.0, -CARD[2] / 2])


def hand_plan(how: str) -> D.Plan:
    """Pick the card, scrape (``how``: "demo", "press", "high", "pinch", "held"), lay it down."""
    p = D.Plan("right")
    top0 = np.array([KERF_XY[0], KERF_XY[1], CARD_START[2] + CARD[2] / 2])
    if how == "pinch":  # the tweezer move: close the gripper on the stinger itself
        p.move(2.5, lambda st: STING + (0.0, 0.0, 0.06), finger=D.gap_finger(0.03), grip=G_PICK, label="over the stinger",
               fallback=STING + (0.0, 0.0, 0.06))
        p.move(2.0, lambda st: STING + (0.0, 0.0, 0.012), label="fingers round it", fallback=STING + (0.0, 0.0, 0.012))
        p.fingers(0.0, 1.0, "pinch it").hold(0.5)
        p.move(1.5, lambda st: STING + (0.0, 0.0, 0.05), label="pull up", fallback=STING + (0.0, 0.0, 0.05))
        return p.home()
    p.move(2.0, lambda st: card_top(st) + (0.0, 0.0, 0.05), finger=D.gap_finger(0.02), grip=G_PICK, label="above the card",
           fallback=top0 + (0, 0, 0.05))
    p.move(1.5, D.remember(p, "grip", lambda st: card_top(st) - (0.0, 0.0, GRIP_BELOW_TOP)), label="pads round its top",
           fallback=top0 - (0, 0, GRIP_BELOW_TOP))
    p.fingers(0.0, 1.0, "pinch the card").hold(0.3)
    p.move(1.5, D.remember(p, "up", lambda st: np.asarray(p.memo["grip"]) + (0.0, 0.0, 0.060)), label="lift it out",
           fallback=top0 + (0, 0, 0.05))
    appr = G_SCRAPE.approach
    tilt_z = TCP_TO_EDGE * appr  # TCP -> edge, the card at its nominal lean

    def over_start(st):
        edge = STING + (0.0, 0.0 if how == "press" else START_DY, HOVER + (0.008 if how == "high" else 0.0))
        p.memo["off"] = np.zeros(3)
        return edge - tilt_z + (0.0, 0.0, 0.02)

    p.move(2.5, D.remember(p, "over", over_start), grip=G_SCRAPE, label="tilt it over the start",
           fallback=STING - tilt_z + (0, START_DY, 0.03))
    p.hold(0.5)
    dy0 = 0.0 if how == "press" else START_DY  # the press cheat comes down right on the stub

    def edge_off(st):  # command minus the edge's live position: the grip's slip and the arm's lag
        return np.asarray(p.memo["_last"]) - edge_mid(st)

    p.move(1.5, D.remember(p, "hover", lambda st: STING + (0.0, dy0, HOVER + (0.008 if how == "high" else 0.0))
                           + edge_off(st)), label="edge just over the skin", fallback=STING - tilt_z + (0, START_DY, 0.01))
    p.hold(0.4)
    ctl = {"z": None, "touch": False, "y": 0.0, "q": 0.0, "v": 0.0, "lift": 0.0}
    dent_t = 0.004 if how == "press" else DENT_T

    def drag(st):
        h = st["hand"]
        q = -float(h.q["skin"])  # the pad's dent
        ctl["v"] = 0.7 * ctl["v"] + 0.3 * (q - ctl["q"]) / DT
        ctl["q"] = q
        if ctl["z"] is None:
            ctl["z"] = np.asarray(p.memo["hover"], float).copy()
        z = ctl["z"]
        if ctl.get("out") or (how != "press" and drawn(h) >= OUT + 0.002):  # out: lift the edge off the skin
            ctl["out"] = True
            z = z + np.array([0.0, 0.0, 0.001])
        elif how == "high":  # pass 4 mm over the skin: no servo
            ctl["y"] = max(ctl["y"] - V_DRAG * DT, END_DY - START_DY)
            z = np.asarray(p.memo["hover"], float) + (0.0, ctl["y"], 0.0)
        elif not ctl["touch"]:
            z = z - np.array([0.0, 0.0, V_DOWN * DT])
            ctl["touch"] = q > 0.0002
        else:
            ahead = q + 1.0 * ctl["v"]
            dz = float(np.clip((-KI * (dent_t - ahead) + KD * ctl["v"]) * DT, -0.0002, 0.0002))
            if how == "press":  # the card's flat face down on the stub: no drag, push down at the stinger
                z = z + np.array([0.0, 0.0, dz])
            else:
                ctl["y"] = max(ctl["y"] - V_DRAG * DT, END_DY - START_DY)
                z = np.array([z[0], np.asarray(p.memo["hover"])[1] + ctl["y"], z[2] + dz])
        ctl["z"] = z
        p.memo["_last"] = p.memo["drag"] = z
        return z

    D.follow(p, drag, 3.0 + (END_DY - START_DY) / -V_DRAG + 2.0, fallback=STING - tilt_z + (0.0, START_DY, 0.0),
             label="drag the edge across")
    p.move(1.5, D.remember(p, "off", lambda st: np.asarray(p.memo["drag"]) + (0.0, 0.0, 0.05)), label="lift off the skin",
           fallback=STING + (0, -0.04, 0.06))
    if how == "held":
        return p.hold(6.0, "keep holding the card")
    lay = np.array([LAY_XY[0], LAY_XY[1], TOP])
    p.move(2.5, D.remember(p, "lay_over", lambda st: lay - tilt_z + (0.0, 0.0, 0.03)), label="over the table",
           fallback=lay - tilt_z + (0, 0, 0.03))
    p.move(1.5, D.remember(p, "lay", lambda st: np.asarray(p.memo["lay_over"]) - (0.0, 0.0, 0.027)),
           label="edge down near the table", fallback=lay - tilt_z + (0, 0, 0.003))
    p.fingers(0.02, 0.8, "let go of the card").hold(0.8)
    p.move(1.5, lambda st: np.asarray(p.memo["lay"]) + (0.0, 0.0, 0.06), label="rise clear", fallback=lay + (0, 0, 0.1))
    return p.home()


def demo():
    def one(how):
        return D.together(hand_plan(how), D.idle_plan("left", 1.0))

    return D.choose(demo=lambda: one("demo"), press=lambda: one("press"), high=lambda: one("high"),
                    pinch=lambda: one("pinch"), held=lambda: one("held"))


# -- what the template tests read ---------------------------------------------------------------------------------------
TEXT_NUMBERS = [("3 mm", PROUD), ("2 mm", PUSHED_IN), ("0.76 mm", CARD_T),
                ("1 N per mm", K_SKIN_N_PER_MM_DECLARED), ("0.5 N", F_HOLD_N), ("30 mm", CLEAR_MM / 1000.0),
                ("5 mm", HAND_MOVED), ("1 s", STILL_S)]
CHEATS = {"none": 0.0, "press": 0.0, "high": 0.0, "pinch": 0.0, "held": 0.5}
HOLD = {"hand.draw": 0.5, "hand.lean_x": 2.0, "hand.lean_y": 2.0}
