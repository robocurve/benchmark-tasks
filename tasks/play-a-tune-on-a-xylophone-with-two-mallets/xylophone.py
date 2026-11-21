"""Play a tune on an 8-bar student xylophone with two mallets, reading the note card.

On the table:
- a wooden xylophone frame (24 x 40 cm board, two felt rails) carrying eight tuned bars C D E F G A B C, each 3.5 cm
  wide, graduated in length from 17 cm (low C, on the robot's left) to 11 cm (high C), each its own colour
- two mallets (22 cm wooden handle, 2.5 cm wooden ball head), one beside each end of the frame, heads forward
- a printed song card on a stand behind the xylophone: seven coloured squares (the bars' colours, left to right) in
  the order the notes are to be played. The tune is C C G G A A G (the first phrase of "Twinkle, Twinkle").
Instruction: "Play the tune on the card, one mallet in each hand."
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import mech  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Sphere  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CONTROL_HZ = 20.0
DEBUG = False

NOTES = ["C", "D", "E", "F", "G", "A", "B", "C2"]
BAR_COLOURS = [(0.90, 0.12, 0.12), (0.98, 0.52, 0.08), (0.98, 0.85, 0.10), (0.18, 0.70, 0.25),
               (0.10, 0.75, 0.80), (0.15, 0.30, 0.85), (0.50, 0.22, 0.75), (0.90, 0.25, 0.62)]
BAR_LEN = [0.170, 0.160, 0.150, 0.142, 0.134, 0.126, 0.118, 0.110]
BAR_W, BAR_T, BAR_PITCH = 0.035, 0.009, 0.040
FRAME_C = (0.36, 0.0)
BOARD = (0.24, 0.40, 0.010)
RAIL = (0.015, 0.36, 0.010)
RAIL_DX = 0.05
BAR_Y = [0.14 - BAR_PITCH * i for i in range(8)]
BAR_TOP = TOP + BOARD[2] + RAIL[2] + BAR_T
TUNE = [0, 0, 4, 4, 5, 5, 4]  # C C G G A A G, as bar indices

# mallets: 22 cm handle (wood), 2.5 cm ball head
HEAD_R = 0.0125
HANDLE = (0.22, 0.012, 0.020)
STAND_H = 0.04  # each mallet lies on two 3 x 4 x 4 cm pine blocks, its handle raised clear of the table
HANDLE_DZ = 0.0
HANDLE_KG, HEAD_KG = 0.040, 0.025
HANDLE_DX = -(HEAD_R + HANDLE[0] / 2)  # handle centre behind the head's centre
GRIP_DX = -0.075  # grip over the mallet's centre of mass, behind the head
HEAD_REST_X, HEAD_REST_Y = 0.33, 0.31
REST_Z = TOP + STAND_H + HANDLE[2] / 2  # height of the ball's centre and the handle's axis on the stand
STAND_X = (-0.03, -0.19)  # block centres behind the ball's centre
WOOD, FELT, HEADCOL, CARD = (0.80, 0.62, 0.38), (0.55, 0.10, 0.12), (0.90, 0.72, 0.45), (0.96, 0.94, 0.86)

OBJECTS = [
    Box("frame_board", size=BOARD, pos=(*FRAME_C, TOP + BOARD[2] / 2), color=WOOD),
    Box("card", size=(0.006, 0.36, 0.20), pos=(0.62, 0.0, TOP + 0.012 + 0.10), color=CARD),
    Box("card_stand", size=(0.08, 0.30, 0.012), pos=(0.65, 0.0, TOP + 0.006), color=WOOD),
]
for _dx in (-RAIL_DX, RAIL_DX):
    OBJECTS.append(Box(f"felt_{'a' if _dx < 0 else 'b'}", size=RAIL, pos=(FRAME_C[0] + _dx, 0.0, TOP + BOARD[2] + RAIL[2] / 2),
                       color=FELT))
for _i, _n in enumerate(NOTES):
    OBJECTS.append(Box(f"bar_{_i}", size=(BAR_LEN[_i], BAR_W, BAR_T),
                       pos=(FRAME_C[0], BAR_Y[_i], TOP + BOARD[2] + RAIL[2] + BAR_T / 2), color=BAR_COLOURS[_i]))
for _k, _t in enumerate(TUNE):  # the card's coloured squares, left to right
    OBJECTS.append(Box(f"card_note_{_k}", size=(0.002, 0.04, 0.04), pos=(0.62 - 0.004, 0.135 - 0.045 * _k, TOP + 0.012 + 0.13),
                       color=BAR_COLOURS[_t]))
MALLETS = {"left": HEAD_REST_Y, "right": -HEAD_REST_Y}
for _s, _y in MALLETS.items():
    OBJECTS.append(Sphere(f"head_{_s}", radius=HEAD_R, pos=(HEAD_REST_X, _y, REST_Z), color=HEADCOL, mass=HEAD_KG))
    OBJECTS.append(Box(f"handle_{_s}", size=HANDLE, pos=(HEAD_REST_X + HANDLE_DX, _y, REST_Z), color=WOOD, mass=HANDLE_KG))
    mech.weld(f"head_{_s}", parent=f"handle_{_s}")
    for _k, _dx in enumerate(STAND_X):
        OBJECTS.append(Box(f"stand_{_s}_{_k}", size=(0.03, 0.04, STAND_H), pos=(HEAD_REST_X + _dx, _y, TOP + STAND_H / 2), color=WOOD))


@mech.on_tick
def _reset(sim):
    if sim.t <= 1.5 / CONTROL_HZ:
        _state.clear()
        _state.update(_fresh())


# -- measurements --------------------------------------------------------------------------
STRIKE_GAP = 0.006  # head bottom within 6 mm of the bar top
STRIKE_SPEED = 0.15  # arriving downward at least this fast (m/s) at some step of the last three
REARM_M = 0.010  # lift the head bottom this far above the bar tops before another strike counts
LET_GO_M = 0.07
STILL_S, STILL_M = 1.0, 0.003
_WINDOW = int(STILL_S * CONTROL_HZ)


def bar_under(hx, hy):
    for i in range(8):
        if abs(hx - FRAME_C[0]) <= BAR_LEN[i] / 2 and abs(hy - BAR_Y[i]) <= BAR_W / 2:
            return i
    return None


def on_frame(p) -> bool:
    return abs(p[0] - FRAME_C[0]) <= BOARD[0] / 2 and abs(p[1] - FRAME_C[1]) <= BOARD[1] / 2


def _fresh():
    return dict(n=0, zh={"left": [], "right": []}, armed={"left": True, "right": True}, pend={"left": None, "right": None}, played=[], hands=[], wrong=False,
                hist=[], fell=False)


_state = _fresh()


def _still(st):
    h = st["hist"]
    if len(h) < _WINDOW:
        return False
    a = np.asarray(h[-_WINDOW:])
    return bool(np.all(np.abs(a - a[-1]) < STILL_M))


def score(objects):
    """Strikes: a mallet head arriving on a bar's top (its bottom within 3 mm, centre over the bar, moving down at
    0.15 m/s or more at one of the last three steps, then reversing upward), counted once until the head bottom has risen 1 cm above the bars.
    Correct prefix k: the first k strikes are the tune's first k notes (any extra note after the tune, or a note that is
    not the next, ends the count and caps the score). Score = 0.6 * k / 7, plus 0.4 when all 7 notes were played with
    no wrong or extra note, each mallet played at least 2 of them, and at the end both mallets lie back on their stands (handle within 3 cm of its start and 1 cm of its height), still for 1 s, with both grippers at least 7 cm from both handles. 0 for doing nothing or a mallet off the
    table."""
    st = _state
    st["n"] += 1
    handles = {s: np.asarray(objects[f"handle_{s}"].pos, float) for s in MALLETS}
    heads = {s: np.asarray(objects[f"head_{s}"].pos, float) for s in MALLETS}
    st["hist"] = (st["hist"] + [np.concatenate([handles["left"], handles["right"]])])[-(_WINDOW + 5):]
    for s in MALLETS:
        if heads[s][2] < TOP - 0.05:
            st["fell"] = True
        z = float(heads[s][2])
        zh = st["zh"][s] = (st["zh"][s] + [z])[-4:]
        bottom = z - HEAD_R
        if DEBUG and 7.0 < st["n"] / CONTROL_HZ < 11.0 and s == "left":
            print(f"DBG L step {st['n']} x={heads[s][0]:.3f} y={heads[s][1]:.3f} bot={bottom - BAR_TOP:+.4f}", flush=True)
        if bottom >= BAR_TOP + REARM_M:
            st["armed"][s] = True
        if st["pend"][s] is not None and len(zh) >= 2 and z > zh[-2] + 0.0005:  # reversing: the strike is complete
            j, v = st["pend"][s]
            st["pend"][s] = None
            ok = (not st["wrong"]) and len(st["played"]) < len(TUNE) and TUNE[len(st["played"])] == j
            if ok:
                st["played"].append(j)
                st["hands"].append(s)
            else:
                st["wrong"] = True
            if DEBUG:
                print(f"DBG strike t={st['n'] / CONTROL_HZ:.2f} hand={s} bar={NOTES[j]} v={v:.2f} ok={ok}", flush=True)
            continue
        j = bar_under(heads[s][0], heads[s][1])
        if j is None or not st["armed"][s] or bottom - BAR_TOP > STRIKE_GAP or bottom - BAR_TOP < -0.012:
            continue
        v = max([(zh[i - 1] - zh[i]) * CONTROL_HZ for i in range(1, len(zh))] + [0.0])
        if v < STRIKE_SPEED:
            continue
        st["armed"][s] = False
        st["pend"][s] = (j, v)
    k = len(st["played"])
    if DEBUG and st["n"] % 20 == 0:
        print(f"DBG t={st['n'] / CONTROL_HZ:.0f} k={k} wrong={st['wrong']} hl=({heads['left'][0]:.3f},{heads['left'][1]:.3f},"
              f"{heads['left'][2] - TOP:.3f}) hr=({heads['right'][0]:.3f},{heads['right'][1]:.3f},{heads['right'][2] - TOP:.3f})", flush=True)
    if st["fell"] or k == 0:
        return 0.0
    base = 0.6 * k / len(TUNE)
    if st["wrong"] or k < len(TUNE):
        return base
    if min(st["hands"].count(s) for s in MALLETS) < 2:
        return base
    down = all(np.linalg.norm(handles[s][:2] - np.array([HEAD_REST_X + HANDLE_DX, MALLETS[s]])) <= 0.03
               and abs(handles[s][2] - REST_Z) <= 0.01 for s in MALLETS)
    far = all(np.linalg.norm(np.asarray(objects[t].pos, float) - handles[s]) >= LET_GO_M
              for t in ("left_tcp", "right_tcp") for s in MALLETS)
    if down and far and _still(st):
        return 1.0
    return base


# -- scripted demo -------------------------------------------------------------------------
DESCEND, TRAVEL = 0.25, 0.3
HOV_DZ = 0.060
TCP_OFF = 0.0065  # the grip sits this far above the head centre's height (the handle is 2 cm tall)
STRIKE_Z = TOP + BOARD[2] + RAIL[2] + BAR_T + HEAD_R + TCP_OFF  # grip height with the head just on the bar
HOVER_Z = STRIKE_Z + HOV_DZ
LIE_Z = REST_Z + TCP_OFF
STRIKE_X = FRAME_C[0] - 0.03  # struck 3 cm toward the player from the bar's middle, so the fingers stay clear of the bar
GX_STRIKE = STRIKE_X + GRIP_DX
GX_REST = HEAD_REST_X + GRIP_DX
PRESS = 0.004
PARK = {"left": (0.18, 0.27, TOP + 0.20), "right": (0.18, -0.27, TOP + 0.20)}
NOTE_T = {"left": [7.4, 8.4], "right": [9.4, 10.4, 11.4, 12.4, 13.4]}


def _arm(side):
    ry = MALLETS[side]
    wp = [Waypoint(2.0, (GX_REST, ry, LIE_Z + 0.05), FINGER_OPEN, "over the mallet's handle"),
          Waypoint(1.2, (GX_REST, ry, LIE_Z), FINGER_OPEN, "down around the handle"),
          Waypoint(0.6, (GX_REST, ry, LIE_Z), FINGER_CLOSED, "grip it"),
          Waypoint(1.0, (GX_REST, ry, HOVER_Z), FINGER_CLOSED, "lift the mallet")]
    cur = 4.8
    seq = sorted([(t, TUNE[i]) for i, t in enumerate(sorted(NOTE_T["left"] + NOTE_T["right"]))])
    mine = [(t, b) for t, b in seq if t in NOTE_T[side]]
    for t, b in mine:
        y = BAR_Y[b]
        wp.append(Waypoint(t - DESCEND - cur, (GX_STRIKE, y, HOVER_Z), FINGER_CLOSED, f"over bar {NOTES[b]}"))
        wp.append(Waypoint(DESCEND, (GX_STRIKE, y, STRIKE_Z - PRESS), FINGER_CLOSED, f"strike {NOTES[b]}"))
        wp.append(Waypoint(TRAVEL, (GX_STRIKE, y, HOVER_Z), FINGER_CLOSED, "lift off"))
        cur = t + TRAVEL
    wp.append(Waypoint(1.6, (GX_REST, ry, HOVER_Z), FINGER_CLOSED, "carry the mallet back"))
    wp.append(Waypoint(1.0, (GX_REST, ry, LIE_Z + 0.002), FINGER_CLOSED, "lay it down"))
    wp.append(Waypoint(0.6, (GX_REST, ry, LIE_Z + 0.002), FINGER_OPEN, "let go"))
    wp.append(Waypoint(1.0, (GX_REST, ry, LIE_Z + 0.07), FINGER_OPEN, "up, clear"))
    wp.append(Waypoint(1.5, PARK[side], FINGER_OPEN, "park"))
    return wp


def demo():
    left, right = _arm("left"), _arm("right")
    tl, tr = sum(w.duration for w in left), sum(w.duration for w in right)
    end = max(tl, tr) + 2.0
    left.append(Waypoint(end - tl, PARK["left"], FINGER_OPEN, "still"))
    right.append(Waypoint(end - tr, PARK["right"], FINGER_OPEN, "still"))
    return {"left": left, "right": right}
