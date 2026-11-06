"""On the station table, centred between the arms and fixed to the table: a large-key computer keyboard with blank key
caps (25 mm key pitch), 38 cm wide and 13 cm deep, in the full standard US (ANSI) layout of a keyboard's main block,
61 keys in five rows:
- number row (far): ` 1 2 3 4 5 6 7 8 9 0 - =, Backspace
- Tab, Q W E R T Y U I O P [ ] \\
- Caps Lock, A S D F G H J K L ; ', Enter
- Left Shift, Z X C V B N M , . /, Right Shift
- bottom row (near): Ctrl, Win, Alt, the space bar, Alt, Win, Menu, Ctrl
Character keys and the space bar are light grey; the other keys are dark grey. The caps are blank: which key is which
follows from the standard layout.

Each key (23 x 23 mm cap; wider keys as on any keyboard) travels 4 mm down and springs back. It types when pushed
down at least 2.5 mm, and types again only after it has come back up. A key typed while either Shift key is held
down types its shifted character (a capital letter, ! for 1, ...); Caps Lock toggles capitals for letters (Shift
then types lower case). Backspace deletes the last character, Space, Tab and Enter type a space, tab and new line;
the other keys type nothing.

Run the scripted demo:  inspect-robots run --embodiment yam_arena --policy yam_scripted \\
    -E task=tasks/keyboard_typing/keyboard_typing.py -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
    --instruction "Type 'RoboCurve' on the keyboard, with a capital R and a capital C." --scorer task_score --max-steps 2400
"""

from __future__ import annotations

import importlib.util
import math
import os
import sys
from pathlib import Path

from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

# Load yampatch.py from this folder (task files are imported by path, so their folder is not on sys.path).
if "yampatch" not in sys.modules:
    _spec = importlib.util.spec_from_file_location("yampatch", Path(__file__).with_name("yampatch.py"))
    sys.modules["yampatch"] = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(sys.modules["yampatch"])
yampatch = sys.modules["yampatch"]

SCENE = "station"
TOP = TABLE_TOP[SCENE]
TARGET = "RoboCurve"
_DEBUG = bool(os.environ.get("KEYBOARD_DEBUG"))  # print keystrokes

# -- the keyboard: one fixed-base articulation, a spring-loaded prismatic joint per key ------------------------------
PITCH = 0.025
CAP = (0.023, 0.023, 0.008)  # x (depth), y (width of a 1-unit key), z
TRAVEL = 0.004
GAP = TRAVEL + 0.0005  # cap bottom above the deck at rest
DECK_H = 0.012  # housing height: the deck
DOWN, UP = -0.0025, -0.0012  # joint position (m): pressed below DOWN, released again above UP
ROW_X = {0: 0.275, 1: 0.25, 2: 0.225, 3: 0.20, 4: 0.175}  # world x of the rows: 0 = number row (far), 4 = bottom
X0 = ROW_X[2]  # the keyboard's origin (world x; y = 0, z = table top)
U_MID = 7.5  # key units from the keyboard's left edge at its centre line (it is 15 units wide)

# Each row from the left: (character typed, or a name for keys that type none, width in units).
_ROW_KEYS = {
    0: [*"`1234567890-=", ("backspace", 2.0)],
    1: [("tab", 1.5), *"qwertyuiop[]", ("\\", 1.5)],
    2: [("caps", 1.75), *"asdfghjkl;'", ("enter", 2.25)],
    3: [("lshift", 2.25), *"zxcvbnm,./", ("rshift", 2.75)],
    4: [("lctrl", 1.25), ("lwin", 1.25), ("lalt", 1.25), (" ", 6.25), ("ralt", 1.25), ("rwin", 1.25), ("menu", 1.25),
        ("rctrl", 1.25)],
}
_NAMES = {"`": "grave", "-": "minus", "=": "equal", "[": "lbracket", "]": "rbracket", "\\": "backslash",
          ";": "semicolon", "'": "quote", ",": "comma", ".": "period", "/": "slash", " ": "space"}
_TYPES = {"tab": "\t", "enter": "\n"}
SHIFTED = dict(zip("`1234567890-=[]\\;',./", '~!@#$%^&*()_+{}|:"<>?', strict=True))
# (key name = joint name minus "key_", row, centre in units from the left edge, width in units, character or None)
LAYOUT = []
for _r, _keys in _ROW_KEYS.items():
    _u = 0.0
    for _k in _keys:
        _label, _w = (_k, 1.0) if isinstance(_k, str) else _k
        _char = _label if len(_label) == 1 else _TYPES.get(_label)
        LAYOUT.append((_NAMES.get(_label, _label), _r, _u + _w / 2, _w, _char))
        _u += _w
SHIFTS = ("lshift", "rshift")
CHAR = {name: c for name, _, _, _, c in LAYOUT}
HOME = {name: (ROW_X[r], -(u - U_MID) * PITCH) for name, r, u, _, _ in LAYOUT}  # world x, y of each key
_LIGHT = {name for name, c in CHAR.items() if c and c not in "\t\n"}

_LO = min(u - w / 2 for _, _, u, w, _ in LAYOUT)
_HI = max(u + w / 2 for _, _, u, w, _ in LAYOUT)
BASE = (len(ROW_X) * PITCH + 0.012, (_HI - _LO) * PITCH + 0.012, DECK_H)
BASE_Y = -((_LO + _HI) / 2 - U_MID) * PITCH  # the housing's centre (world y)

KEYS = [
    yampatch.Button(
        f"key_{name}",
        pos=(ROW_X[r] - X0, -(u - U_MID) * PITCH - BASE_Y, DECK_H + GAP + CAP[2] / 2),
        size=(CAP[0], CAP[1] + (w - 1) * PITCH, CAP[2]),
        color=(0.88, 0.88, 0.85) if name in _LIGHT else (0.45, 0.47, 0.52),
        travel=TRAVEL,
        mass=0.005 * w,
    )
    for name, r, u, w, c in LAYOUT
]
KEYBOARD = yampatch.usd_object(
    "keyboard", (X0, BASE_Y, TOP), kind="articulation", extent=BASE[1],
    build=lambda: yampatch.build_button_panel("keyboard", BASE, (0.13, 0.13, 0.15), KEYS),
)
OBJECTS = [KEYBOARD]


# -- state: the typed text ---------------------------------------------------------------------------------------------


class Typist:
    """Turns key travel into keystrokes. A key goes down when its joint passes DOWN and up when it is back above UP;
    a character key types when it goes down, shifted if a Shift key is down by then (Shift is read first)."""

    def __init__(self) -> None:
        self.text = ""
        self.down: set[str] = set()
        self.caps = False

    def update(self, q: dict[str, float]) -> None:
        for name in sorted(CHAR, key=lambda k: k not in SHIFTS):
            pos = q[f"key_{name}"]
            if name in self.down:
                if pos > UP:
                    self.down.discard(name)
                continue
            if pos >= DOWN:
                continue
            self.down.add(name)
            shift = any(s in self.down for s in SHIFTS)
            c = CHAR[name]
            if name == "backspace":
                self.text = self.text[:-1]
            elif name == "caps":
                self.caps = not self.caps
            elif c and c.isalpha():
                self.text += c.upper() if shift != self.caps else c
            elif c:
                self.text += SHIFTED.get(c, c) if shift else c
            if _DEBUG:
                print(f"[keyboard] {name} -> {self.text!r}", flush=True)

    def score(self) -> float:
        if self.text == TARGET:
            return 1.0
        total = 0.0
        for typed, want in zip(self.text, TARGET, strict=False):
            if typed.lower() != want.lower():
                break
            total += 0.05 + (0.2 if want.isupper() and typed == want else 0.0)
        return round(total, 6)


_TYPIST = Typist()


def on_reset(env) -> None:
    global _TYPIST
    _TYPIST = Typist()


def on_step(env, states) -> None:
    q = yampatch.joint_positions(env, "keyboard")
    _TYPIST.update(q)


yampatch.install(on_reset=on_reset, on_step=on_step)


def success(objects):
    """The text reads exactly "RoboCurve" and no key is down (held for 1 s ends the episode)."""
    return _TYPIST.text == TARGET and not _TYPIST.down


def score(objects):
    """1.0 for exactly "RoboCurve". Otherwise, over the leading characters that spell the word (in either case):
    0.05 each, plus 0.2 for each of the capitals R and C typed as a capital (at most 0.85, for "RoboCurve" with
    more after it)."""
    return _TYPIST.score()


# -- scripted demo: the right arm holds Right Shift for the capitals, the left arm types them -------------------------

CAP_TOP = TOP + DECK_H + GAP + CAP[2]
HOVER_Z = CAP_TOP + 0.025 + PAD_TIP_PAST_TCP
PRESS_Z = CAP_TOP - GAP + PAD_TIP_PAST_TCP  # 0.5 mm past full travel: the joint limit stops the key
_LEFT_HAND = set("qwertasdfgzxcvb")
PARK = {"left": (0.20, 0.20, HOVER_Z + 0.03), "right": (0.20, -0.20, HOVER_Z + 0.03)}  # out of the other's way
# Fingers closing along x: the closed pads are 8 mm along x and 15.5 mm across, inside a 23 mm cap.
YAW = {"left": -math.pi / 2, "right": math.pi / 2}


def _at(name, z):
    return (*HOME[name], z)


def _tap(name, arm, wait=0.5):
    y = YAW[arm]
    return [
        Waypoint(1.0, _at(name, HOVER_Z), FINGER_CLOSED, f"above {name}", y),
        Waypoint(wait, _at(name, HOVER_Z), FINGER_CLOSED, "settle", y),
        Waypoint(0.6, _at(name, PRESS_Z), FINGER_CLOSED, f"press {name}", y),
        Waypoint(0.5, _at(name, PRESS_Z), FINGER_CLOSED, "hold", y),
        Waypoint(0.5, _at(name, HOVER_Z), FINGER_CLOSED, "release", y),
    ]  # 2.6 s + wait


def demo():
    plan = {"left": [], "right": []}
    for ch in TARGET:
        name = ch.lower()
        if ch.isupper():  # a left-hand capital: Right Shift held through the left arm's tap (4.2 s)
            plan["right"] += [
                Waypoint(1.0, _at("rshift", HOVER_Z), FINGER_CLOSED, "above rshift", YAW["right"]),
                Waypoint(0.6, _at("rshift", PRESS_Z), FINGER_CLOSED, "press rshift", YAW["right"]),
                Waypoint(1.8, _at("rshift", PRESS_Z), FINGER_CLOSED, "hold rshift", YAW["right"]),
                Waypoint(0.6, _at("rshift", HOVER_Z), FINGER_CLOSED, "release rshift", YAW["right"]),
                Waypoint(0.2, _at("rshift", HOVER_Z), FINGER_CLOSED, "wait", YAW["right"]),
            ]
            plan["left"] += _tap(name, "left", wait=1.0) + [Waypoint(0.6, _at(name, HOVER_Z), FINGER_CLOSED, "wait", YAW["left"])]
            continue
        arm = "left" if name in _LEFT_HAND else "right"
        other = "right" if arm == "left" else "left"
        plan[arm] += _tap(name, arm)
        plan[other].append(Waypoint(3.1, PARK[other], FINGER_CLOSED, "park", YAW[other]))
    for arm in plan:
        plan[arm].append(Waypoint(1.0, PARK[arm], FINGER_CLOSED, "done", YAW[arm]))
    return plan
