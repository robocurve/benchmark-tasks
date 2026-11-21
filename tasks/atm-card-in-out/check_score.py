"""The scorer and the reader's state machine on hand-built states, with no simulator: python3 check_score.py.
Runs task.py's own `Trial` and `in_wallet`, read from its source, against stand-ins for the card's pose and the robot's contact force."""

import ast
import math
import os
import pathlib

import numpy as np

src = pathlib.Path(__file__).with_name("task.py").read_text()


def rot_y(deg):
    t = math.radians(deg)
    return np.array([[math.cos(t), 0, math.sin(t)], [0, 1, 0], [-math.sin(t), 0, math.cos(t)]])


def rot_x(deg):
    t = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)], [0, math.sin(t), math.cos(t)]])


class Card:
    def __init__(self, pos, R):
        self.pos, self.R, self.lin_vel = np.asarray(pos, float), np.asarray(R, float), np.zeros(3)

    def rotation(self):
        return self.R

    def to_world(self, pts):
        return np.atleast_2d(pts) @ self.R.T + self.pos


class Keys:
    def __init__(self, down=()):
        self.down = set(down)

    def joint(self, link):
        return 0.003 if link in self.down else 0.0


class W:
    card, touch, keys = None, 0.0, Keys()


class FakeM:
    @staticmethod
    def state(env, name):
        return W.keys if name == "keys" else W.card

    on_step = on_reset = staticmethod(lambda fn: fn)


def drive(env, x):  # the reader's rollers: card on the path, chip end in, chip up
    W.card = Card((x, ns["YS"], ns["ZS"]), np.eye(3))


ns = {"np": np, "math": math, "os": os, "m": FakeM, "d": type("D", (), {"TABLE_TOP": 0.7475, "CONTROL_HZ": 20}), "pathlib": pathlib}
for node in ast.parse(src).body:
    seg = ast.get_source_segment(src, node)
    if isinstance(node, ast.Assign):
        names = {n.id for t in node.targets for n in ast.walk(t) if isinstance(n, ast.Name)}
        if names & {"OBJECTS", "TRIAL", "LOOKS", "DOWN", "INSERT", "TARGET_LINK"}:
            continue
        exec(seg, ns)
    if isinstance(node, ast.FunctionDef) and node.name in ("_e", "in_wallet", "_key_name"):
        exec("from __future__ import annotations\n" + seg, ns)
    if isinstance(node, ast.ClassDef) and node.name == "Trial":
        exec("from __future__ import annotations\n" + seg, ns)
    if isinstance(node, ast.Assign) and "KEY_LINKS" in {n.id for t in node.targets for n in ast.walk(t) if isinstance(n, ast.Name)}:
        ns["TARGET_LINK"] = next(n for n in ns["KEY_LINKS"] if ns["_key_name"](n) == ns["TARGET"])
ns["robot_on_card"] = lambda env: W.touch
ns["_drive"] = drive

XB, YS, ZS, CL, ZSH = ns["XB"], ns["YS"], ns["ZS"], ns["CL"], ns["ZSH"]
W_X, W_Y, W_FLOOR, HZ, TOP = ns["W_X"], ns["W_Y"], ns["W_FLOOR"], ns["HZ"], ns["TOP"]
UPRIGHT = rot_y(90)  # as it starts: chip end down, chip face towards the ATM
SEATED = (W_X, W_Y, W_FLOOR + CL / 2 + 0.0005)
RET, STMT = ns["TARGET_LINK"], "kr2"  # RETURN CARD, and the STATEMENT key above it


def run(seq):
    """seq: list of (steps, card pose or None to leave it, touch N[, keys down]). Returns the trial."""
    t = ns["Trial"]()
    for item in seq:
        n, pose, touch = item[:3]
        keys = item[3] if len(item) > 3 else ()
        for _ in range(n):
            if pose is not None and t.reader not in ("drawing", "inside", "ejecting", "capturing"):
                W.card = Card(*pose)
            W.touch, W.keys = touch, Keys(keys)
            t.step(None)
    return t


wallet = (SEATED, UPRIGHT)
lifted = ((W_X, W_Y, SEATED[2] + 0.07), UPRIGHT)
above = lambda h: ((W_X, W_Y, SEATED[2] + h), UPRIGHT)  # noqa: E731
at_throat = lambda depth, R=np.eye(3): ((XB + depth - CL / 2, YS, ZS), R)  # noqa: E731
out_held = ((XB - 0.06 - CL / 2, YS, ZS), np.eye(3))
IN = [(5, wallet, 0.0), (20, lifted, 3.0), (2, at_throat(0.028), 0.0), (int(2 * HZ), None, 0.0)]  # taken in, inside
KEY = IN + [(3, None, 0.0, (RET,)), (int(2 * HZ), None, 0.0)]  # RETURN CARD pressed, ejected
OUT = KEY + [(30, out_held, 3.0)]  # drawn out, held

cases = {
    "nothing": ([(100, wallet, 0.0)], 0.0),
    "lift and put back": ([(5, wallet, 0.0), (20, lifted, 3.0), (60, wallet, 0.0)], 0.25),
    "pushed 12 mm, not taken": ([(5, wallet, 0.0), (20, lifted, 3.0), (100, at_throat(0.012), 0.0)], 0.25),
    "chip down, not taken": ([(5, wallet, 0.0), (20, lifted, 3.0), (100, at_throat(0.028, rot_x(180)), 0.0)], 0.25),
    "stripe end first, not taken": ([(5, wallet, 0.0), (20, lifted, 3.0), (100, at_throat(0.028, rot_y(180) @ rot_x(180)), 0.0)], 0.25),
    "held at 28 mm, never let go": ([(5, wallet, 0.0), (20, lifted, 3.0), (200, at_throat(0.028), 3.0)], 0.25),
    "taken in, no key": (IN + [(int(30 * HZ), None, 0.0)], 0.5),
    "RETURN CARD held down from before the card is in": ([(5, wallet, 0.0, (RET,)), (20, lifted, 3.0, (RET,)), (2, at_throat(0.028), 0.0, (RET,)), (int(30 * HZ), None, 0.0, (RET,))], 0.5),
    "wrong key (STATEMENT)": (IN + [(3, None, 0.0, (STMT,)), (int(10 * HZ), None, 0.0)], 0.5),
    "wrong key, then RETURN CARD": (IN + [(3, None, 0.0, (STMT,)), (5, None, 0.0), (3, None, 0.0, (RET,)), (int(10 * HZ), None, 0.0)], 0.5),
    "every key at once": (IN + [(3, None, 0.0, tuple(ns["KEY_LINKS"])), (int(10 * HZ), None, 0.0)], 0.5),
    "RETURN CARD, card left: captured": (KEY + [(int(25 * HZ), None, 0.0)], 0.75),
    "RETURN CARD, drawn out, held": (OUT, 0.75),
    "drawn out, dropped on the bench": (OUT + [(20, ((0.20, YS, TOP + 0.0004), np.eye(3)), 0.0)], 0.0),
    "back in the wallet, still held": (OUT + [(20, above(0.03), 3.0), (60, wallet, 3.0)], 0.75),
    "dropped into the wallet from 40 mm": (OUT + [(20, above(0.04), 3.0), (60, wallet, 0.0)], 0.75),
    "back in the wallet 1.5 s": (OUT + [(20, above(0.03), 3.0), (2, wallet, 3.0), (30, wallet, 0.0)], 0.75),
    "full": (OUT + [(20, above(0.03), 3.0), (2, wallet, 3.0), (45, wallet, 0.0)], 1.0),
    "full, then lifted out again": (OUT + [(20, above(0.03), 3.0), (2, wallet, 3.0), (45, wallet, 0.0), (10, lifted, 3.0)], 0.75),
    "in the wallet, no ATM": ([(5, wallet, 0.0), (20, lifted, 3.0), (60, wallet, 0.0)], 0.25),
}

bad = 0
for name, (seq, want) in cases.items():
    import contextlib
    import io

    with contextlib.redirect_stdout(io.StringIO()):
        t = run(seq)
    ok = abs(t.score - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {name}: score {t.score} (want {want}), reader {t.reader}, void {t.why or '-'}")
print("all pass" if not bad else f"{bad} failed")
raise SystemExit(1 if bad else 0)
