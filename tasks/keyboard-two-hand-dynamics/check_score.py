"""The scorer on hand-built key logs, with no simulator: the ladder, wrong notes, a key left down, the hold.

    python check_score.py

Exits non-zero if any case gives another value than the scoring text.
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys
import types
from dataclasses import dataclass

import numpy as np

sys.modules["yam_arena"] = types.ModuleType("yam_arena")  # the harness is not needed: only its names, so that mechkit's meshes can be built
sys.modules["yam_arena"].__path__ = []
tf = sys.modules["yam_arena.task_file"] = types.ModuleType("yam_arena.task_file")


@dataclass(frozen=True)
class Box:  # the harness's shape, its fields only
    name: str
    pos: tuple
    color: tuple = (0.5, 0.5, 0.5)
    mass: float | None = None
    yaw: float = 0.0
    size: tuple = (0.04, 0.04, 0.04)


@dataclass(frozen=True)
class ObjectState:
    pos: object
    quat_xyzw: object
    lin_vel: object
    ang_vel: object
    low: object
    high: object


tf.TABLE_TOP, tf.Box, tf.ObjectState = {"station": 0.0}, Box, ObjectState
sys.path.insert(0, str(pathlib.Path(__file__).parent))
spec = importlib.util.spec_from_file_location("task", pathlib.Path(__file__).with_name("task.py"))
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)

bad = 0


def check(label, got, want):
    global bad
    ok = abs(got - want) < 1e-6
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {label}: {got} (want {want})")


B = 0.75  # 80 per minute


def played(beat=B, mel_v=90, bass_v=40, n_mel=8, bass=True, c4_at=0.0, g3_at=0.0, c4_up=3.05, gaps=None, extra=(), held_e=False):
    """A key log: [name, onset, velocity, release]."""
    log, t0 = [], 1.0
    on = [t0 + i * beat for i in range(8)] if gaps is None else list(np.cumsum([t0] + list(gaps)))
    for i, n in enumerate(t.MELODY[:n_mel]):
        if held_e and i == 1:
            continue
        up = on[i] + (1.2 * beat if held_e and i == 0 else 0.4 * beat)
        log.append([n, on[i], mel_v, up])
    if bass:
        log.append(["C4", on[0] + c4_at, bass_v, on[0] + c4_up * beat])
        log.append(["G3", on[4] + g3_at, bass_v, on[4] + 3.3 * beat])
    log += [list(e) for e in extra]
    return sorted(log, key=lambda r: r[1])


def rung(log):
    return t.judge(log, 20.0)["rung"]


check("all of it", rung(played()), 1.0)
check("three melody notes", rung(played(n_mel=3, bass=False)), 3 * 0.25 / 8)
check("the melody alone", rung(played(bass=False)), 0.25)
check("G3 100 ms after melody note 5", rung(played(g3_at=0.10)), 0.375)
check("C4 let up after 2.5 beats", rung(played(c4_up=2.5)), 0.375)
check("one press speed for every note", rung(played(bass_v=90)), 0.5)
check("the bass 20 softer only", rung(played(mel_v=60)), 0.5)
check("one gap 15 percent long", rung(played(gaps=[B] * 3 + [1.15 * B] + [B] * 3, c4_up=3.3)), 0.75)
check("120 per minute", rung(played(beat=0.5)), 0.75)
legato = [list(r) for r in played()]
for r in legato:
    if r[0] in t.MELODY:
        r[3] = r[1] + 1.02 * B  # each melody key let up just after the next sounds
check("a legato melody (each key let up after the next sounds)", rung(sorted(legato, key=lambda r: r[1])), 0.75)
check("a legato bass handover (C4 let up 0.1 beat after G3 sounds)", rung(played(c4_up=4.1)), 0.375)
j = t.judge(played(held_e=True), 20.0)
check("the repeated E held through: at least three wrong notes", min(j["wrong"], 3), 3)
check("an A5 among the notes: one wrong", t.judge(played(extra=[["A5", 3.0, 80, 3.2]]), 20.0)["wrong"], 1)
check("a black key: one wrong", t.judge(played(extra=[["Fs5", 3.0, 80, 3.2]]), 20.0)["wrong"], 1)


def trial(log, down=(), tips_z=None, steps=25):
    """The trial's score after `steps` control steps on a finished log, every key up but `down`, fingertips at `tips_z` above the table."""
    t.KEYBOARD.log = log
    t.KEYBOARD.t = 20.0
    t.KEYBOARD.state = {n: (2 if n in down else 0) for n in t.KEY_Y}
    z = t.Z_WHITE + 0.10 if tips_z is None else tips_z
    t.m.fingertips = lambda env: np.tile([0.3, 0.0, t.TOP + z], (24, 1))
    tr = t.TRIAL
    tr.reset()
    for _ in range(steps):
        tr.step(None)
    return tr.score


check("all of it, held 1 s", trial(played()), 1.0)
check("all of it, 19 steps: not yet held", trial(played(), steps=19), 0.95)
check("all of it, a hand 4 cm over the keys", trial(played(), tips_z=t.Z_WHITE + 0.04), 0.75)
check("one wrong note", trial(played(extra=[["A5", 3.0, 80, 3.2]])), 0.9)
check("three wrong notes", trial(played(held_e=True)), 0.0)
check("G3 still down at the end", trial(played(), down=("G3",)), 0.0)
check("the melody alone, one wrong", trial(played(bass=False, extra=[["A5", 3.0, 80, 3.2]])), 0.15)

print("all cases as the scoring text" if not bad else f"{bad} cases differ")
sys.exit(1 if bad else 0)
