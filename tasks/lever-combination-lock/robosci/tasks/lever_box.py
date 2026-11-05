"""Task: open a locked box whose lid only slides while the right lever is held down.

Ability: causal discovery (bimanual). Hidden variable: which of three levers (A, B, C)
releases the lid lock. The lid slides sideways off the box; a lock (a joint equality)
holds it shut unless the key lever is pressed. One arm must hold a lever down while the
other pulls the lid, so each attempt is a one-lever intervention. Inside is a block to
put on the red mat.

An attempt is the knob held for at least 0.5 s while the lid starts closed; it records
which levers were held and whether the lid moved. The scorer keeps the levers
consistent with every attempt.

Hard: a combination lock. Pressing two different levers in the right order (6 possible
ordered pairs) unlocks the lid for 12 s. A green LED lights while the first lever of the
combination has just been pressed (the lock is waiting for the second). A pressed lever
counts on its own; holding one lever while pulling (the easy method) only opens the lid
when the levers it happens to press in sequence form the combination. The scorer replays
each of the 6 combinations against every recorded press (and the LED seen after it) and
every pull, keeping those consistent with all of them.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import mujoco
import numpy as np

from robosci import props
from robosci.motion import SAFE_Z, Motion
from robosci.skills import move_body, noop
from robosci.specs import TaskSpec

BOX_XY = (0.32, -0.33)
BOX_IN, WALL, BOX_H = 0.055, 0.006, 0.07
LID_HALF, LID_T = 0.068, 0.004
LID_Z = BOX_H + LID_T + 0.001
KNOB = 0.012
KNOB_Z = LID_Z + LID_T + KNOB
SLIDE = 0.13  # lid travel along -y
OPEN_AT = 0.03
LEVERS = ("A", "B", "C")
LEVER_SPECS = tuple(props.ButtonSpec(f"lever_{n}", (x, -0.13), rgba=(0.85, 0.7, 0.15, 1))
                    for n, x in zip(LEVERS, (0.20, 0.28, 0.36)))
BLOCK_HALF = 0.02
LED_XY = (0.44, -0.13)
WINDOW = 12.0  # s the lock stays released after the combination
COMBOS = tuple((a, b) for a in LEVERS for b in LEVERS if a != b)
MAT_XY, MAT_HALF = (0.30, 0.06), (0.06, 0.06)

INSTRUCTION = (
    "The box is locked: its lid only slides open while one of the three levers A, B, C is held "
    "down. Find out which lever unlocks it, open the lid, and put the block from inside the box "
    "on the red mat."
)


@dataclass
class Attempt:
    t: float
    levers: tuple[str, ...]
    opened: bool


@dataclass
class LeverBox:
    difficulty: str = "easy"
    key: str = "lever_box"
    instruction: str = INSTRUCTION
    attempts: list[Attempt] = field(default_factory=list)

    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        if self.difficulty == "easy":
            self.key_lever = str(rng.choice(LEVERS))
        else:
            self.combo = COMBOS[int(rng.integers(len(COMBOS)))]
            self.key_lever = ">".join(self.combo)
            props.lamp(spec, "led", LED_XY, radius=0.012, rgba_on=(0.2, 1.0, 0.3, 1), post=0.03)
        w = spec.worldbody
        wood = [0.55, 0.38, 0.22, 1]
        box = w.add_body(name="box", pos=[BOX_XY[0], BOX_XY[1], 0])
        o = BOX_IN + WALL
        box.add_geom(type=props.G.mjGEOM_BOX, size=[o, o, 0.002], pos=[0, 0, 0.002], rgba=wood)
        for sx, sy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            size = [WALL / 2, o, BOX_H / 2] if sx else [o, WALL / 2, BOX_H / 2]
            box.add_geom(type=props.G.mjGEOM_BOX, size=size, rgba=wood,
                         pos=[sx * (BOX_IN + WALL / 2), sy * (BOX_IN + WALL / 2), BOX_H / 2])
        lid = w.add_body(name="lid", pos=[BOX_XY[0], BOX_XY[1], LID_Z])
        lid.add_joint(name="lid_slide", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 1, 0],
                      range=[-SLIDE, 0], limited=mujoco.mjtLimited.mjLIMITED_TRUE,
                      frictionloss=1.5, damping=[5.0, 0, 0], armature=0.05)
        lid.add_geom(name="lid", type=props.G.mjGEOM_BOX, size=[LID_HALF, LID_HALF, LID_T], mass=0.1,
                     rgba=[0.45, 0.30, 0.17, 1])
        lid.add_geom(name="knob", type=props.G.mjGEOM_BOX, size=[KNOB] * 3, pos=[0, 0, LID_T + KNOB],
                     mass=0.01, rgba=[0.15, 0.15, 0.17, 1], friction=[1.5, 0.02, 0.002], condim=4)
        spec.add_equality(name="lock", type=mujoco.mjtEq.mjEQ_JOINT, name1="lid_slide",
                          data=[0.0] * 11)
        for b, n in zip(LEVER_SPECS, LEVERS):
            props.push_button(spec, b)
            props.text_plate(spec, f"lbl_{n}", (b.pos[0], b.pos[1] + 0.05), n, half=(0.016, 0.016))
        props.flat(spec, "mat", MAT_XY, MAT_HALF, props.RED)
        props.block(spec, "prize", (BOX_XY[0], BOX_XY[1], 0.004 + BLOCK_HALF + 0.001), BLOCK_HALF, 0.05,
                    label="?", bg=(60, 120, 200), fg=(255, 255, 255))

    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._m = model
        self.levers = {n: props.Button(b) for n, b in zip(LEVERS, LEVER_SPECS)}
        for b in self.levers.values():
            b.bind(model)
        self.touch = props.TouchTracker(("lid", "prize") + tuple(b.name for b in LEVER_SPECS))
        self.touch.bind(model, data)
        self._tips = props.gripper_geoms(model)
        self._knob = model.geom("knob").id
        self._prize = model.geom("prize").id
        self._q = model.joint("lid_slide").qposadr[0]
        self._lock = model.equality("lock").id
        self.attempts = []
        self._grab: tuple[float, set[str], bool] | None = None
        if self.hard:
            self.led = props.Lamp("led", (0.2, 1.0, 0.3, 1))
            self.led.bind(model)
            self.progress = 0
            self.unlock_t = -1e9
            self.presses: list[tuple[float, str, bool]] = []  # (t, lever, LED on after)
            self._since_attempt: list[str] = []

    @property
    def hard(self) -> bool:
        return self.difficulty != "easy"

    @staticmethod
    def step_lock(combo: tuple[str, str], progress: int, lever: str) -> tuple[int, bool]:
        """Combination state machine: returns (new progress, unlocked by this press)."""
        if progress == 1 and lever == combo[1]:
            return 0, True
        return int(lever == combo[0]), False

    def unlocked(self, t: float) -> bool:
        return t - self.unlock_t <= WINDOW

    def lid_q(self, data: mujoco.MjData) -> float:
        return -float(data.qpos[self._q])

    def lid_open(self, data: mujoco.MjData) -> bool:
        return self.lid_q(data) > OPEN_AT

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        t = float(data.time)
        self.touch.update(data)
        edges = [n for n, b in self.levers.items() if b.update(data)]
        pressed = {n for n, b in self.levers.items() if b.pressed}
        closed = self.lid_q(data) < 0.004
        if self.hard:
            for n in edges:
                self.progress, opened = self.step_lock(self.combo, self.progress, n)
                if opened:
                    self.unlock_t = t
                self.led.set(model, self.progress == 1)
                self.presses.append((t, n, self.progress == 1))
                self._since_attempt.append(n)
            data.eq_active[self._lock] = int(closed and not self.unlocked(t))
        else:
            data.eq_active[self._lock] = int(closed and self.key_lever not in pressed)
        on_knob = self._knob in props.held_geoms(model, data, self._tips)
        if on_knob:
            if self._grab is None:
                self._grab = (t, set(pressed), closed)
            else:
                self._grab[1].update(pressed)
        elif self._grab is not None:
            t0, levers, was_closed = self._grab
            self._grab = None
            if was_closed and t - t0 >= 0.5:
                if self.hard:
                    self.attempts.append(Attempt(t, tuple(self._since_attempt), self.lid_open(data)))
                    self.attempt_t0 = getattr(self, "attempt_t0", []) + [t0]
                    self._since_attempt = []
                else:
                    self.attempts.append(Attempt(t, tuple(sorted(levers)), self.lid_open(data)))

    def _consistent(self, combo: tuple[str, str]) -> bool:
        prog, unlocks = 0, []
        for tp, n, led in self.presses:
            prog, op = self.step_lock(combo, prog, n)
            if op:
                unlocks.append(tp)
            if (prog == 1) != led:
                return False
        for a, t0 in zip(self.attempts, getattr(self, "attempt_t0", [])):
            pred = any(u <= a.t - 0.3 and u + WINDOW >= t0 + 0.3 for u in unlocks)
            if pred != a.opened:
                return False
        return True

    def hypotheses(self) -> list[str]:
        if self.hard:
            return [">".join(c) for c in COMBOS if self._consistent(c)]
        return [k for k in LEVERS if all((k in a.levers) == a.opened for a in self.attempts)]

    def prize_on_mat(self, data: mujoco.MjData) -> bool:
        held = props.held_geoms(self._m, data, self._tips)
        p = data.xpos[self._m.body("prize").id]
        return self._prize not in held and props.in_box(p, MAT_XY, MAT_HALF, z_max=BLOCK_HALF + 0.01)

    def hud(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
        sc = self.score(model, data)
        held = [n for n, b in self.levers.items() if b.pressed]
        if self.hard:
            held = f"{held} LED {'on' if self.led.on else 'off'}"
        lines = [f"t = {data.time:5.1f} s   levers held {held or '-'}   lid {'OPEN' if self.lid_open(data) else 'shut'}"
                 f" ({100 * self.lid_q(data):.0f} cm)   lock {'on' if data.eq_active[self._lock] else 'off'}",
                 f"rubric stage {sc['stage']}/4   attempts {sc['attempts']}   possible key levers "
                 f"{' '.join(self.hypotheses())}   block on mat {'yes' if sc['success'] else 'no'}"]
        for a in self.attempts[-3:]:
            lines.append(f"  pull with {'+'.join(a.levers) or 'no lever'}: {'opened' if a.opened else 'locked'}")
        return lines

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        success = self.prize_on_mat(data)
        hs = self.hypotheses()
        stage = 0
        if self.touch.touched:
            stage = 1
        if any(a.levers for a in self.attempts):
            stage = 2
        if hs == [self.key_lever]:
            stage = 3
            if success:
                stage = 4
        return {
            "stage": stage,
            "success": float(success),
            "science_success": float(stage >= 3),
            "attempts": len(self.attempts),
            "levers_left": len(hs),
            "procedure_violations": 0,
            "key_lever": self.key_lever,
            "lid_open": float(self.lid_open(data)),
            "log": [a.__dict__ for a in self.attempts],
        }


# ---------- reference policies ----------
def _pull_lid(env, mo: Motion, side: str = "right"):  # noqa: ANN001, ANN202
    """Grasp the knob from above, pull it straight along -y, let go, lift away."""
    knob = np.array([BOX_XY[0], BOX_XY[1], KNOB_Z])
    yield from mo.gripper(side, 0.025, 0.3)
    p = mo.plan_vertical(side, knob, yaws=(0.0, np.pi))
    yield from mo.goto_above(side, p[:2])
    top = np.array([p[0], p[1], SAFE_Z])
    yield from mo.line(side, top, p, 1.2)
    yield from mo.gripper(side, 0.0, 0.6)
    yield from mo.hold(0.3)
    end = p - np.array([0, SLIDE - 0.01, 0])
    yield from mo.line(side, p, end, 2.0)
    yield from mo.gripper(side, 0.025, 0.4)
    yield from mo.line(side, end, np.array([end[0], end[1], SAFE_Z]), 1.0)
    yield from mo.home(side)


def _hold_and_pull(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """One lever at a time: the left arm holds it down while the right arm pulls the lid.
    Stop at the lever that lets the lid slide, then move the block to the mat."""
    t: LeverBox = env.task
    mo = Motion(env)
    for n, b in zip(LEVERS, LEVER_SPECS):
        yield from mo.press_down("left", t.levers[n].top(), b.travel + 0.004)
        yield from _pull_lid(env, mo, "right")
        yield from mo.press_up("left")
        if t.lid_open(env.data):
            break
    yield from mo.home("left")
    if t.lid_open(env.data):
        yield from move_body(env, mo, "prize", "left", MAT_XY, 0.0, half=BLOCK_HALF, grip=0.0, open_to=0.035)
    yield from mo.hold(1.0)


def _tap(mo: Motion, t: LeverBox, n: str):  # noqa: ANN202
    b = LEVER_SPECS[LEVERS.index(n)]
    yield from mo.press("left", t.levers[n].top(), b.travel + 0.004, hold=0.3)
    yield from mo.hold(0.6)


def _combo_search(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Hard: tap levers until the LED shows a correct first lever; then for each candidate
    second lever, put the right arm over the knob, tap first + candidate, and pull."""
    t: LeverBox = env.task
    mo = Motion(env)
    first = None
    for n in LEVERS[:2]:
        yield from _tap(mo, t, n)
        if t.led.on:
            first = n
            break
    first = first or LEVERS[2]
    yield from mo.home("left")
    for cand in [n for n in LEVERS if n != first]:
        if t.lid_open(env.data):
            break
        knob = np.array([BOX_XY[0], BOX_XY[1], KNOB_Z])
        yield from mo.gripper("right", 0.025, 0.3)
        p = mo.plan_vertical("right", knob, yaws=(0.0, np.pi))
        yield from mo.goto_above("right", p[:2])
        if not t.led.on:
            yield from _tap(mo, t, first)
        yield from _tap(mo, t, cand)
        yield from mo.line("left", mo._press["left"][1] + np.array([0, 0, SAFE_Z - mo._press["left"][1][2]]),
                           np.array([0.25, -0.05, SAFE_Z]), 0.6)
        top = np.array([p[0], p[1], SAFE_Z])
        yield from mo.line("right", top, p, 1.0)
        yield from mo.gripper("right", 0.0, 0.5)
        yield from mo.hold(0.3)
        end = p - np.array([0, SLIDE - 0.01, 0])
        yield from mo.line("right", p, end, 2.0)
        yield from mo.gripper("right", 0.025, 0.4)
        yield from mo.line("right", end, np.array([end[0], end[1], SAFE_Z]), 1.0)
        yield from mo.home("right")
    yield from mo.home("left")
    if t.lid_open(env.data):
        yield from move_body(env, mo, "prize", "left", MAT_XY, 0.0, half=BLOCK_HALF, grip=0.0, open_to=0.035)
    yield from mo.hold(1.0)


def oracle(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Easy: hold one lever at a time while pulling. Hard: find the combination."""
    pol = _hold_and_pull if env.task.difficulty == "easy" else _combo_search
    yield from pol(env, rng)


def guess(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Blind baseline: pull the lid without touching any lever."""
    mo = Motion(env)
    yield from _pull_lid(env, mo, "right")
    yield from mo.hold(1.0)


TASK = LeverBox
POLICIES = {"oracle": oracle, "guess": guess, "noop": noop, "naive": _hold_and_pull}
SPEC = TaskSpec(
    key="lever_box",
    title="Find the lever that unlocks the box",
    ability="Causal discovery",
    instruction=INSTRUCTION,
    objects=(
        "wooden box 12 x 12 x 7 cm with a sliding lid and a black knob on top",
        "three yellow spring levers (push buttons) labelled A, B, C",
        "hard only: a small green LED beside lever C",
        "blue block (4 cm) inside the box",
        "red mat, 12 x 12 cm",
    ),
    setup=(
        "Box 32 cm in front of the arms, slightly toward the right arm; lid closed, sliding toward "
        "the right arm.",
        "Levers in a row 13 cm toward the left arm from the box line, at 20, 28 and 36 cm in front.",
        "Block in the box; red mat on the operator's right, 30 cm in front of the left arm.",
    ),
    hidden="Which lever releases the lock (uniform over A, B, C). The lock re-engages when the lid is shut.",
    instruction_hard=(
        "The box is locked and the levers A, B, C work its lock. Open the lid and put the block from "
        "inside the box on the red mat."
    ),
    hidden_hard=(
        "A combination: two different levers pressed in order (6 ordered pairs, uniform) release the lock "
        "for 12 s. A green LED lights while the first lever of the combination has just been pressed. "
        "Holding one lever down while pulling only works by luck (when two consecutive single-lever tries "
        "happen to form the combination)."
    ),
    rubric=(
        "No purposeful contact with the box, the levers or the block",
        "Touches the lid, a lever or the block",
        "At least one pull on the lid while holding a lever",
        "The pulls (and, in hard, the LED after each press) leave only the true key lever (hard: the true "
        "ordered pair) possible",
        "Stage 3 and the block resting on the mat",
    ),
    metrics=(
        "stage (0-4)",
        "success: block on the mat",
        "science_success: stage >= 3",
        "pull attempts used (1-3 for single-lever tests)",
    ),
    report_fields=("attempts", "levers_left", "key_lever", "lid_open"),
    max_seconds=150.0,
    limitations="The lock is a joint constraint switched in code from the lever states (hard: a "
                "combination state machine).",
)
