"""Task: find which of two spinners is biased, by spinning each enough times.

Ability: noise and replication. Hidden variable: which spinner (A or B) favours red, and
by how much (P(red)=0.8 in easy); the other is fair. Each spinner is a free block
with a red half and a blue half and a pointer; pressing that spinner's button spins the
pointer (animated in code) and it stops in a sector drawn from the hidden bias.

The scorer keeps every observed spin and computes the posterior that each spinner is the
biased one (prior 1/2, known bias level). Science is proven when the posterior for the
true biased spinner reaches 0.9.

Hard: red sectors cover 1/3 of one face and 2/3 of the other; fair means red as often as
the red area, rigged means 0.25 more. Counting raw reds picks the wrong spinner half the time.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import zip_longest
from typing import Any

import mujoco
import numpy as np

from robosci import props
from robosci.motion import Motion
from robosci.skills import move_body, noop
from robosci.specs import TaskSpec

NAMES = "AB"
HALF = (0.035, 0.035, 0.015)
SPIN_XY = {"A": (0.32, -0.19), "B": (0.32, -0.42)}
BUTTONS = {k: props.ButtonSpec(f"btn_{k}", (0.21, SPIN_XY[k][1]), rgba=(0.95, 0.95, 0.95, 1)) for k in NAMES}
MAT_POS, MAT_HALF = (0.27, 0.07), (0.075, 0.075)
P_RED = {"easy": 0.8}
AREAS = (1 / 3, 2 / 3)  # hard: red fraction of each spinner's face (one each, random)
DELTA = 0.25  # hard: the rigged spinner lands on red this much more often than its red area
SPIN_TIME = 2.0
POST_GOAL = 0.9

INSTRUCTION = (
    "One of the two spinners, A or B, is biased toward red; the other is fair. Press a "
    "spinner's button to spin it. Spin them as often as you need, then put the biased "
    "spinner on the red mat."
)


def easy_model(p: float) -> dict[str, dict[str, float]]:
    """Hypothesis k -> P(red) of each spinner: biased k has p, the other is a fair 50/50."""
    return {k: {j: (p if j == k else 0.5) for j in NAMES} for k in NAMES}


def posterior(spins: list[tuple[str, bool]], model: dict[str, dict[str, float]]) -> dict[str, float]:
    """P(spinner k is the biased one | spins), uniform prior, P(red) per hypothesis from ``model``."""
    ll = {}
    for k in NAMES:
        s = 0.0
        for who, red in spins:
            q = model[k][who]
            s += np.log(q if red else 1 - q)
        ll[k] = s
    m = max(ll.values())
    w = {k: np.exp(v - m) for k, v in ll.items()}
    z = sum(w.values())
    return {k: float(w[k] / z) for k in NAMES}


@dataclass
class BiasedSpinner:
    difficulty: str = "easy"
    key: str = "biased_spinner"
    instruction: str = INSTRUCTION
    spins: list[tuple[str, bool]] = field(default_factory=list)

    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        self.biased = str(rng.choice(list(NAMES)))
        self.p = P_RED["easy"]
        self._rng = np.random.default_rng(int(rng.integers(1 << 31)))
        # Hard: unequal red areas (1/3 and 2/3 of the face). "Fair" means landing on red as often as
        # the red area; the rigged one lands on red DELTA more often. Drawn after the easy draws.
        self.area = {k: 0.5 for k in NAMES}
        if self.difficulty == "hard":
            order = rng.permutation(2)
            self.area = {k: AREAS[order[i]] for i, k in enumerate(NAMES)}
            self.model = {k: {j: self.area[j] + (DELTA if j == k else 0.0) for j in NAMES} for k in NAMES}
        else:
            self.model = easy_model(self.p)
        self.prob = self.model[self.biased]
        props.flat(spec, "mat", MAT_POS, MAT_HALF, props.RED)
        for k in NAMES:
            x, y = SPIN_XY[k]
            b = spec.worldbody.add_body(name=f"spinner_{k}", pos=[x, y, HALF[2] + 0.001])
            b.add_freejoint(name=f"spinner_{k}_free")
            b.add_geom(name=f"spinner_{k}", type=props.G.mjGEOM_BOX, size=list(HALF), mass=0.15,
                       rgba=[0.9, 0.9, 0.88, 1], friction=[1.0, 0.01, 0.001], condim=4)
            if self.difficulty == "easy":
                # Red half on +y (pointer angle in (0, pi)), blue half on -y; drawn on the top face.
                for nm, sgn, col in (("red", 1, props.RED), ("blue", -1, props.BLUE)):
                    b.add_geom(type=props.G.mjGEOM_BOX, size=[0.03, 0.015, 0.0008],
                               pos=[0, sgn * 0.0155, HALF[2] + 0.0008], rgba=list(col), contype=0, conaffinity=0)
            else:
                # Blue disc with a red sector from angle 0 to 2 pi * area (counter-clockwise from +x),
                # drawn as thin red spokes over the disc.
                b.add_geom(type=props.G.mjGEOM_CYLINDER, size=[0.032, 0.0006], pos=[0, 0, HALF[2] + 0.0006],
                           rgba=list(props.BLUE), contype=0, conaffinity=0)
                span = 2 * np.pi * self.area[k]
                n = int(np.ceil(span / np.deg2rad(4)))
                for i in range(n):
                    a = (i + 0.5) * span / n
                    b.add_geom(type=props.G.mjGEOM_BOX, size=[0.016, 0.0013 + 0.0011, 0.0004],
                               pos=[0.016 * np.cos(a), 0.016 * np.sin(a), HALF[2] + 0.0016],
                               quat=[np.cos(a / 2), 0, 0, np.sin(a / 2)], rgba=list(props.RED),
                               contype=0, conaffinity=0)
            ptr = b.add_body(name=f"pointer_{k}", pos=[0, 0, HALF[2] + 0.002])
            ptr.add_joint(name=f"pointer_{k}", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1],
                          damping=[0.5, 0, 0], armature=0.001)
            ptr.add_geom(type=props.G.mjGEOM_BOX, size=[0.026, 0.003, 0.002], pos=[0.012, 0, 0.002],
                         rgba=[0.05, 0.05, 0.05, 1], mass=0.001, contype=0, conaffinity=0)
            ptr.add_geom(type=props.G.mjGEOM_CYLINDER, size=[0.005, 0.003], pos=[0, 0, 0.002],
                         rgba=[0.9, 0.8, 0.2, 1], mass=0.001, contype=0, conaffinity=0)
            props.text_plate(spec, f"lbl_{k}", (x + 0.07, y), k, half=(0.02, 0.02))
            props.push_button(spec, BUTTONS[k])

    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._m = model
        self.buttons = {k: props.Button(BUTTONS[k]) for k in NAMES}
        for b in self.buttons.values():
            b.bind(model)
        self._pq = {k: model.joint(f"pointer_{k}").qposadr[0] for k in NAMES}
        self._pv = {k: model.joint(f"pointer_{k}").dofadr[0] for k in NAMES}
        self.touch = props.TouchTracker(tuple(f"spinner_{k}" for k in NAMES) + tuple(f"btn_{k}" for k in NAMES))
        self.touch.bind(model, data)
        self._tips = props.gripper_geoms(model)
        self._sgeom = {model.geom(f"spinner_{k}").id: k for k in NAMES}
        self.spins = []
        self._anim: dict[str, tuple[float, float, float, bool]] = {}  # start t, q0, q1, red
        for k in NAMES:  # pointers start pointing at the boundary (+x), neither colour
            data.qpos[self._pq[k]] = 0.0

    def pointer_angle(self, data: mujoco.MjData, k: str) -> float:
        """World-frame pointer angle relative to the spinner's own +x (what a camera sees)."""
        return float(np.mod(data.qpos[self._pq[k]], 2 * np.pi))

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        t = float(data.time)
        self.touch.update(data)
        for k in NAMES:
            if self.buttons[k].update(data) and k not in self._anim:
                red = bool(self._rng.random() < self.prob[k])
                if self.difficulty == "easy":
                    stop = self._rng.uniform(0.25, np.pi - 0.25) + (0 if red else np.pi)
                else:
                    span = 2 * np.pi * self.area[k]
                    stop = (self._rng.uniform(0.25, span - 0.25) if red
                            else self._rng.uniform(span + 0.25, 2 * np.pi - 0.25))
                q0 = float(data.qpos[self._pq[k]])
                turns = 2 * np.pi * int(self._rng.integers(3, 6))
                q1 = q0 + turns + np.mod(stop - q0, 2 * np.pi)
                self._anim[k] = (t, q0, q1, red)
            if k in self._anim:
                t0, q0, q1, red = self._anim[k]
                s = min(1.0, (t - t0) / SPIN_TIME)
                ease = 1 - (1 - s) ** 3  # fast start, slow stop
                data.qpos[self._pq[k]] = q0 + ease * (q1 - q0)
                data.qvel[self._pv[k]] = 0.0
                if s >= 1.0:
                    self.spins.append((k, red))
                    del self._anim[k]
            else:
                data.qvel[self._pv[k]] = 0.0

    def reads_red(self, data: mujoco.MjData, k: str) -> bool:
        """What a camera sees: is the pointer over the red sector?"""
        return self.pointer_angle(data, k) < 2 * np.pi * self.area[k]

    def spinning(self) -> bool:
        return bool(self._anim)

    def last_result(self, k: str) -> bool | None:
        r = [red for who, red in self.spins if who == k]
        return r[-1] if r else None

    def on_mat(self, data: mujoco.MjData) -> list[str]:
        held = {self._sgeom[g] for g in props.held_geoms(self._m, data, self._tips) if g in self._sgeom}
        return [k for k in NAMES if k not in held and props.in_box(
            data.xpos[self._m.body(f"spinner_{k}").id], MAT_POS, MAT_HALF, margin=0.015, z_max=HALF[2] + 0.01)]

    def hud(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
        sc = self.score(model, data)
        cnt = {k: (sum(r for w, r in self.spins if w == k), sum(w == k for w, _ in self.spins)) for k in NAMES}
        return [f"t = {data.time:5.1f} s   spins  A: {cnt['A'][0]}/{cnt['A'][1]} red   "
                f"B: {cnt['B'][0]}/{cnt['B'][1]} red",
                f"posterior biased  A {sc['posterior']['A']:.2f}   B {sc['posterior']['B']:.2f}   (goal 0.9)",
                f"rubric stage {sc['stage']}/4   on mat {sc['on_mat'] or '-'}"]

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        post = posterior(self.spins, self.model)
        mat = self.on_mat(data)
        success = mat == [self.biased]
        stage = 0
        if self.touch.touched or any(b.presses for b in self.buttons.values()):
            stage = 1
        if self.spins:
            stage = 2
        if post[self.biased] >= POST_GOAL:
            stage = 3
            if success:
                stage = 4
        return {
            "stage": stage,
            "success": float(success),
            "science_success": float(stage >= 3),
            "spins": len(self.spins),
            "posterior": post,
            "procedure_violations": 0,
            "biased": self.biased,
            "red_area": self.area,
            "on_mat": mat,
            "log": [{"spinner": w, "red": r} for w, r in self.spins],
        }


# ---------- reference policies ----------
def _both(a, b):  # noqa: ANN001, ANN202
    """Run two single-arm generators on one shared Motion in lockstep (each step's
    command already carries both arms' latest targets)."""
    last = None
    for x, y in zip_longest(a, b):
        last = y if y is not None else x
        yield last


def _taps(mo: Motion, side: str, top: np.ndarray, depth: float):  # noqa: ANN202
    """Tap a button from a hover point 2 cm above it (no trip back to transit height)."""
    hover = top + np.array([0, 0, 0.02])
    low = top - np.array([0, 0, depth])
    yield from mo.line(side, hover, low, 0.5)
    yield from mo.hold(0.3)
    yield from mo.line(side, low, hover, 0.5)


def _spin_until(env, model_of, stop_at: float = 0.95):  # noqa: ANN001, ANN202
    """Spin A and B together (one arm per button), reading each result from the pointer,
    until the posterior computed from its own observed spins passes 0.95, then carry
    the more likely biased spinner to the mat."""
    t: BiasedSpinner = env.task
    mo = Motion(env)
    model = model_of(t)
    arm = {"A": "left", "B": "right"}
    depth = BUTTONS["A"].travel + 0.004
    # Close both grippers and park each arm 2 cm above its button.
    yield from _both(*(mo.press_down(arm[k], t.buttons[k].top(), -0.02) for k in NAMES))
    seen: list[tuple[str, bool]] = []
    for _ in range(40):
        post = posterior(seen, model)
        if max(post.values()) >= stop_at and len(seen) >= 6:
            break
        yield from _both(*(_taps(mo, arm[k], t.buttons[k].top(), depth) for k in NAMES))
        yield from mo.hold(SPIN_TIME + 0.3)
        for k in NAMES:  # read the pointer: red half is angle in (0, pi)
            seen.append((k, t.reads_red(env.data, k)))
    for k in NAMES:
        yield from mo.line(arm[k], t.buttons[k].top() + np.array([0, 0, 0.02]),
                           t.buttons[k].top() + np.array([0, 0, 0.15]), 0.8)
    yield from _both(mo.home("left"), mo.home("right"))
    post = posterior(seen, model)
    best = max(post, key=post.__getitem__)
    yield from move_body(env, mo, f"spinner_{best}", "left", MAT_POS, 0.0, half=HALF[2])
    yield from mo.hold(1.5)


def oracle(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Infer with the task's true model (hard: each spinner's fair rate is its red area; stop at a
    stricter 0.99 there because the rigged 1/3 spinner can look fair for a while)."""
    yield from _spin_until(env, lambda t: t.model, 0.99 if env.task.difficulty == "hard" else 0.95)


def naive(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """The easy method: assume fair = 50 % red and biased = 80 % red, whatever the red areas."""
    yield from _spin_until(env, lambda t: easy_model(P_RED["easy"]))


def guess(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Blind baseline: carry a random spinner to the mat without spinning."""
    rng = rng or np.random.default_rng()
    mo = Motion(env)
    k = str(rng.choice(list(NAMES)))
    yield from move_body(env, mo, f"spinner_{k}", "left", MAT_POS, 0.0, half=HALF[2])
    yield from mo.hold(1.5)


TASK = BiasedSpinner
POLICIES = {"oracle": oracle, "guess": guess, "noop": noop, "naive": naive}
SPEC = TaskSpec(
    key="biased_spinner",
    title="Find the biased spinner",
    ability="Noise and replication",
    instruction=INSTRUCTION,
    objects=(
        "two spinners A and B: 7 x 7 x 3 cm blocks, top half red / half blue, black pointer",
        "one white push button in front of each spinner (spins it)",
        "red target mat, 15 x 15 cm",
    ),
    setup=(
        "Spinners 32 cm in front of the arms, A on the left (y = -19 cm), B on the right (y = -42 cm).",
        "Each spinner's button 21 cm in front of the arms, in line with it.",
        "Red mat on the operator's right, 27 cm in front of the left arm's base.",
    ),
    hidden=(
        "Which spinner is biased (uniform). Biased P(red) = 0.8; the other is fair. One spin is weak "
        "evidence: the reference procedure needs about 10 spins per spinner."
    ),
    instruction_hard=(
        "One of the two spinners, A or B, is rigged to land on red more often than a fair spin would; "
        "the other is fair. Put the rigged spinner on the red mat."
    ),
    hidden_hard=(
        "Red sectors cover 1/3 of one spinner's face and 2/3 of the other's (random). The fair spinner lands "
        "on red with probability equal to its red area; the rigged one 0.25 more (0.58 or 0.92). When the "
        "rigged spinner is the 1/3 one it shows fewer reds than the fair 2/3 one, so 'most reds' is wrong "
        "half the time."
    ),
    rubric=(
        "No purposeful contact with the spinners or buttons",
        "Touches a spinner or presses a button",
        "At least one spin observed",
        "Observed spins give posterior >= 0.9 that the true biased spinner is biased (hard: fair rate = red area)",
        "Stage 3, and the biased spinner (only) resting on the mat",
    ),
    metrics=(
        "stage (0-4)",
        "success: only the biased spinner on the mat, evidence or not",
        "science_success: stage >= 3",
        "spins used; posterior for each spinner",
    ),
    report_fields=("spins", "posterior", "biased", "red_area", "on_mat"),
    max_seconds=240.0,
    limitations=("The spin is animated in code: pressing the button runs a 2 s decelerating rotation "
                 "that stops in a sector drawn from the hidden bias."),
)
