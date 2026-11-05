"""Task: find the one blown fuse among eight using at most three group tests.

Ability: efficient experiment design. Hidden variable: which fuse is blown.
The tester has four fuse slots in series with a lamp and a TEST button. Pressing TEST
lights the lamp iff at least one fuse is seated and every seated fuse is good, so one
press tests a whole group. Eight candidates need log2(8) = 3 tests (halving search).
Each press with a non-empty tester is one test; the scorer keeps the fuses still
consistent with all tests.

Hard: two fuses are blown (28 hypotheses). The minimum adaptive budget with a 4-slot
tester, found by exhaustive minimax search (``_need``), is 6 tests; halving search for a
single fault (the easy method) cannot pin the pair down.
"""

from __future__ import annotations

import functools
import itertools
from dataclasses import dataclass, field
from typing import Any

import mujoco
import numpy as np

from robosci import props
from robosci.motion import Motion
from robosci.skills import move_body, noop
from robosci.specs import TaskSpec

HALF = (0.015, 0.015, 0.03)
LABELS = tuple(f"F{i}" for i in range(1, 9))
Y0 = -0.305
START = tuple((0.22, Y0 + 0.075 * (3.5 - i)) for i in range(8))
TESTER_X = 0.36
TESTER_TOP = 0.006
SLOT_XY = tuple((TESTER_X, Y0 + 0.09 * (1.5 - i)) for i in range(4))
SEAT_TOL = 0.015
BUTTON = props.ButtonSpec("test", (0.45, Y0 - 0.07), rgba=(0.85, 0.1, 0.1, 1))
LAMP_XY = (0.45, Y0 + 0.07)
BIN_XY, BIN_HALF = (0.40, 0.08), (0.07, 0.07)
BUDGET = 3
BUDGET_HARD = 7  # one test of slack over the optimum, _need(all pairs) = 6
GRIP, OPEN = 0.0, 0.03

INSTRUCTION = (
    "One of the eight fuses F1 to F8 is blown. The tester lights its lamp when you press TEST "
    "only if every fuse in its slots is good. Find the blown fuse using at most three presses "
    "of TEST, and put it in the red bin."
)


@dataclass
class GroupTest:
    t: float
    group: tuple[str, ...]
    lit: bool


@dataclass
class BlownFuse:
    difficulty: str = "easy"
    key: str = "blown_fuse"
    instruction: str = INSTRUCTION
    dead: str = ""
    tests: list[GroupTest] = field(default_factory=list)

    @property
    def budget(self) -> int:
        return BUDGET if self.difficulty == "easy" else BUDGET_HARD

    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        if self.difficulty == "easy":
            self.dead = str(rng.choice(LABELS))
            self.deads = frozenset({self.dead})
        else:
            self.deads = frozenset(str(x) for x in rng.choice(LABELS, 2, replace=False))
            self.dead = " ".join(sorted(self.deads))
        order = rng.permutation(8)
        self.start = {LABELS[i]: START[k] for k, i in enumerate(order)}
        w = spec.worldbody
        w.add_geom(name="tester", type=props.G.mjGEOM_BOX, size=[0.04, 0.2, TESTER_TOP / 2],
                   pos=[TESTER_X, Y0, TESTER_TOP / 2], rgba=[0.12, 0.12, 0.14, 1])
        props.flat(spec, "trace", (TESTER_X + 0.028, Y0), (0.002, 0.17), (0.85, 0.65, 0.25, 1), z=TESTER_TOP)
        for i, (x, y) in enumerate(SLOT_XY):
            props.flat(spec, f"slot{i}", (x, y), (0.022, 0.022), (0.75, 0.62, 0.30, 1), z=TESTER_TOP + 0.0005)
        props.push_button(spec, BUTTON)
        props.text_plate(spec, "lbl_test", (BUTTON.pos[0] + 0.065, BUTTON.pos[1]), "TEST", half=(0.025, 0.025))
        props.lamp(spec, "lamp", LAMP_XY, radius=0.022, post=0.05)
        props.tray(spec, "bin", BIN_XY, BIN_HALF, rgba=props.RED)
        for lab in LABELS:
            x, y = self.start[lab]
            props.block(spec, f"fuse_{lab}", (x, y), HALF, 0.04, label=lab, bg=(225, 215, 190))

    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._m = model
        self.lamp = props.Lamp("lamp")
        self.lamp.bind(model)
        self.button = props.Button(BUTTON)
        self.button.bind(model)
        self.touch = props.TouchTracker(tuple(f"fuse_{lab}" for lab in LABELS) + ("test",))
        self.touch.bind(model, data)
        self._tips = props.gripper_geoms(model)
        self._geom = {model.geom(f"fuse_{lab}").id: lab for lab in LABELS}
        self.tests = []
        self._shown: tuple[str, ...] | None = None
        self.proven_at: int | None = None  # number of tests when the fuse was first pinned down

    def pos(self, data: mujoco.MjData, lab: str) -> np.ndarray:
        return data.xpos[self._m.body(f"fuse_{lab}").id]

    def held(self, data: mujoco.MjData) -> set[str]:
        return {self._geom[g] for g in props.held_geoms(self._m, data, self._tips) if g in self._geom}

    def seated(self, data: mujoco.MjData) -> tuple[str, ...]:
        held = self.held(data)
        return tuple(lab for lab in LABELS if lab not in held and any(
            props.in_box(self.pos(data, lab), xy, (SEAT_TOL, SEAT_TOL), z_max=TESTER_TOP + HALF[2] + 0.008)
            for xy in SLOT_XY))

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.touch.update(data)
        group = self.seated(data)
        if self.button.update(data) and group:
            lit = not (self.deads & set(group))
            self.tests.append(GroupTest(float(data.time), group, lit))
            self.lamp.set(model, lit)
            self._shown = group
            if self.proven_at is None and len(self.hypotheses()) == 1:
                self.proven_at = len(self.tests)
        elif self._shown is not None and group != self._shown:
            self._shown = None  # result is shown until the tester contents change
            self.lamp.set(model, False)

    def hypotheses(self) -> list[frozenset[str]]:
        """Dead sets (one fuse in easy, a pair in hard) consistent with every recorded test."""
        k = len(self.deads)
        return [frozenset(h) for h in itertools.combinations(LABELS, k)
                if all((not set(h) & set(x.group)) == x.lit for x in self.tests)]

    def in_bin(self, data: mujoco.MjData) -> set[str]:
        held = self.held(data)
        return {lab for lab in LABELS if lab not in held
                and props.in_box(self.pos(data, lab), BIN_XY, BIN_HALF, z_max=0.08)}

    def hud(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
        sc = self.score(model, data)
        lines = [f"t = {data.time:5.1f} s   lamp {'LIT' if self.lamp.on else 'off'}   in tester "
                 f"{' '.join(self.seated(data)) or '-'}",
                 f"rubric stage {sc['stage']}/4   tests {sc['tests']} (budget {self.budget})   candidates left "
                 f"{len(self.hypotheses())}   bin {sorted(sc['in_bin']) or '-'}"]
        for x in self.tests[-3:]:
            lines.append(f"  test {' '.join(x.group)}: {'lit' if x.lit else 'dark'}")
        return lines

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        binned = self.in_bin(data)
        success = binned == set(self.deads)
        stage = 0
        if self.touch.touched or self.button.presses:
            stage = 1
        if self.tests:
            stage = 2
        if self.proven_at is not None and self.proven_at <= self.budget:
            stage = 3
            if success:
                stage = 4
        return {
            "stage": stage,
            "success": float(success),
            "science_success": float(stage >= 3),
            "tests": len(self.tests),
            "tests_to_proof": self.proven_at,
            "candidates_left": len(self.hypotheses()),
            "procedure_violations": max(0, len(self.tests) - self.budget),
            "dead": self.dead,
            "in_bin": sorted(binned),
            "log": [x.__dict__ for x in self.tests],
        }


# ---------- reference policies ----------
def _carry(env, mo: Motion, lab: str, xy, surface: float = 0.0):  # noqa: ANN001, ANN202
    side = "left" if xy[1] > Y0 else "right"
    return move_body(env, mo, f"fuse_{lab}", side, xy, surface, half=HALF[2], grip=GRIP, open_to=OPEN)


def _halving(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Halving search for ONE blown fuse: each test holds exactly half of the remaining
    candidates (plus any fuses already known good), read the lamp, keep the half that explains it."""
    t: BlownFuse = env.task
    mo = Motion(env)
    cand = sorted(LABELS, key=lambda lab: -t.start[lab][1])
    good: set[str] = set()
    slots: list[str | None] = [None] * 4
    while len(cand) > 1:
        want = set(cand[: len(cand) // 2])
        inside = {f for f in slots if f is not None}
        out = [f for f in inside if f in cand and f not in want]
        add = [f for f in cand if f in want and f not in inside]
        extra = len(inside) - len(out) + len(add) - 4
        out += [f for f in inside if f in good][:max(0, extra)]
        for f in out:
            slots[slots.index(f)] = None
            yield from _carry(env, mo, f, t.start[f])
        for f in add:
            k = slots.index(None)
            slots[k] = f
            yield from _carry(env, mo, f, SLOT_XY[k], TESTER_TOP)
        yield from mo.press("right", t.button.top(), depth=BUTTON.travel + 0.004)
        yield from mo.hold(1.0)
        group = {f for f in slots if f is not None}
        if t.lamp.on:
            good |= group
            cand = [f for f in cand if f not in group]
        else:
            good |= set(cand) - group
            cand = [f for f in cand if f in group]
        yield from mo.home("right")
    yield from _carry(env, mo, cand[0], BIN_XY)
    yield from mo.hold(1.0)


# ---- hard: exact minimax group testing over dead pairs ----
_GROUPS = tuple(frozenset(g) for k in range(1, 5) for g in itertools.combinations(LABELS, k))


@functools.lru_cache(maxsize=None)
def _need(hs: frozenset) -> int:
    """Worst-case number of tests to leave one hypothesis (hypotheses = frozensets of dead fuses)."""
    if len(hs) <= 1:
        return 0
    best = 99
    floor = max(1, (len(hs) - 1).bit_length())
    for g in _GROUPS:
        lit = frozenset(h for h in hs if not (h & g))
        if not lit or lit == hs:
            continue
        best = min(best, 1 + max(_need(lit), _need(hs - lit)))
        if best == floor:
            break
    return best


def _pair_search(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Minimax group testing: each test is a group that keeps the worst case within the
    budget, chosen to need the fewest fuse moves from the current tester contents."""
    t: BlownFuse = env.task
    mo = Motion(env)
    k = len(t.deads)
    hs = frozenset(frozenset(h) for h in itertools.combinations(LABELS, k))
    slots: list[str | None] = [None] * 4
    while len(hs) > 1:
        n = _need(hs)
        inside = {f for f in slots if f is not None}
        best = None
        for g in _GROUPS:
            lit = frozenset(h for h in hs if not (h & g))
            if not lit or lit == hs or 1 + max(_need(lit), _need(hs - lit)) > n:
                continue
            cost = len(inside ^ g)
            if best is None or cost < best[0]:
                best = (cost, g)
        g = best[1]
        for f in sorted(inside - g):
            slots[slots.index(f)] = None
            yield from _carry(env, mo, f, t.start[f])
        for f in sorted(g - inside):
            j = slots.index(None)
            slots[j] = f
            yield from _carry(env, mo, f, SLOT_XY[j], TESTER_TOP)
        yield from mo.press("right", t.button.top(), depth=BUTTON.travel + 0.004)
        yield from mo.hold(1.0)
        group = {f for f in slots if f is not None}
        lit = t.lamp.on
        hs = frozenset(h for h in hs if (not (h & group)) == lit)
        yield from mo.home("right")
    for i, f in enumerate(sorted(next(iter(hs)))):
        yield from _carry(env, mo, f, (BIN_XY[0] + (0.03 if i == 0 else -0.03), BIN_XY[1]))
    yield from mo.hold(1.0)


def oracle(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Easy: halving search. Hard: minimax search over dead pairs."""
    pol = _halving if env.task.difficulty == "easy" else _pair_search
    yield from pol(env, rng)


def guess(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Blind baseline: bin a random fuse (two in hard) without testing."""
    rng = rng or np.random.default_rng()
    mo = Motion(env)
    k = len(env.task.deads)
    for i, f in enumerate(rng.choice(LABELS, k, replace=False)):
        yield from _carry(env, mo, str(f), (BIN_XY[0] + (0.03 if i == 0 else -0.03) * (k > 1), BIN_XY[1]))
    yield from mo.hold(1.0)


TASK = BlownFuse
POLICIES = {"oracle": oracle, "guess": guess, "noop": noop, "naive": _halving}
SPEC = TaskSpec(
    key="blown_fuse",
    title="Find the blown fuse in three tests",
    ability="Efficient experiment design",
    instruction=INSTRUCTION,
    objects=(
        "eight cartridge fuses F1 to F8 (3 x 3 x 6 cm blocks), identical look",
        "fuse tester: four slots in series with a lamp, and a red TEST push button",
        "red bin, 14 x 14 cm",
    ),
    setup=(
        "Tester 36 cm in front of the arms, centred, slots in a row across the table.",
        "TEST button and lamp 45 cm in front, either side of the centre line.",
        "Stand the eight fuses in a row 22 cm in front of the arms, 7.5 cm apart, random order.",
        "Red bin on the operator's right, 40 cm in front of the left arm's base.",
    ),
    hidden="Which fuse is blown (uniform over the 8).",
    instruction_hard=(
        "Some of the eight fuses F1 to F8 are blown. The tester lights its lamp when you press TEST only "
        "if every fuse in its slots is good. Put every blown fuse, and no good fuse, in the red bin, "
        "using at most seven presses of TEST."
    ),
    hidden_hard=(
        "Exactly two fuses are blown (uniform over the 28 pairs; the number is not told). The budget is 7 "
        "tests, one more than the minimum worst-case 6 for a 4-slot tester (exhaustive minimax search). Halving search for a "
        "single fault ends after 3 tests with up to 28 pairs still possible."
    ),
    rubric=(
        "No purposeful contact with the fuses or the tester",
        "Touches a fuse or presses TEST",
        "At least one group test (TEST pressed with one or more fuses seated)",
        "Tests leave exactly one possible set of blown fuses within the budget (3 tests easy, 7 hard)",
        "Stage 3 and the blown fuse(s), and no good fuse, in the bin",
    ),
    metrics=(
        "stage (0-4)",
        "success: only the blown fuse in the bin, proof or not",
        "science_success: stage >= 3",
        "tests used, and tests needed to pin the blown set down (optimum 3 easy, 6 hard)",
        "procedure violations: tests beyond the budget",
    ),
    report_fields=("tests", "tests_to_proof", "candidates_left", "dead", "in_bin"),
    max_seconds=600.0,
    limitations="Fuse state is a hidden flag; the lamp result is computed in code at each TEST press.",
)
