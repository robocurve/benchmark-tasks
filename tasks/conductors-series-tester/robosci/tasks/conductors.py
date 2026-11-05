"""Task: sort six look-alike bars into conductors and insulators with a continuity tester.

Ability: property testing. Hidden variable: which bars conduct (2 to 4 of the 6).
The tester is a metal plate wired to a lamp: a bar standing on the plate lights the
lamp iff it conducts (computed in code). A test is a bar resting on the plate, not
held, for 1 s.

Hard: the tester is two brass pads wired in series with the lamp, so the lamp lights only
when BOTH pads hold a conducting bar. One bar alone never lights it (the easy method reads
every bar as an insulator). The robot has to find a conducting pair, then use a known
conductor as a probe. A test is two bars resting on the two pads, released, for 1 s; the
scorer keeps every conductor set (2 to 4 of 6) consistent with all pair tests.
"""

from __future__ import annotations

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
LABELS = tuple(f"R{i}" for i in range(1, 7))
Y0 = -0.305
START = tuple((0.22, Y0 + dy) for dy in (0.25, 0.15, 0.05, -0.05, -0.15, -0.25))
TESTER = (0.36, Y0)
TESTER_HALF = 0.035
TESTER_TOP = 0.004
LAMP_XY = (0.44, Y0)
BINS = {"lit": (0.40, 0.06), "dark": (0.40, -0.67)}
BIN_HALF = (0.08, 0.08)
BIN_SPOTS = ((-0.032, -0.032), (-0.032, 0.032), (0.032, -0.032), (0.032, 0.032))
GRIP = 0.0
OPEN = 0.03
# Hard: two series pads side by side across the table, lamp beyond them.
PADS = ((0.36, Y0 + 0.065), (0.36, Y0 - 0.065))
PAD_HALF = 0.025
LAMP_XY_HARD = (0.45, Y0)
HYPS = tuple(frozenset(c) for k in (2, 3, 4) for c in itertools.combinations(LABELS, k))

INSTRUCTION = (
    "Six bars labelled R1 to R6 look the same, but some conduct electricity and some do not. "
    "Stand each bar on the tester plate to check it: the lamp lights for a conductor. Put the "
    "conductors in the bin marked LIT and the insulators in the bin marked DARK."
)


@dataclass
class Conductors:
    difficulty: str = "easy"
    key: str = "conductors"
    instruction: str = INSTRUCTION
    conducts: dict[str, bool] = field(default_factory=dict)
    tested: dict[str, bool] = field(default_factory=dict)

    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        while True:
            c = rng.random(6) < 0.5
            if 2 <= c.sum() <= 4:
                break
        self.conducts = {lab: bool(v) for lab, v in zip(LABELS, c, strict=True)}
        order = rng.permutation(6)
        self.start = {LABELS[i]: START[k] for k, i in enumerate(order)}

        w = spec.worldbody
        if self.difficulty == "easy":
            w.add_geom(name="tester", type=props.G.mjGEOM_BOX, size=[TESTER_HALF, TESTER_HALF, TESTER_TOP / 2],
                       pos=[TESTER[0], TESTER[1], TESTER_TOP / 2], rgba=[0.75, 0.62, 0.30, 1],
                       friction=[1.0, 0.01, 0.001])
            w.add_geom(type=props.G.mjGEOM_BOX, size=[0.045, 0.045, 0.002], pos=[TESTER[0], TESTER[1], 0.001],
                       rgba=list(props.DARK), contype=0, conaffinity=0)
            # wire from tester to lamp (printed)
            props.flat(spec, "wire", ((TESTER[0] + LAMP_XY[0]) / 2, Y0 + 0.02), (0.04, 0.002), (0.1, 0.1, 0.1, 1))
            props.lamp(spec, "lamp", LAMP_XY, radius=0.022, post=0.05)
        else:
            # Dark board with two brass pads; printed series wiring pad A -> pad B -> lamp.
            w.add_geom(name="tester", type=props.G.mjGEOM_BOX, size=[0.04, 0.105, TESTER_TOP / 2],
                       pos=[0.36, Y0, TESTER_TOP / 2], rgba=list(props.DARK), friction=[1.0, 0.01, 0.001])
            for i, (x, y) in enumerate(PADS):
                props.flat(spec, f"pad{i}", (x, y), (PAD_HALF, PAD_HALF), (0.75, 0.62, 0.30, 1), z=TESTER_TOP)
            # printed series loop: pad A -> pad B along the far edge, then to the lamp
            props.flat(spec, "wire", (0.392, Y0), (0.002, 0.065), (0.1, 0.1, 0.1, 1), z=TESTER_TOP)
            props.flat(spec, "wire2", (0.42, Y0), (0.03, 0.002), (0.1, 0.1, 0.1, 1))
            props.lamp(spec, "lamp", LAMP_XY_HARD, radius=0.022, post=0.05)
        for name, xy in BINS.items():
            props.tray(spec, f"bin_{name}", xy, BIN_HALF, rgba=props.BLUE if name == "lit" else props.GREY)
            props.text_plate(spec, f"lbl_{name}", (xy[0] + 0.11, xy[1]), name.upper(), half=(0.03, 0.03))
        for lab in LABELS:
            x, y = self.start[lab]
            props.block(spec, f"rod_{lab}", (x, y), HALF, 0.05, label=lab, bg=(150, 150, 155),
                        fg=(20, 20, 20))

    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._m = model
        self.lamp = props.Lamp("lamp")
        self.lamp.bind(model)
        self.touch = props.TouchTracker(tuple(f"rod_{lab}" for lab in LABELS))
        self.touch.bind(model, data)
        self._tips = props.gripper_geoms(model)
        self._geom = {model.geom(f"rod_{lab}").id: lab for lab in LABELS}
        self.tested = {}
        self._on_since: tuple[str, float] | None = None
        self.pair_tests: list[tuple[frozenset[str], bool]] = []
        self._pair_since: tuple[frozenset[str], float] | None = None

    def pos(self, data: mujoco.MjData, lab: str) -> np.ndarray:
        return data.xpos[self._m.body(f"rod_{lab}").id]

    def held(self, data: mujoco.MjData) -> set[str]:
        return {self._geom[g] for g in props.held_geoms(self._m, data, self._tips) if g in self._geom}

    def on_tester(self, data: mujoco.MjData) -> list[str]:
        return [lab for lab in LABELS
                if props.in_box(self.pos(data, lab), TESTER, (TESTER_HALF, TESTER_HALF),
                                z_max=TESTER_TOP + HALF[2] + 0.01)]

    def on_pads(self, data: mujoco.MjData) -> list[list[str]]:
        return [[lab for lab in LABELS if props.in_box(self.pos(data, lab), xy, (PAD_HALF, PAD_HALF),
                                                        z_max=TESTER_TOP + HALF[2] + 0.01)] for xy in PADS]

    def hypotheses(self) -> list[frozenset[str]]:
        """Hard: conductor sets consistent with every pair test (lit iff both conduct)."""
        return [h for h in HYPS if all((pair <= h) == lit for pair, lit in self.pair_tests)]

    def known(self) -> dict[str, bool]:
        """Bars whose type every remaining hypothesis agrees on."""
        hs = self.hypotheses()
        return {lab: hs[0].__contains__(lab) for lab in LABELS
                if hs and len({lab in h for h in hs}) == 1}

    def _monitor_hard(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        t = float(data.time)
        pads = self.on_pads(data)
        full = all(len(p) == 1 for p in pads)
        lit = full and all(self.conducts[p[0]] for p in pads)
        if lit != self.lamp.on:
            self.lamp.set(model, lit)
        held = self.held(data)
        if full and not ({pads[0][0], pads[1][0]} & held):
            pair = frozenset((pads[0][0], pads[1][0]))
            if self._pair_since is None or self._pair_since[0] != pair:
                self._pair_since = (pair, t)
            elif t - self._pair_since[1] >= 1.0 and pair not in {q for q, _ in self.pair_tests}:
                self.pair_tests.append((pair, lit))
                for lab in pair:
                    self.tested[lab] = self.tested.get(lab, False) or lit
        else:
            self._pair_since = None

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        t = float(data.time)
        self.touch.update(data)
        if self.difficulty != "easy":
            self._monitor_hard(model, data)
            return
        on = self.on_tester(data)
        lit = any(self.conducts[lab] for lab in on)
        if lit != self.lamp.on:
            self.lamp.set(model, lit)
        held = self.held(data)
        if len(on) == 1 and on[0] not in held:
            if self._on_since is None or self._on_since[0] != on[0]:
                self._on_since = (on[0], t)
            elif t - self._on_since[1] >= 1.0 and on[0] not in self.tested:
                self.tested[on[0]] = lit
        else:
            self._on_since = None

    def bin_of(self, data: mujoco.MjData, lab: str) -> str | None:
        p = self.pos(data, lab)
        for name, xy in BINS.items():
            if props.in_box(p, xy, BIN_HALF, z_max=0.08):
                return name
        return None

    def hud(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
        sc = self.score(model, data)
        tested = " ".join(f"{k}:{'lit' if v else 'dark'}" for k, v in self.tested.items())
        if self.difficulty != "easy":
            tested = " ".join(f"{'+'.join(sorted(q))}:{'lit' if v else 'dark'}" for q, v in self.pair_tests)
            tested += f"   hypotheses left {len(self.hypotheses())}"
        return [f"t = {data.time:5.1f} s   lamp {'LIT' if self.lamp.on else 'dark'}   on tester "
                f"{self.on_tester(data) if self.difficulty == 'easy' else self.on_pads(data)}",
                f"rubric stage {sc['stage']}/4   tested {len(self.tested)}/6   correctly binned "
                f"{sc['correct']}/6   untested binned {sc['procedure_violations']}",
                f"  tests: {tested or '-'}"]

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        held = self.held(data)
        bins = {lab: (None if lab in held else self.bin_of(data, lab)) for lab in LABELS}
        correct = sum(bins[lab] == ("lit" if self.conducts[lab] else "dark") for lab in LABELS)
        success = correct == 6
        hard = self.difficulty != "easy"
        known = self.known() if hard else self.tested
        untested_binned = sum(bins[lab] is not None and lab not in known for lab in LABELS)
        stage = 0
        if self.touch.touched:
            stage = 1
        if (self.pair_tests if hard else self.tested):
            stage = 2
        if (len(self.hypotheses()) == 1) if hard else len(self.tested) == 6:
            stage = 3
            if success:
                stage = 4
        return {
            "stage": stage,
            "success": float(success),
            "science_success": float(stage >= 3),
            "tested": len(self.tested),
            "correct": correct,
            "procedure_violations": untested_binned,
            "conductors": sorted(k for k, v in self.conducts.items() if v),
            "bins": bins,
            "pair_tests": len(self.pair_tests),
            "log": ([{"pair": sorted(q), "lit": v} for q, v in self.pair_tests] if hard else dict(self.tested)),
        }


# ---------- reference policies ----------
def _carry(env, mo: Motion, lab: str, xy, surface: float = 0.0):  # noqa: ANN001, ANN202
    side = "left" if xy[1] > Y0 else "right"
    return move_body(env, mo, f"rod_{lab}", side, xy, surface, half=HALF[2], grip=GRIP, open_to=OPEN)


def _one_by_one(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Test every bar on the plate, read the lamp, and bin it by the result."""
    t: Conductors = env.task
    mo = Motion(env)
    fill = {"lit": 0, "dark": 0}
    for lab in sorted(LABELS, key=lambda lab: -t.start[lab][1]):
        yield from _carry(env, mo, lab, TESTER, TESTER_TOP)
        yield from mo.hold(1.5)
        res = "lit" if t.lamp.on else "dark"
        dx, dy = BIN_SPOTS[fill[res] % 4]
        fill[res] += 1
        bx, by = BINS[res]
        yield from _carry(env, mo, lab, (bx + dx, by + dy))
    yield from mo.hold(1.0)


def _pairs(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Series tester: try pairs until one lights (both conduct), then keep one known conductor
    on pad A as a probe and test each remaining bar on pad B. Bars leave the pads straight
    into their bin once their type is known."""
    t: Conductors = env.task
    mo = Motion(env)
    fill = {"lit": 0, "dark": 0}
    pad: list[str | None] = [None, None]
    binned: set[str] = set()

    def to_bin(lab: str):  # noqa: ANN202
        res = "lit" if t.known()[lab] else "dark"
        dx, dy = BIN_SPOTS[fill[res] % 4]
        fill[res] += 1
        binned.add(lab)
        return _carry(env, mo, lab, (BINS[res][0] + dx, BINS[res][1] + dy))

    def put(lab: str, k: int):  # noqa: ANN202
        pad[k] = lab
        return _carry(env, mo, lab, PADS[k], TESTER_TOP)

    def clear(k: int):  # noqa: ANN202
        lab, pad[k] = pad[k], None
        if lab in t.known():
            return to_bin(lab)
        return _carry(env, mo, lab, t.start[lab])

    order = sorted(LABELS, key=lambda lab: -t.start[lab][1])
    # Phase 1: find a conducting pair. Keep a bar on pad A while it might still conduct.
    probe = None
    for a, b in itertools.combinations(order, 2):
        if len(t.hypotheses()) == 1:
            break
        if frozenset((a, b)) in {q for q, _ in t.pair_tests}:
            continue
        known = t.known()
        if known.get(a) is False or known.get(b) is False:
            continue
        for k, lab in ((0, a), (1, b)):
            if pad[k] != lab:
                if pad[k] is not None:
                    yield from clear(k)
                if lab in pad:  # on the other pad: move it home first
                    j = pad.index(lab)
                    pad[j] = None
                    yield from _carry(env, mo, lab, t.start[lab])
                yield from put(lab, k)
        yield from mo.hold(1.5)
        if t.lamp.on:
            probe = a
            break
    # Phase 2: probe on pad A, every undetermined bar on pad B.
    if probe is not None:
        for lab in order:
            if len(t.hypotheses()) == 1:
                break
            if lab in t.known():
                continue
            if pad[1] is not None:
                yield from clear(1)
            yield from put(lab, 1)
            yield from mo.hold(1.5)
    for k in (1, 0):
        if pad[k] is not None:
            yield from clear(k)
    for lab in order:
        if lab not in binned and lab in t.known():
            yield from to_bin(lab)
    yield from mo.hold(1.0)


def oracle(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Easy: one bar at a time. Hard: find a conducting pair, then use it as a probe."""
    pol = _one_by_one if env.task.difficulty == "easy" else _pairs
    yield from pol(env, rng)


def guess(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Blind baseline: sort the bars at random, without the tester."""
    rng = rng or np.random.default_rng()
    t: Conductors = env.task
    mo = Motion(env)
    fill = {"lit": 0, "dark": 0}
    for lab in sorted(LABELS, key=lambda lab: -t.start[lab][1]):
        res = "lit" if rng.random() < 0.5 else "dark"
        if fill[res] >= 4:
            res = "dark" if res == "lit" else "lit"
        dx, dy = BIN_SPOTS[fill[res]]
        fill[res] += 1
        bx, by = BINS[res]
        yield from _carry(env, mo, lab, (bx + dx, by + dy))
    yield from mo.hold(1.0)


TASK = Conductors
POLICIES = {"oracle": oracle, "guess": guess, "noop": noop, "naive": _one_by_one}
SPEC = TaskSpec(
    key="conductors",
    title="Sort conductors from insulators",
    ability="Property testing",
    instruction=INSTRUCTION,
    objects=(
        "six grey bars 3 x 3 x 6 cm, labelled R1 to R6, identical look (some metal, some plastic core)",
        "continuity tester: brass plate 7 x 7 cm wired to a lamp (hard: two 5 x 5 cm brass pads 13 cm apart "
        "on a dark board, wired in series with the lamp)",
        "blue bin labelled LIT and grey bin labelled DARK, 16 x 16 cm each",
    ),
    setup=(
        "Tester plate 36 cm in front of the arms, centred; lamp 8 cm beyond it (hard: the two pads 36 cm in "
        "front, 6.5 cm either side of the centre line; lamp 45 cm in front).",
        "Stand the six bars in a row 22 cm in front of the arms, 10 cm apart, in random order.",
        "LIT bin on the operator's right (left arm side), DARK bin on the left (right arm side), "
        "both 40 cm in front.",
    ),
    hidden="Which bars conduct: each with probability 1/2, at least 2 and at most 4 of the 6.",
    instruction_hard=(
        "Six bars labelled R1 to R6 look the same, but some conduct electricity and some do not; at least "
        "two conduct. The tester's two brass pads are wired in series with its lamp. Put the conductors in "
        "the bin marked LIT and the insulators in the bin marked DARK."
    ),
    hidden_hard=(
        "As easy (2 to 4 of 6 conduct, 50 possible sets). The lamp lights only when both pads hold a "
        "conducting bar, so a single bar never lights it and testing bars one at a time reads all six as "
        "insulators. Pair tests narrow the 50 sets; once one conductor is known it probes the rest."
    ),
    rubric=(
        "No purposeful contact with the bars",
        "Touches or moves a bar",
        "At least one bar tested (resting alone on the plate, released, for 1 s)",
        "Easy: all six bars tested. Hard: the pair tests leave exactly one possible set of conductors",
        "Stage 3 and every bar in the correct bin",
    ),
    metrics=(
        "stage (0-4)",
        "success: every bar in the correct bin, tested or not",
        "science_success: stage >= 3",
        "bars tested; bars binned correctly",
        "procedure violations: bars binned before tests determined their type",
    ),
    report_fields=("tested", "correct", "bins"),
    max_seconds=600.0,
    limitations=("Conduction is a hidden per-bar flag read by code when the bar rests on the plate "
                 "(hard: on a pad); the series circuit through two bars is not modelled physically."),
)
