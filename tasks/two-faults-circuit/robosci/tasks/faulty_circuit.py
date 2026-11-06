"""Task: find the faulty part in a snap-block circuit, replace it, bin it.

Ability: fault diagnosis. Hidden variable: which installed part is dead (easy), or which
TWO installed parts are dead (hard; spares are all good). The board has four slots in one loop (battery, wire,
bulb, wire) and a latching switch. The bulb lights iff every slot holds a part of the
right kind, every seated part is good, and the switch is on. Circuit logic runs in code.

A test is a circuit configuration observed with the switch on for 1 s. The scorer keeps
the set of fault hypotheses consistent with all tests; science is proven when every
consistent hypothesis names the same dead installed part.

Hard: two installed parts are dead and the number is not told (the scorer's prior is any
one or two installed parts). Every single-part swap leaves the bulb dark, so the easy
method (swap one part at a time) never learns anything; the robot has to suspect more
than one fault and swap two parts at a time.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import combinations, product
from typing import Any

import mujoco
import numpy as np

from robosci import props
from robosci.motion import Motion
from robosci.skills import move_body, noop
from robosci.specs import TaskSpec

HALF = (0.025, 0.025, 0.02)
BOARD_X, BOARD_Y = 0.34, -0.305
BOARD_TOP = 0.006
SLOT_DY = 0.095
SLOTS = (("S1", "B"), ("S2", "W"), ("S3", "L"), ("S4", "W"))  # loop order
SLOT_XY = {s: (BOARD_X, BOARD_Y + SLOT_DY * (1.5 - i)) for i, (s, _) in enumerate(SLOTS)}
INSTALLED = {"S1": "B1", "S2": "W1", "S3": "L1", "S4": "W2"}
SPARES = ("B2", "B3", "W3", "L2", "L3", "W4")
PARTS = tuple(INSTALLED.values()) + SPARES
SPARE_X = 0.22
SPOTS = tuple((SPARE_X, BOARD_Y + dy) for dy in (0.28, 0.19, 0.10, -0.10, -0.19, -0.28))
PARK = ((0.31, 0.02), (0.31, -0.63))  # two empty parking spots at the row ends
BUTTON = props.ButtonSpec("switch", (0.22, BOARD_Y), rgba=(0.2, 0.2, 0.2, 1))
BIN_XY, BIN_HALF = (0.27, 0.10), (0.07, 0.07)
SEAT_TOL = 0.015
COLORS = {"B": (60, 110, 200), "W": (220, 220, 220), "L": (235, 200, 70)}

INSTRUCTION = (
    "The circuit on the board should light the bulb when the switch is on, but one part is "
    "faulty. Spare batteries (B), bulbs (L) and wires (W) are on the table. Find the faulty "
    "part, replace it so the bulb lights, and put the faulty part in the red bin."
)


def kind(part: str) -> str:
    return part[0]


@dataclass
class Test:
    t: float
    config: tuple[str, ...]
    lit: bool


@dataclass
class FaultyCircuit:
    difficulty: str = "easy"
    key: str = "faulty_circuit"
    instruction: str = INSTRUCTION
    dead: set[str] = field(default_factory=set)
    tests: list[Test] = field(default_factory=list)

    # ---------- build ----------
    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        if self.difficulty == "easy":
            self.dead_installed = str(rng.choice(list(INSTALLED.values())))
            self.dead = {self.dead_installed}
        else:
            self.dead = {str(x) for x in rng.choice(list(INSTALLED.values()), 2, replace=False)}
            self.dead_installed = " ".join(sorted(self.dead))
        spare_order = list(rng.permutation(len(SPARES)))
        self.start_xy = {INSTALLED[s]: SLOT_XY[s] for s, _ in SLOTS}
        for k, i in enumerate(spare_order):
            self.start_xy[SPARES[i]] = SPOTS[k]

        w = spec.worldbody
        w.add_geom(name="board", type=props.G.mjGEOM_BOX, size=[0.06, 0.2, BOARD_TOP / 2],
                   pos=[BOARD_X, BOARD_Y, BOARD_TOP / 2], rgba=[0.10, 0.32, 0.18, 1])
        # Printed loop: two long traces plus end links, and slot outlines.
        trace = [0.85, 0.65, 0.25, 1]
        for dx in (-0.045, 0.045):
            props.flat(spec, f"trace_{dx}", (BOARD_X + dx, BOARD_Y), (0.003, 0.17), trace, z=BOARD_TOP)
        for s, _ in SLOTS:
            x, y = SLOT_XY[s]
            props.flat(spec, f"slot_{s}", (x, y), (0.032, 0.032), (0.05, 0.18, 0.09, 1), z=BOARD_TOP + 0.0005)
        for s, k in SLOTS:
            x, y = SLOT_XY[s]
            props.text_plate(spec, f"lbl_{s}", (x + 0.075, y), k, half=(0.014, 0.014),
                             bg=(240, 240, 235))
        props.push_button(spec, BUTTON)
        props.lamp(spec, "switch_led", (0.22, BOARD_Y + 0.05, 0), radius=0.008, rgba_on=(0.2, 1.0, 0.3, 1),
                   post=0.015)
        props.lamp(spec, "bulb", (0.44, BOARD_Y + 0.0, 0), radius=0.025, post=0.05)
        props.tray(spec, "bin", BIN_XY, BIN_HALF, rgba=props.RED)
        for p in PARTS:
            x, y = self.start_xy[p]
            z = (BOARD_TOP if p in INSTALLED.values() else 0.0) + HALF[2] + 0.001
            props.block(spec, f"part_{p}", (x, y, z), HALF, 0.06, label=p, bg=COLORS[kind(p)])

    # ---------- runtime ----------
    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._m = model
        self.button = props.Button(BUTTON)
        self.button.bind(model)
        self.led = props.Lamp("switch_led", (0.2, 1.0, 0.3, 1))
        self.led.bind(model)
        self.bulb = props.Lamp("bulb")
        self.bulb.bind(model)
        self.touch = props.TouchTracker(tuple(f"part_{p}" for p in PARTS) + ("switch",))
        self.touch.bind(model, data)
        self._tips = props.gripper_geoms(model)
        self._pgeom = {model.geom(f"part_{p}").id: p for p in PARTS}
        self.switch_on = False
        self.lit = False
        self.tests = []
        self._cfg_since: tuple[tuple[str, ...], float] | None = None

    def part_pos(self, data: mujoco.MjData, p: str) -> np.ndarray:
        return data.xpos[self._m.body(f"part_{p}").id]

    def held(self, data: mujoco.MjData) -> set[str]:
        return {self._pgeom[g] for g in props.held_geoms(self._m, data, self._tips) if g in self._pgeom}

    def config(self, data: mujoco.MjData) -> tuple[str | None, ...]:
        """Part seated in each slot (None if empty, ambiguous, held or wrong kind)."""
        held = self.held(data)
        out: list[str | None] = []
        for s, k in SLOTS:
            here = [p for p in PARTS if p not in held and kind(p) == k
                    and props.in_box(self.part_pos(data, p), SLOT_XY[s], (SEAT_TOL, SEAT_TOL),
                                     z_max=BOARD_TOP + HALF[2] + 0.008)]
            out.append(here[0] if len(here) == 1 else None)
        return tuple(out)

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        t = float(data.time)
        self.touch.update(data)
        if self.button.update(data):
            self.switch_on = not self.switch_on
            self.led.set(model, self.switch_on)
        cfg = self.config(data)
        closed = all(c is not None for c in cfg)
        lit = closed and self.switch_on and not (set(cfg) & self.dead)
        if lit != self.lit:
            self.lit = lit
            self.bulb.set(model, lit)
        if closed and self.switch_on:
            key = tuple(cfg)  # type: ignore[arg-type]
            if self._cfg_since is None or self._cfg_since[0] != key:
                self._cfg_since = (key, t)
            elif t - self._cfg_since[1] >= 1.0 and all(x.config != key for x in self.tests):
                self.tests.append(Test(t, key, lit))
        else:
            self._cfg_since = None

    # ---------- inference over recorded tests (observations only) ----------
    def hypotheses(self, prior: str | None = None) -> list[frozenset[str]]:
        """Dead-part sets consistent with every recorded test, under the task's prior
        (easy: one installed part; hard: one or two installed parts, spares good)."""
        inst = list(INSTALLED.values())
        if (prior or self.difficulty) == "easy":
            cands = [frozenset({d}) for d in inst]
        else:
            cands = [frozenset(c) for k in (1, 2) for c in combinations(inst, k)]
        return [h for h in cands if all((not (set(x.config) & h)) == x.lit for x in self.tests)]

    def proven_dead(self, prior: str | None = None) -> frozenset[str] | None:
        hs = self.hypotheses(prior)
        return hs[0] if len(hs) == 1 and self.tests else None

    def in_bin(self, data: mujoco.MjData) -> set[str]:
        held = self.held(data)
        return {p for p in PARTS if p not in held
                and props.in_box(self.part_pos(data, p), BIN_XY, BIN_HALF, z_max=0.06)}

    def hud(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
        sc = self.score(model, data)
        cfg = self.config(data)
        lines = [f"t = {data.time:5.1f} s   switch {'ON ' if self.switch_on else 'off'}   "
                 f"bulb {'LIT' if self.lit else 'dark'}   board {' '.join(c or '--' for c in cfg)}",
                 f"rubric stage {sc['stage']}/4   tests {sc['tests']}   fault hypotheses left "
                 f"{sc['hypotheses_left']}   bin {sorted(sc['in_bin']) or '-'}"]
        for x in self.tests[-3:]:
            lines.append(f"  test {' '.join(x.config)}: {'lit' if x.lit else 'dark'}")
        return lines

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        cfg = self.config(data)
        binned = self.in_bin(data)
        good_board = all(c is not None and c not in self.dead for c in cfg)
        success = good_board and binned == self.dead
        proven = self.proven_dead()
        stage = 0
        if self.touch.touched or self.button.presses:
            stage = 1
        if self.tests:
            stage = 2
        if proven == self.dead:
            stage = 3
            if success:
                stage = 4
        return {
            "stage": stage,
            "success": float(success),
            "science_success": float(stage >= 3),
            "tests": len(self.tests),
            "hypotheses_left": len(self.hypotheses()),
            "procedure_violations": len(binned - self.dead),  # good parts thrown away
            "dead_installed": self.dead_installed,
            "dead": sorted(self.dead),
            "in_bin": sorted(binned),
            "bulb_lit": float(self.lit),
            "log": [x.__dict__ for x in self.tests],
        }


# ---------- reference policies ----------
def _carry(env, mo: Motion, part: str, xy, on_board: bool):  # noqa: ANN001, ANN202
    side = "left" if xy[1] > -0.305 else "right"
    return move_body(env, mo, f"part_{part}", side, xy, BOARD_TOP if on_board else 0.0, half=HALF[2],
                     open_to=0.037)


def _free_spot(env, t: FaultyCircuit) -> tuple[float, float]:  # noqa: ANN001
    taken = [t.part_pos(env.data, p)[:2] for p in PARTS]
    for xy in SPOTS + PARK:
        if all(np.linalg.norm(np.asarray(xy) - q) > 0.04 for q in taken):
            return xy
    raise RuntimeError("no free table spot")


def _swap(env, mo: Motion, t: FaultyCircuit, slot: str, new: str):  # noqa: ANN001, ANN202
    old = t.config(env.data)[[s for s, _ in SLOTS].index(slot)]
    if old is not None:
        yield from _carry(env, mo, old, _free_spot(env, t), on_board=False)
    yield from _carry(env, mo, new, SLOT_XY[slot], on_board=True)


def _next_swap(t: FaultyCircuit, cfg: tuple, prior: str | None = None) -> tuple[str, str] | None:  # noqa: ANN001
    """Greedy experiment choice from the recorded tests only: the single-part swap whose
    predicted bulb outcome splits the consistent fault hypotheses most evenly; if no swap
    is informative, the swap most hypotheses predict will light the bulb."""
    hs = t.hypotheses(prior)
    best, best_key = None, None
    for i, (slot, k) in enumerate(SLOTS):
        for p in PARTS:
            if kind(p) != k or p in cfg:
                continue
            new_cfg = set(cfg[:i] + (p,) + cfg[i + 1:])
            n_lit = sum(not (new_cfg & h) for h in hs)
            key = (max(n_lit, len(hs) - n_lit), -n_lit)
            if best_key is None or key < best_key:
                best, best_key = (slot, p), key
    return best


def _single_swaps(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Switch on, then run single-part swaps chosen greedily to split the single-fault
    hypotheses left by the tests so far, until the bulb is lit and the tests pin down the
    dead part. Uses only the bulb (observable) and the recorded tests, never the hidden fault."""
    t: FaultyCircuit = env.task
    mo = Motion(env)
    yield from mo.press("left", t.button.top(), depth=BUTTON.travel + 0.004)
    yield from mo.home("left")
    yield from mo.hold(1.5)
    for _ in range(14):
        if t.lit and t.proven_dead("easy"):
            break
        cfg = t.config(env.data)
        if any(c is None for c in cfg):  # a part slipped out of its slot
            break
        if not t.hypotheses("easy"):  # single-fault model refuted: the easy method has no next step
            break
        move = _next_swap(t, cfg, "easy")
        if move is None:
            break
        yield from _swap(env, mo, t, *move)
        yield from mo.hold(1.5)
    dead = t.proven_dead("easy")
    if dead is not None:
        yield from _carry(env, mo, next(iter(dead)), BIN_XY, on_board=False)
    yield from mo.hold(1.5)


def _next_config(t: FaultyCircuit, cfg: tuple) -> dict[str, str] | None:  # noqa: ANN001
    """Hard: choose up to two slot changes (each slot gets its original part or a spare of
    its kind) that split the one-or-two-fault hypotheses most evenly; ties go to the change
    most hypotheses predict lights the bulb, then to fewer part moves."""
    hs = t.hypotheses()
    orig = {s: INSTALLED[s] for s, _ in SLOTS}
    options = {}
    for i, (slot, k) in enumerate(SLOTS):
        options[slot] = [p for p in PARTS if kind(p) == k and (p == orig[slot] or p in SPARES)]
    best, best_key = None, None
    names = [s for s, _ in SLOTS]
    for n in (1, 2):
        for slots in combinations(range(4), n):
            for choice in product(*(options[names[i]] for i in slots)):
                new = list(cfg)
                for i, p in zip(slots, choice):
                    new[i] = p
                if len(set(new)) < 4 or tuple(new) == tuple(cfg):
                    continue
                if any(tuple(new) == x.config for x in t.tests):
                    continue
                n_lit = sum(not (set(new) & h) for h in hs)
                moves = sum(new[i] != cfg[i] for i in range(4))
                key = (max(n_lit, len(hs) - n_lit), -n_lit, moves)
                if best_key is None or key < best_key:
                    best, best_key = {names[i]: new[i] for i in range(4) if new[i] != cfg[i]}, key
    return best


def _multi_fault(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Hard: tests over one-or-two-fault hypotheses, changing up to two slots per test,
    until the bulb is lit and one dead set is left; then bin the dead parts."""
    t: FaultyCircuit = env.task
    mo = Motion(env)
    yield from mo.press("left", t.button.top(), depth=BUTTON.travel + 0.004)
    yield from mo.home("left")
    yield from mo.hold(1.5)
    for _ in range(10):
        if t.lit and t.proven_dead():
            break
        cfg = t.config(env.data)
        if any(c is None for c in cfg):
            break
        change = _next_config(t, cfg)
        if not change:
            break
        # Lift every outgoing part first, then seat the incoming ones (a part may move slots).
        for slot in change:
            old = cfg[[s for s, _ in SLOTS].index(slot)]
            yield from _carry(env, mo, old, _free_spot(env, t), on_board=False)
        for slot, p in change.items():
            yield from _carry(env, mo, p, SLOT_XY[slot], on_board=True)
        yield from mo.hold(1.5)
    dead = t.proven_dead()
    if dead is not None:
        for i, p in enumerate(sorted(dead)):
            yield from _carry(env, mo, p, (BIN_XY[0] + (0.03 if i else -0.03) * (len(dead) > 1), BIN_XY[1]),
                              on_board=False)
    yield from mo.hold(1.5)


def oracle(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Easy: greedy single-part swaps. Hard: greedy one-or-two-part changes."""
    pol = _single_swaps if env.task.difficulty == "easy" else _multi_fault
    yield from pol(env, rng)


def guess(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Blind baseline: replace a random installed part (two in hard) with a spare and bin it, no testing."""
    rng = rng or np.random.default_rng()
    t: FaultyCircuit = env.task
    mo = Motion(env)
    if t.difficulty == "easy":
        slot, k = SLOTS[int(rng.integers(4))]
        old = INSTALLED[slot]
        new = str(rng.choice([p for p in SPARES if kind(p) == k]))
        yield from _swap(env, mo, t, slot, new)
        yield from _carry(env, mo, old, BIN_XY, on_board=False)
    else:  # replace two random parts and bin them
        for i, j in enumerate(sorted(rng.choice(4, 2, replace=False))):
            slot, k = SLOTS[int(j)]
            new = str(rng.choice([p for p in SPARES if kind(p) == k and p not in t.config(env.data)]))
            yield from _swap(env, mo, t, slot, new)
            yield from _carry(env, mo, INSTALLED[slot], (BIN_XY[0] + (0.03 if i else -0.03), BIN_XY[1]),
                              on_board=False)
    yield from mo.hold(1.5)


TASK = FaultyCircuit
POLICIES = {"oracle": oracle, "guess": guess, "noop": noop, "naive": _single_swaps}
SPEC = TaskSpec(
    key="faulty_circuit",
    title="Find and replace the faulty part in a circuit",
    ability="Fault diagnosis",
    instruction=INSTRUCTION,
    objects=(
        "circuit board 12 x 40 cm with four labelled snap slots in one loop: battery (B), wire (W), "
        "bulb (L), wire (W)",
        "latching push-button switch with a green ON light, and the bulb lamp on the board",
        "installed parts B1, W1, L1, W2 (5 x 5 x 4 cm snap blocks, colour by kind)",
        "spares B2, B3, L2, L3, W3, W4 on the table in front of the board",
        "red bin, 14 x 14 cm",
    ),
    setup=(
        "Board 34 cm in front of the arms, centred between them; switch 22 cm in front, centred.",
        "Seat B1, W1, L1, W2 in their slots. Switch off.",
        "Place the six spares in a row 22 cm in front of the arms, in random order, with the two "
        "row-end spots left empty.",
        "Put the red bin on the operator's right, 27 cm in front of the left arm's base.",
    ),
    hidden="Which installed part is dead (uniform over B1, W1, L1, W2).",
    instruction_hard=(
        "The circuit on the board should light the bulb when the switch is on, but it does not. The spare "
        "batteries (B), bulbs (L) and wires (W) on the table are all good. Make the bulb light, and put "
        "every faulty part, and no good part, in the red bin."
    ),
    hidden_hard=(
        "Two installed parts are dead (uniform over the 6 pairs; the number is not told). Every single-part "
        "swap leaves the bulb dark, so it rules nothing out; replacing two parts at once is needed."
    ),
    rubric=(
        "No purposeful contact with the parts or the switch",
        "Touches a part or presses the switch",
        "At least one test: switch on with all four slots filled, bulb observed for 1 s",
        "The recorded tests leave one possible set of dead installed parts (easy prior: one part; hard "
        "prior: one or two), and it is the true one",
        "Stage 3, the board holds only good parts, and the dead part(s) (and no good part) are in the bin",
    ),
    metrics=(
        "stage (0-4)",
        "success: board all good and exactly the dead installed part(s) binned, proof or not",
        "science_success: stage >= 3",
        "tests used; fault hypotheses left after the tests",
        "procedure violations: good parts put in the bin",
    ),
    report_fields=("tests", "hypotheses_left", "dead_installed", "in_bin", "bulb_lit"),
    max_seconds=600.0,
    limitations=("Parts rest in slots by friction (no snap mechanism); conduction is computed from "
                 "which part sits in which slot."),
)
