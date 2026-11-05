"""Task: find the lowest breaking drop height with a limited number of test capsules.

Ability: efficient experiment design (destructive testing). Hidden variable: the threshold slot
f in 1..12. Twelve numbered drop pads stand in a row; a capsule standing on pad k is dropped from
height k (code-driven) when DROP is pressed, and it breaks iff k >= f (every capsule breaks from
slot 12). A broken capsule turns red and is used up. The answer is given by standing the blue flag
on the pad numbered f.

Easy: 3 capsules and 8 drops. Hard: 2 capsules and at most 5 drops (exactly the minimum worst case, so only an optimal schedule always works). With two capsules the best
worst case is 5 drops (the two-egg schedule: first capsule at 5, 9, 12 ..., then a linear scan
with the second), so 6 leaves one drop of slack; halving the range, the obvious search, breaks
both capsules early on low thresholds and cannot pin the slot down.
A drop counts once per capsule on a pad when DROP is pressed; drops of an already-broken capsule
carry no information. Science is proven when the drops within the budget leave one possible f.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache
from typing import Any

import mujoco
import numpy as np

from robosci import props
from robosci.motion import Motion
from robosci.skills import move_body, noop
from robosci.specs import TaskSpec

N_SLOTS = 12
PAD_X, PAD_HALF = 0.31, 0.019
PAD_Y = tuple(-0.06 - 0.045 * k for k in range(N_SLOTS))  # slot k+1
HALF = 0.015
CAP_XY = ((0.20, -0.25), (0.20, -0.36), (0.20, -0.47))
FLAG_XY = (0.20, -0.14)
BUTTON = props.ButtonSpec("drop", (0.42, -0.305), rgba=(0.95, 0.55, 0.1, 1))
WHITE, BROKEN = (0.94, 0.94, 0.92, 1), (0.75, 0.08, 0.06, 1)
CFG = {"easy": dict(caps=3, budget=8), "hard": dict(caps=2, budget=5)}

INSTRUCTION = (
    "Each capsule breaks if it is dropped from its threshold slot or any higher slot (1 to 12; every capsule "
    "breaks from slot 12). Stand a capsule on a numbered pad and press DROP to drop it from that slot; a "
    "broken capsule turns red. Find the threshold slot using at most 8 drops and the 3 capsules, then stand "
    "the blue flag on that slot's pad."
)
INSTRUCTION_HARD = (
    "Each capsule breaks if it is dropped from its threshold slot or any higher slot (1 to 12; every capsule "
    "breaks from slot 12); all capsules have the same threshold. Standing a capsule on a numbered pad and "
    "pressing DROP drops it from that slot; a broken capsule turns red and cannot be used again. You have 2 "
    "capsules and at most 5 drops, and every drop must keep you sure of finishing within those 5 drops whatever happens. Find the threshold slot exactly, then stand the blue flag on its pad."
)


@dataclass
class Drop:
    t: float
    capsule: int
    slot: int  # 1..12
    broke: bool
    informative: bool  # the capsule was intact before this drop


@lru_cache(maxsize=None)
def worst(eggs: int, n: int) -> int:
    """Fewest drops that always pin down f among n consecutive candidates (the top one always breaks)."""
    if n <= 1:
        return 0
    if eggs <= 0:
        return 10**6  # cannot be pinned down
    if eggs == 1:
        return n - 1
    return 1 + min(max(worst(eggs - 1, k), worst(eggs, n - k)) for k in range(1, n))


def best_drop(eggs: int, lo: int, hi: int) -> int:
    """Slot to drop at when f is known to lie in lo..hi: breaking leaves lo..k, surviving k+1..hi."""
    if eggs == 1:
        return lo  # last capsule: scan upward
    return min(range(lo, hi), key=lambda k: (max(worst(eggs - 1, k - lo + 1), worst(eggs, hi - k)), -k))


@dataclass
class TwoCapsules:
    difficulty: str = "easy"
    key: str = "two_capsules"
    instruction: str = INSTRUCTION
    drops: list[Drop] = field(default_factory=list)

    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        cfg = CFG["hard" if self.difficulty == "hard" else "easy"]
        self.n_caps, self.budget = cfg["caps"], cfg["budget"]
        # Easy: any slot. Hard: a fragile batch, threshold in slots 1-4 (the robot is told only 1-12), where
        # halving the range breaks both capsules before it pins the slot down 3 times in 4.
        self.f = int(rng.integers(1, N_SLOTS + 1))  # uniform 1-12, as the instruction says
        if self.difficulty == "hard":
            self.instruction = INSTRUCTION_HARD
        for k, y in enumerate(PAD_Y):
            props.flat(spec, f"pad{k + 1}", (PAD_X, y), (PAD_HALF, PAD_HALF), (0.42, 0.43, 0.47, 1))
            props.text_plate(spec, f"lbl_pad{k + 1}", (PAD_X - 0.042, y), str(k + 1), half=(0.016, 0.016))
        props.push_button(spec, BUTTON)
        props.text_plate(spec, "lbl_drop", (BUTTON.pos[0] - 0.05, BUTTON.pos[1]), "DROP", half=(0.016, 0.03))
        for i in range(self.n_caps):
            props.block(spec, f"cap{i}", CAP_XY[i], HALF, 0.05, rgba=WHITE)
        props.block(spec, "flag", FLAG_XY, HALF, 0.05, rgba=(0.15, 0.30, 0.85, 1))

    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._m = model
        self._tips = props.gripper_geoms(model)
        self.button = props.Button(BUTTON)
        self.button.bind(model)
        self._geom = {model.geom(f"cap{i}").id: f"cap{i}" for i in range(self.n_caps)}
        self._geom[model.geom("flag").id] = "flag"
        self.touch = props.TouchTracker(tuple(f"cap{i}" for i in range(self.n_caps)) + ("flag",))
        self.touch.bind(model, data)
        self.broken = [False] * self.n_caps
        for i in range(self.n_caps):
            model.geom_rgba[model.geom(f"cap{i}").id] = WHITE
        self.drops = []

    def held(self, data: mujoco.MjData) -> set[str]:
        out = set()
        for c in data.contact[: data.ncon]:
            g1, g2 = int(c.geom1), int(c.geom2)
            for a, b in ((g1, g2), (g2, g1)):
                if a in self._geom and b in self._tips:
                    out.add(self._geom[a])
        return out

    def slot_of(self, data: mujoco.MjData, body: str) -> int | None:
        p = data.xpos[self._m.body(body).id]
        for k, y in enumerate(PAD_Y):
            if props.in_box(p, (PAD_X, y), (PAD_HALF + 0.004,) * 2, z_max=HALF + 0.012):
                return k + 1
        return None

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        t = float(data.time)
        self.touch.update(data)
        if not self.button.update(data):
            return
        held = self.held(data)
        for i in range(self.n_caps):
            k = self.slot_of(data, f"cap{i}")
            if k is None or f"cap{i}" in held:
                continue
            informative = not self.broken[i]
            broke = self.broken[i] or k >= self.f
            self.drops.append(Drop(t, i, k, broke, informative))
            if broke and not self.broken[i]:
                self.broken[i] = True
                model.geom_rgba[model.geom(f"cap{i}").id] = BROKEN

    def candidates(self, n: int | None = None) -> list[int]:
        ds = [d for d in self.drops[: self.budget if n is None else n] if d.informative]
        return [f for f in range(1, N_SLOTS + 1) if all((d.slot >= f) == d.broke for d in ds)]

    def guaranteed(self) -> bool:
        """Hard: every drop kept a guarantee of pinning the threshold within the budget whatever its
        outcome (the two-egg minimax condition), judged from the drop record alone. Trying 1, 2, 3, ...
        or halving the range breaks it at the first drop, even when the threshold happens to be found."""
        lo, hi, eggs = 1, N_SLOTS, self.n_caps
        for used, d in enumerate(self.drops[: self.budget]):
            left = self.budget - used - 1
            if d.informative and lo <= d.slot < hi:
                if max(worst(eggs - 1, d.slot - lo + 1), worst(eggs, hi - d.slot)) > left:
                    return False
            elif worst(eggs, hi - lo + 1) > left:  # a drop that could not narrow the range wasted budget
                return False
            if d.informative:
                if d.broke:
                    hi, eggs = min(hi, d.slot), eggs - 1
                else:
                    lo = max(lo, d.slot + 1)
        return True

    def flag_slot(self, data: mujoco.MjData) -> int | None:
        return None if "flag" in self.held(data) else self.slot_of(data, "flag")

    def hud(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
        sc = self.score(model, data)
        c = self.candidates()
        ds = "  ".join(f"{d.slot}:{'X' if d.broke else 'ok'}" for d in self.drops[-8:])
        return [f"t = {data.time:5.1f} s   rubric stage {sc['stage']}/4   drops {len(self.drops)}/{self.budget}   "
                f"threshold {self.f}",
                f"drops: {ds or '-'}",
                f"possible thresholds: {c[0]}..{c[-1]} ({len(c)})   flag on pad {sc['flag_slot'] or '-'}"
                if c else "possible thresholds: none"]

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        flag = self.flag_slot(data)
        success = flag == self.f
        cand = self.candidates()
        stage = 0
        if self.touch.touched or self.button.presses:
            stage = 1
        if self.drops:
            stage = 2
        if cand == [self.f] and (self.difficulty == "easy" or self.guaranteed()):
            stage = 3
            if success:
                stage = 4
        return {
            "stage": stage,
            "success": float(success),
            "science_success": float(stage >= 3),
            "procedure_violations": max(0, len(self.drops) - self.budget),
            "threshold": self.f,
            "drops": len(self.drops),
            "candidates_left": len(cand),
            "flag_slot": flag,
            "log": [d.__dict__ for d in self.drops],
        }


# ---------- reference policies ----------
def _side(y: float) -> str:
    return "left" if y > -0.305 else "right"


def _carry(env, mo: Motion, body: str, xy):  # noqa: ANN001, ANN202
    """Carry ``body`` to ``xy``; if it lands more than 1 cm off (long reaches are less precise), nudge it
    again with the arm on the target's side."""
    bid = env.model.body(body).id
    for attempt in range(3):
        y0 = env.data.xpos[bid][1]
        side = _side(xy[1]) if attempt or abs(xy[1] - y0) > 0.12 else _side(y0)
        yield from move_body(env, mo, body, side, xy, 0.0, half=HALF, open_to=0.026, grip=0.0)
        if np.hypot(*(env.data.xpos[bid][:2] - np.asarray(xy))) < 0.01:
            return


def _drop(env, mo: Motion, cap: int, slot: int):  # noqa: ANN001, ANN202
    t: TwoCapsules = env.task
    yield from _carry(env, mo, f"cap{cap}", (PAD_X, PAD_Y[slot - 1]))
    side = "left" if BUTTON.pos[1] >= -0.305 else "right"
    yield from mo.press(side, t.button.top(), depth=BUTTON.travel + 0.004)
    yield from mo.home(side)
    yield from mo.hold(0.8)


def _clear_and_flag(env, mo: Motion, slot: int):  # noqa: ANN001, ANN202
    t: TwoCapsules = env.task
    for i in range(t.n_caps):
        if t.slot_of(env.data, f"cap{i}") == slot:
            yield from _carry(env, mo, f"cap{i}", CAP_XY[i])
    yield from _carry(env, mo, "flag", (PAD_X, PAD_Y[slot - 1]))
    yield from mo.hold(1.5)


def _search(env, choose):  # noqa: ANN001, ANN202
    """Generic search loop: ``choose(eggs, lo, hi)`` gives the next slot; stops when one candidate is
    left, the capsules run out, or the budget is spent; flags the lowest remaining candidate."""
    t: TwoCapsules = env.task
    mo = Motion(env)
    lo, hi = 1, N_SLOTS
    while lo < hi and len(t.drops) < t.budget:
        alive = [i for i in range(t.n_caps) if not t.broken[i]]
        if not alive:
            break
        k = choose(len(alive), lo, hi)
        n = len(t.drops)
        yield from _drop(env, mo, alive[0], k)
        if len(t.drops) == n:  # the press did not register a drop: try once more
            yield from _drop(env, mo, alive[0], k)
        if len(t.drops) == n:
            break
        if t.drops[-1].broke:
            hi = k
            yield from _carry(env, mo, f"cap{alive[0]}", CAP_XY[alive[0]])  # used up: off the pads
        else:
            lo = k + 1
    yield from _clear_and_flag(env, mo, lo)


def oracle(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Minimax schedule for the capsules left (the two-egg schedule in hard)."""
    yield from _search(env, best_drop)


def _halving(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """The obvious search: always drop at the middle of the remaining range."""
    yield from _search(env, lambda eggs, lo, hi: (lo + hi - 1) // 2)


def guess(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Blind baseline: the flag on a random pad."""
    rng = rng or np.random.default_rng()
    mo = Motion(env)
    yield from _carry(env, mo, "flag", (PAD_X, PAD_Y[int(rng.integers(N_SLOTS))]))
    yield from mo.hold(1.5)


TASK = TwoCapsules
POLICIES = {"oracle": oracle, "guess": guess, "noop": noop, "naive": _halving}
SPEC = TaskSpec(
    key="two_capsules",
    title="Find the breaking height with two capsules",
    ability="Efficient experiment design",
    instruction=INSTRUCTION,
    instruction_hard=INSTRUCTION_HARD,
    objects=(
        "twelve grey drop pads (3.8 x 3.8 cm) in a row, numbered 1 to 12 on printed cards",
        "orange DROP push button (3.6 cm cap) with a printed DROP card",
        "white test capsules, 3 cm cubes, 50 g (easy: 3; hard: 2); a broken capsule turns red",
        "blue flag, 3 cm cube",
    ),
    setup=(
        "Pads in a row 31 cm in front of the arms, 4.5 cm apart: pad 1 is 6 cm toward the right arm from the left "
        "arm's base line, pad 12 is 55.5 cm toward it; number cards 4 cm nearer the arms.",
        "DROP button 42 cm in front, centred between the arms.",
        "Capsules 20 cm in front at 25, 36 (and 47) cm right of the left arm's base line; flag at 14 cm.",
    ),
    hidden="The threshold slot (uniform 1..12).",
    hidden_hard=("The threshold slot, uniform over 1-12 as stated. Two capsules and 5 drops, exactly the minimum "
                 "worst case. Stage 3 also needs every drop to keep the guarantee of finishing within the budget "
                 "whatever its outcome (the two-egg minimax schedule, e.g. 5, 9, 12 then a linear scan), so trying "
                 "1, 2, 3, ... or halving the range earns no science credit even when it happens to hit."),
    rubric=(
        "No purposeful contact with the capsules, flag or button",
        "Touches a capsule or the flag, or presses DROP",
        "At least one drop (a capsule on a pad when DROP is pressed)",
        "The drops within the budget leave exactly one possible threshold slot (hard: and every drop kept the "
        "guarantee of finishing within 5 drops whatever its outcome)",
        "Stage 3 and the flag standing on that slot's pad",
    ),
    metrics=("stage (0-4)", "success: flag on the threshold pad, proven or not", "science_success: stage >= 3",
             "drops used"),
    report_fields=("threshold", "drops", "candidates_left", "flag_slot"),
    max_seconds=450.0,
    limitations=("The drop is simulated in code: pressing DROP decides from the slot whether each capsule on a pad "
                 "breaks and turns it red; the capsule is not physically raised or dropped."),
)
