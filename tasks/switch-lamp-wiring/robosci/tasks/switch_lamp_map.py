"""Task: work out which switch drives which lamp, then light only the target lamp.

Ability: causal discovery. Hidden variable: the wiring. Three latching push-button
switches (1, 2, 3) drive three lamps (A, B, C) like two-way stair switches: a lamp is
lit iff an odd number of the switches wired to it are on. One switch is wired to two
lamps, the other two to one lamp each, and every lamp is wired to something. The
target lamp (on the red square) can only be lit alone by combining two or more
switches, so a single-switch guess never works.

An observation is the switch state and lamp state held unchanged for 1 s. The scorer
keeps every wiring consistent with all observations, out of all 512 ways three switches
could each toggle any subset of the lamps (no use of the one-switch-two-lamps hint);
science is proven when exactly one remains, i.e. every switch's effect is pinned down.

Hard: four switches and four lamps. Three lamps are two-way (XOR) lamps as in easy; one
lamp is an AND lamp, lit only while BOTH of its two switches are on. The target is a
two-lamp pattern that the easy method's toggle model gets wrong. Turning the switches on
one at a time in order (the easy method) cannot tell "AND of i and j" from "toggled by j",
so it leaves the wiring ambiguous and sets the wrong pattern. The scorer keeps, per lamp, every
function in {XOR of any subset of switches} plus {AND of any two switches} (22 per lamp)
consistent with the observations; science needs one per lamp.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import product
from typing import Any

import mujoco
import numpy as np

from robosci import props
from robosci.motion import Motion
from robosci.skills import noop
from robosci.specs import TaskSpec

Y0 = -0.305
SW = ("1", "2", "3")
LAMPS = ("A", "B", "C")
DY = (0.10, 0.0, -0.10)
BUTTONS = tuple(props.ButtonSpec(f"sw{s}", (0.25, Y0 + dy), rgba=(0.2, 0.2, 0.22, 1)) for s, dy in zip(SW, DY))
LAMP_XY = tuple((0.42, Y0 + dy) for dy in DY)

INSTRUCTION = (
    "Three switches (1, 2, 3) control three lamps (A, B, C), but the wiring is unknown and one "
    "switch controls two lamps. Each press toggles a switch. Find out how they are wired, then "
    "leave the switches so that only the lamp on the red square is lit."
)


# ---- hard: per-lamp functions of the switch state and of the order switches came on ----
HSW = ("1", "2", "3", "4")
HLAMPS = ("A", "B", "C", "D")
HDY = (0.15, 0.05, -0.05, -0.15)
HBUTTONS = tuple(props.ButtonSpec(f"sw{s}", (0.25, Y0 + dy), rgba=(0.2, 0.2, 0.22, 1)) for s, dy in zip(HSW, HDY))
HLAMP_XY = tuple((0.42, Y0 + dy) for dy in HDY)
# A lamp function: ("x", frozenset S)  toggled by every switch in S (lit iff an odd number of S on)
#                  ("a", (p, q))       lit while p AND q are on
#                  ("i", (p, q))       lit while p is on and q is off (q blocks p)
#                  ("s", (p, q))       lit while both are on AND p came on before q (order lamp)
Func = tuple
FUNCS: tuple[Func, ...] = (
    tuple(("x", frozenset(j for j in range(4) if m >> j & 1)) for m in range(16))
    + tuple(("a", (i, j)) for i in range(4) for j in range(i + 1, 4))
    + tuple(("i", (i, j)) for i in range(4) for j in range(4) if i != j)
    + tuple(("s", (i, j)) for i in range(4) for j in range(4) if i != j))
HSTATES = tuple(product((0, 1), repeat=4))
Cfg = tuple  # (state tuple, frozenset of ordered pairs (p, q): q came on while p was on, both still on)
START: Cfg = ((0, 0, 0, 0), frozenset())


def press(cfg: Cfg, i: int) -> Cfg:
    st, latch = cfg
    st2 = tuple(1 - x if k == i else x for k, x in enumerate(st))
    if st2[i]:
        latch = latch | {(p, i) for p in range(4) if p != i and st2[p]}
    else:
        latch = frozenset(pq for pq in latch if i not in pq)
    return st2, frozenset(latch)


def f_eval(f: Func, cfg: Cfg) -> int:
    st, latch = cfg
    kind, arg = f
    if kind == "x":
        return sum(st[i] for i in arg) % 2
    p, q = arg
    if kind == "a":
        return int(st[p] and st[q])
    if kind == "i":
        return int(st[p] and not st[q])
    return int((p, q) in latch)


def h_lamps(w: tuple[Func, ...], cfg: Cfg) -> tuple[int, ...]:
    return tuple(f_eval(f, cfg) for f in w)


def reachable(start: Cfg, depth: int = 6) -> dict[Cfg, tuple[int, ...]]:
    """Every configuration reachable within ``depth`` presses, with a shortest press list."""
    seen = {start: ()}
    frontier = [start]
    for _ in range(depth):
        nxt = []
        for c in frontier:
            for i in range(4):
                c2 = press(c, i)
                if c2 not in seen:
                    seen[c2] = seen[c] + (i,)
                    nxt.append(c2)
        frontier = nxt
    return seen


def route_to(w: tuple[Func, ...], cfg: Cfg, goal: tuple[int, ...], depth: int = 6) -> tuple[int, ...] | None:
    best = None
    for c, path in reachable(cfg, depth).items():
        if h_lamps(w, c) == goal and (best is None or len(path) < len(best)):
            best = path
    return best


def _naive_hits(w: tuple[Func, ...], goal: tuple[int, ...]) -> bool:
    """Would the easy method (switch 1..4 on in turn, toggle model, then set the model's goal
    state) light exactly ``goal``? Hard targets are drawn where it would not."""
    cfg, effect = START, []
    for i in range(4):
        nxt = press(cfg, i)
        effect.append({j for j in range(4) if h_lamps(w, nxt)[j] != h_lamps(w, cfg)[j]})
        cfg = nxt
    model = tuple(("x", frozenset(i for i in range(4) if j in effect[i])) for j in range(4))
    goals = [st for st in HSTATES if tuple(f_eval(f, (st, frozenset())) for f in model) == goal]
    if not goals:
        return False
    pick = min(goals, key=lambda g: sum(x != y for x, y in zip(g, cfg[0])))
    for i in range(4):
        if pick[i] != cfg[0][i]:
            cfg = press(cfg, i)
    return h_lamps(w, cfg) == goal


def draw_hard_wiring(rng: np.random.Generator) -> tuple[tuple[Func, ...], tuple[int, ...]]:
    """One blocker lamp, one order lamp, two toggle lamps; one switch feeds 3+ lamps; every switch
    is used. Target: a two-lamp pattern with the order lamp lit, at least 2 presses from the
    start, reachable, and missed by the easy method."""
    pats = [tuple(int(i in pr) for i in range(4)) for pr in product(range(4), repeat=2) if pr[0] < pr[1]]
    while True:
        lamps = list(rng.permutation(4))
        p, q = (int(x) for x in rng.choice(4, 2, replace=False))
        u, v = (int(x) for x in rng.choice(4, 2, replace=False))
        w: list[Func] = [None] * 4  # type: ignore[list-item]
        w[lamps[0]] = ("i", (p, q))
        w[lamps[1]] = ("s", (u, v))
        for j in lamps[2:]:
            k = 1 if rng.random() < 0.5 else 2
            w[j] = ("x", frozenset(int(x) for x in rng.choice(4, k, replace=False)))
        w = tuple(w)
        used = set()
        count = [0] * 4
        for f in w:
            sw = f[1] if f[0] == "x" else set(f[1])
            used |= set(sw)
            for i in sw:
                count[i] += 1
        if used != {0, 1, 2, 3} or max(count) < 3 or len(set(w)) < 4:
            continue
        ok = []
        for g in pats:
            if not g[lamps[1]]:
                continue
            r = route_to(w, START, g)
            if r is not None and len(r) >= 2 and not _naive_hits(w, g):
                ok.append(g)
        if ok:
            return w, ok[int(rng.integers(len(ok)))]


def _all_wirings() -> list[tuple[frozenset[int], ...]]:
    out = []
    for d in range(3):
        for pair in ((0, 1), (0, 2), (1, 2)):
            for a, b in product(range(3), repeat=2):
                singles = iter((a, b))
                w = tuple(frozenset(pair) if i == d else frozenset({next(singles)}) for i in range(3))
                if set().union(*w) == {0, 1, 2}:
                    out.append(w)
    return out


WIRINGS = _all_wirings()  # the episode's wiring is drawn from these
# The scorer assumes nothing about the wiring: any switch may toggle any subset of lamps.
ANY_WIRING = [tuple(frozenset(j for j in range(3) if m >> (3 * i + j) & 1) for i in range(3))
              for m in range(512)]


def lamps_for(w: tuple[frozenset[int], ...], state: tuple[int, ...]) -> tuple[int, ...]:
    return tuple(sum(state[i] for i in range(3) if j in w[i]) % 2 for j in range(3))


def target_states(w: tuple[frozenset[int], ...], target: int) -> list[tuple[int, ...]]:
    goal = tuple(int(j == target) for j in range(3))
    return [s for s in product((0, 1), repeat=3) if lamps_for(w, s) == goal]


@dataclass
class Obs:
    t: float
    state: tuple[int, ...]
    lamps: tuple[int, ...]
    order: frozenset = frozenset()  # hard: ordered pairs (p, q) where q came on while p was on


@dataclass
class SwitchLampMap:
    difficulty: str = "easy"
    key: str = "switch_lamp_map"
    instruction: str = INSTRUCTION
    obs: list[Obs] = field(default_factory=list)

    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        if self.difficulty != "easy":
            self._build_hard(spec, rng)
            return
        self.n, self.bspecs = 3, BUTTONS
        while True:
            w = WIRINGS[int(rng.integers(len(WIRINGS)))]
            ok = [j for j in range(3) if target_states(w, j)
                  and all(sum(s) >= 2 for s in target_states(w, j))]
            if ok:
                break
        self.wiring = w
        self.target = int(rng.choice(ok))
        for b, s in zip(BUTTONS, SW):
            props.push_button(spec, b)
            props.text_plate(spec, f"lbl_sw{s}", (b.pos[0] - 0.055, b.pos[1]), s, half=(0.018, 0.018))
            props.lamp(spec, f"led{s}", (b.pos[0] + 0.045, b.pos[1], 0), radius=0.008,
                       rgba_on=(0.2, 1.0, 0.3, 1), post=0.015)
        tx, ty = LAMP_XY[self.target]
        props.flat(spec, "target", (tx, ty), (0.04, 0.04), props.RED)
        for name, xy in zip(LAMPS, LAMP_XY):
            props.lamp(spec, f"lamp{name}", xy, radius=0.022, post=0.06)
            props.text_plate(spec, f"lbl_{name}", (xy[0] + 0.07, xy[1]), name, half=(0.018, 0.018))

    def _build_hard(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        self.n, self.bspecs = 4, HBUTTONS
        self.wiring, self.goal = draw_hard_wiring(rng)
        for b, s in zip(HBUTTONS, HSW):
            props.push_button(spec, b)
            props.text_plate(spec, f"lbl_sw{s}", (b.pos[0] - 0.055, b.pos[1]), s, half=(0.018, 0.018))
            props.lamp(spec, f"led{s}", (b.pos[0] + 0.045, b.pos[1], 0), radius=0.008,
                       rgba_on=(0.2, 1.0, 0.3, 1), post=0.015)
        for j, (name, xy) in enumerate(zip(HLAMPS, HLAMP_XY)):
            if self.goal[j]:
                props.flat(spec, f"target{name}", xy, (0.04, 0.04), props.RED)
            props.lamp(spec, f"lamp{name}", xy, radius=0.022, post=0.06)
            props.text_plate(spec, f"lbl_{name}", (xy[0] + 0.07, xy[1]), name, half=(0.018, 0.018))

    @property
    def hard(self) -> bool:
        return self.difficulty != "easy"

    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        sw, lamps = (HSW, HLAMPS) if self.hard else (SW, LAMPS)
        self.buttons = [props.Button(b) for b in self.bspecs]
        for b in self.buttons:
            b.bind(model)
        self.leds = [props.Lamp(f"led{s}", (0.2, 1.0, 0.3, 1)) for s in sw]
        self.lamps = [props.Lamp(f"lamp{n}") for n in lamps]
        for x in self.leds + self.lamps:
            x.bind(model)
        self.touch = props.TouchTracker(tuple(b.name for b in self.bspecs))
        self.touch.bind(model, data)
        self.state = [0] * self.n
        self.cfg: Cfg = START
        self.obs = []
        self._since = 0.0

    def lamp_state(self) -> tuple[int, ...]:
        if self.hard:
            return h_lamps(self.wiring, self.cfg)
        return lamps_for(self.wiring, tuple(self.state))

    def lamp_hypotheses(self) -> list[list[Func]]:
        """Hard: per lamp, the functions consistent with every observation."""
        return [[f for f in FUNCS if all(f_eval(f, (o.state, o.order)) == o.lamps[j] for o in self.obs)]
                for j in range(4)]

    def goal_state(self) -> tuple[int, ...]:
        return self.goal if self.hard else tuple(int(j == self.target) for j in range(3))

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        t = float(data.time)
        self.touch.update(data)
        changed = False
        for i, b in enumerate(self.buttons):
            if b.update(data):
                self.state[i] ^= 1
                if self.hard:
                    self.cfg = press(self.cfg, i)
                changed = True
                self.leds[i].set(model, bool(self.state[i]))
        if changed:
            for lamp, on in zip(self.lamps, self.lamp_state()):
                lamp.set(model, bool(on))
            self._since = t
        st = tuple(self.state)
        order = self.cfg[1] if self.hard else frozenset()
        if t - self._since >= 1.0 and not any(b.pressed for b in self.buttons) \
                and all((o.state, o.order) != (st, order) for o in self.obs):
            self.obs.append(Obs(t, st, self.lamp_state(), order))

    def hypotheses(self) -> list[tuple[frozenset[int], ...]]:
        return [w for w in ANY_WIRING if all(lamps_for(w, o.state) == o.lamps for o in self.obs)]

    def hud(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
        sc = self.score(model, data)
        names = HLAMPS if self.hard else LAMPS
        lit = "".join(n for n, on in zip(names, self.lamp_state()) if on) or "none"
        total = len(FUNCS) ** 4 if self.hard else len(ANY_WIRING)
        lines = [f"t = {data.time:5.1f} s   switches {''.join(map(str, self.state))}   lit {lit}   "
                 f"target {sc['target']}",
                 f"rubric stage {sc['stage']}/4   observations {len(self.obs)}   wirings left "
                 f"{sc['wirings_left']}/{total}   presses {sc['presses']}"]
        for o in self.obs[-3:]:
            lines.append(f"  switches {''.join(map(str, o.state))} -> lit "
                         f"{''.join(n for n, on in zip(names, o.lamps) if on) or 'none'}")
        return lines

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        success = self.lamp_state() == self.goal_state()
        if self.hard:
            per = self.lamp_hypotheses()
            n_left = int(np.prod([len(x) for x in per]))
        else:
            n_left = len(self.hypotheses())
        stage = 0
        if self.touch.touched:
            stage = 1
        if any(sum(o.state) for o in self.obs):
            stage = 2
        if n_left == 1:
            stage = 3
            if success:
                stage = 4
        return {
            "stage": stage,
            "success": float(success),
            "science_success": float(stage >= 3),
            "observations": len(self.obs),
            "wirings_left": n_left,
            "presses": sum(b.presses for b in self.buttons),
            "procedure_violations": 0,
            "target": ("".join(n for n, g in zip(HLAMPS, self.goal) if g) if self.hard
                       else LAMPS[self.target]),
            "wiring": ({HLAMPS[j]: _fname(f) for j, f in enumerate(self.wiring)} if self.hard else
                       {SW[i]: "".join(LAMPS[j] for j in sorted(self.wiring[i])) for i in range(3)}),
            "log": [{**o.__dict__, "order": sorted(o.order)} for o in self.obs],
        }


def _fname(f: Func) -> str:
    if f[0] == "x":
        return "toggled by " + "+".join(HSW[i] for i in sorted(f[1]))
    p, q = HSW[f[1][0]], HSW[f[1][1]]
    return {"a": f"{p} AND {q}", "i": f"{p} unless {q}", "s": f"{p} then {q}"}[f[0]]


# ---------- reference policies ----------
def _press(mo: Motion, t: SwitchLampMap, i: int):  # noqa: ANN202
    side = "left" if t.bspecs[i].pos[1] >= Y0 else "right"
    yield from mo.press(side, t.buttons[i].top(), depth=t.bspecs[i].travel + 0.004)
    yield from mo.hold(1.5)


def _one_at_a_time(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Turn the switches on one at a time (states 100, 110, 111), watching the lamps after
    each; the three differences give each switch's effect. Then set the switches to the
    combination that lights only the target lamp."""
    t: SwitchLampMap = env.task
    if t.hard:
        yield from _one_at_a_time_hard(env, rng)
        return
    mo = Motion(env)
    yield from mo.hold(1.2)
    for i in range(3):
        yield from _press(mo, t, i)
    hs = t.hypotheses()
    if len(hs) == 1:
        goal = min(target_states(hs[0], t.target), key=lambda s: sum(a != b for a, b in zip(s, t.state)))
        for i in range(3):
            if goal[i] != t.state[i]:
                yield from _press(mo, t, i)
    for s in ("left", "right"):
        yield from mo.home(s)
    yield from mo.hold(1.0)


def _one_at_a_time_hard(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """The easy method on the hard board: switch 1..4 on in turn, take each switch's effect to
    be the lamps that changed (a toggle model), then set the switches that model predicts."""
    t: SwitchLampMap = env.task
    mo = Motion(env)
    yield from mo.hold(1.2)
    effect = []
    for i in range(t.n):
        before = t.lamp_state()
        yield from _press(mo, t, i)
        effect.append(frozenset(j for j in range(4) if t.lamp_state()[j] != before[j]))
    model = tuple(("x", frozenset(i for i in range(4) if j in effect[i])) for j in range(4))
    goals = [st for st in HSTATES if tuple(f_eval(f, (st, frozenset())) for f in model) == t.goal]
    if goals:
        goal = min(goals, key=lambda st: sum(x != y for x, y in zip(st, t.state)))
        for i in range(4):
            if goal[i] != t.state[i]:
                yield from _press(mo, t, i)
    for side in ("left", "right"):
        yield from mo.home(side)
    yield from mo.hold(1.0)


def _worst(per: list[list[Func]], cfgs: list[Cfg]) -> int:
    """Worst-case product of per-lamp hypothesis counts after observing every config in ``cfgs``."""
    out = 1
    for fs in per:
        groups: dict[tuple[int, ...], int] = {}
        for f in fs:
            k = tuple(f_eval(f, c) for c in cfgs)
            groups[k] = groups.get(k, 0) + 1
        out *= max(groups.values())
    return out


def _greedy_hard(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Hard: among press sequences of up to 3 presses, run the one whose observed configurations
    (each intermediate one is observed too) most shrink the worst-case number of wirings left;
    ties go to fewer presses. Once one function per lamp is left, press the shortest route to the
    target pattern under that wiring."""
    t: SwitchLampMap = env.task
    mo = Motion(env)
    yield from mo.hold(1.2)
    seqs = [s for n in (1, 2, 3) for s in product(range(4), repeat=n)]
    for _ in range(30):
        per = t.lamp_hypotheses()
        if all(len(x) == 1 for x in per):
            break
        seen = {(o.state, o.order) for o in t.obs}
        best = None
        for sq in seqs:
            c, cfgs = t.cfg, []
            for i in sq:
                c = press(c, i)
                cfgs.append(c)
            if all(x in seen for x in cfgs):
                continue
            key = (_worst(per, cfgs), len(sq))
            if best is None or key < best[0]:
                best = (key, sq)
        if best is None:
            break
        for i in best[1]:
            yield from _press(mo, t, i)
    per = t.lamp_hypotheses()
    if all(len(x) == 1 for x in per):
        route = route_to(tuple(x[0] for x in per), t.cfg, t.goal)
        for i in route or ():
            yield from _press(mo, t, i)
    for side in ("left", "right"):
        yield from mo.home(side)
    yield from mo.hold(1.0)


def oracle(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Easy: one switch at a time. Hard: greedy most-informative switch states."""
    pol = _greedy_hard if env.task.hard else _one_at_a_time
    yield from pol(env, rng)


def guess(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Blind baseline: press two random switches and stop."""
    rng = rng or np.random.default_rng()
    t: SwitchLampMap = env.task
    mo = Motion(env)
    for i in sorted(rng.choice(t.n, size=2, replace=False)):
        yield from _press(mo, t, int(i))
    yield from mo.hold(1.0)


TASK = SwitchLampMap
POLICIES = {"oracle": oracle, "guess": guess, "noop": noop, "naive": _one_at_a_time}
SPEC = TaskSpec(
    key="switch_lamp_map",
    title="Map switches to lamps, light only the target",
    ability="Causal discovery",
    instruction=INSTRUCTION,
    objects=(
        "three latching push-button switches labelled 1, 2, 3, each with a green ON light (hard: four, 1-4)",
        "three lamps labelled A, B, C on posts (hard: four, A-D)",
        "red square under the target lamp (hard: under each of the two target lamps)",
    ),
    setup=(
        "Switches in a row 25 cm in front of the arms, 10 cm apart, centred; all off.",
        "Hard: four switches and four lamps, same spacing, so the row spans 30 cm.",
        "Lamps in a row 42 cm in front, aligned with the switches.",
        "Put the red square under the target lamp for this episode.",
    ),
    hidden=(
        "The wiring: one switch toggles two lamps, the others one lamp each, every lamp wired "
        "(lamp lit iff an odd number of its switches are on). The target lamp can only be lit "
        "alone by combining at least two switches."
    ),
    instruction_hard=(
        "Four switches (1 to 4) control four lamps (A to D) through unknown wiring. A switch can affect "
        "several lamps, a lamp can depend on a combination of switches, one switch can block another, and "
        "for some lamps the order in which switches are turned on matters. Each press toggles a switch. "
        "Find out how the switches control the lamps, then leave the switches so that exactly the lamps "
        "on the red squares are lit."
    ),
    hidden_hard=(
        "Per trial, each lamp gets one rule: two lamps are toggle lamps (lit iff an odd number of their 1 "
        "or 2 switches are on), one is a blocker lamp (lit while switch p is on and switch q is off), and "
        "one is an order lamp (lit while p and q are both on, but only if p came on before q). One switch "
        "feeds at least three lamps and every switch is used. The target is two lamps including the order "
        "lamp, at least two presses from the start, and missed by switching 1, 2, 3, 4 on in turn and "
        "reading each switch's effect as a toggle."
    ),
    rubric=(
        "No purposeful contact with the switches",
        "Presses a switch",
        "At least one observation with a switch on (state held 1 s)",
        "Observations pin down the wiring (easy: one of 512 switch-to-lamp maps left; hard: one rule per "
        "lamp left out of 46: 16 toggle sets, 6 AND pairs, 12 blocker pairs, 12 order pairs)",
        "Stage 3 and only the target lamp(s) lit at the end",
    ),
    metrics=(
        "stage (0-4)",
        "success: only the target lamp lit, mapping known or not",
        "science_success: stage >= 3",
        "observations and presses used; wirings left out of 512",
    ),
    report_fields=("observations", "wirings_left", "presses", "target"),
    max_seconds=300.0,
    limitations="Wiring is a hidden table evaluated in code at each press.",
)
