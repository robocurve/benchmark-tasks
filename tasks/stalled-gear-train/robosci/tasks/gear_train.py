"""Task: find the stripped gear in a gear train, replace it, bin it.

Ability: fault diagnosis. Hidden variable: which installed gear is stripped (easy). Hard:
the installed fault is a seized gear, which stalls the motor so nothing turns (one run
does not localise it), and one spare is stripped; the gear is found by swapping
replacements in one seat at a time. Four gears G1-G4 sit on seats in a row between a
motor and an output flag. Pressing RUN drives the motor for 4 s. A gear on a seat turns
if the gear before it drives it; a stripped gear turns but does not drive the next one,
so the train stops after it and the flag stays still.

Rotation is kinematic (set in code each monitor tick); gears are free bodies that can
be lifted off and swapped. A test is a RUN with all four seats filled; the observed
pattern of turning gears is recorded and the scorer keeps the fault hypotheses
consistent with every test.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import product
from typing import Any

import mujoco
import numpy as np

from robosci import props
from robosci.motion import Motion
from robosci.skills import drop_point, grasp_point, move_body, noop
from robosci.specs import TaskSpec

Y0 = -0.305
BOARD_X = 0.32
BOARD_TOP = 0.006
PITCH = 0.062
SEAT_XY = tuple((BOARD_X, Y0 + PITCH * (1.5 - i)) for i in range(4))
MOTOR_XY = (BOARD_X, Y0 + PITCH * 2.5)
OUT_XY = (BOARD_X, Y0 - PITCH * 2.5)
R, HT = 0.029, 0.006  # gear radius, half thickness
KNOB = 0.016
INSTALLED = ("G1", "G2", "G3", "G4")
SPARES = ("S1", "S2")
GEARS = INSTALLED + SPARES
SPARE_XY = {"S1": (0.20, Y0 + 0.11), "S2": (0.20, Y0 - 0.11)}
PARK = ((0.22, -0.03), (0.22, -0.58))
BUTTON = props.ButtonSpec("run", (0.20, Y0), rgba=(0.1, 0.6, 0.2, 1))
BIN_XY, BIN_HALF = (0.40, 0.08), (0.07, 0.07)
SEAT_TOL = 0.012
RUN_S = 4.0
OMEGA = 2.0
GRIP, OPEN, DZ = 0.0, 0.03, 0.016

INSTRUCTION = (
    "Pressing RUN should turn every gear and the flag at the end, but one gear is stripped. "
    "Spare gears S1 and S2 are on the table. Find the stripped gear, replace it so the flag "
    "turns, and put the stripped gear in the red bin."
)


@dataclass
class Run:
    t: float
    config: tuple[str, ...]
    turning: tuple[bool, ...]  # four seats + output flag


def predict(config: tuple[str, ...], bad: frozenset[str], seized: frozenset[str] = frozenset()) -> tuple[bool, ...]:
    """Turning pattern (4 seats + flag) for a full train. ``bad`` = stripped gears (turn but do
    not drive the next); ``seized`` = jammed gears: if one is in the driven chain, the motor
    stalls and nothing turns."""
    return pattern(config, bad, seized, running=True)


def pattern(config, stripped, seized, running: bool) -> tuple[bool, ...]:  # noqa: ANN001
    out, driven = [], running
    for g in config:
        out.append(driven and g is not None)
        driven = driven and g is not None and g not in stripped
    if any(o and g in seized for o, g in zip(out, config)):
        return (False,) * (len(config) + 1)
    return tuple(out) + (driven,)


@dataclass
class GearTrain:
    difficulty: str = "easy"
    key: str = "gear_train"
    instruction: str = INSTRUCTION
    bad: set[str] = field(default_factory=set)
    runs: list[Run] = field(default_factory=list)

    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        self.bad_installed = str(rng.choice(INSTALLED))
        self.bad = {self.bad_installed}
        self.seized: set[str] = set()
        if self.difficulty == "hard":
            # Hard: the installed fault is a seized gear (stalls the whole train, so one run
            # shows nothing turning and does not say which), and one spare is stripped.
            self.bad.add(str(rng.choice(SPARES)))
            self.seized = {self.bad_installed}
        w = spec.worldbody
        w.add_geom(name="board", type=props.G.mjGEOM_BOX, size=[0.05, 0.2, BOARD_TOP / 2],
                   pos=[BOARD_X, Y0, BOARD_TOP / 2], rgba=[0.30, 0.30, 0.33, 1])
        for i, (x, y) in enumerate(SEAT_XY):
            w.add_geom(type=props.G.mjGEOM_CYLINDER, size=[0.004, 0.012], pos=[x, y, BOARD_TOP + 0.012],
                       rgba=[0.6, 0.6, 0.62, 1], contype=0, conaffinity=0)
            props.text_plate(spec, f"lbl_seat{i}", (x - 0.06, y), str(i + 1), half=(0.012, 0.012))
        # Motor with its own drive gear, and the output flag (both hinge-driven in code).
        mot = w.add_body(name="motor", pos=[MOTOR_XY[0], MOTOR_XY[1], 0])
        mot.add_geom(type=props.G.mjGEOM_BOX, size=[0.03, 0.025, 0.02], pos=[0, 0.02, 0.02],
                     rgba=list(props.DARK), contype=0, conaffinity=0)
        drv = mot.add_body(name="drive", pos=[0, 0, 0.046])
        drv.add_joint(name="drive_hinge", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1])
        self._gear_geoms(drv, "drive", (0.4, 0.4, 0.42, 1), collide=False)
        out = w.add_body(name="output", pos=[OUT_XY[0], OUT_XY[1], BOARD_TOP])
        out.add_geom(type=props.G.mjGEOM_CYLINDER, size=[0.004, 0.05], pos=[0, 0, 0.05],
                     rgba=[0.6, 0.6, 0.62, 1], contype=0, conaffinity=0)
        flag = out.add_body(name="flag", pos=[0, 0, 0.09])
        flag.add_joint(name="flag_hinge", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1])
        flag.add_geom(type=props.G.mjGEOM_BOX, size=[0.03, 0.002, 0.015], pos=[0.03, 0, 0],
                      rgba=list(props.RED), contype=0, conaffinity=0)
        self._gear_geoms(out, "outgear", (0.4, 0.4, 0.42, 1), collide=False, z=HT)
        props.push_button(spec, BUTTON)
        props.text_plate(spec, "lbl_run", (BUTTON.pos[0], BUTTON.pos[1] - 0.05), "RUN", half=(0.02, 0.02))
        props.tray(spec, "bin", BIN_XY, BIN_HALF, rgba=props.RED)
        for g in GEARS:
            xy = SEAT_XY[INSTALLED.index(g)] if g in INSTALLED else SPARE_XY[g]
            z = (BOARD_TOP if g in INSTALLED else 0.0) + HT + 0.0005
            b = w.add_body(name=f"gear_{g}", pos=[xy[0], xy[1], z])
            b.add_freejoint(name=f"gear_{g}_free")
            self._gear_geoms(b, f"gear_{g}", (0.80, 0.65, 0.30, 1), collide=True)
            mat = props.labelled_material(spec, f"mat_gear_{g}", g, bg=(235, 235, 230))
            b.add_geom(name=f"gear_{g}_knob", type=props.G.mjGEOM_BOX, size=[KNOB] * 3,
                       pos=[0, 0, HT + KNOB], mass=0.01, material=mat, friction=[1.0, 0.01, 0.001], condim=4)

    @staticmethod
    def _gear_geoms(b, name: str, rgba, collide: bool, z: float = 0.0) -> None:  # noqa: ANN001
        kw = {} if collide else {"contype": 0, "conaffinity": 0}
        b.add_geom(name=name, type=props.G.mjGEOM_CYLINDER, size=[R - 0.004, HT], pos=[0, 0, z], mass=0.02,
                   rgba=list(rgba), friction=[1.0, 0.01, 0.001], condim=4, **kw)
        for k in range(10):
            a = 2 * np.pi * k / 10
            b.add_geom(type=props.G.mjGEOM_BOX, size=[0.004, 0.003, HT * 0.9],
                       pos=[(R - 0.002) * np.cos(a), (R - 0.002) * np.sin(a), z],
                       quat=[np.cos(a / 2), 0, 0, np.sin(a / 2)], rgba=list(rgba) if k else [0.1, 0.1, 0.1, 1],
                       mass=0.0005, contype=0, conaffinity=0)

    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._m = model
        self.button = props.Button(BUTTON)
        self.button.bind(model)
        self.touch = props.TouchTracker(tuple(f"gear_{g}" for g in GEARS) + ("run",))
        self.touch.bind(model, data)
        self._tips = props.gripper_geoms(model)
        self._ggeom = {}
        for g in GEARS:
            bid = model.body(f"gear_{g}").id
            for gi in range(model.ngeom):
                if model.geom_bodyid[gi] == bid:
                    self._ggeom[gi] = g
        self._jq = {g: model.joint(f"gear_{g}_free").qposadr[0] for g in GEARS}
        self._jv = {g: model.joint(f"gear_{g}_free").dofadr[0] for g in GEARS}
        self._drive = model.joint("drive_hinge").qposadr[0]
        self._flag = model.joint("flag_hinge").qposadr[0]
        self.runs = []
        self._run_until = -1.0
        self._run_start = 0.0
        self._recorded = True
        self._last_t = float(data.time)
        self.turning: tuple[bool, ...] = (False,) * 5

    def pos(self, data: mujoco.MjData, g: str) -> np.ndarray:
        return data.xpos[self._m.body(f"gear_{g}").id]

    def held(self, data: mujoco.MjData) -> set[str]:
        return {self._ggeom[x] for x in props.held_geoms(self._m, data, self._tips) if x in self._ggeom}

    def config(self, data: mujoco.MjData) -> tuple[str | None, ...]:
        held = self.held(data)
        out: list[str | None] = []
        for xy in SEAT_XY:
            here = [g for g in GEARS if g not in held and props.in_box(
                self.pos(data, g), xy, (SEAT_TOL, SEAT_TOL), z_max=BOARD_TOP + HT + 0.006)]
            out.append(here[0] if len(here) == 1 else None)
        return tuple(out)

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        t = float(data.time)
        dt, self._last_t = t - self._last_t, t
        self.touch.update(data)
        if self.button.update(data):
            self._run_until, self._run_start, self._recorded = t + RUN_S, t, False
        cfg = self.config(data)
        running = t < self._run_until
        self.turning = pattern(cfg, self.bad - self.seized, self.seized, running)
        stalled = running and not any(self.turning[:4]) and cfg[0] is not None
        if running and not stalled:
            data.qpos[self._drive] += OMEGA * dt
            data.qpos[self._flag] -= OMEGA * dt if self.turning[4] else 0.0
            for i, g in enumerate(cfg):
                if g is not None and self.turning[i]:
                    sgn = -1.0 if i % 2 == 0 else 1.0
                    q = data.qpos[self._jq[g] + 3: self._jq[g] + 7]
                    dq = np.array([np.cos(sgn * OMEGA * dt / 2), 0, 0, np.sin(sgn * OMEGA * dt / 2)])
                    out = np.zeros(4)
                    mujoco.mju_mulQuat(out, dq, q.copy())
                    data.qpos[self._jq[g] + 3: self._jq[g] + 7] = out
                    data.qvel[self._jv[g] + 3: self._jv[g] + 6] = 0.0
        if running and not self._recorded and t - self._run_start >= 1.5 and all(c is not None for c in cfg):
            self.runs.append(Run(t, tuple(cfg), self.turning))  # type: ignore[arg-type]
            self._recorded = True

    def hypotheses(self) -> list[frozenset[str]]:
        if self.difficulty == "easy":
            return [h for h in (frozenset({d}) for d in INSTALLED)
                    if all(predict(r.config, h) == r.turning for r in self.runs)]
        # Hard: h = {seized installed gear, stripped spare}.
        return [frozenset({d, e}) for d, e in product(INSTALLED, SPARES)
                if all(predict(r.config, frozenset({e}), frozenset({d})) == r.turning for r in self.runs)]

    def proven_bad(self) -> str | None:
        names = {next(iter(h & set(INSTALLED))) for h in self.hypotheses()}
        return names.pop() if len(names) == 1 and self.runs else None

    def in_bin(self, data: mujoco.MjData) -> set[str]:
        held = self.held(data)
        return {g for g in GEARS if g not in held and props.in_box(self.pos(data, g), BIN_XY, BIN_HALF, z_max=0.08)}

    def hud(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
        sc = self.score(model, data)
        cfg = self.config(data)
        spin = " ".join(f"{c or '--'}{'*' if s else ''}" for c, s in zip(cfg, self.turning))
        lines = [f"t = {data.time:5.1f} s   motor {'RUN' if data.time < self._run_until else 'off'}   "
                 f"seats {spin}   flag {'TURNS' if self.turning[4] else 'still'}   (* = turning)",
                 f"rubric stage {sc['stage']}/4   runs {sc['runs']}   fault hypotheses left "
                 f"{sc['hypotheses_left']}   bin {sorted(sc['in_bin']) or '-'}"]
        for r in self.runs[-3:]:
            lines.append("  run " + " ".join(f"{g}{'*' if s else ''}" for g, s in zip(r.config, r.turning))
                         + f"  flag {'turns' if r.turning[4] else 'still'}")
        return lines

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        cfg = self.config(data)
        binned = self.in_bin(data)
        good_train = all(c is not None and c not in self.bad for c in cfg)
        success = good_train and self.bad_installed in binned and not (binned - self.bad)
        stage = 0
        if self.touch.touched or self.button.presses:
            stage = 1
        if self.runs:
            stage = 2
        if self.proven_bad() == self.bad_installed:
            stage = 3
            if success:
                stage = 4
        return {
            "stage": stage,
            "success": float(success),
            "science_success": float(stage >= 3),
            "runs": len(self.runs),
            "hypotheses_left": len(self.hypotheses()),
            "procedure_violations": len(binned - self.bad),
            "bad_installed": self.bad_installed,
            "bad": sorted(self.bad),
            "in_bin": sorted(binned),
            "log": [r.__dict__ for r in self.runs],
        }


# ---------- reference policies ----------
def _carry(env, mo: Motion, g: str, xy, on_board: bool):  # noqa: ANN001, ANN202
    """Prefer the arm that can do the carry with the least tilted grasp: the knob is small,
    and a tilted grasp lets the gear swing and land off its seat."""
    side = "left" if xy[1] > Y0 else "right"
    grasp = grasp_point(env, f"gear_{g}", DZ)
    target = drop_point(xy, BOARD_TOP if on_board else 0.0, HT)
    plans = {a: mo.plan_pick_place(a, grasp, env.task.pos(env.data, g).copy(), target) for a in ("left", "right")}
    ok = {a: abs(p[1]) for a, p in plans.items() if p is not None}
    if ok:
        side = min(ok, key=lambda a: (ok[a], a != side))
    return move_body(env, mo, f"gear_{g}", side, xy, BOARD_TOP if on_board else 0.0, half=HT,
                     grip=GRIP, dz=DZ, open_to=OPEN)


def _run(mo: Motion, t: GearTrain):  # noqa: ANN202
    yield from mo.press("left", t.button.top(), depth=BUTTON.travel + 0.004)
    yield from mo.home("left")
    yield from mo.hold(0.5)


def _last_turning(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Run the train and watch it: the last gear that turns is stripped (it turns but does
    not drive the next). Bin it, seat an untried spare, run again; repeat until the flag
    turns. (If nothing turns, it blames the first gear.)"""
    t: GearTrain = env.task
    mo = Motion(env)
    spares = list(SPARES)
    for _ in range(4):
        yield from _run(mo, t)
        if not t.runs:
            break
        last = t.runs[-1]
        if last.turning[4] or not spares:
            break
        k = max((i for i in range(4) if last.turning[i]), default=0)
        yield from _carry(env, mo, last.config[k], BIN_XY, on_board=False)
        sp = spares.pop(0)
        yield from _seat(env, mo, t, sp, k)
    yield from mo.hold(1.0)


def _seat(env, mo: Motion, t: "GearTrain", g: str, k: int):  # noqa: ANN001, ANN202
    for _try in range(3):  # look: is the gear seated? if not, set it again
        yield from _carry(env, mo, g, SEAT_XY[k], on_board=True)
        yield from mo.hold(0.5)
        if t.config(env.data)[k] == g:
            break


def _swap_search(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Hard: nothing turns, so some gear is seized. Keep the set of (seized gear, stripped
    spare) hypotheses that fit every run. While more than one gear could be seized, take a
    candidate off its seat, put a replacement in (an installed gear already cleared, else a
    spare), and run. Once one gear is left: bin it, make sure no stripped spare sits in the
    train (swap a cleared installed gear in for the last turning gear), and run again."""
    t: GearTrain = env.task
    mo = Motion(env)
    free = list(PARK)  # table spots for gears taken off the board

    def seized_cands() -> set[str]:
        return {next(iter(h & set(INSTALLED))) for h in t.hypotheses()}

    def stripped_cands() -> set[str]:
        return {next(iter(h & set(SPARES))) for h in t.hypotheses()}

    def off_board() -> list[str]:
        cfg = t.config(env.data)
        return [g for g in GEARS if g not in cfg and not props.in_box(t.pos(env.data, g), BIN_XY, BIN_HALF)]

    def replacement(avoid: set[str]) -> str | None:
        cands = seized_cands()
        safe = [g for g in off_board() if g in INSTALLED and g not in cands and g not in avoid]
        if safe:
            return safe[0]
        spares = [g for g in off_board() if g in SPARES and g not in avoid]
        spares.sort(key=lambda g: g in stripped_cands())  # a spare not suspected of being stripped first
        return spares[0] if spares else None

    def swap(k: int, rep: str):
        out = t.config(env.data)[k]
        spot_rep = t.pos(env.data, rep)[:2].copy()
        yield from _carry(env, mo, out, free.pop(0), on_board=False)
        yield from _seat(env, mo, t, rep, k)
        free.append((float(spot_rep[0]), float(spot_rep[1])))
        yield from _run(mo, t)

    yield from _run(mo, t)
    for _ in range(8):
        cands = seized_cands()
        cfg = t.config(env.data)
        if len(cands) <= 1:
            break
        sc = stripped_cands()
        if len(sc) == 1 and next(iter(sc)) in cfg:  # a known stripped spare blocks the train: replace it
            g = next(iter(sc))
            rep = replacement({g})
            if rep is None:
                break
            yield from swap(cfg.index(g), rep)
            continue
        seats = [k for k in range(4) if cfg[k] in cands]
        rep = replacement(set())
        if not seats or rep is None:
            break
        yield from swap(seats[0], rep)
    cands = seized_cands()
    if len(cands) == 1:
        d = next(iter(cands))
        cfg = t.config(env.data)
        if d in cfg:
            rep = replacement({d})
            if rep is not None:
                yield from swap(cfg.index(d), rep)
        yield from _carry(env, mo, d, BIN_XY, on_board=False)
    for _ in range(2):
        last = t.runs[-1] if t.runs else None
        if last is None or last.turning[4]:
            break
        j = max((i for i in range(4) if last.turning[i]), default=0)
        rep = replacement({last.config[j]})
        if rep is None:
            break
        yield from swap(j, rep)
    # Stripped spare left on the table: put it in the bin too (it is faulty).
    for g in off_board():
        if g in SPARES and stripped_cands() == {g}:
            yield from _carry(env, mo, g, BIN_XY, on_board=False)
    yield from mo.hold(1.0)


def oracle(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Easy: the last turning gear is stripped. Hard: swap search for the seized gear."""
    pol = _swap_search if env.task.difficulty == "hard" else _last_turning
    yield from pol(env, rng)


def guess(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Blind baseline: bin a random gear and seat a spare in its place, without running."""
    rng = rng or np.random.default_rng()
    mo = Motion(env)
    k = int(rng.integers(4))
    yield from _carry(env, mo, INSTALLED[k], BIN_XY, on_board=False)
    yield from _carry(env, mo, str(rng.choice(SPARES)), SEAT_XY[k], on_board=True)
    yield from mo.hold(1.0)


TASK = GearTrain
POLICIES = {"oracle": oracle, "guess": guess, "noop": noop, "naive": _last_turning}
SPEC = TaskSpec(
    key="gear_train",
    title="Find and replace the stripped gear",
    ability="Fault diagnosis",
    instruction=INSTRUCTION,
    objects=(
        "gear board with a motor at one end, an output shaft with a red flag at the other, and "
        "four numbered gear seats between them",
        "gears G1-G4 (6 cm, brass, labelled knob on top) on the seats; spares S1, S2 on the table",
        "green RUN push button (motor runs 4 s per press)",
        "red bin, 14 x 14 cm",
    ),
    setup=(
        "Board 32 cm in front of the arms, centred, running across the table.",
        "Seat G1 to G4 in order from the motor end; place S1 and S2 20 cm in front, 11 cm either "
        "side of the centre line; RUN button 20 cm in front, centred.",
        "Red bin on the operator's right, 40 cm in front of the left arm's base.",
    ),
    hidden="Which installed gear is stripped (uniform over G1-G4).",
    instruction_hard=(
        "Pressing RUN should turn every gear and the flag at the end, but it does not. Spare gears S1 "
        "and S2 are on the table. Get the flag turning, and put every faulty gear you remove in the red bin."
    ),
    hidden_hard=(
        "Which installed gear is seized (uniform over G1-G4): a seized gear stalls the motor, so a run "
        "shows nothing turning and does not say which gear it is. One spare is stripped (it turns but "
        "does not drive the next gear). Blaming the first gear because nothing turns is right only "
        "1 time in 4; swapping replacements into one seat at a time finds it."
    ),
    rubric=(
        "No purposeful contact with the gears or the button",
        "Touches a gear or presses RUN",
        "At least one run with all four seats filled",
        "The recorded runs leave one possible faulty installed gear, and it is the true one (hard: the seized gear, found by runs after swaps)",
        "Stage 3, every seat holds a good gear, and the faulty installed gear (and no good gear) is in the bin",
    ),
    metrics=(
        "stage (0-4)",
        "success: train all good and the stripped installed gear binned, proof or not",
        "science_success: stage >= 3",
        "runs used; fault hypotheses left",
        "procedure violations: good gears put in the bin",
    ),
    report_fields=("runs", "hypotheses_left", "bad_installed", "in_bin"),
    max_seconds=420.0,
    limitations=("Gears do not mesh physically: rotation is set kinematically in code from which "
                 "gear sits on which seat."),
)
