"""Task: sort four look-alike cubes into "floats" and "sinks" bins by testing them in water.

Ability: property testing. Hidden variable: each cube's density (same size and look;
1 to 3 of the 4 float). Buoyancy is code-driven: inside the tank's water volume each
cube gets an upward force rho_w * V_submerged * g plus linear drag, applied
through ``data.xfrc_applied`` in ``monitor``.

A test is a cube resting in the tank, not held, still for 1 s: outcome float if its
centre is above mid-depth, sink if it lies on the tank floor.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import mujoco
import numpy as np

from robosci import props
from robosci.motion import Motion
from robosci.skills import move_body, noop
from robosci.specs import TaskSpec

LABELS = "1234"
H = 0.02  # cube half size
VOL = (2 * H) ** 3
RHO_W = 1000.0
TANK_XY = (0.34, -0.28)
TANK_HALF = (0.09, 0.09)
TANK_WALL_H = 0.08
FLOOR_Z = 0.004
WATER_Z = 0.06
ROW_X = 0.20
ROW_Y = (-0.125, -0.245, -0.365, -0.485)
BINS = {"float": ((0.22, 0.06), props.BLUE), "sink": ((0.375, 0.06), props.GREY)}
BIN_HALF = (0.065, 0.10)
BIN_SPOTS = (-0.06, 0.0, 0.06, 0.03)  # drop offsets along y, one per cube already in the bin
DENS = {"easy": ((0.35, 0.65), (1.5, 2.2)), "hard": ((0.80, 0.92), (1.08, 1.25))}
LEAK_FILL = 3.0  # s for a leaky cube to fill once it starts leaking
LEAK_RHO = 1.45  # effective density of a leaky cube once full of water (g/cm3)

INSTRUCTION = (
    "The four cubes look the same, but some float in water and some sink. Test each cube "
    "in the water tank, then put every floating cube in the blue bin and every sinking "
    "cube in the grey bin."
)


@dataclass
class Test:
    t: float
    cube: str
    floats: bool


@dataclass
class SinkFloat:
    difficulty: str = "easy"
    key: str = "sink_float"
    instruction: str = INSTRUCTION
    tests: list[Test] = field(default_factory=list)

    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        n_float = int(rng.integers(1, 4))
        floats = set(rng.choice(list(LABELS), size=n_float, replace=False).tolist())
        lo, hi = DENS[self.difficulty]
        self.density = {c: float(rng.uniform(*(lo if c in floats else hi))) for c in LABELS}
        self.truth = {c: c in floats for c in LABELS}
        # Hard: one of the sinkers is a sealed hollow cube with a pinhole: it floats high at first,
        # then fills after 9-12 s in the water (over LEAK_FILL s) and sinks. Its true class is 'sink'.
        self.leaky: str | None = None
        self.leak_after = 0.0
        order = rng.permutation(4)
        if self.difficulty == "hard":
            self.leaky = str(rng.choice([c for c in LABELS if c not in floats]))
            self.density[self.leaky] = float(rng.uniform(0.55, 0.70))
            self.leak_after = float(rng.uniform(9.0, 12.0))
        self.slots = {LABELS[i]: (ROW_X, ROW_Y[k]) for k, i in enumerate(order)}

        w = spec.worldbody
        # Tank: floor + 4 walls (glass), water as a visual-only translucent box.
        tank = w.add_body(name="tank", pos=[TANK_XY[0], TANK_XY[1], 0])
        hx, hy = TANK_HALF
        glass = [0.75, 0.85, 0.9, 0.35]
        tank.add_geom(name="tank_floor", type=props.G.mjGEOM_BOX, size=[hx, hy, FLOOR_Z / 2],
                      pos=[0, 0, FLOOR_Z / 2], rgba=[0.6, 0.7, 0.75, 1])
        t = 0.004
        for sx, sy, size in ((1, 0, [t, hy, TANK_WALL_H / 2]), (-1, 0, [t, hy, TANK_WALL_H / 2]),
                             (0, 1, [hx, t, TANK_WALL_H / 2]), (0, -1, [hx, t, TANK_WALL_H / 2])):
            tank.add_geom(type=props.G.mjGEOM_BOX, size=size, pos=[sx * (hx - t), sy * (hy - t), TANK_WALL_H / 2],
                          rgba=glass)
        tank.add_geom(name="water", type=props.G.mjGEOM_BOX, size=[hx - 2 * t, hy - 2 * t, (WATER_Z - FLOOR_Z) / 2],
                      pos=[0, 0, (WATER_Z + FLOOR_Z) / 2], rgba=[0.2, 0.45, 0.9, 0.35], contype=0, conaffinity=0)
        for name, (xy, col) in BINS.items():
            props.tray(spec, f"bin_{name}", xy, BIN_HALF, rgba=col)
            props.text_plate(spec, f"lbl_{name}", (xy[0], xy[1] + 0.125), "F" if name == "float" else "S",
                             half=(0.018, 0.018))
        for c in LABELS:
            x, y = self.slots[c]
            props.block(spec, f"cube_{c}", (x, y), H, self.density[c] * RHO_W * VOL, label=c, bg=(225, 120, 60))

    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._m = model
        self._bid = {c: model.body(f"cube_{c}").id for c in LABELS}
        self._dof = {c: model.joint(f"cube_{c}_free").dofadr[0] for c in LABELS}
        self._cgeom = {model.geom(f"cube_{c}").id: c for c in LABELS}
        self._tips = props.gripper_geoms(model)
        self.touch = props.TouchTracker(tuple(f"cube_{c}" for c in LABELS))
        self.touch.bind(model, data)
        self.tests = []
        self._still: dict[str, float] = {}
        self._wet = 0.0  # seconds the leaky cube has spent in the water
        self._t_last = float(data.time)

    def pos(self, data: mujoco.MjData, c: str) -> np.ndarray:
        return data.xpos[self._bid[c]]

    def in_tank(self, p: np.ndarray) -> bool:
        return props.in_box(p, TANK_XY, TANK_HALF) and p[2] < TANK_WALL_H + 0.05

    def held(self, data: mujoco.MjData) -> set[str]:
        return {self._cgeom[g] for g in props.held_geoms(self._m, data, self._tips) if g in self._cgeom}

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        t = float(data.time)
        dt, self._t_last = t - self._t_last, t
        self.touch.update(data)
        held = self.held(data)
        for c in LABELS:
            p = self.pos(data, c)
            b = self._bid[c]
            data.xfrc_applied[b] = 0.0
            if c == self.leaky:  # water that leaked in stays in (extra weight, in or out of the tank)
                data.xfrc_applied[b, 2] -= self.leak_fill() * (LEAK_RHO - self.density[c]) * RHO_W * VOL * 9.81
            if not self.in_tank(p):
                self._still.pop(c, None)
                continue
            sub = float(np.clip((WATER_Z - (p[2] - H)) / (2 * H), 0.0, 1.0))
            if sub > 0:
                v = data.qvel[self._dof[c]: self._dof[c] + 6]
                f = np.zeros(6)
                f[2] = RHO_W * VOL * sub * 9.81
                f[:3] -= 1.0 * sub * v[:3]  # linear drag (N per m/s)
                data.xfrc_applied[b] += f
                if c == self.leaky:
                    self._wet += dt
            speed = float(np.linalg.norm(data.qvel[self._dof[c]: self._dof[c] + 3]))
            if c in held or speed > 0.01:
                self._still.pop(c, None)
                continue
            self._still.setdefault(c, t)
            if t - self._still[c] >= 1.0:
                floats = bool(p[2] > FLOOR_Z + H + 0.5 * (WATER_Z - FLOOR_Z - 2 * H))
                last = [x for x in self.tests if x.cube == c]
                # Easy: first settled outcome only. Hard: a new outcome is logged whenever the
                # settled cube's state changes (the leaky cube sinks while left in the water).
                if not last or (self.difficulty == "hard" and last[-1].floats != floats):
                    self.tests.append(Test(t, c, floats))

    def leak_fill(self) -> float:
        if self.leaky is None:
            return 0.0
        return float(np.clip((self._wet - self.leak_after) / LEAK_FILL, 0.0, 1.0))

    def observed(self) -> dict[str, bool]:
        return {x.cube: x.floats for x in self.tests}

    def binned(self, data: mujoco.MjData) -> dict[str, str]:
        held = self.held(data)
        out = {}
        for c in LABELS:
            if c in held:
                continue
            for name, (xy, _) in BINS.items():
                if props.in_box(self.pos(data, c), xy, BIN_HALF, z_max=0.06):
                    out[c] = name
        return out

    def hud(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
        sc = self.score(model, data)
        obs = self.observed()
        return [f"t = {data.time:5.1f} s   tested: " + "  ".join(
                    f"{c}:{'float' if obs[c] else 'sink'}" for c in LABELS if c in obs),
                f"bins: " + "  ".join(f"{c}->{b}" for c, b in sorted(sc["bins"].items())),
                f"rubric stage {sc['stage']}/4   tested {sc['tested']}/4   correct in bins {sc['correct_in_bins']}/4"]

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        obs = self.observed()
        bins = self.binned(data)
        correct = sum(bins.get(c) == ("float" if self.truth[c] else "sink") for c in LABELS)
        success = correct == 4
        stage = 0
        if self.touch.touched:
            stage = 1
        if obs:
            stage = 2
        if len(obs) == 4 and all(obs[c] == self.truth[c] for c in LABELS):
            stage = 3
            if success:
                stage = 4
        return {
            "stage": stage,
            "success": float(success),
            "science_success": float(stage >= 3),
            "tested": len(obs),
            "correct_in_bins": correct,
            "procedure_violations": sum(1 for c in bins if c not in obs),  # binned untested
            "floats": [c for c in LABELS if self.truth[c]],
            "bins": bins,
            "log": [x.__dict__ for x in self.tests],
        }


# ---------- reference policies ----------
def _carry(env, mo: Motion, c: str, xy, surface_z: float, side: str = "left"):  # noqa: ANN001, ANN202
    """move_body, retried once with the other arm if an IK solve fails mid-motion
    (cubes that settled off-centre in the tank sit near the left arm's reach limit)."""
    try:
        yield from move_body(env, mo, f"cube_{c}", side, xy, surface_z, half=H, open_to=0.034, dz=-0.004)
    except RuntimeError:
        for s in ("left", "right"):
            yield from mo.gripper(s, 0.034, 0.3)
            yield from mo.home(s)
        other = "right" if side == "left" else "left"
        yield from move_body(env, mo, f"cube_{c}", other, xy, surface_z, half=H, open_to=0.034, dz=-0.004)


def _bin_spot(env, t: SinkFloat, name: str) -> tuple[float, float]:  # noqa: ANN001
    n = sum(1 for b in t.binned(env.data).values() if b == name)
    xy = BINS[name][0]
    return (xy[0], xy[1] + BIN_SPOTS[min(n, 3)])


def _test_and_sort(env, watch: float = 0.0, first_reading: bool = False):  # noqa: ANN001, ANN202
    """Lower each cube to the tank floor and let go, watch whether it rises to the surface
    or stays on the floor (for at least ``watch`` s after release), then lift it out into
    the matching bin. Reads only where the cube settles."""
    t: SinkFloat = env.task
    mo = Motion(env)
    for c in sorted(LABELS, key=lambda c: -t.slots[c][1]):
        n0 = len(t.tests)
        yield from _carry(env, mo, c, TANK_XY, FLOOR_Z, "left")
        n, t0 = len(t.tests), float(env.data.time)
        if first_reading:  # count a test logged while the arm was still letting go
            n = n0
        for _ in range(int(10 * max(12.0, watch + 2))):  # wait for the cube to settle (test recorded)
            if len(t.tests) > n and float(env.data.time) - t0 >= watch:
                break
            yield from mo.hold(0.1)
        floats = t.pos(env.data, c)[2] > FLOOR_Z + H + 0.5 * (WATER_Z - FLOOR_Z - 2 * H)
        name = "float" if floats else "sink"
        yield from _carry(env, mo, c, _bin_spot(env, t, name), FLOOR_Z, "left")
        for _ in range(2 if t.difficulty == "hard" else 0):  # look: did it land in the bin?
            yield from mo.hold(0.5)
            if t.binned(env.data).get(c) == name:
                break
            yield from _carry(env, mo, c, _bin_spot(env, t, name), FLOOR_Z, "left")
    yield from mo.hold(1.5)


def _quick(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Easy method: read each cube as soon as its first settled outcome is logged."""
    yield from _test_and_sort(env, 0.0, first_reading=True)


def oracle(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Easy: read each cube once it settles. Hard: leave each cube in the water for 18 s
    before reading it (18 s), long enough for a leaking cube to fill and sink."""
    yield from _test_and_sort(env, 18.0 if env.task.difficulty == "hard" else 0.0)


def guess(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Blind baseline: put each cube in a random bin without testing."""
    rng = rng or np.random.default_rng()
    t: SinkFloat = env.task
    mo = Motion(env)
    for c in sorted(LABELS, key=lambda c: -t.slots[c][1]):
        name = str(rng.choice(list(BINS)))
        yield from _carry(env, mo, c, _bin_spot(env, t, name), FLOOR_Z, "left")
    yield from mo.hold(1.5)


TASK = SinkFloat
POLICIES = {"oracle": oracle, "guess": guess, "noop": noop, "naive": _quick}
SPEC = TaskSpec(
    key="sink_float",
    title="Sort cubes into floaters and sinkers",
    ability="Property testing",
    instruction=INSTRUCTION,
    objects=(
        "four orange cubes labelled 1-4, 4 cm, identical look, different hidden densities",
        "clear water tank, 18 x 18 cm, 8 cm walls, water 6 cm deep",
        "blue bin marked F (floats) and grey bin marked S (sinks), 13 x 20 cm",
    ),
    setup=(
        "Tank 34 cm in front of the arms, 28 cm right of the left arm base.",
        "Cubes in a row 21 cm in front of the arms, 12 cm apart, random order.",
        "Both bins 6 cm to the left of the left arm's base line, 22 and 37.5 cm out; blue nearer the robot.",
    ),
    hidden=(
        "Each cube's density. 1 to 3 cubes float. Easy: floaters 0.35-0.65 g/cm3, sinkers "
        "1.5-2.2."
    ),
    instruction_hard=(
        "Some of the four cubes float in water and some sink. Put every cube that floats in the "
        "blue bin and every cube that sinks in the grey bin."
    ),
    hidden_hard=(
        "Each cube's density: floaters 0.80-0.92 g/cm3, sinkers 1.08-1.25 (floaters ride low, sinkers "
        "fall slowly). One sinker is a sealed hollow cube with a pinhole: it floats high (0.55-0.70) for "
        "the first 9-12 s it spends in water, then fills over 3 s and sinks (and stays heavy). Reading "
        "each cube as soon as it settles bins the leaky cube as a floater."
    ),
    rubric=(
        "No purposeful contact with the cubes",
        "Touches or moves a cube",
        "At least one cube tested: resting in the water, released, for 1 s",
        "All four cubes tested and the last outcome logged for each matches its true behaviour (hard: the leaky cube must have been watched until it sank)",
        "Stage 3, and every cube in the bin that matches its behaviour",
    ),
    metrics=(
        "stage (0-4)",
        "success: all four cubes in the correct bins, tested or not",
        "science_success: stage >= 3",
        "cubes tested; procedure violations = cubes binned without a test",
    ),
    report_fields=("tested", "correct_in_bins", "floats", "bins"),
    max_seconds=340.0,
    limitations=("Buoyancy and water drag are forces computed in code from how deep each cube sits "
                 "in the tank's water volume; there is no fluid simulation or splash."),
)
