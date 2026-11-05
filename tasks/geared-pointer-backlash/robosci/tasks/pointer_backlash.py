"""Task: turn a geared pointer to a target angle with + / - buttons whose step size is unknown.

Ability: instrument calibration. A pointer on a printed protractor (0 to 180 deg) is driven by a
small motor: each press of "+" or "-" turns the motor by a hidden step s (6-10 deg). A 3-digit
display shows the pointer angle in whole degrees. The target angle is printed on a red plate and
marked with a red tick on the arc.

Easy: no gear play; the target is k steps above the start (k = 3-6). Measure s with one press,
then press the right number of times (budget 10 presses).

Hard: the gearbox has backlash b (2, 4 or 6 deg): after a change of direction the motor turns b deg
before the pointer moves. The pointer was last turned with "+". The target sits below the start on
the grid reachable only while turning "-" (target = start + b - j s, j = 3-5), so the press count
computed from the "+" step alone comes out one short whenever b/s > 0.5, and lands on a "+" grid
angle that is never the target. The robot also puts the card showing b (0, 2, 4, 6 or 8 deg) on the
red mat. Budget 10 presses; the oracle needs 2 + j.

Readings: the pointer angle held unchanged for 1 s with no button down, after a press. Science
needs the readings taken before the last press to pin down the gear (step and, in hard, backlash)
over all integer steps 5-12 deg and backlashes 0-10 deg.

Simulation: the pointer is kinematic (its angle is set in code from the gear model); buttons are
physical spring buttons.
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

DIAL = (0.40, -0.305)
R_ARC = 0.10
PLATE_Z = 0.004
BTN_MINUS = props.ButtonSpec("btn_minus", (0.22, -0.25), rgba=(0.2, 0.2, 0.22, 1))
BTN_PLUS = props.ButtonSpec("btn_plus", (0.22, -0.36), rgba=(0.2, 0.2, 0.22, 1))
DISPLAY = (0.30, -0.305, 0.02)
TARGET_PLATE = (0.30, -0.43)
CARD_HALF = (0.025, 0.025, 0.012)
CARD_XY = ((0.19, 0.05), (0.19, -0.02), (0.19, -0.09), (0.19, -0.16), (0.26, -0.10))  # left arm, near the mat
CARDS = (0, 2, 4, 6, 8)
MAT_POS, MAT_HALF = (0.30, 0.06), 0.06
SPEED = 40.0  # deg/s the pointer swings
S_HYP = range(5, 13)
B_HYP = range(0, 11)

INSTRUCTION = (
    "Each press of the + or - button turns the pointer by the same unknown number of degrees. Turn the "
    "pointer to the angle on the red plate, using at most 10 presses in total."
)
INSTRUCTION_HARD = (
    "The + and - buttons turn the pointer through a small gearbox: each press turns the motor by the same "
    "unknown number of degrees, but the gears have play (backlash), so after a change of direction the "
    "motor turns some degrees before the pointer moves. The pointer was last turned with +. Turn the pointer "
    "to the angle on the red plate, and put the card showing the backlash in degrees on the red mat. Use at "
    "most 10 presses in total."
)


def gear_run(p0: float, s: float, b: float, presses: list[int]) -> list[float]:
    """Pointer angle after each press (+1 / -1), starting engaged in the + direction."""
    m, p, out = p0 + b / 2, p0, []
    for d in presses:
        m += d * s
        p = max(p, m - b / 2) if d > 0 else min(p, m + b / 2)
        out.append(p)
    return out


@dataclass
class Reading:
    t: float
    n: int  # presses before this reading
    angle: int


@dataclass
class PointerBacklash:
    difficulty: str = "easy"
    key: str = "pointer_backlash"
    instruction: str = ""
    readings: list[Reading] = field(default_factory=list)

    @property
    def hard(self) -> bool:
        return self.difficulty != "easy"

    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        self.s = int(rng.integers(6, 11))
        self.p0 = int(rng.integers(70, 111))
        if self.hard:
            self.b = int(rng.choice((2, 4, 6)))
            self.j = int(rng.integers(3, 6))
            self.target = self.p0 + self.b - self.j * self.s
            self.budget = 10
        else:
            self.b = 0
            self.k = int(rng.integers(3, 7))
            self.target = self.p0 + self.k * self.s
            self.budget = 10
        self.instruction = INSTRUCTION_HARD if self.hard else INSTRUCTION
        w = spec.worldbody
        cx, cy = DIAL
        # Dial plate (white) with ticks every 10 deg (long every 30 deg) and a red target tick.
        props.flat(spec, "dial_plate", (cx + 0.055, cy), (0.068, 0.118), (0.96, 0.96, 0.94, 1), z=0.002)
        for a in range(0, 181, 10):
            th = np.deg2rad(a)
            u = np.array([np.sin(th), -np.cos(th)])
            ln = 0.012 if a % 30 == 0 else 0.007
            mid = np.array(DIAL) + u * (R_ARC - ln / 2)
            w.add_geom(type=props.G.mjGEOM_BOX, size=[ln / 2, 0.0012, 0.0006], pos=[mid[0], mid[1], 0.0056],
                       quat=[np.cos(th / 2 - np.pi / 4), 0, 0, np.sin(th / 2 - np.pi / 4)],
                       rgba=[0.1, 0.1, 0.1, 1], contype=0, conaffinity=0)
        th = np.deg2rad(self.target)
        u = np.array([np.sin(th), -np.cos(th)])
        mid = np.array(DIAL) + u * (R_ARC + 0.008)
        w.add_geom(name="target_tick", type=props.G.mjGEOM_BOX, size=[0.012, 0.0022, 0.0007],
                   pos=[mid[0], mid[1], 0.0058], quat=[np.cos(th / 2 - np.pi / 4), 0, 0, np.sin(th / 2 - np.pi / 4)],
                   rgba=[0.9, 0.1, 0.1, 1], contype=0, conaffinity=0)
        hub = w.add_body(name="pointer", pos=[cx, cy, 0.006])
        hub.add_joint(name="pointer_hinge", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1], damping=[1.0, 0, 0])
        hub.add_geom(type=props.G.mjGEOM_BOX, size=[0.0025, 0.044, 0.0015], pos=[0, -0.044, 0.002],
                     rgba=[0.1, 0.1, 0.12, 1], mass=0.01, contype=0, conaffinity=0)
        hub.add_geom(type=props.G.mjGEOM_CYLINDER, size=[0.008, 0.003], pos=[0, 0, 0.002],
                     rgba=[0.1, 0.1, 0.12, 1], mass=0.005, contype=0, conaffinity=0)
        # Buttons, labels, display, target plate.
        # (distinct backgrounds: the label cache names files by their alphanumeric characters only)
        for b, lbl, bg in ((BTN_MINUS, "-", (236, 240, 248)), (BTN_PLUS, "+", (248, 240, 232))):
            props.push_button(spec, b)
            props.text_plate(spec, f"lbl_{b.name}", (b.pos[0] - 0.055, b.pos[1]), lbl, half=(0.02, 0.02), bg=bg)
        props.seg7(spec, "display", DISPLAY, n_digits=3, h=0.032)
        props.text_plate(spec, "target_plate", TARGET_PLATE, f"{self.target}°", half=(0.03, 0.03),
                         bg=(250, 190, 185))
        if self.hard:
            props.flat(spec, "mat", MAT_POS, (MAT_HALF, MAT_HALF), props.RED)
            order = rng.permutation(len(CARDS))
            for i, c in enumerate(CARDS):
                props.block(spec, f"card_{c}", CARD_XY[order[i]], CARD_HALF, 0.02, label=f"{c}°", bg=(245, 245, 240))

    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._m = model
        j = model.joint("pointer_hinge")
        self._q, self._v = j.qposadr[0], j.dofadr[0]
        self.buttons = [props.Button(BTN_PLUS), props.Button(BTN_MINUS)]
        for b in self.buttons:
            b.bind(model)
        self.seg = props.Seg7("display", 3)
        self.seg.bind(model)
        self.presses: list[int] = []
        self.motor = self.p0 + self.b / 2
        self.cmd = float(self.p0)  # pointer angle the gear sets
        self.shown = float(self.p0)  # animated pointer angle
        self.readings = []
        self._still = 0.0
        names = [b.name for b in (BTN_PLUS, BTN_MINUS)] + ([f"card_{c}" for c in CARDS] if self.hard else [])
        self.touch = props.TouchTracker(tuple(names))
        self.touch.bind(model, data)
        self._tips = props.gripper_geoms(model)
        if self.hard:
            self._cards = {c: model.body(f"card_{c}").id for c in CARDS}
            self._cgeom = {model.geom(f"card_{c}").id: c for c in CARDS}
        self._set_pointer(data)

    def _set_pointer(self, data: mujoco.MjData) -> None:
        data.qpos[self._q] = np.deg2rad(self.shown)
        data.qvel[self._v] = 0.0
        self.seg.show(self._m, f"{int(round(self.shown)):3d}")

    def angle(self) -> int:
        """What the display shows."""
        return int(round(self.shown))

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        t = float(data.time)
        self.touch.update(data)
        for d, b in zip((1, -1), self.buttons):
            if b.update(data):
                self.presses.append(d)
                self.motor += d * self.s
                self.cmd = max(self.cmd, self.motor - self.b / 2) if d > 0 else min(self.cmd, self.motor + self.b / 2)
                self._still = t
        dt = 0.02
        step = SPEED * dt
        if abs(self.cmd - self.shown) > 1e-9:
            self.shown += float(np.clip(self.cmd - self.shown, -step, step))
            self._still = t
        if any(b.pressed for b in self.buttons):
            self._still = t
        self._set_pointer(data)
        n = len(self.presses)
        if n and t - self._still >= 1.0 and all(r.n != n for r in self.readings):
            self.readings.append(Reading(t, n, self.angle()))

    def hypotheses(self, upto: int | None = None) -> list[tuple[int, int]]:
        """(step, backlash) pairs consistent with the readings taken after fewer than ``upto`` presses."""
        rs = [r for r in self.readings if upto is None or r.n < upto]
        bs = B_HYP if self.hard else (0,)
        out = []
        for s in S_HYP:
            for b in bs:
                pred = gear_run(self.p0, s, b, self.presses)
                if all(abs(pred[r.n - 1] - r.angle) <= 0.5 for r in rs):
                    out.append((s, b))
        return out

    def on_mat(self, data: mujoco.MjData) -> list[int]:
        if not self.hard:
            return []
        held = props.held_geoms(self._m, data, self._tips)
        return [c for c, bid in self._cards.items()
                if not any(g in held for g, cc in self._cgeom.items() if cc == c)
                and props.in_box(data.xpos[bid], MAT_POS, (MAT_HALF,) * 2, z_max=CARD_HALF[2] + 0.01)]

    def hud(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
        sc = self.score(model, data)
        lines = [f"t = {data.time:5.1f} s   pointer {self.angle()} deg   target {self.target}   presses "
                 f"{len(self.presses)}/{self.budget}   rubric stage {sc['stage']}/4",
                 f"hidden: step {self.s} deg, backlash {self.b} deg   gear models left {sc['models_left']}"
                 + (f"   card on mat {sc['on_mat'] or '-'}" if self.hard else "")]
        for r in self.readings[-3:]:
            lines.append(f"  after press {r.n} ({'+' if self.presses[r.n - 1] > 0 else '-'}): {r.angle} deg")
        return lines

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        n = len(self.presses)
        hyps = self.hypotheses(upto=n)
        pinned = bool(self.readings) and len({h if self.hard else h[0] for h in hyps}) == 1 and \
            any(r.n < n for r in self.readings)
        mat = self.on_mat(data)
        on_target = abs(self.cmd - self.target) <= 1 and abs(self.shown - self.target) <= 1
        card_ok = (mat == [self.b]) if self.hard else True
        success = on_target and card_ok and n <= self.budget
        stage = 0
        if self.touch.touched or n:
            stage = 1
        if self.readings:
            stage = 2
        if pinned and card_ok:
            stage = 3
            if success:
                stage = 4
        return {
            "stage": stage,
            "success": float(success),
            "science_success": float(stage >= 3),
            "procedure_violations": max(0, n - self.budget),
            "presses": n,
            "models_left": len(hyps),
            "pointer": self.angle(),
            "target": self.target,
            "step": self.s,
            "backlash": self.b,
            "on_mat": mat,
            "log": [r.__dict__ for r in self.readings],
        }


# ---------- reference policies ----------
def _press(mo: Motion, t: PointerBacklash, d: int):  # noqa: ANN202
    b = t.buttons[0] if d > 0 else t.buttons[1]
    side = "left" if b.spec.pos[1] >= -0.305 else "right"
    yield from mo.press(side, b.top(), depth=b.spec.travel + 0.004)
    yield from mo.hold(1.6)


def _carry_card(env, mo: Motion, c: int):  # noqa: ANN001, ANN202
    xy = env.data.xpos[env.model.body(f"card_{c}").id][:2]
    side = "left" if xy[1] > -0.305 else "right"
    yield from move_body(env, mo, f"card_{c}", side, MAT_POS, 0.0, half=CARD_HALF[2], open_to=0.034, grip=0.0)


def _finish(mo: Motion):  # noqa: ANN202
    for side in ("left", "right"):
        yield from mo.home(side)
    yield from mo.hold(1.5)


def _easy_method(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Press + once, read the step, then press the computed number of times toward the target.
    In hard mode it also takes the gear to have no play (puts the 0 deg card on the mat)."""
    t: PointerBacklash = env.task
    mo = Motion(env)
    yield from mo.hold(1.0)
    a0 = t.angle()
    yield from _press(mo, t, +1)
    s = max(1, t.angle() - a0)
    n = int(round((t.target - t.angle()) / s))
    for _ in range(abs(n)):
        yield from _press(mo, t, 1 if n > 0 else -1)
    if t.hard:
        yield from _carry_card(env, mo, 0)
    yield from _finish(mo)


def _calibrated(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Hard: press + (read the step s), press - (the pointer moves s - b: read the backlash b), then
    press - until the target, re-reading after each press; card b to the mat."""
    t: PointerBacklash = env.task
    mo = Motion(env)
    yield from mo.hold(1.0)
    a0 = t.angle()
    yield from _press(mo, t, +1)
    a1 = t.angle()
    s = a1 - a0
    yield from _press(mo, t, -1)
    b = s - (a1 - t.angle())
    for _ in range(t.budget - 2):
        if t.angle() == t.target:
            break
        yield from _press(mo, t, -1 if t.angle() > t.target else 1)
    if b in CARDS:
        yield from _carry_card(env, mo, b)
    yield from _finish(mo)


def oracle(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Easy: one press to measure the step, then the computed presses. Hard: also a reverse press to
    measure the backlash, then approach the target from the - side."""
    pol = _calibrated if env.task.hard else _easy_method
    yield from pol(env, rng)


def guess(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Blind baseline: one press of a random button (hard: and a random card to the mat)."""
    rng = rng or np.random.default_rng()
    t: PointerBacklash = env.task
    mo = Motion(env)
    yield from _press(mo, t, int(rng.choice((1, -1))))
    if t.hard:
        yield from _carry_card(env, mo, int(rng.choice(CARDS)))
    yield from _finish(mo)


TASK = PointerBacklash
POLICIES = {"oracle": oracle, "guess": guess, "noop": noop, "naive": _easy_method}
SPEC = TaskSpec(
    key="pointer_backlash",
    title="Aim a geared pointer whose step size is unknown",
    ability="Instrument calibration",
    instruction=INSTRUCTION,
    instruction_hard=INSTRUCTION_HARD,
    objects=(
        "white dial plate with a protractor arc (0-180 deg, a tick every 10 deg, long ticks every 30 deg) and "
        "a black pointer on a centre pivot",
        "two push buttons marked - and +",
        "3-digit red LED display showing the pointer angle in whole degrees",
        "pink plate printed with the target angle; red tick at that angle on the arc",
        "hard only: five white cards (5 x 5 x 2.4 cm) printed 0°, 2°, 4°, 6°, 8°, and a 12 x 12 cm red mat",
    ),
    setup=(
        "Dial pivot 40 cm in front of the arms, centred between them; 0 deg points to the right arm, 90 deg "
        "straight ahead, 180 deg to the left arm.",
        "Buttons 22 cm in front: - 5.5 cm to the left of centre, + 5.5 cm to the right.",
        "Display 30 cm in front, centred; target plate 30 cm in front, 12.5 cm right of centre.",
        "Hard: five cards in front of the left arm (19 cm out at 5, -2, -9 and -16 cm from the left arm's "
        "line, one 26 cm out at -10 cm), red mat 30 cm in front of the left arm's base.",
    ),
    hidden=("Step per press s (6-10 deg) and the start angle (70-110 deg); the target is k = 3-6 steps above "
            "the start. Budget 10 presses (oracle: k)."),
    hidden_hard=("Step s (6-10 deg), backlash b (2, 4 or 6 deg), start angle 70-110 deg, gear engaged in the + "
                 "direction. Target = start + b - j s (j = 3-5): reachable only while turning -. Pressing + once "
                 "and computing the - presses from that step lands one step short whenever b/s > 0.5; and it "
                 "takes the backlash to be 0. Budget 10 presses (oracle: 2 + j)."),
    rubric=(
        "No button pressed and nothing touched",
        "Presses a button or touches a card",
        "At least one reading: the pointer held still for 1 s after a press",
        "The readings taken before the last press pin down the gear (easy: the step; hard: step and backlash) "
        "and, in hard, the card showing the backlash is alone on the mat",
        "Stage 3, the pointer within 1 deg of the target at the end, within the press budget",
    ),
    metrics=("stage (0-4)", "success: pointer on target (hard: and the right card) within budget",
             "science_success: stage >= 3", "presses; gear models left"),
    report_fields=("presses", "models_left", "pointer", "target", "step", "backlash", "on_mat"),
    max_seconds=200.0,
    limitations=("The pointer is moved in code from a gear model (motor step, backlash) when a button press "
                 "registers; a real build would use a stepper motor and a geared dial with an encoder display."),
)
