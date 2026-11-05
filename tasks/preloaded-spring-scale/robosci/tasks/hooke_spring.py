"""Task: weigh an unknown block with an uncalibrated spring, then pick its mass card.

Ability: quantitative prediction. Hidden variables: the spring stiffness (100 g sinks the
pan 2.0-3.5 cm) and the unknown block's mass. Known 50 g and 100 g blocks calibrate the
spring (Hooke's law, x = m g / k); the pan's sink is shown in mm (0.1 mm steps) on a
digital display next to the scale, readable in the top camera. The answer is
given by moving the matching card (one of four printed masses) onto the red mat.
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

HALF = 0.02
CARD_HALF = (0.025, 0.025, 0.012)
STAND = (0.36, -0.305)
PAN_Z0 = 0.13  # pan top when empty
PAN_HALF = 0.045
RANGE = 0.10
BLOCKS = {"50g": 50.0, "100g": 100.0}
BLOCK_XY = {"50g": (0.22, -0.24), "100g": (0.22, -0.305), "unk": (0.22, -0.37)}
CARD_XY = ((0.22, -0.07), (0.22, -0.14), (0.22, -0.47), (0.22, -0.54))
MAT_POS, MAT_HALF = (0.30, 0.06), 0.06
DISPLAY = (0.36, -0.15, 0.02)  # beside the scale, on the left arm's side

INSTRUCTION = (
    "Find the mass of the block marked '?' using the spring scale and the 50 g and 100 g "
    "blocks. Then put the card showing its mass on the red mat."
)


@dataclass
class Measure:
    t: float
    block: str
    sink: float  # m, pan drop as shown on the display (0.1 mm steps)


@dataclass
class HookeSpring:
    difficulty: str = "easy"
    key: str = "hooke_spring"
    instruction: str = INSTRUCTION
    measures: list[Measure] = field(default_factory=list)

    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        step = 40 if self.difficulty == "easy" else 20
        self.d100 = float(rng.uniform(0.020, 0.035))
        self.k = 0.1 * 9.81 / self.d100
        while True:
            v = int(rng.integers(6, 20)) * 10 if self.difficulty == "easy" else int(rng.integers(15, 24)) * 10
            pos = int(rng.integers(4))
            cards = [v + (i - pos) * step for i in range(4)]
            if min(cards) >= 40 and max(cards) <= 240:
                break
        self.true_card = v
        self.cards = cards
        self.unk_mass = float(v + rng.uniform(-3, 3))
        self.masses = {**BLOCKS, "unk": self.unk_mass}
        # Hard: spring preload. The pan rests on its top stop until the load exceeds P grams,
        # so sink = (m - P) g / k and the ratio-through-zero method is biased.
        self.preload = float(rng.uniform(22, 35)) if self.difficulty == "hard" else 0.0

        w = spec.worldbody
        x, y = STAND
        dark, metal = list(props.DARK), [0.72, 0.72, 0.76, 1]
        st = w.add_body(name="stand", pos=[x, y, 0])
        st.add_geom(type=props.G.mjGEOM_BOX, size=[0.06, 0.07, 0.005], pos=[0, 0, 0.005], rgba=dark)
        tube_h = PAN_Z0 - RANGE - 0.01
        st.add_geom(type=props.G.mjGEOM_CYLINDER, size=[0.012, tube_h / 2], pos=[0, 0, tube_h / 2], rgba=metal,
                    contype=0, conaffinity=0)
        # Ruler: vertical white strip beside the pan with 1 cm ticks (decorative; the display is the readout).
        ry = -PAN_HALF - 0.03
        st.add_geom(type=props.G.mjGEOM_BOX, size=[0.004, 0.012, (PAN_Z0 + 0.02) / 2], pos=[0, ry, (PAN_Z0 + 0.02) / 2],
                    rgba=[0.95, 0.95, 0.92, 1], contype=0, conaffinity=0)
        for i in range(11):
            zt = PAN_Z0 - 0.01 * i
            st.add_geom(type=props.G.mjGEOM_BOX, size=[0.0012, 0.008 if i % 5 == 0 else 0.005, 0.0008],
                        pos=[0.004, ry, zt], rgba=[0.05, 0.05, 0.05, 1], contype=0, conaffinity=0)
        pan = w.add_body(name="pan", pos=[x, y, PAN_Z0 - 0.004])
        pm = 0.06
        pan.add_joint(name="pan_slide", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 0, 1], range=[-RANGE, 0.005 if self.preload == 0 else 0.0],
                      limited=mujoco.mjtLimited.mjLIMITED_TRUE, stiffness=self.k, springref=(pm + self.preload / 1000) * 9.81 / self.k,
                      damping=[7.0, 0, 0], armature=0.02)
        pan.add_geom(name="pan", type=props.G.mjGEOM_BOX, size=[PAN_HALF, PAN_HALF, 0.004], mass=pm * 0.8,
                     rgba=[0.80, 0.65, 0.30, 1], friction=[1.0, 0.01, 0.001], condim=4)
        pan.add_geom(type=props.G.mjGEOM_CYLINDER, size=[0.007, RANGE / 2 + 0.005], pos=[0, 0, -RANGE / 2 - 0.009],
                     rgba=metal, mass=pm * 0.2, contype=0, conaffinity=0)
        pan.add_geom(name="pointer", type=props.G.mjGEOM_BOX, size=[0.003, 0.016, 0.0015],
                     pos=[0.004, -PAN_HALF - 0.014, 0.002], rgba=[0.9, 0.1, 0.1, 1], contype=0, conaffinity=0)

        props.seg7(spec, "display", DISPLAY, n_digits=3, h=0.032, dp_after=1)
        props.text_plate(spec, "lbl_mm", (DISPLAY[0] - 0.055, DISPLAY[1]), "mm", half=(0.022, 0.022))
        props.flat(spec, "mat", MAT_POS, (MAT_HALF, MAT_HALF), props.RED)
        for name, xy in BLOCK_XY.items():
            lbl = "?" if name == "unk" else name
            props.block(spec, f"blk_{name}", xy, HALF, self.masses[name] / 1000, label=lbl, bg=(170, 172, 178))
        order = rng.permutation(4)
        for i, c in enumerate(cards):
            props.block(spec, f"card_{c}", CARD_XY[order[i]], CARD_HALF, 0.02, label=f"{c}g", bg=(245, 245, 240))

    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._m = model
        j = model.joint("pan_slide")
        self._q, self._v = j.qposadr[0], j.dofadr[0]
        self._tips = props.gripper_geoms(model)
        self._names = list(BLOCK_XY)
        self._bgeom = {model.geom(f"blk_{n}").id: n for n in self._names}
        self._cgeom = {model.geom(f"card_{c}").id: c for c in self.cards}
        self._pan = model.geom("pan").id
        self.touch = props.TouchTracker(tuple(f"blk_{n}" for n in self._names) + tuple(f"card_{c}" for c in self.cards))
        self.touch.bind(model, data)
        self.measures = []
        self._since: tuple[str, float] | None = None
        self._recorded: str | None = None
        self.seg = props.Seg7("display", 3)
        self.seg.bind(model)
        self._update_display(model, data)

    def sink(self, data: mujoco.MjData) -> float:
        """Visible pan drop from the empty position (ruler reading), m."""
        return -float(data.qpos[self._q])

    def reading(self, data: mujoco.MjData) -> float:
        """What the display shows, in m (sink rounded to 0.1 mm, floored at 0)."""
        return max(0.0, round(self.sink(data) * 1e4) / 1e4)

    def _update_display(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.seg.show(model, f"{min(999, int(round(self.reading(data) * 1e4))):03d}")  # '34.5'

    def pan_top(self, data: mujoco.MjData) -> float:
        return PAN_Z0 - self.sink(data)

    def _contacts(self, data: mujoco.MjData) -> tuple[set[str], set[str]]:
        on, held = set(), set()
        for c in data.contact[: data.ncon]:
            g1, g2 = int(c.geom1), int(c.geom2)
            for a, b in ((g1, g2), (g2, g1)):
                if a in self._bgeom and b == self._pan:
                    on.add(self._bgeom[a])
                if b in self._tips:
                    if a in self._bgeom:
                        held.add(self._bgeom[a])
                    if a in self._cgeom:
                        held.add(str(self._cgeom[a]))
        return on, held

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        t = float(data.time)
        self.touch.update(data)
        self._update_display(model, data)
        on, held = self._contacts(data)
        if not on:
            self._recorded = None
        if len(on) == 1 and not (on & held) and abs(float(data.qvel[self._v])) < 2e-3:
            b = next(iter(on))
            if self._since is None or self._since[0] != b:
                self._since = (b, t)
            elif t - self._since[1] >= 1.0 and self._recorded != b:
                self.measures.append(Measure(t, b, self.reading(data)))
                self._recorded = b
        else:
            self._since = None

    def predicted(self) -> float | None:
        """Unknown mass from the recorded sinks: least-squares spring constant through zero."""
        known = [(BLOCKS[m.block], m.sink) for m in self.measures if m.block in BLOCKS]
        unk = [m.sink for m in self.measures if m.block == "unk"]
        if not known or not unk:
            return None
        if self.difficulty == "hard":
            # Preloaded spring: mass = P + c * sink. Needs both known blocks (two distinct masses).
            if len({mm for mm, _ in known}) < 2 or min(s for _, s in known) <= 0:
                return None
            c, p0 = np.polyfit([s for _, s in known], [mm for mm, _ in known], 1)
            return float(p0 + c * np.mean(unk))
        g_per_m = sum(mm * s for mm, s in known) / sum(s * s for _, s in known)
        return float(g_per_m * np.mean(unk))

    def nearest_card(self) -> int | None:
        p = self.predicted()
        return None if p is None else min(self.cards, key=lambda c: abs(c - p))

    def on_mat(self, data: mujoco.MjData) -> list[int]:
        _, held = self._contacts(data)
        return [c for c in self.cards if str(c) not in held
                and props.in_box(data.xpos[self._m.body(f"card_{c}").id], MAT_POS, (MAT_HALF,) * 2,
                                 z_max=CARD_HALF[2] + 0.01)]

    def hud(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
        sc = self.score(model, data)
        p = self.predicted()
        lines = [f"t = {data.time:5.1f} s   display {1000 * self.reading(data):5.1f} mm   rubric stage {sc['stage']}/4",
                 f"predicted '?' mass {'-' if p is None else f'{p:.0f} g'}   cards {self.cards}   "
                 f"on mat {sc['on_mat'] or '-'}"]
        for m in self.measures[-3:]:
            lines.append(f"  {m.block if m.block != 'unk' else '?'} on pan: display {1000 * m.sink:.1f} mm")
        return lines

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        mat = self.on_mat(data)
        success = mat == [self.true_card]
        measured_unk = any(m.block == "unk" for m in self.measures)
        stage = 0
        if self.touch.touched:
            stage = 1
        if measured_unk:
            stage = 2
        if self.nearest_card() == self.true_card:
            stage = 3
            if success:
                stage = 4
        return {
            "stage": stage,
            "success": float(success),
            "science_success": float(stage >= 3),
            "procedure_violations": 0,
            "predicted_g": self.predicted(),
            "true_card": self.true_card,
            "unknown_g": self.unk_mass,
            "on_mat": mat,
            "log": [m.__dict__ for m in self.measures],
        }


# ---------- reference policies ----------
def _carry(env, mo: Motion, body: str, xy, z: float, half: float, open_to: float):  # noqa: ANN001, ANN202
    side = "left" if xy[1] > -0.305 else "right"
    return move_body(env, mo, body, side, xy, z, half=half, open_to=open_to, grip=0.0)


def _measure(env, mo: Motion, t: HookeSpring, name: str):  # noqa: ANN001, ANN202
    yield from _carry(env, mo, f"blk_{name}", STAND, t.pan_top(env.data), HALF, 0.028)
    yield from mo.hold(3.0)
    s = t.reading(env.data)  # the displayed number
    yield from _carry(env, mo, f"blk_{name}", BLOCK_XY[name], 0.0, HALF, 0.028)
    return s


def _ratio_method(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Calibrate with the 100 g block, measure '?', scale by the ratio of sinks, pick the card."""
    t: HookeSpring = env.task
    mo = Motion(env)
    s100 = yield from _measure(env, mo, t, "100g")
    su = yield from _measure(env, mo, t, "unk")
    est = 100.0 * su / max(s100, 1e-4)
    card = min(t.cards, key=lambda c: abs(c - est))
    yield from _carry(env, mo, f"card_{card}", MAT_POS, 0.0, CARD_HALF[2], 0.034)
    yield from mo.hold(1.5)


def _two_point(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Measure both known blocks, fit mass = P + c * sink (allows a preload), then '?'."""
    t: HookeSpring = env.task
    mo = Motion(env)
    s50 = yield from _measure(env, mo, t, "50g")
    s100 = yield from _measure(env, mo, t, "100g")
    su = yield from _measure(env, mo, t, "unk")
    c = 50.0 / max(s100 - s50, 1e-4)
    est = 100.0 + c * (su - s100)
    card = min(t.cards, key=lambda k: abs(k - est))
    yield from _carry(env, mo, f"card_{card}", MAT_POS, 0.0, CARD_HALF[2], 0.034)
    yield from mo.hold(1.5)


def oracle(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Easy: ratio method. Hard: two-point fit (the spring is preloaded)."""
    pol = _two_point if env.task.difficulty == "hard" else _ratio_method
    yield from pol(env, rng)


def guess(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Blind baseline: a random card to the mat."""
    rng = rng or np.random.default_rng()
    t: HookeSpring = env.task
    mo = Motion(env)
    card = int(rng.choice(t.cards))
    yield from _carry(env, mo, f"card_{card}", MAT_POS, 0.0, CARD_HALF[2], 0.034)
    yield from mo.hold(1.5)


TASK = HookeSpring
POLICIES = {"oracle": oracle, "guess": guess, "noop": noop, "naive": _ratio_method}
SPEC = TaskSpec(
    key="hooke_spring",
    title="Weigh an unknown block with a spring",
    ability="Quantitative prediction",
    instruction=INSTRUCTION,
    objects=(
        "spring scale: 9 x 9 cm pan on a vertical spring, empty pan top 13 cm high",
        "3-digit red LED display (3.2 cm digits, fixed decimal point) showing the pan's sink in mm, "
        "with an 'mm' label, lying flat beside the scale",
        "grey 4 cm cubes printed '50g' and '100g' (true masses), and one printed '?'",
        "four white mass cards (5 x 5 x 2.4 cm blocks) printed with candidate masses",
        "red mat, 12 x 12 cm",
    ),
    setup=(
        "Spring scale 36 cm in front of the arms, centred; check the display reads 00.0 with the pan empty.",
        "Display lying flat 15 cm to the left of the scale's centre (left arm's side), level with it, "
        "digits reading upright from the arms' side; 'mm' label just in front of it.",
        "50g, 100g and '?' cubes in a row 22 cm in front of the arms, centred, 6.5 cm apart.",
        "Cards 22 cm in front: two on the left arm's side, two on the right, random order.",
        "Red mat 30 cm in front of the left arm's base, on the operator's right.",
    ),
    hidden=(
        "Spring stiffness (100 g sinks the pan 2.0-3.5 cm) and the '?' mass (60-190 g). Cards are spaced "
        "40 g apart; the '?' block is within 3 g of one card."
    ),
    instruction_hard="Put the card showing the mass of the block marked '?' on the red mat.",
    hidden_hard=(
        "As easy, plus a spring preload of 22-35 g: the pan rests on its top stop until the load exceeds "
        "the preload, so sink = (m - P) g / k. The '?' mass is 150-230 g and cards are 20 g apart. "
        "Scaling the '?' sink by the 100 g sink (no intercept) over-estimates it by 14-60 g; "
        "a line through the 50 g and 100 g readings gives the right card."
    ),
    rubric=(
        "No purposeful contact with the blocks or cards",
        "Touches or moves a block or card",
        "The '?' block measured: alone on the pan, released, still for 1 s",
        "Known blocks measured too (easy: either; hard: both, for a line fit), and the calibrated estimate picks the right card",
        "Stage 3 and the right card alone on the mat",
    ),
    metrics=(
        "stage (0-4)",
        "success: right card alone on the mat, measured or not",
        "science_success: stage >= 3",
        "predicted mass vs true",
    ),
    report_fields=("predicted_g", "true_card", "unknown_g", "on_mat"),
    max_seconds=200.0,
    limitations=("The spring is a linear slide joint with hidden stiffness. The display shows the simulated pan "
                 "drop rounded to 0.1 mm; a real build needs a displacement sensor (or a load cell) under the pan."),
)
