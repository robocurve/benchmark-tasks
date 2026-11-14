"""Task: make a platform truly level with a motorised levelling foot and a digital angle gauge.

Ability: instrument calibration. A 20 x 14 cm platform pivots on its far edge; its near edge rests on
a motorised levelling foot with UP and DOWN buttons (each press changes the tilt by 0.25 deg). A
digital angle gauge (a 5 cm cube with an arrow showing its sensing direction) sends its reading to a
display on the table, in tenths of a degree. The platform starts tilted 1-2 deg.

Easy: the gauge reads true. Put it on the platform, read, press until the platform is level.

Hard: the gauge has a hidden zero error e (0.6-1.0 deg, either sign). A grey granite block marked REF
is a reference surface stated to be exactly level. Pressing until the gauge reads 0 leaves the platform
tilted by -e (0.6-1.0 deg, beyond the 0.15 deg tolerance); the gauge's reading on REF (= e), or two
platform readings with the gauge turned 180 deg in between (r1 = tilt + e, r2 = -tilt + e), gives the
error. Science needs e pinned (a REF reading, or such a reversal pair at one jack setting) and a
platform reading.

Simulation: the platform's tilt is set in code (a stiff hinge spring whose rest angle follows the jack);
the gauge reading is its body's pitch plus e, rounded to 0.1 deg.
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

PLAT = (0.40, -0.305)  # platform centre
PLAT_HALF = (0.10, 0.07, 0.006)
HINGE_X = PLAT[0] + PLAT_HALF[0]  # far edge
H = 0.06  # hinge height
DELTA = 0.25  # deg per press
TOL = 0.15  # deg
BTN_UP = props.ButtonSpec("btn_up", (0.22, -0.40), rgba=(0.2, 0.2, 0.22, 1))
BTN_DN = props.ButtonSpec("btn_down", (0.22, -0.50), rgba=(0.2, 0.2, 0.22, 1))
DISPLAY = (0.24, -0.24, 0.02)
REF = (0.31, -0.08)
REF_HALF = (0.05, 0.045, 0.025)
GAUGE_HALF = 0.025
GAUGE_XY = (0.19, -0.02)

INSTRUCTION = (
    "The digital angle gauge reads true; its reading appears on the red display, in degrees. Make the "
    "platform level to within 0.15 degrees, using the UP and DOWN buttons of its levelling foot."
)
INSTRUCTION_HARD = (
    "The digital angle gauge may not read true; its reading appears on the red display, in degrees. The grey "
    "granite block marked REF is exactly level. Make the platform truly level, to within 0.15 degrees, using "
    "the UP and DOWN buttons of its levelling foot."
)


@dataclass
class Reading:
    t: float
    where: str  # "platform" or "ref"
    heading: float  # deg, gauge arrow direction
    jack: int  # net presses (UP - DOWN) at the time
    value: float  # displayed reading, deg


def _tenths(v: float) -> str:
    k = int(round(np.clip(v, -9.9, 9.9) * 10))
    return f"{'-' if k < 0 else ' '}{abs(k):02d}"


@dataclass
class LevelReversal:
    difficulty: str = "easy"
    key: str = "level_reversal"
    instruction: str = ""
    readings: list[Reading] = field(default_factory=list)

    @property
    def hard(self) -> bool:
        return self.difficulty != "easy"

    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        n = int(rng.integers(4, 9))
        self.tilt0 = n * DELTA + float(rng.uniform(-0.08, 0.08))
        if self.hard:
            self.e = float(rng.uniform(0.6, 1.0)) * float(rng.choice((-1.0, 1.0)))
        else:
            self.e = 0.0
        self.instruction = INSTRUCTION_HARD if self.hard else INSTRUCTION
        w = spec.worldbody
        # Fixed far support and the levelling foot (visual), platform on a hinge at its far edge.
        w.add_geom(type=props.G.mjGEOM_BOX, size=[0.012, 0.06, H / 2], pos=[HINGE_X - 0.006, PLAT[1], H / 2],
                   rgba=[0.35, 0.35, 0.38, 1], contype=0, conaffinity=0)
        w.add_geom(type=props.G.mjGEOM_CYLINDER, size=[0.016, (H - 0.012) / 2],
                   pos=[PLAT[0] - PLAT_HALF[0] + 0.02, PLAT[1], (H - 0.012) / 2], rgba=[0.75, 0.55, 0.15, 1],
                   contype=0, conaffinity=0)
        pb = w.add_body(name="platform", pos=[HINGE_X, PLAT[1], H], gravcomp=1.0)
        pb.add_joint(name="plat_hinge", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0], stiffness=300.0,
                     springref=float(np.deg2rad(-self.tilt0)), damping=[3.0, 0, 0], armature=0.02)
        pb.add_geom(name="platform", type=props.G.mjGEOM_BOX, size=list(PLAT_HALF), pos=[-PLAT_HALF[0], 0, 0],
                    rgba=[0.55, 0.6, 0.66, 1], mass=0.3, friction=[1.0, 0.01, 0.001], condim=4)
        # Granite reference block.
        rb = w.add_body(name="ref", pos=[REF[0], REF[1], REF_HALF[2]])
        rb.add_geom(name="ref", type=props.G.mjGEOM_BOX, size=list(REF_HALF), rgba=[0.32, 0.33, 0.35, 1],
                    friction=[1.0, 0.01, 0.001], condim=4)
        props.text_plate(spec, "lbl_ref", (REF[0] - REF_HALF[0] - 0.035, REF[1]), "REF", half=(0.025, 0.025))
        # Gauge: yellow cube with an arrow on top (sensing direction = its +x axis).
        gb = props.block(spec, "gauge", GAUGE_XY, GAUGE_HALF, 0.12, rgba=(0.95, 0.8, 0.1, 1))
        gb.add_geom(type=props.G.mjGEOM_BOX, size=[0.015, 0.003, 0.0006], pos=[0, 0, GAUGE_HALF],
                    rgba=[0.1, 0.1, 0.1, 1], contype=0, conaffinity=0, mass=0.0001)
        for sgn in (1, -1):
            gb.add_geom(type=props.G.mjGEOM_BOX, size=[0.007, 0.0025, 0.0006],
                        pos=[0.011, sgn * 0.005, GAUGE_HALF],
                        quat=[np.cos(sgn * 3 * np.pi / 8), 0, 0, np.sin(sgn * 3 * np.pi / 8)],
                        rgba=[0.1, 0.1, 0.1, 1], contype=0, conaffinity=0, mass=0.0001)
        # Jack buttons, display, labels.
        for b, lbl, bg in ((BTN_UP, "UP", (236, 240, 248)), (BTN_DN, "DOWN", (248, 240, 232))):
            props.push_button(spec, b)
            props.text_plate(spec, f"lbl_{b.name}", (b.pos[0] - 0.055, b.pos[1]), lbl, half=(0.022, 0.022), bg=bg)
        props.seg7(spec, "display", DISPLAY, n_digits=3, h=0.032, dp_after=1)

    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._m = model
        j = model.joint("plat_hinge")
        self._q, self._v = j.qposadr[0], j.dofadr[0]
        data.qpos[self._q] = np.deg2rad(-self.tilt0)
        self.buttons = [props.Button(BTN_UP), props.Button(BTN_DN)]
        for b in self.buttons:
            b.bind(model)
        self.seg = props.Seg7("display", 3)
        self.seg.bind(model)
        self.jack = 0
        self.readings = []
        self._gauge = model.body("gauge").id
        self._ggeoms = {g for g in range(model.ngeom) if model.geom_bodyid[g] == self._gauge}
        self._plat = {model.geom("platform").id}
        self._ref = {model.geom("ref").id}
        self._tips = props.gripper_geoms(model)
        self._since: tuple[str, int, float] | None = None
        self.touch = props.TouchTracker(("gauge", BTN_UP.name, BTN_DN.name))
        self.touch.bind(model, data)
        self._show(model, data)

    def tilt(self, data: mujoco.MjData) -> float:
        """True platform tilt, deg (positive: far edge high)."""
        return float(np.rad2deg(-data.qpos[self._q]))

    def gauge_value(self, data: mujoco.MjData) -> float:
        xa = data.xmat[self._gauge].reshape(3, 3)[:, 0]
        return float(np.rad2deg(np.arcsin(np.clip(xa[2], -1, 1)))) + self.e

    def heading(self, data: mujoco.MjData) -> float:
        xa = data.xmat[self._gauge].reshape(3, 3)[:, 0]
        return float(np.rad2deg(np.arctan2(xa[1], xa[0])))

    def displayed(self) -> float:
        txt = self.seg.text
        return float(txt.replace(" ", "")) / 10 if txt.strip() not in ("", "-") else 0.0

    def _show(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.seg.show(model, _tenths(self.gauge_value(data)))

    def _surface(self, data: mujoco.MjData) -> str | None:
        on = None
        for c in data.contact[: data.ncon]:
            g1, g2 = int(c.geom1), int(c.geom2)
            for a, b in ((g1, g2), (g2, g1)):
                if a in self._ggeoms and b in self._tips:
                    return None
                if a in self._ggeoms and b in self._plat:
                    on = "platform"
                if a in self._ggeoms and b in self._ref:
                    on = "ref"
        return on

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        t = float(data.time)
        self.touch.update(data)
        for d, b in zip((1, -1), self.buttons):
            if b.update(data):
                self.jack += d
                model.qpos_spring[self._q] = np.deg2rad(-(self.tilt0 - DELTA * self.jack))
        self._show(model, data)
        where = self._surface(data)
        speed = float(np.linalg.norm(data.cvel[self._gauge]))
        still = where is not None and speed < 0.01 and abs(data.qvel[self._v]) < 1e-3 \
            and not any(b.pressed for b in self.buttons)
        if not still:
            self._since = None
            return
        key = (where, self.jack)
        if self._since is None or self._since[:2] != key:
            self._since = (where, self.jack, t)
        elif t - self._since[2] >= 1.0:
            hd = self.heading(data)
            if not any(r.where == where and r.jack == self.jack and abs((r.heading - hd + 180) % 360 - 180) < 30
                       for r in self.readings):
                self.readings.append(Reading(t, where, hd, self.jack, self.displayed()))

    def error_known(self) -> bool:
        if not self.hard:
            return True
        if any(r.where == "ref" for r in self.readings):
            return True
        plat = [r for r in self.readings if r.where == "platform"]
        return any(a.jack == b.jack and abs((a.heading - b.heading) % 360 - 180) < 30 for a in plat for b in plat)

    def hud(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
        sc = self.score(model, data)
        lines = [f"t = {data.time:5.1f} s   true tilt {self.tilt(data):+.2f} deg   gauge {self.displayed():+.1f}   "
                 f"jack {self.jack:+d}   rubric stage {sc['stage']}/4",
                 f"hidden: gauge error {self.e:+.2f} deg   error pinned {sc['error_known']}"]
        for r in self.readings[-3:]:
            lines.append(f"  on {r.where} (arrow {r.heading:+.0f} deg, jack {r.jack:+d}): {r.value:+.1f}")
        return lines

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        tilt = self.tilt(data)
        success = abs(tilt) <= TOL
        plat = any(r.where == "platform" for r in self.readings)
        known = self.error_known()
        stage = 0
        if self.touch.touched or self.jack or any(b.presses for b in self.buttons):
            stage = 1
        if plat:
            stage = 2
            if known:
                stage = 3
                if success:
                    stage = 4
        return {
            "stage": stage,
            "success": float(success),
            "science_success": float(stage >= 3),
            "procedure_violations": 0,
            "tilt": tilt,
            "gauge_error": self.e,
            "error_known": known,
            "presses": sum(b.presses for b in self.buttons),
            "log": [r.__dict__ for r in self.readings],
        }


# ---------- reference policies ----------
def _press(mo: Motion, t: LevelReversal, up: bool):  # noqa: ANN202
    b = t.buttons[0] if up else t.buttons[1]
    yield from mo.press("right", b.top(), depth=b.spec.travel + 0.004)
    yield from mo.hold(0.6)


def _place(env, mo: Motion, xy, surface_z: float):  # noqa: ANN001, ANN202
    yield from move_body(env, mo, "gauge", "left", xy, surface_z, half=GAUGE_HALF, open_to=0.034)
    yield from mo.hold(2.0)


def _plat_top(env) -> float:  # noqa: ANN001
    return float(env.data.geom_xpos[env.model.geom("platform").id][2]) + PLAT_HALF[2]


def _settle_read(env, mo: Motion, t: LevelReversal):  # noqa: ANN001, ANN202
    yield from mo.hold(1.5)
    return t.displayed()


def _level_by_gauge(env, mo: Motion, t: LevelReversal, offset: float):  # noqa: ANN001, ANN202
    """Read, press the computed number of UP/DOWN steps, re-read; until gauge - offset is within half a step of 0."""
    for _ in range(6):
        r = (yield from _settle_read(env, mo, t)) - offset
        if abs(r) <= DELTA / 2:
            break
        for _ in range(max(1, int(round(abs(r) / DELTA)))):
            yield from _press(mo, t, up=r > 0)


def _easy_method(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Put the gauge on the platform and press until it reads 0 (trusts the gauge)."""
    t: LevelReversal = env.task
    mo = Motion(env)
    yield from _place(env, mo, PLAT, _plat_top(env))
    yield from mo.home("left")
    yield from _level_by_gauge(env, mo, t, 0.0)
    for side in ("left", "right"):
        yield from mo.home(side)
    yield from mo.hold(1.0)


def _calibrated(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Hard: read the gauge on the REF block (its zero error), then level the platform to that reading."""
    t: LevelReversal = env.task
    mo = Motion(env)
    yield from _place(env, mo, REF, 2 * REF_HALF[2])
    e = yield from _settle_read(env, mo, t)
    yield from _place(env, mo, PLAT, _plat_top(env))
    yield from mo.home("left")
    yield from _level_by_gauge(env, mo, t, e)
    for side in ("left", "right"):
        yield from mo.home(side)
    yield from mo.hold(1.0)


def oracle(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Easy: level by the gauge. Hard: zero the gauge on REF first."""
    pol = _calibrated if env.task.hard else _easy_method
    yield from pol(env, rng)


def guess(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Blind baseline: press UP a random 0-10 times without measuring."""
    rng = rng or np.random.default_rng()
    t: LevelReversal = env.task
    mo = Motion(env)
    for _ in range(int(rng.integers(0, 11))):
        yield from _press(mo, t, up=True)
    yield from mo.home("right")
    yield from mo.hold(1.0)


TASK = LevelReversal
POLICIES = {"oracle": oracle, "guess": guess, "noop": noop, "naive": _easy_method}
SPEC = TaskSpec(
    key="level_reversal",
    title="Level a platform with a gauge that may read wrong",
    ability="Instrument calibration",
    instruction=INSTRUCTION,
    instruction_hard=INSTRUCTION_HARD,
    objects=(
        "grey aluminium platform, 20 x 14 x 1.2 cm, hinged along its far edge 6 cm above the table; its near edge "
        "rests on a motorised levelling foot (orange)",
        "two push buttons labelled UP and DOWN driving the foot: each press tilts the platform by 0.25 deg",
        "yellow digital angle gauge, a 5 cm cube with a black arrow on top marking its sensing direction",
        "3-digit red LED display showing the gauge reading in tenths of a degree",
        "grey granite reference block 10 x 9 x 5 cm marked REF (hard version: stated to be exactly level)",
    ),
    setup=(
        "Platform centred between the arms, 30-50 cm in front of them; foot under its near edge.",
        "UP and DOWN buttons 22 cm in front, 9.5 and 19.5 cm to the right of centre.",
        "Display 24 cm in front, 6.5 cm left of centre; REF block 31 cm in front of the left arm's line, "
        "22.5 cm left of centre; gauge 19 cm in front of the left arm, arrow pointing away from the arms.",
        "Start the platform tilted 1-2 deg, far edge high.",
    ),
    hidden="Initial tilt (1-2 deg, n x 0.25 deg plus up to 0.08 deg). The gauge reads true.",
    hidden_hard=("Initial tilt as easy; gauge zero error e = 0.6-1.0 deg, random sign. Pressing until the gauge "
                 "reads 0 leaves the platform 0.6-1.0 deg off. The gauge on REF reads e; the platform's true tilt "
                 "is its reading minus e (or half the difference of two readings with the gauge turned 180 deg)."),
    rubric=(
        "No button pressed and nothing touched",
        "Presses a button or touches the gauge",
        "At least one gauge reading on the platform (gauge resting, untouched, still for 1 s)",
        "The gauge's error is pinned (hard: a reading on REF, or two platform readings with the gauge turned "
        "180 deg at the same foot setting) and the platform has been read",
        "Stage 3 and the platform truly level within 0.15 deg at the end",
    ),
    metrics=("stage (0-4)", "success: true tilt within 0.15 deg", "science_success: stage >= 3",
             "final true tilt; gauge error"),
    report_fields=("tilt", "gauge_error", "error_known", "presses"),
    max_seconds=200.0,
    limitations=("The platform's tilt is set by a stiff hinge spring whose rest angle steps 0.25 deg per press "
                 "(a motorised foot in code); the gauge reading is computed from the cube's pitch plus a fixed "
                 "zero error and shown on a separate display."),
)
