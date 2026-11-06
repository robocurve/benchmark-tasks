"""Task: balance a bar with a hidden internal weight on a fulcrum.

Ability: hidden structure. Hidden variable: where a 100 g slug sits inside a 30 cm bar
(the bar's centre of mass is 3-7 cm off its middle). The fulcrum has a 12 mm flat top;
two low stops 5.5 cm either side catch the bar when it tips (about 3 deg). A probe is
setting the bar on the fulcrum at some point along it and watching which end drops.
Bisection on the drop direction locates the centre of mass.

Hard: the slug sits at one of +-6, 9, 12 cm and the robot must also put the card naming
the slug's position on a mat. The balance point is only 100/180 of the slug offset (the
80 g shell is centred), so the card nearest the balance point is wrong for 9 and 12 cm.

Coordinates: the bar lies along world x (front-back), on the left arm's side. u = the
bar-frame position (m, from the bar's middle, +x) over the fulcrum. The +x (far, blue) end
drops when the CoM is beyond u.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import mujoco
import numpy as np

from robosci import props
from robosci.motion import SAFE_Z, TILTS, Motion, grasp_rot
from robosci.skills import move_body, noop
from robosci.specs import TaskSpec

BAR_HALF = (0.15, 0.02, 0.015)
SHELL_MASS, SLUG_MASS = 0.08, 0.10
FUL_X, FUL_Y = 0.30, -0.10
FUL_TOP = 0.07
FUL_HALF_W = 0.006  # half width of the flat top (along x)
STOP_DY = 0.055
TIP_DEG = 3.0
STOP_TOP = FUL_TOP - STOP_DY * np.tan(np.deg2rad(TIP_DEG)) - 0.001
BAR_START = (0.27, 0.05)
LEVEL_DEG = 1.0
TIPPED_DEG = 1.5
# Hard: name the slug's position with a card. Slug at +-6, 9 or 12 cm from the middle.
SLUG_CARDS = (-12, -9, -6, 6, 9, 12)  # cm, + toward the blue (far) end
CARD_HALF = (0.025, 0.025, 0.012)
CARD_X, CARD_Y = 0.20, (-0.20, -0.275, -0.35, -0.425, -0.50, -0.575)
MAT_POS, MAT_HALF = (0.32, -0.30), (0.06, 0.06)
MASS_RATIO = (SHELL_MASS + SLUG_MASS) / SLUG_MASS  # slug offset = ratio * CoM offset

INSTRUCTION = (
    "The bar has a weight hidden inside it, off-centre. Find where its weight is concentrated "
    "and leave the bar resting balanced (level) on the fulcrum, touching nothing else."
)


@dataclass
class Probe:
    t: float
    u: float  # bar-frame point over the fulcrum
    outcome: int  # +1: +x end down, -1: -x end down, 0: level


@dataclass
class HiddenWeightBar:
    difficulty: str = "easy"
    key: str = "hidden_weight_bar"
    instruction: str = INSTRUCTION
    probes: list[Probe] = field(default_factory=list)

    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        if self.difficulty == "hard":
            self.slug_card = int(rng.choice(SLUG_CARDS))
            s = self.slug_card / 100.0
        else:
            s = float(rng.choice([-1, 1]) * rng.uniform(0.05, 0.12))
        self.slug = s
        self.com = SLUG_MASS * s / (SHELL_MASS + SLUG_MASS)
        w = spec.worldbody
        wood = [0.62, 0.45, 0.28, 1]
        # Fulcrum: dark block with a narrow flat top; stops either side.
        ful = w.add_body(name="fulcrum", pos=[FUL_X, FUL_Y, 0])
        ful.add_geom(type=props.G.mjGEOM_BOX, size=[0.03, 0.04, 0.005], pos=[0, 0, 0.005], rgba=list(props.DARK))
        ful.add_geom(name="fulcrum", type=props.G.mjGEOM_BOX, size=[FUL_HALF_W, 0.03, FUL_TOP / 2],
                     pos=[0, 0, FUL_TOP / 2], rgba=[0.55, 0.55, 0.6, 1], friction=[1.0, 0.01, 0.001])
        for k, sg in (("p", 1), ("n", -1)):
            ful.add_geom(name=f"stop_{k}", type=props.G.mjGEOM_BOX, size=[0.01, 0.03, STOP_TOP / 2],
                         pos=[sg * STOP_DY, 0, STOP_TOP / 2], rgba=[0.75, 0.2, 0.2, 1])
        b = w.add_body(name="bar", pos=[BAR_START[0], BAR_START[1], BAR_HALF[2] + 0.001])
        b.add_freejoint(name="bar_free")
        b.add_geom(name="bar", type=props.G.mjGEOM_BOX, size=list(BAR_HALF), mass=SHELL_MASS, rgba=wood,
                   friction=[1.0, 0.01, 0.001], condim=4)
        # Hidden slug (inside the bar, invisible and non-colliding).
        b.add_geom(name="slug", type=props.G.mjGEOM_BOX, size=[0.01, 0.015, 0.01], pos=[s, 0, 0], mass=SLUG_MASS,
                   rgba=[0, 0, 0, 0], contype=0, conaffinity=0, group=4)
        # Scale marks every 5 cm on the bar top, so probe positions are visible.
        for i in range(-3, 4):
            b.add_geom(type=props.G.mjGEOM_BOX, size=[0.0012, 0.012 if i else 0.015, 0.0005],
                       pos=[0.05 * i, 0, BAR_HALF[2] + 0.0005], rgba=[0.1, 0.1, 0.1, 1], contype=0, conaffinity=0)
        b.add_geom(type=props.G.mjGEOM_BOX, size=[0.006, 0.015, 0.0006], pos=[BAR_HALF[0] - 0.006, 0, BAR_HALF[2] + 0.0006],
                   rgba=[0.15, 0.35, 0.8, 1], contype=0, conaffinity=0)  # blue end = +x
        if self.difficulty == "hard":
            props.flat(spec, "mat", MAT_POS, MAT_HALF, props.RED)
            order = rng.permutation(len(SLUG_CARDS))
            for i, v in enumerate(SLUG_CARDS):
                props.block(spec, _card(v), (CARD_X, CARD_Y[order[i]]), CARD_HALF, 0.02,
                            label=f"{v:+d}cm", bg=(245, 245, 240))

    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._m = model
        self._bar = model.body("bar").id
        self._bar_g = model.geom("bar").id
        self._ful_g = model.geom("fulcrum").id
        self._stop_g = {model.geom("stop_p").id, model.geom("stop_n").id}
        self._tips = props.gripper_geoms(model)
        self.touch = props.TouchTracker(("bar",))
        self.touch.bind(model, data)
        self.probes = []
        self._since: float | None = None
        self._recorded = False
        self.balanced_since: float | None = None
        self._cardg = {model.geom(_card(v)).id: v for v in SLUG_CARDS} if self.difficulty == "hard" else {}

    # ---------- observables ----------
    def tilt_deg(self, data: mujoco.MjData) -> float:
        """Bar pitch; > 0 when the +x (blue) end is down."""
        r = data.xmat[self._bar].reshape(3, 3)
        return float(-np.rad2deg(np.arcsin(np.clip(r[2, 0], -1, 1))))

    def u_now(self, data: mujoco.MjData) -> float:
        """Bar-frame point currently over the fulcrum."""
        r = data.xmat[self._bar].reshape(3, 3)
        d = np.array([FUL_X, FUL_Y, 0]) - data.xpos[self._bar]
        return float(r[:, 0] @ d)

    def _touching(self, data: mujoco.MjData) -> tuple[bool, bool, bool]:
        ful = stop = held = False
        for c in data.contact[: data.ncon]:
            g1, g2 = int(c.geom1), int(c.geom2)
            if self._bar_g not in (g1, g2):
                continue
            o = g2 if g1 == self._bar_g else g1
            ful |= o == self._ful_g
            stop |= o in self._stop_g
            held |= o in self._tips
        return ful, stop, held

    def balanced(self, data: mujoco.MjData) -> bool:
        ful, stop, held = self._touching(data)
        return ful and not stop and not held and abs(self.tilt_deg(data)) < LEVEL_DEG

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        t = float(data.time)
        self.touch.update(data)
        ful, stop, held = self._touching(data)
        v = np.linalg.norm(data.cvel[self._bar])
        resting = (ful or stop) and not held and v < 0.01 and (ful or stop)
        on_rig = (ful or stop) and abs(self.tilt_deg(data)) < TIP_DEG + 2.0  # resting in the rig, not fallen
        if held or not on_rig:
            self._recorded = False
        if resting and on_rig:
            self._since = self._since if self._since is not None else t
            if t - self._since >= 1.0 and not self._recorded:
                tilt = self.tilt_deg(data)
                out = 0 if (abs(tilt) < LEVEL_DEG and ful and not stop) else (1 if tilt > 0 else -1)
                if out != 0 and abs(tilt) < TIPPED_DEG:
                    return
                self.probes.append(Probe(t, self.u_now(data), out))
                self._recorded = True
        else:
            self._since = None
        if self.balanced(data):
            self.balanced_since = self.balanced_since if self.balanced_since is not None else t
        else:
            self.balanced_since = None

    # ---------- inference over probes ----------
    def bracket(self) -> tuple[float, float]:
        """Interval for the CoM implied by the probes (bar frame, m)."""
        lo, hi = -BAR_HALF[0], BAR_HALF[0]
        for p in self.probes:
            if p.outcome > 0:
                lo = max(lo, p.u - FUL_HALF_W)
            elif p.outcome < 0:
                hi = min(hi, p.u + FUL_HALF_W)
            else:
                lo, hi = max(lo, p.u - FUL_HALF_W), min(hi, p.u + FUL_HALF_W)
        return lo, hi

    def slug_cards_left(self) -> list[int]:
        """Hard: slug cards consistent with the CoM bracket (slug = MASS_RATIO * CoM)."""
        lo, hi = self.bracket()
        return [v for v in SLUG_CARDS if MASS_RATIO * lo <= v / 100.0 <= MASS_RATIO * hi]

    def cards_on_mat(self, data: mujoco.MjData) -> list[int]:
        if not self._cardg:
            return []
        held = props.held_geoms(self._m, data, self._tips)
        return [v for g, v in self._cardg.items() if g not in held and props.in_box(
            data.xpos[self._m.body(_card(v)).id], MAT_POS, MAT_HALF, z_max=CARD_HALF[2] + 0.01)]

    def hud(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
        sc = self.score(model, data)
        lo, hi = self.bracket()
        lines = [f"t = {data.time:5.1f} s   bar tilt {self.tilt_deg(data):+5.2f} deg   over fulcrum u = "
                 f"{100 * self.u_now(data):+5.1f} cm   rubric stage {sc['stage']}/4",
                 f"probes {len(self.probes)}   CoM bracket [{100 * lo:+.1f}, {100 * hi:+.1f}] cm   "
                 f"balanced {'yes' if self.balanced(data) else 'no'}"]
        for p in self.probes[-3:]:
            res = {1: "far end down", -1: "near end down", 0: "level"}[p.outcome]
            lines.append(f"  u = {100 * p.u:+5.1f} cm: {res}")
        return lines

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        lo, hi = self.bracket()
        located = (hi - lo) <= 0.03 and lo <= self.com <= hi
        held_bal = self.balanced_since is not None and float(data.time) - self.balanced_since >= 3.0
        mat = self.cards_on_mat(data)
        success = held_bal
        if self.difficulty == "hard":
            # Hard: the probes must pin the slug to one card (slug = 1.8 x CoM offset), and that
            # card must be on the mat; stage 4 also needs the bar balanced.
            located = self.slug_cards_left() == [self.slug_card] and mat == [self.slug_card]
            success = held_bal and mat == [self.slug_card]
        stage = 0
        if self.touch.touched:
            stage = 1
        if self.probes:
            stage = 2
        if located:
            stage = 3
            if success:
                stage = 4
        return {
            "stage": stage,
            "success": float(success),
            "slug_cm": 100 * self.slug,
            "cards_on_mat": mat,
            "science_success": float(stage >= 3),
            "procedure_violations": 0,
            "probes": len(self.probes),
            "bracket_cm": [100 * lo, 100 * hi],
            "com_cm": 100 * self.com,
            "tilt_deg": self.tilt_deg(data),
            "log": [p.__dict__ for p in self.probes],
        }


def _card(v: int) -> str:
    return f"card_{'p' if v > 0 else 'n'}{abs(v)}"


# ---------- reference policies ----------
YAW = np.pi / 2  # finger axis along +y, across the bar's width


def _plan_bar(mo: Motion, side: str, grasp, center, target, pitch: float):  # noqa: ANN001, ANN202
    """(pick tilt, place tilt) that picks at ``grasp`` and sets the bar centre at ``target``
    level. The tool tilts about the finger axis (+y), the same axis the bar pitches about."""
    for tp in TILTS:
        r1 = grasp_rot(YAW, tp)
        if not mo._vertical_ok(side, grasp, r1):
            continue
        tq = tp - pitch
        r2 = grasp_rot(YAW, tq)
        off = r1.T @ (center - grasp)
        if mo._vertical_ok(side, target - r2 @ off, r2):
            return tp, tq
    return None


def _set_bar(env, mo: Motion, t: HiddenWeightBar, u: float):  # noqa: ANN001, ANN202
    """Pick the bar at its middle with the left arm and set it down level with bar point
    ``u`` over the fulcrum, keeping its heading."""
    d = env.data
    bar_c = d.xpos[t._bar].copy()
    pitch = np.deg2rad(t.tilt_deg(d))
    grasp = bar_c + np.array([0, 0, -0.004])
    target = np.array([FUL_X - u, FUL_Y, FUL_TOP + BAR_HALF[2] + 0.006])
    side = "left"
    plan = _plan_bar(mo, side, grasp, bar_c, target, pitch)
    if plan is None:
        raise RuntimeError("left arm cannot move the bar")
    tp, tq = plan
    yield from mo.pick(side, grasp, 0.0, body="bar", plan=(YAW, tp), open_to=0.03)
    off, _ = mo.held[side]
    # Level release if reachable; else the nearest reachable tilt (the bar then settles on its own).
    tilts = sorted({tq, *TILTS, *(tq + dd for dd in (-0.1, 0.1, -0.2, 0.2))}, key=lambda x: abs(x - tq))
    pos = mo.plan_vertical(side, target, yaws=(YAW,), tilts=tuple(tilts), offset=off)
    yield from mo.goto_above(side, pos[:2])
    top = np.array([pos[0], pos[1], SAFE_Z])
    yield from mo.line(side, top, pos, 1.5)
    yield from mo.hold(0.3)
    yield from mo.gripper(side, 0.03, 0.4)
    yield from mo.line(side, pos, top, 0.8)
    mo.held[side] = None
    yield from mo.home(side)
    yield from mo.hold(2.0)


def _balance(env, mo: Motion, t: HiddenWeightBar):  # noqa: ANN001, ANN202
    """Bisection on the drop direction until the bar rests level on the fulcrum."""
    lo, hi = -0.08, 0.08
    for _ in range(9):
        u = float(np.clip(0.5 * (lo + hi), -0.075, 0.075))
        yield from _set_bar(env, mo, t, u)
        tilt = t.tilt_deg(env.data)
        u_obs = t.u_now(env.data)
        if t.balanced(env.data):
            break
        if abs(tilt) > TIP_DEG + 2.0:  # bar fell off the rig: not a probe, try again
            continue
        if tilt > 0:  # far end down: CoM beyond u
            lo = max(lo, u_obs)
        else:
            hi = min(hi, u_obs)
        if hi - lo < 0.004:
            lo, hi = lo - 0.01, hi + 0.01


def _card_to_mat(env, mo: Motion, v: int):  # noqa: ANN001, ANN202
    t: HiddenWeightBar = env.task
    xy = env.data.xpos[env.model.body(_card(v)).id][:2]
    side = "left" if xy[1] > -0.305 else "right"
    yield from move_body(env, mo, _card(v), side, MAT_POS, 0.0, half=CARD_HALF[2], open_to=0.034, grip=0.0)


def _balance_point_card(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Balance the bar, then name the slug position as the balance point (the card nearest
    the CoM). Ignores that the empty shell also weighs something."""
    t: HiddenWeightBar = env.task
    mo = Motion(env)
    yield from _balance(env, mo, t)
    if t.difficulty == "hard":
        yield from _card_to_mat(env, mo, min(SLUG_CARDS, key=lambda v: abs(v / 100.0 - t.u_now(env.data))))
    yield from mo.hold(4.0)


def oracle(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Bisection until the bar balances. Hard: the slug sits 1.8x as far from the middle as
    the balance point (180 g bar, 100 g slug); carry that card to the mat."""
    t: HiddenWeightBar = env.task
    mo = Motion(env)
    yield from _balance(env, mo, t)
    if t.difficulty == "hard":
        com = t.u_now(env.data)
        yield from _card_to_mat(env, mo, min(SLUG_CARDS, key=lambda v: abs(v / 100.0 - MASS_RATIO * com)))
    yield from mo.hold(4.0)


def guess(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Blind baseline: set the bar on the fulcrum at its middle."""
    t: HiddenWeightBar = env.task
    mo = Motion(env)
    yield from _set_bar(env, mo, t, 0.0)
    yield from mo.hold(4.0)


TASK = HiddenWeightBar
POLICIES = {"oracle": oracle, "guess": guess, "noop": noop, "naive": _balance_point_card}
SPEC = TaskSpec(
    key="hidden_weight_bar",
    title="Balance a bar with a hidden weight",
    ability="Hidden structure",
    instruction=INSTRUCTION,
    objects=(
        "wooden bar 30 x 4 x 3 cm, tick marks every 5 cm, far end painted blue; a 100 g slug is sealed "
        "inside somewhere along it (bar 180 g total)",
        "fulcrum 7 cm tall with a 12 mm flat top, and two red stops 5.5 cm either side that catch the "
        "bar when it tips (about 3 deg)",
    ),
    setup=(
        "Fulcrum 30 cm in front of the arms, 10 cm to the right of the left arm's base, top edge "
        "running left-right; stops 5.5 cm in front of and behind it.",
        "Hard only: six 5 cm cards printed -12cm ... +12cm in a row 20 cm in front of the arms (7.5 cm "
        "apart, from 20 to 57.5 cm right of the left arm's base), random order; red mat 32 cm out, 30 cm right.",
        "Bar on the table pointing away from the robot, centred 27 cm in front of the left arm's base "
        "and 5 cm to its left, blue end far.",
    ),
    hidden="Slug position: 5-12 cm from the bar's middle, either side, so the centre of mass is 2.8-6.7 cm off-middle.",
    instruction_hard=(
        "A 100 g weight is sealed inside the 180 g bar. Leave the bar resting balanced (level) on the "
        "fulcrum, touching nothing else, and put the card showing where the weight sits on the red mat."
    ),
    hidden_hard=(
        "Slug position: one of -12, -9, -6, +6, +9, +12 cm from the bar's middle (+ toward the blue end), "
        "uniform. The balance point is only 100/180 of the slug offset (3.3, 5.0 or 6.7 cm), because the "
        "80 g shell is centred: naming the card nearest the balance point is wrong for 9 and 12 cm."
    ),
    rubric=(
        "No purposeful contact with the bar",
        "Touches or moves the bar",
        "At least one probe: bar released on the fulcrum and settled (level or tipped onto a stop)",
        "Probes bracket the centre of mass within 3 cm (and the bracket contains it); hard: the bracket scaled by 1.8 leaves one slug card, and that card is on the mat",
        "Stage 3 and the bar resting level on the fulcrum alone, untouched, for 3 s (hard: with the right card on the mat)",
    ),
    metrics=(
        "stage (0-4)",
        "success: bar balanced for 3 s, located or not",
        "science_success: stage >= 3",
        "probes used",
    ),
    report_fields=("probes", "bracket_cm", "com_cm", "tilt_deg", "slug_cm", "cards_on_mat"),
    max_seconds=180.0,
    limitations="The hidden slug is an invisible internal mass; the robot grasps the bar at its middle.",
)
