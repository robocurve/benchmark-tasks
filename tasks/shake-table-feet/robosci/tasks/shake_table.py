"""Task: which of three look-alike feet holds its place best on a shake table?

Ability: controlled experiments. Hidden variable: the friction coefficient of each foot's
rubber. The deck snaps forward and creeps back, so a foot that slips during the snap
ratchets along the deck; the lower its friction, the farther it walks. Two settings exist: LOW is too gentle to move the two better feet, so it cannot tell
them apart, and comparing feet shaken at different settings is not a fair test. Only a
comparison at one setting that separates all three counts as evidence.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import mujoco
import numpy as np

from robosci import props
from robosci.motion import SAFE_Z, TILTS, Motion, grasp_rot
from robosci.skills import noop
from robosci.specs import TaskSpec

FEET = ("F1", "F2", "F3")
F_HALF = (0.025, 0.025, 0.02)
F_MASS = 0.10
DECK_XY = (0.30, -0.30)
DECK_HALF = (0.15, 0.075)
DECK_TOP = 0.03
SPOTS = ((0.22, -0.30), (0.30, -0.30), (0.38, -0.30))
START = {"F1": (0.24, -0.05), "F2": (0.32, -0.05), "F3": (0.40, -0.05)}
SETTINGS = {"LOW": 0.0012, "HIGH": 0.003}  # stroke, m: snap forward (one tick), creep back
RETURN_STEPS, SHAKE_S = 14, 4.0  # creep back in 14 control ticks (20 ms each)
BUTTONS = {"LOW": props.ButtonSpec("btn_low", (0.17, 0.04), rgba=(0.95, 0.80, 0.15, 1)),
           "HIGH": props.ButtonSpec("btn_high", (0.17, -0.05), rgba=(0.85, 0.15, 0.15, 1))}
MAT_XY, MAT_HALF = (0.48, -0.08), (0.055, 0.055)
MARGIN = 0.003  # drifts closer than this are a tie
PAD_GAINS = (0.6, 1.0, 1.6)  # hard: stroke multiplier of the three pads, one end to the other
PAD_HALF_X = 0.0375

INSTRUCTION = (
    "Feet F1, F2 and F3 look the same but grip differently. Use the shake table to find the foot that "
    "stays in place best and put it on the red mat. The yellow button shakes gently (LOW), the red "
    "button shakes hard (HIGH); each shake lasts 4 seconds."
)


def stroke(tick: int) -> float:
    """Deck target in units of the stroke: a snap forward (one control tick, the servo makes
    it a sharp jolt), then a slow creep back in small steps that no foot slips on."""
    k = tick % (RETURN_STEPS + 1)
    return 1.0 - k / RETURN_STEPS


@dataclass
class Shake:
    t0: float
    setting: str
    start: dict[str, np.ndarray]
    pad: dict[str, int] = field(default_factory=dict)  # which pad each foot stood on
    drift: dict[str, float] = field(default_factory=dict)
    fell: list[str] = field(default_factory=list)
    done: bool = False


@dataclass
class ShakeTable:
    difficulty: str = "easy"
    key: str = "shake_table"
    instruction: str = INSTRUCTION
    mu: dict[str, float] = field(default_factory=dict)
    shakes: list[Shake] = field(default_factory=list)

    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        mus = (0.15, 0.35, 0.70) if self.difficulty == "easy" else (0.20, 0.40, 0.60)
        perm = rng.permutation(3)
        self.mu = {f: float(mus[perm[i]]) for i, f in enumerate(FEET)}
        if self.difficulty == "hard":
            # Position confound: three pads under the spots, one driver each; the stroke grows
            # toward one end of the table (which end is random).
            g = np.array(PAD_GAINS)
            self.gains = tuple(float(x) for x in (g if rng.random() < 0.5 else g[::-1]))
        else:
            self.gains = (1.0,)
        w = spec.worldbody
        base = w.add_body(name="shaker_base", pos=[DECK_XY[0], DECK_XY[1], 0])
        base.add_geom(type=props.G.mjGEOM_BOX, size=[DECK_HALF[0] - 0.01, DECK_HALF[1] - 0.02, 0.01],
                      pos=[0, 0, 0.01], rgba=[0.2, 0.2, 0.22, 1], contype=0, conaffinity=0)  # deck rides on its slide
        for name, cx, hx in self._pad_layout():
            deck = base.add_body(name=name, pos=[cx - DECK_XY[0], 0, DECK_TOP - 0.005])
            deck.add_joint(name=f"{name}_slide", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 1, 0],
                           range=[-0.03, 0.03], limited=mujoco.mjtLimited.mjLIMITED_TRUE)
            deck.add_geom(name=name, type=props.G.mjGEOM_BOX, size=[hx, DECK_HALF[1], 0.005],
                          mass=0.5, rgba=[0.80, 0.82, 0.86, 1], friction=[0.05, 0.005, 0.0001])
            for x, y in SPOTS:  # printed spots on the deck
                if abs(x - cx) > hx:
                    continue
                deck.add_geom(type=props.G.mjGEOM_BOX, size=[0.028, 0.028, 0.0005],
                              pos=[x - cx, y - DECK_XY[1], 0.0055], rgba=[0.3, 0.3, 0.35, 1],
                              contype=0, conaffinity=0, mass=0)
                deck.add_geom(type=props.G.mjGEOM_BOX, size=[0.024, 0.024, 0.0006],
                              pos=[x - cx, y - DECK_XY[1], 0.0056], rgba=[0.80, 0.82, 0.86, 1],
                              contype=0, conaffinity=0, mass=0)
            act = spec.add_actuator(name=f"shaker_{name}", target=f"{name}_slide",
                                    trntype=mujoco.mjtTrn.mjTRN_JOINT)
            act.set_to_position(kp=3000.0, kv=100.0)  # ~60 rad/s, near critically damped
        for b in BUTTONS.values():
            props.push_button(spec, b)
        props.text_plate(spec, "lbl_low", (0.235, 0.04), "LOW", half=(0.02, 0.02))
        props.text_plate(spec, "lbl_high", (0.235, -0.05), "HIGH", half=(0.02, 0.02))
        props.flat(spec, "mat", MAT_XY, MAT_HALF, props.RED)
        for f in FEET:
            props.block(spec, f"foot_{f}", START[f], F_HALF, F_MASS, label=f, bg=(60, 60, 64),
                        fg=(240, 240, 240), friction=(self.mu[f], 0.005, 0.0001))

    def _pad_layout(self) -> list[tuple[str, float, float]]:
        """(body name, centre x, half length x) of each shaking pad."""
        if self.difficulty != "hard":
            return [("deck", DECK_XY[0], DECK_HALF[0])]
        return [(f"deck{i}", x, PAD_HALF_X) for i, (x, _) in enumerate(SPOTS)]

    # ---------- runtime ----------
    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._m = model
        self.buttons = {k: props.Button(b) for k, b in BUTTONS.items()}
        for b in self.buttons.values():
            b.bind(model)
        self.touch = props.TouchTracker(tuple(f"foot_{f}" for f in FEET) + ("btn_low", "btn_high"))
        self.touch.bind(model, data)
        self._tips = props.gripper_geoms(model)
        self._g = {model.geom(f"foot_{f}").id: f for f in FEET}
        self._pads = [(model.body(n).id, model.actuator(f"shaker_{n}").id, cx, hx)
                      for n, cx, hx in self._pad_layout()]
        self.active: Shake | None = None
        self.shakes = []

    def pos(self, data: mujoco.MjData, f: str) -> np.ndarray:
        return data.xpos[self._m.body(f"foot_{f}").id]

    def deck_y(self, data: mujoco.MjData, pad: int = 0) -> float:
        return float(data.xpos[self._pads[pad][0]][1])

    def held(self, data: mujoco.MjData) -> set[str]:
        return {self._g[g] for g in props.held_geoms(self._m, data, self._tips) if g in self._g}

    def pad_of(self, data: mujoco.MjData, f: str) -> int | None:
        p = self.pos(data, f)
        if not DECK_TOP < p[2] < DECK_TOP + 2 * F_HALF[2] + 0.01:
            return None
        for i, (_, _, cx, hx) in enumerate(self._pads):
            if props.in_box(p, (cx, self.deck_y(data, i)), (hx, DECK_HALF[1])):
                return i
        return None

    def on_deck(self, data: mujoco.MjData, f: str) -> bool:
        return self.pad_of(data, f) is not None

    def rel(self, data: mujoco.MjData, f: str, pad: int | None = None) -> np.ndarray:
        """Foot position relative to the deck it stands on (pad 0 if it stands on none)."""
        if pad is None:
            pad = self.pad_of(data, f) or 0
        return self.pos(data, f)[:2] - [self._pads[pad][2], self.deck_y(data, pad)]

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        t = float(data.time)
        self.touch.update(data)
        pressed = [k for k, b in self.buttons.items() if b.update(data)]
        if pressed and self.active is None:
            held = self.held(data)
            pads = {f: self.pad_of(data, f) for f in FEET if f not in held}
            pads = {f: i for f, i in pads.items() if i is not None}
            start = {f: self.rel(data, f, i).copy() for f, i in pads.items()}
            self.active = Shake(t, pressed[0], start, pad=pads)
            self.shakes.append(self.active)
        sh = self.active
        if sh is None:
            return
        el = t - sh.t0
        if el < SHAKE_S:
            for (_, a, _, _), g in zip(self._pads, self.gains, strict=True):
                data.ctrl[a] = SETTINGS[sh.setting] * g * stroke(int(round(el / 0.02)))
        else:
            for _, a, _, _ in self._pads:
                data.ctrl[a] = 0.0
            if el > SHAKE_S + 0.6:  # settled: measure how far each foot walked on its deck
                for f, p0 in sh.start.items():
                    if self.pad_of(data, f) == sh.pad[f]:
                        sh.drift[f] = float(np.linalg.norm(self.rel(data, f, sh.pad[f]) - p0))
                    else:
                        sh.fell.append(f)
                        sh.drift[f] = float("inf")
                sh.done = True
                self.active = None

    # ---------- evidence ----------
    def proven_best(self) -> str | None:
        """Foot with the least drift among all three, all tested at one setting and on the same
        pad (or, in hard mode, each foot on every pad at one setting), by a margin."""
        n = len(self._pads)
        for setting in SETTINGS:
            d: dict[tuple[str, int], float] = {}
            for sh in self.shakes:
                if sh.done and sh.setting == setting:
                    for f, v in sh.drift.items():
                        k = (f, sh.pad[f])
                        d[k] = max(d.get(k, 0.0), v)
            cands = [{f: d[(f, i)] for f in FEET if (f, i) in d} for i in range(n)]
            if n > 1 and all((f, i) in d for f in FEET for i in range(n)):  # balanced rotation
                cands.append({f: sum(d[(f, i)] for i in range(n)) for f in FEET})
            for c in cands:
                if len(c) == 3:
                    order = sorted(c, key=c.__getitem__)
                    if c[order[1]] - c[order[0]] > MARGIN:
                        return order[0]
        return None

    def best(self) -> str:
        return max(self.mu, key=self.mu.__getitem__)

    def on_mat(self, data: mujoco.MjData) -> set[str]:
        held = self.held(data)
        return {f for f in FEET if f not in held and props.in_box(self.pos(data, f), MAT_XY, MAT_HALF, z_max=0.05)}

    def hud(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
        sc = self.score(model, data)
        state = f"SHAKING {self.active.setting}" if self.active else "idle"
        lines = [f"t = {data.time:5.1f} s   table {state}   on deck "
                 f"{[f for f in FEET if self.on_deck(data, f)] or '-'}   mat {sorted(sc['on_mat']) or '-'}",
                 f"rubric stage {sc['stage']}/4   shakes {sc['shakes']}   best (fair test) {sc['proven_best'] or '?'}"]
        for sh in [s for s in self.shakes if s.done][-2:]:
            res = "  ".join(f"{f} {v * 100:.1f} cm" if np.isfinite(v) else f"{f} fell" for f, v in sorted(sh.drift.items()))
            lines.append(f"  {sh.setting}: {res or 'no feet on deck'}")
        return lines

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        mat = self.on_mat(data)
        success = mat == {self.best()}
        proven = self.proven_best()
        stage = 0
        if self.touch.touched or any(b.presses for b in self.buttons.values()):
            stage = 1
        if any(sh.done and sh.drift for sh in self.shakes):
            stage = 2
        if proven == self.best():
            stage = 3
            if success:
                stage = 4
        return {
            "stage": stage,
            "success": float(success),
            "science_success": float(stage >= 3),
            "shakes": len(self.shakes),
            "settings_used": sorted({sh.setting for sh in self.shakes}),
            "proven_best": proven,
            "best": self.best(),
            "on_mat": sorted(mat),
            "procedure_violations": sum(len(sh.fell) for sh in self.shakes),  # feet shaken off the deck
            "log": [{"t0": sh.t0, "setting": sh.setting, "drift": sh.drift} for sh in self.shakes],
        }


# ---------- reference policies ----------
def _carry_yaw(env, mo: Motion, body: str, target, *, dz: float, grip: float = 0.005,  # noqa: ANN001
               open_to: float = 0.04, yaws=(np.pi / 2, -np.pi / 2)):  # noqa: ANN001, ANN202
    """Pick/place with the fingers along world y and the same tilt at both ends (the object
    never turns in the hand). Private copy of the helper in friction_fair_test."""
    m, d = env.model, env.data
    c0 = d.xpos[m.body(body).id].copy()
    grasp = c0 + [0, 0, dz]
    target = np.asarray(target, float)
    arms = ("left", "right") if c0[1] > -0.3 else ("right", "left")
    for arm in arms:
        plan = None
        for tilt in TILTS:
            for y1 in yaws:
                r1 = grasp_rot(y1, tilt)
                if not mo._vertical_ok(arm, grasp, r1):
                    continue
                off = r1.T @ (c0 - grasp)
                for y2 in yaws:
                    rot = grasp_rot(y2, tilt)
                    lo = target - rot @ off
                    if mo._vertical_ok(arm, lo, rot):
                        plan = ((y1, tilt), (y2, tilt, lo, rot))
                        break
                if plan:
                    break
            if plan:
                break
        if plan is None:
            continue
        pick, (y, tilt, lo, rot) = plan
        yield from mo.pick(arm, grasp, grip, body=body, plan=pick, open_to=open_to)
        if mo.held[arm] is not None:
            lo2 = target - rot @ mo.held[arm][0]
            if mo._vertical_ok(arm, lo2, rot):
                lo = lo2
        mo.rot[arm], mo.yaw[arm], mo.tilt[arm] = rot, y, tilt
        yield from mo.goto_above(arm, lo[:2])
        top = np.array([lo[0], lo[1], SAFE_Z])
        yield from mo.line(arm, top, lo, 1.2)
        yield from mo.gripper(arm, open_to, 0.5)
        yield from mo.line(arm, lo, top, 0.8)
        mo.held[arm] = None
        yield from mo.home(arm)
        return
    raise RuntimeError(f"no arm can carry {body} to {np.round(target, 3)}")


def _carry(env, mo: Motion, f: str, xy, surface: float):  # noqa: ANN001, ANN202
    return _carry_yaw(env, mo, f"foot_{f}", (xy[0], xy[1], surface + F_HALF[2] + 0.004), dz=-0.004)


def _shake(env, mo: Motion, setting: str = "HIGH"):  # noqa: ANN001, ANN202
    t: ShakeTable = env.task
    n = len(t.shakes)
    yield from mo.press("left", t.buttons[setting].top(), depth=BUTTONS[setting].travel + 0.004)
    yield from mo.home("left")
    for _ in range(60):
        yield from mo.hold(0.1)
        if len(t.shakes) > n and t.shakes[-1].done:
            break


def _same_spot(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Controlled for position too: each foot alone on the middle spot, one HIGH shake each;
    the foot that walked least goes on the mat. Reads foot positions on the deck only."""
    t: ShakeTable = env.task
    mo = Motion(env)
    walked: dict[str, float] = {}
    for k, f in enumerate(FEET):
        yield from _carry(env, mo, f, SPOTS[1], DECK_TOP)
        before = t.rel(env.data, f).copy()
        yield from _shake(env, mo)
        walked[f] = float(np.linalg.norm(t.rel(env.data, f) - before)) if t.on_deck(env.data, f) else 9.0
        if k < len(FEET) - 1:
            yield from _carry(env, mo, f, START[f], 0.0)
    best = min(walked, key=walked.__getitem__)
    yield from _carry(env, mo, best, MAT_XY, 0.0)
    yield from mo.hold(1.5)


def oracle(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Easy: all three feet shaken together. Hard: one foot at a time on the same spot."""
    pol = _same_spot if env.task.difficulty == "hard" else _all_at_once
    yield from pol(env, rng)


def _all_at_once(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Controlled test: all three feet on the deck, one HIGH shake, compare how far each walked;
    the one that moved least goes on the mat. Reads foot positions on the deck only."""
    t: ShakeTable = env.task
    mo = Motion(env)
    for f, xy in zip(FEET, SPOTS, strict=True):
        yield from _carry(env, mo, f, xy, DECK_TOP)
    before = {f: t.rel(env.data, f).copy() for f in FEET}
    n = len(t.shakes)
    yield from mo.press("left", t.buttons["HIGH"].top(), depth=BUTTONS["HIGH"].travel + 0.004)
    yield from mo.home("left")
    for _ in range(60):
        yield from mo.hold(0.1)
        if len(t.shakes) > n and t.shakes[-1].done:
            break
    walked = {f: float(np.linalg.norm(t.rel(env.data, f) - before[f])) if t.on_deck(env.data, f) else 9.0
              for f in FEET}
    best = min(walked, key=walked.__getitem__)
    yield from _carry(env, mo, best, MAT_XY, 0.0)
    yield from mo.hold(1.5)


def guess(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Blind baseline: a random foot to the mat, no shaking."""
    rng = rng or np.random.default_rng()
    mo = Motion(env)
    yield from _carry(env, mo, str(rng.choice(FEET)), MAT_XY, 0.0)
    yield from mo.hold(1.5)


TASK = ShakeTable
POLICIES = {"oracle": oracle, "guess": guess, "noop": noop, "naive": _all_at_once}
SPEC = TaskSpec(
    key="shake_table",
    title="Find the steadiest foot on a shake table",
    ability="Controlled experiments",
    instruction=INSTRUCTION,
    objects=(
        "three feet F1-F3, 5 x 5 x 4 cm, 100 g, identical dark rubber look, hidden grip",
        "shake table: 30 x 15 cm deck with three printed spots, on a slide that snaps forward and creeps "
        "back, for 4 s per press",
        "two buttons: yellow LOW (1.2 mm stroke) and red HIGH (3 mm stroke)",
        "red target mat, 11 x 11 cm",
    ),
    setup=(
        "Shake table centred between the arms, 30 cm in front of them, stroke along the left-right axis.",
        "Feet in a row 5 cm in front of the left arm's base line, 8 cm apart.",
        "Buttons by the left arm; red mat on the far left of the table.",
    ),
    hidden=(
        "Friction coefficient of each foot, a random permutation of 0.15 / 0.35 / 0.70. On LOW only the "
        "worst foot slips, so LOW cannot rank the other two."
    ),
    instruction_hard=(
        "Feet F1, F2 and F3 look the same but grip differently. Find the foot that stays in place best "
        "on the shake table and put it on the red mat. The two buttons start the shake table."
    ),
    hidden_hard=(
        "Foot friction a random permutation of 0.20 / 0.40 / 0.60. The deck is three pads, one under "
        "each spot, and the stroke grows toward one end: 0.6x / 1.0x / 1.6x the setting's stroke, end "
        "chosen at random. Feet shaken on different spots are not comparable; shaking all three at "
        "once picks the wrong foot in about half the episodes."
    ),
    rubric=(
        "No purposeful contact with the feet or the buttons",
        "Touches a foot or presses a button",
        "At least one shake with a foot on the deck, and its drift observed",
        "All three feet shaken at the same setting (hard: also on the same spot, or each foot on every "
        "spot), and the least-drifting foot is separated by more than 3 mm (the true steadiest foot)",
        "Stage 3 and only the steadiest foot is on the mat",
    ),
    metrics=(
        "stage (0-4)",
        "success: only the steadiest foot on the mat, proof or not",
        "science_success: stage >= 3",
        "shakes used and settings used",
        "procedure violations: feet shaken off the deck",
    ),
    report_fields=("shakes", "settings_used", "proven_best", "best", "on_mat"),
    max_seconds=180.0,  # hard oracle: three separate shakes (~150 s)
    limitations=("The deck is a position-controlled slide following a scripted stroke; foot grip is "
                 "MuJoCo's Coulomb friction on the foot (the deck itself is near-frictionless)."),
)
