"""Task: calibrate a ramp on a practice lane, then land a ball in a target zone in one shot.

Ability: quantitative prediction. Hidden variable: the rolling resistance of the lanes,
which sets how far a ball released from a given ramp notch travels. For a ball of radius r
released from drop height h, energy balance gives a stopping distance of roughly
d = h r / mu_roll; contact losses at the ramp foot bend that line, so practice runs from
several notches calibrate d(h). Balls rest against release pins (one per notch); a button
drops all pins at once, so every release starts from rest. The first ball released on
the test lane is the only one that counts.
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

RB = 0.02  # ball radius
BALL_MASS = 0.06
THETA = np.deg2rad(20.0)
Y_TOP, Y_BOT = 0.11, -0.13
RAMP_X = 0.34
LANE = {"practice": 0.28, "test": 0.40}
NOTCH_H = (0.012, 0.032, 0.052, 0.072)  # pin height above the lane, m
NOTCH_H_HARD = (0.012, 0.027, 0.042, 0.057, 0.072)  # hard: five pins, 4.1 cm apart on the ramp
REST = {4: 0.024, 5: 0.0205}  # ball centre uphill of its pin (closer when pins are denser)
HARD_PRACTICE = 2  # hard: practice balls allowed before the test shot
PIN_UP, PIN_DROP, PIN_DOWN_S = 0.009, 0.02, 2.5
END_Y = -0.92
BALLS = {"white": (0.18, -0.22), "orange": (0.18, -0.40)}
BALL_RGBA = {"white": (0.92, 0.92, 0.90, 1), "orange": (0.95, 0.50, 0.10, 1)}
BUTTON = props.ButtonSpec("release", (0.17, 0.07), rgba=(0.9, 0.55, 0.1, 1))
QR = [np.cos(THETA / 2), np.sin(THETA / 2), 0, 0]  # ramp tilt (surface rises toward +y)

INSTRUCTION = (
    "Roll a ball down the ramp so that it stops inside the green target zone on the test lane "
    "(right lane, T). Only your first ball on the test lane counts. You may practise on the left "
    "lane (P) as often as you like. Rest a ball against one of the numbered release pins and "
    "press the orange button to drop the pins."
)


def ramp_surface(y: float) -> float:
    return float(np.clip(y - Y_BOT, 0.0, Y_TOP - Y_BOT) * np.tan(THETA))


def pin_y(k: int, notch: tuple[float, ...] = NOTCH_H) -> float:
    return Y_BOT + notch[k] / np.tan(THETA)


def ball_rest(lane: str, k: int, lift: float = 0.0, notch: tuple[float, ...] = NOTCH_H) -> np.ndarray:
    """Ball centre just uphill of pin k (it rolls the last few mm onto the pin)."""
    y = pin_y(k, notch) + REST[len(notch)]
    return np.array([LANE[lane], y, ramp_surface(y) + RB / np.cos(THETA) + lift])


def _add_ramp(spec: mujoco.MjSpec, notch: tuple[float, ...] = NOTCH_H) -> None:
    w = spec.worldbody
    L = (Y_TOP - Y_BOT) / np.cos(THETA)
    n = np.array([0.0, -np.sin(THETA), np.cos(THETA)])
    mid = np.array([RAMP_X, (Y_TOP + Y_BOT) / 2, ramp_surface(Y_TOP) / 2])
    w.add_geom(name="ramp", type=props.G.mjGEOM_BOX, size=[0.1, L / 2, 0.005], pos=list(mid - 0.005 * n),
               quat=QR, rgba=[0.62, 0.48, 0.30, 1], friction=[1.0, 0.005, 0.0])
    for k in range(len(notch)):  # release pins: thin bars across both lanes (mocap, drop together)
        y = pin_y(k, notch)
        p = np.array([RAMP_X, y, ramp_surface(y)]) + n * (PIN_UP / 2)
        pin = w.add_body(name=f"pin{k}", mocap=True, pos=list(p), quat=QR)
        pin.add_geom(name=f"pin{k}", type=props.G.mjGEOM_BOX, size=[0.1, 0.002, PIN_UP / 2],
                     rgba=[0.9, 0.55, 0.1, 1])


def _add_ball(spec: mujoco.MjSpec, name: str, pos, mu: float, rgba) -> None:  # noqa: ANN001
    body = spec.worldbody.add_body(name=f"ball_{name}", pos=list(pos))
    body.add_freejoint(name=f"ball_{name}_free")
    body.add_geom(name=f"ball_{name}", type=props.G.mjGEOM_SPHERE, size=[RB], mass=BALL_MASS,
                  rgba=list(rgba), condim=6, friction=[1.0, 0.005, mu])


def simulate_distances(mu: float, notch: tuple[float, ...] = NOTCH_H) -> list[float]:
    """Stopping distance from each pin in a ramp-only copy of the scene (same geometry,
    contact and integrator settings). Used only to place the target zone."""
    from robosci import scene

    base = scene.base_spec()
    sp = mujoco.MjSpec()
    for k in ("timestep", "cone", "integrator", "noslip_iterations", "impratio"):
        setattr(sp.option, k, getattr(base.option, k))
    sp.worldbody.add_geom(type=props.G.mjGEOM_BOX, size=[0.45, 0.65, 0.375], pos=[0.25, -0.305, -0.375],
                          friction=[0.8, 0.01, 0.001])
    _add_ramp(sp, notch)
    _add_ball(sp, "probe", (0, 0, 1), mu, (1, 1, 1, 1))
    m = sp.compile()
    d = mujoco.MjData(m)
    out = []
    for k in range(len(notch)):
        mujoco.mj_resetData(m, d)
        d.qpos[:3] = ball_rest("practice", k, lift=0.001, notch=notch)
        for _ in range(300):  # settle against the pin
            mujoco.mj_step(m, d)
        d.mocap_pos[:, 2] -= PIN_DROP
        for i in range(5000):
            mujoco.mj_step(m, d)
            if i > 300 and np.linalg.norm(d.qvel[:3]) < 0.002:
                break
        out.append(float(Y_BOT - d.qpos[1]))
    return out


@dataclass
class Run:
    t0: float
    ball: str
    lane: str
    notch: int
    dist: float | None = None
    done: bool = False


@dataclass
class RampShot:
    difficulty: str = "easy"
    key: str = "ramp_shot"
    instruction: str = INSTRUCTION
    mu: float = 0.002
    target_notch: int = 2
    runs: list[Run] = field(default_factory=list)

    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        self.notch_h = NOTCH_H if self.difficulty == "easy" else NOTCH_H_HARD
        self.mu = float(rng.uniform(0.0015, 0.0022))
        self.zone_half = 0.035 if self.difficulty == "easy" else 0.025
        self.target_notch = int(rng.integers(len(self.notch_h)))
        self.notch_d = simulate_distances(self.mu, self.notch_h)  # ground truth, used only to place the zone
        self.target_d = self.notch_d[self.target_notch]
        self.target_y = Y_BOT - self.target_d
        w = spec.worldbody
        _add_ramp(spec, self.notch_h)
        w.add_geom(type=props.G.mjGEOM_BOX, size=[0.1, 0.005, ramp_surface(Y_TOP) / 2 + 0.01],
                   pos=[RAMP_X, Y_TOP + 0.004, ramp_surface(Y_TOP) / 2 + 0.01], rgba=[0.5, 0.38, 0.24, 1])
        for k in range(len(self.notch_h)):
            props.text_plate(spec, f"notch{k}", (0.222, pin_y(k, self.notch_h) + 0.02), str(k + 1),
                             half=(0.012, 0.012))
        for lane, x in LANE.items():
            props.flat(spec, f"lane_{lane}", (x, (Y_BOT + END_Y) / 2), (0.03, (Y_BOT - END_Y) / 2),
                       (0.80, 0.78, 0.72, 1))
        for k in range(1, 8):  # 10 cm distance marks
            props.flat(spec, f"mark{k}", (RAMP_X, Y_BOT - 0.1 * k), (0.1, 0.0015), (0.2, 0.2, 0.2, 1), z=0.0002)
        props.flat(spec, "zone", (LANE["test"], self.target_y), (0.032, self.zone_half), (0.15, 0.70, 0.25, 1),
                   z=0.0004)
        props.text_plate(spec, "lbl_practice", (LANE["practice"], Y_BOT - 0.03), "P", half=(0.013, 0.013))
        props.text_plate(spec, "lbl_test", (LANE["test"], Y_BOT - 0.03), "T", half=(0.013, 0.013))
        w.add_geom(type=props.G.mjGEOM_BOX, size=[0.12, 0.006, 0.02], pos=[RAMP_X, END_Y - 0.006, 0.02],
                   rgba=[0.3, 0.3, 0.3, 1])
        props.push_button(spec, BUTTON)
        for b, (x, y) in BALLS.items():
            _add_ball(spec, b, (x, y, RB + 0.001), self.mu, BALL_RGBA[b])

    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._m = model
        self.touch = props.TouchTracker(tuple(f"ball_{b}" for b in BALLS) + ("release",))
        self.touch.bind(model, data)
        self.button = props.Button(BUTTON)
        self.button.bind(model)
        self._tips = props.gripper_geoms(model)
        self._bg = {model.geom(f"ball_{b}").id: b for b in BALLS}
        self._dof = {b: model.joint(f"ball_{b}_free").dofadr[0] for b in BALLS}
        self._pins = [model.body(f"pin{k}").mocapid[0] for k in range(len(self.notch_h))]
        self._pin_up = data.mocap_pos[self._pins].copy()
        self._pins_down_at: float | None = None
        self._still = {b: 0.0 for b in BALLS}
        self.runs = []

    def pos(self, data: mujoco.MjData, b: str) -> np.ndarray:
        return data.xpos[self._m.body(f"ball_{b}").id]

    def speed(self, data: mujoco.MjData, b: str) -> float:
        return float(np.linalg.norm(data.qvel[self._dof[b]:self._dof[b] + 3]))

    def held(self, data: mujoco.MjData) -> set[str]:
        return {self._bg[g] for g in props.held_geoms(self._m, data, self._tips) if g in self._bg}

    def resting_pin(self, data: mujoco.MjData, b: str) -> tuple[str, int] | None:
        """(lane, pin) if ball ``b`` rests against a pin in a lane."""
        p = self.pos(data, b)
        lane = min(LANE, key=lambda ln: abs(LANE[ln] - p[0]))
        if abs(LANE[lane] - p[0]) > 0.03:
            return None
        gap = 0.04 if len(self.notch_h) == 4 else 0.035
        for k in range(len(self.notch_h)):
            if pin_y(k, self.notch_h) < p[1] < pin_y(k, self.notch_h) + gap:
                return lane, k
        return None

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        t = float(data.time)
        self.touch.update(data)
        held = self.held(data)
        if self.button.update(data) and self._pins_down_at is None:
            for b in BALLS:
                rp = self.resting_pin(data, b)
                if rp and b not in held and self.speed(data, b) < 0.05:
                    self.runs.append(Run(t, b, rp[0], rp[1]))
            data.mocap_pos[self._pins] = self._pin_up - [0, 0, PIN_DROP]
            self._pins_down_at = t
        if self._pins_down_at is not None and t - self._pins_down_at > PIN_DOWN_S:
            data.mocap_pos[self._pins] = self._pin_up
            self._pins_down_at = None
        for r in self.runs:
            if r.done:
                continue
            if r.ball in held:
                r.done = True
            elif self.speed(data, r.ball) < 0.003 and t - r.t0 > 0.6:
                self._still[r.ball] += 0.02
                if self._still[r.ball] >= 0.4:
                    r.done, r.dist = True, float(Y_BOT - self.pos(data, r.ball)[1])
            else:
                self._still[r.ball] = 0.0

    # ---------- evidence ----------
    def practice_before_test(self) -> list[Run]:
        out = []
        for r in self.runs:
            if r.lane == "test":
                break
            if r.done and r.dist is not None:
                out.append(r)
        return out

    def test_shot(self) -> Run | None:
        return next((r for r in self.runs if r.lane == "test"), None)

    def in_zone(self, r: Run | None) -> bool:
        return bool(r is not None and r.done and r.dist is not None
                    and abs((Y_BOT - r.dist) - self.target_y) <= self.zone_half)

    def fit_pin(self, prac: list[Run]) -> int | None:
        """Pin whose straight-line-fit distance (least squares on the practice runs) is nearest
        the zone centre; None without two distinct practice pins."""
        if len({r.notch for r in prac}) < 2:
            return None
        h = [self.notch_h[r.notch] for r in prac]
        a, b = np.polyfit(h, [r.dist for r in prac], 1)
        pred = a * np.asarray(self.notch_h) + b
        return int(np.argmin(np.abs(pred - self.target_d)))

    def hud(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
        sc = self.score(model, data)
        lines = [f"t = {data.time:5.1f} s   target zone {self.target_d:.2f} m past the ramp foot   "
                 f"pins {'DOWN' if self._pins_down_at is not None else 'up'}",
                 f"rubric stage {sc['stage']}/4   practice runs {sc['practice_runs']} "
                 f"(pins {sc['practice_notches']})   test shots {sc['test_shots']}"]
        for r in self.runs[-3:]:
            d = f"{r.dist:.2f} m" if r.dist is not None else ("..." if not r.done else "picked up")
            lines.append(f"  {r.lane} lane, pin {r.notch + 1}: {d}")
        return lines

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        prac = self.practice_before_test()
        notches = sorted({r.notch for r in prac})
        shot = self.test_shot()
        success = self.in_zone(shot)
        n_prac = sum(1 for r in self.runs[: self.runs.index(shot)] if r.lane == "practice") if shot \
            else sum(r.lane == "practice" for r in self.runs)
        over = max(0, n_prac - HARD_PRACTICE) if self.difficulty == "hard" else 0
        stage = 0
        if self.touch.touched or self.button.presses:
            stage = 1
        if prac:
            stage = 2
        if self.difficulty == "hard":
            # Two practice pins, within the budget, and the test shot from the pin a straight-line
            # fit d(h) through the practice runs puts nearest the zone (not a copied practice pin).
            ok = (len(notches) >= 2 and over == 0 and shot is not None
                  and shot.notch == self.fit_pin(prac))
        else:
            ok = len(notches) >= 2
        if ok:
            stage = 3
            if success:
                stage = 4
        return {
            "stage": stage,
            "success": float(success),
            "science_success": float(stage >= 3),
            "practice_runs": len(prac),
            "practice_notches": [k + 1 for k in notches],
            "test_shots": sum(r.lane == "test" for r in self.runs),
            "test_dist": shot.dist if shot is not None else None,
            "target_dist": self.target_d,
            "target_notch": self.target_notch + 1,
            "procedure_violations": max(0, sum(r.lane == "test" for r in self.runs) - 1) + over,
            "fit_pin": (p + 1) if (p := self.fit_pin(prac)) is not None else None,
            "test_pin": shot.notch + 1 if shot is not None else None,
            "log": [r.__dict__ for r in self.runs],
        }


# ---------- reference policies ----------
def _set_ball(env, mo: Motion, ball: str, lane: str, k: int):  # noqa: ANN001, ANN202
    c = ball_rest(lane, k, notch=env.task.notch_h)  # move_body releases the centre 4 mm above this point
    return move_body(env, mo, f"ball_{ball}", "left", c[:2], c[2] - RB, half=RB, dz=0.0,
                     grip=0.004, open_to=0.035)


def _shoot(env, mo: Motion, ball: str):  # noqa: ANN001, ANN202
    t: RampShot = env.task
    yield from mo.hold(0.8)
    yield from mo.press("left", t.button.top(), depth=BUTTON.travel + 0.004)
    yield from mo.home("left")
    for _ in range(300):  # watch until the ball stops (long rolls take ~20 s)
        yield from mo.hold(0.1)
        r = next((r for r in reversed(t.runs) if r.ball == ball), None)
        if r is not None and r.done:
            return


def _practice(env, mo: Motion, pins):  # noqa: ANN001, ANN202
    t: RampShot = env.task
    hs, ds = [], []
    for k in pins:
        yield from _set_ball(env, mo, "white", "practice", k)
        yield from _shoot(env, mo, "white")
        hs.append(t.notch_h[k])
        ds.append(Y_BOT - float(t.pos(env.data, "white")[1]))
    return hs, ds


def _interp(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Practice runs from pins 1, 3, 4; interpolate d(h); shoot the test ball from the pin whose
    predicted distance is closest to the zone. Reads ball positions and the zone only."""
    t: RampShot = env.task
    mo = Motion(env)
    hs, ds = yield from _practice(env, mo, (0, 2, 3))
    d_target = Y_BOT - t.target_y  # the green zone is visible on the table
    pred = np.interp(t.notch_h, hs, ds)  # piecewise-linear calibration curve d(h)
    k = int(np.argmin(np.abs(pred - d_target)))
    yield from _set_ball(env, mo, "orange", "test", k)
    yield from _shoot(env, mo, "orange")
    yield from mo.hold(1.0)


def _two_shot_fit(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Hard: two practice runs (pins 1 and 5), straight-line fit d = a h + b, shoot from the pin
    the fit puts nearest the zone (interpolating between or beyond the practised pins)."""
    t: RampShot = env.task
    mo = Motion(env)
    hs, ds = yield from _practice(env, mo, (0, len(t.notch_h) - 1))
    a, b = np.polyfit(hs, ds, 1)
    k = int(np.argmin(np.abs(a * np.asarray(t.notch_h) + b - (Y_BOT - t.target_y))))
    yield from _set_ball(env, mo, "orange", "test", k)
    yield from _shoot(env, mo, "orange")
    yield from mo.hold(1.0)


def naive(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Two practice runs (pins 1 and 5), then copy whichever practice pin landed nearer the zone."""
    t: RampShot = env.task
    mo = Motion(env)
    pins = (0, len(t.notch_h) - 1)
    _, ds = yield from _practice(env, mo, pins)
    k = pins[int(np.argmin(np.abs(np.asarray(ds) - (Y_BOT - t.target_y))))]
    yield from _set_ball(env, mo, "orange", "test", k)
    yield from _shoot(env, mo, "orange")
    yield from mo.hold(1.0)


def oracle(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Easy: three practice pins and interpolation. Hard: two practice pins and a line fit."""
    pol = _two_shot_fit if env.task.difficulty == "hard" else _interp
    yield from pol(env, rng)


def guess(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Blind baseline: one test shot from a random pin, no practice."""
    rng = rng or np.random.default_rng()
    mo = Motion(env)
    yield from _set_ball(env, mo, "orange", "test", int(rng.integers(len(env.task.notch_h))))
    yield from _shoot(env, mo, "orange")
    yield from mo.hold(1.0)


TASK = RampShot
POLICIES = {"oracle": oracle, "guess": guess, "noop": noop, "naive": naive}
SPEC = TaskSpec(
    key="ramp_shot",
    title="Calibrate a ramp, then land the ball in the zone in one shot",
    ability="Quantitative prediction",
    instruction=INSTRUCTION,
    objects=(
        "two-lane ramp, 24 cm long at 20 deg, with four (hard: five) numbered release pins (1.2 to "
        "7.2 cm above the lane) that an orange button drops for 2.5 s",
        "two flat lanes, 80 cm long, with 10 cm marks: practice lane (left, P) and test lane (right, T); "
        "end wall",
        "green target zone on the test lane (7 cm long easy, 5 cm hard)",
        "white practice ball and orange test ball, 4 cm diameter, 60 g",
    ),
    setup=(
        "Ramp top 11 cm in front of the left arm's base line, lanes running toward the right arm.",
        "Tape the target zone on the test lane at the distance drawn for this episode.",
        "Put both balls on the table in front of the arms; release button left of the ramp top.",
    ),
    hidden=(
        "Rolling resistance of the lanes (same for both lanes), drawn per episode, so stopping "
        "distances from each pin change between episodes. The zone is centred on one pin's distance."
    ),
    instruction_hard=(
        "Roll the orange ball down the ramp so that it stops inside the green zone on the far lane (T). "
        "Only your first roll on lane T counts. Before it you may practise at most twice on the near lane "
        "(P) with the white ball, fetching it back between rolls. A ball rests against one of the numbered "
        "pins; the orange button drops the pins."
    ),
    hidden_hard=(
        "As easy, but five pins (1.2 to 7.2 cm high, 1.5 cm apart) and a 5 cm zone centred on one "
        "pin's distance. Only two practice runs are allowed, so at most two pins can be tried: the "
        "zone pin must be predicted by fitting distance against pin height and interpolating or "
        "extrapolating. Shooting from whichever practised pin landed nearer the zone fails when the "
        "zone belongs to an untried pin (3 of 5 pins for a fixed pair)."
    ),
    rubric=(
        "No purposeful contact with a ball or the release button",
        "Touches a ball or presses the button",
        "At least one practice run (ball released from a pin and stopped) before the test shot",
        "Practice runs from at least two different pins before the test shot (enough to fit d(h)); "
        "hard: at most two practice runs, and the test pin is the one a straight-line fit through them "
        "predicts",
        "Stage 3 and the first test-lane ball stops in the target zone",
    ),
    metrics=(
        "stage (0-4)",
        "success: first test shot stops in the zone, calibrated or not",
        "science_success: stage >= 3",
        "practice runs and pins used; test-shot distance vs target",
        "procedure violations: extra test-lane shots (only the first counts)",
    ),
    report_fields=("practice_runs", "practice_notches", "test_dist", "target_dist", "target_notch"),
    max_seconds=180.0,
    limitations=("Lane rolling resistance is MuJoCo's rolling-friction coefficient on the ball. "
                 "Release pins are kinematic bars that drop together."),
)
