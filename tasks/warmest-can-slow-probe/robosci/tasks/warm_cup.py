"""Task: find the warmest of three sealed cans with a slow, noisy temperature probe.

Ability: noise and replication. Hidden variables: each can's water temperature, the probe's
response time and its reading noise. A can stands on the probe pad; the display beside it shows
the pad's sensor temperature in deg C (0.1 C steps), refreshed every 0.5 s with fresh noise.
The sensor follows the can on it as a first-order lag (time constant tau) and relaxes to room
temperature (22 C) when the pad is empty.

Easy: tau 1 s, noise 0.3 C, cans 6-8 C apart: one quick reading per can is enough.
Hard: tau 5 s, noise 1.0 C, cans 2.0-2.5 C apart. A reading 2 s after setting a can down is
only a third of the way from the previous value, so quick readings rank the cans by the order
they were tested. A careful robot waits for the display to stop drifting (about 5 tau) and
averages several readings per can; the drift and the scatter are both visible on the display.
The scorer judges "settled" from the display alone: the readings from the point where a straight
line through the rest of the stretch has no significant slope (at least 8 readings). Science is
proven when every can has settled readings and the warmest settled mean beats every other can by
more than 2 standard errors, with the standard error from the pooled scatter of the readings
themselves (no assumed noise level or response time).
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

CANS = ("A", "B", "C")
HALF = 0.02
CAN_XY = {"A": (0.22, -0.20), "B": (0.22, -0.305), "C": (0.22, -0.41)}
PAD, PAD_HALF = (0.32, -0.305), 0.035
DISPLAY = (0.34, -0.15, 0.02)
MAT_POS, MAT_HALF = (0.30, 0.06), 0.06
ROOM = 22.0
REFRESH = 0.5
PARAMS = {"easy": dict(tau=1.0, sigma=0.3, gap=(6.0, 8.0)), "hard": dict(tau=5.0, sigma=1.0, gap=(2.0, 2.5))}

INSTRUCTION = (
    "Cans A, B and C hold water at different temperatures. Stand a can on the grey probe pad and the "
    "display beside it shows its temperature in degrees C. Find the warmest can and put it on the red mat."
)
INSTRUCTION_HARD = (
    "Cans A, B and C hold water at different temperatures. The grey probe pad measures the temperature "
    "of the can standing on it and shows it in degrees C on the display beside it. Put the warmest can "
    "on the red mat."
)


MIN_SETTLED = 8  # readings (4 s of display updates) in a settled stretch


def settled_from(times: list[float], values: list[float], min_n: int = MIN_SETTLED) -> int | None:
    """Index from which a run of display readings shows no significant drift: the earliest i such
    that a straight-line fit to readings[i:] (at least ``min_n`` of them) has a slope within 2
    standard errors of zero, the standard error coming from the fit's own residuals. Uses only the
    displayed values and their times."""
    n = len(values)
    for i in range(0, n - min_n + 1):
        t = np.asarray(times[i:]) - times[i]
        v = np.asarray(values[i:])
        if np.ptp(t) <= 0:
            continue
        slope, icpt = np.polyfit(t, v, 1)
        resid = v - (slope * t + icpt)
        dof = len(v) - 2
        s2 = float(resid @ resid) / dof if dof > 0 else 0.0
        se = np.sqrt(s2 / float(((t - t.mean()) ** 2).sum())) if s2 > 0 else 0.0
        if abs(slope) <= 2 * se:
            return i
    return None


def winner_z(samples: dict[str, list[float]]) -> tuple[str | None, float]:
    """Warmest mean and its smallest margin over every other can, in standard errors, using the
    pooled sample SD of the readings themselves (no assumed noise level)."""
    if any(len(v) < 2 for v in samples.values()):
        return None, 0.0
    means = {c: float(np.mean(v)) for c, v in samples.items()}
    resid = np.concatenate([np.asarray(v) - means[c] for c, v in samples.items()])
    dof = len(resid) - len(samples)
    sd = float(np.sqrt(resid @ resid / dof)) if dof > 0 else 0.0
    top = max(means, key=means.get)
    if sd == 0:
        return top, float("inf")
    z = min((means[top] - means[c]) / (sd * np.sqrt(1 / len(samples[top]) + 1 / len(samples[c])))
            for c in samples if c != top)
    return (top if z > 2 else None), float(z)


@dataclass
class Reading:
    t: float
    can: str | None  # can standing on the pad, untouched (None: empty or held)
    on_for: float  # s the can has stood there
    value: float  # displayed, deg C


@dataclass
class WarmCup:
    difficulty: str = "easy"
    key: str = "warm_cup"
    instruction: str = INSTRUCTION
    readings: list[Reading] = field(default_factory=list)

    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        p = PARAMS["hard" if self.difficulty == "hard" else "easy"]
        self.tau, self.sigma = p["tau"], p["sigma"]
        top = float(rng.uniform(48, 62))
        g1, g2 = (float(rng.uniform(*p["gap"])) for _ in range(2))
        temps = [top, top - g1, top - g1 - g2]
        order = rng.permutation(3)
        self.temps = {CANS[int(order[i])]: temps[i] for i in range(3)}
        self.warmest = max(self.temps, key=self.temps.get)
        self._noise = np.random.default_rng(int(rng.integers(1 << 31)))
        if self.difficulty == "hard":
            self.instruction = INSTRUCTION_HARD

        props.flat(spec, "probe_pad", PAD, (PAD_HALF, PAD_HALF), (0.45, 0.46, 0.50, 1))
        props.text_plate(spec, "lbl_probe", (PAD[0] - 0.06, PAD[1]), "PROBE", half=(0.016, 0.03))
        props.seg7(spec, "display", DISPLAY, n_digits=3, h=0.032, dp_after=1)
        props.text_plate(spec, "lbl_degc", (DISPLAY[0] - 0.055, DISPLAY[1]), "°C", half=(0.02, 0.02))
        props.flat(spec, "mat", MAT_POS, (MAT_HALF, MAT_HALF), props.RED)
        for c, xy in CAN_XY.items():
            props.block(spec, f"can_{c}", xy, HALF, 0.15, label=c, bg=(176, 180, 186))

    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._m = model
        self._tips = props.gripper_geoms(model)
        self._geom = {model.geom(f"can_{c}").id: c for c in CANS}
        self.touch = props.TouchTracker(tuple(f"can_{c}" for c in CANS))
        self.touch.bind(model, data)
        self.seg = props.Seg7("display", 3)
        self.seg.bind(model)
        self.sensor = ROOM
        self.readings = []
        self._t = float(data.time)
        self._next = float(data.time)
        self._on: tuple[str, float] | None = None
        self._show(model)

    def _show(self, model: mujoco.MjModel, value: float | None = None) -> None:
        v = self.sensor if value is None else value
        self.seg.show(model, f"{min(999, max(0, int(round(v * 10)))):03d}")

    def held(self, data: mujoco.MjData) -> set[str]:
        out = set()
        for c in data.contact[: data.ncon]:
            g1, g2 = int(c.geom1), int(c.geom2)
            for a, b in ((g1, g2), (g2, g1)):
                if a in self._geom and b in self._tips:
                    out.add(self._geom[a])
        return out

    def on_pad(self, data: mujoco.MjData) -> list[str]:
        return [c for c in CANS if props.in_box(data.xpos[self._m.body(f"can_{c}").id], PAD, (PAD_HALF,) * 2,
                                                  z_max=HALF + 0.01)]

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        t = float(data.time)
        dt, self._t = t - self._t, t
        self.touch.update(data)
        held = self.held(data)
        on = [c for c in self.on_pad(data) if c not in held]
        can = on[0] if len(on) == 1 else None
        target = self.temps[can] if can else ROOM
        self.sensor += (target - self.sensor) * (1 - np.exp(-dt / self.tau))
        if can is None:
            self._on = None
        elif self._on is None or self._on[0] != can:
            self._on = (can, t)
        if t >= self._next:
            self._next = t + REFRESH
            value = round(self.sensor + float(self._noise.normal(0, self.sigma)), 1)
            self._show(model, value)
            self.readings.append(Reading(t, can, t - self._on[1] if self._on else 0.0, value))

    def sessions(self) -> list[tuple[str, list[Reading]]]:
        """Stretches of display readings with one can standing untouched on the pad."""
        out: list[tuple[str, list[Reading]]] = []
        for r in self.readings:
            if r.can is None:
                continue
            if out and out[-1][0] == r.can and r.t - out[-1][1][-1].t <= REFRESH + 0.05:
                out[-1][1].append(r)
            else:
                out.append((r.can, [r]))
        return out

    def settled(self) -> dict[str, list[float]]:
        """Per can, the readings from the stretches where the display had stopped drifting, judged
        from the displayed values alone (see ``settled_from``)."""
        out: dict[str, list[float]] = {c: [] for c in CANS}
        for can, rs in self.sessions():
            i = settled_from([r.t for r in rs], [r.value for r in rs])
            if i is not None:
                out[can] += [r.value for r in rs[i:]]
        return out

    def proof(self) -> tuple[str | None, float]:
        """(proven warmest can or None, its smallest margin over the others in standard errors)."""
        return winner_z(self.settled())

    def on_mat(self, data: mujoco.MjData) -> list[str]:
        held = self.held(data)
        return [c for c in CANS if c not in held and props.in_box(
            data.xpos[self._m.body(f"can_{c}").id], MAT_POS, (MAT_HALF,) * 2, z_max=HALF + 0.01)]

    def hud(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
        sc = self.score(model, data)
        s = self.settled()
        means = "  ".join(f"{c}: {np.mean(v):.1f} ({len(v)})" if v else f"{c}: -" for c, v in s.items())
        last = self.readings[-1].value if self.readings else self.sensor
        return [f"t = {data.time:5.1f} s   display {last:5.1f} C   rubric stage {sc['stage']}/4",
                f"settled means (n): {means}   z = {sc['z']:.1f}",
                "true: " + "  ".join(f"{c} {v:.1f}" for c, v in self.temps.items()) + f"   on mat {sc['on_mat'] or '-'}"]

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        mat = self.on_mat(data)
        success = mat == [self.warmest]
        proven, z = self.proof()
        stage = 0
        if self.touch.touched:
            stage = 1
        if any(self.settled().values()):
            stage = 2
        if proven == self.warmest:
            stage = 3
            if success:
                stage = 4
        return {
            "stage": stage,
            "success": float(success),
            "science_success": float(stage >= 3),
            "procedure_violations": 0,
            "warmest": self.warmest,
            "z": z,
            "settled_n": {c: len(v) for c, v in self.settled().items()},
            "on_mat": mat,
            "temps": self.temps,
            "log": [r.__dict__ for r in self.readings if r.can is not None][-200:],
        }


# ---------- reference policies ----------
def _carry(env, mo: Motion, can: str, xy):  # noqa: ANN001, ANN202
    """Carry a can to ``xy``; if it lands more than 1 cm off, carry it again with the arm on the target's side."""
    bid = env.model.body(f"can_{can}").id
    for attempt in range(3):
        y0 = env.data.xpos[bid][1]
        far = abs(xy[1] - y0) > 0.12
        side = "left" if ((xy[1] if attempt or far else y0) > -0.305) else "right"
        yield from move_body(env, mo, f"can_{can}", side, xy, 0.0, half=HALF, open_to=0.034, grip=0.0)
        if np.hypot(*(env.data.xpos[bid][:2] - np.asarray(xy))) < 0.01:
            return


def _measure(env, mo: Motion, can: str, extra: float, cap: float = 90.0):  # noqa: ANN001, ANN202
    """Stand ``can`` on the pad and watch the display until its readings stop drifting (a straight
    line through the latest readings has no significant slope), then keep reading ``extra`` s."""
    t: WarmCup = env.task
    yield from _carry(env, mo, can, PAD)
    t0 = float(env.data.time)
    while float(env.data.time) - t0 < cap:
        yield from mo.hold(2.0)
        rs = [r for r in t.readings if r.t > t0 and r.can == can]
        if settled_from([r.t for r in rs], [r.value for r in rs]) is not None:
            break
    yield from mo.hold(extra)
    yield from _carry(env, mo, can, CAN_XY[can])


def _own_settled(t: "WarmCup", spans: list[tuple[str, float, float]]) -> dict[str, list[float]]:
    out: dict[str, list[float]] = {c: [] for c in CANS}
    for can, a, b in spans:
        rs = [r for r in t.readings if a < r.t <= b and r.can == can]
        i = settled_from([r.t for r in rs], [r.value for r in rs])
        if i is not None:
            out[can] += [r.value for r in rs[i:]]
    return out


def _settled_method(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Measure each can until the display stops drifting, then read a few seconds more; compare
    the settled means with the pooled scatter of the readings. Re-measure the leader and the
    closest rival (longer) until the leader beats every other can by more than 2 standard errors.
    Uses only the display values it watched."""
    t: WarmCup = env.task
    mo = Motion(env)
    spans: list[tuple[str, float, float]] = []

    def measure(c: str, extra: float):  # noqa: ANN202
        a = float(env.data.time)
        yield from _measure(env, mo, c, extra)
        spans.append((c, a, float(env.data.time)))

    for c in CANS:
        yield from measure(c, 4.0)
    for _ in range(4):
        s = _own_settled(t, spans)
        best, _z = winner_z(s)
        if best is not None:
            break
        means = {c: np.mean(v) if v else -1e9 for c, v in s.items()}
        lead = max(means, key=means.get)
        rival = max((c for c in CANS if c != lead), key=means.get)
        for c in (lead, rival):
            yield from measure(c, 8.0)
    s = _own_settled(t, spans)
    best = max(CANS, key=lambda c: np.mean(s[c]) if s[c] else -1e9)
    yield from _carry(env, mo, best, MAT_POS)
    yield from mo.hold(1.5)


def _quick(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """The obvious procedure: stand each can on the pad, read the display once after 2 s, pick the max."""
    t: WarmCup = env.task
    mo = Motion(env)
    seen = {}
    for c in CANS:
        yield from _carry(env, mo, c, PAD)
        yield from mo.hold(2.0)
        seen[c] = t.readings[-1].value
        yield from _carry(env, mo, c, CAN_XY[c])
    best = max(seen, key=seen.get)
    yield from _carry(env, mo, best, MAT_POS)
    yield from mo.hold(1.5)


def oracle(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    yield from _settled_method(env, rng)


def guess(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Blind baseline: a random can to the mat."""
    rng = rng or np.random.default_rng()
    yield from _carry(env, Motion(env), CANS[int(rng.integers(3))], MAT_POS)
    yield from Motion(env).hold(1.5)


TASK = WarmCup
POLICIES = {"oracle": oracle, "guess": guess, "noop": noop, "naive": _quick}
SPEC = TaskSpec(
    key="warm_cup",
    title="Find the warmest can with a slow, noisy probe",
    ability="Noise and replication",
    instruction=INSTRUCTION,
    instruction_hard=INSTRUCTION_HARD,
    objects=(
        "three sealed aluminium cans (4 cm cubes, 150 g) labelled A, B, C",
        "grey probe pad (7 x 7 cm) with a 3-digit red LED display (3.2 cm digits, deg C, 0.1 C steps) beside it",
        "red mat, 12 x 12 cm",
    ),
    setup=(
        "Cans A, B, C in a row 22 cm in front of the arms, centred, 10.5 cm apart (A on the left arm's side).",
        "Probe pad 32 cm in front of the arms, centred between them; display 34 cm in front, 15.5 cm to the left of centre.",
        "Red mat 30 cm in front of the left arm's base, on the operator's right.",
    ),
    hidden="Can temperatures (warmest 48-62 C), 6-8 C apart; probe time constant 1 s, reading noise 0.3 C.",
    hidden_hard=("Can temperatures 2.0-2.5 C apart (warmest 48-62 C); the probe follows the can with a 5 s time "
                 "constant and each 0.5 s display update adds 1.0 C of noise. A reading 2 s after setting a can "
                 "down is only a third of the way from the previous reading."),
    rubric=(
        "No purposeful contact with the cans",
        "Touches or moves a can",
        "At least one settled stretch: a can stood untouched on the pad until the display stopped drifting "
        "(no significant slope over at least 8 readings)",
        "Every can has settled readings, and the warmest mean beats every other can by more than 2 standard "
        "errors, using the pooled scatter of the readings",
        "Stage 3 and the warmest can alone on the mat",
    ),
    metrics=("stage (0-4)", "success: warmest can alone on the mat, proven or not", "science_success: stage >= 3",
             "settled readings per can; z of the winning margin"),
    report_fields=("warmest", "z", "settled_n", "on_mat"),
    max_seconds=660.0,
    limitations=("Temperatures are simulated in code: the pad's sensor follows the can on it as a first-order lag "
                 "with Gaussian reading noise; cans do not cool during the trial."),
)
