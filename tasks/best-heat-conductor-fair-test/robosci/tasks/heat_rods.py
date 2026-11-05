"""Task: find which of three look-alike rods conducts heat best.

Ability: comparative measurement (fair test). Hidden variables: each rod's thermal conductivity,
and (hard) which rod starts warm. Each rod carries a strip of thermochromic paint in 8 segments
that turns from dark blue to orange above 45 C. Laying a rod with one end on the red hot plate
heats that end; heat spreads along the rod (1-D diffusion in code) and the orange band grows,
faster for a better conductor. Off the plate a rod cools slowly in air (time constant 60 s).

The scorer records a session each time a rod rests with an end on the plate, and the band length
(orange segments) after 20 s on the plate. Two displays help time it: PLATE counts the seconds the
rod now on the plate has been there, CLOCK the seconds since the start. Science is proven when every
rod has a fair-start 20-second reading and one rod's band is strictly the longest (easy: fair = mean
temperature below 30 C; hard: see below).

Easy: all rods start at room temperature (22 C). Hard: the middle conductor was just used and
starts warm (part of its band already orange); tested at once it shows a band as long as or
longer than the best rod's. The hard instruction states that 2 min off the plate brings a rod
back to about room temperature, so a fair start is one the robot can time itself: the rod has
been off the plate for 2 min, counting the start of the episode as its last contact.
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

RODS = ("R1", "R2", "R3")
N = 8
L, W = 0.06, 0.016  # half length, half width/height
START = {"R1": (0.22, -0.20), "R2": (0.22, -0.305), "R3": (0.22, -0.41)}
TEST = {r: (0.38, y) for r, (_, y) in START.items()}
PLATE_X, PLATE_Y, PLATE_HALF = (0.42, 0.48), (-0.48, -0.13), None
MAT_POS, MAT_HALF = (0.30, 0.06), 0.07
ROOM, HOT, PAINT = 22.0, 90.0, 45.0
G_PLATE, LOSS_ON, LOSS_OFF = 2.0, 0.02, 1 / 60
READ_AT, COLD = 20.0, 30.0  # easy cold start: the bar's mean temperature below 30 C
REST = 120.0  # hard fair start: the rod has been off the plate for 2 min (stated in the instruction)
SPOT = TEST["R2"]  # hard: every rod is tested on this one spot of the plate
TIMER_XY, CLOCK_XY = (0.46, -0.06, 0.02), (0.46, -0.56, 0.02)  # lab-timer displays (top-camera readable)
COLD_RGB, HOT_RGB = np.array([0.12, 0.14, 0.24]), np.array([1.0, 0.55, 0.08])

INSTRUCTION = (
    "Rods R1, R2 and R3 look the same but conduct heat differently. Each has a strip of heat-sensitive paint that "
    "turns orange above 45 C. Lay a rod with one end on the red hot plate and watch how far the orange spreads. "
    "Find the best heat conductor and put it on the red mat."
)
INSTRUCTION_HARD = (
    "Rods R1, R2 and R3 look the same but conduct heat differently. Each has a strip of heat-sensitive paint that "
    "turns orange above 45 C, and the red plate is hot. At the start, a rod showing no orange is at room "
    "temperature, and a rod left off the plate for 2 minutes has cooled back to about room temperature. The "
    "PLATE display counts the seconds a rod has been on the plate; the CLOCK display counts seconds since the "
    "start. Put the rod that conducts heat best on the red mat."
)


@dataclass
class Session:
    rod: str
    t0: float
    start_max: float
    start_mean: float
    band20: int | None = None
    rested: float = 0.0  # s off the plate (within the episode) before this session started
    fair: bool = False  # hard: a stated fair start (see HeatRods.monitor)
    hard: bool = False

    @property
    def cold(self) -> bool:
        """Fair start. Hard (both checkable by eye): the rod showed no orange at the start and has not
        been on the plate since, or it has been off the plate for >= 2 min by the CLOCK (the start counts
        as its last contact). Easy: mean temperature below 30 C."""
        return self.fair if self.hard else self.start_mean < COLD


@dataclass
class HeatRods:
    difficulty: str = "easy"
    key: str = "heat_rods"
    instruction: str = INSTRUCTION
    sessions: list[Session] = field(default_factory=list)

    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        ranks = rng.permutation(3)  # 0 worst, 1 middle, 2 best
        base = (0.20, 0.45, 0.75)
        self.cond = {r: base[int(ranks[i])] * float(rng.uniform(0.95, 1.05)) for i, r in enumerate(RODS)}
        self.best = max(self.cond, key=self.cond.get)
        self.warm = None
        if self.difficulty == "hard":
            self.instruction = INSTRUCTION_HARD
            self.warm = next(r for r in RODS if int(ranks[RODS.index(r)]) == 1)
            self.prewarm = float(rng.uniform(140, 170))

        props.flat(spec, "hot_plate", ((PLATE_X[0] + PLATE_X[1]) / 2, (PLATE_Y[0] + PLATE_Y[1]) / 2),
                   ((PLATE_X[1] - PLATE_X[0]) / 2, (PLATE_Y[1] - PLATE_Y[0]) / 2), (0.85, 0.15, 0.10, 1))
        props.text_plate(spec, "lbl_hot", (0.50, -0.305), "HOT", half=(0.016, 0.03))
        props.seg7(spec, "timer", TIMER_XY, n_digits=2, h=0.03)
        props.text_plate(spec, "lbl_timer", (TIMER_XY[0] - 0.05, TIMER_XY[1]), "PLATE", half=(0.016, 0.045))
        props.seg7(spec, "clock", CLOCK_XY, n_digits=3, h=0.03)
        props.text_plate(spec, "lbl_clock", (CLOCK_XY[0] - 0.05, CLOCK_XY[1]), "CLOCK", half=(0.016, 0.045))
        props.flat(spec, "mat", MAT_POS, (MAT_HALF, MAT_HALF), props.RED)
        dx = 2 * L / N
        for r, (x, y) in START.items():
            b = props.block(spec, f"rod_{r}", (x, y), (L, W, W), 0.12, rgba=(0.62, 0.63, 0.66, 1))
            for j in range(N):  # node j sits j segments from the +x end
                b.add_geom(name=f"rod_{r}_seg{j}", type=props.G.mjGEOM_BOX, size=[dx / 2 - 0.0008, W - 0.002, 0.0006],
                           pos=[L - dx * (j + 0.5), 0, W + 0.0004], rgba=[*COLD_RGB, 1], contype=0, conaffinity=0)
            props.text_plate(spec, f"lbl_{r}", (x - 0.09, y), r, half=(0.018, 0.018))

    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._m = model
        self._tips = props.gripper_geoms(model)
        self._geom = {model.geom(f"rod_{r}").id: r for r in RODS}
        self._seg = {r: [model.geom(f"rod_{r}_seg{j}").id for j in range(N)] for r in RODS}
        self.touch = props.TouchTracker(tuple(f"rod_{r}" for r in RODS))
        self.touch.bind(model, data)
        self.T = {r: np.full(N, ROOM) for r in RODS}
        if self.warm is not None:  # just used: heated from its +x end, then 2 s off the plate
            for secs, on in ((self.prewarm, 0), (2.0, None)):
                for _ in range(int(secs / 0.02)):
                    self._heat(self.warm, 0.02, on)
        self.sessions = []
        self._open: dict[str, Session] = {}
        self._last_on = {r: float(data.time) for r in RODS}  # episode start counts as the last plate contact
        self._t_start = float(data.time)
        self._ever_on = {r: False for r in RODS}
        self.orange0 = {r: self.band(r) > 0 for r in RODS}  # visible at the start
        self.timer, self.clock = props.Seg7("timer", 2), props.Seg7("clock", 3)
        self.timer.bind(model)
        self.clock.bind(model)
        self._t = float(data.time)
        self._paint(model)

    def _heat(self, r: str, dt: float, hot_end: int | None) -> None:
        T = self.T[r]
        lap = np.empty(N)
        lap[1:-1] = T[:-2] - 2 * T[1:-1] + T[2:]
        lap[0], lap[-1] = T[1] - T[0], T[-2] - T[-1]
        d = self.cond[r] * lap - (LOSS_ON if hot_end is not None else LOSS_OFF) * (T - ROOM)
        if hot_end is not None:
            d[hot_end] += G_PLATE * (HOT - T[hot_end])
        self.T[r] = T + dt * d

    def band(self, r: str) -> int:
        return int((self.T[r] >= PAINT).sum())

    def _paint(self, model: mujoco.MjModel) -> None:
        for r in RODS:
            frac = np.clip((self.T[r] - (PAINT - 3)) / 6, 0, 1)
            for j, g in enumerate(self._seg[r]):
                model.geom_rgba[g, :3] = COLD_RGB + (HOT_RGB - COLD_RGB) * frac[j]

    def held(self, data: mujoco.MjData) -> set[str]:
        out = set()
        for c in data.contact[: data.ncon]:
            g1, g2 = int(c.geom1), int(c.geom2)
            for a, b in ((g1, g2), (g2, g1)):
                if a in self._geom and b in self._tips:
                    out.add(self._geom[a])
        return out

    def hot_end(self, data: mujoco.MjData, r: str) -> int | None:
        """Node index (0 = +x end, N-1 = -x end) resting on the plate, or None."""
        bid = self._m.body(f"rod_{r}").id
        p, xm = data.xpos[bid], data.xmat[bid].reshape(3, 3)
        if p[2] > W + 0.012:
            return None
        for node, s in ((0, 1.0), (N - 1, -1.0)):
            e = p + xm[:, 0] * s * (L - 0.01)
            if PLATE_X[0] <= e[0] <= PLATE_X[1] and PLATE_Y[0] <= e[1] <= PLATE_Y[1]:
                return node
        return None

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        t = float(data.time)
        dt, self._t = t - self._t, t
        self.touch.update(data)
        for r in RODS:
            end = self.hot_end(data, r)  # a brush from the gripper does not end a session; lifting it does
            if dt > 0:
                self._heat(r, dt, end)
            s = self._open.get(r)
            if end is None:
                self._open.pop(r, None)
            elif s is None:
                rested = t - self._last_on[r]
                fair = (not self._ever_on[r] and not self.orange0[r]) or rested >= REST
                s = Session(r, t, float(self.T[r].max()), float(self.T[r].mean()),
                            rested=rested, fair=fair, hard=self.difficulty != "easy")
                self._open[r] = s
                self.sessions.append(s)
            elif s.band20 is None and t - s.t0 >= READ_AT:
                s.band20 = self.band(r)
            if end is not None:
                self._last_on[r] = t
                self._ever_on[r] = True
        self._paint(model)
        open_s = [x for x in self._open.values()]
        self.timer.show(model, str(min(99, int(t - max(open_s, key=lambda x: x.t0).t0))) if open_s else "")
        self.clock.show(model, str(min(999, int(t - self._t_start))))

    def readings(self, cold_only: bool = True) -> dict[str, int]:
        out = {}
        for s in self.sessions:
            if s.band20 is not None and (s.cold or not cold_only):
                out[s.rod] = s.band20
        return out

    def proven(self) -> str | None:
        rd = self.readings()
        if len(rd) < 3:
            return None
        top = max(rd.values())
        win = [r for r, v in rd.items() if v == top]
        return win[0] if len(win) == 1 else None

    def on_mat(self, data: mujoco.MjData) -> list[str]:
        held = self.held(data)
        return [r for r in RODS if r not in held and props.in_box(
            data.xpos[self._m.body(f"rod_{r}").id], MAT_POS, (MAT_HALF,) * 2, z_max=W + 0.01)]

    def hud(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
        sc = self.score(model, data)
        t = float(data.time)
        lines = [f"clock {t - self._t_start:5.1f} s   rubric stage {sc['stage']}/4"]
        for r, x in self._open.items():
            lines.append(f"testing {r}: {min(t - x.t0, READ_AT):4.1f}/{READ_AT:.0f} s on plate"
                         + ("" if x.cold else "  (not a fair start)"))
        rd = ", ".join(f"{r} = {v}/{N}" for r, v in self.readings().items())
        lines.append(f"fair 20 s readings: {rd or '-'}   on mat {sc['on_mat'] or '-'}")
        for r in RODS:
            if self.difficulty != "easy" and r not in self.readings() and r not in self._open:
                off = t - self._last_on[r]
                if self.orange0[r] or self._ever_on[r]:
                    lines.append(f"{r} cooling: off plate {min(off, REST):3.0f}/{REST:.0f} s")
        return lines[:5]

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        mat = self.on_mat(data)
        success = mat == [self.best]
        stage = 0
        if self.touch.touched:
            stage = 1
        if any(s.band20 is not None for s in self.sessions):
            stage = 2
        if self.proven() == self.best:
            stage = 3
            if success:
                stage = 4
        return {
            "stage": stage,
            "success": float(success),
            "science_success": float(stage >= 3),
            "procedure_violations": 0,
            "best": self.best,
            "prewarmed": self.warm,
            "cold_readings": self.readings(),
            "on_mat": mat,
            "log": [s.__dict__ for s in self.sessions],
        }


# ---------- reference policies ----------
def _carry_bar(env, mo: Motion, r: str, xy, arms):  # noqa: ANN001, ANN202
    """Pick and place a bar with the fingers along world y (across its width), so it stays lying along x.
    Same tilt at both ends keeps it level. Returns False if no arm in ``arms`` can do it."""
    m, d = env.model, env.data
    body = f"rod_{r}"
    c0 = d.xpos[m.body(body).id].copy()
    grasp = c0 + [0, 0, 0.002]
    target = np.array([xy[0], xy[1], W + 0.004])
    for arm in arms:
        plan = None
        for tilt in TILTS:
            for y1 in (np.pi / 2, -np.pi / 2):
                r1 = grasp_rot(y1, tilt)
                if not mo._vertical_ok(arm, grasp, r1):
                    continue
                off = r1.T @ (c0 - grasp)
                for y2 in (np.pi / 2, -np.pi / 2):
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
        yield from mo.pick(arm, grasp, 0.0, body=body, plan=plan[0], open_to=0.03)
        y, tilt, lo, rot = plan[1]
        if mo.held[arm] is not None:
            lo2 = target - rot @ mo.held[arm][0]
            if mo._vertical_ok(arm, lo2, rot):
                lo = lo2
        mo.rot[arm], mo.yaw[arm], mo.tilt[arm] = rot, y, tilt
        yield from mo.goto_above(arm, lo[:2])
        top = np.array([lo[0], lo[1], SAFE_Z])
        yield from mo.line(arm, top, lo, 1.2)
        yield from mo.gripper(arm, 0.03, 0.5)
        yield from mo.line(arm, lo, top, 0.8)
        mo.held[arm] = None
        yield from mo.home(arm)
        return True
    return False


RELAY = (0.30, -0.305)


def _carry_once(env, mo: Motion, r: str, xy, prefer_target_side: bool):  # noqa: ANN001, ANN202
    y0 = env.data.xpos[env.model.body(f"rod_{r}").id][1]
    near = "left" if y0 > -0.305 else "right"
    far = "left" if xy[1] > -0.305 else "right"
    arms = (far, near) if prefer_target_side or abs(xy[1] - y0) > 0.12 else (near, far)
    ok = yield from _carry_bar(env, mo, r, xy, arms)
    if not ok:  # hand over through the middle of the table
        yield from _carry_bar(env, mo, r, RELAY, (near, far))
        ok = yield from _carry_bar(env, mo, r, xy, (far, near))
    if not ok:
        raise RuntimeError(f"no arm can carry rod {r} to {xy}")


def _carry(env, mo: Motion, r: str, xy, done=None):  # noqa: ANN001, ANN202
    """Carry a bar to ``xy`` lying along x; while ``done()`` is false afterwards, carry it again (up to 3 tries)."""
    for attempt in range(3):
        yield from _carry_once(env, mo, r, xy, attempt > 0)
        if done is None or done():
            return


def _compare(env, mo: Motion, put_back: bool = False):  # noqa: ANN001, ANN202
    """Lay all three rods with an end on the plate and wait until each has its 20 s reading.
    ``put_back`` returns them to their start spots (easy: clears the plate for the final carry)."""
    t: HeatRods = env.task
    for r in RODS:
        yield from _carry(env, mo, r, TEST[r], done=lambda r=r: t.hot_end(env.data, r) is not None)
    while any(t._open.get(r) is None or t._open[r].band20 is None for r in RODS):
        yield from mo.hold(1.0)
        if env.data.time > 600:
            break
    if put_back:
        for r in RODS:
            yield from _carry(env, mo, r, START[r])


def _one_at_a_time(env, mo: Motion):  # noqa: ANN001, ANN202
    """Hard: one rod at a time on the same plate spot, read at 20 s on the PLATE timer. Rods that showed
    no orange at the start go first (a fair start by the stated rule); a rod that started orange goes
    last, once the CLOCK says it has been off the plate for 2 min. Only the rod on the plate is moved off,
    and the winner goes straight to the mat."""
    t: HeatRods = env.task
    order = sorted(RODS, key=lambda r: (t.orange0[r], RODS.index(r)))
    for k, r in enumerate(order):
        if t.orange0[r]:
            while float(env.data.time) - t._last_on[r] < REST + 1.0:
                yield from mo.hold(1.0)
        yield from _carry(env, mo, r, SPOT, done=lambda r=r: t.hot_end(env.data, r) is not None)
        for _ in range(int(READ_AT + 15)):
            s = t._open.get(r)
            if s is not None and s.band20 is not None:
                break
            yield from mo.hold(1.0)
        if k < len(order) - 1:  # free the spot for the next rod
            yield from _carry(env, mo, r, START[r])


def oracle(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Easy: heat all three together and compare their bands after 20 s. Hard: ``_one_at_a_time``.
    The rod with the strictly longest fair 20 s band goes to the mat; with no strict winner, nothing."""
    t: HeatRods = env.task
    mo = Motion(env)
    if t.difficulty == "easy":
        yield from _compare(env, mo, put_back=True)
    else:
        yield from _one_at_a_time(env, mo)
    best = t.proven()
    if best is None:
        yield from mo.hold(1.0)
        return
    yield from _carry(env, mo, best, MAT_POS, done=lambda: best in t.on_mat(env.data))
    yield from mo.hold(1.5)


def _at_once(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """The obvious test: heat all three straight away and pick the longest band after 20 s."""
    t: HeatRods = env.task
    mo = Motion(env)
    yield from _compare(env, mo)
    rd = t.readings(cold_only=False)
    best = max(rd, key=rd.get) if rd else RODS[0]
    yield from _carry(env, mo, best, MAT_POS, done=lambda: best in t.on_mat(env.data))
    yield from mo.hold(1.5)


def guess(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    rng = rng or np.random.default_rng()
    mo = Motion(env)
    r = RODS[int(rng.integers(3))]
    yield from _carry(env, mo, r, MAT_POS, done=lambda: r in env.task.on_mat(env.data))
    yield from mo.hold(1.5)


TASK = HeatRods
POLICIES = {"oracle": oracle, "guess": guess, "noop": noop, "naive": _at_once}
SPEC = TaskSpec(
    key="heat_rods",
    title="Find the best heat conductor, fairly",
    ability="Comparative measurement",
    instruction=INSTRUCTION,
    instruction_hard=INSTRUCTION_HARD,
    objects=(
        "three grey metal rods R1, R2, R3 (12 x 3.2 x 3.2 cm, 120 g), each with a strip of thermochromic paint in 8 "
        "segments on top (dark blue below about 42 C, orange above 48 C)",
        "red hot plate (6 x 35 cm, 90 C) with a printed HOT card",
        "red mat, 14 x 14 cm",
        "two red 7-segment displays lying flat: PLATE (2 digits, seconds the rod now on the plate has been there) "
        "and CLOCK (3 digits, seconds since the start), each with a printed label",
    ),
    setup=(
        "Rods lie along the forward axis 22 cm in front of the arms (centres), 10.5 cm apart, R1 on the left arm's "
        "side; name cards 9 cm nearer the arms.",
        "Hot plate across the table 42-48 cm in front of the arms, centred between them.",
        "Red mat 30 cm in front of the left arm's base, on the operator's right.",
        "PLATE display 46 cm in front of the arms, 24.5 cm left of the centre line (just left of the plate); CLOCK "
        "display 46 cm in front, 25.5 cm right of the centre line (just right of the plate); labels 5 cm nearer "
        "the arms.",
    ),
    hidden="Each rod's conductivity (ratio about 1 : 2.25 : 3.75, shuffled).",
    hidden_hard=("Conductivities as easy; the middle conductor starts warm, as if just taken off the plate after "
                 "140-170 s of heating (5-8 orange segments at the start; the orange fades after about 35 s and its mean temperature is below 30 C after about 100 s). Heated "
                 "straight away it shows a band as long as or longer than the best rod's."),
    rubric=(
        "No purposeful contact with the rods",
        "Touches or moves a rod",
        "At least one rod read after 20 s with an end on the hot plate",
        "Every rod read after 20 s on the plate from a fair start (easy: mean temperature below 30 C; hard: no orange at the start and not on the plate since, or off the plate for at least 2 min by the CLOCK), and one band strictly the longest",
        "Stage 3 and that rod alone on the mat",
    ),
    metrics=("stage (0-4)", "success: best conductor alone on the mat, proven or not", "science_success: stage >= 3",
             "20 s band lengths per rod (cold starts)"),
    report_fields=("best", "prewarmed", "cold_readings", "on_mat"),
    max_seconds=300.0,
    limitations=("Heat flow is simulated in code: 1-D diffusion along 8 nodes per rod, the plate heats the end "
                 "resting on it, rods cool to 22 C in air (60 s time constant); the paint colour is set from the node temperatures."),
)
