"""A pendulum on a knife-edge bench bracket: a painted rod with a brass bob held on it by a friction spring. The rod is pine or steel, painted alike, by seed, so the bob's right place can be found only by timing.

Run a model:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1400 --instruction "..."

The demo:  python demokit.py task.py OUT --mode full --seed 0     the final video, with the pine rod. `--seed 1` for the steel rod
           python demokit.py task.py OUT --no-record --mode full,fixed,fixed_steel,mid,unset,still,touch,idle --seed 0

`full` swings it, times it, moves the bob and times it again until a cycle takes a second, then leaves it swinging. The others test the scorer: `fixed` puts the bob where a pine rod needs it, with no timing, `fixed_steel` where a steel rod needs it, `mid` between the two, `unset` swings it without moving the bob, `still` sets the bob and never swings it, `touch` swings it and leaves the fingers beside the bob, `idle` does nothing.
"""

from __future__ import annotations

import dataclasses
import json
import math
import os
import pathlib
import re
import sys
from datetime import datetime

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "Regulate the pendulum: move the bob until one full swing takes one second, then leave it swinging by itself."
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
TOP = d.TABLE_TOP

# The pendulum, in metres. The part's origin is on the table under the pivot.
BASE_XY = (0.30, 0.20)
PIVOT_H = 0.46  # the knife edge above the table
ROD_L, ROD_R = 0.30, 0.006
BOB_R, BOB_H, BOB_MASS = 0.022, 0.04, 0.30
BOB_START = 0.19  # the bob's middle below the pivot at the start
BOB_TRAVEL = (0.12, 0.28)  # how far along the rod it can go; the rod's end is at 0.30, and past 0.28 it counts as coming off
ROD_MASS = {"wood": 0.0204, "steel": 0.266}  # 12 mm round rod, 30 cm: pine, 600 kg/m3, and steel, 7,850 kg/m3
BOB_HOLD = 2000.0  # N/m: the friction spring, as a stiff hold on the bob. It carries the bob's 2.9 N and slips above about 4 N, when a gripper moves it
BOB_SLIP = 0.002  # m: how far the bob gives before it slips
KNIFE_FRICTION = 0.0003  # N m
KNIFE_DAMPING = {"wood": 0.0046, "steel": 0.0072}  # N m per rad/s. The solver adds about 11 percent of amplitude a second to a free swing, measured on both rods with the bob in place. This takes that out, and the swing falls from about 7 to 3 degrees in 10 s, faster than on a real knife edge

LOOKS = {
    "rod": m.Look(m.rgb(0.62, 0.62, 0.60), roughness=0.5),
    "bob": m.Look(m.rgb(0.78, 0.58, 0.20), metallic=0.9, roughness=0.3),
    "stand": m.Look(m.rgb(0.13, 0.14, 0.17), roughness=0.6),
    "edge": m.Look(m.rgb(0.70, 0.70, 0.72), metallic=0.9, roughness=0.35),
}

# The score.
CYCLES = 10
FULL_WITHIN = 0.1  # s, over ten cycles
PART_WITHIN = 1.0
FREE_FROM = 0.03  # fingertips from the bob and the rod
MIN_SWING_DEG = 2.0

_SEED = {"value": None}


def rod_kind() -> str:
    """Even reset seeds, and no seed, have the pine rod; odd reset seeds the steel one."""
    seed = _SEED["value"]
    return "steel" if seed is not None and seed % 2 else "wood"


def _pendulum() -> m.Part:
    px, py = 0.0, 0.0
    rod_mesh = m.cylinder(ROD_R, ROD_L, (px, py, PIVOT_H - ROD_L / 2))
    bob_mesh = m.cylinder(BOB_R, BOB_H, (px, py, PIVOT_H - BOB_START))
    rod = m.hinge("rod", "Y", (px, py, PIVOT_H), -60.0, 60.0, [("rod", rod_mesh, "rod"), ("edge", m.box((0.012, 0.03, 0.012), (px, py, PIVOT_H)), "edge")], [m.col_box("rod_c", (2 * ROD_R, 2 * ROD_R, ROD_L), (px, py, PIVOT_H - ROD_L / 2))],
                  mass=ROD_MASS["wood"], damping=KNIFE_DAMPING["wood"], friction=KNIFE_FRICTION, armature=1e-5, gravity=True, com=(px, py, PIVOT_H - ROD_L / 2))
    bob = m.slider("bob", "Z", BOB_START - BOB_TRAVEL[1], BOB_START - BOB_TRAVEL[0], [("bob", bob_mesh, "bob")], [m.col_cylinder("bob_c", BOB_R, BOB_H, (px, py, PIVOT_H - BOB_START))], at=(px, py, PIVOT_H - BOB_START),
                   mass=BOB_MASS, spring=BOB_HOLD, damping=30.0, effort=8.0, friction=0.0, armature=1e-5, gravity=True, com=(px, py, PIVOT_H - BOB_START), parent="rod")
    post_x = 0.15  # forward of the pivot, out of the arm's way
    frame = [
        ("foot", m.box((0.18, 0.12, 0.012), (post_x, py, 0.006)), "stand"),
        ("post", m.box((0.025, 0.025, PIVOT_H + 0.025), (post_x, py, (PIVOT_H + 0.025) / 2)), "stand"),
        ("beam", m.box((post_x + 0.03, 0.025, 0.02), ((post_x - 0.03) / 2, py, PIVOT_H + 0.016)), "stand"),
    ]
    cols = [m.col_box("foot_c", (0.18, 0.12, 0.012), (post_x, py, 0.006)), m.col_box("post_c", (0.025, 0.025, PIVOT_H + 0.025), (post_x, py, (PIVOT_H + 0.025) / 2))]
    part = m.mechanism("pend", (*BASE_XY, TOP), [rod, bob], LOOKS, frame_parts=frame, frame_colliders=cols)
    plain = part.build

    def build() -> str:
        """The rod's real mass and inertia for this seed: a uniform rod about its middle, since the period depends on it."""
        mass = ROD_MASS[rod_kind()]
        inertia = mass * ROD_L**2 / 12
        return re.sub(r'(def Xform "rod".*?float physics:mass = )[\d.]+(.*?float3 physics:diagonalInertia = )\([^)]*\)', lambda g: f"{g.group(1)}{mass:.5f}{g.group(2)}({inertia:.3e}, {inertia:.3e}, 1.000e-06)", plain(), count=1, flags=re.S)

    return dataclasses.replace(part, build=build)


OBJECTS = [_pendulum()]


def _note_seed() -> None:
    """The bench's rod is chosen when the scene is built, inside the first reset, so note the seed as that reset starts."""
    import yam_arena.inspect_embodiment as harness

    cls = harness.YamArenaEmbodiment
    if getattr(cls, "_pendulum_seed", False):
        return
    reset = cls.reset

    def reset_hook(self, scene, *, seed=None):
        _SEED["value"] = seed
        return reset(self, scene, seed=seed)

    cls.reset = reset_hook
    cls._pendulum_seed = True


_note_seed()


def task_score(lost: bool, cycles_s: float | None, swinging: bool, free: bool) -> tuple[float, bool]:
    """The scoring text's table, line by line. `cycles_s` is the last ten free cycles' time. Returns the score, and whether that is the full one."""
    if lost or cycles_s is None or not swinging or not free:
        return 0.0, False
    off = abs(cycles_s - CYCLES)
    if off <= FULL_WITHIN:
        return 1.0, True
    return (0.5, False) if off <= PART_WITHIN else (0.0, False)


class Trial:
    """What the scorer holds across a trial. A cycle is timed from the rod passing upright going one way, as a stopwatch would be, and a touch starts the count again."""

    def reset(self, env=None) -> None:
        self.t = 0.0
        self.bob_held_at = 0.0
        self.last = None
        self.crossings: list[float] = []
        self.peak = 0.0
        self.since_peak = 0.0
        self.swing = 0.0
        self.lost = False
        self.score = 0.0
        self.full = False
        self.rep = {}
        self.trace: list[dict] = []
        self.path = None

    def step(self, env) -> None:
        if self.t == 0.0:  # the rod's damping for this seed: the actuator is built with wood's
            asset, j = m._joint(env, "pend", "rod")
            asset.write_joint_damping_to_sim(m._tensor(asset, KNIFE_DAMPING[rod_kind()]), joint_ids=[j])
        self.t += 1.0 / d.CONTROL_HZ
        if os.environ.get("TASK_STEP_FILE"):  # the simulated time, for anything following the run from outside
            with open(os.environ["TASK_STEP_FILE"], "w") as f:
                f.write(f"{self.t:.2f}")
        s = m.state(env, "pend")
        angle, along = s.deg("rod"), BOB_START - s.joint("bob")
        if abs(s.joint("bob") - self.bob_held_at) > BOB_SLIP:  # pushed past what the friction spring holds, it slips, and holds again where it now is
            self.bob_held_at = s.joint("bob")
            m.set_target(env, "pend", "bob", self.bob_held_at)
        origin = s.pos
        tips = m.fingertips(env)
        rad = math.radians(angle)
        down = np.array([-math.sin(rad), 0.0, -math.cos(rad)])
        pivot = origin + np.array([0.0, 0.0, PIVOT_H])
        rel = tips - pivot
        on_rod = np.clip(rel @ down, 0.0, ROD_L)
        to_rod = float(np.linalg.norm(rel - np.outer(on_rod, down), axis=1).min()) - ROD_R
        to_bob = float(np.linalg.norm(rel - along * down, axis=1).min()) - BOB_R
        free = min(to_rod, to_bob) >= FREE_FROM
        if not free:
            self.crossings = []
        if self.last is not None and self.last < 0.0 <= angle:
            self.crossings.append(self.t - (angle / max(angle - self.last, 1e-6)) / d.CONTROL_HZ)
        self.last = angle
        # The swing's size: the largest angle in about the last cycle.
        self.since_peak += 1.0 / d.CONTROL_HZ
        if abs(angle) >= self.peak or self.since_peak > 1.2:
            self.peak, self.since_peak = abs(angle), 0.0
        self.swing = max(self.peak, abs(angle))
        swinging = self.swing >= MIN_SWING_DEG
        if along >= BOB_TRAVEL[1] - 0.002:
            self.lost = True  # the bob at the rod's end comes off a real rod
        cycles_s = self.crossings[-1] - self.crossings[-1 - CYCLES] if len(self.crossings) > CYCLES else None
        self.score, self.full = task_score(self.lost, cycles_s, swinging, free)
        self.rep = {"t": round(self.t, 2), "score": self.score, "full": self.full, "rod": rod_kind(), "angle_deg": round(angle, 2), "swing_deg": round(self.swing, 2), "bob_m": round(along, 4), "free": free, "crossings": len(self.crossings),
                    "cycles_s": None if cycles_s is None else round(cycles_s, 3), "to_rod_m": round(to_rod, 3), "to_bob_m": round(to_bob, 3), "lost": self.lost}
        n = round(self.t * d.CONTROL_HZ)
        if n % 10 == 0:
            self.trace.append(self.rep)
            folder = os.environ.get("TASK_SCORE_DIR", "pendulum_scores")
            os.makedirs(folder, exist_ok=True)
            self.path = self.path or os.path.join(folder, f"score_{os.environ.get('TASK_DEMO', 'run')}_{datetime.now().strftime('%H-%M-%S')}.json")
            with open(self.path, "w") as f:
                json.dump({"final": self.rep, "trace": self.trace, "crossings": [round(c, 3) for c in self.crossings]}, f, indent=0)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


# -- the demo --------------------------------------------------------------------------------

LEVEL = d.level(0.0)
BACK = 0.12  # how far the gripper stands off, toward the robot
PULL = 0.025  # how far the bob is drawn aside to start it: about 6 degrees
GRIP_FORCE = 15.0
FIXED = {"fixed": 0.2505, "fixed_steel": 0.2722, "mid": 0.2613}
AIM_S = 1.0
WITHIN_S = 0.005


def _bob_point(s: float, x_off: float = 0.0) -> tuple[float, float, float]:
    return (BASE_XY[0] + x_off, BASE_XY[1], TOP + PIVOT_H - s)


def _along(ctx) -> float:
    return BOB_START - ctx.joint("pend", "bob")


def _take_bob(ctx, arm):
    """From beside the hanging rod, close on the bob. The fingers pass either side of it, along the swing."""
    s = _along(ctx)
    yield from arm.move((BASE_XY[0] - 0.14, BASE_XY[1] + 0.06, TOP + PIVOT_H - s), grip=LEVEL, finger=d.FINGER_OPEN, t=2.5, label="level, back from the rod")
    yield from arm.move(_bob_point(s, -BACK), t=1.0, label="beside the bob")
    yield from arm.move(_bob_point(s), t=1.0, label="round the bob")
    yield from arm.close(width=2 * BOB_R, force=GRIP_FORCE, t=0.4, label="close on the bob")


def _slide_bob(ctx, arm, s: float):
    """Slide the gripped bob along the rod until it is seen at `s`. The arm is soft and the friction spring stiff, so a small move would only bend the arm: keep shifting the command by what the bob, as seen, still lacks."""
    c = arm.at().copy()
    rot, good = arm.grip.rot, 0
    for _ in range(round(6.0 * d.CONTROL_HZ)):
        err = s - _along(ctx)
        good = good + 1 if abs(err) < 0.001 else 0
        if good >= 3:
            break
        c[2] -= float(np.clip(0.25 * err, -0.0025, 0.0025))
        arm._solve(c, rot)
        yield
    print(f"[pend] bob asked for {s:.4f} m, now at {_along(ctx):.4f} m", flush=True)


def _let_swing(ctx, arm, s: float):
    """Draw the bob aside on its arc, let go, and back away ahead of the swing's return."""
    yield from arm.move((BASE_XY[0] - PULL, BASE_XY[1], TOP + PIVOT_H - math.sqrt(s * s - PULL * PULL)), t=0.6, label="draw the bob aside")
    yield from arm.wait(0.2)
    yield from arm.open(t=0.15, label="let go")
    yield from arm.move(_bob_point(s, -BACK), t=0.5, label="back away")


def _time_cycles(ctx, n: int):
    """Watch `n` cycles of the rod passing upright, as a policy could. Returns the mean cycle in seconds, or None."""
    last = ctx.deg("pend", "rod")
    seen: list[float] = []
    for _ in range(round((n + 2) * 1.3 * d.CONTROL_HZ)):
        yield
        now = ctx.deg("pend", "rod")
        if last < 0.0 <= now:
            seen.append(ctx.t - (now / max(now - last, 1e-6)) * d.DT)
            if len(seen) > n:
                return (seen[-1] - seen[0]) / n
        last = now
    return None


def _catch(ctx, arm, s: float):
    """Bring the open fingers round the swinging bob, and close as it passes upright. Then stand the rod upright again."""
    yield from arm.move(_bob_point(s), finger=d.FINGER_OPEN, t=0.6, label="round the swinging bob")
    yield from ctx.until(lambda: abs(ctx.deg("pend", "rod")) < 2.5, timeout=1.5, label="the bob passes upright")
    yield from arm.close(width=2 * BOB_R, force=GRIP_FORCE, t=0.2, label="catch the bob")
    yield from arm.wait(0.3)
    yield from arm.move(_bob_point(s), t=0.4, label="the rod upright")
    yield from arm.wait(0.2)


def demo_script(ctx):
    """Closed loop: the rod's angle and the bob's place come from what a policy could see. No scorer state is read, and the rod's kind is never read: it is found by timing."""
    mode, arm = ctx.mode, ctx.left
    if mode == "idle":
        yield from arm.wait(5.0)
        return
    yield from _take_bob(ctx, arm)
    if mode in FIXED or mode == "still":
        s = FIXED.get(mode, 0.2505)
        yield from _slide_bob(ctx, arm, s)
        if mode == "still":
            yield from arm.open(t=0.3, label="let go")
            yield from arm.move(_bob_point(s, -BACK), t=1.0, label="back away")
            yield from arm.wait(14.0)
            return
        yield from _let_swing(ctx, arm, _along(ctx))
        yield from arm.wait(14.0)
        return
    s = _along(ctx)
    yield from _let_swing(ctx, arm, s)
    if mode == "touch":
        yield from arm.move(_bob_point(s), t=0.6, label="round the swinging bob")
        yield from arm.wait(14.0)
        return
    if mode == "unset":
        yield from arm.wait(14.0)
        return
    # Time it, move the bob, and time it again. The cycle's square is close to a straight line in the bob's place, so after the first move each estimate is a secant.
    points = []
    for i in range(4):
        cycle = yield from _time_cycles(ctx, 4)
        if cycle is None:
            print("[pend] the swing was not seen", flush=True)
            return
        points.append((s, cycle))
        print(f"[pend] bob at {s:.4f} m: a cycle takes {cycle:.4f} s", flush=True)
        if abs(cycle - AIM_S) < WITHIN_S and i > 0:
            break
        if len(points) == 1:
            goal = s * (AIM_S / cycle) ** 2
        else:
            (s0, c0), (s1, c1) = points[-2:]
            goal = s1 + (AIM_S**2 - c1**2) * (s1 - s0) / (c1**2 - c0**2)
        goal = float(np.clip(goal, BOB_TRAVEL[0] + 0.01, BOB_TRAVEL[1] - 0.01))
        yield from _catch(ctx, arm, s)
        yield from _slide_bob(ctx, arm, goal)
        s = _along(ctx)
        yield from _let_swing(ctx, arm, s)
    yield from arm.move((0.16, 0.33, TOP + 0.22), grip=d.down(0.0), t=1.5, label="clear of the swing")
    yield from arm.wait(13.0)
