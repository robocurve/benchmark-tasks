"""Task: find which sealed box holds a loose ball, using a rocker plate.

Ability: hidden structure. Three opaque boxes have the same size and mass. One holds a
heavy ball that can roll inside; the others hold the same mass fixed at the centre.
The rocker plate is a see-saw with a weak restoring bob: with a fixed-mass box it
returns level after you push one end down; with the loose-ball box the ball rolls to
the low end and holds the plate tipped. All of this is real MuJoCo physics (the ball is
a child body on two slide joints inside the box; parent-child contacts are excluded).

A test is a box resting alone on the plate after one end was pushed down, plate still
for 1.5 s: outcome = which side the plate rests on (or level). The loose-ball box always
follows the push; a fixed box returns level, or (if placed off centre) always rests on
the same side. So "followed pushes on both ends" proves the ball, and "level" or "did
not follow" rules a box out.

Hard: one of the other boxes is a decoy with its 120 g fixed off-centre, so it follows a
push on its heavy end; one push per box cannot tell it from the loose ball.
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

LABELS = "ABC"
BOX_HALF = 0.04
BALL_R = 0.012
BALL_TRAVEL = BOX_HALF - BALL_R - 0.004
BOX_SHELL, BALL_M = 0.03, 0.12
SLOT_X = 0.20
SLOT_Y = (-0.17, -0.305, -0.44)
PLATE_X, PLATE_Y = 0.36, -0.305
HINGE_Z = 0.06
PLATE_HALF = (0.06, 0.11, 0.004)
TILT_LIM = np.deg2rad(6.0)
TIPPED = np.deg2rad(3.0)
PUSH_DY = 0.09
MAT_POS, MAT_HALF = (0.27, 0.07), (0.075, 0.075)

INSTRUCTION = (
    "Three sealed boxes A, B and C look and weigh the same, but one has a loose ball "
    "inside. Use the rocker plate to find it: put a box on the plate and push one end of "
    "the plate down. Put the box with the loose ball on the red mat."
)


@dataclass
class Test:
    t: float
    box: str
    pushed: int  # +1: the +y (left) end was pushed down, -1: the -y end
    rest: int  # side the plate rests on after release, same sign convention; 0 = level


@dataclass
class SealedBoxBall:
    difficulty: str = "easy"
    key: str = "sealed_box_ball"
    instruction: str = INSTRUCTION
    tests: list[Test] = field(default_factory=list)

    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        self.ball_box = str(rng.choice(list(LABELS)))
        order = rng.permutation(3)
        self.slots = {LABELS[i]: (SLOT_X, SLOT_Y[k]) for k, i in enumerate(order)}
        # Hard: one of the other boxes is a decoy whose 120 g is fixed off-centre (diagonally, so
        # it shifts the weight sideways however the box is turned): it holds the plate tipped
        # toward its heavy side after any push, so a single push can look like the loose ball.
        self.decoy: str | None = None
        self.decoy_off = (0.0, 0.0)
        if self.difficulty == "hard":
            self.decoy = str(rng.choice([lab for lab in LABELS if lab != self.ball_box]))
            sx, sy = rng.choice([-1, 1], size=2)
            self.decoy_off = (float(sx) * BALL_TRAVEL, float(sy) * BALL_TRAVEL)
        w = spec.worldbody
        props.flat(spec, "mat", MAT_POS, MAT_HALF, props.RED)
        for lab in LABELS:
            x, y = self.slots[lab]
            b = props.block(spec, f"box_{lab}", (x, y), BOX_HALF, BOX_SHELL, label=lab)
            off = self.decoy_off if lab == self.decoy else (0.0, 0.0)
            inner = b.add_body(name=f"ball_{lab}", pos=[off[0], off[1], -BOX_HALF + BALL_R + 0.002])
            if lab == self.ball_box:
                for ax, nm in (([1, 0, 0], "x"), ([0, 1, 0], "y")):
                    inner.add_joint(name=f"ball_{lab}_{nm}", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=ax,
                                    range=[-BALL_TRAVEL, BALL_TRAVEL], limited=mujoco.mjtLimited.mjLIMITED_TRUE,
                                    damping=[0.02, 0, 0])
            inner.add_geom(type=props.G.mjGEOM_SPHERE, size=[BALL_R], mass=BALL_M, rgba=[0.3, 0.3, 0.3, 1],
                           contype=0, conaffinity=0, group=4)

        # Rocker plate: hinge about x at the plate's top surface; light bob below restores it.
        base = w.add_body(name="rocker_base", pos=[PLATE_X, PLATE_Y, 0])
        base.add_geom(type=props.G.mjGEOM_BOX, size=[0.05, 0.03, 0.004], pos=[0, 0, 0.004], rgba=list(props.DARK),
                      contype=0, conaffinity=0)
        base.add_geom(type=props.G.mjGEOM_BOX, size=[0.012, 0.012, (HINGE_Z - 0.012) / 2],
                      pos=[0, 0, (HINGE_Z - 0.012) / 2], rgba=[0.7, 0.7, 0.74, 1], contype=0, conaffinity=0)
        plate = base.add_body(name="rocker", pos=[0, 0, HINGE_Z])
        plate.add_joint(name="rocker_hinge", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[1, 0, 0],
                        range=[-TILT_LIM, TILT_LIM], limited=mujoco.mjtLimited.mjLIMITED_TRUE,
                        damping=[0.05, 0, 0], armature=0.0005)
        plate.add_geom(name="rocker", type=props.G.mjGEOM_BOX, size=list(PLATE_HALF), pos=[0, 0, -PLATE_HALF[2]],
                       rgba=[0.25, 0.55, 0.85, 1], mass=0.05, friction=[1.2, 0.01, 0.001], condim=4)
        plate.add_geom(name="rocker_bob", type=props.G.mjGEOM_BOX, size=[0.02, 0.01, 0.005],
                       pos=[0, 0, -0.08], rgba=[0.8, 0.65, 0.3, 1], mass=0.30, contype=0, conaffinity=0)
        plate.add_geom(type=props.G.mjGEOM_CAPSULE, size=[0.003], fromto=[0, 0, -0.008, 0, 0, -0.08],
                       rgba=[0.7, 0.7, 0.74, 1], mass=0.001, contype=0, conaffinity=0)
        # Pointer on the far side shows the tilt; tick marks at level.
        plate.add_geom(type=props.G.mjGEOM_CAPSULE, size=[0.002], fromto=[PLATE_HALF[0], 0, -0.004,
                       PLATE_HALF[0] + 0.035, 0, -0.004], rgba=[0.9, 0.1, 0.1, 1], mass=0.0005,
                       contype=0, conaffinity=0)
        for dy in (-1, 1):  # end pads marking where to push
            plate.add_geom(type=props.G.mjGEOM_BOX, size=[0.02, 0.012, 0.0006],
                           pos=[0, dy * PUSH_DY, 0.0006], rgba=[0.95, 0.95, 0.95, 1], contype=0, conaffinity=0)

    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._m = model
        self._q = model.joint("rocker_hinge").qposadr[0]
        self._v = model.joint("rocker_hinge").dofadr[0]
        self._plate = model.geom("rocker").id
        self._bgeom = {model.geom(f"box_{lab}").id: lab for lab in LABELS}
        self._tips = props.gripper_geoms(model)
        self.touch = props.TouchTracker(tuple(f"box_{lab}" for lab in LABELS) + ("rocker",))
        self.touch.bind(model, data)
        self.tests = []
        self._push: tuple[str, int] | None = None
        self._still_since: float | None = None

    def angle(self, data: mujoco.MjData) -> float:
        return float(data.qpos[self._q])

    def side(self, data: mujoco.MjData) -> int:
        """Which end is down: +1 the +y end (negative hinge angle), -1 the -y end, 0 level."""
        a = self.angle(data)
        return 0 if abs(a) <= TIPPED else (1 if a < 0 else -1)

    def _contacts(self, data: mujoco.MjData) -> tuple[set[str], set[str], float | None]:
        on_plate, held, tip_y = set(), set(), None
        for i, c in enumerate(data.contact[: data.ncon]):
            g1, g2 = int(c.geom1), int(c.geom2)
            for a, b in ((g1, g2), (g2, g1)):
                if a in self._bgeom and b == self._plate:
                    on_plate.add(self._bgeom[a])
                if a in self._bgeom and b in self._tips:
                    held.add(self._bgeom[a])
                if a == self._plate and b in self._tips:
                    tip_y = float(c.pos[1])
        return on_plate, held, tip_y

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        t = float(data.time)
        self.touch.update(data)
        on_plate, held, tip_y = self._contacts(data)
        box = next(iter(on_plate)) if len(on_plate) == 1 and not held else None
        if tip_y is not None:
            self._still_since = None
            if box is not None and abs(self.angle(data)) > TILT_LIM - np.deg2rad(1.5):
                self._push = (box, 1 if tip_y > PLATE_Y else -1)
            return
        if box is None or self._push is None or self._push[0] != box:
            self._still_since = None
            if box is None:
                self._push = None
            return
        if abs(float(data.qvel[self._v])) < 0.02:
            self._still_since = self._still_since if self._still_since is not None else t
            if t - self._still_since >= 1.5:
                self.tests.append(Test(t, box, self._push[1], self.side(data)))
                self._push = None
        else:
            self._still_since = None

    def hypotheses(self) -> list[str]:
        """Boxes that can hold the loose ball given the tests: the ball box follows every
        push; a fixed box never follows pushes on both ends."""
        def follows_both(lab: str) -> bool:
            f = {x.pushed for x in self.tests if x.box == lab and x.rest == x.pushed}
            return f == {1, -1}

        out = []
        for h in LABELS:
            ok = all(x.rest == x.pushed for x in self.tests if x.box == h)
            ok = ok and not any(follows_both(o) for o in LABELS if o != h)
            ok = ok and (follows_both(h) or not any(follows_both(o) for o in LABELS))
            if ok:
                out.append(h)
        return out

    def on_mat(self, data: mujoco.MjData) -> list[str]:
        _, held, _ = self._contacts(data)
        return [lab for lab in LABELS if lab not in held and props.in_box(
            data.xpos[self._m.body(f"box_{lab}").id], MAT_POS, MAT_HALF, margin=0.015, z_max=BOX_HALF + 0.01)]

    def hud(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
        sc = self.score(model, data)
        lines = [f"t = {data.time:5.1f} s   plate {np.rad2deg(self.angle(data)):+5.1f} deg   "
                 f"candidates left {''.join(self.hypotheses()) or '-'}",
                 f"rubric stage {sc['stage']}/4   tests {sc['tests']}   on mat {sc['on_mat'] or '-'}"]
        name = {1: "left end", -1: "right end", 0: "level"}
        for x in self.tests[-3:]:
            lines.append(f"  box {x.box}: pushed {name[x.pushed]} -> rests {name[x.rest]}")
        return lines

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        hyps = self.hypotheses()
        mat = self.on_mat(data)
        success = mat == [self.ball_box]
        stage = 0
        if self.touch.touched:
            stage = 1
        if self.tests:
            stage = 2
        if hyps == [self.ball_box]:
            stage = 3
            if success:
                stage = 4
        return {
            "stage": stage,
            "success": float(success),
            "science_success": float(stage >= 3),
            "tests": len(self.tests),
            "candidates_left": len(hyps),
            "procedure_violations": 0,
            "ball_box": self.ball_box,
            "on_mat": mat,
            "log": [x.__dict__ for x in self.tests],
        }


# ---------- reference policies ----------
def _carry(env, mo: Motion, lab: str, xy, surface_z: float):  # noqa: ANN001, ANN202
    side = "left" if xy[1] > PLATE_Y else "right"
    return move_body(env, mo, f"box_{lab}", side, xy, surface_z, half=BOX_HALF)


def oracle(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Test boxes one at a time: push the left end down, release, watch the plate. Level or
    resting on the other side rules the box out; resting on the pushed side needs the
    opposite push too (an off-centre fixed box also rests tipped, but only one way).
    Stop when the tests leave one candidate; carry it to the mat."""
    t: SealedBoxBall = env.task
    mo = Motion(env)
    order = sorted(LABELS, key=lambda lab: -t.slots[lab][1])
    end = {1: ("left", PLATE_Y + PUSH_DY), -1: ("right", PLATE_Y - PUSH_DY)}
    on_plate: str | None = None
    for lab in order:
        if len(t.hypotheses()) == 1:
            break
        if on_plate is not None:
            yield from _carry(env, mo, on_plate, t.slots[on_plate], 0.0)
        yield from _carry(env, mo, lab, (PLATE_X, PLATE_Y), HINGE_Z)
        on_plate = lab
        yield from mo.hold(1.0)
        for _ in range(2):  # the box can miss or slide off the plate: put it back on
            if lab in t._contacts(env.data)[0]:
                break
            yield from _carry(env, mo, lab, (PLATE_X, PLATE_Y), HINGE_Z)
            yield from mo.hold(1.0)
        for sgn in (1, -1):
            arm, y = end[sgn]
            n = len(t.tests)
            top = np.array([PLATE_X, y, HINGE_Z + PUSH_DY * np.sin(TILT_LIM)])
            for _ in range(2):  # retry a push the monitor did not register
                yield from mo.press(arm, top, depth=0.03, hold=0.8)
                yield from mo.home(arm)
                yield from mo.hold(2.5)
                if len(t.tests) > n:
                    break
            if len(t.tests) == n or t.tests[-1].rest != sgn or len(t.hypotheses()) == 1:
                break
    hyps = t.hypotheses()
    if hyps:
        yield from _carry(env, mo, hyps[0], MAT_POS, 0.0)
    yield from mo.hold(1.5)


def _first_follower(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """The easy instruction taken literally: push one end of the plate for each box in turn;
    the first box that keeps the plate tipped toward the pushed end goes on the mat."""
    t: SealedBoxBall = env.task
    mo = Motion(env)
    order = sorted(LABELS, key=lambda lab: -t.slots[lab][1])
    pick, on_plate = None, None
    for lab in order:
        if on_plate is not None:
            yield from _carry(env, mo, on_plate, t.slots[on_plate], 0.0)
        yield from _carry(env, mo, lab, (PLATE_X, PLATE_Y), HINGE_Z)
        on_plate = lab
        yield from mo.hold(1.0)
        n = len(t.tests)
        top = np.array([PLATE_X, PLATE_Y + PUSH_DY, HINGE_Z + PUSH_DY * np.sin(TILT_LIM)])
        yield from mo.press("left", top, depth=0.03, hold=0.8)
        yield from mo.home("left")
        yield from mo.hold(2.5)
        if len(t.tests) > n and t.tests[-1].rest == 1:
            pick = lab
            break
    if pick is not None:
        yield from _carry(env, mo, pick, MAT_POS, 0.0)
    yield from mo.hold(1.5)


def guess(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Blind baseline: carry a random box to the mat without testing."""
    rng = rng or np.random.default_rng()
    mo = Motion(env)
    lab = str(rng.choice(list(LABELS)))
    yield from _carry(env, mo, lab, MAT_POS, 0.0)
    yield from mo.hold(1.5)


TASK = SealedBoxBall
POLICIES = {"oracle": oracle, "guess": guess, "noop": noop, "naive": _first_follower}
SPEC = TaskSpec(
    key="sealed_box_ball",
    title="Find the sealed box with a loose ball",
    ability="Hidden structure",
    instruction=INSTRUCTION,
    objects=(
        "three sealed cardboard boxes A, B, C, 8 cm cubes, 150 g each",
        "inside one box: a 120 g ball free to roll; inside the others: 120 g fixed at the centre (hard: one of them fixed off-centre)",
        "rocker plate 12 x 22 cm on a centre pivot 6 cm high, tilts +-6 deg, white push pads at both ends",
        "red target mat, 15 x 15 cm",
    ),
    setup=(
        "Boxes in a row 20 cm in front of the arms, 13.5 cm apart, random order.",
        "Rocker plate 36 cm in front of the arms, centred between them, pivot axis pointing away "
        "from the robot; check it rests level when empty.",
        "Red mat on the operator's right, 27 cm in front of the left arm's base.",
    ),
    hidden="Which box holds the loose ball (uniform over A, B, C).",
    instruction_hard=(
        "One of the sealed boxes A, B and C has a loose ball inside. Put that box on the red mat."
    ),
    hidden_hard=(
        "Which box holds the loose ball (uniform), and which other box is a decoy with its 120 g fixed "
        "2.4 cm off-centre (diagonally, random corner). On the rocker plate the decoy holds the plate "
        "tipped toward its heavy side after any push, so it 'follows' a push on that end. Only the "
        "loose-ball box follows pushes on both ends."
    ),
    rubric=(
        "No purposeful contact with the boxes or the plate",
        "Touches or moves a box or the plate",
        "At least one test: a box alone on the plate, one end pushed down and released, plate watched for 1.5 s",
        "Tests leave exactly one candidate, the true loose-ball box (a box that follows only one push direction is not proof; hard: the decoy does that)",
        "Stage 3, and the loose-ball box (only) resting on the mat",
    ),
    metrics=(
        "stage (0-4)",
        "success: only the loose-ball box on the mat, evidence or not",
        "science_success: stage >= 3",
        "tests used (pushes observed)",
    ),
    report_fields=("tests", "candidates_left", "ball_box", "on_mat"),
    max_seconds=260.0,
    limitations=("The ball moves on two frictionless slide joints inside the box rather than rolling; "
                 "the plate's restoring force is a small bob under the pivot."),
)
