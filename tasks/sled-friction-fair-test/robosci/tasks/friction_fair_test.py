"""Task: which sled has the higher-friction base? A fair test needs equal sled masses.

Ability: controlled experiments. Hidden variable: the friction coefficient of each sled's
base. The launcher gives every sled the same impulse J, so a sled of total mass M leaves at
v = J / M and slides d = v^2 / (2 mu g) = J^2 / (2 M^2 mu g). Mass is a confound: the light
sled (150 g) alone flies much farther than the heavy one (250 g) whatever its base. Putting
the 100 g weight on the light sled makes both 250 g, and then the shorter slide means the
higher friction.
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

SLED_HALF = (0.04, 0.03, 0.02)
W_HALF = (0.02, 0.02, 0.015)
SLEDS = {"light": 0.15, "heavy": 0.25}
W_MASS = 0.10
TRACK_X = 0.33
LANES = (0.27, 0.39)  # two lanes, one launcher cradle each
TRACK_TOP = 0.004
TRACK_Y = (0.09, -0.86)
CRADLE_Y = 0.03  # sled centre when loaded in the launcher
J = 0.40  # launch impulse, N s
START = {"light": (0.15, -0.07), "heavy": (0.15, -0.19), "weight": (0.15, 0.05)}
RELAY_XY = (0.15, -0.31)  # free table spot both arms reach
BUTTON = props.ButtonSpec("launch", (0.49, 0.04), rgba=(0.85, 0.15, 0.15, 1))
MAT_XY, MAT_HALF = (0.52, -0.16), (0.06, 0.06)
WALL_Y = TRACK_Y[1] + 0.03
LABEL = {"light": "150g", "heavy": "250g", "weight": "100g"}

INSTRUCTION = (
    "The two sleds have bases made of different materials. Find which base has more friction and put "
    "that sled on the red mat. The track has two lanes with a launcher at the top of each; the red "
    "button fires every loaded launcher, and every launch gives the same push. The sleds weigh 150 g "
    "and 250 g, and there is a 100 g weight you can put on a sled."
)


@dataclass
class Launch:
    t0: float
    sled: str
    mass: float
    y0: float
    dist: float | None = None
    wall: bool = False
    spilled: bool = False
    done: bool = False
    still: float = 0.0


@dataclass
class FrictionFairTest:
    difficulty: str = "easy"
    key: str = "friction_fair_test"
    instruction: str = INSTRUCTION
    mu: dict[str, float] = field(default_factory=dict)
    launches: list[Launch] = field(default_factory=list)

    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        lo, hi = (0.25, 0.45) if self.difficulty == "easy" else (0.30, 0.40)
        hi_sled = str(rng.choice(list(SLEDS)))
        self.mu = {s: (hi if s == hi_sled else lo) for s in SLEDS}
        w = spec.worldbody
        L = (TRACK_Y[0] - TRACK_Y[1]) / 2
        w.add_geom(name="track", type=props.G.mjGEOM_BOX, size=[0.12, L, TRACK_TOP / 2],
                   pos=[TRACK_X, (TRACK_Y[0] + TRACK_Y[1]) / 2, TRACK_TOP / 2], rgba=[0.85, 0.86, 0.88, 1],
                   friction=[0.05, 0.005, 0.0001])
        for k in range(1, 9):  # 10 cm marks from the launcher
            props.flat(spec, f"mark{k}", (TRACK_X, CRADLE_Y - 0.1 * k), (0.12, 0.0015), (0.2, 0.2, 0.2, 1),
                       z=TRACK_TOP)
        for i, x in enumerate(LANES):
            props.flat(spec, f"cradle{i}", (x, CRADLE_Y), (0.05, 0.04), (0.95, 0.80, 0.30, 1), z=TRACK_TOP)
        props.flat(spec, "divider", (TRACK_X, (CRADLE_Y + WALL_Y) / 2), (0.002, (CRADLE_Y - WALL_Y) / 2),
                   (0.3, 0.3, 0.3, 1), z=TRACK_TOP)
        # Launcher back plate (spring plunger housing) and the end wall.
        w.add_geom(type=props.G.mjGEOM_BOX, size=[0.12, 0.008, 0.025], pos=[TRACK_X, TRACK_Y[0] - 0.006, 0.025],
                   rgba=[0.75, 0.15, 0.15, 1])
        w.add_geom(type=props.G.mjGEOM_BOX, size=[0.12, 0.006, 0.02], pos=[TRACK_X, WALL_Y - 0.03, 0.02],
                   rgba=[0.3, 0.3, 0.3, 1])
        props.push_button(spec, BUTTON)
        props.flat(spec, "mat", MAT_XY, MAT_HALF, props.RED)
        for s, m in SLEDS.items():
            props.block(spec, f"sled_{s}", START[s], SLED_HALF, m, label=LABEL[s], bg=(90, 120, 160),
                        fg=(250, 250, 250), friction=(self.mu[s], 0.005, 0.0001))
        props.block(spec, "weight", START["weight"], W_HALF, W_MASS, label=LABEL["weight"],
                    bg=(150, 150, 155), fg=(20, 20, 20), friction=(1.0, 0.01, 0.001))

    # ---------- runtime ----------
    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._m = model
        self.button = props.Button(BUTTON)
        self.button.bind(model)
        self.touch = props.TouchTracker(("sled_light", "sled_heavy", "weight", "launch"))
        self.touch.bind(model, data)
        self._tips = props.gripper_geoms(model)
        self._g = {model.geom(n).id: n for n in ("sled_light", "sled_heavy", "weight")}
        self._dof = {n: model.joint(f"{n}_free").dofadr[0] for n in ("sled_light", "sled_heavy", "weight")}
        self.launches = []

    def pos(self, data: mujoco.MjData, name: str) -> np.ndarray:
        return data.xpos[self._m.body(name).id]

    def held(self, data: mujoco.MjData) -> set[str]:
        return {self._g[g] for g in props.held_geoms(self._m, data, self._tips) if g in self._g}

    def weight_on(self, data: mujoco.MjData, sled: str) -> bool:
        p, q = self.pos(data, "weight"), self.pos(data, f"sled_{sled}")
        return bool(np.all(np.abs(p[:2] - q[:2]) < [SLED_HALF[0], SLED_HALF[1]])
                    and 0 < p[2] - q[2] < SLED_HALF[2] + W_HALF[2] + 0.006)

    def in_cradle(self, data: mujoco.MjData, sled: str) -> bool:
        p = self.pos(data, f"sled_{sled}")
        return (min(abs(p[0] - x) for x in LANES) < 0.03 and abs(p[1] - CRADLE_Y) < 0.035
                and p[2] < TRACK_TOP + 0.02)

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        t = float(data.time)
        self.touch.update(data)
        held = self.held(data)
        busy = any(not ln.done for ln in self.launches)
        if self.button.update(data) and not busy:
            ready = [s for s in SLEDS if self.in_cradle(data, s) and f"sled_{s}" not in held]
            for s in ready if "weight" not in held else []:  # each loaded cradle fires
                riders = ["weight"] if self.weight_on(data, s) else []
                mass = SLEDS[s] + W_MASS * len(riders)
                v = J / mass  # the spring plunger's impulse, shared by sled and load
                for b in [f"sled_{s}", *riders]:
                    data.qvel[self._dof[b] + 1] = -v
                self.launches.append(Launch(t, s, mass, float(self.pos(data, f"sled_{s}")[1])))
        for ln in self.launches:
            if ln.done:
                continue
            b = f"sled_{ln.sled}"
            if b in held:
                ln.done = True
                continue
            v = float(np.linalg.norm(data.qvel[self._dof[b]:self._dof[b] + 3]))
            if v < 0.003 and t - ln.t0 > 0.3:
                ln.still += 0.02
                if ln.still >= 0.3:
                    y = float(self.pos(data, b)[1])
                    ln.done, ln.dist = True, ln.y0 - y
                    ln.wall = y < WALL_Y + SLED_HALF[1] + 0.01
                    ln.spilled = ln.mass > SLEDS[ln.sled] and not self.weight_on(data, ln.sled)
            else:
                ln.still = 0.0

    # ---------- evidence ----------
    def fair_pair(self) -> tuple[Launch, Launch] | None:
        """A light-sled and a heavy-sled launch at equal total mass, both stopped on the track."""
        ok = [ln for ln in self.launches if ln.dist is not None and not ln.wall and not ln.spilled]
        for a in ok:
            for b in ok:
                if a.sled == "light" and b.sled == "heavy" and abs(a.mass - b.mass) < 0.01 \
                        and abs(a.dist - b.dist) > 0.03:
                    return a, b
        return None

    def proven_high(self) -> str | None:
        pair = self.fair_pair()
        if pair is None:
            return None
        return min(pair, key=lambda ln: ln.dist).sled

    def on_mat(self, data: mujoco.MjData) -> set[str]:
        held = self.held(data)
        return {s for s in SLEDS if f"sled_{s}" not in held
                and props.in_box(self.pos(data, f"sled_{s}"), MAT_XY, MAT_HALF, z_max=0.05)}

    def high(self) -> str:
        return max(self.mu, key=self.mu.__getitem__)

    def hud(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
        sc = self.score(model, data)
        lines = [f"t = {data.time:5.1f} s   launches {sc['launches']}   fair comparison: "
                 f"{'yes' if sc['fair_comparison'] else 'no'}   mat {sorted(sc['on_mat']) or '-'}",
                 f"rubric stage {sc['stage']}/4   higher friction (from fair test): {sc['proven_high'] or '?'}"]
        for ln in self.launches[-3:]:
            d = ("wall" if ln.wall else f"{ln.dist:.2f} m") if ln.dist is not None else "..."
            lines.append(f"  {LABEL[ln.sled]} sled, total {ln.mass * 1000:.0f} g: {d}")
        return lines

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        mat = self.on_mat(data)
        success = mat == {self.high()}
        proven = self.proven_high()
        stage = 0
        if self.touch.touched or self.button.presses:
            stage = 1
        if any(ln.dist is not None for ln in self.launches):
            stage = 2
        if proven == self.high():
            stage = 3
            if success:
                stage = 4
        return {
            "stage": stage,
            "success": float(success),
            "science_success": float(stage >= 3),
            "launches": len(self.launches),
            "fair_comparison": float(self.fair_pair() is not None),
            "proven_high": proven,
            "high_friction": self.high(),
            "on_mat": sorted(mat),
            "procedure_violations": sum(ln.spilled for ln in self.launches),  # weight fell off in a run
            "log": [ln.__dict__ for ln in self.launches],
        }


# ---------- reference policies ----------
def _carry_yaw(env, mo: Motion, body: str, target, *, dz: float, grip: float = 0.005,  # noqa: ANN001
               open_to: float = 0.045, yaws=(np.pi / 2, -np.pi / 2), relay_xy=None, relay_z=None):  # noqa: ANN001, ANN202
    """Pick/place with the fingers along world y and the SAME tilt at both ends, so the object
    never turns in the hand (the stock place() may switch tilt and drop a slippery sled).
    Falls back to a hand-over through ``relay_xy``."""
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
        if mo.held[arm] is not None:  # measured in-hand offset
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
    if relay_xy is not None:
        mid = np.array([relay_xy[0], relay_xy[1], relay_z])
        yield from _carry_yaw(env, mo, body, mid, dz=dz, grip=grip, open_to=open_to, yaws=yaws)
        yield from mo.hold(0.3)
        yield from _carry_yaw(env, mo, body, target, dz=dz, grip=grip, open_to=open_to, yaws=yaws)
        return
    raise RuntimeError(f"no arm can carry {body} to {np.round(target, 3)}")


def _carry(env, mo: Motion, name: str, xy, surface: float):  # noqa: ANN001, ANN202
    half = W_HALF[2] if name == "weight" else SLED_HALF[2]
    dz = 0.004 if name == "weight" else -0.004  # pads overlap most of the face, clear the table
    return _carry_yaw(env, mo, name, (xy[0], xy[1], surface + half + 0.004), dz=dz,
                      relay_xy=RELAY_XY, relay_z=half + 0.004)


def _launch(env, mo: Motion):  # noqa: ANN001, ANN202
    t: FrictionFairTest = env.task
    n = len(t.launches)
    yield from mo.hold(0.5)
    yield from mo.press("left", t.button.top(), depth=BUTTON.travel + 0.004)
    yield from mo.home("left")
    for _ in range(80):
        yield from mo.hold(0.1)
        if len(t.launches) > n and t.launches[-1].done:
            return


def oracle(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Fair test: heavy sled alone (250 g) in one lane, light sled + 100 g weight (250 g) in the
    other, launched together. The shorter slide has the higher-friction base. Uses printed
    masses and observed slide distances only."""
    t: FrictionFairTest = env.task
    mo = Motion(env)
    yield from _carry(env, mo, "sled_heavy", (LANES[1], CRADLE_Y), TRACK_TOP)
    yield from _carry(env, mo, "sled_light", (LANES[0], CRADLE_Y), TRACK_TOP)
    yield from _carry(env, mo, "weight", (LANES[0], CRADLE_Y), TRACK_TOP + 2 * SLED_HALF[2])
    yield from _launch(env, mo)
    for _ in range(40):  # both sleds stopped?
        if len(t.launches) >= 2 and all(ln.done for ln in t.launches):
            break
        yield from mo.hold(0.1)
    d = {ln.sled: ln.dist for ln in t.launches if ln.dist is not None}
    if d.get("heavy", 9.0) < d.get("light", 9.0):
        yield from _carry(env, mo, "sled_heavy", MAT_XY, 0.0)
    else:
        yield from _carry(env, mo, "weight", START["weight"], 0.0)
        yield from _carry(env, mo, "sled_light", MAT_XY, 0.0)
    yield from mo.hold(1.5)


def naive(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Unfair test: both sleds launched as they are (150 g vs 250 g); the shorter slide is
    taken as the higher friction, ignoring the mass confound."""
    t: FrictionFairTest = env.task
    mo = Motion(env)
    yield from _carry(env, mo, "sled_heavy", (LANES[1], CRADLE_Y), TRACK_TOP)
    yield from _carry(env, mo, "sled_light", (LANES[0], CRADLE_Y), TRACK_TOP)
    yield from _launch(env, mo)
    for _ in range(40):
        if len(t.launches) >= 2 and all(ln.done for ln in t.launches):
            break
        yield from mo.hold(0.1)
    d = {ln.sled: ln.dist for ln in t.launches if ln.dist is not None}
    pick = "heavy" if d.get("heavy", 9.0) < d.get("light", 9.0) else "light"
    yield from _carry(env, mo, f"sled_{pick}", MAT_XY, 0.0)
    yield from mo.hold(1.5)


def guess(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Blind baseline: a random sled to the mat, no launches."""
    rng = rng or np.random.default_rng()
    mo = Motion(env)
    s = str(rng.choice(list(SLEDS)))
    yield from _carry(env, mo, f"sled_{s}", MAT_XY, 0.0)
    yield from mo.hold(1.5)


TASK = FrictionFairTest
POLICIES = {"oracle": oracle, "guess": guess, "noop": noop, "naive": naive}
SPEC = TaskSpec(
    key="friction_fair_test",
    title="Fair friction test with a mass confound",
    ability="Controlled experiments",
    instruction=INSTRUCTION,
    objects=(
        "two sleds 8 x 6 x 4 cm, same look, printed 150g and 250g, bases of two hidden materials",
        "100 g weight block (4 x 4 x 3 cm) that sits on a sled",
        "low-friction two-lane track 24 x 95 cm with 10 cm marks, one spring launcher (fixed impulse) "
        "per lane at the top, a red button that fires both, end wall",
        "red target mat, 13 x 13 cm",
    ),
    setup=(
        "Track 33 cm in front of the arms, launchers at the left arm's end, running toward the right arm.",
        "Sleds and weight on the table between the left arm and the track; launch button beyond the "
        "launchers.",
        "Red mat on the far side of the track, opposite the launchers' end.",
    ),
    hidden=(
        "Base friction coefficients: one sled 0.25 and the other 0.45, random per episode, independent "
        "of which sled is heavier. Launched alone, the light sled always slides farther (often to the "
        "end wall), so a naive comparison is wrong half the time."
    ),
    instruction_hard=(
        "The two sleds have bases made of different materials. Find which base has more friction and "
        "put that sled on the red mat. The red button fires the launchers at the top of the track."
    ),
    hidden_hard=(
        "Base friction coefficients 0.30 and 0.40, random per episode. The instruction no longer says "
        "that every launch gives the same push, that the sleds differ in mass, or what the 100 g weight "
        "is for; the masses are only printed on the blocks. Comparing the sleds as they are picks the "
        "heavy sled every time (right half the time) and earns no science credit."
    ),
    rubric=(
        "No purposeful contact with the sleds, the weight or the launch button",
        "Touches a sled or the weight, or presses the button",
        "At least one launch with the sled stopping on the track",
        "A fair comparison: both sleds launched at the same total mass (light sled + weight vs heavy, "
        "together or one after the other), "
        "neither hitting the wall; the shorter slide identifies the higher-friction base",
        "Stage 3 and only the higher-friction sled is on the mat",
    ),
    metrics=(
        "stage (0-4)",
        "success: only the higher-friction sled on the mat, proof or not",
        "science_success: stage >= 3",
        "launches used; whether a mass-matched comparison was made",
        "procedure violations: launches where the weight fell off the sled",
    ),
    report_fields=("launches", "fair_comparison", "proven_high", "high_friction", "on_mat"),
    max_seconds=140.0,
    limitations=("The launcher sets the sled's (and its load's) velocity to J / M in code on the button "
                 "press; sliding friction is MuJoCo's Coulomb friction on the sled base."),
)
