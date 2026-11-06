"""Task: two cylinders look and weigh the same; one is hollow. Find it with a roll race.

Ability: comparative measurement. Hidden variable: which cylinder is hollow (its mass sits
at the rim, so its moment of inertia is larger and it rolls down a ramp more slowly).
The robot lays both cylinders behind a start gate at the top of a two-lane ramp and presses
the gate button; the gate drops and both roll down together. A solid cylinder accelerates
at 2/3 g sin(theta), a thin-walled one at 1/2 g sin(theta), so the solid one crosses the
finish line first. A foam pad past the finish line stops them.
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

R = 0.025  # cylinder radius
HL = 0.03  # half length (axis along world x)
MASS = 0.15
LABELS = ("P", "Q")
LABELS_HARD = ("P", "Q", "R")
K_HARD = (1.0, 0.7, 0.5)  # hard: thin-walled tube, thick-walled tube (inner radius 0.63 r), solid (I = k m r^2)
LANE_X = (0.30, 0.40)
THETA = np.deg2rad(15.0)
Y_TOP, Y_BOT = 0.05, -0.16  # ramp runs downhill toward -y
Y_GATE = -0.025
FINISH_Y = -0.37
FOAM_Y = (-0.48, -0.66)
START_XY = {"P": (0.205, -0.02), "Q": (0.205, -0.14), "R": (0.12, -0.26)}
RELAY_XY = (0.205, -0.29)  # empty cradle both arms reach (hand-over spot)
PARK_XY = (0.205, -0.40)  # hard: empty cradle by the right arm, to clear a lane
BUTTON = props.ButtonSpec("gate_button", (0.17, 0.065), rgba=(0.9, 0.55, 0.1, 1))
BIN_XY, BIN_HALF = (0.19, -0.50), (0.05, 0.05)
GATE_DROP = 0.04
INSTRUCTION = (
    "Cylinders P and Q look the same and weigh the same, but one of them is hollow. Use the "
    "ramp to find the hollow cylinder and put it in the red bin. Pressing the orange button "
    "opens the start gate at the top of the ramp."
)


def ramp_z(y: float) -> float:
    """Height of the ramp surface at ``y`` (0 on the flat table below the ramp)."""
    return float(np.clip(y - Y_BOT, 0.0, Y_TOP - Y_BOT) * np.tan(THETA))


@dataclass
class Trial:
    t0: float
    starters: tuple[str, ...]
    cross: dict[str, float] = field(default_factory=dict)
    done: bool = False
    lane_x: dict[str, float] = field(default_factory=dict)


@dataclass
class RollRace:
    difficulty: str = "easy"
    key: str = "roll_race"
    instruction: str = INSTRUCTION
    hollow: str = "P"
    labels: tuple[str, ...] = LABELS
    k: dict[str, float] = field(default_factory=dict)
    trials: list[Trial] = field(default_factory=list)

    # ---------- build ----------
    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        if self.difficulty == "hard":
            self.labels = LABELS_HARD
            perm = rng.permutation(len(K_HARD))
            self.k = {lab: K_HARD[perm[i]] for i, lab in enumerate(LABELS_HARD)}
            self.hollow = max(self.k, key=self.k.__getitem__)  # the thin-walled tube
        else:
            self.labels = LABELS
            self.hollow = str(rng.choice(LABELS))
            self.k = {lab: (1.0 if lab == self.hollow else 0.5) for lab in LABELS}  # I = k m R^2
        w = spec.worldbody
        # Ramp (one wide sloped plank under both lanes).
        L = (Y_TOP - Y_BOT) / np.cos(THETA)
        n = np.array([0.0, -np.sin(THETA), np.cos(THETA)])
        top_mid = np.array([0.35, (Y_TOP + Y_BOT) / 2, ramp_z(Y_TOP) / 2])
        c = top_mid - 0.005 * n
        w.add_geom(name="ramp", type=props.G.mjGEOM_BOX, size=[0.1, L / 2, 0.005], pos=list(c),
                   quat=[np.cos(THETA / 2), np.sin(THETA / 2), 0, 0], rgba=[0.62, 0.48, 0.30, 1],
                   friction=[1.0, 0.005, 0.0])
        w.add_geom(type=props.G.mjGEOM_BOX, size=[0.1, 0.005, ramp_z(Y_TOP) / 2 + 0.01],
                   pos=[0.35, Y_TOP + 0.003, ramp_z(Y_TOP) / 2 + 0.01], rgba=[0.5, 0.38, 0.24, 1])
        for i, x in enumerate(LANE_X):
            props.flat(spec, f"lane{i}", (x, (FINISH_Y + Y_BOT) / 2 - 0.1), (0.002, 0.2), (1, 1, 1, 0.8))
        props.flat(spec, "finish", (0.35, FINISH_Y), (0.1, 0.004), (0.05, 0.05, 0.05, 1))
        props.flat(spec, "foam", (0.35, sum(FOAM_Y) / 2), (0.1, (FOAM_Y[0] - FOAM_Y[1]) / 2),
                   (0.30, 0.45, 0.65, 1))
        # Start gate: a bar across both lanes on a mocap body (drops into the ramp when open).
        gz = ramp_z(Y_GATE) + 0.012
        gate = w.add_body(name="gate", mocap=True, pos=[0.35, Y_GATE, gz])
        gate.add_geom(name="gate", type=props.G.mjGEOM_BOX, size=[0.1, 0.004, 0.012],
                      rgba=[0.9, 0.55, 0.1, 1])
        props.push_button(spec, BUTTON)
        props.tray(spec, "bin", BIN_XY, BIN_HALF, rgba=props.RED)
        extra = (PARK_XY,) if self.difficulty == "hard" else ()
        for x, y in (*(START_XY[lab] for lab in self.labels), RELAY_XY, *extra):  # cradles: two low rails so a cylinder cannot roll away
            for sy in (-1, 1):
                w.add_geom(type=props.G.mjGEOM_BOX, size=[0.03, 0.002, 0.0065],
                           pos=[x, y + sy * 0.022, 0.0065], rgba=[0.25, 0.25, 0.28, 1])
        # Cylinders: identical outside; inertia set explicitly (hollow: mass at the rim).
        for lab in self.labels:
            x, y = START_XY[lab]
            b = w.add_body(name=f"cyl_{lab}", pos=[x, y, R + 0.004])
            b.add_freejoint(name=f"cyl_{lab}_free")
            k = self.k[lab]
            i_t = MASS * (3 * R * R + 4 * HL * HL) / 12
            b.explicitinertial = True
            b.mass = MASS
            b.ipos = [0, 0, 0]  # MjSpec otherwise leaves the CoM at the body's world position
            b.iquat = [1, 0, 0, 0]
            b.inertia = [k * MASS * R * R, i_t, i_t]
            q = [np.cos(np.pi / 4), 0, np.sin(np.pi / 4), 0]  # cylinder axis z -> x
            for s in (-1, 1):  # two rims carry the contacts (stable line contact)
                b.add_geom(name=f"cyl_{lab}_rim{s:+d}", type=props.G.mjGEOM_CYLINDER, size=[R, 0.004],
                           pos=[s * (HL - 0.004), 0, 0], quat=q, rgba=[0.55, 0.57, 0.62, 1],
                           friction=[1.2, 0.005, 0.0], condim=4, mass=0)
            # Body: 2 mm under the rims so only the rims touch the ramp; the pads grip it.
            b.add_geom(name=f"cyl_{lab}_body", type=props.G.mjGEOM_CYLINDER, size=[R - 0.002, HL - 0.008],
                       quat=q, rgba=[0.62, 0.64, 0.70, 1], friction=[1.2, 0.01, 0.0], condim=4, mass=0)
            mat = props.labelled_material(spec, f"mat_cyl_{lab}", lab, bg=(235, 235, 235))
            for s in (-1, 1):
                b.add_geom(type=props.G.mjGEOM_BOX, size=[0.0005, 0.013, 0.013],
                           pos=[s * (HL + 0.0005), 0, 0], material=mat, contype=0, conaffinity=0, mass=0)

    # ---------- runtime ----------
    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._m = model
        self.button = props.Button(BUTTON)
        self.button.bind(model)
        self.touch = props.TouchTracker(tuple(f"cyl_{lab}" for lab in self.labels) + ("gate_button",))
        self.touch.bind(model, data)
        self._tips = props.gripper_geoms(model)
        self._cg = {model.geom(f"cyl_{lab}_{g}").id: lab for lab in self.labels for g in ("rim-1", "rim+1", "body")}
        self._mocap = model.body("gate").mocapid[0]
        self._gate_closed = data.mocap_pos[self._mocap].copy()
        self._dof = {lab: model.joint(f"cyl_{lab}_free").dofadr[0] for lab in self.labels}
        self._body = {lab: model.body(f"cyl_{lab}").id for lab in self.labels}
        self.gate_open = False
        self.trials = []

    def pos(self, data: mujoco.MjData, lab: str) -> np.ndarray:
        return data.xpos[self._m.body(f"cyl_{lab}").id]

    def held(self, data: mujoco.MjData) -> set[str]:
        return {self._cg[g] for g in props.held_geoms(self._m, data, self._tips) if g in self._cg}

    def in_start(self, data: mujoco.MjData, lab: str) -> int | None:
        p = self.pos(data, lab)
        for i, x in enumerate(LANE_X):
            if abs(p[0] - x) < 0.025 and Y_GATE < p[1] < Y_GATE + 0.07:
                return i
        return None

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        t = float(data.time)
        self.touch.update(data)
        if self.button.update(data):
            self.gate_open = not self.gate_open
            data.mocap_pos[self._mocap] = self._gate_closed - [0, 0, GATE_DROP * self.gate_open]
            if self.gate_open:
                held = self.held(data)
                lanes = {lab: self.in_start(data, lab) for lab in self.labels if lab not in held}
                starters = tuple(lab for lab, ln in lanes.items() if ln is not None)
                if starters and len({lanes[s] for s in starters}) == len(starters):
                    self.trials.append(Trial(t, starters, lane_x={s: LANE_X[lanes[s]] for s in starters}))
        for lab in self.labels:
            data.xfrc_applied[self._body[lab]] = 0.0
        if not self.gate_open:  # start chute: squares a cylinder resting behind the closed gate
            held = self.held(data)
            for lab in self.labels:
                if lab in held or self.in_start(data, lab) is None:
                    continue
                d = self._dof[lab]
                if np.linalg.norm(data.qvel[d:d + 3]) > 0.01:
                    continue
                xm = data.xmat[self._body[lab]].reshape(3, 3)
                yaw = (float(np.arctan2(xm[1, 0], xm[0, 0])) + np.pi / 2) % np.pi - np.pi / 2
                if abs(yaw) > 0.02:
                    qa = model.jnt_qposadr[model.body_jntadr[self._body[lab]]]
                    rz = np.array([np.cos(-yaw / 2), 0.0, 0.0, np.sin(-yaw / 2)])
                    q = np.empty(4)
                    mujoco.mju_mulQuat(q, rz, data.qpos[qa + 3:qa + 7])
                    data.qpos[qa + 3:qa + 7] = q
                    data.qvel[d + 3:d + 6] = 0.0
        for tr in self.trials:
            if tr.done:
                continue
            for lab in tr.starters:  # lane groove: soft lateral spring + yaw damping
                b = self._body[lab]
                xm = data.xmat[b].reshape(3, 3)
                yaw = float(np.arctan2(xm[1, 0], xm[0, 0]))
                yaw = (yaw + np.pi / 2) % np.pi - np.pi / 2  # axis direction is sign-free
                vx = float(data.qvel[self._dof[lab]])  # free-joint linear velocity, world x
                data.xfrc_applied[b, 0] = -60.0 * (data.xpos[b][0] - tr.lane_x[lab]) - 3.0 * vx
                data.xfrc_applied[b, 5] = -0.05 * yaw
            for lab in tr.starters:
                if lab not in tr.cross and self.pos(data, lab)[1] < FINISH_Y:
                    tr.cross[lab] = t - tr.t0
            if len(tr.cross) == len(tr.starters) or t - tr.t0 > 5.0:
                tr.done = True
        # Foam pad: strongly damps anything rolling on it.
        for lab in self.labels:
            p = self.pos(data, lab)
            if FOAM_Y[1] < p[1] < FOAM_Y[0] and p[2] < R + 0.01:
                d = self._dof[lab]
                data.qvel[d:d + 6] *= 0.7

    # ---------- evidence ----------
    def slower_pairs(self) -> set[tuple[str, str]]:
        """(slower, faster) pairs shown by two-lane races or by timed solo runs of both."""
        out = set()
        for tr in self.trials:
            if len(tr.starters) == 2 and len(tr.cross) == 2:
                a, b = tr.starters
                if abs(tr.cross[a] - tr.cross[b]) > 0.02:
                    out.add((a, b) if tr.cross[a] > tr.cross[b] else (b, a))
        solo: dict[str, float] = {}
        for tr in self.trials:
            if len(tr.starters) == 1 and tr.cross:
                lab = tr.starters[0]
                solo[lab] = min(solo.get(lab, 9.0), tr.cross[lab])
        for a in solo:
            for b in solo:
                if solo[a] - solo[b] > 0.02:
                    out.add((a, b))
        return out

    def proven_hollow(self) -> str | None:
        """Easy: slower cylinder in a two-lane race, else from timed solo runs of both.
        Hard: the cylinder shown slower than each other one, directly or through a chain."""
        if self.difficulty == "hard":
            rel = self.slower_pairs()
            for _ in self.labels:  # transitive closure
                rel |= {(a, d) for a, b in rel for c, d in rel if b == c and a != d}
            for x in self.labels:
                if all((x, z) in rel for z in self.labels if z != x):
                    return x
            return None
        for tr in self.trials:
            if len(tr.starters) == 2 and len(tr.cross) == 2:
                a, b = tr.starters
                if abs(tr.cross[a] - tr.cross[b]) > 0.02:
                    return a if tr.cross[a] > tr.cross[b] else b
        solo = {}
        for tr in self.trials:
            if len(tr.starters) == 1 and tr.cross:
                lab = tr.starters[0]
                solo[lab] = min(solo.get(lab, 9.0), tr.cross[lab])
        if len(solo) == 2 and abs(solo["P"] - solo["Q"]) > 0.02:
            return max(solo, key=solo.__getitem__)
        return None

    def in_bin(self, data: mujoco.MjData) -> set[str]:
        held = self.held(data)
        return {lab for lab in self.labels if lab not in held
                and props.in_box(self.pos(data, lab), BIN_XY, BIN_HALF, z_max=0.06)}

    def hud(self, model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
        sc = self.score(model, data)
        lines = [f"t = {data.time:5.1f} s   gate {'OPEN' if self.gate_open else 'closed'}   "
                 f"bin {sorted(sc['in_bin']) or '-'}",
                 f"rubric stage {sc['stage']}/4   runs {sc['runs']}   slower (hollow?) "
                 f"{sc['proven_hollow'] or '?'}"]
        for tr in self.trials[-2:]:
            res = "  ".join(f"{lab} {tr.cross[lab]:.2f}s" if lab in tr.cross else f"{lab} --"
                            for lab in tr.starters)
            lines.append(f"  run {'+'.join(tr.starters)}: {res}")
        return lines

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        binned = self.in_bin(data)
        success = binned == {self.hollow}
        proven = self.proven_hollow()
        stage = 0
        if self.touch.touched or self.button.presses:
            stage = 1
        if any(tr.cross for tr in self.trials):
            stage = 2
        if proven == self.hollow:
            stage = 3
            if success:
                stage = 4
        return {
            "stage": stage,
            "success": float(success),
            "science_success": float(stage >= 3),
            "runs": len(self.trials),
            "races": sum(len(tr.starters) == 2 for tr in self.trials),
            "procedure_violations": sum(1 for tr in self.trials if not tr.cross),  # gate opened, nothing rolled
            "hollow": self.hollow,
            "proven_hollow": proven,
            "in_bin": sorted(binned),
            "log": [tr.__dict__ for tr in self.trials],
        }


# ---------- reference policies ----------
def _carry_locked(env, mo: Motion, body: str, side: str, center_target, *, dz: float = -0.007,  # noqa: ANN001
                  grip: float = 0.005, open_to: float = 0.035, yaws=(np.pi / 2, -np.pi / 2),
                  relay: bool = True, pick_yaws=None, tilts=TILTS):  # noqa: ANN001, ANN202
    """Pick/place with the fingers along world y (squeezing the curved surface) at both ends,
    so the cylinder axis stays along x and it rolls down the lane. Tilt about the finger axis
    tips the cylinder axis, so prefer small tilts (TILTS is ordered that way)."""
    m, d = env.model, env.data
    c0 = d.xpos[m.body(body).id].copy()
    grasp = c0 + [0, 0, dz]
    target = np.asarray(center_target, float)
    for arm in (side, "right" if side == "left" else "left"):
        pick = drop = None
        for tilt in tilts:  # same tilt at both ends keeps the cylinder axis level
            for y1 in (pick_yaws or yaws):
                r1 = grasp_rot(y1, tilt)
                if not mo._vertical_ok(arm, grasp, r1):
                    continue
                off = r1.T @ (c0 - grasp)
                for y2 in yaws:
                    rot = grasp_rot(y2, tilt)
                    lo = target - rot @ off
                    if mo._vertical_ok(arm, lo, rot):
                        pick, drop = (y1, tilt), (y2, tilt, lo, rot)
                        break
                if pick:
                    break
            if pick:
                break
        if pick is None:
            continue
        yield from mo.pick(arm, grasp, grip, body=body, plan=pick, open_to=open_to)
        y, tilt, lo, rot = drop
        if mo.held[arm] is not None:  # use the measured in-hand offset
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
    if relay:  # hand over through the empty cradle
        mid = np.array([RELAY_XY[0], RELAY_XY[1], R + 0.004 + 0.003])
        yield from _carry_locked(env, mo, body, side, mid, dz=dz, grip=grip, open_to=open_to, yaws=yaws,
                                 relay=False)
        yield from mo.hold(0.5)
        other = "right" if side == "left" else "left"
        yield from _carry_locked(env, mo, body, other, target, dz=dz, grip=grip, open_to=open_to, yaws=yaws,
                                 relay=False)
        return
    raise RuntimeError(f"no arm can carry {body} to {np.round(target, 3)}")


def _start_center(lane: int) -> np.ndarray:
    y = Y_GATE + 0.004 + R + 0.012
    return np.array([LANE_X[lane], y, ramp_z(y) + R / np.cos(THETA) + 0.003])


def _yaw(env, lab: str) -> float:
    b = env.model.body(f"cyl_{lab}").id
    xm = env.data.xmat[b].reshape(3, 3)
    return float((np.arctan2(xm[1, 0], xm[0, 0]) + np.pi / 2) % np.pi - np.pi / 2)


def _to_start(env, mo: Motion, lab: str, lane: int):  # noqa: ANN001, ANN202
    """Lay ``lab`` behind the gate in ``lane`` (the start chute squares it while the gate is shut)."""
    t: RollRace = env.task
    p = t.pos(env.data, lab)
    yield from _carry_locked(env, mo, f"cyl_{lab}", "right" if p[1] < -0.3 else "left", _start_center(lane))
    yield from mo.hold(0.5)


def _solo(env, mo: Motion, lab: str, lane: int = 0):  # noqa: ANN001, ANN202
    """One timed solo run of ``lab`` in ``lane``. Returns the time from gate opening to the
    finish line, or None."""
    t: RollRace = env.task
    if t.gate_open:
        yield from mo.press("left", t.button.top(), depth=BUTTON.travel + 0.004)
    yield from _to_start(env, mo, lab, lane)
    yield from mo.hold(0.5)
    n = len(t.trials)
    yield from mo.press("left", t.button.top(), depth=BUTTON.travel + 0.004)
    yield from mo.home("left")
    yield from mo.hold(4.0)
    tr = t.trials[n] if len(t.trials) > n else None
    return tr.cross.get(lab) if tr is not None else None


def _bin(env, mo: Motion, lab: str):  # noqa: ANN001, ANN202
    t: RollRace = env.task
    for _ in range(3):  # look after placing: retry if the cylinder did not land in the bin
        p = t.pos(env.data, lab)
        yield from _carry_locked(env, mo, f"cyl_{lab}", "right" if p[1] < -0.3 else "left",
                                 (BIN_XY[0], BIN_XY[1], 0.004 + R + 0.006))
        yield from mo.hold(1.5)
        if lab in t.in_bin(env.data):
            break


def _time_trials(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Hard: time a solo run of each of P, Q, R (P in lane 1, Q in lane 2, then P is parked
    and R runs in lane 1, so no run hits a resting cylinder); the slowest (by more than 0.02 s
    against each other) is the thin-walled tube. Reads positions and its own runs."""
    t: RollRace = env.task
    mo = Motion(env)
    times = {}
    for lab, lane in (("P", 0), ("Q", 1), ("R", 0)):
        if lab == "R":  # clear lane 1's run-out
            yield from _carry_locked(env, mo, "cyl_P", "right", (PARK_XY[0], PARK_XY[1], R + 0.004 + 0.003))
        tm = yield from _solo(env, mo, lab, lane)
        times[lab] = tm if tm is not None else 0.0
    slow = max(times, key=times.__getitem__)
    yield from _bin(env, mo, slow)


def oracle(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Easy: one race of P and Q. Hard: a timed solo run of each of P, Q, R in one lane."""
    pol = _time_trials if env.task.difficulty == "hard" else _one_race
    yield from pol(env, rng)


def _one_race(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Lay P and Q behind the gate in separate lanes, open the gate, watch which crosses the
    finish line last (the hollow one), and bin it. Reads only positions and its own race."""
    t: RollRace = env.task
    mo = Motion(env)
    for _race in range(3):  # re-run an inconclusive race (a cylinder veered off)
        if t.gate_open:
            yield from mo.press("left", t.button.top(), depth=BUTTON.travel + 0.004)
        order = LABELS
        try:
            for lane, lab in enumerate(order):
                p = t.pos(env.data, lab)
                yield from _carry_locked(env, mo, f"cyl_{lab}", "right" if p[1] < -0.3 else "left",
                                         _start_center(lane))
        except RuntimeError:  # a re-race needs a hand-over from the foam; give up re-racing
            if _race == 0:
                raise
            break
        yield from mo.hold(1.0)
        yield from mo.press("left", t.button.top(), depth=BUTTON.travel + 0.004)
        yield from mo.home("left")
        yield from mo.hold(4.0)
        if t.proven_hollow() is not None:
            break
    slow = t.proven_hollow()
    if slow is None:  # inconclusive race: fall back to the later finisher seen
        slow = LABELS[int(rng.integers(2))] if rng is not None else "P"
    for _ in range(3):  # look after placing: retry if the cylinder did not land in the bin
        p = t.pos(env.data, slow)
        yield from _carry_locked(env, mo, f"cyl_{slow}", "right" if p[1] < -0.3 else "left",
                                 (BIN_XY[0], BIN_XY[1], 0.004 + R + 0.006))
        yield from mo.hold(1.5)
        if slow in t.in_bin(env.data):
            break


def guess(env, rng: np.random.Generator | None = None):  # noqa: ANN001, ANN201
    """Blind baseline: pick a random cylinder and bin it without comparing the two. No arm
    reaches from the start row to the bin, so the cylinder is rolled down its lane alone
    (one solo run of one cylinder carries no comparison) and binned from the run-out."""
    rng = rng or np.random.default_rng()
    t: RollRace = env.task
    mo = Motion(env)
    lab = str(rng.choice(t.labels))
    p = t.pos(env.data, lab)
    yield from _carry_locked(env, mo, f"cyl_{lab}", "right" if p[1] < -0.3 else "left",
                             _start_center(t.labels.index(lab) % 2))
    yield from mo.hold(1.0)
    yield from mo.press("left", t.button.top(), depth=BUTTON.travel + 0.004)
    yield from mo.home("left")
    yield from mo.hold(4.0)
    p = t.pos(env.data, lab)
    yield from _carry_locked(env, mo, f"cyl_{lab}", "right" if p[1] < -0.3 else "left",
                             (BIN_XY[0], BIN_XY[1], 0.004 + R + 0.006))
    yield from mo.hold(1.5)


TASK = RollRace
POLICIES = {"oracle": oracle, "guess": guess, "noop": noop, "naive": _one_race}
SPEC = TaskSpec(
    key="roll_race",
    title="Find the hollow cylinder with a roll race",
    ability="Comparative measurement",
    instruction=INSTRUCTION,
    objects=(
        "two cylinders P and Q (hard: three, P, Q, R), 5 cm diameter x 6 cm long, 150 g each, identical "
        "outside; easy: one thin-walled tube with end caps and one solid; hard: thin-walled, thick-walled "
        "and solid",
        "two-lane ramp, 20 cm long at 14 deg, with a start gate across both lanes opened by an orange "
        "push button",
        "finish line 21 cm past the ramp foot, then a foam pad that stops the cylinders",
        "red bin, 11 x 11 cm",
    ),
    setup=(
        "Ramp centred 31 cm in front of the arms, top toward the left arm, rolling toward the right arm.",
        "Close the gate. Gate button in front of the ramp top.",
        "Lay P and Q (hard: and R, in a third cradle nearer the arms' base line) on their sides (axis left-right) "
        "on the table in front of the arms.",
        "Red bin to the right of the ramp, 48 cm in front of the arms.",
    ),
    hidden=(
        "Which cylinder is hollow (random per episode): thin-walled (I = m r^2) vs solid (I = m r^2 / 2)."
    ),
    instruction_hard=(
        "Cylinders P, Q and R look the same and weigh the same. One is a thin-walled tube, one a "
        "thick-walled tube and one solid. Put the thin-walled tube in the red bin. The orange button "
        "opens and closes the start gate at the top of the ramp."
    ),
    hidden_hard=(
        "Which cylinder is which, a random permutation of I = 1.0, 0.7 and 0.5 m r^2. The ramp has two "
        "lanes, so one race ranks only two cylinders: the thin-walled tube must be shown slower than "
        "both others (e.g. a second race of the first race's loser against the third cylinder). A "
        "single race of P and Q misses the tube in 1/3 of episodes and never proves it. Timing a "
        "solo run of each cylinder in the same lane also works (the reference does this)."
    ),
    rubric=(
        "No purposeful contact with the cylinders or the gate button",
        "Touches a cylinder or presses the gate button",
        "At least one run: a cylinder released through the gate crosses the finish line",
        "Runs identify the hollow cylinder: a two-lane race (or a timed solo run of each) where it is "
        "slower; hard: the thin-walled tube shown slower than both others, directly or through a chain",
        "Stage 3 and only the hollow cylinder is in the bin",
    ),
    metrics=(
        "stage (0-4)",
        "success: only the hollow cylinder in the bin, proof or not",
        "science_success: stage >= 3",
        "runs and two-lane races used",
        "procedure violations: gate opened with nothing rolling",
    ),
    report_fields=("hollow", "proven_hollow", "in_bin", "runs", "races"),
    max_seconds=240.0,  # hard: three solo runs and a parking carry
    limitations=("Cylinder inertia is set directly (both bodies use the same collision rims). The "
                 "foam pad is a velocity damper applied in code. The start gate is a kinematic bar. A cylinder "
                 "resting behind the closed gate is squared to the lane in code (a start chute). The lane groove that keeps a racing "
                 "cylinder straight is a soft lateral spring and yaw torque applied in code."),
)
