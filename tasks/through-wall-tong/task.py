"""A mock hot cell on the station table: a closed plywood box with an acrylic window and roof. A steel tong passes through a gimbal in the front wall, so that its jaws move opposite to its handle. Inside, on a bench, a vial stands in a blue rack and a tight grey holder stands beside it. Outside, a sprung red lever on a stand closes the jaws through a cable.

The arms cannot reach into the box. One arm holds the lever closed through the carry, and the other moves the handle.

Debug:  python demokit.py task.py OUT --no-record --size 320x176 --stills 40 --mode full,none,...
Final:  python demokit.py task.py OUT
Model:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1400 --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import dataclasses
import math
import os
import pathlib
import re
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "Use the tong through the wall to move the vial from the blue rack into the grey holder, and leave it standing there with the jaws clear of it. Pull the red lever to close the jaws; they open when it is let go."
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
TOP = d.TABLE_TOP

LOOKS = {
    "ply": m.Look(m.rgb(0.78, 0.63, 0.43), roughness=0.8),
    "bench": m.Look(m.rgb(0.90, 0.88, 0.84), roughness=0.8),
    "acrylic": m.Look(glass=(0.97, 0.99, 1.0)),
    "steel": m.Look(m.rgb(0.62, 0.64, 0.67), metallic=0.8, roughness=0.35),
    "rubber": m.Look(m.rgb(0.07, 0.07, 0.08), roughness=0.9),
    "jaw": m.Look(m.rgb(0.25, 0.27, 0.30), metallic=0.5, roughness=0.5),
    "pad": m.Look(m.rgb(0.95, 0.45, 0.10), roughness=0.8),
    "vial": m.Look(m.rgb(0.90, 0.55, 0.08), roughness=0.4),
    "cap": m.Look(m.rgb(0.80, 0.10, 0.10), roughness=0.5),
    "holder": m.Look(m.rgb(0.55, 0.57, 0.60), metallic=0.3, roughness=0.5),
    "rack": m.Look(m.rgb(0.10, 0.35, 0.95), roughness=0.5),
    "lever": m.Look(m.rgb(0.85, 0.08, 0.06), roughness=0.5),
    "stand": m.Look(m.rgb(0.20, 0.21, 0.23), roughness=0.7),
}

# All heights are above the table. The cell's box:
WALL_X, WALL_T = 0.411, 0.012
BACK_X, SIDE_Y, ROOF_Z = 0.862, 0.270, 0.460  # the roof's front edge stays above the line from the top camera to the vial
SILL_Z = 0.190  # plywood below, acrylic window above
BENCH_X, BENCH_Y, BENCH_Z = (0.50, 0.85), 0.25, 0.060
PORT = 0.030  # half the square port's side

PIVOT = np.array([WALL_X, 0.0, TOP + 0.120])
HANDLE_X = -0.280  # along the rod from the pivot, with the slide at 0
PAD_X = 0.240
PAD_D, PAD_H = 0.030, 0.024  # along the rod, and tall
OPEN_GAP = 0.030  # each pad's inner face from the rod's axis, open
SLIDE = (-0.03, 0.10)
CLOSED_AT = 24.0  # the lever's pull, degrees, for the cable's whole travel
JAW_K, JAW_F = 320.0, 8.0

VIAL_R, VIAL_H, VIAL_MASS = 0.019, 0.085, 0.05
GRIP_UP = 0.060  # where the pads meet the vial, above its base
RACK_XY = (0.70, -0.09)
HOLDER_XY = (0.70, 0.09)
RACK_R, RACK_H = 0.022, 0.015
HOLDER_R, HOLDER_H = 0.024, 0.040

LEVER_AT = (0.30, 0.30, TOP)
LEVER_PIVOT_Z, LEVER_L = 0.035, 0.115
LEVER_MAX = 28.0
PULL = 26.0
BALL_Z = LEVER_PIVOT_Z + LEVER_L

CLEAR = 0.03
FALL_SPEED = 0.65  # m/s downward: a free fall of a little over 2 cm


def _slab(name: str, x, y, z, look: str):
    """A box between these limits: its mesh, and its collider."""
    size = (x[1] - x[0], y[1] - y[0], z[1] - z[0])
    c = (sum(x) / 2, sum(y) / 2, sum(z) / 2)
    return (name, m.box(size, c), look), m.col_box(f"{name}_c", size, c)


def _ring_colliders(name: str, r_in: float, r_out: float, h: float, n: int = 12) -> list[str]:
    """A bore's wall, as `n` straight segments whose inner faces stand `r_in` from its axis."""
    w = 2.0 * r_in * math.tan(math.pi / n) + 0.002
    t = r_out - r_in
    return [m.col_hull(f"{name}_{k}", m.box((t, w, h), (r_in + t / 2, 0.0, h / 2)).turned(360.0 * k / n, "z")) for k in range(n)]


def _ring_mesh(r_in: float, r_out: float, h: float) -> m.Mesh:
    return m.lathe([(r_in, 0.0), (r_out, 0.0), (r_out, h), (r_in, h), (r_in, 0.0)], 40)


def _cell() -> m.Part:
    wx = (WALL_X - WALL_T / 2, WALL_X + WALL_T / 2)
    pz = PIVOT[2] - TOP
    slabs = [
        _slab("wall_low", wx, (-SIDE_Y, SIDE_Y), (0.0, pz - PORT), "ply"),
        _slab("wall_top", wx, (-SIDE_Y, SIDE_Y), (pz + PORT, SILL_Z), "ply"),
        _slab("wall_a", wx, (PORT, SIDE_Y), (pz - PORT, pz + PORT), "ply"),
        _slab("wall_b", wx, (-SIDE_Y, -PORT), (pz - PORT, pz + PORT), "ply"),
        _slab("window", wx, (-SIDE_Y, SIDE_Y), (SILL_Z, ROOF_Z), "acrylic"),
        _slab("side_a", (wx[0], BACK_X), (SIDE_Y, SIDE_Y + WALL_T), (0.0, ROOF_Z), "ply"),
        _slab("side_b", (wx[0], BACK_X), (-SIDE_Y - WALL_T, -SIDE_Y), (0.0, ROOF_Z), "ply"),
        _slab("back", (BACK_X, BACK_X + WALL_T), (-SIDE_Y - WALL_T, SIDE_Y + WALL_T), (0.0, ROOF_Z), "ply"),
        _slab("roof", (wx[0], BACK_X + WALL_T), (-SIDE_Y - WALL_T, SIDE_Y + WALL_T), (ROOF_Z, ROOF_Z + 0.006), "acrylic"),
        _slab("bench", BENCH_X, (-BENCH_Y, BENCH_Y), (0.0, BENCH_Z), "bench"),
    ]
    rim = [
        ("rim_a", m.box((BACK_X + WALL_T - wx[0], 0.012, 0.012), ((BACK_X + WALL_T + wx[0]) / 2, SIDE_Y + WALL_T / 2, ROOF_Z + 0.003)), "ply"),
        ("rim_b", m.box((BACK_X + WALL_T - wx[0], 0.012, 0.012), ((BACK_X + WALL_T + wx[0]) / 2, -SIDE_Y - WALL_T / 2, ROOF_Z + 0.003)), "ply"),
        ("rim_c", m.box((0.012, 2 * SIDE_Y, 0.012), (WALL_X, 0.0, ROOF_Z + 0.003)), "ply"),
        ("rim_d", m.box((0.012, 2 * SIDE_Y, 0.012), (WALL_X, 0.0, SILL_Z)), "ply"),
    ]
    cable = m.tube([(0.30, 0.272, 0.012), (0.33, 0.20, 0.008), (0.37, 0.12, 0.008), (0.395, 0.07, 0.04), (WALL_X - WALL_T / 2, 0.05, 0.08)], 0.004)
    part = m.fixed("cell", (0.0, 0.0, TOP), [s[0] for s in slabs] + rim + [("cable", cable, "rubber")], [s[1] for s in slabs], LOOKS)
    # The box must not put its inside in the dark: a real cell has its own lamps, and this station's lights stay as they are.
    build = part.build
    no_shadow = re.compile(r'(def Mesh "(?:wall_low|wall_top|wall_a|wall_b|window|side_a|side_b|back|roof|rim_[abcd])" \(\s*prepend apiSchemas = \["MaterialBindingAPI"\]\s*\)\s*\{)')
    return dataclasses.replace(part, build=lambda: no_shadow.sub(r'\1\n            bool primvars:doNotCastShadows = 1', build()))


def _tong() -> m.Part:
    rod_parts = [
        ("tube", m.cylinder(0.008, 0.54, ((HANDLE_X + PAD_X - 0.03) / 2, 0.0, 0.0), axis="x"), "steel"),
        ("grip", m.cylinder(0.015, 0.12, (HANDLE_X, 0.0, 0.0), axis="x"), "rubber"),
        ("butt", m.sphere(0.018, (HANDLE_X - 0.065, 0.0, 0.0)), "rubber"),
        ("palm", m.box((0.012, 0.076, 0.020), (PAD_X - 0.030, 0.0, 0.0)), "jaw"),
    ]
    rod_cols = [
        m.col_box("grip_c", (0.12, 0.03, 0.03), (HANDLE_X, 0.0, 0.0)),
        m.col_box("tube_c", (0.45, 0.016, 0.016), ((HANDLE_X + 0.06 + PAD_X - 0.036) / 2, 0.0, 0.0)),
        m.col_box("palm_c", (0.012, 0.076, 0.020), (PAD_X - 0.030, 0.0, 0.0)),
    ]

    def jaw(name: str, sign: float, low: float, high: float) -> m.Link:
        finger = ((PAD_X - 0.036 + PAD_X + PAD_D / 2) / 2, sign * (OPEN_GAP + 0.005), 0.0)
        size = (PAD_X + PAD_D / 2 - PAD_X + 0.036, 0.006, PAD_H)
        pad = (PAD_X, sign * (OPEN_GAP + 0.001), 0.0)
        return m.slider(name, "Y", low, high, [("finger", m.box(size, finger), "jaw"), ("pad", m.box((PAD_D, 0.002, PAD_H), pad), "pad")],
                        [m.col_box(f"{name}_c", (size[0], 0.008, PAD_H), (finger[0], sign * (OPEN_GAP + 0.004), 0.0))],
                        parent="rod", mass=0.02, spring=JAW_K, damping=15.0, effort=JAW_F, friction=0.0, armature=0.001)

    links = [
        m.hinge("yaw", "Z", (0.0, 0.0, 0.0), -32.0, 32.0, (), (), mass=0.05, damping=0.2, friction=0.05, armature=0.002),
        m.hinge("pitch", "Y", (0.0, 0.0, 0.0), -22.0, 22.0, [("ball", m.sphere(0.03), "steel")], (), mass=0.05, damping=0.2, friction=0.05, armature=0.002, parent="yaw"),
        m.slider("rod", "X", SLIDE[0], SLIDE[1], rod_parts, rod_cols, mass=0.20, damping=2.0, friction=0.5, armature=0.002, parent="pitch"),
        jaw("jaw_l", 1.0, -0.028, 0.0),
        jaw("jaw_r", -1.0, 0.0, 0.028),
    ]
    flange = m.lathe([(0.032, -0.009), (0.06, -0.009), (0.06, 0.009), (0.032, 0.009), (0.032, -0.009)], 40).aimed("+X")
    return m.mechanism("tong", tuple(PIVOT), links, {k: LOOKS[k] for k in ("steel", "rubber", "jaw", "pad")}, frame_parts=[("flange", flange, "steel")])


OBJECTS = [
    _cell(),
    _tong(),
    m.fixed("rack", (*RACK_XY, TOP + BENCH_Z), [("ring", _ring_mesh(RACK_R, RACK_R + 0.012, RACK_H), "rack")], _ring_colliders("r", RACK_R, RACK_R + 0.012, RACK_H), LOOKS),
    m.fixed("holder", (*HOLDER_XY, TOP + BENCH_Z), [("ring", _ring_mesh(HOLDER_R, HOLDER_R + 0.018, HOLDER_H), "holder")], _ring_colliders("h", HOLDER_R, HOLDER_R + 0.018, HOLDER_H), LOOKS),
    m.mechanism("lever_unit", LEVER_AT, [m.lever("arm", (0.0, 0.0, LEVER_PIVOT_Z), (0, 0, 1), axis="Y", look="lever", length=LEVER_L, lower=-LEVER_MAX, upper=0.0, spring=2.0, damping=0.1, friction=0.01)], {k: LOOKS[k] for k in ("lever", "stand")},
                frame_parts=[("stand", m.box((0.05, 0.05, LEVER_PIVOT_Z), (0, 0, LEVER_PIVOT_Z / 2)), "stand")], frame_colliders=[m.col_box("stand_c", (0.05, 0.05, LEVER_PIVOT_Z), (0, 0, LEVER_PIVOT_Z / 2))]),
    m.free("vial", (*RACK_XY, TOP + BENCH_Z + 0.001), [("body", m.cylinder(VIAL_R, 0.070, (0, 0, 0.035)), "vial"), ("cap", m.cylinder(VIAL_R - 0.002, 0.015, (0, 0, 0.0775)), "cap")],
           [m.col_cylinder("c", VIAL_R, VIAL_H, (0, 0, VIAL_H / 2))], {k: LOOKS[k] for k in ("vial", "cap")}, mass=VIAL_MASS, friction=(0.9, 0.8)),
]


def rod_rotation(yaw: float, pitch: float) -> np.ndarray:
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    return np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]]) @ np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])


def judge(failed, off_axis: float, height: float, tilt: float, still: bool, jaw_gaps) -> tuple[bool, float, bool, bool]:
    """The score now, from what a tape measure and a video show: (full, score, in the holder, held in the jaws). Run on hand-built states by check_score.py."""
    in_holder = bool(off_axis <= HOLDER_R and abs(height) <= 0.004 and tilt <= 5.0)
    held = bool(height >= 0.03 and max(jaw_gaps) <= 0.004)  # both jaws on it
    full = bool(failed is None and in_holder and still and min(jaw_gaps) >= CLEAR)
    return full, 0.0 if failed else (0.5 if in_holder else (0.25 if held else 0.0)), in_holder, held


class Trial:
    """What the scorer holds across a trial."""

    def reset(self, env=None) -> None:
        self.failed = None
        self.full, self.score = False, 0.0
        self.best = "nothing"
        self.steps = 0
        if env is not None:
            self.step(env)

    def step(self, env) -> None:
        self.steps += 1
        if os.environ.get("TASK_STEP_FILE"):  # the simulated time, for anything following the run from outside
            with open(os.environ["TASK_STEP_FILE"], "w") as f:
                f.write(f"{self.steps * d.DT:.2f}")
        # The cable: the lever's pull sets where the jaws would close to.
        pull = float(np.clip(-m.state(env, "lever_unit").deg("arm") / CLOSED_AT, 0.0, 1.0))
        m.set_target(env, "tong", "jaw_l", -OPEN_GAP * pull)
        m.set_target(env, "tong", "jaw_r", OPEN_GAP * pull)
        tong, vial = m.state(env, "tong"), m.state(env, "vial")
        R = rod_rotation(tong.joint("yaw"), tong.joint("pitch"))
        s = tong.joint("rod")
        axis = vial.rotation()[:, 2]
        jaw_gaps = []
        for sign, q in ((1.0, tong.joint("jaw_l")), (-1.0, tong.joint("jaw_r"))):
            gaps = []
            for x in (-PAD_D / 2, 0.0, PAD_D / 2):
                rel = PIVOT + R @ np.array([PAD_X + s + x, sign * OPEN_GAP + q, 0.0]) - vial.pos
                gaps.append(float(np.linalg.norm(rel - np.clip(rel @ axis, 0.0, VIAL_H) * axis)) - VIAL_R)
            jaw_gaps.append(min(gaps))
        gap = min(jaw_gaps)
        bench = TOP + BENCH_Z
        height = float(vial.pos[2] - bench)
        if self.failed is None:
            if vial.tilt_deg() > 45.0:
                self.failed = "tipped"
            elif height < -0.01:
                self.failed = "off the bench"
            elif vial.lin_vel[2] < -FALL_SPEED:
                self.failed = "fell"
            if self.failed:
                print(f"[tong] failed for good: the vial {self.failed}", flush=True)
        off_axis = float(np.hypot(*(vial.pos[:2] - np.array(HOLDER_XY))))
        self.full, self.score, in_holder, held = judge(self.failed, off_axis, height, vial.tilt_deg(), vial.resting(), jaw_gaps)
        state = "full" if self.full else ("in the holder" if in_holder else ("held up" if held else "nothing"))
        if state != self.best:
            self.best = state
            print(f"[tong] now: {state}; base {height:.3f} above the bench, tilt {vial.tilt_deg():.1f} deg, jaws {gap * 100:.1f} cm from it", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """The vial stands in the holder, with both jaws at least 3 cm from it, and it never tipped past 45 degrees, fell off the bench or was dropped more than about 2 cm."""
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


def distance(objects) -> float:
    return float(np.linalg.norm(objects["vial"].pos[:2] - np.array(HOLDER_XY)))


# -- the demo: reads the rod's three joints and the vial's pose, as a policy could see them ---------------------------------------


R_DOWN = d.down(0.0).rot


def locate(goal, point) -> np.ndarray:
    """The rod's yaw, pitch and slide that bring `point`, fixed in the rod at slide 0, to the world point `goal`."""
    g = np.asarray(goal, float) - PIVOT
    bx, by, bz = point
    x = math.sqrt(max(float(g @ g) - by * by - bz * bz, 1e-9))
    rho = math.hypot(x, bz)
    pitch = math.acos(float(np.clip(g[2] / rho, -1.0, 1.0))) - math.atan2(x, bz)
    u = x * math.cos(pitch) + bz * math.sin(pitch)
    yaw = math.atan2(g[1], g[0]) - math.atan2(by, u)
    return np.array([yaw, pitch, x - bx])


class Handle:
    """The arm that holds the tong's handle: it commands the rod's pose, and servoes out what it sees of droop and slip, from the rod's joints."""

    def __init__(self, ctx):
        self.ctx, self.arm = ctx, ctx.right
        self.want = self.seen()
        self.trim = self.arm.trim.copy()

    def seen(self) -> np.ndarray:
        st = self.ctx.state("tong")
        return np.array([st.joint("yaw"), st.joint("pitch"), st.joint("rod")])

    @staticmethod
    def tcp(q) -> np.ndarray:
        return PIVOT + rod_rotation(q[0], q[1]) @ np.array([HANDLE_X + q[2], 0.0, 0.0])

    def command(self) -> None:
        self.trim = self.trim + np.clip(0.1 * (self.tcp(self.want) - self.tcp(self.seen())), -0.002, 0.002)
        self.arm._solve(self.tcp(self.want) + self.trim, rod_rotation(self.want[0], self.want[1]) @ R_DOWN)

    def go(self, goal, t: float):
        """Move the rod's joints to `goal`, smoothly."""
        start = self.want.copy()
        n = max(1, round(t * d.CONTROL_HZ))
        for k in range(1, n + 1):
            self.want = start + (np.asarray(goal) - start) * d.ik.smoothstep(k / n)
            self.command()
            yield

    def carry(self, goal, point, t: float):
        """Move `point`, fixed in the rod, in a straight line to the world point `goal`."""
        q = self.want
        start = PIVOT + rod_rotation(q[0], q[1]) @ (np.asarray(point) + np.array([q[2], 0.0, 0.0]))
        n = max(1, round(t * d.CONTROL_HZ))
        for k in range(1, n + 1):
            self.want = locate(start + (np.asarray(goal) - start) * d.ik.smoothstep(k / n), point)
            self.command()
            yield

    def settle(self, tol: float = 0.001, timeout: float = 1.0):
        good = 0
        for _ in range(round(timeout * d.CONTROL_HZ)):
            good = good + 1 if np.linalg.norm(self.tcp(self.want) - self.tcp(self.seen())) < tol else 0
            if good >= 3:
                return
            self.command()
            yield

    def held_point(self) -> np.ndarray:
        """Where the vial's base is in the rod, at slide 0, seen now."""
        q = self.seen()
        return rod_rotation(q[0], q[1]).T @ (self.ctx.pos("vial") - PIVOT) - np.array([q[2], 0.0, 0.0])


MODES = ("full", "none", "left_only", "right_only", "lifted", "hold", "near", "drop", "outside", "rack", "slide", "naive", "wrong_way", "miss10", "miss20", "reach")


def demo_script(ctx):
    """Modes: full. none: the arms wait. left_only: the lever is pulled and held, with the handle left alone. right_only: the whole carry, with the lever never pulled. lifted: stops holding the vial up. hold: stops with it in the holder, jaws closed. near: lets go in the holder, with the jaws left round it. drop: lets go over the holder. outside: stands it on the bench. rack: puts it back. slide: drags it along the bench, without lifting. naive: moves the handle the way the jaws should go, unreversed, to take the vial. wrong_way: takes the vial, then carries it with the handle moving towards the holder, unreversed. miss10, miss20: lowers it 1 or 2 cm off the holder's axis, as a poor aim does. reach: an arm goes for the vial itself, through the wall."""
    mode = ctx.mode
    if mode not in MODES:
        raise ValueError(f"unknown mode {mode!r}")
    l, r = ctx.left, ctx.right
    if mode == "none":
        yield from l.wait(3.0)
        return
    if mode == "reach":
        # The straight line from the arm's base to the vial, fingers first: the longest reach it has. The wall stands in the way.
        goal = ctx.pos("vial") + np.array([0.0, 0.0, GRIP_UP])
        z = goal - r.base
        z = z / np.linalg.norm(z)
        x = np.cross((0.0, 0.0, 1.0), z)
        x = x / np.linalg.norm(x)
        grip = d._as_grip(np.stack([x, np.cross(z, x), z], axis=1))
        yield from r.move(r.base + 0.45 * (goal - r.base) + np.array([0.0, 0.0, 0.05]), grip=grip, t=4.0, label="towards the vial")
        yield from r.move(goal, t=4.0, label="reach for the vial")
        tips = m.fingertips(ctx.emb._env)[12:]
        print(f"[tong] reach: commanded x {goal[0]:.3f}; TCP x {r.tcp()[0]:.3f}; furthest fingertip x {tips[:, 0].max():.3f}; wall's face at x {WALL_X - WALL_T / 2:.3f}; vial moved {np.linalg.norm(ctx.pos('vial') - goal + [0.0, 0.0, GRIP_UP]) * 1000:.1f} mm", flush=True)
        yield from r.wait(1.0)
        return
    ball = ctx.pos("lever_unit") + np.array([0.0, 0.0, BALL_Z])
    lever_pivot = ctx.pos("lever_unit") + np.array([0.0, 0.0, LEVER_PIVOT_Z])
    handle = PIVOT + np.array([HANDLE_X, 0.0, 0.0])
    use_lever = mode != "right_only"
    use_handle = mode != "left_only"

    def to_lever():
        yield from l.move(ball + [0.0, 0.0, 0.05], grip=d.down(0.0), finger=d.FINGER_OPEN, label="over the lever")
        yield from l.move(ball, t=0.8, label="on the lever")
        yield from l.servo(ball, timeout=0.6, label="on the lever")
        yield from l.close(0.032, force=15.0, label="hold the lever")

    def to_handle():
        yield from r.move(handle + [0.0, 0.0, 0.05], grip=d.down(0.0), finger=d.FINGER_OPEN, label="over the handle")
        yield from r.move(handle, t=0.8, label="on the handle")
        yield from r.servo(handle, timeout=0.6, label="on the handle")
        yield from r.close(0.03, force=20.0, label="grip the handle")

    def pull():
        yield from l.arc(lever_pivot, (0.0, 1.0, 0.0), -PULL, t=1.0, label="pull the lever")
        a = math.radians(PULL)
        yield from l.servo(lever_pivot + np.array([-LEVER_L * math.sin(a), 0.0, LEVER_L * math.cos(a)]), timeout=1.0, label="hold the lever")

    def release():
        yield from l.arc(lever_pivot, (0.0, 1.0, 0.0), PULL, t=1.0, label="let the lever back")
        yield from l.wait(0.3)

    yield from ctx.together(*([to_lever()] if use_lever else []), *([to_handle()] if use_handle else []))
    if not use_handle:
        yield from pull()
        print(f"[tong] left_only: lever at {ctx.deg('lever_unit', 'arm'):.1f} deg", flush=True)
        yield from l.wait(3.0)
        return
    h = Handle(ctx)
    pad = np.array([PAD_X, 0.0, 0.0])
    vial0 = ctx.pos("vial")
    grip_at = vial0 + np.array([0.0, 0.0, GRIP_UP])
    aim = locate(grip_at, pad)
    if mode == "naive":
        yield from h.go([-aim[0], 0.0, 0.0], 2.5)
        yield from h.go([-aim[0], 0.0, aim[2]], 1.5)
    else:
        yield from h.go([aim[0], 0.0, 0.0], 2.5)
        yield from h.carry(grip_at, pad, 1.5)
    yield from h.settle()
    if use_lever:
        yield from pull()
    print(f"[tong] closed on it: yaw {math.degrees(h.seen()[0]):.1f} deg, slide {h.seen()[2]:.3f}, lever {ctx.deg('lever_unit', 'arm'):.1f} deg, jaws {ctx.state('tong').joint('jaw_l'):.4f}", flush=True)
    held = h.held_point()
    bench = TOP + BENCH_Z
    place = np.array([*HOLDER_XY, bench])

    def over(xy, up: float = HOLDER_H + 0.012) -> np.ndarray:
        return np.array([xy[0], xy[1], bench + up])

    if mode == "slide":
        yield from h.carry(np.array([*HOLDER_XY, vial0[2]]), held, 4.0)
        yield from h.settle()
        yield from release()
        yield from h.go(h.want + [0.0, 0.0, -0.08], 1.5)
        return
    yield from h.carry(vial0 + np.array([0.0, 0.0, HOLDER_H + 0.012]), held, 1.5)
    yield from h.settle()
    print(f"[tong] lifted: vial {np.round(ctx.pos('vial') - [0.0, 0.0, bench], 3)}, tilt {ctx.state('vial').tilt_deg():.1f} deg", flush=True)
    if mode == "lifted":
        yield from h.go(h.want, 2.0)
        return
    if mode == "rack":
        yield from h.carry(vial0 + np.array([0.0, 0.0, 0.001]), held, 1.5)
    elif mode in ("outside", "wrong_way"):
        y = 0.0 if mode == "outside" else -0.20
        yield from h.carry(over((RACK_XY[0], y)), held, 2.0)
        yield from h.carry(np.array([RACK_XY[0], y, bench + 0.001]), held, 1.5)
    elif mode in ("miss10", "miss20"):
        off = np.array([0.0, 0.01 if mode == "miss10" else 0.02, 0.0])
        yield from h.carry(over(HOLDER_XY) + off, held, 2.5)
        yield from h.settle()
        yield from h.carry(place + off + np.array([0.0, 0.0, 0.002]), held, 2.0)
    else:
        yield from h.carry(over(HOLDER_XY), held, 2.5)
        yield from h.settle()
        held = h.held_point()
        print(f"[tong] over the holder: vial off its axis by {np.round((ctx.pos('vial') - place)[:2] * 1000, 1)} mm, tilt {ctx.state('vial').tilt_deg():.1f} deg", flush=True)
        if mode == "drop":
            yield from release()
            yield from h.go(h.want + [0.0, 0.0, -0.08], 1.5)
            return
        yield from h.carry(place + np.array([0.0, 0.0, 0.002]), held, 2.0)
    yield from h.settle()
    if mode == "hold":
        yield from h.go(h.want, 2.0)
        return
    yield from release()
    yield from l.open()
    yield from l.move(l.at() + [0.0, 0.0, 0.06], t=0.8, label="off the lever")
    if mode == "near":
        yield from h.go(h.want, 2.0)
        return
    yield from h.go(h.want + [0.0, 0.0, -0.09], 1.5)
    yield from h.settle()
    print(f"[tong] backed off: slide {h.seen()[2]:.3f}", flush=True)
    yield from r.open()
    yield from r.move(r.at() + [0.0, 0.0, 0.06], t=0.8, label="off the handle")
