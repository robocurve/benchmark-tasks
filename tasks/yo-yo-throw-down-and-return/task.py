"""A classic Duncan Butterfly yo-yo (plastic, fixed axle, 57 mm across, 32 mm wide, 49 g) sits wound up, upright in a
small cradle on top of a 23 cm pine riser block in front of the right arm, between the right gripper's open fingers.
Its cotton-poly string, cut to 40 cm, is wound on its axle; the string's slip loop is already around the tip of one
of the right gripper's fingers. On the table, clear of the arms: a coffee mug, a pair of glasses and an apple.
"""

import math
import pathlib
import sys
from dataclasses import replace

import numpy as np

from yam_arena import station
from yam_arena.task_file import TABLE_TOP

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402  importing it hooks yam_arena
import yoyo_model as Y  # noqa: E402
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # 0.7475

# -- the camera: at the front right, looking back at the right arm, so the yo-yo's whole up-and-down column shows
CAMERA_POS = (0.92, -0.66, 1.34)
CAMERA_TARGET = (0.22, -0.14, 1.00)
_base_rig = getattr(station.rig, "_yoyo_base", station.rig)


def _yoyo_rig(top_cam="calibrated"):
    rig = _base_rig(top_cam)
    forward = np.asarray(CAMERA_TARGET, float) - np.asarray(CAMERA_POS, float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, (0.0, 0.0, 1.0))
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    rot = np.column_stack((right, up, -forward))
    camera = replace(rig.top_cam, pos=CAMERA_POS, rot_xyzw=tuple(float(v) for v in station.matrix_to_quat_xyzw(rot)), convention="opengl")
    return replace(rig, name="station-yoyo", top_cam=camera)


_yoyo_rig._yoyo_base = _base_rig
station.rig = _yoyo_rig

# -- the yo-yo ----------------------------------------------------------------------------------------------------
R, W = Y.R_OUT, Y.WIDTH
N_STR = 40  # string drawn as 1 cm pieces from the axle up, shown as far as the string runs
LOOP_TIP = 14  # mechkit.fingertips index: the right arm's first finger, the middle of its tip: where the slip loop sits


def _half(sign):
    """One butterfly half, its axis along y: flat outer face, flared inner face (the wide 'butterfly' gap)."""
    prof = [(0.0, 0.0012), (0.0042, 0.0012), (0.0060, 0.0030), (R * 0.93, 0.0095), (R, 0.0120), (R, 0.0146), (R * 0.93, W / 2), (0.0, W / 2)]
    mesh = m.lathe(prof, segments=40).turned(-90.0, "x")  # +z (the lathe axis) to +y
    if sign < 0:
        mesh = mesh.turned(180.0, "z")
    return mesh


def _yoyo_parts():
    parts = [
        ("half_p", _half(1), "body"),
        ("half_n", _half(-1), "body"),
        ("cap_p", m.cylinder(0.011, 0.0012, (0.0, W / 2 + 0.0006, 0.0), axis="y"), "cap"),
        ("cap_n", m.cylinder(0.011, 0.0012, (0.0, -W / 2 - 0.0006, 0.0), axis="y"), "cap"),
        ("axle", m.cylinder(0.0030, 0.0030, (0.0, 0.0, 0.0), axis="y"), "axle"),
        ("wound", m.cylinder(Y.R_EFF, 0.0022, (0.0, 0.0, 0.0), axis="y"), "string"),
    ]
    for i in range(N_STR):
        parts.append((f"str{i}", m.box((0.0012, 0.0012, 0.0102), (0.0, 0.0, 0.005 + 0.01 * i)), "string", True))
    return parts


YOYO_LOOKS = {
    "body": m.Look(m.rgb(0.80, 0.08, 0.10), roughness=0.35),
    "cap": m.Look(m.rgb(0.95, 0.95, 0.92), roughness=0.5),
    "axle": m.Look(m.rgb(0.55, 0.55, 0.57), metallic=0.6, roughness=0.4),
    "string": m.Look(m.rgb(0.96, 0.95, 0.90), roughness=0.9),
}

# the riser and cradle: the yo-yo's centre starts between the right gripper's open fingers at its ready pose
START = np.array([0.323, -0.305, 1.006])
RAIL_X, RAIL_W, RAIL_H = 0.016, 0.008, 0.008  # two rails along y, 32 mm apart centre to centre
RAIL_TOP = START[2] - math.sqrt(R**2 - (RAIL_X - RAIL_W / 2) ** 2)
RISER_TOP = RAIL_TOP - RAIL_H
RISER = (0.065, 0.080, RISER_TOP - TOP)  # pine block: x, y, height (22.6 cm)

STAND = m.fixed(
    "stand",
    (START[0], START[1], TOP),
    [
        ("block", m.box(RISER, (0.0, 0.0, RISER[2] / 2)), "pine"),
        ("rail_f", m.box((RAIL_W, 0.050, RAIL_H), (RAIL_X, 0.0, RISER[2] + RAIL_H / 2)), "rail"),
        ("rail_b", m.box((RAIL_W, 0.050, RAIL_H), (-RAIL_X, 0.0, RISER[2] + RAIL_H / 2)), "rail"),
    ],
    colliders=[
        m.col_box("c_block", RISER, (0.0, 0.0, RISER[2] / 2)),
        m.col_box("c_rail_f", (RAIL_W, 0.050, RAIL_H), (RAIL_X, 0.0, RISER[2] + RAIL_H / 2)),
        m.col_box("c_rail_b", (RAIL_W, 0.050, RAIL_H), (-RAIL_X, 0.0, RISER[2] + RAIL_H / 2)),
    ],
    looks={"pine": m.Look(m.rgb(0.80, 0.66, 0.45), roughness=0.8), "rail": m.Look(m.rgb(0.25, 0.25, 0.27), roughness=0.6)},
)

YOYO = m.free(
    "yoyo",
    tuple(float(v) for v in START + np.array([0.0, 0.0, 0.002])),
    _yoyo_parts(),
    [m.col_hull("c_disc", m.cylinder(R, W, (0.0, 0.0, 0.0), axis="y", segments=24))],
    YOYO_LOOKS,
    mass=Y.M,
    friction=(0.8, 0.6),
)

# -- distractors (SimReady, loose; on the left of the table, clear of both arms' paths) -----------------------------
MUG = SimReady("mug", pos=(0.52, -0.44, TOP), size=(0.114, 0.090, 0.080), mass=0.30, package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda", yaw=2.4)
GLASSES = SimReady("glasses", pos=(0.58, -0.04, TOP), size=(0.145, 0.174, 0.057), mass=0.03, package="accessory_eyeglasses_a01", usd="sm_accessory_eyeglasses_a01_01.usd", yaw=0.6)
APPLE = SimReady("apple", pos=(0.46, 0.06, TOP), size=(0.070, 0.067, 0.068), mass=0.18, package="food_produce_apple_b01", usd="sm_food_produce_apple_b01_01.usd", convex=True)

OBJECTS = [STAND, YOYO, MUG, GLASSES, APPLE]

# -- scoring ------------------------------------------------------------------------------------------------------
DT = 0.05
ZONE = 0.10  # a full return: the yo-yo's centre within 10 cm of the fingertip carrying the loop
TOUCH = 0.004  # a fingertip point this close to the yo-yo's surface touches it
STRING_TOUCH = 0.008  # or this close to the string
LEVELS = ((5, 1.0), (3, 0.75), (2, 0.5), (1, 0.25))
STRING_PULL_K, STRING_PULL_MAX = 300.0, 3.0  # N/m, N: a held or resting yo-yo whose string is pulled taut is dragged


def _quat_xyzw(rot):
    return np.asarray(station.matrix_to_quat_xyzw(rot), float)


def _hang_rot(off):
    """The yo-yo's orientation hanging on its string: local +z along the string up to the loop, axle near world y."""
    z = -np.asarray(off, float)
    z /= max(np.linalg.norm(z), 1e-9)
    yv = np.array([0.0, 1.0, 0.0]) - z * z[1]
    yv /= max(np.linalg.norm(yv), 1e-9)
    x = np.cross(yv, z)
    return np.column_stack((x, yv, z))


def _local(points, pos, q_xyzw):
    return (np.asarray(points, float) - pos) @ m.quat_matrix(q_xyzw)


def yoyo_gap(points, pos, q_xyzw):
    """Distance from points to the yo-yo's body (a disc R across, W wide, axle along its local y)."""
    loc = _local(points, pos, q_xyzw)
    radial = np.hypot(loc[:, 0], loc[:, 2])
    dr = np.maximum(radial - R, 0.0)
    dy = np.maximum(np.abs(loc[:, 1]) - W / 2, 0.0)
    return np.hypot(dr, dy)


def _gripped(tips, pos, q_xyzw):
    """Either arm pinching the yo-yo: fingertip points of one finger on each face."""
    loc = _local(tips, pos, q_xyzw)
    radial = np.hypot(loc[:, 0], loc[:, 2])
    near = radial < R + 0.002
    for arm in (0, 1):
        f1 = slice(12 * arm, 12 * arm + 6)
        f2 = slice(12 * arm + 6, 12 * arm + 12)
        on = lambda f, s: bool(np.any(near[f] & (s * loc[f, 1] > W / 2 - 0.006) & (s * loc[f, 1] < W / 2 + 0.008)))
        if (on(f1, 1) and on(f2, -1)) or (on(f1, -1) and on(f2, 1)):
            return True
    return False


def _seg_dist(points, a, b):
    ab = b - a
    t = np.clip(((points - a) @ ab) / max(ab @ ab, 1e-12), 0.0, 1.0)
    return np.linalg.norm(points - (a + t[:, None] * ab), axis=1)


def _in_box(p, centre, half):
    return bool(np.all(np.abs(np.asarray(p) - centre) <= half))


STAND_BOX = (np.array([START[0], START[1], TOP + RISER[2] / 2 + RAIL_H / 2]), np.array([RISER[0] / 2, RISER[1] / 2, RISER[2] / 2 + RAIL_H / 2]) + R)


class Trial:
    def __init__(self):
        self.model = Y.YoYo()
        self.reset()

    def reset(self, env=None):  # the ONLY place a recorded harm is cleared
        self.model.reset()
        self.mode = "body"  # 'body': a physical body (in the cradle, held, on the table); 'string': moved by the model
        self.loops = []  # the loop's last positions, for its velocity and acceleration
        self.foul = False  # a gripper touched the yo-yo or string outside the return zone: 0 for the whole trial
        self.climbing = False
        self.reached = False
        self.streak = 0
        self.best = 0
        self.returns = 0
        self.full = False
        self.score = 0.0
        self.t = 0.0
        self.shown = 0
        self.pulled = False
        if env is not None:
            self._show_string(0)

    # -- drawing and moving -----------------------------------------------------------------------------------
    def _show_string(self, n):
        n = int(min(max(n, 0), N_STR))
        for i in range(N_STR):
            m.set_visible(f"{m.ENV_PATH}/yoyo/visual/str{i}", i < n)
        self.shown = n

    def _write(self, env, pos, rot, vel):
        import torch

        base = env.unwrapped
        asset = base.scene["yoyo"]
        origin = m._np(base.scene.env_origins[0])
        pose = torch.tensor([[*(np.asarray(pos) + origin), *_quat_xyzw(rot)]], device=asset.device, dtype=torch.float32)
        v = torch.tensor([[*np.asarray(vel, float), 0.0, 0.0, 0.0]], device=asset.device, dtype=torch.float32)
        asset.write_root_pose_to_sim_index(root_pose=pose)
        asset.write_root_velocity_to_sim_index(root_velocity=v)

    def _to_body(self, env):
        self.mode = "body"
        m.push(env, "yoyo", (0.0, 0.0, 0.0))
        self._show_string(0)

    # -- one control step -------------------------------------------------------------------------------------
    def step(self, env):
        self.t += DT
        tips = m.fingertips(env)
        loop = tips[LOOP_TIP].copy()
        self.loops = (self.loops + [loop])[-3:]
        if len(self.loops) == 3:
            v_loop = (self.loops[2] - self.loops[1]) / DT
            a_loop = (self.loops[2] - 2 * self.loops[1] + self.loops[0]) / DT**2
        else:
            v_loop, a_loop = np.zeros(3), np.zeros(3)
        st = m.state(env, "yoyo")
        pos, q = np.asarray(st.pos, float), np.asarray(st.quat_xyzw, float)
        mdl = self.model

        if self.mode == "body":
            gripped = _gripped(tips, pos, q)
            rel = pos - loop
            dist = float(np.linalg.norm(rel))
            # a taut string drags a held or resting yo-yo towards the loop
            excess = dist - mdl.s
            if excess > 0.01 and not gripped:
                f = min(STRING_PULL_K * excess, STRING_PULL_MAX)
                m.push(env, "yoyo", tuple(float(v) for v in (-rel / dist * f)))
                self.pulled = True
            elif self.pulled:
                m.push(env, "yoyo", (0.0, 0.0, 0.0))
                self.pulled = False
            over_stand = bool(np.all(np.abs(pos[:2] - START[:2]) < np.array([RISER[0], RISER[1]]) / 2 + 0.01))
            supported = pos[2] < TOP + R + 0.005 or (over_stand and pos[2] < START[2] + 0.015)
            falling = float(st.lin_vel[2]) < -0.05 and not supported
            if falling and not gripped:
                if self.pulled:
                    m.push(env, "yoyo", (0.0, 0.0, 0.0))
                    self.pulled = False
                mdl.release(rel, np.asarray(st.lin_vel, float) - v_loop)
                self.mode = "string"
                m.push(env, "yoyo", (0.0, 0.0, Y.M * Y.G))  # the string, not the solver, now carries its weight
            else:
                self._score()
                return

        # string mode: the model moves the yo-yo
        u_before = mdl.u
        off = mdl.step(DT, a_loop)
        centre = loop + off
        vel = v_loop + mdl.offset_vel()
        rot = _hang_rot(off) if mdl.phase != "slack" else m.quat_matrix(q)
        dist = float(np.linalg.norm(off))
        self._write(env, centre, rot, vel)
        self._show_string(round((mdl.s if mdl.phase != "slack" else dist) / 0.01) if mdl.phase != "slack" else 0)
        q_now = _quat_xyzw(rot)

        # touches: either gripper on the yo-yo or the string
        gap = yoyo_gap(tips, centre, q_now)
        others = np.ones(len(tips), bool)
        others[12:18] = np.linalg.norm(tips[12:18] - loop, axis=1) > 0.035  # the loop finger's own tip carries the string
        sgap = _seg_dist(tips, loop, centre)
        touch = bool(np.any(gap < TOUCH)) or bool(np.any((sgap < STRING_TOUCH) & others))
        if touch:
            if dist > ZONE:
                self.foul = True
            if _gripped(tips, centre, q_now) or dist > ZONE:
                self._end_climb(caught=True)
                mdl.phase = "held"
                mdl.u = 0.0
                self._to_body(env)
                self._score()
                return

        # hitting the table or the riser kills it
        if centre[2] - R < TOP + 0.002 or _in_box(centre, *STAND_BOX):
            self._end_climb(caught=True)
            mdl.s, mdl.u, mdl.phase = Y.L, 0.0, "held"
            self._to_body(env)
            self._score()
            return

        # climbs and returns
        if mdl.phase == "roll" and mdl.u < 0:
            self.climbing = True
            if dist <= ZONE:
                self.reached = True
        if self.climbing and (mdl.u >= 0 or mdl.phase != "roll"):
            self._end_climb()
        self._score()

    def _end_climb(self, caught=False):
        if not self.climbing:
            if caught:
                self.streak = 0
            return
        if self.reached and not caught:
            self.streak += 1
            self.returns += 1
        elif self.reached and caught:
            self.streak += 1  # it got there; catching it ends the run of returns after counting this one
            self.returns += 1
            self.best = max(self.best, self.streak)
            self.streak = 0
        else:
            self.streak = 0
        self.best = max(self.best, self.streak)
        self.climbing = self.reached = False

    def _score(self):
        self.best = max(self.best, self.streak)
        self.full = self.full or (not self.foul and self.best >= 5)
        if self.foul:
            self.score = 0.0
        else:
            self.score = next((s for n, s in LEVELS if self.best >= n), 0.0)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)
SUCCESS_HOLD_S = 1.0


def score(objects):
    return float(TRIAL.score)


def success(objects):
    return bool(TRIAL.full and not TRIAL.foul)


# -- the closed-loop demo (demokit) -----------------------------------------------------------------------------
PLAY = np.array([0.20, -0.15, TOP + 0.51])  # the TCP where the hand plays: the yo-yo bottoms out 8 cm above the table
LIFT, T_LIFT = 0.08, 0.15  # the tug: up 8 cm in 0.15 s
LEAD = 0.15  # s: start the tug this long before the string is seen to run out
LOWER_AFTER, T_LOWER = 0.35, 0.9


def demo_script(ctx):
    """Close on the yo-yo in its cradle, carry it to the play position with the fingers level, open, and then on each
    descent start a sharp 7 cm lift so that the hand is rising fastest as the string runs out; lower the hand again
    slowly while the yo-yo climbs. Reads only the yo-yo's pose and the arm's own joints.
    Failure checks (--mode): none does nothing; drop opens and never tugs; late tugs 0.15 s after the string runs out;
    early tugs 0.45 s before; carry holds the yo-yo and moves it up and down; grab catches the climbing yo-yo with the
    left gripper below the hand; chase drops the hand 15 cm as the yo-yo climbs, to meet it."""
    import demokit as d

    r, lft = ctx.right, ctx.left
    mode = ctx.mode
    for _ in range(4):
        yield
    if mode == "none":
        for _ in range(200):
            yield
        return
    from yam_arena import ik
    from yam_arena.constants import READY

    r.grip = d._as_grip(ik.tcp_pose(np.array(READY), r.base)[1])
    yield from r.close(width=0.026, force=10.0, t=0.6)
    yield from r.move(START + np.array([-0.02, 0.0, 0.08]), t=1.0)
    yield from r.move((0.24, -0.20, TOP + 0.40), grip=d.tilted(60, 0), t=1.5)
    yield from r.move(PLAY, grip=d.level(0), t=1.5)
    yield from r.wait(0.8)
    if mode == "carry":
        for _ in range(5):
            yield from r.move(PLAY + np.array([0.0, 0.0, -0.25]), t=1.0)
            yield from r.move(PLAY, t=1.0)
        return
    yield from r.open(t=0.3)
    if mode == "drop":
        for _ in range(300):
            yield
        return
    lead = {"late": -0.15, "early": 0.45}.get(mode, LEAD)
    lift = LIFT

    def loop_point():
        return r.tcp() + 0.013 * r.approach_axis()

    prev = None
    cycles = 0
    t_end = 600
    k = 0
    while k < t_end and cycles < 7:
        c = ctx.pos("yoyo")
        s = float(np.linalg.norm(c - loop_point()))
        u = 0.0 if prev is None else (s - prev) / d.DT
        prev = s
        if mode == "grab" and cycles == 2 and u < 0 and s < 0.25:
            yield from lft.move(c + np.array([0.0, 0.0, 0.0]), grip=d.down(0), t=0.6)
            yield from lft.close(width=0.02, t=0.3)
            return
        if mode == "chase" and prev is not None and u < -0.05:
            cycles += 1
            yield from r.move(PLAY + np.array([0.0, 0.0, -0.15]), t=0.25)
            yield from r.move(PLAY, t=1.2)
            prev = None
            continue
        go = False
        if u > 0.05 and mode != "chase":
            ttb = (Y.L - s) / u
            go = ttb < lead if lead >= 0 else False
        if lead < 0 and prev is not None and u < -0.05 and s > Y.L - 0.03:
            go = True  # late: the yo-yo is already climbing back
        if go:
            cycles += 1
            yield from r.move(PLAY + np.array([0.0, 0.0, lift]), t=T_LIFT)
            yield from r.wait(LOWER_AFTER - T_LIFT)
            yield from r.move(PLAY, t=T_LOWER)
            prev = None
            # wait until the yo-yo is going down again
            for _ in range(60):
                c = ctx.pos("yoyo")
                s2 = float(np.linalg.norm(c - loop_point()))
                if prev is not None and s2 - prev > 0.002:
                    break
                prev = s2
                yield
            prev = None
            continue
        k += 1
        yield
    yield from r.wait(1.0)
