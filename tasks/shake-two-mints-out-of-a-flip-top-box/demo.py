"""Demo for the mint-box task, run in the yam_arena embodiment.

    python demo.py task.py out_dir [--window] [--max-steps 1800] [--no-record] [--mode full|open|short|long|pour|still]

1. The left hand comes down over the lying box and takes it by its narrow sides, near its bottom end.
2. The right hand, leaning in from the right, lays a fingertip corner against the box's side behind the lid's lip and
   draws it toward the lid end: the lip is pushed past the snap and the flap springs open.
3. The left hand lifts the box, carries it over the dish, and leans forward so that the lid end tips down; it shakes
   the box along its length in short bursts (in layout B after rocking it five times), then levels it again.
4. It puts the box back where it lay and keeps hold of it. The right hand, now behind the lid end, sweeps the open
   flap round and presses it shut until it snaps.
5. Both hands let go and draw away.

Controls: ``--mode short`` stops one burst of shaking early. ``--mode long`` goes on shaking, harder, after the
second mint. ``--mode pour`` tips the box right over and holds it still for three seconds. ``--mode open`` is the full
demo without step 4's lid closing. ``--mode still`` keeps both arms still.

The script is open loop: the strokes were tuned by trial in this simulator for these two layouts, and it never looks
at the dish. Every target comes from the layout constants in the task file and the sizes in ``mintkit.py``. The only
feedback is each hand's own measured position (the robot's proprioception), used to correct for the arm's sag. The
demo never reads the state of the box, the lid or a mint. A real trial, or a different fill, needs the dish watched.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20
FULL_OPEN = 0.0939                 # m between the pads at grip command 1.0


def frame(approach, closing) -> np.ndarray:
    """Tool rotation whose approach axis (z) and finger-closing axis (x) point along these world directions."""
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(closing, float)
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def gap(width: float) -> float:
    """Grip command (1 open .. 0 closed) for a gap between the pads in metres."""
    return min(1.0, max(0.0, width / FULL_OPEN))


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class Hand:
    """One arm: smooth moves (IK on the URDF), each starting where the last command left it."""

    def __init__(self, name: str, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = self.worst_deg = 0.0

    def _solve(self, pos, rot) -> None:
        from yam_arena import ik

        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.worst_deg = max(self.worst_deg, float(np.degrees(re)))
        self.cmd = np.r_[q, self.grip]

    def set_grip(self, grip: float, duration: float = 0.5):
        """Generator: open or close to ``grip`` in place."""
        from yam_arena import ik

        g0 = self.grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self.grip = g0 + ik.smoothstep(k / n) * (grip - g0)
            self.cmd = np.r_[self.q, self.grip]
            yield

    def move_joints(self, q_goal, duration: float, grip: float | None = None):
        """Generator: swing to joint angles ``q_goal``, every joint moving together."""
        from yam_arena import ik

        q0, g0 = self.q.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q, self.grip = q0 + s * (np.asarray(q_goal, float) - q0), g0 + s * (g1 - g0)
            self.pos, self.rot = ik.tcp_pose(self.q, self.base)
            self.cmd = np.r_[self.q, self.grip]
            yield


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def both(*gens):
    """Generator: run several moves side by side until all have finished."""
    live = list(gens)
    while live:
        for g in list(live):
            try:
                next(g)
            except StopIteration:
                live.remove(g)
        if live:
            yield


def best_ik(hand, pos, rot):
    """Joint angles for ``pos``/``rot`` from the best of a few seeds: the one that leaves the joints most room."""
    from yam_arena import ik

    seeds = [hand.q.copy()]
    for q6 in (-1.5, -0.7, 0.7, 1.5):
        s = hand.q.copy()
        s[5] = q6
        seeds.append(s)
    for q1, q5 in ((0.6, 0.8), (-0.6, -0.8), (1.0, 1.2), (-1.0, -1.2)):
        s = hand.q.copy()
        s[0], s[4] = q1, q5
        seeds.append(s)
    best = None
    for seed in seeds:
        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), seed, base_pos=hand.base, iters=400)
        room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
        cost = (pe > 5e-4 or re > 5e-3, -room)
        if best is None or cost < best[1]:
            best = (q, cost)
    return best[0]


TOOL: dict = {}                          # where each hand's tool point actually is (updated every control step)
TOOLROT: dict = {}                       # ... and which way the hand actually points
PHASE: dict = {"name": "start"}
SAGS: dict = {}                          # each hand's sag corrector, for the trace

UP = np.array([0.0, 0.0, 1.0])
TIP = 0.013                              # the fingertips' ends are this far past the tool point
TIP_HALF = 0.0075                        # half the width of a fingertip's end
TIP_THICK = 0.004                        # from the middle of the closed fingertips to their outer face, at the end
# -- the left hand, which holds the box
GRIP_Z = 0.008                           # its tool point this far toward the box's bottom end from the box's middle
GRIP_UP = 0.0145                         # ... and this far above the table when the box lies on it
BOX_OPEN, BOX_GRIP = 0.066, 0.0425        # pads apart on the way in, and the command when closed on the 47 mm width
L_CLOSE = float(os.environ.get("MINT_LCLOSE", "-1"))   # which way round the left hand is (the sign of its closing axis)
LIFT = 0.080                             # the tool point's height on the way to the dish and back
DROP_H = 0.048                           # the opening is held this far above the table over the middle of the dish
# -- the right hand, which works the lid
S_LEAN = 55.0                            # opening the lid: it leans this far from straight down toward the left
F_LEAN = 50.0                            # closing the lid: it leans this far forward
S_CLOSE, F_CLOSE = 1.0, -1.0             # which way round each of those is (the sign of its closing axis)
CORNER_UP = 0.0140                       # the fingertip corner that catches the lip is held this far above the table
D_WALL = -0.0010                         # ... and commanded this far outside the box's side (-: pressed against it)
PULL_FROM, PULL_TO = 0.027, 0.054        # the pulling face goes from here to here along the box (from its middle)
SWEEP_OFF = 0.009                        # closing: the fingertip sweeps this far beyond the lid end ...
SWEEP_FROM, SWEEP_TO = -0.014, 0.019     # ... its leading face from here to here across the box (from its middle)
PRESS = -0.0008                          # ... and then presses to this far beyond the lid end (-: into it)
FWD_UP = 0.0137
PARK_L = (0.230, 0.300, 0.200)
PARK_R = (0.200, -0.280, 0.200)
GAIN, BIAS_MAX = 0.25, 0.025             # how fast, and how far, a hand corrects for its own sag
TURN_GAIN, TURN_MAX = 0.15, math.radians(8.0)   # ... and for pointing off the way it is told to
# The strokes over the dish (see ``pour``), found by trial for each layout: after them exactly two mints have left.
ROCK = "L60:0.5:0.3,L15:0.5:0.2"         # tip the lid end well down and half way back
STROKES = {"A": "L55:1.0:0.6,S15:0.3:3,W0.6,S15:0.3:3,W0.6,L0:0.8:0.3",
           "B": ",".join([ROCK] * 5 + ["L55:0.5:0.5"] + ["S20:0.3:3,W0.6"] * 5 + ["L0:0.8:0.3"])}
# ... and what the control runs do instead.
CONTROL = {"short": "L55:1.0:0.6,S15:0.3:3,W0.6,L0:0.8:0.3",                       # stops one burst early
           "long": STROKES["A"].replace(",L0:0.8:0.3", "") + ",L60:0.5:0.5" + ",T8:0.2:6,W0.6" * 4 + ",L0:0.8:0.3",   # shakes on
           "pour": "L75:1.2:3.0,L0:1.0:0.3"}                                       # tipped right over and held still


def strokes(T, mode: str) -> str:
    return os.environ.get("MINT_POUR") or CONTROL.get(mode) or STROKES[T.LOT]


def _rotvec(rot) -> np.ndarray:
    """The rotation vector of a rotation matrix."""
    a = math.acos(float(np.clip((np.trace(rot) - 1.0) / 2.0, -1.0, 1.0)))
    if a < 1e-9:
        return np.zeros(3)
    return np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]]) * (a / (2.0 * math.sin(a)))


def _turn(vec) -> np.ndarray:
    """The rotation matrix of a rotation vector."""
    a = float(np.linalg.norm(vec))
    if a < 1e-12:
        return np.eye(3)
    k = np.asarray(vec, float) / a
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(a) * kx + (1.0 - math.cos(a)) * (kx @ kx)


def _square(rot) -> np.ndarray:
    """The nearest rotation whose axes are the coordinate axes (the fixed turn between two ways of naming a frame)."""
    out = np.zeros((3, 3))
    m = np.abs(rot).copy()
    for _ in range(3):
        i, j = np.unravel_index(int(np.argmax(m)), m.shape)
        out[i, j] = math.copysign(1.0, rot[i, j])
        m[i, :] = -1.0
        m[:, j] = -1.0
    return out


class Sag:
    """Slowly offsets a hand's command by its measured error, so that its tool point ends up where it is wanted; and,
    while ``live``, turns the command by the hand's measured error in direction, so that it points as wanted."""

    def __init__(self, hand):
        self.hand, self.bias, self.want = hand, np.zeros(3), None
        self.live, self.trim, self.want_rot, self.frame, self.off = True, np.zeros(3), None, None, 0.0

    def measured(self):
        """The hand's measured rotation, in the convention the commands use."""
        rot = TOOLROT.get(self.hand.name)
        if rot is None:
            return None
        if self.frame is None:                       # found once, while the arm rests at its ready pose
            self.frame = _square(rot.T @ self.hand.rot)
        return rot @ self.frame

    def to(self, pos, rot):
        if self.want is not None and self.hand.name in TOOL:
            self.bias = np.clip(self.bias + GAIN * (self.want - TOOL[self.hand.name]), -BIAS_MAX, BIAS_MAX)
        got = self.measured()
        if self.want_rot is not None and got is not None:
            err = _rotvec(self.want_rot @ got.T)
            self.off = math.degrees(float(np.linalg.norm(err)))
            if self.live:
                trim = self.trim + TURN_GAIN * err
                size = float(np.linalg.norm(trim))
                self.trim = trim if size <= TURN_MAX else trim * (TURN_MAX / size)
        self.want, self.want_rot = np.asarray(pos, float), rot
        self.hand._solve(self.want + self.bias, _turn(self.trim) @ rot)


def glide(sag, target, seconds: float, rot, rot_from=None):
    """Generator: take the hand's tool point in a straight line to ``target``, correcting for sag on the way. With
    ``rot_from`` the hand also turns from that rotation to ``rot``."""
    from yam_arena import ik

    start = sag.want if sag.want is not None else sag.hand.pos.copy()
    n = max(1, round(seconds * CONTROL_HZ))
    for k in range(1, n + 1):
        s = ik.smoothstep(k / n)
        r = rot if rot_from is None else ik.slerp_matrix(rot_from, rot, s)
        sag.to(start + s * (np.asarray(target, float) - start), r)
        yield


def go(sag, pos, rot, seconds: float):
    """Generator: swing the arm to a pose it is not yet near, by its joints, then settle on it."""
    yield from sag.hand.move_joints(best_ik(sag.hand, pos, rot), seconds)
    sag.want = None
    yield from glide(sag, pos, 0.3, rot)


# -- where things are, from the layout ------------------------------------------------------------------------------
def spot(T, xb: float, zb: float, up: float) -> np.ndarray:
    """A point given across (xb) and along (zb) the lying box from its middle, ``up`` above the table (world)."""
    centre, _ = T.box_start()
    p = centre + T.K.lying_rot(T.BOX_YAW) @ np.array([xb, 0.0, zb])
    return np.array([p[0], p[1], T.TOP + up])


def left_rot(T, lean: float, roll: float = 0.0) -> np.ndarray:
    """The left hand over the lying box, its fingers closing across the box's width; leaning ``lean`` degrees forward
    (which tips the lid end down) and rolled ``roll`` degrees about its own fingers."""
    a = math.radians(lean)
    return _rz(T.BOX_YAW) @ frame((math.sin(a), 0.0, -math.cos(a)), (0.0, L_CLOSE, 0.0)) @ _rz(math.radians(roll))


HELD = np.array([[-L_CLOSE, 0.0, 0.0], [0.0, 0.0, -L_CLOSE], [0.0, -1.0, 0.0]])    # the held box's axes in the left tool frame


def opening_in_hand(T) -> np.ndarray:
    """The middle of the opening in the left hand's tool frame, while it holds the box."""
    K = T.K
    return np.array([0.0, -L_CLOSE * GRIP_Z, GRIP_UP - K.T / 2]) + HELD @ K.opening()


def pour_tool(T, lean: float, roll: float = 0.0, along: float = 0.0, across: float = 0.0) -> tuple:
    """The left hand's tool point and rotation that hold the opening over the middle of the dish with the box tipped
    ``lean`` degrees; ``along`` shifts the box that far along its own length, ``across`` through its thickness (shakes)."""
    rot = left_rot(T, lean, roll)
    over = np.array([T.DISH_XY[0], T.DISH_XY[1], T.TOP + DROP_H])
    return over - rot @ opening_in_hand(T) - L_CLOSE * along * rot[:, 1] + across * rot[:, 2], rot


def side_rot(T) -> np.ndarray:
    a = math.radians(S_LEAN)
    return _rz(T.BOX_YAW) @ frame((0.0, math.sin(a), -math.cos(a)), (S_CLOSE, 0.0, 0.0))


def side_tool(T, out: float, face: float, up: float = CORNER_UP) -> np.ndarray:
    """Opening the lid: the right hand's tool point with its fingertip corner ``out`` outside the box's lip-side wall,
    its pulling face ``face`` along the box from the middle, the corner ``up`` above the table."""
    a = math.radians(S_LEAN)
    corner = spot(T, T.K.W / 2 + out, face - TIP_THICK, up)
    end = corner - TIP_HALF * (_rz(T.BOX_YAW) @ np.array([0.0, math.cos(a), math.sin(a)]))
    return end - TIP * side_rot(T)[:, 2]


def fwd_rot(T) -> np.ndarray:
    a = math.radians(F_LEAN)
    return _rz(T.BOX_YAW) @ frame((math.sin(a), 0.0, -math.cos(a)), (0.0, F_CLOSE, 0.0))


def fwd_tool(T, face: float, off: float, up: float = FWD_UP) -> np.ndarray:
    """Closing the lid: the right hand's tool point with its fingertip's foremost corner ``off`` beyond the lid end,
    its leading face ``face`` across the box from the middle, that corner ``up`` above the table."""
    a = math.radians(F_LEAN)
    corner = spot(T, face - TIP_THICK, T.K.H / 2 + off, up)
    end = corner - TIP_HALF * (_rz(T.BOX_YAW) @ np.array([math.cos(a), 0.0, math.sin(a)]))
    return end - TIP * fwd_rot(T)[:, 2]


def pour(T, sag, plan: str):
    """Generator: the strokes over the dish. ``plan`` is a comma-separated list of
    ``L<deg>:<move s>:<hold s>``  lean the box to <deg> (lid end down), then hold;
    ``R<deg>:<move s>:<hold s>``  roll it to <deg> about the fingers, then hold;
    ``S<mm>:<period s>:<count>``  shake it <count> times along its own length, <mm> each way;
    ``T<mm>:<period s>:<count>``  shake it <count> times through its thickness, <mm> each way;
    ``W<s>``                      wait."""
    lean = roll = 0.0
    for token in [t for t in plan.split(",") if t]:
        kind, nums = token[0], [float(v) for v in token[1:].split(":")]
        PHASE["name"] = f"pour {token}"
        if kind in "LR":
            lean0, roll0 = lean, roll
            lean, roll = (nums[0], roll) if kind == "L" else (lean, nums[0])
            n = max(1, round(nums[1] * CONTROL_HZ))
            for k in range(1, n + 1):
                s = 0.5 - 0.5 * math.cos(math.pi * k / n)
                sag.to(*pour_tool(T, lean0 + s * (lean - lean0), roll0 + s * (roll - roll0)))
                yield
            for _ in range(round(nums[2] * CONTROL_HZ)):
                sag.to(*pour_tool(T, lean, roll))
                yield
        elif kind in "ST":
            n = max(2, round(nums[1] * CONTROL_HZ))
            for k in range(1, int(nums[2]) * n + 1):
                d = -0.001 * nums[0] * math.sin(2 * math.pi * k / n)
                sag.to(*pour_tool(T, lean, roll, along=d if kind == "S" else 0.0, across=d if kind == "T" else 0.0))
                yield
        elif kind == "W":
            yield from wait(nums[0])


def script(T, left, right, mode: str = "full", idle_s: float = 60.0):
    """The demo as a generator: one yield per control step."""
    if mode == "still":
        yield from wait(idle_s)
        return
    ls, rs = Sag(left), Sag(right)
    SAGS.update(left=ls, right=rs)
    lr, sr, fr = left_rot(T, 0.0), side_rot(T), fwd_rot(T)
    take = spot(T, 0.0, -GRIP_Z, GRIP_UP)
    # 1. The left hand takes the box; the right hand comes to the box's side.
    PHASE["name"] = "take"
    left.grip = right.grip = 1.0
    yield from both(go(ls, take + 0.090 * UP, lr, 2.0), go(rs, side_tool(T, 0.060, PULL_FROM, 0.090), sr, 2.4))
    yield from left.set_grip(gap(BOX_OPEN), 0.3)
    yield from glide(ls, take, 1.6, lr)
    yield from glide(ls, take, 0.4, lr)
    ls.live = False                                # from here the box, not the air, is what the hand turns against
    yield from left.set_grip(gap(BOX_GRIP), 0.8)
    # 2. The right hand flicks the lid open.
    PHASE["name"] = "open"
    yield from right.set_grip(0.0, 0.4)
    yield from glide(rs, side_tool(T, 0.012, PULL_FROM), 1.4, sr)
    yield from glide(rs, side_tool(T, D_WALL, PULL_FROM), 0.8, sr)
    yield from glide(rs, side_tool(T, D_WALL, PULL_FROM), 0.4, sr)
    yield from glide(rs, side_tool(T, D_WALL, PULL_TO), 1.8, sr)
    yield from wait(0.3)
    yield from glide(rs, side_tool(T, 0.040, PULL_TO), 0.8, sr)
    yield from glide(rs, side_tool(T, 0.060, PULL_TO, 0.110), 0.8, sr)
    # 3. Over the dish: tip, shake, level. Meanwhile the right hand goes round behind where the lid end will be.
    PHASE["name"] = "pour"
    ready = fwd_tool(T, SWEEP_FROM, 0.045, 0.090)

    def left_pours():
        yield from glide(ls, take + (LIFT - GRIP_UP) * UP, 1.0, lr)
        over, _ = pour_tool(T, 0.0)
        yield from glide(ls, np.array([over[0], over[1], T.TOP + LIFT]), 1.8, lr)
        yield from glide(ls, over, 0.8, lr)
        yield from glide(ls, over, 0.3, lr)
        yield from pour(T, ls, strokes(T, mode))
        yield from glide(ls, np.array([over[0], over[1], T.TOP + LIFT]), 0.8, lr)
        yield from glide(ls, take + (LIFT - GRIP_UP) * UP, 1.8, lr)
        yield from glide(ls, take + 0.004 * UP, 1.2, lr)
        yield from glide(ls, take, 0.6, lr)
        yield from glide(ls, take, 0.4, lr)

    def right_goes_round():
        yield from wait(1.6)                 # until the left hand has lifted the box clear
        yield from go(rs, ready, fr, 2.4)

    yield from both(left_pours(), right_goes_round())
    # 4. The right hand sweeps the flap round and presses it shut.
    if mode != "open":
        PHASE["name"] = "close"
        yield from glide(rs, fwd_tool(T, SWEEP_FROM, SWEEP_OFF), 1.6, fr)
        yield from glide(rs, fwd_tool(T, SWEEP_TO, SWEEP_OFF), 1.6, fr)
        yield from glide(rs, fwd_tool(T, SWEEP_TO, PRESS), 1.0, fr)
        yield from glide(rs, fwd_tool(T, SWEEP_TO, PRESS), 0.5, fr)
        yield from glide(rs, fwd_tool(T, SWEEP_TO, 0.030), 0.8, fr)
    # 5. Let go, and away.
    PHASE["name"] = "leave"
    yield from left.set_grip(gap(BOX_OPEN), 0.6)

    def right_leaves():
        yield from glide(rs, fwd_tool(T, SWEEP_TO, 0.050, 0.100), 1.0, fr)
        yield from go(rs, np.array([PARK_R[0], PARK_R[1], T.TOP + PARK_R[2]]), fr, 1.6)

    def left_leaves():
        yield from glide(ls, take + 0.080 * UP, 1.0, lr)
        yield from glide(ls, np.array([PARK_L[0], PARK_L[1], T.TOP + PARK_L[2]]), 1.6, lr)

    yield from both(left_leaves(), right_leaves())
    yield from wait(3.0)


def _write_mp4(frames, path: Path) -> None:
    import imageio

    with imageio.get_writer(path, fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=None,
                            ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]) as w:
        for f in frames:
            w.append_data(f)


def run(task_path: str, out_dir: str, max_steps: int = 1800, record: bool = True, window: bool = False,
        hold_s: float = 2.5, mode: str = "full", every: int = 5, cam: tuple = (320, 176)) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam[0],
                               cam_height=cam[1], headless=not window)
    T = emb.task.module
    obs = emb.reset(Scene(id="mint-demo", instruction=T.INSTRUCTION))
    left, right = Hand("left", station.BASES["left"]), Hand("right", station.BASES["right"])
    gen = script(T, left, right, mode, idle_s=max_steps / CONTROL_HZ)
    frames: dict[str, list] = {}
    trace = []

    def snap_frames(o) -> None:
        if record:
            for name, img in o.images.items():
                frames.setdefault(name, []).append(np.asarray(img))

    def box() -> list:
        from yam_arena.scene import object_states   # importable only once the simulator has started

        st = object_states(emb._env, [T.BOX])[T.BOX]
        rot = T.K._rotation(st.quat_xyzw)
        tilt = math.degrees(math.asin(float(np.clip(-rot[2, 2], -1.0, 1.0))))          # lid end below level
        yaw = math.degrees(math.atan2(-rot[1, 2], -rot[0, 2]))                        # its length, from pointing at the robot
        return [round(1000 * float(v), 1) for v in st.pos - [0.0, 0.0, T.TOP]] + [round(tilt, 1), round(yaw, 1)]

    def measure() -> None:
        from mintkit import finger_frames, finger_points

        TOOL["left"], TOOL["right"] = (np.asarray(p, float) for p in finger_points(emb._env))
        TOOLROT["left"], TOOLROT["right"] = finger_frames(emb._env)

    def hands() -> dict:
        from mintkit import _np

        out_ = {}
        for h in (left, right):
            robot = emb._env.unwrapped.scene[f"{h.name}_robot"]
            q, jn = _np(robot.data.joint_pos)[0], list(robot.joint_names)
            out_[h.name] = {"tool_mm": [round(1000 * float(v), 1) for v in TOOL[h.name] - [0.0, 0.0, T.TOP]],
                            "want_mm": [round(1000 * float(v), 1) for v in h.pos - [0.0, 0.0, T.TOP]],
                            "gap_mm": round(1000 * sum(abs(float(q[jn.index(j)])) for j in ("joint7", "joint8")), 1),
                            "off_deg": round(SAGS[h.name].off, 2) if h.name in SAGS else None}
        return out_

    snap_frames(obs)
    measure()
    start = {"task": T.report(None), "box": box()}
    steps, ended, success, info = 0, None, False, {}
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(Action(data=pack(left.cmd, right.cmd)))
        steps += 1
        measure()
        info = res.info
        snap_frames(res.observation)
        if steps % every == 0:
            trace.append({"step": steps, "phase": PHASE["name"], "score": info.get("score"), "task": T.report(None),
                          "hands": hands(), "box": box()})
        if steps % 40 == 0 and trace:
            t = trace[-1]
            print(f"step {steps} {t['phase']} score {t['score']} {t['task']} box {t['box']} L {t['hands']['left']} R {t['hands']['right']}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    result = {"lot": T.LOT, "mode": mode, "pour": "" if mode == "still" else strokes(T, mode),
              "task_score": 1.0 if success else float(info.get("score", 0.0) or 0.0),
              "success": success, "steps": steps, "script_steps": ended,
              "worst_ik": {h.name: {"mm": round(h.worst_mm, 2), "deg": round(h.worst_deg, 2)} for h in (left, right)},
              "start": start, "end": {"task": T.report(None), "box": box()}, "trace": trace}
    if record:
        import imageio

        for name, fs in frames.items():
            _write_mp4(fs, out / f"demo_{name}.mp4")
        top = next((c for c in frames if "top" in c), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
    (out / "result.json").write_text(json.dumps(result, indent=2, default=lambda o: o.item()))
    print(json.dumps({k: v for k, v in result.items() if k != "trace"}), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1800)
    ap.add_argument("--window", action="store_true", help="show the Isaac Sim window on the instance's desktop")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--mode", default="full", choices=("full", "open", "short", "long", "pour", "still"))
    ap.add_argument("--trace-every", type=int, default=5, help="steps between entries of the trace in result.json")
    ap.add_argument("--cam", default="320x176", help="recorded frame size, e.g. 640x352")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, mode=a.mode, every=a.trace_every,
        cam=tuple(int(v) for v in a.cam.split("x")))
