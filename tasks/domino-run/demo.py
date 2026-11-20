"""Scripted full-score demo for the domino-run task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 2400] [--no-record] [--cam 640x352]

Each arm stands the three dominoes on its own side of the lane: it takes a flat domino across its 24 mm width, near
one end, with the gripper leaning 6 degrees back toward the arms, lifts it, turns the gripper 90 degrees about the
fingers' closing line (so the domino hangs from the fingers, standing on end, its faces square to the lane), lowers it
onto its tick, opens and draws back along the fingers. The left arm sets the three far ticks (6, 5, 4), the right the
three near ones (3, 2, 1), far ones first, so a hand never reaches over a standing domino. The right arm then pushes
the top of the first domino away from the arms with its closed fingertips, 12 mm, and draws back; the run falls.
Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.
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


def _rot_axis(axis, angle) -> np.ndarray:
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def _rotvec(r) -> np.ndarray:
    a = math.acos(max(-1.0, min(1.0, (np.trace(r) - 1) / 2)))
    if a < 1e-9:
        return np.zeros(3)
    v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / (2 * math.sin(a))
    return v * a


def _from_rotvec(v) -> np.ndarray:
    a = float(np.linalg.norm(v))
    return np.eye(3) if a < 1e-12 else _rot_axis(v, a)


class Arm:
    """One arm: smooth Cartesian moves (IK on the URDF), each starting where the last command left it. With
    ``solve=False`` it only records where each move ends (for the offline reach check)."""

    def __init__(self, name: str, base_pos, solve: bool = True):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base, self.solve = name, np.asarray(base_pos, float), solve
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.home = (self.pos.copy(), self.rot.copy())
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = 0.0
        self.worst_deg = 0.0
        self.ends: list[tuple[np.ndarray, np.ndarray, str]] = []

    def _solve(self, pos, rot):
        from yam_arena import ik

        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.worst_deg = max(self.worst_deg, math.degrees(re))
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, rot, duration: float, grip: float | None = None, label: str = ""):
        """Generator: go to ``pos``/``rot`` (world TCP); ``grip`` 1 open .. 0 closed (None keeps it)."""
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        pos = np.asarray(pos, float)
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            p, r = p0 + s * (pos - p0), ik.slerp_matrix(r0, rot, s)
            if self.solve:
                self._solve(p, r)
            else:
                self.pos, self.rot = p, r
            yield
        self.ends.append((pos.copy(), np.asarray(rot, float).copy(), label))

    def set_grip(self, grip: float, duration: float = 0.6):
        yield from self.move(self.pos, self.rot, duration, grip, "grip")


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def both(*gens):
    gens = [iter(g) for g in gens]
    while gens:
        for g in list(gens):
            try:
                next(g)
            except StopIteration:
                gens.remove(g)
        if gens:
            yield


# The demo runs open loop: it reads no simulator state. How each domino sits in the hand once it has been turned
# upright (its pose in the TCP frame: a rotation vector and an offset, m) is measured once with DEMO_CALIBRATE=1, which
# reads the simulator's pose of the domino while it hangs above its tick, prints it, and uses it for that placement;
# the numbers are then fixed here. The layout is fixed (object_jitter=0) and the simulation deterministic.
CAL: dict = {   # measured with DEMO_CALIBRATE=1: domino -> its pose in the TCP frame once turned upright
    0: {"rotvec": [1.24188, -1.30911, 1.11468], "t": [0.00116, 0.01146, 0.01256]},
    1: {"rotvec": [1.25097, -1.30694, 1.11589], "t": [0.0011, 0.01128, 0.01274]},
    2: {"rotvec": [1.25832, -1.29702, 1.12799], "t": [0.00209, 0.01241, 0.01263]},
    3: {"rotvec": [1.24838, -1.29275, 1.14196], "t": [0.00143, 0.01051, 0.01268]},
    4: {"rotvec": [1.25682, -1.30767, 1.13525], "t": [0.00113, 0.01109, 0.01274]},
    5: {"rotvec": [1.25541, -1.31079, 1.13258], "t": [0.00083, 0.01078, 0.01282]},
}
CALIBRATE = bool(os.environ.get("DEMO_CALIBRATE"))
_MEASURED: dict = {}

TILT = math.radians(6.0)    # the pick leans the gripper 6 deg from vertical; the place points it 6 deg below
                            # horizontal (level, the wrist would need joint 4 past its limit this low)
TIP_GAP = 0.0015            # picking, the fingertips' lowest corner this far above the table
PICK_BACK = 0.012           # picking, the TCP this far in from the domino's middle toward its far (+x) end
OPEN_PICK = 0.45            # grip opening around a flat domino (1 = fully open)
OPEN_RELEASE = 0.40
PLACE_GAP = 0.0008          # the domino is let go with its base this far above the table
LEFT_JOBS = ((0, 5), (1, 4), (2, 3))    # (domino, tick), far ticks first
RIGHT_JOBS = ((3, 2), (4, 1), (5, 0))
PUSH_AT = 0.040             # the push meets the first domino's back face this high above the table
PUSH_IN = 0.012             # and carries on this far past first contact


def rotations():
    from yam_arena.demo import grip_rotation

    down = grip_rotation(0.0)                       # pointing down, fingers closing along world y
    ey = np.array([0.0, 1.0, 0.0])
    pick = _rot_axis(ey, TILT) @ down               # approach leaning toward -x (the arms)
    place = _rot_axis(ey, -(math.pi / 2 - TILT)) @ down   # approach pointing +x, 6 deg below horizontal
    return pick, place


def in_hand_planned(task):
    """Where a domino sits in the TCP frame once turned upright, as planned: (rotation, offset)."""
    pick, place = rotations()
    top = task.TOP
    tcp = np.array([task.FLAT_X + PICK_BACK, 0.0, top + _pick_tcp_height()])
    dom = np.array([task.FLAT_X, 0.0, top + task.DOM_T / 2])
    r_dom_flat = _from_quat(task.FLAT_XYZW)
    r_hand = pick.T @ r_dom_flat
    t_hand = pick.T @ (dom - tcp)
    return r_hand, t_hand


def _from_quat(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _pick_tcp_height() -> float:
    """TCP height above the table at the pick: the fingertips' lowest corner (13 mm out along the approach, 13 mm
    down the 26 mm wide pad) TIP_GAP above the table."""
    return TIP_GAP + 0.0128 * math.cos(TILT) + 0.013 * math.sin(TILT)


def script(task, ctx):
    """The demo as a generator: one yield per control step (both arms)."""
    from yam_arena.demo import grip_rotation

    pick_rot, place_rot = rotations()
    top = task.TOP
    a_place = place_rot[:, 2]
    flags = {"left_done": False}

    def hand(i):
        if CALIBRATE and i in _MEASURED:
            m = _MEASURED[i]
            return _from_rotvec(m["rotvec"]), np.array(m["t"])
        if i in CAL:
            return _from_rotvec(CAL[i]["rotvec"]), np.array(CAL[i]["t"])
        return in_hand_planned(task)

    def tcp_for(i, dom_pos, dom_rot=np.eye(3)):
        """The TCP pose that puts domino i at dom_pos/dom_rot, given how it sits in the hand."""
        r_h, t_h = hand(i)
        rot = dom_rot @ r_h.T
        return np.asarray(dom_pos, float) - rot @ t_h, rot

    def pick(arm, i):
        x0, y0, _ = task.START[i]
        at = np.array([x0 + PICK_BACK, y0, top + _pick_tcp_height()])
        yield from arm.move(at + np.array([0.0, 0.0, 0.06]), pick_rot, 2.2, grip=OPEN_PICK, label="over the domino")
        yield from arm.move(at, pick_rot, 1.2, label="fingers either side of its far end")
        yield from arm.set_grip(0.0, 0.6)
        yield from wait(0.2)
        yield from arm.move(at + np.array([0.0, 0.0, 0.10]), pick_rot, 1.0, label="lift")

    def stand(arm, i, k):
        spot = np.array([task.TICK_X[k], task.RUN_Y, top + task.DOM_L / 2 + PLACE_GAP])
        above = spot + np.array([0.0, 0.0, 0.035])
        p, r = tcp_for(i, above)
        yield from arm.move(p, r, 2.4, label="turn it upright over its tick")
        yield from wait(0.6)
        if CALIBRATE:
            s = ctx.states()[f"domino_{i}"]
            r_dom = _from_quat(s.quat_xyzw)
            r_h = arm.rot.T @ r_dom
            t_h = arm.rot.T @ (np.asarray(s.pos) - arm.pos)
            _MEASURED[i] = {"rotvec": [round(float(v), 5) for v in _rotvec(r_h)],
                            "t": [round(float(v), 5) for v in t_h]}
            print("CAL so far = " + json.dumps(_MEASURED), flush=True)
            p, r = tcp_for(i, above)
            yield from arm.move(p, r, 0.5, label="corrected")
        p, r = tcp_for(i, spot)
        yield from arm.move(p, r, 1.4, label="down onto the tick")
        yield from wait(0.3)
        yield from arm.set_grip(OPEN_RELEASE, 0.5)
        yield from wait(0.3)
        yield from arm.move(arm.pos - 0.045 * a_place, arm.rot, 1.0, label="draw back along the fingers")

    def left():
        L = ctx.left
        for i, k in LEFT_JOBS:
            yield from pick(L, i)
            yield from stand(L, i, k)
        flags["left_done"] = True
        yield from L.move(*L.home, 2.5, grip=1.0, label="home")

    def right():
        R = ctx.right
        for n, (i, k) in enumerate(RIGHT_JOBS):
            yield from pick(R, i)
            if n == 0:
                p, r = tcp_for(i, np.array([0.18, -0.15, top + 0.09]))
                yield from R.move(p, r, 2.4, label="turn it upright and wait")
                while not flags["left_done"]:
                    yield
            yield from stand(R, i, k)
        # Tip the first domino: the closed fingers, pointing down, put the front edge of their pads (13 mm ahead of the
        # TCP) against its back face, their tips 8 mm below its top; push 12 mm away from the arms, draw back.
        down = grip_rotation(0.0)
        x_face = task.TICK_X[0] - task.DOM_T / 2
        tcp_touch = np.array([x_face - 0.013, task.RUN_Y, top + PUSH_AT + 0.0128])
        yield from R.move(tcp_touch + np.array([-0.03, 0.0, 0.03]), down, 1.8, grip=0.0, label="behind the first")
        yield from R.move(tcp_touch - np.array([0.008, 0.0, 0.0]), down, 0.8, label="fingertips at its back")
        yield from R.move(tcp_touch + np.array([PUSH_IN, 0.0, 0.0]), down, 0.6, label="push its top")
        yield from R.move(tcp_touch + np.array([-0.04, 0.0, 0.04]), down, 0.8, label="draw back")
        yield from R.move(*R.home, 2.5, grip=1.0, label="home")

    yield from both(left(), right())


def offline_plan(task, solve: bool = False):
    """The demo's moves without the simulator: each arm's list of (pos, rot, label) at the end of every move."""
    from yam_arena import station

    class _Ctx:
        left = Arm("left", station.BASES["left"], solve=solve)
        right = Arm("right", station.BASES["right"], solve=solve)

    ctx = _Ctx()
    steps = sum(1 for _ in script(task, ctx))
    return {"left": ctx.left.ends, "right": ctx.right.ends, "steps": steps, "ctx": ctx}


class Ctx:
    def __init__(self, emb):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])

    def states(self):
        from yam_arena.scene import object_states

        return object_states(self.emb._env, self.emb.task.names)

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.cmd, self.right.cmd))


TRACE = int(os.environ.get("DEMO_TRACE", "0") or 0)


def _trace(emb, ctx, steps) -> None:
    """Debug: each arm's commanded and actual TCP (mm), finger joints and the dominoes' poses."""
    from yam_arena.constants import TCP_OFFSET

    base = emb._env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    out = [f"[trace] step {steps}"]
    for arm, side in ((ctx.left, "left_robot"), (ctx.right, "right_robot")):
        robot = base.scene[side]
        names = [n.rsplit("/", 1)[-1] for n in robot.body_names]
        pose = robot.data.body_link_pose_w[0].detach().cpu().numpy()[names.index("gripper")]
        x, y, z, w = pose[3:7]
        r = _from_quat((x, y, z, w))
        tcp = pose[:3] - origin + r @ np.array(TCP_OFFSET)
        jp = robot.data.joint_pos[0].detach().cpu().numpy()
        out.append(f"  {arm.name}: cmd {np.round(1000 * arm.pos, 1).tolist()} act {np.round(1000 * tcp, 1).tolist()} "
                   f"rot_err_deg {math.degrees(math.acos(max(-1, min(1, (np.trace(arm.rot.T @ r) - 1) / 2)))):.1f} "
                   f"grip_cmd {arm.grip:.2f} fingers {np.round(1000 * jp[-2:], 1).tolist()}")
    for name, s in ctx.states().items():
        if name.startswith("domino"):
            out.append(f"  {name}: {np.round(1000 * np.asarray(s.pos), 1).tolist()} q {np.round(s.quat_xyzw, 3).tolist()}")
    print("\n".join(out), flush=True)


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def run(task_path: str, out_dir: str, max_steps: int = 2400, record: bool = True, cam_width: int = 640,
        cam_height: int = 352, hold_s: float = 6.0) -> dict:
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam_width,
                               cam_height=cam_height)
    obs = emb.reset(Scene(id="demo", instruction=emb.task.module.INSTRUCTION))
    task = emb.task.module
    ctx = Ctx(emb)
    gen = script(task, ctx)
    frames: dict[str, list] = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended, success, info, first_full = 0, None, False, {}, None
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(ctx.action())
        steps += 1
        info = res.info
        if first_full is None and float(info.get("score", 0.0) or 0.0) >= 1.0:
            first_full = steps
        snap(res.observation)
        if steps % 100 == 0:
            print(f"step {steps} score {info.get('score')} {task._EP['info']}", flush=True)
        if TRACE and steps % TRACE == 0:
            _trace(emb, ctx, steps)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    # The trial's score is what the task file's score() reported on the last step; it is never set to 1.0 here just
    # because the episode ended.
    final = float(info.get("score", 0.0) or 0.0)
    result = {"task_score": final, "success": success, "steps": steps, "first_full_score_step": first_full,
              "script_ended_step": ended, "last_info": {k: v for k, v in task._EP["info"].items()},
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
    if CALIBRATE:
        result["CAL"] = _MEASURED
    if record:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        top = next((c for c in frames if "top" in c), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(result, default=str), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--cam", default="640x352", help="camera width x height")
    a = ap.parse_args()
    w, h = (int(v) for v in a.cam.split("x"))
    run(a.task, a.out, a.max_steps, record=not a.no_record, cam_width=w, cam_height=h)
