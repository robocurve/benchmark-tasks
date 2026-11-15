"""Scripted full-score demo for the power-supply hot-swap task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1800] [--no-record]

The right arm squeezes the failed supply's release tab against its fixed handle (one pinch, as a technician's thumb
and fingers do), which draws the latch's hook in, pulls the supply out against the connector's hold, and slides it by
its body in front of the live supply. The left arm grips the spare by its body, slides it until it lines up with the
empty bay and pushes it in, correcting its position and turn as it goes, then turns its closed fingers forward and
pushes the spare home by its handle, through the connector's mating force: the latch's ramped hook rides in on the bay
wall and snaps into its notch.
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


class Arm:
    """One arm: smooth Cartesian moves (IK on the URDF), each starting where the last command left it."""

    def __init__(self, name: str, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = 0.0

    def _solve(self, pos, rot):
        from yam_arena import ik

        q, pe, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, rot, duration: float, grip: float | None = None):
        """Generator: go to ``pos``/``rot`` (world TCP); ``grip`` 1 open .. 0 closed (None keeps it)."""
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, rot, s))
            yield

    def set_grip(self, grip: float, duration: float = 0.8):
        yield from self.move(self.pos, self.rot, duration, grip)

    def aim(self, pos, rot):
        """Track a pose this step (for closed-loop moves)."""
        self._solve(pos, rot)


def reachable_yaw(arm, pos, yaw):
    """The grip yaw, or the same parallel grip turned 180 degrees, whichever the arm reaches more exactly. A wrist at
    its joint limit otherwise leaves the fingers several degrees off and they twist light parts as they close."""
    from yam_arena import ik
    from yam_arena.demo import grip_rotation

    best = None
    for y in (yaw, yaw + math.pi, yaw - math.pi):
        _, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(grip_rotation(y)), arm.q, base_pos=arm.base, iters=200)
        if best is None or pe + 0.05 * re < best[0]:
            best = (pe + 0.05 * re, y)
    return best[1]


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


class _Pose:
    """A supply's pose on the table (x, y, yaw about z), with the PartState methods the script uses."""

    def __init__(self, x, y, z, yaw):
        self.pos, self.yaw = np.array([x, y, z], float), float(yaw)

    def rotation(self):
        return _rot_axis((0, 0, 1), self.yaw)

    def to_world(self, local):
        return self.pos + self.rotation() @ np.asarray(local, float)


# The demo runs open loop: it reads no simulator state. The arms are compliant and lag under the supplies' friction,
# so where a supply ends up after a move, and the small corrections that square the spare up in front of the bay, were
# measured once with DEMO_CALIBRATE=1 (which runs the same moves closed loop on the simulator's poses and prints these
# numbers) and are fixed here. The layout is fixed (object_jitter=0) and the simulation deterministic.
CAL = {
    # (x, y, yaw) of the failed supply after the 26 cm pull (the arm lags the pull by 2 cm)
    "failed_pulled": (0.22184, 0.00526, 0.01362),
    # (dx, dy, turn) steps sliding the spare across in line with the bay, short of its mouth, and squaring it up
    "spare_square": [
        (0.02189, -0.03, -0.01566), (0.01053, -0.03, -0.04235), (-6e-05, -0.03, -0.04599),
        (-0.00125, -0.03, -0.04512), (-0.00132, -0.03, -0.04655), (-0.00064, -0.03, -0.04682),
        (-0.00069, -0.03, -0.04659), (-0.00082, -0.03, -0.04761), (-0.00095, 0.00387, -0.0485),
        (-0.00094, 0.00523, -0.05327), (-0.00094, 0.00581, -0.05799), (-0.00096, 0.006, -0.06133),
        (-0.00101, 0.00599, -0.06), (-0.00058, 0.00448, -0.0422), (-0.00049, 0.00381, -0.02521),
        (-0.00048, 0.00353, -0.01811),
    ],
    # (x, y, yaw) of the spare after that, let go
    "spare_lined_up": (0.25722, 0.00498, -0.02044),
}
CALIBRATE = bool(os.environ.get("DEMO_CALIBRATE"))
PINCH_OPEN = 0.6   # fingers opened to 56 mm around handle bar and tab (22 mm across together)
PUSH_PAST = float(os.environ.get("DEMO_PUSH_PAST", 0.06))   # how far past home the final push is commanded (m)
PUSH_Z = 0.028     # height of the fingertips' centre on the handle bar for the final push
_MEASURED: dict = {}


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    from yam_arena.demo import grip_rotation

    L, R = ctx.left, ctx.right
    T = task
    top = T.TOP
    failed0 = _Pose(*T.FAILED.pos[:2], top, 0.0)       # the published starting layout
    spare0 = _Pose(*T.SPARE.pos[:2], top, 0.0)
    # Fingers close across the supply (world y). On the handle: one pad on the fixed handle bar, the other on the
    # release tab beside it, so closing pushes the tab to its stop and draws the latch's hook in. On the body: a firm
    # grip, at its centre, for sliding and pushing the 0.9 kg supply.
    rot_l = grip_rotation(reachable_yaw(L, spare0.pos, 0.0))
    rot_r = grip_rotation(reachable_yaw(R, failed0.pos, 0.0))
    tips = 0.013  # PAD_TIP_PAST_TCP: the TCP sits this far above the pad tips
    up = np.array([0, 0, 0.05])
    ex, ey = np.array([1.0, 0, 0]), np.array([0, 1.0, 0])
    bay = np.array([T.CAGE_XY[0], T.CAGE_XY[1]])

    def handle(psu):   # TCP with the pad tips 14 mm above the table, across handle bar and release tab
        return psu.to_world((T.GRIP_X, T.GRIP_Y, 0.014 + tips))

    def body(psu):     # across the body at its centre, low, so sliding or pushing it doesn't twist or tip it
        return psu.to_world((0.0, 0.0, 0.006 + tips))

    def measured_pose(key, name):
        """Calibration: read the supply's pose; otherwise the calibrated constant."""
        if CALIBRATE:
            s = ctx.states()[name]
            r = s.rotation()
            _MEASURED[key] = [round(float(s.pos[0]), 5), round(float(s.pos[1]), 5), round(math.atan2(r[1, 0], r[0, 0]), 5)]
        x, y, yaw = _MEASURED[key] if CALIBRATE else CAL[key]
        if CALIBRATE:
            print("CAL so far = " + json.dumps({k: v for k, v in _MEASURED.items() if k != "turn"}), flush=True)
        return _Pose(x, y, top, yaw)

    def grab(arm, rot, target, open_to=0.5):
        yield from arm.move(target + up, rot, 2.0, grip=open_to)
        yield from arm.move(target, rot, 1.2)
        yield from arm.set_grip(0.0, 0.8)
        yield from wait(0.3)

    def let_go(arm, rot, open_to=0.5):
        yield from arm.set_grip(open_to, 0.6)
        yield from arm.move(arm.pos + up, rot, 0.8)

    # 1. Right arm: squeeze the failed supply's tab against its handle (latch released), pull the supply straight out
    #    (the connector's contacts hold it until pulled over their ridge), and let go.
    yield from grab(R, rot_r, handle(failed0), open_to=PINCH_OPEN)
    yield from wait(0.3)
    yield from R.move(R.pos - 0.03 * ex, rot_r, 1.5)
    yield from R.move(R.pos - (T.PULL - 0.03) * ex, rot_r, 4.0)
    yield from wait(0.3)
    yield from let_go(R, rot_r)

    # 2. Right arm: grip its body and slide it right, in front of the live supply, clear of the bay's line; move away.
    yield from grab(R, rot_r, body(measured_pose("failed_pulled", "failed_psu")), open_to=1.0)
    yield from R.move(R.pos + T.CLEAR_DY * ey, rot_r, 3.0)
    yield from wait(0.3)
    yield from let_go(R, rot_r, open_to=1.0)
    yield from R.move(R.pos + np.array([-0.04, -0.06, 0.04]), rot_r, 1.2)

    # 3. Left arm: grip the spare's body at its centre and slide it right and a little forward until it lines up with
    #    the empty bay, its back face 2 cm short of the mouth, in steps of at most 3 cm with small corrections of
    #    position and turn (calibrated). Let go, so it settles flat on the table.
    yield from grab(L, rot_l, body(spare0), open_to=1.0)
    base_yaw = reachable_yaw(L, spare0.pos, 0.0)

    def steps_to(key, x_t, y_t, tol_x):
        if not CALIBRATE:
            for dx, dy, turn in CAL[key]:
                yield from L.move(L.pos + np.array([dx, dy, 0.0]), grip_rotation(base_yaw + turn), 0.7)
                yield from wait(0.2)
            return
        turn, done = (_MEASURED["turn"] if "turn" in _MEASURED else 0.0), []
        for _ in range(16):
            s = ctx.states()["spare_psu"]
            r = s.rotation()
            ex_, ey_, eyaw = x_t - s.pos[0], y_t - s.pos[1], -math.atan2(r[1, 0], r[0, 0])
            if abs(ex_) < tol_x and abs(ey_) < 0.0015 and abs(eyaw) < math.radians(0.5):
                break
            dx, dy = float(np.clip(ex_, -0.03, 0.03)), float(np.clip(1.3 * ey_, -0.03, 0.03))
            turn = float(np.clip(turn + eyaw, -math.radians(5), math.radians(5)))
            done.append([round(dx, 5), round(dy, 5), round(turn, 5)])
            yield from L.move(L.pos + np.array([dx, dy, 0.0]), grip_rotation(base_yaw + turn), 0.7)
            yield from wait(0.2)
        _MEASURED[key], _MEASURED["turn"] = done, turn
        print("CAL so far = " + json.dumps({k: v for k, v in _MEASURED.items() if k != "turn"}), flush=True)

    yield from steps_to("spare_square", bay[0] + T.MOUTH_X - T.PSU_L / 2 - 0.020, bay[1], tol_x=0.005)
    yield from let_go(L, L.rot, open_to=1.0)
    yield from wait(1.0)   # let it settle before (in calibration) measuring where it ended up

    # 4. Left arm: close the fingers, turn the gripper to point forward (20 degrees below level), and push the spare
    #    into the bay and home by the front of its handle bar, on the supply's centreline (the bay's lead-ins steer it
    #    in), up to the connector, then on through its
    #    mating force (the contacts climb the blade's ridge and drop behind it). Pushing along the fingers, the force
    #    runs through the arm rather than bending the compliant wrist; the grip is commanded PUSH_PAST beyond home so
    #    the arm presses hard enough, and the back stop takes the rest. The hook's ramp rides in on the bay wall and
    #    the spring snaps it out into its notch at home. Back away.
    spare = measured_pose("spare_lined_up", "spare_psu")
    rot_push = _rot_axis((0, 1, 0), math.radians(-70)) @ grip_rotation(0.0)
    along = rot_push[:, 2]                                     # the fingers' direction

    def push_tcp(dx):   # TCP with the closed fingertips' centre on the handle bar's front face, dx further in
        tip = spare.to_world((T.GRIP_X - 0.005, sum(T.HANDLE_Y) / 2, PUSH_Z)) + dx * ex
        return tip - tips * along

    yield from L.set_grip(0.0, 0.6)
    yield from L.move(push_tcp(-0.012) + 1.5 * up, rot_push, 2.0)
    yield from L.move(push_tcp(-0.012), rot_push, 1.2)
    yield from L.move(push_tcp(bay[0] - spare.pos[0] + PUSH_PAST), rot_push, 9.0)
    yield from wait(1.0)
    yield from L.move(push_tcp(bay[0] - spare.pos[0] - 0.05), rot_push, 1.5)
    yield from L.move(L.pos + up, rot_push, 0.8)


def _lag_mm(emb, arm):
    """Debug: the commanded TCP minus where the compliant arm actually is (mm), which shows how hard it pushes."""
    from yam_arena import ik

    try:
        rob = emb._env.unwrapped.scene[f"{arm.name}_robot"]
        q = rob.data.joint_pos
        q = np.asarray(getattr(q, "torch", q)[0].detach().cpu(), float)
        return np.round(1000 * (arm.pos - ik.tcp_pose(q[:6], arm.base)[0]), 1).tolist()
    except Exception as e:  # noqa: BLE001
        return f"? {e}"


def _yaw_of(p):
    r = p.rotation()
    return math.atan2(r[1, 0], r[0, 0])


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


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def run(task_path: str, out_dir: str, max_steps: int = 1800, record: bool = True, cam_width: int = 320,
        cam_height: int = 176, hold_s: float = 4.0) -> dict:
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam_width,
                               cam_height=cam_height)
    obs = emb.reset(Scene(id="demo", instruction="demo"))
    task = emb.task.module
    ctx = Ctx(emb)
    gen = script(task, ctx)
    frames: dict[str, list] = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended, success, info = 0, None, False, {}
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(ctx.action())
        steps += 1
        info = res.info
        snap(res.observation)
        if str(steps) in os.environ.get("DEMO_SNAP", "").split(","):
            import imageio

            for cam, img in res.observation.images.items():
                imageio.imwrite(out / f"snap_{steps}_{cam}.png", np.asarray(img))
        if steps % 100 == 0 or (os.environ.get("DEMO_DEBUG") and steps % 20 == 0 and steps > int(os.environ.get("DEMO_DEBUG_FROM", 0))):
            msg = f"step {steps} score {info.get('score')}"
            if os.environ.get("DEMO_DEBUG"):
                st = ctx.states()
                for n in ("failed_psu", "spare_psu", "live_psu"):
                    c = task.in_bay(st[n])
                    sl = st[n].sliders
                    msg += (f" | {n} {np.round(1000 * c, 1)}mm latch {1000 * sl['latch']:.1f}"
                            f" contacts {1000 * sl['contact_l']:.1f}/{1000 * sl['contact_r']:.1f}"
                            f" tilt {task.tilt_deg(st[n]):.1f}"
                            f" yaw {math.degrees(math.atan2(st[n].rotation()[1, 0], st[n].rotation()[0, 0])):.2f}")
                msg += " | lag " + " ".join(f"{arm.name} {_lag_mm(emb, arm)}" for arm in (ctx.left, ctx.right))
            print(msg, flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    result = {"task_score": score, "success": success, "steps": steps,
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
    if record:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        top = next((c for c in frames if "top" in c), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
    (out / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1800)
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record)
