"""Demo for the clog task, run in the yam_arena embodiment.

    python demo.py task.py out_dir [--window] [--max-steps 1500] [--no-record] [--mode full|front|still]

1. The left hand, leaning in from the manikin's right side, takes the bare lower leg below the knee. The right hand,
   leaning 60 degrees forward, takes the clog on the floor by the sides of its toe box.
2. The left hand bends the knee back 30 degrees, which points the toes down and lifts the heel, and holds it there.
3. The right hand carries the clog along the floor under the toes, raises it until the toes are in the opening behind
   the vamp, and slides it back until the toes are under the vamp's edge.
4. The left hand straightens the knee again over five seconds. The right hand follows: at every step it puts the clog
   where the toes are as far under the vamp as that knee angle allows, so the heel comes down in front of the strap.
5. Both hands let go, the left first. The clog hangs on the foot with the strap behind the heel.

Controls: ``--mode front`` leaves the leg alone and pushes the clog onto the hanging foot from the front with one hand.
With the strap back, the strap meets the top of the foot. With ``CLOG_STRAP=forward`` in the environment the clog
starts with its strap forward on the vamp and slips on like a mule, without the strap behind the heel.
``--mode still`` keeps both arms still.

Every target comes from the layout constants in the task file and the sizes in ``clogkit.py``. The only feedback is
each hand's own measured position (the robot's proprioception): the knee's angle is read from where the left hand is
actually holding the leg, and the right hand corrects for its own sag. The demo never reads a part's state.
"""
from __future__ import annotations

import argparse
import json
import math
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

    def move(self, pos, duration: float, rot=None, grip: float | None = None):
        """Generator: go in a straight line to ``pos`` (world, tool point) and ``rot``; ``grip`` 1 open .. 0 closed."""
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        r1 = r0 if rot is None else rot
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos, float) - p0), ik.slerp_matrix(r0, r1, s))
            yield

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


def chain(*gens):
    for g in gens:
        yield from g


def together(*gens):
    gens = [iter(g) for g in gens]
    while gens:
        for g in list(gens):
            try:
                next(g)
            except StopIteration:
                gens.remove(g)
        if gens:
            yield


def best_ik(hand, pos, rot):
    """Joint angles for ``pos``/``rot`` from the best of a few seeds (the hand's own, and its wrist turned either way)."""
    from yam_arena import ik

    best = None
    for q6 in (None, -1.0, 1.0):
        seed = hand.q.copy()
        if q6 is not None:
            seed[5] = q6
        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), seed, base_pos=hand.base, iters=400)
        cost = pe + 0.05 * re
        if best is None or cost < best[1]:
            best = (q, cost)
    return best[0]


TOOL: dict = {}                          # where each hand's tool point actually is (updated every control step)

LEAN_L = math.radians(65.0)              # the leg hand leans in this far from straight down, toward the manikin's midline
LEAN_R = math.radians(60.0)              # the clog hand leans this far forward: its wrist stays low, under the leg hand
L_AP = np.array([0.0, -math.sin(LEAN_L), -math.cos(LEAN_L)])
L_GRIP = frame(L_AP, (-1.0, 0.0, 0.0))                                    # pads on the front and back of the shank
R_AP = np.array([math.sin(LEAN_R), 0.0, -math.cos(LEAN_R)])
R_GRIP = frame(R_AP, (0.0, -1.0, 0.0))                                    # pads on the two sides of the vamp

GRIP_DOWN = 0.100                        # the leg hand holds the shank this far below the knee
SHANK_GAP = 0.048                        # pads this far apart on the shank (5.4 cm thick there)
CLOG_AT = np.array([0.030, 0.0, 0.012])  # the clog hand's tool point, in the clog's frame: low on the sides of the toe box.
                                         # Leaning this far, the hand's palm then lies in front of the clog's toe, not on it.
CLOG_GAP = 0.060                         # pads this far apart there (the toe box is 6.7 cm wide)
BEND = 30.0                              # degrees the knee is bent back
LIFT = 0.004                             # the foot's lowest point is kept this far above the footbed while it goes in
OPEN_AT = 0.085                          # the toes first come down this far back from the inside toe end, in the opening
# How far back from the inside toe end the tip of the toes can be at each knee angle with the foot 2 mm clear of the
# vamp, the footbed and the strap (from a side-view model of the foot and the clog; see the README): degrees, metres.
TOES_AT = ((0.0, 0.004), (6.0, 0.004), (8.0, 0.008), (10.0, 0.012), (12.0, 0.016), (14.0, 0.020), (16.0, 0.024),
           (18.0, 0.028), (20.0, 0.030), (22.0, 0.034), (24.0, 0.036), (26.0, 0.040), (28.0, 0.042), (30.0, 0.044))
SLACK = 0.002                            # ... and the demo keeps this much further back while the knee is bent
GAIN, BIAS_MAX = 0.25, 0.020             # how fast, and how far, a hand corrects for its own sag


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def toes_at(deg: float) -> float:
    xs, ys = zip(*TOES_AT)
    return float(np.interp(deg, xs, ys)) + (SLACK if deg > 6.0 else 0.0)


def shank_pose(T, deg: float, back: float = 0.0):
    """The leg hand's tool point and rotation for a knee bent back ``deg``; ``back`` m short of the leg along its approach."""
    r = rot_y(-math.radians(deg))
    rot = r @ L_GRIP
    return T.knee_world() + r @ np.array([0.0, 0.0, -GRIP_DOWN]) - back * rot[:, 2], rot


def toe_tip(T, deg: float) -> np.ndarray:
    """Where the tip of the toes is, at sole level, for a knee bent back ``deg``."""
    K = T.K
    return T.knee_world() + rot_y(-math.radians(deg)) @ np.array([K.TOE_X, 0.0, K.SOLE])


def clog_origin(T, deg: float, toes: float | None = None, lift: float = LIFT) -> np.ndarray:
    """Where the clog's origin (inside toe end, on the footbed) goes for a knee bent back ``deg``: the toes ``toes`` m
    back from the toe end (default: as far forward as that angle allows), the foot's lowest point ``lift`` above the footbed."""
    tip = toe_tip(T, deg)
    return np.array([tip[0] - (toes_at(deg) if toes is None else toes), T.knee_world()[1], tip[2] - lift])


def knee_now(T) -> float:
    """The knee's bend in degrees, from where the leg hand's tool point actually is."""
    d = TOOL["left"] - T.knee_world()
    return math.degrees(math.atan2(float(d[0]), float(-d[2])))


class Sag:
    """Slowly offsets a hand's command by its measured error, so that its tool point ends up where it is wanted."""

    def __init__(self, hand):
        self.hand, self.bias, self.want = hand, np.zeros(3), None

    def to(self, pos, rot):
        if self.want is not None:
            self.bias = np.clip(self.bias + GAIN * (self.want - TOOL[self.hand.name]), -BIAS_MAX, BIAS_MAX)
        self.want = np.asarray(pos, float)
        self.hand._solve(self.want + self.bias, rot)


def glide(sag, target, seconds: float, rot):
    """Generator: take the hand's tool point in a straight line to ``target``, correcting for sag on the way."""
    from yam_arena import ik

    start = sag.want if sag.want is not None else sag.hand.pos.copy()
    n = max(1, round(seconds * CONTROL_HZ))
    for k in range(1, n + 1):
        sag.to(start + ik.smoothstep(k / n) * (np.asarray(target, float) - start), rot)
        yield


def script(T, left, right, mode: str = "full", idle_s: float = 60.0):
    """The demo as a generator: one yield per control step."""
    K = T.K
    if mode == "still":
        yield from wait(idle_s)
        return
    c0 = T.clog_start()
    floor = T.TOP + K.SOLE_T

    def hold(origin, back: float = 0.0) -> np.ndarray:                     # the clog hand's tool point for a clog at ``origin``
        return np.asarray(origin, float) + CLOG_AT - back * R_AP

    # 1. Both hands come in and take hold: the left on the shank, the right on the clog's vamp. The right hand first
    #    hovers over the clog and corrects for its own sag, so that its open fingers come down astride the clog.
    pl, rl = shank_pose(T, 0.0, back=0.08)
    above = hold(c0, back=0.07) + (0.0, 0.0, 0.080)                       # well above the clog: the swing in must not clip it
    first = [right.move_joints(best_ik(right, above, R_GRIP), 2.0, grip=1.0)]
    if mode != "front":
        first.append(left.move_joints(best_ik(left, pl, rl), 2.0, grip=1.0))
    yield from together(*first)
    sag = Sag(right)
    yield from glide(sag, above, 0.4, R_GRIP)
    yield from glide(sag, hold(c0, back=0.07), 1.2, R_GRIP)
    yield from glide(sag, hold(c0), 1.5, R_GRIP)
    yield from glide(sag, hold(c0), 0.4, R_GRIP)
    yield from right.set_grip(gap(CLOG_GAP), 0.6)
    if mode == "front":
        # control: one hand pushes the clog onto the hanging foot from the front
        home = clog_origin(T, 0.0, lift=0.001)
        yield from glide(sag, hold(c0 + (0.0, 0.0, 0.010)), 0.6, R_GRIP)
        yield from glide(sag, hold((c0[0], home[1], home[2])), 2.0, R_GRIP)
        yield from glide(sag, hold(home), 3.0, R_GRIP)
        yield from wait(0.5)
        yield from right.set_grip(1.0, 0.6)
        yield from right.move(right.pos - 0.15 * R_AP, 1.5)
        yield from wait(3.0)
        return
    yield from left.move(shank_pose(T, 0.0)[0], 1.2, rot=rl)
    yield from left.set_grip(gap(SHANK_GAP), 0.6)
    yield from wait(0.3)

    # 2. The left hand bends the knee back; the right lifts the clog just off the floor.
    def bend(a0: float, a1: float, seconds: float):
        n = max(1, round(seconds * CONTROL_HZ))
        for k in range(1, n + 1):
            from yam_arena import ik

            p, r = shank_pose(T, a0 + (a1 - a0) * ik.smoothstep(k / n))
            left._solve(p, r)
            yield

    yield from together(bend(0.0, BEND, 3.0), glide(sag, hold(c0 + (0.0, 0.0, 0.005)), 1.0, R_GRIP))
    yield from wait(0.5)

    # 3. The clog goes along the floor under the toes, up until the toes are in the opening, then back under the vamp.
    deg = knee_now(T)
    under = clog_origin(T, deg, toes=OPEN_AT)
    yield from glide(sag, hold((under[0], under[1], floor + 0.005)), 2.5, R_GRIP)
    yield from glide(sag, hold((under[0], under[1], floor + 0.005)), 0.4, R_GRIP)
    yield from glide(sag, hold(under), 1.5, R_GRIP)
    yield from glide(sag, hold(clog_origin(T, deg)), 1.5, R_GRIP)
    yield from wait(0.3)

    # 4. The knee is straightened; the clog follows the foot, the toes kept as far under the vamp as the angle allows.
    n = round(5.0 * CONTROL_HZ)
    for k in range(1, n + 1):
        from yam_arena import ik

        p, r = shank_pose(T, BEND * (1.0 - ik.smoothstep(k / n)))
        left._solve(p, r)
        sag.to(hold(clog_origin(T, max(0.0, knee_now(T)))), R_GRIP)
        yield
    for _ in range(round(1.0 * CONTROL_HZ)):                                  # ... and is pressed up under the sole
        sag.to(hold(clog_origin(T, max(0.0, knee_now(T)), lift=0.001)), R_GRIP)
        yield
    yield from wait(0.3)

    # 5. Both let go, the leg hand first, and draw back.
    yield from left.set_grip(1.0, 0.6)
    yield from left.move(shank_pose(T, 0.0, back=0.11)[0], 1.2)
    yield from right.set_grip(1.0, 0.6)
    yield from right.move(right.pos - 0.15 * R_AP, 1.5)
    yield from wait(5.0)


def _write_mp4(frames, path: Path) -> None:
    import imageio

    with imageio.get_writer(path, fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=None,
                            ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]) as w:
        for f in frames:
            w.append_data(f)


def run(task_path: str, out_dir: str, max_steps: int = 1500, record: bool = True, window: bool = False,
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
    obs = emb.reset(Scene(id="clog-demo", instruction=T.INSTRUCTION))
    left, right = Hand("left", station.BASES["left"]), Hand("right", station.BASES["right"])
    gen = script(T, left, right, mode, idle_s=max_steps / CONTROL_HZ)
    frames: dict[str, list] = {}
    trace = []
    names = [T.LEG, T.CLOG, T.STRAP]

    def snap_frames(o) -> None:
        if record:
            for name, img in o.images.items():
                frames.setdefault(name, []).append(np.asarray(img))

    def states():
        from yam_arena.scene import object_states   # importable only once the simulator has started

        return object_states(emb._env, names)

    def measure() -> None:
        from clogkit import finger_points

        TOOL["left"], TOOL["right"] = (np.asarray(p, float) for p in finger_points(emb._env))

    def hands() -> dict:
        from clogkit import _np

        out_ = {}
        for h in (left, right):
            robot = emb._env.unwrapped.scene[f"{h.name}_robot"]
            q, jn = _np(robot.data.joint_pos)[0], list(robot.joint_names)
            out_[h.name] = {"tool_mm": [round(1000 * float(v), 1) for v in TOOL[h.name] - [0.0, 0.0, T.TOP]],
                            "want_mm": [round(1000 * float(v), 1) for v in h.pos - [0.0, 0.0, T.TOP]],
                            "gap_mm": round(1000 * sum(abs(float(q[jn.index(j)])) for j in ("joint7", "joint8")), 1),
                            "joint_err_deg": [round(math.degrees(float(q[jn.index(f"joint{i + 1}")]) - float(h.q[i])), 1) for i in range(6)]}
        return out_

    def places(st) -> dict:
        mm = lambda v: [round(1000 * float(x), 1) for x in np.asarray(v, float) - [0.0, 0.0, T.TOP]]   # noqa: E731
        return {n: mm(st[n].pos) for n in names}

    snap_frames(obs)
    measure()
    start = {"task": T.report(states()), "places": places(states())}
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
            st = states()
            trace.append({"step": steps, "score": info.get("score"), "task": T.report(st), "hands": hands(),
                          "places": places(st)})
        if steps % 40 == 0 and trace:
            t = trace[-1]
            print(f"step {steps} score {t['score']} {t['task']} {t['hands']} {t['places']}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    result = {"lot": T.LOT, "mode": mode, "task_score": 1.0 if success else float(info.get("score", 0.0) or 0.0),
              "success": success, "steps": steps, "script_steps": ended,
              "worst_ik": {h.name: {"mm": round(h.worst_mm, 2), "deg": round(h.worst_deg, 2)} for h in (left, right)},
              "start": start, "end": {"task": T.report(states()), "places": places(states())}, "trace": trace}
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
    ap.add_argument("--max-steps", type=int, default=1500)
    ap.add_argument("--window", action="store_true", help="show the Isaac Sim window on the instance's desktop")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--mode", default="full", choices=("full", "front", "still"))
    ap.add_argument("--trace-every", type=int, default=5, help="steps between entries of the trace in result.json")
    ap.add_argument("--cam", default="320x176", help="recorded frame size, e.g. 640x352")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, mode=a.mode, every=a.trace_every,
        cam=tuple(int(v) for v in a.cam.split("x")))
