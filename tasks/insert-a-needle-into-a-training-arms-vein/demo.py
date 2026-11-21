"""Demo for the needle-stick task, run in the yam_arena embodiment.

    python demo.py task.py out_dir [--window] [--max-steps 800] [--no-record]
                   [--mode full|notraction|release|letgo|steep|through|still]

1. The right hand, closed and leaning 45 degrees toward the wrist, presses its fingertips on the forearm's skin
   11 cm below the elbow crease and draws it toward the wrist until the hand itself has moved 9 mm (the skin ends
   about 8 mm drawn): the vein is held. It stays there.
2. Meanwhile the left hand, pointing straight down, takes the tube holder out of its cradle across the barrel, lifts
   it, carries it over to the arm and tips it until the needle points up the arm, 25 degrees below level, bevel up.
3. The left hand brings the needle's tip to 2 cm short of the skin on the needle's own line and goes forward slowly
   until it stops moving: the point is on the skin. It keeps pushing until the skin gives (it feels its own hand move
   on), eases off at once, goes on a set distance from where the skin was, and then leaves its command alone.

Controls: ``--mode notraction`` presses the skin but does not draw it; ``--mode release`` lets the skin go once the
needle is in the vein; ``--mode letgo`` lets go of the holder once the needle is in the vein; ``--mode steep`` goes in
at 45 degrees; ``--mode through`` goes on to 18 mm of needle; ``--mode still`` keeps both arms still. With
``VEIN_BEVEL=down`` the holder starts turned over in its cradle, so the same moves put the needle in bevel down.

Every target comes from the layout constants in the task file and the sizes in ``veinkit.py``. The only feedback is
each hand's own measured position (the robot's proprioception): to correct for the arm's sag, to find the skin (the
hand stops) and to tell when the skin has given (the hand moves on). The demo never reads a part's state.
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

R_DOWN = frame((0.0, 0.0, -1.0), (-1.0, 0.0, 0.0))        # the left hand pointing straight down, closing across x
LEAN = math.radians(45.0)                                 # the anchoring hand leans this far toward the wrist
R_ANCHOR = frame((0.0, math.sin(LEAN), -math.cos(LEAN)), (0.0, -1.0, 0.0))
GRIP_AT = (0.001, 0.004)                 # the tool point on the holder: forward of the barrel's middle, above its axis
OPEN, SQUEEZE = 0.040, 0.013             # the pads' gap over the 19 mm barrel: open, and commanded when holding it
ANGLE = 25.0                             # degrees below level the needle goes in at
SHORT = 0.020                            # the tip waits this far short of the skin, on the needle's line
AIM = 0.0038                             # how far under the skin the tip is to end
BELOW = 0.110                            # the anchoring fingertips press this far below the crease
PRESS = 0.016                            # ... sent this far into the skin, to press on it
DRAW = 0.009                             # ... and then draw the skin this far toward the wrist (by the hand's own measure)
DRAW_LEAD = 0.035                        # the most the command leads the hand while it draws
APPROACH, CREEP = 0.00025, 0.00015       # m per control step: toward the skin, and on through the tissue
STALL = 0.004                            # the hand is this far behind its command and not moving: it is on the skin
LEAD_SKIN, LEAD_IN = 0.030, 0.008        # the most the command leads the hand: to pierce the skin, and once through
GAIN, BIAS_MAX = 0.25, 0.020             # how fast, and how far, a hand corrects for its own sag


def stick_frame(deg: float) -> np.ndarray:
    """The left hand's attitude with the needle pointing up the arm (+y) ``deg`` degrees below level."""
    t = math.radians(deg)
    return frame((0.0, -math.sin(t), -math.cos(t)), (-1.0, 0.0, 0.0))


def tool_for_tip(T, tip, rot) -> np.ndarray:
    """The left hand's tool point that puts the needle's tip at ``tip`` when the hand's attitude is ``rot``."""
    K = T.K
    n, up = rot[:, 1], -rot[:, 2]
    return np.asarray(tip, float) - (K.TIP_Y - GRIP_AT[0]) * n + GRIP_AT[1] * up


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


def settle(sag, seconds: float, rot):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        sag.to(sag.want, rot)
        yield


HOME: dict = {}


def go_home(hand, seconds: float = 2.5):
    from yam_arena.constants import READY

    pos, rot = HOME[hand.name]
    yield from hand.move(pos, seconds, rot=rot, grip=1.0)
    yield from hand.move_joints(np.array(READY, float), 0.5, grip=1.0)


def anchor_point(T) -> np.ndarray:
    """The right hand's tool point with its closed fingertips pressed on the forearm's skin over the vein."""
    K = T.K
    skin = T.arm_pos() + (0.0, -BELOW, K.fore_top(-BELOW) + K.PATCH_GAP + K.PATCH_T)
    return skin - R_ANCHOR[:, 2] * (0.013 - PRESS)


def anchor(T, right, draw: bool = True):
    """Press on the skin below the site and (unless ``draw`` is False) draw it toward the wrist."""
    p = anchor_point(T)
    yield from right.move(p + (0.0, -0.040, 0.090), 2.5, rot=R_ANCHOR, grip=0.0)
    yield from right.move(p + (0.0, 0.0, 0.030), 1.2, rot=R_ANCHOR)
    yield from right.move(p, 1.0, rot=R_ANCHOR)
    if draw:                             # toward the wrist until the hand itself has moved DRAW, then hold that command
        d, start, lead = T.K.patch_axis(), TOOL["right"].copy(), 0.0
        for _ in range(160):
            moved = float((TOOL["right"] - start) @ d)
            if moved >= DRAW:
                break
            lead = min(lead + 0.0005, moved + DRAW_LEAD)
            right._solve(p + lead * d, R_ANCHOR)
            yield


def push_in(T, left, bias, rot, deg: float, goal: float, note: dict):
    """Generator: the stick. The hand is commanded along the needle's line; ``a`` is how far its own measured tool
    point has gone along that line (0 where the layout puts the skin). It goes forward until it stalls (the point is
    on the skin, at ``a0``), pushes until it has moved on past the skin's thickness (the skin has given), eases off,
    goes on to ``goal`` past ``a0``, then sets its command to where it is and stops."""
    K = T.K
    n = rot[:, 1]
    skin = T.arm_pos() + (0.0, 0.0, K.SKIN_H)
    base = tool_for_tip(T, skin, rot)
    meas = lambda: float((TOOL["left"] - base) @ n)                     # noqa: E731
    cmd = lambda a: left._solve(base + a * n + bias, rot)               # noqa: E731
    a_cmd, hist, a0 = -SHORT, [], None
    for _ in range(600):                                 # 1: forward until the hand stalls on the skin
        a = meas()
        hist = (hist + [a])[-5:]
        if len(hist) == 5 and a_cmd - a > STALL and hist[-1] - hist[0] < 0.0004:
            a0 = a
            break
        a_cmd = min(a_cmd + APPROACH, a + LEAD_SKIN)
        cmd(a_cmd)
        yield
    if a0 is None:
        return
    pierce = K.IN_SKIN / math.sin(math.radians(deg))     # the skin's thickness along the needle
    for _ in range(400):                                 # 2: push until the skin gives
        a = meas()
        if a - a0 >= pierce + 0.0005:
            break
        a_cmd = min(a_cmd + APPROACH, a + LEAD_SKIN)
        cmd(a_cmd)
        yield
    a_cmd = meas() + 0.001                               # ease off at once
    cmd(a_cmd)
    yield
    for _ in range(600):                                 # 3: on to the set distance past the skin
        a = meas()
        if a - a0 >= goal - 0.0003:
            break
        a_cmd = min(a_cmd + CREEP, a + LEAD_IN)
        cmd(a_cmd)
        yield
    cmd(meas())                                          # 4: the command goes to where the hand is, and stays
    note.update(a0_mm=round(1000 * a0, 2), end_mm=round(1000 * meas(), 2))
    yield


def take(T, left):
    """The left hand takes the holder out of its cradle and lifts it. It corrects for its own sag before it closes, so
    that the holder sits in the fingers where the plan has it."""
    c = T.holder_start() + (0.0, GRIP_AT[0], GRIP_AT[1])
    yield from left.move(c + (0.0, 0.0, 0.090), 2.5, rot=R_DOWN, grip=gap(OPEN))
    ls = Sag(left)
    yield from glide(ls, c + (0.0, 0.0, 0.090), 0.3, R_DOWN)
    yield from glide(ls, c, 1.5, R_DOWN)
    yield from settle(ls, 0.8, R_DOWN)
    yield from left.set_grip(gap(SQUEEZE), 0.8)
    yield from wait(0.3)
    yield from left.move(left.pos + (0.0, 0.0, 0.100), 1.5, rot=R_DOWN)


def script(T, left, right, mode: str = "full", idle_s: float = 30.0):
    """The demo as a generator: one yield per control step."""
    if mode == "still":
        yield from wait(idle_s)
        return
    K = T.K
    HOME.update({h.name: (h.pos.copy(), h.rot.copy()) for h in (left, right)})
    deg = 45.0 if mode == "steep" else ANGLE
    rot = stick_frame(deg)
    n = rot[:, 1]
    skin = T.arm_pos() + (0.0, 0.0, K.SKIN_H)                 # where the needle is to cross the skin
    goal = (0.018 if mode == "through" else AIM / math.sin(math.radians(deg)))
    yield from together(take(T, left), anchor(T, right, draw=mode != "notraction"))
    # over to the arm, high, turning the needle down the arm; then down to 2 cm short of the skin on the needle's line
    yield from left.move(tool_for_tip(T, skin - 0.035 * n + (0.0, 0.0, 0.050), rot), 3.0, rot=rot)
    ls = Sag(left)
    yield from glide(ls, tool_for_tip(T, skin - 0.035 * n + (0.0, 0.0, 0.050), rot), 0.3, rot)
    yield from glide(ls, tool_for_tip(T, skin - SHORT * n, rot), 2.0, rot)
    yield from settle(ls, 0.8, rot)
    yield from push_in(T, left, ls.bias.copy(), rot, deg, goal, NOTE)        # in along the needle's line, by feel
    yield from wait(0.5)
    if mode == "release":                # control: the skin is let go as soon as the needle is in the vein
        p = anchor_point(T)
        yield from right.move(p + (0.0, -0.040, 0.090), 1.0, rot=R_ANCHOR)
    elif mode == "letgo":                # control: the holder is let go once the needle is in the vein
        yield from left.set_grip(gap(OPEN), 0.6)
        yield from left.move(left.pos + (0.0, 0.0, 0.060), 1.2, rot=rot)
    # from here the left hand's command is left alone, so that the needle stays still
    yield from wait(idle_s)


NOTE: dict = {}                          # what the stick felt (for the log)


def _write_mp4(frames, path: Path) -> None:
    import imageio

    with imageio.get_writer(path, fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=None,
                            ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]) as w:
        for f in frames:
            w.append_data(f)


def run(task_path: str, out_dir: str, max_steps: int = 800, record: bool = True, window: bool = False,
        hold_s: float = 2.5, mode: str = "full", every: int = 5, cam: tuple = (320, 176), top_cam: str | None = None) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam[0],
                               cam_height=cam[1], headless=not window, **({'top_cam': top_cam} if top_cam else {}))
    T = emb.task.module
    obs = emb.reset(Scene(id="needle-stick-demo", instruction=T.INSTRUCTION))
    left, right = Hand("left", station.BASES["left"]), Hand("right", station.BASES["right"])
    gen = script(T, left, right, mode, idle_s=max_steps / CONTROL_HZ)
    frames: dict[str, list] = {}
    trace = []
    names = [T.HOLDER, T.PATCH]

    def snap_frames(o) -> None:
        if record:
            for name, img in o.images.items():
                frames.setdefault(name, []).append(np.asarray(img))

    def states():
        from yam_arena.scene import object_states   # importable only once the simulator has started

        return object_states(emb._env, names)

    def measure() -> None:
        from veinkit import finger_points

        TOOL["left"], TOOL["right"] = (np.asarray(p, float) for p in finger_points(emb._env))

    def hands() -> dict:
        from veinkit import _np

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
              "start": start, "end": {"task": T.report(states()), "places": places(states())}, "felt": dict(NOTE),
              "trace": trace}
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
    ap.add_argument("--max-steps", type=int, default=800)
    ap.add_argument("--window", action="store_true", help="show the Isaac Sim window on the instance's desktop")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--mode", default="full", choices=("full", "notraction", "release", "letgo", "steep", "through", "still"))
    ap.add_argument("--trace-every", type=int, default=5, help="steps between entries of the trace in result.json")
    ap.add_argument("--cam", default="320x176", help="recorded frame size, e.g. 640x352")
    ap.add_argument("--top-cam", default=None, choices=("calibrated", "demo"), help="the station's overhead camera setting")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, mode=a.mode, every=a.trace_every,
        cam=tuple(int(v) for v in a.cam.split("x")), top_cam=a.top_cam)
