"""Demo for the footplates task, run in the yam_arena embodiment.

    python demo.py task.py out_dir [--window] [--max-steps 1800] [--no-record] [--mode full|one|held|drop|plate|still]

For each foot, the manikin's right one first:

1. The arm on the other side reaches across with a closed, level hand, fingers pointing away from the robot, and
   brings it up under the ball of the foot from in front of the toes. The arm on the foot's own side goes to stand
   outside the upright footplate, leaning 60 degrees forward.
2. The supporting hand lifts: it follows the sole as the lower leg swings 50 degrees forward about the knee. The leg
   is not gripped; the sole lies on the fingers, and the knee's hinge keeps the leg in its plane.
3. The other hand pushes the plate's upper part 7 cm toward the chair's middle. Past upright the plate falls onto its
   down stop, under the lifted leg, and the hand draws back.
4. The supporting hand lowers the leg the same way. The heel lands on the plate at about 14 degrees; the hand goes
   on 3 degrees further, clear of the sole, draws out toward the robot from under the toes, and both arms go back.

Controls: ``--mode one`` does the first foot only; ``--mode held`` lifts the first leg, tips the plate and leaves the
leg held up; ``--mode drop`` lifts the first leg and takes the hand away, so that the leg falls; ``--mode plate`` tips
the first plate over while the foot still hangs in its way; ``--mode still`` keeps both arms still.

Every target comes from the layout constants in the task file and the sizes in ``footkit.py``. The only feedback is
each hand's own measured position (the robot's proprioception), used to correct for the arm's sag under the leg's
weight. The demo never reads a part's state.
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

R_LEVEL = frame((1.0, 0.0, 0.0), (0.0, -1.0, 0.0))        # a level hand pointing away from the robot, fingers side by side
LEAN = math.radians(60.0)
R_PUSH = frame((math.sin(LEAN), 0.0, -math.cos(LEAN)), (0.0, -1.0, 0.0))   # the pushing hand leans 60 degrees forward
# The closed fingers' upper edge in a level hand, from the robot's collision shapes: metres forward of the tool point,
# metres above it. The fingers taper to their tips; behind them the palm is level with their roots.
PROFILE = ((0.013, 0.0062), (0.010, 0.0077), (0.000, 0.0137), (-0.020, 0.0199), (-0.040, 0.0295), (-0.060, 0.0340),
           (-0.081, 0.0340), (-0.144, 0.0330))
TIP_AT = 0.125                           # the fingertips go under the sole this far forward of the shank's axis
LIFT = 50.0                              # degrees the lower leg is lifted forward while its plate comes down
LOWER_TO = 3.0                           # the hand goes on down to this far below where the foot comes to rest (degrees)
PUSH_UP = 0.110                          # the plate is pushed this far above its hinge line
PUSH_FROM, PUSH_TO = 0.050, -0.022       # the pushing hand's tool point, out from the hinge line: start and end
GAIN, BIAS_MAX = 0.25, 0.040             # how fast, and how far, a hand corrects for its own sag


def hand_under(T, side: float, deg: float, tip_at: float = TIP_AT, gap: float = 0.0) -> np.ndarray:
    """The tool point of a level hand whose closed fingers lie against the sole of a leg lifted ``deg`` degrees, their
    tips ``tip_at`` m forward of the shank's axis (``gap`` m short of touching)."""
    K = T.K
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    knee = T.knee_world(side)
    origin = knee + np.array([-K.SOLE_DOWN * s, 0.0, -K.SOLE_DOWN * c])     # where the shank's axis meets the sole
    along = np.array([-c, 0.0, s])                                          # along the sole toward the toes
    into = np.array([s, 0.0, c])                                            # from the sole up into the foot
    prof = np.array([[x, 0.0, z] for x, z in PROFILE])
    tcp = origin + tip_at * along - prof[0]
    pts = tcp + prof - origin
    under = (pts @ along >= -K.HEEL_BACK) & (pts @ along <= K.FOOT_L - K.HEEL_BACK)
    return tcp - (float((pts @ into)[under].max()) + gap) * into


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


def sweep(T, sag, side: float, a0: float, a1: float, seconds: float):
    """Generator: the supporting hand follows the sole as the leg goes from ``a0`` to ``a1`` degrees of lift."""
    from yam_arena import ik

    n = max(1, round(seconds * CONTROL_HZ))
    for k in range(1, n + 1):
        sag.to(hand_under(T, side, a0 + (a1 - a0) * ik.smoothstep(k / n)), R_LEVEL)
        yield


def push_point(T, side: float, out: float) -> np.ndarray:
    """The pushing hand's tool point: ``out`` m outside the plate's hinge line, toward the front of the plate."""
    h = T.hinge_world(side)
    return np.array([h[0] - 0.030, h[1] + side * out, h[2] + PUSH_UP])


HOME: dict = {}                          # each hand's tool point and attitude at the start


def go_home(hand, seconds: float = 2.5):
    """Generator: straight back to where the hand started, then exactly to its starting joint angles."""
    from yam_arena.constants import READY

    pos, rot = HOME[hand.name]
    yield from hand.move(pos, seconds, rot=rot, grip=1.0)
    yield from hand.move_joints(np.array(READY, float), 0.5, grip=1.0)


def one_side(T, name: str, holder, pusher, steps: str = "all"):
    """One foot: ``holder`` (the arm on the other side) lifts and holds the leg from under the foot, ``pusher`` (the
    arm on this side) tips the plate over from outside, the foot is lowered onto the plate, both arms go back."""
    side = T.SIDES[name]
    rest = math.degrees(T.K.rest_lift())
    # The pushing hand stands by outside the upright plate. The holding hand crosses over in front of both feet, well
    # short of the toes, turning level as it goes, and then comes in low under the toes. Every move is a straight line.
    stand = push_point(T, side, PUSH_FROM + 0.030)
    start = hand_under(T, side, 0.0, gap=0.012)
    first = start + (-0.110, 0.0, -0.020)
    cross = np.array([first[0] - 0.090, first[1], first[2] + 0.040])
    moves = [holder.move(cross, 3.0, rot=R_LEVEL, grip=0.0)]
    if steps != "drop":
        moves.append(pusher.move(stand, 3.0, rot=R_PUSH, grip=0.0))
    yield from together(*moves)
    hs = Sag(holder)
    yield from glide(hs, cross, 0.3, R_LEVEL)
    yield from glide(hs, first, 1.0, R_LEVEL)
    yield from glide(hs, start, 1.5, R_LEVEL)
    yield from glide(hs, hand_under(T, side, 0.0), 0.6, R_LEVEL)
    yield from sweep(T, hs, side, 0.0, LIFT, 4.5)                    # lift the leg forward
    yield from wait(0.6)
    if steps == "drop":                                              # control: take the hand away and let the leg fall
        yield from glide(hs, hs.want + (-0.100, 0.0, -0.060), 0.5, R_LEVEL)
        yield from wait(3.0)
        return
    ps = Sag(pusher)
    yield from glide(ps, push_point(T, side, PUSH_FROM), 0.5, R_PUSH)
    yield from glide(ps, push_point(T, side, PUSH_TO), 1.4, R_PUSH)  # tip the plate past upright: it falls onto its stop
    yield from glide(ps, push_point(T, side, PUSH_FROM + 0.030), 0.8, R_PUSH)
    yield from wait(1.0)
    if steps == "held":                                              # control: the leg is left held up
        yield from go_home(pusher)
        return
    yield from together(sweep(T, hs, side, LIFT, rest - LOWER_TO, 4.5), go_home(pusher))
    yield from wait(0.4)
    yield from glide(hs, hs.want + (-0.110, 0.0, 0.0), 1.2, R_LEVEL)         # out from under the toes, toward the robot
    yield from glide(hs, cross, 1.0, R_LEVEL)
    yield from go_home(holder)


def script(T, left, right, mode: str = "full", idle_s: float = 60.0):
    """The demo as a generator: one yield per control step."""
    if mode == "still":
        yield from wait(idle_s)
        return
    HOME.update({h.name: (h.pos.copy(), h.rot.copy()) for h in (left, right)})
    if mode == "look":
        # not a trial: the right arm is raised so that its wrist camera looks at the whole set-up, for the setup picture
        for pos, up in (((0.06, -0.16, 0.46), 22.0), ((0.08, -0.02, 0.45), 28.0), ((0.12, -0.10, 0.44), 15.0)):
            a = math.radians(up)
            yield from right.move(np.array(pos) + (0.0, 0.0, T.TOP), 3.0, rot=frame((math.cos(a), 0.0, math.sin(a)), (0.0, -1.0, 0.0)))
            yield from wait(1.0)
        return
    if mode == "plate":
        # control: the plate is tipped over while the foot still hangs in its way
        side = T.SIDES["right"]
        yield from left.move(push_point(T, side, PUSH_FROM + 0.030), 3.0, rot=R_PUSH, grip=0.0)
        ps = Sag(left)
        yield from glide(ps, push_point(T, side, PUSH_FROM), 0.5, R_PUSH)
        yield from glide(ps, push_point(T, side, PUSH_TO), 1.4, R_PUSH)
        yield from glide(ps, push_point(T, side, PUSH_FROM + 0.030), 0.8, R_PUSH)
        yield from go_home(left)
        yield from wait(3.0)
        return
    # the manikin's right foot (on the robot's left): the right arm reaches across and holds, the left arm tips the plate
    yield from one_side(T, "right", right, left, {"held": "held", "drop": "drop"}.get(mode, "all"))
    if mode in ("one", "held", "drop"):
        yield from wait(3.0)
        return
    yield from one_side(T, "left", left, right)
    yield from wait(3.0)


def _write_mp4(frames, path: Path) -> None:
    import imageio

    with imageio.get_writer(path, fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=None,
                            ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]) as w:
        for f in frames:
            w.append_data(f)


def run(task_path: str, out_dir: str, max_steps: int = 1800, record: bool = True, window: bool = False,
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
    obs = emb.reset(Scene(id="footplates-demo", instruction=T.INSTRUCTION))
    left, right = Hand("left", station.BASES["left"]), Hand("right", station.BASES["right"])
    gen = script(T, left, right, mode, idle_s=max_steps / CONTROL_HZ)
    frames: dict[str, list] = {}
    trace = []
    names = [*T.LEGS.values(), *T.PLATES.values()]

    def snap_frames(o) -> None:
        if record:
            for name, img in o.images.items():
                frames.setdefault(name, []).append(np.asarray(img))

    def states():
        from yam_arena.scene import object_states   # importable only once the simulator has started

        return object_states(emb._env, names)

    def measure() -> None:
        from footkit import finger_points

        TOOL["left"], TOOL["right"] = (np.asarray(p, float) for p in finger_points(emb._env))

    def hands() -> dict:
        from footkit import _np

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
    ap.add_argument("--max-steps", type=int, default=1800)
    ap.add_argument("--window", action="store_true", help="show the Isaac Sim window on the instance's desktop")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--mode", default="full", choices=("full", "one", "held", "drop", "plate", "still", "look"))
    ap.add_argument("--trace-every", type=int, default=5, help="steps between entries of the trace in result.json")
    ap.add_argument("--cam", default="320x176", help="recorded frame size, e.g. 640x352")
    ap.add_argument("--top-cam", default=None, choices=("calibrated", "demo"), help="the station's overhead camera setting")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, mode=a.mode, every=a.trace_every,
        cam=tuple(int(v) for v in a.cam.split("x")), top_cam=a.top_cam)
