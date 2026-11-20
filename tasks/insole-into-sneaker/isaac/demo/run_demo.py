"""Scripted demo and cheap strategies for the insole task (one stage per Kit process).

    python run_demo.py STAGE OUT [--max-steps N]
    STAGE: demo | idle | shot | nobend | nopress | short | tongue_free (the demo with the left hand idle: the tongue is left
    leaning back over the opening)

The arms move only through the 14 joint and gripper commands, planned open loop from the fixed start layout (geom.py)
with the YAM IK; every command is clipped to the official Inspect Robots guardrails (joint bounds, 4.5 % of each
joint's range per step) and recorded in OUT/actions.json, which demo/official_replay.py replays. The live score is
the task file's own scorer (task.STATE, reported by the embodiment as info["score"]). Simulator state is read only
for the log.
"""
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
TASK = HERE.parent / "task.py"
sys.path.insert(0, str(HERE.parent))
import geom  # noqa: E402

HZ = 20
S0 = np.array([geom.SHOE_XY[0], geom.SHOE_XY[1], geom.SHOE_Z])  # shoe frame origin (world, on the table), toe along +x
S = S0 + np.array([0.0, 0.0, geom.bed(0.0)])                       # the bed at the heel counter (world)


def bed_w(x):
    """World height of the bed at shoe-frame x."""
    return S0[2] + geom.bed(min(max(x, 0.0), geom.LI))


class Arm:
    def __init__(self, name, base):
        from yam_arena import ik
        from yam_arena.constants import READY
        self.name, self.base = name, np.asarray(base, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = 0.0

    def _solve(self, pos, rot):
        from yam_arena import ik
        q, pe, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=80)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, rot, dur, grip=None):
        from yam_arena import ik
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(dur * HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos, float) - p0), ik.slerp_matrix(r0, rot, s))
            yield

    def set_grip(self, g, dur=1.0):
        yield from self.move(self.pos, self.rot, dur, g)

    def move_joints(self, q1, dur, grip=None):
        """Joint-space move (to READY at the end)."""
        from yam_arena import ik
        q0, g0 = self.q.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(dur * HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q = q0 + s * (np.asarray(q1, float) - q0)
            self.grip = g0 + s * (g1 - g0)
            self.cmd = np.r_[self.q, self.grip]
            yield
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)


def par(*gens):
    """Run several arm motions at the same time, one control step each per step."""
    gens = list(gens)
    while gens:
        for g in list(gens):
            try:
                next(g)
            except StopIteration:
                gens.remove(g)
        if gens:
            yield


def wait(sec):
    for _ in range(max(1, round(sec * HZ))):
        yield


def hand(pitch):
    """Fingers pointing along +x tilted down by ``pitch``; the jaws close through the insole's thickness."""
    c, s = math.cos(pitch), math.sin(pitch)
    z = np.array([c, 0.0, -s])
    x = np.array([s, 0.0, c])
    return np.column_stack([x, np.cross(z, x), z])


DOWN = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)     # fingers pointing down


def pick_and_hang(R):
    """Pinch the heel end where it overhangs the shoe box lid, lift it and hang the insole over the shoe."""
    heel, _ = geom.insole_start()
    grab = np.array([heel[0] + 0.012, heel[1], heel[2]])
    yield from R.move(grab + [-0.08, 0, 0.0], hand(0.0), 2.5, grip=1.0)
    yield from R.move(grab, hand(0.0), 1.5)
    yield from R.set_grip(0.0, 1.2)
    yield from wait(0.5)
    yield from R.move(grab + [0, 0, 0.06], hand(0.0), 1.5)
    yield from R.move(grab + [-0.02, 0, 0.30], hand(0.0), 2.5)
    yield from R.move(np.array([S[0] - 0.16, S[1], S[2] + 0.30]), hand(0.0), 3.0)
    yield from wait(2.0)


def enter_and_push(R, ctx, p0, path):
    """Toe down into the opening behind the throat at a shallow pitch, then forward and down: the toe slides along
    the bed under the vamp while the insole bends over the heel counter's top edge."""
    yield from R.move(np.array([S[0] - 0.14, S[1], S[2] + 0.13]), hand(math.radians(p0)), 2.5)
    yield from wait(1.0)
    ctx.mark("toe_in_opening")
    for k, (dx, dz, dp) in enumerate(path):
        yield from R.move(np.array([S[0] + dx, S[1], S[2] + dz]), hand(math.radians(p0 + dp)), 1.5)
        yield from wait(0.5)
        ctx.mark(f"slide{k}")


def release_and_back(R, ctx):
    """Open fully where the hand is (lifting a half-open hand drags the heel end up and pries the toe into the floor),
    let the heel end drop, then lift straight up out of the opening (moving back would hook the heel counter)."""
    slow = float(os.environ.get("INSOLE_RELEASE_SLOW", "1"))   # (env: diagnostics)
    yield from R.set_grip(1.0, 1.0 * slow)
    yield from wait(0.5 * slow)
    yield from R.move(R.pos + [0.0, 0, 0.10], R.rot, 1.5 * slow)
    yield from wait(0.5)
    ctx.mark("released")


def press(R, ctx, plan):
    """Seat the insole: closed fingertips press on the arch and stroke back toward the heel, which pushes any hump
    back into the heel (as a person smooths an insole flat), and press the heel end down. Planned from where the
    shoe stands now."""
    at = ctx.shoe_offset()   # where the shoe stands now (its pose, as a camera sees it)
    tipw = lambda x, dz, rot: at(np.array([S[0] + x, S[1], bed_w(x) + dz])) - TIP * rot[:, 2]
    for k, step in enumerate(plan):
        if step[0] == "stroke":
            _, x0, x1 = step
            rot = hand(math.radians(65))
            yield from R.move(tipw(x0, 0.06, rot), rot, 1.5, grip=0.0)
            yield from R.move(tipw(x0, 0.004, rot), rot, 1.0)
            yield from R.move(tipw(x1, 0.004, rot), rot, 2.0)
            ctx.mark(f"stroke{k}")
            yield from R.move(tipw(x1, 0.06, rot), rot, 1.0)
        else:
            _, x0 = step
            yield from R.move(at(np.array([S[0] + x0, S[1], bed_w(x0) + 0.09])), DOWN, 1.5, grip=0.0)
            yield from R.move(at(np.array([S[0] + x0, S[1], bed_w(x0) + 0.018])), DOWN, 1.2)
            yield from wait(0.5)
            ctx.mark(f"press{k}")
            yield from R.move(at(np.array([S[0] + x0, S[1], bed_w(x0) + 0.09])), DOWN, 1.0)


def hand_l(pitch_deg=50.0, yaw_deg=-30.0):
    """Left hand: fingers pointing forward and down, turned toward the shoe's centre line."""
    cy, sy = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ hand(math.radians(pitch_deg))


TIP = 0.013   # the fingertips are 13 mm beyond the TCP (the centre of the finger pads) along the approach axis
# fingertip waypoints in the shoe frame (x, z above the table) on the centre line: above and behind the tongue, on its
# back, over its root (upright), and holding it folded forward on the laced vamp
TONGUE_PATH = ((0.040, 0.180), (0.092, 0.117), (0.135, 0.133), (0.176, 0.092))


def tongue_forward(Lh, ctx):
    """Left hand, closed fingers: from behind, push the tongue forward over the laces and hold it there."""
    rot = hand_l()
    tcp = lambda x, z: np.array([S0[0] + x, S0[1], S0[2] + z]) - TIP * rot[:, 2]
    (ax, az), (bx, bz), (cx, cz), (hx, hz) = TONGUE_PATH
    yield from Lh.move(tcp(ax, az + 0.05), rot, 3.0, grip=0.0)
    yield from Lh.move(tcp(ax, az), rot, 1.0)
    yield from Lh.move(tcp(bx, bz), rot, 1.5)
    yield from Lh.move(tcp(cx, cz), rot, 1.5)
    yield from Lh.move(tcp(hx, hz), rot, 1.5)
    ctx.mark("tongue_held")


def clear(R, ctx, Lh=None):
    """Let go of the tongue (it springs back), then both arms go to READY."""
    from yam_arena.constants import READY
    if Lh is not None:   # lift off the tongue and away to the left
        yield from Lh.move(Lh.pos + [0.0, 0.0, 0.06], Lh.rot, 1.0)
        yield from Lh.move(Lh.pos + [-0.02, 0.10, 0.04], Lh.rot, 1.0)
    yield from R.move(R.pos + [0.0, 0.0, max(0.0, S[2] + 0.22 - R.pos[2])], R.rot, 1.5)   # straight up, clear of it
    yield from par(R.move_joints(READY, 3.0, grip=1.0), *( [Lh.move_joints(READY, 3.0, grip=1.0)] if Lh else []))
    ctx.mark("clear")
    yield from wait(3.0)


PATH = [(-0.10, 0.115, 0), (-0.06, 0.100, 5), (-0.03, 0.090, 8), (0.0, 0.085, 10), (0.02, 0.080, 12)]
PATH = PATH + [(0.04, 0.075, 14), (0.06, 0.070, 16)]   # push on past toe-home: the toe stops at the toe box
PRESS = (("press", 0.015), ("press", 0.05), ("press", 0.015))   # heel, near the arch, heel


def script(stage, ctx):
    R = ctx.right
    Lh = None if stage == "tongue_free" else ctx.left
    p0 = float(os.environ.get("INSOLE_P0", "20"))
    if stage == "idle":
        yield from wait(float(os.environ.get("IDLE_S", "30")))
        return
    if stage == "shot":       # setup screenshot: the start layout averaged over 2 s (render noise), then a close-up
        ctx.phase = "ready"   # of the shoe from the left wrist camera held above it (photo only, not a trial)
        yield from wait(2.0)
        ctx.phase = None
        rot = hand_l(70.0, -20.0)
        yield from ctx.left.move(np.array([S0[0] + 0.02, S0[1] + 0.06, S0[2] + 0.30]), rot, 3.0, grip=1.0)
        yield from wait(1.0)
        ctx.phase = "close"
        yield from wait(2.0)
        ctx.phase = None
        # side view: the left wrist camera level with the shoe, looking across it toward the robot's right
        yield from ctx.left.move(np.array([S0[0] + 0.12, S0[1] + 0.26, S0[2] + 0.14]), hand_l(15.0, -90.0), 3.0)
        yield from wait(1.0)
        ctx.phase = "side"
        yield from wait(2.0)
        ctx.phase = None
        return
    yield from par(pick_and_hang(R), *([tongue_forward(Lh, ctx)] if Lh else []))
    ctx.mark("hanging_over_shoe")
    if stage == "nobend":
        # cheap strategy: keep the insole level and lay it straight down over the opening, let go
        yield from R.move(np.array([S[0] - 0.012, S[1], S[2] + 0.12]), hand(0.0), 3.0)
        yield from R.move(np.array([S[0] - 0.012, S[1], S[2] + 0.075]), hand(0.0), 2.0)
        yield from wait(1.0)
        ctx.mark("laid")
        yield from release_and_back(R, ctx)
        yield from clear(R, ctx, Lh)
        return
    if stage == "short":
        # cheap strategy: stop the push halfway (toe in the opening, not run home), let go, press
        yield from enter_and_push(R, ctx, p0, PATH[:2])  # short: 2 of 7 push points
        yield from release_and_back(R, ctx)
        yield from press(R, ctx, PRESS)
        yield from clear(R, ctx, Lh)
        return
    yield from enter_and_push(R, ctx, p0, PATH)
    if stage == "nopress":
        # push it home, let go and leave, without pressing it down into the shoe
        yield from release_and_back(R, ctx)
        yield from clear(R, ctx, Lh)
        return
    yield from release_and_back(R, ctx)
    yield from press(R, ctx, PRESS)
    yield from clear(R, ctx, Lh)


class Ctx:
    def __init__(self, emb):
        from yam_arena import station
        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])
        self.steps = 0
        self.phase = None
        self.sums = {}
        self.marks = {}
        self.actions = []

    def mark(self, tag):
        T = self.emb.task.module  # the task module yam_arena loaded (its live STATE and scorer)
        m = T.measure(self.emb._env)
        row = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in m.items()}
        row.update(step=self.steps, score=T.insole_scoring.score(m)[0], tcp=np.round(self.right.pos, 4).tolist(),
                   ik_mm=round(self.right.worst_mm, 1), tcp_left=np.round(self.left.pos, 4).tolist(),
                   ik_left_mm=round(self.left.worst_mm, 1))
        self.left.worst_mm = 0.0
        st = T.insolekit.insole_state(self.emb._env, "insole")
        sp, sq, _ = T.partkit.pose(self.emb._env, "shoe")
        row["line_local"] = np.round((st.points - sp) @ T._rot(sq), 4).tolist()
        if "tongue" in T.insolekit.FLAPS and os.environ.get("INSOLE_NO_TONGUE") != "1":
            tp, _, tip = T.insolekit.flap_state(self.emb._env, "tongue")
            row["tongue_local"] = np.round((np.vstack([tp, tip]) - sp) @ T._rot(sq), 4).tolist()
        self.right.worst_mm = 0.0
        self.marks[tag] = row
        print("MARK", tag, json.dumps(row), flush=True)

    def shoe_offset(self):
        """Where the shoe stands now (its pose, as a camera sees it): a function that maps a world point planned for
        the start pose to the same point on the shoe now."""
        T = self.emb.task.module
        sp, sq, _ = T.partkit.pose(self.emb._env, "shoe")
        R = T._rot(sq)
        print("SHOE_POSE", np.round(sp, 4).tolist(), "yaw_deg", round(math.degrees(math.atan2(R[1, 0], R[0, 0])), 2),
              flush=True)
        return lambda p: np.asarray(sp, float) + R @ (np.asarray(p, float) - S0)

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack
        from yam_arena import ik
        lo, hi = np.append(ik.LOWER, 0.0), np.append(ik.UPPER, 1.0)
        cmd = np.r_[np.clip(self.left.cmd, lo, hi), np.clip(self.right.cmd, lo, hi)]
        if self.actions:  # official CLI guardrails: bounds and 4.5 % of each range per step, so a replay is exact
            lim = 0.045 * np.r_[hi - lo, hi - lo]
            prev = np.asarray(self.actions[-1], float)
            cmd = np.clip(cmd, prev - lim, prev + lim)
        data = pack(cmd[:7], cmd[7:])
        self.actions.append(np.asarray(data, float).ravel().tolist())
        return Action(data=data)


def luma(img):
    a = np.asarray(img, float)
    return float((0.2126 * a[..., 0] + 0.7152 * a[..., 1] + 0.0722 * a[..., 2]).mean())


def run(stage, out, max_steps):
    import imageio
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    cw, ch = map(int, os.environ.get("INSOLE_CAM", "320x176").split("x"))   # camera size (setup shots: 1280x704)
    emb = yam_arena_embodiment(task=str(TASK), object_jitter=0, control_hz=HZ, cam_width=cw, cam_height=ch)
    T = emb.task.module
    obs = emb.reset(Scene(id="insole", instruction=T.INSTRUCTION))
    print("RESET_S", round(time.time() - t0, 1), "score", T.STATE["score"], T.STATE["why"], flush=True)
    ctx = Ctx(emb)
    ctx.mark("start")
    gen = script(stage, ctx)
    first = {c: np.asarray(i) for c, i in obs.images.items()}
    video, times, log = [], [], []
    res, done, first_full, status = None, False, None, "script_end"
    for step in range(max_steps):
        try:
            next(gen)
        except StopIteration:
            done = True
        if done:
            break
        ts = time.time()
        res = emb.step(ctx.action())
        times.append(time.time() - ts)
        ctx.steps = step + 1
        video.append(np.hstack([np.asarray(res.observation.images[c]) for c in sorted(res.observation.images)]))
        if ctx.phase:
            for c, im in res.observation.images.items():
                acc = ctx.sums.setdefault((ctx.phase, c), [0, 0.0])
                acc[0] += 1
                acc[1] = acc[1] + np.asarray(im, float)
        sc = float(res.info.get("score", 0.0))
        if sc >= 1.0 and first_full is None:
            first_full = ctx.steps
        if ctx.steps % 20 == 0:
            log.append({"step": ctx.steps, "score": sc, "why": T.STATE["why"],
                        **{k: (round(v, 4) if isinstance(v, float) else v) for k, v in (T.STATE["m"] or {}).items()}})
            print("LOG", json.dumps(log[-1]), flush=True)
        if res.terminated:
            status = res.termination_reason or "terminated"
            break
    else:
        status = "max_steps"
    ctx.mark("end")
    final = float(res.info.get("score", 0.0)) if res is not None else 0.0
    if res is not None and res.terminated and res.termination_reason == "success":
        final = 1.0
    last = {c: np.asarray(i) for c, i in res.observation.images.items()}
    frames = {}
    for tag, imgs in (("first", first), ("last", last)):
        img = np.hstack([imgs[c] for c in sorted(imgs)])
        imageio.imwrite(out / f"{tag}.png", img)
        for c in sorted(imgs):
            imageio.imwrite(out / f"{tag}_{c}.png", imgs[c])
        frames[tag] = {"luma": round(luma(img), 1), "cams": sorted(imgs)}
    for (ph, c), (n, tot) in ctx.sums.items():
        imageio.imwrite(out / f"shot_{ph}_{c}.png", np.clip(tot / n, 0, 255).astype(np.uint8))
    imageio.mimsave(out / "run.mp4", video, fps=HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
    (out / "actions.json").write_text(json.dumps({"control_hz": HZ, "stage": stage, "actions": ctx.actions}))
    tt = np.array(times[5:]) if len(times) > 10 else np.array(times)
    result = dict(stage=stage, steps=ctx.steps, status=status, score=final, why=T.STATE["why"],
                  first_full_step=first_full, frames=frames,
                  step_wall_s=dict(median=round(float(np.median(tt)), 4), p95=round(float(np.percentile(tt, 95)), 4)),
                  env={k: v for k, v in os.environ.items() if k.startswith("INSOLE")}, marks=ctx.marks)
    (out / "result.json").write_text(json.dumps(result, indent=1))
    (out / "log.json").write_text(json.dumps(log, indent=1))
    print("RESULT", json.dumps({k: v for k, v in result.items() if k != "marks"}), flush=True)
    print(f"SCORE {stage}: {final} ({ctx.steps} steps, {status})", flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("stage")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    a = ap.parse_args()
    try:
        run(a.stage, a.out, a.max_steps)
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        os._exit(1)
    os._exit(0)
