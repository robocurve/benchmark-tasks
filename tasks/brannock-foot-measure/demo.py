"""Closed-loop scripted demo (reads simulator state to plan; never moves objects or edits the simulation).

    python demo.py OUT_DIR [--layout K] [--max-steps N] [--cam 320x176] [--no-record]

Writes OUT_DIR/demo_<camera>.mp4, OUT_DIR/setup.png (first top-camera frame) and OUT_DIR/result.json. Run it
through the GPU wrapper (see README). The left hand grasps the ankle post, seats the heel in the cup and keeps
holding it; the right hand slides the width bar in until it stops on the foot, then the pointer forward until its
curve stops on the ball joint; then both hands let go and move away.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
HZ = 20


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


class Arm:
    """Cartesian moves of one arm's TCP (any orientation) by IK, warm-started from the last command."""

    def __init__(self, side):
        from yam_arena import station
        from yam_arena.constants import READY
        from yam_arena import ik

        self.ik = ik
        self.base = np.array(station.BASES[side])
        self.q = np.array(READY, float)
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.grip = 1.0  # 1 open, 0 closed
        self.worst_mm = 0.0

    def cmd(self):
        return np.r_[self.q, self.grip]

    def _solve(self, p, R):
        q, pe, _ = self.ik.solve(p, self.ik.matrix_to_quat_xyzw(R), self.q, base_pos=self.base, iters=60)
        self.q = q
        self.worst_mm = max(self.worst_mm, pe * 1000)

    def move(self, p, R=None, dur=1.0, grip=None):
        R = self.rot if R is None else R
        g0, g1 = self.grip, self.grip if grip is None else grip
        p0, R0 = self.pos.copy(), self.rot.copy()
        n = max(1, round(dur * HZ))
        for k in range(1, n + 1):
            s = self.ik.smoothstep(k / n)
            self._solve(p0 + s * (np.asarray(p) - p0), self.ik.slerp_matrix(R0, R, s))
            self.grip = g0 + s * (g1 - g0)
            yield
        self.pos, self.rot = np.asarray(p, float).copy(), R.copy()

    def set_grip(self, g, dur=0.7):
        yield from self.move(self.pos, self.rot, dur, grip=g)

    def step_to(self, p):
        """One control step straight to p (small increments)."""
        self._solve(np.asarray(p, float), self.rot)
        self.pos = np.asarray(p, float).copy()
        yield


def wait(s):
    for _ in range(max(1, round(s * HZ))):
        yield


def strategy(T, arms, log):
    import layout as LY
    import plan as PL
    import simhook as H
    from sizing import DEVICE as D

    L, Rr = arms["left"], arms["right"]
    st = T.states  # callable -> object states

    def form_pose():
        f = st()["foot_form"]
        return np.asarray(f.pos), np.asarray(f.quat_xyzw)

    def post_world(z_mm):
        from scoring import quat_matrix

        p, q = form_pose()
        pu, pv = T.GEO.form.post_center()
        return p + quat_matrix(q) @ (np.array([pu, pv, z_mm]) / 1000.0)

    def form_yaw_dev():
        from scoring import quat_matrix

        _, q = form_pose()
        ux = quat_matrix(q)[:, 0]  # form u axis in the world
        return math.atan2(ux[0], -ux[1])  # + toward medial (world +x)

    # -- left: grasp the post, seat the heel, keep holding --------------------------------------------------------
    RL = PL.grip_matrix(PL.YAW_L, PL.TILT_LEFT)
    back = -PL.APPROACH_BACK * RL[:, 2]
    g = post_world(PL.POST_GRIP_Z)
    yield from L.move(g + back + np.array([0, 0, 0.03]), RL, 2.0, grip=1.0)
    yield from L.move(g + back, RL, 0.8)
    yield from L.move(post_world(PL.POST_GRIP_Z), RL, 1.2)
    yield from L.set_grip(0.0, 0.8)
    for it in range(6):
        m = T.readings(st())
        psi = form_yaw_dev()
        du, dv = -(m["heel_gap_mm"] + 1.5), -m["heel_side_mm"]
        log.append(dict(step="seat", it=it, heel_gap=m["heel_gap_mm"], side=m["heel_side_mm"],
                        yaw_deg=math.degrees(psi)))
        if abs(du + 1.5) < 0.8 and abs(dv) < 0.4 and abs(psi) < math.radians(0.15) and it > 0:
            break
        target = L.pos + np.array([dv / 1000.0, -du / 1000.0, 0.0])  # device (u, v) -> world (-y, +x)
        yield from L.move(target, _rz(-psi) @ L.rot, 1.2 if it < 2 else 0.6)
        yield from wait(0.3)

    # the hand stays round the ankle post but eases its grip (fingers about 3 mm off each side): the heel stays
    # steadied in the cup while the forefoot can turn about it when the width bar brings it onto the guide
    yield from L.set_grip(0.70, 0.5)

    RR = PL.grip_matrix(PL.YAW, PL.TILT_RIGHT)

    def bar_fin(d):
        return LY.to_world(PL.BAR_FIN_U, D["guide_v"] - d - D["bar_t"] - PL.BAR_FIN_OUT, PL.BAR_FIN_Z[1] - PL.GRIP_BELOW_TOP)

    def ptr_fin(u):
        return LY.to_world(u, PL.PTR_FIN_V, PL.PTR_FIN_Z[1] - PL.GRIP_BELOW_TOP)

    def ball_joint_u():
        from scoring import to_device_pts

        p, q = form_pose()
        med = to_device_pts(T.GEO.medial, p, q)
        return float(med[med[:, 1].argmax(), 0])

    # -- right: width bar in until it rests on the foot and the ball joint is on the guide ----------------------------
    d0 = T.sliders_mm()[1]
    yield from Rr.move(bar_fin(d0) + np.array([0, 0, PL.HOVER / 1000.0]), RR, 2.0, grip=1.0)
    yield from Rr.move(bar_fin(d0), RR, 1.0)
    yield from Rr.set_grip(0.0, 0.8)
    dc, trail = d0, []
    while dc > D["bar_travel"][0] + 2:
        dc -= 0.3
        yield from Rr.step_to(bar_fin(dc))
        m = T.readings(st())
        trail.append((round(T.sliders_mm()[1], 1), round(m["bar_gap_mm"], 1), round(m["guide_gap_mm"], 1)))
        if (m["bar_gap_mm"] < 0.3 and m["guide_gap_mm"] < 0.4) or T.sliders_mm()[1] - dc > 8.0:
            break
    yield from wait(0.3)
    m = T.readings(st())
    log.append(dict(step="bar", d=round(T.sliders_mm()[1], 2), gap=round(m["bar_gap_mm"], 2),
                    guide_gap=round(m["guide_gap_mm"], 2), trail=trail[::10]))
    yield from Rr.set_grip(1.0, 0.6)
    yield from Rr.move(Rr.pos + np.array([0, 0, PL.HOVER / 1000.0]), RR, 0.8)

    # -- right: pointer forward until its curve sits on the ball joint (the widest point of the inner side) -------
    u0 = T.sliders_mm()[0]
    yield from Rr.move(ptr_fin(u0) + np.array([0, 0, PL.HOVER / 1000.0]), RR, 1.5)
    yield from Rr.move(ptr_fin(u0), RR, 1.0)
    yield from Rr.set_grip(0.0, 0.8)
    uc, trail = u0, []
    for _ in range(3):  # slide to the ball joint, then correct for the pointer's lag
        target = ball_joint_u()
        dist = target - T.sliders_mm()[0]
        uc += dist
        yield from Rr.move(ptr_fin(uc), RR, max(0.6, abs(dist) / 25.0))
        yield from wait(0.3)
        trail.append((round(target, 1), round(T.sliders_mm()[0], 1)))
        if abs(target - T.sliders_mm()[0]) < 0.4:
            break
    log.append(dict(step="pointer", trail=trail, nose_gap=round(T.readings(st())["nose_gap_mm"], 2)))
    yield from Rr.set_grip(1.0, 0.6)
    yield from Rr.move(Rr.pos + np.array([0, 0, PL.HOVER / 1000.0]), RR, 0.8)
    yield from Rr.move(np.array([0.26, -0.18, LY.Z0 + 0.12]), RR, 1.8)  # park well clear, near its own base

    # -- left lets go and moves away -------------------------------------------------------------------------------
    yield from L.set_grip(1.0, 0.8)
    yield from L.move(L.pos + back * 1.2, L.rot, 1.0)
    yield from L.move(np.array([0.26, 0.18, LY.Z0 + 0.12]), L.rot, 1.8)  # park well clear, near its own base
    m = T.readings(st())
    log.append(dict(step="released", **{k: (round(v, 3) if isinstance(v, float) else v) for k, v in m.items()}))
    yield from wait(3.0)


def write_mp4(frames, path):
    h, w = frames[0].shape[:2]
    p = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}",
                          "-r", str(HZ), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                          str(path)], stdin=subprocess.PIPE)
    for f in frames:
        p.stdin.write(np.ascontiguousarray(np.asarray(f)[..., :3], dtype=np.uint8).tobytes())
    p.stdin.close()
    p.wait()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--layout", type=int, default=None)
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--cam", default="320x176")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--diag", action="store_true")
    a = ap.parse_args()
    a.diag = a.diag or bool(os.environ.get("BRANNOCK_DIAG"))
    if a.layout is not None:
        os.environ["BRANNOCK_LAYOUT"] = str(a.layout)
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    w, h = (int(x) for x in a.cam.split("x"))
    task_path = str(HERE / "brannock_foot_measure.py")
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=HZ, cam_width=w, cam_height=h)
    obs = emb.reset(Scene(id="demo", instruction="demo"))
    T = emb.task.module
    from yam_arena.scene import object_states

    T.states = lambda: object_states(emb._env, emb.task.names)
    arms = {"left": Arm("left"), "right": Arm("right")}
    log: list = []
    gen = strategy(T, arms, log)
    frames: dict = {}

    def snap(o):
        if not a.no_record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, info, ended, success, trace, diag = 0, {}, None, False, [], []
    t0 = time.time()
    while steps < a.max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(Action(data=pack(arms["left"].cmd(), arms["right"].cmd())))
        steps += 1
        info = res.info
        trace.append(float(info.get("score", 0.0) or 0.0))
        if a.diag and (steps <= 150 and steps % 5 == 0 or steps % 25 == 0):
            import simhook as H

            m = T.S["last"] or {}
            jp = T.sliders_mm()
            diag.append(dict(step=steps, gap=round(m.get("heel_gap_mm", 0), 1), side=round(m.get("heel_side_mm", 0), 1),
                             tilt=round(m.get("tilt_deg", 0), 1), low=round(m.get("lowest_mm", 0), 1),
                             ptr=round(jp[0], 1), bar=round(jp[1], 1),
                             lat=T.S["latched"], tcpL=np.round(arms["left"].pos, 3).tolist()))
        snap(res.observation)
        if steps == 3 and os.environ.get("BRANNOCK_CLOSEUP"):
            import imageio
            import simhook as H

            imageio.imwrite(out / "closeup.png", H.closeup_image())
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None:
            break
    score = 1.0 if success else (trace[-1] if trace else 0.0)
    result = dict(layout=T.K, form=T.FORM["name"], task_score=score, success=success, steps=steps,
                  script_end_step=ended, wall_s=round(time.time() - t0, 1), worst_ik_mm=dict(
                      left=round(arms["left"].worst_mm, 2), right=round(arms["right"].worst_mm, 2)),
                  latched=T.S["latched"], faults=T.S["faults"], log=log,
                  final=({k: (round(v, 3) if isinstance(v, float) else v) for k, v in T.S["last"].items()}
                         if T.S["last"] else None),
                  score_trace_every_20=trace[::20], diag=diag)
    if not a.no_record:
        import imageio

        for cam, fs in frames.items():
            write_mp4(fs, out / f"demo_{cam}.mp4")
        imageio.imwrite(out / "setup.png", frames["top_cam"][0])
    (out / "result.json").write_text(json.dumps(result, indent=1, default=str))
    print(f"task_score: {score}", flush=True)
    print(json.dumps({k: result[k] for k in ("layout", "form", "task_score", "success", "steps", "worst_ik_mm",
                                               "faults")}), flush=True)
    os._exit(0)


if __name__ == "__main__":
    main()
