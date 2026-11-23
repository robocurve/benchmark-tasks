"""Closed-loop scripted demo (left arm only) for the cassette-to-cassette wafer transfer.

    TRIAL=k python demo.py OUT_DIR [--max-steps N] [--no-record]

It reads the simulator's state to plan (tweezer, wafer and gripper poses) and acts only through the gripper: the
fingers squeeze the tweezers' sprung leaves, and the tips hold the wafer by friction. Nothing is written to the
simulation. Phases:
  1 grasp the tweezers in their stand (fingers lowered along the tweezers' axis), set the tip gap, lift them out
  2 over the source wafer, tips open about 2 mm; down between its neighbours to 3 mm below its top edge, on the
    measured wafer position; squeeze until the tips grip
  3 lift straight up until the wafer clears the cassette top; carry it over the empty slot, on the measured wafer pose
  4 lower it into the slot, correcting sideways on the way, until it rests on the cassette's rails
  5 open the tips, rise 60 mm, put the tweezers back in the stand, let go, move clear
Writes demo_<cam>.mp4, setup.png and result.json (score, steps, events, phase log).
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
S_GRIP = 0.05  # fingers on the tweezers 50 mm from the tips
GAMMA = math.radians(32)  # tweezers held 32 deg off the gripper's approach axis (reach check)
GRIP_IN = float(os.environ.get("GRIP_IN", 0.005))  # tips 5 mm below the wafer's top edge (edge zone 2-5 mm)
GAP_GRASP = 0.0045  # tip gap while lifting the tweezers out of the stand
GAP_OPEN = 0.0020  # tip gap while going down between neighbours / releasing


def frame(u, s, gamma, sg):
    """Gripper rotation: closing axis (local x) s, approach (local z) gamma off -u (u = tips -> fingers)."""
    s = s / np.linalg.norm(s)
    u = u - s * (u @ s)
    u = u / np.linalg.norm(u)
    w = np.cross(s, u)
    z = -math.cos(gamma) * u + sg * math.sin(gamma) * w
    return np.column_stack([s, np.cross(z, s), z])


# tweezers vertical, leaves parted along y (the wafers' normal); the hand turned 180 deg about its approach
R_HOLD = frame(np.array([0, 0, 1.0]), np.array([0, 1.0, 0]), GAMMA, 1) @ np.diag([-1.0, -1.0, 1.0])


class PoseArm:
    """One arm: smooth Cartesian moves (IK on the URDF, warm-started every control step)."""

    def __init__(self, base):
        from yam_arena.constants import FINGER_OPEN, READY

        self.base = np.asarray(base, float)
        self.q = np.array(READY, float)
        self.f = FINGER_OPEN
        self.FO = FINGER_OPEN
        self.cmd = np.r_[self.q, 1.0]

    def pose(self):
        from yam_arena import ik

        return ik.tcp_pose(self.q, self.base)

    def move(self, pos, R=None, finger=None, dur=1.0):
        from yam_arena import ik

        p0, R0 = self.pose()
        R1 = R0 if R is None else R
        f0, f1 = self.f, (self.f if finger is None else finger)
        n = max(1, round(dur * HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            q, _, _ = ik.solve(p0 + (np.asarray(pos) - p0) * s, ik.matrix_to_quat_xyzw(ik.slerp_matrix(R0, R1, s)),
                               self.q, base_pos=self.base, iters=60)
            self.q = q
            f = f0 + (f1 - f0) * s
            self.cmd = np.r_[q, float(np.clip(f / self.FO, 0.0, 1.0))]
            yield
        self.f = f1

    def finger(self, f, dur=0.3):
        p, R = self.pose()
        yield from self.move(p, R, finger=f, dur=dur)


def wait(s):
    for _ in range(max(1, round(s * HZ))):
        yield


class Ctx:
    def __init__(self, emb):
        import artkit as ak

        self.emb, self.ak, self.T = emb, ak, emb.task.module

    def states(self):
        from yam_arena.scene import object_states

        return object_states(self.emb._env, self.emb.task.names)

    def tcp(self):
        from yam_arena.runtime import tcp_world

        robot = self.emb._env.unwrapped.scene["left_robot"]
        return np.asarray(tcp_world(robot), float) - self.ak._t(self.emb._env.unwrapped.scene.env_origins)[0]

    def tips_mid(self):
        a, b = self.T.tip_points()
        return (a + b) / 2

    def grip_point(self):
        import parts as P

        pts = []
        for h in ("half_a", "half_b"):
            p, q = self.ak.body_pose("tweezers", h)
            pts.append(p + self.T._rot(q) @ np.array(P.half_point(h, P.TW_HINGE_GAP / 2 + P.TW_BODY_T / 2, -(P.TW_LEN - S_GRIP))))
        return (pts[0] + pts[1]) / 2

    def tip_gap(self):
        import parts as P

        return P.tip_gap_from_hinge(self.ak.joints("tweezers")["hinge"])

    def wafer(self):
        w = self.states()["wafer"]
        return np.asarray(w.pos, float), w


def strategy(ctx, L, log):
    import parts as P

    T = ctx.T
    Z = np.array([0, 0, 1.0])
    R = R_HOLD

    def tcp_for_tips(tips):
        """TCP target putting the tips' midpoint at `tips`, from the measured tips offset in the TCP frame."""
        _, Rc = L.pose()
        return np.asarray(tips) - R @ (Rc.T @ (ctx.tips_mid() - ctx.tcp()))

    def set_gap(target, tol=0.0003, max_iter=40):
        for _ in range(max_iter):
            g = ctx.tip_gap()
            if abs(g - target) < tol:
                return
            yield from L.finger(min(0.0, L.f + float(np.clip((g - target) * 0.3, -0.0006, 0.0006))), dur=0.1)
            yield from wait(0.1)

    # 1 tweezers out of the stand
    yield from wait(0.5)
    gp = ctx.grip_point()
    yield from L.move(gp + Z * 0.08, R, finger=L.FO, dur=2.5)
    yield from L.move(gp, R, dur=1.5)
    yield from L.finger(-0.006, dur=0.6)
    yield from set_gap(GAP_GRASP)
    yield from wait(0.3)
    p, _ = L.pose()
    yield from L.move(p + Z * 0.07, R, dur=1.5)
    log["gap_after_grasp_mm"] = round(ctx.tip_gap() * 1000, 2)
    log["tips_lifted_mm"] = round(float(ctx.tips_mid()[2] - T.TOP) * 1000, 1)
    if os.environ.get("DEMO_STOP") == "1":
        return

    # 2 over the source wafer, down between its neighbours, squeeze
    c, _ = ctx.wafer()
    top = c + Z * P.WAFER_R
    yield from set_gap(GAP_OPEN)
    yield from L.move(tcp_for_tips(top + Z * 0.030), R, dur=3.0)
    for _ in range(2):
        c, _ = ctx.wafer()
        yield from L.move(tcp_for_tips(c + Z * (P.WAFER_R + 0.015)), R, dur=0.8)
    for z_off in (0.006, 0.0, -GRIP_IN):
        c, _ = ctx.wafer()
        yield from L.move(tcp_for_tips(c + Z * (P.WAFER_R + z_off)), R, dur=1.0 if z_off > 0 else 1.5)
    yield from wait(0.3)
    for _ in range(6):  # centre the tips on the wafer (x along the edge, y across its faces) before squeezing
        c, _ = ctx.wafer()
        err = (c + Z * (P.WAFER_R - GRIP_IN)) - ctx.tips_mid()
        if abs(err[1]) < 0.00015 and abs(err[0]) < 0.0005:
            break
        p, _ = L.pose()
        yield from L.move(p + np.array([err[0], err[1], 0.0]), R, dur=0.5)
        yield from wait(0.3)
    c, _ = ctx.wafer()
    log["tips_vs_top_edge_mm"] = [round(v * 1000, 2) for v in (ctx.tips_mid() - (c + Z * (P.WAFER_R - GRIP_IN)))]
    # squeeze until the tips close on the wafer's faces, then a small preload: tip gap 60 % of the wafer's thickness
    yield from set_gap(P.WAFER_T * float(os.environ.get("PRELOAD_GAP", 0.6)), tol=0.00003, max_iter=60)
    yield from wait(0.3)
    log["grip_force_N"] = round(ctx.ak.force("grip"), 3)
    log["gap_on_wafer_mm"] = round(ctx.tip_gap() * 1000, 3)

    # 3 lift clear of the cassette, carry over the empty slot
    c0, _ = ctx.wafer()
    src = np.array([*T.slot_xy(T.NEAR_XY, T.SRC)])
    for _ in range(300):  # 1 mm per step, keeping the wafer centred over its slot until it clears the cassette
        c, _ = ctx.wafer()
        if c[2] - P.WAFER_R > T.TOP + P.CAS_H + 0.012:
            break
        corr = np.array([src[0] - c[0], src[1] - c[1], 0.0]) * 0.5
        p, _ = L.pose()
        yield from L.move(p + corr + Z * 0.001, R, dur=1 / HZ)
    yield from wait(0.3)
    c1, _ = ctx.wafer()
    log["wafer_lift_mm"] = round(float(c1[2] - c0[2]) * 1000, 2)
    log["gap_after_lift_mm"] = round(ctx.tip_gap() * 1000, 3)
    log["grip_after_lift_N"] = round(ctx.ak.force("grip"), 3)
    if c1[2] - c0[2] < 0.03:
        log["aborted"] = "wafer not lifted"
        return
    if os.environ.get("DEMO_STOP") == "2":
        return
    tgt = np.array([*T.slot_xy(T.FAR_XY, T.DST), c1[2]])
    for i in range(3):
        c, _ = ctx.wafer()
        p, _ = L.pose()
        yield from L.move(p + (tgt - c), R, dur=5.0 if i == 0 else 1.0)
        yield from wait(0.2)
    c, _ = ctx.wafer()
    log["over_slot_error_mm"] = [round(v * 1000, 2) for v in (c - tgt)]

    # 4 lower into the slot until it rests on the rails
    for _ in range(400):
        c, w = ctx.wafer()
        dz = c[2] - T.SEAT_Z
        if dz < 0.0003:
            break
        step = 0.001 if dz > 0.012 else 0.0004
        corr = np.array([tgt[0] - c[0], tgt[1] - c[1], 0.0]) * 0.5
        p, _ = L.pose()
        yield from L.move(p + corr - Z * min(step, dz + 0.0002), R, dur=1 / HZ)
    yield from wait(0.5)
    c, _ = ctx.wafer()
    log["seat_error_mm"] = round(float(c[2] - T.SEAT_Z) * 1000, 3)

    # 5 release, rise, tweezers home, let go, clear
    yield from set_gap(GAP_OPEN + 0.0005)
    yield from wait(0.5)
    p, _ = L.pose()
    yield from L.move(p + Z * 0.06, R, dur=1.5)
    c, _ = ctx.wafer()
    log["wafer_after_release_mm"] = round(float(c[2] - T.SEAT_Z) * 1000, 3)
    stand = np.array([*T.STAND_XY, T.TOP])
    yield from set_gap(GAP_GRASP)
    yield from L.move(tcp_for_tips(stand + Z * 0.03), R, dur=3.0)
    yield from L.move(tcp_for_tips(stand + Z * 0.03), R, dur=0.6)
    yield from L.move(tcp_for_tips(stand + Z * 0.001), R, dur=1.5)
    yield from L.finger(L.FO, dur=0.8)
    yield from wait(0.3)
    p, _ = L.pose()
    yield from L.move(p + Z * (P.TW_LEN - S_GRIP + 0.04), R, dur=2.5)  # fingers slide up off the tweezers' top end
    p, _ = L.pose()
    yield from L.move(p + np.array([-0.06, 0.0, 0.0]), R, dur=1.5)  # and away
    yield from wait(1.5)
    log["script_done"] = True


def photo(ctx, L):
    """After the scored episode: the empty left hand looks down at the far cassette at a slant (45 deg down, turned
    25 deg off the slot rows) from the robot's side, so the moved wafer's top edge shows next to its neighbours'."""
    import parts as P

    T = ctx.T
    tgt = np.array([*T.slot_xy(T.FAR_XY, T.DST), T.SEAT_Z + P.WAFER_R])
    tilt, yaw = math.radians(45), math.radians(25)
    d = np.array([math.cos(tilt) * math.cos(yaw), math.cos(tilt) * math.sin(yaw), -math.sin(tilt)])
    side = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    side = side - d * (side @ d)
    side /= np.linalg.norm(side)
    R = np.column_stack([side, np.cross(d, side), d]) @ np.diag([-1.0, -1.0, 1.0])
    p, _ = L.pose()
    yield from L.move(p + np.array([0.0, 0.0, 0.03]), finger=-0.01, dur=1.0)
    yield from L.move(tgt - d * 0.12, R, dur=3.0)
    yield from wait(1.5)


def write_mp4(frames, path):
    h, w = frames[0].shape[:2]
    proc = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}",
                             "-r", str(HZ), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                             str(path)], stdin=subprocess.PIPE)
    for f in frames:
        proc.stdin.write(np.ascontiguousarray(np.asarray(f)[..., :3], dtype=np.uint8).tobytes())
    proc.stdin.close()
    proc.wait()


def run(out_dir, max_steps=2400, record=True):
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station
    from yam_arena.constants import READY
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    w, h = (tuple(int(v) for v in os.environ.get("DEMO_CAM", "320x176").split("x")) if record else (64, 36))
    t0 = time.time()
    emb = yam_arena_embodiment(task=str(HERE / "task.py"), object_jitter=0, control_hz=HZ, cam_width=w, cam_height=h)
    obs = emb.reset(Scene(id="demo", instruction="demo"))
    ctx = Ctx(emb)
    L = PoseArm(station.BASES["left"])
    right = np.r_[READY, 1.0]
    log: dict = {}
    gen = strategy(ctx, L, log)
    frames: dict = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended, success, info, scores = 0, None, False, {}, []
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        except Exception as e:  # noqa: BLE001
            log["error"] = repr(e)
            ended = steps if ended is None else ended
        res = emb.step(Action(data=pack(L.cmd, right)))
        steps += 1
        info = res.info
        scores.append(float(info.get("score", 0.0) or 0.0))
        snap(res.observation)
        if os.environ.get("DEMO_DEBUG") and ("grip_force_N" in log or ctx.ak.force("grip") > 0):
            wc, ws = ctx.wafer()
            n = ctx.T._rot(ws.quat_xyzw)[:, 1]
            tm = ctx.tips_mid()
            print(f"dbg {steps} grip {ctx.ak.force('grip'):.3f} gap {ctx.tip_gap()*1000:.3f} wafer_z {(wc[2]-ctx.T.SEAT_Z)*1000:.2f} "
                  f"tips_z_rel {(tm[2]-wc[2])*1000:.2f} tips_y_rel {(tm[1]-wc[1])*1000:.3f} tilt {math.degrees(math.acos(min(1, abs(n[1])))):.2f} "
                  f"finger_cmd {L.f*1000:.2f} W {np.round(wc*1000,1).tolist()} TIP {np.round(tm*1000,1).tolist()} "
                  f"TCP {np.round(ctx.tcp()*1000,1).tolist()} wv {np.round(np.asarray(ws.lin_vel)*1000,0).tolist()}", flush=True)
        if steps % 20 == 0:
            ev = ctx.T.S
            print(f"step {steps} score {info.get('score')} events {ev.get('events')} grip {round(ctx.ak.force('grip'), 3)}",
                  flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= 3 * HZ:
            break
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    try:
        m = ctx.T.measure(ctx.states())
        tips = ctx.T.tip_points()
        mid = (tips[0] + tips[1]) / 2
        _, qa = ctx.ak.body_pose("tweezers", "half_a")
        ax = ctx.T._rot(qa) @ np.array([0.0, 0.0, -1.0])
        log["final"] = {"tw_in_stand": bool(m["tw_in_stand"]), "robot_to_tweezers_mm": round(m["robot_to_tweezers"] * 1000, 1),
                        "tips_vs_stand_mm": [round(float(v) * 1000, 1) for v in (mid - np.array([*ctx.T.STAND_XY, ctx.T.TOP]))],
                        "tweezer_tilt_deg": round(math.degrees(math.acos(min(1.0, -ax[2]))), 2),
                        "tips_to_wafer_mm": round(m["tips_to_wafer"] * 1000, 1), "nb_moved_mm": round(m["nb_moved"] * 1000, 2)}
    except Exception as e:  # noqa: BLE001
        log["final_error"] = repr(e)
    result = {"layout": ctx.T.LAYOUT, "source_slot": ctx.T.SRC, "target_slot": ctx.T.DST, "task_score": score,
              "success": success, "steps": steps, "script_end_step": ended, "events": list(ctx.T.S["events"]),
              "best_score": max(scores) if scores else 0.0, "log": log, "wall_s": round(time.time() - t0, 1)}
    if record and success and os.environ.get("DEMO_PHOTO", "1") == "1":  # outside the scored episode and the video
        import imageio

        gen2, res2 = photo(ctx, L), None
        for _ in range(8 * HZ):
            try:
                next(gen2)
            except StopIteration:
                break
            res2 = emb.step(Action(data=pack(L.cmd, right)))
        if res2 is not None:
            imageio.imwrite(out / "seated.png", np.asarray(res2.observation.images["left_cam"])[..., :3])
            log["photo"] = "seated.png"
    if record:
        import imageio

        for cam, fs in frames.items():
            write_mp4(fs, out / f"demo_{cam}.mp4")
        imageio.imwrite(out / "setup.png", frames["top_cam"][0])
    (out / "result.json").write_text(json.dumps(result, indent=1, default=str))
    print("DEMO_RESULT " + json.dumps(result, default=str), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    run(a.out, a.max_steps, record=not a.no_record)
    os._exit(0)
