"""P1 GPU checks of the merged scene (dev only, not part of the task): smoke (load, reset, 60 idle steps) and the
XT60 pull-apart probe (left hand pulls the pack's XT60 by its sleeve; right hand braces the clipped half if reachable).
Robot commands only; reads poses and the friction gate for logging.
    python run_p1.py OUT --stages smoke,pull
"""
import argparse
import json
import math
import os
import sys
import time
import traceback
from pathlib import Path

import numpy as np

TASK = Path(os.environ.get("CHARGE_TASK", Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(TASK))
HZ = 20
FINGER_OPEN = 0.04695


class Stop(Exception):
    pass


class Arm:
    def __init__(self, side):
        from yam_arena import station
        from yam_arena.constants import READY
        self.side, self.base = side, station.BASES[side]
        self.q = np.array(READY, float)
        self.meas = self.q.copy()
        self.integ = np.zeros(6)
        self.grip = 1.0
        self.cmd = np.r_[self.q, 1.0]
        self.ki, self.clip = 0.15, 0.08

    def solve(self, p, r):
        from yam_arena import ik
        q, pe, re = ik.solve(p, ik.matrix_to_quat_xyzw(r), q_init=self.q, base_pos=self.base, iters=600)
        return q, pe, re

    def command(self, q, grip):
        self.q = np.asarray(q, float)
        self.integ = np.clip(self.integ + self.ki * (self.q - self.meas), -self.clip, self.clip)
        self.grip = float(grip)
        self.cmd = np.r_[self.q + self.integ, self.grip]

    def tcp(self):
        from yam_arena import ik
        return ik.tcp_pose(self.q, self.base)

    def measured_tcp(self):
        from yam_arena import ik
        return ik.tcp_pose(self.meas, self.base)


class Runner:
    def __init__(self):
        from inspect_robots import Scene
        from yam_arena.inspect_embodiment import yam_arena_embodiment
        import livekit
        self.lk, self.Scene = livekit, Scene
        t0 = time.time()
        self.emb = yam_arena_embodiment(task=str(TASK / "task.py"), object_jitter=0, control_hz=HZ, cam_width=640,
                                        cam_height=352)
        self.kit_s = round(time.time() - t0, 1)
        print("KIT_START_S", self.kit_s, flush=True)

    def start(self, out, stage):
        import task
        self.out, self.stage = Path(out), stage
        self.out.mkdir(parents=True, exist_ok=True)
        self.rows, self.frames, self.steps, self.actions = [], {}, 0, []
        t0 = time.time()
        obs = self.emb.reset(self.Scene(id=f"charge-{stage}", instruction=task.INSTRUCTION))
        self.reset_s = round(time.time() - t0, 1)
        print("RESET_S", self.reset_s, flush=True)
        self.left, self.right = Arm("left"), Arm("right")
        self.snap(obs)
        self.wall0 = time.time()

    def snap(self, obs):
        for cam, img in obs.images.items():
            self.frames.setdefault(cam, []).append(np.asarray(img))

    def poses(self):
        return self.lk.STATE["view"].poses()

    def step(self, label):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import STATE_KEY, pack
        from yam_arena import ik
        lo, hi = np.append(ik.LOWER, 0.0), np.append(ik.UPPER, 1.0)
        cmd = np.r_[np.clip(self.left.cmd, lo, hi), np.clip(self.right.cmd, lo, hi)]
        if self.actions:  # stay inside the official CLI guardrails (bounds, 5 % of the range per step) so a replay is exact
            lim = 0.045 * np.r_[hi - lo, hi - lo]
            prev = np.r_[self.actions[-1][:7], self.actions[-1][7:]]
            cmd = np.clip(cmd, prev - lim, prev + lim)
        data = pack(cmd[:7], cmd[7:])
        self.actions.append(np.asarray(data, float).ravel().tolist())  # replayed by the official run
        res = self.emb.step(Action(data=data))
        jp = np.asarray(res.observation.state[STATE_KEY], float).ravel()
        self.left.meas, self.right.meas = jp[:6], jp[7:13]
        self.steps += 1
        self.snap(res.observation)
        tr = self.lk.STATE["trace"][-1]
        p = self.poses()
        lp, lr = self.left.measured_tcp()
        mp_ = p.plug("main")[0]
        self.rows.append(dict(step=self.steps, label=label, score=tr["score"], veto=tr["veto"], live=tr["live"],
                              xt60_rel_mm=np.round(np.array(tr["xt60_rel"]) * 1000, 3).tolist(), engaged=tr["engaged"],
                              grip_cmd=[self.left.grip, self.right.grip], grip_meas=[float(jp[6]), float(jp[13])],
                              left_tcp_meas_mm=np.round(lp * 1000, 2).tolist(),
                              left_tcp_cmd_mm=np.round(self.left.tcp()[0] * 1000, 2).tolist(),
                              left_integ=np.round(self.left.integ, 3).tolist(),
                              plug_in_left_hand_mm=np.round(lr.T @ (mp_ - lp) * 1000, 2).tolist(),
                              gripper_contacts=tr["gripper_contacts"], lcd=tr["lcd"], page=tr["page"],
                              finger_n=self.finger_forces()))
        if self.steps % 10 == 0:
            print(json.dumps(self.rows[-1]), flush=True)

    def finger_forces(self):
        v = self.lk.STATE["view"]
        if not hasattr(self, "fview"):
            self.fview = v.sim.create_rigid_contact_view([v.view.prim_paths[v.index["main"]]],
                                                         filter_patterns=[v.grip_paths], max_contact_data_count=256)
        f = self.fview.get_contact_force_matrix(dt=1.0 / 960).detach().cpu().numpy()[0]
        return {Path(p).parent.name[:5] + "/" + Path(p).name: round(float(np.linalg.norm(x)), 2)
                for p, x in zip(v.grip_paths, f) if np.linalg.norm(x) > 0.01}

    def wait(self, label, s):
        for _ in range(max(1, round(s * HZ))):
            self.step(label)

    def go(self, arm, label, duration, p1, r1, grip1=None):
        from yam_arena import ik
        p0, r0 = arm.tcp()
        g0 = arm.grip
        g1 = g0 if grip1 is None else grip1
        n = max(1, round(duration * HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            q, pe, re = arm.solve(p0 + s * (np.asarray(p1) - p0), ik.slerp_matrix(r0, r1, s))
            if pe > 0.002 or re > math.radians(3):
                raise Stop(f"{label}: unreachable target ({pe * 1000:.1f} mm, {math.degrees(re):.1f} deg)")
            arm.command(q, g0 + s * (g1 - g0))
            self.step(label)

    def reachable(self, arm, p, r):
        q, pe, re = arm.solve(p, r)
        return pe < 0.002 and re < math.radians(3), q

    def top_grip(self, arm, p, yaws):
        from yam_arena.demo import grip_rotation
        from yam_arena import ik
        best = None
        for yaw in yaws:
            r = grip_rotation(yaw)
            ok, q = self.reachable(arm, p, r)
            margin = min(np.min(q - ik.LOWER), np.min(ik.UPPER - q))
            if ok and (best is None or margin > best[0]):
                best = (margin, r)
        return None if best is None else best[1]

    def pull(self, brace):
        lk = self.lk
        p = self.poses()
        mc, mr = p.plug("main")
        axis = mr[:, 0]  # plug frame +X points back along the cable (world -X here)
        sleeve = mc + axis * (lk.XT60[0] / 2 + lk.SLEEVE[0] / 2)
        h = math.atan2(axis[1], axis[0])
        lt = sleeve + axis * float(os.environ.get("PULL_DX", "0")) + [0, 0, 0.002 + float(os.environ.get("PULL_DZ", "0"))]
        lrot = self.top_grip(self.left, lt, (h, h + math.pi))
        if lrot is None:
            raise Stop("left: no top-down grasp of the XT60 sleeve")
        brace_p = lk.CLIP + [0.004, 0, 0.009]  # far end of the clipped half, pad tips above the clip's base plate
        rrot = self.top_grip(self.right, brace_p, (h, h + math.pi)) if brace else None
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics
        cap = UsdPhysics.DriveAPI.Get(get_current_stage().GetPrimAtPath(lk.FRICTION_JOINT["path"]), "transX")
        self.events = dict(brace_requested=brace, right_brace_reachable=rrot is not None, sleeve_target=lt.tolist(),
                           friction_cap_n_at_start=cap.GetMaxForceAttr().Get(), engaged_at_start=lk.STATE["engaged"])
        print("PULL_PLAN", json.dumps(self.events), flush=True)
        self.wait("settle", 0.5)
        if rrot is not None:
            self.go(self.right, "right_hover", 2.5, brace_p + [0, 0, 0.06], rrot, 0.30)
            self.go(self.right, "right_descend", 2.0, brace_p, rrot)
            self.go(self.right, "right_close", 1.0, brace_p, rrot, 0.0)
        self.go(self.left, "left_hover", 2.5, lt + [0, 0, 0.06], lrot, 0.30)
        self.go(self.left, "left_descend", 2.0, lt, lrot)
        self.go(self.left, "left_close", 1.0, lt, lrot, 0.0)
        self.wait("left_squeeze", 0.5)
        if os.environ.get("PULL_COMP") == "strong":  # load compensation of the balance-charge demo (0.3 / step, 0.25 rad)
            self.left.ki, self.left.clip = 0.3, 0.25
        x0 = np.array(self.lk.STATE["trace"][-1]["xt60_rel"])
        self.go(self.left, "left_pull", 3.0, lt + axis * 0.030, lrot)
        self.wait("hold_out", 2.0)
        x1 = np.array(self.lk.STATE["trace"][-1]["xt60_rel"])
        self.events.update(xt60_rel_before_mm=(x0 * 1000).round(3).tolist(), xt60_rel_after_mm=(x1 * 1000).round(3).tolist(),
                           separated_mm=round(float((x1[0] - x0[0]) * 1000), 3), live_after=self.lk.STATE["trace"][-1]["live"],
                           hand_travel_mm=30.0, comp=os.environ.get("PULL_COMP", "default"),
                           left_tcp_meas_start_mm=self.rows[0]["left_tcp_meas_mm"])

    def finish(self, err):
        import imageio
        lk = self.lk
        summ = lk.summary()
        base = self.emb._env.unwrapped
        efforts = {}
        for a in ("left_robot", "right_robot"):
            rob = base.scene[a]
            fi = rob.find_joints(["joint7", "joint8"], preserve_order=True)[0]
            try:
                efforts[a] = dict(actuator=np.asarray(rob.actuators["gripper"].effort_limit.detach().cpu()).ravel().tolist(),
                                  data=np.asarray(rob.data.joint_effort_limits.detach().cpu())[0][fi].tolist())
            except Exception as e:  # noqa: BLE001  diagnostic only
                efforts[a] = repr(e)
        res = dict(stage=self.stage, score=summ["score"], veto=summ["veto"], steps=self.steps, kit_s=self.kit_s,
                   reset_s=self.reset_s, step_wall_s=round((time.time() - self.wall0) / max(1, self.steps), 2),
                   gripper_effort_limits=efforts, error=None if err is None else repr(err), summary=summ,
                   events=getattr(self, "events", None), touch_links=lk.TOUCH_LINKS)
        (self.out / "result.json").write_text(json.dumps(res, indent=2, default=float))
        (self.out / "actions.json").write_text(json.dumps(dict(control_hz=HZ, actions=self.actions)))
        (self.out / "rows.jsonl").write_text("\n".join(json.dumps(r, default=float) for r in self.rows) + "\n")
        (self.out / "trace.jsonl").write_text("\n".join(json.dumps(r, default=float) for r in lk.STATE["trace"]) + "\n")
        for cam, fs in self.frames.items():
            imageio.mimsave(self.out / f"{cam}.mp4", fs, fps=HZ, codec="libx264",
                            output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
            imageio.imwrite(self.out / f"first_{cam}.png", fs[0])
            imageio.imwrite(self.out / f"last_{cam}.png", fs[-1])
        print("RESULT", json.dumps({k: res[k] for k in ("stage", "score", "veto", "steps", "error", "events")},
                                   default=float), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--stages", default="smoke")
    a = ap.parse_args()
    r = Runner()
    for st in a.stages.split(","):
        err = None
        try:
            r.start(Path(a.out) / st, st)
            if st == "smoke":
                r.wait("idle", 3.0)
            elif st in ("pull", "pull2"):
                r.pull(brace=st == "pull2")
        except Exception as e:  # noqa: BLE001  recorded
            err = e
            traceback.print_exc()
        r.finish(err)
    os._exit(0)


if __name__ == "__main__":
    main()
