"""P2 diagnosis of the XT60 pull-apart (dev only). Robot acts only through joint/gripper commands.
Stages: top (top-down grasp of the sleeve), pitchNN (approach pitched NN deg below horizontal, from the robot side).
Every control step logs every contact of the left gripper links (gripper, tip_left, tip_right) with every scene body,
plus the net contact force on them. Pull: IK target moved 30 mm along the axis and a Jacobian feedforward
dq = J^T (F u) / Kp added to the joint targets (F ramped to PULL_N), i.e. a Cartesian force hold through the joint PD
(gravity is compensated by the arm action); joint torques stay within the effort limits.
    python run_p2.py OUT --stages top,pitch40
"""
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_p1 as R  # noqa: E402

PULL_N = float(os.environ.get("PULL_N", "40"))


class P2(R.Runner):
    def contact_views(self):
        if hasattr(self, "cv"):
            return
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics
        v = self.lk.STATE["view"]
        stage = get_current_stage()
        bodies = []
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if "/env_0/" in path and "Arm" not in path and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                bodies.append(path)
        self.bodies = bodies
        self.sensors = v.grip_paths[:3]  # left arm: gripper, tip_left, tip_right
        self.cv = v.sim.create_rigid_contact_view(self.sensors, filter_patterns=[bodies] * len(self.sensors),
                                                  max_contact_data_count=4096)
        print("CONTACT_VIEW", len(self.sensors), self.sensors, len(bodies), flush=True)

    def step(self, label):
        super().step(label)
        self.contact_views()
        dt = 1.0 / 960
        m = self.cv.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
        net = self.cv.get_net_contact_forces(dt=dt).detach().cpu().numpy()
        pairs = []
        for i, s in enumerate(self.sensors):
            for j in np.nonzero(np.linalg.norm(m[i], axis=-1) > 0.01)[0]:
                pairs.append([s.rsplit("/", 1)[1], "/".join(self.bodies[j].split("/")[-2:]),
                              round(float(np.linalg.norm(m[i, j])), 2)])
        self.rows[-1]["contacts"] = pairs
        self.rows[-1]["net_n"] = [round(float(np.linalg.norm(n)), 2) for n in net]
        self.rows[-1]["matrix_sum_n"] = [round(float(np.linalg.norm(m[i].sum(0))), 2) for i in range(len(self.sensors))]
        self.rows[-1]["ff_n"] = getattr(self, "ff_now", 0.0)
        if self.steps % 10 == 0:
            print("C", self.steps, label, pairs, self.rows[-1]["net_n"], flush=True)

    def jac(self, arm):
        from yam_arena import ik
        J = np.zeros((3, 6))
        for i in range(6):
            dq = np.zeros(6); dq[i] = 1e-6
            J[:, i] = (ik.tcp_pose(arm.q + dq, arm.base)[0] - ik.tcp_pose(arm.q - dq, arm.base)[0]) / 2e-6
        return J

    def pull2(self, pitch):
        from yam_arena import ik
        from yam_arena.constants import SHOULDER, WRIST
        KP = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
        EFF = np.array([SHOULDER["effort"]] * 3 + [WRIST["effort"]] * 3)
        lk = self.lk
        p = self.poses()
        mc, mr = p.plug("main")
        axis = mr[:, 0]
        dx = float(os.environ.get("PULL_DX", "0.004" if pitch is not None else "0"))
        lt = mc + axis * (lk.XT60[0] / 2 + lk.SLEEVE[0] / 2 + dx) + [0, 0, 0.002 if pitch is None else 0.0]
        if pitch is None:
            h = math.atan2(axis[1], axis[0])
            rot = self.top_grip(self.left, lt, (h, h + math.pi))
        else:
            yaw = -math.radians(float(os.environ.get("PULL_YAW", "0")))  # + = hand body to the left of the plug axis
            a0 = np.array([-axis[0] * math.cos(math.radians(pitch)), -axis[1] * math.cos(math.radians(pitch)),
                           -math.sin(math.radians(pitch))])  # pointing at the clip, tilted down
            a = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]) @ a0
            x = np.cross(a, [0.0, 0.0, 1.0])
            x /= np.linalg.norm(x)
            rot = np.column_stack([x, np.cross(a, x), a])
            ok, _ = self.reachable(self.left, lt, rot)
            if not ok:
                rot = None
        if rot is None:
            raise R.Stop(f"no reachable grasp (pitch {pitch})")
        app = rot[:, 2]
        self.grasp = (lt, rot, axis)
        self.events = dict(pitch=pitch, yaw=os.environ.get("PULL_YAW", "0"), dx=dx, grasp_tcp=lt.tolist(), pull_n=PULL_N)
        self.wait("settle", 0.5)
        self.go(self.left, "hover", 2.5, lt - app * 0.06, rot, 0.30)
        self.go(self.left, "approach", 2.0, lt, rot)
        self.go(self.left, "close", 1.0, lt, rot, 0.0)
        self.wait("squeeze", 0.5)
        x0 = np.array(lk.STATE["trace"][-1]["xt60_rel"])
        n = 4 * R.HZ
        sep_step = None
        for k in range(1, n + 1):
            s = ik.smoothstep(min(1.0, k / (3 * R.HZ)))
            q, pe, re = self.left.solve(lt + axis * 0.030 * s, rot)
            if pe > 0.002 or re > math.radians(3):
                raise R.Stop(f"pull: unreachable ({pe * 1000:.1f} mm)")
            self.left.q = q
            rel = lk.STATE["trace"][-1]["xt60_rel"][0]
            f = PULL_N * min(1.0, k / (1.5 * R.HZ)) if rel < 0.008 else 0.0
            per_n = self.jac(self.left).T @ axis
            f = min(f, 0.97 * float(np.min(EFF / np.maximum(np.abs(per_n), 1e-9))))  # stay within the joint efforts
            tau = per_n * f
            self.ff_now = f
            self.left.integ = np.clip(self.left.integ + self.left.ki * (q - self.left.meas), -0.08, 0.08)
            self.left.cmd = np.r_[q + self.left.integ + tau / KP, self.left.grip]
            self.step("pull")
            self.rows[-1]["tau_ff"] = np.round(tau, 2).tolist()
            if sep_step is None and lk.STATE["trace"][-1]["xt60_rel"][0] >= 0.010:
                sep_step = self.steps
        self.ff_now = 0.0
        self.wait("hold_out", 1.0)
        x1 = np.array(lk.STATE["trace"][-1]["xt60_rel"])
        self.events.update(xt60_rel_before_mm=(x0 * 1000).round(3).tolist(), xt60_rel_after_mm=(x1 * 1000).round(3).tolist(),
                           max_rel_mm=round(max(r["xt60_rel_mm"][0] for r in self.rows), 3), separated_10mm_step=sep_step,
                           live_after=lk.STATE["trace"][-1]["live"],
                           peak_ff_n=max(r.get("ff_n", 0) for r in self.rows),
                           peak_finger_n=max([max(r["finger_n"].values(), default=0) for r in self.rows]))


def main():
    import argparse
    import traceback
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--stages", default="top")
    a = ap.parse_args()
    r = P2()
    for st in a.stages.split(","):
        err = None
        try:
            r.start(Path(a.out) / st, st)
            if st.startswith("idle"):
                r.wait("idle", 1.5)
                ini = r.lk.STATE["initial"]
                r.events = dict(initial_balance=ini["balance"], initial_xt60=ini["xt60"], initial_power=ini["power"],
                                initial_engaged=r.lk.STATE["initial_engaged"])
            else:
                r.pull2(None if st == "top" else float(st[5:]))
        except Exception as e:  # noqa: BLE001  recorded
            err = e
            traceback.print_exc()
        r.finish(err)
    os._exit(0)


if __name__ == "__main__":
    main()
