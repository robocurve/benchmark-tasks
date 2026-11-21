"""P3 (dev only): scripted force probe of the XT60 friction element (calibration force on the male XT60 body, arms
idle), pitched pull + in-hand re-mate (robot commands only), and reset checks.
    python run_p3.py OUT --stages idle1,fprobe,idle2,pitch45,idle3
"""
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_p2 as P  # noqa: E402
import run_p1 as R  # noqa: E402


class P3(P.P2):
    def rel(self):
        return self.lk.STATE["trace"][-1]["xt60_rel"][0]

    def ramp(self, label, sign, stop):
        """Probe force along the mating axis (sign +1 = out), 0 -> 10 N in 0.5 s, then 5 N/s to 45 N; the breakaway is
        the first force at which the plug moves faster than 5 mm/s (the drive creeps at F/D = 0.6 mm/s at 30 N)."""
        f, out, prev = 0.0, None, self.rel()
        rows = []
        while f < 45.0:
            f = min(45.0, f + (1.0 if f < 10.0 else 0.25))
            self.lk.STATE["probe_n"] = sign * f
            self.step(label)
            x = self.rel()
            v = (x - prev) * R.HZ
            prev = x
            rows.append([round(f, 2), round(x * 1000, 3), round(v * 1000, 2), self.lk.STATE["engaged"]])
            if out is None and sign * v > 0.005:
                out = f
            if stop(x):
                break
        self.lk.STATE["probe_n"] = 0.0
        self.wait(label + "_rest", 0.5)
        return out, rows

    def fprobe(self):
        self.wait("settle", 0.5)
        x0 = self.rel()
        f_out, r_out = self.ramp("probe_out", 1.0, lambda x: x > x0 + 0.004)
        x1 = self.rel()
        f_in, r_in = self.ramp("probe_in", -1.0, lambda x: x < 0.0012)
        self.events = dict(breakaway_out_n=f_out, breakaway_in_n=f_in, rel_start_mm=round(x0 * 1000, 3),
                           rel_after_out_mm=round(x1 * 1000, 3), rel_end_mm=round(self.rel() * 1000, 3),
                           out=r_out, inward=r_in)
        print("FPROBE", json.dumps({k: v for k, v in self.events.items() if k not in ("out", "inward")}), flush=True)

    def remate(self):
        from yam_arena import ik
        from yam_arena.constants import SHOULDER, WRIST
        KP = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
        EFF = np.array([SHOULDER["effort"]] * 3 + [WRIST["effort"]] * 3)
        lt, rot, axis = self.grasp
        x0 = self.rel()
        p0 = self.left.tcp()[0]
        seated = 0
        f_need = []
        for k in range(1, 6 * R.HZ + 1):
            s = ik.smoothstep(min(1.0, k / (3 * R.HZ)))
            q, pe, re = self.left.solve(p0 + s * (lt - axis * 0.002 - p0), rot)
            self.left.q = q
            x = self.rel()
            f = 45.0 * min(1.0, k / (1.5 * R.HZ)) if x < 0.007 else 0.0  # push only once the contacts engage
            per_n = self.jac(self.left).T @ (-axis)
            f_cap = 0.97 * float(np.min(EFF / np.maximum(np.abs(per_n), 1e-9)))
            f = min(f, f_cap)
            f_need.append(f_cap)
            self.ff_now = -f
            self.left.integ = np.clip(self.left.integ + self.left.ki * (q - self.left.meas), -0.08, 0.08)
            self.left.cmd = np.r_[q + self.left.integ + per_n * f / KP, self.left.grip]
            self.step("remate")
            tr = self.lk.STATE["trace"][-1]
            seated = seated + 1 if tr["xt60_seated"] else 0
            if seated >= 4:
                break
        self.ff_now = 0.0
        self.wait("remate_hold", 0.5)
        tr = self.lk.STATE["trace"][-1]
        self.events.update(remate_seated=bool(tr["xt60_seated"]), remate_rel_mm=np.round(np.array(tr["xt60_rel"]) * 1000, 3).tolist(),
                           remate_start_rel_mm=round(x0 * 1000, 3), insertion_force_available_n=round(min(f_need), 1))


class P3b(P3):
    def remate(self):
        """Pose-feedback mate: every step the hand target is the desired plug pose (on the clip axis at depth d, the
        clip's orientation) composed with the measured plug-in-hand transform, plus a slow integral of the measured
        lateral error. Hold at 12 mm until coaxial, then insert; insertion force feedforward once the contacts engage."""
        from yam_arena import ik
        from yam_arena.constants import SHOULDER, WRIST
        KP = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
        EFF = np.array([SHOULDER["effort"]] * 3 + [WRIST["effort"]] * 3)
        lk = self.lk
        lt, rot, axis = self.grasp
        u = lk.RCLIP[:, 0]
        x0 = self.rel()
        corr = np.zeros(3)
        seated = good = 0
        f_cap_min = 1e9
        phase, d0, k0 = "align", x0, 0
        for k in range(1, 12 * R.HZ + 1):
            pp, pr = self.poses().plug("main")
            hp, hr = self.left.measured_tcp()
            lp, lr = hr.T @ (pp - hp), hr.T @ pr
            rel = lk.RCLIP.T @ (pp - lk.CLIP)
            ang = math.degrees(math.acos(float(np.clip((lk.RCLIP.T @ pr)[0, 0], -1, 1))))
            if phase == "align":
                d = d0 + (0.012 - d0) * ik.smoothstep(min(1.0, k / (2 * R.HZ)))
                good = good + 1 if (k > 2 * R.HZ and max(abs(rel[1:])) < 0.0004 and ang < 3) else 0
                if good >= 5 or k > 7 * R.HZ:
                    phase, k0 = "insert", k
            else:
                d = 0.012 + (-0.002 - 0.012) * ik.smoothstep(min(1.0, (k - k0) / (2.5 * R.HZ)))
            err = np.array([0.0, -rel[1], -rel[2]])
            corr = np.clip(corr + 0.06 * err, -0.006, 0.006)
            tp = lk.CLIP + lk.RCLIP @ (np.array([d, 0, 0]) + corr)
            hr_t = lk.RCLIP @ lr.T
            hp_t = tp - hr_t @ lp
            q, pe, re = self.left.solve(hp_t, hr_t)
            self.left.q = q
            f = 45.0 if (phase == "insert" and rel[0] < 0.007) else 0.0
            per_n = self.jac(self.left).T @ (-u)
            cap = 0.97 * float(np.min(EFF / np.maximum(np.abs(per_n), 1e-9)))
            f_cap_min = min(f_cap_min, cap)
            f = min(f, cap)
            self.ff_now = -f
            self.left.integ = np.clip(self.left.integ + self.left.ki * (q - self.left.meas), -0.08, 0.08)
            self.left.cmd = np.r_[q + self.left.integ + per_n * f / KP, self.left.grip]
            self.step("remate_" + phase)
            self.rows[-1].update(remate_rel_mm=np.round(rel * 1000, 3).tolist(), remate_angle_deg=round(ang, 2),
                                 ik_err_mm=round(pe * 1000, 2))
            seated = seated + 1 if lk.STATE["trace"][-1]["xt60_seated"] else 0
            if seated >= 4:
                break
        self.ff_now = 0.0
        self.wait("remate_hold", 0.5)
        tr = lk.STATE["trace"][-1]
        self.events.update(remate_seated=bool(tr["xt60_seated"]), remate_rel_mm=np.round(np.array(tr["xt60_rel"]) * 1000, 3).tolist(),
                           remate_start_rel_mm=round(x0 * 1000, 3), insertion_force_available_n=round(f_cap_min, 1),
                           remate_live=bool(tr["live"]))


class P3c(P3):
    def remate(self):
        """Re-mate at the grasp orientation (the pitched pose with the most axial force): the hand keeps the grasp
        rotation; its position = desired plug position on the clip axis minus the measured plug-in-hand offset, plus a
        slow integral of the measured lateral error. Hold 12 mm out until coaxial, then insert with the insertion
        force feedforward (capped at the joint efforts) until the XT60 measures fully mated."""
        from yam_arena import ik
        from yam_arena.constants import SHOULDER, WRIST
        KP = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
        EFF = np.array([SHOULDER["effort"]] * 3 + [WRIST["effort"]] * 3)
        lk = self.lk
        lt, rot, axis = self.grasp
        u = lk.RCLIP[:, 0]
        x0 = self.rel()
        corr = np.zeros(3)
        seated = good = 0
        caps = []
        phase, d0, k0 = "align", x0, 0
        for k in range(1, 14 * R.HZ + 1):
            pp, pr = self.poses().plug("main")
            hp, hr = self.left.measured_tcp()
            lp = hr.T @ (pp - hp)
            rel = lk.RCLIP.T @ (pp - lk.CLIP)
            ang = math.degrees(math.acos(float(np.clip((lk.RCLIP.T @ pr)[0, 0], -1, 1))))
            if phase == "align":
                d = d0 + (0.012 - d0) * ik.smoothstep(min(1.0, k / (2 * R.HZ)))
                good = good + 1 if (k > 2 * R.HZ and max(abs(rel[1:])) < 0.0003) else 0
                if good >= 5 or k > 7 * R.HZ:
                    phase, k0 = "insert", k
            else:
                d = 0.012 + (-0.003 - 0.012) * ik.smoothstep(min(1.0, (k - k0) / (3 * R.HZ)))
            corr = np.clip(corr + 0.06 * np.array([0.0, -rel[1], -rel[2]]), -0.006, 0.006)
            tp = lk.CLIP + lk.RCLIP @ (np.array([d, 0, 0]) + corr)
            q, pe, re = self.left.solve(tp - rot @ lp, rot)
            self.left.q = q
            per_n = self.jac(self.left).T @ (-u)
            cap = 0.97 * float(np.min(EFF / np.maximum(np.abs(per_n), 1e-9)))
            f = min(45.0 * min(1.0, (k - k0) / R.HZ), cap) if phase == "insert" else 0.0
            if phase == "insert":
                caps.append(cap)
            self.ff_now = -f
            self.left.integ = np.clip(self.left.integ + self.left.ki * (q - self.left.meas), -0.08, 0.08)
            self.left.cmd = np.r_[q + self.left.integ + per_n * f / KP, self.left.grip]
            self.step("remate_" + phase)
            self.rows[-1].update(remate_rel_mm=np.round(rel * 1000, 3).tolist(), remate_angle_deg=round(ang, 2),
                                 ik_err_mm=round(pe * 1000, 2), insert_cap_n=round(cap, 1))
            seated = seated + 1 if lk.STATE["trace"][-1]["xt60_seated"] else 0
            if seated >= 4:
                break
        self.ff_now = 0.0
        self.wait("remate_hold", 0.5)
        tr = lk.STATE["trace"][-1]
        self.events.update(remate_seated=bool(tr["xt60_seated"]), remate_rel_mm=np.round(np.array(tr["xt60_rel"]) * 1000, 3).tolist(),
                           remate_start_rel_mm=round(x0 * 1000, 3), remate_live=bool(tr["live"]),
                           insertion_force_available_n=[round(min(caps), 1), round(max(caps), 1)] if caps else None,
                           score_after_remate=tr["score"], veto=tr["veto"])


class P3d(P3):
    def remate(self):
        """Pose-feedback re-mate: hand target = desired plug pose (clip axis, depth d) composed with the measured
        plug-in-hand transform, plus integrals of the measured lateral and axis-angle errors (they absorb the arm's
        PD sag); insertion force feedforward over the whole insert phase, capped at the joint efforts."""
        from scipy.spatial.transform import Rotation
        from yam_arena import ik
        from yam_arena.constants import SHOULDER, WRIST
        KP = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
        EFF = np.array([SHOULDER["effort"]] * 3 + [WRIST["effort"]] * 3)
        lk = self.lk
        u = lk.RCLIP[:, 0]
        x0 = self.rel()
        corr, rcorr = np.zeros(3), np.zeros(3)
        seated = good = 0
        caps = []
        phase, d0, k0 = "align", x0, 0
        for k in range(1, 14 * R.HZ + 1):
            pp, pr = self.poses().plug("main")
            hp, hr = self.left.measured_tcp()
            lp, lr = hr.T @ (pp - hp), hr.T @ pr
            rel = lk.RCLIP.T @ (pp - lk.CLIP)
            aerr = np.cross(pr[:, 0], u)  # rotation vector taking the plug axis onto the clip axis (small angles)
            ang = math.degrees(math.asin(min(1.0, float(np.linalg.norm(aerr)))))
            if phase == "align":
                d = d0 + (0.012 - d0) * ik.smoothstep(min(1.0, k / (2 * R.HZ)))
                good = good + 1 if (k > 2 * R.HZ and max(abs(rel[1:])) < 0.0003 and ang < 1.0) else 0
                if good >= 5 or k > 8 * R.HZ:
                    phase, k0 = "insert", k
            else:
                d = 0.012 + (-0.003 - 0.012) * ik.smoothstep(min(1.0, (k - k0) / (3 * R.HZ)))
            corr = np.clip(corr + 0.06 * np.array([0.0, -rel[1], -rel[2]]), -0.006, 0.006)
            rcorr = np.clip(rcorr + 0.15 * aerr, -0.25, 0.25)
            tp = lk.CLIP + lk.RCLIP @ (np.array([d, 0, 0]) + corr)
            hr_t = Rotation.from_rotvec(rcorr).as_matrix() @ lk.RCLIP @ lr.T
            q, pe, re = self.left.solve(tp - hr_t @ lp, hr_t)
            self.left.q = q
            per_n = self.jac(self.left).T @ (-u)
            cap = 0.97 * float(np.min(EFF / np.maximum(np.abs(per_n), 1e-9)))
            f = min(45.0 * min(1.0, (k - k0) / R.HZ), cap) if phase == "insert" else 0.0
            if phase == "insert":
                caps.append(cap)
            self.ff_now = -f
            self.left.integ = np.clip(self.left.integ + self.left.ki * (q - self.left.meas), -0.08, 0.08)
            self.left.cmd = np.r_[q + self.left.integ + per_n * f / KP, self.left.grip]
            self.step("remate_" + phase)
            self.rows[-1].update(remate_rel_mm=np.round(rel * 1000, 3).tolist(), remate_angle_deg=round(ang, 2),
                                 ik_err_mm=round(pe * 1000, 2), insert_cap_n=round(cap, 1))
            seated = seated + 1 if lk.STATE["trace"][-1]["xt60_seated"] else 0
            if seated >= 4:
                break
        self.ff_now = 0.0
        self.wait("remate_hold", 0.5)
        tr = lk.STATE["trace"][-1]
        self.events.update(remate_seated=bool(tr["xt60_seated"]), remate_rel_mm=np.round(np.array(tr["xt60_rel"]) * 1000, 3).tolist(),
                           remate_start_rel_mm=round(x0 * 1000, 3), remate_live=bool(tr["live"]),
                           insertion_force_available_n=[round(min(caps), 1), round(max(caps), 1)] if caps else None,
                           score_after_remate=tr["score"], veto=tr["veto"])


class P3e(P3):
    def remate(self):
        """Pose-feedback re-mate: hand target = desired plug pose (clip axis, depth d) composed with the measured
        plug-in-hand transform, plus integrals of the measured lateral and axis-angle errors (they absorb the arm's
        PD sag); insertion force feedforward over the whole insert phase, capped at the joint efforts."""
        from scipy.spatial.transform import Rotation
        from yam_arena import ik
        from yam_arena.constants import SHOULDER, WRIST
        KP = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
        EFF = np.array([SHOULDER["effort"]] * 3 + [WRIST["effort"]] * 3)
        lk = self.lk
        u = lk.RCLIP[:, 0]
        x0 = self.rel()
        corr, rcorr = np.zeros(3), np.zeros(3)
        seated = good = 0
        caps = []
        phase, d0, k0 = "align", x0, 0
        for k in range(1, 14 * R.HZ + 1):
            pp, pr = self.poses().plug("main")
            hp, hr = self.left.measured_tcp()
            lp, lr = hr.T @ (pp - hp), hr.T @ pr
            rel = lk.RCLIP.T @ (pp - lk.CLIP)
            aerr = np.cross(pr[:, 0], u)  # rotation vector taking the plug axis onto the clip axis (small angles)
            ang = math.degrees(math.asin(min(1.0, float(np.linalg.norm(aerr)))))
            if phase == "align":
                d = d0 + (0.012 - d0) * ik.smoothstep(min(1.0, k / (2 * R.HZ)))
                good = good + 1 if (k > 2 * R.HZ and max(abs(rel[1:])) < 0.0003 and ang < 1.0) else 0
                if good >= 5 or k > 8 * R.HZ:
                    phase, k0 = "insert", k
                    lock = (hr.copy(), lp.copy())  # hand orientation and plug-in-hand offset frozen for the insertion
            else:
                d = 0.012 + (-0.003 - 0.012) * ik.smoothstep(min(1.0, (k - k0) / (3 * R.HZ)))
            if phase == "align":
                corr = np.clip(corr + 0.06 * np.array([0.0, -rel[1], -rel[2]]), -0.006, 0.006)
                rcorr = np.clip(rcorr + 0.15 * aerr, -0.25, 0.25)
                tp = lk.CLIP + lk.RCLIP @ (np.array([d, 0, 0]) + corr)
                hr_t = Rotation.from_rotvec(rcorr).as_matrix() @ lk.RCLIP @ lr.T
                hp_t = tp - hr_t @ lp
                self.hand_cmd = (hp_t, hr_t)
            else:  # straight push of the commanded hand pose along the clip axis; no feedback from contact tilt
                hp_t = self.hand_cmd[0] + u * (d - 0.012)
                hr_t = self.hand_cmd[1]
            q, pe, re = self.left.solve(hp_t, hr_t)
            self.left.q = q
            per_n = self.jac(self.left).T @ (-u)
            cap = 0.97 * float(np.min(EFF / np.maximum(np.abs(per_n), 1e-9)))
            f = min(45.0 * min(1.0, (k - k0) / R.HZ), cap) if phase == "insert" else 0.0
            if phase == "insert":
                caps.append(cap)
            self.ff_now = -f
            self.left.integ = np.clip(self.left.integ + self.left.ki * (q - self.left.meas), -0.08, 0.08)
            self.left.cmd = np.r_[q + self.left.integ + per_n * f / KP, self.left.grip]
            self.step("remate_" + phase)
            self.rows[-1].update(remate_rel_mm=np.round(rel * 1000, 3).tolist(), remate_angle_deg=round(ang, 2),
                                 ik_err_mm=round(pe * 1000, 2), insert_cap_n=round(cap, 1))
            seated = seated + 1 if lk.STATE["trace"][-1]["xt60_seated"] else 0
            if seated >= 4:
                break
        self.ff_now = 0.0
        self.wait("remate_hold", 0.5)
        tr = lk.STATE["trace"][-1]
        self.events.update(remate_seated=bool(tr["xt60_seated"]), remate_rel_mm=np.round(np.array(tr["xt60_rel"]) * 1000, 3).tolist(),
                           remate_start_rel_mm=round(x0 * 1000, 3), remate_live=bool(tr["live"]),
                           insertion_force_available_n=[round(min(caps), 1), round(max(caps), 1)] if caps else None,
                           score_after_remate=tr["score"], veto=tr["veto"])


def _t1():
    import importlib.util
    path = Path(os.environ.get("T1_DEMO", Path(__file__).resolve().parent / "livelead_routines.py"))
    spec = importlib.util.spec_from_file_location("t1demo", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Demo(P3e):
    """Full scripted demo (robot commands only): pull the XT60 apart, let go of it, seat black then red banana plug
    (the live-lead task's pick/insert routines), re-grasp the XT60 and re-mate it, hold ENTER 3 s, press ENTER, park,
    hold 6 s."""
    T1 = None

    def plug(self, name):
        return self.poses().plug(name)

    def frames_now(self):
        return self.lk.socket_frames(self.poses())

    def log(self, label, **extra):
        print("LOG", label, json.dumps(extra, default=float), flush=True)

    def joint_go(self, arm, label, duration, q1, grip1):
        q0, g0 = arm.q.copy(), arm.grip
        n = max(1, round(duration * R.HZ))
        from yam_arena import ik
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            arm.command(q0 + s * (np.asarray(q1) - q0), g0 + s * (grip1 - g0))
            self.step(label)

    def mark(self, label):
        tr = self.lk.STATE["trace"][-1]
        row = dict(step=self.steps, label=label, score=tr["score"], veto=tr["veto"], live=tr["live"],
                   xt60_rel_mm=np.round(np.array(tr["xt60_rel"]) * 1000, 2).tolist(), pos=tr["positive"],
                   neg=tr["negative"], lcd=tr["lcd"], page=tr["page"])
        self.stages.append(row)
        print("STAGE", json.dumps(row, default=float), flush=True)

    def away_from_xt60(self):
        """Open wider, then lift straight up (a pad retreating along the pitched approach drags the plug out)."""
        hp, hr = self.left.tcp()
        self.go(self.left, "xt60_open_wide", 0.5, hp, hr, 0.6)
        self.go(self.left, "left_up", 1.5, hp + [0, 0, 0.05], hr)

    def regrasp_xt60(self):
        from yam_arena import ik
        lk = self.lk
        pp, pr = self.plug("main")
        axis = pr[:, 0]
        h = np.array([axis[0], axis[1], 0.0])
        h /= np.linalg.norm(h)
        pitch = math.radians(float(os.environ.get("DEMO_PITCH", "60")))
        a = np.r_[-h[:2] * math.cos(pitch), -math.sin(pitch)]
        x = np.cross(a, [0.0, 0.0, 1.0])
        x /= np.linalg.norm(x)
        rot = np.column_stack([x, np.cross(a, x), a])
        lt = pp + axis * (lk.XT60[0] / 2 + lk.SLEEVE[0] / 2 + float(os.environ.get("PULL_DX", "0.009")))
        ok, _ = self.reachable(self.left, lt, rot)
        if not ok:
            raise R.Stop("re-grasp of the XT60 not reachable")
        self.go(self.left, "xt60_hover", 2.5, lt - a * 0.06, rot, 0.30)
        self.go(self.left, "xt60_approach", 2.0, lt, rot)
        self.go(self.left, "xt60_close", 1.0, lt, rot, 0.0)
        self.wait("xt60_squeeze", 0.5)
        self.go(self.left, "xt60_lift", 1.0, lt + [0, 0, 0.01], rot)

    def press(self, key, hold):
        """Closed fingertips straight down on a key: settle over it on the measured TCP (the arm lags its command by up
        to ~13 mm here), descend with xy feedback until the plunger passes the switch point, keep it pressed for
        ``hold`` s, lift. Stops if a fingertip touches the charger case."""
        from yam_arena.constants import PAD_TIP_PAST_TCP
        from yam_arena.demo import grip_rotation
        lk = self.lk
        cp, cr = self.poses().charger()
        top = cp + cr @ (lk.KEY_POS[key] + [0, 0, 0.0015])
        rot = grip_rotation(0.0)
        above = top + [0, 0, PAD_TIP_PAST_TCP + 0.02]
        arm = self.left
        self.go(arm, f"{key}_hover", 2.0, above, rot, 0.0)
        corr = np.zeros(3)

        def to(label, z_drop):
            nonlocal corr
            target = above - [0, 0, z_drop]
            e = target - arm.measured_tcp()[0]
            if label.endswith("_aim"):  # xy correction only while aiming; frozen while pressing and lifting
                corr[:2] = np.clip(corr[:2] + 0.1 * e[:2], -0.025, 0.025)
            q, pe, re = arm.solve(target + corr, rot)
            arm.command(q, 0.0)
            self.step(label)
            if any(c[1] == "charger/body" and c[2] > 2.0 for c in self.rows[-1]["contacts"]):
                raise R.Stop(f"{label}: fingertip on the charger case")
            return float(np.linalg.norm(e[:2]))

        good = 0
        for _ in range(5 * R.HZ):
            good = good + 1 if to(f"{key}_aim", 0.0) < 0.0008 else 0
            if good >= 5:
                break
        for k in range(1, R.HZ + 1):
            to(f"{key}_descend", 0.02 * k / R.HZ)
        depth = 0.0
        while self.lk.STATE["trace"][-1]["key_travel_max_mm"][key] < 1.0 and depth < 0.030:
            depth += 0.0006
            to(f"{key}_push", 0.02 + depth)
        depth += 0.003  # press on against the bottom stop and keep the plunger down for the whole hold
        for _ in range(max(1, round(hold * R.HZ))):
            if self.lk.STATE["trace"][-1]["key_travel_max_mm"][key] < 1.0 and depth < 0.035:
                depth += 0.0005
            to(f"{key}_hold", 0.02 + depth)
        for k in range(1, R.HZ + 1):
            to(f"{key}_lift", (0.02 + depth) * (1 - k / R.HZ))
        self.mark(f"{key}_pressed")

    def keytest(self):
        lk = self.lk
        self.stages = []
        self.wait("settle", 0.5)
        ch = lk.STATE["charger"]
        self.press("ENTER", 3.3)
        page_long, lcd_long = ch.page, list(ch.display())
        self.wait("after_long", 0.5)
        self.press("ENTER", 0.3)
        self.wait("after_short", 1.5)
        tr = lk.STATE["trace"]
        self.events = dict(page_after_long=page_long, lcd_after_long=lcd_long, page_after_short=ch.page,
                           lcd_after_short=list(ch.display()), charger_events=ch.events,
                           veto=lk.STATE["scorer"].veto, stages=self.stages,
                           passed=bool(page_long == "confirm" and ch.page == "charging" and lk.STATE["scorer"].veto is None))

    def checks(self):
        lk = self.lk
        self.stages = []
        self.wait("settle", 0.5)
        x0 = self.rel()
        self.wait("creep", 30.0)
        x1 = self.rel()
        ch = lk.STATE["charger"]
        self.press("ENTER", 3.3)
        page_long, lcd_long = ch.page, list(ch.display())
        self.wait("after_long", 0.5)
        self.press("ENTER", 0.3)
        self.wait("after_short", 1.5)
        self.events = dict(creep_mm=round((x1 - x0) * 1000, 4), rel_start_mm=round(x0 * 1000, 4),
                           rel_after_30s_mm=round(x1 * 1000, 4), page_after_long=page_long, lcd_after_long=lcd_long,
                           page_after_short=ch.page, lcd_after_short=list(ch.display()), charger_events=ch.events,
                           score=lk.STATE["score"], stages=self.stages)

    def pushrelax(self):
        """Diagnostic (calibration force, arms idle): push the XT60 in with 25 N to near full depth, release, watch
        30 s; log depth, lateral offset, axis tilt and every contact on the plug body."""
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics
        lk = self.lk
        v = lk.STATE["view"]
        stage = get_current_stage()
        male = v.view.prim_paths[v.index["main"]]
        bodies = [str(q.GetPath()) for q in stage.Traverse() if "/env_0/" in str(q.GetPath()) and "Arm" not in str(q.GetPath())
                  and q.HasAPI(UsdPhysics.RigidBodyAPI) and str(q.GetPath()) != male]
        cv = v.sim.create_rigid_contact_view([male], filter_patterns=[bodies], max_contact_data_count=2048)
        log = []

        def tick(label, f):
            lk.STATE["probe_n"] = f
            self.step(label)
            pp, pr = self.poses().plug("main")
            rel = lk.RCLIP.T @ (pp - lk.CLIP)
            tilt = math.degrees(math.acos(float(np.clip(pr[:, 0] @ lk.RCLIP[:, 0], -1, 1))))
            m = cv.get_contact_force_matrix(dt=1.0 / 960).detach().cpu().numpy()[0]
            pairs = [["/".join(bodies[j].split("/")[-2:]), round(float(np.linalg.norm(m[j])), 2)]
                     for j in np.nonzero(np.linalg.norm(m, axis=-1) > 0.01)[0]]
            log.append(dict(step=self.steps, label=label, f=f, rel_mm=np.round(rel * 1000, 4).tolist(),
                            tilt_deg=round(tilt, 3), contacts=pairs))

        self.wait("settle", 0.5)
        for k in range(1, 31):
            tick("push", -25.0 * min(1.0, k / 10))
        for _ in range(30 * R.HZ):
            tick("relax", 0.0)
        lk.STATE["probe_n"] = 0.0
        r = [x["rel_mm"][0] for x in log]
        self.events = dict(min_rel_mm=min(r), rel_at_release_mm=log[29]["rel_mm"][0], rel_end_mm=r[-1],
                           drift_after_release_mm=round(r[-1] - log[29]["rel_mm"][0], 4),
                           tilt_at_release=log[29]["tilt_deg"], tilt_end=log[-1]["tilt_deg"])
        (self.out / "pushrelax.jsonl").write_text("\n".join(json.dumps(x) for x in log) + "\n")

    def driftlog(self):
        """Diagnostic: pull, re-mate and let go as in the demo, then park and hold 30 s; every step log the XT60 depth,
        tilt, pack pose and all contacts on the plug body and on the pack (arm links included)."""
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics
        T1 = self.T1 = self.T1 or _t1()
        Demo.park = T1.Runner.park
        lk = self.lk
        self.stages = []
        self.pull2(float(os.environ.get("DEMO_PITCH", "60")))
        self.events = {}
        self.remate()
        v = lk.STATE["view"]
        stage = get_current_stage()
        male, pack = v.view.prim_paths[v.index["main"]], v.view.prim_paths[v.index["battery"]]
        bodies = [str(q.GetPath()) for q in stage.Traverse() if "/env_0/" in str(q.GetPath())
                  and q.HasAPI(UsdPhysics.RigidBodyAPI) and str(q.GetPath()) not in (male, pack)]
        cv = v.sim.create_rigid_contact_view([male, pack], filter_patterns=[bodies, bodies], max_contact_data_count=4096)
        log = []
        p0 = self.poses().items["battery"][0]

        def tick(label):
            pp, pr = self.poses().plug("main")
            rel = lk.RCLIP.T @ (pp - lk.CLIP)
            tilt = math.degrees(math.acos(float(np.clip(pr[:, 0] @ lk.RCLIP[:, 0], -1, 1))))
            m = cv.get_contact_force_matrix(dt=1.0 / 960).detach().cpu().numpy()
            pairs = [[("plug", "pack")[i], "/".join(bodies[j].split("/")[-2:]), round(float(np.linalg.norm(m[i, j])), 2)]
                     for i in range(2) for j in np.nonzero(np.linalg.norm(m[i], axis=-1) > 0.05)[0]]
            log.append(dict(step=self.steps, label=label, rel_mm=np.round(rel * 1000, 4).tolist(), tilt_deg=round(tilt, 3),
                            pack_shift_mm=np.round((self.poses().items["battery"][0] - p0) * 1000, 3).tolist(),
                            contacts=pairs))

        cmd = self.left.cmd.copy()
        for k in range(1, R.HZ + 1):
            self.left.cmd = np.r_[cmd[:6], cmd[6] + (0.30 - cmd[6]) * k / R.HZ]
            self.step("xt60_let_go")
            tick("xt60_let_go")
        self.left.grip = 0.30
        hp, hr = self.left.tcp()
        from yam_arena import ik
        n = round(2.0 * R.HZ)
        for k in range(1, n + 1):
            s_ = ik.smoothstep(k / n)
            q, pe, re = self.left.solve(hp + [0, 0, 0.05 * max(0.0, (s_ - 0.25) / 0.75)], hr)
            self.left.command(q, 0.30 + 0.30 * min(1.0, s_ / 0.25))
            self.step("left_away")
            tick("left_away")
        from yam_arena.constants import READY
        q0 = self.left.q.copy()
        for k in range(1, round(2.5 * R.HZ) + 1):
            self.left.command(q0 + ik.smoothstep(k / (2.5 * R.HZ)) * (np.array(READY) - q0), 1.0)
            self.step("park")
            tick("park")
        for _ in range(30 * R.HZ):
            self.step("hold")
            tick("hold")
        r = [x["rel_mm"][0] for x in log]
        self.events.update(rel_at_let_go_mm=r[0], rel_after_away_mm=r[R.HZ + n - 1], rel_after_park_mm=r[-30 * R.HZ - 1],
                           rel_end_mm=r[-1], drift_hold_30s_mm=round(r[-1] - r[-30 * R.HZ - 1], 4))
        (self.out / "driftlog.jsonl").write_text("\n".join(json.dumps(x) for x in log) + "\n")

    def bind_t1(self):
        T1 = self.T1 = self.T1 or _t1()
        for name in ("pick", "grasp_rotation", "move_plug", "insert", "release", "park", "touch_from_above", "touch_tip"):
            setattr(Demo, name, getattr(T1.Runner, name))
        self.ik_max = [0.0, 0.0]
        self.stages = []
        return T1

    def neg_hold(self, label, s=6.0):
        self.mark(label)
        self.wait("hold", s)
        self.mark("final")

    def neg_live(self):
        """Live insertion: black plug picked and pushed into its jack while the XT60 is still mated."""
        self.bind_t1()
        self.events = []
        self.pick("negative", jack="negative")
        self.insert("negative", "negative")
        self.release("negative")
        self.park(self.left)
        self.neg_hold("black_seated_live")

    def neg_short(self):
        """Short: the live red pin lowered onto the live black pin (the live-lead task's tip-touch stimulus)."""
        T1 = self.bind_t1()
        self.events = []
        self.pick("positive")
        pn, rn = self.plug("negative")
        self.touch_from_above("positive", pn - rn[:, 0] * 0.008 + [0, 0, T1.PIN_R_M], "tip_to_tip", push=0.003)
        self.neg_hold("pins_touched")

    def neg_wrong(self):
        """Wrong jack: XT60 pulled apart first (dead), then the red plug pushed into the black (-) jack."""
        self.bind_t1()
        self.pull2(float(os.environ.get("DEMO_PITCH", "60")))
        self.events = []
        hp, hr = self.left.tcp()
        self.go(self.left, "xt60_open", 1.0, hp, hr, 0.30)
        self.away_from_xt60()
        self.pick("positive", jack="negative")
        self.insert("positive", "negative", require_seated=False)
        self.neg_hold("red_in_black_jack", 2.0)

    def demo(self, stop=None):
        T1 = self.bind_t1()
        self.ik_max = [0.0, 0.0]
        self.stages = []
        self.pull2(float(os.environ.get("DEMO_PITCH", "60")))
        self.mark("pulled_apart")
        if self.events.get("separated_10mm_step") is None:
            raise R.Stop("XT60 not separated")
        self.events_pull = dict(self.events)
        self.events = []  # the live-lead routines append to a list
        hp, hr = self.left.tcp()
        self.go(self.left, "xt60_open", 1.0, hp, hr, 0.30)
        self.go(self.left, "xt60_away", 1.5, hp - hr[:, 2] * 0.05 + [0, 0, 0.03], hr)
        self.mark("xt60_released")
        self.pick("negative", jack="negative")
        self.insert("negative", "negative")
        self.release("negative")
        self.mark("black_seated")
        self.pick("positive", jack="positive")
        self.insert("positive", "positive")
        self.release("positive", retreat=(-0.04, 0.06, 0.05))
        self.mark("red_seated")
        if stop == "red_seated":  # negative: XT60 never re-mated
            self.park(self.left)
            return self.neg_hold("parked_without_remate")
        self.regrasp_xt60()
        ev = {}
        self.events = ev
        self.remate()
        self.mark("remated")
        cmd = self.left.cmd.copy()  # open while the insertion push is still commanded, so the hand cannot spring back
        for k in range(1, R.HZ + 1):
            self.left.cmd = np.r_[cmd[:6], cmd[6] + (0.30 - cmd[6]) * k / R.HZ]
            self.step("xt60_let_go")
        self.left.grip = 0.30
        held = self.left.cmd.copy()
        free = np.r_[self.left.q + self.left.integ, 0.30]
        for k in range(1, R.HZ + 1):  # take the insertion push off gradually before the hand moves away
            self.left.cmd = held + (free - held) * k / R.HZ
            self.step("push_off")
        self.mark("xt60_let_go")
        self.away_from_xt60()
        self.park(self.left)
        if stop == "let_go":  # negative: charge never started
            return self.neg_hold("parked_without_charge")
        self.press("ENTER", 3.3)
        self.wait("cell_check", 0.5)
        self.press("ENTER", 0.3)
        self.wait("start", 0.5)
        if stop == "keys":  # negative: DEC pressed for 1 s right after the charge starts (inside the 5 s window)
            self.press("DEC", 1.0)
            self.park(self.left)
            self.wait("hold", 2.0)
            self.mark("final")
            self.events = dict(remate=ev, stages=self.stages)
            return
        self.park(self.left)
        self.mark("parked")
        self.wait("hold", float(os.environ.get("DEMO_HOLD_S", "32")))
        self.mark("final")
        self.events = dict(pull=self.events_pull, remate=ev, stages=self.stages)


def main():
    import argparse
    import traceback
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--stages", default="fprobe")
    a = ap.parse_args()
    r = {"pose": P3b, "fixed": P3c, "straight": P3, "posefb": P3d, "lockins": P3e, "demo": Demo}[os.environ.get("REMATE_MODE", "fixed")]()
    for st in a.stages.split(","):
        err = None
        try:
            r.start(Path(a.out) / st, st)
            if st.startswith("idle"):
                r.wait("idle", 1.5)
                ini = r.lk.STATE["initial"]
                r.events = dict(initial_balance=ini["balance"], initial_xt60=ini["xt60"], initial_power=ini["power"],
                                initial_engaged=r.lk.STATE["initial_engaged"], pad_readback=r.lk.STATE.get("pad_readback"))
            elif st == "fprobe":
                r.fprobe()
            elif st == "demo":
                r.demo()
            elif st in ("neg_noremate", "neg_nostart", "neg_keys"):
                r.demo(stop={"neg_noremate": "red_seated", "neg_nostart": "let_go", "neg_keys": "keys"}[st])
            elif st in ("neg_live", "neg_short", "neg_wrong"):
                getattr(r, st)()
            elif st == "checks":
                r.checks()
            elif st == "keytest":
                r.keytest()
            elif st == "pushrelax":
                r.pushrelax()
            elif st == "driftlog":
                r.driftlog()
            else:
                r.pull2(float(st[5:]))
                if (r.events.get("separated_10mm_step") is not None) and os.environ.get("REMATE", "1") == "1":
                    r.remate()
        except Exception as e:  # noqa: BLE001  recorded
            err = e
            traceback.print_exc()
        r.finish(err)
        print("EVENTS", st, json.dumps({k: v for k, v in r.events.items() if k not in ("out", "inward")} if isinstance(r.events, dict) else r.events,
                                       default=float)[:1500], flush=True)
    os._exit(0)


if __name__ == "__main__":
    main()
