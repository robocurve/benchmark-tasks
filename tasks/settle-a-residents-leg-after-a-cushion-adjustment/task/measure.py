"""Source-specific solver contacts and the lower heel's ellipsoid surface height."""

import mujoco as mj
import numpy as np


class Reader:
    def __init__(self, model):
        self.names = [model.geom(i).name for i in range(model.ngeom)]
        self.heel = model.geom("heel").id
        self.size = model.geom("heel").size.copy()
        self.support_ids = [
            model.geom(k).id for k in ["bed", "sole"] + [f"pad{i}" for i in range(6)]
        ]

        self.distal = [
            model.geom("arm_" + k).id
            for k in ("link4", "link5", "gripper", "tip_left", "tip_right")
        ]
        self.care = [
            model.geom(k).id
            for k in ("heel", "achilles", "calf_prox", "calf_dist", "forefoot")
        ] + self.support_ids[1:]

    def robot_person(self, m, d):
        total = 0.0
        force = np.zeros(6)
        jaws = {}
        for i in range(d.ncon):
            c = d.contact[i]
            a, b = (self.names[g] for g in c.geom)
            sign = 1.0
            if a.startswith("arm_"):
                a, b = b, a
                sign = -1.0
            if b.startswith("arm_") and a in {
                "heel",
                "achilles",
                "calf_prox",
                "calf_dist",
                "forefoot",
                "head",
                "torso",
                "thigh",
                "knee_shell",
                "far_leg",
                "far_foot",
                "arm_rest",
            }:
                mj.mj_contactForce(m, d, i, force)
                normal = max(0.0, force[0])
                total += normal
                if b in ("arm_tip_left", "arm_tip_right"):
                    region = (
                        "lower_leg"
                        if a
                        in {"heel", "achilles", "calf_prox", "calf_dist", "forefoot"}
                        else a
                    )
                    pair = jaws.setdefault(region, [np.zeros(3), np.zeros(3)])
                    pair[int(b == "arm_tip_right")] += sign * normal * c.frame[:3]
        pinch = 0.0
        for left, right in jaws.values():
            denom = np.linalg.norm(left) * np.linalg.norm(right)
            if denom and np.dot(left, right) / denom < -0.5:
                pinch = max(pinch, min(np.linalg.norm(left), np.linalg.norm(right)))
        return total, pinch

    def sample(self, m, d, assist_nm):
        force = np.zeros(6)
        strip_forces = np.zeros(6)
        loads = {
            "knee_pad_n": 0.0,
            "heel_n": 0.0,
            "achilles_pad_n": 0.0,
            "prox_n": 0.0,
            "dist_n": 0.0,
            "bed_pad_n": 0.0,
            "robot_person_n": 0.0,
            "robot_pad_n": 0.0,
        }
        people = {
            "heel",
            "achilles",
            "calf_prox",
            "calf_dist",
            "forefoot",
            "head",
            "torso",
            "thigh",
            "knee_shell",
            "far_leg",
            "far_foot",
            "arm_rest",
        }
        for i in range(d.ncon):
            c = d.contact[i]
            a, b = (self.names[g] for g in c.geom)
            mj.mj_contactForce(m, d, i, force)
            normal = float(max(0.0, force[0]))
            pad = a.startswith("pad") or b.startswith("pad") or "sole" in (a, b)
            arm = a.startswith("arm_") or b.startswith("arm_")
            pair = {a, b}
            if "heel" in pair and (pad or arm or "bed" in pair):
                loads["heel_n"] += normal
            if pad:
                upward = float((c.frame.reshape(3, 3).T @ force[:3])[2])
                for k, label in [("calf_prox", "prox_n"), ("calf_dist", "dist_n")]:
                    if k in pair:
                        load = upward if b == k else -upward
                        loads[label] += load
                        other = a if b == k else b
                        if other.startswith("pad"):
                            strip_forces[int(other[3:])] += load
                if "knee_shell" in pair:
                    loads["knee_pad_n"] += normal
                if "achilles" in pair:
                    loads["achilles_pad_n"] += normal
                if "bed" in pair:
                    loads["bed_pad_n"] += upward if a == "bed" else -upward
            if arm and people.intersection(pair):
                loads["robot_person_n"] += normal
            if arm and pad:
                loads["robot_pad_n"] += normal
        for key in ("prox_n", "dist_n", "bed_pad_n"):
            loads[key] = max(0.0, loads[key])
        rot = d.geom_xmat[self.heel].reshape(3, 3)
        low = d.geom_xpos[self.heel, 2] - np.linalg.norm(rot[2, :] * self.size)
        gap = min(
            mj.mj_geomDistance(m, d, self.heel, i, 0.5, None) for i in self.support_ids
        )
        distal_gap = min(
            mj.mj_geomDistance(m, d, a, b, 0.12, None)
            for a in self.distal
            for b in self.care
        )
        active = np.flatnonzero(strip_forces >= 1.0)
        span = float(43 * (active[-1] - active[0])) if len(active) > 1 else 0.0
        return dict(
            **loads,
            support_span_mm=span,
            arm_clear_mm=float(1000 * distal_gap),
            heel_gap_mm=float(1000 * low),
            free_gap_mm=float(1000 * gap),
            assist_nm=float(assist_nm),
            pad_xyz=d.body("cushion").xpos.tolist(),
            tcp=d.site("tcp").xpos.tolist(),
            knee_rad=float(d.joint("knee").qpos[0]),
            knee_velocity=float(d.joint("knee").qvel[0]),
            cushion_speed=float(np.linalg.norm(d.joint("free_cushion").qvel[:3])),
        )
