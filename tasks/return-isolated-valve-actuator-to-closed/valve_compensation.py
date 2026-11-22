"""Feed-forward holding moment expressed as YAM joint position offsets on the override hand."""

import math

import numpy as np
from valve_layout import knob
from yam_arena import ik


class LoadedCrankPlan:
    def __init__(self, plan, side, feedback):
        self.feedback = feedback
        self.filtered_speed = 0.0
        self.plan = plan
        self.side = side
        self.base_target = plan.target
        elapsed = 0.0
        self.arc = None
        for i, w in enumerate(plan.waypoints):
            if w.label == "guide return":
                j = i
                while (
                    j + 1 < len(plan.waypoints)
                    and plan.waypoints[j + 1].label == "guide return"
                ):
                    j += 1
                start = plan.waypoints[i - 1].pos
                end = plan.waypoints[j].pos
                from valve_layout import ORIGIN

                self.arc = (
                    elapsed,
                    sum(k.duration for k in plan.waypoints[i : j + 1]),
                    math.atan2(start[1] - ORIGIN[1], start[0] - ORIGIN[0]),
                    math.atan2(end[1] - ORIGIN[1], end[0] - ORIGIN[0]),
                )
                break
            elapsed += w.duration
        plan.target = self.target

    def target(self, t):
        p, rot, g, label = self.base_target(t)
        if self.arc and label == "guide return":
            from valve_layout import ORIGIN

            start, duration, a, b = self.arc
            u = min(1.0, max(0.0, (t - start) / duration))
            smooth = u**3 * (10 + u * (-15 + 6 * u))
            angle = a + math.atan2(math.sin(b - a), math.cos(b - a)) * smooth
            rad = math.hypot(*knob(self.side, 0)[:2])
            p = np.asarray(
                [
                    ORIGIN[0] + rad * math.cos(angle),
                    ORIGIN[1] + rad * math.sin(angle),
                    p[2],
                ]
            )
        return p, rot, g, label

    def __getattr__(self, name):
        return getattr(self.plan, name)

    def action7(self, t):
        cmd, label = self.plan.action7(t)
        strength = 0.0
        elapsed = 0.0
        for wp in self.plan.waypoints:
            if wp.label == "take load":
                strength = min(1.0, max(0.0, (t - elapsed) / wp.duration))
            if wp.label in ("release knob", "reload pin"):
                strength *= max(0.0, min(1.0, 1 - (t - elapsed) / wp.duration))
            elapsed += wp.duration
        if strength == 0 or label == "done":
            return cmd, label
        q = cmd[:6].copy()
        p, _ = ik.tcp_pose(q, self.plan.base_pos)
        target_p = self.plan.target(t)[0]
        from valve_layout import ORIGIN

        xy = np.asarray(target_p[:2]) - ORIGIN[:2]
        r2 = xy @ xy
        k = knob(self.side, 0)
        delta = math.atan2(k[0] * xy[1] - k[1] * xy[0], k[0] * xy[0] + k[1] * xy[1])
        torque = max(0.0, 3.0 * (45 + math.degrees(delta)) / 45) * strength
        angle, speed = self.feedback()
        self.filtered_speed = 0.75 * self.filtered_speed + 0.25 * speed
        correction = (
            0.22 * (45 + math.degrees(delta) - angle) - 0.009 * self.filtered_speed
        )
        torque += strength * float(np.clip(correction, -0.80, 0.80))
        force = torque * np.array([-xy[1], xy[0], 0.0]) / r2
        for j, kp in enumerate((80.0, 80.0, 80.0, 10.0, 10.0, 10.0)):
            q1 = q.copy()
            q1[j] += 1e-5
            dp = (ik.tcp_pose(q1, self.plan.base_pos)[0] - p) / 1e-5
            cmd[j] += np.dot(dp, force) / kp
        cmd[:6] = np.clip(cmd[:6], ik.LOWER, ik.UPPER)
        return cmd, label
