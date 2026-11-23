"""State-assisted reference controller for the two stock YAM arms."""
import math

import numpy as np

from task import CRANK_GRIP_OFFSET, WIND_DIRECTION, crank, crank_angle_at, crank_grip


class CrankTracking:
    def __init__(self, plan, asset):
        self.plan = plan
        self.asset = asset
        self.axle = asset.joint_names.index("drum_axle")
        self.release_point = None

    def command(self, duration):
        from yam_arena import ik
        from yam_arena.constants import FINGER_OPEN

        target, _, finger, label = self.plan.target(duration)
        if label in {"approach", "done"} and self.release_point is None:
            return self.plan.action7(duration)[0]
        theta = float(self.asset.data.joint_pos[0, self.axle])
        if label in {"release", "clear", "settle", "done"}:
            if self.release_point is None:
                self.release_point = crank(theta) + CRANK_GRIP_OFFSET
                self.release_angle = theta
            clearing = target - np.asarray(self.plan.waypoints[-3].pos)
            return self._action(self.release_point + clearing, finger / FINGER_OPEN, self.release_angle)
        if label != "crank":
            start = np.asarray(self.plan.waypoints[1].pos)
            aim = crank(theta) + CRANK_GRIP_OFFSET
            goal = target + ik.smoothstep(min(1.0, max(0.0, duration - 2.0))) * (aim - start)
            return self._action(goal, finger / FINGER_OPEN, theta)
        nominal = crank_angle_at(target - CRANK_GRIP_OFFSET)
        nominal += math.tau * round((theta - nominal) / math.tau)
        lead = min(0.30, 0.06 + (duration - 3.8) * 0.10)
        delta = WIND_DIRECTION * (nominal - theta)
        theta += WIND_DIRECTION * (np.clip(delta, 0.0, lead) if abs(delta) < 0.35 else lead)
        goal = crank(theta) + CRANK_GRIP_OFFSET
        return self._action(goal, finger / FINGER_OPEN, theta)

    def _action(self, goal, finger, theta):
        from yam_arena import ik
        r = crank_grip(theta).rot
        self.plan.q, _, _ = ik.solve(goal, ik.matrix_to_quat_xyzw(r), self.plan.q,
                                    self.plan.base_pos, iters=60)
        return np.r_[self.plan.q, finger]


class GuideTracking:
    """Gauge the settled inlet path before closing the guide around the cord."""
    def __init__(self, plan, asset, origin):
        from task import CENTER, EXIT_AXIS, LINKS

        self.plan = plan
        self.asset = asset
        self.origin = origin
        self.axis = EXIT_AXIS
        self.reference = np.asarray(plan.waypoints[1].pos)
        self.longitudinal = (self.reference - CENTER) @ EXIT_AXIS
        self.center = CENTER
        self.indices = [asset.body_names.index(f"cord_{i:02d}") for i in range(LINKS)]
        self.offset = np.zeros(3)

    def command(self, duration):
        from yam_arena import ik
        from yam_arena.constants import FINGER_OPEN

        target, rotation, finger, label = self.plan.target(duration)
        if label == "above incoming cord":
            return self.plan.action7(duration)[0]

        if label == "straddle cord":
            positions = self.asset.data.body_pos_w
            points = getattr(positions, "torch", positions)[0, self.indices].detach().cpu().numpy()
            points = points - self.origin
            along = (points - self.center) @ self.axis
            crossings = np.flatnonzero((along[:-1] <= self.longitudinal) & (along[1:] > self.longitudinal))
            if crossings.size:
                i = crossings[0]
                fraction = (self.longitudinal - along[i]) / (along[i + 1] - along[i])
                expected = points[i] + fraction * (points[i + 1] - points[i])
                adjustment = np.clip(expected - self.reference, -0.03, 0.03)
                self.offset = ik.smoothstep(min(1.0, max(0.0, duration - 2))) * adjustment

        target = target + self.offset
        self.plan.q, _, _ = ik.solve(target, ik.matrix_to_quat_xyzw(rotation), self.plan.q,
                                    self.plan.base_pos, iters=60)
        return np.r_[self.plan.q, finger / FINGER_OPEN]


class ForceTracking(CrankTracking):
    def command(self, duration):
        from yam_arena import ik
        from yam_arena.constants import FINGER_OPEN, SHOULDER, WRIST

        _, _, finger, label = self.plan.target(duration)
        if label != "crank":
            return super().command(duration)

        angle = float(self.asset.data.joint_pos[0, self.axle])
        velocity = float(self.asset.data.joint_vel[0, self.axle])
        point = crank(angle + WIND_DIRECTION * 0.035) + CRANK_GRIP_OFFSET
        command = self._action(point, finger / FINGER_OPEN, angle)
        q = command[:6]
        base = self.plan.base_pos
        tcp, _ = ik.tcp_pose(q, base)
        delta = 1e-5
        jac = np.column_stack([
            (ik.tcp_pose(q + row * delta, base)[0] - tcp) / delta
            for row in np.eye(6)
        ])
        tangent = crank(angle + WIND_DIRECTION * delta) - crank(angle)
        tangent /= np.linalg.norm(tangent)
        force = np.clip(6.5 - 3.5 * WIND_DIRECTION * velocity, 0, 8) * ik.smoothstep(min(1, (duration - 3.8) / 0.75))
        stiffness = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
        command[:6] = np.clip(q + (jac.T @ (force * tangent)) / stiffness, ik.LOWER, ik.UPPER)
        return command

def install_policy(mode="force"):
    """Use the reference controller when Inspect Robots loads the scripted policy."""
    from inspect_robots.types import Action, ActionChunk
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY
    from yam_arena.scripted import YamScriptedPolicy

    from scenekit import env

    controller = {"force": ForceTracking, "tracked": CrankTracking}[mode]
    original_reset = YamScriptedPolicy.reset

    def reset_tracking(self, scene):
        self._cord_tracking = None
        self._cord_guide = None
        original_reset(self, scene)

    def act_tracking(self, observation):
        t = float(observation.state_time) + self._dt
        if self._cord_tracking is None:
            scene = env().unwrapped.scene
            asset = scene["reel"]
            self._cord_tracking = controller(self._plans["right"], asset)
            if "left" in self._plans:
                origins = scene.env_origins
                origin = getattr(origins, "torch", origins)[0].detach().cpu().numpy()
                self._cord_guide = GuideTracking(self._plans["left"], asset, origin)
        left = self._cord_guide.command(t) if self._cord_guide else np.r_[READY, 1.0]
        right = self._cord_tracking.command(t)
        meta = {"request_stop": True} if t > self._end + self.hold_s else {}
        return ActionChunk(actions=[Action(data=pack(left, right), meta=meta)], control_hz=1.0 / self._dt)

    YamScriptedPolicy.reset = reset_tracking
    YamScriptedPolicy.act = act_tracking
