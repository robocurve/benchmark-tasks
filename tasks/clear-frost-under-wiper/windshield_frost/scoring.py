"""Area of contacted, swept frost and a supported finish."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


class Scorer:
    def __init__(self):
        x = np.linspace(C.BAND_X0, C.BAND_X1, C.CELLS_X, endpoint=False)
        y = np.linspace(C.BAND_Y0, C.BAND_Y1, C.CELLS_Y, endpoint=False)
        x += (C.BAND_X1 - C.BAND_X0) / (2 * C.CELLS_X)
        y += (C.BAND_Y1 - C.BAND_Y0) / (2 * C.CELLS_Y)
        self.centers = np.stack(np.meshgrid(x, y, indexing="ij"), axis=-1)
        self.reset()

    def reset(self):
        self.cleared = np.zeros(self.centers.shape[:2], dtype=bool)
        self.previous_edge = None
        self.previous_force = 0.
        self.previous_lift = None
        self.opened = False
        self.snapped = False
        self.finish_started = None

    def _edge_sweep(self, tool, force):
        pose = np.asarray(tool["links"]["Blade"])
        rotation = quat_xyzw_to_matrix(pose[3:])
        direction = C.GLASS_ROT.T @ rotation[:, 0]
        lateral = C.GLASS_ROT.T @ rotation[:, 1]
        edge = C.GLASS_ROT.T @ (
            pose[:3] + C.EDGE_X * rotation[:, 0] - C.GLASS_ORIGIN
        )
        prior = self.previous_edge
        prior_force = self.previous_force
        self.previous_edge = edge
        self.previous_force = force
        # The bevel must trail the plastic edge, with actual plane load and
        # positive travel along the scraper's long axis. Both ends of the
        # stroke segment must have edge load to count the swept area.
        attack = np.degrees(np.arctan2(-direction[2], np.linalg.norm(direction[:2])))
        if (
            not 19. <= attack <= 52.
            or abs(edge[2]) > .0016
            or abs(lateral[2]) > .06
            or prior is None
            or not C.FORCE_MIN <= force <= C.FORCE_MAX
            or not C.FORCE_MIN <= prior_force <= C.FORCE_MAX
        ):
            return edge, attack

        advance = edge[:2] - prior[:2]
        norm = np.linalg.norm(advance)
        if not .00012 <= norm <= .018 or np.dot(advance, direction[:2]) < .72 * norm:
            return edge, attack

        lateral_segment = lateral[:2] * C.EDGE_WIDTH
        determinant = np.linalg.det(np.stack([advance, lateral_segment]))
        if abs(determinant) < 1.e-7:
            return edge, attack
        # Solve for each center in the swept parallelogram of the flat edge.
        rel = self.centers - prior[:2]
        u = (rel[..., 0] * lateral_segment[1] - rel[..., 1] * lateral_segment[0]) / determinant
        v = (advance[0] * rel[..., 1] - advance[1] * rel[..., 0]) / determinant
        self.cleared |= (u >= -.22) & (u <= 1.22) & (np.abs(v) <= .47)
        return edge, attack

    def evaluate(self, state):
        tool = state["objects"]["scraper"]
        fixture = state["objects"]["fixture"]
        lift = -np.rad2deg(fixture["joints"]["Lift"])
        speed = np.rad2deg(fixture["joint_speed"]["Lift"])
        force = max(0., tool["joints"]["Flex"] * C.FLEX_K)
        self.opened |= lift >= 9.
        if self.previous_lift is not None:
            return_rate = max(speed, (self.previous_lift - lift) * 20.)
            self.snapped |= self.opened and lift <= 5. and return_rate > 43.
        self.previous_lift = lift
        edge, attack = self._edge_sweep(tool, force)
        coverage = float(self.cleared.mean())
        supported = abs(lift) <= 2.01 and abs(speed) < 3.0
        tool_pose = np.asarray(tool["pose"])
        parked = (
            abs(tool_pose[2] - C.SCRAPER_START[2]) < .012
            and -.60 < tool_pose[1] < .60
            and np.linalg.norm(tool["velocity"][:3]) < .015
        )
        arm = np.asarray(fixture["links"]["Arm"])
        arm_rotation = quat_xyzw_to_matrix(arm[3:])
        held_range = arm[:3] + arm_rotation @ C.GLASS_ROT @ C.ARM_GRIP
        clear = all(
            np.linalg.norm(np.asarray(point) - held_range) > .076
            and np.linalg.norm(np.asarray(point) - tool_pose[:3]) > .071
            for hand in state["hands"].values() for point in hand["points"]
        )
        now = state["time_s"]
        finish = coverage >= .95 and supported and parked and clear and not self.snapped
        if finish:
            if self.finish_started is None:
                self.finish_started = now
        else:
            self.finish_started = None
        stable_s = 0. if self.finish_started is None else now - self.finish_started
        success = stable_s >= .999
        score = 1.0 if success else round(.8 * coverage, 4)
        return dict(
            score=score, success=success, cleared_fraction=coverage,
            cleared_cells=int(self.cleared.sum()), lift_deg=float(lift),
            lift_closing_speed_deg_s=float(speed), tool_contact_n=force,
            edge_glass_xyz=edge.tolist(), scraper_angle_deg=float(attack),
            wiper_supported=bool(supported), scraper_parked=bool(parked),
            grippers_clear=bool(clear), hard_return=bool(self.snapped), stable_s=stable_s,
        )
