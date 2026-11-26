"""Score the current uninterrupted interval of an unsupported, load-bearing tent."""

import numpy as np

from yam_arena.constants import TABLE_HEIGHT
from yam_arena.urdf import quat_xyzw_to_matrix

from geometry import CHASSIS_Z, DESK_CENTER, DESK_SIZE, DISPLAY_Z, FRONT_X, MASS, PHYSICS_HZ, WIDTH


WEIGHT = sum(MASS) * 9.81
MIN_EDGE_LOAD = 0.15 * WEIGHT
MIN_TOTAL_EDGE_LOAD = 0.90 * WEIGHT
OTHER_CONTACT_LIMIT = 0.05  # N, summed normal force
EDGE_BAND = 0.005
EDGE_HEIGHT_TOL = 0.004
HOLD_S = 2.0
PARTIAL_HOLD_S = 0.5
FRONT_CORNERS = tuple(np.array([(FRONT_X, y, z) for y in (-WIDTH / 2, WIDTH / 2)])
                      for z in (CHASSIS_Z[0], DISPLAY_Z[1]))


class TentScore:
    """Physics-rate measurements are read-only. All credit expires when the pose fails."""

    def __init__(self):
        self.samples = self.ticks = 0
        self.score = 0.0
        self.metrics = {}

    def sample(self, poses, velocities, angle, desk_contacts, robot_force, other_force):
        self.ticks += 1
        if not (np.isfinite(poses).all() and np.isfinite(velocities).all() and np.isfinite(angle)
                and np.isfinite([robot_force, other_force]).all()
                and all(np.isfinite(np.c_[p, f]).all() for p, f in desk_contacts)):
            self.samples = 0
            self.score = 0.0
            self.metrics = {}
            return

        rotations = [quat_xyzw_to_matrix(p[3:]) for p in poses]
        corners = np.array([c @ r.T + p[:3] for c, r, p in zip(FRONT_CORNERS, rotations, poses, strict=True)])
        edge_heights = corners[..., 2] - TABLE_HEIGHT
        within_desk = np.all(np.abs(corners[..., :2] - DESK_CENTER[:2]) <= np.asarray(DESK_SIZE[:2]) / 2)
        feet_mid = corners.mean(axis=1)
        span = feet_mid[1, :2] - feet_mid[0, :2]
        separation = np.linalg.norm(span)
        screen_normal = -rotations[1][:, 2]
        outward = float(screen_normal[:2] @ span / max(separation, 1e-9))
        ridge_height = float(min(poses[:, 2]) - corners[..., 2].max())
        shape = bool(238. <= angle <= 304. and 0.10 <= separation <= 0.36 and ridge_height >= 0.125
                     and outward >= 0.55 and screen_normal[2] >= 0.15 and within_desk
                     and np.all(np.abs(edge_heights) <= EDGE_HEIGHT_TOL))

        edge_loads = []
        off_edge_force = 0.0
        for (points, forces), p, r in zip(desk_contacts, poses, rotations, strict=True):
            local_points = (points - p[:3]) @ r
            on_edge = ((np.abs(local_points[:, 0] - FRONT_X) <= EDGE_BAND)
                       & (np.abs(local_points[:, 1]) <= WIDTH / 2 + 0.001))
            edge_loads.append(float(np.maximum(forces[on_edge, 2], 0.).sum()))
            off_edge_force += float(np.linalg.norm(forces[~on_edge], axis=1).sum())

        bearing = min(edge_loads) >= MIN_EDGE_LOAD and sum(edge_loads) >= MIN_TOTAL_EDGE_LOAD
        released = robot_force < OTHER_CONTACT_LIMIT and other_force + off_edge_force < OTHER_CONTACT_LIMIT
        speed = float(np.max(np.linalg.norm(velocities[:, :3], axis=1)))
        spin = float(np.max(np.linalg.norm(velocities[:, 3:], axis=1)))
        still = speed <= 0.008 and spin <= 0.10
        qualified = shape and bearing and released and still
        self.samples = self.samples + 1 if qualified else 0
        seconds = max(0, self.samples - 1) / PHYSICS_HZ
        self.score = 1.0 if seconds >= HOLD_S else 0.5 if seconds >= PARTIAL_HOLD_S else 0.0
        self.metrics = dict(
            hinge_deg=angle, ridge_mm=ridge_height * 1000, edge_height_mm=(edge_heights * 1000).tolist(),
            edge_separation_mm=float(separation * 1000), screen_outward_cos=outward,
            edge_load_N=edge_loads, robot_force_N=robot_force, other_force_N=other_force,
            off_edge_force_N=off_edge_force, max_speed_m_s=speed, max_spin_rad_s=spin,
            shape=shape, bearing=bearing, released=released, still=still, qualified=qualified, hold_s=seconds,
        )
