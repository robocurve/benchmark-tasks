"""Numeric end-state scoring, independent of Isaac Sim and the demonstration policy."""

from dataclasses import asdict, dataclass
import math

import numpy as np


@dataclass(frozen=True)
class Rules:
    minimum_depth_m: float = 0.140
    maximum_depth_m: float = 0.157
    maximum_tilt_deg: float = 10.0
    maximum_heading_error_deg: float = 10.0
    slot_error_x_m: float = 0.002
    # The 46 mm opening has 5 mm clearance per side around the 36 mm
    # blade section passing through the deck. A released blade can lean
    # against that edge while remaining fully inserted.
    slot_error_y_m: float = 0.005
    maximum_speed_m_s: float = 0.010
    maximum_spin_rad_s: float = 0.5
    release_distance_m: float = 0.080
    holder_clearance_m: float = 0.005
    table_edge_margin_m: float = 0.010
    table_penetration_tolerance_m: float = 0.002
    table_rest_gap_m: float = 0.006
    hold_s: float = 1.0


@dataclass
class BodyState:
    pose: np.ndarray  # root position + XYZW orientation
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray
    low: np.ndarray  # exact collider support extrema, not a rotated asset box
    high: np.ndarray


def rotation(q):
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)],
    ])


class TaskScorer:
    def __init__(self, layout, holder_low, holder_high, table_low, table_high, *, hz=20, rules=None):
        self.rules = rules or Rules()
        self.hz = hz
        self.slot_xy = np.array(layout["target_knife_tip_world"][:2], float)
        self.slot_z = float(layout["target_knife_tip_world"][2] + 0.154)
        self.table_z = float(layout["table_top_z"])
        self.holder_low, self.holder_high = np.asarray(holder_low), np.asarray(holder_high)
        self.table_low, self.table_high = np.asarray(table_low), np.asarray(table_high)
        self.reset()

    def reset(self):
        self._candidate, self._consecutive = 0.0, 0
        self.score = 0.0

    def configuration(self):
        return asdict(self.rules)

    def evaluate(self, knife, hamster, tcps):
        r = self.rules
        arrays = [knife.pose, knife.linear_velocity, knife.angular_velocity, knife.low, knife.high,
                  hamster.pose, hamster.linear_velocity, hamster.angular_velocity, hamster.low, hamster.high,
                  *tcps.values()]
        if not all(np.isfinite(v).all() for v in arrays) or any(
            np.linalg.norm(b.pose[3:]) < 1e-8 for b in (knife, hamster)
        ):
            return 0.0, {"valid_state": False}
        rk = rotation(knife.pose[3:])
        axis = rk[:, 2]
        tilt = float(np.degrees(np.arccos(np.clip(axis[2], -1.0, 1.0))))
        heading = float(np.degrees(np.arccos(np.clip(abs(rk[1, 0]), 0.0, 1.0))))
        depth = self.slot_z - float(knife.pose[2])
        if axis[2] > 0.5:
            opening = knife.pose[:3] + axis * (self.slot_z - knife.pose[2]) / axis[2]
            offset = np.abs(opening[:2] - self.slot_xy)
        else:
            offset = np.array([1e6, 1e6])

        def resting(body):
            return bool(np.linalg.norm(body.linear_velocity) <= r.maximum_speed_m_s
                        and np.linalg.norm(body.angular_velocity) <= r.maximum_spin_rad_s)

        knife_handle = knife.pose[:3] + rk @ np.array([0.0, 0.0, 0.215])
        knife_released = bool(tcps and all(np.linalg.norm(t - knife_handle) >= r.release_distance_m for t in tcps.values()))
        hamster_center = (hamster.low + hamster.high) / 2
        hamster_released = bool(tcps and all(np.linalg.norm(t - hamster_center) >= r.release_distance_m for t in tcps.values()))
        hamster_on_table_xy = bool(np.all(hamster.low[:2] >= self.table_low[:2] + r.table_edge_margin_m)
                                   and np.all(hamster.high[:2] <= self.table_high[:2] - r.table_edge_margin_m))
        hamster_supported = bool(self.table_z - r.table_penetration_tolerance_m <= hamster.low[2]
                                 <= self.table_z + 0.014 + r.table_rest_gap_m)
        valid = bool(hamster_on_table_xy and hamster_supported
                     and knife.low[2] >= self.table_z - r.table_penetration_tolerance_m)
        inserted = bool(
            valid and r.minimum_depth_m <= depth <= r.maximum_depth_m
            and tilt <= r.maximum_tilt_deg and heading <= r.maximum_heading_error_deg
            and offset[0] <= r.slot_error_x_m and offset[1] <= r.slot_error_y_m
            and resting(knife) and knife_released
        )
        separated = bool(np.any(hamster.high[:2] <= self.holder_low[:2] - r.holder_clearance_m)
                         or np.any(hamster.low[:2] >= self.holder_high[:2] + r.holder_clearance_m))
        on_table = bool(self.table_z - r.table_penetration_tolerance_m <= hamster.low[2]
                        <= self.table_z + r.table_rest_gap_m)
        removed = bool(separated and on_table and hamster_on_table_xy and resting(hamster) and hamster_released)
        raw = 1.0 if inserted and removed else (0.5 if inserted else 0.0)
        metrics = {
            "valid_state": valid, "knife_inserted": inserted, "hamster_removed": removed,
            "depth_mm": depth * 1000, "tilt_deg": tilt, "heading_error_deg": heading,
            "slot_error_mm": (offset * 1000).tolist(), "knife_released": knife_released,
            "knife_speed_m_s": float(np.linalg.norm(knife.linear_velocity)),
            "hamster_on_table": on_table and hamster_on_table_xy,
            "hamster_released": hamster_released, "hamster_outside_holder": separated,
        }
        return raw, metrics

    def update(self, knife, hamster, tcps):
        candidate, metrics = self.evaluate(knife, hamster, tcps)
        if candidate == 0:
            self._candidate, self._consecutive, self.score = 0.0, 0, 0.0
        else:
            self._consecutive = self._consecutive + 1 if candidate == self._candidate else 1
            self._candidate = candidate
            self.score = candidate if self._consecutive >= math.ceil(self.rules.hold_s * self.hz) else 0.0
        return {"score": self.score, "success": self.score == 1.0,
                "candidate_score": candidate, "held_steps": self._consecutive, **metrics}
