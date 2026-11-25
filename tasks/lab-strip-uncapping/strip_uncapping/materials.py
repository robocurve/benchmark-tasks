"""Compliant plug resistance and water conservation under carrier movement."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C

CAP_ATTACHMENTS = np.array([[0.0, y, -0.003] for y in C.WELL_Y])
TUBE_RIMS = np.array([[0.0, y, C.RIM_Z] for y in C.WELL_Y])
AREA = np.pi * C.WELL_RADIUS**2
BORE_HEIGHT = C.RIM_Z - C.WELL_BOTTOM


class CapFits:
    def __init__(self):
        self.reset()

    def reset(self):
        self.remaining_ul = np.full(len(C.WELL_Y), C.WATER_UL)
        self.peak_force_n = 0.0
        self.max_lift = 0.0
        self.max_shift = 0.0
        self.max_tilt = 0.0

    def advance(self, rack, rack_v, cap, cap_v):
        rack_axes = quat_xyzw_to_matrix(rack[3:])
        cap_axes = quat_xyzw_to_matrix(cap[3:])
        rims = rack[:3] + TUBE_RIMS @ rack_axes.T
        plugs = cap[:3] + CAP_ATTACHMENTS @ cap_axes.T
        displacement = (plugs - rims) @ rack_axes
        gaps = displacement[:, 2]
        offset = np.linalg.norm(displacement[:, :2], axis=1)
        fit = (gaps > 0.0) & (gaps < C.RELEASE_TRAVEL) & (offset < 0.0015)
        axial = np.einsum(
            "ij,j->i",
            cap_v[:3]
            + np.cross(cap_v[3:], plugs - cap[:3])
            - rack_v[:3]
            - np.cross(rack_v[3:], rims - rack[:3]),
            rack_axes[:, 2],
        )
        # A local elastic slope ends in sliding release over the engaged length.
        # The force acts at each plug and is reacted by its tube.
        peak_gap = 0.00105
        resistance = np.where(
            fit,
            np.maximum(
                0.0,
                np.minimum(
                    550.0 * gaps,
                    0.58 * (C.RELEASE_TRAVEL - gaps) / (C.RELEASE_TRAVEL - peak_gap),
                )
                + 0.014 * axial,
            ),
            0.0,
        )
        local_loads = resistance[:, None] * rack_axes[:, 2]
        rack_load = local_loads.sum(axis=0)
        self.peak_force_n = max(self.peak_force_n, float(np.linalg.norm(rack_load)))
        # Open wells retain the volume below the lower lip for the current
        # gravity direction. Sealed wells keep their original contents.
        slope = np.linalg.norm(rack_axes[2, :2]) / max(1.0e-6, rack_axes[2, 2])
        height = max(0.0, BORE_HEIGHT - C.WELL_RADIUS * slope)
        open_wells = (gaps >= C.RELEASE_TRAVEL) | (offset > 0.0015)
        self.remaining_ul[open_wells] = np.minimum(
            self.remaining_ul[open_wells], height * AREA * 1.0e9
        )
        self.max_lift = max(self.max_lift, float(rack[2] - C.RACK[2]))
        self.max_shift = max(
            self.max_shift, float(np.linalg.norm(rack[:2] - C.RACK[:2]))
        )
        self.max_tilt = max(
            self.max_tilt, float(np.arccos(np.clip(rack_axes[2, 2], -1.0, 1.0)))
        )
        torque_rack = np.cross(
            rims - (rack[:3] + rack_axes @ C.RACK_COM), local_loads
        ).sum(axis=0)
        torque_cap = np.cross(
            plugs - (cap[:3] + cap_axes @ C.CAP_COM), -local_loads
        ).sum(axis=0)
        return np.stack((rack_load, -rack_load)), np.stack((torque_rack, torque_cap))

    def snapshot(self):
        return {
            "remaining_ul": self.remaining_ul.tolist(),
            "peak_release_force_n": self.peak_force_n,
            "max_rack_lift_m": self.max_lift,
            "max_rack_shift_m": self.max_shift,
            "max_rack_tilt_deg": float(np.rad2deg(self.max_tilt)),
        }
