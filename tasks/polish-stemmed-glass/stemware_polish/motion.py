"""Calibrated Cartesian program construction using known reset geometry."""

import numpy as np
from yam_arena import ik
from yam_arena.urdf import axis_angle_matrix

from . import config

def rotation(axis, angle):
    a = np.zeros(3)
    a["XYZ".index(axis)] = 1
    return axis_angle_matrix(a, angle)


class Program:
    def __init__(self):
        self.moves = {n: [] for n in ("left", "right")}
        self.pose = {
            n: (*ik.tcp_pose(np.asarray(config.STOWED[n]), config.BASES[n]), 1.0)
            for n in self.moves
        }

    def add(self, duration, label, *, left=None, right=None):
        for name, target in (("left", left), ("right", right)):
            if target is not None:
                p, r, g = target
                self.pose[name] = (np.asarray(p), np.asarray(r), float(g))
            p, r, g = self.pose[name]
            self.moves[name].append(
                {
                    "duration": float(duration),
                    "label": label,
                    "position": np.asarray(p).tolist(),
                    "rotation": np.asarray(r).tolist(),
                    "grip": float(g),
                }
            )
        return self
