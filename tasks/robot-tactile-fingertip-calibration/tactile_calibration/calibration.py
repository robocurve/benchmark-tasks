"""Measured ball position and spring load drive the synthetic optical images."""

import numpy as np

from . import config as C

Y, X = np.mgrid[-0.015:0.015:90j, -0.013:0.013:78j]
LIGHTS = np.array([[0.62, 0.0, 0.78], [-0.32, 0.56, 0.77], [-0.32, -0.56, 0.77]])


def optical_impression(center, depth):
    """Ideal diffuse tri-color lighting of the loaded spherical cap."""
    dx, dy = X - center[0], Y - center[1]
    lateral = dx**2 + dy**2
    radius2 = C.BALL_R**2
    edge2 = max(0.0, radius2 - (C.BALL_R - depth) ** 2)
    inside = (lateral <= edge2) & (depth > 0)
    z = np.sqrt(np.clip(radius2 - lateral, 0, radius2))
    normals = np.stack((-dx / C.BALL_R, -dy / C.BALL_R, z / C.BALL_R), axis=-1)
    normals[~inside] = [0.0, 0.0, 1.0]
    image = np.uint8(np.clip(16 + 217 * (normals @ LIGHTS.T), 0, 255))
    return image, int(inside.sum())


class Calibration:
    def __init__(self):
        self.frames = {}
        self.images = {}
        self.counts = np.zeros(len(C.REGIONS), dtype=int)
        self.max_deflection = 0.0
        self.contact = None
        self.peak_force = 0.0

    def advance(self, center, stroke, height_error):
        depth = max(0.0, -stroke)
        self.max_deflection = max(self.max_deflection, depth)
        self.peak_force = max(self.peak_force, C.SPRING * depth)
        self.contact = [*map(float, center), float(depth), float(height_error)]
        gaps = np.linalg.norm(C.REGIONS - center, axis=1)
        if not C.PRESS_MIN <= depth <= C.PRESS_MAX or abs(height_error) > 0.0011:
            self.counts.fill(0)
            return
        region = int(gaps.argmin())
        if gaps[region] > 0.0023:
            self.counts.fill(0)
            return
        self.counts[np.arange(len(C.REGIONS)) != region] = 0
        self.counts[region] += 1
        if self.counts[region] >= C.HOLD_FRAMES and region not in self.frames:
            rgb, pixels = optical_impression(center, depth)
            self.images[region] = rgb
            self.frames[region] = {
                "center_mm": (np.asarray(center) * 1000).tolist(),
                "depth_mm": float(depth * 1000),
                "force_n": float(depth * C.SPRING),
                "contact_pixels": pixels,
                "diameter_mm": float(2000 * np.sqrt(
                    2 * C.BALL_R * depth - depth**2
                )),
            }

    def score(self):
        value = len(self.frames) / len(C.REGIONS)
        return {
            "score": value, "success": value == 1.0,
            "captured_regions": self.frames.copy(),
            "last_contact": self.contact,
            "peak_force_n": self.peak_force,
            "peak_deflection_mm": self.max_deflection * 1000,
        }

    def snapshot(self):
        return self.score()
