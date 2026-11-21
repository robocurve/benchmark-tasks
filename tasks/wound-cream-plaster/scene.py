"""Where everything is, and its size: the forearm trainer lying across the table (its axis along y), the wound, the
cream station, the plaster stand and the waste pot. Plain numbers and NumPy, no simulator.

Station frame: x forward from the arm bases, y to the robot's left, z up from the table top (m).
"""

from __future__ import annotations

import math

import numpy as np

TABLE = 0.7475  # the station table top (world z)
PAD_Z = 0.007  # the plywood base and the underpad under the forearm
PAD_YC, PAD_SIZE = 0.065, (0.18, 0.47)  # the base's middle (y) and size (x, y)

# -- the forearm trainer: an elliptic section along y, constant over |y| < ARM_Y_FLAT ------------------------------------
XW = 0.300  # the forearm's axis (x)
ARM_A, ARM_B, ARM_ZC = 0.038, 0.032, 0.043  # half-width (x), half-height, centre height above the table
ARM_TOP = ARM_ZC + ARM_B  # 0.075
ARM_Y_FLAT = 0.090  # the section is constant for |y| below this (the dressing region)
SKIN_GIVE = 0.0015  # the collider sits this far inside the visual skin (the vinyl's give)

# -- the wound: a 14 x 7 mm abrasion on the top of the forearm, its long axis along the arm ------------------------------
WOUND_C = (0.0, 0.0)  # (y, s): s is the arc length round the section from the top, positive toward +x
WOUND_HALF = (0.007, 0.0035)  # half-length (along y), half-width (round the arm)
MARGIN = 0.005  # the margin to cover with cream round the wound's edge
BAND = 0.008  # cream allowed up to this far from the wound's edge (the margin and 3 mm)

# -- the plaster: a 1.75 x 4 in fabric strip, pad 38 mm long, two C-folded release liners with end tabs -------------------
PL_LEN, PL_WID = 0.102, 0.0445  # Band-Aid Flexible Fabric Extra Large: 1.75 x 4 in
PAD_HALF = 0.019  # the pad spans |u| < 19 mm, the whole width; the adhesive wings beyond
TAB_OUT = 0.025  # each liner's tab sticks out this far past the strip's end
TAB_GRIP = 0.020  # where a hand holds a tab: this far along it from the strip's end
PL_T = 0.0012  # fabric, adhesive and pad, mean thickness
TAB_THICK = 0.0010  # a liner tab's collider (the paper is about 0.1 mm; thinner would be pinched too lightly)

# -- the plaster stand: the opened wrapper on a riser behind the forearm --------------------------------------------------
STAND_C = (0.440, 0.0)
STAND_SIZE = (0.070, 0.080, 0.035)  # x, y, height: the strip's wings and tabs stand out over its two ends

# -- the cream station (right hand only) and the waste pot -----------------------------------------------------------------
CUP_C = (0.300, -0.205)  # 30 mL graduated medicine cup, standing on its tray
CUP_Z = 0.025  # the cup tray's top (a 25 mm HDPE board, like the applicator tray)
CUP_TRAY = (0.070, 0.070)
CUP_R_TOP, CUP_R_BOT, CUP_H = 0.0195, 0.0160, 0.044
DOLLOP_R, DOLLOP_H = 0.008, 0.004  # 0.5 mL of cream, pre-dispensed, a mound on the cup's floor
TRAY_C = (0.160, -0.170)  # applicator tray (the opened peel pouch on a small HDPE tray)
TRAY_SIZE = (0.130, 0.060, 0.025)  # x, y, height; the stick lies along x, its handle 50 mm out over the far edge
APP_OVER = 0.050
APP_LEN = 0.152  # 6 in wooden applicator
APP_STICK_R = 0.00115
APP_TIP_R, APP_TIP_LEN = 0.0025, 0.014  # cotton bud
APP_GRIP = 0.045  # the demo's grip: this far from the stick's back end
POT_C = (0.150, 0.020)  # waste pot (a 1 L open bin) in front of the forearm, between the hands
POT_R, POT_H = 0.055, 0.110


class Forearm:
    """The forearm's elliptic section in the x-z plane, constant along y. s is the arc length round the section from
    the top, positive toward +x."""

    def __init__(self, a=ARM_A, b=ARM_B, xc=XW, zc=ARM_ZC):
        self.a, self.b, self.xc, self.zc = a, b, xc, zc
        ts = np.linspace(-np.pi, np.pi, 4001)
        seg = np.hypot(np.diff(a * np.sin(ts)), np.diff(b * np.cos(ts)))
        cum = np.r_[0, np.cumsum(seg)]
        self._ts, self._cum = ts, cum - np.interp(0.0, ts, cum)

    def theta_of_s(self, s):
        return np.interp(s, self._cum, self._ts)

    def s_of_theta(self, th):
        return np.interp(th, self._ts, self._cum)

    def point(self, y, s):
        """Surface points and outward normals at (y, s); arrays broadcast."""
        y, s = np.broadcast_arrays(np.asarray(y, float), np.asarray(s, float))
        th = self.theta_of_s(s)
        x, z = self.a * np.sin(th), self.b * np.cos(th)
        n = np.stack([x / self.a**2, np.zeros_like(x), z / self.b**2], -1)
        n /= np.linalg.norm(n, axis=-1, keepdims=True)
        p = np.stack([x + self.xc, y, z + self.zc], -1)
        return p, n

    def signed(self, p):
        """(distance outside the skin, outward normal, (y, s) of the nearest surface point) for points (n, 3);
        approximate off the surface, good within a few cm."""
        p = np.atleast_2d(np.asarray(p, float))
        dx, dz = p[:, 0] - self.xc, p[:, 2] - self.zc
        th = np.arctan2(dx / self.a, dz / self.b)
        # two Newton steps for the nearest point on the ellipse
        for _ in range(3):
            ex, ez = self.a * np.sin(th), self.b * np.cos(th)
            tx, tz = self.a * np.cos(th), -self.b * np.sin(th)
            f = (dx - ex) * tx + (dz - ez) * tz
            df = -(tx * tx + tz * tz) + (dx - ex) * (-self.a * np.sin(th)) + (dz - ez) * (-self.b * np.cos(th))
            th = th - f / np.where(np.abs(df) < 1e-12, -1e-12, df)
        ex, ez = self.a * np.sin(th), self.b * np.cos(th)
        n = np.stack([ex / self.a**2, np.zeros_like(ex), ez / self.b**2], 1)
        n /= np.linalg.norm(n, axis=1, keepdims=True)
        d = (dx - ex) * n[:, 0] + (dz - ez) * n[:, 2]
        return d, n, np.stack([p[:, 1], self.s_of_theta(th)], 1)

    def tangent_s(self, s):
        """Unit tangent (direction of increasing s) at arc s."""
        th = self.theta_of_s(s)
        t = np.stack([self.a * np.cos(th), np.zeros_like(th), -self.b * np.sin(th)], -1)
        return t / np.linalg.norm(t, axis=-1, keepdims=True)


ARM = Forearm()


def wound_dist(y, s):
    """Approximate distance (m) from (y, s) to the wound's edge (negative inside): an ellipse's scaled radius."""
    (yc, sc), (hy, hs) = WOUND_C, WOUND_HALF
    dy, ds = np.asarray(y) - yc, np.asarray(s) - sc
    r = np.sqrt((dy / hy) ** 2 + (ds / hs) ** 2) + 1e-12
    # distance along the ray from the centre: (r - 1) times the ellipse radius in that direction
    rad = np.hypot(dy, ds) / r
    return (r - 1.0) * rad
