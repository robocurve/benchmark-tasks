"""Cartesian reference for native grasping, scraping and controlled lowering."""

import numpy as np

from . import config as C
from .motion import Program, rotation


def pinch(lateral, approach):
    z = np.asarray(approach, dtype=float)
    z /= np.linalg.norm(z)
    x = np.asarray(lateral) - np.dot(lateral, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


WIPER_PINCH = C.GLASS_ROT @ pinch([0., -1., 0.], [.36, -.16, -1.])
TOOL_PINCH = pinch([0., -1., 0.], [.85, .08, -1.])


def arm_at(angle, height=0., grip=.03):
    lift = rotation("Y", -np.deg2rad(angle))
    grip_point = C.HINGE + lift @ C.GLASS_ROT @ C.ARM_GRIP
    return grip_point + [0., 0., height], lift @ WIPER_PINCH, grip


def tool_at(x, y, h, grip=.035):
    tilt = C.GLASS_ROT @ rotation("Y", np.deg2rad(26.))
    target_edge = C.wind(x, y, h)
    tcp = target_edge + tilt @ (C.SCRAPER_GRASP - [C.EDGE_X, 0., 0.])
    return tcp, tilt @ TOOL_PINCH, grip


def pickup(h, grip):
    return C.SCRAPER_START + C.SCRAPER_GRASP + [0., 0., h], TOOL_PINCH, grip


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    p.add(2.4, "Approach spring arm and scraper",
          left=arm_at(0., .080, .59), right=pickup(.105, .59))
    p.add(1.0, "Align grippers with the steel arm and scraper handle",
          left=arm_at(0., 0., .59), right=pickup(0., .59))
    p.add(.6, "Grip the spring arm and scraper",
          left=arm_at(0.), right=pickup(0., .035))
    p.add(1.65, "Lift the spring arm and clear the scraper from the bench",
          left=arm_at(C.LIFT_DEG), right=pickup(.130, .035))
    clearance = .020 if mode == "air-pass" else -.005
    y = -.091
    p.add(2.5, "Bring the scraper ahead of the frosted glass", right=tool_at(.158, y, .034))
    p.add(1.45, "Set the plastic edge on the pane", right=tool_at(.169, y, clearance))
    p.add(2.85, "Scrape one continuous swath beneath the raised wiper",
          right=tool_at(.284, y, clearance))
    p.add(1.45, "Pull the scraper back below the raised wiper",
          right=tool_at(.147, y, max(.003, clearance)))
    p.add(1., "Lift the scraper off the glass", right=tool_at(.147, y, .052))
    p.add(2.75, "Move the scraper out of the wiper's path", right=pickup(.135, .035))
    if mode == "snap-return":
        p.add(1.3, "Release the raised wiper",
              left=arm_at(C.LIFT_DEG, grip=.62), right=pickup(.02, .035))
        p.add(1.35, "Set the scraper down with the wiper already released",
              left=arm_at(C.LIFT_DEG, .070, .62), right=pickup(.001, .035))
        p.add(.55, "Release the scraper", right=pickup(.001, .62))
        p.add(1.05, "Withdraw the open grippers", right=pickup(.11, .62))
    else:
        p.add(1.3, "Lower the wiper against spring force", left=arm_at(6.),
              right=pickup(.02, .035))
        p.add(1.35, "Set the blade gently on the glass and the scraper on the bench",
              left=arm_at(-.5), right=pickup(.001, .035))
        p.add(.55, "Release both supported parts", left=arm_at(-.5, grip=.62),
              right=pickup(.001, .62))
        p.add(1.05, "Withdraw the open grippers", left=arm_at(-.5, .100, .62),
              right=pickup(.11, .62))
    if mode == "no-lift":
        p.moves["left"] = []
    return p.moves
