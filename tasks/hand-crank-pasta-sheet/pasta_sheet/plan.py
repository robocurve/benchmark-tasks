"""Teach the stock pickup and crank path with normal YAM actions."""

import math

import numpy as np

from . import config as C
from .motion import Program, rotation


def aim(forward, jaw):
    forward = np.asarray(forward, dtype=float)
    forward /= np.linalg.norm(forward)
    jaw = np.asarray(jaw, dtype=float)
    jaw -= np.dot(jaw, forward) * forward
    jaw /= np.linalg.norm(jaw)
    return np.column_stack((jaw, np.cross(forward, jaw), forward))


PICK = aim((0.35, -1.0, 0.0), (0.0, 0.0, -1.0))
FEED = rotation("Y", math.pi / 2) @ PICK
CRANK = aim((0.24, 1.0, -0.06), (0.0, 0.0, -1.0))


def crank_pose(angle, opening, retreat=0.0):
    a = C.CRANK_START - 0.15 + angle
    offset = [C.CRANK_RADIUS * math.cos(a), C.CRANK_SPAN, -C.CRANK_RADIUS * math.sin(a)]
    return C.ROLL_NEAR + offset - retreat * CRANK[:, 2], CRANK, opening


def waypoints(mode, guide=None):
    p = Program()
    if mode == "hold":
        return p.moves
    pick = C.STOCK + [-0.021, 0.030, 0.0]
    feed_rot, palm_to_stock = (
        guide if guide is not None else (FEED, np.array([0.004, -0.027, -0.005]))
    )

    def feed(height, grip, side=0.0):
        return C.NIP + [0.0, side, height] - palm_to_stock, feed_rot, grip

    have_feed = mode != "crank-only"
    have_crank = mode != "feed-only"
    p.add(
        2.4,
        "approach the stock and handle",
        left=(pick + [-0.008, 0.063, 0.071], PICK, 0.50) if have_feed else None,
        right=crank_pose(0.0, 0.72, 0.078) if have_crank else None,
    )
    p.add(
        1.1,
        "get level with the overhang",
        left=(pick + [-0.015, 0.041, 0.0], PICK, 0.30) if have_feed else None,
        right=crank_pose(0.0, 0.66) if have_crank else None,
    )
    p.add(
        0.9,
        "surround the dough edge",
        left=(pick, PICK, 0.26) if have_feed else None,
        right=crank_pose(0.0, 0.20) if have_crank else None,
    )
    p.add(0.6, "pinch the loose stock", left=(pick, PICK, 0.0) if have_feed else None)
    p.add(
        1.2,
        "lift the loose sheet",
        left=(pick + [0.0, 0.0, 0.093], PICK, 0.0) if have_feed else None,
    )
    p.add(
        1.05,
        "turn the loose sheet in front of the machine",
        left=(np.array([0.323, 0.092, 1.054]) - palm_to_stock, feed_rot, 0.0)
        if have_feed
        else None,
    )
    p.add(
        1.05,
        "bring the aligned sheet over the nip",
        left=feed(0.143, 0.0, 0.018) if have_feed else None,
    )
    p.add(
        1.2,
        "square the leading edge above the roll nip",
        left=feed(0.103, 0.0) if have_feed else None,
    )
    # The bearing-mounted knob turns freely inside the held grip.
    for angle in np.linspace(0.0, 8.7, 109)[1:]:
        feed_travel = 0.042 * min(angle / 2.7, 1.0)
        if angle < 3.5:
            left = feed(0.103 - feed_travel, 0.035 if angle > 1.6 else 0.0)
        elif angle < 4.1:
            away = (angle - 3.5) / 0.6
            left = feed(0.061 + 0.074 * away, 0.48 * away + 0.10, 0.055 * away)
        else:
            left = feed(0.135, 0.60, 0.055)
        p.add(
            0.13,
            "crank while guiding the dough"
            if angle < 2.7
            else "draw the outfeed and clear the guiding hand",
            left=left if have_feed else None,
            right=crank_pose(angle, 0.17) if have_crank else None,
        )
    p.add(
        14.0,
        "draw the outfeed through the rollers",
        right=crank_pose(8.7, 0.17) if have_crank else None,
    )
    p.add(
        1.15,
        "release both hands and let the sheet settle",
        right=crank_pose(8.7, 0.67, 0.065) if have_crank else None,
    )
    return p.moves
