"""Scene layout of the SO-101 joint-1 task (station frame: x forward from the arm bases, y to the robot's left, z up;
metres). Engine-independent (numpy only).

The partly built arm (base + base motor holder + STS3215 servo with both horns) is clamped to the table. Its servo
frame S (scoring.py) has its origin on the joint-1 axis at the top horn's outer face. The arm's front (S +x) faces the
robot, so the shoulder bracket is slid on away from the robot (station +x) and its web ends up on the robot's side.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

TOP = 0.7475                      # station table top (yam_arena TABLE_TOP["station"])
ASSET = Path(__file__).resolve().parent.parent / 'assets' / 'so101_joint1.npz'
BASE_BOTTOM_S = -0.0648           # base underside in the servo frame (official CAD)


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


# Servo frame in the station: axis 0.40 m forward, 0.10 m right of centre; S x = station -x (front faces the robot).
SERVO_YAW_DEG = 180.0
R_S = rz(SERVO_YAW_DEG)
O_S = np.array([0.40, -0.10, TOP - BASE_BOTTOM_S])

# Horns. The top horn's hole pattern starts 25 deg from the CAD pattern (as left by the previous build step); the
# bottom horn is free on its boss.
HORN_TOP_START_DEG = 25.0
HORN_BOTTOM_START_DEG = 0.0
HORN_R = 0.0100
HORN_CHAMFER = 0.0004                   # rim chamfer on each horn's outer face (estimate)
HORN_TOP_Z = (-0.0024, -0.0002)        # disc, S frame (outer face 0.2 mm below the cheek's inner face: clearance)
SCREW_HEAD = (0.0025, -0.0002, 0.0011)  # radius, z0, z1: M3 pan head 5.0 mm; 1.3 mm proud of the horn, so that with the
# bracket resting on the top horn (the sim's 0.2 mm face clearance taken up) it keeps 0.2 mm under the 1.5 mm groove's roof
# (the CAD stack, head 1.5 mm proud in a 1.5 mm groove, is line-to-line)
HORN_BOTTOM_Z = (-0.0362, -0.0345)      # disc (outer face 0.2 mm above the bottom cheek's inner face at -36.4 mm)
BOSS = (0.00285, -0.0372, -0.0362)      # servo-case idler boss below the bottom horn. CAD: r 3.0 mm, tip at -37.0 mm, i.e. 0.6 mm
# into the bottom cheek's 1 mm deep slot. Sim: r 2.85 (0.25 mm a side in its 6.2 mm slot); the tip 0.2 mm lower (-37.2 mm)
# because the sim's 0.2 mm face clearance at the top horn lets the bracket hang 0.2 mm lower than in the CAD: the boss
# then still engages the slot walls over the CAD's 0.6 mm (and just reaches the slot floor at the top of the play)
HORN_MASS = 0.003                       # kg (estimate)
HORN_TOP_FRICTION_NM = 0.60             # unpowered 1:345 gear train backdrive torque (estimate; measure); the
                                        # sim's top horn is fixed, this is used only by SO101_HORN_JOINTS=articulation
HORN_BOTTOM_FRICTION_NM = 0.002         # idler horn on its boss (free)

# Shoulder bracket (official rotation_pitch_so101_v1, yellow PLA): standing on its bottom cheek on the table, to the
# robot's right of the base; frame B origin on its joint axis at the top cheek's inner face.
BRACKET_MASS = 0.035                    # kg (estimate; weigh the printed part)
BRACKET_FRICTION = 0.35                 # PLA on PLA / aluminium
BRACKET_BOTTOM_B = -0.0462
BRACKET_START_YAW_DEG = 120.0           # station yaw of the bracket's x axis at the start
BRACKET_START = np.array([0.30, -0.25, TOP - BRACKET_BOTTOM_B + 0.0003])


def bracket_start_rot():
    return rz(BRACKET_START_YAW_DEG)


def bracket_home(yaw_deg=0.0):
    """Bracket pose (station) when fully home at yaw ``yaw_deg`` relative to the servo frame."""
    return O_S.copy(), R_S @ rz(yaw_deg)


def horn_rot(start_deg):
    return R_S @ rz(start_deg)


def s_to_world(p_s):
    return O_S + R_S @ np.asarray(p_s, float)


def load_asset(path=ASSET):
    z = np.load(path)
    meta = json.loads(str(z['meta']))
    pieces = [np.asarray(z[f'piece_{i}'], float) for i in range(meta['pieces'])]
    vis = {k[4:-2]: (np.asarray(z[k], float), np.asarray(z[k[:-2] + '_f'], np.int64), np.asarray(z[k[:-2] + '_n'], float))
           for k in z.files if k.startswith('vis_') and k.endswith('_v')}
    static = (np.asarray(z['static_v'], float), np.asarray(z['static_f'], np.int64))
    return dict(meta=meta, pieces=pieces, vis=vis, static=static)


def piece_boxes(pieces):
    """Axis-aligned boxes (bracket frame) of the bracket's convex pieces, merged per slab into one box per piece."""
    return [(p.min(axis=0), p.max(axis=0)) for p in pieces]
