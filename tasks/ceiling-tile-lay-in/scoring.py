"""Stateful scorer for "put the ceiling tile back into its opening", from measured root poses and the robot's
measured TCPs only. The same code runs the success check and offline rescoring.

Geometry (geom.py): the tiles' bearing plane is the flange face, z = FLANGE_Z; the target bay is centred at
(XC, YC) between tee centre lines 609.6 mm apart; its clear opening (between flange edges) is 585.6 mm square. The
tile starts lying askew (12 deg) across the tops of the four tees round the opening.

Ladder (milestones latch):
  0.10  turned to within 5 deg of square (while over the opening: centre within 60 mm of the bay centre).
  0.20  square and centred: within 1 deg of square and its centre within 5 mm of the bay centre.
  0.60  seated: all four bottom corners within 3 mm of the flange face and the tile bearing at least 4 mm on every
        one of the four flanges, at rest (under 1 cm/s) for 1 s.
        (Instead, a tile at rest in the opening with a corner on a flange but cocked -- a corner more than 3 mm high
        or a bearing under 4 mm -- scores 0.35.)
  +0.04 per neighbouring tile undisturbed (5 neighbours, 0.20 in all), given once seated: never lifted more than
        3 mm at any corner and never shifted more than 5 mm.
  +0.20 clean finish, given once seated: both hands withdrawn (every TCP 50 mm or more below the flange face or
        50 mm clear of the tile) for 2 s with the tile still seated.
Dropped -- the tile lands on the bench, or is more than 150 mm below the flange face with no TCP within 100 mm of it
for 0.5 s -- scores 0 and latches (a tile carried down through the opening and brought back up, e.g. to be put in
diagonally, is not dropped). Doing nothing scores 0 (the tile starts 12 deg askew on top of the grid).
Real trial: the 0.1 / 0.2 milestones are read from the video (overview, from below) against a tape line on the
near tee and a pencil mark at the tile's centre: tile edge parallel to the tee within 5 deg / 1 deg, centre mark
within 5 mm of a mark at the bay centre on a cross-string; a straightedge or steel rule held across the opening from
below shows the tile face flush with the grid face (within 3 mm) on all four sides; tape marks on the neighbouring
tiles' edges and the video show any lift or shift; the video shows a drop and the hands' withdrawal.
"""
from __future__ import annotations

import math

import numpy as np

import geom as G

HZ = 20
SEAT_TOL = 0.003
SQUARE_NEAR = math.radians(5.0)
SQUARE = math.radians(1.0)
CENTRED = 0.005
BEAR_MIN = 0.004
REST_V = 0.01
SEAT_FRAMES = 20
CLEAR_FRAMES = 40
NB_LIFT = 0.003
NB_SHIFT = 0.005
HAND_CLEAR = 0.050
DROP_Z = 0.150
HELD = 0.100
DROP_FRAMES = 10


def quat_matrix(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def corners(pose, dz=-0.5):
    """World corners of a tile face (dz = -0.5: bottom face, +0.5: top face)."""
    p = np.asarray(pose, float)
    R = quat_matrix(p[3:7])
    h = G.TILE / 2
    return np.array([p[:3] + R @ np.array([sx * h, sy * h, dz * G.TILE_T]) for sx in (-1, 1) for sy in (-1, 1)])


def bearings(bottom):
    """Bearing (m) of the tile on each of the four flanges of the target bay: near, far, right, left."""
    x0, x1 = G.X_LINES[0] + G.FLANGE_W / 2, G.X_LINES[1] - G.FLANGE_W / 2     # flange inner edges
    y0, y1 = G.Y_LINES[1] + G.FLANGE_W / 2, G.Y_LINES[2] - G.FLANGE_W / 2
    xs, ys = bottom[:, 0], bottom[:, 1]
    # per side, the overlap of the side's two corners past the flange edge; the smaller one counts
    sx, sy = sorted(xs), sorted(ys)
    return {"near": x0 - sx[1], "far": sx[2] - x1, "right": y0 - sy[1], "left": sy[2] - y1}


def tile_yaw(R):
    """The tile's turn from square (rad), folded into -45..45 deg (a square tile is the same every 90 deg)."""
    a = math.atan2(R[1, 0], R[0, 0])
    return (a + math.pi / 4) % (math.pi / 2) - math.pi / 4


def _field(o, name):
    return o[name] if isinstance(o, dict) else getattr(o, name)


class TrialScorer:
    def __init__(self):
        self.step = 0
        self.near_square = False
        self.squared = False
        self.seated = False
        self.cocked = False
        self.seat_run = 0
        self.cock_run = 0
        self.clear_run = 0
        self.finished = False
        self.dropped = False
        self.drop_run = 0
        self.start = None
        self.nb_start = {}
        self.nb_bad = set()

    def update(self, step, objects, root_poses, robot, mechanics=None, **_):
        if step != self.step + 1:
            raise ValueError("scorer needs one update per control step")
        self.step = step
        pose = np.asarray(root_poses["tile"], float)
        if self.start is None:
            self.start = pose.copy()
            for n in G.OBJECT_NAMES[1:]:
                self.nb_start[n] = (np.asarray(root_poses[n], float), corners(root_poses[n]))
        bot = corners(pose)
        zrel = bot[:, 2] - G.FLANGE_Z
        v = float(np.linalg.norm(np.asarray(_field(objects["tile"], "lin_vel"), float)))
        still = v < REST_V
        R = quat_matrix(pose[3:7])
        yaw = tile_yaw(R)
        dxy = math.hypot(pose[0] - G.XC, pose[1] - G.YC)
        if abs(yaw) <= SQUARE_NEAR and dxy <= 0.060:
            self.near_square = True
        if abs(yaw) <= SQUARE and dxy <= CENTRED:
            self.squared = True
        b = bearings(bot)
        bear_min = min(b.values())
        flush = bool(np.all(np.abs(zrel) <= SEAT_TOL))
        seated_now = flush and bear_min >= BEAR_MIN
        self.seat_run = self.seat_run + 1 if (seated_now and still) else 0
        if self.seat_run >= SEAT_FRAMES:
            self.seated = True
        cocked_now = (not seated_now) and still and dxy < 0.15 and zrel.min() > -0.010 and np.any(np.abs(zrel) <= SEAT_TOL) \
            and zrel.max() < 0.10
        self.cock_run = self.cock_run + 1 if cocked_now else 0
        if self.cock_run >= SEAT_FRAMES:
            self.cocked = True
        # neighbours
        for n, (p0, c0) in self.nb_start.items():
            p = np.asarray(root_poses[n], float)
            c = corners(p)
            if np.max(np.abs(c[:, 2] - c0[:, 2])) > NB_LIFT or math.hypot(p[0] - p0[0], p[1] - p0[1]) > NB_SHIFT:
                self.nb_bad.add(n)
        nb_ok = len(self.nb_start) - len(self.nb_bad)
        # hands
        tcps = [np.asarray(robot[s]["tcp"], float)[:3] for s in ("left", "right")]
        h = G.TILE / 2

        def box_dist(t):
            local = R.T @ (t - pose[:3])
            d = np.maximum(np.abs(local) - np.array([h, h, G.TILE_T / 2]), 0.0)
            return float(np.linalg.norm(d))
        clear = all(t[2] <= G.FLANGE_Z - HAND_CLEAR or box_dist(t) >= HAND_CLEAR for t in tcps)
        # dropped: the tile has come down onto the bench, or is more than DROP_Z below the flange face with no hand
        # within HELD of it for 0.5 s (a tile carried down through the opening and back up is not dropped)
        unheld = min(box_dist(t) for t in tcps) > HELD
        low = pose[2] < G.FLANGE_Z - DROP_Z
        self.drop_run = self.drop_run + 1 if (low and unheld) else 0
        if self.drop_run >= DROP_FRAMES or pose[2] < G.TOP + 0.03:
            self.dropped = True
        self.clear_run = self.clear_run + 1 if (self.seated and seated_now and clear) else 0
        if self.clear_run >= CLEAR_FRAMES:
            self.finished = True
        score = 0.0
        if self.near_square:
            score = 0.1
        if self.squared:
            score = 0.2
        if self.seated:
            score = 0.6 + 0.04 * nb_ok + (0.2 if self.finished else 0.0)
        elif self.cocked:
            score = 0.35
        if self.dropped:
            score = 0.0
        success = bool(self.seated and self.finished and nb_ok == len(self.nb_start) and not self.dropped)
        if success:
            score = 1.0
        return {"score": round(float(score), 3), "success": success, "yaw_deg": round(math.degrees(yaw), 2), "centre_off_mm": round(1000 * dxy, 1),
                "near_square": self.near_square, "squared": self.squared, "seated": self.seated,
                "seated_now": bool(seated_now), "cocked": self.cocked, "corner_z_mm": [round(1000 * z, 1) for z in zrel],
                "bearing_mm": {k: round(1000 * x, 1) for k, x in b.items()}, "tile_speed": round(v, 4),
                "neighbours_ok": nb_ok, "disturbed": sorted(self.nb_bad), "clear": bool(clear),
                "clear_frames": self.clear_run, "dropped": bool(self.dropped)}
