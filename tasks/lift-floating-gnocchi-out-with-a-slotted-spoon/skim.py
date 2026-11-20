"""The slotted spoon in the right gripper: where the hand has to be for the spoon to be somewhere, the moves that
take the spoon off its rest, bring its bowl under a floating piece and lift, carry it to the plate and tip it out,
and a check of such moves against the arm's joint limits and the scene. Nothing here needs the simulator.

The spoon's frame is the task's: the middle of the bowl on the plane of its rim; x from the nose to the handle's
end, z up. A pose of the spoon is (origin, heading, nose down, roll): the direction its handle points in, seen
from above, how far its nose is tilted down and how far it is rolled about its length. The hand sits on the
handle ``GRIP`` along it from its start, the fingers closing across it, leaning ``LEAN`` from straight above
towards the bowl, as a person holds a spoon.

Why the hand leans. With the hand straight above the handle, tilting the nose down leans the hand back towards
the arm, and over the pan that runs the wrist into its limits: the whole motion of a scoop (down behind the piece,
under it, up, level; every joint 5 degrees inside its limits) was possible at 2 of 9 places. The more the hand
leans along the handle the better the scoop goes and the worse the tip over the plate, which needs the hand
leaning back: at 30 degrees a scoop is possible at 17 of 25 places on a 3.5 cm grid (15 with 10 degrees or more
to spare) and the tip, nose down 60 degrees with the handle swung to 285 degrees, keeps 12.5 degrees; at 45
degrees 19 of 25 and 6 degrees. The places without a way are the quarter of the pan nearest the right arm: there
the handle would have to leave across the pan. (IK only, 2026-11-18; builder's notes plan7 to plan9.)
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

GRIP = 0.08  # m from the handle's start to the grasp point
LEAN = math.radians(30)  # the hand's approach axis, from straight above the handle towards the bowl
SAMPLE = 0.004  # m between the points the spoon is checked at
FINGER_ON_HANDLE = -0.010  # the finger joints with the pads 20 mm apart, for the collision check
CLEAR = 0.005  # m the arm and hand keep from everything; the spoon from everything but the water
OVER_RIM = 0.008  # m the bowl's lowest point is carried over the pan's rim
UNDER = 0.006  # m the bowl's rim plane passes under a floating piece's underside (with 2 mm the nose met the piece's side: run l7i)
NOSE_BEHIND = 0.018  # m from a piece's middle back to where the bowl's nose enters the water
SIDE = "right"


def rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def spoon_rotation(heading: float, nose_down: float = 0.0, roll: float = 0.0) -> np.ndarray:
    """The spoon's axes in the world (columns x, y, z) for a handle heading, a nose-down tilt and a roll (rad)."""
    return rz(heading) @ ry(-nose_down) @ rx(roll)


@dataclass
class Pose:
    """A pose of the spoon: its origin in the world and its rotation."""

    origin: np.ndarray
    rot: np.ndarray

    @classmethod
    def at(cls, origin, heading: float, nose_down: float = 0.0, roll: float = 0.0) -> "Pose":
        return cls(np.asarray(origin, dtype=float), spoon_rotation(heading, nose_down, roll))

    def point(self, local) -> np.ndarray:
        return self.origin + self.rot @ np.asarray(local, dtype=float)


class Layout:
    """The task's sizes as the planner needs them (heights are over the bench's top)."""

    def __init__(self, m) -> None:
        self.m = m
        self.top = float(m.TOP)
        self.pan = np.array(m.PAN_XY, dtype=float)
        self.pan_r, self.pan_wall = float(m.PAN_R), float(m.PAN_WALL)
        self.floor = float(m.HOB_H + m.PAN_BASE)  # the pan's floor
        self.surface = float(m.HOB_H + m.SURFACE)  # the water's surface
        self.rim = float(m.HOB_H + m.PAN_BASE + m.PAN_DEPTH)  # the pan's rim
        self.under_piece = self.surface + 0.001 - float(m.PIECE[2]) - UNDER  # the rim plane's height under a floating piece
        self.plate = np.array(m.PLATE_XY, dtype=float)
        self.plate_top = float(m.PLATE_T + m.PLATE_RIM)
        self.grasp_local = np.array([m.HANDLE_FROM + GRIP, 0.0, m.HANDLE_Z])
        solid = [s for s in m.mk.MECH.bodies["spoon"].shapes if s.solid]
        edges = [self._edges(s, m) for s in solid]
        self.bowl = np.vstack(edges[:-2])  # floor and walls: points along the edges of their boxes, in the spoon's frame
        self.stem = np.vstack(edges[-2:])  # neck and handle
        self.rest_pose = Pose(np.array(m.SPOON_AT, dtype=float), m.mk.quat_to_matrix(m.SPOON_QUAT))

    @staticmethod
    def _edges(shape, m) -> np.ndarray:
        """Points no more than SAMPLE apart along the twelve edges of a box, in the body's frame."""
        half = np.asarray(shape.half_extents(), dtype=float)
        points = []
        for axis in range(3):
            along = np.linspace(-half[axis], half[axis], max(2, int(math.ceil(2 * half[axis] / SAMPLE)) + 1))
            for a in (-1, 1):
                for b in (-1, 1):
                    edge = np.zeros((len(along), 3))
                    edge[:, axis] = along
                    edge[:, (axis + 1) % 3], edge[:, (axis + 2) % 3] = a * half[(axis + 1) % 3], b * half[(axis + 2) % 3]
                    points.append(edge)
        return np.vstack(points) @ m.mk.quat_to_matrix(shape.quat).T + np.asarray(shape.pos, dtype=float)

    # --- the hand on the handle ---
    def hand(self, pose: Pose) -> tuple[np.ndarray, np.ndarray]:
        """(grasp point, gripper rotation) that hold the spoon in ``pose``: the fingers close along the spoon's y;
        the approach axis leans LEAN from straight down on the handle towards the bowl."""
        xs, ys, zs = pose.rot[:, 0], pose.rot[:, 1], pose.rot[:, 2]
        approach = -math.cos(LEAN) * zs - math.sin(LEAN) * xs
        return pose.point(self.grasp_local), np.column_stack([ys, np.cross(approach, ys), approach])

    def spoon_of(self, grasp, hand_rot) -> Pose:
        """The spoon's pose for a hand pose: the inverse of :meth:`hand`."""
        ys, width, approach = hand_rot[:, 0], hand_rot[:, 1], hand_rot[:, 2]
        rot = np.column_stack([math.cos(LEAN) * width - math.sin(LEAN) * approach, ys, -math.sin(LEAN) * width - math.cos(LEAN) * approach])
        return Pose(np.asarray(grasp, dtype=float) - rot @ self.grasp_local, rot)

    def move(self, seconds: float, pose: Pose, grip: float | None = None, label: str = ""):
        from demo_kit import Move

        grasp, rot = self.hand(pose)
        return Move(seconds, pos=grasp, rot=rot, grip=grip, label=label)

    # --- heights ---
    def lowest(self, rot: np.ndarray) -> float:
        """How far the bowl's lowest corner is under the spoon's origin when the spoon is turned as ``rot``."""
        return float(-(self.bowl @ rot[2]).min())

    def carry_height(self, rot: np.ndarray) -> float:
        """The origin's height (world z) at which the bowl clears the pan's rim by OVER_RIM."""
        return self.top + self.rim + OVER_RIM + self.lowest(rot)

    # --- what the spoon and the arm may not touch ---
    def ground(self, points: np.ndarray) -> np.ndarray:
        """The height (over the bench) of what lies under each point outside the pan: bench, hob, plate, rest."""
        m = self.m
        x, y = points[:, 0], points[:, 1]
        under = np.zeros(len(points))
        on_hob = (np.abs(x - m.HOB_XY[0]) <= m.HOB_X / 2) & (np.abs(y - m.HOB_XY[1]) <= m.HOB_Y / 2)
        under = np.where(on_hob, m.HOB_H, under)
        on_plate = np.hypot(x - self.plate[0], y - self.plate[1]) <= m.PLATE_R
        under = np.where(on_plate, np.maximum(under, self.plate_top), under)
        on_rest = (np.abs(x - m.REST_XY[0]) <= m.REST_X / 2) & (np.abs(y - m.REST_XY[1]) <= m.REST_Y / 2)
        return np.where(on_rest, np.maximum(under, m.REST_T), under)

    def gap(self, points: np.ndarray, pan_solid: bool) -> float:
        """The smallest distance (m) from ``points`` (world) to the scene. Outside the pan: down to bench, hob,
        plate or rest, and sideways to the pan's wall for points below its rim. Over the pan's wall and its
        handle: down to the rim. Inside the wall: down to the rim if ``pan_solid`` (arm and hand stay out of the
        pan), else down to the pan's floor and sideways to the wall. Negative: through something. The points of
        a thing that reaches into the pan have to be close enough together to show where it crosses the wall."""
        z = points[:, 2] - self.top
        x, y = points[:, 0] - self.pan[0], points[:, 1] - self.pan[1]
        r = np.hypot(x, y)
        outer = self.pan_r + self.pan_wall
        band = 0.002  # the wall counts this much thicker on both sides: wider than the points are apart
        gap = z - self.ground(points)
        beside = (r > outer + band) & (z < self.rim)
        gap = np.where(beside, np.minimum(gap, r - outer), gap)
        on_handle = (x > 0) & (x <= outer + 0.18) & (np.abs(y) <= 0.0125) & (r > outer)
        gap = np.where(on_handle, np.minimum(gap, z - self.rim), gap)
        on_wall = (r >= self.pan_r - band) & (r <= outer + band)
        gap = np.where(on_wall, z - self.rim, gap)
        inside = r < self.pan_r - band
        if pan_solid:
            gap = np.where(inside, z - self.rim, gap)
        else:
            within = np.where(z < self.rim, np.minimum(z - self.floor, self.pan_r - r), z - self.floor)
            gap = np.where(inside, within, gap)
        return float(gap.min())

    def spoon_gap(self, pose: Pose) -> float:
        points = np.vstack([self.bowl, self.stem]) @ pose.rot.T + pose.origin
        return self.gap(points, pan_solid=False)

    def arm_gap(self, q: np.ndarray, base, finger: float = FINGER_ON_HANDLE, in_pan: bool = False) -> float:
        """The smallest distance of the arm and hand (the corners of their collision hulls) to the scene.
        ``in_pan``: the hand may reach into the pan. Only corners are looked at, so a link that crosses the pan's
        wall between two of its corners is not seen: the simulator run has to show that the pan stayed put."""
        from yam_arena import analysis
        from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

        joints = {**{n: float(v) for n, v in zip(ARM_JOINTS, q)}, **dict.fromkeys(FINGER_JOINTS, finger)}
        poses = analysis.fk_link_poses_world(joints, base)
        corners = np.vstack([np.asarray(hull.vertices) for link, hull in analysis.world_hulls(poses) if link not in ("base", "base_link")])
        return self.gap(corners, pan_solid=not in_pan)

    # --- the moves ---
    def take_spoon(self, short=(0.0, 0.0, 0.0)) -> list:
        """Off its rest: behind and above the handle, down along the hand's own axis, pinch, straight up.
        ``short``: what the simulated arm falls short of its target by at that reach (world, m): the hand is sent
        that much beyond the planned grasp point, so that the fingers close on the handle where the plan has
        them."""
        from demo_kit import Move

        grasp, rot = self.hand(self.rest_pose)
        grasp = grasp + np.asarray(short, dtype=float)
        up = np.array([0.0, 0.0, 1.0])
        return [
            Move(3.0, pos=grasp - 0.07 * rot[:, 2], rot=rot, grip=1.0, label="over the spoon"),
            Move(1.2, pos=grasp, label="down to the handle"),
            Move(0.8, grip=0.0, label="pinch"),
            Move(1.2, pos=grasp + 0.10 * up, label="lift the spoon"),
        ]

    def scoop_poses(self, place, heading: float, nose_down: float, ahead: float, under: float = UNDER) -> dict[str, Pose]:
        """The poses of a scoop under a piece floating at ``place`` (world x, y), the handle pointing ``heading``:
        ``over`` the entry with the bowl clear of the rim, ``in`` the water behind the piece, ``under`` it (the
        piece ``ahead`` m nose-side of the bowl's middle, the rim plane ``under`` below its underside), ``out``
        straight up from there, and ``level`` for the carry."""
        rot = spoon_rotation(heading, nose_down)
        back = np.array([math.cos(heading), math.sin(heading), 0.0])  # along the handle, level
        below = np.array([place[0], place[1], self.top + self.under_piece + UNDER - under]) + rot @ np.array([ahead, 0.0, 0.0])
        entry = below + (m_half(self.m) + NOSE_BEHIND - ahead) * back
        high = self.carry_height(rot)
        flat = spoon_rotation(heading)
        return {
            "over": Pose(np.array([entry[0], entry[1], high]), rot),
            "in": Pose(entry, rot),
            "under": Pose(below, rot),
            "out": Pose(np.array([below[0], below[1], high]), rot),
            "level": Pose(np.array([below[0], below[1], self.carry_height(flat)]), flat),
        }

    def scoop(self, place, heading: float, nose_down: float, ahead: float, under: float = UNDER, seconds=(1.6, 1.0, 1.2, 0.3, 1.2, 0.8)) -> list:
        from demo_kit import Move

        p = self.scoop_poses(place, heading, nose_down, ahead, under)
        return [
            self.move(seconds[0], p["over"], label="over the water"),
            self.move(seconds[1], p["in"], label="into the water"),
            self.move(seconds[2], p["under"], label="under the piece"),
            Move(seconds[3], label="still"),
            self.move(seconds[4], p["out"], label="lift out"),
            self.move(seconds[5], p["level"], label="level"),
        ]

    def serve_poses(self, carry_heading: float, tip_heading: float, nose_down: float, nose_at=(0.0, 0.0), drop: float = 0.03) -> dict[str, Pose]:
        """``tip``: nose down with the handle pointing ``tip_heading``, the nose's tip ``drop`` over the plate at
        ``nose_at`` from the plate's middle; ``carry``: level with the handle pointing ``carry_heading``, the nose
        over the same place. (The arm reaches a level spoon over the plate with the handle pointing 150 to 255
        degrees and a spoon tipped 50 to 60 degrees with it pointing 270 to 285: the handle swings as it tips.)"""
        flat, tipped = spoon_rotation(carry_heading), spoon_rotation(tip_heading, nose_down)
        nose = np.array([self.plate[0] + nose_at[0], self.plate[1] + nose_at[1], self.top + self.plate_top + drop])
        tip = Pose(nose + tipped @ np.array([m_half(self.m), 0.0, 0.0]), tipped)
        over = np.array([nose[0], nose[1], 0.0]) + flat @ np.array([m_half(self.m), 0.0, 0.0])
        return {"carry": Pose(np.array([over[0], over[1], max(self.carry_height(flat), tip.origin[2])]), flat), "tip": tip}

    def serve(self, carry_heading: float, tip_heading: float, nose_down: float, nose_at=(0.0, 0.0), drop: float = 0.03, seconds=(1.8, 1.0, 0.6, 0.8)) -> list:
        from demo_kit import Move

        p = self.serve_poses(carry_heading, tip_heading, nose_down, nose_at, drop)
        return [
            self.move(seconds[0], p["carry"], label="to the plate"),
            self.move(seconds[1], p["tip"], label="tip out"),
            Move(seconds[2], label="let slide"),
            self.move(seconds[3], p["carry"], label="level again"),
        ]

    # --- which floating pieces a scoop takes ---
    def over_bowl(self, pose: Pose, places) -> list[int]:
        """Which of ``places`` (world x, y of floating pieces' middles) a lift from ``pose`` takes: those over the
        part of the bowl whose rim plane is at least 1.5 mm under a floating piece's underside, no nearer than
        6 mm to the nose's tip and 4 mm inside the rim sideways."""
        m, half = self.m, m_half(self.m)
        underside = self.top + self.surface + 0.001 - float(m.PIECE[2])
        taken = []
        for k, (x, y) in enumerate(places):
            local = pose.rot.T @ (np.array([x, y, underside]) - pose.origin)
            if local[2] < 0.0015 or not -(half - 0.006) <= local[0] <= half:
                continue
            if abs(local[1]) <= float(m.BOWL_W) / 2 * math.sqrt(max(0.0, 1.0 - (local[0] / half) ** 2)) - 0.004:
                taken.append(k)
        return taken

    def under_bowl(self, pose: Pose, places, grow: float = 0.010) -> list[int]:
        """Which of ``places`` lie under the bowl's outline (grown by ``grow``) when the spoon is in ``pose``:
        the floating pieces the bowl comes down on when it enters the water there."""
        m, half = self.m, m_half(self.m)
        hit = []
        for k, (x, y) in enumerate(places):
            local = pose.rot.T @ (np.array([x, y, pose.origin[2]]) - pose.origin)
            if abs(local[0]) <= half + grow and abs(local[1]) <= float(m.BOWL_W) / 2 + grow:
                hit.append(k)
        return hit

    # --- the check ---
    def check(self, moves: list, start_q=None, start=None, hz: float = 10.0, holding_from: float = 0.0, base=None, side: str = SIDE,
              hand_in_pan: bool = False, finger: float = FINGER_ON_HANDLE) -> dict:  # fmt: skip
        """Play ``moves`` with one arm (the right unless ``side`` says otherwise) from joints ``start_q`` (the ready
        pose if None) and hand pose ``start`` (grasp point, rotation; the ready pose's if None), without a
        simulator: the worst IK error, the smallest joint margin, the smallest gap of arm and hand to the scene,
        and from ``holding_from`` s on the smallest gap of the spoon to the scene (the water does not count),
        each with the move's label. ``hand_in_pan``, ``finger``: see :meth:`arm_gap`."""
        import demo_kit as kit
        from yam_arena import ik, station

        base = station.BASES[side] if base is None else base
        arm = kit.Arm(side, base, moves)
        if start_q is not None:
            arm.q = np.array(start_q, dtype=float)
            arm.start_pos, arm.start_rot = ik.tcp_pose(arm.q, arm.base) if start is None else (np.asarray(start[0], dtype=float), np.asarray(start[1], dtype=float))
        worst = {"margin_deg": (1e9, ""), "arm_gap_mm": (1e9, ""), "spoon_gap_mm": (1e9, "")}
        steps = int(math.ceil(arm.seconds * hz))
        for i in range(steps + 1):
            t = min(i / hz, arm.seconds)
            pos, rot, _, label, _, _ = arm.state(t)
            action, _ = arm.action(t)
            q = action[:6]
            margin = float(np.degrees(np.minimum(q - ik.LOWER, ik.UPPER - q)).min())
            if margin < worst["margin_deg"][0]:
                worst["margin_deg"] = (round(margin, 1), f"{label} at {t:.1f} s")
            gap = 1000 * self.arm_gap(q, base, finger, hand_in_pan)
            if gap < worst["arm_gap_mm"][0]:
                worst["arm_gap_mm"] = (round(gap, 1), f"{label} at {t:.1f} s")
            if t >= holding_from:
                gap = 1000 * self.spoon_gap(self.spoon_of(pos, rot))
                if gap < worst["spoon_gap_mm"][0]:
                    worst["spoon_gap_mm"] = (round(gap, 1), f"{label} at {t:.1f} s")
        return {**worst, "ik_mm": round(arm.worst_mm, 2), "ik_deg": round(arm.worst_deg, 2), "seconds": round(arm.seconds, 2), "end_q": arm.q.copy(), "end": arm.state(arm.seconds)[:2]}

    def passes(self, checked: dict, margin: float = 5.0, arm_gap: float = 5.0, spoon_gap: float = 1.0) -> bool:
        return (checked["ik_mm"] <= 1.0 and checked["ik_deg"] <= 0.5 and checked["margin_deg"][0] >= margin
                and checked["arm_gap_mm"][0] >= arm_gap and checked["spoon_gap_mm"][0] >= spoon_gap)  # fmt: skip


def m_half(m) -> float:
    """From the bowl's middle to its nose."""
    return float(m.BOWL_L) / 2
