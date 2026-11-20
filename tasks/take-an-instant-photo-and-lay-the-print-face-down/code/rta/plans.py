"""Waypoint builders for oracle plans (per arm, played by ``rta_oracle``). NumPy only."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from yam_arena import ik, station
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY
from yam_arena.demo import ArmPlan, Waypoint, grip_rotation


# -- grips in any orientation ------------------------------------------------------------------------------------

@dataclass
class Pose(Waypoint):
    """A waypoint with the gripper in any orientation: ``rot`` is the TCP rotation (columns: the closing axis, the
    fingers' width axis, the approach axis; see :func:`grip`). Without ``rot`` it is a straight-down grip turned
    ``yaw``, as a plain :class:`Waypoint`."""

    rot: np.ndarray | None = None


def grip(approach, closing) -> np.ndarray:
    """The TCP rotation pointing the fingers along ``approach`` and closing them along ``closing`` (made
    perpendicular to the approach). ``grip((0, 0, -1), (0, -1, 0))`` is the straight-down grip at yaw 0, and
    ``grip((0, 0, -1), (-1, 0, 0))`` the one at yaw -pi/2."""
    z = np.asarray(approach, float)
    z = z / np.linalg.norm(z)
    x = np.asarray(closing, float)
    x = x - (x @ z) * z
    x = x / np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def rotation_of(w: Waypoint) -> np.ndarray:
    rot = getattr(w, "rot", None)
    return np.asarray(rot, float) if rot is not None else grip_rotation(w.yaw)


class PosePlan(ArmPlan):
    """``yam_arena.demo.ArmPlan`` that also plays :class:`Pose` waypoints (position and rotation interpolated, joint
    targets by IK). Plain waypoints play exactly as in ArmPlan."""

    def target(self, t: float):
        prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
        for w in self.waypoints:
            goal, rot = np.asarray(w.pos, dtype=float), rotation_of(w)
            if t < w.duration:
                s = ik.smoothstep(t / w.duration)
                return (prev_pos + s * (goal - prev_pos), ik.slerp_matrix(prev_rot, rot, s),
                        prev_finger + s * (w.finger - prev_finger), w.label)
            t -= w.duration
            prev_pos, prev_rot, prev_finger = goal, rot, w.finger
        return prev_pos, prev_rot, prev_finger, "done"


# The YAM hand's collision meshes (gripper body and both fingers, closed), as boxes in the TCP frame: (centre, half
# sizes) along (closing, width, approach); measured from the URDF meshes. The body is 10 cm wide along the closing
# axis from 6 to 10 cm behind the TCP: a hand that pinches a sheet lying flat on a wall would sit 5 cm deep in it.
HAND_BOXES = (
    ((0.0, 0.0, -0.020), (0.010, 0.020, 0.020)),  # fingers: TCP - 4 cm .. the TCP
    ((0.0, 0.0, 0.0065), (0.005, 0.009, 0.0065)),  # finger tips: narrower, to 1.3 cm past the TCP
    ((0.0, 0.0, -0.050), (0.024, 0.034, 0.010)),  # finger roots
    ((0.0, 0.0, -0.080), (0.051, 0.034, 0.020)),  # body
    ((0.0, 0.0, -0.122), (0.029, 0.029, 0.022)),  # wrist flange
)


def hand_points(pos, rot, step: float = 0.008, finger: float = 0.0) -> np.ndarray:
    """Points on the surfaces of :data:`HAND_BOXES` (world, m) for the TCP at ``pos`` turned ``rot``, the fingers
    open by ``finger`` (the finger joint, m: 0 closed .. FINGER_OPEN) each."""
    pts = []
    for i, (c, h) in enumerate(HAND_BOXES):
        if i in (0, 1):  # the fingers open along the closing axis
            h = (h[0] + abs(finger), h[1], h[2])
        axes = [np.linspace(-hh, hh, max(2, int(np.ceil(2 * hh / step)) + 1)) for hh in h]
        g = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
        g = g[np.any(np.isclose(np.abs(g), h), axis=1)]  # the surface only
        pts.append(g + c)
    local = np.concatenate(pts)
    return local @ np.asarray(rot, float).T + np.asarray(pos, float)

def opening(value: float) -> float:
    """Finger joint target for a half-opening given in metres. The YAM finger joint runs from 0 (closed) to
    FINGER_OPEN = -0.047 (open), so a positive opening (the natural way to write "2 cm per finger") must be
    negated; a value already on the joint's negative axis passes through. Clipped to the joint range."""
    return float(max(FINGER_OPEN, -abs(value)))


CLOSE_S, SQUEEZE_S = 1.5, 0.5  # minimum close time; lifting before the pads have squeezed slips the grasp
FINGER_SPEED, FINGER_LAG = 0.02, 1.0  # measured in sim: ~2 cm/s per finger after a ~1 s lag (J3 release, S5 close)


def close_time(approach_finger: float) -> float:
    """Seconds for the fingers to close from ``approach_finger`` onto a thin object: the lag plus the full travel
    to closed at FINGER_SPEED, never less than CLOSE_S. (S5: 2.2 cm handle from fully open needed ~2.5 s; the
    old fixed 1.5 + 0.5 s lifted as the pads touched and dropped the iron.)"""
    return max(CLOSE_S, FINGER_LAG + abs(opening(approach_finger)) / FINGER_SPEED)
ALIGN_S = 0.6  # dwell over the place target before lowering: the TCP was still 5-8 mm off at the end of a carry (J2 hit a slot rail)
SETTLE_S = 0.8  # dwell at the bottom, still gripping, before the release: the descent itself adds ~5 mm of x/y error that converges when the arm is still (A4)
RELEASE_S = 2.0  # opening is as slow (measured ~1 s lag + ~1 s travel); a 0.8 s release lifted drinks 3 cm and dropped them (J3 sim)


def ready_tcp(arm: str) -> tuple[float, float, float]:
    """Where an arm's TCP is in the ready pose on the station rig (world, m)."""
    pos, _ = ik.tcp_pose(np.array(READY, dtype=float), station.BASES[arm])
    return tuple(float(v) for v in pos)


# A hover spot beside each arm's own base where it waits with a straight-down grip while the other arm works:
# reachable with a down grip (the ready pose itself is not; see scripts/park_probe.py), 60 cm from the other
# arm's spot so two parked grippers can never touch, and clear of the task area in front of the bases.
PARK = {"left": (0.12, +0.30, station.TABLE_TOP + 0.16), "right": (0.12, -0.30, station.TABLE_TOP + 0.16)}


def wait(arm: str, seconds: float, finger: float = FINGER_OPEN) -> Waypoint:
    """Hover at the arm's park spot for ``seconds``."""
    return Waypoint(seconds, PARK[arm], opening(finger), "wait")


def grasp(pos, *, travel_z: float, approach_finger: float = FINGER_OPEN, yaw: float = 0.0) -> list[Waypoint]:
    """Come down on ``pos`` from ``travel_z``, close, squeeze. ``yaw`` turns the grip about the vertical
    (0 closes the fingers along world y; pi/2 along world x)."""
    x, y, z = pos
    approach_finger = opening(approach_finger)
    return [
        Waypoint(1.5, (x, y, travel_z), approach_finger, "over the object", yaw),
        Waypoint(1.5, (x, y, z), approach_finger, "descend", yaw),
        Waypoint(close_time(approach_finger), (x, y, z), FINGER_CLOSED, "close", yaw),
        Waypoint(SQUEEZE_S, (x, y, z), FINGER_CLOSED, "squeeze", yaw),
    ]


def lift_and_hold(pos, *, hold_at, hold_s: float, yaw: float = 0.0) -> list[Waypoint]:
    """Lift from ``pos`` to ``hold_at`` and stay there (fingers closed)."""
    x, y, _ = pos
    return [
        Waypoint(1.2, (x, y, hold_at[2]), FINGER_CLOSED, "lift", yaw),
        Waypoint(1.5, tuple(hold_at), FINGER_CLOSED, "carry", yaw),
        Waypoint(hold_s, tuple(hold_at), FINGER_CLOSED, "hold", yaw),
    ]


def grasp_and_carry(pick, place, *, travel_z: float, approach_finger: float = FINGER_OPEN, release_finger: float = FINGER_OPEN, yaw: float = 0.0, place_yaw: float | None = None, settle_s: float = SETTLE_S) -> list[Waypoint]:
    """Top-down pick and place with time for the fingers to close before the lift. ``yaw`` turns the grip at the
    pick (fingers along world y at 0); ``place_yaw`` (default ``yaw``) is the grip's turn when setting down;
    ``settle_s`` (0 to skip) is the still dwell at the bottom before the release."""
    (px, py, pz), (qx, qy, qz) = pick, place
    release_finger = opening(release_finger)
    end_yaw = yaw if place_yaw is None else place_yaw
    return [
        *grasp((px, py, pz), travel_z=travel_z, approach_finger=approach_finger, yaw=yaw),
        Waypoint(1.2, (px, py, travel_z), FINGER_CLOSED, "lift", yaw),
        Waypoint(1.8, (qx, qy, travel_z), FINGER_CLOSED, "carry", end_yaw),
        Waypoint(ALIGN_S, (qx, qy, travel_z), FINGER_CLOSED, "align", end_yaw),
        Waypoint(1.2, (qx, qy, qz), FINGER_CLOSED, "lower", end_yaw),
        *([Waypoint(settle_s, (qx, qy, qz), FINGER_CLOSED, "settle", end_yaw)] if settle_s > 0 else []),
        Waypoint(RELEASE_S, (qx, qy, qz), release_finger, "release", end_yaw),
        Waypoint(1.0, (qx, qy, travel_z), release_finger, "retreat", end_yaw),
    ]


def press(pos, *, travel_z: float, press_s: float = 0.5) -> list[Waypoint]:
    """Press ``pos`` with the closed fingertips, coming from and returning to ``travel_z``."""
    x, y, z = pos
    return [
        Waypoint(1.5, (x, y, travel_z), FINGER_CLOSED, "over the button"),
        Waypoint(1.2, (x, y, z), FINGER_CLOSED, "press"),
        Waypoint(press_s, (x, y, z), FINGER_CLOSED, "hold press"),
        Waypoint(1.0, (x, y, travel_z), FINGER_CLOSED, "up"),
    ]


def duration(waypoints) -> float:
    return float(sum(w.duration for w in waypoints))


# -- the whole arm, from its collision meshes ---------------------------------------------------------------------

ARM_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")


def _link_samples(spacing: float = 0.01) -> dict:
    """Surface points of each arm link's collision meshes in the link frame (about one per ``spacing`` squared)."""
    global _LINK_SAMPLES
    if _LINK_SAMPLES is None:
        import os
        import re

        import trimesh
        from yam_arena import YAM_URDF

        text = open(YAM_URDF).read()
        root = os.path.dirname(YAM_URDF)
        out = {}
        for link in ARM_LINKS:
            block = re.search(r'<link name="%s".*?</link>' % link, text, re.S).group(0)
            meshes = []
            for col in re.findall(r"<collision>.*?</collision>", block, re.S):
                fn = re.search(r'filename="([^"]+)"', col).group(1).replace("package://", "")
                fn = fn if os.path.isabs(fn) else os.path.join(root, fn)
                mesh = trimesh.load(fn, force="mesh")
                org = re.search(r"<origin([^>]*)/>", col)
                if org:
                    xyz = re.search(r'xyz="([^"]+)"', org.group(1))
                    rpy = re.search(r'rpy="([^"]+)"', org.group(1))
                    t = np.eye(4)
                    if rpy:
                        t[:3, :3] = trimesh.transformations.euler_matrix(*map(float, rpy.group(1).split()), "sxyz")[:3, :3]
                    if xyz:
                        t[:3, 3] = list(map(float, xyz.group(1).split()))
                    mesh.apply_transform(t)
                meshes.append(mesh)
            mesh = trimesh.util.concatenate(meshes)
            n = int(np.clip(mesh.area / spacing**2, 50, 3000))
            pts, _ = trimesh.sample.sample_surface(mesh, n, seed=0)
            out[link] = np.asarray(pts, float)
        _LINK_SAMPLES = out
    return _LINK_SAMPLES


_LINK_SAMPLES = None


def arm_points(q, base_pos, finger: float = 0.0) -> np.ndarray:
    """Surface points (world, m) of one arm's links 1-6, gripper and fingers for joints ``q`` (6) and finger joint
    ``finger`` (m, 0 closed .. FINGER_OPEN), from the URDF's collision meshes."""
    from yam_arena import YAM_URDF
    from yam_arena.constants import ARM_JOINTS
    from yam_arena.urdf import forward_kinematics

    joints = {n: float(v) for n, v in zip(ARM_JOINTS, q, strict=True)}
    joints.update(joint7=float(finger), joint8=float(finger))
    frames = forward_kinematics(YAM_URDF, joints)
    pts = [samples @ frames[link][:3, :3].T + frames[link][:3, 3] for link, samples in _link_samples().items()]
    return np.concatenate(pts) + np.asarray(base_pos, float)
