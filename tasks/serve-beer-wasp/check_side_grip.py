"""Which side grips can the right arm hold at each place a glass goes? Harness IK and the gripper's collision meshes, no simulator.

Run with the harness's Python, from this folder: HARNESS_DIR=... python check_side_grip.py
A grip here is tilted `theta` from straight down towards a horizontal direction `phi` (degrees from +x), with its fingers closing across that direction, so that the palm stands beside the glass:
90 is fully from the side. The IK is the harness's, through its plan class, starting from the ready pose.
"""

from __future__ import annotations

import math
import os
import pathlib
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.environ["HARNESS_DIR"])
sys.path.insert(0, str(pathlib.Path(__file__).parent))

import yam_arena.demo as demo  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import ArmPlan, Waypoint  # noqa: E402
from yam_arena.station import BASES, TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

D = math.pi / 180
HARNESS = pathlib.Path(os.environ["HARNESS_DIR"]) / "yam_arena" / "assets" / "yam"
TCP = np.array([0.0, 0.0, 0.134])
GLASS_R = 0.035
PLAIN = demo.grip_rotation


class Pose(float):
    rot: np.ndarray = np.eye(3)


def _grip(yaw=0.0):
    return yaw.rot if isinstance(yaw, Pose) else PLAIN(yaw)


demo.grip_rotation = _grip


def grip(theta: float, phi: float, flip: bool) -> np.ndarray:
    a = np.array([math.sin(phi * D), -math.cos(phi * D), 0.0])
    r = axis_angle_matrix(a, theta * D) @ PLAIN(phi * D)
    if flip:
        r = r @ axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.pi)
    return r


def solves(arm: str, pos, rot: np.ndarray) -> tuple[float, float]:
    w = Pose(0.0)
    w.rot = rot
    plan = ArmPlan([Waypoint(2.0, tuple(pos), FINGER_OPEN, "go", w)], BASES[arm])
    t = 0.0
    while t < plan.duration:
        t += 0.1
        plan.worst_ik_mm, plan.worst_ik_deg = 0.0, 0.0
        plan.action7(t)
    return plan.worst_ik_mm, plan.worst_ik_deg


def _stl(path: pathlib.Path) -> np.ndarray:
    with open(path, "rb") as f:
        f.seek(80)
        n = struct.unpack("<I", f.read(4))[0]
        d = np.frombuffer(f.read(n * 50), dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return d["v"].reshape(-1, 3).astype(float)


def _rpy(rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def gripper_points(half_open: float) -> np.ndarray:
    """Every collision vertex of the gripper and fingers, in the frame of the `gripper` link, with the fingers at this half opening."""
    tree = ET.parse(HARNESS / "yam_arm.urdf").getroot()
    joints = {j.get("name"): j for j in tree.findall("joint")}
    out = []
    for link in tree.findall("link"):
        name = link.get("name")
        if name not in ("gripper", "tip_left", "tip_right"):
            continue
        pts = []
        for c in link.findall("collision"):
            o = c.find("origin")
            xyz, rpy = [np.array([float(v) for v in o.get(k).split()]) for k in ("xyz", "rpy")]
            pts.append(_stl(HARNESS / c.find("geometry/mesh").get("filename")) @ _rpy(rpy).T + xyz)
        p = np.vstack(pts)
        if name != "gripper":
            j = joints["joint7" if name == "tip_left" else "joint8"]
            o = j.find("origin")
            xyz, rpy = [np.array([float(v) for v in o.get(k).split()]) for k in ("xyz", "rpy")]
            axis = np.array([float(v) for v in j.find("axis").get("xyz").split()])
            p = (p + axis * (-half_open)) @ _rpy(rpy).T + xyz
        out.append(p)
    return np.vstack(out)


def clearances(rot: np.ndarray, tcp: np.ndarray, tip, neck_lo, neck_hi, half_open: float, rim_z: float) -> dict:
    """For a grip at `tcp` with this rotation: the gap to the spout's neck box and tip, and how much of the glass's mouth lies under the gripper."""
    pts = (gripper_points(half_open)[::7] - TCP) @ rot.T + tcp
    gap = np.maximum(np.maximum(neck_lo - pts, pts - neck_hi), 0.0)
    to_neck = float(np.linalg.norm(gap, axis=1).min())
    above = pts[pts[:, 2] > rim_z]
    over_mouth = np.hypot(above[:, 0] - tip[0], above[:, 1] - tip[1]) < GLASS_R
    return {"to_neck_cm": round(to_neck * 100, 1), "over_mouth": int(over_mouth.sum())}


def main() -> None:
    import beer_usd as u

    grip_h = 0.05  # the TCP above the glass's base: mid-glass
    base_at_spout = u.SPOUT_TIP - 0.010 - u.MUG_H
    targets = {
        "clean glass 1": (u.MUGS[0][0], u.MUGS[0][1], 0.003 + grip_h),
        "clean glass 2": (u.MUGS[1][0], u.MUGS[1][1], 0.003 + grip_h),
        "at the spout": (u.SPOUT[0], u.SPOUT[1], base_at_spout + grip_h),
        "on the pad": (u.MAT[0], u.MAT[1], u.BAR[4] + 0.003 + grip_h),
    }
    print(f"TCP targets, {grip_h * 100:.0f} cm up a glass. For each tilt: the directions phi (deg) that solve within 2 mm and 2 deg, and the best error otherwise.")
    for name, (x, y, z) in targets.items():
        print(f"{name}: ({x:.3f}, {y:.3f}), TCP {z:.3f}", flush=True)
        for theta in (30, 45, 60, 75, 90):
            good, best = [], (1e9, 0.0, 0, False)
            for flip in (False, True):
                for phi in range(-45, 136, 15):
                    mm, deg = solves("right", (x, y, TABLE_TOP + z), grip(theta, phi, flip))
                    if mm < 2.0 and deg < 2.0:
                        good.append(f"{phi}{'f' if flip else ''}")
                    if mm + deg < best[0]:
                        best = (mm + deg, mm, phi, flip, deg)
            print(f"  tilt {theta:2d}: " + (" ".join(good) if good else f"none; best phi {best[2]}{'f' if best[3] else ''}, {best[1]:.0f} mm, {best[4]:.0f} deg"), flush=True)
    tip = (u.SPOUT[0], u.SPOUT[1])
    neck_lo = np.array([u.SPOUT[0], u.SPOUT[1] - 0.007, TABLE_TOP + u.NECK_Z - 0.0065])
    neck_hi = np.array([u.BEAM_X, u.SPOUT[1] + 0.007, TABLE_TOP + u.NECK_Z + 0.0065])
    tcp = np.array([u.SPOUT[0], u.SPOUT[1], TABLE_TOP + base_at_spout + grip_h])
    rim = TABLE_TOP + u.SPOUT_TIP - 0.010
    half = u.MUG_R - 0.002
    print("Clear of the spout, with a glass centred under it, by phi 45 (from the arm's side):")
    for theta in (0, 30, 45, 60, 75, 90):
        for flip in (False, True):
            print(f"  tilt {theta:2d}{' flipped' if flip else ''}: {clearances(grip(theta, 45, flip), tcp, tip, neck_lo, neck_hi, half, rim)}", flush=True)


if __name__ == "__main__":
    main()
