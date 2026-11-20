"""The rest check on hand-built plate poses, with no simulator: python3 check_score.py. Runs plate_rests, plate_gap and the support points from
task.py, read from its source."""

import ast
import math
import pathlib

import numpy as np

src = pathlib.Path(__file__).with_name("task.py").read_text()
ns = {"np": np, "math": math, "TOP": 0.0}
keep = {"R_EDGE", "PLATE_START", "FOOT_R", "FOOT_H", "TOP_PROFILE", "UNDER_PROFILE", "PLATE_H", "N_FIXED", "PITCH", "SEAT_Z", "FIX_Z", "PAN_X", "PAN_Y",
        "PAN_H", "PAN_W", "PANS", "MAT_C", "MAT_H", "MAT_HALF", "REST_TOL", "PLATE_PTS", "OUTLINE", "SUPPORT_PTS", "SUPPORT_STACK"}
funcs = {"_plate_samples", "_plate_outline", "plate_gap", "_support_points", "stack_top_z", "on_stack", "plate_rests"}
ns["os"] = __import__("os")
for node in ast.parse(src).body:
    if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in funcs:
        exec("from __future__ import annotations\n" + ast.get_source_segment(src, node), ns)
    if isinstance(node, ast.Assign) and any(isinstance(n, ast.Name) and n.id in keep for t in node.targets for n in ast.walk(t)):
        exec(ast.get_source_segment(src, node), ns)
plate_rests = ns["plate_rests"]


class Pose:
    def __init__(self, x, y, z, tilt_deg=0.0, tilt_axis=(1.0, 0.0, 0.0)):
        k = np.array(tilt_axis, float) / np.linalg.norm(tilt_axis)
        a = math.radians(tilt_deg)
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        self.R = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K
        self.pos = np.array([x, y, z], float)

    def to_world(self, p):
        return np.atleast_2d(p) @ self.R.T + self.pos

    def to_local(self, p):
        return (np.atleast_2d(p) - self.pos) @ self.R


def first_touch(pose_fn, top=0.0, which=0):
    """Lowering the plate (or closing it in) from 2 cm above where it truly touches, in 0.25 mm steps: where the rest check first fires."""
    for k in range(200):
        h = top + 0.02 - 0.00025 * k
        if plate_rests(pose_fn(h))[which]:
            return h
    return float("nan")


def under_z(r):
    return float(np.interp(r, *zip(*ns["UNDER_PROFILE"])))


sx, sy = ns["PLATE_START"]
FIX_Z, SEAT_Z, PAN_H, MAT_H = ns["FIX_Z"], ns["SEAT_Z"], ns["PAN_H"], ns["MAT_H"]
rim_top = FIX_Z + ns["TOP_PROFILE"][-1][1]
wall = ns["PAN_X"] / 2 - ns["PAN_W"] / 2
pa, pb = ns["PANS"]["pan_a"], ns["PANS"]["pan_b"]
# name: (the plate's pose as a function of its height or offset, which reading (0 rests, 1 on the stack), where it truly touches)
CASES = {"level, centred on the stack": (lambda z: Pose(sx, sy, z), 1, SEAT_Z),
         "level on the counter": (lambda z: Pose(0.62, 0.0, z), 0, 0.0),
         "level in the zone": (lambda z: Pose(*ns["MAT_C"], z), 0, MAT_H),
         "level, its edge over pan_a's +x wall": (lambda z: Pose(pa[0] + wall + 0.125, pa[1], z), 0, PAN_H - under_z(0.125)),
         "level, its edge over pan_a's +x wall, 8 cm along": (lambda z: Pose(pa[0] + wall + 0.125, pa[1] + 0.08, z), 0, PAN_H - under_z(0.125)),
         "level, its edge over pan_b's -y wall": (lambda z: Pose(pb[0] + 0.03, pb[1] - (ns["PAN_Y"] / 2 - ns["PAN_W"] / 2) - 0.125, z), 0, PAN_H - under_z(0.125)),
         "standing on edge, face against the stack's side": (lambda g: Pose(sx, sy - ns["R_EDGE"] - g, 0.06, 90.0, (1, 0, 0)), 1, 0.0)}
for az in range(0, 360, 10):  # the plate's foot side over the stack's outer edge (stack_prop), all round
    u = np.array([math.cos(math.radians(az)), math.sin(math.radians(az))])
    CASES[f"level, foot side over the stack's edge at {az} deg"] = (lambda z, u=u: Pose(sx + 0.198 * u[0], sy + 0.198 * u[1], z), 1, rim_top)
bad = 0
for name, (fn, which, touch) in CASES.items():
    h = first_touch(fn, touch, which=which)
    good = touch <= h <= touch + 0.0035  # fires before the touch, within 3.5 mm of it
    bad += not good
    print(f"{'ok ' if good else 'BAD'} {name}: fires at {h * 1000:.1f} mm, touches at {touch * 1000:.1f} mm")
held = {"held 6.5 cm over the stack": Pose(sx, sy, SEAT_Z + 0.065), "held 5 cm over the zone": Pose(*ns["MAT_C"], MAT_H + 0.05),
        "held 1 cm over pan_a's wall": Pose(pa[0] + wall + 0.125, pa[1], PAN_H - under_z(0.125) + 0.01)}
for name, pose in held.items():
    good = plate_rests(pose) == (False, False)
    bad += not good
    print(f"{'ok ' if good else 'BAD'} {name}: rests nothing")
print(f"{len(ns['SUPPORT_PTS'])} support points; {'all cases ok' if not bad else f'{bad} BAD'}")
raise SystemExit(bad)
