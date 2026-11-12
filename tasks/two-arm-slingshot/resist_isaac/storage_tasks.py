"""Task-owned storage apparatus, native resets and geometric evaluation."""

import itertools
import json
import math
from pathlib import Path
import runpy
import tempfile

from .state import pose_state, reset_joints, tensor


TABLE_TOP = 0.7475
POSITIONS = {
    "01": (0.35, -0.13, TABLE_TOP),
    "02": (0.55, 0.0, TABLE_TOP),
    "04": (0.38, 0.0, TABLE_TOP),
    "05": (0.35, -0.13, TABLE_TOP),
    "10": (0.45, 1.0, 0.72),
    "17": (0.38, 0.0, TABLE_TOP + 0.0025),
}


def _rotation(q):
    import numpy as np

    w, x, y, z = map(float, q)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - w*z), 2 * (x*z + w*y)],
        [2 * (x*y + w*z), 1 - 2 * (x*x + z*z), 2 * (y*z - w*x)],
        [2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2 * (x*x + y*y)],
    ])


def _axis_rotation(axis, angle):
    import numpy as np

    if axis == "X":
        vector = np.array([1.0, 0, 0])
    elif axis == "Y":
        vector = np.array([0, 1.0, 0])
    else:
        vector = np.array([0, 0, 1.0])
    return _rotation([math.cos(angle / 2), *(vector * math.sin(angle / 2))])


def create_assets(task_id, folder, directory, condition="recoverable"):
    """Author each task's native assets; all initial state changes are reset-only."""
    from .usd_assets import create_asset

    if condition not in ("normal", "recoverable") and not (task_id == "01" and condition == "stiff"):
        raise ValueError(f"Unsupported storage condition: {condition}")
    folder, directory = Path(folder), Path(directory)
    description = json.loads((folder / "geometry.json").read_text())
    if task_id == "01" and condition == "stiff":
        hinge = next(joint for joint in description["joints"] if joint["name"] == "p01_hinge")
        hinge.update(static_friction_effort=0.10, dynamic_friction_effort=0.10)
    result = {"apparatus": create_asset(description, directory / "apparatus.usda", folder)}
    for name in {"05": ("stock",), "10": ("utensil",), "17": ("frame",)}.get(task_id, ()):
        source = json.loads((folder / f"{name}.json").read_text())
        result[name] = create_asset(source, directory / f"{name}.usda", folder)
    return result


class StorageTask:
    """One fixed storage design with supported geometric reset variants."""

    hold_s = 1.0

    def __init__(self, task_id, folder, condition):
        if task_id not in POSITIONS or (condition not in ("normal", "recoverable") and not (task_id == "01" and condition == "stiff")):
            raise ValueError("Unsupported storage task or condition")
        self.task_id, self.folder, self.condition = task_id, Path(folder), condition
        self.description = json.loads((self.folder / "geometry.json").read_text())

    def build_scene(self, embodiment):
        from isaaclab_arena.assets.object import Object, ObjectType
        from isaaclab_arena.utils.pose import Pose

        self._temporary = tempfile.TemporaryDirectory(prefix="storage-apparatus-")
        generate = runpy.run_path(str(self.folder / "build_scene.py"))["create_assets"]
        assets = generate(self._temporary.name, self.condition)
        position = list(POSITIONS[self.task_id])
        if self.task_id == "17" and self.condition == "normal":
            position[0] -= 0.008
        objects = [Object(name="apparatus", object_type=ObjectType.ARTICULATION,
                          usd_path=str(assets["apparatus"]), initial_pose=Pose(position_xyz=tuple(position)))]
        if self.task_id == "05":
            normal = self.condition == "normal"
            offset = (0.084, 0.10, 0.121) if normal else (0.05, 0.20, 0.121)
            yaw = math.radians(-55) if normal else 0.0
            objects.append(Object(name="stock", object_type=ObjectType.RIGID, usd_path=str(assets["stock"]),
                                  initial_pose=Pose(position_xyz=tuple(a+b for a, b in zip(position, offset)),
                                                    rotation_xyzw=(0, 0, math.sin(yaw/2), math.cos(yaw/2)))))
        elif self.task_id == "10":
            normal = self.condition == "normal"
            offset = (0, -0.278, 0.166780318) if normal else (0, -0.39, 0.214551525)
            yaw = math.pi / 4 if normal else 0.0
            objects.append(Object(name="utensil", object_type=ObjectType.RIGID, usd_path=str(assets["utensil"]),
                                  initial_pose=Pose(position_xyz=tuple(a+b for a, b in zip(position, offset)),
                                                    rotation_xyzw=(0, 0, math.sin(yaw/2), math.cos(yaw/2)))))
        elif self.task_id == "17":
            objects.append(Object(name="drive_bay", object_type=ObjectType.BASE, usd_path=str(assets["frame"]),
                                  initial_pose=Pose(position_xyz=(0.38, 0, TABLE_TOP))))
        return objects

    def reset(self, base):
        self._invalid = False
        normal = self.condition in ("normal", "stiff")
        positions = {
            "01": {"p01_hinge": 0.0, "p01_cam_angle": math.pi/2 if normal else 0.0},
            "02": {"joint_top_drawer_low": 0.0, "joint_middle_drawer_low": 0.0,
                   "joint_bottom_drawer_low": 0.0, "p02_bar_angle": 1.70 if normal else 0.0},
            "04": {"s04_lid_angle": 0.0, "s04_latch_left": -0.85 if normal else 0.0,
                   "s04_latch_right": -0.85 if normal else 0.0},
            "05": {"p01_hinge": 1.1},
            "10": {"p10_door_joint": math.pi/2, "p10_rack1_joint": 0.33,
                   "p10_rack0_joint": 0.001, "p10_button_lock_joint": 0.0, "p10_button_power_joint": 0.0},
            "17": {"s17_handle_angle": 1.35 if normal else 0.0, "s17_pawl_angle": 0.0, "s17_button_press": 0.0},
        }[self.task_id]
        reset_joints(base, "apparatus", positions)

    def _mechanism_state(self, base):
        import numpy as np

        asset = base.scene["apparatus"]
        q, velocity = tensor(asset.data.joint_pos)[0], tensor(asset.data.joint_vel)[0]
        values = {name: float(q[index].item()) for index, name in enumerate(asset.joint_names)}
        speeds = {name: float(velocity[index].item()) for index, name in enumerate(asset.joint_names)}
        poses = {}
        finite = all(math.isfinite(v) for v in [*values.values(), *speeds.values()])
        for body in self.description["bodies"]:
            p, r, v = pose_state(asset, body["name"])
            p, r, v = np.asarray(p.tolist()), np.asarray(r.tolist()), np.asarray(v.tolist())
            finite &= np.isfinite(p).all() and np.isfinite(r).all() and np.isfinite(v).all()
            finite &= abs(float(r @ r) - 1) <= 0.001
            poses[body["name"]] = (p, _rotation(r), v)
        if not finite:
            return values, speeds, poses, False
        first = self.description["bodies"][0]
        if self.description.get("fixed_base"):
            expected = {first["name"]: (np.asarray(POSITIONS[self.task_id])+first["position"], _rotation(first["orientation"]))}
        else:
            expected = {first["name"]: poses[first["name"]][:2]}
        intact = True
        for joint in self.description["joints"]:
            angle = values[joint["name"]]
            tolerance = 0.001 if joint["type"] == "prismatic" else 0.01
            intact &= joint.get("lower", -math.inf)-tolerance <= angle <= joint.get("upper", math.inf)+tolerance
            pp, pr = expected[joint["parent"]]
            anchor = pp + pr @ np.asarray(joint["position_parent"])
            axes = pr @ _rotation(joint["orientation_parent"])
            rotation = axes
            if joint["type"] == "revolute":
                rotation = axes @ _axis_rotation(joint["axis"], angle)
            elif joint["type"] == "prismatic":
                direction = np.eye(3)[{"X": 0, "Y": 1, "Z": 2}[joint["axis"]]]
                anchor += axes @ direction * angle
            child_rotation = rotation @ _rotation(joint["orientation_child"]).T
            child_position = anchor - child_rotation @ np.asarray(joint["position_child"])
            expected[joint["child"]] = (child_position, child_rotation)
        for name, (ep, er) in expected.items():
            p, r, _ = poses[name]
            intact &= np.linalg.norm(p-ep) <= 0.002
            cosine = min(1.0, max(-1.0, float((np.trace(er.T @ r)-1)/2)))
            intact &= math.acos(cosine) <= 0.02
        return values, speeds, poses, bool(intact)

    @staticmethod
    def _free_state(asset, vertices):
        import numpy as np

        p, q, v = pose_state(asset)
        p, q, v = np.asarray(p.tolist()), np.asarray(q.tolist()), np.asarray(v.tolist())
        finite = np.isfinite(p).all() and np.isfinite(q).all() and np.isfinite(v).all()
        if not finite or abs(float(q @ q)-1) > 0.001:
            return p, _rotation([1, 0, 0, 0]), v, None
        rotation = _rotation(q)
        return p, rotation, v, np.asarray(vertices) @ rotation.T + p

    def _workspace(self, points):
        import numpy as np

        far_y = 1.35 if self.task_id == "10" else 0.65
        return bool(np.all(points >= [-0.19, -0.65, TABLE_TOP-0.002]) and np.all(points <= [0.91, far_y, 1.50]))

    @staticmethod
    def _clearance(base, position, rotation, bounds):
        import numpy as np
        from yam_arena.runtime import tcp_world

        result = math.inf
        for name in ("left_robot", "right_robot"):
            point = (np.asarray(tcp_world(base.scene[name]))-position) @ rotation
            for low, high in bounds:
                displacement = np.maximum(np.maximum(np.asarray(low)-point, point-np.asarray(high)), 0)
                result = min(result, float(np.linalg.norm(displacement)))
        return result

    def evaluate(self, base):
        import numpy as np

        values, speeds, poses, intact = self._mechanism_state(base)
        if not intact:
            self._invalid = True
            return {"success": False, "score": 0.0, "invalid_apparatus": True}
        score, candidate = 0.0, False
        metrics = {}
        if self.task_id in ("01", "02", "04"):
            joint, first, second, goal, limit = {
                "01": ("p01_hinge", 0.20, 0.40, 0.65, 0.10),
                "02": ("joint_top_drawer_low", 0.05, 0.12, 0.18, 0.02),
                "04": ("s04_lid_angle", 0.20, 0.50, 0.85, 0.03),
            }[self.task_id]
            progress = values[joint]
            candidate = progress >= goal and abs(speeds[joint]) < limit
            score = 1.0 if candidate else 0.5 if progress >= second else 0.25 if progress >= first else 0.0
            metrics.update(progress=progress, progress_speed=speeds[joint])
        elif self.task_id == "05":
            corners = list(itertools.product((-0.1, 0.1), (-0.01, 0.01), (-0.02, 0.02)))
            position, rotation, velocity, points = self._free_state(base.scene["stock"], corners)
            if points is None or not self._workspace(points):
                self._invalid = True
                return {"success": False, "score": 0.0, "stock_lost": True}
            low = np.asarray(POSITIONS["05"]) + [0.003, 0.010, 0.100]
            high = np.asarray(POSITIONS["05"]) + [0.165, 0.250, 0.280]
            inside = bool(np.all(points >= low-0.0001) and np.all(points <= high+0.0001))
            floor_gap = abs(float(points[:, 2].min())-low[2])
            clearance = self._clearance(base, position, rotation, [([-0.1, -0.01, -0.02], [0.1, 0.01, 0.02])])
            settled = np.linalg.norm(velocity[:3]) < 0.02 and np.linalg.norm(velocity[3:]) < 0.10
            angle = abs(values["p01_hinge"])
            candidate = inside and angle < 0.005 and abs(speeds["p01_hinge"]) < 0.03 and settled and floor_gap < 0.002 and clearance >= 0.05
            score = 1.0 if candidate else 0.5 if inside and angle < 0.4 else 0.25 if inside and angle < 0.8 else 0.0
            metrics.update(door_angle_rad=angle, stock_inside=inside, stock_floor_gap_m=floor_gap, minimum_tcp_clearance_m=clearance)
        elif self.task_id == "10":
            bounds = [([-0.30, -0.012, -0.005], [0.10, 0.012, 0.005]), ([0.08, -0.045, -0.004], [0.30, 0.045, 0.004])]
            vertices = [[x, y, z] for x, half_y, half_z in ((-0.3, 0.012, 0.005), (0.1, 0.012, 0.005), (0.08, 0.018, 0.004), (0.26, 0.045, 0.004), (0.30, 0.035, 0.004)) for y in (-half_y, half_y) for z in (-half_z, half_z)]
            position, rotation, velocity, points = self._free_state(base.scene["utensil"], vertices)
            if points is None or not self._workspace(points):
                self._invalid = True
                return {"success": False, "score": 0.0, "utensil_lost": True}
            rack_position, rack_rotation, _ = poses["p10_rack1"]
            local = (points-rack_position) @ rack_rotation
            inside = bool(np.all(local >= [-0.260, -0.197, 0.111509379]) and np.all(local <= [0.253, 0.300, 0.4527]))
            clearance = self._clearance(base, position, rotation, bounds)
            settled = np.linalg.norm(velocity[:3]) < 0.02 and np.linalg.norm(velocity[3:]) < 0.10
            remaining = values["p10_rack1_joint"]
            candidate = remaining < 0.005 and inside and settled and clearance >= 0.05 and abs(speeds["p10_rack1_joint"]) < 0.02
            score = 1.0 if candidate else 0.5 if inside and remaining < 0.11 else 0.25 if inside and remaining < 0.22 else 0.0
            metrics.update(rack_remaining_m=remaining, contents_inside=inside, contents_settled=bool(settled), minimum_tcp_clearance_m=clearance)
        else:
            position, rotation, velocity = poses["s17_tray"]
            corners = np.asarray(list(itertools.product((-0.0765, 0.050), (-0.051, 0.051), (-0.0015, 0.026)))) @ rotation.T + position
            if not self._workspace(corners):
                self._invalid = True
                return {"success": False, "score": 0.0, "carrier_lost": True}
            rear = float((position + rotation @ [-0.006, 0, 0.0085])[0] + np.abs(rotation[0]) @ [0.0555, 0.0506, 0.012])
            withdrawn = 0.38-float(position[0])
            bench_pose = 0.12 < position[0] < 0.26 and abs(position[1]) < 0.08 and abs(float(corners[:, 2].min())-TABLE_TOP) < 0.002
            upright = rotation[2, 2] > math.cos(0.10)
            clearance = self._clearance(base, position, rotation, [([-0.10, -0.06, -0.002], [0.06, 0.07, 0.045])])
            settled = np.linalg.norm(velocity[:3]) < 0.015 and np.linalg.norm(velocity[3:]) < 0.10
            candidate = rear < 0.3065 and bench_pose and upright and settled and clearance >= 0.05
            score = 1.0 if candidate else 0.5 if withdrawn >= 0.08 else 0.25 if withdrawn >= 0.03 else 0.0
            metrics.update(carrier_withdrawal_m=withdrawn, carrier_rear_x_m=rear, bench_pose=bool(bench_pose), minimum_tcp_clearance_m=clearance)
        if getattr(self, "_invalid", False):
            candidate, score = False, 0.0
        metrics.update(success=bool(candidate), score=float(score))
        return metrics
