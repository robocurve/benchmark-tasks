"""Native connector scenes and task-specific geometric outcomes.

All state reads are evaluator-private. Scene descriptions contain explicit
passive bodies and colliders; no model inference, object actions or pose guides
are inserted. Contact support and force envelopes still require native audits.
"""

from copy import deepcopy
import json
import math
from pathlib import Path
import tempfile

import numpy as np

from .state import tensor, reset_joints


def rotation(quaternion):
    """Convert a finite unit scalar-first quaternion to a rotation matrix."""
    w, x, y, z = map(float, quaternion)
    if not np.isfinite([w, x, y, z]).all() or abs(w*w+x*x+y*y+z*z-1) > .001:
        raise ValueError("Invalid native body quaternion")
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])


def create_assets(task_directory, condition, output_directory):
    """Author real native USD without launching Kit or creating an environment."""
    task = ConnectorTask(task_directory, condition)
    return _write_assets(task.directory, task._configured_assets(), output_directory)


def _write_assets(directory, assets, output_directory):
    from .usd_assets import create_asset

    return {name: create_asset(item["description"], Path(output_directory)/f"{name}.usda", directory)
            for name, item in assets.items()}


def _bore_segment(a, b, entry, axis, low, high, radius, bore_radius):
    """Conservative circular containment of the line segment overlapping a finite bore."""
    a, b = a-entry, b-entry
    delta = b-a
    if abs(delta[axis]) < 1e-12:
        if not low <= a[axis] <= high:
            return True, False
        t0, t1 = 0.0, 1.0
    else:
        ends = sorted(((low-a[axis])/delta[axis], (high-a[axis])/delta[axis]))
        t0, t1 = max(0.0, ends[0]), min(1.0, ends[1])
        if t0 > t1:
            return True, False
    radial = [index for index in range(3) if index != axis]
    contained = all(np.linalg.norm((a+t*delta)[radial])+radius <= bore_radius for t in (t0, t1))
    return bool(contained), True


class ConnectorTask:
    """Load native task-owned assets and expose one measured endpoint per task."""

    hold_s = 1.0

    def __init__(self, directory, condition):
        self.directory = Path(directory)
        self.spec = json.loads((self.directory / "geometry.json").read_text())
        if condition not in self.spec["conditions"]:
            raise ValueError(f"Unsupported connector condition: {condition}")
        self.condition = condition
        self.variant = self.spec["conditions"][condition]
        self.task_id = self.spec["task_id"]
        self.hold_s = 2.0 if self.task_id in ("12", "16") else 1.0

    def _configured_assets(self):
        self.assets = deepcopy(self.spec["assets"])
        if self.task_id == "13":
            selected = self.variant.get("pin", "compatible")
            self.assets["pin"] = self.assets.pop("pin_" + selected)
            self.assets.pop("pin_oversized" if selected == "compatible" else "pin_compatible")
            self.assets["pin"]["position"][1] += self.variant.get("pin_lateral_bias_m", 0)
            yaw = self.variant.get("pin_yaw_rad", 0)
            if yaw:
                c, s = math.cos(yaw/2), math.sin(yaw/2)
                w, x, y, z = self.assets["pin"]["orientation"]
                self.assets["pin"]["orientation"] = [c*w-s*z, c*x-s*y, c*y+s*x, c*z+s*w]
        if "plug_orientation" in self.variant:
            self.assets["plug"]["orientation"] = self.variant["plug_orientation"]
            # The visible support must fit the rolled overmold without initial penetration.
            rest = self.assets["rest"]["description"]["bodies"][0]["shapes"]
            rest[0]["position"][2] -= .0045
            rest[1]["position"][2] -= .00225
            rest[1]["size"][2] -= .00225
        return self.assets

    def build_scene(self, embodiment):
        from isaaclab_arena.assets.object import Object, ObjectType
        from isaaclab_arena.utils.pose import Pose

        self.assets = self._configured_assets()
        self._asset_specs = self.assets
        self._temporary = tempfile.TemporaryDirectory(prefix="connector-native-")
        paths = _write_assets(self.directory, self.assets, self._temporary.name)
        kinds = {"base": ObjectType.BASE, "rigid": ObjectType.RIGID, "articulation": ObjectType.ARTICULATION}
        out = []
        for name, item in self.assets.items():
            path = paths[name]
            w, x, y, z = item.get("orientation", [1, 0, 0, 0])
            out.append(Object(name=name, object_type=kinds[item["kind"]], usd_path=str(path),
                              initial_pose=Pose(position_xyz=item["position"], rotation_xyzw=(x, y, z, w))))
        return out

    def reset(self, base):
        self._escaped = False
        self._stop_s = 0.0
        for name, positions in self.variant.get("joint_positions", {}).items():
            reset_joints(base, name, positions)

    def _state(self, base, name, body=None):
        obj = base.scene[name]
        kind = self.assets[name]["kind"]
        if kind == "articulation":
            index = obj.body_names.index(body or self.assets[name]["description"]["bodies"][0]["name"])
            pose = tensor(obj.data.body_link_pose_w)[0, index].detach().cpu().numpy()
            velocity = tensor(obj.data.body_com_vel_w)[0, index].detach().cpu().numpy()
        else:
            pose = tensor(obj.data.root_link_pose_w)[0].detach().cpu().numpy()
            velocity = tensor(obj.data.root_com_vel_w)[0].detach().cpu().numpy()
        if not np.isfinite(pose).all() or not np.isfinite(velocity).all():
            raise ValueError("Nonfinite native connector state")
        return pose[:3], rotation(pose[3:]), velocity

    def _joint(self, base, asset, name):
        obj = base.scene[asset]
        value = float(tensor(obj.data.joint_pos)[0, obj.joint_names.index(name)].item())
        if not math.isfinite(value):
            raise ValueError("Nonfinite native connector joint")
        return value

    def _intact(self, base):
        """Check physical retention and native joint anchors before any nonzero credit."""
        self._escaped = getattr(self, "_escaped", False)
        for name, item in self.assets.items():
            if item["kind"] == "base":
                continue
            desc = item["description"]
            states = {body["name"]: self._state(base, name, body["name"])
                      for body in desc["bodies"]} if item["kind"] == "articulation" else {desc["bodies"][0]["name"]: self._state(base, name)}
            for position, _, _ in states.values():
                retained = (-.15 <= position[0] <= .87 and -.61 <= position[1] <= .61
                            and .7455 <= position[2] <= 1.2475)
                self._escaped |= not retained
            if "fixed_base" in desc:
                position, orient, _ = states[desc["fixed_base"]]
                expected = np.asarray(item["position"])
                self._escaped |= np.linalg.norm(position-expected) > .002
                self._escaped |= np.linalg.norm(orient-rotation(item["orientation"])) > .02
            for joint in desc.get("joints", []):
                pa, ra, _ = states[joint["parent"]]
                pb, rb, _ = states[joint["child"]]
                a = pa + ra @ np.asarray(joint.get("position_parent", [0, 0, 0]))
                b = pb + rb @ np.asarray(joint.get("position_child", [0, 0, 0]))
                self._escaped |= np.linalg.norm(a-b) > .002
                if joint["type"] in ("revolute", "prismatic"):
                    value = self._joint(base, name, joint["name"])
                    tolerance = .01 if joint["type"] == "revolute" else .001
                    self._escaped |= value < joint.get("lower", -math.inf)-tolerance
                    self._escaped |= value > joint.get("upper", math.inf)+tolerance
        return not self._escaped

    def _clear_hands(self, base, positions, clearance):
        from yam_arena.runtime import tcp_world

        return all(np.linalg.norm(tcp_world(base.scene[arm])-point) >= clearance
                   for arm in ("left_robot", "right_robot") for point in positions)

    def evaluate(self, base):
        intact = self._intact(base)
        goals = {"11": self._usb, "12": self._unplug, "13": self._pin, "14": self._terminal, "16": self._unplug}
        result = goals[self.task_id](base)
        result["apparatus_intact"] = bool(intact)
        if not intact:
            result["success"], result["score"] = False, 0.0
        return result

    def _usb(self, base):
        position, orient, velocity = self._state(base, "plug", "Plug")
        entry = np.asarray(self.spec["point"])
        tip = position + orient @ np.array([-.025, 0, 0])
        depth = float(entry[0]-tip[0])
        corners = np.array([[-.020, y, z] for y in (-.0062, .0062) for z in (-.0021, .0021)]) @ orient.T + position-entry
        contained = bool(np.all(np.abs(corners[:, 1]) <= .0065) and np.all(np.abs(corners[:, 2]) <= .0024))
        stable = np.linalg.norm(velocity[:3]) < .015 and np.linalg.norm(velocity[3:]) < .1
        success = bool(depth >= .009 and contained and abs(orient[0, 0]-1) < .02 and stable)
        score = 1.0 if success else .5 if contained and depth >= .003 else .25 if contained and depth >= .001 else 0.0
        return dict(success=success, score=score, insertion_depth_m=depth, cross_section_contained=contained)

    def _unplug(self, base):
        position, _, velocity = self._state(base, "plug", "Plug")
        entry = np.asarray(self.spec["point"])
        pad = np.asarray(self.assets["pad"]["position"])
        error = float(np.linalg.norm((position-pad)[:2]))
        released = self._clear_hands(base, [position], .06)
        stable = np.linalg.norm(velocity[:3]) < .015 and np.linalg.norm(velocity[3:]) < .1
        height = float(position[2]-pad[2])
        success = bool(error < .04 and .005 <= height <= .035 and released and stable)
        score = 1.0 if success else .5 if error < .05 and .005 <= height <= .06 else .25 if position[2]-entry[2] >= .035 else 0.0
        return dict(success=success, score=score, pad_error_m=error, pad_relative_height_m=height,
                    hands_clear=released, connector_lift_m=float(position[2]-entry[2]))

    def _pin(self, base):
        position, orient, velocity = self._state(base, "pin")
        entry = np.asarray(self.spec["point"])
        blocked = self.variant.get("pin") == "oversized"
        length, diameter = (.012, .0099955) if blocked else (.010, .0079955)
        tip = position + orient @ np.array([0, 0, -.0125-length])
        rear = position + orient @ np.array([0, 0, -.0125])
        depth = float(entry[0]-tip[0])
        contained, seen = _bore_segment(tip, rear, entry, 0, -.012, 0, diameter/2, .0081205/2)
        contained = bool(contained and seen)
        stable = np.linalg.norm(velocity[:3]) < .015 and np.linalg.norm(velocity[3:]) < .1
        success = bool(not blocked and depth >= .008 and contained and stable)
        score = 1.0 if success else .5 if not blocked and contained and depth >= .004 else .25 if not blocked and contained and depth >= .001 else 0.0
        # Geometric incompatibility and withdrawal are reported separately from insertion credit.
        near_hand = not self._clear_hands(base, [position], .08)
        withdrawn = bool(blocked and depth <= -.007 and stable and near_hand)
        self._stop_s = self._stop_s + float(base.step_dt) if withdrawn else 0.0
        return dict(success=success, score=score, insertion_depth_m=depth, shaft_contained=contained,
                    pin_diameter_m=diameter, bore_diameter_m=.0081205,
                    supported_stop_geometry=bool(self._stop_s >= 2), stop_hold_s=self._stop_s)

    def _terminal(self, base):
        point = np.asarray(self.spec["point"])
        end, end_rotation, velocity = self._state(base, "wire", "Core2")
        tip = end + end_rotation @ np.array([-.010/3-.0008, 0, 0])
        depth = float(point[2]-tip[2])
        core_positions = [self._state(base, "wire", "Wire")[0]]
        neck_contained, seen = True, False
        for index in range(3):
            position, orient, _ = self._state(base, "wire", "Core"+str(index))
            core_positions.append(position)
            other = position + orient @ np.array([-.010/3, 0, 0])
            fits, overlaps = _bore_segment(position, other, point, 2, -.0038, .0008, .0008, .002)
            neck_contained &= fits
            seen |= overlaps
        tool_position, tool_rotation, _ = self._state(base, "tool")
        tool_tip = tool_position + tool_rotation @ np.array([.14, 0, 0])
        cage = self._joint(base, "socket", "cage_angle")
        hands_clear = self._clear_hands(base, core_positions, .05)
        blade_start = tool_position + tool_rotation @ np.array([.125, 0, 0])
        blade = tool_tip-blade_start
        t = np.clip(np.dot(point-blade_start, blade)/np.dot(blade, blade), 0, 1)
        tool_clear = np.linalg.norm(blade_start+t*blade-point) >= .01
        core_com = end + end_rotation @ np.array([-.005/3, 0, 0])
        tip_speed = float(np.linalg.norm(velocity[:3]+np.cross(velocity[3:], tip-core_com)))
        stable = tip_speed < .002
        success = bool(.009 <= depth <= .0118 and neck_contained and seen and -.25 < cage < -.015
                       and tool_clear and hands_clear and stable)
        score = 1.0 if success else .5 if .004 <= depth <= .0118 and neck_contained and seen else .25 if cage < -.20 else 0.0
        return dict(success=success, score=score, wire_depth_m=depth, neck_contained=bool(neck_contained and seen),
                    cage_angle_rad=cage, tool_clear=bool(tool_clear), hands_clear=bool(hands_clear),
                    wire_tip_speed_m_s=tip_speed)
