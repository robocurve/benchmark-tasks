"""Compensate a passive oscilloscope probe with its trim tool.

The calibration lead is connected and the scope is displaying its reference
signal. Adjust the recessed trimmer with the loose tool and put the tool down.
"""

from pathlib import Path
import tempfile

import numpy as np

from resist_isaac.state import pose_state, reset_joints, tensor
from resist_isaac.usd_assets import create_asset
from .contacts import TrimmerContacts
from .geometry import START_ANGLE, TABLE_Z, TOOL_GRIP, TOOL_REST, TOOL_ROLL, TOOL_TIP
from .geometry import TRIMMER_CENTER, geometry, trace_points
from .waveform import evaluate_waveform, waveform_samples

CONDITIONS = ("recoverable",)


class ProbeCompensation:
    physics_hz = 240
    robot_contact_offset = 0.0001
    hold_s = 1.0

    def __init__(self):
        self.descriptions = geometry()
        self.screen = None
        self.last_angle = None
        self.samples = None
        self.adjusted = False
        self.contacts = None
        self.saved_contact_setting = None

    def build_scene(self, embodiment):
        from isaaclab_arena.assets.object import Object, ObjectType
        from isaaclab_arena.utils.pose import Pose

        cache = Path(tempfile.gettempdir()).resolve()
        task = Path(__file__).resolve().parent
        if cache == task or task in cache.parents:
            raise RuntimeError("Set TMPDIR to a writable directory outside the frozen task source")
        self.temporary = tempfile.TemporaryDirectory(prefix="scope-compensation-", dir=cache)
        poses = {"scope": Pose(position_xyz=(0.0, 0.0, 0.0)),
                 "tool": Pose(position_xyz=tuple(map(float, TOOL_REST)), rotation_xyzw=TOOL_ROLL)}
        return [
            Object(name=name, object_type=ObjectType.ARTICULATION if name == "scope" else ObjectType.RIGID,
                   usd_path=str(create_asset(desc, Path(self.temporary.name) / f"{name}.usda")),
                   initial_pose=poses[name])
            for name, desc in self.descriptions.items()
        ]

    def set_evidence_directory(self, directory):
        self.evidence_directory = Path(directory)

    def pose_state(self, base, name, part=None):
        return pose_state(base.scene[name], part)

    def trimmer_angle(self, base):
        asset = base.scene["scope"]
        index = asset.joint_names.index("compensation")
        return float(tensor(asset.data.joint_pos)[0, index].cpu())

    def initial_action(self):
        from yam_arena.constants import READY

        return np.r_[READY, 1.0, READY, 1.0]

    def configure_native(self, cfg):
        cfg.scene.scope.init_state.joint_pos = {"compensation": START_ANGLE}

    def reset(self, base):
        reset_joints(base, "scope", {"compensation": START_ANGLE})
        self.adjusted = False
        self.last_angle = None

    def validate_reset(self, base):
        pos, _, velocity = self.pose_state(base, "tool")
        pos, velocity = pos.cpu().numpy(), velocity.cpu().numpy()
        if np.linalg.norm(pos - TOOL_REST) > 0.004 or np.linalg.norm(velocity[:3]) > 0.005:
            raise RuntimeError(f"Trim tool must settle at its reset position: {pos}, {velocity}")
        if abs(self.trimmer_angle(base) - START_ANGLE) > 0.01:
            raise RuntimeError("Passive trimmer moved during reset")
        if evaluate_waveform(*self.samples).score:
            raise RuntimeError("Reset trace must start visibly outside the credit band")

    def begin_episode(self, base):
        self.close()
        self.base = base
        self.saved_contact_setting = base.sim.get_setting("/physics/disableContactProcessing")
        base.sim.set_setting("/physics/disableContactProcessing", False)
        self.contacts = TrimmerContacts(base)
        self.create_screen()
        self.update_screen(self.trimmer_angle(base))
        base.sim.render()
        base.sim.render()

    def create_screen(self):
        import omni.usd
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        index = len(self.descriptions["scope"]["bodies"][0]["shapes"]) - 1
        from pxr import Usd

        meshes = [prim for prim in stage.Traverse(Usd.TraverseInstanceProxies())
                  if prim.IsA(UsdGeom.Mesh)
                  and str(prim.GetPath()).endswith(f"/instrument/shape_{index:03}")]
        if len(meshes) != 1:
            raise RuntimeError(f"Expected one scope trace mesh, found {len(meshes)}")
        if meshes[0].IsInstanceProxy():
            path = meshes[0].GetPath()
            parent = meshes[0].GetParent()
            while parent.IsInstanceProxy():
                parent = parent.GetParent()
            parent.SetInstanceable(False)
            meshes[0] = stage.GetPrimAtPath(path)
        self.screen = UsdGeom.Mesh(meshes[0])

    def update_screen(self, angle):
        from pxr import Gf, Vt

        if angle == self.last_angle:
            return
        self.samples = waveform_samples(angle)
        self.trace_result = evaluate_waveform(*self.samples)
        points = [Gf.Vec3f(*point) for point in trace_points(self.samples)]
        self.screen.GetPointsAttr().Set(Vt.Vec3fArray(points))
        self.last_angle = angle

    def evaluate(self, base):
        from yam_arena.runtime import tcp_world
        from yam_arena.urdf import quat_xyzw_to_matrix

        angle = self.trimmer_angle(base)
        self.update_screen(angle)
        result = self.trace_result
        load, count, torque = self.contacts.consume_peak()
        pos, quat, velocity = self.pose_state(base, "tool")
        pos, quat, velocity = (value.cpu().numpy() for value in (pos, quat, velocity))
        rotation = quat_xyzw_to_matrix(quat)
        tip = pos + rotation @ TOOL_TIP
        contact_region = (np.linalg.norm(tip[:2] - TRIMMER_CENTER[:2]) < 0.0015
                          and -0.002 < tip[2] - TRIMMER_CENTER[2] < 0.0001
                          and rotation[2, 2] > 0.95)
        self.adjusted |= bool(contact_region and count and abs(angle - START_ANGLE) > 0.06)
        tool_position = pos + rotation @ TOOL_GRIP
        clear = all(np.linalg.norm(tcp_world(base.scene[f"{side}_robot"]) - tool_position) > 0.08
                    for side in ("left", "right"))
        on_table = (0.01 < tool_position[0] < 0.68 and abs(tool_position[1]) < 0.52
                    and abs(tool_position[2] - TABLE_Z - 0.005) < 0.002
                    and abs(rotation[2, 2]) < 0.1 and np.linalg.norm(velocity[:3]) < 0.008
                    and np.linalg.norm(velocity[3:]) < 0.14)
        released = bool(clear and on_table)
        score = result.score if self.adjusted else 0.0
        if score == 1.0 and not released:
            score = 0.75
        return dict(success=score == 1.0, score=score, distortion_score=result.score,
                    trace_error=result.error_fraction if result.error_fraction is not None else 1.0,
                    trace_vpp=result.vpp or 0.0, trace_frequency_hz=result.frequency_hz or 0.0,
                    trimmer_angle_rad=angle, adjustment_observed=self.adjusted,
                    blade_in_port=bool(contact_region), tool_released=released,
                    blade_load_N=load, blade_contacts=count, blade_normal_torque_Nm=torque)

    def scripted_actions(self, base, policy, max_steps):
        from .controller import scripted_actions

        yield from scripted_actions(self, base, policy, max_steps)

    def close(self):
        if self.contacts is not None:
            if getattr(self, "evidence_directory", None) is not None and self.samples is not None:
                import csv

                with (self.evidence_directory / "waveform.csv").open("w", newline="") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(("time_s", "voltage_v"))
                    writer.writerows(zip(*self.samples))
            self.contacts.close()
            self.contacts = None
            self.base.sim.set_setting("/physics/disableContactProcessing", self.saved_contact_setting)


def make_task(condition):
    if condition not in CONDITIONS:
        raise ValueError(condition)
    return ProbeCompensation()
