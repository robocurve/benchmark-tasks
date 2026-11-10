"""Read physical poses, robot joints and filtered egg/pot contacts."""
from pathlib import Path
import json
import subprocess
import numpy as np
import task
from scoring import TaskScorer


def numpy(value):
    value = value.torch if hasattr(value, "torch") else value
    return value.detach().cpu().numpy()


class StateReader:
    def __init__(self, env, specs):
        from pxr import Usd
        from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, FINGER_OPEN
        self.env, self.base = env, env.unwrapped
        self.specs = {s.name: s for s in specs}
        self.layout = json.loads((task.ROOT / "layout.json").read_text())
        self.scorer = TaskScorer()
        self.vertices, self.grasps = {}, {}
        for name in ["egg", "pot"]:
            self.vertices[name] = task.collision_vertices(self.specs[name])
            stage = Usd.Stage.Open(str(self.specs[name].asset))
            self.grasps[name] = np.array(stage.GetDefaultPrim().GetCustomDataByKey("graspCenterLocal"), float)
        self.grasps["pot"] = np.array(self.layout["pot"]["handle_grasp_local"], float)
        self.arm_ids, self.finger_ids = {}, {}
        self.finger_open = FINGER_OPEN
        for side in ["left", "right"]:
            robot = self.base.scene[side + "_robot"]
            self.arm_ids[side] = robot.find_joints(ARM_JOINTS, preserve_order=True)[0]
            self.finger_ids[side] = robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]

    def reset(self):
        import torch
        # Reset only; demonstration and policy steps never write object poses.
        for name in ["egg", "pot"]:
            spec, obj = self.specs[name], self.base.scene[name]
            pose = torch.tensor([[*spec.position, *spec.rotation_xyzw]],
                                device=self.base.device, dtype=torch.float32)
            obj.write_root_pose_to_sim(pose)
            obj.write_root_velocity_to_sim(torch.zeros((1, 6), device=self.base.device))
        self.scorer.reset()

    def body(self, name):
        from yam_arena.urdf import quat_xyzw_to_matrix
        obj = self.base.scene[name]
        pose = numpy(obj.data.root_link_pose_w)[0].astype(float)
        rotation = quat_xyzw_to_matrix(pose[3:])
        points = self.vertices[name] @ rotation.T + pose[:3]
        velocity = numpy(obj.data.root_com_vel_w)[0].astype(float)
        return dict(pose=pose, rotation=rotation, points=points, velocity=velocity,
                    low=points.min(axis=0), high=points.max(axis=0),
                    grasp=pose[:3] + rotation @ self.grasps[name])

    def arm(self, side):
        from yam_arena import ik, station
        q = numpy(self.base.scene[side + "_robot"].data.joint_pos)[0]
        angles = q[self.arm_ids[side]].astype(float)
        grip = float(np.clip(q[self.finger_ids[side]].mean() / self.finger_open, 0, 1))
        position, rotation = ik.tcp_pose(angles, station.BASES[side])
        return angles, grip, position, rotation

    def controls(self):
        egg, pot = self.body("egg"), self.body("pot")
        top = float(self.layout["table_top_z"])
        trivet = np.array(self.specs["trivet"].position)
        cup = np.array(self.specs["egg_cup"].position)
        egg_clear = np.linalg.norm(egg["points"][:, :2] - trivet[:2], axis=1).min() >= .111
        inside_pot = np.linalg.norm(egg["grasp"][:2] - pot["pose"][:2]) < .120
        egg_clear = bool(egg_clear and not inside_pot)
        cup_radial = float(np.linalg.norm(egg["grasp"][:2] - cup[:2]))
        low = float(egg["low"][2])
        egg_in_rest = cup_radial <= .042 and top + .007 <= low <= top + .035
        egg_supported = bool(abs(low - top) <= .006 or egg_in_rest)
        tilt = float(np.degrees(np.arccos(np.clip(pot["rotation"][2, 2], -1, 1))))
        offset = float(np.linalg.norm(pot["pose"][:2] - trivet[:2]))
        floor = top + float(self.layout["trivet"]["height_m"])
        base_local = self.vertices["pot"]
        base_local = base_local[base_local[:, 2] <= base_local[:, 2].min() + .001]
        base_world = base_local @ pot["rotation"].T + pot["pose"][:3]
        footprint_radius = float(np.linalg.norm(base_world[:, :2] - trivet[:2], axis=1).max())
        # The top collider is a 106 mm-radius 64-gon. Its 105.8 mm
        # inscribed support disk gives a conservative whole-base criterion.
        footprint_supported = footprint_radius <= .1058
        on_trivet = bool(footprint_supported and tilt <= 12 and floor - .003 <= pot["low"][2] <= floor + .006)
        # Check the whole object's local collision bounds, including the pot rim
        # and handle, so an alternative grasp cannot masquerade as a release.
        distances = {}
        for name, body in [("egg", egg), ("pot", pot)]:
            low_local, high_local = self.vertices[name].min(axis=0), self.vertices[name].max(axis=0)
            distances[name] = {}
            for side in ["left", "right"]:
                local_tcp = (self.arm(side)[2] - body["pose"][:3]) @ body["rotation"]
                delta = np.maximum(np.maximum(low_local - local_tcp, local_tcp - high_local), 0)
                distances[name][side] = float(np.linalg.norm(delta))
        released = all(v >= .055 for per_arm in distances.values() for v in per_arm.values())
        stable = all(np.linalg.norm(body["velocity"][:3]) <= .020
                     and np.linalg.norm(body["velocity"][3:]) <= .20 for body in [egg, pot])
        contact = self.base.scene["egg_pot_contact"].data.normal_force_matrix_w_history
        if contact is None:
            raise RuntimeError("Filtered egg/pot contact history is unavailable; refusing to score")
        forces = numpy(contact)
        force = float(np.linalg.norm(forces.reshape(-1, 3), axis=1).max())
        valid = np.isfinite(forces).all() and all(
            np.isfinite(body["pose"]).all() and np.isfinite(body["velocity"]).all()
            and body["low"][2] >= top - .006 and body["low"][0] >= -.18
            and body["high"][0] <= .90 and body["low"][1] >= -.64
            and body["high"][1] <= .64 for body in [egg, pot])
        return dict(
            egg_clear=egg_clear, egg_supported=egg_supported, pot_on_trivet=on_trivet,
            released=released, stable=stable, valid=bool(valid), pot_egg_force_n=force,
            egg_in_rest=bool(egg_in_rest), egg_cup_radial_m=cup_radial,
            pot_center_offset_m=offset, pot_tilt_deg=tilt,
            pot_footprint_radius_m=footprint_radius, pot_base_fully_supported=bool(footprint_supported),
            egg_pose=egg["pose"].tolist(), pot_pose=pot["pose"].tolist(),
            egg_low=egg["low"].tolist(), pot_low=pot["low"].tolist(),
            egg_velocity=egg["velocity"].tolist(), pot_velocity=pot["velocity"].tolist(),
            tcp_object_bounds_clearance_m=distances,
        )

    def score(self):
        state = self.controls()
        step = int(self.base.common_step_counter)
        keys = ["egg_clear", "egg_supported", "pot_on_trivet", "released",
                "stable", "pot_egg_force_n", "valid"]
        return self.scorer.update(step=step, **{key: state[key] for key in keys})

    def rgb(self, name):
        return np.ascontiguousarray(numpy(self.base.scene[name].data.output["rgb"])[0, ..., :3],
                                    dtype=np.uint8)


class VideoRecorder:
    def __init__(self, reader, directory, hz=20):
        self.reader, self.directory = reader, Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.processes, self.logs, self.frames = {}, [], 0
        for name in task.POLICY_CAMERAS:
            frame = reader.rgb(name)
            height, width = frame.shape[:2]
            log = (self.directory / f"{name}_encode.log").open("w")
            self.logs.append(log)
            self.processes[name] = subprocess.Popen([
                "ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo",
                "-pixel_format", "rgb24", "-video_size", f"{width}x{height}", "-framerate", str(hz),
                "-i", "pipe:0", "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "22",
                "-pix_fmt", "yuv420p", "-threads", "2", "-movflags", "+faststart",
                str(self.directory / f"{name}.mp4"),
            ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=log)

    def capture(self):
        for name, process in self.processes.items():
            process.stdin.write(self.reader.rgb(name).tobytes())
        self.frames += 1

    def close(self):
        failed = []
        for name, process in self.processes.items():
            if process.stdin and not process.stdin.closed:
                process.stdin.close()
            if process.wait(timeout=30):
                failed.append(name)
        for log in self.logs:
            log.close()
        if failed:
            raise RuntimeError(f"Video encoding failed: {failed}")
