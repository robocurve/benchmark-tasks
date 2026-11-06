"""Shared simulator-state and recording helpers for the local demonstration."""

import json
from pathlib import Path
import subprocess

import numpy as np

import task
from scoring import BodyState, TaskScorer, rotation


def numpy(value):
    value = value.torch if hasattr(value, "torch") else value
    return value.detach().cpu().numpy()


class StateReader:
    def __init__(self, env, specs):
        from yam_arena import station
        from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, FINGER_OPEN

        self.env, self.base = env, env.unwrapped
        self.specs = {s.name: s for s in specs}
        self.vertices = {s.name: task.collision_vertices(s) for s in specs if s.movable}
        self.arm_ids, self.finger_ids = {}, {}
        self.finger_open = FINGER_OPEN
        for side in ("left", "right"):
            robot = self.base.scene[f"{side}_robot"]
            self.arm_ids[side] = robot.find_joints(ARM_JOINTS, preserve_order=True)[0]
            self.finger_ids[side] = robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        self.layout = json.loads((task.ROOT / "layout.json").read_text())
        self.scorer = TaskScorer(self.layout)
        from contacts import ContactMonitor
        self.contacts = ContactMonitor(env,self.specs)

    def reset_history(self):
        self.scorer.reset()
        self.contacts.reset()
        self.last_score = {"score":0.0,"success":False}

    def body(self, name):
        obj = self.base.scene[name]
        pose = numpy(obj.data.root_link_pose_w)[0].astype(float)
        world = self.vertices[name] @ rotation(pose[3:]).T + pose[:3]
        arena = self.base.arena_world
        return BodyState(pose, numpy(arena.get_root_linear_velocity_w(name))[0],
                         numpy(arena.get_root_angular_velocity_w(name))[0], world.min(axis=0), world.max(axis=0))

    def arm(self, side):
        from yam_arena import ik, station

        q = numpy(self.base.scene[f"{side}_robot"].data.joint_pos)[0]
        angles = q[self.arm_ids[side]].astype(float)
        grip = float(np.clip(q[self.finger_ids[side]].mean() / self.finger_open, 0, 1))
        p, r = ik.tcp_pose(angles, station.BASES[side])
        return angles, grip, p, r

    def score(self):
        self.last_score = self.scorer.update({n:self.body(n) for n in self.vertices},
            {side:self.arm(side)[2] for side in ("left","right")},self.contacts.drain(),
            contact_healthy=self.contacts.contact_count>0 and self.contacts.error is None)
        return self.last_score

    def rgb(self, name):
        return np.ascontiguousarray(numpy(self.base.scene[name].data.output["rgb"])[0, ..., :3], dtype=np.uint8)


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
