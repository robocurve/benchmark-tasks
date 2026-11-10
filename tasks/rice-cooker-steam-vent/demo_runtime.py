"""Read actual towel, robot and passive plunger state; no object animation."""
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
        from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, FINGER_OPEN
        from yam_arena.urdf import quat_xyzw_to_matrix
        self.env, self.base = env, env.unwrapped
        self.specs = {s.name:s for s in specs}
        self.layout = json.loads((task.ROOT/"layout.json").read_text())
        self.scorer = TaskScorer()
        self.vertices = {"folded_towel":task.collision_vertices(self.specs["folded_towel"])}
        self.grasps = {"folded_towel":np.array(self.layout["folded_towel"]["grasp_centre_local"],float)}
        self.arm_ids, self.finger_ids = {}, {}
        self.finger_open = FINGER_OPEN
        self.button_rotation = quat_xyzw_to_matrix(np.array(self.specs["cook_button"].rotation_xyzw))
        for side in ["left","right"]:
            robot = self.base.scene[side+"_robot"]
            self.arm_ids[side] = robot.find_joints(ARM_JOINTS,preserve_order=True)[0]
            self.finger_ids[side] = robot.find_joints(FINGER_JOINTS,preserve_order=True)[0]

    def reset(self):
        import torch
        # Reset only. No manipulation step writes towel or plunger poses.
        for name in ["folded_towel","cook_button"]:
            obj = self.base.scene[name if name=="folded_towel" else name+"_moving"]
            spec = self.specs[name]
            obj.write_root_pose_to_sim(torch.tensor([[*spec.position,*spec.rotation_xyzw]],
                device=self.base.device,dtype=torch.float32))
            obj.write_root_velocity_to_sim(torch.zeros((1,6),device=self.base.device))
        self.scorer.reset()

    def body(self, name="folded_towel"):
        from yam_arena.urdf import quat_xyzw_to_matrix
        obj = self.base.scene[name]
        pose = numpy(obj.data.root_link_pose_w)[0].astype(float)
        rotation = quat_xyzw_to_matrix(pose[3:])
        points = self.vertices[name]@rotation.T+pose[:3]
        velocity = numpy(obj.data.root_com_vel_w)[0].astype(float)
        return dict(pose=pose,rotation=rotation,points=points,velocity=velocity,
            low=points.min(0),high=points.max(0),grasp=pose[:3]+rotation@self.grasps[name])

    def arm(self, side):
        from yam_arena import ik,station
        q = numpy(self.base.scene[side+"_robot"].data.joint_pos)[0]
        angles = q[self.arm_ids[side]].astype(float)
        grip = float(np.clip(q[self.finger_ids[side]].mean()/self.finger_open,0,1))
        position,rotation = ik.tcp_pose(angles,station.BASES[side])
        return angles,grip,position,rotation

    def controls(self):
        import cv2
        from yam_arena.urdf import quat_xyzw_to_matrix
        towel = self.body()
        top = float(self.layout["table_top_z"])
        vent = np.array(self.specs["rice_cooker"].position)+np.array(self.layout["rice_cooker"]["vent_centre_local"])
        hull = cv2.convexHull(towel["points"][:,:2].astype(np.float32))
        distance = -float(cv2.pointPolygonTest(hull,tuple(map(float,vent[:2])),True))
        vent_clear = distance >= .026
        tray = np.array(self.specs["towel_tray"].position)
        in_tray = bool(np.all(np.abs(towel["pose"][:2]-tray[:2]) <= [.072,.047])
                       and abs(towel["low"][2]-(top+.008)) <= .006)
        supported = bool(abs(towel["low"][2]-top) <= .006 or in_tray)
        moving = self.base.scene["cook_button_moving"]
        button_pose = numpy(moving.data.root_link_pose_w)[0].astype(float)
        local_button = self.button_rotation.T@(button_pose[:3]-self.specs["cook_button"].position)
        relative_rotation = self.button_rotation.T@quat_xyzw_to_matrix(button_pose[3:])
        travel = float(local_button[0])
        low_local = self.vertices["folded_towel"].min(0)
        high_local = self.vertices["folded_towel"].max(0)
        towel_distance, button_distance = {}, {}
        face = np.array(self.specs["cook_button"].position)+self.button_rotation@np.array([-.0092,0,0])
        for side in ["left","right"]:
            tcp = self.arm(side)[2]
            local = (tcp-towel["pose"][:3])@towel["rotation"]
            delta = np.maximum(np.maximum(low_local-local,local-high_local),0)
            towel_distance[side] = float(np.linalg.norm(delta))
            button_distance[side] = float(np.linalg.norm(tcp-face))
        released = all(d>=.055 for d in towel_distance.values()) and all(d>=.050 for d in button_distance.values())
        stable = bool(np.linalg.norm(towel["velocity"][:3]) <= .020 and np.linalg.norm(towel["velocity"][3:]) <= .20)
        valid = bool(np.isfinite(towel["pose"]).all() and np.isfinite(towel["velocity"]).all()
            and np.isfinite(button_pose).all() and -.002 <= travel <= .0105
            and np.linalg.norm(local_button[1:]) < .003 and np.trace(relative_rotation)>2.99
            and towel["low"][2]>=top-.006 and towel["low"][0]>=-.18 and towel["high"][0]<=.90
            and towel["low"][1]>=-.64 and towel["high"][1]<=.64)
        return dict(vent_clear=bool(vent_clear),towel_supported=supported,released=bool(released),
            stable=stable,valid=valid,button_travel_m=travel,towel_in_tray=in_tray,
            distance_from_vent_to_towel_projection_m=distance,towel_pose=towel["pose"].tolist(),
            towel_low=towel["low"].tolist(),towel_high=towel["high"].tolist(),
            towel_velocity=towel["velocity"].tolist(),button_pose=button_pose.tolist(),
            tcp_towel_clearance_m=towel_distance,tcp_button_clearance_m=button_distance)

    def score(self):
        state = self.controls()
        keys = ["vent_clear","towel_supported","released","stable","button_travel_m","valid"]
        return self.scorer.update(step=int(self.base.common_step_counter),
                                  **{key:state[key] for key in keys})

    def rgb(self, name):
        return np.ascontiguousarray(numpy(self.base.scene[name].data.output["rgb"])[0,...,:3],dtype=np.uint8)


class VideoRecorder:
    def __init__(self, reader, directory, hz=20):
        self.reader,self.directory = reader,Path(directory)
        self.directory.mkdir(parents=True,exist_ok=True)
        self.processes,self.logs,self.frames = {},[],0
        for name in task.POLICY_CAMERAS:
            frame = reader.rgb(name);height,width=frame.shape[:2]
            log = (self.directory/f"{name}_encode.log").open("w");self.logs.append(log)
            self.processes[name] = subprocess.Popen(["ffmpeg","-hide_banner","-loglevel","error","-y",
                "-f","rawvideo","-pixel_format","rgb24","-video_size",f"{width}x{height}",
                "-framerate",str(hz),"-i","pipe:0","-an","-c:v","libx264","-preset","veryfast",
                "-crf","22","-pix_fmt","yuv420p","-threads","2","-movflags","+faststart",
                str(self.directory/f"{name}.mp4")],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=log)

    def capture(self):
        for name,process in self.processes.items():
            process.stdin.write(self.reader.rgb(name).tobytes())
        self.frames += 1

    def close(self):
        failed=[]
        for name,process in self.processes.items():
            if process.stdin and not process.stdin.closed:process.stdin.close()
            if process.wait(timeout=30):failed.append(name)
        for log in self.logs:log.close()
        if failed:raise RuntimeError(f"Video encoding failed: {failed}")
