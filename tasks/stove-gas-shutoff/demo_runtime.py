"""Read actual control poses; implement the plunger's mechanical latch with a drive."""
import json
import numpy as np
import task
from scoring import TaskScorer


def numpy(value):
    value = value.torch if hasattr(value,'torch') else value
    return value.detach().cpu().numpy()


class StateReader:
    def __init__(self,env,specs):
        import omni.usd
        from pxr import UsdPhysics
        from yam_arena.constants import ARM_JOINTS,FINGER_JOINTS,FINGER_OPEN
        self.env,self.base=env,env.unwrapped
        self.specs={s.name:s for s in specs}
        self.scorer=TaskScorer()
        self.arm_ids,self.finger_ids={},{}
        self.finger_open=FINGER_OPEN
        self.layout=json.loads((task.ROOT/'layout.json').read_text())
        for side in ('left','right'):
            robot=self.base.scene[side+'_robot']
            self.arm_ids[side]=robot.find_joints(ARM_JOINTS,preserve_order=True)[0]
            self.finger_ids[side]=robot.find_joints(FINGER_JOINTS,preserve_order=True)[0]
        prim=omni.usd.get_context().get_stage().GetPrimAtPath('/World/envs/env_0/gas_shutoff_button/Joint')
        self.drive=UsdPhysics.DriveAPI(prim,'linear')
        self._drive_latched=False

    def reset(self):
        import torch
        self.drive.GetTargetPositionAttr().Set(0)
        self.drive.GetStiffnessAttr().Set(250)
        self._drive_latched=False
        for name in task.CONTROL_NAMES:
            obj=self.base.scene[name+'_moving']
            spec=self.specs[name]
            pose=torch.tensor([[*spec.position,*spec.rotation_xyzw]],device=self.base.device,dtype=torch.float32)
            obj.write_root_pose_to_sim(pose)
            obj.write_root_velocity_to_sim(torch.zeros((1,6),device=self.base.device))
        self.scorer.reset()

    def controls(self):
        from yam_arena.urdf import quat_xyzw_to_matrix
        values={};valid=True
        for name in task.CONTROL_NAMES:
            spec=self.specs[name]
            pose=numpy(self.base.scene[name+'_moving'].data.root_link_pose_w)[0].astype(float)
            root=quat_xyzw_to_matrix(np.array(spec.rotation_xyzw))
            local_pos=root.T@(pose[:3]-spec.position)
            relative=root.T@quat_xyzw_to_matrix(pose[3:])
            if name=='gas_shutoff_button':
                values['travel_m']=float(local_pos[0])
                valid &= np.linalg.norm(local_pos[1:])<.003 and -.002<local_pos[0]<.0145
                valid &= np.trace(relative)>2.99
            else:
                values[name]=float(np.degrees(np.arctan2(relative[2,1],relative[1,1])))
                valid &= np.linalg.norm(local_pos)<.003 and relative[0,0]>.995
            values[name+'_pose']=pose.tolist()
        values['valid']=bool(valid)
        return values

    def score(self):
        state=self.controls()
        result=self.scorer.update(state['travel_m'],[state['left_burner_knob'],state['right_burner_knob']],valid=state['valid'])
        if result['gas_shutoff_latched'] and not self._drive_latched:
            # The robot must first produce a real 8 mm displacement. This drive
            # holds the joint at its mechanical end stop; no pose is overwritten.
            self.drive.GetStiffnessAttr().Set(1000)
            self.drive.GetTargetPositionAttr().Set(.012)
            self._drive_latched=True
        return result

    def arm(self,side):
        from yam_arena import ik,station
        q=numpy(self.base.scene[side+'_robot'].data.joint_pos)[0]
        angles=q[self.arm_ids[side]].astype(float)
        grip=float(np.clip(q[self.finger_ids[side]].mean()/self.finger_open,0,1))
        p,r=ik.tcp_pose(angles,station.BASES[side])
        return angles,grip,p,r

    def rgb(self,name):
        return np.ascontiguousarray(numpy(self.base.scene[name].data.output['rgb'])[0,...,:3],dtype=np.uint8)


from pathlib import Path
import subprocess
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
