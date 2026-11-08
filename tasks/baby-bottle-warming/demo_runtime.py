"""Measured bottle/fixture state and contact-only task scoring."""
from pathlib import Path
import json,subprocess
import numpy as np
import task
from scoring import TaskScorer


def numpy(value):
    value=value.torch if hasattr(value,'torch') else value
    return value.detach().cpu().numpy()


class StateReader:
    def __init__(self,env,specs):
        import omni.usd
        from pxr import UsdPhysics,UsdShade
        from yam_arena.constants import ARM_JOINTS,FINGER_JOINTS,FINGER_OPEN
        self.env,self.base=env,env.unwrapped;self.specs={s.name:s for s in specs}
        self.layout=json.loads((task.ROOT/'layout.json').read_text());self.scorer=TaskScorer()
        self.vertices=task.collision_vertices(self.specs['baby_bottle'])
        self.arm_ids={};self.finger_ids={};self.finger_open=FINGER_OPEN
        for side in ['left','right']:
            robot=self.base.scene[side+'_robot']
            self.arm_ids[side]=robot.find_joints(ARM_JOINTS,preserve_order=True)[0]
            self.finger_ids[side]=robot.find_joints(FINGER_JOINTS,preserve_order=True)[0]
        self.stage=omni.usd.get_context().get_stage()
        self.door_drive=UsdPhysics.DriveAPI(self.stage.GetPrimAtPath('/World/envs/env_0/door/Joint'),'angular')
        self.led=UsdShade.Shader(self.stage.GetPrimAtPath('/World/envs/env_0/microwave/Looks/RunStatus/Surface'))
        self._latched=False;self._catch_steps=0;self._last_running=None

    def reset(self):
        import torch
        self.door_drive.GetStiffnessAttr().Set(0);self.door_drive.GetTargetPositionAttr().Set(0)
        self._latched=False;self._catch_steps=0;self._last_running=None
        for name in (*task.CONTROL_NAMES,'baby_bottle'):
            spec=self.specs[name];obj=self.base.scene[name+'_moving' if name in task.CONTROL_NAMES else name]
            pose=torch.tensor([[*spec.position,*spec.rotation_xyzw]],device=self.base.device,dtype=torch.float32)
            obj.write_root_pose_to_sim(pose);obj.write_root_velocity_to_sim(torch.zeros((1,6),device=self.base.device))
        self.scorer.reset();self.set_indicator(False)

    def body(self):
        from yam_arena.urdf import quat_xyzw_to_matrix
        obj=self.base.scene['baby_bottle'];pose=numpy(obj.data.root_link_pose_w)[0].astype(float)
        r=quat_xyzw_to_matrix(pose[3:]);points=self.vertices@r.T+pose[:3]
        vel=numpy(obj.data.root_com_vel_w)[0].astype(float)
        return dict(pose=pose,rotation=r,low=points.min(axis=0),high=points.max(axis=0),velocity=vel,
                    grasp=pose[:3]+r@np.array(self.layout['bottle']['grasp_center_local']))

    def arm(self,side):
        from yam_arena import ik,station
        q=numpy(self.base.scene[side+'_robot'].data.joint_pos)[0]
        angles=q[self.arm_ids[side]].astype(float)
        grip=float(np.clip(q[self.finger_ids[side]].mean()/self.finger_open,0,1))
        p,r=ik.tcp_pose(angles,station.BASES[side]);return angles,grip,p,r

    def controls(self):
        from yam_arena.urdf import quat_xyzw_to_matrix
        body=self.body();p=body['pose'][:3];R=body['rotation'];up=R[:,2]
        door=numpy(self.base.scene['door_moving'].data.root_link_pose_w)[0].astype(float)
        D=quat_xyzw_to_matrix(door[3:]);angle=-float(np.degrees(np.arctan2(D[1,0],D[0,0])))
        button=numpy(self.base.scene['start_button_moving'].data.root_link_pose_w)[0].astype(float)
        delta=button[:3]-self.specs['start_button'].position
        bath=np.array(self.specs['water_bath'].position);water=float(self.layout['water_bath']['water_level_world_z'])
        floor=float(self.layout['water_bath']['inner_floor_world_z']);top=float(self.layout['table_top_z'])
        neck_local=np.vstack([self.vertices[self.vertices[:,2]>=.128],np.array([[.035*np.cos(a),.035*np.sin(a),.128] for a in np.arange(24)*2*np.pi/24])])
        neck_points=neck_local@R.T+p
        neck_low=float(neck_points[:,2].min());tilt=float(np.degrees(np.arccos(np.clip(up[2],-1,1))))
        radial=float(np.linalg.norm(p[:2]-bath[:2]));in_water=radial<=.050 and p[2]<=water-.030
        neck_submerged=bool(any(np.linalg.norm(v[:2]-bath[:2])<.100 and v[2]<water for v in neck_points))
        in_bath=bool(in_water and tilt<=15 and neck_low>=water+.015)
        supported=bool(floor-.003<=body['low'][2]<=floor+.006)
        speed=float(np.linalg.norm(body['velocity'][:3]));angular=float(np.linalg.norm(body['velocity'][3:]))
        axis_start=p;axis_end=p+.201*up;segment=axis_end-axis_start
        distances={}
        for side in ['left','right']:
            tcp=self.arm(side)[2];t=np.clip(np.dot(tcp-axis_start,segment)/np.dot(segment,segment),0,1)
            distances[side]=float(np.linalg.norm(tcp-(axis_start+t*segment)))
        released=min(distances.values())>=.100
        oven=np.array(self.specs['microwave'].position);inside=bool(oven[0]-.02<p[0]<oven[0]+.30 and oven[1]-.21<p[1]<oven[1]+.21 and oven[2]+.027<p[2]<oven[2]+.353)
        lifted=bool(p[2]>=self.specs['baby_bottle'].position[2]+.030 and tilt<=20 and not inside)
        valid=bool(np.isfinite(body['pose']).all() and np.isfinite(body['velocity']).all() and np.isfinite(door).all() and np.isfinite(button).all()
            and np.linalg.norm(door[:3]-self.specs['door'].position)<.003 and D[2,2]>.995 and -3<angle<95
            and np.linalg.norm(delta[1:])<.003 and -.002<delta[0]<.010
            and body['low'][2]>=top-.006 and body['low'][0]>=-.18 and body['high'][0]<=.90
            and body['low'][1]>=-.64 and body['high'][1]<=.64)
        return dict(door_angle_deg=angle,button_travel_m=float(delta[0]),bottle_in_bath=in_bath,
            bottle_supported=supported,released=released,lifted=lifted,neck_submerged=neck_submerged,
            stable=speed<=.020 and angular<=.15,valid=valid,bottle_pose=body['pose'].tolist(),
            bottle_low=body['low'].tolist(),bottle_high=body['high'].tolist(),bottle_speed_m_s=speed,
            bottle_angular_speed_rad_s=angular,bottle_tilt_deg=tilt,bath_radial_distance_m=radial,
            neck_water_clearance_m=neck_low-water,tcp_axis_distance_m=distances,door_pose=door.tolist(),button_pose=button.tolist())

    def set_indicator(self,running):
        from pxr import Gf
        if self._last_running==running:return
        self._last_running=running
        self.led.GetInput('diffuseColor').Set(Gf.Vec3f(.02,.65,.08) if running else Gf.Vec3f(.015,.025,.02))
        self.led.GetInput('emissiveColor').Set(Gf.Vec3f(.02,1.2,.08) if running else Gf.Vec3f(0))

    def score(self):
        state=self.controls();angle=state['door_angle_deg']
        self._catch_steps=self._catch_steps+1 if abs(angle)<=2 else 0
        if self._catch_steps>=3 and not self._latched:
            self.door_drive.GetTargetPositionAttr().Set(35.);self.door_drive.GetStiffnessAttr().Set(2.);self._latched=True
        if abs(angle)>8 and self._latched:self.door_drive.GetStiffnessAttr().Set(0);self._latched=False
        keys=['door_angle_deg','button_travel_m','bottle_in_bath','bottle_supported','released','lifted','neck_submerged','stable','valid']
        result=self.scorer.update(**{k:state[k] for k in keys});self.set_indicator(result['microwave_started']);return result

    def rgb(self,name):
        return np.ascontiguousarray(numpy(self.base.scene[name].data.output['rgb'])[0,...,:3],dtype=np.uint8)


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
