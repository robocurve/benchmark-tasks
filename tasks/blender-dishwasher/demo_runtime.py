"""Measured rigid-body geometry and robot state; object writes only during reset."""
from pathlib import Path
import json, subprocess
import numpy as np
import task
from scoring import TaskScorer, PARTS

def numpy(value):
    value=value.torch if hasattr(value,'torch') else value
    return value.detach().cpu().numpy()

def projected_overlap_m2(first, second):
    import cv2
    # OpenCV's float32 clipping tolerance loses nearly collinear edges at metre
    # scale for these 42 mm meshes. Millimetres preserve the same geometry.
    area, _ = cv2.intersectConvexConvex(np.asarray(first,np.float32)*1000.,
                                     np.asarray(second,np.float32)*1000.)
    return max(0.,float(area))*1e-6

class StateReader:
    def __init__(self,env,specs):
        from yam_arena.constants import ARM_JOINTS,FINGER_JOINTS,FINGER_OPEN
        self.env,self.base=env,env.unwrapped
        self.specs={s.name:s for s in specs}
        self.layout=json.loads((task.ROOT/'layout.json').read_text())
        self.scorer=TaskScorer()
        self._cache_step=None;self._body_cache={};self._arm_cache={};self._controls_cache=None
        self.vertices={s.name:task.collision_vertices(s) for s in specs if s.movable}
        self.grasps={n:(v.min(0)+v.max(0))/2 for n,v in self.vertices.items()}
        self.grasps['blender_jug']=np.array(self.layout['blender_jug']['handle_grasp_local'])
        self.grasps['blender_lid']=np.array(self.layout['blender_lid']['grasp_local'])
        self.arm_ids,self.finger_ids={},{};self.finger_open=FINGER_OPEN
        for side in ['left','right']:
            robot=self.base.scene[side+'_robot']
            self.arm_ids[side]=robot.find_joints(ARM_JOINTS,preserve_order=True)[0]
            self.finger_ids[side]=robot.find_joints(FINGER_JOINTS,preserve_order=True)[0]
    def reset(self):
        import torch
        for name in self.vertices:
            obj=self.base.scene[name];spec=self.specs[name]
            obj.write_root_pose_to_sim(torch.tensor([[*spec.position,*spec.rotation_xyzw]],device=self.base.device,dtype=torch.float32))
            obj.write_root_velocity_to_sim(torch.zeros((1,6),device=self.base.device))
        self.scorer.reset()
        self._cache_step=None;self._body_cache={};self._arm_cache={};self._controls_cache=None
    def _refresh(self):
        step=int(self.base.common_step_counter)
        if step!=self._cache_step:
            self._cache_step=step;self._body_cache={};self._arm_cache={};self._controls_cache=None
    def body(self,name='blender_jug'):
        self._refresh()
        if name in self._body_cache:return self._body_cache[name]
        from yam_arena.urdf import quat_xyzw_to_matrix
        obj=self.base.scene[name]
        pose=numpy(obj.data.root_link_pose_w)[0].astype(float)
        rotation=quat_xyzw_to_matrix(pose[3:])
        points=self.vertices[name]@rotation.T+pose[:3]
        result=dict(pose=pose,rotation=rotation,points=points,
            velocity=numpy(obj.data.root_com_vel_w)[0].astype(float),low=points.min(0),high=points.max(0),
            grasp=pose[:3]+rotation@self.grasps[name])
        self._body_cache[name]=result
        return result
    def arm(self,side):
        self._refresh()
        if side in self._arm_cache:return self._arm_cache[side]
        from yam_arena import ik,station
        q=numpy(self.base.scene[side+'_robot'].data.joint_pos)[0]
        angles=q[self.arm_ids[side]].astype(float)
        grip=float(np.clip(q[self.finger_ids[side]].mean()/self.finger_open,0,1))
        position,rotation=ik.tcp_pose(angles,station.BASES[side])
        self._arm_cache[side]=(angles,grip,position,rotation)
        return self._arm_cache[side]
    def controls(self):
        self._refresh()
        if self._controls_cache is not None:return self._controls_cache
        bodies={n:self.body(n) for n in self.vertices}
        valid=all(np.isfinite(b['pose']).all() and np.isfinite(b['velocity']).all() for b in bodies.values())
        origin=np.array(self.specs['dishwasher_rack'].position)
        tcps=[self.arm(s)[2] for s in ['left','right']]
        result={};eligible={}
        for name,b in bodies.items():
            p=b['points']-origin
            distances=[];lo=self.vertices[name].min(0);hi=self.vertices[name].max(0)
            for tcp in tcps:
                t=(tcp-b['pose'][:3])@b['rotation']
                distances.append(float(np.linalg.norm(np.maximum(np.maximum(lo-t,t-hi),0))))
            inside=bool(np.all(np.abs(p[:,:2])<=[.141,.157]))
            supported=bool(abs(p[:,2].min()-.0083)<=.004)
            stable=bool(np.linalg.norm(b['velocity'][:3])<=.020 and np.linalg.norm(b['velocity'][3:])<=.20)
            released=min(distances)>=.045
            result[name]=dict(pose=b['pose'].tolist(),velocity=b['velocity'].tolist(),low=b['low'].tolist(),high=b['high'].tolist(),
                tcp_clearance_m=min(distances),inside=inside,supported=supported,stable=stable,released=released)
            if name in PARTS:eligible[name]=bool(valid and inside and supported and stable and released)
        # Any overlap of the motor's main body with the loading volume latches failure.
        # Only below the rim: passing above the rack is not itself a loading action.
        motor=bodies['motor_base']['points']-origin
        rack_entry=bool(np.all(motor[:,:2].min(0)<[.143,.159]) and np.all(motor[:,:2].max(0)>[-.143,-.159])
                        and motor[:,2].min()<.058 and motor[:,2].max()>.003)
        cabinet=np.array(self.specs['dishwasher'].position)
        m=bodies['motor_base']['points']-cabinet
        cavity_entry=bool(np.all(m.min(0)<[.112,.164,.285]) and np.all(m.max(0)>[-.130,-.164,.033]))
        self._controls_cache=dict(valid=bool(valid),eligible=eligible,motor_in_dishwasher=rack_entry or cavity_entry,bodies=result)
        return self._controls_cache
    def score(self):
        state=self.controls()
        return self.scorer.update(step=int(self.base.common_step_counter),eligible=state['eligible'],motor_in_dishwasher=state['motor_in_dishwasher'],valid=state['valid'])
    def rgb(self,name):
        return np.ascontiguousarray(numpy(self.base.scene[name].data.output['rgb'])[0,...,:3],dtype=np.uint8)

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
