"""Measured rigid-body geometry and robot state; object writes only during reset."""
from pathlib import Path
import json, subprocess
import numpy as np
import task
from scoring import TaskScorer, CRACKERS

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
        self.grasps['desiccant_packet']=np.array(self.layout['desiccant_packet']['grasp_centre_local'])
        self.grasps['serving_plate']=np.array(self.layout['serving_plate']['front_handle_grasp_local'])
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
    def body(self,name='desiccant_packet'):
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
        import cv2
        bodies={n:self.body(n) for n in self.vertices}
        valid=all(np.isfinite(b['pose']).all() and np.isfinite(b['velocity']).all() for b in bodies.values())
        if not valid:
            return dict(valid=False,eligible=dict.fromkeys(CRACKERS,False),packet_in_basket=False,bodies={})
        origin=np.array(self.specs['serving_basket'].position)
        tcps=[self.arm(s)[2] for s in ['left','right']]
        result={};local={};hulls={}
        for name,b in bodies.items():
            p=b['points']-origin;local[name]=p
            hulls[name]=cv2.convexHull(p[:,:2].astype(np.float32))
            distance=[]
            lo=self.vertices[name].min(0);hi=self.vertices[name].max(0)
            for tcp in tcps:
                t=(tcp-b['pose'][:3])@b['rotation']
                distance.append(float(np.linalg.norm(np.maximum(np.maximum(lo-t,t-hi),0))))
            result[name]=dict(pose=b['pose'].tolist(),velocity=b['velocity'].tolist(),low=b['low'].tolist(),high=b['high'].tolist(),tcp_clearance_m=min(distance))
        for name in CRACKERS:
            p=local[name];b=bodies[name]
            f=np.clip((p[:,2]-.008)/.048,0,1)
            inside=bool(np.all((p[:,0]/(.109+.020*f+.002))**2+(p[:,1]/(.064+.020*f+.002))**2<=1)
                and p[:,2].min()>=.008 and p[:,2].max()<=.063)
            stable=bool(np.linalg.norm(b['velocity'][:3])<=.020 and np.linalg.norm(b['velocity'][3:])<=.20)
            released=result[name]['tcp_clearance_m']>=.045
            result[name].update(inside=inside,stable=stable,released=released,supported=False)
        # A support graph rooted at the basket floor excludes food held on a tray
        # suspended over the basket. Support is inferred from surface gaps and
        # projected overlap, not a fitted object pose or an attachment.
        supported=set()
        for _ in CRACKERS:
            for name in CRACKERS:
                if name in supported or not result[name]['inside']:continue
                low=float(local[name][:,2].min())
                support=abs(low-.012)<=.004
                for other in supported:
                    gap=low-float(local[other][:,2].max())
                    if abs(gap)<=.004:
                        area=projected_overlap_m2(hulls[name],hulls[other])
                        support |= area>=.00002
                if support:supported.add(name)
        eligible={}
        for name in CRACKERS:
            result[name]['supported']=name in supported
            eligible[name]=bool(valid and all(result[name][k] for k in ['inside','stable','released','supported']))
        packet=local['desiccant_packet']
        normalized=cv2.convexHull((packet[:,:2]/[.135,.090]).astype(np.float32))
        overlaps=cv2.pointPolygonTest(normalized,(0.,0.),True)>=-1.
        packet_in=bool(overlaps and packet[:,2].min()<=.064 and packet[:,2].max()>=.008)
        self._controls_cache=dict(valid=bool(valid),eligible=eligible,packet_in_basket=packet_in,bodies=result)
        return self._controls_cache
    def score(self):
        state=self.controls()
        return self.scorer.update(step=int(self.base.common_step_counter),eligible=state['eligible'],packet_in_basket=state['packet_in_basket'],valid=state['valid'])
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
