"""Batched observation/scoring and native video recording for this scene."""
import json
from pathlib import Path
import subprocess
import numpy as np
import task
from contacts import ContactMonitor
from scoring import IceScorer


def numpy(v):
    v=v.torch if hasattr(v,'torch') else v
    return v.detach().cpu().numpy()


class Reader:
    def __init__(self,env):
        from yam_arena.constants import ARM_JOINTS,FINGER_JOINTS,FINGER_OPEN
        self.env=env;self.base=env.unwrapped;self.layout=json.loads((task.ROOT/'layout.json').read_text())
        self.collection=self.base.scene['ice'];self.names=list(self.collection.body_names)
        self.arm_ids={};self.finger_ids={};self.finger_open=FINGER_OPEN
        for side in ('left','right'):
            robot=self.base.scene[side+'_robot']
            self.arm_ids[side]=robot.find_joints(ARM_JOINTS,preserve_order=True)[0]
            self.finger_ids[side]=robot.find_joints(FINGER_JOINTS,preserve_order=True)[0]
        self.contacts=ContactMonitor(env,self.layout['objects'])
        self.scorer=IceScorer(self.layout);self.reset_history()

    def reset_history(self):
        self.contacts.reset();self.scorer.reset();self.last_score={'score':0.,'success':False}

    def snapshot(self):
        p=numpy(self.collection.data.body_link_pose_w)[0]
        v=numpy(self.collection.data.body_com_vel_w)[0]
        poses={n:p[i].astype(float) for i,n in enumerate(self.names)}
        velocities={n:v[i].astype(float) for i,n in enumerate(self.names)}
        for n in ('Scoop','CleanLid'):
            data=self.base.scene[n].data
            poses[n]=numpy(data.root_link_pose_w)[0].astype(float)
            velocities[n]=numpy(data.root_com_vel_w)[0].astype(float)
        return poses,velocities

    def arm(self,side):
        from yam_arena import ik,station
        q=numpy(self.base.scene[side+'_robot'].data.joint_pos)[0]
        angles=q[self.arm_ids[side]].astype(float)
        grip=float(np.clip(q[self.finger_ids[side]].mean()/self.finger_open,0,1))
        p,r=ik.tcp_pose(angles,station.BASES[side]);return angles,grip,p,r

    def score(self):
        poses,vel=self.snapshot()
        self.last_score=self.scorer.update(poses,vel,self.contacts.drain(),{s:self.arm(s)[2] for s in ('left','right')},
            healthy=self.contacts.contact_count>0 and self.contacts.error is None)
        return self.last_score

    def measure_reset(self,authored):
        settled,_=self.snapshot()
        error=max(float(np.linalg.norm(p[:3]-np.array(self.layout['objects'][n]['position']))) for n,p in authored.items())
        counts={source:int(sum(abs(settled[n][0]-self.layout['objects'][bin_name]['position'][0])<.120 and
            abs(settled[n][1]-self.layout['objects'][bin_name]['position'][1])<.134 and
            self.layout['objects'][bin_name]['position'][2]<settled[n][2]<self.layout['objects'][bin_name]['position'][2]+.125
            for n,row in self.layout['objects'].items() if row.get('ice_source')==source))
            for source,bin_name in [('clean','CleanBin'),('fish','FishBin')]}
        frames={n:self.rgb(n) for n in task.POLICY_CAMERAS}
        cameras=all(f.shape==(360,640,3) and f.std()>5 for f in frames.values())
        distinct=all(np.abs(frames[a].astype(float)-frames[b].astype(float)).mean()>5 for a,b in [('top_cam','left_cam'),('top_cam','right_cam'),('left_cam','right_cam')])
        contacts=self.contacts.report()
        passed=error<1e-5 and counts==self.layout['ice_counts'] and cameras and distinct and contacts['contact_count']>0 and contacts['error'] is None
        return {'passed':passed,'authored_position_error_m':error,'source_counts':counts,'valid_cameras':bool(cameras),
                'distinct_cameras':bool(distinct),'contact_reports':contacts}

    def rgb(self,name):
        return np.ascontiguousarray(numpy(self.base.scene[name].data.output['rgb'])[0,...,:3],dtype=np.uint8)


class VideoRecorder:
    def __init__(self,reader,directory):
        self.reader=reader;self.processes={};self.logs=[];self.frames=0
        directory=Path(directory)
        for name in task.POLICY_CAMERAS:
            h,w=reader.rgb(name).shape[:2];log=(directory/(name+'_encode.log')).open('w');self.logs.append(log)
            self.processes[name]=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pixel_format','rgb24',
                '-video_size',f'{w}x{h}','-framerate','20','-i','pipe:0','-an','-c:v','libx264','-preset','veryfast','-crf','22',
                '-pix_fmt','yuv420p','-threads','2','-movflags','+faststart',str(directory/(name+'.mp4'))],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=log)

    def capture(self):
        for name,p in self.processes.items():p.stdin.write(self.reader.rgb(name).tobytes())
        self.frames+=1

    def close(self):
        for p in self.processes.values():p.stdin.close()
        failed=[n for n,p in self.processes.items() if p.wait(timeout=30)]
        for log in self.logs:log.close()
        if failed:raise RuntimeError('Video encoding failed: '+str(failed))
