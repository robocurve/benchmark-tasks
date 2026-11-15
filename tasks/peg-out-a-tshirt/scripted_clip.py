"""State-assisted peg alignment through ordinary bounded robot commands.

Free-jaw observations center their mouth on the line before handle release.
This controller writes no body poses, cloth nodes, forces or physics settings.
It is a disclosed scripted demo, never a Dream policy observation or action.
"""
import numpy as np
from interpret_motion import rotation_xyzw


def line_capture(body_poses, line_x=.52, line_z=1.09, radius=.0015):
    """Check whether an open peg physically encompasses a line cross-section.

    This permits an ordinary closure attempt. It does not establish fabric
    capture, retained contact, a successful clip or any task score.
    """
    poses = np.asarray(body_poses, float)
    if poses.shape != (2, 7) or not np.isfinite(poses).all():
        raise ValueError('Invalid free peg observations')
    section_y = float(poses[:, 1].mean())
    point = np.array((line_x, section_y, line_z))
    valid = abs(section_y) < .235
    sections = []
    for index, pose in enumerate(poses):
        rotation = rotation_xyzw(pose[3:])
        local = (point - pose[:3]) @ rotation
        axis = rotation.T @ np.array((0., 1., 0.))
        extent = radius * np.sqrt(np.maximum(0., 1. - axis**2))
        inner_face = -.0005 if index == 0 else .0005
        clearance = (local[2] - extent[2] - inner_face) if index == 0 else (inner_face - local[2] - extent[2])
        inside_length = .003 <= local[0] <= .0345
        inside_width = abs(local[1]) + extent[1] <= .005
        valid = valid and inside_length and inside_width and clearance >= 0.
        sections.append(dict(local_line_axis_m=local.tolist(),
            cross_section_radius_m=extent.tolist(), inner_face_clearance_m=float(clearance),
            axis_within_jaw_length=bool(inside_length), within_jaw_width=bool(inside_width)))
    return dict(can_attempt_closure=bool(valid), line_cross_section_y_m=section_y, jaws=sections)


class PegMouthServo:
    def __init__(self,start=820,end=880,target=(.52,.115,1.09),side='right',functional_jaws=False,align_y=False):
        if side not in ('left','right'):raise ValueError('Unknown robot arm')
        self.functional_jaws=functional_jaws
        self.align_y=align_y
        self.side=side;self.indices=slice(0,6) if side=='left' else slice(7,13)
        self.start=start;self.end=end;self.target=np.asarray(target,float)
        self.offset=np.zeros(6);self.records=[];self.last_error=None
        self.desired_tcp=None;self.desired_q=None;self.closure_geometry=None

    def action(self,step,reference,observed,body_poses):
        from yam_arena import ik,station
        a=np.asarray(reference,float).copy();observed=np.asarray(observed,float).reshape(-1)
        if self.start<step<=self.end:
            poses=np.asarray(body_poses,float)
            if poses.shape!=(2,7) or not np.isfinite(poses).all():raise ValueError('Invalid free peg observations')
            inner=[p[:3]+rotation_xyzw(p[3:])@np.array((.030,0.,sign*.0005)) for p,sign in zip(poses,(-1.,1.))]
            mouth=(np.mean(inner,0)
                   if self.functional_jaws else np.mean(inner,0))
            error=self.target-mouth
            # Any valid position along the line is useful; align its radial x/z.
            if not self.align_y:error[1]=0.
            distance=float(np.linalg.norm(error))
            gap=float(np.linalg.norm(inner[1]-inner[0]))
            if distance>.065 or gap<.006 or gap>(.016 if self.functional_jaws else .012):raise RuntimeError('Peg alignment lost its held open-jaw geometry')
            if self.desired_tcp is None:
                self.desired_tcp,_=ik.tcp_pose(a[self.indices],station.BASES[self.side])
                self.desired_q=a[self.indices].copy()
            _,nominal_rotation=ik.tcp_pose(a[self.indices],station.BASES[self.side])
            increment=error*min(.15,.00075/max(distance,1e-12))
            self.desired_tcp=self.desired_tcp+increment
            q,pe,re=ik.solve(self.desired_tcp,ik.matrix_to_quat_xyzw(nominal_rotation),self.desired_q,base_pos=station.BASES[self.side],iters=100)
            if pe>.002 or re>.025:raise RuntimeError('Peg-mouth servo could not produce a normal reachable action')
            self.desired_q=q.copy()
            a[self.indices]=q;self.offset=q-reference[self.indices];self.last_error=distance
            self.records.append(dict(step=step,mouth_m=mouth.tolist(),error_m=error.tolist(),gap_m=gap,commanded_tcp_increment_m=increment.tolist(),desired_tcp_m=self.desired_tcp.tolist()))
        elif self.end<step<=self.end+140:
            if step==self.end+1:
                self.closure_geometry=line_capture(body_poses,line_x=self.target[0],line_z=self.target[2])
                self.records.append(dict(step=step,closure_geometry=self.closure_geometry))
                if not self.closure_geometry['can_attempt_closure']:
                    raise RuntimeError('Line is outside the open peg jaw region; closure withheld')
            if self.closure_geometry is None:
                raise RuntimeError('Missing observed peg closure geometry')
            blend=1. if step<=self.end+60 else max(0.,1-(step-(self.end+60))/80)
            a[self.indices]+=blend*self.offset
        return a
