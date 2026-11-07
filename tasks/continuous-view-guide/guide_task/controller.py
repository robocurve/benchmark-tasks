"""Known-layout scripted route with RGB-gated progress; no simulator truth inputs."""
import math
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY,FINGER_OPEN,TCP_OFFSET
from yam_arena.demo import ArmPlan,Waypoint,grip_rotation
from yam_arena.urdf import quat_xyzw_to_matrix,axis_angle_matrix
from . import layout as L

MODE='setup'
LOG=[]
RETREAT=None
def inspect_pose(angle,distance=.17,elevation=.025,roll=0.,seed=None):
    aim=L.point(angle)
    pos=aim+np.array([-math.sin(angle)*distance,math.cos(angle)*distance,elevation])
    back=pos-aim;back/=np.linalg.norm(back)
    right=np.cross([0.,0.,1.],back);right/=np.linalg.norm(right)
    up=np.cross(back,right)
    cam_r=np.column_stack((right,up,back))@axis_angle_matrix(np.array([0.,0.,1.]),roll)
    cam=station.rig().wrist_cams['left']
    grip_r=cam_r@quat_xyzw_to_matrix(np.asarray(cam.rot_xyzw)).T
    grip_pos=pos-grip_r@np.asarray(cam.pos)
    tcp=grip_pos+grip_r@np.asarray(TCP_OFFSET)
    q,pe,re=ik.solve(tcp,ik.matrix_to_quat_xyzw(grip_r),q_init=READY if seed is None else seed,
                     base_pos=station.BASES['left'],iters=500)
    return dict(q=q.tolist(),pe=pe,re=re,camera=pos.tolist(),aim=aim.tolist(),roll=roll)

def waypoints():
    h=L.handle(L.START);grasp_z=L.HEIGHT+.012;z=grasp_z+.004;hover=L.HEIGHT+.070
    w=[Waypoint(3.,tuple([h[0],h[1],hover]),FINGER_OPEN,'approach',L.START),
       Waypoint(2.,tuple([h[0],h[1],grasp_z]),FINGER_OPEN,'descend to handle',L.START),
       Waypoint(1.5,tuple([h[0],h[1],grasp_z]),-.010,'grasp handle',L.START),
       Waypoint(1.,tuple([h[0],h[1],z]),-.010,'clear the start support',L.START),
       Waypoint(3.,tuple([h[0],h[1],z]),-.010,'observe start',L.START)]
    if MODE=='lift_probe':
        p=tuple([h[0],h[1],grasp_z+.080])
        return w[:3]+[Waypoint(3.,p,-.010,'attempt direct lift',L.START),
                      Waypoint(2.,p,-.010,'hold lift attempt',L.START)]
    for a in np.linspace(L.START,L.FINISH,45)[1:]:
        p=L.handle(a,z)
        if MODE=='negative' and a>math.radians(25):p[:2]-=.025*np.array([math.cos(a),math.sin(a)])
        w.append(Waypoint(.5,tuple(p),-.010,'follow curved guide',a))
        if abs(a-math.radians(45))<.001:w.append(Waypoint(8.,tuple(p),-.010,'pause and reacquire view',a))
    p=L.handle(L.FINISH,grasp_z)
    w += [Waypoint(2.,tuple(p),-.010,'settle at support',L.FINISH),
          Waypoint(1.,tuple(p),FINGER_OPEN,'release',L.FINISH),
          Waypoint(3.,tuple(L.handle(L.FINISH,hover)),FINGER_OPEN,'withdraw',L.FINISH),
          Waypoint(2.,tuple(L.handle(L.FINISH,hover)),FINGER_OPEN,'settle',L.FINISH)]
    return w

def left_action(plan,poses,t):
    # Known-geometry viewing plan, not image feedback.
    target=plan.target(t)[0]
    angle=math.atan2(target[1]-L.CENTER[1],target[0]-L.CENTER[0])
    fraction=float(np.clip((angle-L.START)/(L.FINISH-L.START),0,1)) if t>=10.5 else 0.
    if MODE=='fixed_view':fraction=0.
    def sample(group,angle):
        aa=np.array([p['angle'] for p in group]);angle=np.clip(angle,aa[0],aa[-1])
        return np.array([np.interp(angle,aa,[p['q'][j] for p in group]) for j in range(6)])
    switch_start=21.5
    group=poses[0] if t<switch_start+4 else poses[1]
    q=sample(group,angle)
    if MODE=='fixed_view':q=np.asarray(poses[0][0]['q'])
    else:
        if switch_start<=t<switch_start+4:
            u=t-switch_start
            if u<2:
                s=ik.smoothstep(u/2);q=(1-s)*q+s*np.asarray(RETREAT['q'])
            else:
                s=ik.smoothstep((u-2)/2);q=(1-s)*np.asarray(RETREAT['q'])+s*np.asarray(READY)
        elif switch_start+4<=t<switch_start+7:
            s=ik.smoothstep((t-switch_start-4)/3);q=(1-s)*np.asarray(READY)+s*q
    approach=ik.smoothstep((t-5)/3.)
    q=(1-approach)*np.asarray(READY)+approach*q
    # Keep the final view after release; do not sweep back through the workpiece.
    if MODE in ('right_only','lift_probe'):q=np.asarray(READY)
    return q,fraction

def install(output):
    global RETREAT
    from yam_arena.scripted import YamScriptedPolicy
    from inspect_robots.types import Action,ActionChunk
    from inspect_robots_yam.packing import pack
    import json
    RETREAT=inspect_pose(math.radians(45),distance=.30,roll=math.radians(80))
    if RETREAT['pe']>.008 or RETREAT['re']>.05:raise RuntimeError(f'Retreat IK: {RETREAT}')
    output.joinpath('retreat_pose.json').write_text(json.dumps(RETREAT,indent=2))
    poses=[]
    for start,end,roll in [(L.START,math.radians(45),math.pi/2),(math.radians(45),L.FINISH,-math.pi/2)]:
        group=[]
        for a in np.linspace(start,end,12):
            best=inspect_pose(a,distance=.24 if roll>0 else .17+.04*float(np.clip((a-math.radians(85))/math.radians(15),0,1)),roll=(math.radians(110)-math.radians(20)*(a-L.START)/(math.radians(45)-L.START)) if roll>0 else roll)
            if best['pe']>.008 or best['re']>.05:raise RuntimeError(f'Inspection IK failed: {a}, {best}')
            group.append(dict(angle=float(a),**best))
        poses.append(group)
    output.joinpath('inspection_poses.json').write_text(json.dumps(poses,indent=2))
    original=YamScriptedPolicy.reset
    def reset(self,scene):
        original(self,scene);LOG.clear()
        from .vision import ViewGate
        self._view_gate=ViewGate();self._plan_time=0.
        self._guide_plan=ArmPlan(waypoints(),station.BASES['right'])
        self._end=self._guide_plan.duration
    def act(self,observation):
        wall_t=float(observation.state_time)+.05
        t=wall_t
        if MODE=='setup':
            return ActionChunk(actions=[Action(data=pack(np.r_[READY,1.],np.r_[READY,1.]))],control_hz=20)
        measurement=None;advance=True
        if MODE=='demo':
            t=self._plan_time
            # Observe before and during both moving segments and before release.
            # The8-second observation-arm transit holds the collar stationary.
            required=(10.5<=t<21.5 or 29.5<=t<42.5)
            advance,measurement=self._view_gate.update(observation.images.get('left_cam'),required)
            if advance:self._plan_time=round(t+.05,8)
            t=self._plan_time
        right,label=self._guide_plan.action7(t)
        q,fraction=left_action(self._guide_plan,poses,t)
        LOG.append(dict(step=len(LOG),time=wall_t,plan_time=t,label=label,view_fraction=fraction,
                        rgb_view=measurement,advanced=advance,
                        worst_ik_mm=self._guide_plan.worst_ik_mm,
                        worst_ik_deg=self._guide_plan.worst_ik_deg))
        return ActionChunk(actions=[Action(data=pack(np.r_[q,1.],right),
              meta={'request_stop':True} if t>self._end+2 else {})],control_hz=20)
    YamScriptedPolicy.reset=reset;YamScriptedPolicy.act=act
    return poses
