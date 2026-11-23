"""Edamame shelling with physical beans, two shell halves and waste sorting.

The shell opens by cohesive separation. The native YAM expert handles
the three beans and disposes of the shell by physical contact.
"""
from __future__ import annotations
import math
import os
from dataclasses import dataclass,field
import numpy as np
from edamame_geometry import (TABLE,POD_POS,PLATE_POS,BOWL_POS,BOWL_RADIUS,BIN_POS,
    BEANS,BEAN_RAD,LATCH,Y_SCALE,Z_SCALE,HALF,bow,radius,generate)
from plan import Track,OPEN,PINCH,waypoints,check_tracks,TIP
from runtime import scalar_array,tick,register,install_writer,wrench
from touch import contact_details, reset as reset_contacts
from yam_arena.constants import TCP_OFFSET
from yam_arena.station import TABLE_CENTER, TABLE_SIZE
from yam_arena.urdf import quat_xyzw_to_matrix

SCENE='station'
SCORE_OVER='end'
SUCCESS_HOLD_S=1.0
MAX_STEPS=2000
INSTRUCTION=('Open the edamame pod, put all three beans in the serving bowl, '
             'and discard the empty pod in the food-scraps bin.')
FORCE_PROBE=os.getenv('EDAMAME_FORCE_PROBE','0')=='1'
FORCE_MAX=float(os.getenv('EDAMAME_FORCE_N','.20'))
IDLE=os.getenv('EDAMAME_DEMO_IDLE','0')=='1'
GRASP_PROBE = os.getenv('EDAMAME_GRASP_PROBE','0') == '1'
LIFT_FIRST = os.getenv('EDAMAME_LIFT_FIRST','0') == '1'
END_GRASPS = os.getenv('EDAMAME_END_GRASPS','0') == '1'
POUR_AFTER_LIFT = os.getenv('EDAMAME_POUR_AFTER_LIFT','0') == '1'
OFFSET_SEAM = os.getenv('EDAMAME_OFFSET_SEAM','0') == '1'
ANGLED_APPROACH = os.getenv('EDAMAME_ANGLED_APPROACH','0') == '1'
MEASURE_ARM_CONTACT = os.getenv('EDAMAME_MEASURE_ARM_CONTACT','0') == '1'
STAGE_SHELL = os.getenv('EDAMAME_STAGE_SHELL','0') == '1'
COMPLIANT_SHELL = os.getenv('EDAMAME_COMPLIANT_SHELL', '0') == '1'
JOINED_SHELL = os.getenv('EDAMAME_JOINED_SHELL', '1') == '1'

@dataclass
class Trial:
    t:float=0.
    hinge_deg:float=0.
    initial_ok:bool|None=None
    opened:bool=False
    broken_seams:int=0
    contained_count:int=0
    handled_upper:bool=False
    handled_lower:bool=False
    beans_in_bowl:int=0
    bean_positions:list=field(default_factory=list)
    shell_in_bin:bool=False
    invalid:bool=False
    settled_s:float=0.
    score_value:float=0.
    reason:str='waiting for initial physics'
    probe_force:float=0.
    tools:dict=field(default_factory=dict)
    arm_contact_samples:int=0

_trial=Trial()
_monitor=None

def score(objects):return _trial.score_value

def success(objects):return _trial.score_value>=.999

def telemetry():
    t=_trial
    return {'t':round(t.t,2),'hinge_deg':round(t.hinge_deg,2),
        'initial_ok':t.initial_ok,'opened':t.opened,
        'broken_seams':t.broken_seams,'contained_count':t.contained_count,'handled_upper':t.handled_upper,'handled_lower':t.handled_lower,
        'beans_in_bowl':t.beans_in_bowl,
        'bean_xyz':[[round(x,3) for x in p] for p in t.bean_positions],
        'shell_in_bin':t.shell_in_bin,'invalid':t.invalid,
        'settled_s':round(t.settled_s,2),'latch_requested':LATCH,
        'force_probe':FORCE_PROBE,'probe_force_n':round(t.probe_force,2),
        'reason':t.reason,'tool_centers_m':t.tools,
        'arm_contact_samples':t.arm_contact_samples if MEASURE_ARM_CONTACT else None}

def _world_body(asset,link=None):
    if link is None:
        return np.asarray(scalar_array(asset.data.root_link_pose_w)[0][:3],float)
    p=scalar_array(asset.data.body_pos_w)[0]
    return np.asarray(p[asset.body_names.index(link)],float)

def _inside_bowl(position):
    p=np.asarray(position,float)
    d=np.linalg.norm(p[:2]-np.asarray(BOWL_POS[:2],float))
    # Count a bean only when its full width is inside the bowl's inner wall.
    return bool(d < BOWL_RADIUS - .004 - .005 and BOWL_POS[2]-.003<p[2]<BOWL_POS[2]+.034)

def _inside_bin(position):
    p=np.asarray(position,float)
    return bool(np.max(np.abs(p[:2]-np.asarray(BIN_POS[:2],float)))<.070
                and BIN_POS[2]-.002<p[2]<BIN_POS[2]+.128)

def _arms_clear(env,places):
    try:
        from scenekit import robot_links
        arms=robot_links(env)
        pts=np.array([p for links in arms.values() for p in links.values()],float)
        targets=np.array(places,float)
        return float(np.linalg.norm(pts[:,None,:]-targets[None,:,:],axis=2).min())>.062
    except Exception:
        return False

class Monitor:
    def __init__(self,env):
        self.scene=env.unwrapped.scene
        self.pod=self.scene['pod']
        self.beans=[self.scene[f'bean_{i}'] for i in range(3)]
        if 'SeamHinge' not in self.pod.joint_names:
            raise RuntimeError(f'SeamHinge not found; joints={self.pod.joint_names}')
        if 'pod_upper' not in self.pod.body_names:
            raise RuntimeError(f'pod_upper not found; links={self.pod.body_names}')
        self.hinge_id=self.pod.joint_names.index('SeamHinge')
        self.start=None
        self._prior_objects=None
        self.contacts = {}
        self.contact_forces = {}
        import torch
        device = env.unwrapped.device
        names = ("pod_lower","pod_upper_left","pod_upper","pod_upper_right")
        self.shell_indices = torch.tensor([self.pod.body_names.index(n) for n in names],device=device)
        self.seam_points = torch.tensor([(-.021,.0075*Y_SCALE,0.), (0.,.0105*Y_SCALE,0.), (.021,.0075*Y_SCALE,0.)],
                                        device=device)
        self.strain_limits = torch.tensor([.00115,.0015,.00115],device=device)
        self.seal = torch.full((3,), LATCH, dtype=torch.bool,device=device)
        self.dt = env.unwrapped.physics_dt
        self.contact_names = [f"pod/{name}" for name in names] + [f"bean_{i}" for i in range(3)]
        self.arm_contact_targets = (
            [f"right_robot/{name}" for name in ("gripper", "tip_left", "tip_right", "link5", "link4")]
            if MEASURE_ARM_CONTACT else []
        )
        install_writer(self.pod,self.apply_seam)
        self.reset()

    def reset(self):
        self.seal.fill_(LATCH)
        self._prior_objects = None

    def apply_seam(self):
        import torch
        pose = self.pod.data.body_link_pose_w.torch[0,self.shell_indices]
        velocity = self.pod.data.body_link_vel_w.torch[0,self.shell_indices]

        def offsets(quaternion):
            vec = quaternion[..., :3].expand_as(self.seam_points)
            scale = quaternion[..., 3:]
            twice = 2 * torch.linalg.cross(vec,self.seam_points,dim=-1)
            return self.seam_points + scale*twice + torch.linalg.cross(vec,twice,dim=-1)

        lower = offsets(pose[0,3:])
        upper = offsets(pose[1:,3:])
        lower_xyz, upper_xyz = pose[0,:3] + lower, pose[1:,:3] + upper
        lower_v = velocity[0,:3] + torch.linalg.cross(velocity[0,3:].expand_as(lower),lower,dim=-1)
        upper_v = velocity[1:,:3] + torch.linalg.cross(velocity[1:,3:],upper,dim=-1)
        delta = upper_xyz - lower_xyz
        ext = torch.linalg.vector_norm(delta,dim=-1)
        self.seal &= torch.isfinite(ext) & (ext < self.strain_limits)
        k, damping, mass = 150., .018, .00075
        gain = 1 + (self.dt*damping + self.dt**2*k) / mass
        force = (-k*delta-(damping+self.dt*k)*(upper_v-lower_v)) / gain
        self.seal &= torch.isfinite(force).all(dim=1)
        force = torch.where(self.seal[:,None],force,0.)
        result = torch.cat((-force.sum(dim=0,keepdim=True),force),dim=0)
        torque = torch.cat((-torch.linalg.cross(torch.nan_to_num(lower),force,dim=-1).sum(dim=0,keepdim=True),
                            torch.linalg.cross(torch.nan_to_num(upper),force,dim=-1)),dim=0)
        if FORCE_PROBE:
            result[2,2] += _trial.probe_force
        wrench(self.pod,result.unsqueeze(0),torque.unsqueeze(0),ids=self.shell_indices)

    def update(self,env,dt):
        t=_trial
        t.t+=dt
        q=float(scalar_array(self.pod.data.joint_pos)[0,self.hinge_id])
        # Numerical sign can differ depending on the joint axis. Debug print
        # the sign rather than silently requiring a particular direction.
        t.hinge_deg=math.degrees(q)
        root=_world_body(self.pod)
        upper=_world_body(self.pod,'pod_upper')
        beans=[_world_body(x) for x in self.beans]
        tips = [_world_body(self.pod,f"pod_upper_{side}") for side in ("left","right")]
        object_positions = np.vstack([root, upper, *tips, *beans])
        if not np.isfinite(object_positions).all() or not math.isfinite(q):
            t.invalid = True
            t.score_value = 0.
            t.reason = 'Non-finite pod or bean state.'
            return
        t.bean_positions=[p.tolist() for p in beans]
        outside_xy = (np.abs(object_positions[:, :2] - np.asarray(TABLE_CENTER[:2])) >
                      np.asarray(TABLE_SIZE[:2]) / 2)
        if outside_xy.any() or (object_positions[:, 2] < TABLE - .015).any():
            t.invalid = True
            t.score_value = 0.
            t.reason = 'A bean or shell section left the tabletop work area.'
            return
        lower_pose = scalar_array(self.pod.data.body_link_pose_w)[0,self.pod.body_names.index("pod_lower")]
        local_beans = (np.asarray(beans)-root) @ quat_xyzw_to_matrix(lower_pose[3:])
        contained = [
            abs(x) < HALF and ((y/Y_SCALE-bow(x))**2+(z/Z_SCALE)**2) < (radius(x)+.00065)**2
            for x,y,z in local_beans]
        t.contained_count = sum(contained)
        self.contact_forces = contact_details(env, self.contact_names + self.arm_contact_targets)
        contacts = {name: list(links) for name, links in self.contact_forces.items()}
        arm_contacts = [path for name in self.arm_contact_targets for path in contacts.pop(name)]
        self.contacts = contacts
        t.arm_contact_samples += int(any("/LeftArm/" in path for path in arm_contacts))
        for arm in ("left","right"):
            robot = self.scene[f"{arm}_robot"]
            pose = scalar_array(robot.data.body_link_pose_w)[0,robot.body_names.index("gripper")]
            tcp = pose[:3] + quat_xyzw_to_matrix(pose[3:]) @ TCP_OFFSET
            t.tools[arm] = [round(float(value),4) for value in tcp]
        t.broken_seams = 3 - int(scalar_array(self.seal).sum())
        t.handled_lower |= any("/tip_" in path for path in contacts["pod/pod_lower"])
        t.handled_upper |= any("/tip_" in path for key, paths in contacts.items()
                               if key.startswith("pod/pod_upper") for path in paths)
        if FORCE_PROBE:
            t.probe_force=(FORCE_MAX*min(1.,max(0.,(t.t-1.)/4.))
                           if 1.<t.t<8. else 0.)
        if .25<t.t<.8 and t.initial_ok is None:
            t.initial_ok=bool(abs(t.hinge_deg)<12. and t.broken_seams == 0 and
                t.contained_count == 3 and
                np.linalg.norm(root[:2]-np.asarray(POD_POS[:2]))<.024)
            if not t.initial_ok:
                t.invalid=True
                t.reason='pod not closed or bean(s) escaped before manipulation'
            else:
                t.reason='Closed pod contained all three beans after settling.'
        if t.broken_seams == 3 and t.hinge_deg > 35.:
            t.opened = bool(t.handled_lower and t.handled_upper and not FORCE_PROBE)
        if not FORCE_PROBE and t.broken_seams and not (t.handled_lower and t.handled_upper):
            t.invalid = True
            t.reason = 'Seam rupture without verified handling of both shell halves.'
        t.beans_in_bowl = sum(
            _inside_bowl(bean) for bean, in_pod in zip(beans, contained) if not in_pod)
        t.shell_in_bin = all(_inside_bin(point) for point in (root,upper,*tips))
        stable=(self._prior_objects is not None and
                np.linalg.norm(object_positions-self._prior_objects,axis=1).max()<.0015)
        self._prior_objects=object_positions.copy()
        # If the beans escape before the seam opens it is NOT shelling.
        if t.initial_ok is True and not t.opened and t.t>1.2:
            if t.contained_count < 3:
                t.invalid=True
                t.reason='bean escaped while pod remained closed'
        if t.initial_ok is not True or t.invalid or FORCE_PROBE or GRASP_PROBE or not LATCH:
            t.score_value=0.
            return
        # Scoring uses object state, not waypoint or scripted progress.
        score=0.
        if t.opened: score=.25
        if t.opened and t.beans_in_bowl>=1: score=.50
        if t.opened and t.beans_in_bowl>=2: score=.70
        if t.opened and t.beans_in_bowl==3: score=.85
        if t.opened and t.beans_in_bowl==3 and t.shell_in_bin:
            if stable and not any(contacts.values()) and _arms_clear(env,[root,upper,*tips,*beans]):
                t.settled_s+=dt
            else:t.settled_s=0.
            if t.settled_s>=1.:score=1.
        else:t.settled_s=0.
        t.score_value=score


def setup():
    from scenekit import UsdBody
    pod_asset = ('edamame_pod_probe' if FORCE_PROBE else
                 'edamame_pod_joined' if JOINED_SHELL else 'edamame_pod')
    pieces=[
        UsdBody('plate',usd='assets/edamame_plate.usda',pos=PLATE_POS,
                size=(.185,.185,.015),mass=None),
        UsdBody('bean_bowl',usd='assets/bean_bowl.usda',pos=BOWL_POS,
                size=(2*BOWL_RADIUS,2*BOWL_RADIUS,.037),mass=None),
        UsdBody('scraps_bin',usd='assets/shell_scraps_bin.usda',pos=BIN_POS,
                size=(.165,.165,.133),mass=None),
        UsdBody('pod',usd=f'assets/{pod_asset}.usda',pos=POD_POS,
                size=(.066,.017,.014),articulation=True,
                joint_pos=(('SeamHinge',0.),('TipFlex_left',0.),('TipFlex_right',0.)),
                joint_stiffness={"SeamHinge": .0003, "TipFlex_.*": .0012},
                joint_damping={"SeamHinge": .00010, "TipFlex_.*": .00004},
                joint_effort_limit=.0015,friction=(.84,.64),solver_iters=64,
                compliance=(6500., 1.2) if COMPLIANT_SHELL else None),
    ]
    for i,x in enumerate(BEANS):
        pieces.append(UsdBody(f'bean_{i}',usd='assets/edamame_bean.usda',
            pos=(POD_POS[0]+x,POD_POS[1],POD_POS[2]),
            size=(.010,.009,.009),mass=.00105,friction=(.55,.42),solver_iters=48))
    def reset(env):
        global _trial,_monitor
        _trial=Trial()
        reset_contacts()
        try:
            if _monitor is None or _monitor.scene is not env.unwrapped.scene:
                _monitor=Monitor(env)
            else:
                _monitor.reset()
        except Exception as err:
            _trial.invalid=True
            _trial.reason=f'{type(err).__name__}: {err}'
            print('[edamame] monitor init failed:',_trial.reason,flush=True)
    def step(env):
        if _monitor is not None:
            try:_monitor.update(env,tick(env))
            except Exception as err:
                _trial.invalid=True
                _trial.reason=f'UPDATE FAILED: {type(err).__name__}: {err}'
                print('[edamame]',_trial.reason,flush=True)
                raise
    return register(pieces,reset,step)


def tracks():
    """Approximate YAM manipulation attempt; it does NOT move beans in code.

    Needs GUI tuning on Lambda. Last pour uses wrist roll/pitch through
    edamame_scripted; plain yam_scripted won't honor roll/pitch.
    """
    l,r=Track(),Track()
    above=TABLE+.120
    root=np.asarray(POD_POS)
    left_grip=(root[0]-.020,root[1],TABLE+.019)
    right_grip=(root[0]+.018,root[1]+.005,TABLE+.019)
    narrow_open=-.011
    l.go((left_grip[0],left_grip[1],above),narrow_open,'above the stem end',2.5)
    l.wait_until(4.0)
    l.go(left_grip,narrow_open,'descend directly over the stem end',2.0,roll=0.)
    l.go(left_grip,0.,'stabilize the stem end',1.3,roll=0.)
    r.go((right_grip[0],right_grip[1],above),narrow_open,'above the opposite end',2.5)
    r.wait_until(7.3)
    r.go(right_grip,narrow_open,'align at the opening seam',2.0,roll=.30)
    r.go(right_grip,0.,'close at the upper shell',1.4,roll=.30)
    l.wait_until(r.duration+2.7)
    r.go((right_grip[0],right_grip[1]+.018,right_grip[2]+.035),0.,
         'pull the seam while the stem end is supported',2.7,roll=.30)
    # Carry both halves above bowl while maintaining the opened shell.
    l.go((BOWL_POS[0]-.018,BOWL_POS[1],TABLE+.120),PINCH,
         'carry pod above serving bowl',2.4,roll=.45)
    r.go((BOWL_POS[0]+.020,BOWL_POS[1]+.024,TABLE+.145),-.009,
         'keep hinge open over bowl',2.4,roll=-.32)
    # Use wrist tilt + small controlled motion to let beans fall naturally.
    l.go((BOWL_POS[0],BOWL_POS[1],TABLE+.124),PINCH,
         'tilt pod to let beans fall',1.8,roll=.80,pitch=.45)
    r.go((BOWL_POS[0]+.018,BOWL_POS[1]+.033,TABLE+.148),-.009,
         'hold lid while pouring',1.8,roll=-.45,pitch=.43)
    for s in (-.012,.012,-.012,.012):
        l.go((BOWL_POS[0]+s,BOWL_POS[1],TABLE+.118),PINCH,
             'gentle bean-release shake',.65,roll=.80,pitch=.40)
        r.go((BOWL_POS[0]+.018+s,BOWL_POS[1]+.033,TABLE+.146),-.009,
             'maintain opening during shake',.65,roll=-.45,pitch=.40)
    # Let right arm withdraw, then put the hinged empty pod in the scraps bin.
    r.go((BOWL_POS[0]+.018,BOWL_POS[1]+.033,TABLE+.148),OPEN,
         'release upper shell',.7)
    r.go((.30,-.275,above),OPEN,'withdraw right arm',1.6)
    l.wait_until(r.duration)
    l.go((BIN_POS[0],BIN_POS[1],TABLE+.140),PINCH,
         'carry emptied pod to bin',2.6,roll=.25)
    l.go((BIN_POS[0],BIN_POS[1],BIN_POS[2]+.070),PINCH,
         'lower pod into bin',1.4)
    l.go((BIN_POS[0],BIN_POS[1],BIN_POS[2]+.070),OPEN,
         'release empty shell',.8)
    l.go((.23,.18,above),OPEN,'clear both containers',1.6)
    r.wait_until(l.duration+2.0)
    l.wait_until(r.duration)
    return {'left':l,'right':r}


def demo():
    if GRASP_PROBE:
        left, right = Track(), Track()
        x, y = POD_POS[:2]
        left.go((.23,.22,TABLE+.14),OPEN,'park the other arm',8.)
        right.go((x,y,TABLE+.12),-.014,'approach the pod center from above',2.5)
        right.wait_until(3.8)
        right.go((x,y,TABLE+.016),-.014,'descend with vertical jaws',2.2)
        right.go((x,y,TABLE+.016),0.,'grasp the closed pod',1.2)
        right.go((x,y,TABLE+.065),0.,'check whether the pod lifts',1.5)
        return waypoints(left,right)
    if LIFT_FIRST:
        left, right = Track(), Track()
        pod_x, y = POD_POS[:2]
        x = pod_x + (.021 if END_GRASPS else 0.)
        stem_x = pod_x - (.010 if END_GRASPS else .024)
        seam_y = y + (.014 if END_GRASPS else .005)
        seam_z = TABLE + (.0545 if END_GRASPS else .064)
        right_pitch = .25 if END_GRASPS else 0.
        left_pitch = -.36 if END_GRASPS else 0.
        left.go((.23,.22,TABLE+.14),OPEN,'park while the right hand picks up the pod',8.7)
        right.go((x,y,TABLE+.12),-.014,'approach the pod center from above',2.5)
        right.wait_until(3.8)
        right.go((x,y,TABLE+.016),-.014,'descend over the support end',2.2,pitch=right_pitch)
        right.go((x,y,TABLE+.016),0.,'grasp the closed pod',1.2,pitch=right_pitch)
        right.go((x,y,TABLE+.065),0.,'lift the pod above plate',1.5,pitch=right_pitch)
        if POUR_AFTER_LIFT:
            dx, dy, dz = BOWL_POS[0]-pod_x-.010, BOWL_POS[1]-y, .027
            shifted_x, shifted_y = x+dx, y+dy
            right_pitch = -.55
            left_pitch = -.55
            right.go((x,y,TABLE+.065+dz),0.,'raise and tilt the pod above both rims',1.5,pitch=right_pitch)
            right.go((shifted_x,shifted_y,TABLE+.065+dz),0.,
                     'carry the closed pod above the serving bowl',2.6,pitch=right_pitch)
            right.wait_until(right.duration+1.0)
            left.wait_until(right.duration)
            stem_x += dx + .010
            seam_y += dy
            seam_z += dz - .026
            if OFFSET_SEAM:
                # The suspended pod rides toward the support arm during
                # the approach. Use the seam location seen in the trace.
                seam_y -= .053
                seam_z -= .006
                left_pitch = -.68
        entry_roll = -.48 if OFFSET_SEAM else -.25
        approach_y = seam_y
        if ANGLED_APPROACH:
            # Keep the wrists separated along the pod axis and approach
            # from the left arm's side with the final wrist angle already set.
            stem_x -= .019
            seam_y -= .012
            seam_z += .008
            if STAGE_SHELL:
                stem_x -= .010
                seam_y -= .017
            approach_y = seam_y + .052
            left_pitch = -.64
            entry_roll = -.65
        left.go((stem_x,approach_y,TABLE+.13),-.010,'approach the stem-side shell',1.8,
                roll=entry_roll if ANGLED_APPROACH else 0.,pitch=left_pitch if ANGLED_APPROACH else 0.)
        left.go((stem_x,seam_y,seam_z),-.010,'align with the raised pod seam',1.2,roll=entry_roll,pitch=left_pitch)
        left.go((stem_x,seam_y,seam_z),0.,'grip the shell edge',1.0,roll=entry_roll,pitch=left_pitch)
        left.go((stem_x-.007,seam_y+.019,seam_z+.022),0.,
                'pull against the supporting gripper',1.8,roll=-.48,pitch=left_pitch)
        if END_GRASPS:
            regrasp_roll = entry_roll if ANGLED_APPROACH else -.28
            left.go((stem_x-.007,seam_y+.019,seam_z+.028),-.011,
                    'reopen the jaws clear of the first seam',.7,roll=-.4,pitch=left_pitch)
            left.go((stem_x+.012,seam_y+.004,seam_z+.002),-.011,
                    'approach the central seam from the opened side',1.3,roll=regrasp_roll,pitch=left_pitch)
            left.go((stem_x+.012,seam_y+.004,seam_z+.002),0.,
                    'grasp the central shell edge',.8,roll=regrasp_roll,pitch=left_pitch)
            left.go((stem_x+.002,seam_y+.026,seam_z+.040),0.,
                    'peel the remaining seam against the held end',1.7,roll=-.55,pitch=left_pitch)
        right.wait_until(left.duration+1.0)
        if POUR_AFTER_LIFT:
            left.wait_until(right.duration)
            left.go((stem_x+.002,seam_y+.026,seam_z+.040),OPEN,
                    'release the shell edge above the bowl',.6,roll=-.55,pitch=left_pitch)
            left.go((.23,.22,TABLE+.14),OPEN,'clear the shell transfer',1.5)
            right.wait_until(left.duration)
            if STAGE_SHELL:
                plate_grasp = (x, y, TABLE+.023)
                plate_hover = (x, y, TABLE+.110)
                right.go((shifted_x,shifted_y,TABLE+.122),0.,
                         'lift the shell above the bowl rim',1.0,pitch=right_pitch)
                right.go(plate_hover,0.,'return the shell to the plate',1.7,pitch=.25)
                right.go(plate_grasp,0.,'lower the shell to the plate',1.0,pitch=.25)
                right.go(plate_grasp,-.015,'release the shell onto the plate',1.0,pitch=.25)
                right.go((x,y,TABLE+.055),-.015,'lift the open jaws clear',.7,pitch=.25)
                right.go(plate_grasp,-.015,'straddle the deposited shell',.9,pitch=.25)
                right.go(plate_grasp,0.,'regrasp the shell from the plate',.8,pitch=.25)
                right.go(plate_hover,0.,'lift the shell free of the plate',1.0,pitch=.25)
            else:
                right.go((shifted_x,shifted_y,TABLE+.128),0.,
                         'raise the emptied pod before moving',1.2,pitch=right_pitch)
            right.go((BIN_POS[0],BIN_POS[1],TABLE+.150),0.,
                     'carry the shell above the waste bin',2.2,pitch=right_pitch)
            right.go((BIN_POS[0],BIN_POS[1],TABLE+.104),0.,'lower the shell into the bin',1.)
            right.go((BIN_POS[0],BIN_POS[1],TABLE+.104),OPEN,'release the shell',.8)
            right.go((.25,-.23,TABLE+.14),OPEN,'clear the bin',1.3)
            right.wait_until(right.duration+1.1)
        left.wait_until(right.duration)
        return waypoints(left,right)
    if IDLE or FORCE_PROBE:
        l,r=Track(),Track()
        l.go((.23,.18,TABLE+.14),OPEN,'idle attachment/control test',12.)
        r.go((.23,-.19,TABLE+.14),OPEN,'idle attachment/control test',12.)
        return waypoints(l,r)
    t=tracks()
    return waypoints(t['left'],t['right'])


def self_test():
    assert all(x.exists() for x in generate())
    check_tracks(tracks(),MAX_STEPS)
    assert not _inside_bowl(POD_POS)
    assert not _inside_bin(POD_POS)
    assert _inside_bowl((BOWL_POS[0],BOWL_POS[1],BOWL_POS[2]+.012))
    assert _inside_bin((BIN_POS[0],BIN_POS[1],BIN_POS[2]+.06))
    print('PASS: Edamame source geometry, layout, scorer bounds, waypoints (PhysX NOT RUN)')

if __name__=='__main__':self_test()
elif os.environ.get('EDAMAME_PURE_TEST','0') != '1':
    OBJECTS=setup()
