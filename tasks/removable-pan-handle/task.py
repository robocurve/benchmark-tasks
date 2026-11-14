"""Attach the detachable handle to the saucepan and use it to move the pan onto the trivet.

The native scorer and full transfer are undergoing validation. Mechanical
feasibility scripts read privileged state but command only robot joints.
"""
import tempfile
import math
from pathlib import Path

import numpy as np

from resist_isaac.state import pose_state, tensor
from resist_isaac.usd_assets import create_asset
from .geometry import geometry, TABLE, TRIVET, OPEN_ANGLE, CLOSED_ANGLE, RIM_RADIUS, RIM_DATUM_HEIGHT, ry
from .contacts import NativeContacts
from .scoring import PanScore

CONDITIONS=('recoverable',)
FOLDER=Path(__file__).parent
PINCH_ANGLE = math.radians(110)
PINCH_ROTATION = np.array([[0.,-1.,0.],
    [math.cos(PINCH_ANGLE),0.,-math.sin(PINCH_ANGLE)],
    [math.sin(PINCH_ANGLE),0.,math.cos(PINCH_ANGLE)]])


def pinch_tcp(upper_gap):
    """Keep the upper finger at the modeled housing roof as the gripper closes.

    25.5 mm is slightly inside the lofted roof at x=-60 mm. The other finger
    squeezes the lower lever through contact; no mechanism joint is commanded.
    """
    y = .0052
    z = .0255 - upper_gap/math.sin(PINCH_ANGLE) - y/math.tan(PINCH_ANGLE)
    return np.array([-.060,y,z])


def create_assets(directory,condition='recoverable'):
    from pxr import Usd,UsdPhysics,UsdShade,PhysxSchema
    if condition not in CONDITIONS: raise ValueError(condition)
    descriptions,_=geometry()
    paths={name:create_asset(data,Path(directory)/(name+'.usda'),FOLDER)
           for name,data in descriptions.items()}
    stage=Usd.Stage.Open(str(paths['handle']))
    bodies=[p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    for p in bodies:
        PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0)
        # Internal bearings/linkage use joint constraints; prevent their coarse
        # enclosed collision meshes from fighting those constraints.
        UsdPhysics.FilteredPairsAPI.Apply(p).CreateFilteredPairsRel().SetTargets(
            [other.GetPath() for other in bodies if other!=p
             and not ((p.GetName().startswith('button_') and other.GetName()=='lever')
                      or (other.GetName().startswith('button_') and p.GetName()=='lever'))])
    stage.GetRootLayer().Save()
    stage=Usd.Stage.Open(str(paths['pan']))
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0)
        if p.IsA(UsdShade.Shader) and p.GetAttribute('info:id').Get()=='UsdPreviewSurface':
            from pxr import Sdf
            shader=UsdShade.Shader(p)
            shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(.85)
            shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.25)
    stage.GetRootLayer().Save()
    return paths


class PanHandle:
    physics_hz=480
    robot_contact_offset=.0001
    hold_s=.05  # The scorer already verifies the complete two-second final hold.

    def __init__(self):
        self.descriptions,self.poses=geometry()
        self.contacts = None
        self.evidence = None
        self.contact_setting = None
        self.scoring = None

    def set_evidence_directory(self, directory):
        self.evidence = Path(directory)

    def begin_episode(self, base):
        self.close()
        self.base = base
        self.contact_setting = base.sim.get_setting('/physics/disableContactProcessing')
        base.sim.set_setting('/physics/disableContactProcessing', False)
        if bool(base.sim.get_setting('/physics/disableContactProcessing')):
            raise RuntimeError('Native contact processing remains disabled')
        self.contacts = NativeContacts(base, self.evidence)
        self.scoring = PanScore(dict(bodies=self.contacts.paths, fingertips=self.contacts.tips,
            supports=self.contacts.supports, physics_dt_s=self.contacts.dt,
            first_physics_step=self.contacts.step+1))

    def close(self):
        self.scoring = None
        if self.contacts is not None:
            self.contacts.close()
            self.contacts = None
        if self.contact_setting is not None:
            self.base.sim.set_setting('/physics/disableContactProcessing', self.contact_setting)
            self.contact_setting = None

    def build_scene(self,embodiment):
        from isaaclab_arena.assets.object import Object,ObjectType
        from isaaclab_arena.assets.object_reference import ObjectReference
        from isaaclab_arena.utils.pose import Pose
        self._temporary=tempfile.TemporaryDirectory(prefix='pan-handle-')
        paths=create_assets(self._temporary.name)
        objects=[]
        for name,path in paths.items():
            obj=Object(name=name,object_type=ObjectType.BASE,usd_path=str(path),
                       initial_pose=Pose(position_xyz=self.poses[name],
                                         rotation_xyzw=(-.6556123495,.0089384401,-.0310934540,.7544041872)
                                         if name=='handle' else (0,0,0,1)))
            objects.append(obj)
            if name!='fixture':
                for part in self.descriptions[name]['bodies']:
                    objects.append(ObjectReference(parent_asset=obj,name=name+'_'+part['name'],
                        prim_path=obj.get_prim_path()+'/'+part['name'],object_type=ObjectType.RIGID))
        return objects

    def pose_state(self,base,name,part=None):
        return pose_state(base.scene[name+'_'+(part or name)])

    def initial_action(self):
        from yam_arena.constants import READY
        return np.r_[READY,1.,READY,1.]

    def validate_reset(self,base):
        p,_,_=self.pose_state(base,'pan')
        if np.linalg.norm(p.cpu().numpy()-self.poses['pan'])>.005:
            raise RuntimeError('Pan did not remain at its registered native reset')
        hp,_,hv=self.pose_state(base,'handle','housing')
        if (np.linalg.norm(hp.cpu().numpy()-self.poses['handle'])>.01
                or np.linalg.norm(hv[:3].cpu().numpy())>.01
                or np.linalg.norm(hv[3:].cpu().numpy())>.1):
            raise RuntimeError('Handle reset is displaced or still moving')
        values=self.evaluate(base)
        if abs(values['lever_angle_rad']-OPEN_ANGLE)>math.radians(5):
            raise RuntimeError('Handle did not remain open during native reset settling')

    def evaluate(self,base):
        from yam_arena.urdf import quat_xyzw_to_matrix
        p,_,v=self.pose_state(base,'pan')
        hp,hq,_=self.pose_state(base,'handle','housing')
        jp,_,_=self.pose_state(base,'handle','jaw')
        _,lq,_=self.pose_state(base,'handle','lever')
        a,_,_=self.pose_state(base,'handle','rod_a')
        b,_,_=self.pose_state(base,'handle','rod_b')
        rotation=quat_xyzw_to_matrix(hq.cpu().numpy())
        relative=rotation.T@quat_xyzw_to_matrix(lq.cpu().numpy())
        buttons={}
        for name in ('short','long'):
            _,q,_=self.pose_state(base,'handle','button_'+name)
            r=rotation.T@quat_xyzw_to_matrix(q.cpu().numpy())
            buttons['button_'+name+'_rad']=float(math.atan2(r[2,1],r[1,1]))
        frames = self.contacts.consume() if self.contacts else []
        grasp_frames = 0
        lever_contact_frames = 0
        for frame in frames:
            loaded = [row for row in frame['rows'] if abs(row['impulse_Ns']) > 1e-9]
            lever_contact_frames += any(row['body'] == 'lever'
                and row['other'].startswith(self.contacts.robot_prefixes) for row in loaded)
            for tips in self.contacts.tips.values():
                normals = [[np.asarray(row['normal']) for row in loaded
                            if row['body'] == 'housing' and row['other'] == tip] for tip in tips]
                if all(normals) and any(np.dot(a, b) < -.5 for a in normals[0] for b in normals[1]):
                    grasp_frames += 1
                    break
        scores = self.scoring.update(frames) if self.scoring else dict(success=False,score=0.)
        return dict(**scores,
                    contact_frames=len(frames),housing_bilateral_frames=grasp_frames,
                    lever_robot_contact_frames=lever_contact_frames,
                    pan_base_height_m=float(p[2])-TABLE,
                    pan_speed_m_s=float(np.linalg.norm(v[:3].cpu().numpy())),
                    handle_height_m=float(hp[2])-TABLE,
                    handle_tilt_rad=float(math.acos(np.clip(rotation[2,2],-1,1))),
                    handle_yaw_rad=float(math.atan2(rotation[1,0],rotation[0,0])),
                    lever_angle_rad=float(math.atan2(relative[0,2],relative[0,0])),
                    jaw_translation_m=float((rotation.T@(jp-hp).cpu().numpy())[0]),
                    spring_length_m=float(np.linalg.norm((b-a).cpu().numpy())),**buttons)

    def servo_handle(self, base, action, destination, seconds, *, rotation_goal=None, left_plan=None,
                     left_joint_goal=None, left_pinch=False, left_support=False, gain=.35,
                     phase='turn open handle upright with privileged pose feedback'):
        """Privileged demonstration servo: read housing pose, command robot joints only.

        This is mechanical feasibility evidence, not a camera-only policy result.
        The benchmark observation interface does not expose these housing poses.
        """
        from yam_arena import ik, station
        from yam_arena.runtime import arm_q
        from yam_arena.urdf import quat_xyzw_to_matrix
        from tools.run_recovery_pilot import planned_actions

        p, q, _ = self.pose_state(base, 'handle', 'housing')
        start = p.cpu().numpy().copy()
        if start[2] < TABLE + .06:
            raise RuntimeError('Handle pickup failed before upright servo')
        start_rotation = quat_xyzw_to_matrix(q.cpu().numpy())
        end_rotation = np.eye(3) if rotation_goal is None else rotation_goal
        count = round(seconds * 20)
        if sum((left_plan is not None, left_joint_goal is not None, left_pinch, left_support)) > 1:
            raise ValueError('Choose one left-arm motion')
        left_start = action[:6].copy()
        if left_pinch:
            left_robot = base.scene['left_robot']
            # joint8 is the upper finger in PINCH_ROTATION; the two native
            # fingers can travel unequally under contact, so do not average them.
            upper_finger = left_robot.find_joints(['joint8'], preserve_order=True)[0][0]
            closed_ticks = 0
        left_actions = iter(planned_actions(action, left_plan, count)) if left_plan else None
        for tick in range(count):
            if left_actions is not None:
                left_action, phase, _ = next(left_actions)
                action[:7] = left_action[:7]
            fraction = ik.smoothstep(min(1., (tick+1)/(count*.7)))
            if left_joint_goal is not None:
                action[:6] = left_start + fraction*(np.asarray(left_joint_goal)-left_start)
            goal = start + fraction*(np.asarray(destination)-start)
            goal_rotation = ik.slerp_matrix(start_rotation, end_rotation, fraction)
            p, q, _ = self.pose_state(base, 'handle', 'housing')
            position = p.cpu().numpy()
            if position[2] < TABLE + .04:
                raise RuntimeError('Lost handle clearance during diagnostic servo')
            rotation = quat_xyzw_to_matrix(q.cpu().numpy())
            if left_pinch:
                _, lever_q, _ = self.pose_state(base, 'handle', 'lever')
                lever_rotation = rotation.T @ quat_xyzw_to_matrix(lever_q.cpu().numpy())
                angle = math.atan2(lever_rotation[0,2], lever_rotation[0,0])
                closed_ticks = closed_ticks + 1 if angle < CLOSED_ANGLE + math.radians(2) else 0
                if closed_ticks >= 3:
                    return  # Withdraw once the real lever passes over centre.
                upper_gap = -float(tensor(left_robot.data.joint_pos)[0,upper_finger])
                target_left = position + rotation @ pinch_tcp(upper_gap)
                left_joints = arm_q(base.scene['left_robot'])
                left_command, pe, re = ik.solve(target_left,
                    ik.matrix_to_quat_xyzw(rotation @ PINCH_ROTATION), left_joints,
                    base_pos=station.BASES['left'], iters=100)
                if pe > .003 or re > .04:
                    raise RuntimeError(f'Mechanism pinch target unreachable: {pe} m, {re} rad')
                action[:6] += np.clip(.2*(left_command-left_joints), -.03, .03)
                action[6] = 1. - fraction
            joints = arm_q(base.scene['right_robot'])
            tcp, tool_rotation = ik.tcp_pose(joints, station.BASES['right'])
            correction = goal_rotation @ rotation.T
            target = goal + correction@(tcp-position)
            if left_support:
                left_joints = arm_q(base.scene['left_robot'])
                left_tcp, left_rotation = ik.tcp_pose(left_joints, station.BASES['left'])
                left_command, pe, re = ik.solve(goal + correction@(left_tcp-position),
                    ik.matrix_to_quat_xyzw(correction@left_rotation), left_joints,
                    base_pos=station.BASES['left'], iters=100)
                if pe > .003 or re > .04:
                    raise RuntimeError(f'Handle support target unreachable: {pe} m, {re} rad')
                action[:6] += np.clip(gain*(left_command-left_joints), -.04, .04)
                action[6] = 0.
            command, pe, re = ik.solve(target, ik.matrix_to_quat_xyzw(correction@tool_rotation),
                                      joints, base_pos=station.BASES['right'], iters=100)
            if pe > .003 or re > .04:
                raise RuntimeError(f'Handle servo target unreachable: {pe} m, {re} rad')
            # Accumulate measured pose error rather than resetting the target to
            # sagging measured joints each tick; cap command changes at 0.8 rad/s.
            action[7:13] += np.clip(gain*(command-joints), -.04, .04)
            action[13] = 0.
            yield action.copy(), phase, {
                'right': dict(position_m=pe, rotation_rad=re)}

    def scripted_actions(self,base,policy,max_steps):
        if policy == 'direct_pan':
            from tools.run_recovery_pilot import planned_actions
            # Adversarial control: deliver by the rim without using the handle.
            plans = []
            # Use a reachable destination 17 mm from the trivet centre, still
            # within the same 20 mm placement tolerance as the proper delivery.
            for phase,seconds,x,y,z,grip in [
                ('approach pan rim directly',2.,.30,.16,.95,1.),
                ('lower left gripper around pan rim',1.5,.30,.16,.83,1.),
                ('grasp pan directly without handle',.8,.30,.16,.83,0.),
                ('lift pan directly by rim',2.,.30,.16,.86,0.),
                ('carry pan directly above trivet',4.,.288,-.158,.86,0.),
                ('lower directly held pan onto trivet',2.,.288,-.158,.842,0.),
                ('release directly placed pan',1.,.288,-.158,.842,1.),
                ('withdraw from directly placed pan',2.,.288,-.058,.862,1.),
                ('observe direct delivery negative control',2.,.288,-.058,.862,1.)]:
                plans.append(dict(phase=phase,seconds=seconds,
                    left=dict(position=[x,y+.09,z],rotation_xyzw=[math.sqrt(.5),-math.sqrt(.5),0.,0.],grip=grip)))
            yield from planned_actions(self.initial_action(),plans,max_steps)
            return
        attachment = policy in ('attach_handle','deliver_pan','support_pan','squeeze_lift')
        if policy in ('pick_handle','seat_handle') or attachment:
            import json
            from tools.run_recovery_pilot import planned_actions
            plans=json.loads((FOLDER/'trajectory.json').read_text())
            plan=plans['pick_handle']
            count = sum(round(stage['seconds']*20) for stage in plan)
            brace_count = sum(round(stage['seconds']*20) for stage in plans['brace_pan'])
            extra = brace_count + 240 if policy != 'pick_handle' else 0
            if attachment:
                extra += 456
            if policy == 'deliver_pan':
                extra += 330
            if count + 100 + extra > max_steps:
                raise ValueError('Pickup and handle-servo trajectory exceeds action budget')
            for action, phase, errors in planned_actions(self.initial_action(),plan,max_steps):
                yield action, phase, errors
            yield from self.servo_handle(base, action, (.22,.04,.95), 5.)
            if policy != 'pick_handle':
                from yam_arena.urdf import quat_xyzw_to_matrix
                yield from self.servo_handle(base, action, (.22,.04,.95), brace_count/20,
                                              left_plan=plans['brace_pan'])
                p, q, _ = self.pose_state(base, 'pan')
                rotation = quat_xyzw_to_matrix(q.cpu().numpy())
                if rotation[2,2] < math.cos(math.radians(5)):
                    raise RuntimeError('Bracing tilted the pan before handle placement')
                rim = p.cpu().numpy() + rotation @ np.array([-RIM_RADIUS,0,RIM_DATUM_HEIGHT])
                tilted = rotation @ ry(math.radians(20))
                yield from self.servo_handle(base, action, rim + rotation@np.array([-.001,0,.060]),
                                              4., rotation_goal=tilted, phase='approach flared rim at an angle')
                yield from self.servo_handle(base, action, rim + rotation@np.array([-.001,0,.018]),
                                              2., rotation_goal=tilted, phase='lower staggered supports around rim')
                yield from self.servo_handle(base, action, rim, 3., rotation_goal=rotation,
                                              gain=.2, phase='rotate open handle level onto rim')
                # Bracing and insertion can move the free pan a few millimetres.
                # Reobserve its rim rather than pressing toward a stale target.
                p, q, _ = self.pose_state(base, 'pan')
                rotation = quat_xyzw_to_matrix(q.cpu().numpy())
                rim = p.cpu().numpy() + rotation @ np.array([-RIM_RADIUS,0,RIM_DATUM_HEIGHT])
                yield from self.servo_handle(base, action, rim, 3., rotation_goal=rotation,
                                              gain=.2, phase='hold seated handle before closure')
                if attachment:
                    from yam_arena import ik, station
                    # Release the pan before using the free hand under the lever.
                    down_rotation = [math.sqrt(.5),-math.sqrt(.5),0.,0.]
                    release = [dict(phase='release pan brace',seconds=.5,
                                    left=dict(position=[.30,.25,.83],rotation_xyzw=down_rotation,grip=1.)),
                               dict(phase='withdraw left above pan',seconds=1.5,
                                    left=dict(position=[.30,.25,.95],rotation_xyzw=down_rotation,grip=1.)),
                               dict(phase='keep left fingers open for mechanism pinch',seconds=.5,
                                    left=dict(position=[.30,.25,.95],rotation_xyzw=down_rotation,grip=1.))]
                    yield from self.servo_handle(base, action, rim, 2.5, rotation_goal=rotation,
                                                  gain=.2, left_plan=release)
                    # A horizontal approach puts link4 through the tabletop.
                    # Tilt down 20 degrees and close across housing and lever,
                    # so the upper finger opposes the lower finger's force.
                    side_rotation = ik.matrix_to_quat_xyzw(PINCH_ROTATION).tolist()
                    side = np.array([.15,.30,.84])
                    joints, pe, re = ik.solve(side, side_rotation,
                        [.80,1.90,.86,-1.63,.71,1.25],base_pos=station.BASES['left'],iters=200)
                    if pe > .003 or re > .04:
                        raise RuntimeError('Left closing approach is unreachable')
                    yield from self.servo_handle(base, action, rim, 3., rotation_goal=rotation,
                        gain=.2, left_joint_goal=joints, phase='turn left wrist beside pan')
                    hp, hq, _ = self.pose_state(base, 'handle', 'housing')
                    housing_rotation = quat_xyzw_to_matrix(hq.cpu().numpy())
                    from yam_arena.constants import FINGER_OPEN, FINGER_JOINTS
                    around = hp.cpu().numpy() + housing_rotation @ (
                        pinch_tcp(-FINGER_OPEN) + np.array([0.,0.,.002]))
                    approach = [dict(phase='straddle housing and lower lever',seconds=2.,
                        left=dict(position=around.tolist(),
                                  rotation_xyzw=ik.matrix_to_quat_xyzw(housing_rotation @ PINCH_ROTATION).tolist(),
                                  grip=1.))]
                    yield from self.servo_handle(base, action, rim, 2., rotation_goal=rotation,
                                                  gain=.2, left_plan=approach)
                    yield from self.servo_handle(base, action, rim, 3., rotation_goal=rotation,
                        gain=.2, left_pinch=True, phase='pinch housing and lever to close mechanism')
                    if policy == 'squeeze_lift':
                        # Negative control: keep external pressure on the lever.
                        yield from self.servo_handle(base, action, rim + [0.,0.,.035], 3.,
                            rotation_goal=rotation, gain=.2, left_support=True,
                            phase='lift while externally squeezing closing lever')
                        yield from self.servo_handle(base, action, rim + [0.,0.,.035], 1.5,
                            rotation_goal=rotation, gain=.2, left_support=True,
                            phase='observe externally squeezed negative control')
                        return
                    from yam_arena.runtime import arm_q
                    action[:6] = arm_q(base.scene['left_robot'])
                    release_position, release_rotation = ik.tcp_pose(action[:6], station.BASES['left'])
                    withdraw = [dict(phase='open left gripper after mechanism pinch',seconds=.5,
                        left=dict(position=release_position.tolist(),
                                  rotation_xyzw=ik.matrix_to_quat_xyzw(release_rotation).tolist(),grip=1.)),
                        dict(phase='withdraw left from closing lever',seconds=2.,
                        left=dict(position=side.tolist(),rotation_xyzw=side_rotation,grip=1.))]
                    yield from self.servo_handle(base, action, rim, 2.5, rotation_goal=rotation,
                                                  gain=.2, left_plan=withdraw)
                    # The original rear grasp rolls under pan load. Share that
                    # load through the front housing, clear of the pan and lever.
                    hp, hq, _ = self.pose_state(base, 'handle', 'housing')
                    housing_rotation = quat_xyzw_to_matrix(hq.cpu().numpy())
                    above = hp.cpu().numpy() + housing_rotation @ np.array([-.035,0.,.11])
                    front = hp.cpu().numpy() + housing_rotation @ np.array([-.035,0.,.012])
                    support_name = 'front housing'
                    if policy == 'support_pan':
                        pp, pq, _ = self.pose_state(base, 'pan')
                        front = pp.cpu().numpy() + quat_xyzw_to_matrix(pq.cpu().numpy()) @ np.array([0.,.09,.0825])
                        above = front + [0.,0.,.12]
                        support_name = 'pan rim'
                    joints, pe, re = ik.solve(above, down_rotation,
                        [-.73,1.42,1.48,-1.63,0.,-.73],base_pos=station.BASES['left'],iters=200)
                    if pe > .003 or re > .04:
                        raise RuntimeError('Front-housing support approach is unreachable')
                    yield from self.servo_handle(base, action, rim, 3., rotation_goal=rotation,
                        gain=.2, left_joint_goal=joints, phase='turn left above '+support_name)
                    support = [dict(phase='lower left around '+support_name,seconds=1.5,
                        left=dict(position=front.tolist(),rotation_xyzw=down_rotation,grip=1.)),
                        dict(phase='grasp '+support_name,seconds=.8,
                        left=dict(position=front.tolist(),rotation_xyzw=down_rotation,grip=0.))]
                    yield from self.servo_handle(base, action, rim, 2.3, rotation_goal=rotation,
                        gain=.2, left_plan=support)
                    yield from self.servo_handle(base, action, rim + np.array([0.,0.,.035]),
                        3., rotation_goal=rotation, gain=.2, left_support=True,
                        phase='lift with direct pan support' if policy=='support_pan' else 'attempt pan lift through handle only')
                    yield from self.servo_handle(base, action, rim + np.array([0.,0.,.035]),
                        1.5, rotation_goal=rotation, gain=.2, left_support=True,
                        phase='observe direct-support negative control' if policy=='support_pan' else 'observe native loaded retention')
                    if policy == 'deliver_pan':
                        # Use the measured connection offset to place the pan,
                        # while commanding only the two robot grasps.
                        hp, hq, _ = self.pose_state(base, 'handle', 'housing')
                        pp, _, _ = self.pose_state(base, 'pan')
                        housing_rotation = quat_xyzw_to_matrix(hq.cpu().numpy())
                        pan_offset = housing_rotation.T @ (pp-hp).cpu().numpy()
                        angle = -math.pi/4
                        carry_rotation = np.array([[math.cos(angle),-math.sin(angle),0.],
                            [math.sin(angle),math.cos(angle),0.],[0.,0.,1.]])
                        destination = np.asarray(TRIVET) + [0.,0.,.035] - carry_rotation@pan_offset
                        yield from self.servo_handle(base, action, destination, 6.,
                            rotation_goal=carry_rotation, gain=.2, left_support=True,
                            phase='carry attached pan above trivet')
                        yield from self.servo_handle(base, action, destination + [0.,0.,-.023], 2.,
                            rotation_goal=carry_rotation, gain=.2, left_support=True,
                            phase='lower pan onto trivet')
                        # Fully opening the rear grasp in place sweeps its lower
                        # finger through the lever. Unload it, then slide clear.
                        right_robot = base.scene['right_robot']
                        ids = right_robot.find_joints(FINGER_JOINTS,preserve_order=True)[0]
                        gap = float(-tensor(right_robot.data.joint_pos)[0,ids].min()) + .0005
                        loose_grip = min(1.,gap/-FINGER_OPEN)
                        loosen = dict(phase='loosen rear housing grasp on trivet',seconds=1.)
                        slide = dict(phase='slide rear fingers clear of lever',seconds=1.)
                        release = dict(phase='open both housing grasps on trivet',seconds=1.)
                        retreat = dict(phase='withdraw both arms from placed pan',seconds=2.)
                        clear_view = dict(phase='move left arm clear of placed pan',seconds=1.5)
                        for side, offset in [('left',0),('right',7)]:
                            action[offset:offset+6] = arm_q(base.scene[side+'_robot'])
                            position, tool_rotation = ik.tcp_pose(action[offset:offset+6],station.BASES[side])
                            quaternion = ik.matrix_to_quat_xyzw(tool_rotation).tolist()
                            loosen[side] = dict(position=position.tolist(),rotation_xyzw=quaternion,
                                grip=loose_grip if side=='right' else 0.)
                            clear = position - .025*tool_rotation[:,2] if side=='right' else position
                            if side=='right':
                                slide[side] = dict(position=clear.tolist(),rotation_xyzw=quaternion,grip=loose_grip)
                            release[side] = dict(position=clear.tolist(),rotation_xyzw=quaternion,grip=1.)
                            shift = np.array([0.,0.,.06]) if side=='left' else -.08*tool_rotation[:,2]
                            retreat[side] = dict(position=(position+shift).tolist(),rotation_xyzw=quaternion,grip=1.)
                            if side=='left':
                                clear_view[side] = dict(position=(position+shift+[0.,.15,0.]).tolist(),rotation_xyzw=quaternion,grip=1.)
                        yield from planned_actions(action,[loosen,slide,release,retreat,clear_view,
                            dict(phase='observe released pan on trivet',seconds=2.)],max_steps)
            return
        if policy!='hold': raise ValueError('Unknown mechanism diagnostic policy')
        for _ in range(min(60,max_steps)):
            yield self.initial_action(),'native passive reset diagnostic',{}


def make_task(condition):
    if condition not in CONDITIONS: raise ValueError(condition)
    return PanHandle()
