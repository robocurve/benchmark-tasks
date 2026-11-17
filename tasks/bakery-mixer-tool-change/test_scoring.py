"""Scorer logic checks only; includes a recorded native pose regression."""
import importlib.util
import math
from pathlib import Path
import unittest
import numpy as np

spec=importlib.util.spec_from_file_location('mixer_task',Path(__file__).with_name('task.py'))
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class Scoring(unittest.TestCase):
    def setUp(self):
        self.task=module.make_task('recoverable')
        identity=np.eye(3);zero=np.zeros(6)
        def state(p,r=identity):return np.array(p,dtype=float),r.copy(),zero.copy()
        angle=-math.pi/3
        rotation=np.array([[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1]])
        self.states={'beater':state([.28,-.22,1.0715],rotation),
            'mixer/shaft':state([.28,-.22,1.0775]),'mixer/carrier':state([.26,-.22,1.0775]),
            'mixer/bowl':state([.26,-.22,.8925]),'guard':state([.26,-.22,1.037]),
            'mixer/start':state([.235,-.282,1.1625]),'mixer/stop':state([.235,-.158,1.1625])}
        self.hands=[[-.1,.5,1.2],[-.1,-.5,1.2]]
        self.still=dict(beater=0.,shaft=0.,carrier=0.)
        self.rotating=dict(beater=136.,shaft=136.,carrier=-60.)
        self.prove_retention()

    def prove_retention(self):
        """Logic fixture: first observe the released tool with bowl down and guard away."""
        bowl=self.states['mixer/bowl'][0].copy();guard=self.states['guard'][0].copy()
        self.states['mixer/bowl'][0][2]-=.075
        self.states['guard'][0][:]=[.51,.31,.7825]
        self.assertEqual(self.score(20)['score'],.25)
        self.states['mixer/bowl'][0][:]=bowl;self.states['guard'][0][:]=guard

    def score(self,n=1,rpm=None):
        for _ in range(n):result=self.task.score_states(self.states,self.hands,rpm or self.still)
        return result

    def test_drive_ramps_are_bounded_continuous_and_reach_target(self):
        start=[module.ramp_fraction(0.,True,tick) for tick in range(41)]
        self.assertEqual(start[0],0.)
        self.assertEqual(start[-1],1.)
        self.assertTrue(all(0<=a<=b<=1 for a,b in zip(start,start[1:])))
        self.assertLess(start[1],.002)
        stop=[module.ramp_fraction(.63,False,tick) for tick in range(41)]
        self.assertEqual(stop[0],.63)
        self.assertAlmostEqual(stop[-1],0.)
        self.assertTrue(all(0<=b<=a<=.63 for a,b in zip(stop,stop[1:])))
        self.assertAlmostEqual(module.ramp_fraction(.63,False,100),0.)

    def test_interlock_cancels_an_in_progress_stop_ramp(self):
        self.task._motor=True
        for _ in range(40):self.task.update_drive_ramp(True)
        self.assertEqual(self.task._drive_fraction,1.)
        self.task._motor=False
        for _ in range(10):self.task.update_drive_ramp(True)
        self.assertGreater(self.task._drive_fraction,0.)
        self.task.update_drive_ramp(False)
        self.assertEqual(self.task._drive_fraction,0.)
        for _ in range(50):
            self.task.update_drive_ramp(True)
            self.assertEqual(self.task._drive_fraction,0.)

    def test_full_run_requires_stop(self):
        self.assertEqual(self.score(20)['score'],.5)
        self.task._motor=True
        self.assertFalse(self.score(99,self.rotating)['verified_run'])
        self.assertTrue(self.score(1,self.rotating)['verified_run'])
        self.assertFalse(self.score()['success'])
        self.task._motor=False
        self.assertFalse(self.score()['success'])
        self.task._stopped_after_run=True
        self.assertTrue(self.score()['success'])

    def test_guard_prop_cannot_count_as_initial_released_retention(self):
        self.task._reset_counters()
        self.states['mixer/shaft'][0][:]=[.28,.03,1.0775]
        self.states['beater'][0][:]=[.28,.03,1.0718]
        self.states['mixer/bowl'][0][:]=[.26,.03,.8175]
        self.states['guard'][0][:]=[.30294967,.04325,1.0043]
        self.hands[0]=[.29169134,.17425,1.0388]
        self.assertTrue(self.task.observable_state(self.states)['beater_attached'])
        for _ in range(40):
            result=self.task.score_states(self.states,self.hands,self.still,external_beater_contact=True)
        self.assertEqual(result['score'],0.)
        self.assertFalse(result['beater_retained'])
        self.assertTrue(result['external_beater_contact'])

    def test_spring_collar_can_compress_without_hitting_carrier(self):
        import json
        geometry=json.loads(Path(__file__).with_name('geometry.json').read_text())
        parts={b['name']:b for b in geometry['mixer']['bodies']}
        spring=next(j for j in geometry['mixer']['joints'] if j['name']=='retaining_spring')
        collar=parts['retaining_collar']
        z=[p[2] for shape in collar['shapes'] for p in shape['mesh']['points']]
        free_bottom=collar['position'][2]+min(z)
        hub_top=max(p[2] for shape in geometry['beater']['bodies'][0]['shapes'] if shape['type']=='mesh' for p in shape['mesh']['points'])
        resting=parts['shaft']['position'][2]+self.task.cfg['hub_rest_offset']+hub_top-free_bottom
        inserted=parts['shaft']['position'][2]+self.task.cfg['hub_insert_offset']+hub_top-free_bottom
        self.assertTrue(0<resting<inserted<spring['upper'])
        carrier=parts['carrier'];disc=carrier['shapes'][0]
        underside=carrier['position'][2]+disc['position'][2]-disc['size'][1]
        self.assertGreater(underside-(collar['position'][2]+max(z)+inserted),.001)
        self.assertAlmostEqual(-spring['stiffness']*spring['target'],5.)
        inner=min(math.hypot(p[0],p[1]) for shape in collar['shapes'] for p in shape['mesh']['points'])*math.cos(math.pi/32)
        bore=.00635+.0005+hub_top*math.sin(math.radians(3))
        self.assertGreater(inner-bore,.0005,'Pressure face can catch the hub bore edge')

    def test_held_beater_has_no_credit(self):
        self.task._reset_counters()
        self.hands[0]=[.28,-.22,1.0315]
        self.assertEqual(self.score(40)['score'],0.)

    def test_held_guard_cannot_earn_assembly_credit(self):
        self.hands[0]=(self.states['guard'][0]+[0.,.13,0.]).tolist()
        self.assertEqual(self.score(40)['score'],.25)
        self.hands[0]=[-.1,.5,1.2]
        self.assertEqual(self.score(19)['score'],.25)
        self.assertEqual(self.score()['score'],.5)

    def test_stop_approach_preserves_completed_run(self):
        self.score(20)
        self.task._motor=True
        self.assertTrue(self.score(100,self.rotating)['verified_run'])
        self.hands[0]=(self.states['guard'][0]+[.08,.04,.14]).tolist()
        self.assertTrue(self.score()['verified_run'])
        self.task._motor=False
        self.task._stopped_after_run=True
        self.hands[0]=[-.1,.5,1.2]
        self.assertTrue(self.score(20)['success'])

    def test_hidden_hub_yaw_does_not_define_score(self):
        self.states['beater']=(self.states['beater'][0],np.eye(3),np.zeros(6))
        self.assertEqual(self.score(40)['score'],.5)

    def test_stationary_beater_cannot_count_run(self):
        self.task._motor=True
        result=self.score(150,dict(beater=0.,shaft=136.,carrier=-60.))
        self.assertFalse(result['verified_run'])

    def test_guard_missing_during_rotation_latches_zero(self):
        self.task._drive_started=True
        self.states['guard'][0][:]=[.51,.31,.7825]
        self.assertTrue(self.score(rpm=self.rotating)['unsafe_event'])
        self.assertEqual(self.score()['score'],0.)

    def test_hand_intrusion_latches_zero(self):
        self.task._drive_started=True
        self.hands[0]=[.26,-.22,1.]
        self.assertTrue(self.score(rpm=self.rotating)['unsafe_event'])
        self.hands[0]=[-.1,.5,1.2]
        self.assertEqual(self.score(40)['score'],0.)

    def test_partial_credit(self):
        self.states['mixer/bowl'][0][2]-=.075
        self.states['guard'][0][2]-=.075
        self.assertEqual(self.score(20)['score'],.25)
        self.states['guard'][0][:]=[.51,.31,.7825]
        self.assertEqual(self.score()['score'],.25)

    def test_nonfinite_state_rejected(self):
        self.states['beater'][0][0]=float('nan')
        with self.assertRaises(RuntimeError):self.score()

    def test_manual_unpowered_twist_is_not_rotating_machine_hazard(self):
        self.task._reset_counters()
        self.hands[0]=[.28,-.22,1.0315]
        result=self.score(rpm=self.rotating)
        self.assertFalse(result['unsafe_event'])
        self.assertEqual(result['score'],0.)

    def test_disassembly_invalidates_previous_run(self):
        self.score(20)
        self.task._motor=True
        self.assertTrue(self.score(100,self.rotating)['verified_run'])
        self.task._motor=False
        self.states['guard'][0][:]=[.51,.31,.7825]
        self.assertFalse(self.score()['verified_run'])

    def test_reversed_rotation_does_not_count(self):
        self.task._motor=True
        self.assertFalse(self.score(150,dict(beater=-136.,shaft=-136.,carrier=60.))['verified_run'])

    def test_interrupted_run_requires_new_five_second_interval(self):
        self.score(20);self.task._motor=True
        self.score(99,self.rotating)
        self.score()
        self.assertFalse(self.score(99,self.rotating)['verified_run'])
        self.assertTrue(self.score(1,self.rotating)['verified_run'])

    def test_unpowered_rotation_cannot_qualify(self):
        self.assertFalse(self.score(150,self.rotating)['verified_run'])

    def test_fidelity_note_fits_submission_limit(self):
        import json
        note=json.loads(Path(__file__).with_name('task.json').read_text())['fidelity']
        self.assertTrue(0<len(note)<=500)

    def test_hub_can_reach_turning_depth_below_carrier(self):
        import json
        geometry=json.loads(Path(__file__).with_name('geometry.json').read_text())
        parts={b['name']:b for b in geometry['mixer']['bodies']}
        carrier=parts['carrier'];disc=carrier['shapes'][0]
        underside=carrier['position'][2]+disc['position'][2]-disc['size'][1]
        pin_height=parts['shaft']['position'][2]
        hub_top=max(p[2] for s in geometry['beater']['bodies'][0]['shapes'] if s['type']=='mesh' for p in s['mesh']['points'])
        required=pin_height+self.task.cfg['hub_insert_offset']+hub_top
        self.assertGreater(underside-required,.0002,'Carrier obstructs the required insertion depth')

    def test_guard_rear_entry_corridor_is_open(self):
        import json
        from yam_arena.urdf import quat_xyzw_to_matrix
        geometry=json.loads(Path(__file__).with_name('geometry.json').read_text())
        for index,shape in enumerate(geometry['guard']['bodies'][0]['shapes']):
            if shape['type']=='capsule':
                ends=np.array(shape['fromto']).reshape(2,3)
                lo=ends.min(axis=0)-shape['size'][0];hi=ends.max(axis=0)+shape['size'][0]
            elif shape['type']=='box':
                q=shape.get('orientation',[1,0,0,0])
                r=quat_xyzw_to_matrix(np.array([*q[1:],q[0]]))
                extent=np.abs(r)@np.array(shape['size']);center=np.array(shape['position'])
                lo=center-extent;hi=center+extent
            else:self.fail('Unreviewed guard shape '+shape['type'])
            obstructs=hi[0]>=.04 and lo[0]<=.14 and lo[1]<=.005 and hi[1]>=-.005
            self.assertFalse(obstructs,f'Guard shape {index} blocks the beater entry corridor')

    def test_rotating_paddle_clears_raised_bowl(self):
        import json
        geometry=json.loads(Path(__file__).with_name('geometry.json').read_text())
        parts={part['name']:part for part in geometry['mixer']['bodies']}
        bowl=parts['bowl'];profile={};supports=[]
        for shape in bowl['shapes']:
            if shape['type']!='mesh':
                self.assertNotIn('orientation',shape,'Review rotated bowl attachments explicitly')
                center=np.array(shape['position']);size=shape['size']
                if shape['type']=='box':extent=np.array(size)
                elif shape['type']=='cylinder':extent=np.array([size[0],size[0],size[1]])
                else:self.fail('Unreviewed bowl attachment '+shape['type'])
                nearest_radius=np.linalg.norm(np.maximum(np.abs(center[:2])-extent[:2],0))
                supports.append((nearest_radius,center[2]-extent[2],center[2]+extent[2]))
                continue
            for x,y,z in shape['mesh']['points']:
                profile[z]=min(profile.get(z,float('inf')),math.hypot(x,y))
        profile=np.array([[radius*math.cos(math.pi/64),z] for z,radius in sorted(profile.items())])
        floor=bowl['shapes'][0]['position'][2]+bowl['shapes'][0]['size'][1]
        height=parts['shaft']['position'][2]+self.task.cfg['hub_rest_offset']-bowl['position'][2]-self.task.cfg['lift_travel']
        orbit=np.linalg.norm(np.array(parts['shaft']['position'][:2])-parts['carrier']['position'][:2])
        for tilt in [0.,math.radians(3)]:
            for shape in geometry['beater']['bodies'][0]['shapes']:
                if shape['type']!='capsule':continue
                a,b=np.array(shape['fromto']).reshape(2,3)
                for fraction in np.linspace(0,1,81):
                    point=a+fraction*(b-a);radius=np.linalg.norm(point[:2])
                    rho=orbit+radius*math.cos(tilt)+abs(point[2])*math.sin(tilt)+.0005
                    z=height+point[2]*math.cos(tilt)-radius*math.sin(tilt)-.0006
                    distances=[z-floor]
                    for start,end in zip(profile,profile[1:]):
                        direction=end-start
                        t=np.clip(np.dot(np.array([rho,z])-start,direction)/np.dot(direction,direction),0,1)
                        distances.append(np.linalg.norm(np.array([rho,z])-start-t*direction))
                    for nearest_radius,zlo,zhi in supports:
                        distance=math.hypot(max(nearest_radius-rho,0),max(zlo-z,z-zhi,0))
                        self.assertGreater(distance-shape['size'][0],.001,
                            f'Paddle sweep hits bowl attachment at radius {nearest_radius:.4f}, z {zlo:.4f}..{zhi:.4f}')
                    self.assertLess(rho,np.interp(z,profile[:,1],profile[:,0]),'Paddle centre lies outside bowl wall')
                    self.assertGreater(min(distances)-shape['size'][0],.001,'Paddle sweep lacks 1 mm clearance')

    def test_native_supported_pocket_edge_is_recognized(self):
        import json
        from yam_arena.urdf import quat_xyzw_to_matrix
        fixture=json.loads(Path(__file__).with_name('supported-pocket-native-fixture.json').read_text())
        row=fixture['trace']
        contacts={key:value for key,value in fixture['last_contact_snapshot']['pairs'].items() if 'beater/beater' in key}
        self.assertEqual(list(contacts),['beater/beater | mixer/shaft'])
        self.assertGreater(contacts['beater/beater | mixer/shaft']['reported_normal_impulse_vector_Ns'][2],.25*9.81/240*.9)
        self.assertEqual(row['metrics']['score'],0.)  # Preserve the historical false negative.
        state={key:(np.array(p),quat_xyzw_to_matrix(np.array(q)),np.array(v))
               for key,(p,q,v) in row['objects'].items()}
        for _ in range(20):result=self.task.score_states(state,list(row['tcp'].values()),self.still)
        self.assertEqual(result['score'],.25)

    def test_inserted_turn_follows_the_current_shaft_frame(self):
        from yam_arena.urdf import axis_angle_matrix
        spec=importlib.util.spec_from_file_location('mixer_controller',Path(__file__).with_name('scripted.py'))
        controller=importlib.util.module_from_spec(spec);spec.loader.exec_module(controller)
        offset=np.array([0.,0.,-.001]);axis=np.array([0.,0.,1.])
        goal=axis_angle_matrix(axis,-math.pi/3)
        for angle in [-.8,0.,1.5]:
            shaft_r=axis_angle_matrix(axis,angle)
            for fraction in [0.,.25,.485,.75,1.]:
                sp=np.array([.28+.02*math.cos(angle),.03+.02*math.sin(angle),1.0775])
                position,rotation=controller.shaft_frame_pose(sp,shaft_r,offset,np.eye(3),offset,goal,fraction)
                np.testing.assert_allclose(shaft_r.T@(position-sp),offset,atol=1e-12)
                np.testing.assert_allclose(shaft_r.T@rotation,axis_angle_matrix(axis,-math.pi/3*fraction),atol=1e-12)

    def test_released_withdrawal_stays_inside_joint_limits(self):
        import json
        from yam_arena import ik,station
        from yam_arena.urdf import quat_xyzw_to_matrix
        spec=importlib.util.spec_from_file_location('mixer_controller',Path(__file__).with_name('scripted.py'))
        controller=importlib.util.module_from_spec(spec);spec.loader.exec_module(controller)
        fixture=json.loads(Path(__file__).with_name('withdrawal-native-fixture.json').read_text())
        q=np.array(fixture['left_q']);p0,r0=ik.tcp_pose(q,station.BASES['left'])
        plan=json.loads(Path(__file__).with_name('trajectory.json').read_text())['mount']
        clear_r=quat_xyzw_to_matrix(np.array(next(s['left']['rotation_xyzw'] for s in plan if s['phase']=='approach beater')))
        for tick in range(1,61):
            f=ik.smoothstep(tick/60)
            p,r=controller.beater_withdrawal(p0,r0,clear_r,f)
            q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(r),q,station.BASES['left'],iters=200)
            self.assertLess(pe,.003)
            self.assertLess(re,.04)
            self.assertGreater(float(np.min(np.minimum(q-ik.LOWER,ik.UPPER-q))),.1)
        self.assertAlmostEqual(float(np.linalg.norm(p-p0)),.12)

    def test_withdrawal_from_native_wrist_limit(self):
        import json
        from yam_arena import ik,station
        from yam_arena.urdf import quat_xyzw_to_matrix
        spec=importlib.util.spec_from_file_location('mixer_controller',Path(__file__).with_name('scripted.py'))
        controller=importlib.util.module_from_spec(spec);spec.loader.exec_module(controller)
        fixture=json.loads(Path(__file__).with_name('withdrawal-limit-native-fixture.json').read_text())
        q=np.array(fixture['left_q']);p0,r0=ik.tcp_pose(q,station.BASES['left'])
        self.assertLess(ik.UPPER[3]-q[3],1e-6)
        plan=json.loads(Path(__file__).with_name('trajectory.json').read_text())['mount']
        clear_r=quat_xyzw_to_matrix(np.array(next(s['left']['rotation_xyzw'] for s in plan if s['phase']=='approach beater')))
        for tick in range(1,61):
            p,r=controller.beater_withdrawal(p0,r0,clear_r,ik.smoothstep(tick/60))
            q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(r),q,station.BASES['left'],iters=200)
            self.assertLess(pe,.003)
            self.assertLess(re,.04)
            self.assertGreater(float(np.min(np.minimum(q-ik.LOWER,ik.UPPER-q))),.005)

    def test_blocked_alignment_does_not_chase_a_rotating_shaft(self):
        from yam_arena.urdf import axis_angle_matrix
        spec=importlib.util.spec_from_file_location('mixer_controller',Path(__file__).with_name('scripted.py'))
        controller=importlib.util.module_from_spec(spec);spec.loader.exec_module(controller)
        z=np.array([0.,0.,1.])
        tool=axis_angle_matrix(z,-1.2)@axis_angle_matrix(np.array([1.,0.,0.]),.02)
        for yaw in [-.5,-.8,-1.1]:
            target=controller.bayonet_alignment_rotation(tool,axis_angle_matrix(z,yaw))
            np.testing.assert_allclose(target,axis_angle_matrix(z,-1.2),atol=1e-12)
            np.testing.assert_allclose(target[:,2],z,atol=1e-12)

    def test_guard_approach_has_wrist_margin_from_native_grasp(self):
        import json
        from yam_arena import ik,station
        from yam_arena.urdf import quat_xyzw_to_matrix
        spec=importlib.util.spec_from_file_location('mixer_controller',Path(__file__).with_name('scripted.py'))
        controller=importlib.util.module_from_spec(spec);spec.loader.exec_module(controller)
        fixture=json.loads(Path(__file__).with_name('guard-approach-native-fixture.json').read_text())
        q=np.array(fixture['left_q']);p=np.array(fixture['objects']['guard'][0])
        r=quat_xyzw_to_matrix(np.array(fixture['objects']['guard'][1]))
        hp,hr=ik.tcp_pose(q,station.BASES['left']);relative_p=r.T@(hp-p);relative_r=r.T@hr
        y=fixture['objects']['mixer/shaft'][0][1]
        goals=[('lift guard clear of rack',[.35,.31,.92],80),
               ('carry guard to front left',[.20,.28,1.015],80),
               ('align guard in front of bowl',[.145,y,1.00],80),
               ('slide guard around tool',[.26,y,1.00],80),
               ('center guard above bowl',[.26,.03,1.00],40),
               ('seat guard on rim',[.26,.03,.962],60)]
        for phase,goal,controls in goals:
            goal_r=controller.guard_goal_rotation(phase)
            for tick in range(1,controls+1):
                f=ik.smoothstep(tick/controls);target=p+f*(np.array(goal)-p)
                rotation=ik.slerp_matrix(r,goal_r,f)
                previous=q.copy()
                q,pe,re=ik.solve(target+rotation@relative_p,ik.matrix_to_quat_xyzw(rotation@relative_r),q,station.BASES['left'],iters=200)
                self.assertLess(pe,.003);self.assertLess(re,.04)
                self.assertGreater(float(np.min(np.minimum(q-ik.LOWER,ik.UPPER-q))),.1)
                self.assertTrue(np.all(abs(q-previous)*20<np.array([5.5]*3+[20.]*3)))
            p=np.array(goal);r=goal_r
        np.testing.assert_allclose(r,np.eye(3),atol=1e-12)

    def test_turn_waits_for_entire_shaft_and_pin_clearance(self):
        from yam_arena.urdf import axis_angle_matrix,quat_xyzw_to_matrix
        spec=importlib.util.spec_from_file_location('mixer_controller',Path(__file__).with_name('scripted.py'))
        controller=importlib.util.module_from_spec(spec);spec.loader.exec_module(controller)
        p=np.array([0.,0.,-.001]);sp=np.zeros(3);r=np.eye(3)
        self.assertTrue(controller.bayonet_turn_clear(p,r,sp,r))
        self.assertFalse(controller.bayonet_turn_clear(p+[.0004,0,0],r,sp,r))
        self.assertFalse(controller.bayonet_turn_clear(p,axis_angle_matrix(np.array([1.,0.,0.]),.03),sp,r))
        self.assertFalse(controller.bayonet_turn_clear(np.array([0.,0.,-.006]),r,sp,r))
        import json
        fixture=json.loads(Path(__file__).with_name('binding-native-fixture.json').read_text())['trace']
        bp,bq,_=fixture['objects']['beater'];sp,sq,_=fixture['objects']['mixer/shaft']
        self.assertFalse(controller.bayonet_turn_clear(np.array(bp),quat_xyzw_to_matrix(np.array(bq)),np.array(sp),quat_xyzw_to_matrix(np.array(sq))))

    def test_insertion_compensation_follows_planned_spring_load(self):
        spec=importlib.util.spec_from_file_location('mixer_controller',Path(__file__).with_name('scripted.py'))
        controller=importlib.util.module_from_spec(spec);spec.loader.exec_module(controller)
        self.assertEqual(controller.insertion_spring_force(-.045,400.,-.0125),0.)
        self.assertAlmostEqual(controller.insertion_spring_force(-.008,400.,-.0125),5.)
        self.assertAlmostEqual(controller.insertion_spring_force(-.001,400.,-.0125),7.8)
        values=[controller.insertion_spring_force(z,400.,-.0125) for z in np.linspace(-.045,-.001,100)]
        self.assertTrue(all(0<=a<=b<=7.8+1e-10 for a,b in zip(values,values[1:])))

    def test_lift_force_bias_obeys_virtual_work(self):
        from yam_arena import ik,station
        from yam_arena.constants import SHOULDER,WRIST
        spec=importlib.util.spec_from_file_location('mixer_controller',Path(__file__).with_name('scripted.py'))
        controller=importlib.util.module_from_spec(spec);spec.loader.exec_module(controller)
        q=np.array([.2,1.6,.7,.8,-1.,-1.5])
        force=np.array([3.,0.,17.]);delta=np.array([1.,-2.,3.,-1.,2.,-3.])*1e-6
        bias=controller.force_position_bias(q,force,station.BASES['right'])
        torque=bias*np.array([SHOULDER['stiffness']]*3+[WRIST['stiffness']]*3)
        displacement=ik.tcp_pose(q+delta,station.BASES['right'])[0]-ik.tcp_pose(q,station.BASES['right'])[0]
        self.assertAlmostEqual(float(torque@delta),float(force@displacement),places=9)
        np.testing.assert_array_equal(controller.force_position_bias(q,np.zeros(3),station.BASES['right']),np.zeros(6))

    def test_retaining_pocket_angle_is_not_a_scoring_predicate(self):
        for offset in [-15,15]:
            with self.subTest(offset=offset):
                angle=self.task.cfg['hub_yaw']+math.radians(offset)
                r=np.array([[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1]])
                self.states['beater']=(self.states['beater'][0],r,np.zeros(6))
                self.assertEqual(self.score(40)['score'],.5)

    def test_actual_native_released_retention_is_recognized(self):
        import json
        from yam_arena.urdf import quat_xyzw_to_matrix
        fixture=json.loads(Path(__file__).with_name('retained-native-fixture.json').read_text())
        row=fixture['trace']
        state={key:(np.array(p),quat_xyzw_to_matrix(np.array(q)),np.array(v))
               for key,(p,q,v) in row['objects'].items()}
        beater_contacts=[key for key in fixture['last_contact_snapshot']['pairs'] if 'beater/beater' in key]
        self.assertEqual(beater_contacts,['beater/beater | mixer/shaft'])
        self.task._reset_counters()
        for _ in range(20):result=self.task.score_states(state,list(row['tcp'].values()),self.still)
        self.assertEqual(result['score'],.25)

    def test_contact_diagnostics_do_not_gate_observable_retention(self):
        self.task._reset_counters()
        self.states['mixer/bowl'][0][2]-=.075
        self.states['guard'][0][:]=[.51,.31,.7825]
        for _ in range(20):
            result=self.task.score_states(self.states,self.hands,self.still,external_beater_contact=True)
        self.assertEqual(result['score'],.25)
        self.assertTrue(result['external_beater_contact'])

    def test_nearby_guard_cannot_prop_the_initial_retention_test(self):
        self.task._reset_counters()
        self.states['mixer/bowl'][0][2]-=.075
        self.states['guard'][0][:]=self.states['beater'][0]+[.349,0,0]
        self.assertEqual(self.score(40)['score'],0.)
        self.states['guard'][0][:]=self.states['beater'][0]+[.351,0,0]
        self.assertEqual(self.score(20)['score'],.25)

    def test_guard_or_raised_bowl_cannot_supply_initial_retention_proof(self):
        for lowered,guard_away in [(False,False),(False,True),(True,False)]:
            with self.subTest(lowered=lowered,guard_away=guard_away):
                self.task._reset_counters()
                self.states['mixer/bowl'][0][2]=.8175 if lowered else .8925
                self.states['guard'][0][:]=[.51,.31,.7825] if guard_away else [.26,-.22,1.037]
                self.assertEqual(self.score(40)['score'],0.)

    def test_rack_beater_does_not_earn_retention_credit(self):
        self.task._reset_counters()
        self.states['beater'][0][:]=self.task.cfg['poses']['beater']
        self.assertEqual(self.score(40)['score'],0.)

    def test_exterior_neck_band_has_a_five_mm_radial_boundary(self):
        original=self.states['beater'][0].copy()
        self.states['beater'][0][:]=original+[.0049,0,0]
        self.assertTrue(self.task.observable_state(self.states)['beater_attached'])
        self.assertEqual(self.score(20)['score'],.5)
        self.states['beater'][0][:]=original+[.0051,0,0]
        self.assertFalse(self.task.observable_state(self.states)['beater_attached'])
        self.assertEqual(self.score()['score'],0.)
        self.states['beater'][0][:]=original
        self.assertEqual(self.score(40)['score'],0.,'Disassembly requires a new released hang test')

    def test_hidden_shaft_speed_and_exact_ratio_do_not_gate_run(self):
        self.task._motor=True
        result=self.score(100,dict(beater=60.,shaft=0.,carrier=-23.))
        self.assertTrue(result['verified_run'])
        self.assertEqual(result['score'],.75)

    def test_visible_rotation_below_one_turn_per_second_does_not_count(self):
        self.task._motor=True
        self.assertFalse(self.score(150,dict(beater=59.9,shaft=136.,carrier=-60.))['verified_run'])

    def test_stop_uses_exterior_motion_and_ignores_hidden_shaft(self):
        self.task._motor=True;self.score(100,self.rotating)
        self.task._motor=False;self.task._stopped_after_run=True
        self.assertTrue(self.score(rpm=dict(beater=.4,shaft=100.,carrier=-.4))['success'])
        self.assertFalse(self.score(rpm=dict(beater=.401,shaft=0.,carrier=0.))['success'])


if __name__=='__main__':unittest.main()
