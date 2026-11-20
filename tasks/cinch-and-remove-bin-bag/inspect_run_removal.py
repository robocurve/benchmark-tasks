"""Loaded drawstring bag removal using YAM / Isaac Lab Arena.

No pre-tied neck, replacement liner or retaining ring. A force-based sliding-cord approximation acts on dynamic hem markers; no object poses are controlled.
"""
from pathlib import Path
import sys, os, json, traceback, argparse
parser=argparse.ArgumentParser();parser.add_argument("--mode",choices=["setup","cinch","interactive","full"],default="cinch");parser.add_argument("--camera",action="store_true");parser.add_argument("--run",default="realistic_v21_run");parser.add_argument('--device',default='cuda:0');args=parser.parse_args()
HERE=Path(__file__).resolve().parent
sys.path.insert(0,os.environ.get('YAM_COMPAT_ROOT',str(HERE)))
sys.path.insert(0,str(HERE))
HERE=HERE/args.run
HERE.mkdir(parents=True,exist_ok=False)
for n in [Path(__file__).name,'loop_chain.py','realistic_meshes_v32.py','drawstring_physics.py','drawstring_score.py','removal_assets.py','evaluate_removal.py','containment.py','plan_table_transfer.py','arena_task.py','removal_runtime.py']:(HERE/('source_'+n)).write_text((Path(__file__).resolve().parent/n).read_text())
app=None

def main():
    from inspect_robots import Scene as InspectScene
    from yam_arena.inspect_embodiment import YamArenaEmbodiment
    from types import SimpleNamespace
    inspect=YamArenaEmbodiment(task=str(Path(__file__).resolve().parent/'arena_task.py'),device=args.device,control_hz=20,cam_width=640,cam_height=360,object_jitter=0)
    inspect.reset(InspectScene(id='bin-removal-scripted',instruction=inspect.task.module.INSTRUCTION))
    import numpy as np
    import torch, trimesh
    import isaaclab.sim as sim
    from pxr import UsdGeom, Gf, Vt
    import omni.usd
    from isaaclab.sim.utils import clone
    from isaaclab.sim.spawners.meshes.meshes import _spawn_mesh_geom_from_mesh
    from isaaclab_physx.sim.schemas import PhysxCollisionCfg, PhysxDeformableBodyPropertiesCfg, PhysxRigidBodyPropertiesCfg, PhysxCollisionPropertiesCfg
    from isaaclab_physx.sim.spawners.materials import PhysxSurfaceDeformableBodyMaterialCfg
    from isaaclab_arena.assets.deformable_object import DeformableObject
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab_arena.scene.scene import Scene
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.embodiment import BimanualYamEmbodiment
    from yam_arena.scene import scene_assets, env_cfg_callback
    from yam_arena.constants import READY, LEFT_BASE_POS, RIGHT_BASE_POS
    from yam_arena.runtime import tcp_world, arm_q, link_poses
    from yam_arena.ik import solve
    from PIL import Image
    from removal_assets import build_assets,WALL_H,RIM_Z,BLOCK_COUNT
    from yam_arena import station
    from station_setup import rig as standard_rig, setup_manifest, save_views, setup_result
    env=inspect._env;base=env.unwrapped
    anchors=inspect.task.module.TRACKER.anchors;bead_count=20
    LEFT_BASE_POS,RIGHT_BASE_POS=station.BASES['left'],station.BASES['right']
    from yam_arena.ik import tcp_pose
    from scipy.spatial.transform import Rotation
    q_ready=[np.asarray(READY,dtype=float).copy() for _ in range(2)]
    initial_rotations=[Rotation.from_matrix(tcp_pose(q,b)[1]).as_quat().tolist() for q,b in zip(q_ready,[LEFT_BASE_POS,RIGHT_BASE_POS])]
    setup_manifest(HERE,SimpleNamespace(rig=inspect.rig),'bin',args.mode,Path(__file__).parent)
    manifest=json.loads((HERE/'setup_manifest.json').read_text());manifest['interface']='Inspect Robots YamArenaEmbodiment';(HERE/'setup_manifest.json').write_text(json.dumps(manifest,indent=2))
    (HERE/'fastener_vertex_indices.json').write_text(json.dumps(anchors,indent=2))
    robots=[base.scene.articulations[x] for x in ['left_robot','right_robot']]
    bags=base.scene.deformable_objects
    # Match Inspect Robots' 0.5-second physical settling before episode reset.
    # The cord's rest length belongs to that reset scene, not the authored pose.
    frames=HERE/'frames';frames.mkdir(exist_ok=True)
    
    # Robot and gripper keep the adapter's standard actuator settings.
    q=q_ready; grip=[1.,1.];count=0; logs=[]
    orientations=[v.copy() for v in initial_rotations]
    def tensor(value):return value.torch if hasattr(value,'torch') else value
    def arr(x):
        if hasattr(x,'torch'):x=x.torch
        return x.detach().cpu().numpy()
    def state(label):
        result={'commanded_joints':[x.tolist() for x in q], 'joint_limits':[arr(r.data.joint_pos_limits)[0].tolist() for r in robots], 'joints':[arr(r.data.joint_pos)[0].tolist() for r in robots], 'step':count,'phase':label,'tcp':[tcp_world(x).tolist() for x in robots], 'grip_command':list(grip)}
        for key,obj in bags.items():
            p=arr(obj.data.nodal_pos_w)[0]
            result[key]={'low':p.min(0).tolist(),'high':p.max(0).tolist(),'mean':p.mean(0).tolist()}
        np.savez_compressed(HERE/('state_%04d.npz'%count),loaded_bag=arr(bags['loaded_bag'].data.nodal_pos_w))
        result['rubbish']=[arr(base.scene.rigid_objects['rubbish_%d'%i].data.root_pos_w)[0].tolist() for i in range(BLOCK_COUNT)]
        result['loop_points']=[loop_links(k).tolist() for k in range(2)]
        result['loop_pinch_sites']=[loop_grip(k).tolist() for k in range(2)]
        result['loop_tcp_distance']=[loop_held_dist(k) for k in range(2)]
        from removal_runtime import finger_opening, finger_joints
        result['finger_opening']=[finger_opening(r) for r in robots];result['finger_joints']=[finger_joints(r) for r in robots]
        result['drawcord']=dict(cable.last)
        result['loaded_mouth_indices']=anchors['loaded_bag']
        result['rubbish_velocities']=[arr(base.scene.rigid_objects['rubbish_%d'%i].data.root_lin_vel_w)[0].tolist() for i in range(BLOCK_COUNT)]
        result['hem_marker_positions']=[body_pos('hem_%02d'%j).tolist() for j in range(bead_count)]
        result['gripper_poses']=[link_poses(r)['gripper'].tolist() for r in robots]
        result['rubbish_quaternions']=[arr(base.scene.rigid_objects['rubbish_%d'%i].data.root_quat_w)[0].tolist() for i in range(BLOCK_COUNT)]
        hem=arr(bags['loaded_bag'].data.nodal_pos_w)[0][anchors['loaded_bag']]
        result['mouth_diameter']=float(np.max(np.linalg.norm(hem[:,None,:]-hem[None,:,:],axis=2)))
        logs.append(result)
        if count%10==0 or label!='trajectory':
            (HERE/'trace.json').write_text(json.dumps(logs,indent=2));print('STATE',json.dumps(result),flush=True)
    def tick(steps=1):
        nonlocal count
        for _ in range(steps):
            action=torch.tensor([[*q[0],grip[0],*q[1],grip[1]]],device=base.device,dtype=torch.float32)
            result=inspect.step(SimpleNamespace(data=np.r_[q[0],grip[0],q[1],grip[1]]));count+=1
            if count%10==0:(HERE/'inspect_result.json').write_text(json.dumps({'step':count,'info':result.info,'terminated':result.terminated}))
            if count>4000:raise RuntimeError('Run limit exceeded')
            state('trajectory')
            if count%20==0:(HERE/'heartbeat.json').write_text(json.dumps({'step':count,'tcp':[tcp_world(r).tolist() for r in robots]}))
            if args.camera and count%4==0:
                im=arr(base.scene.sensors['overview_cam'].data.output['rgb'])[0,:,:,:3]
                Image.fromarray(im.astype('uint8')).save(frames/('%05d.png'%count))
    def move(targets,steps=50):
        origins=[tcp_world(x) for x in robots]
        from scipy.spatial.transform import Rotation,Slerp
        rotations=[Slerp([0,1],Rotation.from_quat([Rotation.from_matrix(link_poses(robots[k])["gripper"][:3,:3]).as_quat(),orientations[k]])) for k in range(2)]
        plan=[];seeds=[np.asarray(v).copy() for v in q]
        for step in range(1,steps+1):
            t=step/steps;t=t*t*t*(10+t*(-15+6*t))
            for k in range(2):
                target=origins[k]*(1-t)+np.array(targets[k])*t
                seeds[k],err,re=solve(target,rotations[k](t).as_quat(),q_init=seeds[k],base_pos=[LEFT_BASE_POS,RIGHT_BASE_POS][k],iters=80,tol=.0005)
                if err>.005 or re>.08:raise RuntimeError('Path rejected before movement: arm %d, %.6f m, %.6f rad'%(k,err,re))
            plan.append([v.copy() for v in seeds])
        for waypoint in plan:
            q[:]=waypoint;tick()
    def body_pos(name):return arr(base.scene.rigid_objects[name].data.root_pos_w)[0]
    # The strap loops are read from their visible link poses, as a camera would
    # see them. Pinch site: across both strands, 22 mm short of the bight.
    from loop_chain import _SPECS as CHAINS, chain_points
    from removal_assets import pinch_point, loop_axes, STRAND_GAP
    pinch_ids=[]
    for k,s in enumerate((1,-1)):
        c,_=CHAINS[f'loop_{k}'].initial_poses();u,nrm=loop_axes(s);p0=pinch_point(s)
        pinch_ids.append([int(np.argmin(np.linalg.norm(c[:,:2]-(p0+side*nrm*STRAND_GAP/2)[:2],axis=1))) for side in (1,-1)])
    def loop_links(k):return chain_points(env,f'loop_{k}')
    def loop_grip(k):return loop_links(k)[pinch_ids[k]].mean(0)
    def loop_held_dist(k):return float(np.linalg.norm(loop_links(k)-tcp_world(robots[k]),axis=1).min())
    def body_rot(name):
        from scipy.spatial.transform import Rotation
        return Rotation.from_quat(arr(base.scene.rigid_objects[name].data.root_quat_w)[0]).as_matrix()
    def nodes(name):return arr(bags[name].data.nodal_pos_w)[0]
    def current_targets():return [tcp_world(r).copy() for r in robots]
    def approach(targets,steps=60):
        # Solve the contact-free hover branch first; interpolating a half-turn
        # wrist rotation in Cartesian space can pass through an IK singularity.
        goals=[];start=[v.copy() for v in q]
        for k in range(2):
            qt,err,re=solve(targets[k],orientations[k],q_init=q[k],base_pos=[LEFT_BASE_POS,RIGHT_BASE_POS][k],iters=600)
            if err>.005 or re>.08:raise RuntimeError('Cord hover unreachable')
            goals.append(qt)
        for n in range(1,steps+1):
            t=n/steps;t=t*t*t*(10+t*(-15+6*t));q[:]=[a*(1-t)+b*t for a,b in zip(start,goals)];tick()
        tick(6)
        if any(np.linalg.norm(tcp_world(r)-t)>.02 for r,t in zip(robots,targets)):raise RuntimeError('Cord hover physically blocked')
    cable=inspect.task.module.TRACKER.runtime.cable
    def mouth_diameter():
        v=nodes('loaded_bag')[anchors['loaded_bag']]
        return float(np.max(np.linalg.norm(v[:,None,:]-v[None,:,:],axis=2)))
    # Scorer checks only (BIN_NEGATIVE): deliberately wrong strategies.
    NEGATIVE=os.environ.get('BIN_NEGATIVE','')
    class NegativeDone(Exception):pass
    def cinch_loaded(release=True):
        yaw=30.  # wrist yaw at the loops; keeps both pinches inside the arms' reach
        orientations[:]=[(Rotation.from_euler('z',s*yaw,degrees=True)*Rotation.from_quat([1.,0,0,0])).as_quat().tolist() for s in (-1,1)]
        direction=[np.array([0,0,-1.]),np.array([0,0,-1.])]
        # Put the strap near the distal pads. The TCP sits 13 mm behind their
        # tips, so centring the TCP on a strap lying on the mat drives the
        # fingers into the mat before they can close around it.
        offsets=[.003,.003]
        targets=[loop_grip(j)-direction[j]*offsets[j] for j in range(2)]
        grip[:]=[.55,.55]
        approach([t-direction[j]*.08 for j,t in enumerate(targets)],40);state('cord_hover')
        targets=[loop_grip(j)-direction[j]*offsets[j] for j in range(2)]
        move(targets,24)
        # Track the measured strap during closure; a fixed target can miss a settling strap.
        for i in range(16):
            u=(i+1)/16;u=u*u*u*(10+u*(-15+6*u))
            grip[:]=[g*(1-u) for g in [.55,.55]]
            now=current_targets()
            desired=[loop_grip(j)-direction[j]*offsets[j] for j in range(2)]
            for d in desired:d[2]=max(d[2],.764)
            bounded=[t+np.clip(d-t,-.002,.002) for t,d in zip(now,desired)]
            move(bounded,1)
        tick(5);state('cord_ends_grasped')
        before=np.array([loop_grip(k) for k in range(2)])
        move([t+[0,0,.05] for t in current_targets()],20);state('cord_grasp_probe')
        if NEGATIVE=='no_pull':
            # Scorer check only: hold both loops without pulling the string out.
            tick(200);state('negative_no_pull_hold');raise NegativeDone()
        after=np.array([loop_grip(k) for k in range(2)])
        if NEGATIVE!='film_lift' and np.min(after[:,2]-before[:,2])<.025:raise RuntimeError('Drawstring loop slipped during measured grasp probe')
        # Peel the folded cuff outward and above the rim before rotating the
        # wrists; rotating while under the brim can trap the tab between a
        # finger and the bin and strip it out of the grasp.
        peel=current_targets()
        peel[0]+=np.array([0,.02,.12]);peel[1]+=np.array([0,-.02,.12])
        move(peel,96);state('cuff_peeled_above_rim')
        for k in range(2):
            if NEGATIVE!='film_lift' and loop_held_dist(k)>.03:raise RuntimeError('Grasp lost during cuff peel on arm %d'%k)
        orientations[:]=[(Rotation.from_euler('y',-60,degrees=True)*Rotation.from_quat(r)).as_quat().tolist() for r in orientations]
        held_targets=current_targets()
        held_targets[0]+=np.array([.03,0,.04]);held_targets[1]+=np.array([.03,0,.04])
        move(held_targets,40);state('cuff_raised')
        # The original ten collinear 20-mm pulls become one continuous sweep.
        # Acceleration and velocity are zero at both ends; no repeated pauses.
        for j,(delta,angle,steps) in enumerate([
            ([[.070,-.090,.200],[.070,.090,.200]],100.,112),
            ([[0,-.030,0],[0,.030,0]],100.,50),
            ([[0,0,.105],[0,0,.105]],110.,160)]):
            orientations[:]=[(Rotation.from_euler('y',-angle,degrees=True)*Rotation.from_quat([1.,0,0,0])).as_quat().tolist() for _ in range(2)]
            held_targets=[t+np.asarray(d) for t,d in zip(held_targets,delta)]
            move(held_targets,steps)
            state('continuous_drawstring_pull_%d'%j)
            if NEGATIVE=='film_lift' and j==1:negative_film_lift()
            for k in range(2):
                if NEGATIVE!='film_lift' and loop_held_dist(k)>.025:raise RuntimeError('Measured strap grasp lost during pull on arm %d'%k)

        state('vertical_pre_cinch')
        if release:raise RuntimeError('Use full mode for the gather-and-lift controller')
    def negative_film_lift():
        """Scorer check only: cinch with the loops (pulls 0 and 1), then let go
        of both loops and lift the bag out by pinching its gathered neck film."""
        move([x+[0,0,-.06] for x in current_targets()],60);state('negative_slacken')
        for n in range(1,9):
            u=n/8;grip[:]=[u,u];tick()
        tick(40);state('negative_loops_released')
        # Left hand turns to a reachable forward-down pinch (checked offline);
        # the right hand keeps its pose.
        orientations[0]=(Rotation.from_euler('z',-30,degrees=True)*Rotation.from_euler('y',-80,degrees=True)*Rotation.from_quat([1.,0,0,0])).as_quat().tolist()
        neck=nodes('loaded_bag')[anchors['loaded_bag']].mean(0)
        axis=Rotation.from_quat(orientations[0]).apply([0,0,1.])
        approach([neck-[0,0,.025]-axis*.05,current_targets()[1]],80);state('negative_neck_hover')
        # Aim 25 mm below the hem, into the gathered film bundle of the neck.
        neck=nodes('loaded_bag')[anchors['loaded_bag']].mean(0)-[0,0,.025]
        move([neck+axis*.01,current_targets()[1]],40);state('negative_neck_reached')
        for n in range(1,17):
            u=n/16;grip[0]=1-u;tick()
        tick(5);state('negative_film_pinched')
        now=current_targets()
        move([now[0]+[0,0,.25],now[1]],200);tick(60);state('negative_film_lifted')
        raise NegativeDone()

    def full_removal():
        cinch_loaded(release=False)
        def wrist(pitch,yaws=(0.,0.)):
            orientations[:]=[(Rotation.from_euler('z',yaw,degrees=True)*Rotation.from_euler('y',-pitch,degrees=True)*Rotation.from_quat([1.,0,0,0])).as_quat().tolist() for yaw in yaws]
        wrist(110)
        now=current_targets()
        move([now[0]+[0,0,.02],now[1]+[0,0,.02]],30);state('converge_020')
        move([[.3226,.0458,1.3777],[.3226,-.0458,1.3777]],100);state('converged_lift')
        wrist(110,(-20.,20.))
        move(current_targets(),30);state('wrist_yaw_clearance')
        wrist(135,(-20.,20.))
        move([[.27,.06,1.49],[.27,-.06,1.49]],130);tick(20);state('high_lift_hold')
        wrist(145,(-20.,20.))
        move([[.24,.06,1.54],[.24,-.06,1.54]],80);tick(60);state('full_clearance_hold')
        # Visible check only (the controller never reads the scorer): the
        # hanging bag must have been above the rim during the hold (it bounces).
        if max(x['loaded_bag']['low'][2] for x in logs[-120:])<RIM_Z+.02:
            raise RuntimeError('Bag not lifted clear of the bin rim')
        from plan_table_transfer import plan
        # Closed loop on measured state: lower the hands until the hanging
        # bag's bottom is just above the tabletop, beside the bin.
        bag=nodes('loaded_bag');hang=float(np.mean([x[2] for x in current_targets()])-bag[:,2].min())
        command,report=plan(logs[-1],steps=240,final_z=.7475+.012+hang,final_x=-.03)
        report['hang_depth_m']=hang
        (HERE/'table_transfer_plan.json').write_text(json.dumps(command))
        (HERE/'table_transfer_preflight.json').write_text(json.dumps(report,indent=2))
        for op in command['operations']:
            orientations[:]=op['orientations'];move(op['targets'],op['steps'])
        state('set_down_on_table');tick(10)
        # The bag now stands on the table and the strings are slack. Lower the
        # loops onto the gathered neck before opening, as a person would, so
        # they are not dropped from carrying height.
        top=float(nodes('loaded_bag')[:,2].max())
        # Lower all the way to just above the neck so the strings go slack and
        # the straps can drop out of the opened fingers.
        for clear in (.06,.10,.15,.20):
            try:move([np.r_[x[:2],max(x[2]-.45,top+clear)] for x in current_targets()],80);break
            except RuntimeError as exc:
                if 'Path rejected before movement' not in str(exc):raise
        tick(10);state('loops_lowered')
        for n in range(1,9):
            u=n/8;u=u*u*u*(10+u*(-15+6*u));grip[:]=[u,u];tick()
        state('loops_released');tick(10)
        # The open fingers point across the bag, so a released strap can hang
        # over the lower finger like a towel on a rail. Draw each hand straight
        # back along its approach axis first so the strap slides off the tip.
        # A strap can lie across the tip of the lower finger: first lower the
        # open hands so the strap settles on the bag, then draw them back.
        try:move([x+[0,0,-.025] for x in current_targets()],30)
        except RuntimeError as exc:
            if 'Path rejected before movement' not in str(exc):raise
        tick(10);state('hands_lowered_open')
        axes=[link_poses(r)['gripper'][:3,2] for r in robots]
        for back in (.07,.05,.03):
            try:move([x-a*back for x,a in zip(current_targets(),axes)],40);break
            except RuntimeError as exc:
                if 'Path rejected before movement' not in str(exc):raise
        state('hands_withdrawn')
        # Retreat straight up; if the arm cannot reach the full height from this
        # pose (checked before any motion), retreat less.
        for rise in (.15,.10,.06,.03):
            try:move([x+[0,0,rise] for x in current_targets()],30);break
            except RuntimeError as exc:
                if 'Path rejected before movement' not in str(exc):raise
        state('hands_retreated')
        tick(80);state('table_landing_settled')
    (HERE/'task_parameters.json').write_text(json.dumps({'bin_center_xy':[.22,0.],'bin_top_z':RIM_Z,'bin_outer_radius_m':.0855,'control_hz':20.,'clear_hold_seconds':2.,'sample_interval_steps':1,'mode':args.mode,'require_table_deposit':True}))
    state('initial')
    try:
        tick(20);state('settled')
        # Fail a numerically damaged reset before attempting manipulation.
        from realistic_meshes_v32 import dirty_mesh
        rv,rf,_=dirty_mesh();reference=trimesh.Trimesh(vertices=rv,faces=rf,process=False)
        edge=reference.edges_unique;rest=np.linalg.norm(reference.vertices[edge[:,1]]-reference.vertices[edge[:,0]],axis=1)
        current=nodes('loaded_bag')
        if np.max(np.linalg.norm(current[edge[:,1]]-current[edge[:,0]],axis=1)/rest)>1.5:raise RuntimeError('Unstable initial film geometry; trial invalid before manipulation')
        if args.camera:save_views(base,HERE)
        if args.mode=='setup':setup_result(HERE,count)
        if args.mode=='interactive':
            import time
            while True:
                command=HERE/'command.json'
                if not command.exists():time.sleep(.1);continue
                request=json.loads(command.read_text());command.unlink()
                if request.get('quit'):break
                try:
                    for op in request.get('operations',[]):
                        if op.get('routine')=='cinch':cinch_loaded();continue
                        if op.get('routine')=='full':full_removal();continue
                        if 'orientations' in op:orientations[:]=op['orientations']
                        if 'grip' in op:grip[:]=op['grip']
                        if 'targets' in op:move(op['targets'],op.get('steps',50))
                        elif 'delta' in op:move([t+d for t,d in zip(current_targets(),np.array(op['delta']))],op.get('steps',50))
                        if 'tick' in op:tick(op['tick'])
                    state(request['label'])
                except Exception as exc:
                    state(request['label']+'_failed');(HERE/'command_error.json').write_text(json.dumps({'label':request['label'],'failure':str(exc)}))
        elif args.mode=='cinch':cinch_loaded()
        elif args.mode=='full':
            try:full_removal()
            except NegativeDone:
                state('negative_done');(HERE/'negative.json').write_text(json.dumps({'negative':NEGATIVE,'live_result':inspect.task.module.TRACKER.result}))
        (HERE/'completed.json').write_text(json.dumps({'completed':True,'mode':args.mode,'full_task_success':False,'score':None,'reason':'Requires independent evaluation; completion alone does not establish success'}))
    except Exception as exc:
        state('sequence_failed');(HERE/'completed.json').write_text(json.dumps({'completed':False,'failure':str(exc),'score':None}));raise
    inspect.close()

try:main()
except Exception:
    (HERE/'trial_error.txt').write_text(traceback.format_exc());traceback.print_exc();raise
finally:
    if app is not None:app.close()


