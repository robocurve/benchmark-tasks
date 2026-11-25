"""Run a fixed YAM task with Inspect Robots and record the actual world and grade."""
import argparse
import hashlib
from importlib.metadata import PackageNotFoundError, version
import json
import math
import os
from pathlib import Path
import re
import time


def collision_mesh_profile(vertices, counts, indices, samples):
    """Intersect vertical probe lines with the live collider's convex mesh faces."""
    import numpy as np

    if not len(vertices) or sum(counts)!=len(indices) or min(indices,default=-1)<0 or max(indices)>=len(vertices):
        raise ValueError('Invalid authored collision mesh')
    triangles=[]
    cursor=0
    for count in counts:
        face=indices[cursor:cursor+count]
        triangles.extend(vertices[[face[0],face[i],face[i+1]]] for i in range(1,len(face)-1))
        cursor+=count
    surfaces=[]
    for x,y in samples:
        heights=[]
        for triangle in triangles:
            a,b,c=triangle
            edge1,edge2=b-a,c-a
            matrix=np.column_stack((edge1[:2],edge2[:2]))
            if abs(np.linalg.det(matrix))<1e-15:
                continue
            u,v=np.linalg.solve(matrix,np.array([x,y])-a[:2])
            if u>=-1e-7 and v>=-1e-7 and u+v<=1+1e-7:
                heights.append(float(a[2]+u*edge1[2]+v*edge2[2]))
        surfaces.append(max(heights) if heights else None)
    return surfaces


def run(task, mode, out, bridge=None, seed=0, task_id=None, stop_after=None):
    import imageio.v2 as imageio
    import numpy as np
    from PIL import Image
    from inspect_robots import Action, ActionChunk, Scene, eval as inspect_eval
    from inspect_robots.guard import ClampGuard, DeltaLimitGuard, GuardChain
    from inspect_robots.logging.sink import NullSink
    from inspect_robots.policy import PolicyBase, PolicyInfo
    from inspect_robots.task import Task
    from task_impl.adapter import make_embodiment, make_scripted
    from task_impl.grading import make_scorer
    from yam_arena.station import TABLE_TOP
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.urdf import quat_xyzw_to_matrix
    from scipy.spatial.transform import Rotation
    import itertools

    card = json.loads((task / 'task.json').read_text())
    limit = int(re.match(r'([0-9,]+)\s+steps?\b', card['run_limit'], re.I)[1].replace(',', ''))
    # Slow settling and multi-second hold rules can reward waiting late in a trial.
    # Give the stationary adversarial baseline the same horizon as both active policies.
    if stop_after is not None and (mode!='scripted' or not 1<=stop_after<=limit):
        raise ValueError('A diagnostic step cap applies only to a bounded scripted replay')
    horizon = limit if stop_after is None else stop_after
    out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    scorer = make_scorer()
    bench_poses = [('bench_top', (0.354, 0, 0.316), (0.405, 0, 0.025), 34.0), ('bench_l', (0.337, 0.177, 0.062), (0.405, 0.055, 0.022), 43.0), ('bench_r', (0.337, -0.177, 0.062), (0.405, -0.055, 0.022), 43.0), ('bench_oblique', (0.182, -0.18, 0.14), (0.402, -0.006, 0.026), 38.0)]

    # Use the station's own Isaac Lab CameraCfg and renderer. The policy and
    # Inspect recorder continue to read only top_cam, left_cam and right_cam.
    import atexit, ctypes
    import yam_arena.inspect_embodiment as runtime
    if runtime._APP is None:
        ctypes.CDLL("libnvidia-ngx.so.1")
        from isaaclab.app import AppLauncher
        runtime._APP = AppLauncher(headless=True, device="cuda:0", enable_cameras=True,
            renderer="RealTimePathTracing", anti_aliasing=4,
            kit_args="--/persistent/rtx/modes/rt2/enabled=true --/persistent/rtx/modes/rt/enabled=false "
                     "--/persistent/rtx/modes/pt/enabled=false").app
        atexit.register(runtime._close_app_at_exit)
    from yam_arena import embodiment as yam
    base_camera_cfg=yam.BimanualYamEmbodiment._camera_cfg

    def configured_bench_cameras(owner, height, width, overview):
        default=base_camera_cfg(owner,height,width,overview)
        fields=[]
        for key, eye, target, focal in bench_poses:
            z=np.asarray(eye)-np.asarray(target)
            z=z/np.linalg.norm(z)
            x=np.cross([0.,0.,1.],z)
            x=x/np.linalg.norm(x)
            quat=Rotation.from_matrix(np.column_stack((x,np.cross(z,x),z))).as_quat()
            position=(eye[0],eye[1],float(TABLE_TOP)+eye[2])
            cfg=yam.CameraCfg(
                prim_path="{ENV_REGEX_NS}/"+key, height=180, width=384,
                data_types=["rgb"], update_latest_camera_pose=True,
                spawn=yam.sim_utils.PinholeCameraCfg(focal_length=focal, horizontal_aperture=20.955, clipping_range=(.005,20.)),
                offset=yam.CameraCfg.OffsetCfg(pos=position,rot=tuple(quat.tolist()),convention="opengl"),
                renderer_cfg=yam.RENDERER)
            fields.append((key,yam.CameraCfg,cfg))
        return yam.make_configclass("BenchObserverCamerasCfg",fields,bases=(type(default),))()

    yam.BimanualYamEmbodiment._camera_cfg=configured_bench_cameras
    embodiment = make_embodiment()
    gauge_bodies={entry[key] for entry in embodiment.spec['metrics'] if entry['kind']=='hex_key_blocked'
                  for key in ('a','b')}
    def gauge_poses():
        return {body:{'positionM':embodiment.frames[body][0].tolist(),'rotation':embodiment.frames[body][1].tolist()}
                for body in sorted(gauge_bodies)}

    def wedge_probes():
        measured={}
        mesh_prims=None
        for actor in embodiment.spec['actors']:
            for index,part in enumerate(actor['parts']):
                if part['type']!='wedge':
                    continue
                from omni.physx import get_physx_scene_query_interface
                import carb
                center,rot,_=embodiment.frames[actor['name']]
                origin=center+rot@part['offset']+[0.,0.,float(TABLE_TOP)]
                basis=rot@Rotation.from_euler('xyz',part['euler_deg'],degrees=True).as_matrix()
                _,length,height=part['size'];rays=[]
                suffix='/Actors/'+actor['name']+'/part_'+str(index)
                for fraction in (-.3,.3):
                    start=origin+basis@np.array([0.,length*fraction,height+.02])
                    points=[]
                    def hit(value):
                        if str(value.collision).endswith(suffix):
                            points.append([float(v) for v in value.position])
                        return True
                    get_physx_scene_query_interface().raycast_all(carb.Float3(*start),carb.Float3(*(-basis[:,2])),
                                                                  float(2*height+.05),hit,True)
                    local=[float(((np.asarray(p)-origin)@basis)[2]) for p in points]
                    rays.append({'lengthFraction':fraction,'surfaceHeightM':max(local) if local else None})
                method='PhysX raycast at reset'
                if any(ray['surfaceHeightM'] is None for ray in rays):
                    # Contact reports can be initialized while reset-time scene queries
                    # still omit entire cloned links. Measure the live collision mesh,
                    # including its composed transform, instead of the fixture recipe.
                    from omni.usd import get_context
                    from pxr import Gf,UsdGeom,UsdPhysics
                    if mesh_prims is None:
                        mesh_prims=[p for p in get_context().get_stage().Traverse()
                                    if p.IsA(UsdGeom.Mesh) and p.HasAPI(UsdPhysics.CollisionAPI)
                                    and p.HasAPI(UsdPhysics.MeshCollisionAPI)]
                    matches=[p for p in mesh_prims if str(p.GetPath()).endswith(suffix)]
                    env=[p for p in matches if '/env_0/' in str(p.GetPath())]
                    candidates=env or matches
                    if len(candidates)==1:
                        prim=candidates[0]
                        convex=UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()=='convexHull'
                        enabled=UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is True
                        if convex and enabled:
                            mesh=UsdGeom.Mesh(prim)
                            transform=UsdGeom.XformCache().GetLocalToWorldTransform(prim)
                            points=np.asarray([tuple(transform.Transform(Gf.Vec3d(*point)))
                                               for point in mesh.GetPointsAttr().Get()])
                            vertices=(points-origin)@basis
                            heights=collision_mesh_profile(vertices,list(mesh.GetFaceVertexCountsAttr().Get()),
                                list(mesh.GetFaceVertexIndicesAttr().Get()),[(0.,length*r['lengthFraction']) for r in rays])
                            for ray,mesh_height in zip(rays,heights,strict=True):
                                if ray['surfaceHeightM'] is None:
                                    ray['surfaceHeightM']=mesh_height
                            method='PhysX raycast and composed USD convex collider'
                measured[f"{actor['name']}:{index}"]={'bodyPositionM':center.tolist(),'rays':rays,'method':method}
        return measured

    def physical_gauges():
        entries=[m for m in embodiment.spec['metrics']
                 if m['kind'] in ('pin_bore_clearance','slot_throat_margin')]
        if not entries:return {},{}
        from omni.physx import get_physx_scene_query_interface
        import carb
        query=get_physx_scene_query_interface()
        parts={a['name']:a['parts'] for a in embodiment.spec['actors']}
        def distance(body,part,local,direction,length):
            center,rot,_=embodiment.frames[body]
            start=center+rot@local+[0.,0.,float(TABLE_TOP)]
            direction=rot@direction
            results=[]
            def hit(v):
                if str(v.collision).endswith('/Actors/'+body+'/part_'+str(part)):
                    results.append(float((np.asarray(list(v.position))-start)@direction))
                return True
            query.raycast_all(carb.Float3(*start),carb.Float3(*direction),float(length),hit,True)
            return min(results) if results else None
        observed={}
        slots={}
        for spec in entries:
            if spec['kind']=='slot_throat_margin':
                width=1-spec['depth_axis']
                axis=np.eye(3)[width]
                body=spec['a']
                center=np.asarray(spec['pa'])
                distances=[distance(body,i,center,sign*axis,.04) for i,sign in zip(spec['sides'],(-1,1))]
                body=spec['b'];i=spec['web_part']; center=np.asarray(spec['pb'])
                normal=np.array([0.,0.,1.])
                span=.006
                hits=[distance(body,i,center+sign*span*normal,-sign*normal,2*span) for sign in (-1,1)]
                slots[spec['name']]={'sideDistancesM':distances,
                    'webThicknessM':2*span-sum(hits) if None not in hits else None}
                continue
            body=spec['a']; origin=np.asarray(spec['pa']); distances=[]
            for i in spec['wall_parts']:
                wall=parts[body][i]
                radial=np.asarray(wall['offset'])-origin
                radial[2]=0.
                direction=radial/np.linalg.norm(radial)
                distances.append(distance(body,i,origin,direction,.04))
            body=spec['b'];i=spec['pin_part']; pin=parts[body][i]
            axis=Rotation.from_euler('xyz',pin['euler_deg'],degrees=True).as_matrix()[:,0]
            center=np.asarray(pin['offset'])
            halfspan=pin['size'][0]
            hits=[distance(body,i,center+sign*halfspan*axis,-sign*axis,2*halfspan) for sign in (-1,1)]
            observed[spec['name']]={'innerFlatRadiiM':distances,
                'shankDiameterM':2*halfspan-sum(hits) if None not in hits else None}
        return observed,slots

    def fixture_projection(width,height):
        cam=embodiment.rig.top_cam
        if cam.convention!='opengl':
            raise RuntimeError('Unrecognized station camera convention')
        lens=cam.lens.resized(width,height)
        basis=quat_xyzw_to_matrix(np.asarray(cam.rot_xyzw))
        bounds={}
        marks={}
        for body in embodiment.spec['actors']:
            center,rot,_=embodiment.frames[body['name']]
            projected=[]
            solids=body['parts']
            for index,part in enumerate(solids+body.get('visuals',[])):
                corners=np.array(list(itertools.product((-0.5,0.5),repeat=3)))*part['size']
                local=Rotation.from_euler('xyz',part['euler_deg'],degrees=True).apply(corners)+part['offset']
                world=local@rot.T+center+[0.,0.,float(TABLE_TOP)]
                optical=(world-cam.pos)@basis
                if np.any(optical[:,2]>=-0.01):
                    raise ValueError('Fixture geometry behind camera: '+body['name'])
                points=np.c_[lens.cx-lens.fx*optical[:,0]/optical[:,2],
                             lens.cy+lens.fy*optical[:,1]/optical[:,2]]
                in_frame=np.minimum(points.max(axis=0),[width,height])-np.maximum(points.min(axis=0),[0.,0.])
                if np.any(in_frame<=0):
                    raise ValueError(f"Fixture component outside setup camera: {body['name']} part {index}")
                if index<len(solids):
                    projected.extend(points)
                else:
                    mark=f"{body['name']}:{index-len(solids)}"
                    marks[mark]=[round(float(n),1) for n in (*points.min(axis=0),*points.max(axis=0))]
            lo=np.min(projected,axis=0);hi=np.max(projected,axis=0)
            visible=np.minimum(hi,[width,height])-np.maximum(lo,[0.,0.])
            if np.any(visible<1.0):
                raise ValueError('Fixture outside setup camera or below one pixel: '+body['name'])
            bounds[body['name']]=[round(float(n),1) for n in (*lo,*hi)]
        return bounds,marks

    class Idle(PolicyBase):
        info = PolicyInfo('hold_initial_joints', embodiment.info.action_space, control_hz=20.)
        held = None

        def act(self, observation):
            if self.held is None:
                self.held = np.array(observation.state['joint_pos'], copy=True)
            return ActionChunk([Action(self.held)], control_hz=20.)

    class WindowCameras:
        def __init__(self):
            self.valid = [0] * len(bench_poses)

        def views(self):
            result = []
            for i, (key, eye, target, focal) in enumerate(bench_poses):
                sensor = embodiment._base.scene[key]
                pixels = sensor.data.output["rgb"]
                pixels = pixels.torch if hasattr(pixels, "torch") else pixels
                im = np.ascontiguousarray(pixels[0, ..., :3].cpu().numpy(), dtype=np.uint8)
                if im.shape != (180,384,3):
                    raise ValueError("Invalid native bench camera resolution: " + key)
                if np.percentile(im,90,axis=(0,1)).max() - np.percentile(im,10,axis=(0,1)).min() > 24 and im.std(axis=(0,1)).max() > 8:
                    self.valid[i] += 1
                result.append(Image.fromarray(im))
            return result

    class Record(NullSink):
        critical = True
        steps = 0
        frames = 0

        def capture(self, obs, initial=False):
            cams = [np.asarray(obs.images[k]) for k in ('top_cam', 'left_cam', 'right_cam')]
            if any(c.dtype != np.uint8 or c.ndim != 3 or c.shape[-1] != 3 for c in cams):
                raise ValueError('Expected RGB uint8 arrays from all three YAM cameras')
            if stop_after is not None and self.steps in (0, 600, 655, 660, 700, 730, 735, 810, horizon):
                Image.fromarray(cams[0]).save(out / f'policy-overview-{self.steps}.png')
            native=embodiment._base.scene['top_cam'].data.output['rgb']
            native=native.torch if hasattr(native,'torch') else native
            overhead=np.ascontiguousarray(native[0,...,:3].cpu().numpy(),dtype=np.uint8)
            height,width=overhead.shape[:2]
            if initial:
                if np.std(overhead) < 3:
                    raise ValueError('Overhead camera is empty')
                Image.fromarray(overhead).save(out / 'setup.png')
                self.projected,self.ink_bounds=fixture_projection(width,height)
                self.dimensions={'nativeOverview':[width,height],'policyRGB':[cams[0].shape[1],cams[0].shape[0]]}
                if stop_after is not None:
                    cam = embodiment.rig.top_cam
                    lens = cam.lens.resized(width,height)
                    self.overview_camera = {
                        'positionM':list(cam.pos),'rotationXYZW':list(cam.rot_xyzw),
                        'convention':cam.convention,'tableTopM':float(TABLE_TOP),
                        'intrinsics':[float(lens.fx),float(lens.fy),float(lens.cx),float(lens.cy)]}
            # The robot still receives the original three rig images. The two
            # additional live RTX products are read only for the referee video.
            if not hasattr(self, "inspection"):
                self.inspection = WindowCameras()
            details = self.inspection.views()
            frame = Image.new('RGB', (width+cams[1].shape[1]+768,height))
            frame.paste(Image.fromarray(overhead))
            for i, image in enumerate(cams[1:]):
                frame.paste(Image.fromarray(image), (width, (height//2)*i))
            for i, detail in enumerate(details):
                frame.paste(detail, (width+cams[1].shape[1]+384*(i//2), (height//2)*(i%2)))
            from PIL import ImageDraw
            draw=ImageDraw.Draw(frame)
            for i, label in enumerate(('Lens coverage', 'Left returns', 'Right returns', 'Hands and frame')):
                x,y=width+cams[1].shape[1]+384*(i//2)+3, (height//2)*(i%2)+3
                draw.text((x,y), label, fill="white", stroke_width=1, stroke_fill="black")
            if self.steps in (100, 300, 450, 500, horizon):
                frame.save(out/("observer-frame-%d.png" % self.steps))
            if self.steps == horizon or (self.steps > 100 and self.steps % 100 == 0):
                if min(self.inspection.valid) < 1:
                    raise RuntimeError("A native referee camera did not supply RGB frames")
            self.video.append_data(np.asarray(frame))
            self.frames+=1

        def log_step(self, t, observation, action, result):
            if t != self.steps:
                raise ValueError('Nonconsecutive Inspect Robots control steps')
            grade = dict(result.info['grade'])
            value = float(grade['score'])
            if not math.isfinite(value) or not 0 <= value <= 1:
                raise ValueError('Grader produced invalid score')
            self.steps += 1
            if self.steps % 2 == 0 or result.terminated or self.steps==horizon:
                self.capture(result.observation)
            measured=np.asarray(result.observation.state['joint_pos'])[[6,13]]
            jaws=(measured*(2*abs(float(FINGER_OPEN)))).round(6).tolist()
            row={'step':self.steps, 'grade':grade,'jawGapM':jaws}
            if stop_after is not None:
                # Development telemetry stays outside both the observation and the scorer.
                # Keeping it on waypoint transitions and every fifth tick makes slip
                # distinguishable from missed contact without large GPU-to-host transfers.
                waypoint = grade.get('demo', {}).get('waypoint')
                if self.steps == 1 or self.steps % 5 == 0 or waypoint != getattr(self, 'last_waypoint', None):
                    row['developmentPoses'] = {
                        name: {'xyzM': np.round(p, 6).tolist(), 'rotation': np.round(r, 5).tolist()}
                        for name, (p, r, _) in embodiment.frames.items() if name != 'table'
                    }
                self.last_waypoint = waypoint
            if gauge_bodies:
                row['gaugePoses']=gauge_poses()
            self.trace.write(json.dumps(row, allow_nan=False,
                                        default=lambda x:x.tolist())+'\n')
            self.final = value

    record = Record()
    unrecorded_reset=embodiment.reset
    def reset_and_record(scene,*,seed=None):
        obs=unrecorded_reset(scene,seed=seed)
        joints=np.asarray(obs.state['joint_pos'],dtype=float).reshape(2,7)
        record.initial_robot={arm:{'jointRadians':values[:6].tolist(),'normalizedJaw':float(values[6]),
                                  'jawGapM':float(values[6])*2*abs(float(FINGER_OPEN))}
                              for arm,values in zip(('left','right'),joints)}
        record.reset_measurements=embodiment._measure(obs)
        record.reset_gauge_poses=gauge_poses()
        record.initial_shim=wedge_probes()
        record.initial_bores,record.initial_slots=physical_gauges()
        record.angular_damping={}
        configured={actor['name']:actor['angular_damping_s_inv'] for actor in embodiment.spec['actors']
                    if 'angular_damping_s_inv' in actor}
        if configured:
            import omni.usd
            stage=omni.usd.get_context().get_stage()
            for name,value in configured.items():
                path=embodiment.bodies.prim_paths[embodiment.rows[name]]
                measured=stage.GetPrimAtPath(path).GetAttribute('physxRigidBody:angularDamping').Get()
                if measured is None or abs(float(measured)-value)>1e-6:
                    raise ValueError('Native angular damping differs from the declared value for '+name)
                record.angular_damping[name]=float(measured)
        record.capture(obs,initial=True)
        return obs
    embodiment.reset=reset_and_record
    if mode == 'dream':
        from inspect_robots_dream.policy import DreamPolicy
        policy = DreamPolicy(model='dreamlabs/dream-1-beta', server_url=bridge, record=False, timeout_s=65.)
    else:
        policy = make_scripted() if mode == 'scripted' else Idle()
    definition = Task(task_id or task.name, scenes=[Scene(id='fixed-start',instruction=card['instruction'])],
                      scorer=scorer, max_steps=horizon)
    space = embodiment.info.action_space
    guards = GuardChain(ClampGuard(space), DeltaLimitGuard(space,max_delta=1.),names=['clamp','delta-limit'])
    try:
        with (out/'trace.jsonl').open('x') as record.trace, imageio.get_writer(
                str(out/'trial.mp4'), fps=10, codec='libx264', pixelformat='yuv420p', macro_block_size=1,
                output_params=['-movflags','+faststart']) as record.video:
            log = inspect_eval(definition,policy,embodiment,log_dir=str(out/'inspect'),sinks=[record],
                               guard=guards,seed=seed,store_frames=False,store_records=True,fail_on_error=True)[0]
        if log.status != 'success' or len(log.samples) != 1:
            raise RuntimeError('Inspect Robots did not complete one scene')
        final = float(log.samples[0].reduced[scorer.name])
        if not record.steps or record.final != final or record.steps > horizon:
            raise ValueError('Inspect scorer and physical trace disagree')
        versions={}
        for package in ('isaacsim','isaaclab','isaaclab-arena','inspect-robots','yam-arena','inspect-robots-dream','dreamlabs-client'):
            try:versions[package]=version(package)
            except PackageNotFoundError:pass
        report = {'task':task_id or task.name,'policy':mode,'seed':seed,'model':'dreamlabs/dream-1-beta' if mode=='dream' else None,
                  'scorer':scorer.name,'score':final,'steps':record.steps,'step_limit':horizon,
                  'declaredStepLimit':limit,'diagnosticOnly':stop_after is not None,
                  'elapsed_s':round(time.monotonic()-started,2),'control_hz':20,
                  'initialBoundsPixels':record.projected,'initialMarkBoundsPixels':record.ink_bounds,
                  'initialMeasurements':record.reset_measurements,
                  'initialRobotBeforeAction':record.initial_robot,
                  'initialGaugePoses':record.reset_gauge_poses,
                  'initialWedgeColliderProbes':record.initial_shim,
                  'initialBoreColliderProbes':record.initial_bores,
                  'initialSlotColliderProbes':record.initial_slots,
                  'configuredAngularDampingSInv':record.angular_damping,
                  'finalWedgeBodyPositions':{k.split(':')[0]:embodiment.frames[k.split(':')[0]][0].tolist()
                                             for k in record.initial_shim},
                  'video_frames':record.frames,'cameraPixels':record.dimensions,
                  'nativeVersions':versions,'gpu':embodiment._torch.cuda.get_device_name(0),
                  'observedSourceHashes':{path.relative_to(task).as_posix():hashlib.sha256(path.read_bytes()).hexdigest()
                       for path in [task/'task_impl/fixture.json',*sorted((task/'task_impl').glob('*.py'))]
                       if path.name!='run.py'},
                  'inspect_robots':version('inspect-robots'),'yam_arena':version('yam-arena')}
        if stop_after is not None:
            report['overviewCamera'] = record.overview_camera
        (out/'result.json').write_text(json.dumps(report, allow_nan=False, indent=2))
    finally:
        embodiment.close()
        if hasattr(policy,'close'):
            policy.close()


if __name__ == '__main__':
    import sys
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=('idle','scripted','dream'))
    source=Path(__file__).resolve().parent
    parser.add_argument('--task',type=Path,default=source.parent if source.name=='task_impl' else source)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--task-id',help='Stable task slug when the task is staged in a temporary directory')
    parser.add_argument('--bridge',help='Optional official Dream bridge URL')
    parser.add_argument('--stop-after',type=int,help='Optional shorter scripted diagnostic; excluded from qualification')
    args = parser.parse_args()
    sys.path.insert(0,str(args.task.resolve()))
    run(args.task.resolve(),args.mode,args.out.resolve(),args.bridge,task_id=args.task_id,stop_after=args.stop_after)
