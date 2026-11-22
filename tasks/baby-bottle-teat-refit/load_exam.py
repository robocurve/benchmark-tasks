"""After policy release, apply a bounded lifting force to the collar alone."""
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from geometry import TABLE
from scoring import SEATING_GAP_MM, evaluate, flange_gaps, inspect_pair, pad_clearance, teat_fit

HZ = 240
RISE_STEPS = 240
SETTLE_STEPS = 120
HOLD_STEPS = 240
HEIGHT = .040


class ObstructedLift(RuntimeError):
    """The released robot was in the evaluator's lifting path."""


def grade(data):
    """Replay the terminal second of the evaluator, with the original mechanical rubric."""
    end = len(data['poses'])
    total = RISE_STEPS+SETTLE_STEPS+HOLD_STEPS
    if (end != 1+total//12 or len(data['robotcounts']) != total
            or len(data['subposes']) != total or len(data['arm_poses']) != end
            or len(data['nodes']) != end or len(data['forces']) != total):
        raise ValueError("Incomplete collar proof")
    for key in ('poses','nodes','arm_poses','robotcounts','subposes','forces'):
        if not np.isfinite(data[key]).all():
            raise ValueError(f"Invalid proof data in {key}")
    if data['robotcounts'].any() or pad_clearance(data['nodes'], data['arm_poses'],
            data['arm_paths'], np.arange(end), data['tets']) <= 0:
        raise ObstructedLift("Evaluator lift obstructed by the robot")
    window = dict(poses=data['poses'][-21:],soft=data['nodes'][-21:],
                  tips=data['arm_poses'][-21:],contacts=data['robotcounts'][-240:],
                  paths=data['arm_paths'],grid=data['grid'],tets=data['tets'],
                  names=list(data['names']))
    fit = evaluate(window)
    bottle = data['subposes'][-240:,window['names'].index('bottle')]
    z_axis = Rotation.from_quat(bottle[:,3:]).as_matrix()[:,2]
    # Conservative bottom of the full 71 mm x 60 mm bottle envelope in any orientation.
    bottom = bottle[:,2] + np.minimum(0.,.071*z_axis[:,2]) - .030*np.linalg.norm(z_axis[:,:2],axis=1)
    clear = float(np.min(bottom-TABLE))
    bottle_lift = float(np.min(bottle[:,2])-data['poses'][0,window['names'].index('bottle'),2])
    if not np.isfinite([clear,bottle_lift]).all():
        raise ValueError("Invalid bottle motion")
    collar = window['names'].index('collar')
    captured, exposed = teat_fit(data['nodes'], data['poses'][:, collar], data['grid'])
    collar_rise = float(data['subposes'][-HZ:, collar, 2].min() - data['poses'][0, collar, 2])
    teat_bottom = data['nodes'][:, :, 2].min(axis=1)
    teat_rise = float(teat_bottom[-21:].min() - teat_bottom[0])
    teat_gap = float(teat_bottom[-21:].min() - TABLE)
    pair = captured.all() and exposed.all() and min(collar_rise, teat_rise, teat_gap) >= .020
    full = False
    if pair and fit['score'] == 1.0 and clear >= .020 and bottle_lift >= .020:
        # A later reseat cannot erase an earlier opening or loss of thread engagement.
        body = window['names'].index('bottle')
        gaps = flange_gaps(data['poses'][:, body], data['nodes'], data['grid'], data['tets'])
        if (np.abs(gaps) <= SEATING_GAP_MM).all():
            full = all(
                lug['under_fraction'] >= 12/15
                for poses in data['subposes']
                for lug in inspect_pair(poses[collar], poses[body])['lugs']
            )
    score = 1.0 if full else (0.5 if pair else 0.0)
    return dict(**{k:v for k,v in fit.items() if k!='score'}, score=score,
                minimum_table_gap_mm=clear*1000, minimum_bottle_rise_mm=bottle_lift*1000,
                minimum_collar_rise_mm=1000*collar_rise, minimum_teat_rise_mm=1000*teat_rise,
                minimum_teat_table_gap_mm=1000*teat_gap,
                collar_sectors_min_during_lift=int(captured.sum(axis=1).min()),
                outlet_exposed_during_lift=bool(exposed.all()))


def run(m,out):
    import torch
    root = Path(out)
    names = [p.rsplit('/',1)[-1] for p in m.objects]
    collar = m.view.create_rigid_body_view(m.objects[names.index('collar')])
    initial_pose = collar.get_transforms()[0].clone()
    start = initial_pose[:3]
    initial_rotation = Rotation.from_quat(initial_pose[3:].cpu().numpy())
    idx = torch.zeros(1,device=start.device,dtype=torch.int32)
    rows,softs,arms,contacts,subposes,forces = [],[],[],[],[],[]
    def capture():
        rows.append(m.rigid.get_transforms().cpu().numpy().copy())
        softs.append(m.soft.get_simulation_nodal_positions()[0].cpu().numpy()[m.order])
        arms.append(m.arm_bodies.get_transforms().cpu().numpy().copy())
    capture()
    for step in range(1,RISE_STEPS+SETTLE_STEPS+HOLD_STEPS+1):
        t = min(1., step/RISE_STEPS)
        target = start.clone()
        target[2] += HEIGHT*t*t*(3.-2.*t)
        vel = collar.get_velocities()[0,:3]
        force = 480.*(target-collar.get_transforms()[0,:3])-3.*vel
        force[2] += 3.*HEIGHT*6.*t*(1.-t)*HZ/RISE_STEPS
        force = force * torch.clamp(3./torch.linalg.vector_norm(force).clamp(min=1e-8),max=1.)
        # The examiner holds opposite sides of the collar and keeps its starting attitude.
        # Apply a bounded wrench only to the ring. The teat and bottle are always free.
        pose = collar.get_transforms()[0]
        rotation = Rotation.from_quat(pose[3:].cpu().numpy())
        error = (initial_rotation * rotation.inv()).as_rotvec()
        torque = .025 * start.new_tensor(error) - .0009 * collar.get_velocities()[0,3:]
        torque *= torch.clamp(.015 / torch.linalg.vector_norm(torque).clamp(min=1e-8), max=1.)
        collar.apply_forces_and_torques_at_position(force[None],torque[None],pose[None,:3].contiguous(),idx,True)
        # No robot actions, policy observations or scorer timestamps advance.
        m.original_step(render=False)
        contacts.append(m.contacts.get_contact_data(dt=1/HZ)[4].cpu().numpy().copy())
        subposes.append(m.rigid.get_transforms().cpu().numpy().copy())
        forces.append(force.cpu().numpy().copy())
        if step%12==0:
            capture()
    data=dict(poses=np.array(rows),nodes=np.array(softs),arm_poses=np.array(arms),
              robotcounts=np.array(contacts),subposes=np.array(subposes),forces=np.array(forces),
              arm_paths=m.arm_paths,names=names,grid=m.grid,tets=m.tets)
    for key in ('poses','nodes','arm_poses','robotcounts','subposes','forces'):
        if not np.isfinite(data[key]).all():
            raise ValueError(f"Invalid proof data in {key}")
    if np.any(data['robotcounts'].sum(axis=(1,2))>=8192):
        raise RuntimeError("Proof-phase contact buffer saturated")
    np.savez_compressed(root/'collar-exam.npz',**data)
    result=grade(data)
    (root/'collar-exam.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def examine(m, out, scene):
    """If the examiner touches an arm, repeat from the same released specimen on a clear bench."""
    import torch
    import monitor
    from geometry import BOTTLE

    # Preserve poses, soft deformation, rigid momenta and nodal velocities before the first lift.
    bodies = [(v.get_transforms().clone(), v.get_velocities().clone()) for v in (m.rigid, m.load)]
    nodes = m.soft.get_simulation_nodal_positions().clone()
    velocities = m.soft.get_simulation_nodal_velocities().clone()
    try:
        return run(m, out)
    except ObstructedLift:
        (out/'collar-exam.npz').rename(out/'obstructed-collar-exam.npz')
    try:
        # This reset is after rollout, outside the policy and its camera/action record.
        m.embodiment.reset(scene, seed=0)
        bench = monitor._active
        if bench.objects != m.objects or not np.array_equal(bench.order, m.order):
            raise RuntimeError("The independent exam fixture differs from the policy fixture")
        bottle = next(i for i,path in enumerate(m.objects) if path.endswith('/bottle'))
        offset = bodies[0][0].new_zeros(3)
        offset[:2] = offset.new_tensor(BOTTLE[:2])-bodies[0][0][bottle,:2]
        for view, (pose, speed) in zip((bench.rigid, bench.load), bodies, strict=True):
            idx = torch.arange(view.count, device=pose.device, dtype=torch.int32)
            pose[:,:3] += offset
            view.set_transforms(pose, idx)
            view.set_velocities(speed, idx)
        idx = torch.zeros(1,device=nodes.device,dtype=torch.int32)
        bench.soft.set_simulation_nodal_positions(nodes + offset,idx)
        bench.soft.set_simulation_nodal_velocities(velocities,idx)
        result = run(bench,out)
        (out/'independent-exam.json').write_text(json.dumps(dict(
            reason='withdrawn arm obstructed examiner',translation_m=offset.cpu().tolist()))+'\n')
        return result
    finally:
        monitor._active = m
        m.sim.step = m.physics_step


def register(out):
    """Capture a physical judgement after rollout. The scorer only reads that judgement."""
    import monitor
    from inspect_robots.registry import grader, scorer
    from inspect_robots.scorer import Score
    from yam_arena.scoring import TaskScore

    @grader('collar_load_exam')
    class CollarExam:
        name = 'collar_load_exam'

        def grade(self, record, scene):
            assembly = TaskScore()(record, scene.target).value
            if monitor._active is None or not monitor._active.scores:
                raise RuntimeError("Missing policy physics")
            if assembly != monitor._active.scores[-1]:
                raise RuntimeError("Released assembly score and final observation disagree")
            result = examine(monitor._active, out, scene) if assembly else None
            score = min(assembly, result['score']) if result else 0.0
            record.operator_judgement = 'success' if score == 1.0 else 'failure'
            record.operator_note = json.dumps({'released_candidate': assembly, 'collar_retention': score})

    @scorer('collar_retention')
    class CollarRetention:
        name = 'collar_retention'

        def __call__(self, record, target):
            evidence = json.loads(record.operator_note)
            value = evidence['collar_retention']
            if value not in (0, 0.5, 1) or value > evidence['released_candidate']:
                raise ValueError('Invalid collar examination score')
            return Score(value=value, explanation='Released endpoint and collar-only lift', metadata=evidence)
