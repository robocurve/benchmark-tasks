"""Diagnostic simulator runner. Saves measured states and camera frames; never overrides physics.

python probe.py --mode hold --steps 200 --out /tmp/film-hold
python probe.py --mode demo --steps 1400 --out /tmp/film-demo
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
import shutil
import numpy as np


def source_hash():
    root = Path(__file__).parent
    names = sorted(p.name for p in root.glob('*.py'))
    return hashlib.sha256(b''.join(name.encode()+b'\0'+(root/name).read_bytes() for name in names)).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['hold', 'demo', 'single', 'press', 'shear','direct','decoy'], default='hold')
    parser.add_argument('--scenario',default='all',help='Direct baseline name, or all')
    parser.add_argument('--measure-load',action='store_true',help='Record tab/tool contact load at each physics substep')
    parser.add_argument('--steps', type=int, default=200)
    parser.add_argument('--epochs', type=int, default=1)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from yam_arena.inspect_embodiment import YamArenaEmbodiment
    from yam_arena.demo import ArmPlan

    emb = YamArenaEmbodiment(task=str(Path(__file__).with_name('task.py')),
                             object_jitter=0, control_hz=20, cam_width=320, cam_height=176)
    loaded_hash = source_hash()
    from adversarial import SCENARIOS
    scenarios=(SCENARIOS if args.scenario=='all' else (args.scenario,)) if args.mode=='direct' else (None,)
    for scenario,epoch in ((s,e) for s in scenarios for e in range(args.epochs)):
        run_out=args.out/scenario if scenario else args.out
        run_out.mkdir(parents=True,exist_ok=True)
        (run_out/'source').mkdir(exist_ok=True)
        for source in Path(__file__).parent.glob('*.py'):
            shutil.copyfile(source,run_out/'source'/source.name)
        start = time.monotonic()
        obs = emb.reset(Scene(id='peel-film', instruction=__doc__), seed=0)
        from yam_arena.scene import object_states
        from filmkit import _paths, _bonds
        stage = _bonds[0].GetPrim().GetStage()
        config = {}
        for name, path in _paths.items():
            prim = stage.GetPrimAtPath(path)
            config[name] = {a.GetName(): a.Get() for a in prim.GetAttributes()
                            if a.GetName().startswith(('physics:', 'physxRigidBody:'))}
        config['bonds'] = [dict(body0=[str(p) for p in b.GetBody0Rel().GetTargets()],
                                body1=[str(p) for p in b.GetBody1Rel().GetTargets()],
                                attributes={a.GetName(): a.Get() for a in b.GetPrim().GetAttributes()})
                           for b in _bonds]
        (run_out/f'{epoch}-physics.json').write_text(json.dumps(config, default=str, indent=2))
        import cv2
        for name, frame in obs.images.items():
            cv2.imwrite(str(run_out / f'{epoch}-initial-{name}.png'), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            raw = emb._base.scene[name].data.output['rgb']
            raw = raw.torch if hasattr(raw, 'torch') else raw
            rgb = raw[0, ..., :3].cpu().numpy()
            cv2.imwrite(str(run_out / f'{epoch}-setup-{name}.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        hold = obs.state['joint_pos']
        plans = {arm: ArmPlan(w, emb.rig.bases[arm]) for arm, w in emb.task.demo().items()}
        active_arm='left' if scenario in ('lift_carrier','push_carrier') else 'right'
        if args.mode=='direct':
            from adversarial import make_plan
            plans[active_arm]=make_plan(scenario,emb.rig.bases[active_arm])
        if args.mode == 'decoy':
            from adversarial import decoy_waypoints
            plans['right'] = ArmPlan(decoy_waypoints(), emb.rig.bases['right'])
        if args.mode in ('press', 'shear'):
            from yam_arena.demo import Waypoint as W
            from geometry import CARRIER_POS, CARRIER_SIZE
            press_z = float(CARRIER_POS[2] + CARRIER_SIZE[2]/2 + .010)
            moves = [
                W(8, (.28, -.09, .83), 0., 'approach film'),
                W(3, (.28, -.09, press_z), 0., 'press film')]
            if args.mode == 'shear':
                moves += [W(3, (.31, -.09, press_z), 0., 'rub across film'),
                          W(3, (.25, -.09, press_z), 0., 'rub back')]
            else:
                moves += [W(5, (.28, -.09, press_z), 0., 'hold pressure')]
            moves += [W(2, (.28, -.09, .83), 0., 'withdraw')]
            plans['right'] = ArmPlan(moves, emb.rig.bases['right'])
        records = []
        step_times = []
        sensor=None
        if args.measure_load:
            from force_probe import ContactLoad
            from toolkit import PATHS
            sensor=ContactLoad(emb,_paths['pull_tab'],[PATHS[n] for n in ('tweezer_a','tweezer_b')])
        for step in range(args.steps):
            action = hold if args.mode == 'hold' else np.concatenate(
                [plans[arm].action7((step+1)/20)[0] for arm in ('left', 'right')])
            if args.mode == 'single':
                action[:7] = hold[:7]
            if scenario in ('lift_carrier','push_carrier'):
                action[7:]=hold[7:]
            tick = time.monotonic()
            result = emb.step(Action(action))
            step_times.append(time.monotonic()-tick)
            states = object_states(emb._env, emb.task.names)
            record = dict(step=step, info=result.info,
                          action=action.tolist(),
                          right_phase=plans['right'].target((step+1)/20)[3],
                          active_arm=active_arm,active_phase=plans[active_arm].target((step+1)/20)[3],
                          joint_pos=result.observation.state['joint_pos'].tolist(),
                          root_poses={n:emb._env.unwrapped.arena_world.get_pose_w(n)[0].cpu().numpy().tolist()
                                      for n in emb.task.names},
                          objects={name: dict(pos=s.pos.tolist(), quat=s.quat_xyzw.tolist(),
                                              vel=s.lin_vel.tolist(), ang_vel=s.ang_vel.tolist())
                                   for name, s in states.items()})
            if sensor is not None:record['contact_load_substeps']=sensor.drain()
            records.append(record)
            if step % 20 == 0 or result.terminated:
                print(json.dumps(dict(scenario=scenario,epoch=epoch, step=step, info=result.info,
                                      carrier=states['carrier'].pos.tolist())), flush=True)
                for name, frame in result.observation.images.items():
                    cv2.imwrite(str(run_out/f'{epoch}-{step:04d}-{name}.png'), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                (run_out/f'{epoch}-trace.json').write_text(json.dumps(records))
            if result.terminated:
                break
        if sensor is not None:sensor.close()
        (run_out/f'{epoch}-trace.json').write_text(json.dumps(records))
        summary = dict(mode=args.mode,scenario=scenario, epoch=epoch, steps=len(records), score=result.info['score'],
                       success=bool(result.terminated), final_info=result.info,
                       wall_seconds=time.monotonic()-start, max_step_seconds=max(step_times),
                       worst_ik={arm:dict(mm=p.worst_ik_mm,deg=p.worst_ik_deg) for arm,p in plans.items()},
                       source_sha256=loaded_hash, source_unchanged=loaded_hash == source_hash())
        if sensor is not None:
            peel_samples=[np.linalg.norm(s['resultant_N']) for r in records
                          if r['right_phase'] in ('lift tab from slot','peel film')
                          for s in r['contact_load_substeps']]
            summary['peel_peak_contact_N']=max(map(float,peel_samples),default=0.)
            summary['peel_load_samples']=len(peel_samples)
            summary['load_physics_dt']=sensor.dt
        (run_out/f'{epoch}-summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        print(f'epoch {epoch}: {time.monotonic()-start:.1f} seconds', flush=True)
    emb.close()


if __name__ == '__main__':
    main()
