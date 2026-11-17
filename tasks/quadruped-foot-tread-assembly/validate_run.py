"""Replay archived scorer and audit native joint evidence; no model calls."""
import argparse
import ast
import hashlib
import importlib
import json
import sys
import tempfile
import zipfile
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('run', type=Path)
a = p.parse_args(); run = a.run.resolve()
reports = [f for f in run.glob('adhoc*.json') if '.live.' not in f.name]
assert len(reports) == 1, 'Require exactly one final official report, not live progress'
official = json.loads(reports[0].read_text())
contract = json.loads((run/'mechanism-contract.json').read_text())
coverage = json.loads((run/'contact-contract.json').read_text())
manifest = json.loads((run/'source_manifest.json').read_text())
archive = np.load(run/'physics-substeps.npz')
raw = {key: archive[key] for key in archive.files}
observed = np.load(run/'joint-observations.npz')
configuration = json.loads((run/'configuration.json').read_text())
samples = len(raw['poses']); hz = int(raw['physics_hz'])
steps = official['stats']['total_steps']
assert len(observed['physics_sample']) == steps+1
assert len(coverage['robot_sensors']) == 18
assert all(len(raw[key]) == samples for key in raw.keys() if key not in ('paths','physics_hz'))
assert np.all(np.diff(observed['physics_sample']) == hz//20)
assert all(np.isfinite(raw[key]).all() for key in raw.keys() if key != 'paths')
paths = raw['paths'].tolist(); poses = raw['poses'].astype(float)
assert paths == contract['paths'] == coverage['object_sensors']
assert all(p in coverage['object_filters'] for p in contract['fixed_colliders'])
assert 'table/geometry/mesh' in coverage['object_filters'][-1]
assert np.max(np.abs(np.linalg.norm(poses[:,:,3:],axis=-1)-1)) < .0001
with tempfile.TemporaryDirectory(prefix='maintenance-archived-') as tmp:
    tmp = Path(tmp)
    with zipfile.ZipFile(run/'source.zip') as archive:
        for name in archive.namelist():
            target = tmp/name
            assert target.resolve().is_relative_to(tmp)
            payload = archive.read(name)
            assert hashlib.sha256(payload).hexdigest() == manifest[name]
        archive.extractall(tmp)
    sys.path.insert(0,str(tmp))
    source_tree=ast.parse((tmp/'task.py').read_text())
    stand_names=[node.args[0].value for node in ast.walk(source_tree)
                 if isinstance(node,ast.Call) and isinstance(node.func,ast.Name)
                 and node.func.id=='Box' and node.args
                 and isinstance(node.args[0],ast.Constant) and node.args[0].value!='build_anchor']
    staging_coverage_ok=all(any('/'+name+'/' in path or path.endswith('/'+name)
                                for path in coverage['robot_environment_filters'])
                            for name in stand_names)
    g = importlib.import_module('assembly.geometry')
    scorer_module = importlib.import_module('assembly.scoring')
    scorer = scorer_module.Outcome(paths,contract,coverage)
    first_full = None
    frame_by_sample={int(sample):frame for frame,sample in enumerate(observed['physics_sample'])}
    frame_scores=[None]*(steps+1)
    for i in range(samples):
        scorer.update(poses[i],raw['robot_contact_counts'][i],raw['object_contact_counts'][i],raw['robot_environment_counts'][i])
        if scorer.value == 1 and first_full is None: first_full=i
        if i in frame_by_sample:frame_scores[frame_by_sample[i]]=float(scorer.value)
    assert all(value is not None for value in frame_scores)
    (run/'score-timeline.json').write_text(json.dumps(dict(control_hz=20,scores=frame_scores,source='archived scorer replay at original observation samples'))+'\n')
    live = json.loads((run/'prototype-outcome.json').read_text())
    assert scorer.value == live['score']
    reported = official.get('results',{}).get('metrics',{}).get('task_score')
    official_ok = official.get('status') == 'success' and not official.get('results',{}).get('errored_trials',0)
    if official_ok: assert reported == scorer.value
    joint_errors=[]; angle_errors=[]; travel=[]
    if g.PREFIX == 'vac_':
        parent=poses[:,paths.index(g.ROOT+'/vac_cup')]
        rp=Rotation.from_quat(parent[:,3:])
        for i in range(4):
            child=poses[:,paths.index(g.ROOT+f'/vac_barb{i}')]
            rel=rp.inv().apply(child[:,:3]-parent[:,:3]);axis=np.array([np.cos(i*np.pi/2),0,np.sin(i*np.pi/2)])
            distance=rel@axis
            joint_errors.append(np.linalg.norm(rel-distance[:,None]*axis,axis=-1).max())
            angle_errors.append((rp.inv()*Rotation.from_quat(child[:,3:])).magnitude().max())
            travel.append([float(distance.min()),float(distance.max())])
        low,high=np.array(contract['radial_travel_mm'])/1000
        limits_ok=all(lo>=low-.0002 and hi<=high+.0002 for lo,hi in travel)
    else:
        parent=poses[:,paths.index(g.ROOT+'/foot_pad')];rp=Rotation.from_quat(parent[:,3:])
        for i,(sign,y) in enumerate(g.FLAPS):
            child=poses[:,paths.index(g.ROOT+f'/foot_flap{i}')];rc=Rotation.from_quat(child[:,3:]);anchor=np.array([sign*.030,y,-.0015])
            joint_errors.append(np.linalg.norm(parent[:,:3]+rp.apply(anchor)-child[:,:3]-rc.apply(anchor),axis=-1).max())
            rv=(rp.inv()*rc).as_rotvec()
            angle_errors.append(np.linalg.norm(rv[:,[0,2]],axis=-1).max())
            travel.append([float(np.rad2deg(rv[:,1]).min()),float(np.rad2deg(rv[:,1]).max())])
        limits_ok=all(lo>=(0 if sign<0 else -25)-.3 and hi<=(25 if sign<0 else 0)+.3
                      for (lo,hi),(sign,_) in zip(travel,g.FLAPS))
        for side in ('left','right'):
            parent=poses[:,paths.index(g.ROOT+f'/foot_{side}_clamp')];rp=Rotation.from_quat(parent[:,3:])
            for k in (0,1):
                child=poses[:,paths.index(g.ROOT+f'/foot_{side}_end{k}')]
                rel=rp.inv().apply(child[:,:3]-parent[:,:3])
                joint_errors.append(np.linalg.norm(rel[:,[0,2]],axis=-1).max())
                angle_errors.append((rp.inv()*Rotation.from_quat(child[:,3:])).magnitude().max())
                low,high=(-.003,0.) if k==0 else (0.,.003)
                limits_ok &= bool(rel[:,1].min()>low-.0002 and rel[:,1].max()<high+.0002)
    constraints_ok=max(joint_errors)<.0002 and max(angle_errors)<np.deg2rad(.3) and limits_ok
    replay=dict(score=scorer.value,final=scorer.detail,first_full_sample=first_full,scorer_sha256=manifest['assembly/scoring.py'])
    (run/'score-replay.json').write_text(json.dumps(replay,indent=2)+'\n')

media_ok=False; frame_counts={}; minimum_std={}
if not configuration['physics_only_diagnostic']:
    directory=Path(official['stats']['frames_dir'])
    for camera in ('top_cam','left_cam','right_cam'):
        files=sorted(directory.glob('*_'+camera+'_*.npy'));frame_counts[camera]=len(files)
        assert len(files)==steps+1
        minimum_std[camera]=255.
        for f in files:
            image=np.load(f);assert image.shape==(176,320,3) and image.dtype==np.uint8
            minimum_std[camera]=min(minimum_std[camera],float(image.std()))
    media_ok=minimum_std['top_cam']>5 and json.loads((run/'image_health.json').read_text()).get('valid',False)
result=dict(official_complete=official_ok,score=scorer.value,replay_matches_live=True,
    fullscore_demo_valid=bool(official_ok and scorer.value==1 and media_ok and constraints_ok and staging_coverage_ok and not configuration.get('preassembled_cover_diagnostic',False)),
    steps=steps,physics_hz=hz,physics_samples=samples,preassembled_cover_diagnostic=configuration.get('preassembled_cover_diagnostic',False),contact_coverage_18_bodies=True,
    constraints_ok=bool(constraints_ok),max_joint_anchor_error_mm=float(max(joint_errors)*1000),
    staging_contact_coverage_ok=staging_coverage_ok,
    max_joint_forbidden_angle_deg=float(np.rad2deg(max(angle_errors))),native_joint_travel=travel,
    media_valid=bool(media_ok),frames=frame_counts,minimum_image_std=minimum_std,
    known_layout=configuration.get('mode','demo')=='demo',model_result=configuration.get('mode')=='dream',custom_engineering_parts=True)
result['contact_force_peaks_N'] = {
    key: float(np.linalg.norm(raw[key],axis=-1).max())
    for key in ('robot_contact_forces','object_contact_forces','robot_environment_forces')
}
# Additional conservative engineering screen, independent of task scoring and
# native constraint accuracy. These are anomaly limits, NOT calibrated material
# strengths: a stiff joint can preserve geometry while contact dynamics explode.
result['transient_dynamics'] = dict(
    max_object_contact_pair_N=result['contact_force_peaks_N']['object_contact_forces'],
    max_reported_body_speed_m_s=float(np.linalg.norm(raw['velocities'][:,:,:3],axis=-1).max()),
    max_pose_body_speed_m_s=float(np.linalg.norm(np.diff(poses[:,:,:3],axis=0)*hz,axis=-1).max()),
    object_pair_limit_N=200.,body_speed_limit_m_s=2.,
    note='Conservative anomaly screen, not a material rating or scoring condition.')
td=result['transient_dynamics']
result['transient_dynamics_ok']=bool(td['max_object_contact_pair_N']<200 and td['max_reported_body_speed_m_s']<2 and td['max_pose_body_speed_m_s']<2)
result['fullscore_demo_valid'] &= result['transient_dynamics_ok']
# A policy may drop a part and produce a legitimate fast fall. Preserve that
# model result; conservative demo force/speed screens alone do not invalidate it.
model_run=configuration.get('mode')=='dream'
result['dynamics_review_required']=bool(model_run and not result['transient_dynamics_ok'])
result['run_valid']=bool(official_ok and media_ok and constraints_ok and staging_coverage_ok and (model_run or result['transient_dynamics_ok']))
(run/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))

raise SystemExit(0 if result['run_valid'] else 1)
