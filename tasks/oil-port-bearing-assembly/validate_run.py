"""Validate native evidence and replay the exact archived live-geometry scorer."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import zipfile


def require(condition,message):
    if not condition:raise ValueError(message)


def validate(run):
    import numpy as np
    config=json.loads((run/'configuration.json').read_text())
    require(not config['config_only'],'Config-only is not an evaluated trial')
    manifest=json.loads((run/'source_manifest.json').read_text())
    archive=run/'task-source-at-run.zip'
    with zipfile.ZipFile(archive) as z:
        for name,digest in manifest.items():
            require(not Path(name).is_absolute() and '..' not in Path(name).parts,'Unsafe archive path')
            require(hashlib.sha256(z.read(name)).hexdigest()==digest,'Archived source hash mismatch: '+name)
    sys.path.insert(0,str(archive))
    from prototype.scoring import Evaluator,SCORING_REVISION
    from prototype.passage import check as passage_check
    from prototype.layout import SCENE_REVISION
    from prototype.geometry import VISUAL_REVISION
    require(config['scene_revision']==SCENE_REVISION,'Scene version mismatch')
    require(config['scoring_revision']==SCORING_REVISION,'Scoring version mismatch')
    require(config['visual_revision']==VISUAL_REVISION,'Appearance version mismatch')
    files=[p for p in run.glob('adhoc_*.json') if not p.name.endswith('.live.json')]
    require(len(files)==1,'Missing or ambiguous final native result')
    native=json.loads(files[0].read_text())
    require(native['status'] in ('success','completed') and not native.get('error'),'Native environment error')
    require(not native['results'].get('errored_trials',0),'Native errored trial')
    steps=native['stats']['total_steps'];require(0<steps<=config['steps']<=2400,'Invalid action budget')
    diagnostics=json.loads((run/'diagnostics.json').read_text());rows=diagnostics['frames'];contact=diagnostics['contacts']
    require(contact['ready'] and contact['error'] is None,'Invalid contact monitor')
    require(contact['samples']>=12*steps and len(rows)==steps,'Incomplete physical trace')
    evaluator=Evaluator()
    for i,row in enumerate(rows):
        require(row['step']==i,'Discontinuous score trace')
        require(row['contact_interval']['valid'] and row['contact_interval']['samples']>=12,'Missing physics samples')
        measured=passage_check(row['objects']['sleeve']['pose_xyzw'])
        require(measured==row['passage'],f'Live-geometry replay mismatch at {i}')
        replay=evaluator.update(row)
        require(replay['valid'] and replay==row['evaluation'],f'Scoring replay mismatch at {i}')
    require(replay['score']==native['results']['metrics']['task_score'],'Native score disagrees with trace')
    health=json.loads((run/'image_health.json').read_text())
    require(health['valid'] and health['error'] is None,'Invalid camera images')
    require(health['frames']==steps+1,'Image-health coverage mismatch')
    frames={}
    for cam in ('top_cam','left_cam','right_cam'):
        images=sorted((run/'frames').rglob(f'*-e0_{cam}_*.npy'));frames[cam]=len(images)
        require(len(images)==steps+1,'Missing frames: '+cam)
        for f in images:
            image=np.load(f)
            require(image.shape==(176,320,3) and image.dtype==np.uint8,'Invalid RGB format')
    if config['mode']=='dream':
        require(config['controller_installed'] is False and config['scripted_plan_supplied'] is False,'Scripted Dream path')
        require(config['scene_mode']=='setup' and config['canonical_scene'],'Diagnostic Dream initialization')
        require(not (run/'controller_trace.json').exists() and not (run/'plan.json').exists(),'Unexpected controller evidence')
        require(all(v in config['argv'] for v in ('record=false','dry_run=false','model=dreamlabs/dream-1-beta')),'Wrong Dream config')
    root=Path(__file__).resolve().parent
    source_matches=all((root/n).is_file() and hashlib.sha256((root/n).read_bytes()).hexdigest()==h for n,h in manifest.items())
    report=dict(valid_run=True,mode=config['mode'],paid=config['paid'],steps=steps,simulation_s=steps/20,
                final_score=replay['score'],final_evaluation=replay,final_passage=rows[-1]['passage'],
                camera_frames=frames,contact_samples=contact['samples'],raw_score_replay=True,
                archived_source_files=len(manifest),current_source_matches=source_matches,
                scene_revision=SCENE_REVISION,visual_revision=VISUAL_REVISION,scoring_revision=SCORING_REVISION,
                diagnostic_initialization=not config['canonical_scene'],
                note='Scripted runs demonstrate known-layout feasibility; they are not visual-feedback or Dream results.')
    (run/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);args=p.parse_args()
    try:print(json.dumps(validate(args.run.resolve()),indent=2))
    except Exception as exc:
        print(f'INVALID RUN: {type(exc).__name__}: {exc}',file=sys.stderr)
        raise SystemExit(1)
