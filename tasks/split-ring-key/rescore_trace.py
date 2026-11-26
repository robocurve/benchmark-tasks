"""Apply the delivered visible rubric to saved measured trajectories.

This never runs physics, creates object motion, or alters a native result. Its
separate report records the scorer and trajectory hashes and is labelled as
post-hoc scoring. Camera footage and the original result remain unchanged.
"""
from pathlib import Path
import argparse
import hashlib
import json
import task as T


def rescore(directory):
    directory=Path(directory)
    trace=directory/'trace.jsonl'
    tracker=T.TipPassage()
    consecutive=0
    peak=0.
    first_full=None
    stages=[]
    original=json.loads((directory/'result.json').read_text())
    rows=[json.loads(line) for line in trace.read_text().splitlines()]
    initial=original.get('initial',{})
    if initial.get('marked_tip_key_local_m') is not None:
        tracker.update(initial['marked_tip_key_local_m'])
    with (directory/'rescored_trace.jsonl').open('w') as output:
        for row in rows:
            passage=tracker.update(row['marked_tip_key_local_m'])
            measured=row['visible_measurements']
            integrity=T.visible_integrity(measured,row['visible_baseline']) and row['finite_physics']
            result=T.evaluate(row['crossings'],row['free_end_clearance_m'],
                row['key_speed_m_s'],row['key_spin_rad_s'],row['grippers_clear'],
                row['ring_closed'],integrity,row['key_world_m'][2],row['splitter_m'],
                row['ring_spin_rad_s'],passage,row['unloaded_inspection_ready'])
            consecutive=consecutive+1 if result['complete'] else 0
            score=1. if consecutive>=40 else result['partial_score']
            if score==1. and first_full is None:first_full=row['step']
            peak=max(peak,score)
            if not stages or stages[-1]!=row['stage']:stages.append(row['stage'])
            output.write(json.dumps({'step':row['step'],'stage':row['stage'],
                'rescored_score':score,'tip_passage_events':tracker.events,**result})+'\n')
    summary={'method':'Post-hoc final-rubric scoring of saved simulator measurements; original native result preserved',
        'scorer_sha256':hashlib.sha256(Path(T.__file__).read_bytes()).hexdigest(),
        'trace_sha256':hashlib.sha256(trace.read_bytes()).hexdigest(),
        'original_native_task_score':original['task_score'],
        'steps':len(rows),'rescored_task_score':score,'peak_rescored_score':peak,
        'first_full_hold_step':first_full,'tip_passage_events':tracker.events,
        'stages_completed_or_entered':stages,'final':result}
    (directory/'final_scorer.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('directory')
    rescore(parser.parse_args().directory)
