"""Check package metadata and evaluation evidence before the official media upload/PR."""
import argparse
import ast
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
REQUIRED={'title','category','instruction','setup','scoring','run_limit','fidelity','scores','media'}
CATEGORIES={'Impact on jobs','Impact on AI timelines','Safety and alignment','Societal relevance','Other'}


def check(model_result=None,demo_result=None,allow_unuploaded=False):
    spec=json.loads((ROOT/'task.json').read_text());problems=[]
    for key in sorted(REQUIRED):
        if not isinstance(spec.get(key),str) or not spec[key].strip():
            if key!='media' or not allow_unuploaded:problems.append(f'Missing required field: {key}')
    if spec.get('category') not in CATEGORIES:problems.append('Invalid category')
    if 'pending' in spec.get('scores','').lower():problems.append('Scores are still marked pending')
    for relative in ('task.py','scoring.py','run.py','launch.py','build_runtime.py','README.md',
                     'runtime/Dockerfile','runtime/prepare.py','assets/inner.usda','assets/outer.usda'):
        if not (ROOT/relative).is_file():problems.append(f'Missing package file: {relative}')
    for source in ROOT.rglob('*.py'):
        if not any(x.startswith('.') for x in source.relative_to(ROOT).parts):ast.parse(source.read_text(),filename=source.name)
    model=None
    if model_result is None:problems.append('Dream run result is not supplied')
    else:
        model=json.loads(Path(model_result).read_text())
        if model.get('policy')!='dream':problems.append('Model result is not a Dream rollout')
        # The official task-board guide permits partial and failed Dream runs.
        # Completion is descriptive metadata, not a submission prerequisite.
        if model.get('score') not in (0.0, 0.5, 1.0):
            problems.append('Model result has no valid measured task score')
        if model.get('instruction')!=spec['instruction']:problems.append('Model instruction differs from task.json')
        if model.get('task')!='denest-trays' or not 0<model.get('steps',0)<=800:problems.append('Wrong task or step limit')
    if model is None or model.get('score')!=1.0:
        if demo_result is None:problems.append('Full-score demo result is not supplied')
        else:
            demo=json.loads(Path(demo_result).read_text())
            if not demo.get('success') or demo.get('score')!=1 or demo.get('final_metrics',{}).get('held_frames',0)<40:
                problems.append('Demo does not demonstrate the full scoring condition')
            # A scripted demo does not consume a language instruction.
    return problems


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model-result',type=Path);p.add_argument('--demo-result',type=Path)
    p.add_argument('--allow-unuploaded',action='store_true',help='Pre-upload check only; media ID is still required before PR')
    a=p.parse_args();problems=check(a.model_result,a.demo_result,a.allow_unuploaded)
    print(json.dumps({'ready':not problems,'problems':problems,
        'manual_checks':['Private task-board novelty and current claims','Real-hardware feasibility','Final blind-review media set']},indent=2))
    raise SystemExit(bool(problems))
