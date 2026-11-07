"""Re-score saved measured poses and raw substep contacts; never a new simulation."""
import argparse,json
from pathlib import Path
from types import SimpleNamespace
from guide_task.scoring import Evaluator
from guide_task.contact import Monitor
from guide_task import rules

def replay(run):
    data=json.loads((run/'evaluation_trace.json').read_text())
    history=SimpleNamespace(ready=data['monitor_ready'],error=data['monitor_error'],samples=0,events=[],violations=[])
    scorer=Evaluator();classifier=Monitor();cursor=0;results=[]
    contacts=data['contacts']
    assert all(a['physics_sample']<=b['physics_sample'] for a,b in zip(contacts,contacts[1:]))
    for row in data['steps']:
        history.samples=row['contact_samples']
        while cursor<len(contacts) and contacts[cursor]['physics_sample']<=history.samples:
            event=contacts[cursor];cursor+=1
            expected=classifier.prohibited(event['a'],event['b'])
            assert expected==event['prohibited'],'Contact classification differs from raw record'
            history.events.append(event)
            if expected:history.violations.append(event)
        results.append(scorer.update(row['pose'],history))
    assert cursor==len(contacts)
    assert history.violations==data['violations']
    return results

def main():
    p=argparse.ArgumentParser();p.add_argument('runs',nargs='+',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();summary=[]
    for run in a.runs:
        rows=replay(run);old=json.loads((run/'evaluation_trace.json').read_text())['steps']
        summary.append(dict(run=str(run),version=rules.VERSION,old_final=old[-1]['score'],new_final=rows[-1]['score'],
                            all_valid=all(r['valid'] for r in rows),first_full_action=next((i+1 for i,r in enumerate(rows) if r['complete']),None),
                            old_first_full_action=next((i+1 for i,r in enumerate(old) if r['complete']),None),
                            changed_scores=sum(x['score']!=y['score'] for x,y in zip(rows,old)),
                            final=rows[-1]))
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(summary,indent=2)+'\n')
    for row in summary:print(row['run'],'old',row['old_final'],'new',row['new_final'],'first full',row['first_full_action'],'valid',row['all_valid'])
if __name__=='__main__':main()
