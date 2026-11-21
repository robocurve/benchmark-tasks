"""CPU scorer truth-table regression; not a physical contact test."""
import itertools,json
from pathlib import Path
from tag_closure import outcome_score
cases=[]
for extracted,bonded,broken,linked,released in itertools.product((False,True),repeat=5):
    for seconds in (0.,.249,.25,1.99,2.,3.):
        value=outcome_score(extracted,bonded,broken,linked,released,seconds)
        if broken or not all((extracted,bonded,linked)):assert value==0
        elif not released or seconds<.25:assert value==.75
        elif seconds<2:assert value==.85
        else:assert value==1.
        cases.append(dict(extracted=extracted,bonded=bonded,broken=broken,linked=linked,released=released,stable_s=seconds,score=value))
assert outcome_score(True,True,False,True,True,3.,False)==0.
p=Path(__file__).parent/'runs/score-outcome-regression.json'
p.parent.mkdir(exist_ok=True)
p.write_text(json.dumps({'kind':'CPU truth-table only; not contact evidence','passed':True,'cases':cases},indent=2))
print(f'{len(cases)} cases passed: {p}')
