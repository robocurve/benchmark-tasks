from scorer import RecoveryScorer
from latch_geometry import InsertionGate
s=RecoveryScorer();kw=dict(lifted=False,held=False,latched=False,position_error_m=.1,rotation_error_rad=0,other_hand_clear=False,live_image=True)
assert s.update(**kw)['score']==0
kw.update(lifted=True,held=True)
for _ in range(20):r=s.update(**kw)
assert r['score']==.25
kw['position_error_m']=.001
for _ in range(20):r=s.update(**kw)
assert r['score']==.5
kw.update(latched=True,held=False);assert s.update(**kw)['score']==.75
kw['other_hand_clear']=True
for _ in range(40):r=s.update(**kw)
assert r['score']==1
assert s.update(**kw,off_table=True)['score']==0
for start in [(0,0,0),(.01,0,.035),(0,.01,.035)]:
 g=InsertionGate();assert not g.update(start,0,0);assert not g.update((0,0,0),0,0)
g=InsertionGate();assert not g.update((0,0,.035),0,0);assert not g.update((.01,0,.010),0,0);assert not g.update((0,0,0),0,0)
g=InsertionGate();assert not g.update((0,0,.035),0,0);assert g.update((0,0,0),0,0)
print('PASS scoring tiers, invalidation, side/rear approach and lost-entry rejection')
