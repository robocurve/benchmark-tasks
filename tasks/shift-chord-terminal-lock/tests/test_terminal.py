import pathlib
import sys
import json
from types import SimpleNamespace
import numpy as np
import pytest
TASK=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(TASK))
for root in TASK.parents[2].rglob('yam-upstream'):
    if (root/'yam_arena').is_dir():
        sys.path.insert(0,str(root)); break
import geometry as g
import logic as l

def step(ledger,*keys,n=1):
    for _ in range(n): l.advance(ledger,{k:-.005 for k in keys})
def tap(ledger,key,*mods):
    step(ledger,*mods,n=3)
    step(ledger,key,*mods,n=4)
    step(ledger,*mods,n=3)
    step(ledger,n=3)
def accepted():
    x=l.new_ledger()
    tap(x,'a','shift'); tap(x,'b'); tap(x,'7'); tap(x,'enter')
    return x
def secured():
    x=accepted(); tap(x,'l','ctrl'); step(x,n=20)
    return x

def test_idle_zero(): assert l.evaluate_state(l.new_ledger(),True,True,True)[0]==0
def test_code_exact():
    x=accepted(); assert x['buffer']=='Ab7' and x['accepted'] and not x['rejected']
def test_full(): assert l.evaluate_state(secured(),True,True,True)[0]==1
@pytest.mark.parametrize('flags',[(False,True,True),(True,False,True),(True,True,False)])
def test_three_quarters(flags): assert l.evaluate_state(secured(),*flags)[0]==.75
def test_accepted_unsecured_half(): assert l.evaluate_state(accepted(),True,True,True)[0]==.5
def test_prefix_quarter():
    x=l.new_ledger(); tap(x,'a','shift'); tap(x,'b')
    assert l.evaluate_state(x,True,True,True)[0]==.25
def test_early_lock_quarter():
    x=l.new_ledger(); tap(x,'l','ctrl'); tap(x,'a','shift')
    assert x['buffer']=='' and l.evaluate_state(x,True,True,True)[0]==.25
def test_wrong_case_permanent_zero():
    x=l.new_ledger()
    for k in ('a','b','7','enter'): tap(x,k)
    assert x['rejected']
    x['accepted']=True; x['locked_after']=True
    assert l.evaluate_state(x,True,True,True)[0]==0
def test_wrong_code_not_accepted():
    x=l.new_ledger(); tap(x,'7'); tap(x,'enter'); assert x['rejected']
def test_ctrl_erase_wipes_permanently():
    x=accepted(); tap(x,'erase','ctrl'); tap(x,'l','ctrl')
    assert x['wipe'] and l.evaluate_state(x,True,True,True)[0]==0
def test_erase_correction():
    x=l.new_ledger(); tap(x,'7'); tap(x,'erase'); assert x['buffer']=='' and not x['wipe']
def test_simultaneous_nonmodifiers_void():
    x=l.new_ledger(); step(x,'a','b',n=4); step(x,n=4); assert x['buffer']==''
def test_press_counted_on_up_only():
    x=l.new_ledger(); step(x,'a',n=10); assert x['buffer']==''
    step(x,n=4); assert x['buffer']=='a'
def test_no_repeat_when_held():
    x=l.new_ledger(); step(x,'7',n=30); step(x,n=4); assert x['buffer']=='7'
def test_rapid_repress_preserves_both_up_events():
    x=l.new_ledger(); step(x,'7'); step(x); step(x,'7'); step(x,n=4)
    assert x['buffer']=='77'
def test_short_tap_waits_for_future_modifier_sample():
    x=l.new_ledger(); step(x,'a'); step(x); step(x,'shift'); step(x,n=3)
    assert x['buffer']=='A'
def test_digit_slider_identifier_normalized():
    x=l.new_ledger()
    for _ in range(4): l.advance(x,{'digit7':-.005})
    step(x,n=4)
    assert x['buffer']=='7' and not l.all_up({'digit7':-.005})
def test_hysteresis_does_not_release_at_two_mm():
    x=l.new_ledger(); step(x,'a',n=3); l.advance(x,{'a':-.002})
    assert x['down']['a'] and not x['buffer']
def test_two_steps_before_down_modifier_counts():
    x=l.new_ledger(); step(x,'shift'); step(x); step(x,'a',n=4); step(x,n=4)
    assert x['buffer']=='A'
def test_two_steps_after_down_modifier_counts():
    x=l.new_ledger(); step(x,'a'); step(x,'a'); step(x,'a','shift'); step(x,n=4)
    assert x['buffer']=='A'
def test_three_steps_after_down_modifier_does_not_count():
    x=l.new_ledger(); step(x,'a',n=3); step(x,'a','shift'); step(x,n=4)
    assert x['buffer']=='a'
def test_shift_digit_wrong_symbol():
    x=l.new_ledger(); tap(x,'7','shift'); assert x['buffer']=='&'
def test_three_character_buffer_limit():
    x=l.new_ledger()
    for k in ('7','7','7','a'): tap(x,k)
    assert x['buffer']=='777'
def test_still_requires_twenty_samples():
    x=l.new_ledger(); step(x,n=19); assert not l.still(x)
    step(x); assert l.still(x)
def test_fixture_fault_latches():
    x=secured(); l.advance(x,{},True); l.advance(x,{})
    assert l.evaluate_state(x,True,True,True)[0]==0
def test_keys_up_threshold():
    assert l.all_up({'a':-.0014}) and not l.all_up({'a':-.0015})
def test_clean_reset():
    a=secured(); b=l.new_ledger(); assert not b['accepted'] and not b['locked'] and not b['events']
def test_demo_limits_and_chords():
    import demo_plan
    plan=demo_plan.make_demo_plan()
    for arm,wp in plan.items():
        assert sum(p.duration for p in wp)*20<=1200
        assert all(p.pos[0]<=.3 and p.pos[2]-g.TABLE_TOP<=.16 and p.yaw==0 for p in wp)
    assert min(p.pos[1] for p in plan['left'])-max(p.pos[1] for p in plan['right'])>=.12
    assert g.CAP_TOP_Z-g.DOWN-(demo_plan.PRESS-.013)>=.008
def test_slides_and_real_loader_contract():
    import task
    assert len(task.DECK.sliders)==8
    assert all(s.axis=='Z' and s.travel==.005 for s in task.DECK.sliders)
    assert all(s.rate<=10 for s in task.DECK.sliders if s.name in g.MODIFIERS)
    assert all(not p.cylinders for p in task.OBJECTS)
    assert task.SCORE_OVER=='end' and callable(task.demo) and callable(task.score)
def test_metadata_and_readme_match():
    raw=(TASK/'task.json').read_bytes(); raw.decode('ascii'); d=json.loads(raw)
    assert len(d['title'])<=120 and len(d['context'])<=900 and len(d['fidelity'])<=500 and 20<=len(d['instruction'])<=540
    assert all(w not in d['instruction'].lower() for w in ('key','shift','ctrl','lock','hold','hazard'))
    assert all(d[k] in (TASK/'README.md').read_text() for k in ('instruction','setup','scoring','context','fidelity','run_limit','scores','reset_notes'))
def test_markers_created_once_and_moved():
    from display import TerminalDisplay
    class Marker:
        def visualize(self,**kw): self.kw=kw
    d=TerminalDisplay(); m=Marker(); d.markers=m
    e=SimpleNamespace(unwrapped=SimpleNamespace(scene=SimpleNamespace(env_origins=np.zeros((1,3)))))
    d.reset(e); assert len(m.kw['translations'])==352
    assert m.kw['translations'][350,2]>0 and m.kw['translations'][351,2]==-10
    d.update(e,dict(buffer='Ab7',locked=True),None)
    assert d.markers is m and m.kw['translations'][350,2]==-10 and m.kw['translations'][351,2]>0
