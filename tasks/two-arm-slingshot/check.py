"""Check rubric failure cases and native URDF reachability, without physics."""
import copy
import json
from pathlib import Path

import numpy as np
from scoring import SlingshotScore

ROOT=Path(__file__).parent
cfg=json.loads((ROOT/'pilot.json').read_text())
identity=np.eye(3)
states={name:(np.array(p),identity.copy(),np.zeros(6))
        for name,p in cfg['poses'].items() if name not in ('launcher','fixture')}
states['launcher/frame']=(np.array(cfg['poses']['launcher']),identity.copy(),np.zeros(6))
states['launcher/pouch']=(states['launcher/frame'][0]+cfg['pouch_rest'],identity.copy(),np.zeros(6))
hands=np.array([states['launcher/frame'][0]+cfg['handle_offset'],cfg['poses']['ball']])

def check():
    rubric=SlingshotScore(cfg)
    for _ in range(120):
        result=rubric.evaluate(states,hands,[1,1])
    assert result['score']==0 and not result['unsafe_event']
    fallen=copy.deepcopy(states)
    for t in (1,2): fallen[f'tower{t}_2'][0][2]=cfg['table_top']+.025
    assert SlingshotScore(cfg).evaluate(fallen,hands,[0,1])['unsafe_event']
    drawn=copy.deepcopy(states)
    for name in ('launcher/pouch','ball'): drawn[name][0][0]-=.15
    draw_hands=hands.copy();draw_hands[1]=drawn['ball'][0]
    assert SlingshotScore(cfg).evaluate(drawn,draw_hands,[1,.4])['unsafe_event']
    rubric=SlingshotScore(cfg)
    assert rubric.evaluate(drawn,draw_hands,[0,.4])['drawn']
    launched=copy.deepcopy(states);launched['ball'][0][0]=.56;launched['ball'][2][0]=2.
    assert rubric.evaluate(launched,draw_hands,[0,1])['launched']
    # Credit is recomputed after settling and revoked if an object leaves bounds.
    fallen['ball']=launched['ball']
    for _ in range(20): result=rubric.evaluate(fallen,draw_hands,[0,1])
    assert result['score']==1 and result['success']
    fallen['ball'][0][1]=.4
    assert rubric.evaluate(fallen,draw_hands,[0,1])['score']==0
    invalid=copy.deepcopy(states);invalid['ball'][0][0]=float('nan')
    assert SlingshotScore(cfg).evaluate(invalid,hands,[0,1])['invalid_state']
    crossing=SlingshotScore(cfg)
    crossing.evaluate(states,hands,[0,1])
    assert crossing.evaluate(launched,draw_hands,[0,1])['unsafe_event']
    crossing=SlingshotScore(cfg)
    crossing.evaluate(drawn,draw_hands,[0,.4])
    before=copy.deepcopy(launched);before['ball'][0][0]=.49
    crossing.evaluate(before,draw_hands,[0,1])
    reflected=copy.deepcopy(launched);reflected['ball'][2][0]=-.3
    assert crossing.evaluate(reflected,draw_hands,[0,1])['launched']
    print('PASS: idle, premature collapse, unbraced draw, valid sequence, escape, NaN, undrawn shot')
    print('PASS: release crossing followed by collision within one sample')


if __name__=='__main__':
    check()
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('--ik',action='store_true')
    if parser.parse_args().ik:
        import sys
        sys.path.insert(0,str(ROOT.parents[1]))
        from task import Slingshot
        from tools.run_recovery_pilot import planned_actions
        task=Slingshot()
        for name,plan in json.loads((ROOT/'trajectory.json').read_text()).items():
            count=sum(1 for _ in planned_actions(task.initial_action(),plan,cfg['max_steps']))
            print(f'PASS: real YAM URDF {name}, {count} controls; not physics validation')
