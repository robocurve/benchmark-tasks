"""Key displacement is read by the scorer; commands remain open loop."""
import json
import os
import numpy as np
import parts
import geometry as g
import logic
from display import DISPLAY

_RUN=logic.new_ledger()
_LAST=None
NAMES=(g.DECK_NAME,g.SCREEN_NAME,g.CARD_NAME)

def reset_screen():
    _reset(None)

def _reset(environment):
    global _LAST
    _LAST=None
    _RUN.clear(); _RUN.update(logic.new_ledger())
    DISPLAY.reset(environment)

def _advance(objects):
    starts={g.DECK_NAME:g.DECK_XY,g.SCREEN_NAME:g.SCREEN_XY,g.CARD_NAME:g.CARD_XY}
    fault=any(np.linalg.norm(np.asarray(objects[n].pos)-[*starts[n],g.TABLE_TOP])>g.FIXTURE_LIMIT for n in NAMES)
    logic.advance(_RUN,objects[g.DECK_NAME].sliders,fault)

def _step(environment):
    objects={n:parts._read_part_state(environment,parts._PARTS[n]) for n in NAMES}
    _advance(objects)
    DISPLAY.update(environment,_RUN,objects[g.DECK_NAME])

parts.on_reset(_reset)
parts.on_step(_step)

def _released(objects):
    environment=parts.env()
    if environment is None:
        return False
    scene=environment.unwrapped.scene
    origin=parts._to_numpy(scene.env_origins[0])
    for arm in ('left','right'):
        if not parts.gripper_open(environment,arm,.9):
            return False
        robot=scene[arm+'_robot']
        idx=list(robot.body_names).index('gripper')
        p=parts._to_numpy(robot.data.body_pos_w)[0,idx]-origin-np.array([0,0,g.TCP_LINK_OFFSET])
        deck=objects[g.DECK_NAME]
        # Fixed fixture faults exclude larger motion; conservatively include the whole deck bounds and caps.
        boxes=[((g.DECK_XY[0],g.DECK_XY[1],g.TABLE_TOP+g.DECK_SIZE[2]/2),g.DECK_SIZE)]
        boxes += [((xy[0],xy[1],g.TABLE_TOP+g.CAP_CENTRE_Z+deck.sliders.get('digit7' if k=='7' else k,0)),g.CAP_SIZE) for k,xy in g.KEYS.items()]
        if min(np.linalg.norm(np.maximum(np.abs(p-c)-np.array(s)/2,0)) for c,s in boxes)<g.RELEASE:
            return False
    return True

def score(objects):
    global _LAST
    if not all(n in objects for n in NAMES):
        return 0.
    if parts.env() is None and _LAST is not objects:
        _LAST=objects
        _advance(objects)
    points,ok,reason=logic.evaluate_state(_RUN,logic.all_up(objects[g.DECK_NAME].sliders),_released(objects),logic.still(_RUN))
    path=os.getenv('TASK_TRACE')
    if path:
        with open(path,'a',encoding='utf-8') as f:
            f.write(json.dumps(dict(score=points,reason=reason,buffer=_RUN['buffer'],accepted=_RUN['accepted'],locked=_RUN['locked'],keys=objects[g.DECK_NAME].sliders))+'\n')
    return points

def success(objects):
    return score(objects)>=1.

def distance(objects):
    return (1.-score(objects))*.1
