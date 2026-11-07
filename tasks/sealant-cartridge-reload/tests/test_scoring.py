"""Synthetic endpoint tests only; these are not native robot feasibility trials."""
import copy
import json
from pathlib import Path
import numpy as np
from sealant_reload.config import FRAME_POSITION, CARTRIDGE_OFFSET, TRAY_POSITION, TABLE_TOP
from sealant_reload.geometry import local_solids, world_points
from sealant_reload.scoring import Scorer
ROOT=Path(__file__).resolve().parents[1]
LOCAL=local_solids(json.loads((ROOT/'scene-inventory.json').read_text())['records'])
IDENTITY=[0.,0.,0.,1.]
def state():
    f=[*FRAME_POSITION,*IDENTITY]
    u=[*(TRAY_POSITION+np.array([0,0,.029])),*IDENTITY]
    n=[*(FRAME_POSITION+CARTRIDGE_OFFSET+[0,0,-.004]),*IDENTITY]
    s=dict(time_s=0.,bodies={'Frame':dict(pose=f,velocity=[0.]*6)},
           used=dict(pose=u,velocity=[0.]*6),replacement=dict(pose=n,velocity=[0.]*6),
           mechanism=dict(rod_slide=-.182,rod_rotation=0.,pawl_angle=0.),
           hands={a:dict(opening_m=.09) for a in ['left','right']},robot_clearance_m=.1)
    update_bounds(s)
    return s

def update_bounds(s):
    vertices=world_points({k:v for k,v in LOCAL.items() if k.startswith(('used:','replacement:'))},
        {k+':Body':s[k]['pose'] for k in ['used','replacement']})
    s['bounds']={k:[v.min(0).tolist(),v.max(0).tolist()] for k,v in vertices.items()}

def test_full_requires_continuous_released_hold():
    c=Scorer();s=state();assert c.evaluate(s)['score']==.75
    s['time_s']=.9;assert not c.evaluate(s)['success']
    s['hands']['left']['opening_m']=.06;assert not c.evaluate(s)['success']
    s['time_s']=1.;s['hands']['left']['opening_m']=.09;assert not c.evaluate(s)['success']
    s['time_s']=1.9;assert not c.evaluate(s)['success']
    s['time_s']=2.01;assert c.evaluate(s)['score']==1.

def test_reversed_replacement_is_not_seated():
    s=state();s['replacement']['pose'][3:]=[0,0,1,0];update_bounds(s)
    r=Scorer().evaluate(s);assert r['score']==.25 and not r['seated']

def test_irrelevant_cartridge_roll_is_accepted():
    s=state();s['replacement']['pose'][3:]=[0,1,0,0];update_bounds(s)
    assert Scorer().evaluate(s)['score']==.75

def test_replacement_above_frame_is_not_seated():
    s=state();s['replacement']['pose'][2]+=.02;update_bounds(s)
    assert Scorer().evaluate(s)['score']==.25

def test_used_cartridge_overhangs_despite_correct_centre():
    s=state();s['used']['pose'][3:]=[0,0,np.sqrt(.5),np.sqrt(.5)];update_bounds(s)
    r=Scorer().evaluate(s);assert r['score']==0. and not r['collected']

def test_missing_used_cartridge_invalidates_all_tiers():
    s=state();s['used']['pose'][0]+=.15;update_bounds(s)
    assert Scorer().evaluate(s)['score']==0.

def test_disengaged_ratchet_cannot_complete():
    s=state();s['mechanism']['rod_rotation']=-1.
    assert Scorer().evaluate(s)['score']==.5

def test_retracted_piston_without_contact_cannot_complete():
    s=state();s['mechanism']['rod_slide']=-.189
    assert Scorer().evaluate(s)['score']==.5

def test_robot_support_cannot_complete():
    s=state();s['robot_clearance_m']=0.;c=Scorer()
    c.evaluate(s);s['time_s']=3.;assert c.evaluate(s)['score']==.75

def test_escape_is_permanent_after_return():
    s=state();c=Scorer();c.observe_physics({'test':np.array([[.5,.7,.8]])})
    assert c.evaluate(s)['score']==0.
    c.observe_physics({'test':np.array([[.5,0,.8]])});assert c.evaluate(s)['score']==0.

def test_nonfinite_geometry_is_invalid():
    c=Scorer();c.observe_physics({'test':np.array([[.3,0,float('nan')]])})
    assert c.evaluate(state())['score']==0.

def test_refills_fit_plain_cylindrical_envelope():
    for key in ('used:Body', 'replacement:Body'):
        vertices = LOCAL[key]
        assert vertices[:, 1].min() >= -.100001
        assert vertices[:, 1].max() <= .100001
        assert np.linalg.norm(vertices[:, [0, 2]], axis=1).max() <= .025
