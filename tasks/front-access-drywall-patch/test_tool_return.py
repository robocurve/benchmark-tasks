import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from repair_scene import Repair, WORLD
from verify import tool_at_rest
from tool_profile import TRIGGER


def recording():
    names=['driver','shoe','spindle','trigger','screw_carrier','carrier']
    paths=[WORLD+'/'+name for name in names]
    arms=[WORLD+'/'+side+'/tip_'+jaw for side in ('LeftArm','RightArm') for jaw in ('left','right')]
    schema=dict(paths=paths,filters=paths+[WORLD+'/table/geometry/mesh']+arms,sensors=arms)
    pose=np.zeros((240,6,7));pose[:,:,6]=1;pose[:,3,:3]=TRIGGER
    forces=np.zeros((240,6,len(schema['filters']),3))
    forces[:,0,6,2]=12.36
    return pose,forces,schema


def live(pose,forces,schema):
    r=Repair.__new__(Repair)
    from putaway import Touchdown
    r.touchdown=Touchdown()
    r.touchdown.seen=True
    r.fasteners=[None]*8
    r.sample=0
    r.park_indices=None
    r.park_samples=0
    r.park_reference=None
    r.diagnostics={}
    r.last_metrics={}
    for i in range(0,len(pose),12):
        r.score=1.
        r.sample+=len(pose[i:i+12])
        r.measure_park(pose[i:i+12],forces[i:i+12],schema['paths'],schema['filters'],240)
    return r.score


@pytest.mark.parametrize('case',['good','switch','left tip','right tip','air','sliding','turning','short hold'])
def test_putaway_requires_a_released_supported_stationary_driver(case):
    p,f,s=recording()
    if case=='switch':p[-1,3,0]-=.003
    if case=='left tip':f[-1,4,7,1]=.04
    if case=='right tip':f[-13,1,-1,1]=.04
    if case=='air':f[-1,:,6]=0
    if case=='sliding':p[-1,:,0]+=.004
    if case=='turning':p[-1,:,3:]=Rotation.from_euler('x',4,degrees=True).as_quat()
    if case=='short hold':p=p[:-12];f=f[:-12]
    assert live(p,f,s)==(1. if case=='good' else .75)
    assert tool_at_rest(p,f,s,240)['passes']==(case=='good')

@pytest.mark.parametrize('held,off,expected',[(True,True,True),(False,True,False),(True,False,False)])
def test_unattended_drop_or_live_touchdown_cannot_earn_full_score(held,off,expected):
    from putaway import Touchdown
    history=Touchdown()
    history.update([0.]*24+[10.]*241,[True]*24+[held]+[False]*240,
                   [off]*25+[True]*240,240,0)
    assert history.seen is expected
    # A later touch while the tool is already lying down cannot fabricate a placement.
    history.update([12.],[True],[True],240,265)
    assert history.seen is expected


def test_a_new_airborne_pickup_can_recover_an_uncontrolled_landing():
    from putaway import Touchdown
    history=Touchdown()
    history.update([0.]*25+[12.],[False]*26,[True]*26,240,0)
    assert not history.seen
    history.update([0.]*25+[12.],[True]*26,[True]*26,240,26)
    assert history.seen and history.event['sample']==51


def test_tool_on_table_at_reset_is_not_a_controlled_return():
    from putaway import Touchdown
    history=Touchdown()
    history.update([12.]*240,[True]*240,[True]*240,240,0)
    assert not history.seen
