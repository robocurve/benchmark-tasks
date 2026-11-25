"""Synthetic scorer checks; no test record represents a measured trainer or run."""
import math

# Stock check.py cases use no live contact/robot evidence and must fail closed.
CASES=[('missing live robot/contact observations',{},0.,False),
       ('do-nothing fixed scene',{},0.,False)]


def run():
    import task
    def state(**changes):
        d=dict(known=True,invalid=False,pressure=270.,seated=True,snap_work=.01,snap_verified=True,hand_contact_n=10.,
               body_touch=False,clear=True,still=True,hold_s=1.,held=False,tilt=0.)
        return d|changes
    tests=[
        ('synthetic full retained released second',state(),True,1.),
        ('invalid force configuration',state(),False,0.),
        ('unknown robot',state(known=False),True,0.),
        ('latched prior invalid cannot recover',state(invalid=True),True,0.),
        ('locked below green with hand adjacent',state(pressure=249.,clear=False,held=True),True,.25),
        ('locked below green released',state(pressure=249.),True,.25),
        ('green locked hand adjacent',state(clear=False),True,.5),
        ('green held level unlocked',state(seated=False,held=True,clear=False),True,.5),
        ('green held tipped more than five degrees',state(seated=False,held=True,tilt=math.radians(5.01)),True,0.),
        ('green dropped unlocked',state(seated=False,snap_work=0.,held=False),True,0.),
        ('exact green boundary',state(pressure=250.),True,1.),
        ('above green range',state(pressure=300.01),True,0.),
        ('proximity without a loaded grasp',state(seated=False,snap_verified=False,held=True,hand_contact_n=0.),True,0.),
        ('one second not yet elapsed',state(hold_s=.999),True,.5),
        ('scene moving',state(still=False),True,.5),
        ('pressing bladder cannot fake green',state(body_touch=True),True,0.),
        ('clip seating without any snap work',state(snap_work=0.,snap_verified=False,held=False),True,0.),
    ]
    for name,s,c,want in tests:
        got=task.value(s,c)
        assert got==want,(name,got,want)
    # Broad face 0.5 mm from rod: corners exceed 10 mm but the face is touching.
    p,r=task.rod_pose(task.WINDLASS.state())
    face=[p+r@__import__('numpy').array((x,y,.0065)) for x in (-.03,0,.03) for y in (-.03,0,.03)]
    gap=task.clearance([__import__('numpy').array(face)],task.WINDLASS.state())
    assert gap<.001, gap
    # A lip's interior surface must count even when the fixed base is farther.
    import numpy as np
    c=task.CLIPS[0];cs=c.state();states={c.name:cs for c in task.CLIPS}
    point=np.array(c.pos)+c.rotation@np.array((0.,task.CLIP_MOUTH/2+task.LIP_BEAD_R,task.LIP_BEAD_Z))
    assert task.clearance([point[None,:]],task.WINDLASS.state(),states)<1e-9
    cs.links.pop('lip_a')
    try:task.clearance([point[None,:]],task.WINDLASS.state(),states)
    except ValueError:pass
    else:raise AssertionError('missing live lip pose did not fail closed')
    assert task.LOAD_SOURCE in ('estimate','measured','technician_phone')
    print(f'{len(tests)+4} synthetic boundary, release-face and fail-closed checks passed; no physical-validation claim')

def run_snap_controls():
    """Exercise the production observer accumulator, not a supplied verified flag."""
    import task,json
    import numpy as np
    def replay(rows):
        q=task.new_snap();observed=[]
        for row in rows:
            z,n,*extra=row;kind=extra[0] if extra else ''
            n=n*task.SNAP_GATE_N/10.  # synthetic forces scale with the unchanged physical gate
            f=np.array([[0.,0.,n/2],[0.,0.,n/2]])
            if kind=='one':f[1]=0.
            d=(.02 if kind=='outside' else 0.,0.,z)
            observed.append(task.snap_update(q,1 if kind=='other' else 0,d,f))
            json.dumps(q)  # instrumentation must serialize native boolean state
        return observed,q
    approach=[(.012,0),(.0105,0),(.009,0),(.006,0)]
    crossing=[(.0055,8),(.005,12),(.0045,15),(.004,20),(.0035,20),(.003,20),(.0025,20),(.002,15),(.0015,12),(.001,6),(.0005,0)]
    yes,q=replay(approach+crossing+[(0.,0)]*25);assert yes[-1] and q['work']>=.001 and q['down']>=.001
    # Below the bead, its reaction reverses and pushes the rod into the seat.
    negative_tail=[(.0025,-5),(.002,-8),(.001,0),(.0005,0)]
    yes,q=replay(approach+crossing[:6]+negative_tail);assert yes[-1],q
    asymmetric=[(.0075,8,'one'),(.007,24,'one'),(.0065,24,'one'),(.006,24,'one'),(.0055,12,'one'),(.005,8,'one'),(.0045,6,'one'),(.004,4),(.003,-4),(.001,0)]
    yes,q=replay(approach[:3]+asymmetric);assert yes[-1] and q['seen_lips']==3,q
    upper_contact=[(.012,0),(.010,8),(.0095,12),(.009,20),(.0085,20),(.008,20),(.006,20),(.003,-5),(.001,0)]
    yes,q=replay(upper_contact);assert yes[-1],q
    controls={
      'below-lip oscillation':[(.0004,20),(.0006,20)]*40,
      'no above-mouth entry':crossing,
      'force-free entry':[(z,0) for z,_ in approach+crossing],
      'below insertion force gate':[(z,min(n,9)) for z,n in approach+crossing],
      'one lip only':[(z,n,'one') for z,n in approach+crossing],
      'post-capture pulse cannot complete under-force entry':approach+[(z,min(n,9)) for z,n in crossing[:5]]+[(.0025,-5),(.0015,-5),(.001,0),(.0004,30),(.0003,40),(.0002,40),(0.,0)],
      'slow reversal cannot bank work':approach+crossing[:4]+[(.004+i*.0001,0) for i in range(1,21)]+[(.001,0)],
      'reversal after contact':approach+crossing[:4]+[(.005,20)]+crossing[4:],
      'changed rod end':approach+crossing[:4]+[(.0035,20,'other')]+crossing[5:],
      'lost alignment':approach+crossing[:4]+[(.0035,20,'outside')]+crossing[5:],
      'free gaps are not contact travel':approach[:3]+[(.0075,20),(.005,0),(.0045,20),(.001,0)],
      'isolated force spike':approach+[(.005,20),(.0045,0),(.004,0),(.001,0),(0.,0)],
      'small dither no net passage':approach+[(.004,20),(.0039,20),(.004,20)]*30+[(.001,0)],
    }
    for label,rows in controls.items():
        v,q=replay(rows);assert not any(v),(label,q)
    # A verified entry is revoked after extraction, not reused on return.
    v,q=replay(approach+crossing+[(.006,0),(.0005,20)]);assert not v[-1]
    assert task.clip_retained(True,{'lip_a':.0002,'lip_b':0.})[0]
    spread=task.ROD_DIAMETER_M-task.CLIP_MOUTH_M
    assert not task.clip_retained(True,{'lip_a':spread,'lip_b':0.})[0]
    assert not task.clip_retained(True,{'lip_a':spread/2,'lip_b':spread/2})[0]
    assert not task.clip_retained(False,{'lip_a':0.,'lip_b':0.})[0]
    print(f'{len(controls)+9} production snap-entry/reversal/force/history controls passed')

if __name__=='__main__':
    run();run_snap_controls()
