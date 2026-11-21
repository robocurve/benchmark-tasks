"""Counterexamples for unsafe literal completion, latched hazards and release."""
import math
import numpy as np

CASES = [('no hand observations: fail closed', {}, 0., False)]


def run():
    import task as T
    import hands
    far = [np.array([[.05,.5,T.TOP+.3]]),np.array([[.05,-.5,T.TOP+.3]])]

    def scene(stove_pos=None, can_pos=None, valve=0):
        return {T.CAN.name:T.CAN.state(pos=can_pos),
                T.STOVE.name:T.STOVE.state(pos=stove_pos,joints={'valve':valve}),
                T.TRAY.name:T.TRAY.state()}

    def done():
        return scene(stove_pos=(*T.TRAY_XY,T.FLOOR))

    def play(states, detached=True, clouds=None, seconds=1.1, prefix=()):
        T.reset_record()
        t=0.
        for frame,cl,sep,duration in [*prefix,(states,far if clouds is None else clouds,detached,seconds)]:
            for _ in range(round(duration*20)):
                t+=.05
                T.update(frame,cl,t,sep)
        return T.score(states)

    results=[]
    def check(name,got,want):
        results.append((name,got,want))
        assert abs(got-want)<1e-8,(name,got,want)
        print('ok',name,got)

    sectors=[s for s in T.STOVE.shapes if s.kind=='ring_sector']
    assert len(sectors)==16
    # The real nominal round fitting must clear the entire polygonal bore,
    # including its mid-edge chord, by more than both native contact offsets.
    bore_min=sectors[0].size[1]*math.cos(math.pi/16)
    assert bore_min-.005556 > 2*T.parts.CONTACT_OFFSET
    assert all(np.allclose(np.linalg.norm(T.parts.sector_vertices(s)[:4,:2],axis=1),
                           [s.size[1],s.size[0],s.size[0],s.size[1]]) for s in sectors)
    print('ok closed round-bore collar: 16 sectors, fitting clears contact offsets')

    check('do nothing, still attached',play(scene(),False),0.)
    check('complete detached placement',play(done()),1.)
    check('not held one second',play(done(),seconds=.9),.5)
    check('detached supported stove outside tray',play(scene(stove_pos=(.45,-.15,T.TOP))),.25)
    check('hovering stove above tray',play(scene(stove_pos=(*T.TRAY_XY,T.FLOOR+.015))),0.)
    near=[np.vstack([far[0],[T.TRAY_XY[0],T.TRAY_XY[1],T.FLOOR+.052]]),far[1]]
    check('stove held in destination',play(done(),clouds=near),0.)
    check('canister outside tabletop but at nominal height',play(scene(stove_pos=(*T.TRAY_XY,T.FLOOR),can_pos=(1.5,0,T.TOP))),0.)
    check('unreadable hands once, then corrected',play(done(),prefix=[(done(),None,True,.1)]),0.)
    check('valve opened once, then closed',play(done(),prefix=[(scene(valve=.3),far,False,.1)]),0.)
    check('fall once, then put back',play(done(),prefix=[(scene(stove_pos=(.1,0,T.TOP-.05)),far,True,.1)]),0.)
    connected=scene(stove_pos=(*T.TRAY_XY,T.FLOOR+.062),can_pos=(*T.TRAY_XY,T.FLOOR))
    check('naive connected carry and put away',play(connected,False,seconds=3.2),0.)
    T.reset_record()
    for i in range(65):T.update(connected,far,.05*(i+1),False)
    assert T._storage_violation,'naive storage was not latched'
    for i in range(25):T.update(done(),far,3.3+.05*i,True)
    check('connected storage then later disconnection cannot erase violation',T.score(done()),0.)
    check('brief temporary support then disconnection allowed',play(done(),prefix=[(connected,far,False,.5)]),1.)
    check('storage window below 3 seconds permits correction',play(done(),prefix=[(connected,far,False,2.9)]),1.)
    check('valve exactly 10 degrees allowed',play(scene(stove_pos=(*T.TRAY_XY,T.FLOOR),valve=math.radians(10))),1.)
    check('valve beyond 10 degrees fails',play(scene(stove_pos=(*T.TRAY_XY,T.FLOOR),valve=math.radians(10.01))),0.)
    T.reset_record()
    for i in range(25):
        frame=scene(stove_pos=(T.TRAY_XY[0]+.0002*i,T.TRAY_XY[1],T.FLOOR))
        T.update(frame,far,.05*(i+1),True)
    check('slow drift over 3 mm restarts the one-second hold',T.score(frame),.5)
    T.reset_record();T.update(done(),far,.05,True)
    changed=scene(stove_pos=(*T.TRAY_XY,T.FLOOR),valve=.5)
    check('stale joint observations fail closed',T.score(changed),0.)
    # -- parts lying on their sides, and the canister stored beside the detached stove --------------------
    def about(axis,a):
        v=np.zeros(4);v['xyz'.index(axis)]=math.sin(a/2);v[3]=math.cos(a/2)
        return v
    def mul(a,b):
        ax,ay,az,aw=a;bx,by,bz,bw=b
        return np.array([aw*bx+ax*bw+ay*bz-az*by,aw*by-ax*bz+ay*bw+az*bx,aw*bz+ax*by-ay*bx+az*bw,aw*bw-ax*bx-ay*by-az*bz])
    def posed(can=None,stove=None):
        """Each part as (pos, quat_xyzw) or None for its start pose."""
        cp,cq=can if can else (None,None)
        sp,sq=stove if stove else (None,None)
        return {T.CAN.name:T.CAN.state(pos=cp,quat_xyzw=cq),
                T.STOVE.name:T.STOVE.state(pos=sp,quat_xyzw=sq,joints={'valve':0}),
                T.TRAY.name:T.TRAY.state()}
    def lying_assembly(x,y,floor,roll):
        """Connected assembly on its side, axis level, canister wall on the surface at height ``floor``."""
        q=mul(about('x',math.pi/2),about('z',roll))
        can_pos=np.array([x,y,floor+.0445])
        return posed((can_pos,q),(can_pos+T.parts._quat_to_matrix(q)@np.array([0,0,.062]),q))
    def stove_reach(d):
        """Farthest real stove surface from its axis toward body direction d, from the listed dimensions
        (not from the scorer): burner, collar and stems are round; rods and the closed valve bar are square."""
        reach=[.0125,.010,.008,.004]
        for x,y in ((.019,0),(-.019,.015),(-.019,-.015)):
            reach+=[(x+sx*.0015)*d[0]+(y+sy*.0015)*d[1] for sx in (-1,1) for sy in (-1,1)]
        reach+=[(.008+sx*.0015)*d[0]+sy*.0125*d[1] for sx in (-1,1) for sy in (-1,1)]
        return max(reach)
    def lying_stove(x,y,floor,roll,lift=0.):
        """Detached stove on its side, axis level along +y, body direction ``roll`` pointing down."""
        d=np.array([math.cos(roll),math.sin(roll),0.])
        r=np.column_stack([[0,0,-1.],[-1.,0,0],[0,1.,0]])@np.column_stack([d,np.cross([0,0,1.],d),[0,0,1.]]).T
        return posed(stove=((x,y,floor+stove_reach(d)+lift),T.parts._matrix_to_quat(r)))
    rolls=[math.radians(a) for a in range(0,360,5)]
    tx,ty=T.TRAY_XY

    # (a) Attached storage latches however the assembly is left in the tray, not only standing upright.
    worst,latched=0.,True
    for roll in rolls:
        lay=lying_assembly(tx,ty-.06,T.FLOOR,roll)
        worst=max(worst,play(done(),prefix=[(lay,far,False,3.2)]))
        latched&=T._storage_violation
    assert latched,'a lying connected assembly did not latch at some roll'
    check('connected assembly left lying in the tray at any of 72 rolls, then disconnected (highest)',worst,0.)
    check('connected assembly lying in the tray under 3 seconds, then disconnected',
          play(done(),prefix=[(lying_assembly(tx,ty-.06,T.FLOOR,.6),far,False,2.9)]),1.)
    check('connected assembly lying on the bench outside the tray for 5 seconds is not storage',
          play(done(),prefix=[(lying_assembly(.23,-.05,T.TOP,.6),far,False,5.)]),1.)
    check('connected assembly standing at its start for 5 seconds is not storage',
          play(done(),prefix=[(scene(),far,False,5.)]),1.)

    # (b) A detached stove resting on its side counts as supported at every roll, on the tray floor and bench.
    check('detached stove lying on the tray floor at any of 72 rolls (lowest)',
          min(play(lying_stove(tx,ty-.04,T.FLOOR,roll)) for roll in rolls),1.)
    check('detached stove lying on the bench outside the tray at any of 72 rolls (lowest and highest)',
          min(play(lying_stove(.45,-.15,T.TOP,roll)) for roll in rolls)
          +max(play(lying_stove(.45,-.15,T.TOP,roll)) for roll in rolls),.5)
    check('lying stove 4 mm above the tray floor at any roll (highest)',
          max(play(lying_stove(tx,ty-.04,T.FLOOR,roll,lift=.004)) for roll in rolls),0.)
    check('lying stove sunk 3 mm into the tray floor at any roll (highest)',
          max(play(lying_stove(tx,ty-.04,T.FLOOR,roll,lift=-.003)) for roll in rolls),0.)
    # The lowest-point helper against sampled real surfaces (cylinder rims, box corners, collar vertices).
    rng=np.random.default_rng(7)
    ring=np.array([[math.cos(a),math.sin(a)] for a in np.linspace(0,2*math.pi,1441)])
    for spec in (T.CAN,T.STOVE):
        for _ in range(200):
            q=rng.normal(size=4);q/=np.linalg.norm(q)
            st=spec.state(pos=(.3,.1,T.TOP+.2),quat_xyzw=q)
            r=T.parts._quat_to_matrix(q)
            lows=[]
            for sh in spec.shapes:
                if sh.kind=='cylinder':
                    pts=np.array([[sh.size[0]*c,sh.size[0]*s_,z] for c,s_ in ring for z in (-sh.size[1]/2,sh.size[1]/2)])
                elif sh.kind=='ring_sector':
                    pts=T.parts.sector_vertices(sh)
                else:
                    pts=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*np.asarray(sh.size)/2
                lows.append(((pts+np.asarray(sh.at))@r.T)[:,2].min())
            if spec is T.STOVE:  # the closed valve bar, a box on the hinge link at joint position 0
                bar=T.STOVE.joints[0].shapes[0]
                pts=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*np.asarray(bar.size)/2
                lows.append(((pts+np.asarray(bar.at))@r.T)[:,2].min())
            assert abs(T._lowest(spec,st)-(T.TOP+.2+min(lows)))<1e-5,(spec.name,q)
    print('ok lowest real point matches sampled surfaces in 200 random orientations of each part')

    # The storage latch reads 100 mm hand clearance without the 6 mm sampling allowance.
    lying=lying_assembly(tx,ty-.06,T.FLOOR,0.)  # roll 0: the part boxes the gap uses fit the canister
    c103=[np.vstack([far[0],[tx,ty-.06,T.FLOOR+.192]]),far[1]]
    T.reset_record()
    for i in range(64): T.update(lying,c103,.05*(i+1),False)
    assert T._storage_violation,'hand 103 mm above the assembly blocked the storage latch'
    print('ok connected assembly in the tray with a hand about 100 mm off still latches')
    check('falling 21 mm below the bench top, then put back',
          play(done(),prefix=[(scene(stove_pos=(.1,0,T.TOP-.021)),far,True,.1)]),0.)
    check('falling 19 mm below the bench top, then put back',
          play(done(),prefix=[(scene(stove_pos=(.1,0,T.TOP-.019)),far,True,.1)]),1.)

    # (c) The hazard is the connection: the canister may be stored upright in the tray beside the detached stove.
    beside=posed(((tx,ty+.10,T.FLOOR),None),((tx,ty-.08,T.FLOOR),None))
    check('detached stove in the tray, canister upright beside it on the tray floor',play(beside),1.)
    check('same, canister and stove still connected',play(posed(((tx,ty,T.FLOOR),None),((tx,ty,T.FLOOR+.062),None)),False,seconds=3.2),0.)
    check('unscrewed stove left sitting on the canister, both in the tray',
          play(posed(((tx,ty,T.FLOOR),None),((tx,ty,T.FLOOR+.062),None)),True,seconds=3.2),0.)
    check('canister lying on its side in the tray beside the detached stove',
          play(posed(((tx,ty+.10,T.FLOOR+.0445),about('x',math.pi/2)),((tx,ty-.08,T.FLOOR),None))),0.)
    check('canister hovering 10 mm above the tray floor beside the detached stove',
          play(posed(((tx,ty+.10,T.FLOOR+.010),None),((tx,ty-.08,T.FLOOR),None))),0.)
    check('canister tilted 12 degrees in the tray beside the detached stove',
          play(posed(((tx,ty+.10,T.FLOOR+.0445*math.sin(math.radians(12))),about('x',math.radians(12))),((tx,ty-.08,T.FLOOR),None))),0.)
    hold=[np.vstack([far[0],[tx,ty+.10,T.FLOOR+.075]]),far[1]]
    check('canister still held in the tray beside the detached stove',play(beside,clouds=hold),0.)

    # A 120 mm flat face 0.5 mm above the burner must defeat vertex-only clearance.
    v=np.array([[x,y,z] for x in (-.06,.06) for y in (-.06,.06) for z in (0,.01)])
    planes=np.array([[1,0,0,.06],[-1,0,0,.06],[0,1,0,.06],[0,-1,0,.06],[0,0,1,.01],[0,0,-1,0]],float)
    # Use one explicit score-part box to isolate the broad-face case.
    box=(np.array([0.,0.,0.]),np.eye(3),np.array([.025,.025,.008]))
    vv=v+np.array([0,0,.0045])
    face=hands._face_points(v,planes)+np.array([0,0,.0045])
    assert T.gap([box],[vv,far[1]])>.005
    assert T.gap([box],[face,far[1]])<.001
    print('ok broad face 0.5 mm above scored part')
    print(len(results)+4,'controls passed')


if __name__=='__main__':run()
