"""Known-layout two-hand repair commissioning. The controller sends ordinary joint targets."""
import json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from yam_arena import ik,station
from yam_arena.constants import READY
import repair_scene as g

NEUTRAL=R.from_matrix([[0,0,1],[0,-1,0],[1,0,0]])
LEFT=R.from_euler("y",-17,degrees=True)*R.from_matrix([[0,-1,0],[-1,0,0],[0,0,-1]])


class Plan:
    def __init__(self):
        self.command=np.r_[READY,1.,READY,1.]
        self.rows=[]
        self.checks=[]
        self.carry=False

    def move(self,label,n,xyz=None,roll=0.,opening=None,yaw=0.,side='right',feedback=None):
        off=7 if side=='right' else 0
        start=self.command.copy();end=start.copy()
        if xyz is not None:
            ori=R.from_euler('x',min(0.,280*(xyz[1]-.025)),degrees=True)*LEFT if side=='left' else R.from_euler('z',yaw,degrees=True)*R.from_euler('x',roll,degrees=True)*NEUTRAL
            if side=='left' and (label.startswith('Align ') or label.startswith('Back clear ')):
                ori=R.from_euler('y',-12,degrees=True)*ori
            if (self.carry or getattr(self,'park',False)) and side=='right':ori=R.from_euler('y',28,degrees=True)*NEUTRAL
            seed=start[off:off+6]
            if np.array_equal(seed,READY):
                seed=[1.2,1.8,.65,1.2,1.1,-1.5] if off else [-.64,1.7,1.5,-1.3,0.,-.6]
            q,d,a=ik.solve(xyz,ori.as_quat(),q_init=seed,base_pos=station.BASES[side],iters=320,tol=4e-5)
            self.checks.append(dict(label=label,mm=float(1000*d),deg=float(np.rad2deg(a))))
            if d>.0015 or a>.015:
                raise ValueError((label,xyz,side,d,a))
            end[off:off+6]=q
        if opening is not None:end[off+6]=opening
        for i in range(n):
            row=dict(label=label,action=(start+ik.smoothstep((i+1)/n)*(end-start)).tolist())
            if feedback is not None:row['repair_drive']=feedback
            self.rows.append(row)
        self.command=end

    def tool(self,label,n,xyz=None,opening=None):
        self.move(label,n,xyz,opening=opening,side='left')

    def transit(self,label,n,pose,y,z,wood='strip',front='wall',depth=-.039):
        self.tool(label,n,pose)
        for row in self.rows[-n:]:
            row['repair_drive']=dict(wood=wood,front=front,y=y,z=z,depth=depth,transit=True)

    def drive(self,name,front,wood,y,z):
        # Acquire the site from the front after travelling in free space.
        pose=np.array([g.FRONT-g.TIP-.008,y,z+.004])
        self.transit('Align '+name,42,pose,y,z,wood,front)
        for label,n,depth in [('Approach',26,-.002),('Drive',70,.0322),('Withdraw',18,-.039)]:
            feedback=dict(wood=wood,front=front,y=y,z=z,depth=depth)
            self.move(label+' '+name,n,feedback=feedback)
        self.transit('Back clear '+name,14,pose+[-.009,0,0],y,z,wood,front)

    def stock(self,stock,z,name):
        mid=np.array([g.FRONT-.050,0,g.MID])
        back=g.FRONT+g.BOARD+g.THICK/2+.019
        self.move('Above '+name,52,stock+[(-.057 if name.startswith('upper') else -.055),0,.125],opening=.62)
        self.move('Enter '+name+' saddles',35,stock+[(-.045 if name.startswith('upper') else -.055),0,0],opening=.62)
        self.move('Surround '+name,24,stock,opening=.54)
        self.move('Grip '+name,16,opening=0.)
        if name.startswith('upper'):
            self.move('Slide upper backer out of shelf',21,stock+[0,.031,.0003])
        lift=np.array([.296,.140,g.MID+.013]) if name.startswith('lower') else stock+[0,.031,.080]
        self.move('Lift '+name,28,lift)
        if name.startswith('lower'):
            # The installed upper strip blocks a second face-on diagonal.
            # Feed the leading end through the opening, sweep sideways in
            # the closed cavity, then square the strip below its neighbour.
            # The lower strip passes the jambs obliquely in both planes.
            # A raised start keeps the trailing end clear of the empty stand.
            points=[(.277,.004,.938,0,0,45),(.277,.001,.900,0,24,27),
                    (.309,.030,.898,24,24,27),(.321,.026,.898,26,24,15),
                    (.351,-.003,.898,26,24,27),(.367,-.030,.898,13,24,22),
                    (.385,-.022,.898,0,24,20),(.383,0,.895,0,0,23),
                    (.371,0,z,0,0,24)]
            reference=json.loads(Path(__file__).with_name('lower-feed.json').read_text())['samples']
            cursor=0
            for i,(x,y,h,yaw,roll,n) in enumerate(points):
                self.move('Side feed lower backer '+str(i+1),n,[x,y,h+.007],roll=roll,yaw=yaw)
                for row in self.rows[-n:]:
                    row['stock_feed']=reference[cursor];cursor+=1
            assert cursor==len(reference)
            return
        self.move('Before opening '+name,40,mid+[-.03,0,0])
        self.move('Turn '+name+' diagonal',38,mid+[-.03,0,0],45.)
        self.move('Present '+name,22,mid,45.)
        self.move('Feed '+name,46,[back,0,g.MID],45.)
        self.move('Rotate '+name+' in cavity',36,[back,0,g.MID],0.)
        self.move('Set working height '+name,36,[back,0,z+.010],yaw=-3.)
        self.move('Pull '+name+' to back of wall',30,[back-.023,0,z+.010],yaw=-3.)

    def release(self,z):
        self.move('Release secured backing',14,opening=.72)
        self.move('Draw fingers out',36,[g.FRONT-.076,0,z+.010],yaw=-3.)
        self.move('Clear wall',12,[.218,-.03,z+.006])
        self.park=True
        self.move('Clear wall',18,[.218,-.202,g.MID+.012])
        self.park=False


def concurrent(rows,start,finish,other_start,other_finish,active):
    """Overlap independent, adjacent arm moves, retaining all controller fields."""
    def span(first,last,lo=0):
        a=next(i for i in range(lo,len(rows)) if rows[i]['label']==first)
        j=next(i for i in range(a,len(rows)) if rows[i]['label']==last)
        b=j
        while b<len(rows) and rows[b]['label']==last:b+=1
        return a,b
    a,b=span(start,finish);c,d=span(other_start,other_finish,b)
    if c!=b:raise ValueError('Concurrent segments must be adjacent')
    left=rows[a:b];right=rows[c:d];merged=[]
    for i in range(max(len(left),len(right))):
        r1=left[min(i,len(left)-1)];r2=right[min(i,len(right)-1)]
        l,r=(r1,r2) if active=='left' else (r2,r1)
        item=dict(label=r1['label']+' / '+r2['label'],action=l['action'][:7]+r['action'][7:])
        if 'repair_drive' in l:item['repair_drive']=l['repair_drive']
        if 'repair_pose' in r:item['repair_pose']=r['repair_pose']
        if 'unhook' in r:item['unhook']=r['unhook']
        if 'stock_target' in r:item['stock_target']=r['stock_target']
        if 'wood_name' in r:item['wood_name']=r['wood_name']
        if 'park_tool' in l:item['park_tool']=l['park_tool']
        if r['label']=='Release secured backing':item['hold_wood']=True
        merged.append(item)
    rows[a:d]=merged


def pack_schedule(p):
    rows=p.rows
    concurrent(rows,'Feed upper backer','Pull upper backer to back of wall','Approach driver stand','Lift out of stand','right')
    for which in ('upper','lower'):
        concurrent(rows,'Withdraw '+which+' positive jamb','Back clear '+which+' positive jamb','Release secured backing','Clear wall','left')
    park=next(i for i,r in enumerate(rows) if r['label']=='Above lower backer')
    wait=dict(rows[park],action=rows[park-1]['action'])
    rows[park:park]=[wait.copy() for _ in range(54)]
    concurrent(rows,'Park beyond opening','Park beyond opening','Above lower backer','Enter lower backer saddles','left')
    concurrent(rows,'Clear lifting path','Clear lifting path','Approach lifting handle','Take lifting handle','left')
    concurrent(rows,'Withdraw patch spare_strip 0.078','Back clear patch spare_strip 0.078','Unhook carrying pins','Pull handle from face','left')
    concurrent(rows,'Withdraw handle','Withdraw handle','Align patch spare_strip -0.052','Align patch spare_strip -0.052','right')
    # Shorten empty-space transits only. Feed, release and edge registration
    # retain their number of 20 Hz control steps.
    from itertools import groupby
    result=[]
    for label,grp in groupby(rows,key=lambda r:r['label']):
        chunk=list(grp)
        factor=.78 if label.startswith(('Align ','Above ','Before opening ','Enter ','Turn ', 'Rotate ','Present ','Draw board','Carry ','Move in front','Back clear ','Clear wall','Pull handle')) else 1.
        if label.startswith('Approach ') and ('jamb' in label or label.startswith('Approach patch ')):factor=.62
        if label.startswith('Back clear '):factor=.36
        if label.startswith('Drive '):factor=.925
        if label.startswith('Side feed lower backer '):factor=1.
        if label in ('Approach above front face','Above upper backer','Take lifting handle','Turn upper backer diagonal'):factor=.65
        if label=='Inspect released repair':factor=.90
        if label.startswith('Withdraw ') and '/' not in label:factor=.61
        if '/ Unhook carrying pins' in label or '/ Pull handle' in label or '/ Withdraw handle' in label:factor=1.
        selected=np.linspace(0,len(chunk)-1,max(1,int(np.ceil(factor*len(chunk))))).round().astype(int)
        result.extend(chunk[i] for i in selected)
    p.rows=result


def park(p,label,n):
    p.tool(label,n,g.HANDLE+[0,0,.070])
    for i,row in enumerate(p.rows[-n:]):
        row['park_tool']=[label,(i+1)/n]


def park_joints(policy,entry,obs,row,poses,paths):
    from inspect_robots_yam.packing import STATE_KEY
    label,fraction=entry['park_tool']
    if not hasattr(policy,'park_paths'):
        policy.park_paths=json.loads(Path(__file__).with_name('park-path.json').read_text())
    waypoints=np.asarray(policy.park_paths[label])
    target=np.array([np.interp(fraction,waypoints[:,0],column) for column in waypoints[:,1:].T])
    if getattr(policy,'parking_label',None)!=label:
        policy.parking_label=label
        policy.park_command=np.asarray(obs.state[STATE_KEY])[:6].copy()
    delta=target-policy.park_command
    policy.park_command+=delta*min(1.,.055/max(1e-9,np.max(abs(delta))))
    row[:6]=policy.park_command
    policy.drive_command=policy.park_command.copy()
    return row


def make(mode):
    """Commission both loose backing strips in the complete wall."""
    from types import SimpleNamespace
    from reach import solve,SEEDS
    from yam_arena.demo import grip_rotation
    rows=json.loads(Path(__file__).with_name('upper-plan.json').read_text())
    left=np.r_[READY,1.]
    # From the original laydown point the nose is under the spare-stock rack.
    # Draw it towards the operator side before lifting through the rack height.
    pc=[]
    attitude=R.from_matrix(grip_rotation(-np.pi/2))
    stages=[
        (50,[0.,0.,.090],.85),(38,[0.,0.,-.002],.85),
        (20,[0.,0.,-.002],.39),(18,[0.,0.,-.002],.39),
        (38,[0.,0.,.074],.39),(30,[0.,0.,.074],.39),
    ]
    for n,offset,opening in stages:
        goal,delta,angle=solve(g.HANDLE+offset,attitude,[left[:6],*SEEDS],'left')
        if delta>.0018 or angle>.015:raise ValueError(('wall pickup',offset,delta,angle))
        next_=np.r_[goal,opening]
        pc.extend(left+ik.smoothstep((i+1)/n)*(next_-left) for i in range(n))
        left=next_
    start=next(i for i,row in enumerate(rows) if 'Approach driver stand' in row['label'])
    start-=36  # Left hand can approach the driver while the right presents the first timber.
    end=next(i for i,row in enumerate(rows) if 'repair_drive' in row)
    import copy
    extra=[copy.deepcopy(rows[end-1]) for _ in range(len(pc)-(end-start))]
    for step in extra:step['label']='Clear spare-stock rack with held driver'
    rows[end:end]=extra
    for i,row in enumerate(rows[start:]):
        row['action'][:7]=pc[min(i,len(pc)-1)].tolist()
    base=next(row for row in rows if 'repair_drive' in row)
    head=rows[:rows.index(base)]
    base=dict(base,stock_target=list(base['stock_target']),repair_drive=dict(base['repair_drive']))
    base['stock_target'][2]-=.028; base['repair_drive']['z']-=.028
    stages=[('Lift above base',98),('Skirt stock rack',68),('Retract clear of jamb',55),('Traverse to upper wood',100),
            ('Register upper entry',47),('Press first fixing',220),('Withdraw first fixing',90)]
    for label,num in stages:
        head.extend(dict(base,label=label) for _ in range(num))
    # One installed screw carries the light backer while the right hand clears the magazine.
    for label,n in [('Open stock jaws',36),('Draw clear of stock',90),('Clear opposite jamb',105)]:
        for k in range(n):
            step=dict(base, label=label, action=base['action'].copy(), clear_stock=[label,(k+1)/n])
            step.pop('stock_target',None)
            step['action'][13]=.85*(ik.smoothstep((k+1)/n) if label=='Open stock jaws' else 1.)
            head.append(step)
    opposite=dict(base,repair_drive=dict(base['repair_drive'],y=-.108),action=base['action'].copy(),clear_stock=['Stay clear',1.])
    opposite.pop('stock_target',None)
    opposite['action'][13]=.85
    for label,num in [('Pass ahead of stock hand',145),('Register second entry',48),('Press second fixing',230),('Withdraw second fixing',135)]:
        head.extend(dict(opposite,label=label) for _ in range(num))
    park=dict(opposite,repair_drive=dict(opposite['repair_drive'],y=.232))
    for i in range(215):
        head.append(dict(park,label='Park for lower stock'))
    loader=Plan()
    stowed,dx,da=ik.solve([.180,-.235,.994],(R.from_euler('y',28,degrees=True)*NEUTRAL).as_quat(),q_init=[1.16,.63,.86,-1.16,.94,-.75],base_pos=station.BASES['right'],iters=100,tol=.0001)
    assert dx<.002 and da<.025
    loader.command[7:13]=stowed;loader.command[13]=.85
    loader.stock(g.SECOND,g.MID-.044,'lower backer')
    overlap=32
    for parent,child in zip(head[-overlap:],loader.rows[:overlap],strict=True):
        parent.pop('clear_stock',None)
        parent['action']=parent['action'][:7]+child['action'][7:]
    for step in loader.rows[overlap:]:
        r=dict(step,action=step['action'].copy(),repair_drive=park['repair_drive'],park_lower=True)
        head.append(r)
    work=dict(base,action=head[-1]['action'].copy(),wood_name='spare_strip',
              repair_drive=dict(wood='spare_strip',front='wall',y=.114,z=g.MID-.044,depth=-.002),
              stock_target=[.371,0.,g.MID-.044,0.])
    for label,n in [('Approach lower strip',110),('Register lower entry',54),('Press third fixing',260),('Withdraw third fixing',130)]:
        head.extend(dict(work,label=label) for _ in range(n))
    for label,n in [('Open stock jaws',36),('Draw clear of stock',85),('Clear opposite jamb',85)]:
        for k in range(n):
            step=dict(work,label=label,action=work['action'].copy(),clear_stock=[label,(k+1)/n])
            step.pop('stock_target',None)
            step['action'][13]=.85*(ik.smoothstep((k+1)/n) if label=='Open stock jaws' else 1.)
            head.append(step)
    fourth=dict(opposite,repair_drive=dict(work['repair_drive'],y=-.108))
    for label,n in [('Pass ahead of stock hand',155),('Register second entry',52),('Press fourth fixing',250),('Withdraw fourth fixing',155)]:
        head.extend(dict(fourth,label=label) for _ in range(n))
    # When both rows are in place the other hand brings the loose panel to the opening.
    park_panel=dict(fourth,repair_drive=dict(fourth['repair_drive'],y=.232))
    for i in range(205):
        head.append(dict(park_panel,label='Park for lower stock'))
    panel=Plan()
    panel.command[7:13]=stowed
    panel.command[13]=.85
    target=place(panel)
    for step in panel.rows:
        head.append(dict(step,park_lower=True,repair_drive=park_panel['repair_drive']))
    def panel_site(name,wood,y,pocket,hold):
        drive=dict(front='patch',wood=wood,y=y,z=0.,depth=0.)
        for phase,depth,n in [('Travel',-.037,125),('Register',-.002,52),('Seat',.033,205),('Unload',-.042,125)]:
            for _ in range(n):
                step=dict(label=phase+' '+name,action=panel.command.tolist(),repair_drive=drive,panel_phase=phase,depth=depth,pocket=pocket)
                if hold:
                    step['repair_pose']=g.FRONT+g.BOARD/2
                head.append(step)
    panel_site('upper panel positive','strip',.075,5,True)
    panel_site('lower panel positive','spare_strip',.075,6,True)
    # Draw the two real studs through their enlarged openings before leaving with the handle.
    panel.rows=[]
    for label,n,offset in [('Unhook carrying pins',35,0.),('Pull handle from face',40,-.075)]:
        panel.move(label,n,target+[offset,0,-.0135])
        for step in panel.rows[-n:]:
            step['unhook']=offset
    panel.move('Withdraw handle',70,[.212,-.235,.970])
    # Keep the freed lifting handle outside the driver's far-side upward path.
    # Rotate the carrying arm about its base only after the studs leave the panel.
    q_near=panel.command.copy()
    q_away=q_near.copy()
    q_away[7]-=.85
    for i in range(56):
        fraction=ik.smoothstep((i+1)/56)
        panel.rows.append(dict(label='Park free lifting handle',action=(q_near+fraction*(q_away-q_near)).tolist()))
    panel.command=q_away
    for step in panel.rows:
        head.append(dict(step,repair_drive=dict(front='patch',wood='spare_strip',y=.075,z=0.,depth=-.042),
                         panel_phase='Travel',depth=-.042))
    panel_site('lower panel negative','spare_strip',-.068,7,False)
    panel_site('upper panel negative','strip',-.068,8,False)
    for _ in range(80):
        head.append(dict(head[-1],label='Observe released panel'))
    rows=head
    return SimpleNamespace(rows=rows)

def place(p):
    p.carry=True
    grip=g.PICK
    p.move('Approach lifting handle',40,grip+[-.054,0,.029],opening=.52)
    p.move('Take lifting handle',35,grip+[.001,0,.0005],opening=.52)
    p.move('Close on handle',21,opening=0.)
    p.move('Raise loose patch from rack',42,grip+[0,0,.100])
    target=np.array([g.FRONT+g.BOARD/2-.044,g.GRIP_Y,g.MID+g.HANDLE_Z])
    for label,n,goal in [
        ('Lift clear above staging posts',47,[.275,-.320,.999]),
        ('Carry above front work area',77,[.275,0,.999]),
        ('Approach above front face',27,[.330,0,.999]),
        ('Register four edges at opening',38,[.330,0,g.MID]),
        ('Insert patch squarely',54,[g.FRONT+g.BOARD/2,0,g.MID])]:
        p.move(label,n)
        for row in p.rows[-n:]:row['repair_pose']=goal
    return target



def carry_increment(policy,attribute,measured,piece,goal,rotation,limits):
    # Increment the last commanded wrist pose about the *observed* workpiece.
    # This supplies steady load compensation without racing a desired path
    # ahead of a stalled panel or banking an unreachable Cartesian offset.
    commanded=getattr(policy,attribute,measured.copy())
    wrist,mat=ik.tcp_pose(commanded,base_pos=station.BASES['right'])
    actual=R.from_quat(piece[3:])
    delta=np.clip(.24*(goal-piece[:3]),-limits,limits)
    turn=.16*(rotation*actual.inv()).as_rotvec()
    norm=np.linalg.norm(turn)
    if norm>.027:turn*=.027/norm
    for scale in (1.,.45,.18):
        dr=R.from_rotvec(scale*turn)
        point=piece[:3]+scale*delta+dr.apply(wrist-piece[:3])
        attitude=dr*R.from_matrix(mat)
        q,d,a=ik.solve(point,attitude.as_quat(),q_init=commanded,base_pos=station.BASES['right'],iters=16,tol=.00008)
        if d<.0015 and a<.024:
            commanded+=np.clip(q-commanded,-.042,.042)
            break
    commanded=np.clip(commanded,measured-.17,measured+.17)
    setattr(policy,attribute,commanded)
    policy.carry_diagnostics=dict(goal=goal.tolist(),part=piece[:3].tolist(),attitude_deg=actual.as_euler('xyz',degrees=True).tolist(),ik_mm=1000*d,ik_rad=a)
    return commanded


def feed_recorded_stock(policy,entry,obs,row,poses,paths):
    # Feed-forward comes from a successful physical passage of this loose strip.
    # A bounded correction slowly accommodates millimetric pickup differences.
    sample=entry['stock_feed']
    part=poses[paths.index('/World/envs/env_0/spare_strip')]
    desired=np.asarray(sample['piece'])
    q=np.asarray(sample['q'])
    if not hasattr(policy,'feed_offset'):
        policy.feed_offset=np.zeros(3);policy.feed_twist=np.zeros(3)
    error=desired[:3]-part[:3]
    twist=(R.from_quat(desired[3:])*R.from_quat(part[3:]).inv()).as_rotvec()
    policy.feed_offset=np.clip(.97*policy.feed_offset+.055*error,-.009,.009)
    policy.feed_twist=np.clip(.98*policy.feed_twist+.045*twist,-.08,.08)
    p,m=ik.tcp_pose(q,base_pos=station.BASES['right'])
    correction=R.from_rotvec(policy.feed_twist)
    tcp=desired[:3]+policy.feed_offset+correction.apply(p-desired[:3])
    adjusted,d,a=ik.solve(tcp,(correction*R.from_matrix(m)).as_quat(),q_init=q,base_pos=station.BASES['right'],iters=16,tol=.00008)
    row[7:13]=adjusted if d<.0015 and a<.025 else q
    policy.carry_diagnostics=dict(goal=desired[:3].tolist(),part=part[:3].tolist(),ik_mm=1000*d,ik_rad=a)
    return row


def track_stock(policy,entry,obs,row,poses,paths):
    from inspect_robots_yam.packing import STATE_KEY
    q0=np.asarray(obs.state[STATE_KEY])[7:13]
    c,m=ik.tcp_pose(q0,base_pos=station.BASES['right'])
    name=entry.get('wood_name','spare_strip')
    piece=poses[paths.index('/World/envs/env_0/'+name)]
    hand=R.from_matrix(m);wood=R.from_quat(piece[3:])
    off=hand.inv().apply(piece[:3]-c);rel=hand.inv()*wood
    if getattr(policy,'wood_name',None)!=name:
        policy.wood_name=name
        policy.wood_q=q0.copy();policy.wood_ref=piece[:3].copy()
        policy.wood_off=off;policy.wood_rel=rel
        policy.wood_angle=wood.as_euler('xyz')[2];policy.wood_roll=wood.as_euler('xyz')[0]
        policy.wood_bias=np.zeros(3);policy.wood_tilt=np.zeros(3)
    if np.linalg.norm(off)>.09:
        row[7:13]=policy.wood_q
        return row
    pose=entry['stock_target'];goal=np.array(pose[:3]);angle=np.deg2rad(pose[3])
    policy.wood_off=.90*policy.wood_off+.1*off
    policy.wood_rel=policy.wood_rel*R.from_rotvec(.10*(policy.wood_rel.inv()*rel).as_rotvec())
    policy.wood_ref+=np.clip(.19*(goal-policy.wood_ref),-.0037,.0037)
    policy.wood_angle+=np.clip(angle-policy.wood_angle,-.025,.025)
    policy.wood_roll+=np.clip(np.deg2rad(pose[4] if len(pose)>4 else 0.)-policy.wood_roll,-.021,.021)
    refrot=R.from_euler('z',policy.wood_angle)*R.from_euler('x',policy.wood_roll)
    policy.wood_bias=np.clip(policy.wood_bias+np.clip(.095*(policy.wood_ref-piece[:3]),-.0013,.0013),-.030,.030)
    policy.wood_tilt=np.clip(policy.wood_tilt+.065*(refrot*wood.inv()).as_rotvec(),-.17,.17)
    rw=R.from_rotvec(policy.wood_tilt)*refrot*policy.wood_rel.inv()
    tcp=policy.wood_ref+policy.wood_bias-rw.apply(policy.wood_off)
    q,d,a=ik.solve(tcp,rw.as_quat(),q_init=policy.wood_q,base_pos=station.BASES['right'],iters=30,tol=.00006)
    if d<.0015 and a<.018:
        change=q-policy.wood_q
        policy.wood_q+=change*min(1.,.049/max(1.e-9,np.max(abs(change))))
    policy.carry_diagnostics=dict(goal=goal.tolist(),part=piece[:3].tolist(),attitude_deg=wood.as_euler('xyz',degrees=True).tolist(),ik_mm=1000*d,ik_rad=a)
    row[7:13]=policy.wood_q
    return row


def align_patch(policy,entry,obs,row,poses,paths):
    from inspect_robots_yam.packing import STATE_KEY
    measured=np.asarray(obs.state[STATE_KEY])[7:13]
    patch=poses[paths.index('/World/envs/env_0/patch')]
    hand,_=ik.tcp_pose(measured,base_pos=station.BASES['right'])
    if np.linalg.norm(hand-patch[:3])>.16:return row
    goal=np.asarray(entry['repair_pose'],float)
    if goal.ndim==0:goal=np.array([float(goal),0.,g.MID])
    else:goal=goal.copy()
    rot=R.from_quat(patch[3:])
    if goal[0]>g.FRONT-.001 and patch[0]<g.FRONT-.004:
        if np.max(abs(patch[1:3]-goal[1:]))>.0018 or np.linalg.norm(rot.as_rotvec())>.018:
            goal[0]=min(patch[0],g.FRONT-.010)
    if goal[2]<.97 and patch[0]<.318:goal[2]=.994
    if goal[1]>-.10 and patch[1]<-.25 and patch[2]<.991:goal[1]=patch[1]
    row[7:13]=carry_increment(policy,'place_q',measured,patch,goal,R.identity(),np.array([.0031,.0058,.0034]))
    return row


def unhook(policy,entry,obs,row,poses,paths):
    from inspect_robots_yam.packing import STATE_KEY
    patch=poses[paths.index('/World/envs/env_0/patch')]
    carrier=poses[paths.index('/World/envs/env_0/carrier')]
    q0=np.asarray(obs.state[STATE_KEY])[7:13]
    if not hasattr(policy,'withdraw_q'):policy.withdraw_q=getattr(policy,'place_q',q0).copy()
    panel_rotation=R.from_quat(patch[3:])
    offset=panel_rotation.inv().apply(carrier[:3]-patch[:3])
    rel=panel_rotation.inv()*R.from_quat(carrier[3:])
    # Both physical heads must fit. A small roll is harmless if neither edge
    # of either head catches the paper, so check their actual extents.
    pins=np.array([[.0519,-.038,.0032],[.0519,.038,.0032]])
    centres=offset+rel.apply(pins)
    half=np.abs(rel.as_matrix())@np.array([.0009,.002,.002])
    margin=.00025
    aligned=(np.max(abs(centres[:,1]-pins[:,1]))+half[1]<g.ENTRY_HALF_WIDTH-margin
             and min(centres[:,2])-half[2]>g.HANDLE_Z+g.ENTRY_BOTTOM+margin
             and max(centres[:,2])+half[2]<g.HANDLE_Z+g.ENTRY_TOP-margin)
    # The guide stops carrying the wrist load as the studs enter the release
    # openings. Drop the old force-producing set point at that transition.
    if g.CURRENT.key_release is not None and not getattr(policy,'free_carrier',False):
        policy.withdraw_q=q0.copy()
        policy.free_carrier=True
    within_board=offset[0]>-.063
    if g.CURRENT.key_release is None:
        x=-.044
    elif within_board:
        x=offset[0]-(.009 if aligned else 0.)
    else:
        x=-.119
    mean=centres.mean(0)
    target=carrier[:3]+panel_rotation.apply([x-offset[0],-mean[1],g.HANDLE_Z+(g.ENTRY_BOTTOM+g.ENTRY_TOP)/2-mean[2]])
    if g.CURRENT.key_release is None:target=patch[:3]+panel_rotation.apply([-.044,0,g.HANDLE_Z-.013])
    limits=np.array([.0025 if within_board else .0034,.0008,.0008])
    row[7:13]=carry_increment(policy,'withdraw_q',q0,carrier,target,panel_rotation,limits)
    return row


class RepairSchedule:
    """Wait for the physical clutch, and release early once a screw is driven."""
    def __init__(self,rows):
        self.rows=rows
        self.index=0
        self.start_rounds=None
        self.site=None
        self.extra=0
        self.finished=0
        self.key_wait=0
        self.pull_wait=0

    def next_index(self,metrics):
        i=min(self.index,len(self.rows)-1)
        row=self.rows[i]
        drive=row.get('repair_drive')
        rounds=metrics.get('remaining',12)
        if drive:
            site=(drive['front'],drive['wood'],drive['y'])
            if site!=self.site:
                self.site=site;self.start_rounds=rounds;self.extra=0;self.finished=0
        label=row['label']
        j=min(i+1,len(self.rows)-1)
        if row.get('panel_phase')=='Seat' and rounds<=8-row['pocket']:
            while j<len(self.rows)-1 and self.rows[j]['label']==label:j+=1
        if label in ('Press first fixing','Press second fixing') and metrics.get('remaining',8)<=(7 if label=='Press first fixing' else 6):
            while j<len(self.rows)-1 and self.rows[j]['label']==label:j+=1
        if label in ('Press third fixing','Press fourth fixing') and metrics.get('remaining',8)<=(5 if label=='Press third fixing' else 4):
            while j<len(self.rows)-1 and self.rows[j]['label']==label:j+=1
        if 'upper ' in label and 'jamb' in label:
            # Keep the experimentally checked hand-off to the second stock rack.
            self.index=j
            return i
        if label.startswith('Drive ') and drive:
            self.finished=self.finished+1 if rounds<self.start_rounds else 0
            if self.finished>=3:
                while j<len(self.rows)-1 and self.rows[j]['label']==label:j+=1
            elif self.rows[j]['label']!=label and self.extra<65:
                j=i;self.extra+=1
        elif drive and label.startswith('Align ') and not '/' in label:
            # A settled stand-off can advance to approach. Check only geometry
            # visible to the commissioning instrumentation and the tool spring.
            face=np.asarray(metrics.get('face',[0.,0.,0.]))
            target=np.array([g.FRONT+drive['depth'],drive['y'],drive['z']])
            if np.max(abs(face-target))<.0045 and metrics.get('axis',[0])[0]>.994:
                while j<len(self.rows)-1 and self.rows[j]['label']==label:j+=1
        about_to_unhook = 'unhook' not in row and 'unhook' in self.rows[j]
        if about_to_unhook and metrics.get('panel_rows', 0) < 2:
            j = i  # Keep the carrying hand on the panel until both backing strips carry it.
        if 'unhook' in row and self.rows[j].get('unhook')!=row['unhook']:
            if row['unhook']==0 and not metrics.get('key_released'):
                self.key_wait+=1
                j=i
            elif row['unhook']<0 and metrics.get('carrier_offset_x_mm',0)>-96:
                self.pull_wait+=1
                j=i
        self.index=j
        return i


def correct_tool(policy,entry,obs,row,poses,paths):
    from inspect_robots_yam.packing import STATE_KEY
    parts={n.rsplit('/',1)[-1]:v for n,v in zip(paths,poses)}
    h=parts['driver'];rot=R.from_quat(h[3:])
    nose=h[:3]+rot.apply(g.NOSE)
    if not hasattr(policy,'acquired_xyz'):
        ac,_=ik.tcp_pose(np.asarray(obs.state[STATE_KEY])[:6],station.BASES['left'])
        policy.rel=rot.inv().apply(ac-h[:3]);policy.drive_command=row[:6].copy()
        policy.acquired_xyz=nose.copy()
    q=policy.drive_command
    tcp,hand=ik.tcp_pose(q,station.BASES['left'])
    w=parts[entry['repair_drive']['wood']]; direction=R.from_quat(w[3:]).apply([0,1,0])
    y=entry['repair_drive']['y']; height=w[2]+(y-w[1])*direction[2]/max(.7,direction[1])
    goal=np.array([g.FRONT-.064,y,height])
    phase=entry['label']
    if y<0 and phase in ('Press fourth fixing','Register second entry','Press second fixing','Withdraw second fixing','Release fastened upper wood','Withdraw third fixing','Withdraw fourth fixing'):
        goal[2]-=.0035
    if phase=='Lift above base':
        goal[:2]=policy.acquired_xyz[:2]
        goal[2]=.926
    if phase=='Skirt stock rack':
        goal[:2]=[policy.acquired_xyz[0],.260]
        goal[2]=.926
    if phase=='Retract clear of jamb':
        goal[:2]=[g.FRONT-.025,.244]
        goal[2]=.926
    if phase=='Traverse to upper wood':goal[0]=g.FRONT-.025
    if phase=='Park for lower stock' or entry.get('park_lower'):goal[:]=[.310,.232,.953]
    if phase=='Approach lower strip':goal[0]=g.FRONT-.029
    if phase=='Pass ahead of stock hand':goal[0]=g.FRONT-.036
    if phase in ('Open stock jaws','Draw clear of stock','Clear opposite jamb'):goal[0]=g.FRONT-.032
    if phase in ('Register upper entry','Register second entry','Register lower entry'):goal[0]=g.FRONT-.002
    if phase in ('Press first fixing','Press second fixing','Press third fixing','Press fourth fixing'):goal[0]=g.FRONT+.033
    if phase in ('Withdraw first fixing','Withdraw second fixing','Release fastened upper wood','Withdraw third fixing','Withdraw fourth fixing'):goal[0]=g.FRONT-.040
    panel_phase=entry.get('panel_phase')
    if panel_phase:
        goal[0]=g.FRONT+entry['depth']
        if y<0:
            # The upper strip slopes under the first fixing. Keep the final
            # screw inside the timber and above the upper-row height gauge.
            goal[2] += .003 if entry['repair_drive']['wood']=='strip' else -.002
    travel=np.clip(.16*(goal-nose),-.0018,.0018)
    axis=rot.apply([1.,0,0]);shoe=parts['shoe'];rim=shoe[:3]+R.from_quat(shoe[3:]).apply(g.NOSE)
    stroke=(h[:3]-shoe[:3])@axis
    unloading=phase in ('Withdraw first fixing','Withdraw second fixing','Release fastened upper wood','Withdraw third fixing','Withdraw fourth fixing') and stroke>.001
    unloading=unloading or (panel_phase=='Unload' and stroke>.001)
    if unloading:travel=-.00135*axis
    if (phase in ('Press first fixing','Press second fixing','Press third fixing','Press fourth fixing') or panel_phase=='Seat') and g.CURRENT.stroke is not None:
        dyz=g.CURRENT.stroke['point'][1:]-rim[1:]
        travel[1:]=np.clip(.25*dyz,-.0015,.0015)
        travel[0]=min(travel[0],.00055 if np.linalg.norm(dyz)<.0025 else 0.)
    from scipy.optimize import lsq_linear
    actual_base,actual_mat=ik.tcp_pose(np.asarray(obs.state[STATE_KEY])[:6],station.BASES['left'])
    offset=actual_mat.T@(nose-actual_base)
    direction=actual_mat.T@axis
    lateral=actual_mat.T@rot.apply([0.,1,0])
    def point_and_axis(q):
        pt,mat=ik.tcp_pose(q,station.BASES['left'])
        return np.r_[pt+mat@offset,.10*mat@direction,.045*(mat@lateral)[2]]
    state=point_and_axis(q)
    jac=np.column_stack([(point_and_axis(q+col*1e-5)-state)*1e5 for col in np.eye(6)])
    change_axis=np.clip(.18*(np.array([1.,0,0])-axis),-.018,.018)
    desired=np.r_[travel,.10*change_axis,.045*np.clip(-.16*rot.apply([0.,1,0])[2],-.021,.021)]
    if unloading:
        desired[3:]=0.  # Axial extraction while the bit is constrained by the installed fastener.
    # At the far lower jamb keep withdrawing along the bit after the shoe spring
    # unloads. Do not introduce an abrupt sideways / roll correction at index.
    if phase=='Withdraw fourth fixing':
        clear=nose[0] < g.FRONT-.031
        desired[:3]=np.zeros(3) if clear else -.00080*axis
        desired[3:]=0.
    reg=.012
    lower=np.maximum(ik.LOWER-q,-.021); upper=np.minimum(ik.UPPER-q,.021)
    dq=lsq_linear(np.vstack([jac,reg*np.eye(6)]),np.r_[desired,np.zeros(6)],
                  bounds=(np.minimum(lower,-1e-9),np.maximum(upper,1e-9)),method='bvls').x
    policy.drive_command=np.clip(q+dq,ik.LOWER,ik.UPPER)
    row[:6]=policy.drive_command
    row[6]=.21
    policy.tool_diagnostics=dict(phase=phase,goal=goal.tolist(),nose=nose.tolist(),entry=rim.tolist(),q=policy.drive_command.tolist())
    return row

def clear_stock(policy,entry,obs,row):
    from inspect_robots_yam.packing import STATE_KEY
    label,fraction=entry['clear_stock']
    if getattr(policy,'clear_label',None)!=label:
        policy.clear_initial=np.asarray(obs.state[STATE_KEY])[7:13].copy()
        policy.clear_label=label
        policy.clear_target=policy.clear_initial.copy()
        if label=='Draw clear of stock':
            tcp,mat=ik.tcp_pose(policy.clear_initial,station.BASES['right'])
            policy.clear_target,d,a=ik.solve(tcp+[-.120,0.,0.],R.from_matrix(mat).as_quat(),
                    q_init=policy.clear_initial,base_pos=station.BASES['right'],iters=240,tol=.00008)
            if d>.002 or a>.03:raise ValueError(('stock withdrawal IK',d,a))
        elif label in ('Clear opposite jamb','Stay clear'):
            # The standard READY pose sweeps an open pad through the staged patch.
            att=R.from_euler('y',28,degrees=True)*NEUTRAL
            q,dx,da=ik.solve([.180,-.235,.994],att.as_quat(),q_init=[1.16,.63,.86,-1.16,.94,-.75],base_pos=station.BASES['right'],iters=100,tol=.0001)
            if dx>.002 or da>.025:raise ValueError(('safe rack stand-off',dx,da))
            policy.clear_target=q
    row[7:13]=policy.clear_initial+ik.smoothstep(fraction)*(policy.clear_target-policy.clear_initial)
    return row
