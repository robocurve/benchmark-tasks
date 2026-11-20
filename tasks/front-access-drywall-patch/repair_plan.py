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
    p=Plan()
    p.move('Settle loose parts',10)
    if mode=='repair-lower':
        p.stock(g.SECOND,g.MID-.044,'lower backer')
        p.move('Inspect held lower strip',25)
        return p
    if mode in ('repair-test','repair-panel'):
        place(p)
        p.move('Observe carrier',35)
        for row in p.rows[-35:]:row['repair_pose']=g.FRONT+g.BOARD/2
        return p
    if mode not in ('repair-tool','repair-brake','repair-finish','repair-sites','repair-far','repair-lower-pair'):p.stock(g.STOCK,g.MID+.066,'upper backer')
    h=g.HANDLE
    p.tool('Approach driver stand',46,h+[0,0,.086],.72)
    p.tool('Lower to grip',33,h+[0,0,.004],.72)
    p.tool('Grip driver',16,opening=0.)
    p.tool('Lift out of stand',34,h+[0,0,.070])
    if mode=='repair-lower-pair':
        p.stock(g.SECOND,g.MID-.044,'lower backer')
        start=len(p.rows)
        p.drive('lower positive jamb','wall','spare_strip',.114,g.MID-.044)
        for row in p.rows[start:]:row['stock_target']=[.371,0,g.MID-.044,0]
        p.release(g.MID-.044)
        p.drive('lower negative jamb','wall','spare_strip',-.108,g.MID-.044)
        p.move('Inspect fastened lower backer',30)
        return p
    if mode=='repair-far':
        p.drive('upper positive jamb','wall','strip',.114,g.MID+.066)
        p.drive('lower positive jamb','wall','spare_strip',.114,g.MID-.044)
        p.drive('lower negative jamb','wall','spare_strip',-.108,g.MID-.044)
        p.drive('patch spare_strip -0.052','patch','spare_strip',-.078,g.MID-.044)
        p.move('Inspect remote fastening',18)
        return p
    if mode=='repair-sites':
        for w,z in [('spare_strip',g.MID-.044),('strip',g.MID+.066)]:
            for y in (.078,-.078):p.drive('patch '+w+' '+str(y),'patch',w,y,z)
        p.move('Inspect four fixture sites',18)
        return p
    if mode!='repair-finish':
        start=len(p.rows)
        p.drive('upper positive jamb','wall','strip',.114,g.MID+.066)
        if mode not in ('repair-tool','repair-brake'):
            for row in p.rows[start:]:
                row['stock_target']=[.3709,0,g.MID+.066,0];row['wood_name']='strip'
        if mode=='repair-brake':
            p.move('Inspect braked stroke',28)
            return p
        if mode=='repair-tool':
            p.drive('upper negative jamb','wall','strip',-.114,g.MID+.066)
            p.move('Inspect trial',25)
            return p
        if mode=='repair-first':
            p.move('Inspect first fastening',30)
            return p
        p.release(g.MID+.066)
        p.drive('upper negative jamb','wall','strip',-.114,g.MID+.066)
        park(p,'Park beyond opening',70)
        start_stock=len(p.rows)
        p.stock(g.SECOND,g.MID-.044,'lower backer')
        for row in p.rows[start_stock:]:row['hold_tool']=True
        starts=len(p.rows)
        p.drive('lower positive jamb','wall','spare_strip',.114,g.MID-.044)
        for row in p.rows[starts:]:row['stock_target']=[.371,0,g.MID-.044,0]
        p.release(g.MID-.044)
        p.drive('lower negative jamb','wall','spare_strip',-.108,g.MID-.044)
        park(p,'Clear lifting path',70)
        if mode=='repair-stock':return p
    start_panel=len(p.rows)
    target=place(p)
    first_patch_step=len(p.rows)
    if mode!='repair-finish':
        for row in p.rows[start_panel:]:row['hold_tool']=True
    # Start two vertically separated screws, one into each piece of backing.
    # The fasteners can now carry the patch while the right hand removes the
    # lifter; the screwdriver never has to traverse through the lifter.
    for n,z in [('strip',g.MID+.066),('spare_strip',g.MID-.044)]:
        p.drive('patch '+n+' 0.078','patch',n,.078,z)
    for row in p.rows[first_patch_step:]:row['repair_pose']=g.FRONT+g.BOARD/2
    start_remove=len(p.rows)
    for name,n,offset in [('Unhook carrying pins',26,0.),('Pull handle from face',30,-.075)]:
        p.move(name,n,target+[offset,0,-.0135])
        for row in p.rows[-n:]:row['unhook']=offset
    p.move('Withdraw handle',39,[.245,-.284,.961])
    for row in p.rows[start_remove:]:row['hold_tool']=True
    for n,z in [('spare_strip',g.MID-.044),('strip',g.MID+.066)]:
        p.drive('patch '+n+' -0.052','patch',n,-.052,z)
    park(p,'Park driver beside completed wall',67)
    p.move('Inspect released repair',65)
    for row in p.rows[-65:]:row['hold_tool']=True
    if mode!='repair-finish':pack_schedule(p)
    return p


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
    f=entry['repair_drive']
    parts={n.rsplit('/',1)[-1]:v for n,v in zip(paths,poses)}
    measured=np.asarray(obs.state[STATE_KEY])[:6]
    key=f.get('site_id',(f['wood'],f['y'],f['front']))
    cutting=f['depth']>0 and not f.get('transit')
    housing=parts['driver']
    shell=R.from_quat(housing[3:])
    tcp,mat=ik.tcp_pose(measured,base_pos=station.BASES['left'])
    hand=R.from_matrix(mat)
    axis=shell.apply([1.,0,0])
    tip=housing[:3]+shell.apply([g.TIP,0,0])
    if getattr(policy,'repair_key',None)!=key:
        policy.repair_key=key
        policy.hand_to_shell=hand.inv()*shell
        policy.grip_offset=hand.inv().apply(housing[:3]-tcp)
        policy.site_roll=shell.as_euler('xyz')[0]
        policy.start_y=tip[1]
        policy.travel_y=tip[1]
        policy.travel_z=tip[2]
        policy.drive_command=measured.copy()
        policy.travel_x=tip[0]
        policy.cut_site=None
        policy.free_roll=0.
        policy.feed_bias=np.zeros(3)
        policy.tilt_bias=np.zeros(3)
    # Track slip of the carried tool. Measurements never write an object pose.
    rel=hand.inv()*shell
    policy.hand_to_shell=policy.hand_to_shell*R.from_rotvec(.15*(policy.hand_to_shell.inv()*rel).as_rotvec())
    policy.grip_offset=.85*policy.grip_offset+.15*hand.inv().apply(housing[:3]-tcp)
    wood=parts[f['wood']]
    along=R.from_quat(wood[3:]).apply([0.,1.,0.])
    z=float(np.clip(wood[2]+(f['y']-wood[1])*along[2]/max(.7,along[1]),f['z']-.013,f['z']+.013))
    if f.get('transit'):
        z=f['z']
    shoe=parts['shoe'];nose=shoe[:3]+R.from_quat(shoe[3:]).apply([g.TIP,0,0])
    compression=float((housing[:3]-shoe[:3])@axis)
    plane_x=g.FRONT
    if f['front']=='patch':
        patch=parts['patch']
        normal=R.from_quat(patch[3:]).apply([1.,0.,0.])
        centre=patch[:3]-g.BOARD/2*normal
        if normal[0]>.9:
            plane_x=centre[0]-np.dot(normal[1:],[f['y']-centre[1],z-centre[2]])/normal[0]
    registered=abs(nose[1]-f['y'])<.0045 and abs(nose[2]-z)<.0045 and axis[0]>.992
    if cutting and registered and compression>.001 and abs(nose[0]-plane_x)<.0015 and policy.cut_site is None:
        policy.cut_site=nose[1:].copy()
    if cutting and f['front']=='patch' and policy.cut_site is None and axis[0]>.985:
        # The unmarked facing accepts any sound fixing in this part of the row.
        # Project the real bit forward and acquire a reachable, fully backed
        # point; no scorer thresholds or simulator objects are changed.
        entry=nose+(plane_x-nose[0])*axis/axis[0]
        if (abs(entry[1]-f['y'])<.012 and abs(entry[2]-z)<.008 and
                plane_x-.030<nose[0]<plane_x+.002 and g.bite(wood,entry,axis)):
            policy.cut_site=entry[1:].copy()
    s=g.CURRENT.stroke
    if cutting and s and s['depth']>.0009 and ((s['front']=='patch')==(f['front']=='patch')):
        actual=parts['patch'][:3]+R.from_quat(parts['patch'][3:]).apply(s['local']) if s['front']=='patch' else s['point']
        if np.max(abs(actual[1:]-[f['y'],z]))<.008:
            policy.cut_site=actual[1:].copy()
    sy,sz=policy.cut_site if cutting and policy.cut_site is not None else (f['y'],z)
    goal_x=plane_x+f['depth']+(.0009 if cutting else 0.)
    speed=.009 if f.get('transit') else .004
    if not f.get('transit') and policy.cut_site is None and not registered:
        # Register the bit at the intended board point before applying pressure.
        goal_x=min(goal_x,plane_x-.013)
    if f.get('transit') and tip[0]>plane_x-.012 and abs(tip[1]-sy)>.035:
        sy=tip[1]
    policy.travel_y+=np.clip(sy-policy.travel_y,-speed,speed)
    # Clear the empty work rests while moving the tool across the opening.
    if f.get('transit') and min(policy.start_y,f['y'])<.015 and max(policy.start_y,f['y'])>.04 and sz<.92:
        fraction=np.clip((policy.travel_y-policy.start_y)/(f['y']-policy.start_y),0.,1.)
        sz+=.079*np.sin(np.pi*fraction)
    policy.travel_z+=np.clip(sz-policy.travel_z,-speed,speed)
    wanted_roll=np.deg2rad(-75. if f['z']<g.MID-.025 else -64. if f['front']=='patch' and f['y']>0 else np.clip(290*(policy.travel_y-.02),-44.,0.))
    policy.site_roll+=np.clip((wanted_roll-policy.site_roll+np.pi)%(2*np.pi)-np.pi,-.044,.044)
    if cutting:
        # Approach then apply feed at a rate a 430-rpm, 8-TPI screw can accept.
        can_advance=(abs(nose[1]-sy)<.005 and abs(nose[2]-sz)<.005 and axis[0]>.985)
        if f['front']=='patch' and policy.cut_site is not None:
            can_advance=(abs(nose[1]-sy)<.007 and abs(nose[2]-sz)<.007 and axis[0]>.981)
        advance=.00103 if can_advance else 0.
    else:advance=.0055
    policy.travel_x+=np.clip(goal_x-policy.travel_x,-.0065,advance)
    free_error=np.array([policy.travel_x,policy.travel_y,policy.travel_z])-tip
    # Lift compensation differs by wrist posture. The lower opposite corner can
    # sag more than 35 mm under the tool even with millimetric IK residual.
    bias_limits=np.array([.035,.035,.075 if f['z']<g.MID-.025 and f['y']<0 else .035])
    # Freeze forward windup at the travel stop while retaining the effort
    # already required to hold the tool against the wall.
    if cutting and compression>=.033:
        free_error[0]=min(free_error[0],0.)
    policy.feed_bias=np.clip(policy.feed_bias+np.clip(.12*free_error,-.0014,.0014),-bias_limits,bias_limits)
    policy.tilt_bias=np.clip(policy.tilt_bias+.09*np.cross(axis,[1.,0,0]),-.14,.14)
    desired_shell=R.from_rotvec(policy.tilt_bias)*R.from_euler('x',policy.site_roll+policy.free_roll)
    desired_hand=desired_shell*policy.hand_to_shell.inv()
    goal=policy.feed_bias+np.array([policy.travel_x,policy.travel_y,policy.travel_z])-desired_shell.apply([g.TIP,0,0])-desired_hand.apply(policy.grip_offset)
    # Solve the full no-load pose. Use the measured compliance correction only
    # after grasping; gradual joint commands keep the attachment seated.
    q=policy.drive_command
    candidate,d,a=ik.solve(goal,desired_hand.as_quat(),q_init=q,base_pos=station.BASES['left'],iters=32,tol=.00009)
    if d>.0015 or a>.025:
        # The PH2 bit can approach axially at any casing roll. Search the
        # remaining degree of freedom if the nominal wrist hits a joint limit.
        reference=policy.free_roll
        for delta in (.16,-.16,.34,-.34,.57,-.57,.84,-.84):
            trial=float(np.clip(reference+delta,-1.45,1.45))
            shell_q=R.from_rotvec(policy.tilt_bias)*R.from_euler('x',policy.site_roll+trial)
            hand_q=shell_q*policy.hand_to_shell.inv()
            new_goal=policy.feed_bias+np.array([policy.travel_x,policy.travel_y,policy.travel_z])-shell_q.apply([g.TIP,0,0])-hand_q.apply(policy.grip_offset)
            sol,dist,ang=ik.solve(new_goal,hand_q.as_quat(),q_init=q,base_pos=station.BASES['left'],iters=16,tol=.00015)
            if dist+.15*ang<d+.15*a:
                candidate,d,a,goal=sol,dist,ang,new_goal
                policy.free_roll=trial
            if d<.0015 and a<.025:break
    if (d>=.0028 or a>=.038) and (f['front']=='patch' or f['y']<0 and f['z']<g.MID):
        # At the far lower corner the bit must reach the screw and stay axial.
        # A round PH2 shank has no preferred handle roll. Use the local tool
        # Jacobian to relax that redundant constraint without moving the tool
        # or changing the wrist, joint, contact or scoring limits.
        offset=hand.inv().apply(tip-tcp)
        local_axis=hand.inv().apply(axis)
        def tool_state(joints):
            centre,matrix=ik.tcp_pose(joints,base_pos=station.BASES['left'])
            return np.r_[centre+matrix@offset,.17*(matrix@local_axis)]
        origin=tool_state(q)
        jac=np.column_stack([(tool_state(q+direction*1.e-5)-origin)/1.e-5 for direction in np.eye(6)])
        travel=np.array([policy.travel_x,policy.travel_y,policy.travel_z])
        inward=(np.array([.9976,-.0698,0.]) if f['y']<0 and f['z']<g.MID else np.array([1.,0.,0.]))
        correction=np.r_[np.clip(.25*(travel-tip),-.0035,.0035),
                         .17*np.clip(.20*(inward-axis),-.026,.026)]
        dq=jac.T@np.linalg.solve(jac@jac.T+.024**2*np.eye(6),correction)
        step=np.clip(dq,-.035,.035)
        policy.drive_command=np.clip(q+step,np.maximum(ik.LOWER,measured-.24),np.minimum(ik.UPPER,measured+.24))
    elif d<.0028 and a<.038:
        change=candidate-q
        policy.drive_command=q+change*min(1.,.056/max(1.e-9,np.max(abs(change))))
    policy.tool_diagnostics=dict(target_x=policy.travel_x,target_z=z,goal=goal.tolist(),ik_mm=d*1000,ik_deg=np.rad2deg(a),servo_error=(policy.drive_command-measured).round(3).tolist(),axis=axis.tolist(),tip=tip.tolist())
    row[:6]=policy.drive_command
    return row


if __name__=='__main__':
    res=make(sys.argv[1]);Path(sys.argv[2]).write_text(json.dumps(dict(steps=res.rows,ik=res.checks)))
