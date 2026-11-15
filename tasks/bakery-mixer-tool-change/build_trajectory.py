"""Author robot TCP waypoints; no object placement or physics substitution."""
import json
import math
from pathlib import Path
import numpy as np
from yam_arena import ik, station
from yam_arena.constants import READY, DOWN_XYZW
from yam_arena.urdf import axis_angle_matrix,quat_xyzw_to_matrix

P=Path(__file__).parent
FRONT=ik.matrix_to_quat_xyzw(np.array([[0,0,1],[-1,0,0],[0,-1,0]])).tolist()


def main():
    cfg=json.loads((P/'pilot.json').read_text())
    y=cfg['poses']['mixer'][1]
    stages=[]
    def stage(phase,seconds,side=None,position=None,grip=1.,rotation=FRONT):
        item=dict(phase=phase,seconds=seconds)
        if side:item[side]=dict(position=position,rotation_xyzw=list(rotation),grip=grip)
        stages.append(item)
    stage('park right hand',3,'right',[.30,-.48,1.04])
    stage('approach beater',3,'left',[.24,.32,1.08])
    stage('lower beside beater',3,'left',[.24,.32,.947])
    stage('enter beater neck',2,'left',[.24,.32,.947],.4)
    stage('grip beater',1,'left',[.24,.32,.947],0.)
    for phase,seconds in [('lift beater',3),('carry to bowl front',5),('lower into bowl',4),
                          ('align below shaft',3),('insert hub',6),('settle inserted hub',2),('turn bayonet',30),
                          ('settle retaining pocket',2),('release beater',2),('withdraw from beater',3),
                          ('observe retention',2)]:stage(phase,seconds)
    mount=stages
    stages=[]
    stage('approach inhibited START',3,'right',[.185,y-.062,1.1625],0.)
    stage('press inhibited START',2,'right',[.225,y-.062,1.1625],0.)
    stage('release inhibited START',2,'right',[.185,y-.062,1.1625],0.)
    z=np.array([.5,math.sqrt(3)/2,0.]);x=np.array([0.,0.,1.])
    handle_rotation=ik.matrix_to_quat_xyzw(np.column_stack([x,np.cross(z,x),z]))
    handle_grip=np.array([.355,y-.140,.9275])-.013*z
    stage('clear head beside handle',3,'right',[.24,y-.28,1.05],1.,handle_rotation)
    stage('approach lift handle',3,'right',(handle_grip-.06*z).tolist(),1.,handle_rotation)
    stage('enter lift handle from outboard',3,'right',handle_grip.tolist(),.6,handle_rotation)
    stage('grip lift handle',2,'right',handle_grip.tolist(),.12,handle_rotation)
    stage('raise mixer bowl',6)
    stage('settle raised bowl',3)
    stage('release lift handle',2)
    stage('withdraw from lift handle',3)
    stage('observe bowl lift',2)
    lift=stages
    stages=[]
    x=np.array([0.,-1.,0.]);z=np.array([math.sqrt(.5),0.,-math.sqrt(.5)])
    guard_r=np.column_stack([x,np.cross(z,x),z])
    guard_rotation=ik.matrix_to_quat_xyzw(guard_r)
    guard_grip=np.array([.456,.441,.8075])-.009*guard_r[:,2]
    stage('approach guard tab',4,'left',(guard_grip+[0,0,.03]).tolist(),.4,guard_rotation)
    stage('lower onto guard tab',3,'left',guard_grip.tolist(),.4,guard_rotation)
    stage('grip guard tab',2,'left',guard_grip.tolist(),.08,guard_rotation)
    for phase,seconds in [('lift guard clear of rack',4),('carry guard to front left',4),
                          ('align guard in front of bowl',4),('slide guard around tool',4),
                          ('center guard above bowl',2),('seat guard on rim',3),('release guard',2),('withdraw from guard',3),('observe guard',2)]:stage(phase,seconds)
    guard=stages
    stages=[]
    stage('clear mixer before guard',2,'left',[.23,.30,1.04])
    clear=stages
    stages=[]
    stage('clear head before START',3,'right',[.185,y-.20,1.1625])
    stage('approach START',3,'right',[.185,y-.062,1.1625],0.)
    stage('press START',2,'right',[.225,y-.062,1.1625],0.)
    stage('release START',2,'right',[.185,y-.062,1.1625],0.)
    stage('withdraw after START',3,'right',[.185,-.30,1.1625],1.)
    stage('observe powered operation',10)
    stage('approach STOP',3,'left',[.185,y+.062,1.1625],0.)
    stage('press STOP',2,'left',[.225,y+.062,1.1625],0.)
    stage('release STOP',2,'left',[.185,y+.062,1.1625],0.)
    stage('withdraw after STOP',3,'left',[.185,.30,1.1625],1.)
    stage('observe safe endpoint',3)
    inhibited=stages[:5]+[dict(phase='observe inhibited motor',seconds=3)]
    ready_p,ready_r=ik.tcp_pose(np.asarray(READY),station.BASES['right'])
    prepare_lift=[dict(phase='prepare right hand for lift',seconds=3,right=dict(
        position=ready_p.tolist(),rotation_xyzw=ik.matrix_to_quat_xyzw(ready_r).tolist(),grip=1.))]
    plan={'hold':[dict(phase='native reset hold',seconds=5)],'mount':mount,'lift_interlock':lift,'lift_only':lift[3:],
          'guard':guard,'guard_pick':guard[:4]+[dict(phase='hold guard grip',seconds=2)],'mount_guard':mount+clear+guard,'full':mount+clear+guard+prepare_lift+lift[3:]+stages,
          'raised_without_guard':lift[3:]+inhibited,
          'guarded_without_lift':mount+clear+guard+inhibited}
    (P/'trajectory.json').write_text(json.dumps(plan,indent=2)+'\n')
    print({key:sum(x['seconds'] for x in value) for key,value in plan.items()})


if __name__=='__main__':main()
