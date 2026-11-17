from pathlib import Path
import numpy as np
from planner import Plan
from common import down
from model import CENTER,SCREW,Model,NA,NR
from yam_arena import ik,station

def rx(a):
 c,s=np.cos(a),np.sin(a);return np.array([[c,0,s],[0,1,0],[-s,0,c]])

if __name__=='__main__':
 p=Plan();L=down(-45);R=down(45);m=Model();last=0
 def add(*args,**kwargs):
  global last
  p.add(*args,**kwargs)
  for f in p.frames[last:]:
   action=np.asarray(f['action14'])
   hands={side:dict(tcp=ik.tcp_pose(action[o:o+6],station.BASES[side])[0],rotation=ik.tcp_pose(action[o:o+6],station.BASES[side])[1],opening=action[o+6]) for side,o in [('left',0),('right',7)]}
   m.update(hands)
  last=len(p.frames)
 add(3,'allow loose fabric to settle',{})
 # Grasp outer cloth, pull opposing edges, and release. The loosely seated
 # hoop lets material slip through; no pose or anchor is changed by the demo.
 for number,angle in enumerate([np.pi/2,np.pi/4,3*np.pi/4]):
  j=round(angle/(2*np.pi)*NA)%NA
  a=m.soft.p[1+(NR-1)*NA+j].copy()+[0,0,.008]
  b=m.soft.p[1+(NR-1)*NA+(j+NA//2)%NA].copy()+[0,0,.008]
  add(2.5,'approach opposite fabric edges',{'left':(a+[0,0,.07],L,.7),'right':(b+[0,0,.07],R,.7)},'joint')
  add(1.5,'lower to loose cloth',{'left':(a,L,.7),'right':(b,R,.7)})
  add(.8,'pinch fabric edges',{'left':(a,L,.03),'right':(b,R,.03)})
  direction=np.array([np.cos(angle),np.sin(angle),0.])
  aa=a+direction*.008+[0,0,0];bb=b-direction*.008+[0,0,0]
  add(2,'draw slack through hoop',{'left':(aa,L,.03),'right':(bb,R,.03)})
  add(.7,'release fabric edges',{'left':(aa,L,.7),'right':(bb,R,.7)})
  add(1,'clear fabric',{'left':(aa+[0,0,.07],L,.7),'right':(bb+[0,0,.07],R,.7)})
 add(2,'clear left arm',{'left':(CENTER+[0,.22,.12],L,.8)},'joint')
 S=np.array([[0,1,0],[0,0,1],[1,0,0.]])
 tcp=SCREW-S[:,2]*.006
 add(2,'approach thumb screw',{'right':(tcp+[0,-.03,.04],S,.65)},'joint')
 add(1.5,'seat fingers around screw',{'right':(tcp,S,.65)})
 add(.8,'pinch screw',{'right':(tcp,S,.05)})
 add(2,'tighten quarter turn',{'right':(tcp,rx(np.pi/2)@S,.05)})
 add(.8,'release screw',{'right':(tcp,rx(np.pi/2)@S,.7)})
 add(2,'withdraw from work',{'right':(tcp+[0,-.10,.08],rx(np.pi/2)@S,.8)})
 add(4,'released tension check',{})
 print(p.save(Path(__file__).with_name('demo-plan.json')))
 print('Local material prediction:',m.score(),m.measure())
