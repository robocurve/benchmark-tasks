"""Planning approximation for rigid thin-shell ball impulses; no object control."""
import math,numpy as np
G=9.81;R=.02;K=2/3;E=.9;MU=.1

def impact(v,w,n,speed=0.):
 v=np.array(v,dtype=float);n=np.array(n);t=np.array([-n[1],n[0]]);rel=float(v@n)-speed
 if rel>=0:return v,w
 j=-(1+E)*rel;vt=float(v@t)+R*w;jt=float(np.clip(-vt*K/(K+1),-MU*j,MU*j));return v+j*n+jt*t,w+jt/(K*R)

def intercept(p,v,w,phase,hitx,table,wall=None):
 x,z=p;vx,vz=v;floor=table+R
 if phase=='wall':
  if wall is None or vx<=.1 or x>=wall:return None
  tw=(wall-x)/vx;zw=z+vz*tw-.5*G*tw*tw
  vw,ww=impact([vx,vz-G*tw],w,[-1.,0.])
  pred=intercept([wall,zw],vw,ww,'table',hitx,table)
  if pred is None:return None
  dt,h,vi,wi=pred;return tw+dt,h,vi,wi
 if vx>=-.1 or x<=hitx:return None
 if phase=='table':
  tb=(vz+math.sqrt(max(0.,vz*vz+2*G*(z-floor))))/G;xf=x+vx*tb
  if xf<=hitx:return None
  vv,ww=impact([vx,vz-G*tb],w,[0.,1.]);dt=(hitx-xf)/vv[0]
  if dt<=0:return None
  return tb+dt,floor+vv[1]*dt-.5*G*dt*dt,np.array([vv[0],vv[1]-G*dt]),ww
 dt=(hitx-x)/vx;return dt,z+vz*dt-.5*G*dt*dt,np.array([vx,vz-G*dt]),w

def choose_stroke(z,vin,w,hitx,wallx,table):
 best=None
 for theta in np.linspace(-.30,.60,46):
  n=np.array([math.cos(theta),math.sin(theta)])
  for speed in np.linspace(.1,1.0,37):
   v,o=impact(vin,w,n,speed)
   if v[0]<.5:continue
   tw=(wallx-hitx)/v[0];zw=z+v[1]*tw-.5*G*tw*tw
   if not table+R+.015<zw<table+.30-R-.005:continue
   vw,ow=impact([v[0],v[1]-G*tw],o,[-1.,0.]);pred=intercept([wallx,zw],vw,ow,'table',hitx,table)
   if pred is None:continue
   eta,hn,vn,on=pred
   if not table+.08<hn<table+.24:continue
   cost=abs(hn-(table+.16))*20+abs(v[0]-1.8)*.08+max(0.,.3-eta)*.3+max(0.,speed-.8)*.2
   if best is None or cost<best[0]:best=(cost,theta,speed,v,hn,eta)
 return best
