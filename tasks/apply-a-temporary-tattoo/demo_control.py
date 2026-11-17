"""Regrip the wet blank margin using ordinary robot joints and native poses."""
import numpy as np
from scipy.optimize import least_squares

class DemoControl:
 def __init__(self):self.start=None
 def action(self,k,wanted,m,hands,observed):
  if k<=1200:return wanted
  from yam_arena import ik,station
  h=hands['left']
  if self.start is None:
   self.start=h['tcp'].copy();self.R=h['rotation'].copy()
   self.edge=m.p('paper_band_11').copy();self.grasp=self.edge-.008*self.R[:,2]
  def mix(a,b,start,end):
   u=np.clip((k-start)/(end-start),0,1);u=u*u*(3-2*u);return a*(1-u)+b*u
  if k<=1220:p=self.start;g=.25
  elif k<=1260:p=mix(self.start,self.grasp,1220,1260);g=.25
  elif k<=1280:p=self.grasp;g=0.
  elif k<=1360:p=mix(self.grasp,self.grasp+[0,.075,.075],1280,1360);g=0.
  elif k<=1420:p=mix(self.grasp+[0,.075,.075],self.grasp+[0,.100,.030],1360,1420);g=0.
  else:p=self.grasp+[0,.100,.030]+[0,0,.060*np.clip((k-1440)/40,0,1)];g=.7
  seed=observed[:6]
  def residual(q):
   pp,rr=ik.tcp_pose(q,station.BASES['left']);return np.r_[pp-p,.15*ik._rot_error(self.R,rr),.001*(q-seed)]
  sol=least_squares(residual,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=30)
  out=wanted.copy();out[:6]=sol.x;out[6]=g;return out
