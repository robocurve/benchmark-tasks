import numpy as np
from contact_model import brushing_patch
x,y=np.meshgrid(np.linspace(-.007,.007,100),np.linspace(-.006,.006,90));R=np.eye(3);face=np.array([.001,0,-.00025]);prev=np.array([0.,0,-.00025])
def dose(f=face,p=prev,force=.1,rot=R,pr=R):return brushing_patch(x,y,f,rot,p,pr,force)
a,s,d=dose();assert a.any() and (d[a]>0).all();assert np.allclose(s[a],.001)
for name,kwargs in [('air_gap',{'f':np.array([.001,0,-.001])}),('frame_only_force',{'force':0.}),('outside_glass',{'f':np.array([.05,0,-.00025])}),('stationary',{'p':face.copy()}),('normal_press_only',{'p':np.array([.001,0,-.0005])}),('wrong_orientation',{'rot':np.diag([1.,-1.,-1.])})]:
 a,s,d=dose(**kwargs);assert not a.any() and np.all(d==0),name
print('PASS: sliding on glass; reject hovering, frame-only force, off-glass motion, stationary pressure, normal-only motion and wrong orientation.')
