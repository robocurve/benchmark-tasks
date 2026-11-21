"""Synthetic contact filtering and dwell regressions, not physics evidence."""
import numpy as np
import tag_closure as c

xyz=np.zeros((10,3)); xyz[9,2]=-.001
R=np.tile(np.eye(3),(10,1,1)); dt=.05
normal=np.array([0.,0.,1.])
contact=(np.array([0.,0.,-.0005]),normal,.075*normal)
assert np.isclose(c.patch_press_force(xyz,R,[contact],dt),1.5)
assert c.patch_press_force(xyz,R,[],dt)==0.
for p in ([.025,0,-.0005],[0,.03,-.0005],[0,0,.01]):
 assert c.patch_press_force(xyz,R,[(np.array(p),normal,.075*normal)],dt)==0.
assert c.patch_press_force(xyz,R,[(contact[0],np.array([1.,0,0]),np.array([.075,0,0]))],dt)==0.
for force in (0.,.1962,.99):
 dwell=0.
 for _ in range(40):dwell=c.press_dwell(dwell,True,0.,force,dt)
 assert dwell==0.
dwell=0.
for _ in range(8):dwell=c.press_dwell(dwell,True,0.,1.5,dt)
assert dwell+1e-9>=c.PRESS_SECONDS
assert c.press_dwell(dwell,False,0.,1.5,dt)==0.
assert c.press_dwell(dwell,True,.1,1.5,dt)==0.
assert c.press_dwell(dwell,True,0.,0.,dt)==0.
c.reset()
try:c.create_bond(None,xyz,R)
except RuntimeError:pass
else:raise AssertionError('No contact must not create bond')
print('Press gate CPU checks passed: no-force, weak-force, wrong-patch, tangential, dwell reset, guarded bond')

for bad in (float("nan"), float("inf")):
 assert c.press_dwell(0.,True,0.,bad,.05)==0.
 assert c.patch_press_force(xyz,R,[contact],bad)==0.
assert c.patch_press_force(xyz,R,[contact],0.)==0.
