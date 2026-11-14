"""Keep headed half supported, rotate upper half and lift only after alignment.

Static targets are a bounded diagnostic, not a controller claiming actual angle.
"""
import numpy as np
import geometry as g
from yam_arena.urdf import axis_angle_matrix

def grasp_target(i,p,angle):
    sign=1 if i else -1
    point=np.r_[g.handle_center(i)+[0,sign*.018],.006]
    normal=np.array([0.,float(sign),0.])
    approach=axis_angle_matrix(normal,np.radians(-40.*sign))@np.array([0.,0.,-1.])
    r=g.rz(angle)
    n=r@normal;z=r@approach
    if n[1]>0:n=-n  # same parallel pinch, reachable wrist branch
    hand=np.column_stack((n,np.cross(z,n),z))
    tcp=np.asarray(p)+r@point-z*.005+[0,0,.00489]
    return tcp,hand

def object_plan(top=.7475):
    p=np.array([*g.PIVOT_XY,top])
    return [
      dict(duration=1.5,positions=np.array([p,p]),angles=np.radians([30,-30]),label='grip side walls with both halves supported'),
      dict(duration=3.,positions=np.array([p,p]),angles=np.radians([30,-60]),label='turn upper half on table to nominal release angle'),
      dict(duration=1.5,positions=np.array([p,p]),angles=np.radians([30,-60]),label='hold supported alignment for inspection before lifting'),
      dict(duration=2.,positions=np.array([p,p+[0,0,.020]]),angles=np.radians([30,-60]),label='test slow axial separation while bottom half stays supported'),
      dict(duration=2.,positions=np.array([p,p+[0,0,.020]]),angles=np.radians([30,-60]),label='hold for diagnostic only')]
