"""Starting pose of the empty, top-open bottom-taped carton."""
TOP=.7475
CUTTER_START=(.13,.21,TOP+.020)
# Empty used carton is presented bottom-up, with top flaps folded back.
BOTTOM_UP=True
CENTER=(.27,0.,TOP+.103)
YAW_DEG=45.
def starting_pose(part,upright=False):
 if upright:return tuple((CENTER[0],0.,TOP+.006)[i]+part['pos'][i] for i in range(3)),part['quat']
 import math
 x,y,z=part['pos'];w,qx,qy,qz=part['quat'];wf,xf,yf,zf=(-qx,w,-qz,qy);a=math.radians(YAW_DEG);c,s=math.cos(a),math.sin(a);ch,sh=math.cos(a/2),math.sin(a/2)
 return (CENTER[0]+c*x+s*y,CENTER[1]+s*x-c*y,CENTER[2]-z),(ch*wf-sh*zf,ch*xf-sh*yf,ch*yf+sh*xf,ch*zf+sh*wf)
