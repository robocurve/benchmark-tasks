"""Starting pose of the empty, top-open bottom-taped carton."""
TOP=.7475
CUTTER_START=(.13,.42,TOP+.020)
CUTTER_PAD_START=(CUTTER_START[0],CUTTER_START[1],TOP+.001)
# Empty used carton starts upright, with top flaps folded back.
BOTTOM_UP=False
CENTER=(.25,-.10,TOP+.183)
YAW_DEG=45.
def starting_pose(part,upright=False):
 if upright:return tuple((CENTER[0],CENTER[1],TOP+.0015)[i]+part['pos'][i] for i in range(3)),part['quat']
 import math
 x,y,z=part['pos'];w,qx,qy,qz=part['quat'];wf,xf,yf,zf=(-qx,w,-qz,qy);a=math.radians(YAW_DEG);c,s=math.cos(a),math.sin(a);ch,sh=math.cos(a/2),math.sin(a/2)
 return (CENTER[0]+c*x+s*y,CENTER[1]+s*x-c*y,CENTER[2]-z),(ch*wf-sh*zf,ch*xf-sh*yf,ch*yf+sh*xf,ch*zf+sh*wf)
