import numpy as np,math
from yam_arena import station,ik
from yam_arena.demo import grip_rotation
TABLE=station.TABLE_TOP;BASES=station.BASES;BOARD=.75;WALL_CENTRE=.75-.0085-.02
Y=.25;GRIP_ROT=grip_rotation(-math.pi/2)
PADDLE_R=np.array([[1.,0.,0.],[0.,0.,1.],[0.,-1.,0.]])
HANDLE=np.array([.32,.12,.91]);Q0,PE,RE=ik.solve(HANDLE,ik.matrix_to_quat_xyzw(GRIP_ROT),base_pos=BASES['left'],iters=500)
if PE>.002 or RE>.02:raise RuntimeError(f'Initial IK {PE}, {RE}')
def ry(a):
 c,s=math.cos(a),math.sin(a);return np.array([[c,0,s],[0,1,0],[-s,0,c]])
