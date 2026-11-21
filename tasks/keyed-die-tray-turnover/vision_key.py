"""Decide whether a quarter turn is needed from the marked corner in a stock wrist RGB frame."""
import numpy as np
from scipy.spatial.transform import Rotation
from layout import STAGE,KEY_CENTER

def key_pixels(eye,roll,key_center=KEY_CENTER):
    back=eye-STAGE;back=back/np.linalg.norm(back)
    across=np.cross([0.,0.,1.],back);across/=np.linalg.norm(across)
    cam=np.column_stack([across,np.cross(back,across),back])@Rotation.from_euler('z',roll).as_matrix()
    points=np.array([[1,1,0],[-1,1,0],[-1,-1,0],[1,-1,0]])*key_center+STAGE
    # Optical coordinates follow the original cropped YAM camera intrinsics.
    xyz=(points-eye)@cam@np.diag([1,-1,-1])
    return (xyz[:,:2]/xyz[:,2:3]*[390.666,390.162]+[317.526+.5,176.146+.5])*[.5,176/360]-.5

def classify(rgb,eye,roll,key_center=KEY_CENTER):
    r,g,b=rgb.astype(int).transpose(2,0,1)
    magenta=(r>g+42)&(b>g+28)&(r>100)&(b>64)
    centers=key_pixels(eye,roll,key_center); yy,xx=np.ogrid[:rgb.shape[0],:rgb.shape[1]]
    counts=[int(np.count_nonzero(magenta&((xx-x)**2+(yy-y)**2<9**2))) for x,y in centers]
    order=np.argsort(counts)[::-1]
    heading=int(order[0]) if counts[order[0]]>=3 and counts[order[0]]-counts[order[1]]>=3 else None
    return heading,counts
