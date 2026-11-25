"""Local glass-contact brushing model, using a pane-only normal-force input."""
import numpy as np
PANE_THICKNESS_M=.0003
# The explicit brush and glass PhysX contact offsets are 0.2 and 0.1 mm.
CONTACT_MARGIN_M=.0003
BRISTLE_HALF_WIDTH_M=.0035
BRISTLE_HALF_HEIGHT_M=.0125

def brushing_patch(x,y,face,rotation,previous_face,previous_rotation,normal_force):
    """All transforms are in the measured pane-body frame; return local cleaning and slip."""
    zeros=np.zeros_like(x,dtype=float);mask=np.zeros_like(x,dtype=bool)
    if previous_face is None or normal_force<=.02 or rotation[2,2]<=.95:
        return mask,zeros,zeros
    dx=x-face[0];dy=y-face[1];dz=-PANE_THICKNESS_M/2-face[2]
    bx=dx*rotation[0,0]+dy*rotation[1,0]+dz*rotation[2,0]
    by=dx*rotation[0,1]+dy*rotation[1,1]+dz*rotation[2,1]
    bz=dx*rotation[0,2]+dy*rotation[1,2]+dz*rotation[2,2]
    # Actual overlap with glass texels and the bristle face, within the configured collision margin.
    mask=(abs(x)<=.007)&(abs(y)<=.006)&(abs(bx)<BRISTLE_HALF_WIDTH_M)&(abs(by)<BRISTLE_HALF_HEIGHT_M)&(abs(bz)<=CONTACT_MARGIN_M)
    # Tangential displacement of the same brush material point, including rigid rotation.
    px=previous_face[0]+previous_rotation[0,0]*bx+previous_rotation[0,1]*by+previous_rotation[0,2]*bz
    py=previous_face[1]+previous_rotation[1,0]*bx+previous_rotation[1,1]*by+previous_rotation[1,2]*bz
    slip=np.sqrt((x-px)**2+(y-py)**2)
    active=mask&(slip>1e-5)&(slip<.003)
    dose=np.where(active,800*slip*min(normal_force/.15,1.),0.)
    return active,slip,dose
