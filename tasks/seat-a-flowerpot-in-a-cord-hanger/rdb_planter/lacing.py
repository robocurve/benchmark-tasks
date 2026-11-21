"""Cut lengths and unloaded drape of a four-sided diagonal cord lacing."""
import numpy as np
from scipy.optimize import least_squares
from . import fixed_scene as s

TOP_LINK = 12
LOW_LINK = 16
SEGMENTS = 6
RADIUS = .0018

def loaded_route(profile):
    """Trace one side of the occupied basket; keep the lace outside the pot."""
    t=np.linspace(0.,1.,SEGMENTS+1)
    z_top=profile[TOP_LINK,1]; z_bottom=profile[LOW_LINK,1]
    z=z_top+(z_bottom-z_top)*t
    # Start around the pot wall and pass around its lower edge before the
    # corner turns underneath the flat base. This sets actual cut lengths.
    rad=np.interp(z,profile[TOP_LINK:LOW_LINK+1,1][::-1],profile[TOP_LINK:LOW_LINK+1,0][::-1])
    rad+=.006*np.sin(np.pi*t)
    phi=np.pi*.25+t*np.pi*.5
    return np.column_stack((rad*np.cos(phi),rad*np.sin(phi),z))

def empty_route(profile,cut):
    t=np.linspace(0.,1.,SEGMENTS+1);phi=np.pi*(.25+.5*t)
    r=(1-t)*profile[TOP_LINK,0]+t*profile[LOW_LINK,0]
    z=(1-t)*profile[TOP_LINK,1]+t*profile[LOW_LINK,1]
    guide=np.column_stack((r*np.cos(phi),r*np.sin(phi),z))
    # Compress the empty basket by letting the slack bight hang down and
    # bow out of its quarter. Endpoints attach only to the physical cords.
    guide[:,2]-=.032*np.sin(np.pi*t)
    guide[:,:2]+=.018*np.sin(np.pi*t)[:,None]*np.column_stack((np.cos(phi),np.sin(phi)))
    def points(x):return np.vstack((guide[0],x.reshape(-1,3),guide[-1]))
    def error(x):
        path=points(x);length=np.linalg.norm(np.diff(path,axis=0),axis=1)
        return np.r_[1000*(length-cut),.16*(path[1:-1]-guide[1:-1]).ravel()]
    fit=least_squares(error,guide[1:-1].ravel(),max_nfev=700,ftol=1e-12,gtol=1e-12)
    path=points(fit.x)
    if np.max(np.abs(np.linalg.norm(np.diff(path,axis=0),axis=1)-cut))>1.e-6:
        raise ValueError('Unloaded cord has inconsistent segment lengths')
    return path

def routes():
    from .cradle_control import rest_profile,seated_profile
    rest=rest_profile()
    loaded=loaded_route(seated_profile(rest))
    empty=empty_route(rest,np.linalg.norm(np.diff(loaded,axis=0),axis=1))
    for i in range(4):
        phi=i*np.pi*.5
        rz=np.array([[np.cos(phi),-np.sin(phi),0],[np.sin(phi),np.cos(phi),0],[0.,0.,1.]])
        yield i,s.HOOK+empty@rz.T,s.HOOK+loaded@rz.T
