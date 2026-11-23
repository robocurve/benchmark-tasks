"""Twelve finite panels on the repaired fold/shear path. Geometry, not physics."""
import itertools
import numpy as np
from scene import layout
from model import LENGTH,WIDTH
from crease import physical_bottom_angle
CORNERS=np.array(list(itertools.product((-1.,1.),repeat=3)))


def rx(degrees):
    a=np.radians(degrees);c,s=np.cos(a),np.sin(a)
    return np.array(((1.,0.,0.),(0.,c,-s),(0.,s,c)))


def configuration(theta,bottom=None,z0=.0045):
    """Canonical global orientation for a continuous closed-loop shear audit."""
    definitions,hinges=layout();definitions={p['name']:p for p in definitions}
    a=np.radians(theta);v=np.array((np.cos(a),0.,np.sin(a)));u=np.array((1.,0.,0.));up=np.array((0.,1.,0.))
    def rotation(x):return np.column_stack((x,np.cross(up,x),up))
    positions={};rotations={}
    for n,p,r in [('north',(0.,0.,z0),rotation(u)),('east',np.array((LENGTH/2,0,z0))+v*WIDTH/2,rotation(v)),('south',np.array((0.,0.,z0))+v*WIDTH,rotation(-u)),('west',np.array((-LENGTH/2,0,z0))+v*WIDTH/2,rotation(-v))]:positions[n]=np.asarray(p);rotations[n]=r
    for h in hinges:
        if not h['b'].endswith(('_top','_bottom')):continue
        angle=-270. if h['b'].endswith('_top') else (bottom or {}).get(h['a'],270.)
        rotations[h['b']]=rotations[h['a']]@rx(angle)
        positions[h['b']]=positions[h['a']]+rotations[h['a']]@h['anchor_a']-rotations[h['b']]@h['anchor_b']
    return definitions,hinges,positions,rotations


def vertices(definitions,positions,rotations):
    return {n:positions[n]+(CORNERS*np.asarray(p['size'])/2)@rotations[n].T for n,p in definitions.items()}


def obb_separation(a,b,definitions,positions,rotations):
    ra,rb=rotations[a],rotations[b];axes=np.concatenate((ra.T,rb.T,np.cross(ra.T[:,None,:],rb.T[None,:,:]).reshape(-1,3)))
    lengths=np.linalg.norm(axes,axis=1);axes=axes[lengths>1e-10]/lengths[lengths>1e-10,None]
    delta=positions[b]-positions[a];radius_a=np.abs(axes@ra)@np.asarray(definitions[a]['size'])/2;radius_b=np.abs(axes@rb)@np.asarray(definitions[b]['size'])/2
    return float(np.max(np.abs(axes@delta)-radius_a-radius_b))


def audit(theta,bottom=None):
    definitions,hinges,positions,rotations=configuration(theta,bottom)
    # Only pairs whose native joint disables their collisions are omitted.
    excluded={frozenset((h['a'],h['b'])) for h in hinges if not h['b'].endswith('_top')}
    failures=[]
    for a,b in itertools.combinations(definitions,2):
        if frozenset((a,b)) in excluded:continue
        gap=obb_separation(a,b,definitions,positions,rotations)
        if gap<-.00001:failures.append((a,b,gap))
    all_vertices=np.concatenate(list(vertices(definitions,positions,rotations).values()))
    return dict(height_span_m=float(np.ptp(all_vertices[:,2])),penetrations=failures)


def packet_thickness(poses,definitions,normal):
    coordinates=np.concatenate([p+((CORNERS*np.asarray(definitions[n]['size'])/2)@r.T) for n,(p,r) in poses.items()])@normal
    return float(np.ptp(coordinates))
