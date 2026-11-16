"""Measured bonded material footprints and blade-cut material connectivity.

This module is CPU-only. It neither moves objects nor creates adhesive state.
Coverage is the union of *current*, pressure-created adhesive patches on the
same physical strip. No tool path, control phase or elapsed episode time is used.
"""
import numpy as np
from bond_geometry import PATCHES
from config import CELL_LENGTH, CELLS
from model import TAPE_WIDTH, TAPE_THICKNESS, CARTON_LENGTH, CARTON_HEIGHT


def segment_box(a, b, half):
    """Whether a measured segment intersects a closed axis-aligned box."""
    a, b, half = map(np.asarray, (a, b, half))
    delta = b-a
    lo, hi = 0., 1.
    for axis in range(3):
        if abs(delta[axis]) < 1e-12:
            if abs(a[axis]) > half[axis]: return False
        else:
            t = sorted(((-half[axis]-a[axis])/delta[axis],
                        (half[axis]-a[axis])/delta[axis]))
            lo, hi = max(lo,t[0]), min(hi,t[1])
            if lo > hi: return False
    return True


def hinge_points(films, index):
    """Current hinge line from the two authored local anchors (not centres)."""
    from config import tape_cell
    p0,y0=tape_cell(index-1);p1,y1=tape_cell(index)
    anchor=(np.asarray(p0)+p1)/2
    endpoints=[]
    for i,p,y in ((index-1,p0,y0),(index,p1,y1)):
        r=np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
        film=films[f'tape_{i}']
        endpoints.append(np.asarray(film['pos'])+np.asarray(film['rotation'])@(r.T@(anchor-p)))
    center=np.mean(endpoints,axis=0)
    width=np.asarray(films[f'tape_{index}']['rotation'])[:,2]
    return np.array([center+width*((k+.5)/PATCHES-.5)*TAPE_WIDTH for k in range(PATCHES)])


class BladeCuts:
    """Whole-width measured blade crossings sever a native material hinge.

    Contact forces must come from the filtered film/cutter sensor. A root-level
    cutter force alone cannot cut: each width sample must also cross the actual
    14 x 0.5 x 8 mm blade collider in the measured moving cutter frame.
    """
    def __init__(self):
        self.previous={};self.covered={};self.cuts=set()

    def step(self, films, cutter, forces):
        rotation=np.asarray(cutter['rotation']);pos=np.asarray(cutter['pos'])
        new=[]
        for index in range(1,CELLS):
            if index in self.cuts: continue
            points=(hinge_points(films,index)-pos)@rotation-np.array((.062,0.,-.016))
            old=self.previous.get(index);self.previous[index]=points
            force=max(np.linalg.norm(forces.get((f'tape_{j}','cutter'),(0,0,0))) for j in (index-1,index))
            if old is None or force < .05: continue
            bins=self.covered.setdefault(index,set())
            for k,(a,b) in enumerate(zip(old,points)):
                if np.linalg.norm(b-a)>1e-6 and segment_box(a,b,(.007,.00025,.004)):
                    bins.add(k)
            if len(bins)==PATCHES:
                self.cuts.add(index);new.append(index)
        return new


def strip_component(cuts, anchor_errors, cell=CELLS-1):
    """Physical contiguous component, excluding cut or overstretched hinges."""
    component={cell};pending=[cell]
    while pending:
        i=pending.pop()
        for j,edge in ((i-1,i),(i+1,i+1)):
            if 0<=j<CELLS and edge not in cuts and anchor_errors.get(edge,float('inf'))<=CELL_LENGTH*.05 and j not in component:
                component.add(j);pending.append(j)
    return component


def bonded_coverage(films, bonds, component, origin, reference):
    """Conservative 4 x 12 samples per seam cell and end-wrap cell.

    A sample counts only inside an actual bonded film-width patch, on its named
    carton face. Four longitudinal quadrature locations avoid centre-only credit
    for a film cell whose footprint merely grazes a scoring cell.
    """
    origin=np.asarray(origin);reference=np.asarray(reference)
    patches=[]
    for (film_name,target),bins in bonds.items():
        if int(film_name.split('_')[1]) not in component:continue
        film=films[film_name];r=np.asarray(film['rotation']);p=np.asarray(film['pos'])+r[:,1]*TAPE_THICKNESS/2
        patches.append((target,set(bins),p,r))
    def covered(point, targets):
        world=origin+reference@np.asarray(point)
        for target,bins,p,r in patches:
            if target not in targets:continue
            v=r.T@(world-p)
            if abs(v[0])>CELL_LENGTH/2+1e-7 or abs(v[1])>.0005:continue
            k=int(np.floor((v[2]/TAPE_WIDTH+.5)*PATCHES))
            if 0<=k<PATCHES and k in bins:return True
        return False
    seam=[]
    for i in range(44):
        count=sum(covered((-CARTON_LENGTH/2+(i+(s+.5)/4)*CELL_LENGTH,
                           ((k+.5)/PATCHES-.5)*TAPE_WIDTH,CARTON_HEIGHT+.003),
                          {'north_top','south_top'}) for s in range(4) for k in range(PATCHES))
        seam.append(count/(4*PATCHES))
    ends=[]
    for sign,target in ((-1,'west'),(1,'east')):
        length=0.
        for i in range(8):
            count=sum(covered((sign*(CARTON_LENGTH/2+.0015),((k+.5)/PATCHES-.5)*TAPE_WIDTH,
                               CARTON_HEIGHT-(i+(s+.5)/4)*CELL_LENGTH),{target})
                      for s in range(4) for k in range(PATCHES))
            if count/(4*PATCHES)>=.8:length+=CELL_LENGTH
        ends.append(length)
    return seam,ends
