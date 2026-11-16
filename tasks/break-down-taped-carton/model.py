"""Bottom-tape layout and conservative swept-blade coverage, in carton metres.

This is a geometry/scoring kernel, not a physics engine. The native articulated
carton and robot adapter are still required. No function changes object poses.
"""
from math import dist, isfinite

LENGTH, WIDTH, HEIGHT, BOARD = .22, .18, .18, .003
TAPE_WIDTH, WRAP = .048, .04
CELL = .002
FILM_Z = -BOARD/2-.00005
# Bottom is z=0. Tape runs along x at y=0, plus end folds across y.
CUT_PATHS = (
    ('seam', (-LENGTH/2, 0., 0.), (LENGTH/2, 0., 0.)),
    ('front_fold', (-LENGTH/2, -TAPE_WIDTH/2, 0.), (-LENGTH/2, TAPE_WIDTH/2, 0.)),
    ('back_fold', (LENGTH/2, -TAPE_WIDTH/2, 0.), (LENGTH/2, TAPE_WIDTH/2, 0.)),
)


def dot(a,b):return sum(x*y for x,y in zip(a,b))


def point_segment_distance(point,start,end):
    delta=[b-a for a,b in zip(start,end)]
    length2=dot(delta,delta)
    t=max(0.,min(1.,dot([p-a for p,a in zip(point,start)],delta)/length2)) if length2 else 0.
    return dist(point,[a+t*d for a,d in zip(start,delta)])


def cells():
    for name,a,b in CUT_PATHS:
        n=round(dist(a,b)/CELL)
        for i in range(n):
            yield name,tuple(x+(y-x)*(i+.5)/n for x,y in zip(a,b))


class BladeCoverage:
    def __init__(self):
        self.cells=list(cells());self.cut=set();self.previous=None

    def update(self,tip,normal_alignment,held,dt):
        tip=tuple(tip)
        valid=(held and normal_alignment >= .85 and
               all(isfinite(x) for x in tip) and FILM_Z <= tip[2] <= .001)
        if valid and self.previous is not None and 0 < dt <= .051:
            # Reject teleported/undersampled tool motion rather than fill gaps.
            if dist(self.previous,tip) <= .015:
                for i,(_,p) in enumerate(self.cells):
                    if point_segment_distance((*p[:2],0.),(*self.previous[:2],0.),(*tip[:2],0.)) <= .0011:self.cut.add(i)
        self.previous=tip if valid else None

    def groups(self):
        return {name:dict(cut=sum(i in self.cut for i,(n,_) in enumerate(self.cells) if n==name),
                          total=sum(n==name for n,_ in self.cells)) for name,_,_ in CUT_PATHS}

    @property
    def complete(self):return len(self.cut)==len(self.cells)
