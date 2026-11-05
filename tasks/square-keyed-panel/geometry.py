"""Generate four single-rigid-body, compound-collider square puzzle pieces in metres."""

STEP = .001
BODY = 50                    # body is 50 x 50 mm
HEIGHT = .024
DEPTH = 12                   # a tab projects 12 mm from its edge
NECK_WIDTH = 14              # neck and head make a T-shaped key
HEAD_WIDTH = 20              # 40% of the 50 mm body width
CLEARANCE = 2                # total slot width clearance, millimetres
# The offsets are different even after reversal; nominal mating tolerates +/-1 mm.
OFFSETS = [-12, -9, -6, -3]
# side, key index; +x/-y edges are male, opposing edges female
EDGES = [ [('right',0,'male'),('bottom',1,'male')],
          [('left',0,'female'),('bottom',2,'male')],
          [('top',1,'female'),('right',3,'male')],
          [('top',2,'female'),('left',3,'female')] ]


def cells_for(piece):
    cells={(x,y) for x in range(-25,25) for y in range(-25,25)}
    for side,key,kind in EDGES[piece]:
        centre=OFFSETS[key]
        # Local coordinates: u along the side, v positive outward from the body.
        for u in range(-25,25):
            for v in range(-14,12):
                w = NECK_WIDTH//2 if v < 4 else HEAD_WIDTH//2
                if kind == 'female':
                    # Reverse the tab profile into the body; widen by one cell at each side.
                    depth=-v-1
                    if not 0 <= depth < DEPTH + 1: continue
                    w = (NECK_WIDTH//2 + 1) if depth < 3 else (HEAD_WIDTH//2 + 1)
                elif not 0 <= v < DEPTH: continue
                # Half-open cells produce the stated even widths exactly.
                if not -w <= u-centre < w: continue
                x,y = {'right': (25+v,u), 'left':(-26-v,u),
                       'top':(u,25+v), 'bottom':(u,-26-v)}[side]
                if kind == 'male': cells.add((x,y))
                else: cells.discard((x,y))
    return cells


def rectangles(cells):
    # Maximal horizontal runs merged along y; each collider stays a convex box.
    remaining=set(cells)
    while remaining:
        x0,y0=min(remaining,key=lambda p:(p[1],p[0]))
        x1=x0
        while (x1,y0) in remaining: x1+=1
        y1=y0+1
        while all((x,y1) in remaining for x in range(x0,x1)): y1+=1
        for x in range(x0,x1):
            for y in range(y0,y1): remaining.remove((x,y))
        yield x0,x1,y0,y1
