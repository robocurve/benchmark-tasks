"""Side-approach retention diagnostic; physical spring jaws with more preload."""
from flat_grip_task import *
def _preload(j):
    dx=-j.sign*.00030
    rest=(j.rest[0]+dx,*j.rest[1:])
    boxes=tuple(((b[0][0]+dx,*b[0][1:]),*b[1:]) for b in j.boxes)
    return replace(j,rest=rest,boxes=boxes)
OBJECTS=[replace(o,sliders=tuple(_preload(j) for j in o.sliders)) if o.name=='driver' else o for o in OBJECTS]
