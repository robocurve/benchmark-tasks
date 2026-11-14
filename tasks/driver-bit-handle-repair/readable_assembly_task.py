"""Readable tool rack: identical label styling, shape-based type markings."""
from shifted_assembly_task import *
LABEL=(.88,.89,.86)
for name,y in [('flat',-.22),('cross',-.13),('hex',-.04)]:
    boxes=[((0,0,.001),(.032,.032,.002),DARK)]
    if name in ('flat','cross'):
        boxes.append(((0,0,.0021),(.022,.003,.0002),LABEL))
        if name=='cross':boxes.append(((0,0,.0021),(.003,.022,.0002),LABEL))
    else:
        for k in range(6):
            a=k*math.pi/3
            boxes.append(((.0095*math.cos(a),.0095*math.sin(a),.0021),(.011,.0025,.0002),LABEL,a+math.pi/2))
    # Each plaque shares its bit/rest centreline; all three form one straight row.
    OBJECTS.append(part('type_label_'+name,(.175,y,TOP),boxes,.01,fixed_to_world=True))
