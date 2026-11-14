"""Tool and supports translated together to improve right-wrist margin."""
from clearance_assembly_task import *
TOOL_SHIFT_X=.045
OBJECTS=[replace(o,pos=(o.pos[0]+TOOL_SHIFT_X,*o.pos[1:])) if o.name=='driver' or o.name.startswith(('tool_support','tool_backstop')) else o for o in OBJECTS]
