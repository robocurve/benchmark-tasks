"""Actual assembly layout with tool-pickup clearance."""
from readable_assembly_task import *
OBJECTS=[replace(o,pos=(o.pos[0]+.045,*o.pos[1:])) if o.name=='drawer_context' or o.name.startswith('drawer_support') else o for o in OBJECTS]
