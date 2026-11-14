"""Preassembled diagnostic with flat molded grip faces; not final task."""
from preassembled_task import *
# 40 mm square grip collar around existing 38 mm body, centered on grasp.
# Ordinary collision geometry; no attachment constraint or pose writes.
OBJECTS=[replace(o,boxes=o.boxes+(((0,0,.050),(.040,.040,.060),DARK),)) if o.name=='driver' else o for o in OBJECTS]
