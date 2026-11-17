"""Native YAM station; task material law is coupled by run.py."""
from yam_arena.task_file import Box
SCENE="station"
OBJECTS=[Box('edge_support',pos=(.32,.010,.775),size=(.10,.025,.055),color=(.43,.28,.15)),Box('blunt_ruler',pos=(.32,0.,.7975),size=(.090,.001,.030),color=(.62,.66,.70))]
def success(objects):return False
