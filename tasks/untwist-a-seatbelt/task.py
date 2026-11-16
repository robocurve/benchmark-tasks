"""Native YAM station; task materials are coupled by run.py."""
SCENE="station"
from yam_arena.task_file import Box
OBJECTS=[Box('anchor_left',pos=(.32,.228,.7675),size=(.070,.022,.040),color=(.30,.31,.33)),Box('anchor_right',pos=(.32,-.228,.7675),size=(.070,.022,.040),color=(.30,.31,.33))]
def success(objects):return False
