"""Native YAM station; task materials are coupled by run.py."""
SCENE="station"
from yam_arena.task_file import Box
OBJECTS=[Box('workmat',pos=(.34,0,.7480),size=(.65,.85,.001),color=(.30,.31,.32))]
def success(objects):return False
