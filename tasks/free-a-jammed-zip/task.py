"""Native YAM station; task materials are coupled by run.py."""
SCENE="station"
from yam_arena.task_file import Box
OBJECTS=[Box('weighted_washbag',pos=(.34,0,.7655),size=(.30,.28,.036),color=(.12,.22,.28))]
def success(objects):return False
