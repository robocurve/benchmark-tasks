"""Native YAM station; task materials are coupled by run.py."""
SCENE="station"
from yam_arena.task_file import Box
OBJECTS=[Box('release_card',pos=(.25,-.115,.7480),size=(.035,.035,.001),color=(.94,.94,.88))]
def success(objects):return False
