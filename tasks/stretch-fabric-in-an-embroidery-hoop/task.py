"""Native YAM station; task material law is coupled by run.py."""
from yam_arena.task_file import Box
SCENE="station"
OBJECTS=[Box('stand_base',pos=(.32,.14,.7505),size=(.12,.08,.006),color=(.45,.29,.15))]
def success(objects):return False
