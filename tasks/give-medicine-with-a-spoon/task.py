"""Native station anchor; task materials are coupled by run.py."""
from yam_arena.task_file import Box
SCENE = "station"
OBJECTS = [Box('spoon_rest', pos=(.23, -.15, .7855), size=(.085, .060, .035), color=(.20, .32, .38))]
def success(objects):
    return False
