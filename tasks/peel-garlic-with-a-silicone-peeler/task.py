"""Native YAM station; task material law is coupled by run.py."""
from yam_arena.task_file import Box
SCENE="station"
OBJECTS=[Box('chopping_board',pos=(.37,0.,.749),size=(.55,.50,.003),color=(.54,.34,.19))]
def success(objects):return False
