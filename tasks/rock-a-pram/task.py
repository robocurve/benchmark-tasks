"""Arm mounting shelf and level-floor pram scene. Run with run.py."""
from yam_arena.task_file import Box
from body_contact import SHELLS
SCENE='station'
OBJECTS=[Box('arm_mount_beam',pos=(0,0,.690),size=(.14,.90,.06),color=(.20,.22,.25)),*SHELLS]
def success(objects):return False
