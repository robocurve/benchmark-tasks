"""A pointed blue steel tool is bolted to the left gripper. Its exposed tip touches a silver reference contact.

The black reference post is 6 cm square and 5 cm tall. Its silver contact is 2 mm across and 1 mm tall.
The contact centre is at x 27 cm, y 22 cm, 5.1 cm above the table.
Hold contact for one second at each of four tool axes, separated by at least 15 degrees in three dimensions.
Keep the tool between 10 and 35 degrees from vertical. The right arm stays clear.

Run with the yam_tool_calibration embodiment. The tool mount uses a native fixed joint.
The embodiment scores measured contact force, tip position, and the fixed tool-to-gripper transform.
"""

from yam_arena.task_file import Box, TABLE_TOP

SCENE = "station"
OBJECTS = [Box('reference_base', pos=(.27,.22,TABLE_TOP[SCENE]+.002),
               size=(.08,.08,.004), color=(.95,.45,.05))]


def success(objects):
    return False  # The custom embodiment reads tool poses and the contact sensor.
