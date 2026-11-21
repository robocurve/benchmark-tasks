"""On the station table, straight ahead of the robot, a grey steel machine housing stands 60 mm from a dark wall, both
18 cm long and 13 cm tall, running away from the robot. The gap between them is open at its near end, facing the
robot, and closed at the far end, at the bottom, and on top by a grey steel cover strip. Somewhere on the housing's
side face inside the gap, coolant has leaked: a bright green trace 22 mm wide and 50 mm tall, at one of five places
along the face, 20 mm apart and 7.5 to 15.5 cm in from the open end (different from trial to trial). Looking into the open end, that face is seen edge-on;
no camera sees it face-on.

In front of the right arm, an Ullman HTC-2LT lighted inspection mirror rests on two grey blocks: a black cushioned
grip (100 mm long, 12.7 mm across) pointing away from the robot, its free end toward the robot, a steel shaft
telescoped 120 mm past it, and at the far end a 60 mm round glass mirror in a black battery case with a built-in
light, hanging from its swivel, set at 45 degrees. Slid into the gap from its open end, the mirror shows the housing's side face to a camera behind the grip.

In front of the left arm, an oil-based paint marker (142 mm long, 16 mm across, yellow paint) stands tip down in a
steel pen cup (34 mm square, 60 mm tall) on a 50 mm steel stand. Where its tip touches the cover strip, it leaves a
yellow dot.
"""

from yam_arena.task_file import Box

from inspection_mirror import design as d

SCENE = "station"
SUPPORT_GREY = (0.50, 0.52, 0.55)


def _objects():
    # two support blocks under the mirror's grip, clear of its free end (the free end overhangs, so a hand
    # can take it from behind)
    objs = []
    for i, dx in enumerate(d.SUPPORT_DX):
        x = d.TOOL_REST[0] + dx
        objs.append(Box(f"support_{i}", pos=(float(x), float(d.TOOL_REST[1]), d.TOP + d.SUPPORT[2] / 2),
                        size=d.SUPPORT, color=SUPPORT_GREY))
    return objs


OBJECTS = _objects()


def success(objects):
    return False  # scored by the embodiment yam_inspection_mirror (inspection_mirror/scoring.py)
