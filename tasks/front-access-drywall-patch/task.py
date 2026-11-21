"""End score of the loose-stock repair, or zero for an isolated component probe."""
from yam_arena.task_file import Box

SCENE = "station"
SCORE_OVER = "end"
OBJECTS = [Box("build_anchor", pos=(.53, .30, .748), size=(.003, .003, .001))]


def score(objects):
    import repair_scene
    return repair_scene.CURRENT.score if repair_scene.CURRENT is not None else 0.0


def demo():
    return {}
