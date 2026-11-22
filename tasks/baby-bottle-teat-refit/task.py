"""Fit the silicone teat and screw collar onto the bottle, then release."""
from yam_arena.task_file import Box

SCENE = "station"
SCORE_OVER = "end"
OBJECTS = [Box("build_anchor", pos=(.48,.20,.748), size=(.005,.005,.001), color=(.3,.3,.3))]

def score(objects):
    from monitor import end_score
    return end_score()

def demo():
    return {}
