"""Outcome-scored surface preparation on the original calibrated YAM station."""
from yam_arena.task_file import Box
SCENE='station'
SCORE_OVER='end'
OBJECTS=[Box('build_anchor',pos=(.48,.2,.748),size=(.005,.005,.001),color=(.3,.3,.3))]
def score(*a,**k):
    from surface_scoring import score as current_score
    return current_score()
def success(*a,**k):return False
def distance(*a,**k):return 1.
def demo(*a,**k):return {}
