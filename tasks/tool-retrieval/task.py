"""Recover the blue part from the low, closed-back grey machine guard.

The guard cannot move. Its low front opening excludes a top-down gripper.
Use the long grey spatula to draw the blue part out towards the robots, then
put the part in the green tray and return the spatula to its two supports.
The spatula is flat, with the broad blade pointing away from the robots.
The back and sides are closed; the tool must work through the front opening.
"""
from yam_arena.task_file import TABLE_TOP, Box
from tool_retrieval.scoring import History, evaluate

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Use the spatula to pull the blue part out from under the grey guard. Put the part in the green tray, return the spatula to its two supports, and release both objects."
PART = Box("part", pos=(.40, .075, TOP + .005), size=(.03, .025, .01), mass=.025, color=(.05, .3, .85))
# Placeholder bounds are replaced by the existing Arena spatula mesh.
TOOL = Box("spatula", pos=(.24, .27, TOP + .039), size=(.273, .059, .0074), mass=.08)
OBJECTS = [PART, TOOL,
    # Static authored hood: five collision panels plus external mounting lugs.
    # The 28 mm mouth and 20 mm observation slot retain the working clearances.
    Box("recovery_hood",pos=(.45,.055,TOP),size=(.23,.241,.04)),
    Box("tool_support_handle",pos=(.18,.27,TOP+.0175),size=(.025,.06,.035),color=(.42,.44,.46)),
    Box("tool_support_blade",pos=(.34,.27,TOP+.0175),size=(.025,.075,.035),color=(.42,.44,.46)),
    Box("tray_floor",pos=(.235,-.175,TOP+.002),size=(.11,.11,.004),color=(.10,.55,.20)),
    Box("tray_front",pos=(.1775,-.175,TOP+.007),size=(.005,.12,.014),color=(.10,.55,.20)),
    Box("tray_back",pos=(.2925,-.175,TOP+.007),size=(.005,.12,.014),color=(.10,.55,.20)),
    Box("tray_left",pos=(.235,-.1175,TOP+.007),size=(.11,.005,.014),color=(.10,.55,.20)),
    Box("tray_right",pos=(.235,-.2325,TOP+.007),size=(.11,.005,.014),color=(.10,.55,.20))]
HISTORY = History()
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0


def score(objects):
    return evaluate(objects, HISTORY, TOP)


def success(objects):
    return score(objects) == 1.0


def demo():
    from tool_retrieval.policy import make_plan
    return make_plan(TOP)
