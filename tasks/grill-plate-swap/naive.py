"""Wrong-strategy control for the occupied scene. Starts the swap immediately."""

from task import OBJECTS, SCENE, SCORE_OVER, SUCCESS_HOLD_S, score, success
from task import demo as wait_demo


def demo():
    return wait_demo(wait_for_diner=False)
