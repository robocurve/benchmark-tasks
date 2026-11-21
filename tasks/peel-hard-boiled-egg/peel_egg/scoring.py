"""Check the actual albumen and shell positions after controlled manipulation."""

import numpy as np

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.lifted = False
        self.tapped = False
        self.damaged = False
        self.previous = None
        self.dwell = 0.

    def evaluate(self, state):
        objects = state["objects"]
        white = np.asarray(objects["core"]["pose"][:3])
        speed = np.asarray(objects["core"]["velocity"][:3])
        shells = [np.asarray(objects[n]["pose"][:3]) for n in C.SHELL_BOUNDS]
        distance = [np.linalg.norm(x - white) for x in shells]
        held = any(np.linalg.norm(np.asarray(hand["points"][0]) - white) < .048
                   and .022 < hand["opening_m"] < .053 for hand in state["hands"].values())
        self.lifted |= bool(white[2] >= C.BOARD[2] + .05 and held)
        over_board = np.all(np.abs(white[:2] - C.BOARD[:2]) <= [.122, .1125])
        if self.previous is not None and over_board and self.lifted:
            previous_white, previous_speed = self.previous
            near_board = white[2] <= C.START[2] + .0015
            impact = previous_speed[2] < -.045 and speed[2] - previous_speed[2] > .035
            self.tapped |= bool(near_board and impact and held)
            self.damaged |= bool(near_board and previous_speed[2] < -.45)
        self.previous = white, speed
        self.damaged |= bool(white[2] < C.TABLE - .005 or
                             (not self.tapped and max(distance) > .025) or
                             (self.lifted and not held and speed[2] < -.43))
        released = bool(all(np.linalg.norm(np.asarray(hand["points"]) - white,
                                           axis=1).min() > .066
                            for hand in state["hands"].values()))
        plated = bool(np.linalg.norm(white[:2] - C.PLATE[:2]) < .048
                      and .019 < white[2] - C.TABLE < .041
                      and np.linalg.norm(speed) < .018)
        separated = sum(d > .046 for d in distance)
        shell_clear = all(np.linalg.norm(p[:2] - C.PLATE[:2]) > .098 for p in shells)
        if self.tapped and separated == len(C.SHELL_BOUNDS) and plated and shell_clear and released and not self.damaged:
            self.dwell += .05
        else:
            self.dwell = 0.
        if self.damaged or not self.tapped:
            score = 0.
        elif self.dwell >= .999:
            score = 1.
        elif separated == len(C.SHELL_BOUNDS):
            score = .8
        elif separated:
            score = .5
        else:
            score = .20
        return dict(score=score, success=score == 1., lifted=bool(self.lifted),
                    tapped=bool(self.tapped), damaged=bool(self.damaged),
                    detached_shells=int(separated), shell_distances_m=distance,
                    plated=plated, shell_clear=bool(shell_clear), released=released,
                    dwell_s=self.dwell)
