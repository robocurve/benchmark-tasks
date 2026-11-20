"""Score exhausted sheet removal and actual lint retained on the fresh roll."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


class Scorer:
    def reset(self):
        self.since = None

    def evaluate(self, state):
        material = state["material"]
        sheet_free = material["peeled_m"] >= C.SHEET_LENGTH
        collected = material["collected"]
        cloth = np.asarray(state["objects"]["cloth"]["pose"])
        upright = quat_xyzw_to_matrix(cloth[3:])[2, 2] >= .94
        intact = upright and np.linalg.norm(cloth[:2] - C.CLOTH[:2]) < .05 and abs(cloth[2] - C.CLOTH[2]) < .012
        complete = sheet_free and collected >= 18 and intact
        if not complete:
            self.since = None
        elif self.since is None:
            self.since = state["time_s"]
        success = bool(complete and state["time_s"] - self.since >= .25)
        score = 1. if success else .60 if sheet_free and collected >= 10 and intact else .30 if sheet_free and intact else 0.
        return dict(score=score, success=success, retired_sheet=sheet_free,
                    lint_collected=collected, cloth_in_work_area=bool(intact),
                    **material)
