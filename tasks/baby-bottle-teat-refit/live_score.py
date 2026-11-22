"""One second of live YAM states, scored at each policy step."""
from collections import deque

import numpy as np

from scoring import SEATING_GAP_MM, flange_gaps
from scoring import inspect_pair, metrics, pad_clearance


class AssemblyScore:
    def __init__(self, names, arm_paths, grid, tets):
        self.bottle = names.index("bottle")
        self.collar = names.index("collar")
        self.arm_paths = arm_paths
        self.grid = grid
        self.tets = tets
        self.frames = deque(maxlen=21)
        self.released_intervals = deque(maxlen=20)

    def observe(self, poses, nodes, arm_poses, contact=None):
        if not all(np.isfinite(value).all() for value in (poses, nodes, arm_poses)):
            raise ValueError("Non-finite live assembly state")
        if contact is not None:
            counts = np.asarray(contact)
            if len(counts) != 12 or not np.isfinite(counts).all():
                raise ValueError("Incomplete physics interval")
            self.released_intervals.append(not counts.any())
        collar = poses[self.collar]
        bottle = poses[self.bottle]
        soft = nodes[None]
        retained = metrics(soft, collar[None], self.grid)[1].all()
        seated = retained and (np.abs(flange_gaps(bottle[None], soft, self.grid,
                                                   self.tets)) <= SEATING_GAP_MM).all()
        threaded = seated and all(round(lug["under_fraction"] * 15) >= 12
                                 for lug in inspect_pair(collar, bottle)["lugs"])
        valid = threaded and pad_clearance(soft, arm_poses[None], self.arm_paths,
                                           np.array([0]), self.tets) > 0
        self.frames.append(bool(valid))
        return float(len(self.frames) == 21 and len(self.released_intervals) == 20
                     and all(self.frames) and all(self.released_intervals))
