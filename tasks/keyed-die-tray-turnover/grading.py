"""At 20 Hz, grade the last second of the exposed receiver in its own key frame."""
from collections import deque
import json
from itertools import product
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from endpoint import covered, exposed, overlap
from layout import CELLS

CURRENT = None
EXPECTED = [(y, x) for x, y in CELLS]
CORNERS = np.array(list(product((-.0045, .0045), (-.0045, .0045), (-.00025, .00025))))
RIM = np.array([[x, y, 0] for x, y in product((-.0254, .0254), repeat=2)])
MOUTH = .0109
# Relative to the receiving rim. A nominal flat 0.50 mm back has 0.10 mm allowance.
MIN_BACK_RECESS = .00040
PITCH = .013
HISTORY = 21
OPENING = MOUTH / 2 * np.array([[-1,-1], [1,-1], [1,1], [-1,1]])


class Grader:
    def __init__(self, names, filters):
        self.index = {n: i for i, n in enumerate(names)}
        if [p.rsplit('/', 1)[-1] for p in filters[:4]] != ['source', 'receiver', 'source_frame', 'receiver_frame'] or len(filters) <= 4:
            raise ValueError('Invalid contact filter order')
        self.shape = (len(names), len(filters))
        self.receiver_parts = [self.index[n] for n in ('receiver', 'receiver_frame', 'receiver_knob')]
        self.dies = [self.index[f'd{i}'] for i in range(4)]
        geometry = json.loads(Path(__file__).with_name('source-holder-geometry.json').read_text())
        self.holder = [np.asarray(p) for p in geometry.values()]
        self.window = deque(maxlen=HISTORY)
        self.score = 0.
        self.good_cells = [False] * 4

    def update(self, poses, counts):
        if poses.shape != (self.shape[0], 7) or counts.shape != self.shape:
            raise ValueError('Invalid physics array shape')
        if not np.isfinite(poses).all() or not np.isfinite(counts).all() or (counts < 0).any() or counts.sum() >= 16384:
            raise ValueError('Invalid contact or transform sample')
        if not np.allclose(np.linalg.norm(poses[:, 3:], axis=1), 1, atol=2e-4):
            raise ValueError('Invalid body orientation')
        rec = poses[self.index['receiver']]
        rotation = Rotation.from_quat(rec[3:])
        inv = rotation.inv()
        rim = rotation.apply(RIM) + rec[:3]
        released = not counts[self.receiver_parts, 4:].any()
        ready = released and rotation.as_matrix()[2, 2] > np.cos(np.deg2rad(10)) and counts[self.index['receiver'], 3] > 0
        qualifying = [-1] * 4
        if ready:
            chips = poses[self.dies]
            centers = inv.apply(chips[:, :3] - rec[:3])
            orient = (inv * Rotation.from_quat(chips[:, 3:])).as_matrix()
            corners = centers[:, None, :] + np.einsum('nij,kj->nki', orient, CORNERS)
            for j, cell in enumerate(EXPECTED):
                # Also exclude a second die perched on or overhanging this mouth.
                center = PITCH * np.asarray(cell)
                xy = corners[:, :, :2] - center
                hits = ((xy.min(1) < MOUTH / 2).all(1) & (xy.max(1) > -MOUTH / 2).all(1)
                        & (corners[:, :, 2].max(1) >= -.00110))
                if hits.sum() > 1:
                    for candidate in np.flatnonzero(hits):
                        footprint = xy[candidate]
                        hull = footprint[ConvexHull(footprint).vertices]
                        hits[candidate] = overlap(hull, OPENING)
                if hits.sum() != 1:
                    continue
                i = np.flatnonzero(hits)[0]
                z = corners[i, :, 2]
                if (np.abs(xy[i]).max() <= MOUTH / 2 + .00005 and z.max() <= -MIN_BACK_RECESS
                        and z.min() >= -.00110 and -orient[i, 2, 2] > .8
                        and counts[self.dies[i], 1] > 0 and not counts[self.dies[i], 4:].any()):
                    qualifying[j] = int(i)
            if max(qualifying) >= 0:
                src = poses[self.index['source']]
                ready = exposed(inv.apply(src[:3] - rec[:3]), (inv * Rotation.from_quat(src[3:])).as_matrix(), MOUTH)
                if ready:
                    sf = poses[self.index['source_frame']]
                    t, r = inv.apply(sf[:3] - rec[:3]), inv * Rotation.from_quat(sf[3:])
                    ready = not any(covered(part, t, r) for part in self.holder)
        self.window.append((rim, bool(ready), qualifying))
        self.score = 0.
        self.good_cells = [False] * 4
        if len(self.window) == HISTORY and all(row[1] for row in self.window):
            rims = np.array([row[0] for row in self.window])
            if np.linalg.norm(rims - rim, axis=-1).max() <= .002:
                ids = np.array([row[2] for row in self.window])
                self.good_cells = ((ids[0] >= 0) & (ids == ids[0]).all(0)).tolist()
                self.score = sum(self.good_cells) / 4
        return self.score
