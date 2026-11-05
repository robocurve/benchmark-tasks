"""Ink is supplied only by the embodiment's measured nib/paper contacts.

Spatial coverage, rather than summed jitter distance, rejects stationary dots.
The raster is also the scorer's simple ruler approximation: 0.5 mm cells.
"""
import math
import numpy as np

GRID = 0.0005
MIN_COVERAGE = 0.010
MIN_SPAN = 0.012
BOX_HALF = 0.011


class Ink:
    def __init__(self, boxes, answers):
        self.boxes = np.asarray(boxes, dtype=float)
        self.answers = tuple(answers)
        self.cells = [[set() for _ in row] for row in self.boxes]
        self.previous = None
        self.last_time = -math.inf
        self.segments = []
        self.dots = []
        self.contact_count = 0

    def contact(self, tip_xy, time):
        p = np.asarray(tip_xy, dtype=float)
        self.contact_count += 1
        anchor = p
        if self.previous is None or time-self.last_time > 0.009:
            self.dots.append(p.copy())
            self._register(p[None,:])
        # Missing more than two 240 Hz physics samples breaks a stroke.
        if self.previous is not None and 0 < time-self.last_time <= 0.009:
            a = self.previous
            length = float(np.linalg.norm(p-a))
            if length < 0.00005:
                anchor = a
            if 0.00005 <= length <= 0.004:
                self.segments.append((a.copy(), p.copy()))
                points = a + np.linspace(0, 1, max(2, math.ceil(length/(GRID/2))))[:,None]*(p-a)
                self._register(points)
        self.previous = anchor
        self.last_time = time

    def _register(self, points):
        for row, centers in enumerate(self.boxes):
            for col, center in enumerate(centers):
                for q in points[np.max(np.abs(points-center),axis=1) <= BOX_HALF-0.0006]:
                    self.cells[row][col].add(tuple(np.floor((q-center)/GRID+1e-8).astype(int)))

    def mark_present(self, row, col):
        return bool(self.cells[row][col])

    def long_stroke(self, row, col):
        remaining=set(self.cells[row][col])
        while remaining:
            component={remaining.pop()};pending=list(component)
            while pending:
                x,y=pending.pop()
                for dx in (-1,0,1):
                    for dy in (-1,0,1):
                        q=(x+dx,y+dy)
                        if q in remaining:
                            remaining.remove(q);component.add(q);pending.append(q)
            if len(component)*GRID < MIN_COVERAGE:continue
            p=np.asarray(list(component),dtype=float)*GRID
            span=np.max(np.linalg.norm(p[:,None,:]-p[None,:,:],axis=-1))
            if span >= MIN_SPAN:return True
        return False

    def score(self):
        correct = 0
        for row, answer in enumerate(self.answers):
            if self.long_stroke(row, answer) and not any(self.mark_present(row,c) for c in range(len(self.boxes[row])) if c != answer):
                correct += 1
        return correct/len(self.answers)
