"""Record native contact witnesses and award full credit after parking and recording the reading."""


class Measurement:
    def __init__(self):
        self.last_tick = -1
        self.counts = [0, 0, 0]
        self.witness = [False, False, False]
        self.invalid = False
        self.score = 0.

    def update(self, tick, floor, roof, stop, intact, parked, recorded=False):
        if tick == self.last_tick:
            return self.score
        self.last_tick = tick
        self.invalid |= not intact
        for i, contact in enumerate((floor, roof, stop)):
            self.counts[i] = self.counts[i] + 1 if contact else 0
            self.witness[i] |= self.counts[i] >= 4
        lower, upper, blocked = self.witness
        if self.invalid:
            self.score = 0.
        elif lower and upper and blocked:
            self.score = 1. if parked and recorded else .75
        elif lower and upper:
            self.score = .50
        else:
            self.score = .25 if floor or lower else 0.
        return self.score
