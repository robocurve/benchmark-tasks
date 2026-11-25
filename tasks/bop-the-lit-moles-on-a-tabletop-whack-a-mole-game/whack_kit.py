"""The electronics of a light-up tabletop whack-a-mole game, as a pure-Python state machine.

What it models. The real unit has a few sprung mole heads in its deck. A round starts, then the unit lights one mole
at a time at random, for a short time; bopping a lit mole down past its switch scores a hit and puts its light out at
once; bopping a mole that is not lit costs a point; a lit mole that is not bopped in time goes dark and the next one
lights after a short random pause.

Here each mole head is a slide joint of the game's mechanism (yam_joints): `update()` is given the joints' positions
(metres, 0 at the top stop, negative when pressed) once per control step and runs the same rules. The lit mole is
shown by the kit's visibility lamp on the mole's dome (`lamp_values()` is the mechanism's lamp_logic). Nothing here
moves a body: a hit happens only when something physically presses a mole past its switch, and only when the
task says that something was the head of the held mallet (`who(mole) == "head"`). A lit mole pressed past its switch
by anything else (the handle, a loose mallet, a finger, the gripper) goes dark as the real unit's would, but earns no
hit: that lighting is lost (counted in `foul`).

Tuned numbers (all ESTIMATES; a person checks them on the real unit with a ruler and a stopwatch):
  N_LIT = 8 lightings a round; LIT_S = 1.6 s lit (the easier level); GAP_S = 0.6-1.4 s dark between lightings
  (uniform, seeded); never the same mole twice running; SWITCH = 4 mm of travel registers a press, RELEASE = 2 mm
  re-arms it, and a switch that closes again within DEBOUNCE_S = 0.25 s of opening is the same press (a mole
  bouncing under one blow is one bop); the round starts START_DELAY = 1.0 s after the trial's start signal (here: the mallet lifted), or at
  START_LATEST = 30 s.

How to use it (pure Python, offline and on the simulator host):

    import whack_kit as wk
    game = wk.Game(["m0", "m1", "m2", "m3", "m4"])
    game.reset(seed)                       # at each episode start
    game.update(t, {"m0": q0, ...}, started=..., who=lambda mole: "head" | "gripper" | "handle" | "loose" | "other")
    game.hits, game.wrong, game.bare, game.foul, game.missed, game.lit, game.done
"""
from __future__ import annotations

import random

N_LIT = 8
LIT_S = 1.6
GAP_S = (0.6, 1.4)
SWITCH = 0.004
RELEASE = 0.002
DEBOUNCE_S = 0.25
START_DELAY = 1.0
START_LATEST = 30.0


class Game:
    def __init__(self, moles, n_lit=N_LIT, lit_s=LIT_S, gap_s=GAP_S):
        self.moles = list(moles)
        self.n_lit, self.lit_s, self.gap_s = n_lit, lit_s, gap_s
        self.reset(0)

    def reset(self, seed):
        rng = random.Random(seed)
        order, prev = [], None
        for _ in range(self.n_lit):
            m = rng.choice([m for m in self.moles if m != prev])
            order.append(m)
            prev = m
        self.order = order
        self.gaps = [rng.uniform(*self.gap_s) for _ in range(self.n_lit)]
        self.seed = seed
        self.t_signal = None       # when the start signal was first seen
        self.t_start = None
        self.i = 0                 # index of the next lighting
        self.lit = None            # mole lit now
        self.t_on = None
        self.t_next = None
        self.hits = 0
        self.wrong = 0
        self.missed = 0
        self.bare = 0              # presses with a gripper finger on the mole
        self.foul = 0              # lit moles pressed by something other than the held mallet's head: no hit
        self.down = {m: False for m in self.moles}
        self.t_up = {m: -1e9 for m in self.moles}
        self.events = []
        self.t = 0.0

    @property
    def started(self):
        return self.t_start is not None

    @property
    def done(self):
        return self.started and self.i >= self.n_lit and self.lit is None

    def lamp_values(self):
        return {m: (m == self.lit) for m in self.moles}

    def update(self, t, q, started=False, who=None):
        """One control step: t (s since the episode began), q {mole: slide position (m)}, started (the start signal),
        who(mole) -> what pressed that mole: "head" (the head of the held mallet: the only press that can be a
        hit), "gripper" (a gripper finger), or anything else ("handle", "loose", "other")."""
        self.t = t
        if self.t_start is None:
            if started and self.t_signal is None:
                self.t_signal = t
            if (self.t_signal is not None and t >= self.t_signal + START_DELAY) or t >= START_LATEST:
                self.t_start = t
                self.t_next = t + self.gaps[0]
        for m in self.moles:
            pos = float(q.get(m, 0.0))
            if not self.down[m] and pos < -SWITCH:
                self.down[m] = True
                if t - self.t_up[m] < DEBOUNCE_S:
                    continue
                by = who(m) if who is not None else "other"
                if by == "gripper":
                    self.bare += 1
                if not self.started:
                    kind = "early"
                elif m == self.lit and by == "head":
                    kind = "hit"
                    self.hits += 1
                    self._dark(t)
                elif m == self.lit:
                    kind = "foul"          # the unit's switch closed, so the light goes out, but no hit is credited
                    self.foul += 1
                    self._dark(t)
                else:
                    kind = "wrong"
                    self.wrong += 1
                self.events.append((round(t, 2), m, kind, by))
            elif self.down[m] and pos > -RELEASE:
                self.down[m] = False
                self.t_up[m] = t
        if self.lit is not None and t >= self.t_on + self.lit_s:
            self.missed += 1
            self.events.append((round(t, 2), self.lit, "missed", ""))
            self._dark(t)
        if self.started and self.lit is None and self.i < self.n_lit and t >= self.t_next:
            self.lit, self.t_on = self.order[self.i], t
            self.events.append((round(t, 2), self.lit, "lit", ""))

    def _dark(self, t):
        self.lit = None
        self.i += 1
        if self.i < self.n_lit:
            self.t_next = t + self.gaps[self.i]

    def lit_age(self):
        return None if self.lit is None else self.t - self.t_on
