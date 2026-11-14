"""grindkit: grinding peppercorns in a mortar by fracture of real pieces (the bookkeeping; no simulator needed).

The batch is a set of rigid bodies in the simulator (spawned by cornkit): whole corns in the bowl at the start and,
parked out of sight, a pool of cracked pieces. Each control step the task gives this kit where every body is (in the
mortar's own frame), where the pestle's head is, and how hard the corns are pressed (the corns lie on a sprung disk,
so its compression times its stiffness is the force through them). The kit decides which pieces are caught under the
head, adds to each the work done on it, and breaks a piece once that work reaches its rupture energy:

- A whole corn breaks into `split` cracked pieces (still over the sieve's opening): the corn's body is parked, and as
  many pool bodies are moved to where it was, side by side on the same floor.
- A cracked piece breaks into medium pepper (through a 1.4 mm sieve, held on 0.6 mm): its body is parked and its
  mass is counted. (Pepper under 1.4 mm is far too fine to simulate as bodies; it is counted by mass.)
- Medium pepper is ground fine (through 0.6 mm) where the head rubs it against the bowl's bottom: while the head is
  pressed and slides with its end on the bottom (within `floor_touch`, nothing caught under it), the friction work
  goes into the medium pepper under the head (the head's share of the bottom's area, the powder lying spread over
  it), at `e_fine_g` J per gram, a first-order process. Only the part of that work done on powder counts: a thin
  scatter of powder leaves the head rubbing bare stone, so the work is scaled by the powder's cover of the bottom,
  its mass over `layer_g` (one layer of medium particles over the whole bottom). That energy per gram is the cracked pieces' (`e_corn` per
  gram) scaled by Rittinger's law, energy proportional to the new surface made, 1/d_after - 1/d_before, between the
  stages' typical sizes (`piece_d` -> `medium_d` -> `fine_d`).

The rupture energy of a whole corn is a measured figure (`e_corn`, from compression tests of black pepper: the force
at rupture times the deformation at rupture); a cracked piece needs the same energy per gram. Energy reaches the
pieces caught under the head in two ways, shared equally among them:
- Rubbing: while pressed (the disk pushed down at least `press_min`), the head slides across the pieces by ds (the
  middle of its end, in the mortar's frame): the friction work mu x F x ds.
- Striking: a press that starts with the head coming down at speed v delivers the pestle's kinetic energy
  1/2 m v^2 (the pestle's own mass; a hand driving it adds more, which is not counted).

A piece is caught when its body is inside the head's end disk (seen along the pestle's axis) and touches the end's
plane (its top within `touch` of it, or pressed into it). Nothing is credited to pieces the head does not touch, to
a head that touches nothing, or while a gripper touches the disk. A live piece whose body leaves the bowl (beyond the
rim's radius, or below the mortar's foot) is spilled, for good. A mortar tipped past `tip_deg` empties: everything,
the counted pepper included, is spilled. Scorers read `passing()` (through 1.4 mm: medium and fine, over the starting
mass) and `fine_share()` (through 0.6 mm).

Plain Python and NumPy. `step()` returns the body moves the simulator must make: (index, position in the mortar's
frame) to bring a pool body in, (index, None) to park a body back in the pool.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

CORN, PIECE = "corn", "piece"
LIVE, POOL, GONE, OUT = "live", "pool", "gone", "out"  # in the bowl; parked; broken (parked); spilled


@dataclass(frozen=True)
class Law:
    corn_d: float = 0.0046  # m: a whole corn's diameter
    corn_g: float = 0.043  # g: a whole corn's mass
    e_corn: float = 0.043  # J: the work that breaks a whole corn
    split: int = 2  # cracked pieces per corn
    k: float = 5000.0  # N/m: the corn bed's spring
    travel: float = 0.003  # m: to its stop
    press_min: float = 0.0015  # m: pressed = the bed pushed down at least this
    mu: float = 0.3  # the head sliding over the corns (dynamic friction)
    touch: float = 0.0007  # m: a piece within this of the head's end plane is touching it
    max_slide: float = 0.01  # m of head sliding counted per control step at most
    pestle_kg: float = 0.33  # the pestle's mass, for a strike's energy
    head_r: float = 0.020  # m: the head's end radius
    tip_deg: float = 60.0  # mortar tilt that empties it
    medium_d: float = 0.00092  # m: typical medium particle (geometric mean of the 1.4 and 0.6 mm openings)
    fine_d: float = 0.00042  # m: typical fine particle (geometric mean of 0.6 and 0.3 mm)
    floor_touch: float = 0.001  # m: the head's end this close to the bottom rubs the powder lying on it
    bed_r: float = 0.031  # m: the bottom's radius (the powder lies spread over it)
    packing: float = 0.6  # share of a one-particle layer's volume the particles fill (loose)
    density: float = 1.2  # g/cm^3: a pepper particle

    @property
    def layer_g(self) -> float:
        """g of medium pepper that covers the whole bottom one particle deep."""
        return math.pi * self.bed_r ** 2 * self.medium_d * self.packing * self.density * 1e6

    @property
    def e_fine_g(self) -> float:
        """J per gram to grind medium pepper fine: the cracked pieces' energy per gram, scaled by Rittinger's law."""
        per_g = self.e_piece / self.piece_g
        return per_g * (1 / self.fine_d - 1 / self.medium_d) / (1 / self.medium_d - 1 / self.piece_d)

    @property
    def piece_d(self) -> float:
        """A cracked piece's diameter: the same volume as 1/split of a corn."""
        return self.corn_d / self.split ** (1.0 / 3.0)

    @property
    def piece_g(self) -> float:
        return self.corn_g / self.split

    @property
    def e_piece(self) -> float:
        """The same rupture energy per gram as a whole corn."""
        return self.e_corn / self.split

    def size(self, kind: str) -> float:
        return self.corn_d if kind == CORN else self.piece_d


def bodies(n_corns: int, law: Law) -> list:
    """The kinds of the bodies, in order: n whole corns, then n x split cracked pieces (the pool)."""
    return [CORN] * n_corns + [PIECE] * (n_corns * law.split)


class Grinder:
    """One trial's batch: `n_corns` whole corns (bodies 0..n-1) and a pool of n x split pieces (the rest)."""

    def __init__(self, n_corns: int, law: Law = Law()):
        self.law, self.n = law, int(n_corns)
        self.kind = bodies(self.n, law)
        self.state = [LIVE] * self.n + [POOL] * (self.n * law.split)
        self.work = np.zeros(len(self.kind))  # J taken by each body so far
        self.start = self.n * law.corn_g  # g
        self.medium = 0.0  # g through 1.4 mm, held on 0.6 mm
        self.fine = 0.0  # g through 0.6 mm
        self.fine_energy = 0.0  # J rubbed into the powder on the bottom
        self.spilled = 0.0  # g outside the mortar
        self.energy = 0.0  # J delivered to caught pieces in all
        self.breaks = {CORN: 0, PIECE: 0}
        self.presses = self.strikes = 0
        self.tipped = False
        self.caught = []  # indices caught under the head at the last step
        self._next = self.n  # the next pool body to bring in
        self._pressed = False
        self._floor = False  # the head rubbing the bottom's powder at the last step
        self._last_c = None  # the head's end middle at the last step (mortar frame)

    def mass(self, i: int) -> float:
        return self.law.corn_g if self.kind[i] == CORN else self.law.piece_g

    def _g(self, kind: str) -> float:
        return sum(self.mass(i) for i, s in enumerate(self.state) if s == LIVE and self.kind[i] == kind)

    @property
    def whole(self) -> float:
        return self._g(CORN)

    @property
    def cracked(self) -> float:
        return self._g(PIECE)

    def counts(self) -> tuple:
        """(whole corns, cracked pieces) in the bowl."""
        live = [k for k, s in zip(self.kind, self.state) if s == LIVE]
        return live.count(CORN), live.count(PIECE)

    @property
    def ground(self) -> float:
        """g through 1.4 mm (medium and fine), still in the mortar."""
        return self.medium + self.fine

    def passing(self) -> float:
        """Share of the starting mass through the 1.4 mm sieve (medium and fine, still in the mortar)."""
        return self.ground / self.start

    def fine_share(self) -> float:
        """Share of the starting mass through the 0.6 mm sieve."""
        return self.fine / self.start

    def catch(self, pos, c, a) -> list:
        """Live bodies caught under the head: inside its end disk along the axis a, touching its end plane at c."""
        L = self.law
        out = []
        for i, s in enumerate(self.state):
            if s != LIVE:
                continue
            r = L.size(self.kind[i]) / 2
            d = np.asarray(pos[i], float) - c
            h = float(d @ a)  # height above the end plane, along the pestle
            radial = float(np.linalg.norm(d - h * a))
            if radial <= L.head_r and -(r + L.touch) <= h <= r:
                out.append(i)
        return out

    def _break(self, i: int, p, moves: list) -> None:
        L = self.law
        if self.kind[i] == CORN:
            free = [j for j in range(self._next, len(self.kind)) if self.state[j] == POOL][:L.split]
            if len(free) < L.split:
                return  # (never: the pool holds split pieces for every corn)
            self.state[i] = GONE
            moves.append((i, None))
            rp = L.piece_d / 2
            ang = 2.399963 * i  # a different direction for each corn (golden angle)
            u = np.array([math.cos(ang), math.sin(ang), 0.0])
            z = float(p[2]) - (L.corn_d / 2 - rp)
            for k, j in enumerate(free):
                off = (k - (L.split - 1) / 2) * (L.piece_d + 0.0002)
                q = np.array([float(p[0]), float(p[1]), z]) + off * u
                self.state[j] = LIVE
                moves.append((j, q))
            self._next = free[-1] + 1
            self.breaks[CORN] += 1
        else:
            self.state[i] = GONE
            moves.append((i, None))
            self.medium += L.piece_g
            self.breaks[PIECE] += 1

    def step(self, pos, head_c, head_a, compression: float, other_on_bed: bool, tilt_deg: float, rim_r: float,
             v_down: float = 0.0, floor_gap: float = 1.0) -> list:
        """Advance one control step. pos: (N, 3) body positions in the mortar's frame (origin at the middle of its
        foot, z up); head_c, head_a: the middle of the head's end and the pestle's axis (head to handle), same frame;
        compression: the bed pushed down (m); other_on_bed: a gripper touches the bed; tilt_deg: the mortar's tilt;
        rim_r: the bowl's radius at the rim; v_down: the head's downward speed over the last steps (m/s);
        floor_gap: the head's lowest point above the bowl's bottom (m). Returns the body moves."""
        L, moves = self.law, []
        pos = np.asarray(pos, float)
        c, a = np.asarray(head_c, float), np.asarray(head_a, float)
        if tilt_deg > L.tip_deg:
            self.tipped = True
        if self.tipped:
            for i, s in enumerate(self.state):
                if s == LIVE:
                    self.state[i] = OUT
                    self.spilled += self.mass(i)
            self.spilled += self.ground
            self.medium = self.fine = 0.0
            self.caught, self._pressed, self._last_c = [], False, c
            return moves
        for i, s in enumerate(self.state):  # thrown out of the bowl
            if s == LIVE and (math.hypot(pos[i][0], pos[i][1]) > rim_r or pos[i][2] < -0.005):
                self.state[i] = OUT
                self.spilled += self.mass(i)
        caught = self.catch(pos, c, a)
        pressed = bool(caught) and compression >= L.press_min and not other_on_bed
        dw = 0.0
        if pressed and not self._pressed:
            self.presses += 1
            if v_down > 0.0:
                self.strikes += 1
                dw += 0.5 * L.pestle_kg * v_down ** 2
        ds = min(float(np.linalg.norm(c[:2] - self._last_c[:2])), L.max_slide) if self._last_c is not None else 0.0
        if pressed and self._pressed:
            dw += L.mu * L.k * min(compression, L.travel) * ds
        floor = (not caught and compression >= L.press_min and floor_gap <= L.floor_touch and not other_on_bed)
        if floor and self._floor and ds > 0.0 and self.medium > 0.0:  # rubbing the powder on the bottom
            w = L.mu * L.k * min(compression, L.travel) * ds
            under = self.medium * min(1.0, (L.head_r / L.bed_r) ** 2)
            w *= min(1.0, self.medium / L.layer_g)  # the part of the rubbing done on powder, not bare stone
            made = under * (1.0 - math.exp(-w / (L.e_fine_g * under)))
            self.medium -= made
            self.fine += made
            self.fine_energy += w
        self._floor = floor
        if pressed and dw > 0.0:
            self.energy += dw
            share = dw / len(caught)
            for i in caught:
                self.work[i] += share
                if self.work[i] >= (L.e_corn if self.kind[i] == CORN else L.e_piece):
                    self._break(i, pos[i], moves)
        self.caught, self._pressed, self._last_c = caught, pressed, c
        return moves
