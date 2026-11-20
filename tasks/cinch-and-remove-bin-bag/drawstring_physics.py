"""Force-based sliding drawstring for an ordinary drawstring trash bag.

One continuous string runs around the mouth hem and leaves it at two opposite
cut-outs, where a short loop is exposed on each side. The string is modelled
as a unilateral routed-length spring: tension acts along every hem segment
and on the end links of the two strap loops, so pulling a loop away from its cut-out slides
string out of the hem and gathers the mouth. There is NO lock, ratchet or
one-way element: when the loops are released the string tension drops to zero
and only film contact/friction keeps the mouth gathered, as on a real bag.

This is a lumped, uncalibrated model, not explicit rope/hem contact. Forces are
applied to dynamic rigid hem markers and strap end links; no poses are set.
"""
import numpy as np


def routed_length_gradient(points):
    delta = np.diff(points, axis=0)
    length = np.linalg.norm(delta, axis=1)
    direction = delta / np.maximum(length[:, None], 1e-9)
    gradient = np.zeros_like(points)
    gradient[:-1] -= direction
    gradient[1:] += direction
    return float(length.sum()), gradient


class SlidingDrawstring:
    """One string: n hem markers, then the two end links of each exposed loop.

    positions rows: hem markers 0..n-1, then loop0 end a, loop0 end b, loop1
    end a, loop1 end b. The exposed loops themselves are physical strap chains
    (loop_chain.py) that carry the string tension through their joints; this
    force acts only on the string's hidden run inside the hem and on the newly
    exposed run between each strap end and its hem cut-out.
    """

    def __init__(self, positions, exit_index, stiffness=40., damping=.5, hem_mass=.001, end_mass=.002, slack=.002, tension_cap=.7):
        positions = np.asarray(positions)
        self.tension_cap = tension_cap
        self.n = len(positions) - 4
        self.k = stiffness
        self.d = damping
        n = self.n
        c0 = int(exit_index) % n
        c1 = (c0 + n // 2) % n
        half0 = [(c0 + i) % n for i in range(n // 2 + 1)]
        half1 = [(c1 + i) % n for i in range(n // 2 + 1)]
        # loop0 end b -> cut-out 0 -> hem (one side) -> cut-out 1 -> loop1 end b ;
        # loop1 end a -> cut-out 1 -> hem (other side) -> cut-out 0 -> loop0 end a
        self.routes = [np.array([n + 1, *half0, n + 3]), np.array([n + 2, *half1, n])]
        self.cutouts = (c0, c1)
        self.rest = sum(routed_length_gradient(positions[r])[0] for r in self.routes) + slack
        self.mass = np.r_[np.full(n, hem_mass), np.full(4, end_mass)]
        self.max_extension = 0.
        self.last = {}

    def length_gradient(self, positions):
        G = np.zeros_like(positions)
        total = 0.
        for r in self.routes:
            length, g = routed_length_gradient(positions[r])
            np.add.at(G, r, g)
            total += length
        return total, G

    def forces(self, positions, velocities, dt=1 / 240):
        length, G = self.length_gradient(positions)
        extension = length - self.rest
        rate = float((G * velocities).sum())
        self.max_extension = max(self.max_extension, max(0., extension))
        # Implicit (backward-Euler) unilateral spring along the routed length.
        W = float(np.einsum('ij,ij,i->', G, G, 1 / self.mass))
        A = 1. + (dt * self.d + dt * dt * self.k) * W
        rhs = self.k * extension + (dt * self.k + self.d) * rate
        # Elastic part capped (numerical safeguard: 0.7 N, about twice the 0.4 N
        # each strand carries when the 160 g bag hangs from it). The viscous
        # part stands for the string's sliding friction in the hem; it is
        # limited separately, so a hanging bag cannot bounce on the capped
        # spring without losing energy.
        elastic = min(self.k * extension / A, self.tension_cap) if extension > 0 else 0.
        viscous = float(np.clip((dt * self.k + self.d) * rate / A, -self.tension_cap, self.tension_cap))
        tension = max(0., elastic + viscous) if (extension + dt * rate > 0) else 0.
        out = -tension * G
        hem = np.r_[np.arange(self.n), 0]
        perimeter = routed_length_gradient(positions[hem])[0]
        self.last = {'length': length, 'rest_length': self.rest, 'tension': float(tension),
                     'hem_perimeter': perimeter, 'max_extension': self.max_extension,
                     'net_force': out.sum(0).tolist()}
        return out
