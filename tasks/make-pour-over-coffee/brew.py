"""Water model for the pour-over (no fluids in the simulator).

Kettle (Hario V60 Drip Kettle Air, 350 mL): a cylinder of inner radius R_K; its spout tip sits at TIP in the kettle's
frame (origin at the centre of its base, +x towards the spout). The water surface is level; it meets the kettle's axis
at V / A_K from the base. Water leaves the tip when the surface is above it: Q = CDA * sqrt(2 g head).

Cone (Hario V60 01): a cone from the outlet (radius R_HOLE) to the rim (radius R_RIM, CONE_D higher). The coffee bed
fills it to H_BED and first soaks up RETAIN mL (the bloom) before any water drains. Water standing above the bed drains
through it at Q = K_DRAIN * depth. It overflows once the water above the bed reaches the rim.
"""
from __future__ import annotations
import math

G = 9.81
R_K = 0.037                  # m, kettle inner radius
A_K = math.pi * R_K ** 2
V_K0 = 350e-6                # m3 in the kettle at the start
TIP = (0.100, 0.0, 0.105)    # spout tip in the kettle's frame
CDA = 1.25e-5                # m2, discharge coefficient x tip area (about 11 mL/s at 4 cm head)
TAU_FLOW = 0.15              # s, the stream builds and stops over this

R_HOLE, R_RIM, CONE_D = 0.008, 0.0475, 0.070
H_BED = 0.0355               # m, top of the 15 g coffee bed above the outlet
RETAIN = 30e-6               # m3 held by the grounds (about 2 g per g)
K_DRAIN = 7.0e-5               # m2/s: about 2.7 mL/s with the cone full
MUG_R = 0.037                # m, glass mug inner radius


def cone_vol(h):
    r = R_HOLE + (R_RIM - R_HOLE) * h / CONE_D
    return math.pi / 3 * h * (R_HOLE ** 2 + R_HOLE * r + r * r)


V_BED = cone_vol(H_BED)
CAPACITY = cone_vol(CONE_D) - V_BED          # water the cone holds above the bed


def depth_of(w):
    """Depth of water standing above the bed for a volume w (m3)."""
    lo, hi = 0.0, CONE_D - H_BED + 0.02
    for _ in range(40):
        mid = (lo + hi) / 2
        if cone_vol(H_BED + mid) - V_BED < w:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def head(V, R, base):
    """Water head (m) over the spout tip, for kettle volume V, kettle rotation R (3x3 rows) and base position."""
    h_axis = V / A_K
    ez = [R[0][2], R[1][2], R[2][2]]
    z_surf = base[2] + h_axis * ez[2]
    z_tip = base[2] + sum(R[2][i] * TIP[i] for i in range(3))
    return z_surf - z_tip


class Brew:
    def __init__(self):
        self.V = V_K0          # in the kettle
        self.q = 0.0           # stream (m3/s)
        self.soak = 0.0        # held by the grounds
        self.W = 0.0           # standing in the cone above the bed
        self.mug = 0.0
        self.spilt = 0.0       # landed outside the cone
        self.poured = 0.0

    def step(self, hd, into_cone, dt):
        q_want = CDA * math.sqrt(2 * G * hd) if hd > 0 and self.V > 1e-6 else 0.0
        self.q += (q_want - self.q) * min(1.0, dt / TAU_FLOW)
        dv = min(self.q * dt, self.V)
        self.V -= dv
        self.poured += dv
        if into_cone:
            take = min(dv, RETAIN - self.soak)
            self.soak += take
            self.W += dv - take
        else:
            self.spilt += dv
        out = min(self.W, K_DRAIN * depth_of(self.W) * dt)
        self.W -= out
        self.mug += out

    @property
    def depth(self):
        return depth_of(self.W)

    @property
    def overflow(self):
        return self.W > CAPACITY

    @property
    def brewed(self):
        """What the mug will hold once the cone has finished dripping."""
        return self.mug + self.W
