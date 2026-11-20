"""How a 12 AWG solid copper conductor bends: the moment it carries against how far it has been bent, with work
hardening. Pure numpy, so it runs offline and inside the simulator.

The numbers: E 117 GPa, conductor 2.05 mm, first yield 70 MPa, then stress = 70 MPa + 420 MPa x (plastic strain)^0.5.
The hardening is fitted to handbook annealed C11000 copper (yield 69 MPa, tensile 220 MPa). A round bar first yields
at 59 N mm, is fully plastic near 100 N mm, and keeps getting harder to bend as it is bent tighter.

The laptop check of this rule against beam theory is in the task kit notes (outlet-box/validate/wire_tests.py).
"""
from __future__ import annotations

import math

import numpy as np

E = 117e9  # Pa
D12 = 0.00205  # m, 12 AWG conductor
SIG_Y = 70e6  # Pa, first yield of annealed building wire
K_H, N_H = 420e6, 0.5  # hardening: stress = SIG_Y + K_H * (plastic strain) ** N_H
SIG_MAX = 320e6  # Pa: the true stress at copper's tensile strength; hardening stops there
I_AREA = math.pi * D12**4 / 64
EI = E * I_AREA  # N m^2
M_Y = SIG_Y * I_AREA / (D12 / 2)  # N m, moment at first yield
M_P = SIG_Y * D12**3 / 6  # N m, fully plastic moment with no hardening


def stress(eps, hard: float = 1.0):
    """Stress (Pa) at strain ``eps``; ``hard`` scales the work hardening (0: none)."""
    ey = SIG_Y / E
    e = np.abs(eps)
    return np.sign(eps) * np.where(e <= ey, E * e, np.minimum(SIG_Y + hard * K_H * np.maximum(e - ey, 0.0) ** N_H, SIG_MAX))


def _trapz(f: np.ndarray, x: np.ndarray) -> float:
    return float(np.sum((f[1:] + f[:-1]) * np.diff(x)) / 2)


def moment_curve(hard: float = 1.0, kmax: float = 600.0, n: int = 400) -> tuple:
    """(curvature 1/m, bending moment N m) of the round section bent once from straight."""
    r = D12 / 2
    kap = np.concatenate([[0.0], np.geomspace(1e-3, kmax, n)])
    y = np.linspace(0.0, r, 500)
    width = 2 * np.sqrt(np.maximum(r * r - y * y, 0.0))
    return kap, np.array([2 * _trapz(stress(k * y, hard) * y * width, y) for k in kap])


def flow_table(hard: float = 1.0) -> tuple:
    """(plastic curvature a joint has been through 1/m, the moment needed to bend it further N m), both rising."""
    kap, m = moment_curve(hard)
    kp = kap - m / EI
    keep = (m > M_Y) & (kp > 0)
    kp, m = np.maximum.accumulate(kp[keep]), np.maximum.accumulate(m[keep])
    return np.concatenate([[0.0], kp]), np.concatenate([[M_Y], m])


def plastic_step(q, rest, acc, groups, stiffness: float, segment: float, kp, m) -> tuple:
    """One update of the wire's permanent bends.

    ``q``, ``rest``: every wire joint axis's angle and rest angle (rad). ``groups``: (joints, axes per joint) indices
    into them, the axes of one joint in one row (a round wire has no preferred bending direction, so a joint's two
    axes are treated as one bend). ``acc``: the plastic curvature each joint has been through (1/m). Each joint is a
    spring of ``stiffness`` (N m/rad) about its rest; when its moment passes the copper's flow moment the rest moves
    so the moment sits on it, and the joint hardens. Returns (rest, acc, flow moment per joint)."""
    q, rest, acc = np.asarray(q, float), np.array(rest, float), np.array(acc, float)
    groups = np.asarray(groups, int)
    e = q[groups] - rest[groups]
    mag = np.linalg.norm(e, axis=1)
    over = mag * stiffness > np.interp(acc, kp, m)
    if np.any(over):
        # How far the rest moves: the bend left over must carry exactly the flow moment of the joint as hardened by
        # this very move (solved by bisection, so one large update gives the same answer as many small ones; the
        # simulator updates only once per control step).
        lo, hi = np.zeros(int(over.sum())), mag[over].copy()
        for _ in range(40):
            mid = (lo + hi) / 2
            too_little = stiffness * (mag[over] - mid) > np.interp(acc[over] + mid / segment, kp, m)
            lo, hi = np.where(too_little, mid, lo), np.where(too_little, hi, mid)
        move = (lo + hi) / 2
        rest[groups[over]] += e[over] * (move / mag[over])[:, None]
        acc[over] += move / segment
    return rest, acc, np.interp(acc, kp, m)
