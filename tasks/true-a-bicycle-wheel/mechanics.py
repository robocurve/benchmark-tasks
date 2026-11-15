"""CPU-only, small-displacement prestressed-wheel feasibility model.

This is an engineering probe, NOT an Isaac scene or a successful robot trial.
Nipple turns change spoke rest lengths. A coupled elastic ring responds by
minimising strain energy; a slack spoke cannot push. No target shape is supplied
to the solver. Material/section values are uncalibrated development choices.

The linearized ring is useful below roughly 2% displacement/radius. It omits
rim torsion, geometric buckling, spoke crossings and threaded tool contact;
those omissions prohibit treating its output as task-validation evidence.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import numpy as np


@dataclass(frozen=True)
class WheelSpec:
    spokes: int = 32
    rim_radius_m: float = .210
    hub_flange_radius_m: float = .025
    hub_half_width_m: float = .025
    spoke_diameter_m: float = .002
    spoke_modulus_pa: float = 200e9
    rim_modulus_pa: float = 69e9
    rim_area_m2: float = 1.0e-4
    rim_second_moment_m4: float = 2.0e-9
    baseline_tension_n: float = 700.
    nipple_pitch_m_per_turn: float = .000454
    cross_angle_rad: float = .60

    def validate(self):
        if type(self.spokes) is not int or self.spokes < 8 or self.spokes % 2:
            raise ValueError("An even number of at least eight spokes is required")
        for name, value in asdict(self).items():
            if name != "spokes" and (not np.isfinite(value) or value <= 0):
                raise ValueError(f"Invalid positive parameter: {name}")
        if self.hub_flange_radius_m >= self.rim_radius_m:
            raise ValueError("Hub must be inside rim")


class WheelModel:
    def __init__(self, spec: WheelSpec | None = None):
        self.spec = spec or WheelSpec()
        s = self.spec
        s.validate()
        n = s.spokes
        theta = np.arange(n) * 2 * np.pi / n
        side = np.where(np.arange(n) % 2 == 0, 1., -1.)
        self.reference = np.column_stack((s.rim_radius_m*np.cos(theta),
                                         s.rim_radius_m*np.sin(theta), np.zeros(n)))
        anchors = np.column_stack((s.hub_flange_radius_m*np.cos(theta+side*s.cross_angle_rad),
                                   s.hub_flange_radius_m*np.sin(theta+side*s.cross_angle_rad),
                                   s.hub_half_width_m*side))
        vectors = self.reference-anchors
        self.spoke_lengths = np.linalg.norm(vectors, axis=1)
        directions = vectors/self.spoke_lengths[:, None]
        self.spoke_k = s.spoke_modulus_pa*np.pi*(s.spoke_diameter_m/2)**2/self.spoke_lengths
        self.A = np.zeros((n, 3*n))
        self.K = np.zeros((3*n, 3*n))
        chord = 2*s.rim_radius_m*np.sin(np.pi/n)
        hoop_k = s.rim_modulus_pa*s.rim_area_m2/chord
        bend_k = s.rim_modulus_pa*s.rim_second_moment_m4/chord**3
        for i in range(n):
            j = (i+1) % n
            self.A[i, 3*i:3*i+3] = directions[i]
            edge = self.reference[j]-self.reference[i]
            row = np.zeros(3*n)
            row[3*i:3*i+3] = -edge/chord
            row[3*j:3*j+3] = edge/chord
            self.K += hoop_k*np.outer(row, row)
            for axis in range(3):
                row = np.zeros(3*n)
                row[3*((i-1)%n)+axis] = 1
                row[3*i+axis] = -2
                row[3*j+axis] = 1
                self.K += bend_k*np.outer(row, row)
            # Prestress transverse tangent, held at the reference equilibrium.
            sl = slice(3*i, 3*i+3)
            self.K[sl, sl] += s.baseline_tension_n/self.spoke_lengths[i] * (
                np.eye(3)-np.outer(directions[i], directions[i]))

    def equilibrium(self, nipple_turns, *, external_forces_n=None, tolerance_n=1e-7, max_iterations=100):
        s = self.spec
        turns = np.asarray(nipple_turns, dtype=float)
        if turns.shape != (s.spokes,) or not np.isfinite(turns).all():
            raise ValueError("One finite nipple-turn measurement per spoke is required")
        forces = np.zeros((s.spokes, 3)) if external_forces_n is None else np.asarray(external_forces_n, dtype=float)
        if forces.shape != (s.spokes, 3) or not np.isfinite(forces).all():
            raise ValueError("One finite xyz external force per rim node is required")
        shortening = turns*s.nipple_pitch_m_per_turn
        active = np.ones(s.spokes, dtype=bool)
        visited = set()
        for iteration in range(max_iterations):
            state = tuple(active)
            if state in visited:
                raise RuntimeError("Slack-spoke active set did not converge")
            visited.add(state)
            Aa = self.A[active]
            ka = self.spoke_k[active]
            matrix = self.K + Aa.T @ (ka[:, None]*Aa)
            rhs = self.A.T @ np.full(s.spokes, s.baseline_tension_n)
            rhs -= Aa.T @ (ka*shortening[active]+s.baseline_tension_n)
            rhs += forces.ravel()
            u = np.linalg.solve(matrix, rhs)
            trial = s.baseline_tension_n+self.spoke_k*(self.A@u+shortening)
            new_active = trial > 0
            if np.array_equal(new_active, active):
                tension = np.maximum(trial, 0.)
                residual = self.K@u+self.A.T@(tension-s.baseline_tension_n)-forces.ravel()
                if np.max(np.abs(residual)) > tolerance_n:
                    raise RuntimeError("Equilibrium force residual exceeds tolerance")
                points = self.reference+u.reshape(-1, 3)
                return {
                    "positions_m": points,
                    "displacement_m": u.reshape(-1, 3),
                    "tension_n": tension,
                    "slack_spokes": np.flatnonzero(~active),
                    "maximum_force_residual_n": float(np.max(np.abs(residual))),
                    "maximum_displacement_ratio": float(np.max(np.linalg.norm(u.reshape(-1,3), axis=1))/s.rim_radius_m),
                    "lateral_peak_to_peak_m": float(np.ptp(points[:, 2])),
                    "dish_offset_m": float(np.mean(points[:, 2])),
                    "radial_peak_to_peak_m": float(np.ptp(np.linalg.norm(points[:, :2], axis=1))),
                    "iterations": iteration+1,
                    "load_prediction_in_scope": bool(np.all(active)),
                }
            active = new_active
        raise RuntimeError("Equilibrium did not converge")


RESET_PROFILES = ("four-defect-reference", "two-defect-candidate", "two-defect-corrected-control", "zero-defect-control")


def starting_turn_errors(spec: WheelSpec | None = None, profile="four-defect-reference"):
    """Published deterministic resets; neither is a calibrated product.

    The historical four-defect reset stays the default so old comparisons remain
    reproducible. The shorter candidate requires native verification before it
    is promoted to the actual task reset.
    """
    s = spec or WheelSpec()
    turns = np.zeros(s.spokes)
    if profile == "four-defect-reference":
        turns[[0, 2, 4, 6]] = -1.1
    elif profile == "two-defect-candidate":
        turns[[0, 2]] = -.75
    elif profile == "two-defect-corrected-control":
        turns[[0, 2]] = -.25
    elif profile == "zero-defect-control":
        pass  # Native control for restoring both loose nipples by three quarter-turns.
    else:
        raise ValueError(f"Unknown wheel reset profile: {profile}")
    return turns
