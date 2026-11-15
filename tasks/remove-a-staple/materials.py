"""Uncalibrated constitutive laws for a staple/paper feasibility probe.

No robot, grasp, extraction trigger or automatic attachment is implemented here.
The steel law is a one-dimensional elastoplastic bending hinge with linear
isotropic hardening. The paper law is an irreversible cohesive opening bond.
Both need a contact-capable geometry solver before this can become a task.
"""
from __future__ import annotations

from dataclasses import dataclass
import math


def positive(name, value):
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class SteelSpec:
    wire_width_m: float = .00050
    wire_thickness_m: float = .00030
    hinge_length_m: float = .00050
    young_modulus_pa: float = 200e9
    yield_stress_pa: float = 250e6
    hardening_fraction: float = .02

    def __post_init__(self):
        for name, value in vars(self).items():
            positive(name, value)

    @property
    def stiffness_nm_per_rad(self):
        return self.young_modulus_pa*self.wire_width_m*self.wire_thickness_m**3/(12*self.hinge_length_m)

    @property
    def initial_yield_moment_nm(self):
        return self.yield_stress_pa*self.wire_width_m*self.wire_thickness_m**2/6


@dataclass
class PlasticHinge:
    spec: SteelSpec
    plastic_angle_rad: float = 0.
    accumulated_plastic_angle_rad: float = 0.
    dissipated_work_j: float = 0.
    stored_hardening_j: float = 0.

    def advance(self, total_angle_rad):
        """Return material bending moment opposing the measured total angle.

        The caller must obtain the angle from physical rod geometry. A policy
        cannot call this to bend a staple; this is a material update only.
        """
        if not math.isfinite(total_angle_rad):
            raise ValueError("A finite measured angle is required")
        k = self.spec.stiffness_nm_per_rad
        h = k*self.spec.hardening_fraction
        trial = k*(total_angle_rad-self.plastic_angle_rad)
        yield_moment = self.spec.initial_yield_moment_nm+h*self.accumulated_plastic_angle_rad
        excess = abs(trial)-yield_moment
        plastic_increment = 0.
        if excess > 0:
            plastic_increment = excess/(k+h)
            sign = 1. if trial >= 0 else -1.
            self.plastic_angle_rad += sign*plastic_increment
            self.accumulated_plastic_angle_rad += plastic_increment
            self.dissipated_work_j += self.spec.initial_yield_moment_nm*plastic_increment
            self.stored_hardening_j = .5*h*self.accumulated_plastic_angle_rad**2
        moment = k*(total_angle_rad-self.plastic_angle_rad)
        return {
            "moment_nm": moment,
            "plastic_angle_rad": self.plastic_angle_rad,
            "plastic_increment_rad": plastic_increment,
            "elastic_energy_j": .5*moment**2/k,
            "dissipated_work_j": self.dissipated_work_j,
            "stored_hardening_j": self.stored_hardening_j,
        }


@dataclass(frozen=True)
class PaperSpec:
    # A numerical trial set; NONE of these are calibrated office-paper data.
    thickness_m: float = .00010
    bond_width_m: float = .00050
    peak_traction_pa: float = 15e6
    opening_at_peak_m: float = .00001
    fracture_energy_j_per_m2: float = 300.
    compression_stiffness_n_per_m: float = 75_000.

    def __post_init__(self):
        for name, value in vars(self).items():
            positive(name, value)
        if self.final_opening_m <= self.opening_at_peak_m:
            raise ValueError("Fracture energy must permit a softening branch")

    @property
    def final_opening_m(self):
        return 2*self.fracture_energy_j_per_m2/self.peak_traction_pa

    @property
    def area_m2(self):
        return self.thickness_m*self.bond_width_m


@dataclass
class PaperBond:
    spec: PaperSpec
    maximum_opening_m: float = 0.

    @property
    def broken(self):
        return self.maximum_opening_m >= self.spec.final_opening_m

    @property
    def damage(self):
        s, kappa = self.spec, self.maximum_opening_m
        if kappa <= s.opening_at_peak_m:
            return 0.
        if kappa >= s.final_opening_m:
            return 1.
        return s.final_opening_m*(kappa-s.opening_at_peak_m)/(kappa*(s.final_opening_m-s.opening_at_peak_m))

    def advance(self, separation_m):
        """Opening resistance from geometry; compression remains contact-like.

        This scalar opening law does not pretend to model paper bending,
        mixed-mode tearing, fibre direction, puncture, or tool/paper friction.
        """
        if not math.isfinite(separation_m):
            raise ValueError("Finite separation is required")
        self.maximum_opening_m = max(self.maximum_opening_m, separation_m)
        if separation_m < 0:
            force = self.spec.compression_stiffness_n_per_m*separation_m
        else:
            k = self.spec.peak_traction_pa*self.spec.area_m2/self.spec.opening_at_peak_m
            force = (1-self.damage)*k*separation_m
        return {"force_n": force, "damage": self.damage, "broken": self.broken,
                "maximum_opening_m": self.maximum_opening_m}


def clinched_staple_centerline(crown_width_m=.0128, leg_length_m=.006,
                              paper_stack_m=.0002, sample_m=.0005):
    """Planar installed-staple centreline, for geometry development only.

    It contains finite-radius approximately quarter-circle clinch bends. Two
    paper layers and their four original holes must be resolved independently;
    a centreline crossing is not permission to suppress contact dynamically.
    """
    import numpy as np
    for name, v in (("crown", crown_width_m), ("leg", leg_length_m),
                    ("stack", paper_stack_m), ("sample", sample_m)):
        positive(name, v)
    radius = .00035
    above = .00015
    # Bend begins below the lower sheet; the old draft began the quarter
    # circle at the top sheet and cut diagonally through the paper stack.
    bend_start = -(paper_stack_m+.00005)
    below = bend_start-radius
    vertical = above-bend_start
    horizontal = leg_length_m-vertical-math.pi*radius/2
    if horizontal <= 0:
        raise ValueError("Leg is too short for this stack and clinch radius")
    # Distinct samples preserve the U crown and each under-paper inward tail.
    right = [[crown_width_m/2, 0., above], [crown_width_m/2, 0., below+radius]]
    for a in np.linspace(0., -math.pi/2, 6)[1:]:
        right.append([crown_width_m/2-radius+radius*math.cos(a), 0., below+radius+radius*math.sin(a)])
    right.append([crown_width_m/2-radius-horizontal, 0., below])
    left = [[-x, y, z] for x,y,z in right]
    control = np.asarray(list(reversed(left))+right)
    result = [control[0]]
    for a, b in zip(control[:-1], control[1:]):
        count = max(1, math.ceil(float(np.linalg.norm(b-a))/sample_m))
        result.extend(a+(b-a)*t for t in np.linspace(0., 1., count+1)[1:])
    return np.asarray(result)
