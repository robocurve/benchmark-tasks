"""Engineering material model for coupled thread cutting, not a pose controller.

The two generalized coordinates are inward travel in metres and cutting angle
in radians. A passive axial spring penalizes departure from the thread pitch.
An elastic/perfectly plastic torsional element represents cutting resistance.
This isolated backward-Euler model has not yet been validated inside PhysX.
It neither creates geometry nor changes any native body state.

The default 0.11 Nm is a nominal reference to the reported average maximum for
oiled M3 tapping of 6061-T6 at 200 rpm in the 2022 thesis "The Measurement of
Torque for Micro Tapping of Aluminum Alloy 6061-T6" (CCUR item 51314).
It is not a calibration for hand speed, 6063-T5 extrusion, or tap breakage.
The axial/torsional stiffness and damping are uncalibrated engineering values.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class HelicalCuttingResponse:
    pitch_m: float = .0005
    axial_stiffness_N_m: float = 20000.
    axial_damping_N_s_m: float = 5.
    torsional_stiffness_Nm_rad: float = .5
    yield_torque_Nm: float = .11

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in (
            self.pitch_m, self.axial_stiffness_N_m, self.axial_damping_N_s_m,
            self.torsional_stiffness_Nm_rad, self.yield_torque_Nm)):
            raise ValueError('thread material parameters must be positive and finite')

    @property
    def lead_per_radian_m(self):
        return self.pitch_m/(2*math.pi)

    def energy(self, coordinates, plastic_angle_rad):
        q = np.asarray(coordinates, dtype=float)
        if q.shape != (2,) or not np.isfinite(q).all() or not math.isfinite(plastic_angle_rad):
            raise ValueError('invalid thread material coordinates')
        error = q[0]-self.lead_per_radian_m*q[1]
        twist = q[1]-plastic_angle_rad
        return .5*self.axial_stiffness_N_m*error**2 + .5*self.torsional_stiffness_Nm_rad*twist**2

    def step(self, coordinates, velocities, plastic_angle_rad, *, mass_kg,
             inertia_kg_m2, dt):
        """Solve an isolated passive response to supplied, already loaded velocity.

        External impulses, if any, precede this solve. Returned coordinates and
        velocities are predictions for engineering validation, never native pose
        or velocity commands. Production use needs native substep reconciliation;
        a model prediction alone cannot certify actual cutting or thread depth.
        """
        q = np.asarray(coordinates, dtype=float)
        v = np.asarray(velocities, dtype=float)
        if (q.shape != (2,) or v.shape != (2,) or not np.isfinite(q).all()
                or not np.isfinite(v).all() or not math.isfinite(plastic_angle_rad)
                or not all(math.isfinite(x) and x > 0 for x in (mass_kg, inertia_kg_m2, dt))):
            raise ValueError('invalid thread material state, mass, inertia or time step')
        masses = np.array((mass_kg, inertia_kg_m2))
        inverse = np.diag(1/masses)
        b = np.array((1., -self.lead_per_radian_m))
        outer = np.outer(b, b)
        stiffness = self.axial_stiffness_N_m*outer
        damping = self.axial_damping_N_s_m*outer
        torsion = np.diag((0., self.torsional_stiffness_Nm_rad))
        phi = np.array((0., plastic_angle_rad))
        matrix = np.eye(2)+dt*inverse@(dt*(stiffness+torsion)+damping)
        rhs = v-dt*inverse@(stiffness@q+torsion@(q-phi))
        next_v = np.linalg.solve(matrix, rhs)
        next_q = q+dt*next_v
        elastic_torque = self.torsional_stiffness_Nm_rad*(next_q[1]-plastic_angle_rad)
        yielded = abs(elastic_torque) > self.yield_torque_Nm
        next_phi = plastic_angle_rad
        if yielded:
            sign = math.copysign(1., elastic_torque)
            matrix = np.eye(2)+dt*inverse@(dt*stiffness+damping)
            rhs = v-dt*inverse@(stiffness@q+np.array((0., sign*self.yield_torque_Nm)))
            next_v = np.linalg.solve(matrix, rhs)
            next_q = q+dt*next_v
            next_phi = next_q[1]-sign*self.yield_torque_Nm/self.torsional_stiffness_Nm_rad
        reaction = masses*(next_v-v)/dt
        plastic_work = self.yield_torque_Nm*abs(next_phi-plastic_angle_rad)
        damping_work = dt*self.axial_damping_N_s_m*float(b@next_v)**2
        before = self.energy(q, plastic_angle_rad)+.5*float(masses@(v*v))
        after = self.energy(next_q, next_phi)+.5*float(masses@(next_v*next_v))
        residual = after+plastic_work+damping_work-before
        if residual > 1e-9*max(1., before):
            raise RuntimeError('thread response generated energy')
        return {'predicted_coordinates': next_q, 'predicted_velocities': next_v,
                'predicted_plastic_angle_rad': next_phi,
                'reaction_force_N': float(reaction[0]),
                'reaction_torque_Nm': float(reaction[1]),
                'yielded': yielded, 'plastic_work_J': plastic_work,
                'damping_work_J': damping_work, 'energy_before_J': before,
                'energy_after_J': after, 'energy_residual_J': residual}
