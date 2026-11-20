"""Compliant single-bore material response, independent of policy and task scoring.

The cutter advances only under load and follows measured bit motion. The bore is
created at the actual entry pose, never at the task's target. Forces act back on
the drill; this is a reduced material model, not a fracture simulation.
"""

from dataclasses import dataclass, field
import numpy as np

BIT_RADIUS = 0.003
SURFACE_STIFFNESS = 16000.0  # N/m; finite compliance is visible in diagnostics
SURFACE_DAMPING = 35.0      # N s/m
MIN_CUT_FORCE = 8.0        # N, an approximate softwood feed threshold
FEED_GAIN = 0.0004         # m/(N s)
MAX_FEED = 0.003           # m/s
WALL_STIFFNESS = 500.0     # N/m, compliant coupling for the tool's low tip inertia
WALL_DAMPING = 0.5         # N s/m
WALL_CLEARANCE = 0.0007    # m, binding after the bore actually exists
MAX_FORCE = 55.0           # numerical saturation, not the operating load
SPINDLE_RPM = 900.0


@dataclass
class CuttingModel:
    centre: np.ndarray
    size: np.ndarray
    surface_z: float
    depth: float = 0.0
    entry: np.ndarray | None = None
    bore_axis: np.ndarray | None = None
    rpm: float = 0.0
    phase: float = 0.0
    qualified_depth: float = 0.0
    maximum_force: float = 0.0
    previous_tip: np.ndarray | None = None
    engaged: bool = False
    last: dict = field(default_factory=dict)

    def step(self, tip, axis, trigger, dt, *, qualified=False):
        tip, axis = np.asarray(tip, float), np.asarray(axis, float)
        axis = axis / max(np.linalg.norm(axis), 1e-12)
        velocity = np.zeros(3) if self.previous_tip is None else (tip-self.previous_tip)/dt
        self.previous_tip = tip.copy()
        requested_rpm = SPINDLE_RPM * np.clip((trigger-0.001)/0.004, 0., 1.)
        self.rpm += (requested_rpm-self.rpm) * (1.-np.exp(-dt/0.12))
        self.phase = (self.phase + self.rpm*(2*np.pi/60)*dt) % (2*np.pi)
        force, torque = np.zeros(3), np.zeros(3)
        inside_xy = bool(np.all(np.abs(tip[:2]-self.centre) <= self.size[:2]/2))
        surface_penetration = self.surface_z-tip[2]
        if surface_penetration < -.001:
            self.engaged=False
        cut_increment = 0.
        axial_force = 0.
        wall_error = 0.
        # This model represents top-entry boring. The ordinary box collider
        # remains active for the housing, chuck, arms and all other objects.
        if inside_xy and surface_penetration >= -0.001:
            direction = np.array([0., 0., -1.]) if self.bore_axis is None else self.bore_axis
            reference = np.array([*tip[:2], self.surface_z]) if self.entry is None else self.entry
            along = float(np.dot(tip-reference, direction))
            radial = tip-reference-direction*along
            wall_error = float(np.linalg.norm(radial))
            in_bore = self.entry is not None and wall_error < BIT_RADIUS+WALL_CLEARANCE
            if in_bore and surface_penetration <= .001:
                self.engaged=True
            # A bit already inside a bore hits its sidewall horizontally.
            # Crossing the radial clearance must not teleport the floor from
            # the bore bottom to the original top surface underneath the bit.
            floor = self.depth if self.engaged else 0.
            compression = max(0., along-floor)
            feed_velocity = float(np.dot(velocity, direction))
            axial_force = float(np.clip(SURFACE_STIFFNESS*compression
                                       + SURFACE_DAMPING*max(feed_velocity, 0.), 0., MAX_FORCE))
            if compression <= 0.:
                axial_force = 0.
            force -= direction*axial_force
            if self.entry is not None and along > 0.:
                excess = max(0., wall_error-WALL_CLEARANCE)
                if excess:
                    normal=radial/max(wall_error,1e-12)
                    wall_force=np.clip(WALL_STIFFNESS*excess+WALL_DAMPING*np.dot(velocity,normal),0.,MAX_FORCE)
                    force -= normal*wall_force
            # A rotating tool can create its first bore anywhere on the wood.
            # No grasp, target, orientation or demo-stage condition enables cutting.
            cutting = self.rpm >= 300. and axial_force > MIN_CUT_FORCE and compression > 0.
            if cutting and (self.entry is None or in_bore):
                if self.entry is None:
                    self.entry = np.array([*tip[:2], self.surface_z])
                    self.bore_axis = axis.copy() if axis[2] < -0.5 else np.array([0., 0., -1.])
                    self.engaged=True
                cut_increment = min(compression, MAX_FEED*dt,
                                    FEED_GAIN*(axial_force-MIN_CUT_FORCE)*dt,
                                    max(0., self.size[2]-self.depth))
                self.depth += cut_increment
                if qualified:
                    self.qualified_depth += cut_increment
                torque -= axis*min(0.12, 0.035*axial_force/12.)
        self.maximum_force = max(self.maximum_force, float(np.linalg.norm(force)))
        self.last = {
            'cut_depth_m': self.depth, 'qualified_depth_m': self.qualified_depth,
            'hole_entry_m': None if self.entry is None else self.entry.tolist(),
            'bore_axis': None if self.bore_axis is None else self.bore_axis.tolist(),
            'spindle_rpm': self.rpm, 'feed_force_n': axial_force,
            'wall_error_m': wall_error, 'cut_increment_m': cut_increment,
            'material_force_n': force.tolist(), 'material_torque_nm': torque.tolist(),
            'maximum_material_force_n': self.maximum_force,
        }
        return force, torque
