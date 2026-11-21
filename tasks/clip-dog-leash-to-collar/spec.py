"""Clip a dog-leash bolt-snap onto a collar D-ring; SI units."""
CONTROL_HZ = 20
PHYSICS_HZ = 240
DWELL_STEPS = 10
FINGER_FORCE_LIMIT = 40.
PAD_TORSIONAL_RADIUS = .002

TABLE_Z = .500
STAND_H = .120
STAND_TOP = TABLE_Z + STAND_H
# Collar segment with D-ring fixed on a stand (pet not modelled).
# Collar on the midline so the right arm can seat the snap without crossing far.
COLLAR_START = (.400, .000, STAND_TOP + .080)
DRING_LOCAL = (.000, .000, .035)
DRING_INNER = .018  # inner radius of D-ring
# Single rigid snap body (no child-only handle collider). Raised into the
# ~0.74 m TCP band on a pedestal; body is thick enough in Y for a pad pinch.
SNAP_PEDESTAL_H = .220
SNAP_PEDESTAL_SIZE = (.120, .100, SNAP_PEDESTAL_H)
SNAP_SIZE = (.080, .050, .045)
SNAP_MASS = .060
SNAP_START = (
    .400,
    -.220,
    TABLE_Z + SNAP_PEDESTAL_H + SNAP_SIZE[2] / 2 + .001,
)
GATE_TRAVEL = .016
GATE_OPEN_MIN = .010
HOOK_XY = .025
HOOK_Z = .020



WORKSPACE = ((-.15, -.65, .490), (1.05, .65, 1.35))


def finger_joint_position(aperture):
    if not 0 <= aperture <= .075:
        raise ValueError('Aperture exceeds 75 mm')
    return -min(aperture, .0749) / 2
