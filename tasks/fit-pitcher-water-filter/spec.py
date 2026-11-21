"""Seat a pitcher water-filter cartridge and close the lid; SI units."""
CONTROL_HZ = 20
PHYSICS_HZ = 240
DWELL_STEPS = 10
FINGER_FORCE_LIMIT = 40.
PAD_TORSIONAL_RADIUS = .002

TABLE_Z = .500
# Raise the station into the YAM reachable TCP band without exceeding ~0.80 m.
STAND_H = .120
STAND_TOP = TABLE_Z + STAND_H
# Compact pitcher reservoir (procedural; no branded CAD).
PITCHER_OD = .140
PITCHER_ID = .120
PITCHER_H = .140
PITCHER_MASS = .280
PITCHER_START = (.360, -.050, STAND_TOP + PITCHER_H / 2)
# Cartridge seats in the upper reservoir well.
CART_OD = .050
CART_H = .070
CART_MASS = .100
CART_START = (.360, -.260, STAND_TOP + CART_H / 2 + .002)
CART_SEAT_XY = .045
CART_SEAT_Z = .025
# Well floor (relative to pitcher root at mid-height).
WELL_Z_LOCAL = PITCHER_H / 2 - .040
# Hinged lid on the pitcher mouth.
LID_SIZE = (.130, .130, .012)
LID_MASS = .060
LID_OPEN = 1.2  # rad about +Y, open toward +X / arms
LID_CLOSED_TOL = 0.12  # rad from flat
LID_HINGE_LOCAL = (0., 0., PITCHER_H / 2)

WORKSPACE = ((-.15, -.65, .490), (1.05, .65, 1.35))


def finger_joint_position(aperture):
    if not 0 <= aperture <= .075:
        raise ValueError('Aperture exceeds 75 mm')
    return -min(aperture, .0749) / 2
