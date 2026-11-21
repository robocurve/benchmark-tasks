"""Microwave start only after removing a metal fork left in the dish; SI units."""
CONTROL_HZ = 20
PHYSICS_HZ = 240
DWELL_STEPS = 10
FINGER_FORCE_LIMIT = 40.
PAD_TORSIONAL_RADIUS = .002

TABLE_Z = .500
# Countertop microwave mock (no powered magnetron): cavity + hinged door + start button.
# Front/door/button face -X toward the arms so a gripper can reach the cavity.
# Body sits farther +X so the apron grasp stays in clear arm workspace (like the gate latch).
MW_ORIGIN = (.560, .0, TABLE_Z)
MW_OUTER = (.360, .300, .240)
CAVITY = (.240, .220, .160)
DOOR_THICK = .018
DOOR_OPEN_YAW = -1.4  # rad about +Z; open swings the free edge toward -Y
# Dish with metal fork standing in food stand-in.
DISH_OD = .090
DISH_H = .025
DISH_MASS = .100
# Dish on the apron in clear space in front of the open door.
_FRONT_X = MW_ORIGIN[0] - MW_OUTER[0] / 2
# Raised apron stand: YAM tracking stalls ~7 cm above bare table height (z≈0.52).
# YAM right-arm TCP floor in this station is ~0.61–0.66 m; put the utensil there.
STAND_H = .200
STAND_TOP = TABLE_Z + STAND_H
DISH_START = (.330, -.250, STAND_TOP + .010)
# Tall utensil handle so the pinch zone sits in the reachable TCP band (~0.74 m).
FORK_SIZE = (.060, .060, .060)
FORK_MASS = .150
FORK_START = (DISH_START[0], DISH_START[1], STAND_TOP + FORK_SIZE[2] / 2 + .002)
# Apron in front of the door counts as the metal-in-oven zone for scoring.
CAVITY_APRON = .120
# Safe parking spot for the fork on the counter (must leave cavity before start).
FORK_SAFE = (.250, -.350, STAND_TOP + FORK_SIZE[2] / 2 + .002)
# Must match scene.py: microwave root at (ox, oy, oz+oz_sz/2) + joint LocalPos0.
# LocalPos0 = (-ox_sz/2+0.02, -0.050, -oz_sz/2+0.12) → world ≈ (0.40, -0.05, 0.62).
BUTTON_POS = (
    MW_ORIGIN[0] - MW_OUTER[0] / 2 + .02,
    MW_ORIGIN[1] - .050,
    MW_ORIGIN[2] + MW_OUTER[2] / 2 + (-MW_OUTER[2] / 2 + .12),
)
BUTTON_TRAVEL = .004

WORKSPACE = ((-.15, -.65, .490), (1.05, .65, 1.35))


def finger_joint_position(aperture):
    if not 0 <= aperture <= .075:
        raise ValueError('Aperture exceeds 75 mm')
    return -min(aperture, .0749) / 2
