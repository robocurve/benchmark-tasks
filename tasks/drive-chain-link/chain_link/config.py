"""Fixed bench-scale training geometry, SI units and XYZW rotations."""

import numpy as np

CONTROL_HZ, PHYSICS_HZ, MAX_STEPS = 20, 240, 2400
TABLE_TOP = 0.7475
DECK_TOP = 0.837
PITCH = 0.100
CHAIN_POSITION = np.array([0.3816025403784439, 0.0, DECK_TOP + 0.010])
CONNECTOR_POSITION = np.array([0.195, 0.0, 0.839])
CONNECTOR_PICKUP_POSITION = np.array([0.35, -0.35, TABLE_TOP + 0.002])
CONNECTOR_GRASP = np.array([0.0, -PITCH / 2, 0.042])
CHAIN_GRASP_HEIGHT = 0.013
KEEPER_POSITION = np.array([0.18, 0.35, TABLE_TOP + 0.004])
RADIUS = PITCH / (2 * np.sin(np.pi / 12))
ANGLES = np.deg2rad(np.arange(195, 526, 30))
NODES = np.c_[RADIUS * np.cos(ANGLES), RADIUS * np.sin(ANGLES), np.zeros(12)]
CLOSED_NODES = NODES.copy()
# Only the two free end links are splayed. Joining swings them around their
# existing hinges instead of pulling the whole loop away from its supports.
for end, inner, y in ((0, 1, -0.080), (-1, -2, 0.080)):
    NODES[end, 1] = y
    NODES[end, 0] = NODES[inner, 0] - np.sqrt(PITCH**2 - (y - NODES[inner, 1]) ** 2)
END_NODES = {"right": NODES[0], "left": NODES[-1]}
END_BODIES = {"right": "Link00", "left": "Link10"}
GRIPS = {"right": (NODES[0] + NODES[1]) / 2, "left": (NODES[-1] + NODES[-2]) / 2}
ROOT_CENTER = (NODES[0] + NODES[1]) / 2
ROOT_YAW = np.arctan2(*(NODES[1] - NODES[0])[1::-1])
ROOT_POSITION = CHAIN_POSITION + ROOT_CENTER
ROOT_QUATERNION = np.array([0.0, 0.0, np.sin(ROOT_YAW / 2), np.cos(ROOT_YAW / 2)])
CHAIN_SHIFT = CONNECTOR_POSITION[0] - (CHAIN_POSITION + NODES[0])[0]
INSTRUCTION = (
    "Join and retain the two loose ends of the chain using the yellow connecting "
    "link and blue retaining plate. Leave the assembled joint in its service "
    "cradle, the chain seated on its guide, and both hands released."
)
