"""Fixed layout, SI units; rotations read from native state are XYZW."""
import numpy as np

CONTROL_HZ = 20
PHYSICS_HZ = 240
MAX_STEPS = 2400
TABLE_TOP = .7475
FRAME_POSITION = np.array([.32, -.03, .88])
CARTRIDGE_OFFSET = np.array([0, .112, 0])
OLD_POSITION = FRAME_POSITION + CARTRIDGE_OFFSET
NEW_POSITION = np.array([.18, .40, TABLE_TOP + .025])
TRAY_POSITION = np.array([.43, .40, TABLE_TOP])
ROD_GRIP_OFFSET = np.array([0, -.070, 0])
HANDLE_GRIP_OFFSET = np.array([0, -.025, -.085])
# Command enough travel to reach the real bulkhead stop despite arm/grasp
# compliance. Native piston/bulkhead contact still sets the actual endpoint.
STROKE = .195
INSTRUCTION = (
    "Replace the used orange cartridge with the prepared blue cartridge. "
    "Leave the used cartridge in the collection tray and the dispenser resting in its cradle, "
    "with the replacement seated and the piston rod re-engaged at its rear. "
    "The rod handle may remain extended behind the dispenser. Release both hands."
)
