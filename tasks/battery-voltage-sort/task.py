"""Test identical AA batteries with two loose handheld voltmeter probes and sort them by the reading.

The cells lie horizontally on individual V supports, with their raised positive terminals toward the robot's
left. Red and black probes rest in separate saddles beside the rack; their leads are connected to a fixed,
powered DC voltmeter behind the rack. Its large screen shows the signed voltage only while both bare tips
touch opposite terminals of the same cell. With either tip disconnected the screen shows dashes.

Use the magnitude of the voltage and the cutoff printed on the meter. High readings go in the green
ALIVE bin on the left; lower readings go in the red DEAD bin on the right.
Only a measured battery released fully inside the correct bin earns credit. The ordinary fourteen YAM
joint/gripper targets and three RGB cameras are used; no electrical state is added to observations.
"""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

import spec as S
from integration import Solid, STATE, install

__doc__ += "\n\n" + S.INSTRUCTION

SCENE = "station"
SUCCESS_HOLD_S = 0.05  # Trial already enforces the full one-second final-state hold.
SCORE_OVER = "end"
OBJECTS = [
    Solid(name=name, pos=tuple(position), size=(0.0145, 0.0505, 0.0145), mass=0.023,
          asset_file="aa_cell.usda") for name, position in S.CELL_START.items()
]
OBJECTS.extend([
    Solid(name="red_probe", pos=tuple(S.PROBE_RESTS["red_probe"]), mass=0.028,
          size=(0.024, 0.152, 0.024), asset_file="red_probe.usda", quaternion=(0, 0, 1, 0)),
    Solid(name="black_probe", pos=tuple(S.PROBE_RESTS["black_probe"]), mass=0.028,
          size=(0.024, 0.152, 0.024), asset_file="black_probe.usda"),
])
for _name in ("battery_cradles", "red_probe_rest", "black_probe_rest", "alive_bin", "dead_bin"):
    OBJECTS.append(Solid(name=_name, pos=(0, 0, S.TOP), size=(0.5, 0.8, 0.07), asset_file=_name+".usda"))
OBJECTS.append(Solid(name="voltage_meter", pos=S.METER_POS, size=(0.165, 0.250, 0.05), asset_file="voltage_meter.usda"))
install(OBJECTS)


def score(objects):
    return STATE.trial.score


def success(objects):
    return STATE.trial.success
