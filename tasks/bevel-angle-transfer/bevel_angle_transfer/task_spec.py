# SPDX-License-Identifier: MIT
"""Copy a physical reference angle with a passive hinged bevel gauge."""
from pathlib import Path
import math
from yam_arena.task_file import Box
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED

SCENE = "station"
TABLE_TOP = .7475
BOARD_CENTER = (.29, 0., .7555)
BOARD_SIZE = (.14, .18, .016)
BOARD_TOP = .7635
REFERENCE_ANGLE_DEG = -20.
REFERENCE_ANGLE = math.radians(REFERENCE_ANGLE_DEG)
GAUGE_INITIAL_ANGLE = math.radians(10.)
GAUGE_INITIAL_POS = (.18, .28, .7655)
GAUGE_INITIAL_YAW = 0.
GAUGE_PIVOT_LOCAL = (0., -.1075, .001)
GAUGE_CAPTURE_POS = (.208, .2875, .7655)
REFERENCE_PIVOT = (.208, .18, .7665)
BLADE_LOCAL_EDGE = ((0., -.01, -.003), (.17, -.01, -.003))
BLADE_GRASP_LOCAL = (.115, .015, .015)
STOCK_INNER_X = .012
GAUGE_STATIC_FRICTION = .01
GAUGE_DYNAMIC_FRICTION = .008
GAUGE_VISCOUS_FRICTION = .001
GAUGE_DAMPER_PER_DEGREE = .010
GAUGE_DAMPER_MAX_TORQUE = .10
TARGET_START = (.22, -.03)
TARGET_END = (.36, -.03 + .14 * math.tan(REFERENCE_ANGLE))
# The marking path sits 2.5 mm outside the blade's low-y working edge.
GAUGE_TRANSFER_POS = (.208, TARGET_START[1] - (.22-.208)*math.tan(REFERENCE_ANGLE) + .0125/math.cos(REFERENCE_ANGLE) + .1075, .7655)
MARKER_NIB_LOCAL = (0., 0., -.05)
MARKER_NIB_RADIUS = .002
MARKER_GRASP_LOCAL = (0., 0., .023)
INK_WIDTH = .002
VISUAL_ONLY = ("reference_tick",)
SCORE_OVER = "end"
ASSETS = Path(__file__).parent / "assets"
COMPOUND_ASSETS = {name: str(ASSETS/(name+".usda")) for name in ("board", "marker")}
GAUGE_ASSET = str(ASSETS/"gauge.usda")
REF_BLADE_CENTER = (REFERENCE_PIVOT[0]+.10*math.cos(REFERENCE_ANGLE)+.02*math.sin(REFERENCE_ANGLE), REFERENCE_PIVOT[1]+.10*math.sin(REFERENCE_ANGLE)-.02*math.cos(REFERENCE_ANGLE), .7600)
INSTRUCTION = ("Fit the blue-stock bevel gauge to the two grey reference faces by adjusting its hinged blade. "
               "Lift and carry the gauge to the wooden board without changing the captured angle. "
               "Seat its stock against the board's near edge and use the orange marker along the blade "
               "to draw one straight line through the red tick across the board. Avoid extra marks, "
               "then lift the marker clear. The gauge has a passive friction hinge; there is no locking screw.")
OBJECTS = [
    Box("board", pos=BOARD_CENTER, size=BOARD_SIZE, color=(.72,.52,.30)),
    Box("marker", pos=(.18,-.19,.8175), size=(.022,.022,.08), mass=.04),
    Box("reference_stock", pos=(.23,.365,.7600), size=(.02,.05,.025), color=(.45,.47,.50)),
    Box("reference_blade", pos=REF_BLADE_CENTER, size=(.14,.02,.025), yaw=REFERENCE_ANGLE, color=(.45,.47,.50)),
    Box("reference_tick", pos=(.222,-.03,BOARD_TOP+.00015), size=(.004,.002,.0002), color=(.9,.05,.03)),
    Box("holder_base", pos=(.18,-.19,TABLE_TOP+.009), size=(.04,.04,.018), color=(.15,.18,.22)),
    Box("holder_front", pos=(.175,-.19,TABLE_TOP+.0255), size=(.004,.014,.015), color=(.15,.18,.22)),
    Box("holder_back", pos=(.185,-.19,TABLE_TOP+.0255), size=(.004,.014,.015), color=(.15,.18,.22)),
    Box("holder_left", pos=(.18,-.185,TABLE_TOP+.0255), size=(.006,.004,.015), color=(.15,.18,.22)),
    Box("holder_right", pos=(.18,-.195,TABLE_TOP+.0255), size=(.006,.004,.015), color=(.15,.18,.22)),
]

def success(objects):
    return False

def score(objects):
    return 0.

def pickup_waypoints():
    O,C = FINGER_OPEN,FINGER_CLOSED
    yaw=-math.pi/2
    return {
        "left":[Waypoint(1.5,(.18,.27,.84),O,"stock hover",yaw),Waypoint(1.5,(.18,.27,.7825),O,"stock grasp",yaw),Waypoint(.8,(.18,.27,.7825),C,"grip stock",yaw),Waypoint(1.2,(.18,.27,.84),C,"lift stock",yaw)],
        "right":[Waypoint(1.5,(.18,-.19,.875),O,"marker hover"),Waypoint(1.5,(.18,-.19,.8405),O,"marker grasp"),Waypoint(.8,(.18,-.19,.8405),C,"grip marker"),Waypoint(1.3,(.18,-.19,.875),C,"lift marker")],
    }
