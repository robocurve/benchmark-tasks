"""Fixed scene: a loaded VariDesk Pro Plus 30 sit-stand converter on the calibrated dual-arm YAM station."""

from pathlib import Path

import numpy as np

from . import layout as L

try:                                            # the station numbers come from yam_arena when it is installed
    from yam_arena import station as _station

    TOP = float(_station.TABLE_TOP)
    _BASES = {arm: tuple(float(v) for v in _station.BASES[arm]) for arm in ("left", "right")}
except ImportError:                             # the same numbers, so that the grader's tests need only NumPy
    TOP = .7475
    _BASES = {"left": (0., .305, .75), "right": (0., -.305, .75)}

TASK_ID = "raise-a-loaded-sit-stand-desk-converter-with-both-handles"
SCORER = "desk_converter_score"
USD = Path(__file__).with_name("desk_converter.usda")
CONTROL_HZ = 20
PHYSICS_HZ = 240
MAX_STEPS = 2400                                # 120 s
ARMS = ("left", "right")
BASES = {arm: np.asarray(_BASES[arm]) - (0., 0., TOP) for arm in ARMS}        # tabletop frame: z = 0 on the table

ROOT_NAME = "Converter"                         # articulation root prim (under the fixture's prim)
BODIES = ("base", "lift_arm", "rear_arm", "top", "latch_left", "latch_right")
LOADS = ("monitor", "keyboard")
LIFT, LEVEL, REAR = "lift", "level", "rear"
LATCH = {"left": "latch_left_slide", "right": "latch_right_slide"}
JOINTS = (LIFT, LEVEL, REAR, LATCH["left"], LATCH["right"])

START_THETA = L.START_ANGLE                     # seated in notch 4; every joint reads 0 here; lift joint = theta - this
LOAD_LIFT = .0005                               # loads are spawned this far above the surface they rest on
TOP_START = L.top_origin(START_THETA)
MONITOR_START = TOP_START + (L.MONITOR_AT[0], L.MONITOR_AT[1], LOAD_LIFT)
KEYBOARD_START = TOP_START + (L.KEYBOARD_AT[0], L.KEYBOARD_AT[1], -L.TRAY_DROP + LOAD_LIFT)
LATCH_K = (L.PADDLE_FORCE[1] - L.PADDLE_FORCE[0]) / L.PADDLE_TRAVEL      # N/m
LATCH_PRELOAD = L.PADDLE_FORCE[0] / LATCH_K                              # m below the lower limit (drive target)
LIFT_TARGET = L.SPRING_ZERO - START_THETA                                # rad, lift drive target (joint coordinate)
# Reset check: after the station settles, the top must still be in notch 4 with both locks seated. The window covers
# the whole slot play (the pins may rest on either contact) plus 0.02 settings.
RESET_SETTING_TOL = 2 * L.SEAT_SETTINGS + .02       # settings, around the seated start
RESET_TOP_TOL = 2 * L.SEAT_PLAY * L.ARM_L + .002     # m, around the seated start position
RESET_LATCH_TOL = .002                               # m

INSTRUCTION = (
    "The sit-stand desk converter on the table is locked at its fourth setting with a monitor on its top and a "
    "keyboard on its tray. Raise it to its ninth setting, where the work surface is level with the top of the green "
    "card: take the handle at each side, squeeze both release paddles together, lift the top along its arc to that "
    "setting, release both paddles there so that both locks drop into the same notch, then let go and move both "
    "arms back. Nothing on it may fall."
)


def side_camera():
    """Review camera at the robot's left front, looking across both handles and the top. Not a policy input."""
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera, Lens

    eye = np.array((-.05, .78, TOP + .62))
    look = np.array((.48, 0., TOP + .27))
    back = eye - look
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(16.))
