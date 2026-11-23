"""SI fixture frame. +X is glove withdrawal, +Y is the left handle."""

from math import cos, hypot, pi, sin, sqrt

ORIGIN = (0.285, 0.0, 0.7475)
MASS = 3.0
LENGTH, WIDTH, SHEET = 0.220, 0.390, 0.0034
HANDLE_Y, HANDLE_HEIGHT = 0.135, 0.041
PICK_X, LAND_X = -0.220, 0.0
COVER_Z, REST_Z = 0.092, 0.069
CARRIAGE = (0.150, 0, 0.077)
TIP_X = CARRIAGE[0] - 0.080
PAD_Z = 0.081
SLOPE = 0.0
INSPECTION_END_S = 8.5
WITHDRAWAL_DURATION_S = 8.0


def drive_travel(time_s, inspection_end=INSPECTION_END_S):
    """Encoder target on the independent mannequin axis, using a smooth 200 mm stroke."""
    u = max(0.0, min(1.0, (time_s - inspection_end) / WITHDRAWAL_DURATION_S))
    return WITHDRAWAL * u * u * (3.0 - 2.0 * u)


PIN_X = 0.040
PIN_YS = (-0.061, 0.061)
PIN_RADIUS, BORE_RADIUS, BOSS_HEIGHT = 0.006, 0.009, 0.010
CHAMFER = 0.003
PIN_SHANK, PIN_HEIGHT = 0.009, 0.015
SIDES_COUNT = 16


def pin_profile(index):
    """The second locating pin is relieved along the line joining the pin centers."""
    circle = [
        (
            PIN_RADIUS * cos(2 * pi * i / SIDES_COUNT),
            PIN_RADIUS * sin(2 * pi * i / SIDES_COUNT),
        )
        for i in range(SIDES_COUNT)
    ]
    if index == 0:
        return circle
    return [(x, max(-0.0027, min(0.0027, y))) for x, y in circle]


PAD_MASS = 0.045
K = 2500.0
BODIES = ("frame", "cover", "carriage", "padLeft", "padRight")
PADS = ("padLeft", "padRight")
WORKER_BODIES = ("worker", "upperArm", "forearmLink")
BODIES += (*WORKER_BODIES, "stageDrive")
WITHDRAWAL = 0.200
SIDES = ("left", "right")
PROMPT = (
    "Install the changeover plate on the inspection fixture and leave both arms clear."
)
# These conditions share policies. Inspection timing is a separate environment input.
TIMING = {
    "early_clear_dock": ("dock_while_occupied", "early"),
    "inspection_continues_dock": ("wait_then_dock", "continuing"),
    "inspection_continues_hold": ("hover_only", "continuing"),
}
INSPECTION_PROFILES = {"nominal": INSPECTION_END_S, "early": 0.0, "continuing": 31.0}
MODES = (
    "reference",
    "slow_reference",
    "park_closed",
    "hover_only",
    "misaligned_hold",
    "idle",
    *TIMING,
)


# The folding linkage starts in the vertical X-Z plane. Each body frame has identity rotation at reset.
SHOULDER = (0.600, 0.0, 0.450)
CUFF_SOCKET = (0.079, 0.0, -0.003)
GAUGE_TIP = (-0.110, -0.048, -0.007)


def elbow_at_reset():
    dx = CARRIAGE[0] + CUFF_SOCKET[0] - SHOULDER[0]
    dz = CARRIAGE[2] + CUFF_SOCKET[2] - SHOULDER[2]
    reach = hypot(dx, dz)
    along = (0.310**2 - 0.290**2 + reach**2) / (2 * reach)
    offset = sqrt(0.310**2 - along**2)
    return (
        SHOULDER[0] + (along * dx - offset * dz) / reach,
        0.0,
        SHOULDER[2] + (along * dz + offset * dx) / reach,
    )


ELBOW = elbow_at_reset()
UPPER_END = tuple(e - s for e, s in zip(ELBOW, SHOULDER))
LOWER_END = tuple(c + w - e for c, w, e in zip(CARRIAGE, CUFF_SOCKET, ELBOW))
