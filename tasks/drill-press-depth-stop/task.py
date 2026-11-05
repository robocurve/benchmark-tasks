"""Set the depth of an unpowered drill-press trainer. Blue is the adjustable
stop, orange releases its catch, green marks the 10 mm setting, and yellow
starts a test stroke. Press and release yellow twice with hands off the press.
"""

from dataclasses import dataclass

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP

SCENE = "station"
TOP = TABLE_TOP[SCENE]
X = 0.300
QUILL = (X, -0.085, TOP + 0.152)
STOP = (X, 0.020, TOP + 0.107)
CATCH = (0.354, 0.083, TOP + 0.0855)
BUTTON = (0.248, -0.244, TOP + 0.055)
DATUM_GAP = 0.005
STOP_GAP = 0.007
PITCH = 0.004
SETTING = 0.008
CATCH_THROW = 0.011
INSTRUCTION = (
    "Hold the orange catch open and slide the blue depth stop to the green 10 mm mark. "
    "Release the catch, take both hands off the press, then press and release the yellow "
    "test button twice, waiting for the spindle to return to the top each time."
)

GREY = (0.53, 0.57, 0.61)
DARK = (0.19, 0.22, 0.25)
STEEL = (0.76, 0.79, 0.82)
BLUE = (0.12, 0.44, 0.80)
ORANGE = (0.93, 0.32, 0.07)
GREEN = (0.10, 0.72, 0.30)
YELLOW = (0.99, 0.76, 0.12)
WHITE = (0.92, 0.93, 0.94)


@dataclass(frozen=True)
class Part:
    name: str
    center: tuple
    size: tuple
    color: tuple
    collides: bool = True


@dataclass(frozen=True)
class Guide:
    axis: str
    limits: tuple
    spring: tuple


OBJECTS = []
PARTS = {}
GUIDES = {}


def body(name, pos, parts, mass=None, guide=None):
    # Box is the Arena registry entry. run.py authors these compound colliders.
    OBJECTS.append(Box(name=name, pos=pos, size=parts[0].size, color=parts[0].color, mass=mass))
    PARTS[name] = parts
    if guide is not None:
        GUIDES[name] = guide


body("frame", (X, 0.0, TOP), [
    Part("base", (0.022, -0.015, 0.009), (0.203, 0.285, 0.018), DARK),
    Part("column", (0.075, -0.086, 0.090), (0.036, 0.063, 0.152), GREY),
    Part("head", (0.055, -0.086, 0.154), (0.044, 0.085, 0.035), GREY),
    Part("catch_mount", (0.094, 0.083, 0.047), (0.038, 0.091, 0.060), GREY),
    Part("scale", (0.000, 0.113, 0.069), (0.028, 0.022, 0.100), DARK),
    Part("target_flag", (-0.022, 0.112, 0.099), (0.019, 0.034, 0.003), GREEN, False),
    Part("coupon_left", (-0.010, -0.112, 0.0285), (0.073, 0.035, 0.021), GREY),
    Part("coupon_right", (-0.010, -0.058, 0.0285), (0.073, 0.035, 0.021), GREY),
    *[
        Part(
            f"tick_{index}", (-0.015, 0.112, 0.107 - index * PITCH),
            (0.001, 0.017 if index == 2 else 0.010, 0.0015),
            GREEN if index == 2 else WHITE, False,
        )
        for index in range(5)
    ],
])
body("spindle", QUILL, [
    Part("handle", (0.0, 0.0, 0.0), (0.048, 0.046, 0.012), STEEL),
    Part("shaft", (0.0, 0.0, -0.057), (0.016, 0.016, 0.102), STEEL),
    Part("strike_plate", (0.0, 0.047, -0.034), (0.026, 0.070, 0.008), STEEL),
    Part("tip_mark", (-0.0085, 0.0, -0.102), (0.001, 0.016, 0.002), WHITE, False),
], 0.160, Guide("Z", (-0.025, 0.0), (190.0, 30.0, 18.0)))
body("depth_stop", STOP, [
    Part("carrier", (0.009, 0.009, -0.012), (0.034, 0.034, 0.042), BLUE),
    Part("shoulder", (0.0, -0.020, -0.004), (0.032, 0.060, 0.008), BLUE),
    Part("handle_link", (-0.029, 0.009, -0.012), (0.048, 0.030, 0.012), BLUE),
    Part("handle", (-0.054, -0.036, -0.005), (0.053, 0.066, 0.010), BLUE),
    Part("pointer", (0.0, 0.071, 0.0), (0.009, 0.027, 0.0012), BLUE, False),
    *[
        Part(
            f"tooth_{index}", (0.0315, 0.009, -0.0209 + index * PITCH + 0.00065),
            (0.011, 0.024, 0.0013), STEEL,
        )
        for index in range(5)
    ],
], 0.100, Guide("Z", (-0.017, 0.001), (0.0, 20.0, 8.0)))
body("catch", CATCH, [
    Part("bar", (0.0, 0.006, 0.0), (0.015, 0.128, 0.010), ORANGE),
    Part("pawl", (-0.0175, -0.054, 0.0), (0.014, 0.022, 0.0012), STEEL),
    Part("paddle_post", (0.0, 0.070, 0.010), (0.014, 0.014, 0.027), ORANGE),
    Part("paddle", (0.0, 0.070, 0.028), (0.030, 0.042, 0.018), ORANGE),
], 0.080, Guide("X", (0.0, CATCH_THROW), (330.0, 8.0, 8.0)))
body("test_base", (BUTTON[0], BUTTON[1], TOP + 0.019), [
    Part("pedestal", (0.0, 0.0, 0.0), (0.057, 0.058, 0.038), DARK),
])
body("test_button", BUTTON, [
    Part("cap", (0.0, 0.0, 0.0), (0.049, 0.048, 0.010), YELLOW),
], 0.060, Guide("Z", (-0.010, 0.0), (220.0, 6.0, 6.0)))


def demo():
    """Use normal YAM motion targets for the catch, stop, and proof button."""
    travel_z = TOP + 0.215
    catch_y = CATCH[1] + 0.063
    catch_z = CATCH[2] + 0.029 + PAD_TIP_PAST_TCP
    catch_start = (0.300, catch_y, catch_z)
    catch_open = (0.380, catch_y, catch_z)
    stop_x, stop_y = STOP[0] - 0.054, STOP[1] - 0.039
    stop_touch = STOP[2] + PAD_TIP_PAST_TCP
    stop_set = stop_touch - SETTING + 0.0009
    button_x, button_y, button_z = BUTTON
    button_press = button_z + 0.005 + PAD_TIP_PAST_TCP - 0.017
    left_park = (0.205, 0.270, travel_z)

    def wp(duration, pos, label):
        return Waypoint(duration, pos, FINGER_CLOSED, label)

    left = [
        wp(2.0, (*catch_start[:2], travel_z), "above orange catch"),
        wp(1.5, catch_start, "lower beside catch"),
        wp(1.5, catch_open, "open catch"),
        wp(4.5, catch_open, "hold catch open"),
        wp(0.7, catch_start, "let catch lock"),
        wp(1.1, (*catch_start[:2], travel_z), "lift clear"),
        Waypoint(1.5, left_park, FINGER_OPEN, "park left arm"),
    ]
    right = [
        wp(2.0, (stop_x, stop_y, travel_z), "above blue stop"),
        wp(3.2, (stop_x, stop_y, travel_z), "wait for catch"),
        wp(2.0, (stop_x, stop_y, stop_touch + 0.002), "approach stop"),
        wp(1.3, (stop_x, stop_y, stop_set), "set depth"),
        wp(1.8, (stop_x, stop_y, stop_set), "hold through catch release"),
        wp(1.1, (stop_x, stop_y, travel_z), "lift off stop"),
        wp(1.8, (button_x, button_y, travel_z), "above remote test button"),
    ]
    for label in ("first stroke", "second stroke"):
        right.extend((
            wp(1.5, (button_x, button_y, button_press), label),
            wp(1.2, (button_x, button_y, travel_z), "release button"),
            wp(1.8, (button_x, button_y, travel_z), "wait for return"),
        ))
    return {"left": left, "right": right}
