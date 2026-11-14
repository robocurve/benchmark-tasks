"""Demo plan for air_fryer_basket_release_button_v2.

The right arm presses the small release button on the front of the basket handle and holds it in while the left arm
lifts the basket out of the drawer by the handle's left post; the left arm then sets the basket on the mat.
Every grasp hovers clear of everything until the arm has converged, then goes straight down.
"""

from __future__ import annotations

import os

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint

from geometry import (
    BASKET_HANDLE_BOXES,
    BASKET_START_POS,
    BUTTON_AT,
    BUTTON_BOXES,
    BUTTON_TRAVEL_M,
    L_READY,
    MAT_CENTRE,
    R_READY,
    TABLE_TOP,
)

# yam_arena: yaw 0 closes the fingers along world y. Finger commands open NEGATIVE (FINGER_OPEN = -0.04695 m per finger).
POST_OPEN = -0.020    # left hand: fingers 20 mm out each side along y, around the 14 mm wide post
CARRY_OPEN = -0.012   # to let go of the post: a 24 mm span (measured on the GPU: at 40 mm the outer pad touched the basket's left wall as the hand rose)

BX, BY = BASKET_START_POS[0], BASKET_START_POS[1]
POST_Y = BY + BASKET_HANDLE_BOXES[0][0][1]                 # left post centre (world y)
POST_X = BX + BASKET_HANDLE_BOXES[0][0][0]                 # post mid length (world x)
CAP_FACE_X = BX + BUTTON_AT[0] + BUTTON_BOXES[0][0][0] - BUTTON_BOXES[0][1][0] / 2   # button front face at rest
BUTTON_Y = BY + BUTTON_AT[1] + BUTTON_BOXES[0][0][1]       # button centre (world y)
PAD_HALF_X = 0.006                                         # a closed finger pad is 12 mm deep in x
PUSH_Z = 0.0825       # tool point above the table when pressing: pads centred on the button
GRIP_Z = 0.0825       # tool point above the table on the post: pads centred on the post
LIFT_Z = 0.175        # basket floor rises 8 cm, past the drawer walls
SET_Z = 0.053         # basket floor 3 mm above the mat top when let go (the basket hangs 3.5 cm below the grip)
# GPU-measured landing offsets are added here after the first runs
LEFT_DX, LEFT_DY = 0.0, 0.0   # centred on the post: pads 12 mm clear of the bar and of the hot front wall
RIGHT_DX, RIGHT_DY = 0.0, 0.0


def plans() -> dict[str, list[Waypoint]]:
    """Return waypoint lists for left and right arms."""
    top = TABLE_TOP
    if os.getenv("SETTLE_ONLY"):
        return {
            "left": [Waypoint(8.0, L_READY, FINGER_OPEN, "settle", 0.0)],
            "right": [Waypoint(8.0, R_READY, FINGER_OPEN, "settle", 0.0)],
        }

    gx, gy = POST_X + LEFT_DX, POST_Y + LEFT_DY
    mat_gy = MAT_CENTRE[1] + (POST_Y - BY) + LEFT_DY           # the basket centre lands on the mat centre
    shove = os.getenv("ABLATION") == "shove"
    gx2 = gx + (0.015 if shove else 0.0)                       # after a shove the hand keeps the basket where it was pushed
    left = [
        # Hover over the post until the arm has converged, then straight down and close
        Waypoint(3.0, (gx, gy, top + 0.130), POST_OPEN, "over handle post", 0.0),
        Waypoint(2.5, (gx, gy, top + 0.130), POST_OPEN, "settle over post", 0.0),
        Waypoint(2.0, (gx, gy, top + GRIP_Z), POST_OPEN, "down beside post", 0.0),
        Waypoint(1.0, (gx, gy, top + GRIP_Z), FINGER_CLOSED, "grip post", 0.0),
        # Wait while the right arm presses the release button
        Waypoint(3.0, (gx, gy, top + GRIP_Z), FINGER_CLOSED, "hold while button is pressed", 0.0),
        # Bypass probe (ablation "shove"): the left arm alone shoves the whole basket 15 mm toward the housing, hoping to
        # slide the lugs out from under the ledges, then lifts; the right arm never presses the button
        *([Waypoint(1.5, (gx + 0.015, gy, top + GRIP_Z), FINGER_CLOSED, "shove basket back", 0.0)] if shove else []),
        # Lift: first 25 mm slowly (the right arm rises with the basket), then clear of the drawer walls
        Waypoint(2.5, (gx2, gy, top + GRIP_Z + 0.025), FINGER_CLOSED, "lift 25 mm", 0.0),
        Waypoint(2.5, (gx2, gy, top + LIFT_Z), FINGER_CLOSED, "lift out of drawer", 0.0),
        # Carry to the mat on the left and set down
        Waypoint(4.0, (gx2, mat_gy, top + LIFT_Z), FINGER_CLOSED, "carry over mat", 0.0),
        Waypoint(2.5, (gx2, mat_gy, top + SET_Z), FINGER_CLOSED, "lower onto mat", 0.0),
        Waypoint(1.0, (gx2 - 0.012, mat_gy, top + SET_Z), CARRY_OPEN, "let go", 0.0),
        Waypoint(2.0, (gx2 - 0.020, mat_gy, top + 0.140), CARRY_OPEN, "lift above basket", 0.0),   # back toward the robot while rising: the hand drifts +x as it lifts and must not reach the wall
        Waypoint(2.0, L_READY, FINGER_OPEN, "withdraw left", 0.0),
        Waypoint(1.5, L_READY, FINGER_OPEN, "stillness wait", 0.0),
    ]
    total = sum(w.duration for w in left)

    if os.getenv("ABLATION"):
        # Ablation: the right arm never presses the button, so the catches hold the basket in the drawer (ablation "shove": see above)
        right = [Waypoint(total, R_READY, FINGER_OPEN, "right idle (ablation)", 0.0)]
    else:
        rx = CAP_FACE_X - PAD_HALF_X + RIGHT_DX               # tool point when the pad face touches the button
        ry = BUTTON_Y + RIGHT_DY
        stop_x = rx + BUTTON_TRAVEL_M + 0.006                 # 14 mm stroke, commanded 6 mm past the stop (measured on the GPU: 12 mm shoved the basket back 15 mm)
        right = [
            # Closed fingers (pads side by side across the 25 mm button), hover in front of the button, then down
            Waypoint(3.0, (rx - 0.030, ry, top + 0.130), FINGER_CLOSED, "over button", 0.0),
            Waypoint(2.5, (rx - 0.030, ry, top + 0.130), FINGER_CLOSED, "settle over button", 0.0),
            Waypoint(2.0, (rx - 0.030, ry, top + PUSH_Z), FINGER_CLOSED, "down in front of button", 0.0),
            # Wait until the left hand has closed on the post (about 9 s), then press the button in to its stop
            Waypoint(2.0, (rx - 0.030, ry, top + PUSH_Z), FINGER_CLOSED, "ready to press", 0.0),
            Waypoint(1.5, (stop_x, ry, top + PUSH_Z), FINGER_CLOSED, "press button to stop", 0.0),
            Waypoint(0.5, (stop_x, ry, top + PUSH_Z), FINGER_CLOSED, "hold button pressed", 0.0),
            # Rise with the basket while still pressing, then let go and withdraw
            Waypoint(2.5, (stop_x, ry, top + PUSH_Z + 0.025), FINGER_CLOSED, "rise with basket", 0.0),
            Waypoint(1.0, (rx - 0.030, ry, top + PUSH_Z + 0.025), FINGER_CLOSED, "release button", 0.0),
            Waypoint(2.0, R_READY, FINGER_OPEN, "withdraw right", 0.0),
        ]
        r_time = sum(w.duration for w in right)
        if total > r_time:
            right.append(Waypoint(total - r_time, R_READY, FINGER_OPEN, "right idle at ready", 0.0))

    return {"left": left, "right": right}
