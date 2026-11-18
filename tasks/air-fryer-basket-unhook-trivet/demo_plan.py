"""Demo plan for the air fryer fry-basket task.

The right hand stands in for the thumb: it pushes the Sliding Button Guard forward along the top of the handle's arm,
then presses the Basket Release Button (the thumb pad of the release lever) and holds it while the left hand, closed on
the handle's vertical grip, lifts the fry basket out of the outer basket. The left hand then sets the fry basket on the
mat. Every move into contact hovers clear first, then goes straight down.
"""

from __future__ import annotations

import os

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint

from geometry import (
    BASKET_START_POS,
    GRIP_CENTRE,
    GUARD_TAB_BACK_X,
    GUARD_TRAVEL_M,
    L_READY,
    LEVER_AT,
    MAT_CENTRE,
    PAD_CENTRE,
    PAN_START_POS,
    R_READY,
    TABLE_TOP,
)

# yam_arena: yaw 0 closes the fingers along world y. Finger commands open NEGATIVE (FINGER_OPEN = -0.04695 m per finger).
# Measured from the URDF finger meshes: the fingertip is 12.8 mm below the tool point, and across the closing axis a
# closed finger is 1.5 cm wide within 1 cm of its tip, 2.8 to 4 cm wide from 1 to 6 cm up, and 6.8 cm wide at its root
# (7 cm above the tip).
TIP_BELOW_TCP = 0.0128
GRIP_OPEN = -0.020     # left hand: fingers 2 cm out each side, around the 2.4 cm wide grip and arm
LET_GO_OPEN = -0.012   # to let go of the grip

BX, BY, BZ = BASKET_START_POS
GRIP_X, GRIP_Y = BX + GRIP_CENTRE[0], BY + GRIP_CENTRE[1]          # 0.2075, 0.000
TAB_BACK_X = BX + GUARD_TAB_BACK_X                                 # guard tab back face, 0.273
PAD_X = BX + PAD_CENTRE[0] + 0.004                                 # press 3.4 cm inboard of the pivot, for leverage (measured on the GPU: at 2.6 cm the stalled hand turned it only 0.14 rad)
PAD_TOP_Z = BZ + PAD_CENTRE[2] + 0.003                             # pad top at rest (T + 12.9 cm)
ARM_TOP_Z = BZ + 0.090                                             # handle arm top (T + 11.1 cm)

GRIP_Z = 0.082          # tool point above the table on the grip: fingertips at 6.9 cm (grip spans 1.1 to 9.1 cm, arm 9.1 to 11.1 cm; measured on the GPU: commanded lower, the hand stops at 8.4 cm)
HOVER_GRIP_Z = 0.140    # fingertips 1.6 cm above the arm top, then straight down either side of grip and arm
LIFT_Z = 0.190          # the fry basket rises 10.8 cm; it hangs tipped back (measured on the GPU: 18 deg), so its back corner needs the height
SET_Z = 0.079           # fry basket floor 3 mm above the mat top when let go
PUSH_TIP_Z = PAD_TOP_Z + 0.003 - TABLE_TOP      # pushing fingertip 3 mm above the pad top, on the guard tab (tab z 12.1 to 14.2 cm)
PRESS_OVER = 0.032      # pad stroke is 9.5 mm to its stop; commanded 32 mm (measured on the GPU: the right hand, 30 cm out from its base, stalls about 2.4 cm short; at 40 mm it held the lever on its stop and pinned the basket down)
LEFT_DX, LEFT_DY = 0.0, 0.0     # GPU-measured landing offsets
LIFT_DX = 0.025         # the lifting hand drifts 1 to 2.5 cm toward the robot as it rises (measured on the GPU), dragging the fry basket into the
                        # outer basket's front wall: aim 25 mm the other way so it rises straight, a little away from that wall
RIGHT_DX, RIGHT_DY = 0.0, 0.0


def plans() -> dict[str, list[Waypoint]]:
    """Return waypoint lists for left and right arms."""
    top = TABLE_TOP
    if os.getenv("POSE_CHECK"):
        return pose_check_plans()
    if os.getenv("SETTLE_ONLY"):
        return {
            "left": [Waypoint(8.0, L_READY, FINGER_OPEN, "settle", 0.0)],
            "right": [Waypoint(8.0, R_READY, FINGER_OPEN, "settle", 0.0)],
        }
    mode = os.getenv("ABLATION", "")

    gx, gy = GRIP_X + LEFT_DX, GRIP_Y + LEFT_DY
    mat_gy = MAT_CENTRE[1] + gy - BY                 # fry basket centre lands on the mat centre in y
    mat_gx = MAT_CENTRE[0] + GRIP_CENTRE[0] + LEFT_DX
    left = [
        Waypoint(3.0, L_READY, FINGER_OPEN, "left waits while the guard is pushed", 0.0),
        Waypoint(5.0, L_READY, FINGER_OPEN, "left waits", 0.0),
        Waypoint(3.0, (gx, gy, top + HOVER_GRIP_Z), GRIP_OPEN, "over the grip", 0.0),
        Waypoint(2.0, (gx, gy, top + HOVER_GRIP_Z), GRIP_OPEN, "settle over the grip", 0.0),
        Waypoint(2.0, (gx, gy, top + GRIP_Z), GRIP_OPEN, "down either side of the grip", 0.0),
        Waypoint(1.0, (gx, gy, top + GRIP_Z), FINGER_CLOSED, "close on the grip", 0.0),
        Waypoint(3.0, (gx, gy, top + GRIP_Z), FINGER_CLOSED, "hold while the release is pressed", 0.0),
        Waypoint(2.5, (gx + LIFT_DX, gy, top + GRIP_Z + 0.060), FINGER_CLOSED, "lift while the button is held", 0.0),   # aimed 6 cm up: the hand lags well behind a lifting target (measured on the GPU)
        Waypoint(1.5, (gx + LIFT_DX, gy, top + GRIP_Z + 0.060), FINGER_CLOSED, "hold while the right hand lets go", 0.0),
        Waypoint(2.5, (gx + LIFT_DX, gy, top + LIFT_Z), FINGER_CLOSED, "lift out of the outer basket", 0.0),
        Waypoint(1.5, (gx + LIFT_DX, gy, top + LIFT_Z), FINGER_CLOSED, "hold above the outer basket", 0.0),
        Waypoint(4.0, (mat_gx, mat_gy, top + LIFT_Z), FINGER_CLOSED, "carry over the mat", 0.0),
        Waypoint(2.5, (mat_gx, mat_gy, top + SET_Z), FINGER_CLOSED, "lower onto the mat", 0.0),
        Waypoint(1.0, (mat_gx - 0.010, mat_gy, top + SET_Z), LET_GO_OPEN, "let go", 0.0),
        Waypoint(2.0, (mat_gx - 0.020, mat_gy, top + 0.150), LET_GO_OPEN, "lift above the grip", 0.0),
        Waypoint(2.0, L_READY, FINGER_OPEN, "withdraw left", 0.0),
        Waypoint(1.5, L_READY, FINGER_OPEN, "stillness wait", 0.0),
    ]
    total = sum(w.duration for w in left)

    push_x0 = TAB_BACK_X - 0.014 - 0.004 + RIGHT_DX               # fingertip face (1.4 cm wide at the tab's height) 4 mm behind the tab
    push_x1 = push_x0 + GUARD_TRAVEL_M + 0.004                    # guard to its stop
    py = BY + RIGHT_DY
    px = PAD_X + RIGHT_DX
    press_z = PAD_TOP_Z - TABLE_TOP + TIP_BELOW_TCP               # fingertip on the pad top
    right = []
    if mode != "idle":
        right += [
            # Closed fingers over the arm behind the guard tab, down, then push the guard forward to its stop
            Waypoint(3.0, (push_x0, py, top + 0.175), FINGER_CLOSED, "over the guard tab", 0.0),
            Waypoint(1.5, (push_x0, py, top + 0.175), FINGER_CLOSED, "settle over the tab", 0.0),
            Waypoint(1.5, (push_x0, py, top + PUSH_TIP_Z + TIP_BELOW_TCP), FINGER_CLOSED, "down behind the tab", 0.0),
            Waypoint(2.0, (push_x1, py, top + PUSH_TIP_Z + TIP_BELOW_TCP), FINGER_CLOSED, "slide the guard forward", 0.0),
            Waypoint(1.0, (px, py, top + PUSH_TIP_Z + TIP_BELOW_TCP + 0.005), FINGER_CLOSED, "back over the open pad", 0.0),
        ] if mode != "noguard" else [
            Waypoint(3.0, (push_x0, py, top + 0.175), FINGER_CLOSED, "guard left closed (ablation)", 0.0),
            Waypoint(6.0, (push_x0, py, top + 0.175), FINGER_CLOSED, "guard left closed (ablation)", 0.0),
        ]
        right += [
            Waypoint(2.0, (px, py, top + 0.155), FINGER_CLOSED, "over the release button", 0.0),
            Waypoint(2.0, (px, py, top + press_z + 0.010), FINGER_CLOSED, "down to the button", 0.0),
            # Wait until the left hand has closed on the grip (about 16 s), then press and hold
            Waypoint(2.0, (px, py, top + press_z + 0.010), FINGER_CLOSED, "wait for the grip", 0.0),
            Waypoint(1.5, (px, py, top + press_z - PRESS_OVER), FINGER_CLOSED, "press the release button", 0.0),
            Waypoint(2.5, (px, py, top + press_z - PRESS_OVER), FINGER_CLOSED, "hold it pressed", 0.0),
            # Rise with the fry basket for its first 25 mm, still pressing, then let go upward and withdraw
            Waypoint(2.5, (px, py, top + press_z - PRESS_OVER + 0.040), FINGER_CLOSED, "rise with the basket", 0.0),
            Waypoint(1.5, (px, py - 0.100, top + press_z - PRESS_OVER + 0.040), FINGER_CLOSED, "let go of the button sideways", 0.0),
            Waypoint(2.5, R_READY, FINGER_OPEN, "withdraw right", 0.0),
        ]
    r_time = sum(w.duration for w in right)
    if total > r_time:
        right.append(Waypoint(total - r_time, R_READY, FINGER_OPEN, "right at ready", 0.0))
    return {"left": left, "right": right}


def pose_check_plans() -> dict[str, list[Waypoint]]:
    """Check run (run.sh posecheck) for the gripper quaternion order and for both contact checks with turned wrists.

    Both wrists turn through yaw 0, 45 degrees and about 90 degrees in free space, with the fingers open and closed. The
    turned left hand (yaw -90 degrees) then hovers with its fingertips 1.6 cm above the handle's arm (not released) and
    leaves (released). Last, the turned right hand (yaw 80 degrees) lowers its fingertips onto the top of the outer
    basket's hot right wall (hot contact, a permanent zero). The scorer logs the simulator's fingertip link positions at
    every step.
    """
    top = TABLE_TOP
    yaw_l, yaw_r = -1.5708, 1.40          # the wrist limits (see the yam_arena notes): left -pi/2, right 1.40 rad
    free_l, free_r = (0.200, 0.200, top + 0.150), (0.200, -0.220, top + 0.150)
    wall = (PAN_START_POS[0] - 0.050, PAN_START_POS[1] - 0.109)   # top of the outer basket's hot right wall (z 7.6 cm)
    left = [
        Waypoint(3.0, free_l, FINGER_OPEN, "free yaw 0 open", 0.0),
        Waypoint(2.5, free_l, FINGER_OPEN, "free yaw -45 open", -0.7854),
        Waypoint(2.5, free_l, FINGER_CLOSED, "free yaw -45 closed", -0.7854),
        Waypoint(2.5, free_l, FINGER_CLOSED, "free yaw -90 closed", yaw_l),
        Waypoint(2.5, free_l, GRIP_OPEN, "free yaw -90 part open", yaw_l),
        Waypoint(3.0, (GRIP_X, GRIP_Y, top + HOVER_GRIP_Z), GRIP_OPEN, "turned hand over the handle", yaw_l),
        Waypoint(2.0, (GRIP_X, GRIP_Y, top + HOVER_GRIP_Z), GRIP_OPEN, "hold over the handle", yaw_l),
        Waypoint(3.0, L_READY, FINGER_OPEN, "turned hand leaves", yaw_l),
        Waypoint(9.0, L_READY, FINGER_OPEN, "left wait", yaw_l),
    ]
    right = [
        Waypoint(3.0, free_r, FINGER_OPEN, "free yaw 0 open", 0.0),
        Waypoint(2.5, free_r, FINGER_OPEN, "free yaw 45 open", 0.7854),
        Waypoint(2.5, free_r, FINGER_CLOSED, "free yaw 45 closed", 0.7854),
        Waypoint(2.5, free_r, FINGER_CLOSED, "free yaw 80 closed", yaw_r),
        Waypoint(2.5, free_r, GRIP_OPEN, "free yaw 80 part open", yaw_r),
        Waypoint(8.0, free_r, FINGER_CLOSED, "wait for left hand", yaw_r),
        Waypoint(3.0, (wall[0], wall[1], top + 0.150), FINGER_CLOSED, "turned hand over hot wall", yaw_r),
        Waypoint(2.5, (wall[0], wall[1], top + 0.084), FINGER_CLOSED, "fingertips onto hot wall", yaw_r),
        Waypoint(1.5, (wall[0], wall[1], top + 0.084), FINGER_CLOSED, "touch hot wall", yaw_r),
        Waypoint(2.5, R_READY, FINGER_OPEN, "withdraw right", yaw_r),
    ]
    return {"left": left, "right": right}
