"""Every size and position of the scene (metres, kg), and the scoring constants. Pure Python.

World frame (yam_pair's): origin on the floor under the midpoint between the two stations; x toward robot B, y toward
the rack, z up; the bench top is at TABLE_TOP. Values marked "estimate" are not published by the makers.

Products (research/products.md):
- server: iStarUSA D-118V2-ITX 1U chassis (datasheet: 482.6 x 44.4 x 282.4 mm over its ears; drawing: body 427.5 wide);
- rails: iStarUSA TC-RAIL-20 (20 in, 3-section ball bearing; 22.3-31 in between front and rear posts);
- rack: VEVOR 9U open-frame 4-post rack SPT-504-09 (depth 23-40 in; set to 25 in post to post).
"""

from __future__ import annotations

import math

TABLE_TOP = 0.7475                    # yam_pair / yam_arena station table top
X_STATION = 0.40                      # each station's origin at x -+0.40 (Layout("facing", separation=0.8))

# -- rack --------------------------------------------------------------------------------------------------------------
U = 0.04445
Y_FRONT = 0.380                       # front posts' front faces
POST_TO_POST = 0.635                  # 25 in, front posts' front faces to rear posts' front faces (set with a tape)
Y_REAR = Y_FRONT + POST_TO_POST
RACK_BASE = 0.052                     # estimate: U1's floor above the bench (base frame, casters off)
RACK_U = 9
RACK_TOP = RACK_BASE + RACK_U * U     # top of the posts' mounting area, above the bench
RACK_HEIGHT = 0.560                   # overall
OPENING = 0.450                       # EIA-310 clear width between the posts' flanges
FLANGE_X = (OPENING / 2, 0.255)       # |x| span of a post's front flange (outer edge: 510 mm overall width)
FLANGE_T = 0.002                      # flange thickness along y (estimate)
LEG_DEPTH = 0.040                     # the post's side leg along y (estimate)
LEG_T = 0.002
HOLE_PITCH_X = 0.4651 / 2             # EIA-310 hole centres
SERVER_U = 3                          # the server goes in U3

# -- server (server frame: x across, y from the ears' back face toward the rack, z up from its underside) ------------
BODY_W = 0.4275                       # body width (drawing)
BODY_H = 0.0435                       # body height (estimate; 1U is 44.45, ears 44.4)
DEPTH = 0.2824                        # over the front panel (datasheet)
PANEL_T = 0.020                       # front panel ahead of the ears' back face (estimate)
BODY_Y = (0.0, DEPTH - PANEL_T)       # steel body behind the ears' plane
PANEL_Y = (-PANEL_T, 0.0)
EAR_W = (0.4826 - BODY_W) / 2         # each ear's width beyond the body
EAR_H = 0.0444
EAR_T = 0.003                         # estimate
MEMBER_FROM_FRONT = 0.030             # chassis member starts 30 mm behind the front face
MEMBER_LEN = 0.470                    # estimate
MEMBER_H = 0.018                      # estimate
MEMBER_T = 0.004                      # estimate
MEMBER_Y = (MEMBER_FROM_FRONT - PANEL_T, MEMBER_FROM_FRONT - PANEL_T + MEMBER_LEN)   # (0.010, 0.480)
MEMBER_X = (BODY_W / 2 + 0.00005, BODY_W / 2 + 0.00005 + MEMBER_T)                  # |x| 0.21380 .. 0.21780
MEMBER_ZC = BODY_H / 2                # member centre height in the server frame
TIP_TAPER = 0.012                     # the member's last 12 mm narrowed (stepped lead-in, estimate)
MASS = 6.4                            # stated: 6.4 +- 0.1 kg with the chassis members
MEMBER_MASS = 0.30                    # each (estimate)
COM_Y = ((MASS - 2 * MEMBER_MASS) * (DEPTH / 2 - PANEL_T) + 2 * MEMBER_MASS * sum(MEMBER_Y) / 2) / MASS
COM = (0.0, COM_Y, BODY_H / 2)

# -- rack rails at U3 (outer and centre members as one fixed channel) ------------------------------------------------
Z_SLOT = TABLE_TOP + RACK_BASE + (SERVER_U - 1) * U   # U3's floor
Z_RACKED = Z_SLOT + 0.0005                            # the server's underside when on its rails
RAIL_ZC = Z_RACKED + MEMBER_ZC                        # channel centre height
POCKET_H = 0.019                                      # estimate (0.5 mm play above and below the member)
LIP_T = 0.002
WEB_T = 0.003
LIP_IN = MEMBER_X[0] + 0.001                          # lips reach in to |x| 0.2148
WEB_IN = MEMBER_X[1] + 0.001                          # web inner face at |x| 0.2188
RAIL_Y = (Y_FRONT + FLANGE_T + 0.006, Y_REAR - 0.006)
FLARE_L = 0.015                                       # mouth flares 3 mm over its first 15 mm (estimate)
FLARE = 0.003
RAIL_FRICTION = 0.05                                  # estimate (ball bearings)

# -- stand and mat -----------------------------------------------------------------------------------------------------
MAT = dict(size=(0.600, 0.640, 0.010), center=(0.0, 0.060))
STAND = dict(size=(0.300, 0.250, 0.050), center=(0.0, -0.049))
STAND_TOP = TABLE_TOP + MAT["size"][2] + STAND["size"][2]          # 0.060 above the bench
E_START = -0.070                                      # members' tips 70 mm short of the posts' front faces


def ear_plane_y(e: float) -> float:
    """World y of the ears' back face when the members' tips are ``e`` past the posts' front faces."""
    return Y_FRONT + e - MEMBER_Y[1]


SERVER_START = (0.0, ear_plane_y(E_START), STAND_TOP)                # server-frame origin, world (-0.170 y)

# -- scoring -----------------------------------------------------------------------------------------------------------
ENGAGED_MIN = 0.050           # member tip past the posts' front faces
HOME_GAP = 0.003              # ear back face to post front face
TOUCH = 0.001                 # a gripper link hull within 1 mm of the server (PhysX contact offset)
HOLD_TOUCH = 0.002            # both finger pads within 2 mm, gripper closed: holding
CLOSED_BELOW = 0.6            # gripper opening (0 closed .. 1 open) below this: closed
LIFT_CLEAR = 0.010            # all four body corners this far above the stand top: clear of the stand
DROP_FALL = 0.020             # unsupported fall that counts as a drop
LANDED_Z = TABLE_TOP + 0.045  # any server corner below this (on the mat, the bench or the rack's base): dropped
CLEAR_S = 1.0
CONT = (9.0, 9.0, 9.0, 3.0, 3.0, 3.0)      # DM-J4340 (joints 1-3) and DM-J4310 (4-6) continuous ratings, N m
PEAK = (27.0, 27.0, 27.0, 7.0, 7.0, 7.0)
RATED_KG = 2.0

if __name__ == "__main__":
    print(f"COM_Y={COM_Y:.4f} Z_SLOT={Z_SLOT - TABLE_TOP:.4f} RAIL_ZC={RAIL_ZC - TABLE_TOP:.4f} "
          f"start ear plane y={SERVER_START[1]:.3f} member x={MEMBER_X} lips {LIP_IN:.4f} web {WEB_IN:.4f} "
          f"overhang={MEMBER_Y[1] - BODY_Y[1]:.4f} lift={Z_RACKED - STAND_TOP:.4f}")
