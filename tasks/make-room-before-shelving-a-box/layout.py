"""Every size and position in the scene, in metres, and the scoring constants. Pure Python.

World frame: the station's. x forward, y to the robot's left, z up; the arm mounts at (0, +-0.305, 0.75). The station
table's top is at TABLE_Z, and heights given "above the table" are added to it.
"""

from __future__ import annotations

TABLE_Z = 0.76 + 0.75 + 0.0095 - 0.772  # yam_arena.station.TABLE_TOP: 0.7475
TABLE_X = (-0.19, 0.91)                 # the station table: 1.10 x 1.30 m, centred on (0.36, 0)
TABLE_Y = (-0.65, 0.65)


def H(h: float) -> float:
    """World z of a height above the table."""
    return TABLE_Z + h


# -- the shelf: Regency 14" x 42" x 14" NSF Chrome Dunnage Shelf, 460EC1442D11, on two 2x4 battens on the table -------
# Each pair of feet (left, right) stands on a 2x4 laid flat, from the shelf's front edge to the table's back edge, the
# feet strapped to it; the battens are clamped to the table where a clamp reaches (its back edge, its right edge).
BATTEN_H, BATTEN_W = 0.038, 0.089       # a 2x4 laid flat
BATTEN_X = (0.400, 0.910)
BATTEN_Y = ((0.4047 - BATTEN_W / 2, 0.4047 + BATTEN_W / 2), (-0.650, -0.650 + BATTEN_W))   # left, right (flush with
                                                                                         # the table's right edge)
SHELF_X = (0.400, 0.756)                # front edge, back edge (14 in deep)
SHELF_Y = (-0.6461, 0.4207)             # 42 in wide; the right end 4 mm inside the table's edge
SHELF_T = 0.025                         # the shelf as a 25 mm plate
WORK_TOP = 0.330                        # the shelf's top surface, above the table (33 cm above the arm mounts' level)
POST_R = 0.0127                         # 1 in posts
FOOT_H = 0.019                          # the feet add 3/4 in
POST_H = FOOT_H + 0.3556                # 14 in posts on their feet, standing on the battens
UNIT_POSTS = ((0.416, 0.4047), (0.740, 0.4047), (0.416, -0.6301), (0.740, -0.6301))

# -- the cases: Uline S-18385 boxes, outside 11 5/8 x 9 1/8 x 5 5/8 in -----------------------------------------------------
CASE = (0.2953, 0.2318, 0.1429)         # long side along x (front to back)
SOUP_KG = 4.4                           # twelve 10.75 oz cans and the box (estimate)
N_KG = 1.5                              # rice in sealed bags and the box
FRONT_X = 0.420                         # the front-edge line: the starting front faces of C1 to C3, 2 cm behind the
                                        # shelf's front edge
# N's hand holes (ASTM D6804 guide): one in each long side panel, 4 x 1-1/4 in, centred along the panel, its top edge
# 2 in below the box's top edge. Box wall 4 mm.
WALL = 0.004
HOLE_L, HOLE_H = 0.1016, 0.03175
HOLE_TOP = 0.1429 / 2 - 0.0508           # the hole's top edge above the box's centre
HOLE_BOT = HOLE_TOP - HOLE_H
ROW_X = FRONT_X + CASE[0] / 2
ROW = {"C1": 0.196, "C2": -0.196, "C3": -0.488}     # centres in y: C1 +31.2..+8.0, C2 -8.0..-31.2, C3 -37.2..-60.4
N_X = 0.300 + CASE[0] / 2               # N on the table: x 0.300 to 0.595, centred on y 0, its far 19.5 cm under the shelf
SOUPS = ("C1", "C2", "C3")
CASES = (*SOUPS, "N")
# friction (estimates): case on the wire shelf 0.3, case on case 0.5. PhysX averages the two surfaces' values, so cases
# are given 0.5 and the shelf 0.1.
CASE_MU = 0.5
ROOM_MU = 0.1


def start_pos(name: str) -> tuple[float, float, float]:
    """A case's starting centre."""
    if name in ROW:
        return (ROW_X, ROW[name], H(WORK_TOP + CASE[2] / 2))
    return (N_X, 0.0, H(CASE[2] / 2))


# -- scoring ---------------------------------------------------------------------------------------------------------------
BACK_LINE = SHELF_X[1]                  # out of the back: any part of a case past the shelf's back edge
DROP_FALL = 0.03                        # dropped: a fall of 3 cm or more with no gripper touching
ON_SHELF_TOL = 0.01                     # lowest corner within 1 cm of the shelf top
UPRIGHT_DEG = 10.0
ON_SIDE_DEG = 45.0                      # N's top more than 45 degrees from up: on its side or upside down (scores 0)
ROW_DEG = 15.0                          # in the row: long side within 15 degrees of front to back
ROW_BEHIND = 0.02                       # ... and front face no more than 2 cm behind the front-edge line
ROOM = 0.238                            # a clear gap of 23.8 cm (N's width and 6 mm) along the front edge line
CLEAR = 0.03                            # every fingertip 3 cm or more from every case
STILL_V = 0.01                          # m/s
HOLD_STEPS = 20                         # 1 s at 20 Hz
TOUCH = 0.008                           # a fingertip or hand point within 8 mm of a case touches it
