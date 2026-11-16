// Shared dimensions for the printed carrier, sprung rail, guide-rod mounts and SSD dummy (all in mm, grams, N/mm).
//
// Every line is either
//   NAME = value; // task.py: <expression>   a number taken from task.py; check_params.py evaluates <expression>
//                                            against the imported task module (box(BOXES, i) gives a box's
//                                            [x0, x1, y0, y1, z0, z1] in mm) and asserts it equals value
//   NAME = value; // fab: <reason>           a fabrication choice (fit, hardware size); check_params.py checks
//                                            that these are consistent with the task numbers
// Keep one assignment per line and use only numbers, + - * /, names and [ ] lists: check_params.py parses this file.
//
// Frames, as in task.py:
//   carrier: origin on the table under the carrier's centre; x along the carrier, handle bezel toward -x, the drive's
//            connector end (rear) at +x; y across, the sprung rail at +y, the fixed rail at -y; z up.
//   SSD:     origin under the drive's centre on its bottom face; x along it, connector end toward +x; z up.

// ---- the SSD (task.py "the SSD, in its own frame") -----------------------------------------------------------------
SSD_L = 100; // task.py: SSD_L * 1000
SSD_W = 69.85; // task.py: SSD_W * 1000
SSD_H = 7; // task.py: SSD_H * 1000
SSD_MASS_G = 55; // task.py: SSD_MASS * 1000
HALF_W = 34.925; // task.py: HALF_W
HOLE_X = [-36.0, 40.6]; // task.py: HOLE_X
HOLE_Z = 3; // task.py: HOLE_Z
HOLE = 4; // task.py: HOLE
HOLE_DEPTH = 3; // task.py: HOLE_DEPTH
CONN_X = [44, 50]; // task.py: box(SSD_BOXES, 1)[0:2]
CONN_Y = [-31.925, 0]; // task.py: box(SSD_BOXES, 1)[2:4]
LABEL_X = [-42, 30]; // task.py: box(SSD_BOXES, 3)[0:2]
LABEL_Y = [-26, 26]; // task.py: box(SSD_BOXES, 3)[2:4]

// ---- the carrier (task.py "the carrier, in its own frame") ---------------------------------------------------------
CAR_L = 140; // task.py: CAR_L * 1000
CAR_W = 76; // task.py: CAR_W * 1000
CAR_H = 15; // task.py: CAR_H * 1000
CAR_MASS_G = 200; // task.py: CAR_MASS * 1000
SEAT_X = 20; // task.py: SX
SEAT_Z = 4; // task.py: SZ
GAP = 0.5; // task.py: GAP
PIN = 2.5; // task.py: PIN
PIN_IN = 2; // task.py: PIN_IN
RAIL_T = 3; // task.py: RAIL_T
RAIL_IN = 35.425; // task.py: RAIL_IN
RAIL_OUT = 38.425; // task.py: RAIL_OUT
PIN_X = [-16.0, 60.6]; // task.py: PIN_X
PIN_Z = 7; // task.py: PIN_Z
RAIL_SEGS = [[-61, -4], [48, 70]]; // task.py: RAIL_SEGS
RIB_X = [[-25, -10], [50, 65]]; // task.py: RIB_X
RIB_Y_END = 33; // task.py: box(CARRIER_BOXES, 3)[3]
BEZEL_X = [-70, -62]; // task.py: box(CARRIER_BOXES, 0)[0:2]
BEZEL_Y = 38; // task.py: box(CARRIER_BOXES, 0)[3]
HANDLE_X = [-74, -70]; // task.py: box(CARRIER_BOXES, 1)[0:2]
HANDLE_Y = 25; // task.py: box(CARRIER_BOXES, 1)[3]
HANDLE_Z = [3, 12]; // task.py: box(CARRIER_BOXES, 1)[4:6]
FLOOR_X = [-62, -34]; // task.py: box(CARRIER_BOXES, 2)[0:2]
FLOOR_Y = 33; // task.py: box(CARRIER_BOXES, 2)[3]
FLOOR_H = 3; // task.py: box(CARRIER_BOXES, 2)[5]
FIXED_BAR_X = [-70, 70]; // task.py: box(CARRIER_BOXES, 5)[0:2]
FIXED_BAR_H = 3; // task.py: box(CARRIER_BOXES, 5)[5]
FIXED_SEG0_X0 = -62; // task.py: box(CARRIER_BOXES, 6)[0]
RAIL_H = 12; // task.py: box(CARRIER_BOXES, 6)[5]

// ---- the sprung rail (task.py _latch_rail) -------------------------------------------------------------------------
RAIL_Z0 = 1; // task.py: box(rail.boxes, 0)[4]
RAIL_BAR_X = [-61, 70]; // task.py: box(rail.boxes, 2)[0:2]
RAIL_BAR_H = 3; // task.py: box(rail.boxes, 2)[5]
RAIL_TRAVEL = 7; // task.py: RAIL_TRAVEL * 1000
RAIL_RATE = 0.15; // task.py: RAIL_RATE / 1000
RAIL_MASS_G = 30; // task.py: rail.mass * 1000
TAB_X = [-61, -49]; // task.py: TAB_X
TAB_Y = [46.425, 48.925]; // task.py: TAB_Y
TAB_Z = [8, 34]; // task.py: TAB_Z
BRACKET_Z = [8, 12]; // task.py: box(rail.boxes, 3)[4:6]

// ---- print fits (fabrication choices) ------------------------------------------------------------------------------
SLIDE_CLR = 0.3; // fab: diametral clearance of a sliding fit (rod in its mount bore), and the rail's floor clearance
PRESS_CLR = 0; // fab: printed at nominal size for a press fit; horizontal printed holes come out slightly small
POCKET_CLR = 0.3; // fab: clearance round ballast plates and bars in their pockets
$fn = 48; // fab: facets on round holes

// ---- hardware (fabrication choices) --------------------------------------------------------------------------------
PIN_D = 2.5; // fab: 2.5 mm steel dowel pin, as wide as the task's 2.5 mm square pins (0.75 mm clear in a 4 mm hole)
PIN_L = 6; // fab: dowel pin length; PIN_IN + GAP stands proud of the rail, the rest is pressed in
PIN_BOSS = PIN_L - PIN_IN - GAP - RAIL_T; // fab: 0.5 mm pad on the rail's outer face so the pin sits flush there
ROD_D = 3; // fab: 3 mm ground steel rod (silver steel / drill rod / 3 mm linear shaft)
ROD_X = [-28, 52]; // fab: the two guide rods, one in each rail segment, clear of the pins and the pull tab
ROD_Z = 6.5; // fab: rod axis height, mid-height of the rail
BOSS_W = 8; // fab: width (along x) of the rod boss on the sprung rail's outer face
BOSS_L = 6; // fab: how far the rod boss stands out from the rail's outer face
ROD_EMBED = 8; // fab: depth of the blind press-fit bore for the rod in the rail and its boss
MOUNT_W = 12; // fab: guide-rod mount width along x
MOUNT_T = 10; // fab: guide-rod mount length along the rod (the sliding bore's length)
MOUNT_H = 12; // fab: guide-rod mount height
MOUNT_Y0 = RAIL_OUT + BOSS_L + RAIL_TRAVEL; // fab: the boss meets the mount's inner face at full travel (hard stop)
MOUNT_Y1 = MOUNT_Y0 + MOUNT_T; // fab: the mount's outer face
ROD_PROUD = 1; // fab: the rod stands 1 mm out of the mount's outer face with the rail closed
ROD_L = ROD_EMBED + RAIL_TRAVEL + MOUNT_T + ROD_PROUD; // fab: cut length of each guide rod (26 mm)
TONGUE_W = 20; // fab: width of the base web that carries each mount
N_SPRINGS = 2; // fab: one compression spring over each guide rod
SPRING_WIRE = 0.3; // fab: music (spring steel) wire diameter
SPRING_OD = 5; // fab: spring outside diameter (inside 4.4 mm, clear of the 3 mm rod)
SPRING_ACTIVE = 10; // fab: active coils; closed and ground ends add one dead coil at each end
SPRING_POCKET = 6; // fab: depth of the spring pocket in the mount; the spring's length at full travel
SPRING_PRELOAD = 0.5; // fab: compression already in each spring with the rail closed
SPRING_FREE = SPRING_POCKET + RAIL_TRAVEL + SPRING_PRELOAD; // fab: free length (13.5 mm)
SPRING_RATE = RAIL_RATE / N_SPRINGS; // fab: rate of each spring, 0.075 N/mm, so the pair matches the task's 150 N/m
SPRING_POCKET_D = SPRING_OD + 1; // fab: spring pocket diameter

// ---- the real fixture's floor and ballast (fabrication choices) ----------------------------------------------------
BASE_T = RAIL_Z0 - SLIDE_CLR; // fab: 0.7 mm base web under the whole carrier and the sprung rail, below the rail
STOP_H = 3; // fab: rib extensions up to y = RAIL_IN are the sprung rail's closed stop; they stay below the drive (4)
SLAB_X = [-10, 50]; // fab: ballast slab between the ribs, under the drive
SLAB_Y = 30; // fab: slab half-width (the fingers' room beside the drive is left open)
SLAB_H = 3.5; // fab: slab top, 0.5 mm below the seated drive's bottom face
SLAB_POCKET = [55, 55, 3]; // fab: steel ballast plate, 55 x 55 x 3 mm, set in from the top
BEZEL_POCKET = [6, 70, 12]; // fab: steel flat bar 6 x 12 mm, 70 mm long, set in the bezel from below
FLOOR_POCKET = [24, 60, 2]; // fab: steel plate 24 x 60 x 2 mm, set in the front floor plate from below
