// params.scad -- shared dimensions for every printed part of the PSU hot-swap build (mm; spring rates in N/mm;
// masses in g). Included by each part file; nothing here makes geometry by itself.
//
// Frames are the ones task.py uses, in mm:
//   SUPPLY frame: origin on the table under the supply's centre; +x toward its back (connector end), the face and
//                 handle toward -x; +y to the supply's left (the hook side); +z up.
//   CAGE frame:   origin on the table where a latched supply's origin sits in the LEFT bay (task.CAGE_XY); +x into
//                 the bays, +y left. The right bay is centred on y = -BAY_PITCH. A supply seated in the left bay has
//                 its supply frame equal to the cage frame.
//
// Tags (read by check_params.py):
//   "// task: <expr> | <note>"  the value comes from the simulation. <expr> is evaluated with task.py imported as T
//                               (helpers: psu_box(i), latch_box(i), cage_box(i), yawbox(spec), mm(v); see
//                               check_params.py) and must equal the value here.
//   "// fab: <note>"            a fabrication choice that is not in the simulation: print clearances, wall and floor
//                               thicknesses, fasteners, springs, and the internal mechanism layout.
//
// PRINT-FIT CLEARANCES (explicit): every sliding fit has FIT_SLIDE = 0.3 mm per side (tab link through the face,
// latch carriage in its guide, hook through the side window, contact carriers in their pockets, blade through the
// back stop, plug tongue in its socket); located fits (handle spine in its face recess, link in the carriage socket)
// have FIT_PRESS = 0.15 mm per side. The outer faces of every part are at the simulation's size, with no clearance
// added, so the bays keep their 2 mm per side and 1 mm under the roof.

// ===================== simulation dimensions: the supply (SUPPLY frame) =====================
PSU_L = 185;                         // task: T.PSU_L * 1000 | CRPS length
PSU_W = 73.5;                        // task: T.PSU_W * 1000 | CRPS width
PSU_H = 39.5;                        // task: T.PSU_H * 1000 | CRPS height
PSU_MASS_G = 900;                    // task: T.PSU_MASS * 1000 | each supply's mass, g (ballast target)
FACE_X = -92.5;                      // task: T.FACE_X * 1000 | the supply's face
BODY_X1 = 72;                        // task: psu_box(0)[1] | _psu_boxes: solid body up to the connector slot
SLOT_Y = [-8, 8];                    // task: [psu_box(1)[3], psu_box(2)[2]] | _psu_boxes: connector slot, across
SLOT_Z1 = 30;                        // task: psu_box(3)[4] | _psu_boxes: connector slot top (open below and behind)
FACEPLATE = [-94.5, -92.5, -36, 36, 1, 38.5];     // task: psu_box(4) | _psu_boxes: face plate [x0,x1,y0,y1,z0,z1]
HANDLE_BAR = [-132, -122, -8, 0, 2, 50];          // task: psu_box(5) | _psu_boxes: fixed handle grip bar
HANDLE_ARM_LO = [-122, -94.5, -8, 0, 2, 7];       // task: psu_box(6) | _psu_boxes: handle's lower arm
HANDLE_ARM_HI = [-122, -94.5, -8, 0, 45, 50];     // task: psu_box(7) | _psu_boxes: handle's upper arm
LED = [-95.5, -94.5, -30, -20, 28, 34];           // task: psu_box(8) | _psu_boxes: status light (raised pad + sticker)
INLET = [-95, -94.5, -35, -14, 4, 22];            // task: psu_box(9) | _psu_boxes: AC inlet (C14) face
PLUG = [-132, -94.5, -33, -15, 5, 21];            // task: psu_box(10) | _psu_boxes(plug=True): live supply's C13 plug
HANDLE_Y = [-8, 0];                  // task: mm(T.HANDLE_Y) | HANDLE_Y
TAB_Y = [6, 14];                     // task: mm(T.TAB_Y) | TAB_Y: release tab at rest
TAB = [-132, -122, 6, 14, 2, 50];    // task: latch_box(0) | _latch: release tab
LINK = [-122, -95.5, 6, 14, 2, 7];   // task: latch_box(1) | _latch: link from the tab toward the face
CATCH = [-80, -76, 36.75, 41.75, 14, 26];         // task: latch_box(2) | _latch: hook's square catch
RAMP_TOP = [-76, 42.75];             // task: list(T.RAMP_TOP) | RAMP_TOP: ramp crest (x, y)
RAMP_DEG = 27;                       // task: T.RAMP_DEG | RAMP_DEG: ramp angle to the side
RAMP_T = 3;                          // task: yawbox(latch_spec(3))['t'] | _hook_ramp: plate thickness
RAMP_Z = [14, 26];                   // task: yawbox(latch_spec(3))['z'] | _hook_ramp: plate bottom and top
LATCH_TRAVEL = 5;                    // task: T.LATCH_TRAVEL * 1000 | LATCH_TRAVEL: tab travel to its stop
LATCH_RATE = 1;                      // task: T.LATCH_RATE / 1000 | LATCH_RATE: tab spring, N/mm
LATCH_MASS_G = 60;                   // task: T._latch().mass * 1000 | _latch: tab + arm + hook mass, g
CONTACT_X = [89, 92];                // task: list(T.CONTACT_X) | CONTACT_X: contact tip along the supply
CONTACT_Y = [3, 7];                  // task: list(T.CONTACT_Y) | CONTACT_Y: tip inner/outer face at rest (left)
CONTACT_Z = [6, 26];                 // task: list(T.CONTACT_Z) | CONTACT_Z
CONTACT_RATE = 5;                    // task: T.CONTACT_RATE / 1000 | CONTACT_RATE: each contact's spring, N/mm
CONTACT_TRAVEL = 4.5;                // task: T.CONTACT_TRAVEL * 1000 | CONTACT_TRAVEL: max spread of a contact

// ===================== simulation dimensions: the cage (CAGE frame) =====================
BAY_W = 77.5;                        // task: T.BAY_W * 1000 | BAY_W: bay width
BAY_PITCH = 92.5;                    // task: T.BAY_PITCH * 1000 | BAY_PITCH: bay centre to centre
MOUTH_X = -90.5;                     // task: T.MOUTH_X * 1000 | MOUTH_X: bay mouth
BACK_X = 93.5;                       // task: T.BACK_X * 1000 | BACK_X: back stop face
NOTCH_X = [-81.5, -64];              // task: mm(T.NOTCH_X) | NOTCH_X: hook notch along the wall
NOTCH_Z = [13, 27];                  // task: mm(T.NOTCH_Z) | NOTCH_Z: hook notch height
WALL_X = [-90.5, 94];                // task: [cage_box(0)[0], cage_box(1)[1]] | _notched_wall: wall front and rear
WALL_Y = [38.75, 44.75];             // task: cage_box(0)[2:4] | _notched_wall: left outer wall
WALL_H = 40.5;                       // task: cage_box(0)[5] | _notched_wall: wall height (= roof underside)
DIVIDER_Y = [-53.75, -38.75];        // task: cage_box(4)[2:4] | _notched_wall: 15 mm divider (notched for the live hook)
RIGHT_WALL = [-90.5, 93.5, -137.25, -131.25, 0, 40.5];  // task: cage_box(8) | CAGE_BOXES: right outer wall
ROOF = [-90.5, 99, -137.25, 44.75, 40.5, 44.5];         // task: cage_box(9) | CAGE_BOXES: roof
BACKSTOP = [93.5, 99, -137.25, 44.75, 0, 40.5];         // task: cage_box(10) | CAGE_BOXES: back stop
SERVER = [99, 230, -137.25, 44.75, 0, 44.5];            // task: cage_box(11) | CAGE_BOXES: server body behind the cage
LEADIN_L = 15;                       // task: yawbox(cage_spec(12))['l'] | _lead_in: plate length
LEADIN_T = 2;                        // task: yawbox(cage_spec(12))['t'] | _lead_in: plate thickness
LEADIN_H = 40;                       // task: yawbox(cage_spec(12))['h'] | _lead_in: plate height (printed taller, see LEADIN_PRINT_H)
LEADIN_DEG = 30;                     // task: abs(yawbox(cage_spec(12))['deg']) | _lead_in: flare angle
BLADE_X = [78, 93.5];                // task: list(T.BLADE_X) | BLADE_X: blade core from its tip to the back stop
BLADE_Z = [6, 26];                   // task: list(T.BLADE_Z) | BLADE_Z
BLADE_T = 3;                         // task: cage_box(16)[3] - cage_box(16)[2] | _blade: core thickness
RIDGE_APEX = [85.6, 5];              // task: list(T.RIDGE_APEX) | RIDGE_APEX: ridge crest (x, y off centreline)
RIDGE_DEG = 50;                      // task: T.RIDGE_DEG | RIDGE_DEG: front face angle (back face is 90 - this = 40)
RIDGE_SIDES = [4.6, 5.5];            // task: list(T.RIDGE_SIDES) | RIDGE_SIDES: ridge block sides (front, back face)

// ===================== simulation layout (world frame from the arm-base midpoint; for marking the board) ==========
CAGE_XY = [460, 0];                  // task: mm(T.CAGE_XY) | CAGE_XY: left bay's seated supply origin
SPARE_XY = [235, 220];               // task: mm(T.SPARE_XY) | SPARE_XY: spare supply's origin, face toward the arms

// ===================== fabrication: general =====================
$fn = 48;                            // fab: circle resolution
EPS = 0.01;                          // fab: overlap for clean boolean cuts
BIG = 1000;                          // fab: half-space size used to split the cage
BED = 220;                           // fab: smallest print bed (square) every part is laid out to fit
FIT_SLIDE = 0.3;                     // fab: clearance per side on sliding fits (see header)
FIT_PRESS = 0.15;                    // fab: clearance per side on located fits (see header)
WALL = 2;                            // fab: shell side and front wall thickness
FLOOR = 2;                           // fab: shell floor thickness; equal to LINK[4], so the tab link rides on the floor
LID_T = 2.5;                         // fab: shell lid thickness; the lid's top is at PSU_H

// ===================== fabrication: fasteners =====================
M3_CLEAR = 3.4;                      // fab: M3 clearance hole
M3_PILOT = 2.5;                      // fab: pilot hole an M3 screw cuts its own thread into (plastic)
M3_HEAD_D = 6.5;                     // fab: counterbore for an M3 socket or pan head (5.5 mm head)
M3_HEAD_H = 3;                       // fab: M3 socket-head height
M3_CSK_D = 6.4;                      // fab: countersink diameter for an M3 countersunk (flat) head, 90 degrees
M3_CSK_H = (M3_CSK_D - M3_CLEAR) / 2;                   // fab: countersink depth
M3_NUT_AF = 5.7;                     // fab: M3 nut trap across flats (5.5 nut)
NUT_FN = 6;                          // fab: hexagon
INSERT_D = 4.2;                      // fab: hole for an M3 heat-set insert (use your insert's recommended hole)
INSERT_L = 6;                        // fab: insert hole depth (for an insert up to 5.7 mm long)

// ===================== fabrication: cage =====================
LEADIN_PRINT_H = ROOF[5];            // fab: lead-ins printed full height (to the roof top) so they print on the bed upside down
REAR_RAIL_D = 10;                    // fab: printed stub of SERVER behind the back stop (full height); holds the blades
SPLIT_Y = (DIVIDER_Y[0] + DIVIDER_Y[1]) / 2;            // fab: the cage splits here into two halves for small beds
BLADE_FLANGE_T = 4;                  // fab: blade flange thickness (behind the rail)
BLADE_FLANGE_W = 24;                 // fab: blade flange width
BLADE_SCREW_DY = 9;                  // fab: blade flange screws at bay centre +- this
BOARD_INSERT_RAIL_DY = 30;           // fab: board-mounting inserts under the rail at bay centre +- this
BOARD_INSERT_DIV_X = [-40, 50];      // fab: board-mounting inserts under the divider at these x
DIV_INSERT_DY = (DIVIDER_Y[1] - DIVIDER_Y[0]) / 4;      // fab: ... at SPLIT_Y +- this (one in each half)
JOIN_X = [-30, 60];                  // fab: bolts through the divider joining the halves, at these x
JOIN_Z = 20;                         // fab: ... at this height
JOIN_CBORE = 4;                      // fab: head counterbore depth (left bay face)
JOIN_NUT_DEPTH = 3.5;                // fab: nut trap depth (right bay face)

// ===================== fabrication: supply shell (tray + contact housing + lid) =====================
HOUSING_X0 = 70;                     // fab: shell splits here: tray in front, contact housing behind (2 mm ahead of BODY_X1)
TRAY_REAR_T = 8;                     // fab: tray rear wall, carries the joint screws and two lid bosses
JOINT_Y = 20;                        // fab: tray-to-housing screws at y +- this ...
JOINT_Z = 18;                        // fab: ... and this height
JOINT_CBORE = 4;                     // fab: head counterbore in the tray rear wall
LID_SCREWS_TRAY = [-50, 66];         // fab: lid bosses in the tray at these x ...
LID_SCREW_X_HOUSING = 81;            // fab: ... and lid inserts in the housing at this x
LID_SCREW_Y = 31;                    // fab: ... all at y +- this
BOSS_D = 8;                          // fab: lid boss diameter
BALLAST_HALF_W = 26;                 // fab: ballast bay between two ribs at y +- this (takes 50 mm flat bar)
BALLAST_RIB_H = 15;                  // fab: rib height above the floor
RIB_T = 2;                           // fab: rib thickness (ballast ribs, latch guide rib)
HANDLE_SCREW_Z = [20, 32];           // fab: handle spine screws through the face, at these heights

// ===================== fabrication: latch (tab + link | carriage | hook) =====================
LINK_SOCKET_D = 6.5;                 // fab: the link reaches this far behind the front wall into the carriage
CARR_Y0 = 2;                         // fab: carriage's -y end at rest (seat of the tab spring)
CARR_X1 = -60;                       // fab: carriage and hook post rear end
CARR_TOP = 12;                       // fab: carriage top (the hook part sits on it)
POST_T = 7;                          // fab: hook post thickness (y), inboard of the side wall
POST_FRONT = 6;                      // fab: hook post runs this far in front of the catch
HOOK_SCREW_X = [-83, -63];           // fab: two M3 screws down through the hook post into the carriage
HOOK_SCREW_DEPTH = 6;                // fab: their pilot depth in the carriage
SPRING_BLOCK_T = 11;                 // fab: fixed spring block (y); its +y face is the push stop
SPRING_BLOCK_HALF = 8;               // fab: spring block half-length (x)
TAB_SPRING_POS = [-78, 8.5];         // fab: tab spring axis (x, z)
TAB_SPRING_OD = 5;                   // fab: tab spring max outer diameter
TAB_SPRING_FREE = 15;                // fab: tab spring free length
TAB_SPRING_PRELOAD = 0.5;            // fab: tab spring compression at rest (holds the hook on its stop)
TAB_SPRING_SOLID = 6;                // fab: tab spring max solid length
TAB_POCKET_D = TAB_SPRING_FREE - TAB_SPRING_PRELOAD - LATCH_TRAVEL;   // fab: spring pocket depth in the block

// ===================== fabrication: connector (contact carriers + blade) =====================
CARRIER_X0 = 80;                     // fab: front of each contact carrier's body (tip is CONTACT_X)
CARRIER_Y1 = 13;                     // fab: outer face of each carrier's body at rest (body spans CONTACT_Y[1]..this)
CARRIER_SCREW_X = 85;                // fab: guide screw / spring axis along the supply
CARRIER_SCREW_DEPTH = 5.5;           // fab: guide-screw pilot depth into the carrier
STOP_WALL_T = 3;                     // fab: wall between the spring and the guide-screw head
CONTACT_SPRING_OD = 6;               // fab: contact spring max outer diameter (inner diameter >= 3.5 to pass an M3 screw)
CONTACT_SPRING_INSTALLED = 13;       // fab: contact spring length at rest (carrier face to stop wall)
CONTACT_SPRING_FREE = 13.3;          // fab: contact spring nominal free length (0.3 mm preload)
CONTACT_SPRING_SOLID = 8;            // fab: contact spring max solid length

// ===================== fabrication: inlet and dummy plug =====================
INLET_BOSS_D = 4;                    // fab: boss behind the front wall that takes the plug's tongue
PLUG_TONGUE_INSET = 3;               // fab: tongue is the plug's section inset by this on each side
CORD_D = 7;                          // fab: hole in the plug's front for a short cut cord stub (optional)
CORD_DEPTH = 15;                     // fab: ... and its depth

// ===================== shared helpers =====================
module box6(b) translate([b[0], b[2], b[4]]) cube([b[1] - b[0], b[3] - b[2], b[5] - b[4]]);
function ramp_tail_x() = RAMP_TOP[0] + (RAMP_TOP[1] - PSU_W / 2) / tan(RAMP_DEG);
function link_end_x() = FACE_X + WALL + LINK_SOCKET_D;
function link_screw_x() = FACE_X + WALL + LINK_SOCKET_D / 2;
function tongue_x1() = FACE_X + WALL + INLET_BOSS_D;
function plug_screw_x() = FACE_X + WALL + INLET_BOSS_D / 2;
function plug_screw_y() = (PLUG[2] + PLUG[3]) / 2;
