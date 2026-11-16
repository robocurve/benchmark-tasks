// psu_shell.scad -- the supply's shell: a CRPS-size box PSU_L x PSU_W x PSU_H (task.py: _psu_boxes), SUPPLY frame, mm.
//
// Three printed sections, each smaller than a BED x BED bed, screwed together into one rigid body:
// PART = "tray"     front section, x FACE_X .. HOUSING_X0: floor, walls, face plate (with the handle recess, the tab
//                   link's slot, the status-light pad and the inlet), the hook window in the left side, the latch's
//                   guide channel (front wall + guide rib) and fixed spring block (tab spring
//                   pocket and push stop), the ballast bay (between two ribs, for 50 mm steel flat bar), four lid
//                   bosses and the rear wall with the two joint screws. Print floor down; no supports.
//        "housing"  back section, x HOUSING_X0 .. PSU_L/2: the connector slot (task.py's 16 x 30 mm slot, open below
//                   and behind) with a pocket on each side for a contact carrier, the carrier's spring bore, a stop wall
//                   and a counterbore for the M3 guide screw's head, two lid inserts and two joint inserts.
//                   Print standing on its back face; no supports.
//        "lid"      the top, PSU_L x PSU_W x LID_T, countersunk for six M3 flat-head screws. Print flat.
//        "assembly" all three in place.
// Outer faces are task.py's sizes; nothing is added outside them except the screw heads, which are countersunk or
// sit in counterbores.

include <params.scad>
use <plug_inlet.scad>

PART = "tray";

TOP = PSU_H - LID_T;                    // the tray's and housing's top (the lid sits on it)
HW = PSU_W / 2;

// The latch's fixed parts: the spring block (tab spring pocket; its +y face is the push stop) and the guide rib
// that, with the front wall, forms the channel the carriage slides across in. The carriage is held down by the link
// in its face slot and by the hook's catch in the side window.
module latch_guides() {
    stop_y = CARR_Y0 - LATCH_TRAVEL;
    y_start = stop_y - SPRING_BLOCK_T;
    difference() {
        translate([TAB_SPRING_POS[0] - SPRING_BLOCK_HALF, y_start, 0])
            cube([2 * SPRING_BLOCK_HALF, SPRING_BLOCK_T, CARR_TOP]);
        translate([TAB_SPRING_POS[0], stop_y + EPS, TAB_SPRING_POS[1]]) rotate([90, 0, 0])
            cylinder(h = TAB_POCKET_D + EPS, d = TAB_SPRING_OD + 2 * FIT_SLIDE);
    }
    translate([CARR_X1 + FIT_SLIDE, y_start, 0]) cube([RIB_T, HW - WALL - y_start + EPS, CARR_TOP]);
}

module ballast_ribs() {
    x0 = CARR_X1 + FIT_SLIDE + RIB_T;
    x1 = HOUSING_X0 - TRAY_REAR_T;
    for (s = [1, -1])
        translate([x0, s > 0 ? BALLAST_HALF_W : -BALLAST_HALF_W - RIB_T, 0])
            cube([x1 - x0 + EPS, RIB_T, FLOOR + BALLAST_RIB_H]);
}

module tray() {
    difference() {
        union() {
            difference() {
                translate([FACE_X, -HW, 0]) cube([HOUSING_X0 - FACE_X, PSU_W, TOP]);
                translate([FACE_X + WALL, -HW + WALL, FLOOR])
                    cube([HOUSING_X0 - TRAY_REAR_T - FACE_X - WALL, PSU_W - 2 * WALL, TOP]);
            }
            box6(FACEPLATE);
            box6(LED);
            inlet_add();
            latch_guides();
            ballast_ribs();
            for (x = LID_SCREWS_TRAY)
                for (s = [1, -1])
                    translate([x, s * LID_SCREW_Y, 0]) cylinder(h = TOP, d = BOSS_D);
        }
        // recess in the face plate for the handle's spine
        translate([FACEPLATE[0] - EPS, HANDLE_BAR[2] - FIT_PRESS, FACEPLATE[4] - EPS])
            cube([FACEPLATE[1] - FACEPLATE[0] + EPS, HANDLE_BAR[3] - HANDLE_BAR[2] + 2 * FIT_PRESS,
                  FACEPLATE[5] - FACEPLATE[4] + 2 * EPS]);
        // handle screws through the front wall
        for (z = HANDLE_SCREW_Z)
            translate([FACEPLATE[0] - EPS, (HANDLE_BAR[2] + HANDLE_BAR[3]) / 2, z]) rotate([0, 90, 0])
                cylinder(h = FACEPLATE[1] - FACEPLATE[0] + WALL + 2 * EPS, d = M3_CLEAR);
        // slot for the tab's link, long enough for its travel
        translate([FACEPLATE[0] - EPS, LINK[2] - LATCH_TRAVEL - FIT_SLIDE, FLOOR])
            cube([FACEPLATE[1] - FACEPLATE[0] + WALL + 2 * EPS, LINK[3] - LINK[2] + LATCH_TRAVEL + 2 * FIT_SLIDE,
                  LINK[5] + FIT_SLIDE - FLOOR]);
        // window in the left side for the hook (catch and ramp)
        translate([CATCH[0] - FIT_SLIDE, HW - WALL - EPS, CATCH[4] - FIT_SLIDE])
            cube([ramp_tail_x() - CATCH[0] + 2 * FIT_SLIDE, WALL + 2 * EPS, CATCH[5] - CATCH[4] + 2 * FIT_SLIDE]);
        inlet_cut();
        // lid inserts
        for (x = LID_SCREWS_TRAY)
            for (s = [1, -1])
                translate([x, s * LID_SCREW_Y, TOP - INSERT_L]) cylinder(h = INSERT_L + EPS, d = INSERT_D);
        // joint screws through the rear wall into the housing, heads counterbored
        for (s = [1, -1]) {
            translate([HOUSING_X0 - TRAY_REAR_T - EPS, s * JOINT_Y, JOINT_Z]) rotate([0, 90, 0])
                cylinder(h = TRAY_REAR_T + 2 * EPS, d = M3_CLEAR);
            translate([HOUSING_X0 - TRAY_REAR_T - EPS, s * JOINT_Y, JOINT_Z]) rotate([0, 90, 0])
                cylinder(h = JOINT_CBORE + EPS, d = M3_HEAD_D);
        }
    }
}

// The left contact carrier's seat (+y side); mirrored for the right.
module carrier_seat() {
    pocket_y1 = CARRIER_Y1 + CONTACT_TRAVEL + FIT_SLIDE;
    seat_y = CARRIER_Y1 + CONTACT_SPRING_INSTALLED;
    zc = (CONTACT_Z[0] + CONTACT_Z[1]) / 2;
    // the pocket the carrier's body slides in: open to the slot and at the back
    translate([CARRIER_X0 - FIT_SLIDE, SLOT_Y[1] - EPS, CONTACT_Z[0] - FIT_SLIDE])
        cube([PSU_L / 2 - CARRIER_X0 + FIT_SLIDE + EPS, pocket_y1 - SLOT_Y[1] + EPS,
              CONTACT_Z[1] - CONTACT_Z[0] + 2 * FIT_SLIDE]);
    // the spring's bore
    translate([CARRIER_SCREW_X, pocket_y1 - EPS, zc]) rotate([-90, 0, 0])
        cylinder(h = seat_y - pocket_y1 + EPS, d = CONTACT_SPRING_OD + 2 * FIT_SLIDE);
    // the guide screw through the stop wall
    translate([CARRIER_SCREW_X, seat_y - EPS, zc]) rotate([-90, 0, 0])
        cylinder(h = STOP_WALL_T + 2 * EPS, d = M3_CLEAR);
    // counterbore for the screw head, open on the supply's side; the head moves out in it as the contact spreads
    translate([CARRIER_SCREW_X, seat_y + STOP_WALL_T, zc]) rotate([-90, 0, 0])
        cylinder(h = HW - seat_y - STOP_WALL_T + EPS, d = M3_HEAD_D);
}

module housing() {
    difference() {
        translate([HOUSING_X0, -HW, 0]) cube([PSU_L / 2 - HOUSING_X0, PSU_W, TOP]);
        // the connector slot (task.py), open below and behind
        translate([BODY_X1, SLOT_Y[0], -EPS]) cube([PSU_L / 2 - BODY_X1 + EPS, SLOT_Y[1] - SLOT_Y[0], SLOT_Z1 + EPS]);
        carrier_seat();
        mirror([0, 1, 0]) carrier_seat();
        for (s = [1, -1]) {
            // lid inserts
            translate([LID_SCREW_X_HOUSING, s * LID_SCREW_Y, TOP - INSERT_L]) cylinder(h = INSERT_L + EPS, d = INSERT_D);
            // joint inserts in the front face
            translate([HOUSING_X0 - EPS, s * JOINT_Y, JOINT_Z]) rotate([0, 90, 0]) cylinder(h = INSERT_L + EPS, d = INSERT_D);
        }
    }
}

module lid() {
    difference() {
        translate([FACE_X, -HW, TOP]) cube([PSU_L, PSU_W, LID_T]);
        for (x = concat(LID_SCREWS_TRAY, [LID_SCREW_X_HOUSING]))
            for (s = [1, -1]) {
                translate([x, s * LID_SCREW_Y, TOP - EPS]) cylinder(h = LID_T + 2 * EPS, d = M3_CLEAR);
                translate([x, s * LID_SCREW_Y, PSU_H - M3_CSK_H]) cylinder(h = M3_CSK_H + EPS, d1 = M3_CLEAR, d2 = M3_CSK_D);
            }
    }
}

if (PART == "tray") tray();
else if (PART == "housing") translate([0, 0, PSU_L / 2]) rotate([0, 90, 0]) housing();
else if (PART == "lid") translate([0, 0, -TOP]) lid();
else if (PART == "assembly") {
    tray();
    housing();
    lid();
}
