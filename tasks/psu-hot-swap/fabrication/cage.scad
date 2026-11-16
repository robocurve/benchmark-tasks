// cage.scad -- the two-bay power-supply cage (task.py: CAGE_BOXES), CAGE frame, mm.
//
// Notched left wall and divider (hook notches), plain right wall, roof, back stop, 30-degree lead-ins at each mouth,
// a full-height stub of the server body behind the back stop (REAR_RAIL_D) that carries each bay's connector blade
// (blade passage + two flange inserts), M3 heat-set inserts underneath for screwing the cage to its board, and two
// bolts through the divider that join the halves when the cage is printed in two pieces.
//
// PART = "whole"  one piece (needs about 214 x 189 mm of bed)
//        "left" / "right"  halves split on the divider's centreline (each about 214 x 95 mm), joined by JOIN_X bolts
//        "assembly"  the whole cage the right way up, in the CAGE frame (for viewing)
// The printable PARTs are placed upside down (roof on the bed): no supports are needed; the notches, blade passages
// and divider bolt holes print as short bridges.
//
// Print-fit changes from the simulation (outer faces that a supply can touch are unchanged):
//   * lead-ins run the full height to the roof top (LEADIN_PRINT_H instead of LEADIN_H) so they print on the bed;
//   * each lead-in is joined to its wall's front face by a small web filling the 1 mm wedge between them in task.py;
//     the bay side of that web continues the wall's inner face 1 mm forward, so the lead-in still meets it;
//   * the server body behind the back stop is printed as a REAR_RAIL_D deep stub (it is never touched in the task).

include <params.scad>

PART = "whole";

// A side wall (or the divider) from the mouth to the back stop, with the hook notch cut through it.
module notched_wall(y0, y1) {
    difference() {
        translate([WALL_X[0], y0, 0]) cube([WALL_X[1] - WALL_X[0], y1 - y0, WALL_H]);
        translate([NOTCH_X[0], y0 - EPS, NOTCH_Z[0]])
            cube([NOTCH_X[1] - NOTCH_X[0], y1 - y0 + 2 * EPS, NOTCH_Z[1] - NOTCH_Z[0]]);
    }
}

// task._lead_in(y_edge, outward): a LEADIN_L x LEADIN_T plate flared LEADIN_DEG, its inner edge meeting the wall's
// inner face (y_edge) at the mouth and flaring outward (+1 or -1 in y), plus the web joining it to the wall.
module lead_in(y_edge, outward) {
    a = LEADIN_DEG;
    cx = MOUTH_X - LEADIN_L / 2 * cos(a) - LEADIN_T / 2 * sin(a);
    cy = y_edge + outward * (LEADIN_L / 2 * sin(a) + LEADIN_T / 2 * cos(a));
    w = LEADIN_T / cos(a);
    // the plate
    translate([cx, cy, 0]) rotate([0, 0, -outward * a])
        translate([-LEADIN_L / 2, -LEADIN_T / 2, 0]) cube([LEADIN_L, LEADIN_T, LEADIN_PRINT_H]);
    // the web: hull of the plate's back end and a strip of the wall's front face
    hull() {
        translate([cx, cy, 0]) rotate([0, 0, -outward * a])
            translate([LEADIN_L / 2 - EPS, -LEADIN_T / 2, 0]) cube([EPS, LEADIN_T, LEADIN_PRINT_H]);
        translate([MOUTH_X, outward > 0 ? y_edge : y_edge - w, 0]) cube([EPS, w, LEADIN_PRINT_H]);
    }
}

// The blade passage through the back stop and rail of the bay centred on yc, and the flange's two inserts.
module blade_mount_cut(yc) {
    rear = SERVER[0] + REAR_RAIL_D;
    zc = (BLADE_Z[0] + BLADE_Z[1]) / 2;
    translate([BACKSTOP[0] - EPS, yc - RIDGE_APEX[1] - FIT_SLIDE, BLADE_Z[0] - FIT_SLIDE])
        cube([rear - BACKSTOP[0] + 2 * EPS, 2 * (RIDGE_APEX[1] + FIT_SLIDE), BLADE_Z[1] - BLADE_Z[0] + 2 * FIT_SLIDE]);
    for (s = [1, -1])
        translate([rear + EPS, yc + s * BLADE_SCREW_DY, zc]) rotate([0, -90, 0])
            cylinder(h = INSERT_L + EPS, d = INSERT_D);
}

// Heat-set inserts in the underside: two under the rail behind each bay, two under each half of the divider.
module board_insert_holes() {
    for (yc = [0, -BAY_PITCH])
        for (s = [1, -1])
            translate([SERVER[0] + REAR_RAIL_D / 2, yc + s * BOARD_INSERT_RAIL_DY, -EPS])
                cylinder(h = INSERT_L + EPS, d = INSERT_D);
    for (x = BOARD_INSERT_DIV_X)
        for (s = [1, -1])
            translate([x, SPLIT_Y + s * DIV_INSERT_DY, -EPS]) cylinder(h = INSERT_L + EPS, d = INSERT_D);
}

// Bolts across the divider: head counterbored into the left bay's face, nut trapped in the right bay's face.
module join_bolt_holes() {
    for (x = JOIN_X) {
        translate([x, DIVIDER_Y[1] + EPS, JOIN_Z]) rotate([90, 0, 0])
            cylinder(h = DIVIDER_Y[1] - DIVIDER_Y[0] + 2 * EPS, d = M3_CLEAR);
        translate([x, DIVIDER_Y[1] + EPS, JOIN_Z]) rotate([90, 0, 0])
            cylinder(h = JOIN_CBORE + EPS, d = M3_HEAD_D);
        translate([x, DIVIDER_Y[0] - EPS, JOIN_Z]) rotate([-90, 0, 0])
            cylinder(h = JOIN_NUT_DEPTH + EPS, d = M3_NUT_AF / cos(180 / NUT_FN), $fn = NUT_FN);
    }
}

module cage_body() {
    difference() {
        union() {
            notched_wall(WALL_Y[0], WALL_Y[1]);           // left outer wall, notched for the failed/spare hook
            notched_wall(DIVIDER_Y[0], DIVIDER_Y[1]);     // divider, notched for the live supply's hook
            box6(RIGHT_WALL);
            box6(ROOF);
            box6(BACKSTOP);
            translate([SERVER[0], SERVER[2], SERVER[4]])
                cube([REAR_RAIL_D, SERVER[3] - SERVER[2], SERVER[5] - SERVER[4]]);
            lead_in(WALL_Y[0], 1);
            lead_in(DIVIDER_Y[1], -1);
            lead_in(DIVIDER_Y[0], 1);
            lead_in(RIGHT_WALL[3], -1);
        }
        blade_mount_cut(0);
        blade_mount_cut(-BAY_PITCH);
        board_insert_holes();
        join_bolt_holes();
    }
}

// side = 1: the left half (left bay); side = -1: the right half.
module cage_half(side) {
    intersection() {
        cage_body();
        translate([-BIG / 2, side > 0 ? SPLIT_Y : SPLIT_Y - BIG, -BIG / 2]) cube([BIG, BIG, BIG]);
    }
}

// Upside down, roof on the bed.
module print_pose() {
    translate([0, 0, ROOF[5]]) rotate([180, 0, 0]) children();
}

if (PART == "whole") print_pose() cage_body();
else if (PART == "left") print_pose() cage_half(1);
else if (PART == "right") print_pose() cage_half(-1);
else if (PART == "assembly") cage_body();
