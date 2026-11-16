// plug_inlet.scad -- the AC inlet on each supply's face and the live supply's dummy power-cord plug
// (task.py: _psu_boxes INLET and PLUG), SUPPLY frame, mm.
//
// inlet_add() / inlet_cut() are used by psu_shell.scad: the inlet's raised face (INLET), a shallow recess in it the
// plug body fits into, a socket through the face and front wall into a boss behind it for the plug's tongue, and an
// M3 screw hole down through the boss into the tongue. The screw makes the plug rigid with its supply, as in task.py
// (knocking the plug moves the live supply). Only the live supply gets a plug; on the other two the socket is empty.
//
// PART = "plug"      the dummy C13 plug: a block PLUG with a tongue at its back, and an optional hole in its front
//                    for a short stub of cut power cord (no conductors connected, never plugged into anything).
//                    Printed standing on its front face.
//        "assembly"  in place.

include <params.scad>

PART = "plug";

module inlet_add() {
    box6(INLET);
    // boss behind the front wall, from the floor to the top of the inlet
    translate([FACE_X + WALL - EPS, PLUG[2], 0]) cube([INLET_BOSS_D + EPS, PLUG[3] - PLUG[2], INLET[5]]);
}

module inlet_cut() {
    // recess in the inlet's face: the plug body reaches the face plate, as in task.py
    translate([INLET[0] - EPS, PLUG[2] - FIT_SLIDE, PLUG[4] - FIT_SLIDE])
        cube([INLET[1] - INLET[0] + EPS, PLUG[3] - PLUG[2] + 2 * FIT_SLIDE, PLUG[5] - PLUG[4] + 2 * FIT_SLIDE]);
    // socket for the tongue
    translate([INLET[1] - EPS, PLUG[2] + PLUG_TONGUE_INSET - FIT_SLIDE, PLUG[4] + PLUG_TONGUE_INSET - FIT_SLIDE])
        cube([tongue_x1() - INLET[1] + FIT_SLIDE + EPS,
              PLUG[3] - PLUG[2] - 2 * PLUG_TONGUE_INSET + 2 * FIT_SLIDE,
              PLUG[5] - PLUG[4] - 2 * PLUG_TONGUE_INSET + 2 * FIT_SLIDE]);
    // screw from the top of the boss into the tongue
    translate([plug_screw_x(), plug_screw_y(), PLUG[5] - PLUG_TONGUE_INSET])
        cylinder(h = INLET[5] - (PLUG[5] - PLUG_TONGUE_INSET) + EPS, d = M3_CLEAR);
}

module plug() {
    difference() {
        union() {
            box6(PLUG);
            translate([PLUG[1] - EPS, PLUG[2] + PLUG_TONGUE_INSET, PLUG[4] + PLUG_TONGUE_INSET])
                cube([tongue_x1() - PLUG[1] + EPS, PLUG[3] - PLUG[2] - 2 * PLUG_TONGUE_INSET,
                      PLUG[5] - PLUG[4] - 2 * PLUG_TONGUE_INSET]);
        }
        translate([plug_screw_x(), plug_screw_y(), PLUG[4] - EPS]) cylinder(h = PLUG[5] - PLUG[4] + 2 * EPS, d = M3_PILOT);
        translate([PLUG[0] - EPS, plug_screw_y(), (PLUG[4] + PLUG[5]) / 2]) rotate([0, 90, 0])
            cylinder(h = CORD_DEPTH + EPS, d = CORD_D);
    }
}

if (PART == "plug") translate([0, 0, -PLUG[0]]) rotate([0, -90, 0]) plug();
else if (PART == "assembly") plug();
