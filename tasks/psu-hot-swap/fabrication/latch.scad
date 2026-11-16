// latch.scad -- the release latch (task.py: _latch, _hook_ramp), SUPPLY frame at rest, mm.
//
// Three printed parts, screwed together inside the supply so they slide as one, LATCH_TRAVEL toward the handle (-y):
// PART = "tab"       the release tab (task: TAB) and its link (task: LINK), which runs on through a slot in the face
//                    into the supply, to link_end_x(), riding on the floor. Print lying on its side (8 mm face down).
//        "carriage"  the latch arm: a plate that slides across the supply in the guide channel (front wall and guide
//                    rib, psu_shell.scad), with a socket that drops over the link's end (one M3 screw down into the
//                    link) and two pilot holes for the hook. Print floor down.
//        "hook"      the hook: a post that sits on the carriage's top against the side wall, carrying the square catch
//                    (task: CATCH) and the ramp (task: RAMP_TOP, RAMP_DEG, RAMP_T, RAMP_Z) that stand out through the
//                    window in the supply's left side; two M3 screws down through the post into the carriage. It is a
//                    separate part so it can be slid out through the window after the carriage is in place (the
//                    catch stands out further than the latch's travel). Print upside down (as placed); no supports.
//        "assembly"  all three in place.
// The tab spring (LATCH_RATE, about 1 N/mm) sits in a pocket of the fixed spring block in the tray and pushes on the
// carriage's -y end. Rest stop: the carriage's +y face against the side wall's inner face, below the window (the
// catch then stands CATCH[3] - PSU_W/2 = 5 mm out of the side). Push stop: the carriage's -y end against the spring
// block after LATCH_TRAVEL, which leaves the tab TAB_Y[0] - LATCH_TRAVEL - HANDLE_Y[1] = 1 mm short of the handle, as
// in task.py. Inside the supply the carriage runs FIT_SLIDE above the floor; the outer faces are task.py's.

include <params.scad>

PART = "tab";

INNER = PSU_W / 2 - WALL;                // the side wall's inner face

module tab_link() {
    difference() {
        union() {
            box6(TAB);
            translate([LINK[0] - EPS, LINK[2], LINK[4]])
                cube([link_end_x() - LINK[0] + EPS, LINK[3] - LINK[2], LINK[5] - LINK[4]]);
        }
        translate([link_screw_x(), (LINK[2] + LINK[3]) / 2, LINK[4] - EPS])
            cylinder(h = LINK[5] - LINK[4] + 2 * EPS, d = M3_PILOT);
    }
}

module carriage() {
    x0 = FACE_X + WALL + FIT_SLIDE;
    z0 = FLOOR + FIT_SLIDE;
    difference() {
        translate([x0, CARR_Y0, z0]) cube([CARR_X1 - x0, INNER - CARR_Y0, CARR_TOP - z0]);
        // socket for the link's end (open at the front and below)
        translate([x0 - EPS, LINK[2] - FIT_PRESS, z0 - EPS])
            cube([link_end_x() + FIT_PRESS - x0 + EPS, LINK[3] - LINK[2] + 2 * FIT_PRESS, LINK[5] + FIT_SLIDE - z0 + EPS]);
        // M3 screw down into the link
        translate([link_screw_x(), (LINK[2] + LINK[3]) / 2, LINK[5]])
            cylinder(h = CARR_TOP - LINK[5] + EPS, d = M3_CLEAR);
        // pilots for the hook's screws
        for (x = HOOK_SCREW_X)
            translate([x, INNER - POST_T / 2, CARR_TOP - HOOK_SCREW_DEPTH]) cylinder(h = HOOK_SCREW_DEPTH + EPS, d = M3_PILOT);
    }
}

// task._hook_ramp: a RAMP_T plate whose outer face runs from the crest RAMP_TOP back to the supply's side at
// RAMP_DEG, filled in behind down to the side wall's inner face so it is one solid with the post.
module ramp() {
    x0 = RAMP_TOP[0];
    y0 = RAMP_TOP[1];
    y1 = PSU_W / 2;
    x1 = ramp_tail_x();
    a = atan2(y1 - y0, x1 - x0);
    rl = sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2));
    nx = -sin(a);
    ny = cos(a);
    cx = (x0 + x1) / 2 - nx * RAMP_T / 2;
    cy = (y0 + y1) / 2 - ny * RAMP_T / 2;
    xf = x0 - RAMP_T * nx;
    hull() {
        translate([cx, cy, RAMP_Z[0]]) rotate([0, 0, a])
            translate([-rl / 2, -RAMP_T / 2, 0]) cube([rl, RAMP_T, RAMP_Z[1] - RAMP_Z[0]]);
        translate([xf, INNER - EPS, RAMP_Z[0]]) cube([x1 - xf, EPS, RAMP_Z[1] - RAMP_Z[0]]);
    }
}

module hook() {
    difference() {
        union() {
            // the post, on the carriage's top
            translate([CATCH[0] - POST_FRONT, INNER - POST_T, CARR_TOP])
                cube([CARR_X1 - (CATCH[0] - POST_FRONT), POST_T, CATCH[5] - CARR_TOP]);
            // the square catch, through the side window
            translate([CATCH[0], INNER - EPS, CATCH[4]])
                cube([CATCH[1] - CATCH[0], CATCH[3] - INNER + EPS, CATCH[5] - CATCH[4]]);
            ramp();
        }
        for (x = HOOK_SCREW_X)
            translate([x, INNER - POST_T / 2, CARR_TOP - EPS]) cylinder(h = CATCH[5] - CARR_TOP + 2 * EPS, d = M3_CLEAR);
    }
}

if (PART == "tab") rotate([90, 0, 0]) translate([0, -TAB[2], 0]) tab_link();
else if (PART == "carriage") translate([0, 0, -(FLOOR + FIT_SLIDE)]) carriage();
else if (PART == "hook") translate([0, 0, CATCH[5]]) rotate([180, 0, 0]) hook();
else if (PART == "assembly") {
    tab_link();
    carriage();
    hook();
}
