// connector.scad -- the edge connector (task.py: _blade, _contacts), mm.
//
// PART = "blade"    the cage-side blade (CAGE frame, for the left bay; print two). A BLADE_T core from its tip at
//                   BLADE_X[0] through the back stop and rail, two ridges with RIDGE_DEG (50-degree) front faces and
//                   90 - RIDGE_DEG (40-degree) back faces cresting at RIDGE_APEX, and a flange that screws to the rail's
//                   rear face. The flange sets the ridge's position along the bay. Insert it from behind the cage.
//        "contact"  one sprung contact carrier (SUPPLY frame, the left one at rest; print two and turn one over for
//                   the right: the part is a prism, so turned over it is its own mirror image). Its tip is task.py's
//                   contact (CONTACT_X x CONTACT_Y x CONTACT_Z); the body behind it slides in a pocket of the contact
//                   housing (psu_shell.scad) on an M3 guide screw that passes through its spring.
//        "assembly" both in place, the right way up.
// Both print standing on their bottom face (z up): the ridge and contact faces are then smooth vertical walls along
// the sliding direction, and the screw holes are horizontal.

include <params.scad>

PART = "blade";

// One ridge in plan, as task._blade builds it: a block with sides RIDGE_SIDES, yawed side * RIDGE_DEG, whose apex
// corner sits at RIDGE_APEX (mirrored to this side). Its front face rises at RIDGE_DEG, its back face falls at
// 90 - RIDGE_DEG.
module ridge_2d(side) {
    a = side * RIDGE_DEG;
    lu = RIDGE_SIDES[0];
    lv = RIDGE_SIDES[1];
    ux = cos(a);
    uy = sin(a);
    vx = -sin(a);
    vy = cos(a);
    cx = RIDGE_APEX[0] - lu / 2 * ux - side * lv / 2 * vx;
    cy = side * RIDGE_APEX[1] - lu / 2 * uy - side * lv / 2 * vy;
    translate([cx, cy]) rotate(a) square([lu, lv], center = true);
}

module blade() {
    rear = SERVER[0] + REAR_RAIL_D;
    zc = (BLADE_Z[0] + BLADE_Z[1]) / 2;
    difference() {
        translate([0, 0, BLADE_Z[0]]) linear_extrude(height = BLADE_Z[1] - BLADE_Z[0]) union() {
            translate([BLADE_X[0], -BLADE_T / 2]) square([rear - BLADE_X[0], BLADE_T]);
            ridge_2d(1);
            ridge_2d(-1);
            translate([rear, -BLADE_FLANGE_W / 2]) square([BLADE_FLANGE_T, BLADE_FLANGE_W]);
        }
        for (s = [1, -1])
            translate([rear - EPS, s * BLADE_SCREW_DY, zc]) rotate([0, 90, 0])
                cylinder(h = BLADE_FLANGE_T + 2 * EPS, d = M3_CLEAR);
    }
}

module contact_carrier() {
    zc = (CONTACT_Z[0] + CONTACT_Z[1]) / 2;
    difference() {
        translate([0, 0, CONTACT_Z[0]]) linear_extrude(height = CONTACT_Z[1] - CONTACT_Z[0]) union() {
            // the contact tip (task.py's contact block)
            translate([CONTACT_X[0], CONTACT_Y[0]]) square([CONTACT_X[1] - CONTACT_X[0], CONTACT_Y[1] - CONTACT_Y[0]]);
            // the body that slides in the housing's pocket, outboard of the ridge's crest
            translate([CARRIER_X0, CONTACT_Y[1] - EPS]) square([CONTACT_X[1] - CARRIER_X0, CARRIER_Y1 - CONTACT_Y[1] + EPS]);
        }
        // pilot for the M3 guide screw (it cuts its own thread; turning it sets the contact's rest position)
        translate([CARRIER_SCREW_X, CARRIER_Y1 + EPS, zc]) rotate([90, 0, 0])
            cylinder(h = CARRIER_SCREW_DEPTH + EPS, d = M3_PILOT);
    }
}

if (PART == "blade") translate([0, 0, -BLADE_Z[0]]) blade();
else if (PART == "contact") translate([0, 0, -CONTACT_Z[0]]) contact_carrier();
else if (PART == "assembly") {
    blade();
    contact_carrier();
    mirror([0, 1, 0]) contact_carrier();
}
