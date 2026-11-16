// handle.scad -- the fixed grip handle (task.py: _psu_boxes grip bar and arms), SUPPLY frame, mm.
//
// The bar (HANDLE_BAR) on its two arms (HANDLE_ARM_LO, HANDLE_ARM_HI), plus a spine joining the arms' ends that sits
// in a recess in the face plate (flush with it, FIT_PRESS clearance) and is bolted through the front wall with two
// M3 screws (heads inside the supply, nuts on the spine between the arms). In task.py the upper arm ends in the air
// above the face; the spine is what holds it on a real part.
// Print lying on its side (an 8 mm face on the bed): the bar, arms and spine are then one continuous outline.

include <params.scad>

PART = "handle";

module handle() {
    difference() {
        union() {
            box6(HANDLE_BAR);
            box6(HANDLE_ARM_LO);
            box6(HANDLE_ARM_HI);
            // the spine, in the face plate's recess
            translate([FACEPLATE[0] - EPS, HANDLE_BAR[2], HANDLE_BAR[4]])
                cube([FACEPLATE[1] - FACEPLATE[0] + EPS, HANDLE_BAR[3] - HANDLE_BAR[2], HANDLE_BAR[5] - HANDLE_BAR[4]]);
        }
        for (z = HANDLE_SCREW_Z)
            translate([FACEPLATE[0] - EPS, (HANDLE_BAR[2] + HANDLE_BAR[3]) / 2, z]) rotate([0, 90, 0])
                cylinder(h = FACEPLATE[1] - FACEPLATE[0] + 2 * EPS, d = M3_CLEAR);
    }
}

if (PART == "handle") rotate([90, 0, 0]) translate([0, -HANDLE_BAR[2], 0]) handle();
else if (PART == "assembly") handle();
