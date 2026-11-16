// SSD dummy: a 2.5-inch drive's envelope (SSD_L x SSD_W x SSD_H) with the task's four side holes (HOLE square,
// HOLE_DEPTH deep, centres HOLE_Z above the bottom face at HOLE_X along it; the connector end is +x). Drawn in the
// SSD frame of task.py: origin under its centre on its bottom face. Print it flat, bottom down, no supports (the holes
// bridge HOLE mm). Weigh it and adjust the infill to reach SSD_MASS_G (see BUILD.md); add a paper label on top over
// LABEL_X x LABEL_Y, and blacken the connector end over CONN_X x CONN_Y.

include <params.scad>
include <common.scad>

module ssd_dummy() {
    difference() {
        box(-SSD_L / 2, SSD_L / 2, -SSD_W / 2, SSD_W / 2, 0, SSD_H);
        for (x = HOLE_X) {
            // the +y side's holes, then the -y side's
            box(x - HOLE / 2, x + HOLE / 2, HALF_W - HOLE_DEPTH, HALF_W + 1, HOLE_Z - HOLE / 2, HOLE_Z + HOLE / 2);
            box(x - HOLE / 2, x + HOLE / 2, -HALF_W - 1, -HALF_W + HOLE_DEPTH, HOLE_Z - HOLE / 2, HOLE_Z + HOLE / 2);
        }
    }
}

ssd_dummy();
