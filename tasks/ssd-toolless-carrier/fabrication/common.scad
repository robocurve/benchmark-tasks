// Small helpers shared by the part files. Include after params.scad.

// An axis-aligned box given by its extents, as task.py's _mm(x0, x1, y0, y1, z0, z1) does.
module box(x0, x1, y0, y1, z0, z1) {
    translate([x0, y0, z0]) cube([x1 - x0, y1 - y0, z1 - z0]);
}

// A round hole or rod along +y from y0 to y1, its axis at (x, z).
module ybore(x, y0, y1, z, d) {
    translate([x, y0, z]) rotate([-90, 0, 0]) cylinder(d = d, h = y1 - y0);
}
