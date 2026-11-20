// Units are millimetres. Select one part at a time and export as STL.
part = "assembly"; // [assembly,base,cover,carrier,riser,saddle,reset_template]
$fn = 48;

module plate(size, centre) {
    translate(centre) cube(size, center=true);
}

module bore(x, y, z, length) {
    translate([x, y, z]) rotate([90, 0, 0])
        cylinder(h=length, d=3.25, center=true);
}

module base() {
    difference() {
        union() {
            plate([139, 106, 9], [0, 0, 4.5]);
            for (side = [-1, 1])
                plate([16, 10, 22.4], [66, side*52, 20]);
        }
        for (side = [-1, 1])
            bore(66, side*52, 27, 12);
        for (x = [-61, 61], y = [-45, 45])
            translate([x, y, -0.5]) cylinder(h=10, d=3.4);
    }
}

module cover() {
    difference() {
        union() {
            for (side = [-1, 1])
                plate([124, 6, 8], [-61, side*44, 0]);
            for (k = [0:6])
                plate([5, 95, 6], [-121 + k*19.5, 0, 0]);
            plate([19, 60, 21], [-116, 0, 16]);
            plate([19, 60, 5], [-116, 0, 4.5]);
            // Continuous hinge knuckle between the two tray ears.
            translate([0, 0, 0]) rotate([90, 0, 0])
                cylinder(h=91, d=8, center=true);
        }
        bore(0, 0, 0, 94);
    }
}

module carrier() {
    difference() {
        union() {
            plate([106, 82, 1.5], [0, 0, .75]);
            for (side = [-1, 1])
                plate([25, 11, 5], [43, side*45, 2.5]);
        }
        // Align the 100 x 75 working field on these four corner marks.
        for (x = [-50, 50], y = [-37.5, 37.5])
            translate([x, y, 1.2]) cylinder(h=.4, d=1.4);
    }
}

module riser() {
    difference() {
        translate([-95.5, -104.5, 0]) cube([237, 163, 78.5]);
        // Open underside for fastening the tray and conserving print material.
        translate([-90.5, -99.5, -.5]) cube([227, 153, 74]);
        for (x = [-61, 61], y = [-45, 45])
            translate([x, y, 72]) cylinder(h=8, d=3.4);
    }
}

module saddle() {
    for (side = [-1, 1])
        plate([9, 34, 129], [side*69, 0, 64.5]);
    plate([147, 34, 3], [0, 0, 1.5]);
}

module reset_template() {
    difference() {
        plate([100, 75, 1.2], [0, 0, .6]);
        for (point = [[-29, -22, 14], [12, -2, 16], [28, 21, 14]])
            translate([point[0], point[1], -.2])
                cylinder(h=1.6, d=point[2]);
    }
}

if (part == "base") base();
if (part == "cover") cover();
if (part == "carrier") carrier();
if (part == "riser") riser();
if (part == "saddle") saddle();
if (part == "reset_template") reset_template();
if (part == "assembly") {
    translate([0, 0, -78.5]) riser();
    base();
    translate([0, 0, 9]) carrier();
    translate([66, 0, 27]) rotate([0, 96, 0]) cover();
    translate([0, -141, -78.5]) saddle();
}
