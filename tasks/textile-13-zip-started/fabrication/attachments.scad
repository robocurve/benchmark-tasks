// Units: millimeters. Print two blue plates and one orange pull.
// PETG, 0.2 mm layers, 40% infill. Blue assembly: two 2.8 mm plates
// plus 0.4 mm compressed tape/bond layer = 6 mm total.
part = "blue_plate";
if (part == "blue_plate") cube([14,10,2.8], center=true);
if (part == "orange_pull") cube([18,16,24], center=true);
if (part == "blue_assembly") {
    for (z=[-1.6,1.6]) translate([0,0,z]) cube([14,10,2.8], center=true);
}
