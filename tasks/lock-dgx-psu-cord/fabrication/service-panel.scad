// Millimetres. Flat fabrication profile seen looking at the rear panel.
// Schurter 4798 cutouts from page 3; bay pitch and panel envelope are prototype.
$fn = 64;
module inlet_cutout() {
    offset(r=2.5) square([32.60-5,24.50-5],center=true);
    for (x=[-21,21]) translate([x,0]) circle(d=3.5);
}
linear_extrude(2) difference() {
    square([237,58],center=true);
    for (x=[-79,0,79]) translate([x,0]) inlet_cutout();
}
