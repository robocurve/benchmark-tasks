// Inert bench display, dimensions in millimetres.
// Export one selectable part at a time. No interface to an industrial spindle.
part = "chuck_display";
$fn = 96;
module chuck_display() {
    union() {
        cylinder(h=50,d=101,center=true);
        for (a=[0,120,240])
            rotate([0,0,a]) translate([32,0,26]) cube([17,15,17],center=true);
    }
}
module tray() {
    union() {
        cube([85,83,41],center=true);
        for (y=[-40,40])
            translate([0,y,25]) cube([87,5,12],center=true);
        for (x=[-40,40])
            translate([x,0,26.5]) cube([5,79,15],center=true);
    }
}
// Paint template for the flat face. Align zero with the upward datum.
// The matching rim stripes are 48 mm long and centered on the same angles.
module face_indices() {
    for (p=[[12,17,38],[141,10,24]])
        rotate([0,0,-p[0]]) translate([0,47-p[2]/2,0]) square([p[1],p[2]],center=true);
}
if (part=="face_indices") face_indices();
if (part=="chuck_display") chuck_display();
if (part=="tray") tray();

// Moving cap on the isolated instrument pad. Inside faces are 27 by 30 mm.
module service_pad() {
    union() {
        cube([40,40,12], center=true);
        for (x=[-14.5,14.5])
            translate([x,0,8]) cube([2,34,4], center=true);
        for (y=[-16,16])
            translate([0,y,8]) cube([27,2,4], center=true);
    }
}
if (part=="service_pad") service_pad();
