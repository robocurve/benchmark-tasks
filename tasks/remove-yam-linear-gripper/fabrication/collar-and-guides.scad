// Millimetres. Prototype coordinate origin is the pinned URDF gripper mount.
// Render one part at a time for export: saddle, bridge, bushing, or exploded.
part = "exploded";
$fn = 80;
module box(at, size) { translate(at) cube(size, center=true); }
module saddle() {
    box([42,0,-50.9], [81,77,6.2]);
    for (y=[-38.5,38.5]) box([42,y,-48.5], [80,6,11]);
    box([-19,0,-5], [10,55,49]);
}
module bridge() {
    difference() {
        union() {
            box([-27,0,21.5], [19,48,16]);
            for (y=[-15.5,15.5]) box([-35,y,32],[3,11,24]);
        }
        for (y=[-15.5,15.5])
            translate([-27,y,21.5]) cylinder(d=4.4,h=30,center=true);
    }
}
module bushing() {
    difference() {
        cylinder(d=11.6,h=4.3);
        cylinder(d=5.56,h=3.4);
        translate([0,0,3.0]) cylinder(d1=5.56,d2=10.6,h=1.4);
    }
}
if (part=="saddle") saddle();
if (part=="bridge") bridge();
if (part=="bushing") bushing();
if (part=="exploded") {
    saddle();
    translate([0,0,12]) bridge();
    for (y=[-15.5,15.5]) {
        translate([-27,y,60]) bushing();
        translate([-27,y,54]) color("silver") cylinder(d=5.5,h=3);
        translate([-27,y,46]) color("silver") cylinder(d=3,h=8);
    }
}
