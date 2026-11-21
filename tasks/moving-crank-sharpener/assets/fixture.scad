// Millimetre geometric reference; use real functional tools, not printed heated or cutting equipment.
// Source SHA-256: 97d97832c856c9dcbde0cf4b1b9216d82a9513e5030d10fc58a440a32c2207de
$fn=48;
module stage(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([180, 140, 30],center=true);
translate([0, 0, 42.5]) rotate([0, 0, 0]) cube([25, 50, 55],center=true);
}
module tool(){
translate([0, -19.75, 0]) rotate([0, 0, 0]) cube([60, 30.5, 70],center=true);
translate([0, 19.75, 0]) rotate([0, 0, 0]) cube([60, 30.5, 70],center=true);
translate([0, 0, -19.75]) rotate([0, 0, 0]) cube([60, 9, 30.5],center=true);
translate([0, 0, 19.75]) rotate([0, 0, 0]) cube([60, 9, 30.5],center=true);
}
module wheel(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=8,h=18,center=true);
translate([0, 0, -17.5]) rotate([0, 0, 0]) cube([8, 12, 43],center=true);
translate([20, 0, -35]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=3,h=40,center=true);
}
module knob(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=8,h=30,center=true);
}
module cutter(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([3, 8, 8],center=true);
}
module pencil(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=3.5,h=140,center=true);
}
module cradle_left(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([16, 40, 30],center=true);
}
module cradle_right(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([16, 40, 30],center=true);
}
translate([245, -40, 762.5]) rotate([0,0,0]) stage();
translate([245, -40, 867.5]) rotate([0,0,0]) tool();
translate([285, -40, 867.5]) rotate([0,0,0]) wheel();
translate([331, -40, 832.5]) rotate([0,0,0]) knob();
translate([232, -40, 867.5]) rotate([0,0,0]) cutter();
translate([120, 100, 781]) rotate([0,0,0]) pencil();
translate([55, 100, 762.5]) rotate([0,0,0]) cradle_left();
translate([160, 100, 762.5]) rotate([0,0,0]) cradle_right();
