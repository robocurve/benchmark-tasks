// Millimetres. A 3 mm transverse retractable pin retains the spring-loaded sliding flag.
part = "slider";
$fn=36;
module slider() {
  difference() {
    union() {
      translate([-8,-2,0]) cube([16,4,36]);
      translate([-10,-3,0]) cube([20,6,3]);
    }
    translate([0,5,21]) rotate([90,0,0]) cylinder(h=10,d=3.4);
  }
}
module guide() {
  difference() {
    translate([-13,-7,0]) cube([26,14,42]);
    // Service opening underneath for spring and slider. Top shoulder captures its stop foot.
    translate([-10.35,-3.35,-1]) cube([20.7,6.7,37]);
    translate([-8.4,-2.4,35]) cube([16.8,4.8,8]);
    translate([0,8,29]) rotate([90,0,0]) cylinder(h=16,d=3.3);
  }
}
module base() {
  difference() {
    translate([-15,-10,0]) cube([30,20,4]);
    translate([0,0,3]) cylinder(h=2,d=6.5);
    for(x=[-12,12]) translate([x,0,-1]) cylinder(h=6,d=3.2);
  }
}
if(part=="guide") guide();
else if(part=="base") base();
else slider();
