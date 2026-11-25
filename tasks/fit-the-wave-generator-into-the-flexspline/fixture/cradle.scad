// Assembly cradle and shaft handle for a StepperOnline HSS-14-50-I-D8 strain-wave gearbox. Units: mm.
// Gearbox figures from the maker's dimension drawing: 73 flange with 8 x 4.5 holes on a 65 circle, 56 h7 output pilot,
// input face 34 above the output face. The gearbox stands input side up: its 56 h7 output pilot drops into the
// riser's 56.5 bore, which centres it, and its flange is bolted to the riser top with 8 x M4 x 16 screws tapped into
// 3.3 holes (12 deep) on the 65 circle. The output flange, its cross-roller bearing and the circular spline's mounting
// are the gearbox's own; the output turns freely inside the bore, clear of the bench.
// Print: cradle in PLA, handle in PETG; 0.2 mm layers, 4 perimeters, 40 % gyroid infill, no supports
// (cradle flat side down, handle disc down). make_stl.py writes the same parts as cradle.stl and handle.stl.
$fn = 96;
plate = [120, 120, 10];
riser_od = 84;  riser_bore = 56.5;  riser_top = 53;    // bore centres the 56 h7 output pilot; flange seat 53 up
bolt_pcd = 65;  tap_d = 3.3;  tap_depth = 12;  n_bolts = 8;  // M4 tapped holes under the flange's 8 x 4.5 holes

module cradle() {
  difference() {
    union() {
      translate([-plate[0] / 2, -plate[1] / 2, 0]) cube(plate);
      cylinder(d = riser_od, h = riser_top);
    }
    translate([0, 0, -1]) cylinder(d = riser_bore, h = riser_top + 2);
    for (k = [0 : n_bolts - 1]) rotate((k + 0.5) * 360 / n_bolts)
      translate([bolt_pcd / 2, 0, riser_top - tap_depth]) cylinder(d = tap_d, h = tap_depth + 1, $fn = 32);
  }
}

// Handle: 30 x 3 finger-stop disc under a 20 x 20 x 16 square grip, pressed and glued on the 8 mm shaft.
module handle() {
  difference() {
    union() { cylinder(d = 30, h = 3); translate([-10, -10, 3]) cube([20, 20, 16]); }
    translate([0, 0, -1]) cylinder(d = 7.9, h = 21, $fn = 48);
  }
}

cradle();
translate([100, 0, 0]) handle();
