// assembly.scad -- every part in place, for checking fits in OpenSCAD's preview (not for printing). CAGE frame, mm.
// The failed supply sits latched home in the left bay and the live supply (with its plug) in the right bay, each
// face SEAT proud of the mouth as in task.py; the spare is left out. Set SECTION = true to cut the view at the
// height of the hook and the connector, to see the catch in its notch and the contacts behind the ridges.

include <params.scad>
use <cage.scad>
use <connector.scad>
use <latch.scad>
use <handle.scad>
use <plug_inlet.scad>
use <psu_shell.scad>

SECTION = false;

module supply(with_plug) {
    color("DimGray") tray();
    color("DimGray") housing();
    color("DimGray", 0.5) lid();
    color("Orange") handle();
    color("SeaGreen") tab_link();
    color("SeaGreen") carriage();
    color("SeaGreen") hook();
    color("Silver") contact_carrier();
    color("Silver") mirror([0, 1, 0]) contact_carrier();
    if (with_plug) color("Black") plug();
}

module everything() {
    color("LightGray") cage_body();
    color("Silver") blade();
    color("Silver") translate([0, -BAY_PITCH, 0]) blade();
    supply(false);
    translate([0, -BAY_PITCH, 0]) supply(true);
}

if (SECTION)
    intersection() {
        everything();
        translate([-BIG / 2, -BIG / 2, 0]) cube([BIG, BIG, (CATCH[4] + CATCH[5]) / 2]);
    }
else
    everything();
