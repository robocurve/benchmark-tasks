// Assembly preview (not for printing): the frame, the sprung rail (opened by `open` mm, 0 to RAIL_TRAVEL), the
// guide rods, springs and dowel pins, and the SSD dummy seated, all in the carrier frame of task.py.

include <params.scad>
include <common.scad>
use <carrier_frame.scad>
use <sprung_rail.scad>
use <ssd_dummy.scad>

open = 0;

color("dimgray") carrier_frame();
translate([0, open, 0]) {
    color("royalblue") sprung_rail();
    // guide rods, from the bottom of their blind bores in the rail
    for (x = ROD_X) color("silver") ybore(x, RAIL_OUT + BOSS_L - ROD_EMBED, RAIL_OUT + BOSS_L - ROD_EMBED + ROD_L,
                                          ROD_Z, ROD_D);
    // the sprung rail's dowel pins
    for (x = PIN_X) color("silver") ybore(x, RAIL_OUT + PIN_BOSS - PIN_L, RAIL_OUT + PIN_BOSS, PIN_Z, PIN_D);
}
// the fixed rail's dowel pins
for (x = PIN_X) color("silver") ybore(x, -RAIL_OUT - PIN_BOSS, -RAIL_OUT - PIN_BOSS + PIN_L, PIN_Z, PIN_D);
// springs (shown as tubes) between each rod boss and the bottom of its pocket in the mount
for (x = ROD_X) color("gold") difference() {
    ybore(x, RAIL_OUT + BOSS_L + open, MOUNT_Y0 + SPRING_POCKET, ROD_Z, SPRING_OD);
    ybore(x, RAIL_OUT + BOSS_L + open - 1, MOUNT_Y0 + SPRING_POCKET + 1, ROD_Z, SPRING_OD - 2 * SPRING_WIRE);
}
// the SSD dummy, seated
translate([SEAT_X, 0, SEAT_Z]) color("lightgray") ssd_dummy();
