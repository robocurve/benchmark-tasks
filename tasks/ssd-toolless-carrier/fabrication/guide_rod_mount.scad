// Guide-rod mount: the fixed block that each guide rod slides through, with a pocket for the rod's spring.
// It is printed as part of the carrier frame (carrier_frame.scad places two of them on base tongues outboard of the
// sprung rail), so the bore lines up with the rail's rod boss without any adjustment. Opening this file on its own
// shows one mount, in the carrier frame, at the first rod position.
//
// The rod is pressed into the sprung rail and slides in this mount's bore (ROD_D + SLIDE_CLR). The spring sits over
// the rod between the rail's boss and the bottom of the pocket; it pushes the rail closed. With the rail fully open
// the boss meets the mount's inner face: that is the 7 mm travel stop.

include <params.scad>
include <common.scad>

module guide_rod_mount(x) {
    difference() {
        box(x - MOUNT_W / 2, x + MOUNT_W / 2, MOUNT_Y0, MOUNT_Y1, 0, MOUNT_H);
        // sliding bore for the guide rod, right through
        ybore(x, MOUNT_Y0 - 1, MOUNT_Y1 + 1, ROD_Z, ROD_D + SLIDE_CLR);
        // spring pocket, open toward the rail
        ybore(x, MOUNT_Y0 - 1, MOUNT_Y0 + SPRING_POCKET, ROD_Z, SPRING_POCKET_D);
    }
}

guide_rod_mount(ROD_X[0]);
