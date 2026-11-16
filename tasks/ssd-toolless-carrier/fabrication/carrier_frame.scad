// Carrier frame: the handle bezel and release handle, front floor plate, two cross ribs the drive rests on, the
// fixed rail with its two pin bores, the sprung rail's closed stops, the two guide-rod mounts, and ballast pockets.
// Coordinates are the carrier frame of task.py (see params.scad). Print it as it lies: base down, no supports except
// under the release handle (HANDLE_Z[0] mm overhang).
//
// Additions to task.py's boxes, none of which the drive or the fingers touch:
//   - a BASE_T base web under the whole carrier, and two tongues of it that carry the guide-rod mounts;
//   - the ribs continue (STOP_H tall, below the drive) to y = RAIL_IN on the sprung side: the rail's closed stop;
//   - a ballast slab between the ribs, under the drive (top SLAB_H, below the seated drive's bottom at SEAT_Z);
//   - 0.5 mm pads on the fixed rail's outer face round the pin bores;
//   - pockets for steel ballast in the slab (from the top), the bezel and the front floor plate (from below).

include <params.scad>
include <common.scad>
use <guide_rod_mount.scad>

module carrier_frame() {
    difference() {
        union() {
            // base web under the carrier, from the bezel to the rear edge and from rail to rail
            box(BEZEL_X[0], CAR_L / 2, -RAIL_OUT, RAIL_OUT, 0, BASE_T);
            // tongues of the web carrying the guide-rod mounts, under the sprung rail's travel
            for (x = ROD_X) box(x - TONGUE_W / 2, x + TONGUE_W / 2, RAIL_OUT - 1, MOUNT_Y0 + 1, 0, BASE_T);
            for (x = ROD_X) guide_rod_mount(x);
            // handle bezel and its release handle
            box(BEZEL_X[0], BEZEL_X[1], -BEZEL_Y, BEZEL_Y, 0, CAR_H);
            box(HANDLE_X[0], HANDLE_X[1] + 1, -HANDLE_Y, HANDLE_Y, HANDLE_Z[0], HANDLE_Z[1]);
            // front floor plate
            box(FLOOR_X[0] - 1, FLOOR_X[1], -FLOOR_Y, FLOOR_Y, 0, FLOOR_H);
            // cross ribs the drive rests on (top at SEAT_Z), and their low extensions: the sprung rail's closed stop
            for (r = RIB_X) box(r[0], r[1], -RAIL_OUT, RIB_Y_END, 0, SEAT_Z);
            for (r = RIB_X) box(r[0], r[1], RIB_Y_END - 1, RAIL_IN, 0, STOP_H);
            // ballast slab between the ribs, under the drive
            box(SLAB_X[0], SLAB_X[1], -SLAB_Y, SLAB_Y, 0, SLAB_H);
            // fixed rail: base bar along the full length, and the two segments around the pins
            box(FIXED_BAR_X[0], FIXED_BAR_X[1], -RAIL_OUT, -RAIL_IN, 0, FIXED_BAR_H);
            box(FIXED_SEG0_X0 - 1, RAIL_SEGS[0][1], -RAIL_OUT, -RAIL_IN, 0, RAIL_H);
            box(RAIL_SEGS[1][0], RAIL_SEGS[1][1], -RAIL_OUT, -RAIL_IN, 0, RAIL_H);
            // pads round the pin bores on the fixed rail's outer face
            for (x = PIN_X)
                box(x - PIN_D / 2 - 1.5, x + PIN_D / 2 + 1.5, -RAIL_OUT - PIN_BOSS, -RAIL_OUT + 1,
                    PIN_Z - PIN_D / 2 - 1.5, PIN_Z + PIN_D / 2 + 1.5);
        }
        // press-fit bores for the two fixed dowel pins, right through the rail and its pad
        for (x = PIN_X) ybore(x, -RAIL_OUT - PIN_BOSS - 1, -RAIL_IN + 1, PIN_Z, PIN_D + PRESS_CLR);
        // ballast pocket in the slab, open at the top (the plate's top is flush with the slab, below the drive)
        translate([(SLAB_X[0] + SLAB_X[1]) / 2, 0, SLAB_H - SLAB_POCKET[2] + SLAB_POCKET[2] / 2 + 0.5])
            cube([SLAB_POCKET[0] + POCKET_CLR, SLAB_POCKET[1] + POCKET_CLR, SLAB_POCKET[2] + 1], center = true);
        // ballast pocket in the bezel, open underneath
        translate([(BEZEL_X[0] + BEZEL_X[1]) / 2, 0, (BEZEL_POCKET[2] + POCKET_CLR - 1) / 2])
            cube([BEZEL_POCKET[0] + POCKET_CLR, BEZEL_POCKET[1] + POCKET_CLR, BEZEL_POCKET[2] + POCKET_CLR + 1],
                 center = true);
        // ballast pocket in the front floor plate, open underneath
        translate([(FLOOR_X[0] + FLOOR_X[1]) / 2, 0, (FLOOR_POCKET[2] + POCKET_CLR - 1) / 2])
            cube([FLOOR_POCKET[0] + POCKET_CLR, FLOOR_POCKET[1] + POCKET_CLR, FLOOR_POCKET[2] + POCKET_CLR + 1],
                 center = true);
    }
}

carrier_frame();
