// Sprung rail: the separate side rail that slides outward (+y) up to RAIL_TRAVEL against the springs. It carries two
// dowel pins (pressed through it, standing PIN_IN + GAP in from its inner face), the pull tab on its bracket, and two
// rod bosses on its outer face, each with a blind press-fit bore for a guide rod.
//
// sprung_rail() draws it in the carrier frame, closed (its inner face at y = RAIL_IN), as assembly.scad uses it.
// The part as rendered by this file is moved down to the bed: print it upright, as it stands in the carrier, with
// supports under the tab's bracket only (BRACKET_Z[0] - RAIL_Z0 mm up). Print in blue, or paint the tab blue.

include <params.scad>
include <common.scad>

module sprung_rail() {
    difference() {
        union() {
            // the two rail segments around the pins, and the low bar joining them
            for (s = RAIL_SEGS) box(s[0], s[1], RAIL_IN, RAIL_OUT, RAIL_Z0, RAIL_H);
            box(RAIL_BAR_X[0], RAIL_BAR_X[1], RAIL_IN, RAIL_OUT, RAIL_Z0, RAIL_BAR_H);
            // the pull tab's bracket, and the tab
            box(TAB_X[0], TAB_X[1], RAIL_OUT - 1, TAB_Y[1], BRACKET_Z[0], BRACKET_Z[1]);
            box(TAB_X[0], TAB_X[1], TAB_Y[0], TAB_Y[1], TAB_Z[0], TAB_Z[1]);
            // rod bosses on the outer face
            for (x = ROD_X) box(x - BOSS_W / 2, x + BOSS_W / 2, RAIL_OUT - 1, RAIL_OUT + BOSS_L, RAIL_Z0, RAIL_H);
            // pads round the pin bores on the outer face
            for (x = PIN_X)
                box(x - PIN_D / 2 - 1.5, x + PIN_D / 2 + 1.5, RAIL_OUT - 1, RAIL_OUT + PIN_BOSS,
                    PIN_Z - PIN_D / 2 - 1.5, PIN_Z + PIN_D / 2 + 1.5);
        }
        // press-fit bores for the two dowel pins, right through
        for (x = PIN_X) ybore(x, RAIL_IN - 1, RAIL_OUT + PIN_BOSS + 1, PIN_Z, PIN_D + PRESS_CLR);
        // blind press-fit bores for the guide rods, from the boss's outer face
        for (x = ROD_X) ybore(x, RAIL_OUT + BOSS_L - ROD_EMBED, RAIL_OUT + BOSS_L + 1, ROD_Z, ROD_D + PRESS_CLR);
    }
}

translate([0, 0, -RAIL_Z0]) sprung_rail();
