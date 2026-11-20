// Two-track peristaltic bench head. Dimensions are millimetres.
// Export each named part to STL with -D 'part="name"'.
part = "assembly"; // [assembly,housing,retainer,rotor_web,roller_spacer,cam,shoe,lever,stop_flag,stop_plate,motor_mount,rest]
$fn = 48;

rotor_z = 49;
pivot_x = 48;
pivot_z = 99;
occlusion_radius = 31.2;
tracks = [-9, 9];

module box(size, center) {
    translate(center) cube(size, center=true);
}

module bore(diameter, length, center, axis="Z") {
    translate(center)
        rotate(axis == "X" ? [0, 90, 0] : axis == "Y" ? [90, 0, 0] : [0, 0, 0])
            cylinder(d=diameter, h=length, center=true);
}

module housing() {
    difference() {
        union() {
            box([102, 101, 56], [0, 0, 28]);
            for (side = [-1, 1]) {
                // Fixed cam bearing ears and extension-spring anchors.
                box([14, 7, 55], [48, side*45, 79.5]);
                hull() {
                    box([14, 7, 5], [44, side*45, 54]);
                    box([12, 7, 7], [48, side*45, 83]);
                }
                hull() {
                    box([9, 7, 12], [48, side*45, 76]);
                    box([9, 7, 12], [73, side*45, 76]);
                }
            }
        }
        bore(65, 59.4, [0, 0, rotor_z], "X");
        bore(8.6, 105, [0, 0, rotor_z], "X");
        for (side = [-1, 1]) {
            // 608 bearing seat accessible from each of the two shaft ends.
            bore(22.15, 7.2, [side*34.8, 0, rotor_z], "X");
            bore(23, 13.3, [side*44.4, 0, rotor_z], "X");
            bore(8.2, 8, [pivot_x, side*45, pivot_z], "Y");
            bore(3.4, 8, [73, side*45, 76], "Y");
            for (z = [76, 91])
                bore(3.2, 8, [50, side*45, z], "Y");
            for (x = [-18, 18])
                bore(3, 12, [x, side*46, 54]);
            bore(4.4, 57, [-41, side*37, 28]);
        }
    }
}

module retainer() {
    difference() {
        union() {
            box([49, 11, 21], [0, 0, 10.5]);
            for (x = [-18, 0, 18]) {
                width = x == 0 ? 5 : 4;
                box([width, 11, 10], [x, 0, 25.5]);
                hull() {
                    box([width, 11, 0.5], [x, 0, 30.4]);
                    box([width - 2, 9, 0.5], [x, 0, 33]);
                }
            }
        }
        for (x = [-18, 18]) {
            bore(3.4, 36, [x, 0, 16.5]);
            bore(6.1, 3.4, [x, 0, 19.4]);
        }
    }
}

module rotor_web() {
    difference() {
        bore(54, 5, [0, 0, 0], "X");
        intersection() {
            bore(8.2, 6, [0, 0, 0], "X");
            box([7, 8.6, 10], [0, -1.0, 0]);
        }
        for (a = [0:90:270])
            bore(5.2, 6, [0, 19*sin(a), 19*cos(a)], "X");
    }
}

module roller_spacer() {
    difference() {
        bore(25, 3.7, [0, 0, 0], "X");
        bore(18.15, 4, [0, 0, 0], "X");
    }
}

// Coordinates are relative to the pivot when the cam is closed.
module cam() {
    difference() {
        union() {
            box([59, 78, 8], [-35, 0, -9]);
            box([14, 81, 13], [-6.8, 0, -3]);
            bore(14, 81, [0, 0, 0], "Y");
            for (side = [-1, 1])
                hull() {
                    box([9, 8, 3], [-31, side*36, -6]);
                    box([9, 8, 7], [-31, side*36, 13]);
                }

        }
        bore(8.15, 84, [0, 0, 0], "Y");
        for (x = tracks, y = [-18, 18]) {
            bore(3.4, 12, [x-pivot_x, y, -9]);
            bore(6.7, 2.5, [x-pivot_x, y, -12.1]);
        }
        bore(3.2, 8, [0, 28, 4]);
        for (side = [-1, 1])
            bore(3.2, 10, [-31, side*36, 13], "Y");
    }
}

module shoe() {
    // A 116-degree track keeps at least one of four rollers in the occlusion arc.
    difference() {
        union() {
            intersection() {
                difference() {
                    bore(2*(occlusion_radius+5), 13, [0, 0, 0], "X");
                    bore(2*occlusion_radius, 14, [0, 0, 0], "X");
                }
                translate([-7, 0, 0])
                    rotate([90, 0, 90])
                        linear_extrude(height=14)
                            polygon([[0, 0], [-44, 44/tan(58)], [-44, 45],
                                     [44, 45], [44, 44/tan(58)]]);
            }
            for (y = [-18, 18])
                hull() {
                    box([9, 7, 2], [0, y, 29.2]);
                    box([9, 7, 3], [0, y, 30.5]);
                }
        }
        for (y = [-18, 18])
            bore(2.7, 6, [0, y, 32]);
    }
}


module shaft_arm(tip_x, tip_z, tip_diameter, pin_diameter) {
    difference() {
        hull() {
            bore(16, 8, [0, 0, 0], "Y");
            bore(tip_diameter, 8, [tip_x, 0, tip_z], "Y");
        }
        bore(8.15, 9, [0, 0, 0], "Y");
        bore(pin_diameter, 9, [tip_x, 0, tip_z], "Y");
        bore(3.2, 7, [0, 0, 6]);
    }
}

module lever() {
    shaft_arm(-20, -20, 12, 5.3);
}

module stop_flag() {
    shaft_arm(0, -18, 9, 3.3);
}

module stop_plate() {
    difference() {
        box([38, 5, 33], [-9, 0, -13]);
        bore(12, 6, [0, 0, 0], "Y");
        for (z = [-23, -8])
            bore(3.4, 7, [2, 0, z], "Y");
        // The follower contacts this slot's ends at the two operating stops.
        for (a = [0:2:68])
            hull() {
                bore(4.3, 6, [-18*sin(a), 0, -18*cos(a)], "Y");
                bore(4.3, 6, [-18*sin(a+2), 0, -18*cos(a+2)], "Y");
            }
    }
}

module motor_mount() {
    difference() {
        union() {
            box([53, 56, 5], [0, 0, 2.5]);
            box([6, 48, 79], [23.5, 0, 43.5]);
            for (y = [-23, 23])
                hull() {
                    box([4, 4, 3], [9, y, 5.5]);
                    box([4, 4, 15], [21, y, 12]);
                }
        }
        bore(22.6, 8, [23.5, 0, rotor_z], "X");
        for (y = [-15.5, 15.5], z = [-15.5, 15.5])
            bore(3.5, 8, [23.5, y, rotor_z+z], "X");
        for (x = [-20, 15], y = [-20, 20])
            bore(4.4, 6, [x, y, 2.5]);
    }
}

module rest() {
    union() {
        box([90, 16, 10], [0, 0, 84]);
        box([22, 14, 80], [0, 0, 40]);
        box([34, 30, 3], [0, 0, 1.5]);
    }
}

module assembly() {
    color("DimGray") housing();
    for (side = [-1, 1]) {
        translate([0, side*46, 56]) color("Gray") retainer();
        translate([side*26.2, 0, rotor_z]) color("Gray") rotor_web();
        translate([-13, side == -1 ? -137 : 126, 0]) rest();
    }
    for (a = [0:90:270]) {
        p = [0, 19*sin(a), rotor_z+19*cos(a)];
        color("Silver") bore(18, 47, p, "X");
        for (x = [-18, 0, 18])
            translate([x, p[1], p[2]]) color("Gray") roller_spacer();
    }
    color("Silver") bore(8, 145, [7.5, 0, rotor_z], "X");
    color("Silver") bore(8, 180, [pivot_x, -25, pivot_z], "Y");
    translate([pivot_x, 0, pivot_z]) rotate([0, 70, 0]) {
        color("DarkSlateGray") cam();
        for (x = tracks)
            translate([x-pivot_x, 0, rotor_z-pivot_z])
                color("SlateGray") shoe();
        translate([0, -110, 0]) color("Goldenrod") lever();
        color("Goldenrod") bore(19, 19, [-20, -110, -20], "Y");
        translate([0, 53, 0]) stop_flag();
    }
    translate([pivot_x, 60, pivot_z]) stop_plate();
    translate([80, 0, 0]) motor_mount();
}

if (part == "assembly") assembly();
if (part == "housing") housing();
if (part == "retainer") retainer();
if (part == "rotor_web") rotor_web();
if (part == "roller_spacer") roller_spacer();
if (part == "cam") cam();
if (part == "shoe") shoe();
if (part == "lever") lever();
if (part == "stop_flag") stop_flag();
if (part == "stop_plate") stop_plate();
if (part == "motor_mount") motor_mount();
if (part == "rest") rest();
