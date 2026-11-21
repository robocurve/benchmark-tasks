// Millimetres. Inert training fixture, dimensional assembly reference.
// Select "assembly", "frame", "lever", "follower", "spare" (intact) or "reserve" (pierced).
part = "assembly";

module installed() {
  color([0.76, 0.77, 0.75]) translate([0, 0, 0]) rotate([0,0,0]) rotate_extrude($fn=48) polygon([[0, 0], [10, 0], [12.7, 4], [12.7, 103], [12.2, 109], [10, 116], [6, 122], [5.2, 124], [5.2, 139], [3.3, 139], [1, 139], [1, 136.199997], [0, 136.199997]]); // metal
  color([0.42, 0.42, 0.42]) translate([-12.7, 0, 95]) rotate([0,0,0]) cube([0.45, 2, 15],center=true); // index_b
  color([0.42, 0.42, 0.42]) translate([12.7, 0, 95]) rotate([0,0,0]) cube([0.45, 2, 15],center=true); // index_a
  color([0.42, 0.42, 0.42]) translate([0, 0, 0]) rotate([0,0,0]) rotate_extrude($fn=48) polygon([[5.3, 124], [7.3, 125], [7.3, 133], [5.3, 134]]); // bayonet
  color([0.42, 0.42, 0.42]) translate([0, 0, 0]) rotate([0,0,0]) rotate_extrude($fn=48) polygon([[5.2, 134], [5.8, 134.5], [5.8, 135.5], [5.2, 136]]); // seal_ring
  color([0.42, 0.42, 0.42]) translate([6.25, 0, 134.5]) rotate([0,0,0]) cube([1.4, 1.4, 5],center=true); // ring_web_0
  color([0.42, 0.42, 0.42]) translate([-3.125, 5.412659, 134.5]) rotate([0,0,0]) cube([1.4, 1.4, 5],center=true); // ring_web_1
  color([0.42, 0.42, 0.42]) translate([-3.125, -5.412659, 134.5]) rotate([0,0,0]) cube([1.4, 1.4, 5],center=true); // ring_web_2
  color([0.42, 0.42, 0.42]) translate([6.5, 0, 137]) rotate([0,0,0]) cube([1.6, 1.79, 1.5],center=true); // ring_land_0
  color([0.42, 0.42, 0.42]) translate([6.278518, 1.682324, 137]) rotate([0,0,15]) cube([1.6, 1.79, 1.5],center=true); // ring_land_1
  color([0.42, 0.42, 0.42]) translate([5.629165, 3.25, 137]) rotate([0,0,30]) cube([1.6, 1.79, 1.5],center=true); // ring_land_2
  color([0.42, 0.42, 0.42]) translate([4.596194, 4.596194, 137]) rotate([0,0,45]) cube([1.6, 1.79, 1.5],center=true); // ring_land_3
  color([0.42, 0.42, 0.42]) translate([3.25, 5.629165, 137]) rotate([0,0,60]) cube([1.6, 1.79, 1.5],center=true); // ring_land_4
  color([0.42, 0.42, 0.42]) translate([1.682324, 6.278518, 137]) rotate([0,0,75]) cube([1.6, 1.79, 1.5],center=true); // ring_land_5
  color([0.42, 0.42, 0.42]) translate([0, 6.5, 137]) rotate([0,0,90]) cube([1.6, 1.79, 1.5],center=true); // ring_land_6
  color([0.42, 0.42, 0.42]) translate([-1.682324, 6.278518, 137]) rotate([0,0,105]) cube([1.6, 1.79, 1.5],center=true); // ring_land_7
  color([0.42, 0.42, 0.42]) translate([-3.25, 5.629165, 137]) rotate([0,0,120]) cube([1.6, 1.79, 1.5],center=true); // ring_land_8
  color([0.42, 0.42, 0.42]) translate([-4.596194, 4.596194, 137]) rotate([0,0,135]) cube([1.6, 1.79, 1.5],center=true); // ring_land_9
  color([0.42, 0.42, 0.42]) translate([-5.629165, 3.25, 137]) rotate([0,0,150]) cube([1.6, 1.79, 1.5],center=true); // ring_land_10
  color([0.42, 0.42, 0.42]) translate([-6.278518, 1.682324, 137]) rotate([0,0,165]) cube([1.6, 1.79, 1.5],center=true); // ring_land_11
  color([0.42, 0.42, 0.42]) translate([-6.5, 0, 137]) rotate([0,0,180]) cube([1.6, 1.79, 1.5],center=true); // ring_land_12
  color([0.42, 0.42, 0.42]) translate([-6.278518, -1.682324, 137]) rotate([0,0,195]) cube([1.6, 1.79, 1.5],center=true); // ring_land_13
  color([0.42, 0.42, 0.42]) translate([-5.629165, -3.25, 137]) rotate([0,0,210]) cube([1.6, 1.79, 1.5],center=true); // ring_land_14
  color([0.42, 0.42, 0.42]) translate([-4.596194, -4.596194, 137]) rotate([0,0,225]) cube([1.6, 1.79, 1.5],center=true); // ring_land_15
  color([0.42, 0.42, 0.42]) translate([-3.25, -5.629165, 137]) rotate([0,0,240]) cube([1.6, 1.79, 1.5],center=true); // ring_land_16
  color([0.42, 0.42, 0.42]) translate([-1.682324, -6.278518, 137]) rotate([0,0,255]) cube([1.6, 1.79, 1.5],center=true); // ring_land_17
  color([0.42, 0.42, 0.42]) translate([-0, -6.5, 137]) rotate([0,0,270]) cube([1.6, 1.79, 1.5],center=true); // ring_land_18
  color([0.42, 0.42, 0.42]) translate([1.682324, -6.278518, 137]) rotate([0,0,285]) cube([1.6, 1.79, 1.5],center=true); // ring_land_19
  color([0.42, 0.42, 0.42]) translate([3.25, -5.629165, 137]) rotate([0,0,300]) cube([1.6, 1.79, 1.5],center=true); // ring_land_20
  color([0.42, 0.42, 0.42]) translate([4.596194, -4.596194, 137]) rotate([0,0,315]) cube([1.6, 1.79, 1.5],center=true); // ring_land_21
  color([0.42, 0.42, 0.42]) translate([5.629165, -3.25, 137]) rotate([0,0,330]) cube([1.6, 1.79, 1.5],center=true); // ring_land_22
  color([0.42, 0.42, 0.42]) translate([6.278518, -1.682324, 137]) rotate([0,0,345]) cube([1.6, 1.79, 1.5],center=true); // ring_land_23
  color([0.42, 0.42, 0.42]) translate([-8.8, 0, 130]) rotate([0,0,0]) cube([5.8, 4, 3],center=true); // lug_0
  color([0.42, 0.42, 0.42]) translate([8.8, 0, 130]) rotate([0,0,0]) cube([5.8, 4, 3],center=true); // lug_1
}
module spare() {
  color([0.76, 0.77, 0.75]) translate([0, 0, 0]) rotate([0,0,0]) rotate_extrude($fn=48) polygon([[0, 0], [10, 0], [12.7, 4], [12.7, 103], [12.2, 109], [10, 116], [6, 122], [5.2, 124], [5.2, 139], [3.3, 139], [0, 139]]); // metal
  color([0.42, 0.42, 0.42]) translate([-12.7, 0, 95]) rotate([0,0,0]) cube([0.45, 2, 15],center=true); // index_b
  color([0.42, 0.42, 0.42]) translate([12.7, 0, 95]) rotate([0,0,0]) cube([0.45, 2, 15],center=true); // index_a
  color([0.42, 0.42, 0.42]) translate([0, 0, 0]) rotate([0,0,0]) rotate_extrude($fn=48) polygon([[5.3, 124], [7.3, 125], [7.3, 133], [5.3, 134]]); // bayonet
  color([0.42, 0.42, 0.42]) translate([0, 0, 0]) rotate([0,0,0]) rotate_extrude($fn=48) polygon([[5.2, 134], [5.8, 134.5], [5.8, 135.5], [5.2, 136]]); // seal_ring
  color([0.42, 0.42, 0.42]) translate([6.25, 0, 137.5]) rotate([0,0,0]) cube([1.4, 1.4, 11],center=true); // ring_web_0
  color([0.42, 0.42, 0.42]) translate([-3.125, 5.412659, 137.5]) rotate([0,0,0]) cube([1.4, 1.4, 11],center=true); // ring_web_1
  color([0.42, 0.42, 0.42]) translate([-3.125, -5.412659, 137.5]) rotate([0,0,0]) cube([1.4, 1.4, 11],center=true); // ring_web_2
  color([0.42, 0.42, 0.42]) translate([6.5, 0, 143]) rotate([0,0,0]) cube([1.6, 1.79, 1.5],center=true); // ring_land_0
  color([0.42, 0.42, 0.42]) translate([6.278518, 1.682324, 143]) rotate([0,0,15]) cube([1.6, 1.79, 1.5],center=true); // ring_land_1
  color([0.42, 0.42, 0.42]) translate([5.629165, 3.25, 143]) rotate([0,0,30]) cube([1.6, 1.79, 1.5],center=true); // ring_land_2
  color([0.42, 0.42, 0.42]) translate([4.596194, 4.596194, 143]) rotate([0,0,45]) cube([1.6, 1.79, 1.5],center=true); // ring_land_3
  color([0.42, 0.42, 0.42]) translate([3.25, 5.629165, 143]) rotate([0,0,60]) cube([1.6, 1.79, 1.5],center=true); // ring_land_4
  color([0.42, 0.42, 0.42]) translate([1.682324, 6.278518, 143]) rotate([0,0,75]) cube([1.6, 1.79, 1.5],center=true); // ring_land_5
  color([0.42, 0.42, 0.42]) translate([0, 6.5, 143]) rotate([0,0,90]) cube([1.6, 1.79, 1.5],center=true); // ring_land_6
  color([0.42, 0.42, 0.42]) translate([-1.682324, 6.278518, 143]) rotate([0,0,105]) cube([1.6, 1.79, 1.5],center=true); // ring_land_7
  color([0.42, 0.42, 0.42]) translate([-3.25, 5.629165, 143]) rotate([0,0,120]) cube([1.6, 1.79, 1.5],center=true); // ring_land_8
  color([0.42, 0.42, 0.42]) translate([-4.596194, 4.596194, 143]) rotate([0,0,135]) cube([1.6, 1.79, 1.5],center=true); // ring_land_9
  color([0.42, 0.42, 0.42]) translate([-5.629165, 3.25, 143]) rotate([0,0,150]) cube([1.6, 1.79, 1.5],center=true); // ring_land_10
  color([0.42, 0.42, 0.42]) translate([-6.278518, 1.682324, 143]) rotate([0,0,165]) cube([1.6, 1.79, 1.5],center=true); // ring_land_11
  color([0.42, 0.42, 0.42]) translate([-6.5, 0, 143]) rotate([0,0,180]) cube([1.6, 1.79, 1.5],center=true); // ring_land_12
  color([0.42, 0.42, 0.42]) translate([-6.278518, -1.682324, 143]) rotate([0,0,195]) cube([1.6, 1.79, 1.5],center=true); // ring_land_13
  color([0.42, 0.42, 0.42]) translate([-5.629165, -3.25, 143]) rotate([0,0,210]) cube([1.6, 1.79, 1.5],center=true); // ring_land_14
  color([0.42, 0.42, 0.42]) translate([-4.596194, -4.596194, 143]) rotate([0,0,225]) cube([1.6, 1.79, 1.5],center=true); // ring_land_15
  color([0.42, 0.42, 0.42]) translate([-3.25, -5.629165, 143]) rotate([0,0,240]) cube([1.6, 1.79, 1.5],center=true); // ring_land_16
  color([0.42, 0.42, 0.42]) translate([-1.682324, -6.278518, 143]) rotate([0,0,255]) cube([1.6, 1.79, 1.5],center=true); // ring_land_17
  color([0.42, 0.42, 0.42]) translate([-0, -6.5, 143]) rotate([0,0,270]) cube([1.6, 1.79, 1.5],center=true); // ring_land_18
  color([0.42, 0.42, 0.42]) translate([1.682324, -6.278518, 143]) rotate([0,0,285]) cube([1.6, 1.79, 1.5],center=true); // ring_land_19
  color([0.42, 0.42, 0.42]) translate([3.25, -5.629165, 143]) rotate([0,0,300]) cube([1.6, 1.79, 1.5],center=true); // ring_land_20
  color([0.42, 0.42, 0.42]) translate([4.596194, -4.596194, 143]) rotate([0,0,315]) cube([1.6, 1.79, 1.5],center=true); // ring_land_21
  color([0.42, 0.42, 0.42]) translate([5.629165, -3.25, 143]) rotate([0,0,330]) cube([1.6, 1.79, 1.5],center=true); // ring_land_22
  color([0.42, 0.42, 0.42]) translate([6.278518, -1.682324, 143]) rotate([0,0,345]) cube([1.6, 1.79, 1.5],center=true); // ring_land_23
  color([0.42, 0.42, 0.42]) translate([-8.8, 0, 130]) rotate([0,0,0]) cube([5.8, 4, 3],center=true); // lug_0
  color([0.42, 0.42, 0.42]) translate([8.8, 0, 130]) rotate([0,0,0]) cube([5.8, 4, 3],center=true); // lug_1
}
module reserve() {
  color([0.76, 0.77, 0.75]) translate([0, 0, 0]) rotate([0,0,0]) rotate_extrude($fn=48) polygon([[0, 0], [10, 0], [12.7, 4], [12.7, 103], [12.2, 109], [10, 116], [6, 122], [5.2, 124], [5.2, 139], [3.3, 139], [1, 139], [1, 136.199997], [0, 136.199997]]); // metal
  color([0.42, 0.42, 0.42]) translate([-12.7, 0, 95]) rotate([0,0,0]) cube([0.45, 2, 15],center=true); // index_b
  color([0.42, 0.42, 0.42]) translate([12.7, 0, 95]) rotate([0,0,0]) cube([0.45, 2, 15],center=true); // index_a
  color([0.42, 0.42, 0.42]) translate([0, 0, 0]) rotate([0,0,0]) rotate_extrude($fn=48) polygon([[5.3, 124], [7.3, 125], [7.3, 133], [5.3, 134]]); // bayonet
  color([0.42, 0.42, 0.42]) translate([0, 0, 0]) rotate([0,0,0]) rotate_extrude($fn=48) polygon([[5.2, 134], [5.8, 134.5], [5.8, 135.5], [5.2, 136]]); // seal_ring
  color([0.42, 0.42, 0.42]) translate([6.25, 0, 134.5]) rotate([0,0,0]) cube([1.4, 1.4, 5],center=true); // ring_web_0
  color([0.42, 0.42, 0.42]) translate([-3.125, 5.412659, 134.5]) rotate([0,0,0]) cube([1.4, 1.4, 5],center=true); // ring_web_1
  color([0.42, 0.42, 0.42]) translate([-3.125, -5.412659, 134.5]) rotate([0,0,0]) cube([1.4, 1.4, 5],center=true); // ring_web_2
  color([0.42, 0.42, 0.42]) translate([6.5, 0, 137]) rotate([0,0,0]) cube([1.6, 1.79, 1.5],center=true); // ring_land_0
  color([0.42, 0.42, 0.42]) translate([6.278518, 1.682324, 137]) rotate([0,0,15]) cube([1.6, 1.79, 1.5],center=true); // ring_land_1
  color([0.42, 0.42, 0.42]) translate([5.629165, 3.25, 137]) rotate([0,0,30]) cube([1.6, 1.79, 1.5],center=true); // ring_land_2
  color([0.42, 0.42, 0.42]) translate([4.596194, 4.596194, 137]) rotate([0,0,45]) cube([1.6, 1.79, 1.5],center=true); // ring_land_3
  color([0.42, 0.42, 0.42]) translate([3.25, 5.629165, 137]) rotate([0,0,60]) cube([1.6, 1.79, 1.5],center=true); // ring_land_4
  color([0.42, 0.42, 0.42]) translate([1.682324, 6.278518, 137]) rotate([0,0,75]) cube([1.6, 1.79, 1.5],center=true); // ring_land_5
  color([0.42, 0.42, 0.42]) translate([0, 6.5, 137]) rotate([0,0,90]) cube([1.6, 1.79, 1.5],center=true); // ring_land_6
  color([0.42, 0.42, 0.42]) translate([-1.682324, 6.278518, 137]) rotate([0,0,105]) cube([1.6, 1.79, 1.5],center=true); // ring_land_7
  color([0.42, 0.42, 0.42]) translate([-3.25, 5.629165, 137]) rotate([0,0,120]) cube([1.6, 1.79, 1.5],center=true); // ring_land_8
  color([0.42, 0.42, 0.42]) translate([-4.596194, 4.596194, 137]) rotate([0,0,135]) cube([1.6, 1.79, 1.5],center=true); // ring_land_9
  color([0.42, 0.42, 0.42]) translate([-5.629165, 3.25, 137]) rotate([0,0,150]) cube([1.6, 1.79, 1.5],center=true); // ring_land_10
  color([0.42, 0.42, 0.42]) translate([-6.278518, 1.682324, 137]) rotate([0,0,165]) cube([1.6, 1.79, 1.5],center=true); // ring_land_11
  color([0.42, 0.42, 0.42]) translate([-6.5, 0, 137]) rotate([0,0,180]) cube([1.6, 1.79, 1.5],center=true); // ring_land_12
  color([0.42, 0.42, 0.42]) translate([-6.278518, -1.682324, 137]) rotate([0,0,195]) cube([1.6, 1.79, 1.5],center=true); // ring_land_13
  color([0.42, 0.42, 0.42]) translate([-5.629165, -3.25, 137]) rotate([0,0,210]) cube([1.6, 1.79, 1.5],center=true); // ring_land_14
  color([0.42, 0.42, 0.42]) translate([-4.596194, -4.596194, 137]) rotate([0,0,225]) cube([1.6, 1.79, 1.5],center=true); // ring_land_15
  color([0.42, 0.42, 0.42]) translate([-3.25, -5.629165, 137]) rotate([0,0,240]) cube([1.6, 1.79, 1.5],center=true); // ring_land_16
  color([0.42, 0.42, 0.42]) translate([-1.682324, -6.278518, 137]) rotate([0,0,255]) cube([1.6, 1.79, 1.5],center=true); // ring_land_17
  color([0.42, 0.42, 0.42]) translate([-0, -6.5, 137]) rotate([0,0,270]) cube([1.6, 1.79, 1.5],center=true); // ring_land_18
  color([0.42, 0.42, 0.42]) translate([1.682324, -6.278518, 137]) rotate([0,0,285]) cube([1.6, 1.79, 1.5],center=true); // ring_land_19
  color([0.42, 0.42, 0.42]) translate([3.25, -5.629165, 137]) rotate([0,0,300]) cube([1.6, 1.79, 1.5],center=true); // ring_land_20
  color([0.42, 0.42, 0.42]) translate([4.596194, -4.596194, 137]) rotate([0,0,315]) cube([1.6, 1.79, 1.5],center=true); // ring_land_21
  color([0.42, 0.42, 0.42]) translate([5.629165, -3.25, 137]) rotate([0,0,330]) cube([1.6, 1.79, 1.5],center=true); // ring_land_22
  color([0.42, 0.42, 0.42]) translate([6.278518, -1.682324, 137]) rotate([0,0,345]) cube([1.6, 1.79, 1.5],center=true); // ring_land_23
  color([0.42, 0.42, 0.42]) translate([-8.8, 0, 130]) rotate([0,0,0]) cube([5.8, 4, 3],center=true); // lug_0
  color([0.42, 0.42, 0.42]) translate([8.8, 0, 130]) rotate([0,0,0]) cube([5.8, 4, 3],center=true); // lug_1
}
module lever() {
  color([0.42, 0.42, 0.42]) translate([16.5, 0, -0.5]) rotate([0,0,0]) cube([2.2, 3.4, 2.6],center=true); // toe
  color([0.42, 0.42, 0.42]) translate([8.5, 0, 1.1]) rotate([0,0,0]) cube([17, 3.4, 2],center=true); // nose_arm
  color([0.42, 0.42, 0.42]) translate([0, 31, 1]) rotate([0,0,0]) cube([4, 65, 4],center=true); // crossbar
  color([0.42, 0.42, 0.42]) translate([27, 61, 1]) rotate([0,0,0]) cube([55, 7, 4],center=true); // handle
  color([0.42, 0.42, 0.42]) translate([53, 61, 1]) rotate([0,0,0]) cube([20, 14, 4],center=true); // paddle
}
module follower() {
  color([0.42, 0.42, 0.42]) translate([0, -6.45, 2]) rotate([0,0,0]) cube([1.5, 1.4, 4],center=true); // foot_0
  color([0.42, 0.42, 0.42]) translate([0, 6.45, 2]) rotate([0,0,0]) cube([1.5, 1.4, 4],center=true); // foot_1
  color([0.42, 0.42, 0.42]) translate([0, 0, 4.8]) rotate([0,0,0]) cube([2, 14.4, 1.6],center=true); // bridge
  color([0.42, 0.42, 0.42]) translate([10, 0, 5]) rotate([0,0,0]) cube([20, 3, 1.6],center=true); // arm
  color([0.82, 0.1, 0.08]) translate([20, 0, 6.6]) rotate([0,0,0]) cube([2, 12, 3.6],center=true); // red
  color([0.1, 0.68, 0.25]) translate([20, 0, 2.9]) rotate([0,0,0]) cube([2, 12, 3.7],center=true); // green
}
module frame() {
  color([0.91, 0.49, 0.08]) translate([0, 0, 2]) rotate([0,0,0]) cube([240, 195, 4],center=true); // service_panel
  color([0.15, 0.17, 0.17]) translate([0, 0, 4.8]) rotate([0,0,0]) cube([50, 290, 1.5],center=true); // waist_webbing
  color([0.42, 0.42, 0.42]) translate([36, 0, 10]) rotate([0,0,0]) cube([70, 74, 10],center=true); // trainer_base
  color([0.42, 0.42, 0.42]) translate([48, 0, 101.25]) rotate([0,0,0]) cube([14, 43, 172.5],center=true); // trainer_post
  color([0.42, 0.42, 0.42]) translate([31, 0, 188.5]) rotate([0,0,0]) cube([32, 37, 14],center=true); // socket_back
  color([0.42, 0.42, 0.42]) translate([8.768124, 8.768124, 187.5]) rotate([0,0,45]) cube([9, 2.7, 6],center=true); // bay_shelf_4
  color([0.42, 0.42, 0.42]) translate([6.889071, 10.310223, 187.5]) rotate([0,0,56.25]) cube([9, 2.7, 6],center=true); // bay_shelf_5
  color([0.42, 0.42, 0.42]) translate([4.745275, 11.456106, 187.5]) rotate([0,0,67.5]) cube([9, 2.7, 6],center=true); // bay_shelf_6
  color([0.42, 0.42, 0.42]) translate([2.41912, 12.161737, 187.5]) rotate([0,0,78.75]) cube([9, 2.7, 6],center=true); // bay_shelf_7
  color([0.42, 0.42, 0.42]) translate([0, 12.4, 187.5]) rotate([0,0,90]) cube([9, 2.7, 6],center=true); // bay_shelf_8
  color([0.42, 0.42, 0.42]) translate([-2.41912, 12.161737, 187.5]) rotate([0,0,101.25]) cube([9, 2.7, 6],center=true); // bay_shelf_9
  color([0.42, 0.42, 0.42]) translate([-4.745275, 11.456106, 187.5]) rotate([0,0,112.5]) cube([9, 2.7, 6],center=true); // bay_shelf_10
  color([0.42, 0.42, 0.42]) translate([-6.889071, 10.310223, 187.5]) rotate([0,0,123.75]) cube([9, 2.7, 6],center=true); // bay_shelf_11
  color([0.42, 0.42, 0.42]) translate([-8.768124, 8.768124, 187.5]) rotate([0,0,135]) cube([9, 2.7, 6],center=true); // bay_shelf_12
  color([0.42, 0.42, 0.42]) translate([-8.768124, -8.768124, 187.5]) rotate([0,0,225]) cube([9, 2.7, 6],center=true); // bay_shelf_20
  color([0.42, 0.42, 0.42]) translate([-6.889071, -10.310223, 187.5]) rotate([0,0,236.25]) cube([9, 2.7, 6],center=true); // bay_shelf_21
  color([0.42, 0.42, 0.42]) translate([-4.745275, -11.456106, 187.5]) rotate([0,0,247.5]) cube([9, 2.7, 6],center=true); // bay_shelf_22
  color([0.42, 0.42, 0.42]) translate([-2.41912, -12.161737, 187.5]) rotate([0,0,258.75]) cube([9, 2.7, 6],center=true); // bay_shelf_23
  color([0.42, 0.42, 0.42]) translate([0, -12.4, 187.5]) rotate([0,0,270]) cube([9, 2.7, 6],center=true); // bay_shelf_24
  color([0.42, 0.42, 0.42]) translate([2.41912, -12.161737, 187.5]) rotate([0,0,281.25]) cube([9, 2.7, 6],center=true); // bay_shelf_25
  color([0.42, 0.42, 0.42]) translate([4.745275, -11.456106, 187.5]) rotate([0,0,292.5]) cube([9, 2.7, 6],center=true); // bay_shelf_26
  color([0.42, 0.42, 0.42]) translate([6.889071, -10.310223, 187.5]) rotate([0,0,303.75]) cube([9, 2.7, 6],center=true); // bay_shelf_27
  color([0.42, 0.42, 0.42]) translate([8.768124, -8.768124, 187.5]) rotate([0,0,315]) cube([9, 2.7, 6],center=true); // bay_shelf_28
  color([0.42, 0.42, 0.42]) translate([15.8, 0, 188.5]) rotate([0,0,0]) cube([3.4, 3.2, 21],center=true); // socket_wall_0
  color([0.42, 0.42, 0.42]) translate([15.496407, 3.082427, 188.5]) rotate([0,0,11.25]) cube([3.4, 3.2, 21],center=true); // socket_wall_1
  color([0.42, 0.42, 0.42]) translate([14.597297, 6.046398, 188.5]) rotate([0,0,22.5]) cube([3.4, 3.2, 21],center=true); // socket_wall_2
  color([0.42, 0.42, 0.42]) translate([13.13722, 8.77801, 188.5]) rotate([0,0,33.75]) cube([3.4, 3.2, 21],center=true); // socket_wall_3
  color([0.42, 0.42, 0.42]) translate([11.172287, 11.172287, 188.5]) rotate([0,0,45]) cube([3.4, 3.2, 21],center=true); // socket_wall_4
  color([0.42, 0.42, 0.42]) translate([8.77801, 13.13722, 188.5]) rotate([0,0,56.25]) cube([3.4, 3.2, 21],center=true); // socket_wall_5
  color([0.42, 0.42, 0.42]) translate([6.046398, 14.597297, 188.5]) rotate([0,0,67.5]) cube([3.4, 3.2, 21],center=true); // socket_wall_6
  color([0.42, 0.42, 0.42]) translate([3.082427, 15.496407, 188.5]) rotate([0,0,78.75]) cube([3.4, 3.2, 21],center=true); // socket_wall_7
  color([0.42, 0.42, 0.42]) translate([0, 15.8, 188.5]) rotate([0,0,90]) cube([3.4, 3.2, 21],center=true); // socket_wall_8
  color([0.42, 0.42, 0.42]) translate([-3.082427, 15.496407, 188.5]) rotate([0,0,101.25]) cube([3.4, 3.2, 21],center=true); // socket_wall_9
  color([0.42, 0.42, 0.42]) translate([-6.046398, 14.597297, 188.5]) rotate([0,0,112.5]) cube([3.4, 3.2, 21],center=true); // socket_wall_10
  color([0.42, 0.42, 0.42]) translate([-8.77801, 13.13722, 188.5]) rotate([0,0,123.75]) cube([3.4, 3.2, 21],center=true); // socket_wall_11
  color([0.42, 0.42, 0.42]) translate([-11.172287, 11.172287, 188.5]) rotate([0,0,135]) cube([3.4, 3.2, 21],center=true); // socket_wall_12
  color([0.42, 0.42, 0.42]) translate([-13.13722, 8.77801, 188.5]) rotate([0,0,146.25]) cube([3.4, 3.2, 21],center=true); // socket_wall_13
  color([0.42, 0.42, 0.42]) translate([-14.597297, 6.046398, 188.5]) rotate([0,0,157.5]) cube([3.4, 3.2, 21],center=true); // socket_wall_14
  color([0.42, 0.42, 0.42]) translate([-15.496407, 3.082427, 188.5]) rotate([0,0,168.75]) cube([3.4, 3.2, 21],center=true); // socket_wall_15
  color([0.42, 0.42, 0.42]) translate([-15.8, 0, 188.5]) rotate([0,0,180]) cube([3.4, 3.2, 21],center=true); // socket_wall_16
  color([0.42, 0.42, 0.42]) translate([-15.496407, -3.082427, 188.5]) rotate([0,0,191.25]) cube([3.4, 3.2, 21],center=true); // socket_wall_17
  color([0.42, 0.42, 0.42]) translate([-14.597297, -6.046398, 188.5]) rotate([0,0,202.5]) cube([3.4, 3.2, 21],center=true); // socket_wall_18
  color([0.42, 0.42, 0.42]) translate([-13.13722, -8.77801, 188.5]) rotate([0,0,213.75]) cube([3.4, 3.2, 21],center=true); // socket_wall_19
  color([0.42, 0.42, 0.42]) translate([-11.172287, -11.172287, 188.5]) rotate([0,0,225]) cube([3.4, 3.2, 21],center=true); // socket_wall_20
  color([0.42, 0.42, 0.42]) translate([-8.77801, -13.13722, 188.5]) rotate([0,0,236.25]) cube([3.4, 3.2, 21],center=true); // socket_wall_21
  color([0.42, 0.42, 0.42]) translate([-6.046398, -14.597297, 188.5]) rotate([0,0,247.5]) cube([3.4, 3.2, 21],center=true); // socket_wall_22
  color([0.42, 0.42, 0.42]) translate([-3.082427, -15.496407, 188.5]) rotate([0,0,258.75]) cube([3.4, 3.2, 21],center=true); // socket_wall_23
  color([0.42, 0.42, 0.42]) translate([0, -15.8, 188.5]) rotate([0,0,270]) cube([3.4, 3.2, 21],center=true); // socket_wall_24
  color([0.42, 0.42, 0.42]) translate([3.082427, -15.496407, 188.5]) rotate([0,0,281.25]) cube([3.4, 3.2, 21],center=true); // socket_wall_25
  color([0.42, 0.42, 0.42]) translate([6.046398, -14.597297, 188.5]) rotate([0,0,292.5]) cube([3.4, 3.2, 21],center=true); // socket_wall_26
  color([0.42, 0.42, 0.42]) translate([8.77801, -13.13722, 188.5]) rotate([0,0,303.75]) cube([3.4, 3.2, 21],center=true); // socket_wall_27
  color([0.42, 0.42, 0.42]) translate([11.172287, -11.172287, 188.5]) rotate([0,0,315]) cube([3.4, 3.2, 21],center=true); // socket_wall_28
  color([0.42, 0.42, 0.42]) translate([13.13722, -8.77801, 188.5]) rotate([0,0,326.25]) cube([3.4, 3.2, 21],center=true); // socket_wall_29
  color([0.42, 0.42, 0.42]) translate([14.597297, -6.046398, 188.5]) rotate([0,0,337.5]) cube([3.4, 3.2, 21],center=true); // socket_wall_30
  color([0.42, 0.42, 0.42]) translate([15.496407, -3.082427, 188.5]) rotate([0,0,348.75]) cube([3.4, 3.2, 21],center=true); // socket_wall_31
  color([0.42, 0.42, 0.42]) translate([17.8, 0, 202.5]) rotate([0,0,0]) cube([1, 17, 9],center=true); // window_lower
  color([0.42, 0.42, 0.42]) translate([17.8, 0, 214.9]) rotate([0,0,0]) cube([1, 17, 9],center=true); // window_upper
  color([0.42, 0.42, 0.42]) translate([17.8, -7.6, 208.7]) rotate([0,0,0]) cube([1, 2.5, 13],center=true); // window_side_0
  color([0.42, 0.42, 0.42]) translate([17.8, 7.6, 208.7]) rotate([0,0,0]) cube([1, 2.5, 13],center=true); // window_side_1
  color([0.42, 0.42, 0.42]) translate([-23, 94, 98]) rotate([0,0,0]) cube([12, 10, 196],center=true); // lever_mount
  color([0.42, 0.42, 0.42]) translate([-23, 79, 197]) rotate([0,0,0]) cube([10, 40, 6],center=true); // lever_pivot
  color([0.42, 0.42, 0.42]) translate([4.64952, 9.747921, 192.5]) rotate([0,0,64.5]) cube([4.8, 2, 8],center=true); // stop_a
  color([0.42, 0.42, 0.42]) translate([-4.64952, -9.747921, 192.5]) rotate([0,0,244.5]) cube([4.8, 2, 8],center=true); // stop_b
  color([0.42, 0.42, 0.42]) translate([30, -120, 4.5]) rotate([0,0,0]) cube([60, 50, 9],center=true); // parts_stand
  color([0.42, 0.42, 0.42]) translate([-132, 84, 9]) rotate([0,0,0]) cube([4, 54, 18],center=true); // cup_front
  color([0.42, 0.42, 0.42]) translate([-82, 84, 9]) rotate([0,0,0]) cube([4, 54, 18],center=true); // cup_back
  color([0.42, 0.42, 0.42]) translate([-107, 59, 9]) rotate([0,0,0]) cube([46, 4, 18],center=true); // cup_left
  color([0.42, 0.42, 0.42]) translate([-107, 109, 9]) rotate([0,0,0]) cube([46, 4, 18],center=true); // cup_right
  color([0.42, 0.42, 0.42]) translate([110, -197, 4.5]) rotate([0,0,0]) cube([60, 50, 9],center=true); // reserve_stand
  color([0.8, 0.15, 0.09]) translate([-186, -85, 9]) rotate([0,0,0]) cube([4, 170, 18],center=true); // hold_front
  color([0.8, 0.15, 0.09]) translate([-14, -85, 9]) rotate([0,0,0]) cube([4, 170, 18],center=true); // hold_back
  color([0.8, 0.15, 0.09]) translate([-100, -168, 9]) rotate([0,0,0]) cube([168, 4, 18],center=true); // hold_left
  color([0.8, 0.15, 0.09]) translate([-100, -2, 9]) rotate([0,0,0]) cube([168, 4, 18],center=true); // hold_right
  color([0.42, 0.42, 0.42]) translate([11.869107, 11.461863, 176]) rotate([0,0,0]) cube([3, 3, 19],center=true); // seat_index_back_0
  color([0.96, 0.85, 0.15]) translate([12.948116, 12.503851, 168.5]) rotate([0,0,0]) cube([2, 2, 10],center=true); // seat_index_0
  color([0.42, 0.42, 0.42]) translate([-11.869107, -11.461863, 176]) rotate([0,0,0]) cube([3, 3, 19],center=true); // seat_index_back_1
  color([0.96, 0.85, 0.15]) translate([-12.948116, -12.503851, 168.5]) rotate([0,0,0]) cube([2, 2, 10],center=true); // seat_index_1
}
if (part=="assembly") {
  frame();
  translate([0, 0, 62.5]) rotate([0,0,45]) installed();
  translate([30, -120, 9]) rotate([0,0,0]) spare();
  translate([110, -197, 9]) rotate([0,0,0]) reserve();
  translate([-23, 0, 204.5]) rotate([0,0,0]) lever();
  translate([0, 0, 202.1]) rotate([0,0,0]) follower();
}
if (part=="frame") frame();
if (part=="installed") installed();
if (part=="spare") spare();
if (part=="reserve") reserve();
if (part=="lever") lever();
if (part=="follower") follower();
