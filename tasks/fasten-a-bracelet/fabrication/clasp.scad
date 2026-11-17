// Units: mm. Part selector: female_base, female_cover, male, spring, assembly.
// BUILD.md specifies materials, tolerances, threads, forming and assembly.
part = "assembly";
$fn=64;
module box(lo,hi){translate(lo)cube(hi-lo);}
module hole_z(x,y,z,h,d){translate([x,y,z])cylinder(h=h,d=d);}
module female_blank(){
 difference(){
  box([0,-6,-3.5],[10,6,3.5]);
  // Mouth and bore; a small relief behind mouth accommodates spring stock.
  box([-.01,-2,-2],[7.8,2,2]);
  box([1.2,-2.06,-1.1],[7.8,2.06,1.1]);
  // Two spring anchors: M1.2 x 0.25 tapped from inside towards outside.
  for (side=[-1,1]) translate([7,side*2.04,0]) rotate([side==1?-90:90,0,0])
   cylinder(h=2.7,d=.95);
  // Rear leather slot, separate from socket by 0.2 mm wall.
  box([8,-5.1,-1.1],[10.01,5.1,1.1]);
  // Cover: four M1.6 x 0.35 screws. Counterbores keep all heads flush.
  for (x=[1.6,6]) for (y=[-4.5,4.5]) {
   hole_z(x,y,-2.8,6.4,1.25);
   hole_z(x,y,1.25,2.3,1.8);
   hole_z(x,y,2,1.6,2.9);
  }
  // Leather fastening: two M1.2 screws pierce hidden leather tabs.
  for (y=[-3,3]) {
   hole_z(8.75,y,-2.8,6.4,.95);
   hole_z(8.75,y,-1.1,4.7,1.3);
   hole_z(8.75,y,2.2,1.4,2.4);
  }
 }
}
module female_base(){intersection(){female_blank();box([-.1,-6.1,-3.6],[10.1,6.1,1.25]);}}
module female_cover(){intersection(){female_blank();box([-.1,-6.1,1.25],[10.1,6.1,3.6]);}}
module male(){
 difference(){
  union(){
   box([-10,-6,-3.5],[0,6,3.5]);
   box([0,-2.5,-2.5],[3,2.5,2.5]);
   box([3,-1.1,-1.1],[4.2,1.1,1.1]);
   translate([4.2,0,0])rotate([0,90,0])cylinder(h=1.4,d=1.6);
   box([5.6,-1.1,-1.1],[5.8,1.1,1.1]);
   hull(){box([5.799,-1.1,-1.1],[5.801,1.1,1.1]);box([5.999,-.8,-.8],[6,.8,.8]);}
  }
  box([-10.01,-5.1,-1.1],[-8,5.1,1.1]);
  for(y=[-3,3]){
   hole_z(-8.75,y,-2.8,6.4,.95);hole_z(-8.75,y,-1.1,4.7,1.3);hole_z(-8.75,y,2.2,1.4,2.4);
  }
 }
}
// Formed 0.0508 mm spring, two opposite-handed copies. Schematic bends
// have sharp intersections; use R0.15 forming radii and inspect final coordinates.
module spring(){
 difference(){
  union(){
   box([1.6,.8,-1],[2.1,.8508,1]);
   translate([2.1,.8,-1])linear_extrude(height=2)
    polygon([[0,0],[4.1,1.2],[4.1,1.2508],[0,.0508]]);
   box([6.2,2,-1],[7.8,2.0508,1]);
  }
  translate([7,1.95,0])rotate([-90,0,0])cylinder(h=.2,d=1.3);
 }
}
if(part=="female_base")female_base();
if(part=="female_cover")female_cover();
if(part=="male")male();
if(part=="spring")spring();
if(part=="assembly"){
 color("silver",.3)female_blank();
 color("gold")spring();color("gold")mirror([0,1,0])spring();
 // Seated male tip s=2.5: front-face gap = 3.5 mm.
 color("gray")translate([-3.5,0,0])male();
}
