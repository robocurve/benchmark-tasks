// Dimensions in mm. Print the unloaded teaching fixture in the indicated colors.
// Plate top is Z=0. The rotor axis is the XY origin. No spring mount is provided.
part="housing";
$fn=72;
pin=[-108,10.9]; tray=[-168,0]; shaft=8.2;
module zhole(x,y,z,h,d){translate([x,y,z]) cylinder(d=d,h=h);}
module block(x,y,z,dx,dy,dz){translate([x-dx/2,y-dy/2,z])cube([dx,dy,dz]);}
module housing(){
 difference(){
  union(){block(0,0,0,129,89,64);block(0,0,0,147,106,6);}
  zhole(0,0,-1,70,9.0);zhole(0,0,-1,8.2,22.15);zhole(0,0,56.8,8,22.15);
  for(x=[-67,67],y=[-45,45]) zhole(x,y,-1,9,5.5);
 }
}
module lever(){
 difference(){
  union(){
   cylinder(d=28,h=10);
   rotate([0,0,45])block(78,0,0,144,9,9);
   block(-57,0,-1,114,12,13);
  }
  zhole(0,0,-2,16,shaft);
  translate([0,-16,5]) rotate([-90,0,0]) cylinder(d=3.3,h=16);
 }
}
module crank(){
 difference(){
  union(){
   cylinder(d=27,h=11);
   block(10,0,0,12,302,10);
   for(y=[-151,151]) translate([10,y,0]) cylinder(d=20,h=10);
  }
  zhole(0,0,-1,14,shaft);
  translate([-14,0,5])rotate([0,90,0])cylinder(d=3.3,h=14);
  for(y=[-151,151]) zhole(10,y,-1,14,5.4);
 }
}
module knob(){difference(){cylinder(d=18,h=41);zhole(0,0,-1,44,5.3);zhole(0,0,35,7,9.5);}}
module pin_handle(){
 difference(){
  block(0,4.5,0,27,9,21);
  zhole(0,0,-1,9.5,10.35);zhole(0,0,8.5,5.0,18.4);
  for(x=[-9.8,9.8])translate([x,-1,17.1])rotate([-90,0,0])cylinder(d=2.7,h=12);
 }
}
module guide(){
 difference(){
  union(){
   block(0,0,0,40,42,6);block(0,0,6,27,27,65);
   block(0,0,97,27,27,8);block(-18,0,6,10,20,99);
   block(-10,0,97,20,20,8);
  }
  zhole(0,0,47,61,11.0);
  for(y=[-16.5,16.5]) zhole(0,y,-1,9,4.5);
 }
}
module receiving_tray(){
 difference(){block(0,0,0,100,115,24);block(0,0,12,96,111,13);}
 translate([0,0,12])difference(){cylinder(d=50,h=53);translate([0,0,-1])cylinder(d=38,h=55);}
}
module gauge_base(){
 difference(){block(0,0,0,29,65,8);block(0,0,3,17,53,7);
  for(y=[-28,28])zhole(0,y,-1,12,3.5);}
}
module witness_slide(){
 union(){block(0,0,0,15.7,22,3);block(0,-5,3,3,4,30);block(0,-5,28,22,4,12);}
}
module pointer(){union(){block(0,0,0,15.7,18,3);block(0,0,3,8,6,9);block(0,-3,4,34,3,3);}}
module pedestal(h){difference(){block(0,0,0,29,65,h);for(y=[-28,28])zhole(0,y,-1,h+2,3.5);}}
module panel_template(){
 difference(){translate([-245,-205,-1])cube([490,410,1]);
  for(x=[-67,67],y=[-45,45])zhole(x,y,-2,4,5.5);
  for(y=[-16.5,16.5])zhole(pin.x,pin.y+y,-2,4,4.5);
  for(x=[-205,0,205],y=[-155,155])zhole(x,y,-2,4,8.4);
  for(p=[[145,-39],[107,-22]],y=[-28,28])zhole(p.x,p.y+y,-2,4,3.5);
 }
}
if(part=="housing")housing();
if(part=="lever")translate([0,0,1])lever();
if(part=="crank")crank();
if(part=="knob")knob();
if(part=="pin_handle")pin_handle();
if(part=="guide")guide();
if(part=="tray")receiving_tray();
if(part=="witness_rail")gauge_base();
if(part=="wrist_flag")witness_slide();
if(part=="pointer")pointer();
if(part=="wrist_pedestal")pedestal(75);
if(part=="pointer_pedestal")pedestal(71);
if(part=="plate")projection()panel_template();
