// Millimetres. Export each part individually with openscad -D 'part="scale"' ...
// Orange instrument only. The blue sleeve and yellow glove mount to the fixed shelf.
// The rigid crown core accepts a 2 mm compliant sleeve to give 50 mm finished OD.
// Tap the pilot with M4 tooling after manufacture. Tolerances depend on process.
part = "core";
$fn=96;
module core(){
    difference(){
        rotate([90,0,0]) cylinder(d=46,h=58,center=true);
        translate([0,0,-23.1]) cylinder(d=3.3,h=15);
    }
}
module scale(){
    // Scale carries eight white tick inserts, 5 mm apart; zero at Z=44.
    difference(){
        cube([3,41,60]);
        for(i=[0:7])
            translate([-.25,4,44-i*5]) cube([1.25,i%2==0 ? 33:20,1.5]);
        for(z=[4,55]) translate([-.5,20.5,z]) rotate([0,90,0]) cylinder(d=3.4,h=4);
    }
}
module pointer(){
    difference(){
        union(){
            cube([3,33,3]);
            translate([0,-8,-2]) cube([3,10,7]);
        }
        translate([-.2,-4,1.5]) rotate([0,90,0]) cylinder(d=3.2,h=3.5);
    }
}
module handle(){
    difference(){
        union(){
            cube([19,37,72]);
            translate([-8,0,0]) cube([35,37,6]);
        }
        for(x=[-4,23],y=[6,31]) translate([x,y,-1]) cylinder(d=4.5,h=8);
    }
}
if(part=="core")core();
else if(part=="scale")scale();
else if(part=="pointer")pointer();
else if(part=="handle")handle();
else assert(false,"part must be core, scale, pointer, or handle");
