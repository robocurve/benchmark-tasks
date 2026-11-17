# Physical construction: empty-bowl bench apparatus

This is a proposed, low-voltage construction of the generic apparatus in
`build_assets.py`, not a claim that a physical unit has been built or tested.
It is for an empty-bowl manipulation trial, not food preparation. The spring
rate, drive capacity, friction, stopping time and interlock behavior must pass
the commissioning checks below before anyone uses a constructed unit. The
native simulation results do not establish those physical properties.

The construction uses printed parts, stock shafts and bearings, two small
12 V gearmotors, and a manual rack lift. It retains the pin-and-slot coupling,
spring preload, detachable guard, 75 mm lift and observable start/stop sequence.
Its hidden drive supports, material stiffness and inertias are approximations
of the simulated apparatus. Its guard and interlock are experimental equipment,
not a certified human-safety system or an OEM-equivalent mixer.

## Why retain a spring

This mechanism has a physical precedent. The Hobart-authored
[N50CE manual](https://resources.itwfeg.com/sites/resourcecenter/ProductDocumentation/F23138.PDF), printed
page 11, describes fitting the agitator's L slot over a shaft pin, placing the
guard on the bowl, and raising the bowl; page 14, Figure 4, identifies the
agitator spring and requires it to be fitted. Page 18 lists spring V-111227 and
wire guard 478114. [Hobart's product page](https://www.hobart.com.sg/products/food-preparation/mixers/product/n50)
identifies the guarded CE version as **N50-619**, distinct from the unguarded
N50-604. [Hobart Australia](https://www.hobartfood.com.au/products/food-preparation/mixers/product/n50)
also specifies a removable stainless guard and self-locking manual bowl lift.

Those references justify a spring-loaded pin/slot mechanism. They do not
establish the generic model's dimensions, 5 N preload, 0.4 N/mm stiffness,
4 N s/m damping, speed ratio or tolerances. The construction below specifies
its own spring; it does not require purchasing Hobart parts. Secure retention,
guard seating, powered operation and stopping are judged by the task's
observable rubric, not by measuring this spring during a trial.

## Datums and construction drawings

All dimensions below are millimetres. Use a level worktop at world z = 747.5.
The mixer-local origin is world **(350, 30, 747.5)**; add it to mixer-local
coordinates to obtain the scene coordinates. +x is rearward into the mixer
column, +y is left, and +z is up. These are the scene axes, not a new camera.
Keep the standard YAM station camera for evaluation.

The actual manufacturing files are in [fabrication/](fabrication/README.md):
watertight PETG meshes, steel-ballast blanks, dimensioned shaft/collar and carrier
plate drawings, and a reproducible generator. `build_assets.py` / `geometry.json`
remain the native scene master. The exported beater keeps the original bore and
L-slot; only ballast fastener holes below the grip are added. The printed bowl
has a continuous 2 mm wall/floor joint. Static draining platforms are wood.

The physical drive uses a shorter orbit shaft supported above the spring and a
round nose pod with a 40 mm-radius lower opening. These hidden supports differ
from the native idealized drive. The tool, bowl, guard, pin/slot geometry,
spring force target and lift travel are retained. The physical lower cover
extends 12 mm below the rendered annulus; verify approach clearance on the
constructed unit rather than treating the native robot path as hardware proof.
No manufactured specimen or physical commissioning result is claimed.

| Datum or part | Mixer-local dimension / location |
| --- | --- |
| Base | 310 x 280 x 28; centre (-5, 0, 14) |
| Column envelope | 85 x 130 x 330; centre (105, 0, 190) |
| Head cover envelope | 250 x 190 x 110; centre (15, 0, 420) |
| Physical nose pod | 100 mm OD body; flange 130 mm OD; origin (-90,0,347), top 435 |
| Orbit axis | (-90, 0); shaft-pin plane z = 330 |
| Spindle, at reset | (-70, 0); 20 mm eccentricity from orbit axis |
| Bowl origin, lowered | (-90, 0, 70); raised origin (-90, 0, 145) |
| Handle pivot | (105, -140, 180), axis +y; knob centre starts 100 mm in -x |
| START / STOP face centres | (-115, -62, 415) / (-115, 62, 415) |
| Beater draining support | World centre (240, 320, 755), 130 x 130 x 15 |
| Guard draining platform | World centre (450, 310, 757.5), 260 x 250 x 20 |

Use approximately 0.5 mm fabrication tolerance for non-mating covers and racks.
Machine or ream the 12.7 mm hub bore to +0.1/-0.0 mm and use straight 12 mm
ground shaft stock. Align the spindle bearings within 0.2 mm over their 12 mm
spacing; require at most 0.3 mm indicated runout at the pin. These are build
checks with power disconnected, not scoring tolerances. Rework binding parts;
do not enlarge scoring thresholds to accommodate a faulty build.

## Parts and cost allowance

This workshop route has a **US$353.01 parts estimate**; allow **US$400**
for material price changes and rework before tax/shipping. It uses ordinary stock,
printed parts and an already-equipped workshop. Robot, worktop, labour and tools
are excluded. Prices checked on 2026-11-17 and explicit allowances are itemized in
[costed-bom.json](fabrication/costed-bom.json); the total is not a fully quoted
supplier kit. No N50 purchase or custom machining service is necessary.

| Part / quantity | USD total | Basis |
| --- | ---: | --- |
| [Two Pololu 3041 12 V 100.37:1 motors](https://www.pololu.com/product/3041) | 52.90 | checked |
| [Two Pololu 4036 DRV8876 drivers](https://www.pololu.com/product/4036) | 16.30 | checked |
| Two TLC555CP circuits, trimmers, perfboard and 5 V regulator | 20.00 | allowance |
| [Adafruit 798 enclosed 12 V / 1 A supply](https://www.adafruit.com/product/798) | 8.95 | checked |
| [Adafruit 775 12 x 17 mm six-circuit slip ring](https://www.adafruit.com/product/775) | 17.50 | checked |
| [Ten 6801-2RS 12 x 21 x 5 mm bearings; four used](https://jsbgreatbearings.com/product/6801-2rs-two-side-rubber-seals-bearing-6801-rs-ball-bearings-6801-rs/) | 11.00 | allowance; supplier listing varies by quantity |
| [goBILDA 2100-0012-0200 12 x 200 mm steel shaft; cut 83 and 70 mm](https://www.gobilda.com/12mm-shaft-stainless-steel-200mm-length/) | 6.39 | checked |
| 4 mm drive pin, 2 mm cross-pin, shaft collars and spacers | 8.00 | allowance |
| [K&S 5502 music wire, 0.039 inch x 36 inch, 20 pieces](https://ksmetals.com/products/mw36-039) | 15.99 | checked |
| Steel collar/seat blanks, 2 mm guides, M2-M8 fasteners | 15.00 | allowance |
| [Two MGN9 250 mm rail-and-carriage sets](https://www.robotdigg.com/product/779/Quality-440C-SUS-MGN9-linear-rail-with-carriage) | 40.00 | allowance above listed 18 USD/set |
| Lift pivot, friction washers, knob and stops | 8.00 | allowance |
| 180 mm GT2 belt, stock 12T/36T pulleys and hubs | 8.00 | allowance |
| 2 mm round belt and printed 24 mm spin pulleys | 5.00 | allowance |
| [Jameco 291507 LB4H-12DS-R 4PDT relay, socket 175557 and diode](https://www.jameco.com/z/LB4H-12DS-R-Jameco-ValuePro-Electromechanical-Relay-4PDT-5A-12-Volt-160Ohm-Plug-In_291507.html) | 12.00 | 4.95 USD relay checked; socket/diode allowance |
| START/STOP and five position/cover switches | 15.00 | allowance |
| Plywood, aluminium offcuts, brackets and clamps | 35.00 | allowance |
| [Two 1 kg PETG spools, ELEGOO Rapid PETG](https://us.elegoo.com/products/rapid-petg-filament-1-75mm-colored-1kg?variant=44479775375463) | 27.98 | checked |
| Captive mild-steel ballast, nuts and ties | 10.00 | allowance |
| Clear screen, guards, terminals, wiring and fuses | 20.00 | allowance |
| **Parts total** | **353.01** | Checked prices plus allowances |

All exported plastic meshes use about 0.72 kg at solid PETG density. Two complete
spools cover supports, mounting pieces, calibration coupons, waste and reprints.
Use plywood offcuts for the two static platforms rather than printing solid blocks.
The rail-set allowance preserves ordinary low-friction MGN9 guides; it does not
rely on an untested printed slide to lower the budget.

The [motor manufacturer's specification](https://www.pololu.com/product/3041)
gives a nominal 330 rpm no-load speed at 12 V. It does not guarantee either
assembled drive's speed under load. Do not use stall torque as a continuous
rating. Set a conservative 0.20 A continuous current limit per motor initially;
reject the build if this cannot operate the empty apparatus without repeated
limiting. Do not raise the limit merely to overcome a misaligned bearing or hub.

## Frame and supports: cut list

1. Cut a 310 x 280 mm base from two laminated 14 mm plywood sheets. Mark its
   centreline and datums. Clamp it at four corners to the worktop. The simulator
   fixes the chassis, so the physical fixture must not rely on its weight alone.
2. Make the column as a rigid 85 x 130 x 330 mm box, with its bottom at z = 25.
   Use 12 mm plywood sides with internal corner blocks, or an aluminium frame
   with 1 mm cosmetic covers. Bolt through the base using four M6 fasteners.
3. Make the 250 x 190 x 110 mm head cover from 1 mm sheet or 3 mm printed panels.
   A 240 x 100 x 6 mm aluminium bridge, centred at (5,0,438), bolts to the
   column and carries the bearings. At the orbit axis bore 26 mm through, then
   counterbore 38 mm diameter, 3 mm deep **from below**. Fit the printed orbit
   bearing housing from below: its flange occupies z = 435–438. Drill three
   3.2 mm holes at radius 16.5, angles 0/120/240 degrees. Use countersunk M3
   screws flush with the bridge top and nuts below the flange; no screw head
   may enter the belt plane. The cover carries no bearing load.
   Print `drive_pod.stl` and mount it at (-90,0,347). Its 2 mm lower annulus
   is z = 347–349, its 40–65 mm radius flange is z = 359–371, and its 50 mm
   outer-radius wall reaches z = 435. Bolt its flange to side brackets from
   the column/head. Fit the rectangular cover around its round nose, with
   enclosed joints and removable top. This pod accommodates the real carrier
   and motor outside the front edge of the rectangular native cover.
   Its 40 mm-radius bottom opening differs from the rendered 30 mm opening.
   Keep at least 3 mm clearance to rotating parts; hand-check the full sweep.
4. Make the bowl cradle: rear bar 18 x 255 x 18; two 130 x 20 x 12 side arms;
   two 20 x 20 x 25 risers. Their centres relative to the **bowl origin** are
   (130,0,80), (75,+/-129,80), and (15,+/-129,93.5). Add the 15 x 65 x 50 rear
   mounting block at (145,0,80). Use printed brackets over aluminium bars, not
   unsupported printed cantilevers. Bolt the bowl tabs to the cradle for the
   entire trial; bowl removal is outside this task.
5. Make the beater rack from the 130 x 130 x 15 base, an 18 x 130 x 45 upright
   centred at world (240,320,785), and two 8 x 95 x 25 rails centred at
   world (225,320,815) and (255,320,815). Copy the guard platform from the datum
   table. Round all exposed edges. The supports may be screwed to the table.

## Spindle, hub and spring: section drawing

For this section, use a **spindle coordinate system** with z = 0 at the drive-pin
centre, and +y along its protruding end. Thus its origin is mixer-local
(-70,0,330) at reset. Rotate all of these parts together with the spindle.

* Cut the 12 mm spindle from z = -10 to +73 (83 mm overall). The extra 3 mm
  at the hidden upper end accommodates a real pulley collar. Chamfer the bottom from radius
  3 mm at z = -10 to radius 6 mm at z = -5. Cross-drill a 4 mm hole at z = 0
  using a drill press and V block. Press/retain the 16 mm dowel with its ends
  at y = -6 and +10, so only +y has a 4 mm exposed projection. The opposite
  end must be flush. Deburr the pin and shaft. The simulated capsule has rounded
  ends; a real dowel's end must be rounded without reducing its 4 mm diameter.
* Support the spindle in two 6801 bearings with centres at z = +47 and +59. Keep their bearing block within z = +44 to +62
  and at most 26 mm OD. Print `spindle_bearing_housing.stl`, installed with
  its bottom at mixer-local 374. Its lowest face then gives
  3 mm above the fixed cover's top at 371.
  Fit axial collars against the bearing inner races; leave the outer races
  fixed in the rotating carrier. Do not put the beater's axial or radial loads
  through the gearmotor's output bearings.
* Make a retaining collar 32 mm OD, 18 mm ID and 1.5 mm thick. Its unloaded
  centre plane is z = +12.75. Drill three 2.2 mm guide holes at radius 13.5,
  120 degrees apart. Three smooth 2 mm steel guide rods have M2 threaded ends
  and fasten to a 32 mm OD, 12.1 mm bore, 3 mm thick upper spring seat. Lower
  stop washers/nuts have upper faces at z = +12.00, supporting the collar's
  lower face at zero travel. Lock and round all rod ends.
* Fit three 11.5 mm-long spacers, 4 mm OD / 2.2 mm ID, around the guide rods
  immediately below the spring seat at z = +33. Their lower faces are +21.5,
  stopping the collar's upper face after 8 mm travel. The complete rotating
  spring-seat/guide envelope fits inside a 38 mm eccentric opening in the lower
  carrier plate. The central orbit shaft begins above this mechanism, at
  mixer-local z = 382, so it cannot intersect the collar, seat or guide sweep.
* Put the compression spring between the collar's upper face and an upper
  seat whose underside starts at z = +33.00. The installed spring length is
  19.5 mm at zero collar travel: (32 - 19.5) x 0.4 = **5 N preload**. At 8 mm
  travel the length is 11.5 mm and the nominal force is 8.2 N. The spring's
  measured solid height must be below 10 mm. Use an adjustable split collar
  above the upper seat and lock it after calibration. A spacer above that clamp seats
  against the lower bearing inner race; set axial play without crushing the
  bearing seals. Its top must stay below
  z = +44.5, the lower bearing's bottom face; use a low-profile clamp/spacer
  stack rather than overlapping the bearing. The upper spin-pulley underside
  is z = +64, leaving a 2.5 mm spacer above the upper bearing's top face at
  +61.5; this pulley clamp retains that inner race axially. Keep the guide rods
  parallel; the spring must not rub the spindle. No physical damper is specified;
  the simulation's viscous damping is an explicit unvalidated approximation.
  A workable lower stack is the 3 mm seat at mixer-local 363–366, a 7 mm
  split clamp at 366–373 and a 1.5 mm spacer at 373–374.5. Make bearing
  spacers at most 18 mm OD so they contact only inner races and clear the
  housing pockets; the 24 mm OD clamp must remain below the housing.
* Measure spring force at 0, 4 and 8 mm collar travel with a small spring scale
  or digital force gauge, before installing the beater. Accept 5 +/-0.5 N at
  zero and a rate of 0.40 +/-0.05 N/mm. Adjust seat height for preload, not by
  stretching or cutting the spring. Reject coil binding or collar sticking.

For the spring, use 0.039 inch (0.991 mm) K&S 5502 music wire, a 36 inch
length. Cut a roughly 450 mm blank. Cold-wind a first coupon with about
3.5 active turns and two closed end turns, right handed, on the supplied
17.2 mm mandrel. Remove it before measuring springback; aim for 18.2 mm ID,
20.2 mm OD and 32 mm free length, with squared, deburred ends. The mandrel is
a starting jig, not a guarantee of final diameter. Adjust the next mandrel's
diameter and winding pitch using the coupon; maintain the final 32 mm length
and select the turn count by the measured rate. The ordinary steel-spring
estimate `k = G*d^4/(8*D^3*n)`, with G approximately 79,000 N/mm2, d = 0.991,
D = 19.2 and n = 3.5, gives about 0.38 N/mm. It is a sizing estimate only.
The measured 0.35–0.45 N/mm rate, preload and solid-height limits above decide
acceptance. Guard the wire while winding; the plastic mandrel is not a heat
treatment fixture. The spring does not need a purchased OEM part.

At maximum travel the nominal axial hand load is 8.2 N spring force plus
2.45 N tool weight, approximately **10.7 N** before friction. At an assumed
dry pad coefficient of 0.3 this requires about 17.8 N normal force per finger.
The lift estimate, including its maximum 1.1 N m brake, is
`(1.2*9.81*0.047746 + 1.1)/0.100 = 16.6 N` at the knob, or about 27.7 N
normal force per finger under the same friction assumption. These estimates
are below 40 N per finger; actual friction and gripping must be measured.
Reject binding or a knob force above 20 N, and reject any grasp needing over
40 N per finger or slipping at that limit. Do not reduce the specified spring
force or lifted mass to claim feasibility.

Make the beater hub as a continuous 24 mm OD, 40 mm high body, with a 12.7 mm
bore. The **hub-local** origin is the middle of that 40 mm height. The angular
zero is +x; angles increase toward +y. Its lower wall is continuous from
z = -20 to -3.5. The remaining wall above -3.5 has this section:

| Angular sector | Material present above z = -3.5 |
| --- | --- |
| 65–115 degrees | None: axial entry opening to the top |
| 115–125 degrees | Upper wall from z = +3.5 to +20 |
| 125–175 degrees | Upper wall from z = +8 to +20: deeper retaining pocket |
| All other angles | Continuous wall from z = -3.5 to +20 |

This is the explicit L-slot/pocket geometry in the source, rather than an
unspecified commercial bayonet. Print in tough nylon or PETG with at least six
perimeters, then ream the bore and finish the slot; do not use brittle resin.
Inspect layer adhesion before loading. Alternatively machine this section from
aluminium. A printed hub is a material substitute, not evidence of OEM fatigue
life. Use the 96 angular sectors of `build_assets.py` if reproducing its polygonal
profile; the boundary angles above describe the nominal section.

At entry, the hub origin is about 1 mm below the pin. Push upward against the
collar, turn the hub clockwise about 60 degrees viewed from above, then release.
The pin moves into the deeper pocket and the hub origin rests about 6 mm below
the pin. With a 4 mm pin, its upper surface then bears against the pocket ceiling
at hub-local z = +8. The spring prevents upward bounce out of that pocket; the
pin and pocket carry the hanging load. Do not glue, magnetically latch or screw
the beater to the spindle.

## Beater and bowl

Print [beater.stl](fabrication/beater.stl) in PETG, 0.2 mm layers, at least six
perimeters and solid infill. Inspect and ream the 12.7 mm hub bore. The mesh
includes six 3.2 mm holes below the grip. Cut two 20 x 80 x 6 mm and one
20 x 25 x 6 mm mild-steel webs using the supplied metal-blank files/drawing;
**do not print these ballast pieces**. Deburr and round their edges.

Install the two long webs with centres at hub-local (+8,+20,-125) and
(-8,-20,-125); the short web is centred at (+8,0,-110). Their holes coincide
with the beater's crossbar and centre rod. Use six M3 x 20 bolts, washers,
0.5 mm spacers where needed, locking nuts and smooth end caps. These permanent
webs remain outside the grip and inside the original rotating blade envelope;
none supports the tool from outside. Weigh the complete beater: target
**250 +/-10 g**. The density calculation gives approximately 251 g including
12 g of fasteners; trim the short steel web or add captive washers to calibrate.
The physical centre of mass is approximately 110 mm below the hub, versus the
native lumped model's 80 mm. Neither centre of mass nor inertia is a score gate.

Make the bowl as a continuous printed shell with 2 mm walls and bottom. Its
inner radial section, (height, radius), is (0,70), (25,80), (65,98), (105,110),
(140,112); linearly join the points. The upper locating rim has inner radius
110, outer radius 116, bottom z = 138 and top z = 142, with a rear +x clearance
notch spanning +/-0.08 radians. Add the cradle tabs from `build_assets.py` and
bolt them to the cradle. Fit captive steel stock below the bowl floor/support until the complete
bowl/cradle moving assembly is 1.20 +/-0.05 kg. An 80 x 60 x 20 mm steel block
is approximately 754 g; select/trim it after weighing the actual cradle. Mount
with flush countersunk fasteners and exterior backing bosses, leaving the cavity
floor flush and the moving ballast at least 10 mm above the base at the lower stop. The bowl cavity
must remain empty; no ballast, tools or wires may enter the swept volume.

Printed material is a low-cost substitute for the simulated aluminium beater
and stainless bowl. Match the outline, mass, clearance and retention behavior;
record material and any measured differences. This document does not establish
that the substitute has the same contact friction or failure strength.

## Bowl lift: physical realization of the gear constraint

Fix the two vertical MGN9 rails to the front column face x = 62.5 at y = +/-50,
from z = 60 to 310. Bolt the cradle's rear bar to the carriage blocks using
spacers so that its nominal centre remains (-90+130,0,70+80) = (40,0,150)
when lowered. Set the blocks' centre heights to 150 when lowered and 225 when
raised. Adjustable hard stops, not limit switches, establish the 75 mm travel.

The lift shaft runs along y through (105,-140,180). Fit the 100 mm handle and a
printed 96-tooth, 20-degree-pressure-angle pinion to the same shaft. Use a
custom rack tooth pitch of **3.125 mm**, hence module 3.125/pi = 0.994718 mm
and pinion pitch radius 47.746483 mm. Twenty-four teeth pass in a quarter turn,
giving exactly 24 x 3.125 = **75 mm**. Standard involute proportions are:
addendum 0.994718, dedendum 1.243398, and pitch-line tooth thickness 1.5625 mm.
Make pinion and rack 8 mm wide; use a rack at least 150 mm long. At the lower
position it spans z = 80–230. Its pitch line is x = 105-47.746483 = 57.253517,
y = -150, with teeth pointing toward the pinion. Keep the handle at its
specified y = -140 plane and extend its shaft to carry the pinion 10 mm
outboard, centred at y = -150. This puts the pinion's inner face at -146,
7 mm outside the nearest cradle-arm face at -139. Attach the rack body to the cradle
with an aluminium angle. Turning the handle from -x toward +z raises the rack.

An M8 shoulder-bolt pivot with two 30–40 mm friction washers and a locknut gives
the handle a passive holding brake. Before connecting the rack, adjust it to
hold 0.8 N m and slip below 1.1 N m, measured with a spring scale at the 100 mm
knob. A 1.2 kg lifted mass requires approximately 0.56 N m at this pitch radius,
so this brake should hold the bowl when the hand releases. Verify this with the
assembled mass, including at mid-travel. Add positive padded stops at 0 and
90 degrees. A switch is actuated only at the raised stop. Cover the rack/pinion
and rail pinch points. The gear law matches the native linear/angular mimic
constraint; actual friction and backlash still require commissioning.

## Detachable guard

Use the guard's own origin at the centre of its lowest wire ring. Build four
open rings from 5 mm diameter rod with centreline (radius, height) pairs
(115,0), (95,15), (70,32), (38,45). Omit the rear sector within 0.55 radians
(31.5 degrees) of +x. Join the rings with radial spokes at the source's
22.5-degree intervals, excluding that rear opening. Use welded stainless rod
if available, or printed PETG segments with mechanically captive joints. All
free wire/segment ends need rounded caps. Do not represent a loose push-fit
printed cage as a safety-rated guard.

Add three 4 mm diameter locating feet at radius 107, angles 90, 180 and
270 degrees, from guard-local z = -12 to 0. These fit inside the bowl rim.
The lowest 5 mm ring rests on the rim: guard-origin height is then
142+2.5 = **144.5 mm above the bowl origin**. Attach the 18 x 24 x 8 grip tab
at (0,131,28), with a 6 mm support from (0,112,0) to (0,131,28). Add the
10 x 8 x 5 orientation key at radius 112, angle 45 degrees, z = -1. Target
guard mass 180 +/-10 g. The solid PETG mesh is about 72 g: capture approximately
108 g of steel nuts on the intermediate ring near radius 95 mm, above the rim
and outside the beater sweep. Eight ordinary M12 nuts are a starting quantity;
weigh them. Use two independent locking ties/fasteners per nut, capped ends, and
verify that none can fall or touch the beater. This ballast stays inside the
guard's existing outer diameter. It is permanent apparatus, not an added trial prop.

Install three small normally-open seat switches on the bowl cradle, one by
each locating foot, using protected, recessed actuators. All three close only
when the bottom ring rests on the rim and its orientation key enters its pocket.
No switch or exposed wiring may support the beater or obstruct guard removal.
Set their actuators by lowering a correctly seated cage, not by imposing a
submillimetre position test. Add three visible seating witness marks and a
handle UP mark. These make the assembled state observable without accessing
the coupling during powered operation.

## Supported planetary drive

Cut the two 90 mm carrier discs using [carrier-plates.svg](fabrication/carrier-plates.svg).
The lower plate is 3 mm thick, centred at z = 354, with a 38 mm eccentric
opening centred on the spindle axis (-70,0). The upper plate is 6 mm thick,
centred at z = 390, with a 12.1 mm central shaft bore and 26 mm eccentric
bearing-block bore. Three standoffs no more than 6 mm OD connect the plates at
orbit-local (0,+24), (-24,0), (0,-24). They leave ample clearance inside the
physical cover's 40 mm inner radius. Keep the spindle-bearing block at most
26 mm OD, between z = 374 and 392; retain it in the upper plate with a close
fit and a radial M3 set screw.

The **70 mm orbit shaft** occupies z = 382–452. It is supported by fixed
6801 bearings centred at z = 415 and 435 in a 26 mm OD housing attached to the
head bridge. Install `orbit_bearing_housing.stl` at z = 410–438, using the
counterbore described above. Ream/fit both housings' 20.96 mm nominal pockets
to the actual 21 mm bearing outer races; they must not spin or crack the print.
The opposing inner-race collars/spacers retain each bearing inward against its
housing shoulder. Set axial play below 0.2 mm with free rotation, without
loading the seals. At the lower fixed bearing use a 24 mm OD clamp at
z = 405–410 and an at-most-18 mm OD spacer at 410–412.5; a large collar must
not enter the 21 mm bearing pocket. A 2 mm x 25 mm cross-pin along y at z = 390 locks the upper plate
to the shaft; drill through an aligned jig, with the pin centred in the 6 mm
plate thickness. The pin's ends are captive in the plate. The lower plate has
no central shaft or clamp: load travels through the upper plate and standoffs.
This keeps ordinary 32 mm spring-seat parts clear of the central axle.

Drill a 4 mm axial wire passage through the orbit shaft, breaking out below the
fixed housing at z = 405; deburr and sleeve the exit. The slip ring carries only
the rotating motor's two leads. Mount its 12 x 17 mm capsule at z = 455–472,
with its stationary half fixed to the head. It bears no mechanical load.

The beater spindle is 20 mm in +x from the carrier axis at reset. Its two
bearings and spring seat rotate with the carrier. Put the spin motor 26 mm in
-x from the carrier axis, shaft downward. Its output face is approximately
z = 403 and pulley centre z = 396 (4 mm pulley width). Print the supplied
`spin_motor_pulley.stl` (3.1 mm bore) and `spin_spindle_pulley.stl` (12.1 mm
bore); both have 24 mm belt pitch diameter. Drill/tap the motor wheel radially
along y for an M2 x 8 set screw, recessed below the groove. Fix the spindle
wheel to a 5 mm-high split 12 mm shaft collar above it with two M2 screws
through its face holes at x = +/-8.5 mm; drill/tap the collar face to match.
The wheel is z = 394–398 and collar z = 398–403; the spindle ends at 403.
Use a 2 mm round
polyurethane belt between the motor and supported spindle. Both pulley pitch
diameters must match. Start with roughly 167.4 mm installed belt centreline
length at 46 mm shaft spacing; weld/cut the belt to the supplier's specified
stretch and provide 3 mm of motor-slot adjustment. Use the **3041 without
encoder**, not the longer 5217. The manufacturer's
[dimension drawing](https://www.pololu.com/file/0J949/micro-metal-gearmotors-dimensions.pdf),
page 3, gives the HPCB body a maximum length of 25.6 mm for this ratio. At the
403 mm output-face datum, its rear therefore ends at or below 428.6, leaving
6.4 mm below the bridge underside at 435. Its 10 x 12 mm body and tucked
terminal wiring must fit a 20 mm swept diameter about the motor axis, with
the terminal wires below z = 432; route the two leads sideways and down.
Keep at least 3 mm measured
clearance from the fixed central bearing housing and fixed covers
throughout a hand-turned revolution. Reposition hidden brackets, not the
task's spindle axis or guard, if needed.

Use belt tension as a calibrated slip coupling: with power disconnected,
restrain the motor pulley and pull tangentially on a temporary 50 mm-radius
spindle test arm. Adjust belt tension until it transmits normal empty running
load but slips before 0.03 N m (0.6 N at the arm). Remove the arm. Reject a
build that cannot run without exceeding this setting. The slip threshold is
a commissioning limit for this lightweight prototype, not an injury threshold.

The fixed orbit motor drives the supported orbit shaft using the 12T/36T GT2
pair and 180 mm belt. Use approximately 65.6 mm centre distance with slotted
adjustment. Its output face is z = 437 and pulley centre about z = 445.5.
Use ordinary 6 mm-wide GT2 pulleys, with a 3 mm motor bore and a 12 mm shaft
bore (ream the larger pulley if needed). The belt plane is z = 442.5–448.5,
above the bridge; a 5 mm spacer separates the upper bearing inner race
(top 437.5) from the pulley underside at 442.5. Clamp the pulley to the shaft.
Put the fixed motor axis at (-24.4,0), rearward of the orbit axis. Cut a
16 x 18 mm motor-body window in the bridge there and use a slotted angle
bracket to clamp its body with the specified output-face height. The 9 mm
output shaft ends at 446; put the pulley set screw at 445.5 and verify grip.
The motor remains clear of the carrier's sweep. Cover both belts. Adjust electronic current
limiting and reject any belt or gear load above the motor's continuous rating.

For the simulation's stated directions, viewed from above:

* Carrier: 60 rpm counterclockwise.
* Beater: 136 rpm clockwise in the fixed world.
* Spin motor and spindle relative to carrier: **196 rpm clockwise**.

The sum is 196 clockwise minus 60 counterclockwise = 136 clockwise. The orbit
motor therefore turns its 12T pulley at 180 rpm in the appropriate direction;
the spin transmission is 1:1. Feed only the rotating motor's two power leads
through two slip-ring circuits. Individually insulate the unused slip-ring
wires. Set each drive's fixed PWM trimmer against visible output marks during
commissioning, then lock the trimmers before a trial. PWM is not closed-loop
speed regulation; speed changes with load. There is no programmed soft start.
The physical score requires observable continuous clockwise rotation at least
once per second, not the exact simulation speeds or ratio. Do not substitute
an assumed motor speed for the visible rotation check.

These small motors provide a proposed empty-bowl drive. Sustaining both speeds
with the specified preload and slip limit has **not** been demonstrated on
hardware. That is a release-blocking commissioning item for a physical trial,
not a result supplied by this document.

## Wiring and operation

Use only enclosed mains-to-12 V power. Fit the supply fuse before any branch.
Motor power passes through a relay contact and the two branch fuses before the
drivers. A second relay contact supplies the drivers' hardware-enable circuit;
pull-down resistors disable both drivers when its wire is open. The drive uses
two analogue timer circuits and the hardwired relay; it requires no firmware.

Wire the relay-coil circuit as follows, with all contacts shown unactuated:

```text
12 V -- fuse -- mushroom STOP (NC) -- cover closed (NO) -- guard foot 1 (NO)
     -- guard foot 2 (NO) -- guard foot 3 (NO) -- lift UP (NO)
     -- [START (NO) in parallel with relay's holding contact (NO)] -- coil -- 0 V
```

Choose relay/contact ratings for the DC inductive load and use the supplier's
coil suppression component. START cannot energize the relay unless the cage,
lift and fixed drive cover are in place. Opening any series contact drops the
relay. Closing it again does not restart the motor: START is required again.
Make the START/STOP external faces match the datum table; internal commercial
contact blocks may have more than the simulated 4 mm travel. The scoring
criterion is the observed press and resulting operation, not hidden contact
travel. The red mushroom may latch after STOP; reset it between trials.

Use one relay pole for holding, one for the fused 12 V motor supply, and one
to switch regulated 5 V to a logic RUN rail. Insulate the unused fourth pole.
The RUN rail supplies both timers and both driver SLEEP inputs. Place a
10 kOhm pull-down at each SLEEP input and a 10 kOhm bleed from RUN to ground.
A broken enable wire therefore sleeps its driver, while STOP also removes
motor supply. Share regulator, timer and driver grounds. Supply the 5 V regulator
from fused 12 V before the motor relay, so its voltage is established before
START. Connect driver VIN to the switched, branch-fused 12 V; OUT1/OUT2 go to
the motor. Fit 100 nF ceramic suppression across each motor's terminals.

Build two identical circuits using **TLC555CP**, one per driver. This is the
ordinary astable circuit in the [TI datasheet](https://www.ti.com/lit/ds/symlink/tlc555.pdf),
Section 6.3.2. Use the PDIP pin numbers below; keep timing traces short and
separate from motor leads:

| Timer connection | Point-to-point wiring |
| --- | --- |
| GND, pin 1 | Common ground |
| VDD / RESET, pins 8 / 4 | Logic RUN rail, 5 V when the relay is energized |
| CONT, pin 5 | 10 nF capacitor to ground |
| TRIG / THRES, pins 2 / 6 | Join together; 1 nF C0G timing capacitor to ground |
| RA | RUN through fixed 1 kOhm plus 100 kOhm multiturn trimmer to DISCH pin 7 |
| RB | Fixed 10 kOhm from pin 7 to the joined pins 2/6 |
| OUT, pin 3 | Corresponding driver's EN/IN1 input |
| Supply bypass | 100 nF ceramic between pins 8 and 1 at the IC, plus 10 microfarads across RUN/ground at the board |

Wire each trimmer's wiper to the outer terminal at the DISCH end, so loss of
wiper contact leaves its full resistance rather than an open timing path.
Use 1 percent fixed resistors. Nominal output-high duty is
`(RA+RB)/(RA+2*RB)`: **52.4–91.7 percent** as RA varies from 1 to 101 kOhm.
Frequency is approximately `1/(0.693*(RA+2*RB)*C)`: **68.7–11.9 kHz** with
C = 1 nF. Actual component tolerances and propagation delays require checking
the waveform. TI specifies a 5 V output-high minimum of 4.1 V at 1 mA load;
the [DRV8876 datasheet](https://www.ti.com/lit/ds/symlink/drv8876.pdf) accepts
logic high from 1.5 V and PWM up to 100 kHz. Verify at the driver input that
the actual high is at least 1.5 V, low at most 0.8 V and frequency below 100 kHz
with an oscilloscope before connecting motors.

Tie each DRV8876 PMODE to ground (PH/EN mode). Initially strap the orbit driver's
PH/IN2 to ground and the spin driver's PH/IN2 to RUN. Check actual rotation
directions with the beater removed. Because the two motors face different ways,
reverse a motor's OUT1/OUT2 leads if needed to obtain the marked carrier
counterclockwise and relative-spindle clockwise directions viewed from above;
then fix and label that wiring. Leave the board's
CS resistor and IMODE default intact. Add **1.10 kOhm from VREF to ground** on
each carrier: with its existing 10 kOhm pull-up to a 5 V SLEEP input, VREF is
about 0.496 V, giving a nominal 0.20 A chopping limit. Verify the actual limit
with a current-limited supply and dummy load before connecting a motor. The
[driver documentation](https://www.pololu.com/product/4036) describes this
adjustment and its default, much higher limit; the default setting is unsuitable
here. Do not ground CS, which disables regulation.

Keep FAULT and CS accessible as insulated diagnostic test points; do not add
loads to CS that alter its current-limit resistor. No encoder, watchdog or
automatic overspeed detector is present. With the covers installed, calibrate
the two trimmers from behind the screen using visible reference marks and
timed video or a non-contact tachometer. The rough unloaded starting duties
for the motor outputs are 180/330 = 54.5 percent and 196/330 = 59.4 percent;
these are starting estimates, not measured speed predictions. Adjust for the
assembled empty mechanism, retaining the current and belt-slip limits. Lock
both trimmers with their locking screws or a removable sealing cover. Do not
change them during a trial. Reject a build that cannot sustain visible beater
rotation above one clockwise turn per second without repeated current limiting.

STOP opens the relay, removes motor power and logic RUN, and **coasts**.
Stopping time must be measured. Supply interruption requires a fresh START
after restoration. This circuit reproduces neither regulated two-second ramps
nor constant-speed control; those are disclosed simulator approximations.

Place a fixed clear screen between the apparatus and people, outside the
robot workspace. Enclose the drive, belts and lift mechanism. Keep an independent
supply disconnect reachable. During manual fitting, calibration or reset,
disconnect the 12 V supply and confirm motion has stopped; an interlock alone
is not isolation. The listed experimental interlock is not fault-tolerant:
a welded contact or shorted wire can defeat it. It must not be relied on as
the sole protection for a person near moving components. A qualified builder
must assess the finished mechanical/electrical assembly before powered use.

## Construction and commissioning record

Keep this record with the apparatus. Do not describe the construction as
validated until every relevant item passes on the physical unit.

1. Record photos, material, purchased part numbers, any dimensional departures
   and total cost. Measure the external datums, 20 mm orbit offset, lowered
   bowl height, 75 mm lift, guard seating and mass of each moving assembly.
   Check base clamps and captive fasteners. Reject cracked parts or unsecured
   ballast.
2. With power isolated, record collar force at 0/4/8 mm, freedom of travel,
   bore fit and spindle runout. Fit the beater without tools, release both
   hands, keep the bowl at its lower mark and guard at least 350 mm from the
   hub, and confirm it hangs in the retaining pocket without any other
   support. Confirm it can be removed only by lifting and reversing the twist.
   Verify at least 50 mm clearance between beater and bowl floor throughout
   the permitted external working band. If the beater can be pulled straight
   down out of the pocket, reject it. Apply the exterior-neck and moving
   carrier-witness reference procedure in `PHYSICAL_SCORING.md`; do not substitute
   a fixed world-space box for an orbiting reference.
3. Hand-turn at least two complete carrier revolutions with bowl lowered,
   then raised with the guard seated. Check all beater/guard/bowl clearances,
   including the spring guide rods; reject scraping, snagging, binding or a
   contact that props up the beater. Check that 0.8 N m lift-brake adjustment
   holds the assembled bowl when released at every height. Record actual
   knob force and backlash; require at most 20 N knob force and a stable grip
   at at most 40 N per finger. Do not assume the estimate proves it holds.
4. Verify each switch independently with a meter and motors disconnected.
   START must fail with each individual guard switch open, bowl below UP,
   STOP actuated or drive cover removed. Cut and restore supply while START
   is released; the relay must remain off. Press STOP with the relay latched;
   both motor supply and enable must fall. Use a test lead to check a broken
   series wire causes the same result. Do not power a deliberately exposed
   rotor to test an interlock.
5. Before connecting motors, verify each timer waveform, logic RUN interruption
   and driver current limit with test equipment. Disconnect the beater and
   run the enclosed drive with both trimmers at minimum duty. Verify slip-ring
   continuity, current and the direction of both axes. Isolate again, fit the
   beater and cage, then adjust the trimmers from minimum to the intended
   setting from behind the screen. Stop on vibration,
   loosening, repeated current limit, overheating or contact with the bowl.
6. Record timed video of at least five seconds at the intended low setting,
   with visible beater/carrier reference marks and no hand support. Verify
   speed, secure retention, stationary guard seating and UP mark. Press STOP
   and record until all movement ceases, followed by the task's stationary
   endpoint interval. Record measured stopping time. Repeat ten complete
   fit/run/stop cycles and inspect the hub/pin and guard feet after each.
7. Score a full trial using the ordinary observable rubric. Test a deliberately
   unturned beater **with power isolated**: it must fail retention. Test an
   unseated guard and lowered bowl using the disconnected-motor meter test:
   they must fail the corresponding interlock checks. Preserve failures and
   their corrections; do not replace them with a successful simulation receipt.

Release criteria include reliable retention, no loose parts, free running
clearances, lift holding, observable seating, correct interlock behavior,
repeatable empty running and a demonstrated stop. Failure of any is a reason
to reject or revise the hardware before a physical trial. In particular, this
document currently lacks a built specimen, measured motor
margin, spring curve, contact wear record and physical stop/interlock results.
It provides a construction specification and commissioning procedure; those
missing measurements remain engineering work, not evidence already obtained.
