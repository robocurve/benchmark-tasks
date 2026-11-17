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

`build_assets.py` is the dimensional master for the external parts. `geometry.json`
contains every primitive and convex mesh, and `setup-state.json` contains the
settled starting poses. Do not manufacture the artificial seams between convex
colliders: make each body a continuous solid. A printer/slicer mesh still needs
to be generated from these dimensions; no production STL, assembly CAD or
manufactured specimen is included. The coordinate and section descriptions here
are the construction drawing, and the commissioning record must identify any
departure from them.

| Datum or part | Mixer-local dimension / location |
| --- | --- |
| Base | 310 x 280 x 28; centre (-5, 0, 14) |
| Column envelope | 85 x 130 x 330; centre (105, 0, 190) |
| Head cover envelope | 250 x 190 x 110; centre (15, 0, 420) |
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

The following is a workshop purchasing specification. Except for the linked
motor specification, costs are estimates, not supplier quotes or proof of stock.
They exclude a robot, worktop, tax, shipping, labour and tools. A workshop with a
printer and drill press should budget roughly **US$750–900**, including allowance
for reprints and replacement parts. Outsourced machining can exceed this amount.

| Quantity | Part / purchasing specification | Allowance, USD |
| --- | --- | ---: |
| 2 | Pololu **3041**, 100.37:1 HPCB 12 V micro gearmotor, 3 mm D shaft, no encoder; leads and mounting brackets | 65 |
| 2 | Pololu **4036** DRV8876 motor driver carrier, plus 1.10 kOhm VREF resistors and headers | 40 |
| 1 set | Two **TLC555CP** PDIP timers and sockets, two 100 kOhm multiturn trimmers, timing/decoupling components and perfboard specified below; enclosed 12-to-5 V regulator, 1 A | 30 |
| 1 | Enclosed, listed 12 V DC, 3 A desktop supply; no exposed mains construction | 25 |
| 1 | 6-circuit capsule slip ring, at most 12.5 mm OD, at least 1 A/circuit and 300 rpm rating | 20 |
| 4 | 6801-2RS bearings, 12 x 21 x 5 | 20 |
| 2 | Ground 12 mm steel shafts: spindle 80 mm; orbit shaft 99 mm | 15 |
| 1 | Ground 4 mm dowel, 16 mm long; 12 mm split shaft collars; spacers | 15 |
| 1 | Compression spring: 20.2 mm OD, 18.2 mm ID, 32 mm free length, 0.40 N/mm nominal rate; stainless or music wire; verify by measurement | 20 |
| 1 set | 32 mm OD collar, 27 mm OD upper seat, three 1 mm guide wires, 22 mm OD spring shroud/upper stop, lower-stop washers and M2–M8 fasteners | 25 |
| 2 each | MGN9 rail, 250 mm, and matching long carriage block | 40 |
| 1 set | Printed 96-tooth lift pinion and 150 mm rack, M8 pivot, friction washers, handle and two stops | 20 |
| 1 set | GT2 pulleys: 12 teeth/3 mm bore and 36 teeth/12 mm bore; 180 mm pitch-length belt, 6 mm wide | 20 |
| 1 set | Two 24 mm OD smooth spin pulleys (3 mm and 12 mm bores), 2 mm polyurethane round belt, split-clamp hubs | 15 |
| 1 set | Finder **55.34.9.012.0040**, 4-pole 12 V DC relay with matching socket, coil suppression, 2 A supply fuse and 0.5 A motor-branch fuses; terminal blocks | 35 |
| 1 set | Latching mushroom STOP with normally-closed contact, normally-open START; three guard-seat switches, upper-lift switch and enclosure/cover switch | 35 |
| 1 set | Plywood/aluminium frame stock, 1 mm cover sheet, brackets, table clamps | 55 |
| 1 set | PETG/nylon for bowl, beater, supports, cage and mechanisms; captive ballast | 55 |
| 1 set | Fixed clear containment screen, cable guards, warning/position marks, miscellaneous wiring | 30 |
| | Subtotal before rework allowance | **580** |

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
   A separate 6 mm aluminium bridge, z = 435–441, bolted to the column supports
   the central bearing housing. Bore its orbit-axis opening 26 mm to admit the
   bearing housing, which is clamped to it by a flange above the bridge. The
   cover carries no bearing load.
   The existing lower bearing-cover envelope is a 30–65 mm radius annulus about
   the orbit axis, z = 359–371; retain at least 3 mm clearance to rotating parts.
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

* Cut the 12 mm spindle from z = -10 to +70. Chamfer the bottom from radius
  3 mm at z = -10 to radius 6 mm at z = -5. Cross-drill a 4 mm hole at z = 0
  using a drill press and V block. Press/retain the 16 mm dowel with its ends
  at y = -6 and +10, so only +y has a 4 mm exposed projection. The opposite
  end must be flush. Deburr the pin and shaft. The simulated capsule has rounded
  ends; a real dowel's end must be rounded without reducing its 4 mm diameter.
* Support the spindle in two 6801 bearings with centres at z = +47 and +59. Keep their bearing block within z = +44 to +62
  and at most 26 mm OD. Its lowest face is then mixer-local 374, giving
  3 mm above the fixed cover's top at 371.
  Fit axial collars against the bearing inner races; leave the outer races
  fixed in the rotating carrier. Do not put the beater's axial or radial loads
  through the gearmotor's output bearings.
* Make a retaining collar 32 mm OD, 18 mm ID and 1.5 mm thick. Its unloaded
  centre plane is z = +12.75. Drill three 1.2 mm guide holes at radius 13.0,
  120 degrees apart. Three 1 mm steel guide wires pass through these holes;
  braze their upper ends to the underside of a **27 mm OD**, 1.5 mm thick
  steel spring seat. Keep the complete seat/braze swept diameter at most
  27.4 mm. These wires guide sideways only; they do not carry the hanging
  beater load. Their inner swept radius is 12.5 mm, clear of the 12 mm-radius
  hub. Braze small captive washers at their lower ends, with washer upper
  faces at z = +12.00, to support the collar's lower face at zero travel.
  Round these lower ends. These washers lie below the orbit shaft and do not
  impose its narrow clearance requirement.
* Braze a metal spring shroud/upper stop to the spring seat's underside:
  **22 mm OD, 20.6 mm ID, 11.5 mm long**. Its bottom is z = +21.5 (mixer-local
  height 351.5). The collar's upper face starts at +13.5
  and contacts this tube after exactly 8 mm upward travel. The 20.2 mm OD
  spring fits inside the shroud. No upper stop nuts project outside the guide
  wire circle. Check that the spring remains free inside the shroud throughout
  compression; reject rubbing or binding rather than assuming its 0.2 mm
  nominal radial clearance proves it works.
* Put the compression spring between the collar's upper face and an upper
  seat whose underside starts at z = +33.00. The installed spring length is
  19.5 mm at zero collar travel: (32 - 19.5) x 0.4 = **5 N preload**. At 8 mm
  travel the length is 11.5 mm and the nominal force is 8.2 N. The spring's
  measured solid height must be below 10 mm. Use an adjustable split collar
  above the upper seat and lock it after calibration. Its top must stay below
  z = +44.5, the lower bearing's bottom face; use a low-profile clamp/spacer
  stack rather than overlapping the bearing. The upper spin-pulley underside
  is z = +64, leaving a 2.5 mm spacer above the upper bearing's top face at
  +61.5; this pulley clamp retains that inner race axially. Keep the guide rods
  parallel; the spring must not rub the spindle. No physical damper is specified;
  the simulation's viscous damping is an explicit unvalidated approximation.
* Measure spring force at 0, 4 and 8 mm collar travel with a small spring scale
  or digital force gauge, before installing the beater. Accept 5 +/-0.5 N at
  zero and a rate of 0.40 +/-0.05 N/mm. Adjust seat height for preload, not by
  stretching or cutting the spring. Reject coil binding or collar sticking.

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

Print the beater outline as one continuous PETG/nylon body, or fabricate the
same outline from aluminium. Below the hub, add a 22 x 14 x 30 grip block centred
at (0,0,-35). The remaining rods are 9 mm diameter, joining these hub-local
points: (0,0,-50) to (0,0,-170); (0,0,-55) to (0,-50,-105) to
(0,-35,-170) to (0,+35,-170) to (0,+50,-105) to (0,0,-55); and the crossbar
(0,-50,-105) to (0,+50,-105). Overall width is approximately 109 mm and height
195 mm. Use captive screws/washers in the grip and centre rod to bring mass to
250 +/-10 g and centre of mass about 80 mm below the hub. Do not attach loose
weights to the outer beaters. A scale and balance edge suffice for this build
check; the source inertias are not physically verified.

Make the bowl as a continuous printed shell with 2 mm walls and bottom. Its
inner radial section, (height, radius), is (0,70), (25,80), (65,98), (105,110),
(140,112); linearly join the points. The upper locating rim has inner radius
110, outer radius 116, bottom z = 138 and top z = 142, with a rear +x clearance
notch spanning +/-0.08 radians. Add the cradle tabs from `build_assets.py` and
bolt them to the cradle. Add captive ballast outside the bowl bottom/support
until the bowl/cradle moving assembly is approximately 1.2 kg. The bowl cavity
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
y = -140, with teeth pointing toward the pinion. Attach its body to the cradle
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
guard mass 180 +/-10 g, adding captive ballast near its bottom ring if needed.

Install three small normally-open seat switches on the bowl cradle, one by
each locating foot, using protected, recessed actuators. All three close only
when the bottom ring rests on the rim and its orientation key enters its pocket.
No switch or exposed wiring may support the beater or obstruct guard removal.
Set their actuators by lowering a correctly seated cage, not by imposing a
submillimetre position test. Add three visible seating witness marks and a
handle UP mark. These make the assembled state observable without accessing
the coupling during powered operation.

## Supported planetary drive

Build the hidden carrier as two 3 mm aluminium plates, 90 mm diameter, centred
on the orbit axis, at z = 354 and 390. Join them with three standoffs no more
than 6 mm OD, using M3 fasteners at orbit-local (0,+24), (-24,0) and (0,-24).
Their outer radius is at most 27 mm, leaving 3 mm inside the fixed bearing
cover's 30 mm inner radius; none intersects the eccentric bearing/spring bore. The lower plate's underside is z = 352.5, 1 mm above
the collar's highest possible upper face (351.5). Bore a **28 mm clearance
hole** through that plate, centred on the spindle axis (-70,0), to clear the
spring shroud and the guide wires' complete rotating sweep. The 32 mm collar
stays below the plate. This hole is tangent to the central 12 mm shaft bore:
there is **no metal web** between the bores. Use a C-shaped flanged central
clamp with M3 attachment ears at orbit-local (0,+10) and (0,-10), bolted to the
plate, and carry load through the upper plate and three standoffs. Machine the
same 28 mm eccentric relief through this lower clamp. The upper plate and
its central clamp also need a 28 mm eccentric opening/relief around the
spindle-bearing block, whose outside diameter must not exceed 26 mm. No clamp
material or fastener may enter either clearance cylinder. Do not rely on the tangent edge
for strength. The central clamp holds the **99 mm orbit shaft**, cut/located
from **z = 353 to 452**. Its lower end is 1.5 mm above the collar's maximum
height. The 27.4 mm maximum upper-seat/braze envelope leaves only
20-27.4/2-6 = 0.3 mm radial clearance to that shaft; the 1 mm guide wires at
13 mm radius leave 0.5 mm. Measure the complete hand-turned sweep and reject
any build with less than 0.3 mm clearance or contact. Its fixed 6801 bearing
centres are z = 415 and 435 in a 26 mm OD bearing
housing hanging from the head bridge. Drill a 4 mm axial wiring hole through
the orbit shaft, from both ends with the shaft held in a V block; break out
radially below the fixed bearing housing at z = 405. Deburr and sleeve the cable
exit. The slip ring is electrical only and
must carry no mechanical load. Mount it coaxially above the orbit shaft,
approximately z = 455–472, leaving 3 mm above the shaft end; its stationary
half is restrained by the fixed head.

The beater spindle is 20 mm in +x from the carrier axis at reset. Its two
bearings and spring seat rotate with the carrier. Put the spin motor 26 mm in
-x from the carrier axis, shaft downward. Its output face is approximately
z = 403 and pulley centre z = 396 (4 mm pulley width). Use the 24 mm smooth pulleys and a 2 mm round
polyurethane belt between the motor and supported spindle. Both pulley pitch
diameters must match. Start with roughly 174 mm installed belt centreline
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
The fixed motor lies rearward of the orbit axis, clear of the carrier's sweep.
Fit axial retaining collars and cover both belts. Adjust electronic current
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
   knob force and backlash; do not assume the estimate proves it holds.
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
document currently lacks a built specimen, fabrication meshes, measured motor
margin, spring curve, contact wear record and physical stop/interlock results.
It provides a construction specification and commissioning procedure; those
missing measurements remain engineering work, not evidence already obtained.
