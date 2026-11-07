# Replace a cut-out drain-pipe section with slip couplings

Replace a cracked length of drain pipe by dry-fitting a new section into the
cut-out gap with two slip couplings. A closed sleeve cannot get onto a run whose
two ends are already captured, so both sleeves go onto accessible old ends, or
onto the replacement, before the span closes. Only one of three bore sizes fits.
A fixed steel support with PTFE slide liners catches and aligns the replacement
while each hand slides its own sleeve across its joint, forming a dry fit before cement.

The combination differs from holding a skewer while threading pieces, sliding
a carriage onto an accessible rail, supporting a door until a pin is inserted,
putting a split spacer around a captive shaft, and fitting retainer bars after
a panel is flush. Here closed sleeves lose access when the gap closes, both
joints need axial overlap, and look-alike sleeves have different fits.
Plumbers (O*NET 47-2152.00) assemble pipe sections using couplings, clamps,
screws, bolts, cement and plastic solvent; general maintenance and repair
workers (49-9071.00) also perform such repairs.

## Parts, materials and reproducible layout

Positions are in mm, forward of the line through the two arm bases and left or right (robot's view) of the point midway between them; heights are above the 747.5 mm table top. The arm bases are 305 mm left and right of that point.

**Old drain run**, fixed to the bench: grey PVC-U pipe, 32 mm outside diameter, running left-right 200 mm forward with its axis 70 mm above the table. A 344 mm length has been cut out, leaving two stubs that run from 172 to 300 mm left and right of centre; each cut end has a 2 mm long, 28 mm diameter chamfer step. At 307 mm left and right a grey PVC-U elbow, 38 mm outside diameter, turns each stub down into a 32 mm diameter vertical riser standing on the table. A galvanised-steel pipe clamp holds each stub at 279 mm left and right: a post 12 mm forward-back, 8 mm left-right and 54 mm tall under the pipe, its centre 43 mm below the axis, and a strap 8 mm wide along the pipe, made of a 44 by 4 mm top bar centred 18 mm above the axis and two 4 by 32 mm side bars centred 20 mm in front of and behind the axis.

**Replacement section**, loose: orange PVC-U drain pipe, 330 mm long, 32 mm outside diameter, 1.5 mm wall, 63 g, with a 1.9 mm long, 28 mm diameter chamfer step on each cut end. It lies left-right, centred 280 mm forward and 20 mm right, axis 31 mm above the table (bottom 15 mm), on two fixed grey PVC-U rails, each 5 mm wide, 126 mm long and 15.1 mm tall, centred 273 and 287 mm forward and 20 mm right; their inner top edges touch the pipe 4.5 mm either side of its lowest line.

**Pipe support**, fixed, under the gap: a grey steel stand with PTFE slide liners, 109.2 mm forward-back by 140 mm left-right, centred 200 mm forward on the midline. Its two shoulders are 54.1 mm tall either side of a 9 mm wide centre channel; their inner edges touch the pipe 4.5 mm either side of its lowest line and hold its axis 69.5 mm above the table, 0.5 mm below the run. Two round guide bars, 38 mm diameter and 140 mm long, run left-right along its front and back edges at 164.4 and 235.6 mm forward, axes 69.5 mm high, leaving 0.6 mm side clearance to the pipe.

**Couplings**, loose, black ABS, each lying forward-back with its centre 120 mm forward and its bottom 8.5 mm above the table: two matching slip couplings (repair couplings with no centre stop), 80 mm long, 34 mm bore, 52 mm outside diameter, 30 g, centred 260 mm left and 260 mm right; a small look-alike, 60 mm long, 23 mm bore, 40 mm outside diameter, 15 g, 380 mm left; a large look-alike, 80 mm long, 43.5 mm bore, 65 mm outside diameter, 40 g, 367 mm right. The matching couplings have rounded mouths, the look-alikes square-cut ones.

**Trays**, fixed: two grey polypropylene trays with rubber-lined floors, centred 120 mm forward and 320 mm left and right; each floor is 90 mm forward-back by 230 mm left-right by 4 mm thick, with 4 mm wide, 8 mm tall rims along its front and back edges and open left and right ends. Each coupling rests on four grey rubber blocks, 6 by 4 mm in plan and 8.5 mm tall (8.4 mm under the large look-alike), 30 mm either side of its centre along its length and 4 mm either side of its axis.

Friction, static/dynamic: PVC-U and ABS 0.6/0.5, rubber 1.2/1.0, PTFE 0.05/0.04.

## Simulation construction

The simulator builds these parts from rigid primitives, as listed below; a
real rebuild uses the plain parts above. Every task part has a 1 mm contact
offset. Each sleeve wall is a ring of overlapping ball-capped rods (square-cut
on the look-alikes) with diagonal bore relief and an inner wall web, and the
section's end portions are faceted within 1.9 mm of round.

Left and right are from the robot's view. The section lies left-right; all four
sleeves lie forward-back in the trays. The left matching sleeve starts at yaw
+90 degrees and the right at -90 degrees about vertical. Each turns to yaw 0
above the open gap. The decoys have their respective tray's yaw. Sleeves have
continuous bores and no centre stops. Pipe interiors are solid because flow
is outside this dry-fit task.

The section is a plain round 32 mm-diameter pipe, 330 mm long. Construction
coordinates below are relative to its centre. Its middle 250 mm is truly circular, modelled by coaxial cylinders spanning y=-125..-85
and -85..+125 mm (lengths 40/210 mm, centres -105/+20 mm). Each end portion,
y=+/-125..163.1 mm, uses a faceted profile within 1.9 mm of round: one core
17 by 28.2 mm in cross-section, six 1.9 mm side strips, and four radius-0.95 mm
round edges at cardinal radius 15.05 mm. On each side the strips' outer
faces are 10.4, 12.25 and 14.1 mm from the axis; their full heights are twice
the square root of (16 squared minus outer-face distance squared) mm. Each
core, strip and edge is 38.1 mm long and centred at y=+/-144.05 mm. This
profile contains a radius-14.1 mm disc and fits within radius 16.464 mm
(rounded); inward/outward deviations are at most 1.9/0.464 mm. Two
28 mm-diameter, 1.9 mm-long chamfer steps are centred at y=+/-164.05 mm.
The 63 g mass represents a 330 mm length of 32 mm PVC-U waste pipe with
1.5 mm walls; its simulated interior is solid. The hand and both seats
contact the true round middle. The grasp is at y=-105 mm; the complete
36 mm pad length lies inside that middle with 2 mm axial margin.
The old stubs' main cylinders are 126 mm long, centred 237 mm either side of
the old run's origin, with end steps centred 173 mm either side. Each clamp
has a 12 by 8 by 54 mm post centred 43 mm below the axis, a 44 by 8 by 4 mm
top strap centred 18 mm above it, and two 4 by 8 by 32 mm side straps centred
20 mm either side of the pipe line. Each riser is centred 35 mm below the axis.

The construction table gives one quadrant of each matching sleeve, rounded
to 0.1 mm. Reflect across both axes, counting cardinal rods once. Angles are
measured from local +x toward +z. Rods run along local y; each cylinder is
80 mm minus twice its radius long, with equal-radius balls at its two ends.

| Angle (degrees) | Centre radius (mm) | Rod radius (mm) |
|---|---|---|
| 0, 22.5, 75, 90 | 21.5 | 4.5 |
| 42.75 | 21.9 | 4.1 |
| 61.5 | 21.8 | 4.2 |

The bore remains 17 mm at the top, bottom and both sides, with relief near
the diagonals. Its maximum radial bore is 19.5 mm and the outside dips by up
to 3.3 mm between rods (upper bounds rounded to 0.1 mm). The rods overlap
throughout the wall. Each matching sleeve has a solid web inside the
side wall, 6.4 by 70 by 6 mm, centred 21.5 mm from its axis at the equator: local +x for the left sleeve,
local -x for the right. The pads initially
close on the outside at axis height for both the tray pick and the supported re-grip.
The small decoy uses 12 equally spaced full-length rods, radius 4.3 mm at
centre radius 15.8 mm; the large uses 16, radius 5.4 mm at 27.1 mm. Their
solid webs are 16 by 50/70 by 1 mm, centred 14.5/26 mm above their axes.
These dimensions are rounded to 0.1 mm; the supplied geometry specifies the
full construction. The matching mouths are rounded; decoy mouths are square-cut.

Tray floors are 90 by 230 by 4 mm; side rims are 4 by 230 by 8 mm and the ends
are open; the floors are rubber-lined. Each sleeve rests on four rubber blocks, 6 by 4 mm in plan,
placed 30 mm along either side of its centre and 4 mm either side of its axis.
Their heights are 8.5 mm for the matching and small sleeves, 8.4 mm for the
large sleeve (rounded to 0.1 mm); the sleeve support gaps are 0.5 mm.
The two PVC-U section rails are 5 by 126 mm, at 7 mm either side of its axis,
spanning world y=-83..+43 mm. Their inner top edges carry the round section 4.5 mm
either side of its centre, forming a shallow V. The rail height is
31 - 0.5 - sqrt(16 squared - 4.5 squared) mm, about 15.1 mm.
The section settles 0.5 mm to an axis height of 30.5 mm and bottom height
14.5 mm. Each rest has over 16 degrees of stability before its centre of mass
passes a contact edge. Sleeves on either section end clear the rails by 22 mm.
All support-block bottoms stand on the table. Tray floors are 4 mm high;
all cradle tops are at least 8.4 mm high. The
right-hand section grasp, 105 mm right of its centre (world y=-125 mm at pickup), clears both starting cradles and the pipe support.

The fixed grey steel support has PTFE slide liners on its V-seat and round
guides. The bed spans 109.2 mm forward-back and 140 mm left-right, with a
9 mm channel down its centre. Each shoulder is 50.1 mm wide, centred
29.55 mm either side of the line; its exact height is 69.5 minus
sqrt(16 squared minus 4.5 squared) mm, about 54.1 mm. The two inner top
edges contact the round middle at x=+/-4.5 mm; the seated axis is 69.5 mm
above the table, 0.5 mm below the run. These contacts give 16.335 degrees
(rounded) before the centre of mass passes a contact edge. The start rests
use the same two-contact cross-section. A circular pipe has no preferred
axial roll orientation.

Two 38 mm-diameter, 140 mm-long guides run left-right at 35.6 mm either side
of the line, axes 69.5 mm above the table. They leave 0.6 mm side clearance.
Their crowns direct a displaced pipe inward from arrival yaw 7 degrees
counter-clockwise or 2 clockwise about the grasp, +/-3 mm sideways and
0..4 mm high. The lower guide crowns also clear the upper fingers while
the pads pass beside them. All closing widths leave at least 6 mm between the placing hand and support,
including 1 mm of TCP sag. With a whole 30 g sleeve at
either extreme end and the other end unloaded, the tipping margin is 1.547x.
The section's weight moment about its grasp is 0.064893 N m (rounded).
The support occupies y=-70..+70 mm, wholly under the round middle, leaving
15 mm to an 80 mm sleeve preloaded on either end. It is a stand with no
attachment to any loose part.

All retention uses gravity and passive rigid-body contact. PVC/ABS parts and
the PVC-U section rails use static/dynamic friction 0.6/0.5; the rubber tray
linings and blocks 1.2/1.0, so the sleeves stay put until picked; PTFE slide liners on the steel bed and guides 0.05/0.04. Each task part has a 1 mm contact offset.
No cement, weld or attachment is used.

## Scoring

| Score | End state |
|---|---|
| 1.0 | Section in line, both joints coupled, three repair parts released and still for 1 s |
| 0.75 | Section in line and both joints coupled, awaiting released stillness |
| 0.5 | Section in line and either exactly one joint coupled or both matching sleeves on the run |
| 0.25 | Both matching sleeves on the run, including staging, section not in line |
| 0 | Every other outcome, untouched start, or a repair part fallen off the bench (sticky) |

End-state tiers; either matching coupling may join either end. In line: section axis within 4 degrees of the pipe line (200 mm forward, 70 mm above the table); both end centres within 5 mm radially of that line; both ends between the cut faces, 172 mm left and right of centre, allowing 2 mm axial penetration. Coupled: a matching coupling's axis within 5 degrees of the pipe line, its centre within 3 mm radially, overlapping both the stub and the section by at least 25 mm along the pipe. On run: the same 5-degree/3 mm alignment and at least 25 mm overlap with a stub, or with the section on the section's own axis, including while the section is on its start rails. 1.0: section in line, both joints coupled, and the section and both matching couplings released and still, continuously for 1 s. Released: each gripper's centre point between the pads at least 3 cm from each of these three parts' bounding boxes. Still: over the last 1.0 s, each part's centre stays within 1.0 mm of where it ends and its axis direction within 1.0 degree of its final direction; spin about a part's own axis does not count, as a round pipe or coupling can turn without moving the assembly. Alignment, coupling and release must hold over that same second. Below 1.0, release and stillness are not required. 0.75: section in line and both joints coupled, but held, moving or short of the 1 s hold. 0.5: section in line and either exactly one joint coupled (the other coupling anywhere) or both matching couplings on run. 0.25: both matching couplings on run, including parked on stubs or section, section not in line. 0: every other outcome, including the untouched start and the section in line with no joint coupled and not both couplings on run. Parts may be set down on the bench. Permanent 0 for the episode if any point of the section or a matching sleeve ever goes more than 10 mm below the bench top (bench top 0.7475 m, so below 0.7375 m), i.e. off the bench; picking it back up does not clear it. Look-alikes are not checked and never count: the small one cannot fit the 32 mm pipe; the large one hangs at least 5.75 mm off-axis, beyond 3 mm. Success requires score 1.0 for a further 1 s.

A real trial is measured with a tape along the pipe for each coupling's overlap on its stub and on the section, a straight edge across both stubs for the section being in line, and the video for stillness and the 3 cm hand clearance.

Simulator implementation of the 1 s window: it counts consecutive control steps, with no extra stillness timer; repeated scorer calls in one step count once, a missing step restarts the window, and a reset clears it. Offline (a single layout scored with no simulator clock, as in the layout tests), each part's reported speed must be under 1 cm/s and spin under 0.5 rad/s.

## Open-loop demo

The YAM's two fingers are driven independently, so a 52 mm sleeve settles
against one fully open finger; the demo plans for that measured offset.
Both hands close with the sleeve centre 21.1 mm along gripper +x from the TCP.
They lift to 150 mm above the table before turning to yaw -90/+90 degrees,
keeping the offset sleeve clear while it rotates about the TCP. The sleeves
then descend into the gap at y=+/-127 mm, axis+1 mm, and slide outward 90 mm
over 4.5 s to y=+/-217 mm. Each inner end is 5 mm behind its stub face.
A sleeve parked 5 mm outboard of a cut face (centre 217 mm left/right)
clears the clamp by 18 mm.
The hands open for 1 s while moving 7.1 mm toward the sleeve axis, clearing
the finger that was at its open limit, then rise clear.

The right hand picks the section 105 mm right of its centre, at world y=-125 mm, allowing its measured 2 mm
settle along gripper +x. It lifts to 150 mm above the table, carries from
280 to 200 mm forward above the guides while recentering the section 20 mm left, then lowers to axis+5.5 mm at the TCP.
The measured 4 mm hang leaves the section axis 2 mm above its physical V-seat rest height.
It opens, rises and leaves the gap. The left waits clear throughout placement.
Neither hand holds the section while a sleeve slides.

Both hands then approach their parked sleeves together, re-grip at the same
21.1 mm lateral offset with the TCP 1 mm above the old pipe axis. The pads
close on the outer side wall; the pipe carries the sleeve throughout this
re-grip and slide. Both slide inward 48.5 mm over 2.5 s to
y=+/-168.5 mm. Each joint
has 36.5 mm nominal overlap on both members. They open with the same 7.1 mm
inward release movement, rise and park at 90 mm forward, 245 mm left/right,
160 mm above the table. The last stationary hold is 3.5 s. The plan lasts
41.9 s; with the 1 s success hold it stays below 72 s, 60% of the 120 s limit.
Targets depend only on the known start layout and elapsed time.
All close/open segments last 1 s.

## Run and score

In the official simulator environment, from this task folder:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=repair_drain_pipe_with_slip_couplings.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the orange section into the gap in the grey drain pipe and join both ends with the black slip couplings that fit." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ./demo-run
```

Use `--policy dream` for the model trial with the same setup and limit.
The measured `task_score` is the end-state tier; success requires a further
1 s at score 1.0. Resets use the exact layout above and object_jitter=0;
seeds do not change it. Runtime is 41.9 s simulated for the full scripted plan,
up to 120 s for a trial; wall runtime and startup depend on hardware.

No third-party assets; all geometry is primitives from the included kit.
