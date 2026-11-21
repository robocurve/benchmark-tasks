# Seat the water bottle in the side-entry cage of a small frame

**Credit.** This task takes up the idea of PR #2420, **TK-LJWYLG** ("Put a water bottle into a side-entry cage under
a small mountain bike's top tube, past its sprung front arm"), which was reviewed as Not accepted on Nov 18, 2026; the idea
became open to other teams when its 48-hour hold lapsed without a revision. Its idea, its sources for the frame and cage, and its spring estimates are
credited to that task. The scene, scoring, demo and fixture here are a separate implementation written for this
submission; none of that task's code or assets is used.

A full-size replica of an extra-small trail hardtail's front triangle (Salsa Rangefinder, size XS) stands on a plywood
base on the bench, its left side to the robot. An Elite Custom Race X side-entry cage is bolted to the down tube's
upper two bosses. On this small frame a 550 ml bottle cannot go into the cage along the cage's own axis: the top tube
leaves 60 to 70 mm of room above the seated bottle's cap along the cage axis, and the bottle's base would have to rise
about 90 mm along it to clear the front arm's band. It has to come in from the side, under the top tube, pushing the cage's sprung front arm open
with its own side until the arm snaps shut round it, and then be set down on the cage floor and let go. The robot
picks up the empty bottle lying on the bench, turns it to the cage's slope, pushes it in sideways, lowers it onto the
floor, lets go and draws its hand back out of the frame.

Instruction: "Put the water bottle in the bottle cage on the bike frame so the cage holds it, then let go."

## What changed for each review point

| Review point (TK-LJWYLG, Nov 18) | What this task does |
|---|---|
| Flag: "The simulated seating check allows the base up to 10 mm from the floor center without checking floor contact; align it with the physical seating check." | Seated now means the base actually on the cage floor: the lowest point of the base's rim within 1.0 mm of the floor along the cage axis, the base centre within 3 mm of the cage axis and the bottle's axis within 3 degrees of the cage's, held for 1 s with both hands more than 5 mm away. A released, still bottle within 1 mm of the floor can only be resting on it (gravity pulls it down the 51-degree cage). The physical check is the same: a 1.0 mm feeler gauge cannot be passed under the base rim all the way round, and the same 3 mm and 3 degree limits by ruler and inclinometer. A bottle 1.2 mm up, or hung up higher behind the arm, gets 0.5, not 1.0 (near_misses.py). |
| Feasibility (not met): "The required XS bike costs $600-$700 used or more new, and lending or resale does not provide a reliably available low-cost route." | No bike is used. The bike is replaced by a full-size frame fixture built from stock aluminium tube and plywood, about $200 in materials (bill below). No work stand is needed. |
| "Provide a reliably available, low-cost frame or a reproducible full-size fixture with drawings or build files ..." | `fixture/frame-fixture-1to1.svg` is a 1:1 drawing (print at 100 %; a 100 mm check bar is on it) of the tube centrelines and outlines, gussets, bosses, cage floor and seated bottle. `fixture/cut-list.csv` gives every piece's material, length, angle and end points. Both are written by `fixture/make_drawing.py` from the same numbers the simulation uses, so drawing and simulation cannot drift apart. |
| "... that preserves the specified cage mounting ..." | The cage is the real Elite Custom Race X, bolted with M5 bolts into two M5 rivet nuts in the down tube 64 mm apart, at the positions of the XS frame's upper two bosses, with the cage floor 35 mm below the lower bolt. |
| "... frame clearance ..." | Tube centrelines follow Salsa's 1:1 drawing of the XS front triangle (source below): the down tube's centreline is 71 mm from the cage axis (an 8 mm gap from the bottle to the 2 in tube), and the top tube passes 60 to 70 mm above the seated bottle's cap along the cage axis (the bottle would need about
90 mm to slide out over the arm's band). Tube diameters (2 in down and head tube, 1-3/8 in top and seat tube) are close to the drawing's in-plane tube widths (50, 35, 35 mm). |
| "... and obstructed approach." | The front triangle stays closed: top tube, down tube, seat tube and head tube are all present at full size. Lifting the bottle out or in along the cage axis is blocked by the top tube, so the only way in is sideways past the sprung arm, as on the bike. |
| "Confirm that the substitute and support can be obtained or built for a few hundred dollars or less." | Bill of materials below: about $200 including the cage and bottle, from catalogue items; built with a hacksaw, a drill and a rivet-nut tool. |

The task also fixes points that later reviews of other tasks raise: no credit while the bottle is held (every credited
state needs both hands more than 5 mm away, measured from points sampled every 3 mm over the gripper's and fingers'
collision faces, not just their corners); no credit for leaning the bottle against the arm or resting it near the cage
(the original's 0.25 "resting on the cage" tier is gone); the over-bent arm rule is judged over the whole trial; any
unreadable hand, bottle or cage state gives 0; and the bottle's start changes between two layout lots so that a
replay of one layout fails the other.

## Run

Install yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena) and run, from this folder:

    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
      --policy yam_scripted --no-prompt \
      --instruction "Put the water bottle in the bottle cage on the bike frame so the cage holds it, then let go." \
      --scorer task_score --max-steps 1000 --epochs 2

Trial k uses lot k mod 2 (lot A first); `--epochs 2` runs both lots and reports their mean as `task_score`.
`TASK_LOT=A` or `TASK_LOT=B` pins every trial to one lot. Use `--policy dream` for Dream-1. The task needs
`object_jitter=0`. No seeds are used.

- **Runtime per trial:** the scripted demo reaches full score at about 21 s of simulated time per trial; a two-lot demo
  run took 6.5 min on one shared L40S, including about 20 s to start the simulator.
- **Offline checks (no simulator):** `python near_misses.py` scores every near miss below and the history rules.
- **The demo** (`demo()` in task.py) is open loop. It reads only the published layout of the trial: where the bottle
  lies and which way (the two layouts are written in `setup` and both are plainly visible to the top camera) and the
  fixed fixture. It never reads the simulator's
  states. The right hand grips the lying bottle across its body 126 mm from its base, lifts it, turns it so the bottle
  lies along the cage's 51-degree slope with the hand coming from the robot's side and 60 degrees round from the front
  of the bottle (a hand square to the frame would put one finger between the bottle and the down tube), holds it
  150 mm out from the frame and 12 mm above the floor, pushes it in sideways for 3 s (the bottle's side opens the front
  arm, which springs shut behind it), lowers it onto the floor, opens the fingers to 1.5 mm clear of the bottle, draws
  back 65 mm along its fingers and out of the frame, and returns to its rest pose. The demo turns the gripper with
  `tilt.py` (a small runtime patch so a scripted waypoint can carry a full gripper rotation; it affects only the
  scripted demo). `lots.py` moves the bottle between lots; `parts.py` builds the jointed fixture.

## Scene

World frame: x away from the robot, y to its left, z up; the bench top is at z = 0.7475 m.

- **Fixture frame**, fixed, its plane at x = 0.430 m (the near tube faces 25 mm closer to the robot), head tube to the
  robot's left, tipped 10 degrees nose-down. Round 6061 aluminium tube: down tube 2 in (50.8 mm) in two straight pieces
  193 mm and 410 mm long; top tube 1-3/8 in (34.9 mm) in two pieces 211 and 217 mm; seat tube 1-3/8 in, 378 mm; head
  tube 2 in, 95 mm; bottom-bracket stand-in a 73 mm piece of the 2 in tube across the frame (modelled 46 mm). Joints are clamped between pairs of 12 mm plywood gussets.
  Frame-plane coordinates of every end are in `fixture/cut-list.csv` and task.py. The cage floor's middle is at
  (0.430, -0.515, bench + 0.250 m); the bottom bracket's underside stands 40 mm above the bench on a plywood block,
  and the head tube's lower end rests on a 45 x 45 mm post; both stand on an 18 mm plywood board 240 x 620 mm.
- **Cage**, Elite Custom Race X (74 mm, 147.3 mm tall, 28.5 g, glass-fibre polyamide), black. Modelled as a fixed back
  (spine) between bottle and down tube, a floor with 10 mm lips at the front and far side (open on the robot's side),
  a fixed far-side wall up to 105 mm, and the sprung front arm: a 4 mm band from 48 to 88 mm up the cage that wraps
  from a hinge on the far front round the front to 40 degrees past it toward the robot. The arm is a hinged link with
  a torsion spring (0.68 N m/rad, preloaded 0.13 rad onto its closed stop: about 1.3 N at the tip closed and 3.9 N
  at 15 degrees open, the original task's estimates), mechanical limit 40 degrees.
- **Bottle**, Elite Fly Tex 550 ml, empty, cap closed: 74 mm body 160 mm long, 59 mm cap to 184.5 mm, 51 g, blue body and
  white cap in the simulation (the listing in the bill of materials is the red one; the colour plays no part). Lot A: lying on the bench with its base centre at (0.250, -0.300), cap toward +y (the robot's left).
  Lot B: base centre at (0.300, -0.170), turned 25 degrees anticlockwise seen from above (cap toward the robot's left
  and toward the robot).

## Scoring

| Score | Condition (every one needs both hands more than 5 mm from the bottle and the front arm, and the bottle and arm still within 1 mm for 1 s) |
|---|---|
| 1.0 | Seated: base rim within 1.0 mm of the cage floor along the cage axis, base centre within 3.0 mm of the cage axis, bottle axis within 3 degrees of the cage's; front arm within 5 degrees of closed. |
| 0.5 | Inside the cage behind the closed front arm but not seated: bottle axis within 6 mm of the cage axis at the arm's band, within 10 degrees of it, base rim no more than 30 mm above the floor; arm within 5 degrees of closed. |
| 0 | Anything else, including a bottle held, outside the arm, leaning on the arm's tip, upside down, or on the bench. |

Whole trial: if the front arm is ever bent more than 30 degrees open (more than the cage's band can take without
damage), or the hands, bottle or cage cannot be read at any step, the trial scores 0. Trials are run on lots A and B
and averaged.

Real trial, 1.0: base rim on the floor (a 1.0 mm feeler gauge cannot be passed under it all the way round); base
centre within 3 mm of the cage centreline (ruler, in two end photos square to the cage, one from the front and one
from the open near side, each against a centre mark made at set-up on the cage floor's edge, which stays visible
beside the seated bottle; the two offsets are combined as the sides of a right triangle); axis within 3 degrees of
the down tube (phone inclinometer on bottle and tube); arm shut (its tip within 5 degrees, 6 mm, of its rest
position, marked with tape; ruler).
Real trial, 0.5: arm shut as above; base rim no more than 30 mm above the floor (ruler along the cage, from the floor
to the lowest point of the rim); axis within 10 degrees of the down tube (inclinometer); the bottle's centreline
within 6 mm of the cage's at the middle of the arm's band, 68 mm up the cage (the same two photos, read at that
height).
Both levels: hands clear and nothing moving for 1 s (video); arm never opened past 30 degrees (slow-motion video
against a card behind the tip marked at the 30 degree position, about 36 mm from the tip's rest mark in the model,
where 5 degrees is 6 mm; an estimate for the real cage).

Near misses (`near_misses.py`, all pass): seated 1.0; rim 1.0 mm up 1.0; rim 1.2 mm up 0.5; hung 20 mm up 0.5; hung
35 mm up 0; base 1 mm into the floor 1.0; 3.0 mm off axis 1.0, 3.5 mm 0.5; 3 degrees tilted 1.0, 4 degrees 0.5; arm
5.0 degrees open 1.0, 6 degrees 0; resting on the arm's tip part way in 0; on the floor 20 mm short outside the arm 0;
upside down 0; on the bench 0; seated but a fingertip 4 mm away 0, 5.5 mm 1.0. History checks: 0.9 s of stillness 0,
1.0 s 1.0; arm bent to 30.5 degrees at one step 0, 29.5 degrees 1.0; a 1.5 mm movement in the last second restarts
the hold; an unreadable hand or bottle state at one step 0; a point 0.5 mm off the bottle's side reads as 0.5 mm.

## Results

Simulator runs with the command above (`--epochs 2`, `object_jitter=0`), on an L40S host.

| Run | Lot A | Lot B | `task_score` |
|---|---|---|---|
| Scripted demo (final) | 1.0 | 1.0 | 1.0 |
| Dream-1, 1,000 steps per trial | 0.0 (step limit) | 0.0 (step limit) | 0.0 |

In the earlier trace of the same demo the front arm opened to 3.9 degrees during the push and shut again; the bottle
ended with its base rim on the floor (0.0 mm), its base centre 1.8 to 2.0 mm off the cage axis (resting against the
cage's back), its axis within 0.05 degrees of the cage's, and the nearest hand point 88 mm away when the 1 s hold
completed. A first trace with a round bottle body scored 0: the lying bottle rolled before the grip and went in
crooked; the body is now a 16-sided prism (flats 0.7 mm inside the 74 mm circle) so it lies still.

## Fixture: bill of materials, build and set-up

| Item | Source | Price |
|---|---|---|
| Elite Custom Race X cage | https://www.biketiresdirect.com/product/elite-custom-race-x-bottle-cage (as cited by TK-LJWYLG) | $24.99 |
| Elite Fly Tex 550 ml bottle (the listing is the red one) | https://planetcyclery.com/products/elite-fly-tex-water-bottle-550ml-red (as cited by TK-LJWYLG) | $10.99 |
| 6061-T6 round tube 2 in OD x 0.065 in, 48 in (down tube, head tube) | https://speedymetals.com/p-4604-2-od-x-065-wall-tube-6061-t6-aluminum.aspx | $27.42 |
| 6061-T6 round tube 1-3/8 in OD x 0.058 in, 3 ft (top tube, seat tube) | https://www.aircraftspruce.com/catalog/mepages/alumtube_6061t6.php (part 03-37500, listed per foot) | about $32 |
| 18 mm and 12 mm plywood, 45 x 45 mm softwood | any home centre | about $40 (estimate) |
| M5 rivet nuts with a hand setting tool, M6 bolts, washers and nuts for the gussets, two C-clamps | any hardware store | about $45 (estimate) |
| Shipping and tax allowance | | about $30 (estimate) |
| **Total** | | **about $210** |

Prices were read from the listed pages on Nov 19, 2026; the rows marked "estimate" are not quoted. Tools: hacksaw or
mitre saw, drill, file, tape measure, steel ruler, 1.0 mm feeler gauge, phone (inclinometer, camera with slow motion),
luggage scale.

Build (all dimensions from the printed 1:1 drawing, checked against its 100 mm bar):
1. Cut the six tube pieces to the lengths in `cut-list.csv`; file the ends to fit against the neighbouring tube.
2. Lay the pieces on the printed drawing. Clamp each joint between two gusset plates (cut to the sizes in the cut
   list; pack the 1-3/8 in tubes with 8 mm plywood spacers to the 2 in thickness) and bolt through with M6 bolts.
3. Drill 6.8 mm holes in the down tube at the two marked boss positions (64 mm apart, on the tube's upper face) and
   set M5 rivet nuts. Bolt the cage on with its floor at the bottom of its slots; check the floor's middle is 35 mm
   below the lower bolt.
4. Screw the block and post to the base board and the frame to them, tipped 10 degrees nose-down (check with the
   inclinometer on the down tube: 51 degrees from level), the cage floor's middle 250 mm above the bench. Clamp the
   board to the bench with the frame plane 430 mm in front of the arm bases and the cage floor's middle 515 mm to the
   robot's right of the midline.
5. Measure once and record: the room above the seated bottle's cap along the cage axis (60 to 70 mm in the model);
   the force at the arm's tip to open it 15 degrees (about 18 mm at the tip in the model, where 5 degrees is 6 mm)
   with a luggage scale (3.9 N in the model; a real reading above 5 N is noted with the results).
6. Mark for scoring: the cage's centreline on the cage floor's front edge and on its open near edge, the arm tip's
   rest position with tape, and the 30 degree position on a card behind the tip.

Reset (numbered steps, per trial):
1. Pull the bottle out of the cage sideways toward the robot (the arm springs open and shut).
2. Lay it on the bench on its side for the lot: lot A base centre at (0.250, -0.300) cap toward the robot's left;
   lot B base centre at (0.300, -0.170), turned 25 degrees anticlockwise from lot A. Use tape marks on the bench.
3. Check the arm is shut and the frame has not moved against its marks. Nothing is consumed.

Safety: the fixture is bolted to its base and the base clamped to the bench; there are no blades, heat, motors or
stored energy beyond the cage's light spring; the push needs under 5 N and the bottle weighs 51 g.

## Sources for the geometry

| Item | Source |
|---|---|
| Frame tube centrelines and bosses | Salsa, "Rangefinder Front Triangle Frame Bag Guide", XS, a 1:1 vector drawing: https://cdn.shopify.com/s/files/1/0912/4973/8003/files/470236_SAL_Frame_Bag_Guide_Rangefinder_VF_OL.pdf (read at 25.4/72 mm per PDF point; bosses 64 mm apart) |
| Cage diameter, height, mass, material, side entry | Elite spec sheet: https://api.elite-it.com/assets/documents/Schede-tecniche/CUSTOM-RACE-X_2026_EN.pdf |
| Bottle diameter, height, mass | Elite spec sheet: https://api.elite-it.com/assets/documents/FLY-TEX_2026_EN.pdf |
| Cap size, cage floor 35 mm below the lower bolt, bottle-to-tube gap 8 mm, arm spring 1.3 N closed / 3.9 N at 15 degrees | estimates, taken from TK-LJWYLG's README; to be measured on the real cage (set-up step 5) |
| Cage wall shapes (spine width, lip and wall heights, band 48-88 mm, hinge 75 degrees round) | estimates from the maker's product photo |

## Known limits

- The frame is tipped 10 degrees nose-down in its base, as a bike often sits clamped in a repair stand; this turns the
  cage axis from 61 to 51 degrees above level and brings the cage within the YAM's reach. Everything that makes the
  task hard is fixed to the frame (the top tube over the cage, the closed triangle, the cage's side opening), so it is
  the same at any tip. Front wheel, fork, cranks and rear triangle are not part of the fixture; none of them is on the
  near side of the cage (the near crank is turned back out of the way when filling a bottle on a bike).
- The fixture is not a bicycle: the curved lower down tube is two straight pieces and the joints are gusseted, not
  welded. The tubes the hand passes (top, down and seat tube round the cage) follow the drawing to about 2 mm; the
  gussets sit at the three corners, away from the bottle's path.
- The cage's flex is lumped into one hinged front arm with a spring; the real cage flexes in its band and sides. The
  spring numbers are estimates until measured.
- The bottle is rigid; a real empty bottle squeezes a little, which makes the real push easier.
- Mainly one-armed: the fixture is fixed, so the second hand has nothing to steady. The difficulty is the blocked
  straight approach, squaring the bottle to a sloping cage, pushing against the arm, and letting go and drawing the
  hand back out of a closed triangle with millimetres to spare.
- Two lots stop a single memorised replay; a policy that memorised both layouts is not ruled out.
- In this embodiment the YAM's fingers are about 68 mm wide, so the grip spans 92 to 160 mm up the bottle; a person
  holds it nearer the top.
