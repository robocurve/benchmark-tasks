# Hold a mail drop box's spring-shut hopper open, lay three letters in, then let it shut so its return posts them

Posting letters at a wall-mounted collection box, the kind that stands beside a bank of wall mailboxes: the hopper
door has to be pulled down by its handle and held, because it shuts by itself as soon as it is let go. One hand
holds the hopper open; the other takes three stuffed #10 envelopes from a letter rack beside the box and lays them in
the hopper. Then the hopper is let back, and only its return posts them: as it shuts, its floor tips down into the
box and the envelopes slide off inside. Every envelope must end inside the box, none on the table or the floor, and the
hopper shut.

## Why it matters

Mailing a letter at a collection box is an everyday errand. The Postal Service's "more than 140,000 blue boxes" are
"the most visible part of its collection network", which also counts lobby drops (USPS OIG, 2022). Florence's hopper
collection box, the unit modeled here, is sold to "finish off your centralized mail installation" beside 4C wall
mailboxes and is "USPS Approved to STD-4C regulations". Collection boxes have opened by a pull-down door for a long
time (the 1889 Doremus box had "a lid near the top ... that pulled down to reveal the letter slot", and from 1955
post-mounted boxes were lettered "PULL DOWN", USPS Historian), and their handles leave the deposit to the other hand:
a 1996 patent proposing a one-handed replacement says the staple-shaped handle "in current use" on the Postal
Service's 1170/K collection box "puts the opening hand on the UNDER side of the closure door", and that such
receptacles' openers "require the second hand to effect a deposit".

The hopper is what makes it a robotics problem. It is a sprung moving receptacle that one hand has to hold steady
against its springs while the other hand lays thin, flat items in over its door. Open, it closes off the box (on the
unit, "when open, the collection door ... seals off access to the main collection compartment"), so nothing can be
pushed straight in, and it is the hopper's return that delivers the letters. The order matters: hopper open and held,
letters wholly in, then let back. Putting envelopes inside a mailbox is a BEHAVIOR-1K activity (mailing_letters: its
goal is only that every envelope is inside the mailbox); here the receptacle moves, has to be held against its
springs, does the delivery itself, and has to be left shut and let go.

## The scene

Positions in cm from the midpoint between the arm bases (x forward, y to the robot's left), heights above the table
top.

- **Drop box (full-size mock of the hopper section of a Florence versatile 4C collection box, model 4C11S-HOP,
  drawing 4C11S-HOPCS):** a 3/4 in plywood face 44.5 cm (17 1/2 in) wide and 34 cm high, toward the arms at x 42, its
  middle at y 0, standing on the table. The hopper opening in it is 30.5 cm wide, from 9.6 to 28 cm above the table.
  Behind the face, a compartment 30.5 cm (12 in) wide and 38.1 cm (15 in) deep, the drawing's compartment width and
  depth, with 1/2 in plywood sides, top, back and floor.
- **Hinge:** a 3/8 in steel rod, 9 cm above the table and 1.2 cm in front of the face, turning in two pillow blocks
  screwed to the face beside the opening; the rod is pinned through lobes on the hopper's cheeks, so it turns with the
  hopper.
- **Hopper:** a 1/4 in plywood door 33 cm wide and 20 cm (7.9 in) from the hinge to its top edge, overlapping the
  opening; a floor 28.9 cm (11 3/8 in, the drawing's largest bundle) wide, glued along the door's foot, that runs level
  through the face for 3.8 cm, then 14 cm down into the box at 34 degrees; two 1/8 in hardboard cheeks. 0.70 kg.
  Pulled by its handle, its top swings out toward the arms and down, to a stop at 45 degrees (where the door's top
  stands about 15 cm proud of the face; the drawing shows the open door 5 7/16 in proud). Opening raises its floor, so
  the open hopper is a V between the door and the floor. Shut, the floor slopes 34 degrees down into the box.
- **Curved back:** fixed inside the box behind the hopper, a 1/4 in hardboard plate (the drawing's curve) from the
  face's back just above the opening, bending round the hinge, its inner face on an 18 cm (7 in) radius to 10 degrees
  below level. From 20 to 45 degrees open the end of the hopper's floor lies about 6 mm from it, so the open hopper is
  a closed pocket and the way into the box is shut. Shut, the floor's end is 13 degrees below the curved back's lower
  edge, and letters slide off under it.
- **Springs:** two Lee Spring LTR070M01S stainless torsion springs on the rod's ends, outboard of the pillow blocks,
  both the same way round (not mirrored), each with one leg in a shaft collar pinned to the rod and the other on a
  screw in the face, wound 40 degrees at shut and wound tighter as the hopper opens. Each is rated 6.998 lb-in
  (790.7 N mm) at 90 degrees of wind: together 0.70 N m holding the hopper shut and 1.49 N m at the stop.
- **Handle:** an Amerock BP40517 bar pull (128 mm center to center, 187 mm long, a 13 mm bar standing 35 mm proud),
  level across the door's middle, its bar's top 1.5 cm under the door's top edge, where the unit's user access handle
  is.
- **Letters:** three #10 envelopes (Quality Park 11112: 9 1/2 x 4 1/8 in, 24 lb paper), each holding one 20 lb letter
  sheet folded in thirds: about 10 g and 0.6 mm thick each. They stand together on their long edge in the 1 cm slot of
  a letter rack at x 30, y 30 (27 cm long along y, slot walls 4 cm high), leaning about 12 degrees, the
  stamp corner up and to the robot's right. The rack is fixed to the table, as is the box.
- Both arms start at their home poses.

Nothing gets into the compartment below except by the hopper's return: shut, the face is closed apart from the hopper
opening and the shut door covers it; open, the curved back closes off the compartment behind the hopper. The open
hopper's floor itself lies behind the face, inside the box.

### Mock vs unit

The mock copies the unit at full size where the robot touches it (door, floor, handle place, opening, compartment
width and depth). It differs here:

- **Height:** the unit's handle is 60 7/8 in above the floor; the mock stands on the work table, its hinge 9 cm above
  the table top.
- **Compartment height:** the unit's compartment is 20 5/8 in deep below the hopper; the mock's is only as tall as the
  box (34 cm), and the hopper's floor ends 1 cm above the compartment's floor, where the letters land.
- **Handle:** the unit's handle is a pull across the door's top edge; the mock's is a 5 in cabinet bar pull in the same
  place.
- **Closing:** Florence does not publish the hopper's closing mechanism; the mock closes with two named torsion
  springs, and their pull is checked on the rig (below).
- **Hopper profile:** the door, floor and curved back follow the drawing's side view; the unit's anti-fishing comb is
  not copied (the curved back and the floor's end do its job: nothing passes between them).
- **Material:** the unit is aluminum; the mock is plywood and hardboard.

## Run a trial

Scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/post-letters-through-pull-down-hopper-drop-box/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Post the three letters from the letter rack in the drop box, and leave its hopper shut." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/dropbox-demo
```

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`. Video:
`inspect-robots video logs/dropbox-demo/*.json`. The scorer (`task_score`) reports the score each step; the trial's
score is the one at its end. `DROPBOX_TRACE=<file>` writes one line per control step (the hopper's angle, the widest it
has been open, whether an arm is on it, each envelope's middle in the box's frame and whether it counts, envelopes
that came down outside). The run log says "[dropbox] hands-off check: 16 link poses from the sim, hull check on" once
the arms' collision hulls are read. Offline checks: `python3 test_task.py` in this folder.

## Runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (2 minutes) per trial.
- Scripted demo: open loop; the recorded run scored 1.0 and ended at step 782 (about 39 s).
- No randomness: `-E object_jitter=0` keeps every object on its mark.

## Scoring

Judged every control step from the poses the simulator reports.

- **An envelope is in** when it is wholly inside the box (every corner behind the face's back plane and inside the side
  walls, back, floor and top), at rest, and no part of either arm is within 5 mm of it (a release check).
- **Set down or dropped outside:** an envelope that comes down on the table or the floor outside the box (its middle
  outside the box's footprint and a corner within 4 mm of the table top, or below it) counts for nothing for the rest
  of the trial, even if it is picked up and posted afterwards. Standing in the rack is not "on the table".
- **Shut:** the hopper within 0.8 degrees of closed (its top edge within 4 mm of the face).
- **Score:** 0.25 for each envelope that is in (0, 0.25, 0.5, 0.75), at most 0.5 while the hopper is not shut. 1.0 when
  all three are in, the hopper is shut and let go (released: no part of either arm within 5 mm of its door or handle),
  held for 1 s; the trial then ends. Possible scores: 0, 0.25, 0.5, 0.75, 1.0. Doing nothing scores 0.

Envelopes in the open hopper's V in front of the face do not count; ones that have slid past the face onto the open
hopper's floor count, but the score stays at most 0.5 until the hopper is shut. A hopper jammed open on an envelope is
not shut. A hand still holding the handle at the end withholds the 1.0.

## Real-world build (a low-cost setup)

A full-size mock of the hopper section, from ordinary materials; common shop tools are assumed.

- Box: 3/4 in plywood face, 17 1/2 x 13 3/8 in, with a 12 x 7 1/4 in opening whose bottom edge is 3 3/4 in above the
  table. Behind it a compartment 12 in wide and 15 in deep inside: 1/2 in plywood sides, top and floor, screwed and
  glued; the back screwed only, so it comes off to count and reset.
- Hinge: a 3/8 in steel rod in two 3/8 in bore pillow block ball bearings screwed to the face beside the opening, its
  axis 1/2 in in front of the face and 1/4 in below the opening's bottom edge.
- Hopper: 1/4 in plywood door (13 x 7 5/8 in, its bottom edge 1/4 in above the rod's axis); 1/4 in plywood floor
  11 3/8 in wide, a 1 1/2 in level strip glued along the door's foot, then a 5 1/2 in run glued on at 34 degrees; two
  1/8 in hardboard cheeks cut to the profile in `task.py`, each with a lobe round the rod; the rod pinned through the
  lobes. Weigh the hopper with its pull and glue washers under the floor to bring it to 0.70 kg. A block inside the
  face stops it at 45 degrees.
- Curved back: two layers of 1/8 in hardboard, each wetted on its back, bent over three plywood formers screwed to the
  compartment's sides and glued together (1/4 in in all), its inner face on
  a 7 in radius about the rod, from just above the opening's top edge to 10 degrees below level.
- Springs: two Lee Spring LTR070M01S torsion springs on the rod's ends outboard of the pillow blocks, both slid on
  the same way round (not mirrored: both are right-hand wound); each has one leg in a 3/8 in shaft collar pinned to
  the rod and the other on a screw in the face. Set them wound 40 degrees with the hopper shut, so that opening winds
  both tighter (their coils close up).
- Handle: an Amerock BP40517 bar pull, level across the door's middle, its bar's top 5/8 in under the door's top edge.
- Letters: three Quality Park 11112 #10 envelopes, each with one sheet of 20 lb copy paper folded in thirds, sealed.
- Rack: a 1/4 in plywood base 27 x 6.2 cm with two 1/4 in plywood strips 27 x 4 cm glued upright on it, 1 cm apart
  in its middle (the slot).
- Mounting: two steel corner braces screwed to the box's sides at the back, each held to the table by a C-clamp; the
  rack held down with double-sided tape. Both are fixed in the simulation.
- Tools assumed: a digital luggage scale with peak hold, a kitchen scale (1 g), a steel ruler, a spirit level, 4 mm
  and 8 mm drill bits as feeler gauges, and a phone (its level app reads angles).

## Real-world evaluation

Marks (once): the midpoint between the arm bases is taped on the table. From it, the box's face (its front surface)
stands 42 cm forward, square to the arms, with its middle on the center line (y 0), and the rack's middle is 30 cm
forward and 30 cm to the left, its slot running left-right. Tape a line along the foot of the face and round the
rack's base; both stay within 3 mm of their marks.

Build check (after building, then at the start of each session):
- the opening 30.5 cm wide, its bottom edge 9.6 cm and its top edge 28 cm above the table; the hinge rod's axis 9 cm
  above the table and 1.2 cm in front of the face; each within 2 mm (steel ruler). The box level both ways (spirit
  level on its top).
- the hopper weighed at 0.70 kg (within 20 g) before it is hung; at its stop, the door reads 45 degrees from upright
  (within 2 degrees, phone level on the door).
- with the back off and the hopper held 30 degrees open (phone level), the gap between the floor's far end and the
  curved back takes the 4 mm bit but not the 8 mm one.
- with the back off and the hopper shut, a sealed letter laid flat on the hopper's sloping floor slides off it by
  itself (the paint is not tacky).

Before each trial (about 2 minutes; the trial does not start until every check passes):
1. Pull: the luggage scale hooked on the middle of the handle's bar, pulled square to the door slowly (about 2 s):
   4.5 to 6 N as the hopper leaves shut, 6.5 to 8.5 N at the 45-degree stop (the model: about 5.3 and 7.5 N). If it
   is out of the band, check the rod turns freely in its blocks (clean it, square the blocks) and the springs' legs
   sit on their screws; replace both springs if it is still out.
2. Return: let go from the stop, it shuts by itself; let go 10 degrees open, it shuts by itself; shut, the 4 mm bit
   does not go in at the door's top edge.
3. Letters: the three together weigh 30 g (within 2 g); each is sealed, uncreased, and lies flat on the table with no
   corner more than 3 mm up; a spare set replaces any that fails. They stand together in the rack's slot on their long
   edges, centered along it (1.5 cm from each end), leaning on the far wall, stamp corner up and to the robot's right.
4. The box's back screwed on, the hopper shut, both arms at their home poses.

Trial: run the policy for up to 2 minutes (the run limit). Video: a phone at 30 fps or more on a stand 60 cm to the
box's right, level with the hinge, seeing the hopper's whole swing, the face and the table in front of the box; a
second phone overhead, seeing the table round the box and the rack; plus the robot's cameras. The operator holds the
stop and stops the arms if one jams on the box or the hopper pinches a finger; a stopped trial is scored as it stands
when stopped.

Scoring, in this order:
1. The hopper, before anyone touches it: the 4 mm bit at the door's top edge (if it does not go in, the hopper is
   shut); from the side video, neither arm touches the door or its handle in the last second.
2. From both videos: any envelope that touched the table or the floor outside the box at any time counts for nothing.
3. Unscrew the box's back and count the envelopes lying wholly inside it (behind the face). One still in the open
   hopper's V in front of the face is not in.

Reset (about 3 minutes with the checks): letters back in the rack's slot, hopper shut, back screwed on. Nothing is
consumed.

## Fidelity

Rigid bodies at full size. The hopper turns on a revolute joint; the two springs are PhysX's own joint drive at the
springs' rated stiffness and preload; the stops at shut and at 45 degrees are the joint's limits. The rod and the
springs are not colliders (the joint and its drive stand for them); the pillow blocks are. The handle, the envelopes,
the rack and the curved back act by real contact only (friction 0.4 static, 0.3 dynamic for paper on paper, wood and
painted plywood; the letters slide off the shut floor for any friction below its slope, tan 34 degrees = 0.67). The
envelopes are rigid plates at their real size and weight; real envelopes flex a little.

## Files

- `task.py`: the scene, the scorer and the scripted demo.
- `test_task.py`: offline checks of the scorer and the layout.
- `yamkit.py`, `yamgeom.py`, `guardkit.py`: scene and geometry helpers.

## Sources

- Florence Manufacturing, versatile 4C Collection Box 4C11S-HOP, drawing 4C11S-HOPCS, rev. E (17 1/2 in actual width,
  collection compartment 20 5/8 x 12 x 15 in, hopper door shown open 5 7/16 in proud, largest bundle 11 3/8 in wide,
  user access handle across the door's top): florencecommercialmailboxes.com/image/catalog/Downloads/4C11S-HOPCS.pdf
- Lee Spring LTR070M01S torsion spring: max torque 6.998 lb-in (790.668 N mm), deflection to max torque 90 degrees,
  stainless steel, right-hand wind, for a 0.385 in rod: leespring.com/product/torsion-spring-ltr070m01s-stainless-steel
- Amerock BP40517 bar pull: 128 mm center to center, 187 x 13 x 35 mm (Würth Wood Group, Build.com and DK Hardware
  listings).
- Quality Park 11112: #10 business envelope, 4 1/8 x 9 1/2 in, 24 lb white wove (Staples listing).
- Florence versatile 4C11S-HOP product page ("Finish off your centralized mail installation with the Florence 4C
  hopper collection box"; "USPS Approved to STD-4C regulations required for all new construction and major renovation
  projects"): florencemailboxes.com/products/std-4c-mailboxes/4c-recessed-mount-mailboxes/versatile-4c11s-hop/
- Mailboxworks listing of the 4C11S-HOP ("When open, the collection door extends 5-7/16" from the unit and seals off
  access to the main collection compartment"):
  mailboxworks.com/product/florence-4c-mailbox-hopper-collection-drop-box-4c11s-hop
- USPS Office of Inspector General, "The Postal Service's Collection Point Management System", RISC-WP-22-007, May 18,
  2022 ("the Postal Service's more than 140,000 blue boxes ... are the most visible part of its collection network";
  collection points include "lobby drops"): uspsoig.gov/sites/default/files/reports/2023-01/RISC-WP-22-007.pdf
- USPS Historian, "Mail Collection Boxes: A Brief History", March 2017 (the 1889 Doremus box's pull-down lid; from
  1955, "PULL DOWN" painted on post-mounted boxes): about.usps.com/who/profile/history/pdf/mail-collection-boxes.pdf
- US Patent 5,482,208, "One-handed opening device" (B. Johnston, 1996), on the handle of the USPS 1170/K collection
  box: patents.google.com/patent/US5482208A
- BEHAVIOR-1K activity mailing_letters (three envelopes; goal: each inside the mailbox):
  github.com/StanfordVL/BEHAVIOR-1K, bddl3/bddl/activity_definitions/mailing_letters/problem0.bddl

## Sources and licenses

No third-party data files are included: every part of the scene is built in `task.py` from the published dimensions
above, and no drawing, image or mesh is shipped. The robot model and its collision hulls load at run time from the
competition environment (arena revision 0059d02a) and are not redistributed.
