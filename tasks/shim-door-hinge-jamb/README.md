# Shim a door's hinge jamb with two opposed tapered shims

A section of a stud wall stands on the table with the middle-hinge part of a prehung door's hinge jamb beside its jack stud, a 4.0 to 6.35 mm
(5/32 to 1/4 in) gap behind the jamb at its middle hinge. The robot takes one 8 in wood shim from each holder, pushes them into the
gap thin end first from the two faces of the wall (one per hand), and drives the pair until it is snug against the
jamb without bowing it, then lets go. Everything the shims touch (studs, board, the jamb's cross-section, the gap, the
shims) is true size; only lengths are cut. The jamb section rides on a spring with the stiffness the whole jamb has at that
hinge, so pushing too far bows it, and the gap changes from trial to trial, so where to stop has to be felt.

## Run and score

With the official simulator installed, run from this folder at 20 Hz:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=shim_door_hinge_jamb.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --seed 0 \
  --instruction "Shim behind the door's hinge jamb at its hinge: push one wood shim in thin end first from each face of the wall until the pair is snug, without bowing the jamb." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ./demo-run
```

The model run uses the same command with `--policy dream`. Encode recorded cameras with
`inspect-robots video <run.json>`. Scores are in task.json.

**Seeds and gaps.** The trial's seed sets the gap: g = 4.0 + 0.05 x (seed mod 48) mm, from 4.0 to 6.35 mm (5.0 mm
with no seed). Inspect Robots derives each trial's seed from `--seed` and the epoch (`--seed 40 --epochs 2` gives 6.35
then 4.45 mm; `--seed 102` gives 4.0 mm; `--seed 24` gives 6.35 mm). The range lies within the 1/8 to 1/4 in a rough
opening usually leaves each side of a prehung door's jamb, and stops at 1/4 in. It ran to 6.5 mm in an earlier build;
we cut it to the 1/4 in that the sources support because there the robot failed: a pair of these shims must overlap
189 of its 203 mm to bear on the jamb at 6.5 mm, and each shim's tip then reaches the other hand's far finger first.
At each reset the fixture's gap-setting slide is put at g and locked there by closing its
travel limits on it, as the rig's screw stop is locked (the only state written, and only at reset); the jamb's spring
is then unloaded. Joint stiction alone did not hold the slide: under the spring's 1 N it crept open. The
shims are dynamic bodies but are not jittered by `object_jitter`, because they stand in 7 mm slots.

**CPU checks.** `python -m pytest -q -p no:cacheprovider` in this folder (with yam_arena installed) runs the scorer
on the start, the goal and named partial and cheat layouts, and `test_red_team.py` checks the wedge geometry, the
gap per seed, every scoring band and shortcut, the release and stillness windows over control steps, and the
demonstration's plan at several gaps.

Runtime per trial: at most 3,600 control steps (180 s simulated). The demonstration's plan takes at most 57.5 s (55.7 s at 0.8x
pace: only the pick, lift, carries, back-offs and park speed up; every move at or in the wall keeps its duration); on the GPU it
lets go of the pack by about 43 s and reaches full score at about 46 s.

## Scoring (task.json has the full text)

- Pair: one shim entered from each face of the wall, each thin end leading; both lying in the gap (length across the
  wall within 3 degrees, middle within 25 mm of the hinge's centre height, their part between the jamb's edges between
  the stud's face and the jamb's back); tapered faces on each other (flat faces out, either way round); overlapping
  70 mm or more, with the overlap covering the middle 40 mm of the jamb's width.
- Snug: the pack bears on the jamb: the free space left between the stud and the jamb at the jamb's centre line, gap
  minus the two shims' thicknesses there, is 0.05 mm or less (on a real door: a 0.05 mm feeler gauge, the thinnest
  blade, will not enter between the pack and the jamb there). A pack with any real free space is loose, not snug.
- Bow: how far the jamb section stands from where the trial set it (its spring's deflection).
- 1.0 pair, snug, bow <= 0.4 mm (1/64 in); 0.75 bow <= 0.8 mm (1/32 in); 0.5 bow <= 1.6 mm (1/16 in); 0.25 a snug pair bowing the
  jamb more than 1.6 mm, a pair not snug with <= 1.5 mm of free space, or (no pair) one shim alone in the gap at the
  hinge touching both the stud and the jamb; 0 otherwise.
- The score is read once both grippers' centre points have been 30 mm or more from both shims and the jamb for 2 s and
  the shims and jamb have moved under 0.5 mm, 1 degree and 0.05 mm over the last second; until then it is at most
  0.25. The scorer reads only poses, joint positions and the grippers' centre points; no contact forces.
- Shortcuts and what they score: one shim only (the quickest fill) 0.25 at most; the driven shim rammed on (on the GPU,
  80 mm past where the pair touches) bowed the jamb 1.0 mm while pushing and 1.2 mm once let go, and the shim turned
  3.8 degrees in the hand, so the pair was out of line: 0 (straight, 1.2 mm would score 0.5; the 0.75 band ends at
  0.8 mm, below the 1.0 to 1.2 mm the ram made, so ramming earns 0.5 at most); the same depth every trial, because g
  changes by trial, reaches 1.0 only on gaps within 0.45 mm of the one it suits: computed with the scorer over all 48
  gaps, the best fixed pack (5.60 mm) scores 1.0 on 10 of them (21%), mean 0.57, the pack bowing the jamb (0.75 or
  0.5) or left loose (0.25) on the rest (an earlier draft of the scorer counted up to 0.3 mm of free space as snug and
  gave 1.0 up to 0.5 mm of bow and 0.75 up to 1.0 mm; there the same shortcut scored 1.0 on 17 of the 48 gaps (35%),
  mean 0.68, and a ram bowing 1.0 mm or less earned 0.75); both shims from one face, a shim butt first or a pack
  beside the hinge are not a pair; a pack that only touches without force cannot hold the shims up, and they slide out.

## The jamb section and its stiffness

The jamb section is the middle-hinge part of an ordinary prehung interior door's hinge jamb, true size in cross-section
and cut only in length (270 mm), at the moment that hinge is shimmed: the top and bottom hinges have already been shimmed and tacked, which is the usual order and the
place where over-shimming bows the jamb. Pushed away from the stud at the middle hinge, the jamb bends as a beam
between those two tacked points. The fixture gives the section that beam's stiffness, on a slide, so the robot feels
what a carpenter feels.

- Section: 11/16 x 4-9/16 in (17.46 x 115.89 mm) finger-jointed pine, bent about its thin axis:
  I = b t^3 / 12 = 115.89 x 17.46^3 / 12 = 51,425 mm^4.
- Span: hinges by the common 7/11 rule on an 80 in door (top hinge 7 in from the top, bottom hinge 11 in from the
  bottom, 3-1/2 in leaves), so the top and bottom hinge centres are 58.5 in = 1486 mm apart, the middle hinge halfway.
- Modulus along the grain, 12 % moisture: about 8.5 GPa (eastern white pine) to 10.2 GPa (radiata pine), the usual
  published values for the pines finger-jointed jambs are made of.
- A point load at mid-span: pinned ends (tacked with finish nails through the shims) k = 48 E I / L^3 =
  **6.4 to 7.7 N/mm**; fully fixed ends k = 192 E I / L^3 = 26 to 31 N/mm. Tacked shim packs are much nearer pins
  than clamps, so the task uses **7 N/mm**, the softest credible value; a stiffer real jamb needs proportionally more
  push for the same bow, so the task does not get easier on a real door.
- Over the section's 270 mm the beam's deflected shape falls only to 0.92 of the hinge's deflection at the ends, so a
  rigid section that moves as a whole matches it closely.
- Twist: the jamb's torsion stiffness at the middle hinge, with its ends held by the tacked shim pairs, is
  4 G J / L = 280 to 500 N m/rad (J = 186,000 mm^4, G = 0.55 to 1.0 GPa), i.e. 4.8 to 8.8 N m per degree. The largest
  pack force the robot's push can make (about 13 N) acting at the jamb's edge would turn it by less than 0.15 degree,
  so the section has no twist joint. What twists a real jamb is the hinge screw pulling it onto a pack that supports
  only one edge; the scoring therefore asks for a pair whose flat overlap lies under the jamb's middle.
- Damping (50 N s/m, about 0.6 of critical for the 0.35 kg carriage) only lets the section settle; it changes no
  force at rest.

## The shims

8 in wood shims as sold for door hanging (Nelson-type kiln-dried cedar; the maker lists "approx. 8 in x 1-3/8 in x
1/4 in", and retail listings give 1/32 in at the feathered tip): 203.2 long, 34.9 wide, 6.35 thick at the butt, 0.79 at
the tip, one face flat, so the taper is 0.02734 (1.566 degrees). Volume 25.3 cm^3; at 0.37 to 0.45 g/cm^3 that is 9 to
11 g; the task uses 10 g. Two shims tip past tip, inclined faces on each other, make a flat pack 1.58 + 0.02734 x
overlap thick: 4.0 mm at 88 mm of overlap, 5.0 at 125, 6.35 at 174. One millimetre of push moves the pack 27 um.

## Wedge statics (why the robot can do it, and why ramming is caught)

The driven shim slides on the fixed face (stud or jamb) and on the other shim's inclined face (taper a). With pack
force N between stud and jamb and friction mu, the push needed is
P = N (sin a + mu cos a) / (cos a - mu sin a) + mu N, about N (2 mu + a).
The jamb's spring sets N = 7 N/mm x bow. With mu = 0.4 to start and 0.3 while sliding:

| bow (mm) | pack force N (N) | push to start (N) | push while sliding (N) | score band |
|---|---|---|---|---|
| 0.04 | 0.28 | 0.23 | 0.18 | least that holds two 10 g shims up by friction |
| 0.25 | 1.75 | 1.46 | 1.10 | 1.0 |
| 0.40 | 2.80 | 2.33 | 1.76 | 1.0 / 0.75 boundary |
| 0.80 | 5.60 | 4.66 | 3.53 | 0.75 / 0.5 boundary |
| 1.60 | 11.2 | 9.32 | 7.06 | 0.5 / 0.25 boundary |

So a snug, straight pack needs under about 2.5 N of push, well inside what the arm delivers, while pushing on (an arm
pushing 6 to 8 N would bow the jamb 1.4 to 1.8 mm) costs credit band by band; on the GPU, driving the second shim 80 mm
past touch bowed the jamb 1.0 mm while pushing and 1.2 mm after letting go. The pair is self-locking: each newton of pack
force pushes a shim back out with only 0.027 N against 0.8 N of static friction, so the pack stays put when the hands
let go. A pack that only just touches (no force) cannot hold the shims up, and they slide down out of the gap: as on
a real door, "snug" has to mean a little force.

## The demonstration

Each hand picks its shim from its holder near the butt (193 mm from the tip, the pads on its two faces 9 mm below its
top edge; above a 6.0 mm gap, 196 mm), with the grip pitched 20 degrees about the finger axis so that the hand's top
leans away from the wall (straight down, the forearm would pass within 2 mm of the jamb at the largest gaps). Above
about 6.1 mm the shim is thinner where it is held than the gap, so the far finger stands where the other shim's tip
slides in, about 11 mm from the hand's centre; holding nearer the butt leaves room for the deeper overlap, but a pinch
on only the last 13 mm of the shim let it roll, turn and tip in the hand (GPU), so the grip moves just 3 mm. A tapered
shim pinched between parallel pads sits a little turned in the hand (on the GPU 0.7 to 1.7 degrees, enough to put the
tip 4 mm off line) and a little rolled about its length (up to 2 degrees), so once each shim is clear of its holder
the hand reads the shim's yaw and roll and turns the wrist by them until the shim runs straight across the wall and
stands upright. The left hand brings shim A to the left face, waits for the arm to
settle and centres A's tip on the gap's mouth (it reads the tip and corrects), closes in slowly to 3 mm from the face
and centres it again (moved quickly along the wall, the arm swings the tip 3 to 5 mm sideways), enters slowly, lays
the flat face on the stud, reads the tip again and corrects sideways once (the arm stands up to about 1 mm off its
command sideways), pushes A in to the depth that puts the pair's overlap under the jamb's middle for this trial's gap,
and holds it there. The right hand then enters shim B from the right face the same way (at large gaps A's tip stands
up to 32 mm out of that face, so B is squared and aimed again beyond A's tip and slid in slowly past it), lays its
flat face flush on the jamb's back (read and corrected once, so that B's tip passes A's with the whole free space
between them), pushes it quickly to 15 mm short of the depth at which the pair would touch, and then in 2 mm steps,
while the left hand leans A in 1 mm a step (at most 20 mm) so that A keeps its depth against B's push, reading the
jamb's position between steps, until the jamb has moved 0.25 mm (about 1.75 N of pack force: firm enough that the
pack holds both shims level when let go) with the pair's own thickness filling the bowed gap; a person feels the shim
stiffen instead. (Shims cocked in the hands also bow the jamb, and that bow vanishes when they are let go, so such a
bow does not stop the steps.) At the widest gaps the steps also stop before either tip comes within 13.5 mm of the
other hand (about 0.22 mm of bow at 6.35 mm). Both hands then take out their pushes along the shims together (a hand
runs a few millimetres ahead of a shim it is pushing; eased one at a time, the other hand's push shoves the pack) and
stay where they actually stand sideways and in height (a pushing arm sags a few millimetres; sent back up to its old
height it levers the shims), so that opening the fingers does not tug the pack; the right hand opens, backs away and parks, then the left. The demonstration reads
simulator state (shim poses, the gap, the jamb's position, its own hand) to plan its moves; it never writes to the
simulation.

## Real-trial rig

The jamb section is held at each end on a linear guide square to the stud, between opposed preloaded compression
springs (3.5 N/mm per end, 7 N/mm in all), with a screw stop to set the unloaded gap with feeler gauges at the top and
bottom; the stop is then locked (in the simulation, the slide's travel limits are closed on it at reset). Snug is checked with a
0.05 mm feeler gauge between the pack and the jamb at the hinge, bow with a dial indicator zeroed on the jamb's face at
the start, the overlap and height with a tape.

## Simplifications

- Only lengths are cut: the wall to 290 mm tall and 90 mm long, the jamb to 270 mm. Every cross-section, the gap
  and the shims are true size. The door, the jamb's other two hinges and the rest of the frame are replaced by the
  spring and guides above, which give the section the stiffness the whole jamb has at this hinge. No screws or
  nails are driven.
- Shims are rigid wedges of the real taper; real cedar crushes a little under load and has a rough feathered tip.
- Friction is Coulomb, 0.4 static and 0.3 sliding for every wood, gypsum and steel pair (published dry wood-on-wood
  values are 0.25 to 0.5 static and 0.2 to 0.3 sliding).
- The jamb section has no twist joint (see its stiffness above).

## Layout and colliders (millimetres; x forward, y left, z up from the table top)

| Object | Colliders | Material, mass | Notes |
| --- | --- | --- | --- |
| wall (fixed) | steel base plate x 150..330, y -90..90, z 0..6; king stud x 184..222 and jack stud x 222..260, y -44.45..44.45, z 6..296; gypsum board x 170..260, y 44.45..57.15 and -57.15..-44.45, z 6..296 | steel, SPF pine, gypsum | jack stud's face x = 260 is the rough opening's side |
| jamb fixture (fixed) | rail x 262..300, y -30..30, z 6..14; post x 300..316, y -20..20, z 6..300; arm x 262..316, y -20..20, z 288..300 | steel | carries the jamb section |
| gap-setting slide | hidden steel block in the rail | 0.20 kg | slide along x; set to g - 4.0 mm at reset and locked there (limits closed on it) |
| jamb section (sprung) | board x 264..281.46 (at g = 4.0), y -57.945..57.945, z 16..286; hinge leaf y -54.95..-22.95, z 85.5..174.5; knuckle r 6.35 at (287.46, -56.0), z 85.5..174.5; end clamps | primed finger-jointed pine, satin nickel, steel; 0.35 kg | slide along x on the setting slide, 7 N/mm, 50 N s/m, -3.5..+6.0 mm |
| holders (fixed) | block centred (190, +/-205), 50 x 220 x 30 tall, slot 7 wide and 8 deep along y | painted MDF | |
| shims (dynamic) | one convex wedge each: 203.2 long, 34.9 wide, 6.35 to 0.79 thick | cedar, 10 g | start on edge in the slots, butts 10 proud of the blocks' outer ends; left tip toward -y, flat face toward -x; right mirrored |

Every contact offset is 1 mm. Colours: wall paper face light grey, studs pale pine, jamb primed white, shims red-brown cedar,
holders blue-grey, steel grey.
