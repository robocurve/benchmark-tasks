# Checks for the sprung bed slat task

All runs on edward's TITAN RTX GPUs 0, 1, 4 and 5, Isaac Lab-Arena with yam_arena, 2,400-step limit, 20 Hz, one trial
per process.

## 1. Real hardware and where each number comes from

| Item | Value in the scene | Source |
|---|---|---|
| Category | Societal relatability: refitting a slat that popped out of a slatted bed base is an everyday household chore | |
| Slat | sprung (bowed) beech, 53 mm wide, 755 mm long: the replacement size sold for small single and toddler beds | [Green Design pack of 3 sprung beech slats 53 x 755 mm](https://www.desertcart.lk/products/522953930-green-design-pack-of-3-slats-53-mm-wide-beech); [Uncle Bob replacement sprung slats 755 x 53 x 8.5 mm](https://www.desertcart.nz/products/52754769-uncle-bob-pack-of-6-replacement-sprung-wooden-bed-slats); [53 x 8 mm sprung beech slats, eBay](https://ebay.com/itm/153678848029) |
| Slat section 53 x 8 mm | as listed for 53 mm sprung beech slats (8 to 8.5 mm thick) | same listings |
| Rest bow 20 mm crown up | estimate for a 755 mm sprung slat (sprung slats are sold pre-bowed) | estimate |
| Mass 0.25 kg | 0.227 kg of beech (53 x 8 x 745 mm at 720 kg/m^3) plus two 10 g caps; a 900 x 53 x 8 mm plywood slat is listed at 0.2 kg | computed; [B&Q/DIY.ie plywood bed slat 0.9 m x 53 mm x 8 mm](https://www.diy.ie/departments/plywood-bed-slat-l-0-9m-w-53mm-t-8mm/1822064_BQ.prd) |
| End caps and holders | plastic caps/holders for 51-53 mm slats, sold separately; the lip that retains the cap and the 45 deg lead-in are modelled on side-rail holders | [Replacement plastic caps / slat holders 51-53 mm, eBay](https://www.ebay.com/itm/Replacement-Plastic-Caps-Bed-Slat-Holders-51mm-53mm-wide-Choice-of-Caps-Qty-/221473484090); holder dimensions estimated |
| Bending stiffness | four hinges of 25 N m/rad on 133 mm segments: EI about 3.3 N m^2, 8x less than solid beech (American beech MOE 11.9 GPa gives 26.9 N m^2 for 53 x 8 mm) so that a YAM can bow it; stated in the fidelity note | [USDA Wood Handbook FPL-GTR-190, Table 5-3a (American beech: MOE 11.9 GPa, MOR 103 MPa)](https://www.fpl.fs.usda.gov/documnts/fplgtr/fplgtr190/chapter_05.pdf) |
| Crack limits | a hinge bent 0.20 rad past rest over its 133 mm segment is a 0.67 m radius, 0.6 % surface strain in 8 mm beech, about 70 % of its failure strain (MOR/MOE = 0.87 %); a 75 mm bow over the span is a 0.95 m radius | computed from the Wood Handbook values |
| Frame | two pine side rails with three holder pairs at 110 mm slat pitch, 6 kg, loose; rail section and frame mass estimated | estimate |

A real trial needs one replacement slat with caps (about $10), six slat holders (about $10) and two pine battens.

## 2. Why the difficulty is robotic

The slat cannot be laid in: at rest it is a little longer than the holders' wall-to-wall span, and the second cap
only clears its lip when the slat is bowed about 30 mm more than its rest bow, which shortens it about 7 mm. The
robot must seat one cap end-wise under its lip, then hold that end home while lifting the slat near mid-span and
drawing the free end in with the other hand, keep the slat over the holders' 64 mm pockets (the arms sag 10 to 20 mm
under the slat's spring), lower the free cap into its pocket, and then let the slat spring back to length without
letting go too early (the free end flips out) or bending it so far that it cracks. Two arms must work together
through a flexible part under load; no single grasp or placement does it.

## 3. Contact only

The slat is one articulation: five rigid beech segments joined by four revolute hinges whose drives are springs
(25 N m/rad) toward the rest bow; the caps are boxes on the end segments. The holders, lips and lead-ins are boxes
of the loose frame part. A cap stays in its holder only because the lip is above it, the end wall beyond it and the
slat's own spring pushes it outward. No joint, latch or proximity trigger ever connects the slat to a holder or a
gripper.

## 4. Observable inputs

The demo uses the top-camera images, the arms' joint angles (forward kinematics, including both finger joints) and
the frame layout and slat dimensions from the setup text. It finds the slat on the bench from its two grey end caps
in the top image (the caps are the only colourless mid-grey pixels in front of the frame). The simulated YAM drives
its two fingers separately, so a held slat slides across the jaws until one finger reaches its stop; the demo reads
where the slat sits from the two finger joints and keeps the slat, not the TCP, on the holders' centre line. It never
reads simulator object poses or the slat's hinges (`grep` of `script.txt` finds no `states(`, `_STATE`, `sliders`,
`write_pose` or `body_link_pose`). Demo cameras were 640 x 352.

## 5. Demo and baselines

Scripted demo, final task and script, one trial per process (the seed draws the slat's start offset and angle on the bench). Success is declared after the 1 s hold; the step count is when the hold began. 'Max bow / hinge' is the largest mid-span bow (mm) and hinge bend past rest (rad) during the run, against the 75 mm and 0.20 rad crack limits.

| Seed | Start (dx mm, dy mm, yaw deg) | Score | Steps to success | Cap x off the holder centre line (mm, -y / +y) | Bow at end (mm) | Max bow (mm) / hinge (rad) |
|---|---|---|---|---|---|---|
| 0 | +7.6, +18.6, -3.5 | 1.0 | 1801 | +1.9 / +1.9 | 32.6 | 59.1 / 0.153 |
| 1 | -1.2, +5.9, -1.3 | 1.0 | 1830 | +1.1 / +1.1 | 32.5 | 56.7 / 0.125 |
| 2 | -2.0, -7.8, +2.5 | 1.0 | 1748 | -2.5 / +2.6 | 34.3 | 58.1 / 0.128 |
| 3 | -7.0, -16.4, +3.8 | 1.0 | 1887 | -2.4 / -2.4 | 32.4 | 53.2 / 0.115 |
| 4 | -0.0, -10.2, -1.0 | 1.0 | 1835 | -2.5 / -2.5 | 32.2 | 57.0 / 0.149 |
| 5 | -2.3, +12.8, -0.2 | 1.0 | 1774 | -2.4 / +2.5 | 34.2 | 56.5 / 0.137 |
| 6 | -4.9, -3.7, -3.1 | 1.0 | 1888 | -2.4 / +1.1 | 33.8 | 57.2 / 0.132 |
| 7 | +4.7, -1.6, +3.0 | 1.0 | 1875 | -2.5 / -2.5 | 32.3 | 55.1 / 0.122 |

Baselines (same task file, 2,400 steps, `BASELINE_SEED` per run; 'naive' is the demo's grasp and carry, then the slat laid flat on top of the empty holders and let go):

| Baseline | Score | Gate | Caps (side, state) at end | Bow at end (mm) |
|---|---|---|---|---|
| noop seed 0 | 0.0 | none | [[-1, None], [1, None]] | 15.6 |
| noop seed 1 | 0.0 | none | [[-1, None], [1, None]] | 15.6 |
| random seed 0 | 0.0 | neighbouring slat 0 moved 11.9 mm | [[-1, None], [1, None]] | 15.9 |
| random seed 1 | 0.0 | none | [[-1, None], [1, None]] | 15.7 |
| random seed 2 | 0.0 | none | [[-1, None], [1, None]] | -3.6 |
| reach seed 0 | 0.0 | neighbouring slat 0 moved 11.3 mm | [[-1, None], [1, None]] | 1.9 |
| reach seed 1 | 0.0 | none | [[-1, None], [1, None]] | 15.5 |
| reach seed 2 | 0.0 | none | [[-1, None], [1, None]] | 15.6 |
| naive seed 0 | 0.0 | none | [[-1, 'in'], [1, None]] | 16.0 |
| naive seed 3 | 0.0 | none | [[-1, None], [1, None]] | 17.2 |
| naive seed 5 | 0.0 | none | [[-1, 'in'], [1, None]] | 16.1 |

Failure modes seen while building the demo (fixed in the script or the model, none in the scorer): the left hand
could not reach mid-span over the frame (now it grips 13 cm left of mid-span); the open jaws hit the near fitted
slat (the loose slat now starts 1 cm nearer the robot); the held slat slid 20 mm across the jaws to a finger stop and was laid on
a side wall (now read from the finger joints); imposing a bow shape with both hands made the grips slide along the
slat (now the left hand only lifts and the right only draws in); letting go with the right hand first while the slat
was bowed flipped the free end out (now the slat is unbowed with that cap held down, then released); the top camera
cannot see the caps over the frame because the arms are above them (not used there). The crown window is 10 to
40 mm because a refitted slat settles at a 32 to 34 mm bow: its caps bear on the end walls and lips, so it keeps some
spring preload above its 20 mm free bow (every demo seed ends there).

## 6. Overlap check (board, Nov 22)

Live `gh search prs --repo robocurve/benchmark-tasks` for "bed slat", "slat", "bow a spring" and "bend to fit": no
bed-slat task ("slat" finds only mini-blind and tambour tasks, #2527, #3383, #2225, #2613). `board_titles.txt` grep for
slat|bed frame|bow: no bed-frame or bed-slat task; nearest hits listed below.

| Task | Status | How ours differs |
|---|---|---|
| #716 fit a spring tension rod between two cupboard walls | merged | closest in "make a sprung part fit between two supports": a tension rod is shortened by axial telescoping compression and wedges by friction anywhere on flat walls. Ours bends a bowed beam to shorten its chord, with a crown orientation, an over-bend crack gate, retaining lips that set where each end must go, and two arms (one end seated and held home while the other arm bows the slat and drops the second end) |
| #1949 flex a 3D printer's build plate with both hands to pop a printed part free, without kinking it | merged | also bends a flexible plate with two hands under a kink limit, but to release a part from it; ours bends to shorten and seat a part in a fixture, with placement and release order |
| #1274 string a model recurve bow with its bow stringer, then let go | closed | bends a bow with a tool to slip a string loop into nocks; ours bends a slat by hand to drop its own end into a holder |
| #3167 put a shock-corded tent pole together: let the cord pull both joints home | closed (live `gh`, Nov 22) | joins pole sections that an elastic cord pulls home; nothing is bent to shorten it or seated in a holder |
| #3410 hold a polymer beam at 1 mm and 2 mm of bend on a lever test stand | open | holds a set deflection on a test stand; no fitting, no release into holders |
| #1994 refit a delta robot's sprung parallel-rod pair | merged | spreads a sprung rod pair over ball studs; no bowed beam |
| #2527 open a mini blind's slats by twirling its tilt wand | merged | "slats" of a blind turned by a wand; unrelated |
