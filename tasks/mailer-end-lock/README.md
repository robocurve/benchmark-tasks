# Close a roll-end mailer box: lock both ends by their tabs, then shut and tuck the lid; its walls start folded up

Shipping, receiving, and inventory clerks (O*NET 43-5071.00; 816,870 US jobs, BLS OEWS May 2025) have the Core task 'Pack, seal, label, or affix postage to prepare materials for shipping, using hand tools, power tools, or postage meter.' Hand packers and packagers (O*NET 53-7064.00; 559,820 US jobs, same survey) have the Core task 'Assemble, line, and pad cartons, crates, and containers, using hand tools.' Much of it goes in one-piece die-cut mailers that close without tape or glue (FEFCO 0427, roll end, tuck front). Packlane's assembly guide locks each roll end the same way: 'snap each side panel into the holes in the bottom of the box, continuing the same motion', at both ends, before the lid is closed and tucked. The box starts part-erected, as after step 3 of Packlane's five-step guide: its front and back walls are folded up with their four dust flaps folded in. The robot does every fold after that: it locks both roll ends, then shuts the lid and tucks it in. At each end one hand holds the sprung end wall upright while the other rolls the wall's inner panel over a double crease and down inside the box blind, so that its two tabs find their holes along the base crease; then the lid, standing up in line with the back wall, is pushed over and its tuck flap folded down and pushed inside the front wall. The box is free on the table and moves under every push.

**Instruction.** The box's front and back walls are already folded up: lock both roll ends with every tab in its hole, then shut the lid with its tuck inside the front wall.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py` and `gatekit.py` are in this folder.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 3200 \
  --instruction "The box's front and back walls are already folded up: lock both roll ends with every tab in its hole, then shut the lid with its tuck inside the front wall."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                                   # full score, with a video
python demokit.py task.py OUT --no-record --mode unheld,oneend,laid,lidonly,lidover,shut,undo,push,hover,lean   # the check runs below
python check_score.py                                                       # the scorer on hand-built states, no simulator
```

At each end the hand on that side comes down outside the leaning end wall, presses it upright onto its dust flaps 9 cm above the table and slides the box on the table until that end is within both hands' reach, pushing off the wall's middle toward the side that turns the box back square (feedback on the box's observed turn). It then holds the wall. The other hand passes over the unrolled inner panel's edge, comes down outside it with its fingers' broad side on the panel, and rolls the panel over the top, turning its fingers from side-on to straight down; down inside, it follows the panel, aiming 30 degrees ahead of it, and presses it flat against the wall with its fingers tilted back toward the wall, until both tabs snap into their holes. The other end is the same with the hands' roles swapped. Then the box is turned a quarter on the table, sliding: in passes of up to 28 degrees, one hand pushes the outside of the wall that faces the robot near one end while the other hooks its inside near the other end, both following the box's observed turn, and the pair also slide the box's middle toward 32 cm out (feedback on its observed position), until the lid's hinge is to the left. The box's pose is then read: if its middle is more than 1 cm off 1.5 cm right of the arms' midline, or it is more than 2 degrees off square, the right hand slides it back across the table, its fingers down inside the front wall drawing it (or outside it, pushing), off the wall's middle by the observed turn, until the middle is within 3 mm (feedback on its observed pose). Both hands lift over the walls and leave past the box's near end, 10 cm short of it, so that neither crosses the standing lid; they go back to their ready joint angles, and the box's turn is read again (one more set of passes if it is over 4 degrees off). The left hand comes to the lid at the box's observed pose and the lid's observed angle (a lid leaning out is first stood up by feedback), lays its closed fingers along the hinge line, broad side on the lid's outside, and swings the lid over to about 35 degrees open, where it holds it. The right hand puts a fingertip on the tuck's outside face 3 cm from its crease and curls the tuck in under the lid past square, to about 125 degrees (the tuck's fold angle, measured from the line of the lid; the tuck latches from 60 degrees), so that its edge points down and in; it keeps the curl while the left hand swings the lid on down, and lets go once the tuck's edge is inside the far wall at its top. The lid goes on down, the tuck sliding down inside the wall, and the right fingertip presses the lid's top 2 cm in from its front edge, deeper each step until the lid reads flat and the tuck holds it shut (feedback on the lid's angle). The demo reads the scorer's TRIAL state (the box's pose, joint angles and latch flags) to aim; it never moves an object except through the grippers.

## Size table

Every dimension a hand or the scorer touches. `size_check()` in task.py prints the scene's figures against the published ones.

| What | Scene | Published | Source |
|---|---|---|---|
| Outside length, end to end | 340 mm | 13 3/8 in, 339.7 mm | Uline S-16934 |
| Outside width, front to back | 314 mm | 12 3/8 in, 314.3 mm | Uline S-16934 |
| Outside height | 159 mm | 6 1/4 in, 158.8 mm | Uline S-16934 |
| Board | 3.2 mm | B flute, 1/8 in | WebstaurantStore |
| Rolled end, outer face to inner panel | 17.5 mm | (13 3/8 - 12) / 2 in, 17.5 mm | Uline outside less inside length, halved |
| Inner panel | 305 x 152 mm | - | estimate: the inside width less clearance, the wall's height less two boards |
| Tabs, two per inner panel | 25 mm wide, 5 mm proud, at 80 mm each side of the middle | - | estimate (FEFCO 0427 template) |
| Tab holes, four in the base | 6 x 30 mm, 16 mm inside each end's crease | - | estimate (FEFCO 0427 template) |
| Dust flaps, four | 140 x 140 mm | - | estimate (FEFCO 0427 template) |
| Lid, hinge to tuck crease | 307 mm | - | the inside width less two boards, estimate |
| Tuck flap | 280 x 45 mm | - | estimate (FEFCO 0427 template; mailer tuck flaps run 1 1/2 to 2 in deep): 12.5 mm clear of each rolled end |
| Tuck flap's free corners, rounded (its lead-in) | 15 mm radius | - | estimate, from photos of tuck-front mailers (die-cut outline, true size) |
| Tuck crease | folded 90 degrees, springs back with 0.35 N at the flap's edge, against 0.27 N of crease friction (0.012 N m) | - | estimate: a scored B-flute crease, folded by a light finger push, springs part way back |
| Box mass | 450 g | 0.99 lb, 449 g | Uline S-16934 |

`size_check()` output: outside length 340.0 mm (published 339.7), width 314.0 (314.3), height 159.0 (158.8), board 3.2 (3.2), rolled end 17.5 (17.5), lid to tuck crease 306.6 (307.3, estimate), tuck flap depth 45.0 (45.0, estimate), width 280.0 (279.8, estimate), corner radius 15.0 (15.0, estimate), tuck crease hold 0.35 N (0.35, estimate), mass 450 g (449).

## The scene and what is simplified

The whole box is modelled at Uline S-16934's outside size; nothing is shrunk. Board panels are rigid plates on sprung hinges; crease forces are simulation settings, not measurements (no published figure exists for B-flute creases). Each departure, plainly:
- The front and back walls start folded up, their four dust flaps folded in, and in the simulation they are part of the box's rigid base (real: a packer folds them up first, and they lean out until the rolled ends trap their dust flaps; in a real trial the tester folds them up and sets the flaps in by hand before each trial, as after Packlane's step 3, and nothing else holds them). The dust flaps are sprung to rest folded in, where a real one springs out. The task starts after those two folds; everything after them is the robot's.
- The double crease at each end is one hinge, the rolled end 17.5 mm thick (Uline: outside less inside).
- A tab's give is a 5 mm sprung slide; pressed 2 mm into its hole for 3 steps with its inner panel at 150 degrees or more it latches, and with both in, the end is held locked, as seated tabs and trapped flaps hold a real one. The latch is set by the tab's real position, reached through the robot's push. A locked end cannot be undone in the simulation (a real tab is unhooked by a deliberate pull).
- The tuck's friction in the front wall is a latch: once the tuck, folded down 60 degrees or more, bears on the wall's inside (its face on it: a gap of 0.6 mm at most) at least 2 cm below the top, with the lid shut and both ends locked, the lid is held shut against up to 2 N lifting its front edge (an estimate); pulled harder, the tuck comes out and the lid springs open. The tuck crease's friction (0.012 N m) is less than its spring (0.01 N m/rad) past about 70 degrees, so a tuck curled past square and let go springs back onto the wall.
- The panels, lid and tuck carry no weight of their own in the simulation (the box's 450 g is on its base); their springs and the hands move them.
- The lid's two side flaps, which tuck inside the ends as a real lid shuts, are not modelled (real: about 4 cm deep each).
- The simulator's non-twisting panel omits cardboard skew: a real inner panel can skew and seat one tab before the other.
- Damage (a torn tab, a new fold) cannot happen in the rigid-panel model, so only a real trial can score it.
- The end walls rest at 80 degrees above the table where their creases spring them (a setting; a real S-16934 is checked with a protractor before each trial).
- The demo reads the scorer's TRIAL state (the box's pose, joint angles and latch flags) to aim its pushes; it never moves an object except through the grippers.

## Scoring

Defined terms. An end is **rolled** when its inner panel is folded past 100 degrees with its free edge inside the box and below the wall's top; **down** when the panel is within 10 degrees of flat against the wall and its edge within 1 cm of the base; **locked** when both its tabs are in their holes (tip at least 2 mm down into the base board, inside the hole's outline), the inner panel within 2 degrees of flat and the wall within 5 degrees of upright with both dust flaps within 10 degrees of folded in. The lid is **shut** within 5 degrees of flat on the box, and **tucked** when the tuck is folded down 60 degrees or more with its face on the front wall's inside (a gap of 0.6 mm at most), at least 2 cm below the wall's top.

Tiers. Each end is worth 0.4: 0.1 rolled, 0.2 down, 0.3 one tab in, 0.4 locked. The lid is worth 0.2, counted only with both ends locked (the real order): 0.1 shut, 0.2 shut and tucked. Everything done gives 0.95; 1.0 needs, in addition, both grippers' fingers at least 3 cm from the closed box's outside and no point of the box moving more than 1 mm over that same last 1 s. The score is read from the final state: the lowest step score over the last 1 s, so a step undone before the end loses its tier.

Zeros that stick. Score 0, and it stays 0, if the box at any time leaves the bench (its middle past the bench top's edge or below it) or the board is damaged: a tab torn or bent past 45 degrees, a new fold or tear away from the creases, a dent deeper than 2 mm, or a dust flap swung out more than 30 degrees. Sliding, turning or tipping the box costs nothing by itself; the tiers are read in the box's own frame. In the simulation only the bench-leave and flap-out zeros are measured; tab, fold and dent damage cannot occur in the rigid-panel model and is scored in a real trial.

Possible scores: 0, 0.1 to 0.8 in steps of 0.1, 0.9, 0.95, 1.0.

Real trial, rule by rule: photos from the front and above, a ruler, a square and a protractor (about 10 USD); an end's wall top is flat, 'not slanted or peaked' (Packlane's check), for the inner panel within 2 degrees; each tab is seen in its hole from underneath after the photos; the lid is shut when its front edge is within 2.5 cm of the front wall's top (5 degrees at 30 cm), and tucked when the tuck cannot be seen from above and the lid stays down when let go.

## Results

Scripted demo on GPU physics, final code. The README command (`--mode full`, filmed at 640 x 360) on a lab PC (RTX 4070 Ti SUPER); the check runs at 320 x 176 (`--no-record`):

| Run | What it does | Score |
|---|---|---|
| `full` | Both ends locked (every tab in its hole), the box turned a quarter, the lid shut with its tuck's face on the front wall's inside, hands clear | 1.0 (1,559 steps, 0 blank frames) |
| `full`, start shifted | The box starts 1 cm nearer, 1 cm further, 1 cm left or 1 cm right | 1.0 in each: 1 cm nearer 1,556 steps, 1 cm further 1,558, 1 cm left 2,110, 1 cm right 1,564 |
| `hover` | Both ends locked, the lid pressed on a tuck folded by a hand and not seated (its edge not on the wall), hands let go | 0.8 (the lid springs open: a hovering tuck does not latch) |
| `lean` | End 1's wall held leaning 7 degrees while its panel is pressed down, stood upright once the tabs are in, then the rest | 1.0 (2,588 steps; the end locks once the wall reads upright) |
| `unheld` | End 1's wall stood upright, held 1 s, let go: it springs back | 0 |
| `oneend` | End 1 only, locked | 0.4 |
| `laid` | End 1's panel pressed to about 150 degrees, short of flat, with its tabs on the base's edge, then let go: it springs back open | 0 |
| `lidonly` | The lid pushed at with both ends open: it is not carried shut | 0 |
| `lidover` | End 1 locked, end 2 left open, the box turned and the lid closed over the open end | 0.4 (with end 2 open the turn goes astray and the lid is not reached; the lid counts for nothing over an unlocked end in any case) |
| `shut` | Both ends locked, the lid pressed shut with its tuck unfolded, the hand still holding it at the end | 0.9 |
| `undo` | Both ends locked, the lid swung shut with its tuck unfolded, held 1.5 s and let go: it springs open | 0.8 |
| `push` | Nothing folded: the box only pushed, slid and turned a quarter on the table by both hands | 0 |
| `check_score.py` | Hand-built states: each rung, the lazy routes (a tuck curled clear of the wall or hovering 6 mm off it, a lid held shut, a lid over an unlocked end, the box pushed), undo before the end, stillness, latched failures | all as the text says |

## Runtime and seeds

The demo takes about 1,560 steps (78 s at 20 Hz); the run limit is 3,200 steps (2 minutes 40 seconds), about twice the demo. There is no randomness: `object_jitter=0`. The shifted starts above set the box's start 1 cm off (`MAILER_MX`, `MAILER_Y0` in task.py); the demo's feedback reads where it is.

## Building it for a real trial

Use Uline S-16934 as bought, erected as in the setup: base on a laminate or wood bench, front and back walls folded up by hand with their dust flaps in (as after Packlane's step 3; nothing holds them but the folds until the rolled ends trap the flaps), both end walls springing back to 80 +/- 5 degrees (protractor), inner panels unrolled, the lid standing up in line with the back wall. Judge as in Scoring. Between trials unhook the tabs, roll the panels back out, open the lid and fold it back up; replace the box after 10 trials, or once an end wall no longer rests at 80 +/- 5 degrees.

The forces the scene asks for are small: holding an end wall takes 3 N at its top, the other settings are under 1 N (rolled panel 0.2 N, tab 0.45 N, lid 0.5 N), and sliding the free box takes about 1.8 N (0.45 kg x 9.81 x 0.4); nothing is carried. The bench needs no guarding: the only object is a 0.45 kg board box at table height, with nothing energised, hot or breakable.

## Sources

O*NET 43-5071.00, Core task 20277 (local O*NET 30 database; USDOL/ETA, CC BY 4.0): https://www.onetonline.org/link/details/43-5071.00. BLS OEWS May 2025 national estimates, 43-5071 816,870 (national_M2025_dl.xlsx): https://www.bls.gov/oes/. O*NET 53-7064.00, Core task 'Assemble, line, and pad cartons, crates, and containers, using hand tools.' (read 11-19): https://www.onetonline.org/link/details/53-7064.00; BLS OEWS May 2025, 53-7064 559,820 (same file). Box size, board and mass: Uline S-16934, 'Outside: 13 3/8 x 12 3/8 x 6 1/4" (+/- 1/8")', 'MATERIAL: B flute', 'Unit Weight: 0.99 lbs.', price 4.34 USD per mailer at 25, bundle of 25 (read 11-16, 11-19): https://www.uline.com/Product/Detail/S-16934/Indestructo-and-Literature-Mailers/12-x-12-x-6-Kraft-Tab-Locking-Literature-Mailers. Style: FEFCO 0427 in the FEFCO/ESBO international fibreboard case code; the die-line is a template built to it at that outside size (e.g. Pacdora's FEFCO 0427 generator: https://www.pacdora.com/tools/fefco-0427). Folding steps: Packlane, 'How do you assemble a Mailer Box?', five steps; step 3 'Fold up the front and back flaps', step 4's check 'the tops of the side walls are flat, not slanted or peaked' (read 11-16, 11-19): https://packlane.com/support/how-do-you-assemble-a-mailer-box. Board thickness: WebstaurantStore, 'Types of Corrugated Boxes', B flute 'Flute Height: 1/8"' (read 11-16): https://www.webstaurantstore.com/blog/1138/types-and-sizes-of-corrugated-boxes.html.
Friction: Park, Choi, Kim, Jung, J. Korea Soc. Packaging Sci. Tech. 25:131-137 (2019), corrugated board on board, mean static 0.38. O*NET is a trademark of USDOL/ETA. Uline and other maker names are for identification only.

## Licence

MIT
