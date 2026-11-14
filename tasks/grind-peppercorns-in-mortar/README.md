# Grind half a teaspoon of peppercorns in a granite mortar to a medium-fine grind

Files for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). A 12 cm granite mortar holds
27 whole black peppercorns (1.2 g, about half a teaspoon), its granite pestle leaning in the bowl. The robot grinds the
corns by pressing the free 330 g pestle down on them and working it round the loose bowl, then puts the pestle
down and lets go. There is no crank or fixed path. In the simulator every corn is a rigid body, and a corn the pestle
has worked hard enough breaks into real pieces. The setup, scoring and fidelity notes are in `task.json`.

Why it matters: home cooks crush whole spices in a mortar just before use for fresher flavor, a step common to cuisines
worldwide and taught in cookbooks and cooking schools. Half a teaspoon is a typical amount for one recipe; at about
2.3 g per teaspoon of ground black pepper (USDA FoodData Central, "Spices, pepper, black"), it is about 1.2 g. (No
O*NET task line for cooks names grinding spices, so the task is filed under societal relatability.)

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/grind-peppercorns-in-mortar/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Grind the peppercorns." --scorer task_score --max-steps 2400 --store-frames

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.

Render the videos of a run with `inspect-robots video <log.json>`. Offline checks (no simulator): `python3 test_task.py`.

The demo: `demo()` is open-loop from the fixed layout. The pestle settles at once into its rest against the rim
(58 degrees from upright). The right hand takes it by its handle 10 cm up from the head's end, fingers closing along x
across the handle, with the wrist rolled 43 degrees so the hand comes in from the right arm's side, nearly in line
with the handle. It lifts the pestle 10 cm, turns the wrist back to straight down with the head above the rim, and
stands the pestle on the clear middle of the bowl's bottom; let go, it settles upright on its flat 4 cm head. The hand
takes it again 7.5 cm up from the head's end (the lowest grip that keeps the 14 cm pestle's top clear of the gripper's
palm), straight down, so the push goes down the pestle rather than skating its head sideways. It aims the head 2 cm
below the bowl's bottom (the arm gives about 2.4 mm per newton downward, so it presses about 11 N), then works the
hand round 33 circles, 2 s each, with the hand's orientation held. A sideways pull on the head bends the arm 8 to
12 mm per newton, mostly in the wrist servos (from the arm's joint stiffnesses), so against the corns' friction the
hand leads the head by 2.5 to 4 cm: it circles 5 cm out, and the wall of the 6.2 cm bottom keeps the head within about
1.2 cm of the middle, so the head is worked round the bowl against its side, over the ring of corns, trapping them
and then their cracked pieces under its edge, then rubbing their powder against the bottom. It eases off and, still
holding the pestle, lays it over toward the rim on the right: the wrist rolls 28 degrees while the head pivots on its
edge, past the 19 degrees where the pestle would tip by itself (its center of mass is about 6 cm up its 4 cm head).
Let go there, it can only fall the rest of the way, onto the rim, and rests at about 34 degrees; it is never left
standing on its flat head, where it would rock on what is under it. The open fingers' gap runs along the pestle's
lean, so the hand slides 8 cm square to the fallen handle, out of that gap, and lifts 8 cm clear.
The left hand stays where it starts. A cook often steadies the mortar with the other hand; here the 1,280 g mortar stands loose and does not move
under the arm's push, and the score does not care how it is held.

## Scoring

`--scorer task_score` gives 1.0 when the task succeeds (the full condition held for 1 s) and otherwise `score()` at
the end of the trial. The score is the grind itself, read with two stacked sieves: the share by mass of the starting
27 whole corns (1.2 g), still in the mortar, that passes 1.4 mm (US No. 14) and the share that passes 0.6 mm (US
No. 30). How the pepper was ground (pressing and circling, pounding, how long, how many strokes) never changes the
score. The real trial sieves and weighs; the simulation counts the same two shares from its bodies (below). The title's "medium-fine" is the 0.75 and 1.0 result: 80% through 1.4 mm with 20% through 0.6 mm.

The two sieves split the batch into the grind sizes pepper is sold in. A pepper-mill maker's guide (Cole & Mason,
"Choosing the Right Size Grind for Black Pepper", 2021) gives cracked peppercorns as 8-10 mesh, coarse as 12 mesh,
table ground as 18-28 mesh and fine ground as 30-36 mesh. US No. 14 (1.40 mm) and No. 30 (0.600 mm) are standard
sieve openings (ASTM E11): held on No. 14 is whole, cracked or coarse; through No. 14 and held on No. 30 is medium,
the table-grind range; through No. 30 is fine.

- **0.25: 40% or more through 1.4 mm.** The corns are being broken down, but much of the batch is still whole or
  cracked.
- **0.5: 80% or more through 1.4 mm.** Nearly all of it is table grind or finer; it leaves room for a few cracked
  bits.
- **0.75: 80% or more through 1.4 mm and 20% or more through 0.6 mm.** A well-ground batch, the job's main result:
  mostly table grind with a real share of fine pepper, as a mortar gives when the powder is worked against the bowl.
- **1.0: the same, and the pestle put down.** The pestle rests in the bowl (its head's end inside the bowl,
  below the rim) or lies on the worktop (its head's end within 4 cm of it, its lowest point from 3 mm below to 5 mm
  above it); no gripper touches it (the contact check in `guardkit.py`: the gripper's own shape within 2 mm of the
  pestle); the mortar stands upright within 10 degrees; and pestle and mortar move under 1 cm/s and 0.5 degree per
  step, all for 1 s. A 330 g granite pestle left in hand can still fall onto the mortar or the counter and chip it, so
  the job ends with it put down. The arm's pose is not scored.
- **Spill cap: more than 0.12 g out, at most 0.25.** Pepper thrown out of the mortar is lost from the batch and
  scattered on the counter. More than 0.12 g (about 10% of the batch, three corns) is too much to call a clean job, so
  the score is capped at 0.25 for the rest of the trial (spilled pepper does not come back).
- **Knocked over.** A mortar tipped past 60 degrees pours its pepper out onto the counter. There is no separate
  rule: it counts as spilled, so nothing passes (and the spill cap applies). A mortar tilted less and stood back up
  keeps its pepper and scores normally.
- Doing nothing scores 0 (all the corns are whole). Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

Each threshold is defined once in `task.py` (`PASS_14`, `PASS_30`, `PASS_14_LOW`, `SPILL_CAP_G`, `UPRIGHT_DEG`, `STILL_STEP`, `STILL_TURN`,
`HOLD_STEPS`, the contact check in `set_down()`) or in grindkit's `Law`, set in `task.py` as `LAW`.

Scoring a real trial (only the sieves, the scale and the video; no rule about how it was ground):

- Grind: stack the No. 14 sieve on the No. 30 sieve over a pan (ordinary kitchen or soil-test sieves), tip the
  mortar's contents in, brushing the bowl out, and shake for 30 s. Weigh on a 0.01 g pocket scale what sits on the
  No. 30 sieve and what reached the pan. With the batch weighed before the trial (about 1.2 g): No. 30 and pan together
  40% of it is 0.25, 80% (about 0.93 g) is 0.5, and 80% with the pan alone holding 20% (about 0.23 g) is 0.75.
- Spill: brush up any pepper on the worktop outside the mortar and weigh it (more than 0.12 g caps at 0.25).
- Knocked over: on the video, the mortar tipped past 60 degrees (on its side or held over), or simply weigh the spill:
  at that angle the pepper pours out, so the weighed spill also shows it.
- Set-down: on the video, the pestle ends in the bowl or on the worktop, both hands away from it, nothing moving.

## The pepper in the simulation

The simulation measures the same thing as the sieves: the mass broken down below 1.4 mm, and below 0.6 mm. It is
counted from the bodies (`grindkit.py` keeps the books, `cornkit.py` spawns, reads and moves the bodies).

- **Real corns.** Each of the 27 corns is a rigid sphere, 4.6 mm and 43 mg (dry Malabar-grade black pepper: geometric
  mean diameter 4.63 mm, 1000-corn mass 42.8 g; Balasubramanian et al. 2013, Journal of Spices and Aromatic Crops
  22(2): 131-137). They start in one ring round the edge of the bowl's bottom, and the pestle pushes, traps and scatters them.
- **Breaking by energy.** A dry corn squeezed between plates ruptures at 74.6 N after 0.575 mm, an energy of 43 mJ
  (the same study; damper corns are softer, 50.1 N and 32.8 mJ at 18% moisture). Here a corn breaks once the work done
  on it reaches 43 mJ, in one blow or over many rubs. It then becomes two cracked pieces: the corn's body is parked out
  of sight and two pool bodies (spheres of 3.7 mm, 21.5 mg, the same volume between them) are moved in where it was.
  The No. 14 sieve holds them. A cracked piece breaks at the same energy per gram, 21.5 mJ, into medium pepper
  (through 1.4 mm, held on 0.6 mm): its body is parked and its mass is counted. Pepper under 1.4 mm is the only part
  of the batch that is not bodies (it is far too fine to simulate); it is drawn as thin layers on the bowl's bottom,
  brown for medium and paler for fine, each spreading as it grows.
- **Ground fine against the bottom.** Medium pepper lies spread on the bottom. Where the pressed head (7.5 N or more)
  slides with its end on the bottom (within 1 mm of it, no piece caught under it), the friction work 0.3 x force x
  slide goes into the share of the powder under the head (the head's 4 cm end over the 6.2 cm bottom: 42%), at
  1.59 J per gram, a first-order process. Only the rubbing done on powder counts: a thin scatter leaves the head
  rubbing bare stone, so the work is scaled by the powder's cover of the bottom, its mass over 2.0 g (one layer of
  0.92 mm particles over the bottom, 60% packed, at 1.2 g/cm^3). That energy per gram is not measured for pepper: it is the pieces' 1 J per
  gram scaled by Rittinger's law of comminution (energy in proportion to the new surface made, 1/d after minus 1/d
  before), between the stages' typical sizes: 3.65 mm pieces, 0.92 mm medium (between the 1.4 and 0.6 mm openings),
  0.42 mm fine (between 0.6 and 0.3 mm). A head rubbing nothing, hovering, resting by its own weight or pressing
  without moving grinds nothing fine.
- **Force, from a sprung bottom.** The bowl's bottom under the corns is a disk on a slide with a spring, 5 N per mm
  over 3 mm of travel (15 N at its stop), part of the mortar (a `Spring` link from `mechkit.py`), so how far it is
  pushed down is the force through the corns.
- **Which corns take the work.** Work goes only to pieces caught under the pestle's head: inside its 4 cm end (seen
  along the pestle) and touching it (within 0.7 mm of the end's plane, or pressed into it), shared equally among them.
  Pieces the head does not touch take nothing, and so does a head that touches none; nothing counts while a gripper
  touches the bowl's bottom.
- **Rubbing.** While the bottom is pushed down at least 1.5 mm (7.5 N; the pestle's own 3.2 N weight pushes it
  0.65 mm, so a pestle laid in the bowl or swirled lightly grinds nothing) and the head slides across the pieces (the
  middle of its end, in the mortar's own frame, so a mortar turning along with the pestle grinds nothing, and neither
  does rocking the pestle in place), the work is the friction 0.3 x force x slide (counted up to 1 cm per step).
- **Pounding.** A press that starts with the head coming down at speed v delivers the pestle's kinetic energy,
  half of 330 g x v^2 (a hand driving it adds more, which is not counted): brought down 3.6 cm in a step it carries
  86 mJ, a tap from 3 mm under 1 mJ.
- **Thrown out.** A piece whose body leaves the bowl (beyond the rim's 4 cm radius) is spilled, for good. A mortar
  tipped past 60 degrees empties completely (its pieces fall out as bodies; the counted powder goes with them). 60
  degrees comes from the bowl's shape: tilted 55 degrees it still holds about 23 cm^3 below its lowest rim point, and
  near 60 degrees the mortar itself falls on its side.

What it does not model: the stresses inside a corn (breakage is by accumulated work, so the 10 to 15 N the head
carries, shared among the corns under it, is far below the 50 to 75 N that cracks a corn in a single squeeze; the model
treats grinding as wear by repeated rubbing, as a real grind is); the shapes of real pieces (spheres); the powder as
particles (and the fine stage's energy, scaled from the measured one); force above 15 N (the bottom's stop), while a cook presses harder. Frictions: granite on the worktop 0.5 (static) and 0.4 (dynamic); corns on granite 0.35 and 0.3. The corns are spheres whose rolling is
damped (wrinkled corns do not roll on like balls), and the bowl's curved wall is a ring of colliders in three steps.
The pestle's head has torsional friction where it presses (a 1 cm contact patch, as a flat end pressed into grit
has), so a pestle let go on the corns does not keep turning about its axis.

## Runtime and seeds

- Demo: a 90 s plan (1,808 control steps); the run succeeds (1.0) at step 1,762; run with `--max-steps 2400`.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seeds: none. The layout is fixed (`object_jitter=0`) and nothing in the task is random.

## Real-world build

- A 12 cm unpolished granite mortar and pestle set (mortar about 1,280 g, 12 cm across, 8.5 cm tall, bowl 8 cm across
  and 5 cm deep; pestle about 330 g, 14 cm long, 4 cm head, handle 2.8 cm across). Any standard 12 cm granite set
  within about 10% of these figures will do; measure and note the actual ones.
- Whole dried black peppercorns (Malabar grade, 4 to 5 mm), 27 corns (about 1.2 g) per trial, shaken on the 1.4 mm
  sieve first so nothing passes at the start, weighed on a 0.01 g pocket scale.
- Two stacking sieves, 1.4 mm (US No. 14) and 0.6 mm (US No. 30), 8 in. (20 cm) frames, with a pan (kitchen or
  soil-test sieves); a soft brush for the bowl and the worktop.
- Set the mortar loose on the worktop (no mat, no clamp) with its foot centered at x 30.0, y -8.0, put the corns in a
  ring round the edge of the bowl's bottom (the middle clear), and lean the pestle in the bowl with its handle on the
  right (-y) rim.
- Safety: the only hazard is a dropped 330 g pestle; keep hands out of the work area while the arms run.

## Files

- `task.py`: the scene, `score()`, `success()` and the scripted `demo()`.
- `grindkit.py`: the task's helper module that keeps the pepper's books (which pieces the head works, when they break,
  what passes the sieve, what is spilled).
- `cornkit.py`: the task's helper module that spawns the corns and the hidden pool of cracked pieces as rigid bodies,
  reads them each step and moves pieces in and out of the pool.
- `yamkit.py`, `mechkit.py`: the task's helper modules that spawn the modeled props and the sprung bottom under the corns.
- `guardkit.py`: the task's helper module for contact between the grippers' own shapes and the parts.
- `test_task.py`: offline checks of the scorer and the demo plan.
