# Seat a robot gearbox's cartridge stack on its motor: find the hidden spline by feel, then square the housing

A brushless motor stands shaft up in a stand. The input stage of a REV MAXPlanetary planetary gearbox is screwed to
its face and the gearbox's input coupler is on the motor shaft, inside the input stage. Beside it, on a four-post
cradle, stands the rest of the gearbox, already stacked as the maker's instructions have it: a 4:1 cartridge, a 3:1
cartridge and the 1/2 in hex output stage (12:1 overall). The robot must put the stack on the motor: thread it down
the motor shaft, engage the rear cartridge's input spline in the coupler, turn the housing until its alignment tabs
drop into the input stage's notches, and let go with no gap left. The job stops where the maker's next step begins:
the two gearbox screws go in after the stack is seated.

## Why it matters

Robot joints and drivetrains are built from motors and planetary gearboxes, and kits such as the MAXPlanetary are
assembled by hand: cartridges are stacked to the ratio a joint needs and the stack is fitted to its motor. REV's own
assembly tips describe the step this task tests:

- "Slide the stack of gear cartridges onto the motor shaft and input stage."
- "This may require twisting the gear cartridges by hand slightly to line up the splines", "rotating back once the
  spline is engaged".
- "Make sure the stack is fully seated and no gap is present between the stack and the input plate" before the
  assembly screws go in; the NEO guide marks the gap: "Having a gap in this location is bad", because it means "all
  alignment features are not properly engaged".

Sources: [assembly tips](https://docs.revrobotics.com/ion-build/motion/maxplanetary-system/assembly-tips-and-tricks),
[NEO assembly guide](https://docs.revrobotics.com/ion-build/build-guides/maxplanetary-gearbox-assembly/neo-and-other-500-motors),
[system features](https://docs.revrobotics.com/ion-build/motion/maxplanetary-system/system-features) (the alignment
tabs and notches "keep the rotating components concentric as well as providing angular alignment").

## What the robot has to do, and why each part is hard

1. **Thread a 0.40 kg stack down a 8 mm shaft.** The rear cartridge's sun has an 8.3 mm bore (maker's drawing), so
   the stack goes onto the motor's 35 mm shaft with 0.15 mm of radial play. The shaft stands 15.9 mm above the input
   stage. Lowered more than about 1.2 mm off the shaft's axis, the stack lands on the shaft's end.
2. **Find the spline's phase by feel.** About 17 mm further down, the cartridge's 39-tooth input spline (20.5 mm
   across) meets the coupler's internal spline. The teeth repeat every 9.2 degrees and only about 2.6 degrees of that
   lets them in (1.3 degrees either side of meshing, on their 0.10 mm tip chamfers); otherwise the tooth flats land on
   each other and the stack stops 3.0 mm proud. Nothing about the phase can be seen: the spline is under the stack, inside the input stage. The sun turns
   freely in the housing (it drives the gears inside), so twisting the housing turns the sun only through the gear
   train's drag. A light twist with light pressure slides the teeth over each other until they drop in; pressing hard
   makes the tooth flats grip each other and the sun stays put.
3. **Turn the housing back.** The spline engages wherever the twist found it, and a twist that cannot see when it
   happened carries on past it: in the demo the housing ends up 5 to 7 degrees from square with the spline in. The
   housing's two alignment tabs (20 mm wide, 1.1 mm deep) then stand on the input stage's face, 1.1 mm proud: they
   only start into the notches within about 1.3 degrees of square (the notches' 0.25 mm mouth chamfers) and seat
   within 0.7 degrees. The sun is now held by the coupler,
   so turning the housing back turns the gears inside, not the spline: a second, separate turn, the "rotating back"
   in the maker's tip. Square, the tabs drop in and the stack seats with no gap, its ears over the input stage's ears
   (where the screws go). Turned 90 degrees, the tabs stand on the face; 180 degrees is also correct (the tabs and
   ears repeat every half turn).
4. **Let go without unseating it** and leave it still.

Each layout (lot) starts the stack at a different place and turn on the bench. In both, the spline starts a quarter
tooth (2.3 degrees) off meshing when the housing is square, one way in lot A and the other way in lot B. Because the
spline has an odd number of teeth (39), half a turn moves its phase by half a tooth, so the same quarter-tooth offset
holds at both square turns (0 and 180 degrees): a plain press, square either way, stops on the tooth flats. A straight
press in each lot was run on the GPU to check it (Results).

## Nearest tasks, and how this differs

- **Meshing a gear train by finding each gear's tooth phase by contact** (an accepted task): each gear goes onto its
  own fixed post and meshes sideways with a neighbour whose teeth are in plain view. Here the teeth are hidden inside
  the part, the toothed sun is free inside the part's housing, and the part's outside has its own angular register;
  the phase is found by a light twist under load, then the housing is turned back separately. The part is also a
  0.40 kg stack threaded on a shaft with 0.15 mm play, not a gear dropped onto a post.
- **Pressing a servo horn onto a splined output shaft at a zero mark**: there the horn's own angle sets both the
  spline phase and the visible mark, so one turn before pressing does it. Here the spline's phase and the housing's
  register are independent (the gear train between them), so no single turn can satisfy both.
- **Refitting a slip yoke onto a splined shaft in phase**: there the phase is a choice read from a picture and the
  rest is sliding a sleeve on. Here no picture shows the phase (the spline is under the stack, inside the input
  stage); it is found only by feel, under a light load, and the stack's own outside must then be squared separately.
- **Closing a jaw coupling on its spider** (seating a spider in one hub, turning the loose hub's shaft until its jaws
  face the spider's gaps, sliding it along its key): there the phase is coarse (three jaws, 8 mm legs with 0.6 mm play
  and lead-ins), visible from above, set before the slide by turning the hub, and the hub can only slide along its
  key. Here the phase is fine (39 teeth, 9.2 degrees apart, flats that grip under load), hidden, found during the
  insertion by twisting under light pressure, and then undone and redone at the housing by a second turn while the
  spline holds; the part is free in six directions until the shaft takes it. The core skill is search-by-feel spline
  mating plus a decoupled second registration, not turning a visible phase and sliding.

Not every way of seating it needs both turns: pressed at a turn that happens to be 1.0 to 1.3 degrees off square in
the right direction, the spline can catch on its chamfers and the tabs on theirs. That window is about 0.3 degrees
wide on each side; a press that does not know the hidden phase finds it by chance only.

## Questions a reviewer may ask

- **Are the spline phase and the outer register really independent?** Yes, in the real part and in the simulation:
  the input spline is on the cartridge's sun gear, which turns relative to the housing through the planet gears
  (REV: "Input Spline: accepts torque from the previous stage"). With the spline engaged, turning the housing turns
  the gears inside. The GPU traces show it: after the spline engaged, the housing was turned back 5 to 7 degrees to
  square with the spline staying in (Results).
- **Does the fit need force or deformation?** The maker says the coupling "may require a decent amount of force" and
  "is not expected to be a slip-fit". Nothing here is scored on force, and the real press force is not measured: it
  is an open estimate. The simulation's teeth are rigid with 0.10 to 0.15 mm of side play, and the demo presses with a
  few newtons. If the real coupling is tighter, the twist-then-press sequence is the same and the final press is
  harder; a real trial would show whether the arm can make it.
- **Is the endpoint a released one?** Yes. Every credited state is scored after both hands are more than 5 mm away and
  the stack has stayed still for 1 s. A seated stack stays put: it is on the shaft, the spline and the tabs.
- **Can a stack resting on its spline tips look seated?** No. It stands 3.0 mm proud at every corner (feeler gauge).
  A spline that is in but not squared leaves 1.1 mm (tabs on the face). Seated is 0.3 mm or less all round, with both
  ear holes within 0.5 mm of the input stage's, so the kit's screw goes up through both and starts by hand; a tab on
  the face cannot give that.
- **Why stop before the screws?** The maker makes the seat its own check before the screws (make sure "the stack is
  fully seated and no gap is present before inserting and tightening the gearbox assembly screws"), and a stack that is
  not seated cannot be put right by the screws: they would clamp it on its tabs or tooth flats. The two 1-1/2 in
  #10-32 socket head screws (4.8 mm thread, 5/32 in hex key) go up through the input stage's ears into the output
  stage: a separate fastening step, with parts too small for these grippers. What this task leaves is checked the way
  the screws need it: ears over ears.
- **Fixed configuration?** One: a NEO V1.1 motor, the Universal Input Stage, the 8 mm keyed input coupler, a 4:1 and a
  3:1 cartridge and the 1/2 in hex output stage, with every size, mass and start pose below.

## Run a trial

From this folder, in the Isaac Lab-Arena environment of the program's simulation walkthrough (yam-isaac-lab-arena,
the program's YAM embodiment for Isaac Lab-Arena). Trial k uses lot k mod 2, so `--epochs 2` runs both lots and
`task_score` is their mean:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the gearbox stack onto the motor and its input stage, fully seated with no gap, then let go." \
  --scorer task_score --max-steps 800 --epochs 2 --store-frames --log-dir ~/yam-eval/gearbox-dream
```

- The scripted demo: the same command with `--policy yam_scripted -E cam_width=640 -E cam_height=360`.
- `TASK_LOT=A` or `TASK_LOT=B` in front pins every trial to one lot (debugging only).
- Runs use `object_jitter=0`: the stack's place matters to the millimetre on the shaft, so the layout is fixed per lot.
- Close cameras for the videos: `TASK_VIEWS_DIR=<dir>` in front records a side view level with the seat (1280 x 720,
  about 7 pixels per mm at the seat in its 2x crop) and a view from beyond the motor; they are not in the policy's
  observation. `bash make_video.sh <run dir> <dir> demo "<t0>-<t1>"` then builds the video and `setup.png` from the
  recorded frames (a real-speed grid, the window t0 to t1 at half speed, the end held).
- Runtime: a scripted trial is done at 608 steps (30.4 s of simulated time); a Dream-1 trial runs its 800 steps. On one
  NVIDIA L40S a two-trial scripted run took about 10 minutes of wall time (the 39-tooth splines make many contacts).

## Scoring

Every trial is scored by `score()` in `task.py`; `task.json` has the short text. Lengths are in mm.

**Whole trial (checked every control step):** 0 for the trial if the stack is ever let go of (both hands more than
5 mm away) while falling faster than 0.5 m/s (a fall of about 13 mm) or tipped past 45 degrees. Until the screws go
in, the real stages are held together only by their press fit, so a real stack dropped from its cradle or the shaft's
end may part or be knocked out of square; the simulation holds them as one body and cannot show that, and the rule
stands in for it. It never applies while the stack is held. 0 for the trial if the hands or the stack cannot be read
at any step (the scorer fails closed).

**Let go and still:** every credited state needs every sampled point of both grippers and all four fingertips (their
collision hulls' faces sampled every 3 mm, not only the vertices) more than 5 mm from the stack, and three points on the
stack (two side centres and the top centre) moving no more than 1 mm, with the level unchanged, for 1 s.

| Score | Simulation (`task.py`) | Real trial |
|---|---|---|
| 1.0 seated | all four corners (23 mm out from the axis each way) within 0.3 mm of the input stage's top face (and none more than 0.5 mm below it); both ear holes within 0.5 mm of the input stage's ear holes, across the bench (square or half a turn round); and the 0.5 conditions | a 0.30 mm feeler leaf does not enter at any corner; a 1-1/2 in #10-32 screw, pushed up through each of the input stage's ear holes, passes into the stack's ear and starts in the output stage's thread by hand (then is taken out); video for let-go and stillness |
| 0.5 spline engaged | the bottom face's centre (the mean of the four corner gaps) at most 2.0 mm above the input stage; axis within 1.0 mm of the motor's; upright within 3 degrees | on the shaft; the four corner gaps (feeler leaves or ruler) average at most 2.0 mm |
| 0.25 on the shaft | axis within 1.5 mm; upright within 5 degrees; bottom face less than 19.0 mm up (the shaft at least 1 mm into the bore) | the shaft is inside the stack's bore (the stack stands on the motor, not beside it), upright by eye against a square |
| 0 | anything else | left in the cradle, set down elsewhere, balanced on the shaft's end |

What each way of doing it scores (the cases are in `near_misses.py`):

- Left in the cradle, or set down on the bench: 0. Dropped or tipped over once let go, at any moment: 0.
- Lowered off the shaft's axis so it lands on the shaft's end: 0 (it balances 20.1 mm up, or falls).
- Pressed straight down onto the motor without twisting, square either way: 0.25 (it stops on the tooth flats,
  3.0 mm proud).
- Twisted until the spline drops in, then let go without turning back: 0.5 (tabs on the face, 1.1 mm proud).
- Squared 90 degrees off: 0.5 (tabs on the face). Square, or half a turn round: 1.0.
- Seated but still held, or fingers within 5 mm: 0 until the hands leave and 1 s passes.

Trials run on lot A, then lot B; `task_score` is their mean.

## The scripted demo

`demo()` plays one arm, open loop, from what the cameras show: the cradle's place and turn (lot A on the robot's
right uses the right arm, lot B on the left the left arm) and the motor's fixed place. It never reads the sun's or the
coupler's angle, an object's simulated pose, or the lot's name for anything a camera could not see. One correction is
calibrated, not observed: each arm holds the stack 1.7 to 2.4 mm short in x and 2.8 mm out in y of where it is sent
(measured in a trace, `COMP` in `task.py`); a person teaching the arm would make the same correction. The loaded arm
also sags about 11 mm at the motor, so its heights over the shaft are reached about 11 mm low; the demo was tuned with
that sag in place.

1. Grip the output stage across two sides, square to the stack's turn in the cradle; lift it clear of the shaft.
2. Carry it over the motor, turning it square on the way (the input stage's ears show which way is square).
3. Lower it straight onto the shaft's end and down the shaft (the bore's mouth chamfer and the shaft's end take up
   about 1.2 mm of misalignment), then 16 mm along it until the spline rests on the coupler; the target is 1.5 mm
   below that contact, so the arm presses only lightly (about the stack's weight plus half a newton).
4. Twist 7 degrees one way, then 7 degrees the other (14 degrees, more than one 9.2 degree tooth pitch, so the
   spline's phase is passed wherever it is), then back to square: the spline drops in during the twist, and turning
   back squares the housing over the notches.
5. Press 1 mm past the seat, rock 2 degrees each way and back to square (the tabs drop into the notches), hold, open,
   lift clear and park.

`GEARBOX_DEMO=press` in front of the run command plays the plain way instead (a check, not the demo): the same carry,
square, then a straight press without any twist, and let go. Lot B's start phase is lot A's at half a turn, so the two
lots together cover a straight press at both square turns.

## Fidelity

| Value | Source |
|---|---|
| Stage outlines: 50.8 mm square; cartridges 12.7 mm, output stage 19.1 mm, input stage 19.1 mm thick; ears 61.9 mm across, holes 5.0 mm at 54.0 x 42.9 mm | REV drawings REV-21-2101/2102/2103-DR, REV-21-2104-DR, REV-21-2106-DR |
| Cartridge bore 8.3 mm; coupler 25.4 mm across, 14.4 mm long, spline 4.9 mm deep; motor 60 mm across, 58.3 mm long, shaft 8 mm, 35 mm from the face | REV-21-2101-DR, REV-21-2108-DR, REV-21-1650-DR |
| Cartridge masses 133 g (4:1), 155 g (3:1); motor 0.43 kg | REV product pages |
| Spline: 39 teeth, 20.5/19.1 mm (male), 20.7/19.3 mm (female) | estimated: teeth counted and diameters measured on the drawings (the maker gives the tooth count as odd) |
| Tooth side play 0.10 to 0.15 mm; tip chamfer 0.10 mm | estimate (typical for a small press-together spline) |
| Spline engagement 3.0 mm when seated (coupler's top 1.2 mm below the seat); shaft 15.9 mm above the seat | NEO V1.1 drawing (REV-21-1650-DR: 3.5 mm pilot, shaft 31.5 mm beyond it) with the coupler's 14.4 mm resting on the pilot and the input stage's 19.1 mm |
| Alignment tabs 20 x 2 x 1.1 mm, two opposite edges; notches 20.6 mm wide, 1.4 mm deep, 0.25 mm mouth chamfer | estimate from the drawings' views (not dimensioned) |
| Output stage 90 g (stack 0.40 kg); sun with planets 20 g | estimate (base kit 261 g includes the output and input stages, coupler and screws) |
| Gear-train drag on the sun 0.6 N m s/rad; motor shaft drag the same | estimate; set so that a light twist (a few newtons of press) turns the sun with the housing while the tooth flats slide |
| Friction: housings 0.4/0.3, spline teeth 0.3/0.25 (static/dynamic) | estimate (anodised aluminium, greased steel) |

Simplified: the input stage's two ears are drawn at its foot rather than at its top (nothing in the task touches
them; the stack seats on the input stage's square face either way). The three pressed-together stages are one rigid
body, and of the gears inside only the 4:1 cartridge's
input sun is modelled (a hinge with drag stands for the gear train; the real sun is also held by its planets, which the
drag represents). Bores are drawn as 12-sided or square holes, ears and tabs as blocks. The coupler's key is not drawn
(coupler and shaft turn as one). No screws: the task ends where the maker's screw step begins. The robot's arms are the
program's YAM model; a seated stack stays put without them. Two lots stop one fixed habit, not a memorised replay of
both layouts.

## Real-world build

| Part | Source | Price (Nov 2026) |
|---|---|---|
| NEO Brushless Motor V1.1, REV-21-1650 (any motor with an 8 mm keyed shaft at least 35 mm long and the NEO's face serves; unpowered) | revrobotics.com | $42.50 (listed as discontinued) |
| MAXPlanetary Base Kit, REV-25-2109: output stage REV-21-2104, Universal Input Stage REV-21-2106, 8 mm keyed input coupler REV-21-2108, hardware pack | revrobotics.com | $36.00 |
| MAXPlanetary 4:1 cartridge REV-21-2102 and 3:1 cartridge REV-21-2101 | revrobotics.com | $32.00 each |
| Motor stand and cradle, printed PLA (about 90 g of filament) | `fixture/motor-stand-1to1.svg`, `fixture/cradle-1to1.svg` | about $3 |
| Two 1-1/2 in #10-32 socket head cap screws (the length REV gives for two cartridges), for the seat check | any hardware store | about $1 |
| Feeler gauge set with 0.30, 2.0 and 2.5 mm leaves; steel rule; protractor | any hardware store | about $15 |

Total about $160. The printed stand holds the motor with two M4 clamp screws. The input stage's two ears stand at its
top, level with the seat, 34.5 mm out from the axis (outside the motor body), so a gearbox screw can be pushed up
through each from below for the seat check, with about 45 mm clear above the stand.
Assembly: screw the input stage to the motor with two of the kit's 1/2 in 10-32 screws (REV step 3), fit the key
and slide the coupler fully onto the shaft (steps 4 to 6), stack the 3:1 cartridge and then the 4:1 onto the output
stage and press them together (steps 7 and 8), stand the motor in its stand at the setup's position and screw the
stand to the bench, then set the start as `reset_notes` says. Nothing is powered, nothing is
sharp, and a dropped stack falls 25 to 80 mm onto the bench.

## Results

All runs with the instruction above, `object_jitter=0`, the 800-step limit and `--epochs 2` (lots A and B), scored by
`task_score`.

| Run | Lot A | Lot B | Mean |
|---|---|---|---|
| Scripted demo (the submitted video) | 1.0 | 1.0 | 1.0 |
| Dream-1 | 0.0 | 0.0 | 0.0 |
| Straight-press check, `GEARBOX_DEMO=press` (no twist) | 0.25 | 0.25 | 0.25 |

- **Demo.** Both trials were done at 608 steps (tabs in their notches by about 23 s, let go at about 27 s, still 1 s
  later). In
  a trace of the same code, lot A's spline stood on its tooth flats 3.0 mm proud until the twist; it dropped in at
  5.5 degrees from square (tabs on the face, 1.0 to 1.6 mm at the corners), and turning back square let the tabs into
  the notches; let go, the corners read 0.0 mm. Lot B's spline dropped in at 6.8 degrees the other way and seated the
  same way. The motor shaft turned a few degrees with it (its drag is the estimate above).
- **Dream-1.** In both trials both arms rose and stayed still over the table; neither touched the stack (0.0).
- **Straight press.** Carried square and pressed straight down, the stack threaded onto the shaft and then stood on
  the spline's tooth flats, 3.0 mm proud at all four corners, in both lots (lot A's start phase is lot B's at half a
  turn, so this covers both square turns): 0.25 each. Lot A came from a two-lot check run made before the last change
  to the demo's approach (a slower descent onto the shaft); lot B was re-run alone (`TASK_LOT=B`) on the final code,
  because in that check its stack landed on the shaft's end and fell off (0.0).
- **Development.** The first trace landed the stack on the shaft's end 3.8 mm off the axis, which set `COMP`. An
  earlier version (spline starting half a tooth off, 0.15 mm tip chamfers) also scored 1.0 in both lots; the independent
  review then showed that half a tooth off at square is tooth-in-gap at half a turn (39 is odd), so the start became a
  quarter tooth and the chamfers 0.10 mm. One demo run with the heights over the shaft raised to undo the arm's sag
  missed the shaft in lot A and was stopped; that change was dropped.

## Files and licences

- `task.py`: scene, scoring, demo. `near_misses.py`: offline cases for every score level and the whole-trial rules.
  `make_video.sh`: builds the videos and `setup.png` from the recorded camera frames (nothing is drawn in but labels).
- `parts.py`: a multi-shape parts kit for yam_arena task files (hinged links, explicit inertia, per-part contact
  offset); `lots.py`: the per-trial layouts. Both are task-local helpers that patch only this task's own process; no
  installed file is edited. Task code and helpers: MIT licence.
- `fixture/`: full-size drawings of the two printed parts (`motor-stand-1to1.svg`, `cradle-1to1.svg`) and the script
  that draws them.
- No third-party meshes or assets are included: every shape is a box or cylinder built from the dimensions above. The
  YAM robot, station table and cameras are runtime dependencies from yam-isaac-lab-arena (its YAM models are I2RT's,
  MIT licence) and Isaac Lab-Arena; none are redistributed. REV part numbers, dimensions and quotations come from REV
  Robotics' public product pages, drawings and documentation, cited above; naming them is not a claim of sponsorship.
