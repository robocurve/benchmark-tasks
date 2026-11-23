# Load a full column of eight tips onto an 8-channel pipette, every tip sealed and level, without moving the tip box

Two YAM arms at the station table. A 96-tip box of universal 200 µL tips sits loose on the bench on its tape marks, with
columns 1 to 3 already used. Beside it an 8-channel pipette (20–200 µL, conical friction-fit nozzles, 9 mm pitch) stands
upright in a two-rail stand, as labs park one between uses; both arms start at the ready pose, so the right hand
first takes the pipette from its stand. The robot must load all eight tips of column 4, every one fully on its nozzle, lift the pipette clear of
the box and hold it, while keeping the box on its marks and never touching a tip with a gripper.

Each tip is a friction fit on a cone: it goes on only as far as the force pushing it. The tips stand at slightly
different heights, so a straight push loads the highest first and leaves the end tips short unless it is pressed very
hard. The manuals' way, which the reference demo uses, is to press and rock the row side to side, so that the force goes
to a few tips at a time.

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena),
Isaac Lab-Arena 0.3.1).

## Run a trial

Any policy, here Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Load tips from column 4 of the tip box onto all eight channels of the pipette, every tip fully sealed, then lift the pipette so the tips are clear of the box and hold it there. Don't touch the tips with a gripper, and keep the box on its marks." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir runs/dream
```

Scripted demo, through Inspect Robots' CLI with the demo registered as a policy:

```bash
python run_demo.py runs/demo            # 640 x 360; --width 320 --height 176 for Dream-1's size
TASK_CLOSEUP=runs/demo/closeup python run_demo.py runs/demo   # also saves the video's close-up, one picture a step
```

The run prints `task_score` at the end: 1.0 once all eight tips are seated and level with the pipette held for 2 s.

**The demo is a scripted reference that reads the poses of visible objects.** Each control step it reads the pipette's
and the tip box's poses from the simulator (both in view of every camera) and its own joints. It never reads a load, a
tip's depth or the scorer's state. It works out where the nozzles are over the tip mouths from the two poses and the
pipette's geometry, presses each time to the depth at which a tip is fully on plus 0.5 mm (the tips' own heights are not
visible), and closes out what the soft arm gives under load from the pipette's observed pose. A policy gets only the
cameras and the joints. `tipdemo.py` holds the demo; `demokit.py` its arm primitives.

## What the video shows

`demo.mp4` (the scripted demo, score 1.0; 86.5 s simulated, 107.5 s of video) has four panels: top left, the top camera
(the whole scene); top right, the left wrist camera (it looks down at the tip box's far end while that hand holds the
rim) above the right wrist camera (mostly the white pipette handle it holds). The wrist cameras are fixed to the
grippers, so the held pipette and the box rim block much of their view.

Below them is a close-up of the nozzles seating in the tips, labelled "not a policy camera": a fixed camera used only
for the video (`closeup.py`, on only with `TASK_CLOSEUP` set), on the arms' side just above the box rim, looking across
the empty columns 1–3 at column 4. It shows the eight nozzles going into the tip mouths and pressing in, and the tips
coming up on them at the lift. It adds no body, collider or light, and the policy's cameras and inputs are as without
it; the run with it on gives the same trace, step for step, as without it. The video plays 4x slower from 15 to 18 s
and from 81 to 85 s (marked on screen, with the simulated time); the times below are simulated time.

- 0–6 s: the right hand comes in behind the pipette in its stand and grips the handle (about 6 s); the left hand goes to
  the box's far rim and holds it down.
- 7.5–10.5 s: the pipette is lifted out of the stand and carried over column 4.
- 10.5–16 s: it is lined up over the column and lowered; all eight nozzles enter their tips at about 16 s.
- 17.5–81.5 s: press and rock: rolls of ±6° that push the end tips on, then smaller rolls for the middle ones; a small
  front-to-back rock; then it eases off. The rocking is the manufacturers' loading technique: each roll loads only two or three tips,
  so all eight seat at a peak of about 11 N instead of the 30–40 N a straight push needs (beyond the arm). The score
  checks only that all eight tips are seated; a straight push that seats them all would also score.
- 81.5–84 s: the lift, straight up. All eight tips come away at full seat (82.0 s); column 4's holes are empty from then
  on.
- About 84 s: the left hand lets go of the box and moves up, while the pipette is raised a little further and turned 35°
  (84–85.3 s). The turn has no role in the task or the score: it only moves the
  wrist out of the top camera's line of sight so the seated tips can be seen.
- 85.3–86.5 s: the pipette is held, tips more than 3 cm above the rim; the run ends at 86.5 s, after 2 s of the
  full-score state.

The seated tips themselves are small and mostly hidden behind the right hand in the top camera; the close-up shows them
coming up on the nozzles at 82–84 s, and column 4's row of empty holes after the lift (also in the left wrist camera
from 82 s, top camera from about 85 s, once the left hand has moved away). Their depth
and level are measured by the scorer from the simulated poses.

## How it is scored

`tip_rules.py` holds the scoring as plain functions; `tipkit.py` measures the scene for it every control step.

- **FULL_SEAT_MM** is measured once per session on the trial's own pipette: hard-seat tips on it by hand and measure
  each tip's end below the manifold's underside with a steel ruler. The simulated pipette's value is 54 mm.
- **Seated:** the tip is on its own nozzle and its lower end is within 1 mm of FULL_SEAT_MM below the manifold's
  underside, measured from the tip's and the pipette's poses.
- **Level:** the seated tips' ends are within 1 mm of each other.
- **Held:** the pipette touches both fingers of one gripper, is within 10° of vertical, and every tip on it has its end
  at least 3 cm above the box rim. The score is taken at the end of the run, and the pipette must have been held for
  the last 2 s (a full score ends the run once it has held 2 s).
- 1.0 for all eight column-4 tips seated, level and held; 0.5 for seven; 0.25 for six; otherwise 0.
- 0 at once, for good, if: a tip from another column, or a column-4 tip on another row's nozzle, is pushed on; a gripper
  touches a column-4 tip (PhysX contact; the other columns' tips are part of the box and are not checked); a corner of the box moves more than 5 mm from its tape mark or lifts more than 5 mm; the total load on the rack
  insert passes 80 N (it pops out of its ledges); the pipette is dropped; or the pipette or a tip on it touches the table.

Offline checks of the fit law, its calibration and the scoring, no simulator: `python -m pytest test_tips.py`.

## Fidelity: how a tip goes on

All bodies are rigid; the tip fit is a model, in `tiplaw.py` (the law) and `tipkit.py` (applying it every 240 Hz
physics step).

- **Engagement.** Each nozzle is matched with the rack hole under it. A nozzle enters a tip if, as its end reaches the
  tip's mouth, it is within 0.7 mm and 4° of the tip's axis. Further off, but within half a pitch (4.6 mm), it lands on a rim and
  jams (a stiff stop, no seating). An entered nozzle keeps its tip until it rises out of the mouth. There is no
  attachment by nearness.
- **The fit.** Past the point where the cone first wedges in the mouth, a tip slides up the cone plastically: the force
  needed rises with the depth reached, F = k·u, k = 1.75 N/mm, and the depth reached stays. Below that depth the fit
  unloads elastically (15 N/mm). At full seat (u = 2.0 mm, F_SEAT = 3.5 N) the tip is hard against the nozzle's stop
  and what gives is the rack plate, 1.67 N/mm. Each tip's own seat height differs: the rack is moulded 0.3 mm higher in
  the middle rows than at the ends (±0.3 mm). The force acts up on the pipette at the nozzle's end and down on the box at
  the tip's hole, so the push goes through the rack and the box. The box is loose; in the simulator it stays put without
  a hold; the demo steadies it as labs do.
- **Coming off the rack.** When the nozzle has risen to about the tip's mouth, a tip pushed to full seat is moved onto
  its nozzle (the body is set there in one step, a snap of about 9 mm) and locked to the pipette with a fixed joint at
  full seat (its end 54 mm below the manifold's underside); a tip pushed to between
  0.4 F_SEAT and F_SEAT is locked 1.5 mm short, which fails the seated test; a tip pushed less stays in its hole. A
  short tip pressed back onto the rack with F_SEAT is re-seated. Only the full-seat lock counts as seated.
- **Calibration, from vendor data.** F_SEAT = 3.5 N a tip comes from Thermo Scientific's ClipTip application note
  (47.1 N to keep 12 friction-fit tips on, about 3.9 N a tip). That is a retention figure, so the real insertion force is
  likely higher. The straight-push total, 32 N, is about 8 × 3.9 N; the rocking peak, 12 N, is about 3 tips at a time.
  k and the rack's give are fitted together so that a straight, level push needs PEAK_STRAIGHT_N =
  32 N to bring all eight to full seat (`straight_push_total()`, exactly 32.0 N), and the best press-and-rock needs about
  PEAK_ROCK_N (`best_rock_peak()`: 13.0 N, within 10% of 12 N, with rolls up to ±6°, two or three tips loaded at a
  time). In the simulator, the scripted rocking demo seats all eight at a peak total of 11.4 N; a level push stopped at
  the script's extra-depth cap at 13.5 N with tips 0.74–1.58 mm on, none at full seat (32 N needed in the model).
- **Force check.** At the vendor figure of 3.5–3.9 N a tip the rocking peak is 12–13 N, under a fifth of the arm's
  computed 67 N push (below); if a real tip needs twice that to go on (7–8 N), the peak is about 24–27 N, still under
  0.7 × 67 N = 47 N.
- **Simplifications.** No liquid and no air seal: "sealed" is "at full seat". A nozzle that has entered a tip carries
  it sideways (the tip tilts in its hole) and only comes out by rising above the mouth; the nozzles collide with the
  scene only at their roots, below that the fit model acts. Tips in other columns are part
  of the box, rigid; pushing one on breaks a rule. A tip pushed on by the wrong row's nozzle is a broken rule rather than
  a loaded tip. The rack insert popping out of its ledges is not modelled: a total load over 80 N counts as it. The
  tips' own small masses (0.35 g) do not carry the fit's load in the rack: a 0.35 g rigid body under
  several newtons is not stable at 240 Hz, so the load goes into the box at the tip's hole.

## Recalibrating from a measurement

The simulator's numbers come from vendor data (above), not from a measurement of the build. To measure them:

- **Forces:** stand the full tip box on a kitchen scale (5 kg × 1 g, peak-hold or a filmed readout) and tare it. Press
  the pipette by hand onto a fresh column each time, three times each: straight down until all eight are at full seat
  ±1 mm (PEAK_STRAIGHT_N); then press about 15 N and rock ±6–8° until all eight are at full seat (PEAK_ROCK_N). Record
  each mean peak in kgf × 9.81.
- **Refit:** set `PEAK_STRAIGHT_N` and `PEAK_ROCK_N` in `tiplaw.py`; `K_HARD` follows from `PEAK_STRAIGHT_N`; refit
  `F_SEAT` (and with it `K_CONE`) and `K_ELASTIC` until `best_rock_peak()` matches PEAK_ROCK_N within 10%
  (`test_tips.py` checks both). A higher insertion force than 3.5 N raises both peaks about in proportion.
- **The arm:** the arm holds the pipette by the shoulder grip over a padded block on the same scale and pushes down
  under position control until the reading stops rising, three times. A static check in the simulator's arm model
  (effort limits 27 N·m shoulder and 7 N·m wrist, gravity compensation included, Jacobian at the seating pose) gives
  67 N, limited by wrist joint 4, well above 12 N / 0.7. If the measured rocking peak is above 0.7 × the arm's push,
  use the same maker's lower-retention tips and measure again.

## Checks in the simulator

Scripted runs, each from the same start (`TASK_DEMO` selects the mode in `tipdemo.py`):

In the model a level push needs 32 N to seat all eight (`tiplaw.py`); the scripted `straight_max` push stops at its
extra-depth cap well before that.

| Mode | What it does | Result |
|---|---|---|
| `full` (the demo) | press and rock ±6°, then smaller rolls; lift | 1.0: all eight at full seat, peak total 11.4 N |
| `straight_max` | one level push, pressing on until the end nozzles are seen at full-seat depth | 0: a level push stopped at the script's extra-depth cap at 13.5 N with tips 0.74–1.58 mm on, none at full seat (32 N needed in the model) |
| `offcol` | the same rocking over column 5 | 0: column-5 tips pushed on |
| `touch` | the left hand leaves the box to press tip H4 | 0, by the box leaving its marks before the fingers reached the tip (the tip-touch rule is covered by the offline tests) |
| `nosteady` | the demo with the left hand off the box | 1.0 |

Model run: Dream-1 on the task's own instruction and 3,600-step limit (320 x 176 cameras, 10 rows per request), one
session from the first action to the end: task score 0.0 (no tip seated). The video is `model_run.mp4`.

## Build spec

- **Pipette:** a manual 8-channel adjustable pipette, 20–200 µL, conical friction-fit nozzles without spring loading,
  9 mm pitch (for example United Scientific PMC8-200, about $310). Simulated geometry: manifold 85 × 40 × 70 mm, handle
  32 mm across and 120 mm long with a finger hook, plunger and ejector button, nozzles 12 mm long tapering 5.6 → 4.2 mm; 0.2 kg.
- **Tips and box:** universal-fit 200 µL yellow polypropylene tips, racked 96 to a box (for example Fisherbrand SureOne
  200 µL racked, about $10): tips 51 mm long, mouth 5.3 mm inside, 0.35 g; box 122 × 82 mm, rack plate 50 mm and tip
  mouths 58 mm above the bench, rim 52 mm, about 150 g full; hinged lid open flat on the −y side. Columns 1–3 empty.
- **Stand:** a two-rail pipette stand of white plastic: base 70 × 110 mm, two rails 50 mm long, 75 mm up and 72 mm
  apart inside, holding the pipette upright by the manifold's ends. Labs park pipettes in stands like this.
- **Layout** (table top z 0.7475 m, x forward, y left, m): box centre (0.2875, 0.000), long side along x, column 1
  nearest the arms, column 4 at x 0.265, rows A–H along y from +31.5 to −31.5 mm; tape marks at the box's four
  corners. Pipette in its stand at (0.31, −0.19), nozzle row along y. Friction: box on bench 0.4 / 0.35; pipette 0.9 / 0.8.
- **Restocking a trial:** eject the loaded tips into waste; wearing nitrile gloves, drop 8 tips from a spare box of the
  same tips into the column-4 holes; set the box on its marks and the pipette back in its stand.

## Runtime and seeds

- Demo: about 1,700 steps (85 s simulated); the success hold ends it. About 2.2 control steps per second of wall time
  on one GPU (the fit model runs in every physics step), so about 13 minutes with start-up.
- A full 3,600-step trial: about 27 minutes.
- `object_jitter=0` keeps the layout fixed; nothing in the scene is random, so no seed is needed.

## Files

- `task.py`: the scene, `score()` and `success()`, and the demo's entry point.
- `tiplaw.py`: the fit law, the vendor-data force values and the calibration; `tip_rules.py`: the scoring; `test_tips.py`: tests.
- `tipkit.py`: the parts (box, tips, pipette, stand, tape marks) and the fit model run in the physics step.
- `tipdemo.py`, `demokit.py`, `run_demo.py`: the scripted demo; `closeup.py`: the demo video's close-up of the nozzles
  (only with `TASK_CLOSEUP=<folder>` set; not a policy camera).
- `bodykit.py`: rigid bodies of several shapes for task files; `steptime.py`: step timing.

## Sources

Every shape is generated in `tipkit.py` from plain boxes, cylinders and turned profiles; there are no third-party assets.
Sizes come from the products named in the build spec. Technique references: Rainin PIPETMAN Multichannel and Pipet-Lite
manuals (press and rock to mount tips), Hamilton SoftGrip multichannel manual; force reference: Thermo Scientific,
*ClipTip technology – part 2* application note.

## Licence

MIT
