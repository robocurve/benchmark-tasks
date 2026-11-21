# Set a 1000 µL pipette to 250 µL the way its manual says, lock the volume and hang it back in its stand

An adjustable single-channel air-displacement pipette (100-1000 µL in 2 µL steps, like the Rainin Pipet-Lite XLS+
L-1000XLS+) hangs upright in a single-pipette stand, its volume locked at 200 µL. The instruction asks for it to be
set to 250 µL the way its manual says (finishing by dialling down onto 250 from at least a third of a turn above),
locked again and hung back in its stand.

Setting the volume comes before nearly every pipetting step in a wet lab, and the makers all describe the same
procedure. Rainin, Pipet-Lite XLS+ operating instructions, "1.2 Setting Volume" ([manual][rainin]):

> 1. Turn the volume lock lever to the "unlock" position so the volume setting mechanism is unlocked and free to turn.
> 2. With the mechanism unlocked, orient the pipette so you are looking at the volume indicator, then rotate the
>    plunger button to change volume – counter-clockwise to increase, and clockwise to decrease volume.
> 3. To eliminate errors due to mechanical backlash: when setting the desired volume, first turn the knob 1⁄3 turn
>    above the desired volume. Then turn the knob slowly clockwise until the desired volume is displayed. Always dial
>    down to the desired volume.
> 4. Turn the volume lock to the "lock" position (Fig. 2 above) to prevent changes to the volume setting.

and, on storage: "The two biggest sources of damage occur with sample contamination inside the shaft, or with
dropping the pipette. ... Storing your pipette on appropriate hangers when not in use will reduce the chance of
drops." Gilson's PIPETMAN L user's guide ([guide][gilson]) gives the same lock and rule ("unlock the thumbwheel by
pushing it up"; "when increasing the volume setting, pass the required value by 1/3 of a turn and then slowly decrease
to reach the volume, making sure not to overshoot the mark"; "Push down the thumbwheel; the new volume selected is
locked"), and so does Eppendorf's Research plus operating manual ([manual][eppendorf]: "We recommend to adjust the
volume setting from a higher value to a lower value. If required, turn beyond the required value and then back
again.").

## The hand skill, and why two hands

A person holds the pipette's handle in one hand and turns the plunger button with the thumb and finger of the other
(or of the same hand, which a gripper cannot do). The robot has the same problem, and it is visible in the scene:

- The button is a 14 mm knob on the end of a 25 cm, 0.1 kg pipette that is not fixed to anything. Turned on its own,
  the whole pipette turns with it: it rests loose on its shoulder in the stand, and in fingers that do not squeeze it, it slides out and falls.
  So one gripper has to hold the handle firmly while the other turns the button (and the lock collar).
- A wrist turns only about 240 degrees, and the setting needs 1.3 turns of the button in all (up a third of a turn
  past 250 µL, then back down), so the button is turned in strokes: grip, turn, let go, turn the open hand back, grip
  again.
- The reading must stop on one 2 µL step, 7.2 degrees of the button, and must come down onto it: overshooting below
  250 µL means going up past 284 µL again.
- The pipette has to come out of the stand (its button is 26 cm above the table, out of the arms' reach for turning
  in place) and go back into the stand's slot upright.

Two more ways the scene punishes a careless hand: the lock collar sits right under the button, so a grip on the
collar must not also turn the button, and dropping the pipette (one of the two biggest causes of damage the maker names) sets the score to 0.

The task is the task file [`set_a_micropipette_to_250_microlitres.py`](set_a_micropipette_to_250_microlitres.py) for
the `yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac
Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable,
installs everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no
third-party assets: the pipette and the stand are built in these files from boxes and cylinders.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or put one on a joint. The
pipette (a loose body with the lock collar and the plunger button on revolute joints about its axis) and the stand
are built with `bodykit.py`, which adds such bodies and joints: importing it hooks `yam_arena.scene` when the simulator
starts, without changing yam_arena, and gives the task file each body's pose and each joint's angle. The task file
sets the button joint's spring (stiffness and target) every control step through Isaac Lab's articulation API: that
is how the lock and the button's detent are modelled (below).

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, z up from the table top.

- Single-pipette stand, fixed (taped down): a base plate 12 x 10 x 0.8 cm centred at x 30, y 0; a post 1.6 x 3 cm at
  x 31.5 up to the arm; the arm, 6 mm thick, its top 13.5 cm above the table, ending in a U-slot 2.3 cm wide open
  toward the robot, centred at x 25, y 0.
- Pipette, loose, hanging upright in the slot with its handle's shoulder resting on the arm (axis at x 25, y 0). From
  the shoulder down: a lower taper 1.6 cm across and 3 cm long, the ejector sleeve 1.3 cm across and 8.5 cm long, the
  tip cone 0.7 cm across and 1 cm long (no tip fitted). Up: the handle 2.8 cm across and 10 cm tall, with a finger hook
  on its far side and the volume display window on its near side; the volume-lock collar (2.2 cm across, 0.8 cm tall,
  with a lever tab, dark); the plunger button (1.4 cm across, 1.7 cm tall, blue, a white line down its side). 25 cm
  long, 0.097 kg. It is locked at 200 µL.
- Both arms start at the ready pose, grippers open. The layout is the same in every trial.

What is modelled:

- The lock collar turns about the pipette's axis from 0 deg (locked) to an end stop at 40 deg (unlocked), with
  0.02 N m of friction. The lock is open above 30 deg; at 30 deg or less it holds the button.
- The plunger button turns about the axis. Counter-clockwise seen from the button's end raises the volume, 100 µL per
  turn (2 µL per 7.2 deg). Unlocked, a weak spring pulls it toward its nearest 2 µL step (at most 0.013 N m, half a
  step away: a detent); locked, a stiff spring (20 N m/rad) holds it at the step where it was locked.
- The display reads the volume to the nearest 2 µL step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the pipette's pose, the collar's and the button's joint angles, and the points of
each finger's collision hulls.

- 0 for the rest of the run: the pipette dropped (it reaches the table or the floor faster than 0.3 m/s, or with no
  finger within 1 cm of it, as when it falls out of a hand).
- 0.1: the lock has been released (collar past 30 deg) at some time.
- 0.3: the display reads 250 µL (the volume 249.0 to 250.99 µL).
- 0.5: it reads 250 µL, set from above: since the reading was last below 250, it has been at 284 µL or more (a third
  of a turn above).
- 0.75: as 0.5, with the volume locked (collar at 10 deg or less).
- 1.0 (success, held 1 s): as 0.75, with the pipette hanging in its stand: its axis within 10 deg of vertical and in
  the slot (within 3 mm of the slot's centre line, from 9 mm toward the open end to 4 mm toward the round end, measured
  from the round end's centre), its shoulder within 3 mm of the arm's top; every finger point 1 cm or more from it; and
  it still (moving less than 1 mm in 1 s).

Doing nothing scores 0. A real trial is scored from a photo and the video: the display read in a photo at the end
(0.250 mL); whether it passed 0.284 mL before coming down to 0.250 (and never went below 0.250 after that) from the
video of the display; the lock lever's position and the pipette hanging in its stand, let go, in the photo; a drop
from the video.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=set_a_micropipette_to_250_microlitres.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Set this pipette to 250 µL the way its manual says (finish by dialling down onto 250 from at least a third of a turn above), lock the volume again and hang the pipette back in its stand." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/pipette-dream
```

The scripted demo, which reaches full score (open loop from the known layout). The left gripper comes in level from
the robot's side and takes the handle (fingers closing across it, 4.5 cm above the shoulder, squeezing it), lifts the
pipette 4 cm out of the slot, draws it back 3.5 cm clear of the stand, turns its wrist a quarter turn so that the
pipette lies level along y with its button toward the right arm, and lowers it 2.5 cm. The right gripper, pointing
along the pipette (+y), takes the lock collar with its fingertips just onto the collar (the 14 mm button sits between
its fingers, untouched), turns it 45 deg to the 40 deg stop (unlocked), lets go and backs onto the button. It turns
the button in three strokes, opening and turning the open hand back between them: 180 deg and 140 deg
counter-clockwise seen from the button's end (200 to 288 µL), then 140 deg clockwise (down to 250 µL). It takes the
collar again, turns it back to its 0 stop (locked), lets go, backs away along the pipette and returns to the ready
pose. The left gripper turns the pipette upright, carries it over the slot, lowers it until its shoulder rests on the
arm, lets go and backs away. It reaches success at step 1,356 (67.8 s).

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=set_a_micropipette_to_250_microlitres.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Set this pipette to 250 µL the way its manual says (finish by dialling down onto 250 from at least a third of a turn above), lock the volume again and hang the pipette back in its stand." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/pipette-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/pipette-dream/*.json; do inspect-robots video "$log"; done
```

`PIPV_DEBUG=1` prints, every 5 steps (`PIPV_DEBUG_EVERY=1` for every step), the volume and the display's reading,
the highest volume reached, the button's and the collar's angles, whether it is locked, whether the lock has been
released and the reading has come from above, the pipette's tilt, its shoulder's distance from the slot, how close the
nearest finger is, where each gripper is and the wrist angle, and whether it was dropped.

`PIPV_MODE=loose` runs the demo the wrong way: once the pipette is level, the left fingers only touch the handle
(a 28.6 mm gap round the 28 mm handle) instead of squeezing it, so nothing holds the handle while the right hand works.
The pipette slides forward out of the fingers and falls onto the table about 20 cm away, with no finger within 1 cm of
it: dropped, 0 for the rest of the run. Score 0. (`PIPV_MODE=nolock` sets the volume but leaves it unlocked; by the
rules above that ends at 0.5. It was not run separately.)

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell Server Edition shared with other simulation jobs: the scripted demo while
rendering 1280 x 720 video took about 9 minutes from start to finish (1,356 steps, 67.8 s of simulated time, ending
on success); a Dream-1 trial of 2,400 steps at 320 x 176 took 10.6 minutes. Debug runs of the demo without video
took 6 to 9 minutes. Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

Any adjustable 100-1000 µL single-channel pipette with a turning volume lock (Rainin Pipet-Lite XLS+ L-1000XLS+, or
a Gilson PIPETMAN L P1000L, whose lock is a thumbwheel pushed up and down) and a single-pipette stand or a carousel
with a U-slot hanger, taped down so that the slot is 25 cm in front of the midpoint between the arm bases. Before each
trial unlock the pipette, set it to 200 µL coming down from above, lock it and hang it in the slot with its display
toward the robot.

[rainin]: https://jack-sheets.squarespace.com/s/Pipet-Lite_XLS-_User-Manual_2014_Rainin-Rev-G-2.pdf
[gilson]: https://jack-sheets.squarespace.com/s/Pipetman_L-User_Manual_Jan_2013_Gilson.pdf
[eppendorf]: https://jack-sheets.squarespace.com/s/Research-Plus-User-Manual-2016-Eppendorf.pdf
