# Set a gym weight stack to 70 lb: pull the selector pin and push it fully home in the 70-lb plate

Moving the pin on a weight stack is the first thing anyone does at a cable machine, a lat pulldown or a home gym, and
everyone who has been to a gym has seen it done. The scene is the weight stack of a selectorized machine at true
size: twelve 10-lb plates (28 x 15 x 2.5 cm) on two guide rods, the selector stem running down through them, and the
selector pin (9.5 mm, 3/8 in, with a red knob) sitting in the 40-lb plate. The instruction is only "Set the weight
stack to 70 lb."; it does not say how.

What makes it hard is a blind insertion with one hand, and the object makes it so. The pin first has to come straight
out of the 40-lb plate against the magnet in its knob. It then has to go into the 7th plate from the top (the plates
are 2.5 cm apart), level and square to that plate's 13 mm bore, and be pushed 11.5 cm in until the knob meets the
plate. The stem's cross hole, which actually carries the load, is hidden 6 cm inside the plate and is only 0.5 mm wider
all round than the pin: the hand cannot see it, so it has to keep the pin straight while it pushes and let the pin's
rounded tip find the hole. A pin tilted a few degrees binds against the stem part-way in; a pin at the wrong height
hits a plate's face; a pin in the 6th or 8th plate selects 60 or 80 lb; a pin left part-way in carries the stack on
the tip of its shaft. One arm does it, as a person does; the other arm is not needed.

The task is the task file
[`set_a_gym_weight_stack_to_70_lb_with_its_pin.py`](set_a_gym_weight_stack_to_70_lb_with_its_pin.py) for the
`yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Why it matters

Selectorized machines (cable stations, lat pulldowns, leg extensions, home gyms) are in every gym, and the pin is how
every user sets the load. Owner's manuals insist that the pin goes fully in: "Make sure that the weight pin is
inserted fully into the weight stack before you exercise" ([Gold's Gym GGSY30580 manual][golds], Important
Precautions, item 13); "Be certain that weight pins are completely inserted prior to exercising" ([Nautilus NS200
manual][nautilus]). A service manual's selector-pin check reads "Verify the selector pin slides in and out of each
weight plate" ([True SPL0900 manual][true], Weight Stack Selector Pin Inspection). Anyone watching a clip knows at once
what the robot is doing and why getting the pin fully home matters.

The parts follow commercial stacks: the plate footprint and thickness follow Body-Solid's selectorized stack plates
([listing][bodysolid]: "11.02" L x 5.91" W x 0.98" H"), labelled in 10-lb steps as on most machines (Body-Solid's
SP150 stack has fifteen 10-lb plates); the pin is a 3/8 in (9.5 mm) magnetic selector pin, the common size for
commercial stacks, and its 11.5 cm shaft reaches 77 % of the way through the 15 cm plate, as a supplier of magnetic
stack pins advises ([Physioparts][physio]: "the pin must go through the center of the stack, the selector and far
into the other half of the stack. The pin must go at least 75% through the stack.").

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac Lab-Arena
0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable, installs
everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no
third-party assets: the stack, the stem and the pin are built in these files from measurements.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or put them on a joint. The
plates (each a plate with a middle hole and a pin bore, built from 11 boxes), the stem with its cross holes and the
top plate, and the pin are built with `bodykit.py`, which adds such bodies and joints: importing it hooks
`yam_arena.scene` when the simulator starts, without changing yam_arena, and lets the task file read every body's pose
after each control step and move the stem for the lift test.

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, z up from the table top.

- Base (fixed): a steel frame foot 19 x 34 x 5.25 cm under the stack, x 30.5 to 49.5, y -20 to 14. Two 1-inch
  (2.54 cm) chrome guide rods rise from it at x 40, y -13.5 and 7.5, to a top bar 52 to 54 cm up.
- Twelve steel plates, 28 cm wide (y) x 15 cm deep (x) x 2.5 cm thick, black, each with its weight printed in white on
  its front face (10, 20, ... 120 lb from the top down): front faces at x 32.5, backs at 47.5, y -17 to 11, stacked
  from 5.3 to 35.6 cm above the table. Each plate has a 3 cm square hole in its middle for the stem and a 13 mm square
  bore running in from the middle of its front face (at y -3) across the stem's hole to 2.5 cm short of its back face,
  its mouth marked by two white lines. Plates 2 to 12 each ride on the guide rods (a slide joint along z), free to rise
  and resting on the plate below; the top plate (10 lb) is fixed to the stem. The 7th plate (70 lb) has its middle
  19.2 cm above the table, the 4th (40 lb) 26.8 cm.
- Selector stem: a 2.5 cm square steel bar at x 40, y -3, from the base up through every plate's middle hole to 41.6 cm,
  with a 10.5 mm square cross hole (along x) at the middle of each plate, lined up with the plates' bores; a clevis and
  the cable on top (visual). It is held in place (kinematic) and moves only for the lift test.
- Selector pin, loose, 0.11 kg: a 9.5 mm steel shaft 11.5 cm long from the knob's face to its rounded tip, and a red
  plastic knob 4.4 cm across and 2.5 cm long. It starts in the 40-lb plate (the 4th from the top), pushed fully home:
  the knob's face against the plate's face at x 32.5, y -3, 26.8 cm up; the shaft through the stem, its tip 11.5 cm in.
  The magnet in its knob holds it there until it is pulled with 6 N (a joint to the stem that breaks for good at 6 N
  or 0.5 N m).
- Both arms start at the ready pose, grippers open. The layout is the same in every trial.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the pin's pose, every plate's height, and the points of each finger's collision
hulls.

- The lift test (the cable pulling the stem, as a person tugs the handle to check the setting): once the pin has been
  touched and then left alone (no finger point within 10 mm of it) for 1 s, the stem rises 4 cm at 1 cm/s, holds
  0.5 s and comes back down. The plates that rose 2 cm or more with it are counted (the top plate, fixed to the stem,
  counts as one). Only the pin can carry plates up: through the stem's cross hole and the plate's bore. The test runs
  again after every new touch of the pin (a finger point within 4 mm of it), and a touch cancels the earlier
  result.
- 1.0 (success, held 1 s): the latest lift test, with the pin not touched since, raised exactly 7 plates (70 lb), and
  the pin is fully home: its knob's face within 5 mm of the plate's face.
- 0.5: the latest test raised exactly 7 plates but the pin is not fully home (it carries the stack on the tip of its
  shaft in the stem's hole: the part-way pin the manuals warn about).
- 0: the latest test raised 2 to 6 or 8 to 12 plates (a wrong weight is selected), or the pin was never drawn out of
  the 40-lb plate (doing nothing scores 0).
- 0.3: otherwise, if the pin was drawn fully out of the 40-lb plate (its tip in front of the plates' faces) at some
  point: the old setting is cleared but no weight is selected (a test raised only the top plate: the pin lies on the
  table or sits in a bore short of the stem), or the pin is still held at the end, or no test ran.

A real trial is scored the same way when the run ends: if a gripper still holds the pin, the stack is not tested;
otherwise a person pulls the cable handle (or lifts the stem by its top) about 4 cm and counts the plates that rise,
and checks with a ruler that the knob is within 5 mm of the plate's face; whether the pin was drawn fully out of the
40-lb plate is seen on the video.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=set_a_gym_weight_stack_to_70_lb_with_its_pin.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Set the weight stack to 70 lb." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/weightstack-dream
```

The scripted demo, which reaches full score (open loop from the known layout; the left arm stays at the ready pose).
The right gripper, level and pointing at the stack with its fingers closing across the knob (along y), comes in from
the right in front of the knob, moves round it, closes on it and draws the pin straight out 14 cm, its tip 2.7 cm
clear of the plates. It carries the pin down 7.6 cm to the 70-lb plate's height with its tip
1.2 cm in front of the plate, holds it there 1 s, and pushes it home over 5 s, aiming 1 cm past the seat (the arm stops
short of a push target). It holds it there 1.5 s, opens, backs off 8 cm and stays clear while the lift test runs.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=set_a_gym_weight_stack_to_70_lb_with_its_pin.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Set the weight stack to 70 lb." \
  --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/weightstack-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/weightstack-dream/*.json; do inspect-robots video "$log"; done
```

`GYM_DEBUG=1` prints, every 5 steps (`GYM_DEBUG_EVERY=1` for every step), the lift test's phase and the stem's lift,
the score state, the results of the tests so far, whether the pin is fully home, whether it was drawn out of the 40-lb
plate, how close the nearest finger is, the pin's pose and tip, how far each plate has risen, and where each gripper
is.

Wrong ways:

- `GYM_WAY=plate6`: the same demo, but the pin is pushed home in the 60-lb plate (the 6th from the top). The lift test
  raises 6 plates: a wrong weight is selected. Score 0.
- `GYM_WAY=tilt`: the same demo, but the pin is pushed in pitched 4 deg (tip up). It binds part-way: the knob stops
  3.6 cm short of the plate, the tip 1.6 cm into the stem's 2.5 cm cross hole. The lift test raises 7 plates, but on
  the tip of the pin, which is not fully home. Score 0.5.

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell shared with other simulation jobs, the scripted demo while rendering 1280 x 720
video took about 4.6 minutes from start to the end of the run (681 steps, 34 s of simulated time, ending on success);
a Dream-1 trial of 2,400 steps at 320 x 176 took about 14.7 minutes. Debug runs of the demo without video took 4 to 8
minutes. Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## What is modelled and what is not

The plates, the stem and the pin are rigid bodies at true size; the pin's magnet is a joint that lets go for good at
6 N. Each plate weighs 1.0 kg in the simulation instead of 4.5 kg (10 lb): with 4.5 kg plates the light pin, squeezed
between the stepped stem and the plates during the lift test, was pushed through the plate's bore wall in the
simulator (the lift then counted the wrong plates). The robot never moves the plates; only the lift test does, and it
counts plates, not force. The stem is stepped up 0.5 mm per control step rather than pulled by a cable. The machine's
frame, cable run, pulleys and handle are not modelled; the pin's lanyard is left out. The pin's magnet does not hold it
again once it has let go, so the full-score check measures the knob against the plate instead.

## Building it for real

A selectorized weight stack of twelve 10-lb plates (for example from a home gym, or a Body-Solid stack, a few hundred
US dollars) on its guide rods and base, set at the table's far edge so the plates' faces are 32.5 cm in front of the
arm bases and the 70-lb plate's bore is 19.2 cm above the table (on a raised base under the stack if needed), with its
selector stem and a 3/8 in magnetic selector pin. Its frame, cable and handle can stay; for the check, a person pulls
the handle or lifts the stem by its top. For each trial put the pin fully into the 40-lb plate and start. The plates
weigh 54 kg in all: the stand must carry them, and the robot only ever handles the 0.11 kg pin.

[golds]: https://auto.manualsonline.com/manuals/mfg/golds_gym/ggsy30580.html?p=3
[nautilus]: https://www.manualslib.com/manual/799058/Nautilus-Ns200.html?page=5
[true]: https://www.manualslib.com/manual/3323246/True-Spl0900.html?page=48
[bodysolid]: https://bodysolid.com/300-lb-selectorized-weight-plates
[physio]: https://www.physioparts.co.uk/magnetic-weight-stack-pin
