# Set an adjustable walking pole to its owner's mark: open the flip lock, draw the shaft out to the red line, lock it

Setting an adjustable walking pole (a trekking pole, or the pole many older walkers use as a walking stick) to its
owner's length is something most people who own one do by hand. A real-size three-section aluminium pole with two
flip-lock clamps lies across the table, set to 105 cm; its owner's length, 120 cm, is marked on the middle shaft's
printed length scale with a red line, still 15 cm inside the upper shaft. The instruction asks for the pole to be
lengthened until the red line is at the edge of its upper clamp, locked at that length, and let go of; it does not
say how.

What the task tests is the adjustment itself: a length adjustment against friction to a mark, then a friction lock
that must hold load. The upper clamp is a cam lever on a split collar. With the lever closed the shaft does not move
(250 N); the lever has to be flipped open (against the cam), the middle shaft drawn out 15 cm against the open
collar's drag (4 N) and stopped at the line (within 1 cm), and the lever pressed back down flat. The holding force
builds over the lever's last 25 degrees, so a lever left half shut slips when the pole is leaned on; the score's last
stage is that lean: once the robot has let go, the pole is pushed together along its axis with 150 N for 1 s.

Why two arms, visible in the scene: the pole lies loose on the table. With the lever open the collar still drags on
the shaft (4 N) while the pole's grip and clamp slide on the table at about 1.5 N, so a single hand pulling the shaft
drags the whole pole along the table and the length does not change; flipping the lever open with no hand on the pole
shoves the pole too. One hand holds the upper shaft above the clamp the whole time (as the maker says: "Hold the pole
above the FlickLock"), while the other flips the lever open, draws the shaft out to the line, and presses the lever
shut. The core is not "hold one part while the other hand turns it" or "press until it clicks": it is a slide to a
visual mark against friction and a cam lock whose holding force depends on how far it is closed.

The task is the task file
[`set_an_adjustable_walking_stick_to_its_mark.py`](set_an_adjustable_walking_stick_to_its_mark.py) for the
`yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Why it matters

Adjustable poles are sold by the million to hikers and to older people who walk with one; each owner sets the length
for their height and resets it after the pole has been collapsed for travel or lent to someone else. Black Diamond's
trekking pole instructions for FlickLock clamps: "Hold the pole above the FlickLock to prevent the pole from rotating
while pushing the cam lever open with your thumb.", "Adjust pole to the desired length.", "'Snap' close the lever
firmly." A pole that is not locked fully slips shorter when it is leaned on, which is how a walker falls. Anyone
watching a clip knows at once what the robot is doing.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on
Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable, installs everything, including the
Dream-1 policy. The task needs no models from Arena's object library and uses no third-party assets: the pole is built
in the task file from boxes and cylinders, from measurements and estimates of the product class.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or put one on a slide. The pole
is built with `bodykit.py`, which adds such bodies and joints: importing it hooks `yam_arena.scene` when the simulator
starts, without changing yam_arena, and gives the task file each body's pose. The pole is one loose PhysX
articulation: the upper section (grip, upper shaft and clamp) is its base, the middle section (with the lower clamp,
lower shaft and tip) slides in it on a prismatic joint with Coulomb friction, and the lever turns on a revolute joint
with Coulomb friction. Every control step the task file writes both frictions from the lever's angle
(`write_joint_friction_coefficient_to_sim_index`, effort-based on Isaac Sim 6.1: N and N m).

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, z up from the table top. Sizes and masses are those of the product class (Black Diamond Trail Back: 3-section
aluminium, two FlickLock clamps, 100 to 140 cm, 534 g a pair); the part dimensions are our estimates.

- The pole lies along y at x 24, loose, grip to the right and tip to the left, resting on its grip and its upper clamp
  with its axis 1.3 cm above the table. Total 267 g: upper section 150 g, middle and lower sections 111 g, lever 6 g.
- Grip: dual-density rubber, flat-sided (13 x 3 x 2.6 cm, so the pole does not roll), from y -61.5 (top) to -48.5,
  then a foam grip extension 2.4 cm across to y -41.
- Upper shaft: 18 mm aluminium tube (blue) to the upper clamp: a black collar 2.5 cm across, 3 cm long, y -3.5 to
  -0.5. Its lever (aluminium, 4.6 x 1.5 x 0.9 cm) pivots across the pole on top of the collar 5 mm from its lower edge;
  closed, it lies flat on the clamp pointing toward the grip; it opens up to 100 degrees.
- Middle shaft: 16 mm tube (silver), sliding in the upper shaft. Its printed scale (black lines every 5 cm, 100 to 140
  cm) is read at the clamp's lower edge (y -0.5). At the start it reads 105 cm. A red paint-marker line marks 120 cm
  (15 cm inside the upper shaft at the start); the maker's STOP line marks 140 cm.
- Lower clamp (closed at its setting, not part of the task), 14 mm lower shaft (blue) and a rubber tip, ending at y +43
  at the start (y +58 at 120 cm).
- Both arms start at the ready pose, grippers open. The layout is the same in every trial.

The clamp (modelled from how the cam works): the middle shaft's sliding friction in the collar is 4 N with the lever
open more than 25 degrees and rises linearly to 250 N as the lever closes from 25 to 2 degrees; the lever's pivot
friction is 0.06 N m open and rises the same way to 0.20 N m (the cam's squeeze, about 5 N at the lever's end).
These numbers are estimates.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the pole's pose, the slide's travel and the lever's angle. The length is measured
from the grip's top to the tip: 105 cm at the start, 120 cm with the red line at the clamp's edge.

| Score | When |
|---|---|
| 0.4 | the length has been within 1 cm of 120 cm at some time (the red line within 1 cm of the clamp's edge) |
| 0.7 | locked at the mark: the lever within 5 degrees of flat and the length within 1 cm of 120 cm |
| 1.0 | locked at the mark, released (both grippers' centre points 4 cm or more from the pole's axis), still for 1 s (under 1.5 mm), and then the load test passed; held 1 s |
| 0 | for the rest of the run once any part of the pole falls 2 cm below the table top, or once the shaft is drawn past its STOP line (140 cm) |

The load test: once the pole is locked at the mark, released and still for 1 s, the scorer pushes the pole together
along its axis with a force across the slide (on the middle section toward the grip and, as its reaction, on the upper
section toward the tip, as a person leaning on the grip with the tip on the floor), ramped over 0.2 s and held for
1 s, and measures how far the shaft slips: 2 mm or less, with the lever still within 5 degrees of flat, passes. If the
pole is touched again, unlocked or moved afterwards, it is tested again. In the simulation the lean is 20 N, not the
real trial's 150 N: PhysX's joint friction creeps under sustained loads well below its set value (with the lever shut,
a 40 N lean creeps the lever open past 5 degrees within the second, and an 80 N lean collapses the shaft), so the
simulated test is scaled down to a load the simulator holds as a real closed clamp holds 150 N. While the test runs
the lever's pivot friction is raised to 5 N m, as a closed cam lever is over its centre and the collar's load does
not open it.

Doing nothing scores 0. A real trial is scored the same way: the length with a tape measure from the grip's top to the
tip (or the red line against the clamp's edge with a ruler), the lever's position from a photo, and the load test by
hand: stand the pole on its tip on a bathroom scale, hold it upright by the grip and press down until the scale reads
15.3 kg (150 N) for 1 s; the red line must move 2 mm or less against the clamp's edge.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=set_an_adjustable_walking_stick_to_its_mark.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Lengthen the walking pole until the red line on its shaft is at the edge of its upper clamp, lock it at that length, and let go of it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/walkpole-dream
```

Dream-1 scored 0.0 (2,400 steps): both grippers came down beside the pole near its clamp and hovered there; neither opened the lever or gripped the shaft, and the length stayed at 105 cm.

The scripted demo, which reaches full score (open loop from the known layout): the right gripper comes down over the
upper shaft 27 cm above the clamp and holds it, fingers closing across it, for the whole demo. The left gripper
pinches the closed lever's end across its width and lifts it along an arc round its pivot, flipping it open (about
65 degrees), lets go, grips the middle shaft 8 cm past the clamp and draws it out 17.5 cm (commanded 2.5 cm past the
mark, as the arm lags under the collar's drag), then lets go (the open collar's drag keeps the shaft where it is).
With its fingers shut it comes down behind the open lever, sweeps it over toward the grip 25 mm above the pivot, and
presses its end down flat onto the collar. Both hands let go and rise.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=set_an_adjustable_walking_stick_to_its_mark.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Lengthen the walking pole until the red line on its shaft is at the edge of its upper clamp, lock it at that length, and let go of it." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/walkpole-demo
```

The demo scores 1.0; it succeeds at step 757 (37.9 s). In its debug run the length reached 120.18 cm (120.07 cm once
released), the lever was pressed to 0.4 degrees past flat, and the 20 N load test slipped the shaft 0.5 mm and crept
the lever to 2.3 degrees.

Videos: add `--video` (top camera) to either command; the frames are under the log directory.

## Debug and the wrong way

- `WS_DEBUG=1` prints a `DBG` line every 5 steps (`WS_DEBUG_EVERY=n` changes it) (length, slide travel, lever angle, the collar's current friction,
  each stage, the load test, both grippers' TCPs and openings) and a line for each load test.
- `WS_ONE=1` runs the same left-arm motions with the right arm left at its ready pose (nobody holds the pole).
  The left gripper's lift on the lever drags the loose pole 2 cm across the table and the lever stays shut; its pull
  on the middle shaft then drags the whole pole 22 cm along the table while the length stays at 105.0 cm: score 0.
- `WS_ONE_PULL=1` lets the right hand hold the pole only while the lever is flipped open, then lets go: the left
  hand's pull on the middle shaft drags the whole pole 17 cm along the table (the open collar's 4 N drag is more than
  the table's grip on the pole) and the length grows by only 0.4 cm: score 0.

Other knobs (for checking the model, not for trials): `WS_F_OPEN`, `WS_F_CLOSED` (N), `WS_TEST_N` (N, the simulated
lean), `WS_VEL_ITERS` (the pole's solver velocity iterations, 32), and the demo's `WS_*` geometry knobs.

## Runtime per trial

The scripted demo ends at step 757 (37.9 s of simulated time); a debug run of it (320 x 176 cameras) takes about 4 to
5 minutes of wall time on an NVIDIA RTX PRO 6000 Blackwell shared with other runs, and the 1280 x 720 media run about
6 minutes. A Dream-1 run is 2,400 steps.

## Seeds

The layout is fixed (`object_jitter=0`); nothing in the task is random.

## Building it for real

- One adjustable trekking pole with flip-lock clamps, 100 to 140 cm, with a printed length scale on the middle shaft
  (for example Black Diamond Trail Back, about USD 50 a pair; any flip-lock pole of that size works), its strap and
  basket removed. Set its lower clamp at its usual setting and leave it.
- Draw a 3 mm red line round the middle shaft at the 120 cm line of its scale with a paint marker.
- Two strips of tape on the table: one at the grip's top (y -61.5) and one under the upper clamp (y -3.5 to -0.5), x 24.
- A bathroom scale for the load test.
- Reset: open the upper lever, slide the pole to 105 cm, close the lever and lay the pole on its tape marks, lever up.
  If the lever no longer holds the load test, turn its tension screw by a quarter turn as the maker says.
