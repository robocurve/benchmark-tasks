# Take an instant photo, then draw the print out of the held camera and lay it face down

A free instant camera stands on the bench aimed at an apple. A press on the shutter button on its top, through the
button's spring travel, takes the photo, and the print comes out when the button is let go. The print then stands
4.5 cm out of a slot 5 mm above the bench, too low for fingers to close across it: one hand has to pick the camera
up and turn the lens up, and the other comes in from the side, pinches the print across its faces and draws it out
against the rollers' drag. The print is laid face down and the camera stood back on the bench, as the maker says.
The work is one press through a spring and one pull out of a device that the other hand holds.

| | |
|---|---|
| Embodiment | `yam_memory` (bimanual I2RT YAM, Isaac Lab-Arena 0.3.1, the Dream station rig, 3 cameras), 20 Hz |
| Instruction | "Take a photo of the apple with the instant camera, leave the print in the slot for 5 seconds before you pull it out, lay it face down on the bench, and leave the camera standing on the bench." |
| Camera | a Polaroid Go Generation 2 as modelled: free, 105 x 84 x 62 mm (front to back, across, high), 259 g with its film pack, white; centre at (0.279, +0.055), lens toward +x; red shutter button on the top at the front right, on a sprung joint; the print slot at the bottom of the front face, 5 mm over the bench; underside friction 0.3 |
| Apple | fixed, 7 cm, at (0.429, +0.055): on the lens axis, 8.8 cm in front of the lens |
| Print | free, Go film size 66.6 x 53.9 mm (2.5 mm thick in the simulation), 5 g, inside the camera at the start, picture side up, black back down |
| Shutter | the button travels 2 mm against a spring (4 N pressed home); the first press home (9/10 of the travel) takes the photo. A good photo: camera level within 20 degrees, under 2 cm/s, apple within 12 degrees of the lens axis and 6 to 40 cm away; otherwise the exposure is wasted and the trial ends at 0 |
| Print out | when the button is let go (the maker's manual), the print comes 4.5 cm out of the slot in 1.5 s; 2.2 cm stay in the rollers, which hold it however the camera is turned until a gripper pinches it across its faces. Pulling it out drags the camera with up to 1.5 N; the unheld camera slides under 0.8 N |
| Milestones | photo taken (0.25) · the print drawn free of the slot (0.25) · the print face down on the bench, hands off (0.25) · the print stayed 5 s in the slot after it came out before it was drawn free, and the camera stands on the bench, hands off (0.25). The first two count once reached; the last two are end states, read at every step: they count only while they hold, and the last pays only on success |
| Success | the last milestone held 3 s: only then the full score |
| Ends early | `wasted_exposure` (first press home with the camera not aimed, level and still: 0) · `camera_knocked_over` (let go lying on the bench, tipped over 60 degrees) · `dropped_off_bench`; the score is then the events reached and the end states as they stand (a print off the bench is not face down) |
| Run limit | 1,200 steps (60 s at 20 Hz): the scripted run reaches its end state at 37.5 s and the full score, after the 3 s hold, at 40.45 s; it waits out each motion (the hands give under load, so it moves slowly and settles), and a policy that is less sure needs the rest |

## The camera, from the maker

The camera is the Polaroid Go Generation 2, modelled at its outer size without name or logo, and its film is Polaroid Go
film. From Polaroid's own pages (read at the addresses given):

- Product page (https://www.polaroid.com/en_us/products/go-polaroid-camera-generation-2): "Camera Dimensions"
  "105mm x 84mm x 62mm"; "Weight: 239 grams (8.43 ounces) (without film pack)"; colours "Black", "White", "Blue",
  "Purple".
- Film page (https://www.polaroid.com/en_us/products/color-go-instant-film): "Format: 66.6mm x 53.9mm"; "Image area:
  47mm x 46mm"; "Development time" "10-15 minutes".

From the Go's user manual (search result for Polaroid's manual of the Go and Go Generation 2):

- "Press the shutter button. The photo will eject from the slot at the front of the camera as soon as you let go of the
  shutter button. You will see that the photo will be covered with a film shield to protect it from light."
- "Leave your photo under the film shield for about 5 seconds. Gently lift the film shield so it rolls back into the
  camera. Remove the photo and place it face down to continue shielding it from light as it develops."

From reviews (T3, Tom's Guide, Amateur Photographer): the large red shutter button is on the top, the flash button
beside it.

Not stated by the maker, so our estimates: which outer size is which (105 mm front to back, 84 mm across, 62 mm high:
the 53.9 mm print leaves through the front); the button's place on the top (front right), its size (12 mm across),
travel (2 mm) and force (4 N pressed home); the slot's place and size (5 mm over the bench, 62 mm wide); the film
pack's mass (20 g); the print's thickness and mass; the rollers' drag (1.5 N) and the camera's underside friction
(0.3). A bench measures the button's force with a push gauge and its travel with a calliper's depth rod, the drag
with a spring scale clipped to a print as it is drawn out, and the friction with the same scale pulling the camera.

## What stands in for what

- **The camera** is one rigid body with the shutter button as a second link on a prismatic joint, held up by a spring:
  a press is read from the button's travel, as a camera's switch is, never from where a fingertip is.
- **The rollers** are two rules. Release: they hold the print at its place in the slot wherever the camera goes until
  a gripper pinches it across its faces (both fingers closed on it, the fingers' closing line within 30 degrees of the
  print's face normal, the hand within 4.5 cm of its centre). Drag: while a pinched print is pulled out, a force is
  pushed on the camera's body along its lens axis (1.5 N at 2 cm/s or faster, in proportion below), not felt by the
  print. The print is pushed out by moving it along the slot over 1.5 s.
- **The film shield** is not modelled. Its 5 s wait is scored from the time the print came out to the time it was
  drawn free of the rollers.
- **The print** is a rigid card 2.5 mm thick; a real print is thinner and bends. A modelling choice: the rigid card
  cannot buckle as a real print would when it is squeezed by its two side edges, so the rollers release it only to a
  pinch across its faces. The picture's development (10 to 15 minutes) is not simulated and not scored; the picture
  area stays dark.
- **The apple** is fixed to the bench (on putty in the real trial). The flash is not modelled.

## States set directly

The print's pose while it is in the slot: inside the camera until the button is let go after the photo, then moved
4.5 cm out along the slot over 1.5 s, then held at the slot, following the camera, until a gripper pinches it across
its faces. While it is held so, a force equal to its weight (0.05 N) carries it, so that it does not drop between two
control steps. From then on it moves only by physics. Nothing else is set; the camera, the button and the hands move
by physics alone.

## Scoring on a real bench

| Score | Asked | Read on the real bench |
|---|---|---|
| 0.25 | photo taken: at the first press home the camera level, still and aimed at the apple | the trial video: the button pressed home, the camera's tilt and motion, the apple in line; the developed print (10 to 15 minutes later) shows whether the apple is in the shot |
| 0.5 | the print drawn free of the slot (it comes 4.5 cm out by itself after the press; 2.2 cm stay in the rollers) | the video |
| 0.75 | at the end, the print face down on the bench, still, hands at least 4 cm away | the final photo |
| 1.0 | the print stood 5 s in the slot after it came out before it was drawn free; at the end the camera standing on the bench, still, hands off, held 3 s | the video (the two moments, and the last 3 s) and the final photo |

A first press home that misses the shot ends the trial at 0, as a wasted exposure does on a real camera. Face down
and the camera standing are read at the end: a print knocked off the bench or turned face up afterwards no longer
counts.

## Ways round the task, and what they score

- Pressing the button while the camera is carried, tilted or turned away: the exposure is wasted, 0.
- Pinching the print by its side edges from above while the camera stands on the bench: the rollers keep it (the
  modelling choice above: the rigid card cannot buckle as a real print would); the print cannot be drawn out that way.
- Drawing the print out at once: it stops at 0.75 (the 5 s wait is part of the full score).
- Pulling the print while the camera stands unheld: the camera follows the print (1.5 N of drag against 0.8 N of
  friction). In any case the print cannot be pinched across its faces while the camera stands on the bench.
- Dropping the print or leaving it face up: face up scores 0.5; a print off the bench ends the trial.

## Can it be seen

`start_top.png` is the model run's first top-camera picture at the policy's own size (320 x 176): the white camera
with its red button and the apple are clear; the print is inside the camera, as in the real start. `end_top.png` is
the scripted run's last top-camera frame scaled to that size: the print lies face down, a black patch (its black
back) beside the apple; the camera stands; both arms are at rest. A print left face up would show its white face
around the dark picture area instead.

## Setting up and resetting the real bench

- The camera: a Polaroid Go Generation 2 loaded with Polaroid Go film, standing on its marks (centre at x 27.9, y 5.5
  cm, within 0.5 cm), lens toward the apple.
- The apple: a red artificial (decorative) apple about 7 cm across on a ring of reusable putty, on the lens axis at
  x 42.9, y 5.5 (within 0.5 cm, on marks). It is only what the photo is of and is never touched.
- Each trial exposes one new sheet of film, and its print is never reused: a pack of 8 serves 8 trials. Reload: open
  the film door, take out the empty pack, put in a new one and close the door (the top sheet comes out as the
  darkslide).
- Between trials: take the last print off the bench, stand the camera on its marks, check the apple. About 1 minute;
  3 minutes with a new pack.

## Repeating the real trial

A double pack of Go film (16 sheets) gives 16 trials. Each trial is filmed from the front of the bench; the two moments
of the shield's wait (the print out, the print drawn free) are read from that video's time stamps, the end state from
the last frame. Nothing is weighed or measured during the trial.

## Design notes

- **Why the shot is taken with the camera standing on the bench.** The run presses the button while the camera
  stands aimed on the bench, and picks it up only afterwards, for two measured reasons. Over the button (31 cm out,
  3 cm past the centre line on the other arm's side) a hand pointing straight down reaches 12 cm over the bench
  (inverse kinematics within 0.1 mm), not 14 cm (6.8 mm short) or 16 cm (22.7 mm short, a wrist joint at its limit);
  pressing the button of the standing camera already puts that point 8 cm over the bench, so there is little room to
  lift the camera for the shot. And with the other hand holding the camera's rear part from straight above, the two
  hands' bodies would overlap by 6 mm (the arms' hand boxes); a holding hand leaned back 20 degrees clears by 1 to 2
  cm, but then cannot turn the camera lens up (its hand would have to point 20 degrees up: 95 mm short). The task does
  not ask for either order; a policy may hold the camera for the shot if its hands allow it.
- **Why the camera has to be picked up and held.** After the shot the print sticks out of the front of the camera,
  flat, 5 mm above the bench: too low for fingers to close across it (a finger is about 1 cm thick). So one hand lifts
  the camera and turns the lens up, which stands the print on end, where the other hand pinches it from the side.
  Second, while the print is drawn out, the rollers' drag (1.5 N, assumed) is more than the friction that keeps the
  259 g camera in place (about 0.8 N): an unheld camera would follow the print.
- **The hands' figures.** The press is from above, the fingertips 6.5 cm over the bench. The camera is held across
  its width (84 mm; the gripper opens to 94 mm) on its rear part, from above, the fingers pointing 15 degrees back
  towards the arms: turned lens up, the hand then points 15 degrees down, which the arm reaches (pointing level or
  up, it does not). The print is pinched from the side, the hand level, 19.5 cm over the bench, and drawn up to
  25.5 cm: there a hand pointing straight down is already 19 mm short at 15 cm, a level hand reaches these points
  (inverse kinematics within 0.1 mm, no joint at its limit, and the run below). The print is stood on its end with
  the hand level, about 6 cm over the bench.
- **Forces, read from the scripted run.** The press: the hand stopped 17.7 mm short of its command while the button
  was home, which at the arm's 3.2 mm per newton there is about 5.6 N. The camera: each finger stopped 6.0 mm short of
  its command, about 30 N per pad at the simulated gripper's 5,000 N/m (its limit 50 N; the real fingers give about
  40 N). The print: each finger 1.1 mm short, about 5.5 N per pad. Nothing twists in a pinch beyond the print's 5 g.
- **One exposure.** A press home with the camera off the apple, tilted or moving wastes it and ends the trial.
- **Face down.** The print is stood on its lower end leaning 20 degrees towards its picture side and let go; it falls
  face down.

## Nearest tasks elsewhere

- On this board: a photo taken with a single-use camera ends at the press; here the work is drawing the print out of
  the camera the other hand holds, and laying it as the maker says.
- DexJoCo's "Bimanual Photograph": two hands hold a camera before a logo and press its shutter; it ends at the press,
  with no print.
- RoboCasa and LIBERO press buttons on fixed appliances; none takes something out of a device held in the other hand.

## Models

`code/tasks/h07_instant_photo/assets/build.py` writes `camera.usda`, `print.usda` and `layout.json` (every dimension the
task file reads); the camera and print geometry and their plain colours are ours (no textures). The apple is
`apple_02` from Isaac Lab-Arena's object library, which loads it from the Objaverse folder of NVIDIA's RoboLab asset
set: an Objaverse model, listed in RoboLab's third-party notices
(https://github.com/NVlabs/RoboLab/blob/main/THIRD_PARTY_NOTICES.md, the objaverse folder) as "CC BY-NC-SA 4.0 | ©
2026 NVIDIA Corporation": research use and redistribution with attribution under the same licence, not commercial
use. Isaac Lab-Arena's code that loads it is Apache-2.0; the original Objaverse author is not stated there.

## Rule tests

`code/tasks/h07_instant_photo/tests/test_rules.py`: 22 tests that set states directly and read them with the task
file's own functions and the embodiment's own per-step scoring (`rta.spec`), no simulator: one start for every seed;
face down read from the print body's own rotation (its picture is the model's top face); a press short of home takes
no photo; the first press home takes it; a first press home tilted, moving or turned away wastes the exposure and no
later press counts; the print comes out only when the button is let go, over 1.5 s, 4.5 cm out with 2.2 cm in the
rollers; the rollers hold an unpinched print wherever the camera goes; a pinch by the edges leaves the print in the
rollers, a pinch across its faces frees it once it is out; the drag on the camera (1.5 N, half at 1 cm/s, none when
pushed in or unpinched), the print's weight carried only while the rollers hold it, and nothing left pushing once it
is free; the drag is above the friction; the full-score end; drawn out before 5 s stops at 0.75; face up, or tilted
past 15 degrees, is not face down; a hand within 4 cm of the print or the camera; a camera left tipped, or lying on
its back; off the bench; the photo and the drawn print stay counted, the end states do not; the end held 1 s and then
the camera knocked over never gives 1.0; a print turned face up after 0.75 gives 0.5; 1.0 only after the 3 s hold.
Run from `code/`: `python -m pytest -q tasks/h07_instant_photo/tests` (22 passed).

## Reproduce

Isaac Lab-Arena 0.3.1 (Isaac Sim 6.1.0) with the `yam-arena` embodiment package and Inspect Robots; `code/Dockerfile`
builds the exact environment. Install this task's code into that environment and run:

```bash
uv pip install --python $ARENA_DIR/.venv/bin/python --no-deps -e code
python3 code/tasks/h07_instant_photo/assets/build.py   # rebuild the models and layout.json (plain Python)
inspect-robots run --task rta/instant_photo -T num_scenes=1 --embodiment yam_memory -E control_hz=20 --policy rta_oracle --store-frames --log-dir out/demo   # the scripted full-score run
inspect-robots run --task rta/instant_photo -T num_scenes=1 --embodiment yam_memory -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy dream --store-frames --log-dir out/dream   # Dream-1
```

Scores per trial: `q_score` (0.25 each for the photo and the drawn print once reached and for the print face down
while it holds; 1.0 on success), `rta_success` and one `milestone/<name>` column per milestone (an event reached, an
end state holding at the last step), all from simulator state. `-E trace_dir=<dir>` writes a per-step event log
(object poses, the shutter's travel, the times of the photo, the let-go, the print out and drawn free). One fixed
layout: every scene and every seed is the same start, so `--seed 0` under the harness (trial seed 1415158438) gives
this same start. Runtime per trial: at most 60 s of simulated time (the scripted trial stops at 40.45 s); 7 to 8
minutes of wall time on an RTX 4090 with three 640 x 360 cameras, including the simulator's 2-minute start.

## Reference runs

From the task file in this folder, 20 Hz, scene `seed-0` unless named.

- **The scripted run** (`rta_oracle`, the demo; three 640 x 360 cameras): `q_score` 1.0, `rta_success` 1, 809 steps
  (40.45 s). The button pressed home at 4.60 s with the camera standing on the bench, still and aimed: the photo
  (0.25); let go at 5.95 s; the print out at 7.45 s; pinched across its faces and drawn free of the rollers at
  27.50 s, 20.05 s after it came out (the 0.5 counts at 27.55 s, the print then 3 mm clear of the slot); face down on
  the bench, both hands at least 4 cm away, at 36.55 s (0.75; its picture side's normal 179 degrees from up, read from
  the print's own pose); the camera standing (tilt 0.0 degrees) and both hands off from 37.50 s, the score staying
  0.75 until that end had held 3 s: 1.0 at 40.45 s, which ends the trial. Its first top-camera frame is the setup
  picture; its last frame shows the camera standing, the print face down and both arms at rest. Under scenes `seed-0`,
  `seed-1` and `seed-2` (the one start under three seeds; one run of three trials) the same plan scored 1.0 in each,
  at 40.45 s; the print drawn free of the rollers at 27.50 s in each (the 0.5 counting at 27.55 or 27.60 s) and face
  down at 36.55 s, its picture side's normal 179 to 180 degrees from up, at places within 2.3 mm of each other.
- **Dream-1** (`dream`, three 320 x 176 cameras, the instruction above, over the whole run limit): `q_score` 0 after
  all 1,200 steps (60.0 s), no failure. Both hands moved about 10 cm forward on their own sides of the bench (the right
  also 8 cm towards the middle), 12 to 27 cm over it, and closed their grippers on nothing at 11 to 13 s; the nearest
  came 29 cm from the camera's centre. The button was never pressed (0.01 mm of travel) and the camera did not move.
