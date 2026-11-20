# Take one photo with a single-use film camera: wind its thumbwheel to the stop, then press the shutter

A single-use 35 mm film camera (a Kodak FunSaver, 27 exposures: 120 x 59 x 35 mm, 117 g) stands on the table, lens
away from the robot. Its last frame has been taken, so its thumbwheel is unwound and its shutter is locked. The
instruction ("Wind this single-use film camera on to the next frame and fire its shutter once, then leave it standing
on the table.") asks for one photo and the camera left standing on the table. Dream-1's service refused the plainer
wording "Take one photo with this single-use film camera, then leave it standing on the table." ("instruction invades
privacy ... Taking a photo with a camera explores beyond the tabletop task"), so the instruction names the camera's
own two actions instead.

Taking a snapshot with a disposable camera at a wedding, a party or on holiday is something most people have done or
watched. The camera tells you how: its back sticker gives the steps, which "boil down to frame advance and pressing
the shutter button" ([Digital Camera World review of the FunSaver][dcw]); "The film winder sits at the top right of
the rear panel"; "The film advance wheel is interlocked with the shutter mechanism, to avoid the risk of
double-exposures"; "a frame counter that counts down from 27 to zero". The product description: "advance it with the
thumbwheel until it stops turning. Then look through the viewfinder and click a picture" ([product listing][listing]).

## The hand skill, and why two hands

A person holds the camera in one hand and rolls the thumbwheel with the thumb of the same hand, in several strokes.
A gripper cannot hold and roll at once, and the camera is too light to stay put on its own:

- The thumbwheel is a ridged 24 mm wheel that stands only 3 mm proud of the camera's back, at its top right corner.
  It turns only when something rubs its edge sideways while pressing on it: a fingertip pressed onto the 3 mm edge and
  drawn across it, lifted, brought back and drawn again. One frame takes 1.5 turns of the wheel (our estimate), so
  many strokes (ten in the scripted demo), each of which must stay on the edge without slipping off it or catching the camera's back.
- Every stroke pushes the 117 g camera forward and sideways. Standing loose on the table, it slides away from the
  finger and the wheel barely turns (see the one-arm wrong way below). So one hand holds the camera while the other
  strokes the wheel.
- The shutter button is locked until the wheel has reached its stop: pressing it early does nothing, and a wheel that
  is not wound all the way leaves the shutter locked. After the photo the camera goes back down, standing, let go.

The task is the task file [`take_a_photo_with_a_single_use_camera.py`](take_a_photo_with_a_single_use_camera.py) for
the `yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac
Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable,
installs everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no
third-party assets: the camera is built in these files from boxes and cylinders.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or put one on a joint. The
camera (a loose body with the thumbwheel and the shutter button on revolute joints) is built with `bodykit.py`, which
adds such bodies and joints: importing it hooks `yam_arena.scene` when the simulator starts, without changing
yam_arena, and gives the task file the camera's pose and each joint's angle. The task file sets the two joints'
springs (stiffness and target) every control step through Isaac Lab's articulation API: that is how the wheel's
anti-reverse pawl and stop and the shutter's lock are modelled (below).

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, z up from the table top.

- Kodak FunSaver single-use camera (yellow and black plastic, 117 g), loose, standing on its base, lens toward +x,
  back toward the robot. Body 12 cm along y, 5.9 cm tall, 3.0 cm deep plus a 0.5 cm lens barrel in front; its centre
  at x 22.5, y 2.0; its back at x 21.0. Seen from the back, its right end is at y -4.0.
- Thumbwheel: black, ridged (36 ridges), 2.4 cm across and 0.7 cm thick, its axis vertical at x 21.9, y -2.4, its top
  0.15 cm under the camera's top, in a recess at the top-back-right corner (2.0 cm deep, 4.2 cm wide, 1.0 cm tall);
  it stands 0.3 cm proud of the back. It winds clockwise seen from above (its back edge moving toward +y).
- Shutter button: a black 0.9 x 0.9 cm cap 0.15 cm proud of the top, centred at x 22.9, y 1.2.
- Flash off. Both arms start at the ready pose, grippers open. The layout is the same in every trial.

What is modelled (forces and turns are estimates):

- The wheel turns against 0.01 N m of friction; an anti-reverse pawl stops it turning back by more than 1 deg (a
  stiff spring, 2 N m/rad, toward the furthest point reached); at 540 deg (1.5 turns: one frame) it stops (the same
  spring) and stays stopped until the shutter fires, which frees it for the next 540 deg.
- The shutter button is a cap on a lever (pivot 3 cm away inside the body). Once the wheel is at its stop (within
  2 deg), its spring needs about 2.5 N to start moving and 3.5 N at 1.0 mm; 1.0 mm down fires the shutter (a photo is
  taken, the counter moves on, the shutter locks again until the next full wind). Before that it is locked (a spring
  that would need about 100 N to push it 1 mm).

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the camera's pose, the wheel's and the button's joint angles, and the points of each
finger's collision hulls.

- 0.4: the wheel has been wound to its stop (within 2 deg of 540 deg) at some time.
- 0.6: the shutter has fired after winding (a photo taken).
- 1.0 (success, held 1 s): exactly one photo taken, and the camera standing on its base on the table (tilted under
  10 deg, its lowest point within 3 mm of the table), every finger point 1 cm or more from it, and it still (moving
  less than 1 mm in 1 s).

Doing nothing scores 0. Pressing the shutter before winding does nothing (0). A second photo (wound again and fired
again) keeps the score at 0.6. Nothing else is penalised: a camera that slides or falls over can be stood up again.

A real trial is scored from the trial video (with sound) and one check after it: a photo taken is the shutter's
click on the sound track (and the frame counter one number lower); two clicks are two photos. If there was no click,
the operator presses the shutter after the trial with the lens covered: a click then means the robot wound to the
stop without firing (0.4); no click means the wheel was not wound to its stop (0). Standing, let go and still at the
end, from the video.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=take_a_photo_with_a_single_use_camera.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Wind this single-use film camera on to the next frame and fire its shutter once, then leave it standing on the table." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/funsaver-dream
```

Dream-1 run (2,400 steps): 0.0. The left gripper took hold of the camera and moved it about, tilting it, and the
right gripper came near it; the thumbwheel never reached its stop and the shutter never fired.

The scripted demo (`--policy yam_scripted`, same arguments, `--max-steps 1700`), open loop from the known layout:
the left gripper takes the camera's left end from above (fingers across its 3 cm depth) and lifts it 6 cm. The right
gripper, pointing forward and 30 deg down with its fingers closed, sets its fingertips on the wheel's back edge,
draws them 2.4 cm toward +y, lifts off 8 mm, comes back and strokes again, 13 times (each stroke turns the wheel
about 50-65 deg; it reached its stop at step 896, in the tenth stroke; the last strokes rub the stopped wheel). The
left hand sets the camera down and lets go; the right hand turns straight down and presses the shutter button (the
shutter fired at step 1,301), then leaves. Success (held 1 s) at step 1,371. Videos: `inspect-robots video <log>.json`.

`FUNSAVER_DEBUG=1` prints a `DBG` line every 5 control steps (wheel angle, pawl, button angle, photos, the camera's
pose, finger clearance, both TCPs). Wrong ways:

- `FUNSAVER_MODE=one`: one arm. The right hand strokes the wheel of the camera standing loose on the table, the same
  strokes as in the demo. The first stroke shoves the camera 9 mm sideways and 5 mm forward instead of turning the
  wheel (it turns 7 deg), and the later strokes no longer reach it: score 0. (The wheel's friction needs about 0.8 N
  at its edge; the camera slides at about 0.9 N, and the stroke's push toward the camera adds to the sideways pull.)
  One arm could still find a way the scene allows, such as laying the camera on its front and stroking the wheel
  from above so the push presses it onto the table; that would score like any other way.
- `FUNSAVER_MODE=press`: the shutter pressed without winding first, with the same press as the demo: the button does
  not move (locked), no photo: score 0.

## Runtime per trial

The scripted demo succeeds at step 1,371 (69 s at 20 Hz; run limit 2,400 steps). On one NVIDIA RTX PRO 6000
Blackwell, shared by two runs, a 1,800-step debug run at 320 x 176 took about 5 minutes including about 1 minute of
start-up, and the demo recorded at 1280 x 720 about 6 minutes.

## Seeds

The layout is fixed (`object_jitter=0`); the demo was run with the default seed.

## Building it for real

A Kodak FunSaver (27 exposures, ISO 800, flash) costs about US$15-20 and gives 27 trials. Before each trial its last
frame has been fired (the wheel unwound, the shutter locked); stand it on the marked spot (tape outline of its base)
with its lens away from the robot and the flash off. Each trial uses one frame. Reset: if the wheel was left partly
wound, wind it to the stop; press the shutter with the lens covered (the post-trial check above) so it starts
unwound; stand it on its spot. When the counter reaches 0, take a new camera.

[dcw]: https://www.digitalcameraworld.com/reviews/kodak-funsaver-single-use-camera-review
[listing]: https://itsmanual.com/?p=72316
