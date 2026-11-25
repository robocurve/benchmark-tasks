# Slate a take: hold a clapperboard square to the phone camera and clap its stick shut once, sharply

Slating a take is the moment everyone knows from film sets and behind-the-scenes clips: someone holds a clapperboard
up in front of the camera and claps its stick shut. On the station table a phone on a small tabletop tripod, at the
far side of the table, films toward the robot. A film clapperboard (a 30 x 25 cm acrylic slate with a hinged
clapstick, like the Neewer acrylic clapboard) leans back on a small hard equipment case in front of it, its face
toward the phone. The instruction asks for the take to be slated: the clapperboard held facing the phone camera,
fully in its frame, and its stick clapped shut once, sharply. It does not say how.

What makes it hard is a hand skill a person also uses two hands for, and it is visible in the scene. The 5 mm acrylic
board does not stand by itself: it leans on the case, its face 28 deg off the camera's axis, and if a hand lets go of
it, it falls back. So one hand must hold the board upright, square to the lens and still, while the other opens the
stick and drives it shut. The clap has to be fast (the sticks must meet within a frame or so, or the sync frame is
smeared) and the board must not jerk with it (a slate that moves with the clap blurs the frame where the sticks meet).
The hinge has a tension screw, as on most slates, so the stick stays where it is left: letting go of it does not clap
it, the hand must drive it shut. A two-finger gripper also has to take the stick by its very end, with only the
fingertips, so that its fingers stay clear of the lower stick (a finger there would be caught between the sticks and
block the clap).

The task is the task file [`slate_a_take_with_a_clapperboard.py`](slate_a_take_with_a_clapperboard.py) for the
`yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Why it matters

The clapperboard is the start of every take on film and video sets, and a familiar sight to anyone who has watched a
behind-the-scenes clip; with phone and tabletop filming (recipe videos, product shots, interviews) it is used far
beyond professional sets. Its purpose is synchronisation: the frame where the sticks meet is lined up with the sharp
peak of the clap in the separately recorded sound. A camera assistant's guide to slating ([ProVideo Coalition, "The
secret art of the slate: 25 tips to help you slate like a pro"][pvc]) describes exactly the hand skill the task tests:
"Proper slating technique dictates holding the slate with one hand and closing the clapper with the other"; "THE
SLATE MUST NOT MOVE WHILE THE CLAPPER IS IN MOTION"; "If both the slate and the clapper show motion blur it becomes
very hard to figure out when the clapper stops moving"; "Do not let the clapper bounce." The skill a robot needs is the
same: a steady hold with one hand while the other makes one fast strike with a light hinged part, in view of a camera.

The clapperboard is modelled on the Neewer acrylic director's clapboard ([product page][neewer]: 30 x 25 cm, a
30 x 20 cm writing area, 520 g, acrylic board and clapsticks).

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Isaac Lab-Arena / Isaac Sim, on Linux
with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable, installs everything, including the Dream-1
policy. The task needs no models from Arena's object library and uses no third-party assets: the slate, the case and
the phone on its tripod are built in these files from measurements.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or put one on a hinge. The
slate (a board with its lower clapstick, and the upper clapstick on a hinge with friction and a stop) is built with
`bodykit.py`, which adds such bodies and joints: importing it hooks `yam_arena.scene` when the simulator starts,
without changing yam_arena, and gives the task file each body's pose and the hinge's angle. This copy of `bodykit.py`
also lets a hinged body start turned (`init_quat`), here the slate leaning on the case.

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, z up from the table top.

- Clapperboard, loose: a black acrylic writing board 30.0 x 20.0 cm, 5 mm thick, printed with white lines (PROD,
  ROLL / SCENE / TAKE, DIRECTOR / CAMERA / DATE); a fixed lower clapstick along its top edge (30.0 x 2.5 cm, 8 mm thick,
  white with black diagonal stripes on both faces); the upper clapstick on it (the same size), on a hinge pin 1 cm in from the board's
  left end (as the robot sees it; on the phone's picture it is on the right), so its free end is the robot's right end.
  30 x 25 cm overall; 0.45 kg board and lower stick, 0.07 kg upper stick (0.52 kg in all). The upper stick turns about
  the hinge pin (normal to the board's face) from shut (resting on the lower stick) up to 45 deg open, against 0.12 N m
  of friction (the hinge's tension screw: the stick, 0.10 N m of its own weight at most, stays where it is left). The
  slate stands on its bottom edge, the middle of that edge at x 27.0, y 2.0, its face toward +x (the phone), leaning
  back 28 deg with its back on the case's far top edge.
- Hard plastic equipment case, loose: 20 (x) x 16 (y) x 15 cm tall, 2.0 kg, x -1 to 19, y -6 to 10.
- Phone on a tabletop tripod, fixed: the phone landscape, its camera lens at x 70, y 2, 13 cm above the table,
  looking straight back along -x (toward the robot); 16:9 video, 69 x 43 deg field of view (a 26 mm-equivalent main
  camera). A strip of tape on the table (x 38 to 58, y 2) marks the camera's axis.
- Both arms start at the ready pose, grippers open. The layout is the same in every trial.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. The task
file reads the hinge's angle and the slate's pose at every physics step (240 Hz):

- A clap: the stick shuts (within 1 deg of closed) after being open 20 deg or more. Any later shut after the stick was
  opened 10 deg or more is another clap. The first clap is the take's sync mark and is the one judged.
- 0: no clap in the run; or, at the first clap, the slate's face more than 20 deg off the camera's axis (the line from
  the slate's middle to the lens), or a corner of the slate outside the camera's field of view.
- 0.5: the first clap faces the camera and is in frame, but it was slow (it took longer than 50 ms from 10 deg open
  to shut), or the slate moved 1 cm or more in the 0.5 s around it (its middle, over the 0.25 s before and after the
  clap), or there was more than one clap in the run.
- 1.0 (success, held 1 s): one clap only, facing the camera and in frame, from 10 deg to shut within 50 ms, the slate
  still within 1 cm.

Why 50 ms: a film frame at 24 fps lasts 41.7 ms. A stick that goes from 10 deg open to shut within about one frame
gives a clean sync frame (the frame before shows the sticks clearly apart, the next shows them shut); a slower close
spreads the meeting over several frames. The YAM's scripted snap reaches it (the demo closes from 10 deg in
45.8 ms), so the threshold is one the robot can meet.

Doing nothing scores 0. For real trials, record the full slate with the phone at 240 captured frames per second and
use the overhead video to check facing. Step through the original captured frames. Mark frame i as the last frame
with the upper stick at least 10 deg open before the first clap (about a 5.0 cm gap at the free end). Mark frame j as
the first subsequent frame with the upper stick within 1 deg of shut (about a 5 mm gap at the free end). If the
stick passes the 10 deg mark more than once, use the last passage into the closing stroke. Closing time is
(j - i) / 240 seconds. For example, frames 100 through 112 span 12 intervals, or 50 ms, and pass. Use the capture
rate when stepping through slow-motion exports, which may play back at a different rate.

Read slate movement against the tape and the board's printed lines over the 0.25 s before and after the clap.
Read facing from overhead against the tape on the camera axis (the board's top edge must be within 20 deg of square
to it). Check that the whole slate is visible in the phone's frame at the clap. Count claps in the video.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=slate_a_take_with_a_clapperboard.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Slate the take: hold the clapperboard facing the phone camera, fully in its frame, and clap its stick shut once, sharply." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/slate-dream
```

The scripted demo, which reaches full score (open loop from the known layout; it ends on success after about 535
steps). The left hand, pointing across the table with its fingers closing across the board, comes down along the
leaning slate beside its left edge, pinches the board 3.5 cm in from that edge and 10 cm up, and turns it upright about
its bottom edge (aimed 1.5 cm down along the board, so the edge stays pressed on the table and does not skid, and 3 deg
past upright); it holds it there to the end. The right hand comes down over the clapstick's free end, pointing along
the stick, and pinches only its last 2 cm with its fingertips (the fingertips are 27 mm wide, the fingers behind them
40 mm, so only the tips close on the stick and nothing touches the lower stick). It opens the stick 25 deg along its
arc and lets go (the hinge holds it open), rises 1 cm, closes its fingers, sets the fingertips on the stick's top edge
4 cm in from its end and strikes it shut: two waypoints 0.05 s apart along the arc, aimed 8 deg past shut and 6 cm in
toward the hinge (a fast move of the arm first swings out across the stick's line). The stick goes from 10 deg open to
shut in 45.8 ms (peak 224 deg/s), the slate's middle moves 2.6 mm in the 0.5 s around the clap, its face 4.7 deg off the
camera's axis; it rests on the shut stick 1 s and lifts off.

Striking the open stick shut matters. Snapping it shut while still pinching its end (`SLATE_STRIKE=0`) drags the
light slate with the arm's swing: in our runs it closed in 46 to 54 ms but moved the slate 6 to 26 mm, which scores 0.5.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=slate_a_take_with_a_clapperboard.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Slate the take: hold the clapperboard facing the phone camera, fully in its frame, and clap its stick shut once, sharply." \
  --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/slate-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/slate-dream/*.json; do inspect-robots video "$log"; done
```

`SLATE_DEBUG=1` prints, every 5 steps (`SLATE_DEBUG_EVERY=1` for every step), the stick's opening angle and its peak,
the slate's middle, its face angle off the camera's axis and whether it is in frame, how far the right fingers are
from the lower and the upper stick, the score and its reason, every clap (time, closing time, peak closing speed,
facing, in frame, how far the slate moved) and where each gripper is; after a clap, the slate's middle through the
0.5 s around it.

`SLATE_ONE=1` runs the demo the wrong way, with one arm: the left hand stays at the ready pose and the right hand opens
and strikes the stick on the slate where it leans on the case. Nothing holds the slate: the stick swinging open and
the strike knock it back until it lies on the case (its middle moved 9.7 cm), and the clap comes with the slate's face
46 deg off the camera's axis. Score 0.

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell shared with other simulation jobs, debug runs of the demo without video (about
535 steps, 26.7 s of simulated time, ending on success) took 5 to 8 minutes, with two or three runs at once. With video, the scripted demo at 1280 x 720 took about 3.5 minutes (534 steps, ending on
success) and a Dream-1 trial of 2,400 steps at 320 x 176 about 6 minutes, the two running at the same time. Isaac Sim's
first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A 30 x 25 cm acrylic clapperboard (Neewer acrylic director's clapboard, about 15 USD), its hinge screw tightened so
the stick stays open at any angle; a small hard equipment case (or any box) about 20 x 16 x 15 cm, 2 kg; a phone on a
tabletop tripod with its lens 13 cm above the table, 43 cm in front of the slate's face, filming in 16:9 at 240 fps;
a strip of tape on the table along the camera's axis. For each trial lean the slate, stick shut, on the case as in the
screenshot (its bottom edge 27 cm in front of the arm bases, leaning back 28 deg). Start the phone in 240 fps capture
mode and start the overhead recording before the trial.

[pvc]: https://www.provideocoalition.com/the-secret-art-of-the-slate-25-tips-to-help-you-slate-like-a-pro/
[neewer]: https://neewer.com/products/neewer-acrylic-plastic-directors-film-clapboard-cut-66601326
