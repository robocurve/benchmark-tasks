# Set a wristwatch to 10:10: crown out to the time click, turned in strokes, pushed back in, the date left as it was

An ordinary quartz three-hand wristwatch with a date window (38 mm steel case, like a Timex Easy Reader Date 38 mm) is
buckled onto a single-watch display stand, an oval C-ring on a post, taped to the table. Its hands read 9:46. The
instruction asks for it to be set to 10:10 without changing its date, with its crown left pushed fully in.

Setting an analogue watch is something most people do by hand: twice a year for daylight saving time, after a battery
change, or when it has drifted. Watch shops set the watches on display to 10:10. Timex's guide, "How to Set the Time
on Any Watch" ([Timex][timex]), says the crown is "pulled out as far as it will go" (some watches have "two or three
'stops.'"), turned to set the hands, and then to "push the crown back to its starting position": "Make sure the crown
is pushed all the way back in when you're done." On a watch with a date the first stop sets the date and the last
stop sets the time (retailers' setting notes for the Easy Reader Date: the date with the crown "halfway out", the time
"all the way out").

## The hand skill

- The crown is 5.5 mm across and stands only 4 mm out from the case. Fingertips can take it only by its end, their ends
  within about half a millimetre of the case (closer and they push on the case; further and they hold too little of
  the crown).
- It is drawn out past two clicks 0.9 mm apart. The first click is the date position: turning the crown there changes
  the date, which the task forbids.
- Setting 9:46 to 10:10 takes two full turns of the crown (12 minutes of the minute hand per turn). A wrist does not
  turn that far, so the crown is turned in strokes: grip, turn, let go, turn the open hand back, grip again, without
  pulling or pushing the crown off its click on the way. The turning must stop within 30 degrees of the crown (one
  minute of the minute hand).
- The crown is then pushed back in past both clicks, all the way home.

One arm is enough: the stand holds the watch, as a hand or a wrist would. (The other arm stays at its ready pose.)

The task is the task file [`set_a_wristwatch_to_ten_past_ten.py`](set_a_wristwatch_to_ten_past_ten.py) for the
`yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac
Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable,
installs everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no
third-party assets: the watch and the stand are built in these files from boxes and cylinders.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or put one on a joint. The watch
(a fixed base with the crown's stem on a slide joint, the crown on a revolute joint on the stem, and the hour and
minute hands on revolute joints) and the stand are built with `bodykit.py`, which adds such bodies and joints:
importing it hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, and gives the task file each
body's pose and each joint's position. This copy of `bodykit.py` adds slide (prismatic) links to its linkages, for the
stem. Every control step the task file sets the stem's spring target (its detents) and the hands' drive targets (the
gear train) through Isaac Lab's articulation API, as described below.

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, z up from the table top.

- Single-watch display stand, fixed (taped down): a base plate 9 x 7 x 0.6 cm centred at x 28, y 0; a post 1.2 cm
  across; an oval C-ring 6.4 cm wide (along x) and 5.0 cm tall outside, 2.6 cm deep (along y), 3 mm thick, open at the
  bottom over the post; the ring's top 13.5 cm above the table.
- Wristwatch, about 40 g, buckled onto the ring, dial up, centred over x 28, y 0, 12 o'clock toward +x and the crown
  (3 o'clock) toward -y: steel case 38 mm across and 9.5 mm thick, its back on the ring's top; white dial with hour
  marks, a date window at 3 o'clock, black hour and minute hands, a red seconds hand; 20 mm brown leather strap. The
  battery is out, so the hands move only when the crown turns them. The crown: 5.5 mm across, 4 mm long, its axis
  14.0 cm above the table and 4.7 mm above the case back, centred at x 28, y -2.15 when pushed in (its inner face
  0.5 mm off the case).
- The hands read 9:46 and the crown is pushed in. Both arms start at the ready pose, grippers open. The layout is the
  same in every trial.

What is modelled:

- The crown's stem slides along the crown's axis between home and the 2nd click, 1.8 mm out, where it stops. A spring
  pulls it toward its nearest click (home, the 1st click 0.9 mm out, or the 2nd click 1.8 mm out), so it takes up to
  1.5 N, half way between two clicks, to move it from one click to the next, and it stays at a click when let go.
- The crown turns about its axis against 1.5 mN m of friction.
- At the 2nd click (time) the crown drives the hands: one turn of the crown moves the minute hand 12 minutes, forward
  when the crown's top turns toward 12 o'clock, backward the other way, and the hour hand with it (1/12 as fast).
- At the 1st click (date), half a turn of the crown (either way, in total, while it stays at that click) changes the
  date.
- Home, the crown turns freely and moves nothing (as on a quartz watch).

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the stem's position, the crown's turn and the points of each finger's collision
hulls. "The hands read" is the time they show on the 12-hour dial.

- 0.25: the crown has been out at its 2nd click (the time position) at some time.
- 0.5: the hands read within 5 minutes of 10:10.
- 0.75: the hands read within 1 minute of 10:10 and the date has not been changed.
- 1.0 (success, held 1 s): as 0.75, with the crown pushed fully home (nearer home than the 1st click) and every finger
  point 1 cm or more from the watch head.

Once the date has been changed (half a turn of the crown at the 1st click), the score is at most 0.5 for the rest of
the run. Doing nothing scores 0 (the hands read 9:46, 24 minutes off). A real trial is scored from a photo of the dial
at the end: the minute hand within one minute mark of the 2 (10 minutes) and the hour hand just past the 10, and the
date as it was at the start. The 2nd click (0.25) is seen in the trial video, or from the hands having moved at all
(they move only at the 2nd click). Whether the crown is home is checked after the photo by turning it gently without
pulling: home, it moves neither the hands nor the date.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=set_a_wristwatch_to_ten_past_ten.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Set the watch on the stand to 10:10 without changing its date, and leave its crown pushed fully in." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/watch-dream
```

The scripted demo, which reaches full score (open loop from the known layout; right arm only). The right gripper comes
in level, pointing along the crown's axis (+y) with its fingers closing across the crown, slides its fingertips either
side of the crown until their ends are 0.6 mm off the case, and pinches it. It draws the crown out past both clicks
(aimed 8 mm past the stop, as the arm gives under load), eases off and, keeping hold, turns its wrist 110 deg forward.
It then turns the crown in three more strokes, letting go and turning the open hand back between them (220, 220 and
190 deg): two turns in all, 24 minutes of the minute hand, from 9:46 to 10:10 (it ends at 10:10.1 in the simulation).
It lets go, backs off 8 mm, closes its fingers, pushes the crown home with the fingertips' ends and backs away to the
ready pose. It reaches success at step 760 (38 s).

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=set_a_wristwatch_to_ten_past_ten.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Set the watch on the stand to 10:10 without changing its date, and leave its crown pushed fully in." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/watch-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/watch-dream/*.json; do inspect-robots video "$log"; done
```

`WATCH_DEBUG=1` prints, every 5 steps (`WATCH_DEBUG_EVERY=1` for every step), the time the hands read and how far it
is from 10:10, the stem's position and its click, the furthest click reached, the crown's turn, the turn made so far at
the 1st click and whether the date has changed, how close the nearest finger is to the watch head, and where each
gripper is and how far it is open.

`WATCH_NOPULL=1` runs the demo the wrong way: the same pinch, strokes and push, without first drawing the crown out.
At home the crown turns (about two turns, as in the demo) but moves nothing, so the hands stay at 9:46, 24 minutes
off; the crown never reached the 2nd click. Score 0.

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell Server Edition shared with other simulation jobs: debug runs of the scripted demo
without video (760 steps, 38 s of simulated time, ending on success) took about 5 minutes from start to finish;
a Dream-1 trial of 2,400 steps at 320 x 176 took about 5 minutes, and the demo rendered at 1280 x 720 (900 steps) about 4 minutes. Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

Any quartz three-hand watch with a date window and a two-stop pull-out crown (for example a Timex Easy Reader Date
38 mm with a leather strap), its battery taken out so that the hands stay where they are set, and a single-watch
display stand with an oval C-ring whose top is about 13.5 cm above the table (or a lower stand on a block), taped down
so that the watch's centre is 28 cm in front of the midpoint between the arm bases, with the crown toward the robot's
right. Before each trial pull the crown to the time click, set the hands to 9:46 (coming up to it), push the crown
fully in, check that the date has not moved (set it back at the date click if it has), and buckle the watch snugly on
the ring, dial up.

[timex]: https://timex.com/blogs/the-timex-blog/how-to-set-the-time-on-your-watch
