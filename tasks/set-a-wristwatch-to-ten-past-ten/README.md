# Set a wristwatch to 10:10: crown out to the time click, turned in strokes, pushed back in, the date left as it was

An ordinary quartz three-hand wristwatch with a date window (38 mm steel case, like a Timex Easy Reader Date 38 mm)
lies loose and unbuckled on a stack of hardcover books. Its hands read 9:46. The instruction asks for it to be set
to 10:10 without changing its date, with its crown left pushed fully in.

People set analogue watches after a daylight saving change, a battery change or clock drift. Watches are often
displayed at 10:10. The Timex guides explain how to distinguish the date and time positions and how to reseat the
crown after setting the time. This trial uses a date watch with a middle click for the date and an outer click for
the time. [Timex time guide][timex] · [Timex date guide][timex-date]

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

The crown overhangs the edge of the top book so the fingertips can rotate around it. The watch can slide, lift and
turn. The demo uses one hand to steady the case, as a person would, while the other operates the crown.

The task is the task file [`set_a_wristwatch_to_ten_past_ten.py`](set_a_wristwatch_to_ten_past_ten.py) for the
`yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac
Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable,
installs everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no
third-party assets. The watch and books are built in these files from boxes and cylinders.

`bodykit.py` gives the loose watch a free base, a sliding crown stem, a revolving crown and revolving watch hands.
Its scene hook supplies the task file with body poses and joint positions. The task file sets the stem spring's
detent target and the hands' gear-train targets through Isaac Lab's articulation API at each control step.

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, z up from the table top.

- Three hardcover books, each 26 x 15 x 4.5 cm and about 1 kg, stacked flush. The covers are cloth over board and
  the pages are paper. The stack top is 13.5 cm above the table. The common near edge is at y -1.85 cm, and the
  books are centred at x 28 cm. The stack provides a flat work surface with clearance for the crown and fingertips.
- Wristwatch, about 40 g, unbuckled, dial up, centred at x 28, y 0, 12 o'clock toward +x and the crown
  (3 o'clock) toward -y. The steel case is 38 mm across and 9.5 mm thick, with its back on the top book and its rim
  overhanging the edge by about 0.5 mm. It has a white dial with hour marks, a date window at 3 o'clock, black hour
  and minute hands, a red seconds hand and a 20 mm brown leather strap spread out along x. The
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
- 1.0: as 0.75, with the crown pushed fully home (nearer home than the 1st click), the case resting on the top book
  at no more than 10 degrees of tilt, and every finger point 1 cm or more from the watch head. Hold all conditions
  for one second with at most 1 mm of case movement.

Once the date has been changed (half a turn of the crown at the 1st click), the score is at most 0.5 for the rest of
the run. Doing nothing scores 0 (the hands read 9:46, 24 minutes off). For a real trial, take a square-on dial photo
at the start and finish, and record the trial with a close view of the date window and crown plus a side view of
the case and fingers. Compare the final minute hand to the 2 (10 minutes), and check that the hour hand is just past
10. Watch the date window through the trial for any change. Credit the 2nd click when the video shows it or shows
that the hands have moved. For full credit, use the side view and a 1 cm reference on the book to confirm the dial
up, caseback resting on the book and the fingers clear for the final second. Check the case for movement against
a metric ruler beside it. Check the crown position in the side view or after the photo by turning it gently while
fully in. It will move neither the hands nor the date.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=set_a_wristwatch_to_ten_past_ten.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Set the watch to 10:10 without changing its date, and leave its crown pushed fully in." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/watch-dream
```

The scripted demo is open loop from the known layout. The left gripper braces the back half of the case against the
book. The right gripper comes in level, pointing along the crown's axis (+y) with its fingers closing across the
crown. It slides its fingertips either side of the crown until their ends are 0.6 mm off the case, pinches it and
draws it to the second click. It eases off and, keeping hold, turns its wrist 110 deg forward.
It then turns the crown in three more strokes, letting go and turning the open hand back between them (220, 220 and
190 deg): two turns in all, moving the minute hand 24 minutes from 9:46 to 10:10.
It lets go, backs off 8 mm, closes its fingers, pushes the crown home with the fingertips' ends and backs away to the
ready pose. The left gripper then lifts off and both hands stay clear. The demo finished at step 969 (48.45 s
of simulated time) with the hands within 0.27 min of 10:10, the date unchanged and the crown home.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=set_a_wristwatch_to_ten_past_ten.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Set the watch to 10:10 without changing its date, and leave its crown pushed fully in." \
  --scorer task_score --max-steps 1050 --store-frames --log-dir ~/yam-eval/watch-demo
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
Turning the crown at home leaves the hands at 9:46, 24 minutes off target.

`WATCH_NOBRACE=1` leaves the case free while the right arm follows the same crown-setting trajectory.

## Runtime per trial

On an NVIDIA RTX PRO 6000 Blackwell Server Edition, the 969-step demo took 3 min 18 s without storing frames and
4 min 4 s with frames stored at 1280 x 720. The 2,400-step Dream-1 trial took 4 min 28 s and scored 0.0. Its
right gripper came down from above, failed to engage the crown and withdrew. Isaac Sim's first start on a new
machine can take longer while it compiles shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A quartz three-hand watch with a date window and a two-stop pull-out crown, such as the 38 mm Timex Easy Reader
Date with a leather strap. Remove the battery so the hands stay where they are set. Stack three ordinary hardcovers
as specified above. Before each trial pull the crown to the time click, set the hands to 9:46 a.m., push the crown fully
in and record the date. Place the watch dial up and unbuckled with its 3 o'clock edge over the near edge of the top
book. The crown should be free to turn and pull. Spread the strap along the top cover and leave it loose.

[timex]: https://timex.com/blogs/the-timex-blog/how-to-set-the-time-on-your-watch
[timex-date]: https://timex.com/blogs/the-timex-blog/how-to-set-the-day-and-date-on-your-timex-watch
