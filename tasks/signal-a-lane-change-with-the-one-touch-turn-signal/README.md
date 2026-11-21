# Signal a lane change to the left with a car's one-touch turn signal: three flashes, the stalk never locked on

Every driver signals lane changes with the turn-signal stalk, and every passenger has watched it done. A car's
steering column section stands on a bench stand, as the driver sees it from the seat: a 37 cm steering wheel facing the
robot, the column shroud behind it with the combination switch, and the turn-signal stalk standing out of the switch on
the driver's left, behind the wheel's rim. An instrument panel behind the wheel has the two green turn-signal arrows.
The instruction asks for the three-flash lane-change signal for a lane change to the left; it does not say how.

What makes it hard is a precise, gentle push, and the scene shows why. Most cars have a one-touch lane-change signal:
the stalk is pushed only part of the way, to a light stop, and let go, and the signal flashes three times. Pushed
further, the stalk passes the crest of its detent and clicks into its locked turn position, and the signal keeps
flashing, telling the drivers behind about a turn that is not coming. Pushed 2 cm from its end, where a fingertip
reaches it, the stalk closes its switch contact after 7.6 mm, meets the light stop at 13 mm and passes the detent's
crest at 28 mm, against a spring that rises from 1.9 N at the stop to 6.3 N at the crest. The finger has to stop inside
that window and let go before the third flash ends (about 2 s), reaching in from above between the wheel's rim (its
back 4.3 cm in front of the push point) and the column. An arm that pushes "down until it stops" locks it. One arm does
it; the other is not needed.

The task is the task file
[`signal_a_lane_change_with_the_one_touch_turn_signal.py`](signal_a_lane_change_with_the_one_touch_turn_signal.py) for
the `yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Why it matters

Signalling a lane change is part of every drive, and the one-touch signal is how most current cars do it. The car
makers' owner's manuals describe it:

- Kia, [One-touch lane change function][kia1]: "To activate an one-touch lane change function, move the turn signal
  lever slightly and then release it. The lane change signals will blink 3 times." The same manual ([turn signals][kia2])
  gives the full turn signal as the whole movement of the lever ("move the lever up or down"), which self-cancels after
  the turn; if it keeps flashing, the driver returns the lever to OFF by hand. Held slightly moved instead of released,
  the lane-change signal flashes until the lever is let go.
- Mazda, [Lane-change signals][mazda]: "Move the lever halfway toward the direction of the lane change―until the
  indicator flashes― and hold it there." On the three-flash function ([Mazda][mazda3]): "The operation can be canceled
  by moving the lever in the direction opposite to which it was operated."
- Toyota's owner's manuals describe the same thing: the lever moved partway and released flashes the signal three
  times, and operating it the opposite way stops it.

In a left-hand-drive car the stalk is on the column's left and moves the way the top of the wheel turns: up for right,
down for left. Anyone watching a clip knows at once what the robot is doing.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac Lab-Arena
0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable, installs
everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no
third-party assets: the column, wheel, switch and panel are built in these files from measurements.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or put one on a hinge. The column
(wheel, shroud, switch housing, stand and panel as one fixed base) and the stalk (on a hinge in the switch housing) are
built with `bodykit.py` as a PhysX articulation; importing it hooks `yam_arena.scene` when the simulator starts,
without changing yam_arena, and gives the task file the stalk's angle every control step. The stalk's spring is the
hinge's joint drive: the task file rewrites its stiffness and rest angle every control step, which gives the switch's
piecewise spring and its detent (below). The two lit arrows are visual-only bodies (nothing collides with them) that
the task file lays over the panel's dark arrows while they are lit.

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, z up from the table top.

- Steering column section on a bench stand, fixed: a left-hand-drive car's column with its wheel and combination
  switch. Steering wheel 37.2 cm across (a 2.8 cm grip on a 17.2 cm radius; three spokes, a 15 cm hub), facing the
  robot, its centre at x 16.5, y -5.0, z 20.5 (its rim 1.9 cm above the table at the bottom). Column shroud 7 cm across
  from x 19.5 to 47; combination switch housing at x 23 to 29 (11 x 8 cm), with the wiper stalk (fixed here) on its
  right; a steel post (4 x 5 cm, at x 40.5) and a plate (16 x 20 cm) under the shroud.
- Turn-signal stalk on the switch's left, on a hinge about the x axis at x 26.0, y -0.2, z 18.3. It is 17 cm long,
  points to +y, 15 deg back toward the robot and 8 deg down; a 14 mm shaft and a 19 mm grip on its last 6 cm; its end at
  x 21.6, y 16.1, z 15.9. Down signals left. Its spring and detent, either way from rest: 0 to 5 deg the light
  lane-change spring (0.28 N m at 5 deg); 5 to 11 deg a stiffer rise (0.92 N m at 11 deg); past 11 deg (the detent's
  crest) it snaps into the locked turn position at 14 deg (a stop at 15 deg) and stays there until pushed back inside
  8 deg. Its switch contact closes at 3 deg and opens below 2.5 deg. At the push point 2 cm from its end (14.5 cm from
  the hinge's axis) that is 7.6 mm of travel to the contact, 13 mm to the light stop and 28 mm to the crest, and 1.9 N
  at the stop rising to 6.3 N at the crest.
- Instrument panel behind the wheel, 18 x 7 cm, its face turned 50 deg up, centred at x 33, y -5, z 28, with a dark
  green arrow 3.2 cm long on each side (left arrow at y 0, right at y -10) that lights bright green with each flash.
- The flasher: a switch contact closing starts the flashing (0.75 s period, the arrow lit 0.4 s of it); it gives at
  least 3 flashes, and goes on flashing while the contact stays closed or the stalk is locked; when the flashing ends
  the arrow goes dark.
- Both arms start at the ready pose. The layout is the same in every trial.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the stalk's angle, its switch contacts and lock and the flasher's state (the device's
own output), and the points of each finger's collision hulls.

- 0: no left signal; the right arrow ever flashed (the stalk was pushed up: 0 for the rest of the run); or the stalk
  still locked into the left turn position at the end.
- 0.25: the stalk was locked into the left turn position (it passed the detent's crest at 11 deg and clicked in) at
  some point, then returned to rest.
- 0.5: a left signal (the stalk pushed down far enough to close its contact, never locked), but more than 3 flashes
  (held too long), more than one push, or at the end the stalk not at rest (within 1 deg of where it rests), the arrow
  still flashing, or a finger within 1 cm of the stalk.
- 1.0 (success, held 1 s): exactly 3 left flashes from one push, never locked, the stalk at rest, the flasher stopped,
  and every finger point 1 cm or more from the stalk.

Doing nothing scores 0. A real trial is scored from the video and the device: the arrows' flashes are counted on the
video (or in the flasher's log, see below); a locked stalk stays down when the hand leaves it; a finger on the stalk
and the stalk at rest are seen on the video.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=signal_a_lane_change_with_the_one_touch_turn_signal.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Give the three-flash lane-change signal for a lane change to the left." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/signal-dream
```

The scripted demo, which reaches full score (open loop from the known layout; the right arm stays at the ready pose).
The left gripper, closed, its fingers closing along y (so the hand is 6.5 cm thick front to back where it passes the
rim), comes round the left of the wheel and over the stalk's grip, down to 5 mm above its top 2 cm from its end, and
pushes it down: the gripper is told to go 24 mm below where it would touch, 19 mm of stalk travel (7.4 deg) and
about 5 mm that the arm gives under the 3.7 N. It holds 0.3 s (the first flash), rises so the stalk springs back up, and moves
up and out to the left of the rim. The signal flashes three times and stops.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=signal_a_lane_change_with_the_one_touch_turn_signal.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Give the three-flash lane-change signal for a lane change to the left." \
  --scorer task_score --max-steps 400 --store-frames --log-dir ~/yam-eval/signal-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/signal-dream/*.json; do inspect-robots video "$log"; done
```

How narrow the push is, from the scripted demo with only the push depth changed (`TS_PRESS`, metres below the touch):
told 8 mm, the stalk reached 2.7 deg and its contact never closed: score 0. Told 12 mm, it reached 4.0 deg; 16 mm,
5.3 deg; 24 mm (the demo), 7.4 deg; 30 mm, 9.0 deg; 36 mm, 10.6 deg: all full score. Told 38 mm (2 mm more), it passed
the crest and locked: score 0. So even a scripted arm that knows the layout exactly has a window of about 2.5 cm of
commanded depth, and the stalk's own window (contact to crest) is 2 cm at the push point.

`TS_DEBUG=1` prints, every 2 control steps (`TS_DEBUG_EVERY=1` for every step), the stalk's angle and spring zone, the
furthest it has been pushed down, whether it is locked (now / ever), the switch contacts, the pushes and flashes on
each side, whether each arrow is lit and the flasher running, the nearest finger's distance to the stalk, whether it is
at rest, and each gripper's position.

`TS_MODE=full` runs the demo the wrong way, pushing the stalk down 45 mm as for a turn: it passes the crest (11 deg),
snaps into the locked position and stays there when the gripper leaves (14.2 deg); the arrow keeps flashing (13 flashes
by step 300). Score 0. `TS_MODE=hold` holds the stalk at the light stop for 3 s instead of 0.3 s: the signal flashes 5
times. Score 0.5.

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell, debug runs of the scripted demo without video (172 steps, 8.6 s of simulated time, ending on
success) took 1 to 2 minutes each with two to five runs sharing the GPU; the demo rendering 1280 x 720 video took about
50 s; a Dream-1 trial of 2,400 steps at 320 x 176 took 4.5 to 7 minutes, beside other runs. Isaac Sim's first start on a new machine
adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A used steering column with its wheel and combination switch from any common left-hand-drive car (a salvage-yard part,
about 50 to 150 USD), or a new combination turn-signal switch (about 30 to 60 USD) with a steering wheel on a short tube,
bolted to a bench stand at the heights above. The switch's one-touch function lives in the car's body computer, so a
microcontroller (an Arduino Uno, about 25 USD) reads the switch's left and right contacts and runs the same flasher
rule as the task file (3 flashes of 0.75 s per touch, more while the contact stays closed or the stalk is locked),
lighting two green LEDs behind arrow cut-outs in a panel and logging each flash with its time. Before the trials,
measure the stalk's travel to its contact, its light stop and its click with a protractor, and its force with a
spring scale; the simulation's numbers are estimates of a typical switch. For each trial wait until the flashing has
stopped, return the stalk to centre if it is locked, set the wheel straight, and start.

[kia1]: https://ownersmanual.kia.com/full_webhelp/SK3/2024/en_US/topics/t00147.html
[kia2]: https://ownersmanual.kia.com/full_webhelp/YBC/2023/en_GB/topics/chapter4_20_9.html
[mazda]: https://owners-manual.mazda.com/gen/en/cx-5/cx-5_8hy1eo20a/contents/05030402.html
[mazda3]: https://mazda.ca/en/digital-owners-manual/2021/cx-5/contents/05030403.html
