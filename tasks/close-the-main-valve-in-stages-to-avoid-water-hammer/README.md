# Close the main valve in stages to avoid water hammer

A main line valve with a sliding handle, three marks along its slot, a FLOW lamp and an ALARM lamp. The robot is told
to shut the valve slowly: one mark at a time, with a pause of at least four seconds at each, until the handle is at
the stop. Slamming a valve shut on a flowing main stops the moving water column at once, and the pressure wave that
follows (water hammer) can burst pipes and joints a long way down the line. Operators close large valves in stages so
the flow dies away gradually. The sim times the handle's crossings of the marks every step: any interval under 4 s
trips the surge, latches the ALARM lamp red, and scores 0.

The task needs one arm: the left pushes the handle three times; the right stays at its ready pose.

## Files

| File | What |
|---|---|
| `close_the_main_valve_in_stages_to_avoid_water_hammer.py` | The yam_arena task file: objects, `success()`, `score()`, `distance()` and the scripted `demo()` |
| `benchkit.py` | Tools built from several boxes with sliders (the valve's handle), and per-step hooks |
| `plantkit.py` | The lamps, the clock and the placement checks |
| `test_close_the_main_valve_in_stages_to_avoid_water_hammer.py` | Offline tests: no GPU needed |

`benchkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), with a Dream API
key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=close_the_main_valve_in_stages_to_avoid_water_hammer.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Shut the main line valve slowly: close it one mark at a time, pausing at least four seconds at each mark, until the handle is at the shut stop." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/hammer-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=close_the_main_valve_in_stages_to_avoid_water_hammer.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Shut the main line valve slowly: close it one mark at a time, pausing at least four seconds at each mark, until the handle is at the shut stop." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/hammer-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/hammer-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. A trial ends at 1.0 once the handle has stood at 5.5 cm or more for 1 s with
no surge and the valve body still; otherwise it scores the last step.

Offline tests (no GPU), with `yam_arena` importable:

```bash
python -m pytest -q test_close_the_main_valve_in_stages_to_avoid_water_hammer.py
```

## Scoring

The handle's travel is read every step in the valve's own frame, from OPEN (0) to the stop (6 cm). The marks stand
2, 4 and 6 cm ahead of the handle's leading face at rest. The first mark is crossed at 2.0 cm of travel, the second
at 4.0 cm, and the valve is shut at 5.5 cm or more. Open to the first mark is free. From crossing the first mark to
crossing the second, and from crossing the second to reaching 5.5 cm, each interval needs at least 4.0 s. Each
crossing is timed from the latest crossing of the level below; a level the handle is pulled back more than 2 mm below
is timed afresh when it is crossed again. An interval under 4.0 s is a surge. The FLOW lamp is green while the
handle is under 5.5 cm and grey once shut; the ALARM lamp latches red at a surge.

| Outcome | Score |
|---|---|
| Handle at 5.5 cm or more (shut), at least 4.0 s from the 2 cm crossing to the 4 cm crossing and from there to 5.5 cm | 1.0 |
| Handle at 2.0 cm or more but under 5.5 cm, no surge | 0.5 |
| Any interval under 4.0 s (surge), whatever else happened | 0 |
| Handle pulled back after shutting | scored by where it ends: 1.0 shut, 0.5 from 2.0 to 5.5 cm, 0 under 2.0 cm |
| Handle under 2.0 cm, no surge | 0 |
| Valve body moved over 3 cm or tipped over 20 degrees | 0 |
| Nothing done | 0 |

A person scores a real trial from the end photo (the handle's leading face against the marks and the stop, checked with
a tape measure) and the video, reading from the frame times when the leading face passed the 2 cm and 4 cm marks and
when it came within 5 mm of the stop.

## Design notes

- **Timing is what's scored.** The end photo can't show how fast the valve was shut, so the step hook times the
  handle's crossings for the whole trial. A surge latches: the ALARM lamp stays red and the score is 0, below the 0.5
  for stopping part-way, as a burst main is worse than a half-shut valve.
- **The first push is free.** A gate valve cuts off most of its flow near the end of its stroke, so the first third of
  the travel barely slows the water. Open to the first mark has no limit; the two later segments are timed.
- **Pausing 4 s always passes.** Each interval also holds the moves on either side of the pause, so a robot that stops
  4 s at each mark is never caught out by its own push times. A slow, steady close that takes as long passes too (18 s
  from open to shut does, 12 s doesn't): what the line feels is how fast the flow is cut off, not whether the handle
  stopped.
- **No way round the clock.** Backing the handle more than 2 mm below a mark and pushing through it again restarts that
  mark's timing, so pulling back and slamming it through counts as slamming. A handle that jumps past two levels in one
  step crosses both 0 s apart. The 2 mm of slack keeps a handle jittering on a mark from restarting it. Shutting,
  pulling back open and slamming it shut again trips the surge; pulling back after a paced close is scored by where the
  handle ends, and a paced close back to the stop earns 1.0 again.
- **The trial ends at the stop.** Once the valve is shut without a surge there is nothing left to get wrong, so the
  usual 1 s hold ends the trial.
- **Pushing along x.** The fingers close along y, so a push along x leads with the finger body, 4 cm wide in x from
  the TCP up. Each push comes down 3 cm behind the handle with the pad tips 5 mm over the guide plate. The 5 cm
  handle's top stays under the finger links' wider part, 4 cm above the TCP; the stop block stays 2.4 cm ahead of the
  finger face at the end of the last push; and the lamps' tops stand 4.9 cm under the wrist camera, so the posts are
  kept low. The first two pushes end 4 mm past their mark and at least 1 cm short of the next level, so an arm a few
  millimetres off still makes one crossing per push. The last is commanded 6 mm past the stop.
- **Unintended strategies.** Shoving the valve body: the handle is read in the body's own frame, so a few millimetres
  change nothing; a body moved over 3 cm or tipped over 20 degrees has been torn off its line: 0. Tipping the body so
  the handle slides: 0 by the tilt, and the slide is timed like a push. Pulling the handle back to restart the clock:
  see above. Holding the handle short of the first mark earns nothing.
- **The rule behind it.** AWWA manuals M11 and M49 set out surge control on water mains: a valve on a flowing main is
  closed slowly, in stages. A closure quicker than the time a pressure wave takes to run to the far end of the line and
  back gives the full surge, about 10 bar for each metre per second of flow stopped. Water distribution operators and
  pipefitters apply the same rule to every large line valve.
- **What isn't modelled.** No water or pressure: the surge is a rule on the handle's timing and the lamps are
  recoloured boxes. A sliding handle stands in for a handwheel, and every 2 cm counts the same. On a long main each
  stage takes minutes; 4 s is the bench's stand-in, long enough to be a deliberate pause and short enough for a 2 minute
  trial.

## Runtime and seeds

- The scripted demo's plan takes 38 s of simulated time (760 steps at 20 Hz). The handle crosses the marks at about
  4.9 s and 19.4 s and reaches 5.5 cm at about 33.7 s: 14.5 s and 14.3 s between crossings, against the 4.0 s needed.
  The trial ends at 1.0 about 1 s later (about 695 steps).
- Dream-1 runs the full 2,400 steps unless it finishes first (2 minutes of simulated time).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start.
