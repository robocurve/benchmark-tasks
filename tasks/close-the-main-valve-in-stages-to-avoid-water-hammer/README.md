# Close the main valve in stages to avoid water hammer

A main line valve with a sliding handle, three 2 cm colour bands along its guide plate, a FLOW lamp and an ALARM lamp.
The robot is told to shut the valve slowly: one mark at a time, taking at least four seconds from each mark to the
next, until the handle is at the stop. Slamming a valve shut on a flowing main stops the moving water column at once,
and the pressure wave that follows (water hammer) can burst pipes and joints a long way down the line. Operators close
large valves in stages so the flow dies away gradually. The sim checks the handle's pace every step: any stage taken
in under 4 s, or more than 2.6 cm of travel in any 4 s, trips the surge, latches the ALARM lamp red, and scores 0.

Other valve tasks in this set score the order of steps; this one scores pace. There is one handle and one direction
to push it, and the same three pushes score 1.0 or 0 by their timing alone.

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
  --instruction "Shut the main line valve slowly: close it one mark at a time, taking at least four seconds from each mark to the next, until the handle is at the shut stop." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/hammer-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=close_the_main_valve_in_stages_to_avoid_water_hammer.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Shut the main line valve slowly: close it one mark at a time, taking at least four seconds from each mark to the next, until the handle is at the shut stop." \
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

The handle's travel is read every step in the valve's own frame, from OPEN (0) to the stop (6 cm). Beside its path the
plate carries three 2 cm bands, white, black and white, starting level with the handle's leading face at rest; their
far edges, at 2, 4 and 6 cm of travel, are the marks. A mark counts as reached 3 mm before its edge, about what a person
can call from the video: the first mark at 1.7 cm, the second at 3.7 cm, and the valve is shut at 5.5 cm or more.
Open to the first mark is free. Past it, two rules set the pace:

- **4.0 s per stage.** From reaching 1.7 cm to reaching 3.7 cm, and from 3.7 cm to 5.5 cm, each takes at least
  4.0 s. Each crossing is timed from the latest crossing of the level below; a level the handle goes back more than
  2 mm below is timed afresh when it is crossed again; two levels crossed in one step are 0 s apart.
- **2.6 cm per 4.0 s.** Counting only travel past 1.7 cm, the handle advances no more than 2.6 cm in any 4.0 s, measured
  from the furthest back it was in those 4 s.

Breaking either rule is a surge. The FLOW lamp is green while the handle is under 5.5 cm and grey once shut; the ALARM
lamp latches red at a surge.

| Outcome | Score |
|---|---|
| Handle at 5.5 cm or more (shut), every stage 4.0 s or longer, never more than 2.6 cm in 4.0 s past 1.7 cm | 1.0 |
| Handle at 1.7 cm or more but under 5.5 cm, no surge | 0.5 |
| A stage under 4.0 s, two levels in one step, or more than 2.6 cm in 4.0 s (surge), whatever else happened | 0 |
| Handle pulled back after shutting | scored by where it ends: 1.0 shut, 0.5 from 1.7 to 5.5 cm, 0 under 1.7 cm |
| Handle under 1.7 cm, no surge | 0 |
| Valve body moved over 3 cm or tipped over 20 degrees | 0 |
| Nothing done | 0 |

A person scores a real trial from the end photo (the handle's leading face against the band edges and the stop,
checked with a tape measure) and the top camera's video, the `top_cam` MP4 at 20 frames a second. The frames at which
the leading face comes within 3 mm of each of the first two band edges, and within 5 mm of the stop, give the stage
times; and in no 80 frames may the leading face move more than 2.6 cm forward past the point 3 mm short of the first
edge, which is a band and a third.

## Design notes

- **Pace is what's scored.** The end photo can't show how fast the valve was shut, so the step hook checks the handle
  for the whole trial. A surge latches: the ALARM lamp stays red and the score is 0, below the 0.5 for stopping
  part-way, as a burst main is worse than a half-shut valve.
- **Why two rules.** Timing the crossings alone can be gamed: cross a level, wait, then slam the handle to just short of
  the next and nudge it over 4 s later, or skip a mark's pause with one fast push between two well-timed crossings.
  Every crossing is then 4 s or more apart while the handle has been slammed. The 2.6 cm cap catches those that move
  the handle more than 2.6 cm (a band and a third) within 4 s, and the tests run each of them to 0. A quick push of
  one band or less, held at the next mark for the rest of the stage, is still allowed: that is one stage done briskly,
  as the demo does it, and the 4 s per stage still applies.
- **Honest pauses pass.** A robot that stops with the leading face on a band edge, or 2 mm short of it, has reached the
  mark: levels sit 3 mm before the edges. Pausing 4 s at each mark always passes, since a stage also holds the moves
  on either side of the pause. A slow, steady close passes too (18 s from open to shut does, 12 s doesn't): what the
  line feels is how fast the flow is cut off, not whether the handle stopped.
- **The first push is free.** A gate valve cuts off most of its flow near the end of its stroke, so the first third of
  the travel barely slows the water. Open to 1.7 cm has no limit, and travel before 1.7 cm doesn't count toward the cap.
- **No way round the clock.** Backing the handle more than 2 mm below a level and pushing through it again restarts that
  level's timing, and the cap is read against the furthest back the handle went, so pulling back and slamming it
  through counts as slamming. The 2 mm of slack keeps a handle jittering on a level from restarting it. A paced close
  back to the stop after pulling back earns 1.0 again.
- **The trial ends at the stop.** Once the valve is shut without a surge there is nothing left to get wrong, so the
  usual 1 s hold ends the trial.
- **Pushing along x.** The fingers close along y, so a push along x leads with the finger links, which taper outward:
  from the URDF's collision hulls, half their extent in x is 1.2 cm at the TCP and widens about 0.39 cm per cm, to
  2.17 cm at 2.5 cm above it and 2.45 cm at 3.2 cm. The handle is 3.5 cm tall, so its top is 1.7 cm above the TCP,
  where the fingers reach 1.86 cm ahead of it, and every push is planned on that face. The last push then overdrives the
  stop by 6 mm at most. The first two pushes end 7 mm past their level and at least 1.1 cm short of the next (with the
  fingers reaching 2 mm less or more than measured, still 5 mm past and 9 mm short), and move the handle 2.0 cm and
  1.6 cm past 1.7 cm, well under the cap. Each comes down 3 cm behind the handle with the pad tips 5 mm over the plate; the stop block stays 2.4 cm
  ahead of the finger face at the end; the wrist camera passes 3 cm over the handle's top and 4.9 cm over the lamps.
- **A wide base.** The bracket runs the plate's full 10 cm, so pushed at the handle's top (10.9 cm up) the 3 kg body
  tips only past about 13.5 N, against about 6 N on a 5 cm base.
- **Unintended strategies.** Shoving the valve body: the handle is read in the body's own frame, so a few millimetres
  change nothing; a body moved over 3 cm or tipped over 20 degrees has been torn off its line: 0. Tipping the body so
  the handle slides: 0 by the tilt, and the slide is checked like a push. Pulling the handle back to restart the clock:
  see above. Holding the handle short of the first mark earns nothing.
- **The rule behind it.** AWWA M11 (Steel Pipe), in its chapter on water hammer and pressure surge: a valve shut faster
  than the line's critical period 2L/a (the time a pressure wave takes to run to the far end and back) raises the full
  Joukowsky surge, so valves on flowing mains are closed slowly. That surge is rho a dv, about 10-14 bar for each metre
  per second of flow stopped, with a wave speed a of about 1,000-1,400 m/s. Water distribution operators and
  pipefitters close large gate valves in stages as standard operating practice.
- **What isn't modelled.** No water or pressure: the surge is a rule on the handle's pace and the lamps are recoloured
  boxes. The FLOW and ALARM labels exist only on a real bench, where an operator with a stopwatch, or limit switches at
  the band edges feeding a timer relay, turns the ALARM lamp red. A sliding handle stands in for a handwheel, and every
  2 cm counts the same; the last 5 mm, from 5.5 cm to the stop, is not timed, though a real gate valve cuts most of its
  flow there. On a long main each stage takes minutes; 4 s is the bench's stand-in, long enough to be a deliberate
  pause and short enough for a 2 minute trial.

## Runtime and seeds

- The scripted demo's plan takes 38 s of simulated time (760 steps at 20 Hz). The handle reaches 1.7 cm at about
  4.8 s, 3.7 cm at about 19.3 s and 5.5 cm at about 33.7 s: 14.5 s and 14.4 s per stage, against the 4.0 s needed, and
  never more than 2.0 cm past 1.7 cm in any 4 s, against the 2.6 cm cap. The trial ends at 1.0 about 1 s after the
  handle reaches 5.5 cm (about 695 steps).
- Dream-1 runs the full 2,400 steps unless it finishes first (2 minutes of simulated time).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start.
