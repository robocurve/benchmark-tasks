# Fill the day tank to the mark and close the valve

A white day tank with a level strip up its front face and a red mark at the sixth segment, and a brass transfer valve
with a handle that slides open and shut. The robot is told to top the tank up: open the valve, watch the level climb
one segment every two seconds, and close the valve when it reaches the red mark. A valve left open runs the tank over
onto the mat, and a valve not closed at the end scores nothing whatever the level; closing it early or late scores
less. There is no sensor or timer doing the watching: the robot has to attend the transfer, as the rules for chemical
day tanks ask of an operator.

The task needs one arm: the right pushes the handle open from one side, waits, and pushes it closed from the other.

## Files

| File | What |
|---|---|
| `fill_the_day_tank_to_the_mark_and_close_the_valve.py` | The yam_arena task file: objects, `success()`, `score()`, `distance()` and the scripted `demo()` |
| `benchkit.py` | Tools built from several boxes with sliders (the valve's handle), per-step hooks and `recolor` |
| `plantkit.py` | The clock, the level strip, the mat as a lamp, and the slider checks |
| `test_fill_the_day_tank_to_the_mark_and_close_the_valve.py` | Offline tests: no GPU needed |

`benchkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), with a Dream API
key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=fill_the_day_tank_to_the_mark_and_close_the_valve.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Top up the day tank: open the transfer valve, watch the level, and close the valve when the level reaches the red mark. Don't let it overflow." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tank-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=fill_the_day_tank_to_the_mark_and_close_the_valve.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Top up the day tank: open the transfer valve, watch the level, and close the valve when the level reaches the red mark. Don't let it overflow." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tank-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/tank-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. A trial always runs to its step limit and scores the last step, so a robot that
closes the valve at the mark still has time to reopen it, and that would count. The scripted demo stops by itself
shortly after its plan ends.

Offline tests (no GPU), with `yam_arena` importable:

```bash
python -m pytest -q test_fill_the_day_tank_to_the_mark_and_close_the_valve.py
```

## Scoring

Scored from the end state plus one latch: whether the level ever went above the rim. The tank is 10 x 10 cm inside,
so 1 cm of depth is 100 mL; the strip's eight segments are 2.5 cm apart, 250 mL each, and the tank starts at 500 mL
(two lit). The valve passes liquid once the handle is past 70% of its 3 cm travel (21 mm), at 125 mL/s, and is closed
when the handle is back within 30% (9 mm) of its closed stop. The red mark is the 1500 mL line; the rim is 2000 mL.

In the sim the strip lights the nearest whole segment, and the bands are drawn where it changes: six lit is exactly
1375 to 1625 mL, the surface within 1.25 cm of the red line; four or five lit is 875 to under 1375 mL; seven or eight
lit is over 1625 mL up to the rim.

| Outcome | Score |
|---|---|
| Level 1375 to 1625 mL at the end (six lit), valve closed | 1.0 |
| Level 875 to under 1375 mL (four or five lit), valve closed (stopped early) | 0.5 |
| Level over 1625 up to 2000 mL (seven or eight lit), valve closed (overshot, nothing spilled) | 0.3 |
| Valve not closed at the end, at any level | 0 |
| Overflowed at any time (above 2000 mL: liquid on the mat), whatever follows | 0 |
| Level under 875 mL: valve never opened, or open for under 3 s | 0 |
| Valve tipped more than 20 degrees | 0 |
| Nothing done | 0 |

## Design notes

- **The trial doesn't end when the safe state is reached.** With the usual 1 s hold, closing the valve at the mark
  would end the trial at 1.0 while a robot might be about to reopen it. `SUCCESS_HOLD_S` is set longer than the run,
  so the end state decides.
- **A valve not closed scores 0, whatever the level.** The run is 120 s and the tank runs over after 12 s of flow, so
  any end with the valve open and the tank not yet over means it was opened in the last 12 s. Paying for the level
  in that state would reward idling and opening the valve late; so it pays nothing.
- **An overflow is for good.** The step hook counts the level up while the handle is open, and the first step past
  2000 mL recolours the mat blue and latches. Closing the valve afterwards scores 0.
- **Bands sit on what can be seen.** The strip lights the nearest whole segment, so a band edge inside a segment
  could not be read from the picture. The edges are at 875, 1375 and 1625 mL: the first level with four lit, the
  first with six lit, and the last with six lit (the seventh lights at the next step); on a real bench they are the
  8.75, 13.75 and 16.25 cm lines of the sight gauge.
- **Timing.** The mark band is 7 to 9 s of flow. The demo's handle passes liquid for 8 s, 1 s inside both ends, so
  a handle that lags or leads the fingers a little in the simulator still lands at the mark. A kinematic replay in
  the tests plays the demo's waypoints against a rigid model of the handle and checks the result: 1500 mL, six
  segments lit, valve closed.
- **Nothing springs back.** The handle has no spring and stays where it is pushed; the scorer reads it at the end.
  A handle left between 30% and 70% passes nothing in the sim, but it isn't closed, so it scores 0.
- **Where the fingers push.** The closed fingers are 13 mm wide for the 3 cm above their pad tips and widen above
  that, so the demo pushes the 5 cm handle 2 cm up its height, where its top still meets the narrow part.
- **Clearance.** The handle travels at x 20, while the tank's front face is at x 27.4 and its right-hand face at
  y -12.6. The gripper housing is 6.8 cm across, so at push height its near edge stays 4.0 cm short of the front
  face; the wrist camera rides 7 cm ahead of the fingers and 7.5 cm above their tips, and passes 4.9 cm to the right
  of the right-hand face. A test proves both from the task file's layout.
- **Unintended strategies.** Shoving the valve along the table changes nothing: the handle is read against its own
  block. Tipping the valve more than 20 degrees scores 0, and it is taped down on a real bench. There is nothing loose
  to prop the handle with. Pumping the handle part way passes nothing. Reopening after the mark and closing again
  ends overshot (0.3), or still at the mark if the valve was open for 1 s or less; reopening and walking away ends
  with the valve open or overflows, 0 either way.
- **Scoring a real trial.** A photo of the sight gauge and of the handle against a tape measure at the end gives the
  level and the valve; the video shows whether anything ran over. The mat is dark so liquid shows.
- **The rule behind it.** The Ten States Standards (Recommended Standards for Water Works, 2012) ask that a day tank
  be scale-mounted or carry a calibrated gauge showing its actual volume, and that chemical storage tanks have a level
  indicator, an overflow and a basin or drain for accidental spills. Illinois's adoption
  ([35 Ill. Adm. Code 604.1105](https://my.ilga.gov/ftp/JCAR/AdminCode/035/035006040K11050R.html)) adds that filling
  a day tank must not be automated: an operator attends the transfer.
- **What isn't modelled.** No liquid: the level is a count shown on the strip to the nearest whole segment, so a
  surface between graduations has no sim equivalent. A real gate valve passes some liquid at any opening; here it
  passes liquid only past 70% of travel. The strip stands on a vertical face: the top camera sees it at a grazing
  angle of about 30 degrees, about 19 px tall at 320 x 176 (2.4 px per segment, the red mark about 1.5 px), and the
  right wrist camera, which looks down and back at the valve from the watch pose, does not see it unless the wrist is
  turned. The hypochlorite is water on a real bench.

## Runtime and seeds

- The scripted demo takes about 16 s of simulated time (322 steps at 20 Hz).
- Dream-1 runs the full 2,400 steps (2 minutes of simulated time).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start.
