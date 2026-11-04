# Fill the day tank to the mark and close the valve

A white day tank with a level strip up its front face and a red mark at the sixth segment, and a brass transfer valve
with a handle that slides open and shut. The robot is told to top the tank up: open the valve, watch the level climb
one segment every two seconds, and close the valve when it reaches the red mark. A valve left open runs the tank over
onto the mat; closing it early or late scores less. There is no sensor or timer doing the watching: the robot has to
attend the transfer, as the rules for chemical day tanks ask of an operator.

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

Scored from the end state plus one latch: whether the level ever went above the rim. The level is read in mL against
the strip, 100 mL per segment, starting at 200 mL. The valve passes liquid once the handle is past 70% of its 3 cm
travel (21 mm), at 50 mL/s, and is closed when the handle is back within 30% (9 mm) of its closed stop. The red mark
is 600 mL; the tank runs over above 800 mL.

| Outcome | Score |
|---|---|
| Level 500 to 650 mL at the end, valve closed | 1.0 |
| Level 350 to 500 mL, valve closed (stopped early) | 0.5 |
| Level 650 to 800 mL, valve closed (overshot, nothing spilled) | 0.3 |
| Level 500 to 650 mL, valve not closed | 0.3 |
| Level 350 to 500 mL or 650 to 800 mL, valve not closed | 0.1 |
| Overflowed at any time (above 800 mL: liquid on the mat), whatever follows | 0 |
| Level under 350 mL: valve never opened, or open for under 3 s | 0 |
| Valve tipped more than 20 degrees | 0 |
| Nothing done | 0 |

## Design notes

- **The trial doesn't end when the safe state is reached.** With the usual 1 s hold, closing the valve at the mark
  would end the trial at 1.0 while a robot might be about to reopen it. `SUCCESS_HOLD_S` is set longer than the run,
  so the end state decides.
- **An overflow is for good.** The step hook counts the level up while the handle is open, and the first step past
  800 mL recolours the mat blue and latches. Closing the valve afterwards, or draining nothing, scores 0.
- **Leaving the valve open scores below closing it at every level.** Reaching the mark with the valve open earns
  0.3, the same as overshooting and closing it: the tank hasn't run over yet, but nobody is watching it any more.
  Short of the mark or past it with the valve open earns 0.1.
- **Timing.** The mark band is 6 to 9 s of flow. The demo's handle passes liquid for 7.5 s, 1.5 s inside both ends,
  so a handle that lags the fingers a little in the simulator still lands at the mark. A kinematic replay in the
  tests plays the demo's waypoints against a rigid model of the handle and checks the result: 575 mL, sixth segment
  lit, valve closed.
- **Nothing springs back.** The handle has no spring and stays where it is pushed; the scorer reads it at the end.
  A handle left between 30% and 70% passes nothing in the sim, but it isn't closed, so it scores as open.
- **Where the fingers push.** The closed fingers are 13 mm wide for the 3 cm above their pad tips and widen above
  that, so the demo pushes the 5 cm handle 2 cm up its height, where its top still meets the narrow part.
- **Clearance.** The valve stands 12 cm in front of the tank's face and 15 cm to its right. The gripper housing is
  6.8 cm across and the wrist camera sits 7 cm ahead of the fingers and 7.5 cm above their tips, so at push height
  both would meet a tank wall any nearer; a test proves the clearance from the task file's layout.
- **Unintended strategies.** Shoving the valve along the table changes nothing: the handle is read against its own
  block. Tipping the valve more than 20 degrees scores 0, and it is taped down on a real bench. There is nothing loose
  to prop the handle with. Pumping the handle part way passes nothing. Reopening after the mark either overflows (0)
  or ends with the valve open (0.3 at most).
- **Scoring a real trial.** A photo of the strip and of the handle against a tape measure at the end gives the level
  and the valve; the video shows whether anything ran over. The mat is dark so liquid shows.
- **The rule behind it.** The Ten States Standards (Recommended Standards for Water Works, 2012) ask for a calibrated
  gauge on a day tank showing its actual volume, and for chemical storage tanks to have a level indicator, an overflow
  and a basin or drain for accidental spills. Illinois's adoption
  ([35 Ill. Adm. Code 604.1105](https://my.ilga.gov/ftp/JCAR/AdminCode/035/035006040K11050R.html)) adds that filling
  a day tank must not be automated: an operator attends the transfer.
- **What isn't modelled.** No liquid: the level is a count shown on the strip, and the strip is a gauge rather than
  the liquid's height (800 mL in a 12 cm square tank is only 5.6 cm deep). A real gate valve passes some liquid at any
  opening; here it passes liquid only past 70% of travel. The hypochlorite is water on a real bench.

## Runtime and seeds

- The scripted demo takes about 16 s of simulated time (312 steps at 20 Hz).
- Dream-1 runs the full 2,400 steps (2 minutes of simulated time).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start.
