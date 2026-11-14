# Pull a reject bin from the middle of a gravity flow lane without letting the next bin run down

Files for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). A gravity carton-flow lane
(free-running skate wheels sloped toward a front stop, no motor) holds four small-parts bins; the second or third from
the stop carries a red reject tag. The robot puts the tagged bin in the reject tote without letting the next bin up the
lane run down into the gap, then leaves the lane closed up with nothing held. The setup, scoring and fidelity notes are
in `task.json`.

## Run a trial

Scripted demo (reaches full score in both slots; run it with `FLOW_TAG_SLOT=2` and again with `FLOW_TAG_SLOT=3`). It is open
loop for the layout `FLOW_TAG_SLOT` fixes, like any fixed layout; a model gets the random draw and must find the tag on camera.

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/pull-reject-bin-from-flow-lane/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "Put the red-tagged bin in the reject tote. Until it is in the tote, the next bin up the lane must not move more than 2 cm. At the end, the other three bins must be down against the front stop and each other, with nothing holding them." --scorer task_score --max-steps 600 --store-frames

Model run (Dream-1): the same command with `--policy dream --max-steps 2400` (2,400 steps = 2 minutes at 20 Hz).

Offline tests: `PYTHONPATH=<yam_arena> python3 test_task.py` prints "all pass" (it runs both slots and the random draw).

Render the videos of a run with `inspect-robots video <log.json>`.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s; otherwise the trial scores `score()` at the end of
the run. The scorer remembers what happened during the run: it follows how far the next bin moves from its start, in any direction (rolling down, lifted, or pushed back
up the lane), until the tagged bin is in the tote: its middle inside the tote's walls and its bottom below the rim.
Movement after that does not count, since rolling down is then what the lane is for.

- **Placed:** the tagged bin rests on the tote's floor (bottom within 1.5 cm of it, not on the rim), upright within
  20 degrees (set down, not propped on a wall), no hand touching it, still for 1 s. "Touching" is contact between
  the gripper's own shape and the bin's outline (within 2 mm), not nearness.
- **Closed up:** the other three bins in their order, the first within 1 cm of the stop and each of the others
  within 1 cm of the bin ahead (the bins touch; 1 cm allows for their molded draft), all flat on the wheels within
  10 degrees and 1 cm, still, no hand touching any.
- **Still:** moving under 1 cm/s and turning under 10 degrees/s.

| Score | When | Why this much |
|---|---|---|
| 1.0 | placed, next bin moved at most 2 cm, lane closed up, held 1 s | the job done the way a stocker does it |
| 0.75 | placed, next bin moved at most 2 cm, lane not closed up (or a hand still on a lane bin) | the queue was held, but the lane isn't ready to pick from |
| 0.5 | placed, next bin moved over 2 cm and at most 8 cm | it started to run and was caught short of the bin ahead |
| 0.25 | placed, next bin moved over 8 cm | the reject is out, but the next bin ran more than half of the 13.7 cm gap |
| 0 | the tagged bin is not placed | doing nothing scores 0 |
| 0 for the whole trial | a lane bin (one of the three untagged bins) falls: outside the lane, moving down faster than 0.5 m/s | a dropped bin of bolts is stock on the floor |
| 0 at the end | a lane bin off the lane (middle outside the guide rails or past either end) or tipped past 30 degrees | the lane isn't usable; a bin lifted out or tipped and stood back on the wheels counts again |

Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

Thresholds: 2 cm is the hold: a hand closing on a bin's wall moves it about 1 cm, while a bin that has begun to run
(0.22 m/s^2 on this slope) passes 2 cm in under half a second. 8 cm is more than half of the 13.7 cm gap the tagged
bin leaves. 30 degrees is a bin gone over, not one rocking on the wheels. 0.5 m/s down is a fall (a bin dropped from
the lane's 4 to 7 cm passes it), not a hand setting a bin down.

Any way of keeping the next bin within 2 cm counts: it is blocked until the tagged bin's bottom clears its hopper
front (about 10 cm up), then needs about 0.43 s to run 2 cm, so a very quick one-hand lift could also stay within
2 cm. Holding the next bin is the way that doesn't depend on speed.

Real trial: tape a paper scale marked every 1 cm on the near guide rail beside the next bin's rear corner and film
from a side camera at table height; read the next bin's movement from the paused video (lifting or pushing back shows
there too), the end gaps with a ruler, and the tagged bin's seat and tilt from a photo.

## Runtime and seeds

- Demo: 411 control steps with the tag in slot 2 and 409 in slot 3 (about 20.5 s simulated); it ends early once the
  full score has held for 1 s.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed: the only random draw is the tagged bin's slot (2 or 3 from the stop, equal odds), made each time the task
  loads and printed to the log (`flow-lane: tagged bin in slot N from the stop`); `FLOW_TAG_SLOT=2` or `3` fixes it.
  A fixed slot would let a replayed plan skip finding the tag. `object_jitter=0`; nothing else varies.

## Real-world build

- Lane: skate-wheel carton-flow track (three strips, wheels about 2 cm across at 4 cm pitch) cut to 58 cm, screwed to
  two steel cross bars on legs so the wheel tops sit 4 cm above the table at the low end and 7.0 cm at the high end
  (3 degrees); aluminum angle guide rails 2.5 cm above the wheels, 12 cm apart inside; a 4 cm tall steel stop plate.
  Clamp the stand to the table.
- Check before building: a 0.35 kg bin set at rest at the loading end must start on its own and run to the stop
  (3 degrees pulls it with about 0.18 N). If it doesn't on the chosen track, raise the lane to 3.6 degrees, the top
  of the usual range, and change the slope in the scene to match.
- Bins: four 30210-size polypropylene shelf bins (13.7 x 10.5 x 7.6 cm) with a bag of 1/4-20 zinc hex bolts in
  each, made up to 0.35 kg; a red reject tag on the one in slot 2 or 3 (toss a coin for each trial).
- Tote: a red polypropylene tote about 18 x 26 x 10 cm, set behind the lane and taped down.
- Safety: the bins weigh 0.35 kg and roll at walking pace or less; nothing sharp is exposed (the bolts are bagged).

## Files

- `task.py`: the scene, `score()`, `success()` and the scripted `demo()`; its docstrings explain every rule.
- `flowkit.py`: the task's helper module that gives the lane's colliders their own friction material (0.03, combine mode "min").
- `yamkit.py`, `guardkit.py`: the task's helper modules (`yamkit.py` spawns the modeled props; `guardkit.py` checks gripper contact).
- `test_task.py`: offline checks of the scorer and the demo plan (no simulator).

In the code, the next bin is `back_bin` and its movement is `travel()`.
