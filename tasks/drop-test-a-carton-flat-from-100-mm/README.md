# Drop-test a carton flat from 100 mm onto the pad

The robot has to run a small drop test: pick up the carton, hold it level about 100 mm above the pad, and let go,
so that it falls freely and lands flat on a large face, wholly on the pad. Setting it down gently earns only 0.2,
because the job is the release, not the placement.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena), on the `station` scene. `near_misses.py` lists near-successes and wrong outcomes
with the score each must get; they are plain-Python checks of `score()` and `success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable (replace `<task.py>` and `<log dir>`):

```bash
I="Drop-test the carton: hold it level with its underside about 100 mm above the grey pad, let go, and leave it where it lands. It must land flat on one of its two large faces, wholly on the pad."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=<task.py> -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir <log dir>

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=<task.py> -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 300 --store-frames --log-dir <log dir>
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 30 s of simulated time (600 steps at 20 Hz); about a minute of wall time on an
  L40S once Isaac Sim has started (its first start takes several minutes).
- **Control rate:** run at `control_hz=20`. Free fall is recognised from the change in speed over one and two
  control steps (0.05 and 0.1 s).
- **Seeds:** none needed. Submitted runs use `-E object_jitter=0`, so the carton starts exactly where `task.py`
  puts it. With jitter on, seeded resets shift it by up to the jitter in x and y.
- **History:** the drop height and the landing pose are kept at module level over the trial and cleared only by an
  episode reset (the carton back within 0.5 mm of its start and at rest, reached by a jump of more than 20 mm).

## Scripted demo

`demo()` drives the left arm: it grips the carton across its 80 mm width near the bottom, carries it over the pad,
holds it still, and opens the fingers in 0.3 s. In a trace held at a commanded 100 mm, the carton hung at 93.5 mm,
fell, landed flat (tilt under 1.5 degrees in flight) and stayed. The demo commands 106 mm to make up the
difference.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

Single states: set down flat (0.2), turned 4 degrees (0.2), exactly 5 degrees off flat (0.2), 6 degrees off flat,
on an end or side, held 1.5 mm up, overhanging, sliding, off the pad (0); tilted 5 degrees at the pad's edge with the
resting face's corners 1 mm inside (0.2) or 1 mm over (0). Simulated step-by-step drops at 20 Hz: from 100, exactly
75 and exactly 150 mm (1.0); 74, 155, 55 and 240 mm (0.5); 40 and 30 mm (0.2); 260 mm (0); landing 8 degrees off flat
or overhanging (0); a 6 mm bounce on landing (still 1.0); lowered at 5 cm/s (0.2); still in the gripper, driven down
at 0.7 g onto the pad, or jabbed down 60 mm at 1 g and braked (0.2: set down); tossed up to a 100 mm apex (1.0,
accepted); a good drop then picked up and set down again (0.2); a good drop then pushed 8 mm (0); a good drop then
turned 4 degrees in place (still 1.0). The single-state cases only show landings with no drop seen; the checker
does not return to the start pose between them, so they are ordered to avoid disturbing an earlier landing.
