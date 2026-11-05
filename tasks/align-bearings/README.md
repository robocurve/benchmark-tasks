# Line up two bearings with a motor's shaft

A motor is fixed on the table, turned 30 degrees, with only a short stub of its shaft showing. Two pillow-block
bearings stand off to the side, each turned. The robot sets them on the shaft's line, one about 10 cm and one about
20 cm out from the motor, each centred on the line within 3 mm and square to it within 2 degrees, so a straight shaft
would pass through the motor and both bores. Nothing marks the line beyond the stub, so the skill is carrying a line
out by eye and matching both position and angle to it. It is the first step of fitting a drive shaft, which
millwrights do before the shaft goes in.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=align_bearings.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Line up the two bearings with the motor's shaft: set one about 10 cm and one about 20 cm out from the motor's front face, each centred on the shaft's line and square to it, so a straight shaft would pass through both." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/align-bearings-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=align_bearings.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Line up the two bearings with the motor's shaft: set one about 10 cm and one about 20 cm out from the motor's front face, each centred on the shaft's line and square to it, so a straight shaft would pass through both." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/align-bearings-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/align-bearings-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once both bearings have been in line for 1 s;
otherwise it scores the partial credit below at the last step.

## Scoring
The shaft's line runs from the stub straight out along the motor's axis. A bearing is in line when it stands on its
base (within 5 degrees, its lowest point within 3 mm of the table), still, with its centre within 3 mm of the line and
its bore within 2 degrees of square to it. There are two places along the shaft, 8 to 12 cm and 18 to 22 cm out from
the motor's front face, and either bearing can go in either; the bearings are matched to the places whichever way
scores more.

| Credit | For |
|---|---|
| 0.5 | each place with a bearing in line (within 3 mm and 2 degrees) |
| 0.25 | each place with a bearing roughly in line (within 10 mm and 6 degrees) |
| 0.75 | both in line, but the line through the two bearings' centres more than 1 degree off the shaft |
| 0 overall | a bearing off the table (its lowest point more than 1 cm below the table top) |

Both in line, with the line through them within 1 degree of the shaft: 1.0, so a straight shaft through both bores
would meet the motor's. Still means moving under 1 cm/s and turning under 0.5 rad/s. On a real table, a string pulled
straight out from the shaft stub shows the line; a ruler measures each bearing's offset from it, and a square held
against the string checks the 2 degrees.

## The scripted demo
The left arm moves the bearings one at a time. It grips each across its 3 cm thickness, with the grip turned to match
the bearing and the fingers aimed 22 mm apart so the grip centres it, lifts it 4 cm above every top, turns it square
to the shaft while carrying it, and sets it down slowly on the shaft's line, 10 and 20 cm out from the motor. It lets
go gently and leaves straight up.

## Runtime and seeds
- The scripted demo's plan takes 27 s of simulated time (544 steps at 20 Hz), within the 2,400-step limit. On an L40S,
  an official demo run takes about 70 s of wall clock and a Dream-1 run about 300 s (the full 2,400 steps), start-up
  included; making the videos takes a little longer.
- The scoring and reach checks run without the simulator: `python -m pytest -q test_align_bearings.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
