# Fill the gaps in a seedling tray

A seedling plug tray has 12 cells; 9 hold green plugs and 3, scattered, are empty. Four spare plugs stand beside it. The
robot fills each empty cell with one spare, standing and centred, without moving any plug already in the tray, and
leaves the fourth spare out. The skill is finding the gaps in a pattern and fitting into them among close neighbours,
and doing only what is asked. It is the step nursery workers call gapping: filling the cells where a seed didn't come up
before a tray ships.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=seedling_tray.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fill the empty cells of the seedling tray with the spare plugs, one plug per cell, without disturbing the plugs already in the tray or knocking any plug over." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/seedling-tray-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=seedling_tray.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fill the empty cells of the seedling tray with the spare plugs, one plug per cell, without disturbing the plugs already in the tray or knocking any plug over." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/seedling-tray-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/seedling-tray-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once all three cells have been filled, with every plug
standing and still, for 1 s; otherwise it scores the partial credit below at the last step.

## Scoring
A plug is in a cell when it stands upright (within 10 degrees) with its bottom within 3 mm of the tray floor and its
centre inside the cell's walls, wherever it stands there.

| Score | When, at the end |
|---|---|
| 1/3 each | an empty cell holds a spare plug that is in it and still (all three: 1.0) |
| 0 overall | a plug already in the tray moved more than 5 mm or left its cell, or any plug knocked over or off the table |

Still means moving under 1 cm/s and turning under 0.5 rad/s. The sim can't tell a gripped plug from a released one,
so a spare held still up to 3 mm above a gap's floor counts; on a real tray, open the gripper before scoring. A look
from above shows which cells are filled.

## The scripted demo
The left arm moves three of the spares, one at a time, into the empty cells. It comes down with the fingers only 44 mm
apart, 7 mm clear of a 30 mm plug, so they stay 13 mm from the plugs beside each empty cell, and grips each plug across
its middle 1.2 cm below its top with the fingers aimed 22 mm apart, so the grip centres it. It carries each plug 2.7 cm
above every plug top, sets it down slowly with the pad tips 1.4 cm above the walls, and lets go gently. The fourth
spare and the plugs already in the tray are never touched.

## Runtime and seeds
- The scripted demo's plan takes 40 s of simulated time (804 steps at 20 Hz), within the 2,400-step limit.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
