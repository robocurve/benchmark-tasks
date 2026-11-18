# Hand a spirit level across a bench wider than either arm's reach and drop it into a deep, narrow caddy slot

> Put the spirit level away in the slot of the wooden tool caddy.

A 30 cm spirit level lies on a tray at the far left end of the bench; the wooden tool caddy it is stowed in stands at
the far right end. The two are farther apart than either arm reaches, so the level has to change hands on the way,
in the air or by setting it down in between. The caddy's slot is deep and narrow: its 10 cm walls hide much of a
level lying in it from the overhead camera, so the drop into it is best guided by the wrist cameras.

## Run a trial

From the repository root, in the simulator's environment (Isaac Lab-Arena with the `yam_arena` embodiment, as in the
program's simulation walkthrough), with a Dream Labs key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/stow-level-across-wide-bench/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the spirit level away in the slot of the wooden tool caddy." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/stow-level-dream
```

The scripted full-score demonstration (both arms, open loop):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/stow-level-across-wide-bench/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the spirit level away in the slot of the wooden tool caddy." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/stow-level-demo
```

The run prints the trial's score (`task_score`), and the log keeps it. Render the videos with
`inspect-robots video <log>.json`.

Offline checks, without the simulator (layout, reach, occlusion, the demo's inverse kinematics, and the scoring on a
kinematic replay of the demo plus negative controls):

```bash
python tasks/stow-level-across-wide-bench/check_task.py
```

## Runtime and seeds

Runtime per trial: the scripted demonstration takes about 503 control steps (25 s of simulated time at 20 Hz); a
Dream-1 trial runs until the level has been stowed for 1 s or the run limit of 2,400 steps (2 minutes).

Seeds: none are needed. `-E object_jitter=0` keeps every object exactly where `task.py` places it, so every trial
starts from the same layout as the screenshot.

## The layout

Positions are measured on the table from the point midway between the two arm bases, which stand 61 cm apart:
forward means away from the arms, left and right are as seen from behind the arms.

- **Spirit level** (`spirit_level`): an aluminium box-beam spirit level, 300 mm long, 25 mm thick and 50 mm tall,
  300 g, painted yellow. It stands on its 25 mm face on the tray, its length pointing forward, its centre 18 cm
  forward and 55 cm left (it spans 3 to 33 cm forward).
- **Inbound tray** (`inbound_tray`): a flat grey plastic tray 36 x 10 x 1.2 cm, its length pointing forward, its
  centre 18 cm forward and 55 cm left, taped to the table.
- **Tool caddy** (`caddy_floor`, `caddy_wall_near`, `caddy_wall_far`, `caddy_wall_inner`, `caddy_wall_outer`): 12 mm
  plywood. A floor 38.4 x 8.4 cm; on it, four walls 10 cm tall around a slot 36 cm long and 6 cm wide inside. Its
  length points forward and its centre is 18 cm forward and 55 cm right; it is taped to the table.

Nothing else is on the table.

**Why the level must change hands.** Each arm's tool point reaches at most about 79 cm from its base above the table,
in any pose (`check_task.py` samples each arm's joint limits). Every part of the level on its tray is at least 84 cm
from the right arm's base, and every point inside the slot at least 82 cm from the left arm's base.

**How much the walls hide.** From the station's top camera, the walls hide 28 to 67 % of a level lying in the slot,
depending on where across the slot it lies (45 to 78 % for Dream's lower "demo" top-camera variant). The
wrist cameras can see into the slot from above it.

## Scoring

The score is the highest milestone reached, each needing the ones before it:

- **0.25, lifted:** the level's lowest point at least 3 cm above the tray's top.
- **0.5, carried across:** the level's centre at least 30 cm right of the centre line between the arms while its
  lowest point is at least 1 cm above the table (carried, not slid).
- **0.75, entering the slot:** the level's centre over the slot's inside footprint and its lowest point below the rim.
- **1.0, stowed:** the whole level lies inside the slot, seen from above (within 3 mm of the inner walls), entirely
  below the rim, still (under 1 cm/s and 0.5 rad/s), and released (neither gripper's tool point within 2.5 cm of
  it), held for 1 s. The trial then ends with full score.

The score is 0 if the level falls off the table (its lowest point more than 3 cm below the table top) at any time.
A level leaning in the slot against a wall still counts as stowed if it is entirely below the rim; a level wedged
upright or sticking out above the rim scores 0.75.

`task.py` implements these rules: `score()` reports the milestone each step and `success()` the stowed state; the
trial's score is 1.0 on success, otherwise the score at the end of the trial.

## Scoring a real trial

Mark a tape line on the table 30 cm right of the centre line between the arms. Film the trial from the front of the
table, level with its top, and from above. Lifted: the level's underside clears the tray by 3 cm. Carried across: the
level's centre passes the tape line while clear of the table. Entering: the level is over the slot and partly below
its rim. Stowed: after the arms let go, the level rests inside the slot with no part above the rim, for 1 s. A level
that ends on the floor scores 0.

## The demonstration

Scripted, open loop, straight-down grips with the fingers closing across the level's 25 mm thickness. The left arm
lifts the level by its rear part (10 cm behind its centre) and carries it to the centre line, 10 cm above the table;
the right arm takes it near its balance point (4 cm ahead of its centre), 14 cm from the left hand, so it hangs
steady in the grip; the left lets go and moves clear; the right carries it slowly over the caddy 4.5 cm above the rim,
pauses, lowers it until its finger tips are 8 mm above the rim, pauses again and lets go: the level, its lower 2 cm already between the walls, drops about 8 cm onto the slot's floor.
The right arm then rises and moves clear.

## Files

- `task.py`: the scene, the scoring and the scripted demonstration.
- `check_task.py`: offline checks (no simulator).

Diagnostics: set `STOW_TRACE=/path/trace.jsonl` before a run to log, every control step, the level's pose and both
grippers' tool points.
- `task.json`: the task's description for the task board.
