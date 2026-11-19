# Pick the ordered item from a deep tote into the order bin, leaving its look-alike

> Pick the red item from the tote into the order bin.

Each picking is a warehouse order picker's core operation: take exactly the ordered item out of a storage tote and put
it in the customer's order bin. Here the tote's 12 cm walls hide part of the ordered item from the overhead camera,
and an orange carton of the same size lies beside it. Picking the look-alike is a mis-pick, the error picking
processes exist to prevent, and scores zero.

## Run a trial

From the repository root, in the simulator's environment (Isaac Lab-Arena with the `yam_arena` embodiment), with a
Dream Labs key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/pick-ordered-item-from-deep-tote/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pick the red item from the tote into the order bin." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/pick-dream
```

The scripted full-score demonstration (left arm, open loop): the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/pick-demo`. Render videos with `inspect-robots video <log>.json`.

Offline checks, without the simulator (the layout, the camera occlusion, the demo's inverse kinematics, and the scoring
on a kinematic replay plus negative controls): `python tasks/pick-ordered-item-from-deep-tote/check_task.py`.

Diagnostics: set `PICK_TRACE=/path/trace.jsonl` before a run to log, every control step, both items' poses and both
grippers' tool points.

## Runtime and seeds

The scripted demonstration takes about 290 control steps (14.5 s at 20 Hz); a Dream-1 trial runs until the pick has
held for 1 s or the run limit of 2,400 steps (2 minutes). No seeds: `-E object_jitter=0` keeps every object exactly
where `task.py` places it, matching the setup screenshot.

## The layout

Positions are measured on the table from the point midway between the two arm bases, which stand 61 cm apart:
forward means away from the arms, left and right as seen from behind the arms.

- **Tote** (`tote_floor`, `tote_wall_*`): grey polypropylene, inside 24 cm (forward) x 18 cm (across), walls 12 cm
  tall and 12 mm thick on a 1 cm floor. Centre 25.5 cm forward, 28 cm left. Fixed to the table.
- **Red item** (`red_item`, the ordered item): a red cardboard product carton, 80 x 50 x 30 mm, 150 g, lying flat with
  its 80 mm side across. Centre 20 cm forward, 23.5 cm left: near the tote's front-right corner, 4 cm from the
  front wall and 5 mm from the right wall.
- **Orange item** (`orange_item`, the look-alike): an orange carton of the same size and mass, same orientation.
  Centre 32 cm forward, 23.5 cm left, 12 cm behind the red one.
- **Order bin** (`order_bin_floor`, `order_bin_wall_*`): blue polypropylene, inside 16 cm (forward) x 12 cm (across),
  walls 6 cm tall on a 1 cm floor. Centre 20 cm forward, on the centre line between the arms. Fixed to the table.

Nothing else is on the table. From the station's top camera, the tote's walls hide 27 % of the red item's top face
(32 % for Dream's lower "demo" top-camera variant); the wrist cameras see into the tote from above it.

## Scoring

The score is the highest milestone the red item reaches, each needing the ones before it:

- **0.25, lifted:** its lowest point at least 3 cm above the tote floor.
- **0.5, out of the tote:** its lowest point above the tote's rim, and its centre outside the tote's inside footprint.
- **0.75, in the bin:** its centre over the order bin's inside footprint and its lowest point below the bin's rim.
- **1.0, picked:** the red item lies entirely inside the order bin (seen from above, within 3 mm of the inner walls),
  entirely below its rim, still (under 1 cm/s and 0.5 rad/s) and released (no gripper tool point within 2.5 cm of
  it), with the orange item still in the tote, held for 1 s. The trial then ends. A pick completed less than 1 s
  before the run limit scores 0.75.

The score is **0 (mis-pick)** if the orange item is ever over the order bin, or is taken out of the tote and left
resting outside it, and 0 if the red item falls off the table.

## Scoring a real trial

Film from the front at table height and from above. Lifted: the red carton's underside 3 cm above the tote floor.
Out of the tote: it clears the rim and passes outside the tote. In the bin: over the bin and partly below its rim.
Picked: after the gripper lets go, it rests inside the bin below the rim for 1 s, and the orange carton is still in
the tote. The orange carton in the bin, or out of the tote, scores 0.

## The demonstration

Scripted, open loop, left arm only, a straight-down grip turned -90 degrees so the fingers close across the item's
50 mm width. The item sits far enough from the walls for the gripper's housing, which reaches 5.1 cm either side of
the tool point along the closing direction. The arm descends into the tote,
grips, lifts the item 4.5 cm clear of the rim, moves it clear of the tote, carries it over the order bin, lowers it
until its bottom is 1 cm below the bin's rim, lets go and moves clear. The right arm holds its ready pose.

## Files

- `task.py`: the scene, the scoring and the scripted demonstration.
- `check_task.py`: offline checks (no simulator).
- `task.json`: the task's description for the task board.
