# Put first-aid supplies within wheelchair reach

A 1:10 wall has three ledges side by side: high (53 in at scale), middle (31.5 in) and low (4.7 in). The first-aid kit
starts on the high ledge and the bandages on the low one. The robot has to put both where a wheelchair user can reach
them. The instruction names the ADA standards, not the heights: 15 to 48 in above the floor (2010 ADA Standards,
sections 308.2.1 and 308.3.1). The low ledge looks easy to reach from a seat, and it is out of range too.

## Run a trial
On a machine set up with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s
`setup.sh`), with a Dream API key in `~/.dreamlabs/credentials`:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=wheelchair_reach.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the first-aid kit and the bandages where a wheelchair user can reach them, as the ADA accessibility standards define it. The wall and its ledges are at 1:10 scale." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/wheelchair-reach-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=wheelchair_reach.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the first-aid kit and the bandages where a wheelchair user can reach them, as the ADA accessibility standards define it. The wall and its ledges are at 1:10 scale." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/wheelchair-reach-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/wheelchair-reach-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends with 1.0 when `success()` has held for 1 s; otherwise it scores
`score()` at the last step.

## Scoring
An item is within reach when all of these hold:
- the whole of it is 15 to 48 in above the floor at scale: 38 to 122 mm above the table, which stands for the floor;
- it stands on a ledge or on the other item: its bottom within 2 mm of that surface, its centre over it;
- it lies on any face, within 10 degrees of flat.

Both start out of range: the kit on the high ledge spans 53 to 62 in, the bandages on the low ledge 5 to 13 in. Under
any reading of which part of an item is reached for, both are out.

| Score | Condition | Check on a real table |
|---|---|---|
| 1.0 | both items within reach and still for 1 s (ends the trial) | a ruler from the table to each item's bottom |
| 0.5 | one item within reach | |
| 0 | neither: each on the high ledge, the low ledge, the table or held in the air | |

Only the middle ledge is in range: an item standing on it spans 31.5 to 40 in, or 43 in on its 30 mm side. Stacked,
the top item reaches 48.8 in or more, out of range. `score()` counts an item while it moves; `success()` also needs
both still. The hooks don't see the gripper; an item held in the air doesn't count, but one held while standing on a
ledge does.

## Runtime and seeds
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
