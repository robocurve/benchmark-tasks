# Set a row of tiles with 3 mm joints

Four tiles lie beside a straightedge fixed to the table, each turned 10 to 30 degrees from square. The robot lays them in one row along the straightedge, each
tile against it and square to it, with a 3 mm joint between neighbours (2 to 4 mm counts). Each tile is placed
relative to the one before it, so a small error in one joint carries into the next: the skill is even spacing, not
hitting a single mark. It is the first course of a tiled floor, which tile setters lay along a straightedge.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=set_tiles.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lay the four tiles in one row along the straightedge, each tile square to it and against it, with a 3 mm joint between neighbouring tiles." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/set-tiles-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=set_tiles.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lay the four tiles in one row along the straightedge, each tile square to it and against it, with a 3 mm joint between neighbouring tiles." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/set-tiles-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/set-tiles-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once all four tiles have been set with three good joints
for 1 s; otherwise it scores the partial credit below at the last step.

## Scoring
A tile is set when it lies flat on the table (tilted at most 2 degrees), is square to the straightedge (turned at most
2 degrees), touches the straightedge or is at most 1 mm from it, lies within the straightedge's length, and is still.

| Credit | For |
|---|---|
| 0.1 | each set tile (up to 0.4) |
| 0.2 | each joint between two neighbouring set tiles in the row that is 2 to 4 mm wide at its narrowest point (up to 0.6). The row is the set tiles on the side of the straightedge with the most of them; tiles set on the other side earn 0.1 each but nothing for their joints |
| 0 overall | any tile face down (tilted more than 135 degrees) or off the table; a tile on its edge is only not set |

All four set with three good joints: 1.0. On a real table, feeler gauges check the gap to the straightedge (at most
1 mm) and each joint (2 to 4 mm); a square held against the straightedge checks the 2 degrees; a photo from straight
above shows the row.

## The scripted demo
The left arm moves the tiles one at a time, farthest out first, straight across the straightedge into the row. It
grips each tile across its sides with the grip turned to match the tile, and turns it square while carrying it. Set
down, the fingers close along the straightedge's width and never go into a joint; the finger next to the straightedge
passes 4 mm above it. Each tile is set down 0.5 mm off the straightedge and 3 mm short
of the one before.

## Runtime and seeds
- The scripted demo's plan takes about 38 s of simulated time (770 steps at 20 Hz), within the 2,400-step limit.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
