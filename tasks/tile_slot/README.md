# Slide the right tile into a tight gap

A 4 x 4 grid of coloured tiles, glued to the table, has one gap in its edge. The robot has to choose which of three
loose tiles to put in the gap, so that the largest single-colour region of the grid becomes as large as possible,
and then fit it in. The choice takes counting connected regions, not matching a neighbour's colour: the biggest
region next to the gap is blue, but the best choice is red, which joins two smaller red regions into one.

Fitting the tile is a tight insertion. The gap leaves 2.5 mm on each side of the tile, less than the thickness of a
YAM fingertip (about 4 mm at its end), so the tile cannot be held while it goes in. It has to be set down outside
the gap, lined up, and slid in from the gap's open side, as when fitting a part into a tight recess or a drawer.

The task is the task file `tile_slot.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (YAM arms at the calibrated `station` in
Isaac Lab-Arena 0.3.1 / Isaac Lab 3.0 / Isaac Sim 6.1). It needs no other files or packages.

## Setup

Coordinates: x forward from the line between the two arm bases, y to the left, from the midpoint between the bases.
All tiles are painted wood, 3 cm thick, lying flat and axis-aligned; colours are red, blue and yellow.

Grid: fifteen tiles, 6.1 x 6.1 x 3 cm, glued to the table, centres 6.3 cm apart (2 mm between neighbours): rows at
x = 37, 30.7, 24.4 and 18.1 cm (row 1 furthest from the arms), columns at y = 26, 19.7, 13.4 and 7.1 cm (column 1 on
the arms' left). Row 3, column 1 is empty: the gap, open towards the arms' left (+y).

| | column 1 (y 26) | column 2 (y 19.7) | column 3 (y 13.4) | column 4 (y 7.1) |
|---|---|---|---|---|
| row 1 (x 37) | red | red | yellow | yellow |
| row 2 (x 30.7) | red | blue | blue | yellow |
| row 3 (x 24.4) | gap | blue | blue | blue |
| row 4 (x 18.1) | red | red | red | yellow |

The gap is 6.5 cm wide in x, between the tiles in rows 2 and 4, so a 6 cm tile in it has 2.5 mm on each side; it is
6.3 cm deep, from the tile in row 3, column 2 to the grid's edge.

Loose tiles, 6 x 6 x 3 cm, 65 g each, lying flat on the table in front of the right arm:

| Tile | Centre (x, y) |
|---|---|
| blue | (17, -25) cm |
| red | (26, -25) cm |
| yellow | (35, -25) cm |

The largest single-colour region (tiles joined by shared edges) once the gap holds each tile:

| Tile in the gap | Largest region |
|---|---|
| yellow | 5 (the blue region; the yellow tile joins nothing) |
| blue | 6 (the blue region plus the new tile) |
| red | 7 (red 3 in the top-left corner, red 3 in row 4, and the new tile) |

**Why two arms:** an arm cannot reach in front of the other arm. The loose tiles lie in front of the right arm, which
cannot reach the gap; the left arm can reach the gap but not the loose tiles. The chosen tile has to change hands,
for example by setting it down on the centre line.

## Scoring

A loose tile is **over** the gap when its centre is within 3.15 cm of the gap's centre in x and in y (over the gap's
cell), at any height. It is **seated** when its centre is within 1 cm of the gap's centre (horizontally), its
bottom is within 5 mm of the table, it is 3 cm tall within 5 mm (lying flat) and it is at rest (under 1 cm/s and
0.5 rad/s).

| Outcome at the end of the trial | Score |
|---|---|
| the red tile seated in the gap, and no other loose tile over it | 1.0 |
| the blue tile seated in the gap, and no other loose tile over it | 0.5 |
| the yellow tile seated in the gap, and no other loose tile over it | 0.25 |
| anything else: the gap empty, a tile not seated (jammed, tilted, stacked or short of the gap's centre by 1 cm or more), or two loose tiles over the gap | 0 |

The score is read at the end of the trial; the trial does not end early.

## Running

Needs Linux with an NVIDIA RTX GPU (Isaac Sim needs RT cores) and about 200 GB of disk. Install the pinned
adapter, which installs Isaac Lab-Arena and Inspect Robots (from the root of this repository):

```bash
mkdir -p .deps
git clone https://github.com/kairosiann/yam-isaac-lab-arena.git .deps/yam-isaac-lab-arena
git -C .deps/yam-isaac-lab-arena checkout 0059d02a1a91bc6834c1d0846022891df0c78730
ARENA_DIR="$PWD/.deps/IsaacLab-Arena" .deps/yam-isaac-lab-arena/setup.sh
```

In each shell:

```bash
source .deps/IsaacLab-Arena/yam.env
export ISAACSIM_ASSET_ROOT=https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0
```

Dream-1 (from the repository root):

```bash
inspect-robots run --embodiment yam_arena --policy dream -E task=tasks/tile_slot/tile_slot.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Slide one loose tile into the gap so that the largest single-colour region is as large as possible." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir runs/tile-slot-dream
```

Scripted full-score demo: the same command with `--policy yam_scripted` (and `--log-dir runs/tile-slot-demo`).
`inspect-robots video <log>.json` renders the stored frames to MP4.

- **Run limit:** 2,400 steps (2 minutes at 20 Hz).
- **Seeds and layout:** one fixed starting layout, `-E object_jitter=0` (no seed needed). With jitter, seeded resets
  shift each loose tile by up to 3 cm.
- **Runtime:** the scripted demo finishes its plan after 863 steps (about 43 s simulated) and stops with full score;
  the run takes about 2 minutes wall-clock on one RTX 5090, including Isaac Sim start-up.

The demo: the right arm sets the red tile down on the centre line and moves aside. The left arm sets it down just
outside the gap, grips it again from the front and back (the parallel fingers square it up and centre it on the
gap's row), slides it to 5 mm short of the gap's mouth, lets go, and pushes it in with closed fingers. It is open
loop, hence `-E object_jitter=0`.

## Simulation approximations

Tiles are rigid boxes with the adapter's default contact settings; the glued tiles are fixed bodies.

## Media

Recorded with `-E cam_width=640 -E cam_height=360` (the Dream-1 run at Dream's 320 x 176) and rendered with
`inspect-robots video`:

- `media/setup.png`: the setup, top camera, first frame.
- `media/demo.mp4`: the scripted demo, full score (1.0), top camera.
- `media/dream.mp4`: the Dream-1 run (`dreamlabs/dream-1-beta`, all 2,400 steps), score 0.0, top camera.
