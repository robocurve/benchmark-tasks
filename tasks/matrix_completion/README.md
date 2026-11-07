# Complete a 3 x 3 pattern matrix

A physical version of the matrix-completion items in non-verbal reasoning tests (Raven's progressive matrices). A
3 x 3 grid of plates stands on the left half of the table; seven hold a fixed piece, and the two in the middle row on
the right are empty. Each piece has a shape (cube, cylinder or bar) and a colour (red, blue or yellow). Four loose
pieces lie in front of the right arm. The robot has to infer the rule from the grid and put the right loose piece on
each empty plate. Nothing but the grid says which pieces go where.

The rule: in every row and every column, each shape and each colour appears exactly once. The middle row alone
leaves two answers open (red bar and yellow cube, or red cube and yellow bar); only the columns decide between them.
The rule reads the same from any side of the grid, mirrored or not.

The task is the task file `matrix_completion.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (YAM arms at the calibrated `station` in
Isaac Lab-Arena 0.3.1 / Isaac Lab 3.0 / Isaac Sim 6.1). It needs no other files or packages.

## Setup

Coordinates: x forward from the line between the two arm bases, y to the left, from the midpoint between the bases.
All pieces are painted wood, 5 cm tall and upright: a cube is 5 x 5 cm, a cylinder 5 cm across, a bar 7 cm along x by
3 cm along y. Colours are red, blue and yellow.

Grid: nine grey painted wooden plates, 8 x 8 x 0.5 cm, taped to the table, centres 9 cm apart: rows at x = 38, 29
and 20 cm (row 1 furthest from the arms), columns at y = 32, 23 and 14 cm (column 1 on the arms' left). The fixed
pieces are glued to the centres of their plates:

| | column 1 (y 32) | column 2 (y 23) | column 3 (y 14) |
|---|---|---|---|
| row 1 (x 38) | red cube | yellow cylinder | blue bar |
| row 2 (x 29) | blue cylinder | empty | empty |
| row 3 (x 20) | yellow bar | blue cube | red cylinder |

Loose pieces, 60 g each, standing on the table:

| Piece | Centre (x, y) |
|---|---|
| red cube | (17, -21) cm |
| red bar | (17, -30) cm |
| yellow cube | (26, -21) cm |
| yellow bar | (26, -30) cm |

**Answer:** the red bar on row 2, column 2, and the yellow cube on row 2, column 3.

**Why two arms:** an arm cannot reach in front of the other arm. The loose pieces lie in front of the right arm,
which cannot reach the empty plates; the left arm can reach the plates but not the loose pieces. The pieces have to
change hands, for example by setting them down on the centre line.

## Scoring

A loose piece is **on** an empty cell when its centre is within 4 cm of the cell's centre in x and in y (over its
plate), at any height. A cell is **filled correctly** when:

- its answer piece is on it, standing upright on the plate (bottom within 1 cm of the plate's top, height 5 cm
  within 1 cm) and at rest (under 1 cm/s and 0.5 rad/s), and
- no other loose piece is on that cell.

The score is 0.5 for each correctly filled cell (0, 0.5 or 1.0), read at the end of the trial. The trial does not
end early on a correct answer. Two loose pieces on the same cell, a decoy on either cell, a piece lying on its side
or anything off the plates score 0 for that cell.

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
inspect-robots run --embodiment yam_arena --policy dream -E task=tasks/matrix_completion/matrix_completion.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Complete the pattern: put the right pieces on the two empty squares." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir runs/matrix-completion-dream
```

Scripted full-score demo: the same command with `--policy yam_scripted` (and `--log-dir runs/matrix-completion-demo`).
`inspect-robots video <log>.json` renders the stored frames to MP4.

- **Run limit:** 2,400 steps (2 minutes at 20 Hz).
- **Seeds and layout:** one fixed starting layout, `-E object_jitter=0` (no seed needed). With jitter, seeded resets
  shift each loose piece by up to 3 cm.
- **Runtime:** the scripted demo finishes its plan after 909 steps (about 45 s simulated) and stops with full score;
  the run takes about 3 minutes wall-clock on one RTX 5090, including Isaac Sim start-up.

The demo: the right arm sets the red bar and the yellow cube down on the centre line and moves aside, then the left
arm puts each on its cell. It is open loop, hence `-E object_jitter=0`.

## Simulation approximations

Pieces are rigid primitives (boxes and cylinders) with the adapter's default contact settings; the plates and the
glued pieces are fixed bodies.

## Media

Recorded with `-E cam_width=640 -E cam_height=360` (the Dream-1 run at Dream's 320 x 176) and rendered with
`inspect-robots video`:

- `media/setup.png`: the setup, top camera, first frame.
- `media/demo.mp4`: the scripted demo, full score (1.0), top camera.
- `media/dream.mp4`: the Dream-1 run (`dreamlabs/dream-1-beta`, all 2,400 steps), score 0.0, top camera.
