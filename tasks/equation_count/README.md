# Count out the sum of an equation

Picking and kitting work often comes with a written quantity: a work order, a label or a recipe says how many parts go
into the tray. Here the quantity is not given as a plain number but as a small sum laid out on the table in flat black
disks, `2 + 3`. The robot must read the two digits, add them, and put exactly that many cubes onto a grey plate. The
equation is the manual: nothing else says how many cubes to move, and there are more cubes than the answer, so moving
all of them scores nothing.

The task is the task file `equation_count.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (YAM arms at the calibrated `station` in
Isaac Lab-Arena 0.3.1 / Isaac Lab 3.0 / Isaac Sim 6.1). It needs no other files or packages.

## Setup

Coordinates: x forward from the line between the two arm bases, y to the left, from the midpoint between the bases.

| Object | Mass | Where (centre x, y) |
|---|---|---|
| 27 flat disks, black painted wood, 2.4 cm diameter, 3 mm thick, glued to the table, forming the equation `2 + 3` | fixed | rows at x = 54, 51, 48, 45, 42 cm; columns at y = 15, 12, ..., -15 cm (see below) |
| plate, grey painted wood, 16 x 16 x 0.5 cm, taped to the table | fixed | (25, 0) cm |
| 8 cubes, red painted wood, 4 cm, axis-aligned | 50 g each | left: (14, 19), (14, 27), (22, 19), (22, 27) cm; right: (14, -19), (14, -27), (22, -19), (22, -27) cm |

The equation is three dot-matrix glyphs of 5 rows x 3 columns on a 3 cm grid, with one empty column between glyphs
(11 columns, centred on y = 0, read left to right from the arms' side, i.e. towards -y; row 1 is the far row at
x = 54 cm). `#` is a disk:

```
   "2"       "+"       "3"
  # # #     . . .     # # #      row 1, x = 54 cm
  . . #     . # .     . . #      row 2, x = 51 cm
  # # #     # # #     # # #      row 3, x = 48 cm
  # . .     . # .     . . #      row 4, x = 45 cm
  # # #     . . .     # # #      row 5, x = 42 cm
  columns 1-3: y = 15, 12, 9 cm; columns 5-7: y = 3, 0, -3 cm; columns 9-11: y = -9, -12, -15 cm
```

**Why two arms:** an arm cannot reach in front of the other arm. Four cubes start in front of each arm and the answer
is five, so neither arm can do it alone; the plate is on the centre line, where both arms reach.

## Scoring

A cube is **on the plate** when its centre is over the plate (inside its 16 x 16 cm outline, seen from above) and
it stands on the plate (its lowest point within 1 cm of the plate's top) or on another cube that is on the plate (its
lowest point within 1 cm of that cube's top, centres within 2 cm horizontally). A cube held in the air, leaning off
the plate's edge with its centre outside the outline, or lying on the table does not count. With n cubes on the
plate at the end of the trial:

- **1.0** for n = 5. Once that holds with every cube at rest (under 1 cm/s and 0.5 rad/s) for 1 s, the trial ends.
- **0.1 x n** for n = 0 to 4 (0.0, 0.1, 0.2, 0.3, 0.4).
- **0** for n = 6, 7 or 8.

The score is taken at the end of the trial, so adding a sixth cube after the fifth loses the credit. Where the cubes
stand on the plate, and which cubes are used, does not matter.

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
```

The scene uses only primitive shapes (boxes and cylinders) and the station, so it needs no extra asset settings.

Dream-1 (from the repository root):

```bash
inspect-robots run --embodiment yam_arena --policy dream -E task=tasks/equation_count/equation_count.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put as many cubes on the grey plate as the sum on the table." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir runs/equation-count-dream
```

Scripted full-score demo: the same command with `--policy yam_scripted` (and `--log-dir runs/equation-count-demo`).
`inspect-robots video <log>.json` renders the stored frames to MP4.

- **Run limit:** 2,400 steps (2 minutes at 20 Hz).
- **Seeds and layout:** one fixed starting layout, `-E object_jitter=0` (no seed needed). With jitter, seeded resets
  shift each loose cube by up to 3 cm; the disks and plate are fixed.
- **Runtime:** the scripted demo reaches full score in about 50 s simulated (996 steps); the run takes about 3 minutes wall-clock on one RTX 5090, including Isaac Sim start-up.

The demo: the left arm puts three of its cubes on the plate and moves aside, then the right arm puts two of its own.
It is open loop, hence `-E object_jitter=0`.

## Simulation approximations

Cubes are rigid boxes with PhysX default friction; the glued disks and the taped plate are fixed bodies. The disks
are flat cylinders 3 mm high, low enough for the fingers to pass over.

## Media

Recorded with `-E cam_width=640 -E cam_height=360` (the Dream-1 run at Dream's 320 x 176) and rendered with
`inspect-robots video`:

- `media/setup.png`: the setup, top camera, first frame.
- `media/demo.mp4`: the scripted demo, top camera, full score (1.0).
- `media/dream.mp4`: the Dream-1 run at Dream's settings (320 x 176), score 0.1.
