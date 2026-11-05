# Sort kitchen waste by the local rules of Karlsruhe

Household waste separation in Germany follows federal law (packaging law, the electrical-equipment and battery laws)
and each city's waste statute, and every city sets its own bins. A robot clearing up a kitchen has to know them: here
the instruction names only the city, Karlsruhe, and says that the bins are colour-coded by its rules, and the robot has to sort six everyday items into four bins coloured
like the lids of Karlsruhe's household bins, without being told what goes where. Several items are chosen where the
obvious guess is wrong: a drink carton is not paper but composite packaging (red Wertstoffe bin), a ceramic mug is
residual waste, and a smartphone belongs in no household bin at all (small electrical devices go to a collection
point) and stays on the table.

The task is the task file `waste_sorting.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (YAM arms at the calibrated `station` in
Isaac Lab-Arena 0.3.1 / Isaac Lab 3.0 / Isaac Sim 6.1). It uses only the documented task-file objects (`Box` and
`Asset`, models from Arena's object library) and needs no other files or packages.

## Setup

Coordinates: x forward from the line between the two arm bases, y to the left, from the midpoint between the bases.

**Bins:** four open, square plastic bins, fixed to the table; inside 16 x 16 x 9 cm, 6 mm walls, each in one colour,
without labels. They are coloured like the lids of Karlsruhe's household bins (Team Sauberes Karlsruhe, "Was kommt
wohin", 2026):

| Bin | Colour | Centre (x, y) | Takes in Karlsruhe |
|---|---|---|---|
| paper | blue | (20, 36) cm | paper, cardboard, packaging made of them |
| organic | green | (40, 33) cm | food and kitchen waste |
| Wertstoffe | red | (20, -36) cm | plastic and metal, packaging of plastic, metal or composite material |
| residual | black | (40, -33) cm | everything else household bins take (e.g. ceramics, porcelain) |

**Items:** models from Arena's object library (`Asset`), standing on the table, sizes as spawned (x by y by z):

| Item (library model) | Size | Mass | Centre (x, y) | Belongs in |
|---|---|---|---|---|
| empty tin can, tomato soup (`tomato_soup_can_ycb_robolab`) | 6.6 cm diameter, 10 cm tall | 50 g | (15, 10) cm | red |
| banana, small (`banana_ycb_robolab`), lying, long side along x | 12 x 7.4 x 2.5 cm | 100 g | (25, 16) cm | green |
| smartphone (`smartphone_hot3d_robolab`), lying flat | 4.8 x 10 x 0.7 cm | 150 g | (34, 6) cm | none: stays on the table |
| empty drink carton, orange juice (`orange_juice_carton_hope_robolab`), standing | 4.7 x 4.6 x 12 cm | 30 g | (17, -14) cm | red |
| empty cardboard box, popcorn (`popcorn_box_hope_robolab`), standing on its long edge | 2.7 x 8 x 5.5 cm | 50 g | (30, -7) cm | blue |
| ceramic espresso mug (`ceramic_mug_hot3d_robolab`), handle along y | 5.8 x 7.6 x 5 cm | 150 g | (29, -18) cm | black |

**Why two arms:** an arm cannot reach in front of the other arm. The blue and green bins are in front of the left arm,
the red and black ones in front of the right arm. The tin can starts on the left half but belongs in the red bin on
the right; the popcorn box starts on the right but belongs in the blue bin on the left. Both have to change hands,
for example by setting them down on the centre line.

## Scoring

An item is **in a bin** when the centre of its bounding box is inside the bin's 16 x 16 cm opening and below its rim
(9.6 cm above the table).

- **+1/5** for each of the five waste items in its bin; **-1/5** for each waste item in a wrong bin, and **-1/5** if the
  smartphone is in any bin; at least 0, at the end of the trial.
- **1.0** once all five are in their bins, the smartphone is in none, and every item is at rest (under 1 cm/s and
  0.5 rad/s) for 1 s; the trial ends then.

Tipping every item into one bin scores 0, and so does doing nothing. The usual mistake, the carton in the paper bin,
scores 0.6.

## Running

Needs Linux with an NVIDIA RTX GPU (Isaac Sim needs RT cores) and about 200 GB of disk. Install the pinned
adapter, which installs Isaac Lab-Arena and Inspect Robots (from the root of this repository):

```bash
mkdir -p .deps
git clone https://github.com/kairosiann/yam-isaac-lab-arena.git .deps/yam-isaac-lab-arena
git -C .deps/yam-isaac-lab-arena checkout 0059d02a1a91bc6834c1d0846022891df0c78730
ARENA_DIR="$PWD/.deps/IsaacLab-Arena" .deps/yam-isaac-lab-arena/setup.sh
```

In each shell (the library models are downloaded from this asset root):

```bash
source .deps/IsaacLab-Arena/yam.env
export ISAACSIM_ASSET_ROOT=https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0
```

Dream-1 (from the repository root):

```bash
inspect-robots run --embodiment yam_arena --policy dream -E task=tasks/waste_sorting/waste_sorting.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "This kitchen is in Karlsruhe, Germany. The bins are colour-coded according to the local waste sorting rules. Sort the waste on the table into the bins where it belongs; leave anything that has no fitting bin on the table." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir runs/waste-sorting-dream
```

Scripted full-score demo: the same command with `--policy yam_scripted` (and `--log-dir runs/waste-sorting-demo`).
`inspect-robots video <log>.json` renders the stored frames to MP4. `WASTE_DEBUG=N` prints every N-th step where each
item is.

- **Run limit:** 2,400 steps (2 minutes at 20 Hz).
- **Seeds and layout:** one fixed starting layout, `-E object_jitter=0` (no seed needed). With jitter, seeded resets
  shift each item by up to 3 cm.
- **Runtime:** the scripted demo reaches full score in about 52 s simulated; the run takes about 2 minutes
  wall-clock on one RTX 5090, including Isaac Sim start-up and the first download of the library models.

The demo moves one arm at a time: the left arm puts the banana in the green bin and sets the tin can down on the
centre line; the right arm puts the carton and the mug in the red and black bins, sets the popcorn box down on the
centre line and puts the tin can in the red bin; the left arm puts the popcorn box in the blue bin. It is open loop,
hence `-E object_jitter=0`.

## Simulation approximations

The items are Arena's scanned library models, scaled to the sizes above, with the adapter's rubber-like friction (1.2
static, 1.0 dynamic) and the masses above; the drink carton collides as one convex hull (its own collision mesh
catches on the fingers). Bins are rigid boxes. On a real station: small kitchen sorting bins of this size in these
colours, and real items of about these sizes (a 200 ml drink carton, a small banana, an espresso cup).

## Media

Recorded with `-E cam_width=640 -E cam_height=360` (the Dream-1 run at Dream's 320 x 176) and rendered with
`inspect-robots video`:

- `media/setup.png`: the setup, top camera, first frame.
- `media/demo.mp4`: the scripted demo, full score (1.0): top camera above, the wrist cameras below.
- `media/dream.mp4`: the Dream-1 run, score 0.0, top camera.
