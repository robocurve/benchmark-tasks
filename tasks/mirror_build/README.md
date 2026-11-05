# Build the mirror image of a model

Assembly work often comes in left- and right-hand versions of the same part, and a worker builds one from a
sample of the other. Here a small model of four glued cubes stands on the left half of the table; the robot builds
its mirror image from four loose cubes on a grey build plate on the right half. The model is the manual: nothing
else says how the cubes go together, and the shape is chiral, so a copy of it, however it is turned, scores nothing.

The task is the task file `mirror_build.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (YAM arms at the calibrated `station` in
Isaac Lab-Arena 0.3.1 / Isaac Lab 3.0 / Isaac Sim 6.1). It needs no other files or packages.

## Setup

Coordinates: x forward from the line between the two arm bases, y to the left, from the midpoint between the bases.
All cubes are 4 cm painted wooden cubes, axis-aligned.

| Object | Mass | Where (centre x, y) |
|---|---|---|
| model, red cube, glued to the table | fixed | (32, 22) cm |
| model, blue cube, glued to the table and to red | fixed | (32, 18) cm |
| model, yellow cube, glued to the table and to blue | fixed | (28, 18) cm |
| model, green cube, glued on top of red | fixed | (32, 22) cm, 4 cm up |
| build plate, grey painted wood, 20 x 20 x 0.5 cm, taped to the table | fixed | (30, -20) cm |
| red cube | 50 g | (16, 20) cm |
| green cube | 50 g | (16, 10) cm |
| blue cube | 50 g | (16, -24) cm |
| yellow cube | 50 g | (14, -15) cm |

The model's cubes touch face to face: red and blue side by side across the table (red further left), yellow in
front of blue (towards the arms), green on red. Its mirror image has the same cubes with left and right swapped:
blue left of red, yellow in front of blue, green on red. No turn about the vertical makes the model and its mirror
image the same.

**Why two arms:** an arm cannot reach in front of the other arm. Red and green start in front of the left arm, blue
and yellow in front of the right arm, so red and green have to change hands to reach the build plate, for example
by setting them down on the centre line.

## Scoring

The build can stand anywhere on the build plate and be turned any way about the vertical.

- **0.5** for the base: red, blue and yellow stand on the plate (centres over the plate, within 1 cm of 2 cm above
  its top) and match the mirror image's base: after the best turn about the vertical and shift, each centre is
  within 1.5 cm of its place (red and blue, and blue and yellow, 4 cm apart, at a right angle, yellow on the mirror
  image's side).
- **1.0** for the base plus green on red (centre within 1.5 cm horizontally of red's, and 4 cm higher within 1 cm).
  Once that holds with every cube at rest (under 1 cm/s and 0.5 rad/s) for 1 s, the trial ends.
- **0** otherwise. Only turns are fitted, never reflections, so a copy of the model (yellow on the other side) scores
  0 at any position and angle, and so does anything built off the plate.

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
inspect-robots run --embodiment yam_arena --policy dream -E task=tasks/mirror_build/mirror_build.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Build the mirror image of the model on the grey plate, using the four loose cubes." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir runs/mirror-build-dream
```

Scripted full-score demo: the same command with `--policy yam_scripted` (and `--log-dir runs/mirror-build-demo`).
`inspect-robots video <log>.json` renders the stored frames to MP4.

- **Run limit:** 2,400 steps (2 minutes at 20 Hz).
- **Seeds and layout:** one fixed starting layout, `-E object_jitter=0` (no seed needed). With jitter, seeded resets
  shift each loose cube by up to 3 cm.
- **Runtime:** the scripted demo reaches full score in about 60 s simulated; the run takes about 2 minutes
  wall-clock on one RTX 5090, including Isaac Sim start-up.

The demo: the left arm sets red and green down on the centre line and moves aside, then the right arm builds the
mirror image on the plate, where the model mirrored across the centre line would stand. It is open loop, hence
`-E object_jitter=0`.

## Simulation approximations

Cubes are rigid boxes with PhysX default friction; the model's glued cubes and the taped plate are fixed bodies.

## Media

Recorded with `-E cam_width=640 -E cam_height=360` (the Dream-1 run at Dream's 320 x 176) and rendered with
`inspect-robots video`:

- `media/setup.png`: the setup, top camera, first frame.
- `media/demo.mp4`: the scripted demo, full score (1.0): top camera above, the wrist cameras below.
- `media/dream.mp4`: the Dream-1 run, score 0.0.
