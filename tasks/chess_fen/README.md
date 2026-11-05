# Set up a chess position from its FEN

Setting up a board for a lesson, a puzzle or an adjourned game means reading a position in a standard notation and
putting each piece on its square. Here the instruction gives a position in Forsyth-Edwards Notation (FEN), the
standard text format for chess positions; the robot has to decode it (ranks from 8 down to 1, files a to h, digits
for empty squares, upper case white, lower case black), find the squares on an unlabelled board from the usual
orientation, tell the six piece types apart by shape, and leave the two pieces that are not in the position off the
board.

The task is the task file `chess_fen.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (YAM arms at the calibrated `station` in
Isaac Lab-Arena 0.3.1 / Isaac Lab 3.0 / Isaac Sim 6.1), plus `yampatch.py` beside it, which spawns the board and
the pieces (compound shapes the plain task-file format cannot express) as generated USD. It needs no other files
or packages.

## Setup

Coordinates: x forward from the line between the two arm bases, y to the left, from the midpoint between the bases.

**Board** (fixed, taped to the table): 25 x 25 x 1 cm, centred at (24.5, 0) cm; 8 x 8 squares of 2.8 cm, buff and
green (a vinyl tournament board), in a 1.3 cm off-white border, without coordinates printed on it. White sits at the
near side: rank 1 is nearest the arms (x 14.7 cm) and rank 8 furthest (x 34.3 cm); the a-file is on the left
(y 9.8 cm), the h-file on the right (y -9.8 cm). So a1 is a dark square and h1 a light one, as on every chessboard.

**Pieces** (Staunton style, turned wood, ivory or black; every piece has a 2.2 cm wide, 6 mm tall round base):

| Piece | Height | Shape above the base | Mass |
|---|---|---|---|
| pawn | 3.8 cm | thin column, collar, ball | 20 g |
| rook | 4.6 cm | thick column, wider top with four battlements | 30 g |
| knight | 5.2 cm | column, horse's head facing forward | 30 g |
| bishop | 6.0 cm | column, collar, round mitre with a small ball | 30 g |
| queen | 6.2 cm | column, collar, crown with a ball | 35 g |
| king | 7.1 cm | column, collar, crown with a cross | 40 g |

White knights face away from the arms, black ones towards them. At the start every piece stands on the table beside
the board, in two rows along x:

| Left of the board (y 17 cm) | x | Right of the board (y -17 cm) | x |
|---|---|---|---|
| white bishop | 15 cm | black knight | 13 cm |
| black queen | 20 cm | white king | 17 cm |
| white knight | 25 cm | black rook | 21 cm |
| white pawn | 30 cm | black pawn | 25 cm |
| | | white rook | 29 cm |
| | | black king | 33 cm |

**The position:** `6k1/8/1q3n2/4p3/3P4/2B5/8/4K2R w - - 0 1`, that is white king e1, rook h1, bishop c3, pawn d4;
black king g8, queen b6, knight f6, pawn e5. The white knight and the black rook are not in it.

**Why two arms:** an arm cannot reach in front of the other arm. Each arm reaches almost the whole board (the left arm
the a- to g-files, the right arm the b- to h-files) but only the pieces beside the board on its own side. The pieces
for the a- to d-files start on the left and those for the e- to h-files on the right, so each arm sets up its half
(or the pieces change hands).

## Scoring

A piece is **on its square** when its centre is within 1 cm (horizontally) of the square's centre, it stands on the
board (its bottom within 5 mm of the board's top) and it is upright (tilted less than 20 degrees).

- **1/8 for each piece of the position on its square, minus 1/8 for every other piece over the board** (centre over
  the 64 squares: a piece of the position elsewhere on the board, toppled, or the white knight or black rook), at
  least 0, at the end of the trial.
- **1.0** once all eight are on their squares, no other piece is over the board and every piece is at rest (under
  1 cm/s and 0.5 rad/s) for 1 s; the trial ends then.

Putting pieces on the board at random scores 0: every misplaced piece cancels a correct one. Knights may face any
way.

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
inspect-robots run --embodiment yam_arena --policy dream -E task=tasks/chess_fen/chess_fen.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set up this chess position, white at the near side: 6k1/8/1q3n2/4p3/3P4/2B5/8/4K2R w - - 0 1" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir runs/chess-fen-dream
```

Scripted full-score demo: the same command with `--policy yam_scripted` (and `--log-dir runs/chess-fen-demo`).
`inspect-robots video <log>.json` renders the stored frames to MP4.

- **Run limit:** 2,400 steps (2 minutes at 20 Hz).
- **Seeds and layout:** one fixed starting layout, `-E object_jitter=0` (no seed needed). With jitter, seeded resets
  shift each piece by up to 3 cm.
- **Runtime:** the scripted demo reaches full score in about 50 s simulated; the run takes about 2 minutes
  wall-clock on one RTX 5090, including Isaac Sim start-up.

The demo: both arms first set up the outer files at once (left: queen b6, bishop c3; right: rook h1, king g8), then
the left arm the pawn on d4 and the right arm king e1, pawn e5 and knight f6. It is open loop, hence
`-E object_jitter=0`.

## Simulation approximations

Pieces are rigid compounds of cylinders, spheres and boxes (a knight's head is three boxes) with a rubber-like
friction (1.2 static, 1.0 dynamic, as the adapter uses for its graspable objects) and a centre of mass 1 cm above
the bottom (weighted bases); the board is a fixed slab. On a real station any small Staunton set with these
dimensions works (about 2.2 cm bases on 2.8 cm squares; a "travel" or "mini" set).

## Implementation

The stock task-file format spawns only single boxes, cylinders and spheres. `yampatch.py` (beside the task file)
adds generated USD objects at runtime, without changing the adapter's files: `chess_fen.py` writes the board (one
fixed collider slab with 64 thin visual squares) and each piece (one rigid body made of several collision shapes)
as USD when the scene is built, and `yampatch.py` spawns them. It wraps `yam_arena.scene.task_object` and
`yam_arena.scene.object_states` of the pinned adapter (the task uses neither its `on_step` nor its `on_reset` hook).
Scoring reads only the pieces' poses and bounding boxes, as for any task-file object.

## Media

Recorded with `-E cam_width=640 -E cam_height=360` and rendered with `inspect-robots video`:

- `media/setup.png`: the setup, top camera, first frame.
- `media/demo.mp4`: the scripted demo, full score (1.0): top camera above, the wrist cameras below.
- `media/dream.mp4`: the Dream-1 run at Dream's settings (320 x 176), score 0.0.
