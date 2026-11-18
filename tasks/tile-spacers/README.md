# Insert spacers into a dry-laid tile patch without moving the tiles

Four loose 10 cm ceramic tiles lie on a backer board in a 2 x 2 square, already aligned with 3 mm joints.
The robot takes four 3 mm plastic spacers from two slotted blocks and stands one in each joint, pushed
down to the board, without moving any tile, then lets go.

The tiles are loose (dry-laid, no adhesive) and a spacer is exactly as thick as the joint, so a spacer that
comes down even a millimetre off the joint's line lands on a tile's edge or shoves the tile aside. The
two side joints run left to right and the far and near joints away from the robot, so each spacer has to
be turned to its joint before it goes in.

`task.py` sets out the scene, the scoring and the scripted demonstration. `geometry.py` gives the tiles'
eased edges and the spacers' chamfered edges. `runtime.py` is the embodiment: the standard `yam_arena`
one, with those shapes and their materials, and the grippers' positions passed to the scorer for the final
release check. `run.py` runs a trial. No installed package is modified.

## Runtime and installation

Linux with an NVIDIA RTX GPU. Install the YAM adapter for Isaac Lab-Arena at the pinned commit, from the
benchmark repository root:

```bash
TASK_DIR="$(pwd)/tasks/tile-spacers"
ADAPTER_DIR="$HOME/yam-arena-tile-spacers"
export ARENA_DIR="$HOME/IsaacLab-Arena-tile-spacers"
git clone https://github.com/kairosiann/yam-isaac-lab-arena.git "$ADAPTER_DIR"
git -C "$ADAPTER_DIR" checkout 0059d02a1a91bc6834c1d0846022891df0c78730
git clone --branch release/0.3.1 https://github.com/isaac-sim/IsaacLab-Arena.git "$ARENA_DIR"
git -C "$ARENA_DIR" checkout -B release/0.3.1 c7edf3a9d5d8276565208f72422f8f8a0806d6cf
"$ADAPTER_DIR/setup.sh"          # uses the Arena checkout above and installs everything
source "$ARENA_DIR/yam.env"
cd "$TASK_DIR"
```

The reference environment: Python 3.12, Isaac Sim 6.1.0.0, Isaac Lab 17.0.2, Isaac Lab-Arena 0.3.0
(release/0.3.1 at c7edf3a9d5), `yam-arena` 0.1.0, `inspect-robots` 1.0.0, `inspect-robots-dream` 0.1.0
and `dreamlabs-client` 1.1.3. The task uses the adapter unmodified, and the CPU checks pass against a clean
checkout of the pinned commit. The task uses only primitive and procedural geometry, so it needs no asset
downloads beyond the robot and station that the adapter installs. Dream-1 needs a Dream Labs API key in
`~/.dreamlabs/credentials` (see the adapter's README).

## Run and score

```bash
python run.py demo      # scripted demonstration (yam_scripted, plays task.py's demo())
python run.py dream     # Dream-1 (dreamlabs/dream-1-beta)
```

Both read the instruction from `task.json` and run with seed 0, no object jitter, 20 Hz control, 240 Hz
physics and 320 x 176 policy camera frames, scored by `task_score`. The limit is 2,400 actions (120 s of
simulation); a full success ends the trial early. Logs go to
`$YAM_LOG_ROOT/tile-spacers/<mode>-<timestamp>` (default `~/yam-eval/...`), or `--log-dir DIR`. Add
`--headless false` to watch in the Isaac Sim window. Render the cameras of a trial with:

```bash
inspect-robots video /path/to/adhoc_LOG_ID.json
```

The log's JSON holds the score. `measurements.jsonl` records, every 5 control steps, each tile's corner
shift and lift, each spacer's pose and which joint (if any) it is seated in, the grippers' positions and
the running score. `source_manifest.json` holds the hashes of the Python files that ran.

## Results

Seed 0, the settings above, on one RTX GPU. `demo_result.json` and `dream_result.json` hold each trial's
final measurements and object poses, and the hashes of the source files that ran. They match this folder.

| Policy | Score | Actions | Simulated time | Wall time | End |
| --- | --- | --- | --- | --- | --- |
| Scripted demo (`yam_scripted`) | 1.0 | 782 | 39.1 s | 89 s | success: all four joints filled; tiles moved at most 0.04 mm |
| Dream-1 (`dreamlabs/dream-1-beta`) | 0.0 | 2,400 | 120 s | 258 s | step limit: no spacer left its slot (the most any moved was 0.09 mm), no tile moved |

In the demo the arms take turns: each lifts a spacer straight out of its slot, turns it to the joint over the
patch, lowers it onto the glaze, pushes it down into the joint, and lets go, while the other arm waits with its
next spacer over its block.

## The scene

World frame, metres: X forward from the arm bases, Y to the robot's left, Z up. Two 6-DoF YAM arms with
parallel grippers stand at (0, +0.305, 0.750) and (0, -0.305, 0.750), starting at joint angles
[0, 1, 1, -0.6, 0, 0] rad with their grippers open. The station's oak-coloured table top is at Z = 0.7475.

| Object | What | Where |
| --- | --- | --- |
| Backer board | fixed; grey fibre-cement board, 300 x 300 x 12 mm | centred at (0.270, 0); top at Z = 0.7595 |
| 4 tiles | loose; glazed white ceramic tiles, 100 x 100 x 7 mm, 150 g, top edges eased by a 0.8 mm 45-degree chamfer | dry-laid square to the axes, 3 mm apart; centres at X = 0.2185 / 0.3215, Y = +-0.0515 |
| 4 spacers | loose; blue polypropylene tabs, 25 x 3 x 20 mm (length x thickness x height), 1.5 g, all four long edges chamfered 0.8 mm | one upright in each slot, at the slot's middle (X = 0.270), length along X, bottom on the slot floor |
| 2 spacer blocks | fixed; light pine, 60 (X) x 70 (Y) x 18 mm, each with two 4 mm wide, 8 mm deep slots through its length along X, 30 mm apart | centred at (0.270, +-0.215); slots at Y = +-0.200 and +-0.230 |

The four joints, each 3 mm wide and 100 mm long between two tile edges: *far* along X at Y = 0, from
X = 0.2715 to 0.3715; *near* along X at Y = 0, from X = 0.1685 to 0.2685; *left* along Y at X = 0.270, from
Y = 0.0015 to 0.1015; *right* along Y at X = 0.270, from Y = -0.1015 to -0.0015. A spacer stands 12 mm proud
of its block and, seated on the board, 13 mm proud of the glaze. Nothing else is on the table. The
station's camera stand, top camera, wrist cameras and lights are those of the pinned adapter
(`station.py`, `scene.py`). The uploaded setup image is the top camera's view after the reset.

Building it for real: cut the board from 12 mm fibre-cement backer board; use 100 x 100 mm glazed
ceramic tiles with cushion edges (about 7 mm thick); cut the spacers from 3 mm polypropylene sheet
and ease their long edges with a file; saw two 4 mm kerfs, 8 mm deep, into an 18 mm pine block. Before the
first trial, lay the tiles with 3 mm spacers as gauges, remove the gauges, and pencil each tile's corners
on the board.

## Scoring

The final state is scored, so a reviewer needs only a photo from above and a ruler.

- **Layout kept:** every tile corner is within 1 mm (a third of the joint) of its pencil mark, and no tile
  corner is more than 1 mm above the board (no tile tipped up on a spacer).
- **Seated spacer:** the spacer stands in a joint (its centre within 1.5 mm of the joint's centre line,
  its faces within 10 degrees of the tiles' edges), its whole length lies between the two tiles' edges
  (to within 1 mm), and its bottom is within 1 mm of the board, which a ruler reads as its top standing no
  more than 14 mm proud of the glaze. A spacer standing on its end in a joint also counts.
- **Released:** both grippers' centre points (between the finger pads) are at least 30 mm from every
  spacer.

| Outcome | Score |
| --- | --- |
| Any tile moved or tipped beyond the layout tolerance | 0 |
| Layout kept: 0, 1, 2, 3 or 4 joints holding a seated spacer (two in one joint count once) | 0.2 per joint |
| Layout kept, all four joints filled, both grippers released, every tile and spacer still, for 1 s | 1.0 (ends the trial) |

Spacers left in their blocks, lying on the tiles or the board, or stuck part-way into a joint score
nothing and cost nothing. A spacer forced in by shoving a tile aside fails the layout check, which
removes all credit. The score at the end of the trial is reported (`SCORE_OVER = "end"`), so moving a
tile and putting it back on its marks is not penalised. In simulation "still" means every tile and
spacer moving under 1 cm/s and turning under 0.5 rad/s.

## Fidelity

PhysX at 240 Hz, YAM control at 20 Hz, rigid bodies at real size and mass. The tiles and spacers collide
as their convex hulls, eased and chamfered edges included; the board and the blocks are fixed boxes.
Friction is estimated, not measured: tile and board 0.7 static / 0.6 sliding, spacer 0.5 / 0.4, finger
pads 0.5 (PhysX averages the two bodies at each contact). The spacers use 16 solver position iterations.
Simplified: the joints are exactly 3.0 mm and the spacers exactly 3.0 mm thick (real tiles and spacers
vary by a few tenths of a millimetre); the spacer is a flat tab, not a cross; the spacer does not flex;
the board is perfectly flat and the tiles have no lippage.

## Reset

Lay the tiles back on their pencil marks (3 mm spacers as gauges, then removed) and stand a spacer in the
middle of each slot. Nothing is consumed. The simulator's reset restores every pose and settles for 0.5 s.

## CPU checks

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests
```

They check the hull shapes, the starting layout, every scoring outcome and the shortcuts that must not
score (a spacer only started into a joint, lying over a joint, at the crossing, hanging past the patch,
turned across a joint; a moved, turned or tipped tile; a gripper still on a spacer), and that the
scripted plan is reachable, keeps the grippers apart and ends inside the limit.

## Assets and licenses

- YAM robot meshes and wrist mounts: I2RT geometry bundled by the pinned adapter, MIT License
  (`yam_arena/assets/yam/LICENSE.i2rt`).
- Station room, table, camera stand, lights and cameras: the pinned adapter and Isaac Lab-Arena
  (Apache-2.0); keep their notices.
- Board, tiles, spacers and blocks: primitive and procedural geometry in this folder; no third-party
  meshes or textures.
