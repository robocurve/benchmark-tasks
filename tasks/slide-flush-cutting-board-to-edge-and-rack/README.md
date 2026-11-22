# Slide a flush cutting board to the counter-edge overhang, lift it level and rack it

Slide a heavy flush-lying commercial cutting board straight out to a safe counter-edge overhang window — a
board lying flat has no graspable purchase until an edge hangs free in air, and pushing it past its 50%
balance point tips it off the block — pinch the exposed edges from below in free air, lift it level, and seat
it flat inside a full-size commercial dishwashing rack.

No existing benchmark task tests this ability:

- **#80** [FINAL, TK-4ES56Q] (*Set a paint tin on the overhanging end of a plank*): static cantilever mass
  balancing on an already-overhanging plank — not sliding a flush board into a controlled overhang window to
  lift level and rack it.
- **#2138** [FINAL, TK-PLTSLA] (*table-edge recovery safety*): a generic cube pushed back from a table edge —
  no real flat-surface ungraspability and no opposing-edge pinch / level carry into a dish rack.
- **#2166** [CLOSED, TK-SCJB8M] (*Put a chef's knife on a cutting board left hanging over the worktop edge*):
  starts from an already-overhanging board; not flush-board edge extraction and rack placement.

Instruction: "Slide the cutting board from the butcher block to the counter-edge overhang, lift it level from
below, and place it flat inside the dishwashing rack."

Files in this folder: `task.json`, `slide_flush_cutting_board_to_edge_and_rack.py` (scene, scorer, scripted
demo), `labkit.py` (multi-shape parts and step hooks), `arena_assets_fix.py` (library asset paths),
`pitch_demo.py` (yaw+pitch+roll waypoints), `test_slide_flush_cutting_board_to_edge_and_rack.py` (offline
tests), `README.md`.

## Environment
Isaac Lab-Arena 0.3.1 on Isaac Sim 6.1 (Brev launchable), `inspect-robots` with the `yam_arena` embodiment,
`station` scene. Task file SHA-256 (first 12): see the freeze log.

## Scene
- `butcher_block` (fixed): 508 x 381 x 57.2 mm reversible maple prep block, 9.07 kg, centred at
  (0.320, +0.3405) m; its top face is at 804.7 mm and its -Y front face at y = +0.150 m is the counter-edge
  cliff the board slides over.
- `board` (loose, 1.47 kg): rigid 457.2 x 304.8 x 12.7 mm white HDPE polyethylene cutting board, a single free
  rigid body lying flush flat on the block top at (0.320, +0.3224) m — its -Y edge at +0.170 m sits only
  20 mm back from the cliff, so nothing yet hangs over.
- `dish_rack` (fixed): 502 x 502 x 101.6 mm soft-gray polypropylene full-size base rack, 1.43 kg, at
  (0.635, -0.395) m — a 19 mm floor plate plus four 6 mm walls leaving an open inner cavity
  (x in [390, 880] mm, y in [-640, -150] mm) whose floor top is at 766.5 mm.

## How it is modelled
| Mechanism | Physical body / joint / contact | Source for the numbers |
|---|---|---|
| Cutting board | one loose rigid body, 457.2 x 304.8 x 12.7 mm, 1.47 kg | 18 x 12 x 1/2 in white polyethylene cutting board (Real-world build) |
| Prep block | fixed rigid body, 508 x 381 x 57.2 mm, 9.07 kg | 20 x 15 x 2-1/4 in reversible maple butcher block |
| Dish rack | fixed compound body: floor plate + four walls, 502 x 502 x 101.6 mm | 19-3/4 x 19-3/4 x 4 in full-size base dishwashing rack |
| Slide friction | PhysX average pair, HDPE board on maple block mu = 0.20 | HDPE-on-maple coefficient band (Real-world build) |
| Rack friction | board on polypropylene rack mu = 0.40 | PP board-on-rack coefficient |
| Edge pinches | finger-pad contacts on the slab's exposed -Y / -X edge lips | Stage-0 contact replay |

The board is loose — nothing welds or joints it — and the slide plus both opposing-edge pinches come only
from the finger pads.

## Scoring (see `score()`; text == code constants)
- 0.0 if the board tips past 15 deg; or drops onto the table outside the rack (underside below table top +
  12 mm); or sits inside the rack but is not seated flat (tilts past 6 deg, rides the walls without reaching
  the floor, or is still moving); or tilts past 10 deg anywhere; or stays flush on the block (its -Y edge
  under 30 mm past the counter edge); or either gripper is within 40 mm of it.
- 0.5 if the board is upright within 10 deg with its -Y edge slid more than 30 mm past the counter edge while
  it still lies over the block's span, or it has been lifted clear of the block; both grippers at least 40 mm
  clear; and it is not yet seated flat inside the rack.
- 1.0 if the board is completely inside the rack's inner cavity, flat within 6 deg with its underside seated
  on the rack floor (between 8 mm below and 12 mm above the floor top), at rest, and both grippers are at
  least 40 mm clear of it.

## Offline check (no GPU)
Run `PYTHONDONTWRITEBYTECODE=1 pytest -q -p no:cacheprovider test_slide_flush_cutting_board_to_edge_and_rack.py`
with `yam_arena` on `PYTHONPATH`: start / holding-still / flat-grab-resettled / over-push-tip-off all score
0.0, the safe overhang window and a lifted-clear board score 0.5, clean delivery scores 1.0, a board tilted
8 deg inside the cavity / resting on the wall tops / dropped on the table / still moving / a gripper inside
the 40 mm clearance all score 0.0; both demo plans are IK-reachable (< 2 mm, < 2 deg, max joint step
< 0.18 rad), the demo reads no simulator state, the scoring text matches the constants, and both top cameras
see every scored object with >= 40 mm sight-line clearance at the demo's actual end pose (URDF forward
kinematics of the final parked joint angles, plus a positive-control blocker test).

## Run and score one Dream-1 trial
```
inspect-robots run --embodiment yam_arena -E task=slide_flush_cutting_board_to_edge_and_rack.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy dream \
  --instruction "Slide the cutting board from the butcher block to the counter-edge overhang, lift it level from below, and place it flat inside the dishwashing rack." \
  --scorer task_score --max-steps 1200 --seed 0
```

## Replay the scripted demo
```
inspect-robots run --embodiment yam_arena -E task=slide_flush_cutting_board_to_edge_and_rack.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy yam_scripted \
  --instruction "Slide the cutting board from the butcher block to the counter-edge overhang, lift it level from below, and place it flat inside the dishwashing rack." \
  --scorer task_score --max-steps 1200 --seed 0
```
`demo()` reads geometry constants only. The left arm's closed finger pads slide the board 163 mm toward the
block's -Y cliff until the -Y edge hangs 55 mm into free air — short of the 50 % balance point — then it
releases and swings around. The right arm pinches the overhanging -Y edge from below in free air and pulls it
low while the rear stays supported on the block. The left arm comes down fully outboard of the block's -X
face, straddles the newly exposed -X edge at slab height and pinches it; both lift level to z = 0.925 with
matched displacement (the opposing grips bracket the board's centre of mass and split the 1.47 kg load,
~1.0 N m worst per gripper, under the 1.2 N m budget), carry it over the rack, lower diagonally onto the
floor plate, release, hover a closed settle check over the seated board inside the 40 mm clearance (the
undo-replay probe ends at that grasp and scores 0.0), then park outboard. The demo finishes early, at about
357 steps (17.9 s) into the trial.

## Demo results
| Check | Result |
|---|---|
| Offline MuJoCo physics check (`mj_check`), scripted demo | 1.0 at 357 steps (~17.9 s); `verdict` PASS, `ik_worst_mm` 0.094, `arm_table_N` 0, `tip_table_N` 5.2, `end_occluded_by_arms` {}, all adversarial replays 0.0, all robustness cases 1.0 |
| GPU simulator (cloud, isaacsim 6.1.0.0), scripted demo | 1.0 at 357 steps, ended by success |
| GPU simulator (cloud), Dream-1 with the instruction above | 0.0 over the full 1,200-step run |
| Offline MuJoCo, holding still at ready pose | 0.0 |
| MuJoCo jitter sweep, demo | 1.0 at jitter seeds 0-4 (+-5 mm, +-3 deg) |
| MuJoCo friction sweep, demo | 1.0 at friction x0.70 / x1.30 |
| MuJoCo `sweep` / `liftdrop` / `half` / `undo` shortcut replays | 0.0 each |

## Real-world build
One-time parts cost for one trial set: $198.93 (`real_trial_cost_usd`) -- the per-unit sum of board $6.99 +
butcher block $170.95 + rack $20.99, excluding the reusable counter structure.

- 18 x 12 x 1/2 in white polyethylene cutting board, 457.2 x 304.8 x 12.7 mm, 1.47 kg, $6.99 each --
  https://www.webstaurantstore.com/choice-18-x-12-x-1-2-white-polyethylene-cutting-board/40718125WH.html
- 20 x 15 x 2-1/4 in reversible maple wood prep block, 508 x 381 x 57.2 mm, 9.07 kg, $170.95 each --
  https://www.webstaurantstore.com/john-boos-ra02-20-x-15-x-2-1-4-reversible-maple-wood-cutting-board/650RA02.html
- 19-3/4 x 19-3/4 x 4 in soft-gray full-size base dishwashing rack, 502 x 502 x 101.6 mm, 1.43 kg, $20.99 each --
  https://www.webstaurantstore.com/cambro-br258151-camrack-soft-gray-full-size-base-dishwashing-rack/214BR258GY.html
- Friction citation: HDPE/polyethylene-on-maple coefficient band, nominal mu = 0.20 (polyethylene kinetic 0.20, dry plastic-on-wood 0.20..0.40, swept across 0.160..0.350 in Stage-0) -- Engineering ToolBox friction tables:
  https://www.engineeringtoolbox.com/friction-coefficients-d_778.html

Importance citation: US BLS OEWS occupations 35-2021 (food preparation workers) and 35-9021 (dishwashers) --
sliding heavy flat boards to a counter edge to grip them from below and racking them is routine dishpit /
prep-station clearing work.

Real-trial verification protocol:
- At the end of the trial, check the board lies fully inside the rack's inner cavity, flat within a few
  degrees with its underside on the floor plate, nothing riding the wall tops, and both hands withdrawn at
  least 40 mm.
- Review the run video: a board tipped past 15 deg, dropped on the table, left tilted past 6 deg or resting
  on the walls, still moving at the end, still flush on the block, or a hand within 40 mm scores 0.0.

## Runtime per trial
1,200 control steps max at 20 Hz (60 s simulated time); the scripted demo finishes once the 1.0 s success hold
completes (about 357 steps / 17.9 s).

## Seeds
Seed 0 (`object_jitter=0`, fixed starting layout).

## Reset between trials
Set the cutting board flush flat on the butcher block at its start mark, empty the rack, and send both arms to
ready pose.

## Licence
No mesh or texture assets: every part is built from boxes and cylinders in
`slide_flush_cutting_board_to_edge_and_rack.py`; original work, released under the repository's MIT licence.
