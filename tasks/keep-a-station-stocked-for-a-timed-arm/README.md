# Keep a hand-off tray stocked for a second robot arm that empties it on a fixed clock

## Impact on jobs
O*NET 53-7062.00 "Laborers and Freight, Stock, and Material Movers, Hand": 2,940,300 employees (2025),
https://www.onetonline.org/link/summary/53-7062.00. Core task (importance 76) from the occupation's details page: "Move freight,
stock, or other materials to and from storage or production areas, loading docks, delivery vehicles, ships, or
containers, by hand or using trucks, tractors, or other equipment." Keeping a pick-up station stocked for a machine or
coworker that takes parts on a fixed cycle is that task at a shared station: each part must be there in time, put down
only when the station is empty and the taker's hand is clear, without getting in the taker's way.

Instruction: "Keep the white tray stocked with one wood block at a time from the table, so the other arm can crate all five."

Files in this folder: `task.json`, `keep_a_station_stocked_for_a_timed_arm.py` (scene, arm-touch watch, scorer, coworker
routine, scripted demo), `coworker.py` (drives the right arm from the fixed routine), `labkit.py` (multi-shape parts and
step hooks), `arena_assets_fix.py` (library asset paths), `test_keep_a_station_stocked_for_a_timed_arm.py` (offline
tests), `README.md`, `DEVIATIONS.md`.

## Review items and how this version answers them
This idea was first submitted as #441 (TK-TT776S). The review of 2026-11-12 read:

> Construct validity (not met): A robot that does nothing receives one-sixth credit when the scripted arm crates the block supplied at setup.
>
> Scoring (not met): The score counts the preloaded block as evaluated-robot progress, producing a nonzero score without any stocking action.
>
> What to change: Exclude the block initially on the pickup station from credited progress and normalize the score over the five blocks the evaluated arm must supply; verify that holding still scores zero and update the scoring documentation and demo results.

| Review item | How it is answered here |
|---|---|
| Exclude the block initially on the pickup station from credited progress | `SUPPLIED = BLOCKS[1:]`: the scorer only looks at block_2..block_6. block_1 (preloaded on the tray) is never counted, wherever it ends up (`score()`, `crated()`). |
| Normalize over the five blocks the evaluated arm must supply | `score = len(crated(objects)) / len(SUPPLIED)` with `len(SUPPLIED) == 5`; possible scores 0.0, 0.2, 0.4, 0.6, 0.8, 1.0. Test `test_preloaded_block_never_credited_and_score_is_over_five` checks every step 1/5..5/5 with block_1 also in the crate, and that four supplied blocks give 0.8, never 5/6. |
| Verify that holding still scores zero | Test `test_holding_still_scores_zero` (block_1 crated by the coworker, nothing else moved -> 0.0); also a GPU-simulator run with the tested arm parked at its ready pose for the whole run limit (see Demo results). |
| Update the scoring documentation and demo results | `task.json` `scoring` states the five-block normalisation and "holds still scores 0.0"; `test_text_matches_scorer` ties the text to the constants. Demo results below. |

## Environment
Isaac Lab-Arena 0.3.1 on Isaac Sim 6.1 (Brev launchable), `inspect-robots` with the `yam_arena` embodiment, `station` scene.
Task file SHA-256 (first 12): see the freeze log.

## Offline check (no GPU)
Run `PYTHONDONTWRITEBYTECODE=1 pytest -q -p no:cacheprovider test_keep_a_station_stocked_for_a_timed_arm.py` with
`yam_arena` on `PYTHONPATH`: verifies the start scores 0.0, holding still scores 0.0, the preloaded block is never
credited and the score is over five, an arm touch zeroes the score, the crate region rejects blocks above the rim,
outside the walls or still on the tray, the coworker closes in the tray on its fixed clock, both plans are reachable
(IK < 2 mm, < 2 deg) and the tested arm is never over the tray while the coworker is in it, the demo reads no
simulator state, the scoring text matches the constants, shortcut end states (blocks on the rim, piled above it,
left on the tray, beside the crate, knocked off the table or back on the mat) score nothing, and both top cameras see
the crate floor and every supplied block with at least 40 mm between each sight line and every arm link or finger,
at the end of a GPU-simulator demo run and with the coworker parked at its wait pose.

## Run and score one Dream-1 trial
```
inspect-robots run --embodiment yam_arena -E task=keep_a_station_stocked_for_a_timed_arm.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy dream \
  --instruction "Keep the white tray stocked with one wood block at a time from the table, so the other arm can crate all five." \
  --scorer task_score --max-steps 2200 --seed 0
```

## Scripted demo
```
inspect-robots run --embodiment yam_arena -E task=keep_a_station_stocked_for_a_timed_arm.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy yam_scripted \
  --instruction "Keep the white tray stocked with one wood block at a time from the table, so the other arm can crate all five." \
  --scorer task_score --max-steps 2200 --seed 0
```
`demo()` reads geometry constants and the coworker's fixed clock only (never simulator state). The left arm picks each
table block in turn, carries it to a staging pose clear of the tray and the coworker, and sets it in the tray 2.6 s
after the coworker's grab (once the coworker has lifted the previous block away), then returns to the staging pose and
fetches the next block 6.4 s after the grab. The right arm runs the coworker routine: it closes in the tray at 4 s and
every 11 s after, eight times, lifts whatever is there and lets go over its own spot of a 3 x 3 grid (40 mm pitch) on
the crate floor, so the blocks lie side by side below the rim. Between visits it waits to the right of the crate, out of
the top cameras' view of the crate. The last supplied block is crated about 62.6 s in; the
demo then holds clear for the 1.0 s success hold (about 1,300 control steps of the 2,200-step run limit).

## Demo results
| Check | Result |
|---|---|
| Offline MuJoCo physics check (`mj_check --quick`), scripted demo | 1.0 (all five supplied blocks crated) at 1,268 steps |
| Offline MuJoCo, tested arm holding still at ready pose | 0.0 (the coworker crates block_1 on its first visit; it is not credited) |
| Local GPU simulator (Isaac Lab-Arena, RTX 5090), scripted demo, run 1 | 1.0 |
| Local GPU simulator, scripted demo, run 2 | 1.0 |
| Coworker wait pose moved out of the top cameras' view of the crate: offline MuJoCo, scripted demo | 1.0 (all five crated), no block hidden by an arm at the end |
| Same, offline MuJoCo, tested arm only parks at its staging pose | 0.0 |
| Same, local GPU simulator, scripted demo | 1.0 at 1,268 steps; nearest arm 81 mm from any camera sight line to the crate |
| Local GPU simulator, tested arm holding still at ready pose | 0.0 (block_1 crated by the coworker at (259.7, -337.6) mm, inside the bin, not credited; supplied blocks untouched; closest arm gap 302 mm) |

## How it is modelled
| Mechanism | Physical body / joint / contact | Source for the numbers |
|---|---|---|
| Hand-off tray (station B) | fixed thin-walled square box, 89 x 89 x 25 mm outside, 3 mm walls, 2 mm floor (`TRAY_IN = 0.083`, `TRAY_WALL = 0.003`, `TRAY_H = 0.025`) at (0.28, -0.04) m | Globe Scientific 3621 square weigh boat, 89 x 89 x 25 mm, https://www.usplastic.com/catalog/item.aspx?itemid=149052 |
| Crate | fixed thin-walled square box, 152 x 152 x 51 mm outside, 3 mm walls and floor (`CRATE_IN_X = CRATE_IN_Y = 0.146`, `CRATE_H = 0.051`) at (0.30, -0.30) m | STORi SimpleSort 6 x 6 x 2 in stackable organizer bin, 15.2 x 15.2 x 5.1 cm, https://www.amazon.com/STORi-6-Piece-Stackable-Organizer-Dividers/dp/B07NDGYQZN |
| Blocks | six free rigid 30.5 mm cubes, 16 g each (`CUBE = 0.0305`, `CUBE_M = 0.016`); block_1 on the tray, block_2..block_6 on the table in a die-five layout, corners 65 mm off (0.30, 0.22) m | Uxcell 1.2 in unfinished wooden blocks, https://business.walmart.com/ip/Uxcell-1-2-Inch-Unfinished-Wooden-Blocks-30-Pack-Natural-Wood-Cube-Square-Wood-Blocks-for-DIY-Craft/3261240513 ; mass estimated (see DEVIATIONS) |
| Coworker | right YAM arm, fixed routine of timed waypoints (stock `ArmPlan` interpolation + IK), replayed open loop on the embodiment clock; `coworker.py` replaces the right half of every action | timings `GRAB_T0 = 4.0 s`, `CYCLE_S = 11.0 s`, `N_VISITS = 8` |
| Arm touch | origins of link 4, link 5, gripper and both fingertips of one arm vs the other, every control step; a gap under `TOUCH = 0.050` m latches "touched" for the rest of the trial | proxy for contact between the arms |

Every scoring threshold in `task.json` matches a named constant in `keep_a_station_stocked_for_a_timed_arm.py`
(`SUPPLIED`, `CRATE_IN_X`, `CRATE_IN_Y`, `CRATE_H`, `TOUCH`, `SUCCESS_HOLD_S`).

## Scoring
Judged from end-state block poses plus one trial-long flag; succeeds once 1.0 holds for 1.0 s:
- score = (number of the five supplied blocks, block_2..block_6, whose centres end inside the crate's walls and between
  its floor and its `51 mm` rim) / 5. The preloaded block_1 is never credited, so an arm that holds still scores 0.0
  (the coworker crates block_1 on its first visit).
- `0.0` if the two arms ever came within `50 mm` of each other (body origins, checked every control step).
Possible scores: `0.0`, `0.2`, `0.4`, `0.6`, `0.8`, `1.0`.

## Real-world build
One-time parts cost for one trial set: $111.96 (`real_trial_cost_usd`), from the listed pack prices below.
- Globe Scientific 3621 square weigh boat, polystyrene, 89 x 89 x 25 mm (hand-off tray), case of 500, $64.48 —
  https://www.cplabsafety.com/weighing-boat-plastic-square-antistatic-89x89x25mm-polystyrene-white-100ml/gsci-3621 (also
  https://www.usplastic.com/catalog/item.aspx?itemid=149052)
- STORi SimpleSort 6 x 6 x 2 in stackable organizer bins, 6-piece set, 15.2 x 15.2 x 5.1 cm (crate), $30.49 (last US listed
  price found, 2025-05-28; the live Amazon page could not be read) —
  https://www.amazon.com/STORi-6-Piece-Stackable-Organizer-Dividers/dp/B07NDGYQZN
- Uxcell 1.2 in unfinished wooden blocks, 30-pack, $16.99 (six used) —
  https://business.walmart.com/ip/Uxcell-1-2-Inch-Unfinished-Wooden-Blocks-30-Pack-Natural-Wood-Cube-Square-Wood-Blocks-for-DIY-Craft/3261240513
- Per-trial consumables: $0 (all parts are reused).

Real-trial verification protocol:
- At the end of the trial, count the table blocks (not the one that started on the tray; mark it beforehand) lying in
  the bin with no part above the 51 mm rim.
- Record both arms on video; any approach of the arms closer than 50 mm (or any contact) scores 0.0.

## Runtime per trial
2,200 control steps max at 20 Hz (110 s simulated time); the scripted demo finishes once the 1.0 s hold completes
(about 1,300 steps / 65 s).

## Seeds
Seed 0 (`object_jitter=0`, fixed starting layout).

## Reset between trials
Put the marked block on the tray and five on the table in the die-five pattern, empty the crate, and send both arms to
ready pose.

## Licence
No mesh or texture assets: every part is built from boxes in `keep_a_station_stocked_for_a_timed_arm.py`; original work,
released under the repository's MIT licence.
