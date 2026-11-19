# Scrape diced carrot off a cutting board into a sheet pan: one arm holds the board, the other drives the bench scraper

> Scrape all the diced carrot off the cutting board into the sheet pan with the bench scraper, holding the board still, then put the scraper back in its holder and let go of the board.

Transferring chopped vegetables from the cutting board to a sheet pan with a bench scraper is a core, many-times-a-shift prep operation for restaurant cooks (about 1.4 million US workers, BLS OEWS May 2024, 35-2014) and food preparation workers (35-2021). By hand it is two-handed: one hand holds the board still, the other drives the scraper across it and pushes the pieces off its edge into the pan.

**Physical difficulty (one fixed level):** the 1600 g board slides on the worktop (friction 0.25) when the scraper is dragged across it pressed down, so one arm must press it down and hold it throughout, while the other lifts the 15 cm bench scraper out of its holder and drags it through 20 pieces scattered over a 11.5 x 8.6 cm patch and off the board's edge into the pan, with the blade's edge kept down on the board so no piece slips under it, and then puts it back in its slot.

## Run a trial

From the repository root, in the simulator's environment (Isaac Lab-Arena with the `yam_arena` embodiment), with a
Dream Labs key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/scrape-board-into-sheet-pan/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Scrape all the diced carrot off the cutting board into the sheet pan with the bench scraper, holding the board still, then put the scraper back in its holder and let go of the board." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/board-dream
```

The scripted full-score demonstration: the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/board-demo`. Render videos with `inspect-robots video <log>.json`.

Offline checks, without the simulator (the demo's inverse kinematics and the scoring rules on fake states):
`python tasks/scrape-board-into-sheet-pan/check_task.py`. Diagnostics: set `BOARD_TRACE=/path/trace.jsonl` before a run to log,
every control step, every part's pose and joint positions and both grippers' tool points.

## Runtime and seeds

The scripted demonstration takes about 25 s; a Dream-1 trial runs until the finished state has held for
1 s or the run limit of 2,400 steps (2 minutes at 20 Hz). No seeds: `-E object_jitter=0` keeps every object exactly
where `task.py` places it, matching the setup screenshot.

## The layout

Positions are measured on the table from the point midway between the two arm bases, which stand 61 cm apart:
forward means away from the arms, left and right as seen from behind the arms.

- **Cutting board**: end-grain maple, 23 cm (forward) x 30 cm x 3.8 cm thick, lying flat, centred 27 cm forward, 6 cm left; its right edge 9 cm right; friction on the table 0.25.
- **Sheet pan**: a quarter sheet pan, 33 cm (forward) x 23 cm outside with a 2.5 cm rim, butted against the board's right edge, centred 27 cm forward, 20.9 cm right.
- **Bench scraper**: a stainless blade 15 cm long x 7.5 cm tall with a wooden handle along its top edge, standing blade down in a utensil holder (utensil_holder, fixed: a 4 cm block with a 6 mm slot whose top 2 cm flares to 3 cm) 22 cm forward, 38.5 cm right, its blade running forward.
- **Diced carrot**: 20 pieces of 10 mm, 1 g each, scattered on the board between 21.9 and 32.4 cm forward and 0.5 cm left and 7.1 cm right, at least 1.7 cm apart (exact positions in task.py).

Nothing else is on the table.

## Scoring

A piece is in the pan when its centre, in the pan's own frame, lies inside the pan's walls and no higher than 1 cm above the rim. The bench scraper is back in its holder when its bottom is within 2.5 cm of the slot's middle along the slot and 1 cm across it, below the holder's top, and it stands within 15 degrees of upright.

- **0.6 x (pieces in the pan) / 20** (0 to 0.6), at the end of the trial.
- **1.0:** at least 18 of 20 in the pan, scraper back in its holder, nothing moving (under 5 mm over 0.3 s), no gripper tool point within 5 cm of the scraper's handle or the board's top, for 1 s. The trial then ends. Completed less than 1 s before the run limit, the partial score stands.

## Building the real setup

- An end-grain or edge-grain cutting board about 23 x 30 x 3.8 cm, without rubber feet (common 12 x 9 x 1.5-inch boards).
- A quarter sheet pan (standard 13 x 9-inch aluminium, 1-inch rim).
- A 6-inch stainless bench scraper with a handle along its top edge, and a wood block 4 cm tall with a 6 mm saw-cut slot, chamfered to 3 cm at the top, that holds it upright.
- 20 pieces of raw carrot diced to 10 mm, laid out on the board at the positions in task.py with a printed template.
- The board is set on the bare work table with the pan butted against it.

## Scoring a real trial

At the end of the trial, count the carrot pieces inside the pan: score 0.6 x count / 20. Full score (1.0) when at least 18 are inside, the scraper is standing back in its holder, nothing is moving and neither hand is touching the board or the scraper, for 1 s. The same rule as the simulation.

## The demonstration

Scripted, open loop. The left arm presses its closed fingers down on the board's left half and holds it there throughout. The right arm lifts the bench scraper out of its holder by its handle, sets the blade's edge on the board left of the pieces and drags it to the right across the board and 2.5 cm past its edge over the pan, twice; then it puts the scraper back in its holder and lets go, and the left arm lifts off the board.

## Files

- `task.py`: the scene, the scoring and the scripted demonstration.
- `check_task.py`: offline checks (no simulator).
- `task.json`: the task's description for the task board.
- `labkit.py`, `arena_assets_fix.py`, `taskbase.py`: helpers. `labkit.py` and `arena_assets_fix.py` are copied unchanged from the accepted task `tasks/glovebox-filter` (labkit itself follows the technique of `tasks/bearing-press/partkit.py`, PR #40); they build multi-shape parts with hinged and sliding joints for yam_arena without modifying it. `taskbase.py` holds this task's per-trial scoring memory and small geometry helpers.
