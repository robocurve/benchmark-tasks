# Change a receipt printer's paper roll: lift out the spent core, drop in the new roll, push the lid shut

> Change the printer's paper roll: put the empty core in the waste tray, drop in the new roll and shut the lid.

Changing the receipt printer's paper roll is a core, repeated operation for cashiers (about 3.3 million US workers, BLS OEWS May 2023, 41-2011): take the spent core out, drop the new roll into the pocket on its side, close the lid. The lid only closes over a seated roll with the old core gone.

**Physical difficulty (one fixed level):** the spent core (22 mm) is lifted out of a roll pocket only 1 cm wider each side than a fresh 60 mm roll; the roll goes in lying on its side, between the pocket's walls; the hinged lid, standing 110 degrees open towards the arms, is pushed over and down through its arc. The lid cannot close over a roll that is not seated or over the old core.

## Run a trial

From the repository root, in the simulator's environment (Isaac Lab-Arena with the `yam_arena` embodiment), with a
Dream Labs key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks//task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Change the printer's paper roll: put the empty core in the waste tray, drop in the new roll and shut the lid." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/printer-dream
```

The scripted full-score demonstration: the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/printer-demo`. Render videos with `inspect-robots video <log>.json`.

Offline checks, without the simulator (the demo's inverse kinematics and the scoring rules on fake states):
`python tasks//check_task.py`. Diagnostics: set `PRINTER_TRACE=/path/trace.jsonl` before a run to log,
every control step, every part's pose and joint positions and both grippers' tool points.

## Runtime and seeds

The scripted demonstration takes about 19 s; a Dream-1 trial runs until the finished state has held for
1 s or the run limit of 2,400 steps (2 minutes at 20 Hz). No seeds: `-E object_jitter=0` keeps every object exactly
where `task.py` places it, matching the setup screenshot.

## The layout

Positions are measured on the table from the point midway between the two arm bases, which stand 61 cm apart:
forward means away from the arms, left and right as seen from behind the arms.

- **Receipt printer** (`printer`): black body 14 cm (forward) x 12 cm x 7 cm tall, centred 26 cm forward on the centre line, fixed. Its roll pocket is 8 x 7.5 cm and 6.2 cm deep; its lid (`lid` joint) is hinged along the near top edge and stands 110 degrees open, leaning back towards the arms (hinge friction 0.04 N m).
- **Spent core** (`spent_core`): a brown cardboard tube 22 mm across and 58 mm long, 10 g, lying across the pocket floor.
- **New roll** (`paper_roll`): white thermal roll 60 mm across and 58 mm wide on a core, 150 g, lying across a small supply tray (`supply_tray`) at 16 cm forward, 24 cm left.
- **Waste tray** (`waste_tray`): grey, inside 10 x 10 cm, 2 cm walls, at 18 cm forward, 25 cm right.

Nothing else is on the table.

## Scoring

The core is **binned** when its centre lies inside the waste tray less than 2 cm above its floor. The roll is **loaded** when its centre lies inside the pocket seen from above, less than 1 cm above where it rests on the pocket floor, lying on its side (axis within 10 degrees of across). The lid is **shut** within 3 degrees of flat. **Still**: moved under 5 mm over the last 0.3 s.

- **0.2 each** (0 to 0.6), at the end of the trial: core binned and still; roll loaded and still; lid shut.
- **1.0:** all three, nothing moving, no gripper tool point within 4 cm of the lid's middle, for 1 s. The trial then ends. Completed less than 1 s before the run limit scores 0.6.
- **0, defect:** the roll or the core falling off the table.

## Scoring a real trial

Film from above and the side. 0.2 each for the core in the waste tray, the roll lying across the pocket floor, and the lid shut; full score when all three hold with the grippers clear for 1 s.

## The demonstration

Scripted, open loop; pads close along the forward axis, across the core's and the roll's diameters. The right arm reaches into the pocket, lifts the spent core out and drops it into the waste tray. The left arm lifts the new roll from the supply tray, carries it over the pocket with its bottom 3 cm clear of the printer, lowers it until the finger tips are 6 mm above the printer's top and lets it drop into the pocket; then, fingers closed, it pushes the open lid over and down through its arc (finger tips trailing 1.3 cm behind a point 9 cm out on the lid) and presses it shut.

## Files

- `task.py`: the scene, the scoring and the scripted demonstration.
- `check_task.py`: offline checks (no simulator).
- `task.json`: the task's description for the task board.
- `labkit.py`, `arena_assets_fix.py`, `taskbase.py`: helpers. `labkit.py` and `arena_assets_fix.py` are copied unchanged from the accepted task `tasks/glovebox-filter` (labkit itself follows the technique of `tasks/bearing-press/partkit.py`, PR #40); they build multi-shape parts with hinged and sliding joints for yam_arena without modifying it. `taskbase.py` holds this task's per-trial scoring memory and small geometry helpers.
