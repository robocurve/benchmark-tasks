# Isolate and drain a filter housing: close both line ball valves, then open the drain valve

> Isolate the filter for a cartridge change: close both line valves, then open the drain valve.

Isolating and draining a filter housing before changing its cartridge is a core operation for general maintenance and repair workers (about 1.6 million US workers in 2024, BLS employment projections, 49-9071): close the isolation valves on both sides, then open the drain. Opening the drain while the line is still live sprays the room.

**Physical difficulty (one fixed level):** three quarter-turn ball valves, each with a stiff lever (stem friction 0.15 N m) that has to be pushed through a full 90-degree arc about its vertical stem by finger tips trailing the moving lever, in a safe order: both line valves closed before the drain opens.

## Run a trial

From the repository root, in the simulator's environment (Isaac Lab-Arena with the `yam_arena` embodiment), with a
Dream Labs key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks//task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Isolate the filter for a cartridge change: close both line valves, then open the drain valve." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/valve-dream
```

The scripted full-score demonstration: the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/valve-demo`. Render videos with `inspect-robots video <log>.json`.

Offline checks, without the simulator (the demo's inverse kinematics and the scoring rules on fake states):
`python tasks//check_task.py`. Diagnostics: set `VALVE_TRACE=/path/trace.jsonl` before a run to log,
every control step, every part's pose and joint positions and both grippers' tool points.

## Runtime and seeds

The scripted demonstration takes about 17 s; a Dream-1 trial runs until the finished state has held for
1 s or the run limit of 2,400 steps (2 minutes at 20 Hz). No seeds: `-E object_jitter=0` keeps every object exactly
where `task.py` places it, matching the setup screenshot.

## The layout

Positions are measured on the table from the point midway between the two arm bases, which stand 61 cm apart:
forward means away from the arms, left and right as seen from behind the arms.

- **Filter line** (`filter_line`): a copper water line 30 cm long, 2.2 cm across, 5 cm up, running across the table 29 cm forward on two stands, fixed, with a blue filter housing (5.6 cm across, 6 cm tall) on the centre line and a drain branch running 5.5 cm from it towards the arms.
  - Line valves (`valve_left`, `valve_right` joints): brass quarter-turn ball valves 10 cm either side of the housing, with 6.5 cm levers (red grips) lying along the pipe: open. Closing turns a lever 90 degrees about its vertical stem to point at the arms.
  - Drain valve (`drain` joint): on the branch, its 6.5 cm lever (blue grip) lying across the branch: closed. Opening turns it 90 degrees to point along the branch, at the arms.
  All three stems have 0.15 N m of friction, so a lever stays where it is left.

Nothing else is on the table.

## Scoring

A valve is **turned** when its lever has moved at least 80 degrees of its quarter turn: the line valves are then closed, the drain open.

- **0.2 per line valve closed, plus 0.2 for the drain open** (0 to 0.6), at the end of the trial.
- **1.0:** all three, nothing moving (under 0.03 rad over the last 0.3 s), no gripper tool point within 4 cm of a lever's grip, for 1 s. The trial then ends. Completed less than 1 s before the run limit scores 0.6.
- **0, defect:** the drain opened more than 30 degrees while either line valve is less than 80 degrees closed (draining a live line).

## Scoring a real trial

Film from above. 0.2 for each line lever pointing at the operator and 0.2 for the drain lever along its branch; full score when all three hold with the grippers clear for 1 s. Opening the drain before both line valves are shut scores 0.

## The demonstration

Scripted, open loop, closed fingers. Each arm pushes its line valve's lever, 4.5 cm out from the stem, through 95 degrees about the stem (finger tips trailing 1.6 cm behind it) so that it ends pointing at the arms, and lifts off. Once both line valves are shut, the left arm turns the drain lever through 95 degrees the same way.

## Files

- `task.py`: the scene, the scoring and the scripted demonstration.
- `check_task.py`: offline checks (no simulator).
- `task.json`: the task's description for the task board.
- `labkit.py`, `arena_assets_fix.py`, `taskbase.py`: helpers. `labkit.py` and `arena_assets_fix.py` are copied unchanged from the accepted task `tasks/glovebox-filter` (labkit itself follows the technique of `tasks/bearing-press/partkit.py`, PR #40); they build multi-shape parts with hinged and sliding joints for yam_arena without modifying it. `taskbase.py` holds this task's per-trial scoring memory and small geometry helpers.
