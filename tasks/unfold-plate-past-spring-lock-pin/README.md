# Unfold a hand truck's nose-plate extension: hold its spring pop-pin out while folding the plate down

> Unfold the nose-plate extension: pull the pin out and hold it, fold the plate down flat, then let go.

Unfolding a hand truck's nose-plate extension before moving a long load is a routine operation for laborers and freight, stock and material movers (about 3.0 million US workers, BLS OEWS May 2023, 53-7062): pull the spring pin out and hold it, fold the extension down flat, let the pin go.

**Physical difficulty (one fixed level):** the extension stands folded up, locked by a spring pop-pin (40 N/m) that runs along the hinge line through a frame bracket and a lug on the extension's edge, so the lug cannot swing past it. One arm must hook the pin's T-handle from a 3.5 cm gap and hold it pulled out against the spring, while the other pushes the 17 cm-wide plate over and down through a 90-degree arc; let go too early and the pin springs back across the lug's path and stops the plate.

## Run a trial

From the repository root, in the simulator's environment (Isaac Lab-Arena with the `yam_arena` embodiment), with a
Dream Labs key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks//task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Unfold the nose-plate extension: pull the pin out and hold it, fold the plate down flat, then let go." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/plate-dream
```

The scripted full-score demonstration: the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/plate-demo`. Render videos with `inspect-robots video <log>.json`.

Offline checks, without the simulator (the demo's inverse kinematics and the scoring rules on fake states):
`python tasks//check_task.py`. Diagnostics: set `PLATE_TRACE=/path/trace.jsonl` before a run to log,
every control step, every part's pose and joint positions and both grippers' tool points.

## Runtime and seeds

The scripted demonstration takes about 15 s; a Dream-1 trial runs until the finished state has held for
1 s or the run limit of 2,400 steps (2 minutes at 20 Hz). No seeds: `-E object_jitter=0` keeps every object exactly
where `task.py` places it, matching the setup screenshot.

## The layout

Positions are measured on the table from the point midway between the two arm bases, which stand 61 cm apart:
forward means away from the arms, left and right as seen from behind the arms.

- **Nose plate** (`nose_plate`): a hand truck's steel nose plate (17 x 19 cm, 3 cm thick), centred 31 cm forward on the centre line, fixed. Its red extension (`plate` joint), 14 cm long x 17 cm wide x 1 cm thick, is hinged along the nose plate's near edge (24 cm forward, 3.6 cm up) and stands folded straight up. A 6 mm lug on its left edge carries a 14 mm square hole 7 cm from the hinge.
- **Pop-pin** (`pin` joint): a zinc 10 mm square pin running left-right (along the hinge line) through a guide bracket on the frame and through the lug's hole, 2.5 cm in front of the standing extension and 10.6 cm up; its T-handle is on the outside, 14.8 cm left of centre, with a 3.5 cm gap to the bracket. A 40 N/m spring pushes it in; it draws out 2.5 cm to the left, which clears the lug.

Nothing else is on the table.

## Scoring

The pin is **pulled** once it has been drawn at least 2 cm of its 2.5 cm travel; the plate is **down** when its hinge is within 3 degrees of flat.

- **0.3** once the pin has been pulled (at any time), **plus 0.3** for the plate down (0 to 0.6), at the end of the trial.
- **1.0:** plate down, pin let go (back within 5 mm of home), nothing moving and no gripper tool point within 4 cm of the plate's middle or the pin's handle, for 1 s. The trial then ends. Completed less than 1 s before the run limit scores 0.6.

## Scoring a real trial

Film from the side. 0.3 once the pin is drawn out; 0.3 when the extension lies flat; full score when it lies flat with the pin released and both hands clear for 1 s.

## The demonstration

Scripted, open loop, closed fingers. The left arm lowers its closed fingers (pads closing left-right, so they are thin enough for the gap) between the guide bracket and the pin's T-handle and draws the handle 3.3 cm outwards, past the 2.5 cm stop because the arm lags under load, and holds it. The right arm pushes the plate's near face 6.5 cm right of centre over and down through its arc (finger tips trailing 1.3 cm behind a point 11 cm up the plate) until it lies flat, presses and lifts off; then the left arm lifts away and the spring returns the pin.

## Files

- `task.py`: the scene, the scoring and the scripted demonstration.
- `check_task.py`: offline checks (no simulator).
- `task.json`: the task's description for the task board.
- `labkit.py`, `arena_assets_fix.py`, `taskbase.py`: helpers. `labkit.py` and `arena_assets_fix.py` are copied unchanged from the accepted task `tasks/glovebox-filter` (labkit itself follows the technique of `tasks/bearing-press/partkit.py`, PR #40); they build multi-shape parts with hinged and sliding joints for yam_arena without modifying it. `taskbase.py` holds this task's per-trial scoring memory and small geometry helpers.
