# Close a vehicle's hood off its prop rod: hold the hood up, stow the rod, then let the hood down and press it shut

> Close the hood: lift it off the prop rod, stow the rod, then let the hood down and press it shut.

Closing the hood after service is a core, many-times-a-day operation for automotive service technicians and mechanics (about 0.8 million US workers in 2024, BLS employment projections, 49-3023): lift the hood off its prop rod and hold it, stow the rod, then lower the hood and press it shut. Dropping the hood on a standing rod bends the rod or the hood.

**Physical difficulty (one fixed level):** the hood rests on its prop rod, so the rod cannot be stowed until the hood is lifted off it and held. One arm grips the hood's front-edge handle and raises it about 10 degrees, holding it; the other pushes the rod's top over and down through a 90-degree arc until it lies along the fender; then the hood is lowered, let go and pressed shut.

## Run a trial

From the repository root, in the simulator's environment (Isaac Lab-Arena with the `yam_arena` embodiment), with a
Dream Labs key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks//task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Close the hood: lift it off the prop rod, stow the rod, then let the hood down and press it shut." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/hood-dream
```

The scripted full-score demonstration: the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/hood-demo`. Render videos with `inspect-robots video <log>.json`.

Offline checks, without the simulator (the demo's inverse kinematics and the scoring rules on fake states):
`python tasks//check_task.py`. Diagnostics: set `HOOD_TRACE=/path/trace.jsonl` before a run to log,
every control step, every part's pose and joint positions and both grippers' tool points.

## Runtime and seeds

The scripted demonstration takes about 21 s; a Dream-1 trial runs until the finished state has held for
1 s or the run limit of 2,400 steps (2 minutes at 20 Hz). No seeds: `-E object_jitter=0` keeps every object exactly
where `task.py` places it, matching the setup screenshot.

## The layout

Positions are measured on the table from the point midway between the two arm bases, which stand 61 cm apart:
forward means away from the arms, left and right as seen from behind the arms.

- **Front end** (`front_end`): a service-training mock-up of a car's engine bay, 18 cm (forward) x 20 cm x 4 cm, centred 30 cm forward on the centre line, fixed, with an engine block inside and a fender ledge along its right side.
  - Hood (`hood` joint): red, 17 x 19 cm, hinged along the bay's far top edge (the windscreen end), standing 30 degrees open, with a chrome handle bar 14 mm across at its front edge, 4 cm left, and a side tab over the fender (hinge friction 0.01 N m, damping 0.05).
  - Prop rod (`prop_rod` joint): yellow, 7 cm, pivoting on the fender ledge 28 cm forward, 11.7 cm right, standing straight up under the hood's side tab; it stows lying along the fender towards the arms.

Nothing else is on the table.

## Scoring

The rod is **stowed** when it lies within 9 degrees of flat along the fender; the hood is **shut** within 3 degrees of closed.

- **0.3** for the rod stowed, **plus 0.3** for the hood shut (0 to 0.6), at the end of the trial.
- **1.0:** both, nothing moving (under 0.03 rad over the last 0.3 s), no gripper tool point within 4 cm of the hood's front edge or middle, for 1 s. The trial then ends. Completed less than 1 s before the run limit scores 0.6.

## Scoring a real trial

Film from the side. 0.3 when the rod lies on the fender, 0.3 when the hood is down; full score when the hood is shut over the stowed rod with the grippers clear for 1 s.

## The demonstration

Scripted, open loop. The left arm grips the hood's chrome handle (pads closing along the forward axis, across the bar, which can turn between them) and raises the hood 11 degrees off the rod, holding it. The right arm, fingers closed, pushes the rod's top from its far side towards the arms and down through 90 degrees (finger tips trailing 1.3 cm behind a point 1.2 cm below the top) until it lies along the fender, and moves clear. The left arm lowers the hood back to 30 degrees and lets go; the hood falls shut, slowed by its hinge damping, and the left arm presses its front edge down and lifts clear.

## Files

- `task.py`: the scene, the scoring and the scripted demonstration.
- `check_task.py`: offline checks (no simulator).
- `task.json`: the task's description for the task board.
- `labkit.py`, `arena_assets_fix.py`, `taskbase.py`: helpers. `labkit.py` and `arena_assets_fix.py` are copied unchanged from the accepted task `tasks/glovebox-filter` (labkit itself follows the technique of `tasks/bearing-press/partkit.py`, PR #40); they build multi-shape parts with hinged and sliding joints for yam_arena without modifying it. `taskbase.py` holds this task's per-trial scoring memory and small geometry helpers.
