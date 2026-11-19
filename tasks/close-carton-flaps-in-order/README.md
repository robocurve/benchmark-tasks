# Close a filled shipping carton's top flaps in overlap order: short flaps first, long flaps over them

> Close the carton's top: the two short flaps first, then the two long flaps over them.

Closing a regular slotted carton's top before taping is a core, repeated operation for hand packers and for stockers and order fillers (about 2.9 million US workers in May 2023, BLS OEWS 53-7065). The two short (minor) flaps must go down first and the two long (major) flaps over them: a long flap closed first blocks the short flap under it, and the carton cannot then be closed without reopening it.

**Physical difficulty (one fixed level):** four hinged corrugated flaps, each pushed through an 85-degree arc by finger tips that must stay behind the moving flap, two arms working on opposite flaps at once, in an order the board itself enforces: the flaps collide, so a long flap closed early leaves a short flap jammed open.

## Run a trial

From the repository root, in the simulator's environment (Isaac Lab-Arena with the `yam_arena` embodiment), with a
Dream Labs key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/close-carton-flaps-in-order/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Close the carton's top: the two short flaps first, then the two long flaps over them." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/carton-dream
```

The scripted full-score demonstration: the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/carton-demo`. Render videos with `inspect-robots video <log>.json`.

Offline checks, without the simulator (the demo's inverse kinematics and the scoring rules on fake states):
`python tasks/close-carton-flaps-in-order/check_task.py`. Diagnostics: set `CARTON_TRACE=/path/trace.jsonl` before a run to log,
every control step, every part's pose and joint positions and both grippers' tool points.

## Runtime and seeds

The scripted demonstration takes about 16 s; a Dream-1 trial runs until the finished state has held for
1 s or the run limit of 2,400 steps (2 minutes at 20 Hz). No seeds: `-E object_jitter=0` keeps every object exactly
where `task.py` places it, matching the setup screenshot.

## The layout

Positions are measured on the table from the point midway between the two arm bases, which stand 61 cm apart:
forward means away from the arms, left and right as seen from behind the arms.

- **Carton** (`carton`): kraft corrugated board, 13 cm (forward) x 22 cm (across) x 6 cm tall, 4 mm walls, centred 25 cm forward on the centre line, standing against the packing bench's back stop (fixed). Its four top flaps are hinged at the top of its walls and start 85 degrees open, leaning slightly in, held by the board's own stiffness (hinge friction 0.02 N m):
  - short flaps (`minor_left`, `minor_right`, darker): on the left and right walls, 10 cm x 6.5 cm;
  - long flaps (`major_near`, `major_far`): on the near and far walls, 22 cm x 6.5 cm, hinged 4 mm higher so that they close over the short flaps.

Nothing else is on the table.

## Scoring

A flap is **closed** when its hinge is within 7 degrees of flat.

- **0.15 per closed flap** (0 to 0.6), at the end of the trial.
- **1.0:** all four flaps closed (the board only allows this with the short flaps under the long ones), the carton still (no flap moving more than 0.03 rad over the last 0.3 s) and no gripper tool point within 4 cm of a flap's centre, held for 1 s. The trial then ends. Closed less than 1 s before the run limit scores 0.6.

## Scoring a real trial

Film from the side and above. 0.15 for each flap lying flat; full score when all four lie flat with the long flaps on top and the grippers clear, for 1 s. A long flap closed first jams the short flap under it, which caps the score until it is reopened.

## The demonstration

Scripted, open loop, both arms with closed fingers. Each arm first pushes the short flap on its side down through its arc, finger tips trailing 1.2 cm behind a point 4.5 cm out from the hinge, and lifts clear; then each pushes the long flap on its side down over the short flaps (left arm the near one at 7.5 cm left, right arm the far one at 7.5 cm right), presses it flat and lifts clear.

## Files

- `task.py`: the scene, the scoring and the scripted demonstration.
- `check_task.py`: offline checks (no simulator).
- `task.json`: the task's description for the task board.
- `labkit.py`, `arena_assets_fix.py`, `taskbase.py`: helpers. `labkit.py` and `arena_assets_fix.py` are copied unchanged from the accepted task `tasks/glovebox-filter` (labkit itself follows the technique of `tasks/bearing-press/partkit.py`, PR #40); they build multi-shape parts with hinged and sliding joints for yam_arena without modifying it. `taskbase.py` holds this task's per-trial scoring memory and small geometry helpers.
