# Bolt a sprung storage-room door shut: hold it closed against its seal while sliding the bolt home

> Bolt the door shut: hold it closed and slide the bolt into the keeper, then let go.

Bolting a door shut at lock-up, holding it closed against its compressed seal while sliding the barrel bolt into the keeper, is a core operation for janitors and building cleaners (about 2.4 million US workers in 2024, BLS employment projections, 37-2011). A door that is not bolted springs back open.

**Physical difficulty (one fixed level):** the door is sprung open by its seal (a hinge spring of 0.6 N m/rad towards 29 degrees open), so one arm must push it shut and keep holding it while the other slides a barrel bolt 3 cm along the door's face into the keeper's slot. Only then does the keeper's lip hold the door. A bolt thrown early fouls the latch post.

## Run a trial

From the repository root, in the simulator's environment (Isaac Lab-Arena with the `yam_arena` embodiment), with a
Dream Labs key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks//task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Bolt the door shut: hold it closed and slide the bolt into the keeper, then let go." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/door-dream
```

The scripted full-score demonstration: the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/door-demo`. Render videos with `inspect-robots video <log>.json`.

Offline checks, without the simulator (the demo's inverse kinematics and the scoring rules on fake states):
`python tasks//check_task.py`. Diagnostics: set `DOOR_TRACE=/path/trace.jsonl` before a run to log,
every control step, every part's pose and joint positions and both grippers' tool points.

## Runtime and seeds

The scripted demonstration takes about 16 s; a Dream-1 trial runs until the finished state has held for
1 s or the run limit of 2,400 steps (2 minutes at 20 Hz). No seeds: `-E object_jitter=0` keeps every object exactly
where `task.py` places it, matching the setup screenshot.

## The layout

Positions are measured on the table from the point midway between the two arm bases, which stand 61 cm apart:
forward means away from the arms, left and right as seen from behind the arms.

- **Door set** (`doorset`): a small storage-room door in a light grey frame, its closed line 30 cm forward, fixed. The door (`door` joint) is blue, 21 cm wide x 11 cm tall x 12 mm thick, hinged on a post at 14 cm left (vertical axis); its seal pushes it 29 degrees open towards the arms (hinge spring 0.6 N m/rad). The latch post, 7-9 cm right, has a keeper slot 9.5 cm up with a lip on its near side.
- **Barrel bolt** (`bolt` joint): brass, on the door's near face 9.5 cm up, 5 cm long with a knob; it slides 3 cm towards the latch post (friction 0.4 N).

Nothing else is on the table.

## Scoring

The door is **shut** when its hinge is within 2 degrees of the frame stop; the bolt is **thrown** when it has slid at least 2.5 cm of its 3 cm travel.

- **0.3** for the door shut, **plus 0.3** for the bolt thrown (0 to 0.6), at the end of the trial.
- **1.0:** shut and bolted, nothing moving (under 0.03 rad or 5 mm over the last 0.3 s), and no gripper tool point within 5 cm of the door's face or the bolt knob, so the bolt alone holds the door, for 1 s. The trial then ends. Completed less than 1 s before the run limit scores 0.6.

## Scoring a real trial

Film from above. 0.3 when the door is against the frame, 0.3 when the bolt is in the keeper; full score when the door stays shut on its bolt with both hands off it for 1 s.

## The demonstration

Scripted, open loop, closed fingers. The left arm pushes the door shut along its arc (finger tips trailing 1 cm behind a point 4.5 cm from the hinge, 9 cm up, near the top edge so the gripper stays above the door), pressing on 17 degrees past the stop, and holds it there. The right arm then pushes the bolt's knob 4.4 cm along the door into the keeper and lifts off; the left arm lets go last. Both grippers turn 90 degrees so that their pads close along the forward axis and their housings stay apart.

## Files

- `task.py`: the scene, the scoring and the scripted demonstration.
- `check_task.py`: offline checks (no simulator).
- `task.json`: the task's description for the task board.
- `labkit.py`, `arena_assets_fix.py`, `taskbase.py`: helpers. `labkit.py` and `arena_assets_fix.py` are copied unchanged from the accepted task `tasks/glovebox-filter` (labkit itself follows the technique of `tasks/bearing-press/partkit.py`, PR #40); they build multi-shape parts with hinged and sliding joints for yam_arena without modifying it. `taskbase.py` holds this task's per-trial scoring memory and small geometry helpers.
