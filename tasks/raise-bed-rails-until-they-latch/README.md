# Raise both hospital bed side rails until their sprung latch pins snap under them

> Raise both bed rails until they latch, then let go of them.

Raising both side rails until they latch before leaving a patient in bed is a core safety operation for nursing assistants (about 1.35 million US workers, BLS OEWS May 2023, 31-1131). A rail that is lifted but not latched drops back down when let go.

**Physical difficulty (one fixed level):** each 400 g rail (slide friction 0.2 N) has to be lifted straight up 6.5 cm by its top bar, past a sprung latch pin (30 N/m) whose bevelled underside it pushes back on the way up. Only when the rail's bottom edge is above the pin does the pin spring out beneath it; the rail must then be let down onto the pin. Released early, it drops back down.

## Run a trial

From the repository root, in the simulator's environment (Isaac Lab-Arena with the `yam_arena` embodiment), with a
Dream Labs key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks//task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Raise both bed rails until they latch, then let go of them." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/rail-dream
```

The scripted full-score demonstration: the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/rail-demo`. Render videos with `inspect-robots video <log>.json`.

Offline checks, without the simulator (the demo's inverse kinematics and the scoring rules on fake states):
`python tasks//check_task.py`. Diagnostics: set `RAIL_TRACE=/path/trace.jsonl` before a run to log,
every control step, every part's pose and joint positions and both grippers' tool points.

## Runtime and seeds

The scripted demonstration takes about 12 s; a Dream-1 trial runs until the finished state has held for
1 s or the run limit of 2,400 steps (2 minutes at 20 Hz). No seeds: `-E object_jitter=0` keeps every object exactly
where `task.py` places it, matching the setup screenshot.

## The layout

Positions are measured on the table from the point midway between the two arm bases, which stand 61 cm apart:
forward means away from the arms, left and right as seen from behind the arms.

- **Bed** (`bed`): a table-scale training model of a hospital bed, 16 cm (forward) x 18 cm, frame 5 cm tall with a 3 cm mattress, centred 27 cm forward on the centre line, fixed, with four rail guide posts.
  - Side rails (`rail_left`, `rail_right` joints): 12 cm long, 5 cm tall, 1 cm thick, with a 14 mm top bar, 10 cm either side of the centre line; they slide straight up 6.5 cm in their guides and start down, their tops level with the mattress.
  - Latch pins (`latch_left`, `latch_right` joints): red sprung pins (30 N/m) under each rail's raised position, 9 cm up, with a bevelled underside; they retract 1.6 cm.

Nothing else is on the table.

## Scoring

A rail is **latched** when it is up (travel between 5.9 and 7.2 cm) and its latch pin is sprung out under it (within 3 mm of fully out).

- **0.3 per rail latched** (0 to 0.6), at the end of the trial.
- **1.0:** both latched, nothing moving, no gripper tool point within 4 cm of either rail's top bar, so the pins alone hold the rails, for 1 s. The trial then ends. Completed less than 1 s before the run limit scores 0.6.

## Scoring a real trial

Film from the side. 0.3 for each rail resting up on its latch pin; full score when both stay up on their pins with both hands off them for 1 s.

## The demonstration

Scripted, open loop, both arms at once. Each arm grips its rail's top bar (pads closing across the bar), lifts it straight up 7.5 cm (1.5 cm past the latch, so the pin can spring out), pauses, lowers it 2 cm so the rail settles onto the pin, lets go and lifts clear.

## Files

- `task.py`: the scene, the scoring and the scripted demonstration.
- `check_task.py`: offline checks (no simulator).
- `task.json`: the task's description for the task board.
- `labkit.py`, `arena_assets_fix.py`, `taskbase.py`: helpers. `labkit.py` and `arena_assets_fix.py` are copied unchanged from the accepted task `tasks/glovebox-filter` (labkit itself follows the technique of `tasks/bearing-press/partkit.py`, PR #40); they build multi-shape parts with hinged and sliding joints for yam_arena without modifying it. `taskbase.py` holds this task's per-trial scoring memory and small geometry helpers.
