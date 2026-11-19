# Couple a delivery hose to a tank outlet with a camlock fitting: seat the coupler, then close both cam arms

> Couple the hose to the tank outlet: seat the coupler fully on the adapter, then close both cam arms.

Coupling the delivery hose to a tank outlet with a camlock (cam-and-groove) fitting is a core operation for tanker drivers among heavy and tractor-trailer truck drivers (about 2.0 million US workers, BLS OEWS May 2023, 53-3032): lift the coupler off its dummy adapter, seat it fully over the outlet's adapter, then swing both cam arms down. Arms closed on a coupler that is not seated leave the joint leaking.

**Physical difficulty (one fixed level):** the coupler's bore leaves 6 mm of play around the adapter and has to be lowered straight over it until it seats on the collar; then its two cam arms are pushed down from 10 degrees above horizontal through a 100-degree arc, one gripper on each side. Closed arms hang below the coupler's sleeve, so an arm closed too early stops the coupler going on.

## Run a trial

From the repository root, in the simulator's environment (Isaac Lab-Arena with the `yam_arena` embodiment), with a
Dream Labs key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks//task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Couple the hose to the tank outlet: seat the coupler fully on the adapter, then close both cam arms." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/camlock-dream
```

The scripted full-score demonstration: the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/camlock-demo`. Render videos with `inspect-robots video <log>.json`.

Offline checks, without the simulator (the demo's inverse kinematics and the scoring rules on fake states):
`python tasks//check_task.py`. Diagnostics: set `CAMLOCK_TRACE=/path/trace.jsonl` before a run to log,
every control step, every part's pose and joint positions and both grippers' tool points.

## Runtime and seeds

The scripted demonstration takes about 23 s; a Dream-1 trial runs until the finished state has held for
1 s or the run limit of 2,400 steps (2 minutes at 20 Hz). No seeds: `-E object_jitter=0` keeps every object exactly
where `task.py` places it, matching the setup screenshot.

## The layout

Positions are measured on the table from the point midway between the two arm bases, which stand 61 cm apart:
forward means away from the arms, left and right as seen from behind the arms.

- **Tank outlet** (`tank_outlet`): an aluminium male camlock adapter, 36 mm across and 5.7 cm tall with a collar 3 cm up, on a steel base plate 27 cm forward on the centre line, fixed.
- **Dummy adapter** (`dummy_adapter`): the same adapter at 15 cm forward, 24 cm right, fixed, where the hose's coupler is parked.
- **Coupler** (`coupler`): the hose's female camlock coupler, 250 g: an aluminium sleeve 6 cm square outside with a 48 mm bore, 3 cm tall, a capped top with a black 30 mm hose stub 4 cm tall, and two yellow cam arms (`arm_left`, `arm_right` joints), 5.5 cm long, hinged at the sleeve's top on its left and right sides. It starts seated on the dummy adapter with both arms open, sticking out 10 degrees above horizontal (hinge friction 0.03 N m).

Nothing else is on the table.

## Scoring

The coupler is **seated** when its axis is within 4 mm of the tank adapter's (seen from above), it rests on the collar (within 4 mm of seated height) and stands upright (within 5 degrees). An arm is **closed** when it hangs within 8 degrees of straight down.

- **0.3** for the coupler seated and still, **plus 0.15 per arm closed while it is seated** (0 to 0.6), at the end of the trial.
- **1.0:** seated with both arms closed, nothing moving, no gripper tool point within 4 cm of the hose stub or an arm, for 1 s. The trial then ends. Completed less than 1 s before the run limit scores 0.6.
- **0, defect:** the coupler falling off the table.

## Scoring a real trial

Film from the side. 0.3 when the coupler sits down on the outlet's collar, 0.15 for each cam arm folded down against it; full score when both arms are down on a seated coupler with the grippers clear for 1 s.

## The demonstration

Scripted, open loop. The right arm grips the hose stub (pads closing along the forward axis, across the 30 mm stub), lifts the coupler straight off its dummy, carries it with its sleeve 3 cm above the tank adapter, lowers it straight down to 3 mm above the collar and lets go. Then each arm in turn, fingers closed, pushes the cam arm on its side from outside, finger tips trailing 1.3 cm behind a point 4 cm out along it, down through 100 degrees until it hangs straight, and lifts clear: the left arm first, then the right.

## Files

- `task.py`: the scene, the scoring and the scripted demonstration.
- `check_task.py`: offline checks (no simulator).
- `task.json`: the task's description for the task board.
- `labkit.py`, `arena_assets_fix.py`, `taskbase.py`: helpers. `labkit.py` and `arena_assets_fix.py` are copied unchanged from the accepted task `tasks/glovebox-filter` (labkit itself follows the technique of `tasks/bearing-press/partkit.py`, PR #40); they build multi-shape parts with hinged and sliding joints for yam_arena without modifying it. `taskbase.py` holds this task's per-trial scoring memory and small geometry helpers.
