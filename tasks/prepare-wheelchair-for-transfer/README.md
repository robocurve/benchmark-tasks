# Ready a wheelchair for a transfer: swing both legrests away and lock both wheel brakes

> Get the wheelchair ready for a transfer: swing both legrests away and put both brakes on.

Before a patient moves from a wheelchair to a bed or toilet, nursing assistants (about 1.35 million US workers, BLS OEWS May 2023, 31-1131) lock both wheel brakes and swing both legrests out of the way, so the chair cannot roll and the patient's feet have a clear path. A chair with a brake off can roll away during the transfer.

**Physical difficulty (one fixed level):** four separate mechanisms, two per arm. Each swing-away legrest is a hinged hanger that must be swung out through a 90-degree arc about its vertical hinge by its release handle, a post rising above the seat at its free end, pushed round from the seat side; each brake is a friction-held lever pushed forward at least 25 degrees onto the tyre.

## Run a trial

From the repository root, in the simulator's environment (Isaac Lab-Arena with the `yam_arena` embodiment), with a
Dream Labs key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks//task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Get the wheelchair ready for a transfer: swing both legrests away and put both brakes on." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/chair-dream
```

The scripted full-score demonstration: the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/chair-demo`. Render videos with `inspect-robots video <log>.json`.

Offline checks, without the simulator (the demo's inverse kinematics and the scoring rules on fake states):
`python tasks//check_task.py`. Diagnostics: set `CHAIR_TRACE=/path/trace.jsonl` before a run to log,
every control step, every part's pose and joint positions and both grippers' tool points.

## Runtime and seeds

The scripted demonstration takes about 16 s; a Dream-1 trial runs until the finished state has held for
1 s or the run limit of 2,400 steps (2 minutes at 20 Hz). No seeds: `-E object_jitter=0` keeps every object exactly
where `task.py` places it, matching the setup screenshot.

## The layout

Positions are measured on the table from the point midway between the two arm bases, which stand 61 cm apart:
forward means away from the arms, left and right as seen from behind the arms.

- **Wheelchair** (`wheelchair`): parked facing the arms, centred 27 cm forward on the centre line, fixed: seat 17 cm wide at 7 cm, back rest, two 12 cm rear wheels 10 cm either side, front casters.
  - Legrests (`legrest_left`, `legrest_right` joints): grey hangers 7 cm wide and 6 cm tall with footplates and a 4 cm release handle standing at the free end, in front of the seat, each hinged on a vertical axis at the seat's outer front corner (18.5 cm forward, 8.5 cm either side); they swing out towards the arms (pivot friction 0.05 N m). Their free edges meet 1.5 cm either side of the centre line.
  - Brakes (`brake_left`, `brake_right` joints): red levers 5 cm tall with a knob, 25 cm forward, 11.2 cm either side, pivoting forward onto the tyres (friction 0.08 N m); off when upright.

Nothing else is on the table.

## Scoring

A legrest is **swung away** when its hinge has turned at least 80 degrees out; a brake is **on** when its lever has been pushed at least 25 degrees forward onto the tyre.

- **0.2 per legrest swung away and per brake on** (0 to 0.8), at the end of the trial.
- **1.0:** all four, nothing moving (under 0.03 rad over the last 0.3 s), no gripper tool point within 4 cm of a brake lever's knob, for 1 s. The trial then ends. Completed less than 1 s before the run limit scores 0.8.

## Scoring a real trial

Film from above. 0.2 for each legrest swung out of the way and each brake lever pushed onto its tyre; full score when all four hold with the grippers clear for 1 s.

## The demonstration

Scripted, open loop, closed fingers. The left arm does its legrest then its brake; the right arm does its brake first and starts its legrest only once the left legrest is out of the way, so the two grippers are never together at the legrests' meeting edges. For a legrest the arm lowers its closed fingers behind the release handle, on the seat side and above the seat, and pushes the handle round 100 degrees about the hinge (finger tips trailing 1 cm behind it), then lifts clear. For a brake it pushes the lever's knob forward 32 degrees and lifts off.

## Files

- `task.py`: the scene, the scoring and the scripted demonstration.
- `check_task.py`: offline checks (no simulator).
- `task.json`: the task's description for the task board.
- `labkit.py`, `arena_assets_fix.py`, `taskbase.py`: helpers. `labkit.py` and `arena_assets_fix.py` are copied unchanged from the accepted task `tasks/glovebox-filter` (labkit itself follows the technique of `tasks/bearing-press/partkit.py`, PR #40); they build multi-shape parts with hinged and sliding joints for yam_arena without modifying it. `taskbase.py` holds this task's per-trial scoring memory and small geometry helpers.
