# Sweep scattered croutons into a hand-held dustpan: one arm holds the pan down, the other sweeps with the brush

> Sweep all the croutons into the dustpan with the brush, holding the dustpan steady, then put the brush down and let go of both.

Sweeping spilled food and debris into a dustpan is a core, many-times-a-shift duty of janitors and building cleaners (about 2.2 million US workers, BLS OEWS May 2024, 37-2011, whose listed tasks include sweeping floors) and of maids and housekeeping cleaners (37-2012). By hand it is a two-handed job: one hand holds the pan's lip down and keeps the pan from sliding, the other sweeps the debris up over the lip.

**Physical difficulty (one fixed level):** the 150 g pan slides on the table (friction 0.4) when the brush and croutons push against its lip, so one arm must grip its handle fin and hold the lip pressed to the table throughout, while the other sweeps 20 croutons scattered over a 12.8 x 10.3 cm patch up the 2 cm lip with the 18 cm brush. A pan that slides or lifts opens a gap at the lip; a brush driven too fast or skewed scatters croutons round its ends. Both objects are full-size household items.

## Run a trial

From the repository root, in the simulator's environment (Isaac Lab-Arena with the `yam_arena` embodiment), with a
Dream Labs key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/sweep-crumbs-into-held-dustpan/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Sweep all the croutons into the dustpan with the brush, holding the dustpan steady, then put the brush down and let go of both." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/dustpan-dream
```

The scripted full-score demonstration: the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/dustpan-demo`. Render videos with `inspect-robots video <log>.json`.

Offline checks, without the simulator (the demo's inverse kinematics and the scoring rules on fake states):
`python tasks/sweep-crumbs-into-held-dustpan/check_task.py`. Diagnostics: set `DUSTPAN_TRACE=/path/trace.jsonl` before a run to log,
every control step, every part's pose and joint positions and both grippers' tool points.

## Runtime and seeds

The scripted demonstration takes about 25 s; a Dream-1 trial runs until the finished state has held for
1 s or the run limit of 2,400 steps (2 minutes at 20 Hz). No seeds: `-E object_jitter=0` keeps every object exactly
where `task.py` places it, matching the setup screenshot.

## The layout

Positions are measured on the table from the point midway between the two arm bases, which stand 61 cm apart:
forward means away from the arms, left and right as seen from behind the arms.

- **Dustpan**: 23 cm wide (forward) x 18 cm deep, lying flat with its lip on the centre line facing right, centred 27 cm forward; a 2 cm lip ramps up 2 mm onto its floor; 4 cm side walls, a 7 cm back wall, and a flat handle fin 11 cm long, 3 cm tall and 12 mm thick standing on edge behind the back wall, at the top.
- **Hand brush**: a bristle block 18 cm long (forward) x 4.5 cm x 3.5 cm under a wooden stock, with a 10 cm wooden handle (22 mm square) off its far end; lying bristles down, centred 23 cm forward, 21 cm right.
- **Croutons**: 20 cubes of 10 mm, 1 g each, scattered between 20.1 and 31.9 cm forward and 5.7 and 15 cm right, at least 1.8 cm apart (exact positions in task.py).

Nothing else is on the table.

## Scoring

A crouton is in the pan when its centre, in the pan's own frame, lies over the pan's floor (between the side walls and between the top of the lip and the back wall) and no more than 2.5 cm above the floor.

- **0.6 x (croutons in the pan) / 20** (0 to 0.6), at the end of the trial.
- **1.0:** at least 18 of 20 in the pan, pan upright (under 10 degrees), nothing moving (under 5 mm over 0.3 s), no gripper tool point within 5 cm of either handle, for 1 s. The trial then ends. Completed less than 1 s before the run limit, the partial score stands.

## Building the real setup

- A household hand-brush-and-dustpan set with a flat-bottomed pan close to the dimensions above (any common set with a pan about 23 cm wide and a brush head about 18 cm long). If the pan's handle is round, glue a flat plastic fin to it so the gripper closes across a flat face, as in the simulation.
- 20 croutons cut to about 10 mm cubes (or 10 mm wooden craft cubes of about 1 g), laid out at the positions in task.py with a printed template.
- The pan and brush are laid out at the positions above; the table surface is the bare work table.

## Scoring a real trial

At the end of the trial, count the croutons resting inside the pan on its floor: score 0.6 x count / 20. Full score (1.0) when at least 18 are inside, the pan is upright, nothing is moving and neither hand is touching the pan or the brush, for 1 s. The same rule as the simulation.

## The demonstration

Scripted, open loop. The left arm grips the pan's handle fin (pads closing across the fin) and presses the pan down onto the table, holding it there throughout. The right arm grips the brush by its handle, lifts it, sets the bristles down behind the croutons and sweeps them forward up the lip onto the pan's floor, twice, lifting the brush back off the pan after each pass; then it sets the brush down where it found it and lets go, and the left arm lets go of the pan.

## Files

- `task.py`: the scene, the scoring and the scripted demonstration.
- `check_task.py`: offline checks (no simulator).
- `task.json`: the task's description for the task board.
- `labkit.py`, `arena_assets_fix.py`, `taskbase.py`: helpers. `labkit.py` and `arena_assets_fix.py` are copied unchanged from the accepted task `tasks/glovebox-filter` (labkit itself follows the technique of `tasks/bearing-press/partkit.py`, PR #40); they build multi-shape parts with hinged and sliding joints for yam_arena without modifying it. `taskbase.py` holds this task's per-trial scoring memory and small geometry helpers.
