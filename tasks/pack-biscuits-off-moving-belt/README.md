# Pack six biscuit packs off a moving belt into two cases before the red line, letting the two cracked packs pass

Eight biscuit packs ride in line on a moving belt, one every 1.7 seconds. Every third pack is cracked and marked with a red band.
The robot must take the six good packs off the belt before they cross the red line, put one in each cell of two three-cell
cases, and let the two cracked packs ride past.

Hand packers on food lines take products off a moving belt into cases at the pace of the line, and let damaged ones pass to the
reject end (O*NET 53-7064.00, Packers and Packagers, Hand). The task tests catching moving objects, keeping pace, placing into
tight cells, and leaving the rejects alone. The arms have to share the line: a pack passes every 1.7 s, and the fastest
single-arm pick-and-place cycle we scripted took about 4 s.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and the two-arm `demo()` |
| `rigidparts.py` | Rigid parts built from several boxes (the packs and cases), and the moving belt |

`rigidparts.py` wraps `yam_arena.scene.task_object` and `object_states` when they load, without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment, `yam_scripted` policy and `task_score` scorer. The belt is a kinematic part with
PhysX surface velocity. **It needs CPU physics, `-E device=cpu`**: on GPU physics the packs fall through it. The cameras still
render on the GPU.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E device=cpu -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the six good biscuit packs off the moving belt before they cross the red line and put one in each cell of the two cases. Let the cracked packs with the red band ride past." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/belt-dream
```

The scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E device=cpu -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the six good biscuit packs off the moving belt before they cross the red line and put one in each cell of the two cases. Let the cracked packs with the red band ride past." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/belt-demo
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

Each arm takes every third pack, 5 s apart, and fills the case on its own side; neither arm's turn falls on a cracked pack.

1. The arm comes down with its fingers open either side of the belt, so that the pack rides in between them.
2. It closes, lifts, carries the pack to its cell, lets go and comes back.
3. The cracked packs ride through to the end stop.

The demo reaches 1.0 (success) in about 430 steps (22 s) with `-E object_jitter=0`, on NVIDIA L40S, RTX A6000 and RTX A5000 GPUs.
With 5 mm of object jitter it scored 1.0 and 0.75 (two seeds); it is open-loop and tuned to the fixed layout. Dream-1
(`dreamlabs/dream-1-beta`, 2,400 steps) scored 0.0: it packed nothing, and the packs collected at the end stop.

Scripted checks: the right arm's half of the demo alone scored 0.45, three packs. The left arm's half alone scored 0: with the
right arm idle, missed packs back up from the end stop, and its third grab put a cracked pack in the case. Simulated scoring
checks confirm that a pack held back on the belt or past the red line earns nothing.

Why one arm falls short: a pack passes every 1.7 s, and the fastest single-arm cycle we scripted took about 4 s, which leaves
time for about five of the six packs before the last one reaches the line. An arm also reaches the far case's forward cell only
to within about 2 cm. This comes from these measurements and the arm's kinematics; it is not a proof that no single-arm policy
could do better.

## Scoring

Evaluated every control step; the score at the end of the episode counts.

- A pack is in a cell when its centre is inside the cell, it is within 10 degrees of flat, and its underside is within 6 mm of its
  case's floor, so a pack up on a wall doesn't count.
- A pack is lost once it crosses the red line while on the belt, or if it moves slower than 2 cm/s on the belt for 3 s, which
  rules out holding packs back. A lost pack earns nothing, even if it is packed later.
- 1.0 (success, held for 1 s): each of the six cells holds exactly one good pack that was never lost, both cracked packs are past
  the red line, and the packed packs are still.
- Otherwise, at the end: 0.15 for each cell holding exactly one good pack that was never lost, less 0.3 if a cracked pack is in a
  case, never below 0.

In a real trial: a photo from above shows which pack is in which cell, a tape measure checks a pack that may be up on a wall, and
a video shows the red line and any held-back pack.

## Runtime and seeds

A Dream-1 trial of 2,400 steps took about 5 minutes on one RTX-class GPU, plus about 1 minute for Isaac Sim to start (longer on a
fresh instance). The scripted demo takes a few minutes on CPU physics. The layout is fixed (`-E object_jitter=0`); there are no
seeds to choose.

## Fidelity

Packs are rigid, where real foil packs give a little. The belt is a fixed slab whose surface carries what rests on it, with no rollers or belt stretch, and it runs on CPU physics (-E device=cpu), because packs fall through it on GPU physics. Missed packs collect at the end stop. Nothing is deformable.

## Real-world build

A variable-speed belt conveyor 12 cm wide, set to 6 cm/s, with side rails and an end stop; eight 4 x 7 x 3.5 cm foil-wrapped packs,
two with a red band; two cases with three 7.5 x 10.5 cm cells, fixed to the table; a red mark beside the belt. The packs are light
and the belt is slow, so real trials are safe.
