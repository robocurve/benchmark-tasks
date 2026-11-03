# Store oxygen apart from fuel-gas cylinders

Three welding gas cylinders, at 1:20, stand in a row with the oxygen between two acetylene cylinders. The robot
stores them as US federal rules require (29 CFR 1910.253(b)(4)(iii)): oxygen separated from fuel gas by at least
20 ft, or by a noncombustible barrier at least 5 ft high. The instruction names the gases and the rule's source, not
the distances. Two fixed walls stand nearby: a 6 ft one, which is a barrier, and a 3 ft one, which looks like one but
is too low. Either answer works: move the oxygen (or the acetylene) 20 ft away, or put the tall wall between them. The
cylinders are tall and narrow, so they must be carried and set down without tipping.

## Run a trial
On a machine set up with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s
`setup.sh`), with a Dream API key in `~/.dreamlabs/credentials`:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=gas_cylinders.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Store these welding gas cylinders as US federal rules require. The green cylinder holds oxygen and the two maroon ones hold acetylene. The scene is at 1:20 scale." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/gas-cylinders-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=gas_cylinders.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Store these welding gas cylinders as US federal rules require. The green cylinder holds oxygen and the two maroon ones hold acetylene. The scene is at 1:20 scale." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/gas-cylinders-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/gas-cylinders-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends with 1.0 when `success()` has held for 1 s; otherwise it scores
`score()` at the last step.

## Scoring
Distances are at scale (1 ft = 15.2 mm).

| Condition | Number | Check on a real table |
|---|---|---|
| a cylinder is upright | standing on the table (bottom within 2 mm), axis within 10 degrees of vertical | a square |
| an acetylene cylinder is separated from the oxygen | a gap of at least 20 ft (305 mm) between them, or the line between their centres crosses a wall at least 5 ft (76 mm) high, at least 16 mm in from its ends | a ruler; a string between the two |

| Score | Condition |
|---|---|
| 0 | any cylinder not upright (down, tipped, lifted off the table) |
| 1.0 | both acetylene cylinders separated from the oxygen, every cylinder upright and still for 1 s (ends the trial) |
| 0.5 | one separated |
| 0 | none |

A knocked-over cylinder is a hazard of its own, so any cylinder down scores 0 for the trial at that moment. The line
test makes the barrier count only when it stands between the two cylinders; a line that only grazes the wall's end
doesn't count. The 3 ft wall never counts. `score()` counts while a cylinder moves; `success()` also needs every
cylinder still. The hooks don't see the grippers, so a cylinder held standing on the table counts the same as one let
go.

## Runtime and seeds
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
- Dream-1 gets 1,200 control steps (1 minute at 20 Hz); the scripted demo finishes in about 14 s.
- `python -m pytest -q test_gas_cylinders.py` checks the scoring and the demo's reach without the simulator.
