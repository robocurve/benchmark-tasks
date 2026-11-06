# Set a sand core in its prints, then close the cope between the guide posts

A small sand mould stands in front of the arms: a brown drag (the lower half) with a 4 cm cavity between two walls, a
print ledge at each end of the cavity, and four black guide posts around it. A long grey core lies on two rests beyond
the drag and a long blue cope (the upper half) on two rests in front of it. The robot must lay the core along the
cavity with each end on its ledge, then lower the cope between the four posts so it sits flat on the drag. The core
leaves no room for the fingers to enter the cavity, and the cope seals the core in with no reach left to correct it. A
core left proud stops the cope sitting flat. The cope is a long bar, so two arms keep it level, but one centred grip
is not penalised and the scorer does not check how many hands were used. Not tested: sand behaviour or core
fragility.

## Files

- `set_the_core_and_close_the_mould.py`: the task file for the `yam_arena` embodiment (scene, `score()`, `success()`,
  `demo()`).
- `belimits.py`: small geometry helpers the task file imports (footprints, tilt, height checks).
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Set the grey core in the mould with each end on its ledge, then lower the blue cope flat between the posts."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=set_the_core_and_close_the_mould.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/core-dream

# Scripted demo (full score): the left arm drops the core onto the ledges, then both arms carry the cope over the
# posts together, lower it onto the drag, let go and park to the sides
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=set_the_core_and_close_the_mould.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 700 --store-frames --log-dir ~/yam-eval/core-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: the core's 1 cm end play and the cope's 1 cm clearance
  to the posts are measured from the fixed layout.
- **Runtime per trial:** the demo's arms finish in 27 s of simulated time (540 steps); a Dream-1 trial runs up to
  2,400 steps (2 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 120 s for the demo and about 380 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 5 s (core seated, cope closed, both at rest, both hands clear of the cope: success() starts once the hands clear, about 25 s, so it fires near 30 s, about 600 steps),
  otherwise `score()` at the end of the run.

## Scoring

At the end of the run (grippers opened). Heights are above the table top. Measure the cope first (height at both
ends, outline against the posts). Then lift the cope straight off and measure the core: its top is level with the drag
top within 0.4 cm, and it sits between the walls with both ends on the ledges.
Tape equivalents: core tilt 3 degrees = 1.4 cm difference between its ends over 26 cm; cope tilt 2 degrees = 0.24 cm
across its 7 cm width.

- **Core seated:** core centre at x 20.5 to 23.5 cm and within 1.2 cm of y 0, underside within 0.4 cm of 1 cm, tilt at
  most 3 degrees, at rest.
- **Cope closed:** cope outline inside the four posts (x 17.5 to 26.5, y -16.5 to 16.5), underside within 0.25 cm of 5 cm at
  both ends, tilt at most 2 degrees, at rest. Either face of either part may be up.

| Score | Condition |
|---|---|
| 1.0 | Core seated and cope closed, both at rest with both hands clear of the cope |
| 0.7 | Core seated, cope inside the posts but proud or tilted |
| 0.5 | Core seated, cope anywhere else |
| 0.2 | Core not seated but over the cavity (centre at x 20.5 to 23.5, within 2 cm of y 0), underside at most 2 cm above the table; cope not closed |
| 0 | Otherwise: a cope closed over an unseated core, a core lying across the walls, either part more than 2 cm below the table top (off the table) |

A moving or held cope counts as cope anywhere else (held: a gripper TCP within 2 cm of the cope's box); a moving core counts as not placed. A part more than 1 cm above every surface it could rest on counts as not placed (in the real trial the grippers are
opened first, so this only matters in simulation, where the last step is scored with the grippers still closed).
Doing nothing scores 0. Pushing cannot score: the undersides of the core (1 cm) and the cope (2 cm) meet the drag's 5 cm wall. Nothing is latched over the run; only the end state counts. The arms' path is not scored.

## Real-world reset

Lay the core and the cope back on their rests and brush any grit off the ledges and the drag top. The drag, posts and
rests stay fixed. A real trial can use a grey rigid foam stand-in (80 g) for the core; check it with a tape: seated, its top
is level with the drag top, and the closed cope is 5 cm above the table at both ends.
