# Lube a bike chain

A whole 14 inch child's bike stands across the table in two wheel stands, chain side toward the robot. One arm takes
the squeeze bottle of lube, tips it nozzle-down over the top run of the chain and squeezes; the other turns the near
pedal backwards so every link passes under the nozzle. Then the bottle goes back upright on the table.

## Files

- `lube_a_bike_chain.py`: the task file (layout, scorer, scripted demo).
- `drivekit.py`: the bike. The drivetrain (cranks, chainring, 56-link chain, cog), the fixed frame, wheels, bars,
  saddle and stands, the squeeze bottle, and the rule for drops.
- `labkit.py`: helper that spawns these objects, resets them and runs demo steps planned from the simulator's state.
- `test_lube_a_bike_chain.py`: scoring tests on made-up end states (no simulator needed).

Third-party assets: none. Every shape is built in code for this task (CC0); there are no meshes or textures.

## Run a trial

From the simulator's folder, with its environment active. Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream -P model=dreamlabs/dream-1-beta \
  -E task=tasks/lube-a-bike-chain/lube_a_bike_chain.py \
  -E object_jitter=0 -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lube the bike chain: drip the lube onto the chain while turning the crank backwards." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir evaluations/dream
```

Scripted demo (`-E object_jitter=0` is needed: its grasps are fixed for the one layout):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/lube-a-bike-chain/lube_a_bike_chain.py \
  -E object_jitter=0 -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lube the bike chain: drip the lube onto the chain while turning the crank backwards." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir evaluations/demo
```

The run prints `task_score` at the end. Scoring: `score()` and `success()` in the task file, and `scoring` in
task.json. `LUBE_LOG=1` in the environment prints the bottle's pose, tilt and squeeze twice a second; every drop is
printed with the link it landed on, or as a miss.

Physics runs on the CPU (`-E device=cpu` in every command); CPU and GPU PhysX can give different results, and this
task has only been run on the CPU.

## Runtime and seeds

- Demo: full score at step 1,726 (86 s simulated), when the trial ends itself. About 15 minutes of wall time on one
  A10 with CPU physics, of which about 3 minutes is the simulator starting.
- Dream-1: 3,000 steps (150 s simulated); about 7 minutes of wall time on an L4.
- `object_jitter=0` keeps the one layout fixed, so no seed is needed.

## The submitted runs

- Demo: 93 drops released, none missed, 51 of the 56 links wet (50 needed), the cranks turned 4.25 turns backwards,
  the bottle set back upright 5 mm from its start. `task_score` 1. The video is the whole trial, 1,727 frames at
  20 a second.
- Dream-1: took the bottle by its neck, lifted it and carried it slowly toward the robot, upright; it never tipped
  or squeezed it and never touched a pedal. No drops. `task_score` 0.

## How the demo works

It is closed-loop but reads only what cameras could show: once a second it reads where the bottle's nozzle is and
how far the cranks have turned, then moves the left hand most of the way to put the nozzle over the chain and sends
the right hand 39 degrees further round the pedal's circle. It never moves an object. The two grasps (the bottle,
4.5 cm above its base; the pedal, from above at a steep angle) were first found by driving the arms by keyboard.

The left hand turns 25 degrees about the vertical before tipping the bottle. That keeps hand and bottle on the rear
wheel's side of the near pedal's sweep; pointing straight ahead, the two hands meet at about 200 degrees of crank.

## Checks without the simulator

```bash
python -m pytest -q tasks/lube-a-bike-chain/test_lube_a_bike_chain.py
```

They need `yam_arena` importable but not Isaac Sim.

## Status and limits

- Run in Isaac Sim on CPU physics only: the scripted demo scores 1.0 (twice, with the same counts), Dream-1 scores 0.
- The chain is 56 separate links driven only by the teeth; the measured drive ratio is 1.86 (26/14 = 1.857).
- No liquid is simulated. Drops are counted by a rule (nozzle at least 20 degrees below level, wall squeezed in
  3 mm a side, one drop each 0.45 s) and wet the link under the nozzle. Lube does not run, wick or fling off.
- The simulated bottle needs about 24 N a side to drip, more than a real 4 oz bottle (roughly 5 to 10 N). The
  stiffer wall lets a position-controlled gripper hold the bottle dry at 18 N and squeeze it as a separate act.
- The bike's frame, wheels and stands are fixed, plain shapes; the rear wheel does not turn.
- The physical procedure in task.json (tinted lube, marked link, white sheet, weighing the bottle) has not been
  tried on a real bike.
