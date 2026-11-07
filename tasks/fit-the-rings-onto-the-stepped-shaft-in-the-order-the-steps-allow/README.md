# Fit two rings onto a stepped shaft in the order their bores allow

A stepped steel shaft stands on a base plate: a 30 mm lower section with a shoulder 35 mm up, and an 18 mm upper
section above it. Two loose square rings lie on the table, both 60 mm square outside. The blue ring, in front of the
right arm, has a 38 mm bore, so it passes both sections and its seat is the plate. The orange ring, in front of the
left arm, has a 26 mm bore, which passes only the upper section, so its seat is the shoulder. A 26 mm square bore
cannot pass a 30 mm round section at any turn, because its inscribed circle is 26 mm.

The order is forced by the bores. Blue must go on first. If orange goes on first it sits on the shoulder, and blue
then lands on top of orange, 60 mm above the plate, and cannot reach its seat until orange is taken off again.
The rings look alike apart from colour and bore, so neither size, side nor arm gives the order away: a fixed habit
such as "left arm first" fits orange first and scores 0. Because the layout is fixed, a policy that memorised "blue
first" would succeed without reading the bores. The instruction states only the goal.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena, run through Inspect Robots), on the `station` scene. It uses `parts.py` (in this folder) to build each
ring from four boxes. `near_misses.py` lists near-successes and wrong outcomes with the score each must get; it runs
without a simulator.

## Run a trial

From this folder, on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Fit both rings onto the stepped shaft, each one down on its own seat."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 2400 --store-frames

# Scripted demo (reaches 1.0 after about 26 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 700 --store-frames
```

The run prints `task_score`: 1.0 on success, otherwise 0.

- **Runtime per trial:** at most 120 s of simulated time (2,400 steps at 20 Hz), a few minutes of wall time on an
  L40S once Isaac Sim has started.
- **Seeds:** none needed. Submissions run with `-E object_jitter=0`, so every object starts exactly where `task.py`
  puts it.

## Scoring details

Read from the ring poses at the end of the trial (`score()`), and `success()` must hold for 1 s:

- **Blue seated:** the blue ring's centre within 6.5 mm of the shaft axis (only a ring around the shaft can be
  there: its bore lets it move at most 5.7 mm), the lowest point of its bounding box within 1 mm of the plate top,
  its own vertical axis within 3 degrees of vertical, and it is still (under 1 cm/s and 0.5 rad/s).
- **Orange seated:** the same, with the lowest point within 1.5 mm of the shoulder top (35 mm above the plate).
- **Pass/fail:** 1.0 when both are seated, otherwise 0.

Every credited state is a ring resting on its seat. The demo releases each ring 8 mm above its seat (it drops about
4-5 mm after the arm's sag), so a ring still held at the release point does not count; it must fall onto the seat.
One ring alone earns nothing, nor does the stack the wrong order leads to (blue stuck on orange).

## Demo

Fixed waypoints, straight-down grips across each ring's outside, no feedback from object poses. The right arm picks
up blue, carries it over the shaft, lowers it to 8 mm above the plate and opens; then it parks. The left arm then does
the same with orange, releasing it 8 mm above the shoulder. Each gripper is told to close only 3 mm past the ring's
faces: when told to close fully, one finger overpowers the other and the ring ends up held 13-25 mm off centre in the
grip (seen in the first GPU trace).

`RING_DEMO=wrong` plays the wrong order (left arm and orange first, then blue), for checking the scorer on the GPU: blue comes to
rest on top of orange, 70 mm above the table, and the trial scores 0. `RING_DEBUG=<file>` logs the finger-tip link
positions; both are for debugging only and change nothing in a normal run.

## Checked

- Offline: `near_misses.py` (23 cases: each tolerance on both sides, tilted and turned rings, moving rings, one
  ring alone, a ring held in the air, the wrong order a "left arm first" habit gives, a ring on the floor) and the arm's reach and clearance along the demo. The offline checker
  models the fingers fully closed, which puts the finger tips inside the shaft while a ring is lowered over it, so
  the shaft is listed in `DEMO_TOUCHES`; with the real finger openings the fingers stay at least 13 mm clear of the
  fixture's bounding boxes.
- GPU (yam-isaac-lab-arena on an L40S): the demo scores 1.0 (blue 2.6 mm off the axis, orange 0.6 mm);
  the wrong-order replay (left arm and orange first) scores 0 with blue stuck on orange, 60 mm above the plate.
  Dream-1 (2,400 steps) scores 0: it reaches for the orange ring with the left arm and fits neither.

## Licence

All geometry is built from primitives in `task.py`; no third-party assets. Released under the repository's licence.
