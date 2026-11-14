# Stop a glass bottle rolling along the worktop before it goes over the open edge

A glass beer bottle has just been knocked over on a sloping drainboard: it rolls off it and along a worktop towards
the open end on the robot's right. The instruction is "Stop the bottle before it rolls off the edge." Everything about
it is timing: the robot has to see where the bottle is going and how fast, get a hand into its path before it gets
there, and stop it without knocking it off. It is a moving target, which no placement task asks for.

## The real step
Lunging for a bottle rolling off a counter is something most people have done; a full glass bottle that goes over
the edge smashes. Done by: anyone in a kitchen or a bar.

What goes wrong in real life: too late, and it goes over; too hard, and it is knocked off sideways or back. A look at
the worktop afterwards (bottle on it, still) checks it.

What we simplified: the worktop is a 6 cm block on the table, so going over the edge is a short drop here (any fall
scores 0). The bottle is a capsule at full size and weight; it rolls straight, where a real bottle's shoulder
makes it curve a little. In this simulator a rolling capsule gains a little speed
on a level surface once it moves, so a stopped bottle let go of can start rolling again; the task counts a bottle
stopped and kept still, with or without a hand in front of it.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=rolling_bottle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stop the bottle before it rolls off the edge." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/rolling-bottle-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=rolling_bottle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stop the bottle before it rolls off the edge." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/rolling-bottle-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/rolling-bottle-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the bottle has been stopped and still on the
worktop for 1 s; otherwise it scores the partial credit below at the last step.

## Setup
Positions in cm on the table, from the midpoint between the two arm bases: x forward (away from the arms), y to the
robot's left.

| Object | Description | Position |
|---|---|---|
| worktop | fixed laminate worktop section, 30 cm deep, 55 cm long, top 6 cm above the table, open end on the robot's right | x 15 to 45, y -22.5 to 32.5 |
| drainboard | stainless steel, 30 x 14 cm, loose but 15 kg, sloping 1.2 degrees down towards the right | y 18 to 32, on the worktop's left end |
| bottle | 330 ml brown glass beer bottle, full: 530 g, 6.1 cm across, 23 cm long; lying on its side, axis along x, neck away from the arms; rolling from the start | (29, 30), on the drainboard |

The bottle reaches the worktop at about 0.25 m/s and, left alone, goes over the edge about 1.6 s after the trial
starts. Every loose object has friction 1.2 static, 1.0 dynamic. Both grippers start open at the ready pose. One
fixed layout: `-E object_jitter=0`.

## Scoring

| Score | When |
|---|---|
| 1.0 | the bottle still (under 1 cm/s) on the worktop or the drainboard, lying or stood up, resting on it (not held up off it), never having gone over an edge; held for 1 s, which ends the trial (if time runs out first, this state scores 0.9). A hand may stay in front of it |
| 0 | anything else: still rolling, held up, on the table, or over an edge at any point |

Going over an edge is recorded as it happens (the bottle's lowest point more than 2 cm below the worktop's top) and
lasts for the rest of the trial, so a bottle that went over and was put back still scores 0; each reset clears it.
The lowest point comes from the bottle's centre and tilt (the simulator's bounds for a lying capsule sit about 1 cm
low). There is no part credit for a bottle still rolling: it starts that way, and on this simulator's level worktop a
rolling capsule doesn't slow down.

## The scripted demo
Open loop, from the fixed start: the right arm comes straight down with its fingers closed, their 26 mm width across
the bottle's path, and stands them on the worktop 5 cm before the edge; the bottle rolls into them at about 0.33 m/s
and stops, and the fingers stay there until the trial ends on success.

What the traces taught (official sim, 2026-11-14): on a 3 degree drainboard the bottle was over the edge 1 s into
the trial, before an arm could get there, so the slope is 1.2 degrees; a fixed mechanism needs at least one joint, so
the worktop's edge trim sits on a stiff one; the bounds reported for a lying capsule sit about 1 cm low, so heights
come from its centre; and a rolling capsule gains speed on the level, so the demo keeps its fingers in place.

## Mechanism and part modules
`mechanism.py` is copied unchanged from robocurve/benchmark-tasks pull request #223 (MIT License, notice in its
header); it builds the fixed worktop and runs a hook at each reset that clears the "went over" record. `parts.py`
(from the accepted `tasks/install-processor`, with round pieces added here) builds the bottle and the drainboard.

## Runtime and seeds
- The scripted demo's plan takes 1.6 s to put its fingers in the bottle's path, then waits; it ends on success at
  about 2.6 s (53 steps at 20 Hz), well within the 2,400-step limit.
- Official runs (2026-11-14): the demo 1.0, the bottle stopped with its middle 7 cm short of the edge; traced with the fingers
  1 cm earlier and 1 cm later in its path (and 1 cm across), also success at 2.55 and 2.6 s; Dream-1 0.0 over the
  full 2,400 steps.
- The scoring and reach checks run without the simulator: `python -m pytest -q test_rolling_bottle.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
