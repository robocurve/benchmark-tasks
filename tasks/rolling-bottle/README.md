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
makes it curve a little. PhysX has no rolling resistance, so the bottle is given one (below), tuned to slow it as a
real bottle slows on a worktop.

## Reproducing it for real
The release and the speeds are set so a real trial matches the simulated one:
1. Lay a stainless steel drainboard (30 x 14 cm) on the worktop's left end, sloping 2.2 degrees down towards the
   open end: a 5.4 mm shim under its high end. The worktop's open end is 40 cm from the drainboard's foot.
2. Lay a full 330 ml brown glass beer bottle on its side across the drainboard, its middle 13 cm up from the
   drainboard's foot (a 5 mm drop), held by a straightedge laid across the drainboard just below it.
3. Lift the straightedge straight up. The bottle rolls from rest.
4. Check it with a 60 fps phone video against a ruler laid along the worktop: the bottle crosses the drainboard's foot
   at about 0.24-0.27 m/s (from its energy: the glass rolls, the beer mostly slides with it) and slows by about
   0.02-0.05 m/s each second along the worktop. In the simulator it crosses the foot at 0.23-0.25 m/s (0.275 m/s just
   after the drainboard's 3 mm edge), slows by 0.038 m/s each second, and reaches the open end 1.6 s after the foot,
   at 0.22 m/s.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=rolling_bottle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stop the bottle before it rolls off the edge." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/rolling-bottle-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=rolling_bottle.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stop the bottle before it rolls off the edge." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/rolling-bottle-demo

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
| drainboard | stainless steel, 30 x 14 cm, 3 mm thick, loose but 15 kg, sloping 2.2 degrees down towards the right | y 18 to 32, on the worktop's left end |
| bottle | 330 ml brown glass beer bottle, full: 530 g, 6.1 cm across, 23 cm long; lying on its side, axis along x, neck away from the arms; let go at rest 13 cm up the drainboard (a 5 mm drop) | (29, 31), on the drainboard |

The bottle is let go as the reset starts, so when the trial starts (the reset spends about 1 s settling the scene) it
is just reaching the worktop, at about 0.23 m/s. Left alone, it goes over the open end 1.7 s after the trial starts.
Every loose object has friction 1.2 static, 1.0 dynamic. Both grippers start open at the ready pose. One
fixed layout: `-E object_jitter=0`.

## Scoring

| Score | When |
|---|---|
| 1.0 | the bottle still (under 1 cm/s) on the worktop or the drainboard, lying or stood up, resting on it (not held up off it), never having gone over an edge; held for 1 s, which ends the trial (if time runs out first, this state scores 0.9). A hand may stay in front of it |
| 0 | anything else: still rolling, held up, on the table, or over an edge at any point |

Going over an edge is recorded as it happens (the bottle's lowest point more than 2 cm below the worktop's top) and
lasts for the rest of the trial, so a bottle that went over and was put back still scores 0; each reset clears it.
The lowest point comes from the bottle's centre and tilt (the simulator's bounds for a lying capsule sit about 1 cm
low). There is no part credit for a bottle still rolling: it starts that way, and left alone it is still rolling at
0.22 m/s when it reaches the open end.

## The bottle's rolling resistance
`parts.py` pushes the bottle against its horizontal motion before every physics substep (240 Hz) with a force of its
mass times 0.05 m/s^2 (0.026 N), faded in over its first 1 cm/s so a bottle at rest isn't pushed about. Real rolling
resistance is about constant with speed, and PhysX has none. A rolling solid cylinder takes 1.5 times its mass to
slow, so this slows the bottle by 0.033 m/s each second; the simulator measured 0.038 (0.04 m/s^2 gave 0.031, 0.06 gave
0.045).

The worktop's top 6 mm is a fixed box. A capsule rolling on the mechanism's base link (an articulation) gained about
0.06 m/s each second on the level, so a bottle that had stopped rolled off again; on the fixed box, without the
rolling resistance, it kept its speed (0.297 to 0.292 m/s across the worktop).

## The scripted demo
Open loop, from the fixed start: the right arm comes straight down with its fingers closed, their 26 mm width across
the bottle's path, and stands them on the worktop 4 cm before the edge; the bottle rolls into them at about 0.23 m/s
and stops, and the fingers stay there until the trial ends on success.

What the traces taught (official sim, 2026-11-14 and 2026-11-17): on a 3 degree drainboard the bottle was over the
edge 1 s into the trial, before an arm could get there; a fixed mechanism needs at least one joint, so the worktop's
edge trim sits on a stiff one; the bounds reported for a lying capsule sit about 1 cm low, so heights come from its
centre; a capsule rolling on an articulation link gains speed, so the worktop's top is a fixed box; and with linear
damping instead of a rolling resistance, a bottle that bounced slowly back off the fingers sped up again (damping
grows with speed, so it could only balance that gain at one speed).

## Mechanism and part modules
`mechanism.py` is copied unchanged from robocurve/benchmark-tasks pull request #223 (MIT License, notice in its
header); it builds the fixed worktop's body and runs a hook at each reset that clears the "went over" record.
`parts.py` (from the accepted `tasks/install-processor`, with round pieces and the rolling resistance added here)
builds the bottle and the drainboard.

## Runtime and seeds
- The scripted demo's plan takes 1.6 s to put its fingers in the bottle's path, then waits; the arm's per-step limit
  makes it arrive a little later. It ends on success at 2.65 s (53 steps at 20 Hz); the 1,000-step limit (50 s) is
  about 19 times that.
- Official runs (2026-11-17, the Robocurve Brev launchable, 1,000 steps, `object_jitter=0`): the demo 1.0, the bottle
  rolling into the fingers at 1.45 s and the trial ending on success at step 53; a state trace of the same demo also
  ended on success. Dream-1 0.0 over the full 1,000 steps: it barely moved either hand for the first 2 s, and the
  bottle went over the open end; going over is recorded, so no later step could have changed the score.
- Free roll (the same scene, both arms held clear, traced): the bottle crosses the drainboard's foot 0.05 s into the
  trial at 0.23-0.25 m/s, slows from 0.275 to 0.224 m/s across the worktop, and goes over the open end at 1.7 s.
- The scoring, the release and the reach checks run without the simulator: `python -m pytest -q test_rolling_bottle.py`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
