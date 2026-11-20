# Open a door through a child-safety knob cover

A full-size 30 x 80 in hollow-core interior door hangs in a wall along the right edge of the station table. Its
passage knob wears a child-safety cover that spins freely on the knob. The robot must squeeze the cover's two grip
zones hard enough to grip the knob through them by friction, turn the knob against its spring until the latch bolt is
in, keep that grip while pushing the door past the strike, and open the door at least 100 mm at its latch edge.


## Run and score a trial

```
inspect-robots run --embodiment yam_arena -E task=door_knob_cover.py -E object_jitter=0 -E control_hz=20 \
  --instruction "The door's knob has a child-safety cover on it. Open the door at least 10 cm at its latch edge using the knob, without taking the cover off." \
  --scorer task_score --max-steps 600 --policy <policy> --seed <seed>
```

`dev/run.sh <name> <policy> <seed> 600 [KNOB_FAIL=...]` does the same and writes a per-step log (knob angle, cover
angle, grip-zone flex, bolt travel, door angle, latch-edge opening, score). `--scorer task_score` reports the best
score of the trial (`SCORE_OVER = "best"`).

- Run limit: 600 steps (30 s at 20 Hz). The scripted demo takes 435 steps.
- Runtime per trial: about 16 minutes wall clock on the shared GPU machine (including about 3 minutes of start-up).
- Seeds: any. Each seed shifts the wall, frame and door together by up to 40 mm along x and up to 10 mm away from the
  table, and turns the cover 0 to 180 degrees on the knob (`layout()`).

## Scripted demo

`--policy yam_scripted` plays the demo. It **reads object poses from the simulator** to plan (the knob axis, the
cover's angle on the knob, the door's hinge line); it moves the parts only through the grippers, by contact. The
right hand lines its fingers up with the grip zones, squeezes (a finger target 8 mm past the knob at the finger drive's
5,000 N/m: about 40 N per finger, under the 50 N cap that the gripper maker's driver also sets; both zones flex
3.7 mm onto the knob; the turn to the stop needs about 29 N per finger), turns 54 degrees, pushes the door 3.4 degrees open with the grip held, lets go, then closes its
fingers and pushes the front of the cover until the door is 9 degrees open. The left arm stays at its ready pose.

Wrong outcomes on purpose (`KNOB_FAIL=`): `fixed` (the plan for the reference layout of the setup text, not the
poses), `light` (fingers just closed on the grip zones, no squeeze), `early` (the knob turned back before the push).

## Scoring in the simulator

The score uses the latch edge's best opening (100 mm: 1.0; 20 mm: 0.6) and the knob's best turn from level (45
degrees, where the bolt is fully drawn: 0.3; 15 degrees: 0.1). The knob angle is the one a person reads on the far
side's dial in a real trial: the far-side knob is part of the same spindle body, with its index line and the dial marks
(0, 15 and 45 degrees each way) drawn on it and on the far door face; nothing collides with them.

## Results (final code)

All on the same code (sha256 of the three Python files in each run's `sha256.txt`), `--max-steps 600`. Logs without
frames are in `runs/`. "Knob max" is the knob's largest turn from level (what the far side's dial shows); "bolt" the
most the bolt was drawn in (of 12.7 mm); "open" the latch edge's best opening.

| Run | Seed | Score | Steps | Knob max | Bolt | Open | What happened |
|---|---|---|---|---|---|---|---|
| c1-s0 | 0 | 1.0 | 435 | 50 deg | 12.7 mm | 357 mm | unlatched at 45 mm with the grip held; pushed open (100 mm at step 307) |
| c1-s1 | 1 | 1.0 | 435 | 50 deg | 12.7 mm | 344 mm | same (100 mm at step 308) |
| c1-s2 | 2 | 1.0 | 435 | 50 deg | 12.7 mm | 333 mm | same (100 mm at step 309) |
| c1-fixed-s0 | 0 | 0.0 | 435 | 12.9 deg | 3.0 mm | 3.4 mm | plan for the reference layout: one finger lands on a grip zone, the other on the shell; the one-sided press turns the knob 13 deg, under the 15 deg line, then the cover slips round |
| c1-light-s0 | 0 | 0.0 | 435 | 11.5 deg | 2.6 mm | 3.2 mm | fingers only closed on the zones (2 mm flex): the cover spins on a still knob; pushing the latched door moves it about 3 mm (strike play); a finger closing later flicks the knob 11.5 deg for 3 steps |
| c1-early-s0 | 0 | 0.3 | 465 | 50 deg | 12.7 mm | 2.9 mm | knob turned to its stop (past 45 deg, bolt fully in), then back before the push: the door never left the strike |
| dream-try1 (Dream-1, video) | 0 | 0.0 | 600 | 0 deg | 0 mm | 0.4 mm | model run for the full run limit: the knob never turned |

The demo video is the seed 0 run of the previous code with frames stored. Since then only the far side's knob, dial
and index line (out of every camera's view) and the 0.3 tier's test changed; c1-s0 gives the same result (435 steps,
357 mm).

The door opens past 100 mm by step 309 and then coasts on (low hinge friction). Each seed shifts the wall and door
(seeds 0, 1, 2: x +16, +28, +20 mm) and turns the cover (148, 101, 96 degrees).
