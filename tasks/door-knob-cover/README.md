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
right hand lines its fingers up with the grip zones, squeezes (the finger drive at its 50 N limit flexes both zones
3.7 mm onto the knob), turns 54 degrees, pushes the door 3.4 degrees open with the grip held, lets go, then closes its
fingers and pushes the front of the cover until the door is 9 degrees open. The left arm stays at its ready pose.

Wrong outcomes on purpose (`KNOB_FAIL=`): `fixed` (the plan for the reference layout of the setup text, not the
poses), `light` (fingers just closed on the grip zones, no squeeze), `early` (the knob turned back before the push).

## Results (final code)

All on the same code (sha256 of the three Python files in each run's `sha256.txt`), `--max-steps 600`. Logs without
frames are in `runs/`. "Open" is the latch edge's best opening; "bolt" the most the bolt was drawn in (of 12.7 mm).

| Run | Seed | Score | Steps | Knob max | Bolt | Open | What happened |
|---|---|---|---|---|---|---|---|
| b1-s0 | 0 | 1.0 | 435 | 50 deg | 12.7 mm | 357 mm | unlatched at 45 mm with the grip held; pushed open |
| b1-s1 | 1 | 1.0 | 435 | 50 deg | 12.7 mm | 344 mm | same |
| b1-s2 | 2 | 1.0 | 435 | 50 deg | 12.7 mm | 333 mm | same |
| b1-demo-s0 (video) | 0 | 1.0 | 435 | 50 deg | 12.7 mm | 357 mm | as b1-s0, frames stored |
| b1-fixed-s0 | 0 | 0.0 | 435 | 12.9 deg | 3.0 mm | 3.4 mm | plan for the reference layout: one finger lands on a grip zone, the other on the shell; the one-sided press turns the knob 13 deg, then the cover slips round |
| b1-light-s0 | 0 | 0.0 | 435 | 11.5 deg | 2.6 mm | 3.2 mm | fingers only closed on the zones (2 mm flex): the cover spins 54 deg on a still knob; pushing the latched door moves it about 3 mm (strike play); a finger closing later flicks the knob 11.5 deg for 3 steps |
| b1-early-s0 | 0 | 0.3 | 465 | 50 deg | 12.7 mm | 2.9 mm | knob turned back before the push: the bolt was fully in, but the door never left the strike |
| dream-s0 (Dream-1, video) | 0 | 0.0 | 600 | - | - | - | model run for the full run limit |

The door opens past 100 mm by step 320 and then coasts on (low hinge friction). Each seed shifts the wall and door
(seeds 0, 1, 2: x +16, +28, +20 mm) and turns the cover (148, 101, 96 degrees).
