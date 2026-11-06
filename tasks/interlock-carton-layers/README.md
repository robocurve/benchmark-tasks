# Stack cartons in two interlocked layers

Four identical narrow shipping cartons lie beside a square pallet board, all pointing the same way. The robot
must build a two-layer unit load on the board, two cartons per layer, with the second layer turned 90 degrees so
each top carton rests across both cartons below it (an interlocked pattern, not a column stack). Two of the
cartons have to be turned on the way; which two, and which way the first layer runs, is up to the policy.

| File | What |
|---|---|
| `task.py` | the task file: objects, `success`, `score`, `distance`, scripted `demo` |
| `task.json` | the submission fields (`media` is added by the upload script) |
| `test_task.py` | offline checks (NumPy and pytest, no Isaac Sim) |

## Run a trial

From a checkout of the arena (yam-isaac-lab-arena, tested at `bd62771`) on the GPU machine, with this folder at
`tasks/interlock-carton-layers/`. The layout is fixed (`-E object_jitter=0`), so no seed changes the scene; the
scripted demo is open loop and needs the fixed layout.

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/interlock-carton-layers/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stack the four cartons on the pallet board in two layers of two, with the second layer turned 90 degrees to the first so that each top carton rests across both cartons below it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/interlock-carton-layers-dream
```

Scripted demo (the arms take turns, 43.5 s of waypoints ending in a 2 s still hold; success should end the trial
at about 41 s, roughly step 820-880):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/interlock-carton-layers/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stack the four cartons on the pallet board in two layers of two, with the second layer turned 90 degrees to the first so that each top carton rests across both cartons below it." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/interlock-carton-layers-demo
```

`--max-steps 1000` is the plan (870 steps) plus about 130; running both at the 2,400-step
run limit is also fine.

Videos, one MP4 per camera, in each log's `frames/` folder:

```bash
for log in ~/yam-eval/interlock-carton-layers-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end: 1.0 when the trial ended in success, otherwise `score()` at the last
step. Put the Dream-1 number in `task.json` `scores`, e.g. `"Dream-1 run: 0.4; demo: 1.0"`.

Runtime per trial: 2,400 steps (2 minutes simulated) for Dream-1, about 44 s simulated for the demo, plus Isaac
start-up. Wall clock on an RTX 5090, start-up included: 97 s for the demo, 228 s for the Dream-1 run.

Offline tests (about 25 s):

```bash
YAM_ARENA_DIR=/path/to/yam-isaac-lab-arena python -m pytest tasks/interlock-carton-layers/test_task.py -q
```

## Scoring at a glance

| Result | Score |
|---|---|
| Two cartons flat on the board, two on top turned 90 +/- 10 deg and each bridging both (1/5 of its length on each) | 1.0 (success after 1 s) |
| Same, but one top carton the same way round as the layer below | 0.8 |
| Column stack (second layer the same way round as the first) | 0.6 |
| Single layer of two (or three) on the board | 0.4 |
| Any carton tilted more than 20 deg (on its side or end, collapsed stack) or off the table | 0 |

Per carton: 0.2 first layer (at most two), 0.3 interlocked second layer, 0.1 second layer not interlocked (at most
two). Gaps inside a layer are allowed, so a stack pushed tight scores 1.0 too, and so does the whole pattern turned
90 deg (first layer pointing away from the arms). `distance()` is the summed metres from the cartons to the nearest
slot assignment of either turn of the pattern.

## Design numbers

- **Gripper.** At `FINGER_OPEN` (-0.04695 m per finger) the pads are 94 mm apart (arena `tests/test_constants.py`);
  the widest allowed grip is 84 mm. Every carton is gripped across its 75 mm width, so the fully open fingers
  clear each side by 9.5 mm on the way down. The 180 mm length and 50 mm height cannot be gripped straight down
  with the long side between the pads, and a carton on its side stands 75 mm tall, not 50 mm, so it never scores.
- **Finger room between neighbours.** The demo leaves a 30 mm gap between the two cartons of a layer. The offline
  sweep plays every control step against the URDF collision hulls with the fingers taken **wide open** whenever a
  carton is held, because either pad can sit at its open stop when the grip is off centre: the closest any link
  comes to the neighbouring carton is 13.6 mm (first layer, carton 3 next to carton 1) and 14.1 mm (second layer).
  The carried carton, modelled 15 mm off centre either way between the pads, stays 15 mm clear of its neighbour.
  The release is a 4 mm loosening of each pad 5 mm above the support (in sim the pads re-centred a carried item
  when they loosened), then a 3 cm straight lift.
- **Grip offset in the scoring.** The pattern is centred on the board with 30 mm of board on every side, and the
  test scatters every carton of the final state by up to +/-15 mm and +/-4 deg: all draws still score 1.0. The
  bridging threshold (1/5 of the length, 36 mm) leaves 39 mm of slack against the nominal 75 mm.
- **Reach and joint limits.** Board centre at x = 0.25 m; the first layer runs across the arms (along y) so its
  cartons, which straddle the midline, are released low (TCP 5.3 cm above the table). The second layer points away
  from the arms, gripped at y = +/-5.25 cm. Travel height 14 cm (the stack top is at 11.2 cm). Worst IK error
  0.08 mm and 0.015 deg; joint4 keeps 0.117 rad of margin (left arm), joint6 0.84 rad. The first-layer turn is
  -90 deg for the left arm and +90 deg for the right; the other way round puts joint6 on its limit.
- **Timing.** The arms take turns at the board: the waiting arm picks its next carton and holds it at its park
  spot until the other arm has left the board plus 0.6 s. Lowering 8.7 cm takes 2.6 s (3.3 cm/s on average).
- **Last frames.** The episode ends on success about 1 s after the last carton comes to rest, while the right arm
  is still lifting away, so in the recorded demo's last seconds the right wrist covers most of the finished stack;
  the scorer's 1.0 is the record of the interlocked pattern.
- **Changes from the first design.** It had eight 20 x 10 x 10 cm cartons on a 40 x 30 cm board. A
  10 cm carton is wider than the 84 mm grip in every direction but its length; two 10 cm layers put the travel
  height past joint4's limit; a 40 cm board reaches past the arms' straight-down range on the midline; and eight
  cartons at about 10 s each, all needing a finger gap, would fill the run limit with a scripted demo alone. So:
  four cartons, 18.0 x 7.5 x 5.0 cm, two per layer, on a 24 x 24 cm board. Two per layer keeps the rule meaningful:
  a top carton can only bridge the joint below if it is turned 90 deg, and the column stack is the natural wrong
  answer. Its scoring (0.5 x bottom/4 + 0.5 x rotated top/4, 0 on collapse) became 0.2 / 0.3 per carton,
  plus 0.1 for a second-layer carton that is stacked but not interlocked, plus the bridging condition (so turned
  cartons both perched on one carton below do not count), plus an explicit collapse test (tilt over 20 deg). All
  four cartons start the same way round, as they would come off an infeed, so the policy has to decide which two
  to turn.

## Novelty: nearest existing tasks

- **Board "Stack cartons flush" (B58, previous cohort, planned):** three cartons stacked with adjacent edges
  aligned within 2 mm. That is an alignment-precision task on a single column. This one is the opposite trade:
  loose position tolerances (board edge, 10 deg, gaps allowed) and a **pattern rule**: alternate the layer
  orientation and make each top carton span the joint below. A flush column stack, the B58 goal, scores 0.6 here.
- **Board "Build a DUPLO bridge" (B81):** a spanning brick across two pillars, but by snap-fit with one arm
  holding the pillar; no orientation rule, no layering.
- **RoboCasa `StackCans`** ("stack two cans on top of the other two cans") and LIBERO / RoboCasa bowl stacking
  (`StackBowlsCabinet`, `KITCHEN_SCENE2_stack_...`): stacking with no layer pattern; any on-top placement counts.
- No task we found in LIBERO, RoboCasa, BEHAVIOR or on the RoboCurve board (2026-11-02) scores an interlock or
  column-versus-brick palletizing pattern.

## Assets and licences

None. Every object is a primitive box defined in `task.py`; the room and robot come from the arena.

## Verification

Run on an RTX 5090 with `yam-isaac-lab-arena`, using the commands above (`object_jitter=0`):

- Scripted demo (`demo()`): `task_score` 1.0, ended by success; 97 s wall clock including Isaac start-up.
- Dream-1: `task_score` 0.0 after the full 2,400-step run limit; 228 s wall clock.
- Offline tests: 32 passed.
