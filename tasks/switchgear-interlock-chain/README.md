# Work a chain of interlock bars in the only order it allows to reach a fuse carrier

A white fuse carrier sits in a bay roofed by a sliding blue door with a 3.5 x 3.5 cm window, through which the carrier's white top shows (the 4.5 cm carrier cannot pass it). The door is held by an orange bar, which is held by
a yellow bar, which is held by a red bar, and each bar's tip stands in the next one's path. Nothing says which to
move first: the order has to be read from how the hardware blocks itself. A bar that has moved also stops the one
before it from sliding back. The robot must slide the bars and the door, lift the carrier out and set it on the green
mat. The score rewards each stage in order, with 1.0 for the carrier on the mat.

## Files

- `switchgear_interlock_chain.py`: the task file for the `yam_arena` embodiment (scene, `score()`, `success()`,
  `demo()`).
- `mechkit.py`, `belimits.py`: small helpers the task file imports (moving parts, and outline and height checks).
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Take the white fuse carrier out of the locked bay and set it on the green mat. The bay door is held by a chain of interlock bars. See how they block each other and slide them in order."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=switchgear_interlock_chain.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/sic-dream

# Scripted demo (full score): the right arm slides the red and yellow bars and parks; the left arm slides the orange
# bar and the door, then carries the carrier out and sets it on the mat
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=switchgear_interlock_chain.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 1400 --store-frames --log-dir ~/yam-eval/sic-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps the carrier exactly in the middle of the bay, where
  the demo takes it from.
- **Runtime per trial:** the demo plan lasts 50.5 s (about 1,010 steps); success is declared after the carrier has rested
  4 s on the mat, about 52 s in (about 1,040 steps), about 1.6 s after the last move, with the gripper open and clear; a Dream-1 trial runs up to 2,400 steps (2 minutes simulated).
  Wall-clock time on one L40S, including Isaac Sim start-up: about 260 s for the demo and about 400 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 4 s (carrier on the mat, bars moved, carrier still),
  otherwise `score()` at the end of the run.

## Scoring

At the end of the run, in a real trial after opening the grippers, with a tape for the handles and a top photo for
the carrier:

| Score | Condition |
|---|---|
| 1.0 | bars A, B and C moved 2.4, 1.8 and 0.8 cm or more, and the carrier wholly inside the mat, its bottom within 1 cm of the deck top, tilted at most 10 degrees (one bottom edge at most 8 mm higher than the opposite edge) |
| 0.8 | bars A, B, C moved 2.4, 1.8, 0.8 cm or more and the door moved 10 cm or more (carrier anywhere else, including still in the bay) |
| 0.6 | bars A, B, C moved 2.4, 1.8, 0.8 cm or more |
| 0.4 | bars A and B moved 2.4 and 1.8 cm or more |
| 0.2 | bar A moved 2.4 cm or more |
| 0 | anything else |

Bands 0.2 to 0.8 each need the ones before it; 1.0 does not need the door to stay open once the carrier is out. So a run that goes straight to the orange bar scores 0 (it is blocked, and the
score would not count it). The thresholds sit just under each bar's freeing travel, above the 5 mm a blocked bar can move, so the order stays enforced. The hardware already enforces this: the yellow bar cannot get past the red bar's tip
until it has cleared its path (2.5 cm of red travel from the setup coordinates), the orange bar cannot get past the yellow bar's end until it has cleared it (1.9 cm), and the
door cannot get past the orange bar's tip until it has cleared the door's lug (0.9 cm). A blocked bar can still move 5 mm.

Pushing never scores. The bay is roofed by the door and walled 6 cm high on all sides, so the carrier
cannot be pushed or dragged out before the door is open and can only leave by being lifted. A carrier whose bottom is
more than 1 cm above the deck counts as not placed, so one still held in the air scores no better than if it had been
let go, and a carrier let go outside the mat or still in the bay earns no more than the band the bars and door reached.

## Real-world reset

Stand the carrier back in the bay, centred at x 30.5, y 18.5, with the door still open. Then slide the door, bar C, bar B
and bar A back to their start stops, in that order, since each moved bar blocks the one before it.
