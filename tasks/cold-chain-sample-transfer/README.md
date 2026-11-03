# Move a frozen sample between cold boxes within 20 seconds

A frozen sample vial goes from the left cold box to the right one, relayed through a pad in the middle. The rule,
from cold-chain handling: at most 20 seconds in total outside the boxes. A hesitant relay still reaches the goal but
scores less, and a slow one scores 0.

| File | What |
|---|---|
| `coldchain.py` | The task file: objects, scoring (`score`) and the scripted demo (`demo`) |
| `test_coldchain.py` | Offline checks: every scoring outcome, the demo's reach, and the demo replayed through the scorer |
| `task.json` | The submission's details |

## Run a trial

On the Brev instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Move the magenta sample vial from the left cold box into the right cold box. The vial must spend no more than 20 seconds in total outside the cold boxes."
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=coldchain.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=coldchain.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval
```

Offline checks (no simulator): `python -m pytest test_coldchain.py -q`

## Runtime and seeds

- Run limit 2,400 control steps (2 minutes at 20 Hz); the scripted demo takes about 20 s, about 12 s of it outside.
- The scorer measures time by counting control steps, so runs must use `-E control_hz=20`.
- `-E object_jitter=0` fixes the layout, so no seed is needed.

## Scoring

| Score | Vial resting in the right box at the end, and its total time outside the boxes is |
|---|---|
| 1.0 | at most 20 s |
| 0.5 | at most 30 s |
| 0.25 | at most 45 s |
| 0 | more than 45 s, or the vial is not in the right box (or is off the table) |

The vial is outside whenever it is not down in a box: centre over the box's inside and bottom below its rim, so
holding it above a box counts as outside.

In the real world: film the trial and count the frames in which the vial is out of both boxes.
