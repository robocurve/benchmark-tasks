# Let a washed part drain over the bath before moving it

A part soaking in a parts-washer bath goes to the drying rack, relayed through a pad in the middle. The rule, from
parts-washer practice: after lifting it out, hold it still above the bath for at least 3 seconds before moving it
away. Carrying it straight off still reaches the rack, but it breaks the rule and scores 0.

| File | What |
|---|---|
| `drain.py` | The task file: objects, scoring (`score`) and the scripted demo (`demo`) |
| `test_drain.py` | Offline checks: every scoring outcome, the demo's reach, and the demo replayed through the scorer |
| `task.json` | The submission's details |

## Run a trial

On the Brev instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Take the grey metal part out of the parts-washer bath and put it on the green drying rack. After lifting it out, hold it still above the bath for at least 3 seconds so it stops dripping before you move it away."
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=drain.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=drain.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval
```

Offline checks (no simulator): `python -m pytest test_drain.py -q`

## Runtime and seeds

- Run limit 2,400 control steps (2 minutes at 20 Hz); the scripted demo takes about 25 s.
- The scorer measures time by counting control steps, so runs must use `-E control_hz=20`.
- `-E object_jitter=0` fixes the layout, so no seed is needed.

## Scoring

| Score | Condition |
|---|---|
| 1.0 | No violation; the part rests on the drying rack |
| 0.5 | No violation; the part rests on the table or pad, away from the bath |
| 0.25 | No violation; the part drained but went no further |
| 0 | Otherwise: any violation, the part off the table, or nothing done |

Drained: held above the bath (centre over its inside, bottom above the rim), moving slower than 3 cm/s, for 3 s in
a row since it was last in the bath. Violation: the part leaves the area over the bath before it has drained;
dipping it back in means it must drain again.

In the real world: film the trial and check, from the frames, that the part hangs still over the bath for 3 s
before it moves off.
