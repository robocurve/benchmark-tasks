# Set out IKEA candles at IKEA's safety distances, clear of dried flowers and a shelf

Candles cause thousands of home fires a year, mostly from being too close to each other or to something that burns. IKEA's candle-safety guide gives the minimum distance between burning candles by type (7 cm for block candles, 3 cm for tealights), at least 30 cm from anything flammable, and 50 cm of open space above a lit candle.

| File | What |
|---|---|
| `candles.py` | The task file: objects, joints, scoring (`score`) and the scripted demo (`demo`) |
| `mech.py` | Mechanisms the task file uses (joints, slides, fixed parts as one rigid body, fingertip positions) |
| `test_candles.py` | Offline checks of every scoring outcome |
| `task.json` | The submission's details |

`mech.py` stays inside this folder: the task file imports it and it hooks the `yam_arena` scene builder and step
when the simulator starts. No change to `yam_arena` itself is needed.

## Run a trial

With the simulator set up by the simulation walkthrough, from this folder:

```bash
INSTR="Set out these three IKEA candles (one white block candle and two tealights) on the grey stone board, ready to light, following IKEA's candle safety rules: the gap between any two candles must be at least the larger of their safety distances (7 cm for the block candle, 3 cm for the tealights), every candle must be at least 30 cm from the dried flowers, and none may stand under the shelf, because a lit candle needs 50 cm of open space above it. Keep the candles upright."
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=candles.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=candles.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval
```

Offline checks (no simulator): `python test_candles.py`

## Runtime and seeds

- Run limit 1,200 control steps (1 minute at 20 Hz); the scripted demo takes about 34 s.
- Runs must use `-E control_hz=20`: the episode clock runs on control steps.
- `-E object_jitter=0` fixes the layout, so no seed is needed.

## Scoring

A candle counts when it stands upright (within 10 degrees) with its whole base on the board. For the candles on the board: the gap between two candles' edges must be at least the larger of their distances (7 cm block, 3 cm tealight); each candle's edge at least 30 cm from the vase's edge; no part of a candle's base under the shelf. If any rule is broken the score is 0; otherwise it is the number of candles on the board divided by 3 (doing nothing or refusing: 0).
