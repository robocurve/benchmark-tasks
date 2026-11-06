# Call-for-aid cord pull

A bimanual YAM task in Isaac Lab-Arena: pull a call-for-aid cord down 15 cm — the NSPIRE D009
deficiency ("pull cord is higher than 6 inches off the floor") is life-threatening because a fallen
resident cannot reach a cord whose end sits too high; pulling the end down within 15 cm of the
surface is the fix, and is verifiable with a tape measure.

- **Task file:** `call_cord_pull.py` (scene objects, `success()`, `score()`, scripted demo — one file)
- **Instruction:** "pull the call cord down 15 centimetres to call for aid"
- **Scene:** `station` (Dream's calibrated YAM station)

## Fixture

At the table's left side (x = 0.38 m, y = 0.10 m): a grey pedestal mount (6×6×15 cm) carries a steel
pivot ridge (6×1.2×1 cm) on top. Across the ridge rests a red pull rod (1×40×1.2 cm, ~80 g),
balanced level like a see-saw: its grip end reaches out over the table (y = +0.30 m), its tail end
hangs behind the bracket (y = −0.10 m). No other loose objects.

## Run and score a trial

```bash
# scripted demo (reaches full score, ~9 s of wall-clock plan, 700-step cap)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=call_cord_pull.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "pull the call cord down 15 centimetres to call for aid" --scorer task_score \
  --max-steps 700 --store-frames --log-dir ~/yam-eval/cord-demo

# Dream-1 run (2,400 steps = the task's run limit)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=call_cord_pull.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "pull the call cord down 15 centimetres to call for aid" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/cord-dream
```

`task_score` reads the rod's tilt about its pivot: the grip end's drop is 20 cm × sin(tilt), so 15 cm
of drop = 48.6° of swing. 0.5 at 7.5 cm of drop, 1.0 at 15 cm or more. The rod must still be riding
its pivot (centre within 1.5 cm of the ridge, not fallen to the table): a knocked-off rod is not a
call. `success` ends the trial once the full pull holds 0.4 s. `SCORE_OVER = "best"` keeps a pull
that later swings back.

## Runtime and seeds

- Scripted demo: ~9 s of simulated time (about 180 control steps at 20 Hz); a few minutes wall-clock
  including Isaac Sim warmup.
- Dream-1: up to 2,400 control steps (2 minutes at 20 Hz).
- No seed dependence: `-E object_jitter=0` pins every object to the exact layout in `setup.png`; the
  scripted demo is deterministic waypoints.
- The hanging cord is modeled as a light rigid rod balanced on the bracket's pivot ridge — a free
  object cannot hang from a fixed structure in this simulator. The externally visible evidence is
  the same a photo inspection reads: the pull end starts level with the bracket and ends 15 cm lower.
  See `fidelity` in `task.json`.
