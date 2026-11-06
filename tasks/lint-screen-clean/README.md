# Dryer lint screen clean-and-reseat

A bimanual YAM task in Isaac Lab-Arena: pull the dryer's lint screen out of its slot, clear the lint
off it, and reseat it fully. NSPIRE D021/D023: a lint screen blocked with visible lint accumulation
is a clothes-dryer fire hazard, and the check — pull, look, clean, reseat — is fully photo-verifiable.

- **Task file:** `lint_screen_clean.py` (scene objects, `success()`, `score()`, scripted demo — one file)
- **Instruction:** "clean the lint screen and reseat it fully"
- **Scene:** `station` (Dream's calibrated YAM station)

## Fixture

On the table at (x = 0.40 m, y = 0): the top section of a clothes dryer — a light-grey cabinet box
(30×26×10 cm) — carries a slot along its front edge: two steel walls (8 mm thick, 26 cm long, 2 cm
tall) 4.4 cm apart. In the slot lies the lint screen: a white tray 2.4 cm across, 22 cm long and
2 cm thick (~120 g) with a long handle bar; seated, its handle end protrudes 7 cm past the slot
mouth (y = +0.20 vs the mouth at y = +0.13). Six grey lint flakes (8 mm cubes, ~2 g) are settled on
the screen's face inside the slot (y = 0.07..0.11). No other loose objects.

## Run and score a trial

```bash
# scripted demo (reaches full score, ~11 s of wall-clock plan, 700-step cap)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=lint_screen_clean.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "clean the lint screen and reseat it fully" --scorer task_score \
  --max-steps 700 --store-frames --log-dir ~/yam-eval/lint-demo

# Dream-1 run (2,400 steps = the task's run limit)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=lint_screen_clean.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "clean the lint screen and reseat it fully" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/lint-dream
```

`task_score`: 0.5 for the screen fully reseated (its centre within 6 mm of the seated pose) and 0.5
for the lint cleared (no flake within the screen's footprint + 4 mm at the screen's height — flakes
knocked off onto the cabinet or the table count as cleared); 1.0 for both. `success` ends the trial
when both hold for 0.4 s; the final state is static, so `SCORE_OVER = "end"`.

## Runtime and seeds

- Scripted demo: ~11 s of simulated time (about 220 control steps at 20 Hz); a few minutes
  wall-clock including Isaac Sim warmup.
- Dream-1: up to 2,400 control steps (2 minutes at 20 Hz).
- No seed dependence: `-E object_jitter=0` pins every object to the exact layout in `setup.png`;
  the scripted demo is deterministic waypoints.
- The screen is a rigid tray in an open slot (no mesh); the lint is six rigid flakes — "cleaning"
  means no flake remains on the screen's face, the same thing a photo inspection reads. See
  `fidelity` in `task.json`.
