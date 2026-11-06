# GFCI outlet test-and-reset

A bimanual YAM task in Isaac Lab-Arena: verify a ground-fault outlet by its sequence — press the
TEST end down (the trip), then press the RESET end down to restore it. Pressing RESET without a
test first proves nothing.

- **Task file:** `gfci_test_reset.py` (scene objects, `success()`, `score()`, scripted demo — one file)
- **Instruction:** "test the GFCI outlet: press TEST, then press RESET"
- **Scene:** `station` (Dream's calibrated YAM station)

## Fixture

Between the arms at x = 0.30 m: a grey pedestal (16×10×12 cm) carries a light faceplate
(16×14×1 cm) whose top holds a dark rocker (5.2 cm across × 9 cm long × 1.4 cm thick, ~80 g)
resting level on a central fulcrum ridge (6×1.2×1 cm). Two low walls (8 mm wide, 10 cm long) stand
just outside the rocker's ends so it cannot slide sideways off the ridge. The TEST end faces the
left arm (+y), the RESET end faces the right arm (−y); the ends are otherwise identical.

## Run and score a trial

```bash
# scripted demo (reaches full score, ~11 s of wall-clock plan, 700-step cap)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=gfci_test_reset.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "test the GFCI outlet: press TEST, then press RESET" --scorer task_score \
  --max-steps 700 --store-frames --log-dir ~/yam-eval/gfci-demo

# Dream-1 run (2,400 steps = the task's run limit)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=gfci_test_reset.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "test the GFCI outlet: press TEST, then press RESET" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/gfci-dream
```

`task_score` reads the rocker's tilt: pressing an end ~3 mm past level "presses" it. 0.5 once the
TEST end has been pressed down (the trip); 1.0 when the RESET end is then pressed down and held
0.4 s — the full ordered test-and-reset sequence. The state machine re-arms after a completed
sequence returns to level, so the score is order-sensitive: RESET-then-TEST never scores.
`success` ends the trial when the sequence completes.

## Runtime and seeds

- Scripted demo: ~11 s of simulated time (about 220 control steps at 20 Hz); a few minutes
  wall-clock including Isaac Sim warmup.
- Dream-1: up to 2,400 control steps (2 minutes at 20 Hz).
- No seed dependence: `-E object_jitter=0` pins every object to the exact layout in `setup.png`;
  the scripted demo is deterministic waypoints.
- No electricity is simulated; the trip/reset is a tilt-state machine read from the rocker's
  orientation — the same thing a photo of a tripped GFCI reads. See `fidelity` in `task.json`.
