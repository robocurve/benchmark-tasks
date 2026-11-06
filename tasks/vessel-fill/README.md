# Vessel fill and shut-off

A bimanual YAM task in Isaac Lab-Arena: carry a tub under a faucet's spout, hold it there while the
tap runs, shut the water off, and return the tub — the NSPIRE D161 (Sink) check end to end: run
water, then prove the valve closes, with the same two arms doing the carrying and the turning.

- **Task file:** `vessel_fill.py` (scene objects, `success()`, `score()`, scripted demo — one file)
- **Instruction:** "fill the tub under the spout, then shut the water off and return the tub"
- **Scene:** `station` (Dream's calibrated YAM station)

## Fixture

At the table's left-centre (x = 0.30 m, y = 0): a dark faucet body (5×17×7 cm) with a gooseneck
riser (3×3×14 cm) and a forward spout arm (8×2.4×2.5 cm) whose outlet hangs over x = 0.38 m, 10 cm
above the seated tub's rim. On the body's left face (y = +0.065 m) a steel stem (r = 1.4 cm, 3 cm)
carries a square blue knob (5×5×4 cm, 150 g) riding on four small posts. The vessel is a square
light-blue tub (5.2×5.2×6 cm, ~80 g) resting at (0.44 m, −0.18 m). No other loose objects.

## Run and score a trial

```bash
# scripted demo (reaches full score, ~45 s of wall-clock plan, 950-step cap)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=vessel_fill.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "fill the tub under the spout, then shut the water off and return the tub" \
  --scorer task_score --max-steps 950 --store-frames --log-dir ~/yam-eval/fill-demo

# Dream-1 run (2,400 steps = the task's run limit)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=vessel_fill.py -E scene=station \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "fill the tub under the spout, then shut the water off and return the tub" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/fill-dream
```

`task_score` = 0.5 × fill fraction + 0.2 × shut-off + 0.3 × clean return.

- **Fill:** the tub's centre within 4 cm of the seat (0.41 m, 0) while the square knob sits ≥ 25°
  off either closed orientation (0/180° — a square knob is 180-deg symmetric, so "closed" repeats
  every 180°), for ≥ 8.0 s cumulative at 20 Hz.
- **Shut-off:** the knob ends within 15° of a closed orientation.
- **Clean return:** the tub back within 4 cm of its start, upright within 15°, never tipped past
  45° (a tip past 45° is a spill and voids the return credit).

`success` ends the trial when all three hold together; `SCORE_OVER = "end"`.

## Runtime and seeds

- Scripted demo: ~45 s of simulated time (about 900 control steps at 20 Hz); a few minutes
  wall-clock including Isaac Sim warmup.
- Dream-1: up to 2,400 control steps (2 minutes at 20 Hz).
- No seed dependence: `-E object_jitter=0` pins every object to the exact layout in `setup.png`;
  the scripted demo is deterministic waypoints.
- Water is not simulated: the open-tap dwell (the vessel held under the spout while the valve is
  off its seat) stands in for the delivered volume, and the shut-off is the knob's return to a
  closed orientation. The knob's yaw transfer from the wrist is friction-dependent and slips
  between runs — the model must re-grasp and re-turn rather than rely on one blind sweep. See
  `fidelity` in `task.json`.
