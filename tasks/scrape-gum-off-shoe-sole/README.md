# Scrape chewing gum off a shoe sole

A sneaker lies sole-up with a flat wad of chewing gum stuck to the ball of its sole. One arm braces the shoe, the
other slides a plastic putty knife under the gum and pushes it off.

## Files

- `scrape_gum_off_shoe_sole.py`: the task file (objects, layout, scorer, scripted demo). Gum settings are in `GUM`.
- `gum.py`: when a gum strip may come free (the scrape rule) and the joints that hold the wad in Isaac Sim.
- `labkit.py`: helper that loads the two models and runs `gum.reset()` after every reset.
- `assets/sneaker-sole-up`, `assets/putty-knife`: the models, with their physics in `asset.json`.
- `test_scrape_gum_off_shoe_sole.py`: scoring and scrape-rule tests on made-up end states (no simulator needed).

Third-party assets: none. The shoe and knife models are procedural, made for this task, CC0.

## Run a trial

From the simulator's folder, with its environment active. Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/scrape-gum-off-shoe-sole/scrape_gum_off_shoe_sole.py \
  -E object_jitter=0 -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Scrape the chewing gum off the sole of the shoe with the putty knife." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir evaluations/dream
```

Scripted demo (open loop, so `-E object_jitter=0` is needed):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/scrape-gum-off-shoe-sole/scrape_gum_off_shoe_sole.py \
  -E object_jitter=0 -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Scrape the chewing gum off the sole of the shoe with the putty knife." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir evaluations/demo
```

The run prints `task_score` at the end. Scoring: `score()` and `success()` in the task file, and `scoring` in
task.json.

Physics runs on the CPU (`-E device=cpu` in every command); CPU and GPU PhysX can give different results.

## Runtime and seeds

- Demo: full score at step 748 (37 s simulated; the script itself is 42 s); about 2.5 minutes of wall time on an L4 with CPU physics.
- Dream-1: up to 2,400 steps (2 minutes simulated); about 5 minutes of wall time on the same machine.
- `object_jitter=0` keeps the one layout fixed, so no seed is needed.

## Checks without the simulator

```bash
python -m pytest -q tasks/scrape-gum-off-shoe-sole/test_scrape_gum_off_shoe_sole.py
```

They need `yam_arena` importable but not Isaac Sim.

## Status

Run in Isaac Sim on CPU physics: the scripted demo scores 1.0; the same stroke with the left arm left out slides the
shoe and scores 0. The spring strength and release distance of the gum are first values, not calibrated to real gum.
