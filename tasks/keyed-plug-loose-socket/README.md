# Plug a keyed connector into a loose socket block

One arm holds an unclamped block steady while the other arm carries a keyed plug to the block's socket, turns it to
match the key and pushes it in against a spring latch. The score depends on how far the plug is seated, on the latch
clicking, on both hands letting go, and on the block staying put. The scripted demo scores 1.0 in 376 steps; the same
demo with the left arm idle scores 0.25 (the plug ends tilted and not seated).

## Files

| File | What it is |
|---|---|
| `task.json` | The submission's details (instruction, setup, scoring, run limit, fidelity, scores, media ID) |
| `task.py` | The yam_arena task file: the two parts, `success()`, `score()` and the demo |
| `geometry.py` | Every dimension, in one place |
| `parts.py` | Builds one rigid body from many boxes, with a spring-loaded slider (the task format only has single primitives) |
| `scoring.py` | The scorer, as plain functions of the part poses |
| `demo_plan.py` | The scripted demo's waypoints |
| `check_reach.py`, `check_collisions.py` | Offline checks that both arms reach the demo's waypoints and keep clear of the block and each other (no simulator needed) |
| `run.sh` | One command per trial: `demo`, `nobrace`, `settle`, `dream` |
| `tests/` | Offline tests for the scorer, the geometry, the helper and `task.json` |

## Requirements

The Python 3.12 / Isaac Lab-Arena 0.3.1 environment from the Brev launchable in the simulation walkthrough, with the
`yam_arena` embodiment, `inspect-robots` and the Dream-1 client. Nothing else to install: `parts.py` hooks `yam_arena`
without modifying it, so the task runs with the standard embodiment and the `task_score` scorer.

## Run

From this folder, after `source <ARENA_DIR>/yam.env`:

```bash
./run.sh demo       # scripted demo, both arms: expected 1.0
./run.sh nobrace    # the same with the left arm idle: scores 0.25, the plug ends tilted and not seated
./run.sh dream      # Dream-1 (key in ~/.dreamlabs/credentials)
```

Each writes the run log, camera frames and `trace.jsonl` (what the scorer saw at every step) to `~/yam-eval/<mode>`
(`OUT=<folder> ./run.sh demo` to change it). The underlying command is:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Plug the connector into the socket on the loose block, then let go of both. Do not let the block slide." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/demo
```

Videos: `inspect-robots video <log>.json` writes one MP4 per camera next to the frames.

## Scoring

See `scoring` in `task.json`; the code is `scoring.py`. The scorer reads the plug and block poses, the plunger and
whether both grippers are open; none of these reach the policy. Offline checks, with no simulator:

```bash
python -m unittest discover -s tests -v
python check_collisions.py --yam-arena <a checkout of yam-isaac-lab-arena>
```

## Layout, seeds and reset

The layout is fixed (`-E object_jitter=0`), so there are no seeds. `reset_notes` in `task.json` says how to reset a
real trial. Tuning the demo needs no code edits: the knobs are environment variables listed at the top of
`demo_plan.py`. The fingers are commanded only 5 mm past touching the part: commanded fully closed, both finger drives
saturate (50 N each) and the clamp slides sideways until a finger hits its limit.

## Runtime per trial

About 1.5 minutes per trial on an L40S once Isaac Sim is running (the scripted demo is 376 steps, 19 s of simulated
time; the first start of Isaac Sim on a new instance takes several minutes more).

## Real-world build

The block is 3D-printed PLA infilled to 0.30 kg, resting on a smooth sheet. The socket and plug are printed to the dimensions in
`geometry.py` (1.0 mm of play per side around the nose); the retention latch is an off-the-shelf spring plunger set
into the cavity wall, with a matching groove on the nose.

## Credits

`parts.py` follows the technique first shown in this repository's `tasks/bearing-press/partkit.py` (an import hook
on `yam_arena.scene` and the embodiment that adds compound bodies and spring sliders without modifying `yam_arena`).
It is a separate implementation.
