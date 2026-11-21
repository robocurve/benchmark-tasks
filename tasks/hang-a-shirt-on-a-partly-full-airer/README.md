# Hang a baby T-shirt over the free front rail of a partly full airer

A small damp cotton T-shirt lies flat on the table in front of a three-rail wooden airer. Damp face cloths already
hang on the airer's middle and back rails. The robot hangs the shirt to dry over the free front rail: folded over
that rail alone, one half each side, held up only by its balance (no pegs), spread out, without dragging the cloths
already there. DESIGN.md has the real-world protocol, the scoring and the simulation model.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3 / Isaac Sim 6.1 (`source <ARENA_DIR>/yam.env`), inspect-robots 1.0.
- The modules in this folder. Importing `bodykit` hooks `yam_arena.scene` and the embodiment without modifying them.
  `clothkit` adds the fabric as PhysX surface-FEM cloth; `pinch` adds the contact-gated finger pinch.
- About 9 GB of GPU memory for one trial.

## Run a trial

From the repository root.

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/hang-a-shirt-on-a-partly-full-airer/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 --store-frames \
  --instruction "Hang the T-shirt to dry over the free front rail of the airer, folded over that rail alone with one half each side and spread out, then let go. Leave the two face cloths as they are." \
  --log-dir ~/yam-eval/airer-dream
```

The scripted demo plays the planned joint targets. It records three cameras, writes MP4s into `<out>/videos/` and
puts the final metrics in `<out>/summary.json`:

```bash
python tasks/hang-a-shirt-on-a-partly-full-airer/run_demo.py --out ~/yam-eval/airer-demo
```

The layout is fixed (`object_jitter=0`); there is no seed to set. The physics runs at 240 Hz and control at 20 Hz.
Set `SHIRT_PROBE=1` to print the score and its measurements once a simulated second.

**Runtime per trial** (one RTX 5090, the cloth on the GPU): the demo (about 700 steps, three 640 x 360 cameras) takes
about 8 minutes of wall time, and a 1,200-step Dream-1 run about 3 minutes (the model answered in about 2.4 s per request).

## Results

- Scripted demo: **1.0** (success at step 695, held 3 s; two runs of the same plan both scored 1.0).
- Dream-1: **0.0** after 1,200 steps. It moved both arms around the airer, brushing the blue face cloth (12 mm),
  but never grasped the T-shirt, which stayed flat on the table.

## Score

`task.py` evaluates `scoring.evaluate()` after every control step, and the run's score is its value at the end
(`SCORE_OVER = "end"`):

| Score | Condition |
|---|---|
| 0 | The shirt touches the table, is not held up by the airer, or is still in a hand; or a face cloth came off its rail or touches the table |
| 0.25 | The shirt hangs from the airer, clear of the table, released |
| 0.5 | And it rests on the free front rail only, and each half hangs at least 80 mm below that rail's top |
| 0.75 | And it lies along at least 187 mm of the rail |
| 1.0 | And no corner of either face cloth moved more than 30 mm, and both hands are open and at least 50 mm from the shirt |

A full score must last 3 s (`SUCCESS_HOLD_S`) before it ends the episode early. The offline tests of these rules
need no simulator:

```bash
python -m pytest tasks/hang-a-shirt-on-a-partly-full-airer/test_scoring.py
```

## Files

| File | Contents |
|---|---|
| task.py | The task file: scene objects, per-step scoring, `score()`, `success()`, `demo()` |
| layout.py | Dimensions; the T-shirt and washcloth meshes and their starting shapes |
| rack.py | The airer (one fixed body) and the three fabric sheets with their materials |
| clothkit.py | PhysX surface-FEM cloth for task files |
| pinch.py | The contact-gated pinch (see DESIGN.md) |
| scoring.py, test_scoring.py | Scoring rules and their offline tests |
| shirtplan.py, run_demo.py | The scripted demo and its recorder |
| bodykit.py, fastik.py | Multi-shape rigid bodies and hooks for task files; fast YAM kinematics |

All assets are authored procedurally in these files; no third-party meshes are used.
