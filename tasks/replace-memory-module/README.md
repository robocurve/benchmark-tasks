# Replace a memory module

An old DDR5 memory module is seated in slot A2 of a motherboard, both latches shut on it. A new module stands in a
holder on the right, a quarter turn from how it fits; an empty holder waits on the left. The robot has to press both
latches open, take the old module out and stand it in the left holder, then turn the new one the right way for the
key, press it into A2 and make both latches click shut.

Both arms have work to do: one can take the old module out while the other fetches the new one.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, `success()`, `score()`, `self_test()` and the scripted `demo()` |
| `latches.py` | The latches' over-centre spring, applied before every physics step |
| `scenekit.py` | USD bodies and articulations for yam_arena task files (hooks yam_arena without changing it) |
| `assets/board.usda` | The ATX board: static, slot A2's two latches a fixed-base articulation |
| `assets/dimm_ddr5.usda` | The DDR5 module (used for both), 40 g, compliant contact edge |

The models' visuals come from pc-anatomy (MIT, github.com/Yoosseph/pc-anatomy), without printed text. Collision
shapes, the slot's contacts and lead-ins, and the latches are built to its dimensions.

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Replace the memory module in slot A2 with the new one from the holder on the right, and stand the old one in the empty holder on the left." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/replace-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

Check the scoring rules offline (no GPU): `python -c "import task; task.self_test()"`.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Replace the memory module in slot A2 with the new one from the holder on the right, and stand the old one in the empty holder on the left." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/replace-demo
```

The right arm presses the near latch's tab open and goes to pick up the new module. The left arm then presses the far
latch's tab open, pulls the old module straight up and stands it in the left holder. Once the left arm is clear, the
right arm turns the new module to fit the key, pushes it into A2 while still gripping it, and lets go. The arms take
turns at the slot: reaching the far latch, the left arm's forearm crosses over the near one. At 20 Hz it reaches 1.0
(success) in about 900 steps (45 s). The demo is open loop with `-E object_jitter=0`.

## Scoring

The score at the end of the episode counts (`SCORE_OVER = "end"`).

- **Torn out:** the old module left A2 before both of its latches had been pressed open. A latch counts as opened
  once it is 20 degrees open while the module's end over it is still within 2.5 mm of the slot floor (pressing a tab
  lifts that end 1.8 mm; tearing a module out lifts it before the latch gives).
- **In the left holder:** standing in its groove, upright within 20 degrees (it may lean on a wall), its length along
  the groove, released.
- **New module in A2:** its centre within 1.5 mm of the slot's centre line and centre, within 5 degrees of the
  orientation that fits the key, upright within 5 degrees, its contact edge below the slot's top.

| Outcome | Score |
|---|---|
| The old module was torn out, a module fell off the table, or the old module is still in A2 | 0 |
| The old module is out of A2 | 0.25 |
| ...and stands in the left holder | 0.5 |
| ...and the new module stands in A2 the right way round | 0.75 |
| ...and the new module is fully seated (both ends within 0.3 mm of the slot floor) with both latches shut, both modules released and at rest, for 1 s (success) | 1.0 |

Real check: a video of the removal shows whether both latches were opened before the module came out; a side photo
and a feeler gauge show the new module seated and latched; a look at the left holder shows the old one.

## Runtime and seeds

A scripted-demo run takes about 2.5 minutes on an RTX GPU (RTX 5000 Ada, A40), including Isaac Sim's start once its
shaders are cached; the first start on a fresh machine takes several minutes. A 2,400-step Dream-1 trial took about
13 minutes, the model's round trips included.

The layout is fixed (`-E object_jitter=0`), so there are no seeds to choose. Dream-1 is a hosted model without a
seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_c8102ba3.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps
  (inspect-robots 1.0.0, inspect-robots-dream 0.1.0, dreamlabs-client 1.1.3).
- Demo: `task_score = 1.0`.
