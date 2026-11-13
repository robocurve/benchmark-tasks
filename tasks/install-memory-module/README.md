# Install a memory module

A DDR5 desktop memory module stands in a holder beside a motherboard, a quarter turn from how it fits. The robot has
to pick it up, turn it the right way for its key notch to meet the key in slot A2, set it in the slot and press it
down until both latches snap shut.

The difficulty is the real one. The key only fits one way round, so the module has to be turned the correct way. The
slot's lead-ins forgive about 2 mm and 1 degree of misalignment. Seating needs a firm, square push of about 16 N:
a module pressed at one end, or tipped, jams or seats only that end.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, `success()`, `score()`, `self_test()` and the scripted `demo()` |
| `latches.py` | The latches' over-centre spring, applied before every physics step |
| `scenekit.py` | USD bodies and articulations for yam_arena task files (hooks yam_arena without changing it) |
| `assets/board.usda` | The ATX board: static, slot A2's two latches a fixed-base articulation |
| `assets/dimm_ddr5.usda` | The DDR5 module, 40 g, compliant contact edge |

The models' visuals come from pc-anatomy (MIT, github.com/Yoosseph/pc-anatomy), without printed text. Collision
shapes, the slot's contacts and lead-ins, and the latches are built to its dimensions.

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Install the memory module in slot A2 and press it down until both latches click shut." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/dimm-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

Check the scoring rules offline (no GPU): `python -c "import task; task.self_test()"`.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Install the memory module in slot A2 and press it down until both latches click shut." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/dimm-demo
```

The right arm picks the module by its heat spreader, turns it a quarter turn to fit the key, sets it in A2's
lead-in and pushes it home while still gripping it, so it stays upright. It lets go and presses each end with
closed fingers. The left arm stays still. At 20 Hz it reaches 1.0 (success) in 445 to 585 steps (22 to 29 s,
depending on whether the end presses are needed); it also succeeds at 10 Hz. The demo is open loop with
`-E object_jitter=0`.

## Scoring

The score at the end of the episode counts (`SCORE_OVER = "end"`).

- **0** if a gripper still holds the module (a gripper centre within 4.5 cm of the module's middle), or the module
  is not standing in A2 the right way round. Standing in A2 the right way round means: its centre within 1.5 mm of
  the slot's centre line and centre, within 5 degrees of the orientation that fits the key, upright within 5
  degrees, and its contact edge below the slot's top. A reversed module stops on the key above that, so it scores 0,
  as does a module in another slot or off the table.
- **0.25** standing in A2 the right way round, neither latch shut.
- **0.5** one or both latches shut (within 5 degrees of closed), but not the full condition below.
- **1.0 (success, held for 1 s):** both latches shut, both ends of the contact edge within 0.3 mm of the slot floor,
  released, and at rest (moved less than 1 mm over the last second).

Possible scores: 0, 0.25, 0.5, 1.0.

Real check, after the arms park: a side photo shows whether both latch arms stand upright with their hooks in the
module's notches; a feeler gauge between the module's edge and the slot's top tells a seated module from a raised one.

Simulation details:
- The slot's contact strips pinch the module's edge (compliant contact). The strips stand 0.015 mm clear of the
  module at the slot top and press harder lower down.
- 45-degree lead-ins over the strips, over each end's guide groove and beyond each end stop centre a module set down
  within about 2 mm and 1 degree. The module's contact edge has a 0.3 mm bevel.
- Each latch has a foot under the module's end and an over-centre spring. Within 6 degrees of shut it snaps shut
  and holds about 4-5 N at its top; otherwise it is pulled gently open. Pressing its T-shaped tab tips it open, and
  the foot lifts that end of the module.
- Seating takes about 16 N pushed square, less than a real DDR5 socket (up to about 107 N), within the arms' reach.

## Runtime and seeds

A scripted-demo run takes about 75 seconds on an RTX GPU (RTX 5000 Ada, A40), including Isaac Sim's start once its
shaders are cached; the first start on a fresh machine takes several minutes. A 2,400-step Dream-1 trial took about
6 minutes, the model's round trips included.

The layout is fixed (`-E object_jitter=0`), so there are no seeds to choose. Dream-1 is a hosted model without a
seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_e09b39b1.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps
  (inspect-robots 1.0.0, inspect-robots-dream 0.1.0, dreamlabs-client 1.1.3).
- Demo: `task_score = 1.0`.
