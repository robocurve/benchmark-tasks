# Lower a gas-spring hospital overbed table to a marked height

Instruction given to the policy: `Lower the overbed table to the marked height. Its single column is a locking gas spring, released by the paddle under the front edge of the top.`

The column is a locking gas spring. It moves only while the release paddle under the top's front edge is held
up, and then it pushes the top up. Lowering the top takes both hands: one holds the paddle up and follows it
down, and the other presses the top down near its middle. The paddle is let go first, which locks the column, and
the top second. The instruction names the mechanism but not this two-handed sequence. The score latches to 0 in three cases: the top springs
up 40 mm, the top tilts more than 4 deg, or it is pushed onto the patient's shins with more than 30 N.
Height is read from the column's travel, not a corner of the top, and the top must end within 1 deg of level.

Files: `task.json`, `overbed_table_lower.py` (the scene, `success()`, the partial-credit `score()`, the scripted
`demo()` and an offline self-test) and `mechkit.py` (joints, sliders and sprung drives added to the stock runtime
in memory).

## Environment

This runs on the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and
inspect-robots as it installs them. Run `source ~/IsaacLab-Arena/yam.env` before every shell. The Dream key is in
`~/.dreamlabs/credentials`.

## Offline check, no GPU

```bash
python tasks/overbed-table-lower/overbed_table_lower.py
```

```
scoring: 15 cases pass
left: 18.5 s plan, worst IK 0.091 mm / 0.013 deg
right: 20.5 s plan, worst IK 0.087 mm / 0.012 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/overbed-table-lower/overbed_table_lower.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lower the overbed table to the marked height. Its single column is a locking gas spring, released by the paddle under the front edge of the top." \
  --scorer task_score --max-steps 900 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/overbed-table-lower-dream
```

`task_score` is 1.0 when `success()` has held for 1 s. Otherwise it is `score()` at the last step.

## Run and score the full-score demo

Run the same command with `--policy yam_scripted` and `--log-dir ~/yam-eval/overbed-table-lower-demo`. The demo
is open loop and deterministic. Its plan is about 20 s, and it reaches 1.0 at about step 370. The left arm grips
the paddle's tip and lifts it while the right arm rests its closed fingers on the top near the middle. Both arms
then go down 4 cm together. The left arm lets the paddle go, which locks the column, and backs away. Then the
right arm lifts off. Two failure variants of the demo can be run by adding an environment variable:
`OVERBED_DEMO=press_only` presses without the paddle (score 0) and `OVERBED_DEMO=lever_only` lifts the paddle
without pressing, so the top springs up (score 0). `OVERBED_DEMO=press_far_end` leans one hand on the
far end with the column locked (score 0), and `OVERBED_DEMO=release_top_first` lets the top go while the paddle
is still held, so it springs up (surge latch, score 0). `OVERBED_DEBUG=1` prints the column state every 5 steps.

## Videos and screenshot

```bash
inspect-robots video ~/yam-eval/overbed-table-lower-demo/*.json
ffmpeg -y -i ~/yam-eval/overbed-table-lower-demo/frames/<run>/*top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 900 steps (45 seconds at 20 Hz).
- There is one fixed layout (`-E object_jitter=0`), so the seed does not change the start. The demo is
  deterministic. Dream-1 is hosted and not seeded.
- About 4 minutes of wall clock per trial on an L40S, including Isaac Sim warm-up.

## Reset between trials

Lift the paddle and let the top rise back to its starting height, with the surface 14 cm up and 4 cm above the
red tab. Then let the paddle go so the column locks there.
