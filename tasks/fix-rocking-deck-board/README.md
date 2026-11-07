# Fix a rocking deck board: pop it out, take out the pebble under it, and lay it back flat

A section of decking: three boards side by side in a frame. The middle board rocks because a pebble is stuck under it
near its left end. It has to be fixed: the board taken out, the pebble put in the bucket and the board laid back flat
in its slot. The board fits its slot with only 7 mm to spare on each side and the frame closes both ends, so there is
nothing to take hold of. Pressed down at its high left end, it pivots on the pebble like a seesaw and its right end
rises clear of the deck, but only while the left end is held down: one gripper presses while the other takes the
risen end. The whole task (scene, scoring and the scripted demo) is one task file,
[`fix_rocking_deck_board.py`](fix_rocking_deck_board.py), for the `yam_arena` embodiment.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task uses only boxes, so it needs no models from
Arena's object library.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` holds for 2 s, otherwise `score()` at the end of the episode
(0.5 or 0). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from
the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=fix_rocking_deck_board.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The middle deck board rocks because a pebble is stuck under it. Take the board out, put the pebble in the black bucket, and lay the board back flat in its slot." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/deck-dream
```

The scripted demo, which reaches full score (the left gripper presses the high left end down; the board pivots on the
pebble and its right end rises; the right gripper, turned a quarter turn, takes the risen end, lifts the board out and
holds it aside; the left gripper picks the pebble out of the slot and drops it in the bucket; the right gripper lays
the board back, low end first):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=fix_rocking_deck_board.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The middle deck board rocks because a pebble is stuck under it. Take the board out, put the pebble in the black bucket, and lay the board back flat in its slot." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/deck-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/deck-dream/*.json; do inspect-robots video "$log"; done
```

`RB_DEBUG=1` prints where the board and the pebble are, the board's tilt and both success conditions, every 10 steps.

## Runtime per trial

On one L40S: about 5 minutes for a Dream-1 trial of 2,400 steps (300 s on the clock, with Dream answering in
0.76 s on average), and under 1.5 minutes for the scripted demo (508 steps, 25.4 s of simulated time; 84 s while
rendering 1280 x 720 video). Isaac Sim's first start on a new machine adds a few minutes while it compiles its
shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

Three 7 x 25 cm offcuts of 25 mm decking in a frame of battens, the outer two screwed down; a flat pebble about 8 mm
thick under the loose middle board; any small box as the bucket.
