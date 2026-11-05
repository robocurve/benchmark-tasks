# Sort round-bottom flasks into their rings when every ring is taken

Three round-bottom flasks sit in three cork rings, each in the wrong ring; a coloured tag in front of each ring says
which flask belongs there. Every ring is taken and no flask may touch the bench, so a single gripper has nowhere to
put a flask down: one gripper has to hold a flask in the air while the other moves the rest round, then put its
flask in the ring that comes free. The whole task (scene, scoring and the scripted demo) is one task file,
[`sort_flasks_in_rings.py`](sort_flasks_in_rings.py), for the `yam_arena` embodiment.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task uses only boxes and balls, so it needs no
models from Arena's object library.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` holds for 2 s, otherwise `score()` at the end of the episode
(0.5, 0.25 or 0). The bench rule holds for the whole trial and is latched in the task file's module state: once any
flask's lowest point comes within 3 mm of the table, the trial scores 0. Run one trial per process (as below) so this
state starts fresh. `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial
starts from the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=sort_flasks_in_rings.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put each round-bottom flask in the cork ring tagged with its colour. No flask may touch the bench: keep every flask either in a ring or in a gripper the whole time." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/flasks-dream
```

The scripted demo, which reaches full score (the left gripper lifts the blue flask out of the left ring and holds
it; the right gripper moves the red flask into the left ring and the green flask into the middle ring; the left
gripper carries the blue flask round behind the row and lowers it into the right ring):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=sort_flasks_in_rings.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put each round-bottom flask in the cork ring tagged with its colour. No flask may touch the bench: keep every flask either in a ring or in a gripper the whole time." \
  --scorer task_score --max-steps 1100 --store-frames --log-dir ~/yam-eval/flasks-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/flasks-dream/*.json; do inspect-robots video "$log"; done
```

`SF_DEBUG=1` prints which ring each flask is in, their heights and the lowest point any flask has reached, every 10
steps.

## Runtime per trial

On one L40S: about 5.5 minutes for a Dream-1 trial of 2,400 steps (316 s on the clock, with Dream answering in
0.85 s on average), and under 1 minute for the scripted demo at 320 x 176 (926 steps, 46.3 s of simulated time;
2.5 minutes when rendering 1280 x 720 video). Isaac Sim's first start on a new machine adds a few minutes while it
compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. With a nonzero
`object_jitter`, the trial's seed sets how far the flasks shift (they settle back into their rings).

## Building it for real

Three 100 ml round-bottom flasks (about 6-8 cm across) in cork rings, or 3D-printed square rings with a 5 cm
opening, fixed to the bench in a row 11 cm apart, and three tape tags. Water tinted with food colouring marks the
flasks. Use plastic flasks, or pad the bench, for the robot trials.
