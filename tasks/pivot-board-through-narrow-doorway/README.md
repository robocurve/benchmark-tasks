# Get a long board through a narrow doorway in a partition: turn it end-on, push it through and take it on the other side

A tall partition runs straight out from between the arms and splits the table into a left and a right side, with one
narrow doorway, 8 cm wide. A 30 cm board lies on the left side, parallel to the partition, and has to lie on a mat on
the right side, parallel to the partition too. The board only fits through the doorway end first, so it has to be
turned across the partition, pushed through, taken on the far side and turned back. Neither arm reaches past the
partition, and the partition is taller than either arm can lift the board, so the two arms meet at the doorway: one
feeds the board through and the other takes it. The whole task (scene, scoring and the scripted demo) is one task
file, [`pivot_board_through_narrow_doorway.py`](pivot_board_through_narrow_doorway.py), for the `yam_arena` embodiment.

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
  -E task=pivot_board_through_narrow_doorway.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Move the wooden board to the grey mat on the other side of the partition and lay it flat along the mat. It only fits through the narrow doorway end first." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/doorway-dream
```

The scripted demo, which reaches full score (the left gripper picks the board up near one end, turns it across the
partition and pushes it end first through the doorway; the right gripper takes the end that comes through, the left
lets go, and the right pulls the board through, takes it again by its middle, turns it back and lays it on the
mat):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pivot_board_through_narrow_doorway.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Move the wooden board to the grey mat on the other side of the partition and lay it flat along the mat. It only fits through the narrow doorway end first." \
  --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/doorway-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/doorway-dream/*.json; do inspect-robots video "$log"; done
```

`PV_DEBUG=1` prints where the board is, its heading and tilt, its extent across the table and both scoring
conditions, every 10 steps.

## Runtime per trial

On one L40S: about 5 minutes for a Dream-1 trial of 2,400 steps (269 s on the clock, with Dream answering in
0.74 s on average), and about 1.5 minutes for the scripted demo (637 steps, 31.9 s of simulated time; 95 s while
rendering 1280 x 720 video). Isaac Sim's first start on a new machine adds a few minutes while it compiles its
shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A partition of 12 mm plywood or foam board, 35 cm tall, in two 18 cm panels with an 8 cm gap, standing on the centre
line; a 30 x 4 x 1.5 cm pine strip; and a 34 x 8 cm grey mat taped down on the right.
