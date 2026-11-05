# Lift a ball too big to grip onto its stand, cradled on two battens held one in each gripper

A 15 cm ball lies on the table between two wooden battens, one in front of it and one behind it. It has to go into
the brass holder on top of a stand nearer the arms. The ball is much wider than a gripper opens and, being round,
rolls out of a squeeze between the two grippers; it cannot be rolled up onto the 6 cm stand either. The two battens,
one in each gripper, brought against the ball low down from front and back, cradle it on their top edges when they
rise together. Carried level and across the battens (along them it would roll off) and lowered on either side of the
stand, they leave the ball in the holder. The whole task (scene, scoring and the scripted demo) is one task file,
[`cradle_ball_onto_stand_on_two_battens.py`](cradle_ball_onto_stand_on_two_battens.py), for the `yam_arena` embodiment.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task uses only boxes and a sphere, so it needs no
models from Arena's object library.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` holds for 2 s, otherwise the best `score()` reached during the
episode (0.5 or 0; the task sets `SCORE_OVER = "best"`). `-E object_jitter=0` keeps every object exactly where the
task file puts it, so every trial starts from the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=cradle_ball_onto_stand_on_two_battens.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the blue ball in the brass holder on top of the stand. It is too big to grip: hold one batten in each gripper, bring the battens against the ball low down from front and back, and lift it on the two together." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cradle-dream
```

The scripted demo, which reaches full score (each gripper takes a batten by one end; the battens slide in against the
ball low down from front and back and rise together with the ball on their top edges; held at one end each, they sag
a little, so the ball rolls along them until it rests against the left gripper; the battens carry it back across
their length to the stand, lower on either side of it so the ball settles in the holder, and drop to the table):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=cradle_ball_onto_stand_on_two_battens.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the blue ball in the brass holder on top of the stand. It is too big to grip: hold one batten in each gripper, bring the battens against the ball low down from front and back, and lift it on the two together." \
  --scorer task_score --max-steps 470 --store-frames --log-dir ~/yam-eval/cradle-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/cradle-dream/*.json; do inspect-robots video "$log"; done
```

`BC_DEBUG=1` prints where the ball and the two battens are and both scoring conditions, every 10 steps.

## Runtime per trial

On one L40S: about 5 minutes for a Dream-1 trial of 2,400 steps (295 s on the clock, with Dream answering in
0.80 s on average), and about 1 minute for the scripted demo (450 steps, 22.5 s of simulated time; 75 s while
rendering 1280 x 720 video). Isaac Sim's first start on a new machine adds a few minutes while it compiles its
shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A 15 cm hollow plastic ball of about 150 g, two 40 cm lengths of 2 x 2 cm square pine batten, and a stand: a block
4 x 6 x 6 cm with a thin frame on top round a 2.4 x 4 cm slot (four strips of brass or wood 6 mm tall).
