# Raise a two-piece arch: each half falls over on its own, so set both down together and let them lean on each other

Two wooden arch halves stand at the front of the table. Each is an L: an upright post with an arm along its top that
overhangs towards the middle, so its centre of mass lies 1.5 cm past the inner edge of the post's foot and a half on
its own falls over towards its arm. At the start each is propped up by a dark block under the end of its arm. The
task is to stand the two on two blue marks further back, where the ends of their arms meet on the centre line and
the two lean on each other and stand as an arch. Whichever half is let go of first falls unless the other is
already there, so the arms carry one half each, set both down together and let go at the same moment. The task
(scene, scoring and the scripted demo) is the task file [`raise_two_piece_arch.py`](raise_two_piece_arch.py) for the
`yam_arena` embodiment, with [`compound_body.py`](compound_body.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`compound_body.py` adds what yam_arena's task files lack: one rigid body made of several boxes (each half is a post
box and an arm box). The task file imports it from its own folder; it patches `yam_arena.scene` when that module is
imported, so that such a body spawns as one rigid body of box colliders and its state is reported like any other
object's. It also reports each gripper's TCP, computed from the arm's joint positions with yam_arena's own URDF
kinematics, which the scoring uses to check that the grippers have let go. Nothing else in yam_arena is changed.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` holds for 2 s, otherwise the best `score()` reached during the
episode (0.5, 0.25 or 0). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every
trial starts from the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=raise_two_piece_arch.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stand the two wooden arch halves on the two blue marks, facing each other, so that the ends of their arms meet and the arch stands on its own. Each half falls over by itself, so set both down together." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/arch-dream
```

The scripted demo, which reaches full score (each gripper takes one half by the top of its post, lifts it off the
table and its block, carries it back to the marks and in towards the middle until the ends of the arms meet; both
set their halves down on the marks, hold them a second, let go at the same moment and rise clear):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=raise_two_piece_arch.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stand the two wooden arch halves on the two blue marks, facing each other, so that the ends of their arms meet and the arch stands on its own. Each half falls over by itself, so set both down together." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/arch-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/arch-dream/*.json; do inspect-robots video "$log"; done
```

`ARCH_DEBUG=1` prints, every 10 steps, where each half's post foot is, its tilt, the gap between the ends of the
arms, both grippers' TCPs and each scoring condition. `COMPOUND_DEBUG=1` prints each compound body's pose once.

## Runtime per trial

On one L40S: about 5 minutes for a Dream-1 trial of 2,400 steps (292 s on the clock, with Dream answering in
0.83 s on average), and about 1 minute for the scripted demo (421 steps, 21.1 s of simulated time; 69 s while
rendering 1280 x 720 video). Isaac Sim's first start on a new machine adds a few minutes while it compiles its
shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

Each half: a 3 x 5 x 15 cm pine post with an 11 x 5 x 3 cm pine arm glued to the top of one of its 5 cm faces,
flush with the post's top (about 180 g). Two blocks 5 x 2 x 12 cm to prop them at the start, and two 6 x 5 cm squares of blue tape
25 cm apart (centre to centre) on the table.
