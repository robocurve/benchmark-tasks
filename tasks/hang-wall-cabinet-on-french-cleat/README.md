# Hang a wall cabinet on its French cleat

A small wall cabinet (an open-fronted shelf box) is to be hung on a French cleat. A wall section stands at the back of
the table with a cleat fixed to it: a strip whose top edge is bevelled at 45 degrees, sloping down toward the wall. The
cabinet stands on the table in front of it, with the matching cleat along the top of its back. The task is to hang it
the way a carpenter does: lift it by its two side panels, carry it level to the wall with its cleat above the wall's,
lower it so that its cleat drops behind the wall cleat's edge and the two bevels wedge it against the wall, and let go.

The cabinet is 32 cm wide and has nothing to hold in its middle: lifted by one side panel it tips (11 degrees in the
one-arm run below) and its other side stays on the table. Its cleat has to come down onto the wall cleat along its
whole length at once, so both hands carry it level and lower it together. The task is the task file
[`hang_wall_cabinet_on_french_cleat.py`](hang_wall_cabinet_on_french_cleat.py) for the `yam_arena` embodiment (scene,
scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres. The cabinet (panels, cleat and spacer) and the wall
cleat (with its bevelled face) are built from several shapes, so `bodykit.py` adds rigid bodies made of several shapes,
some of them fixed in place: importing it hooks `yam_arena.scene` when the simulator starts, without changing
yam_arena, and gives `score()` each body's own pose.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run, which is
the best stage reached (0.75, 0.5, 0.25 or 0) unless the cabinet tipped over (more than 30 degrees), which sets it to 0
for the rest of the run. `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial
starts from the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=hang_wall_cabinet_on_french_cleat.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hang the cabinet on the wall: lift it by both side panels, carry it level to the wall with its cleat above the wall cleat, lower it so the cleats hook together, then let go." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cleat-dream
```

The scripted demo, which reaches full score (each gripper comes down over the top edge of its side panel, above the
cabinet's centre of mass, and clamps it; together they lift the cabinet until its cleat clears the wall cleat's edge,
carry it 7.5 cm to the wall, lower it so its cleat drops onto the wall cleat's bevel, let go and move out to the sides):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=hang_wall_cabinet_on_french_cleat.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hang the cabinet on the wall: lift it by both side panels, carry it level to the wall with its cleat above the wall cleat, lower it so the cleats hook together, then let go." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/cleat-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/cleat-dream/*.json; do inspect-robots video "$log"; done
```

`CLEAT_DEBUG=1` prints, every 10 steps, how far the cabinet is from its hung position, how far it is tilted, where both
grippers are, and the stage reached so far.

`CLEAT_ONE_ARM=1` runs the same demo with the right arm kept still: the left gripper lifts its side panel exactly as
before, the cabinet tips up to 11 degrees with its other side still on the table, never counts as lifted, and the run
scores 0.

## Runtime per trial

On one RTX PRO 6000 (Blackwell): about 4 minutes for a Dream-1 trial of 2,400 steps (248 s on the clock, with
Dream answering in 0.78 s on average), and under 1.5 minutes for the scripted demo while rendering 1280 x 720 video
(368 steps, 18 s of simulated time; 76 s). Isaac Sim's first start on a new machine adds a few minutes while it
compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A board 60 x 15 cm standing upright as the wall, a 40 cm length of 1.5 x 3 cm pine ripped at 45 degrees along one edge
and screwed to it with the bevel up and toward the wall, its sharp edge 11 cm above the table, and a small pine shelf
box (30 x 10 x 12 cm inside, 12 mm sides) with the matching bevelled strip along the top of its back and a 1.5 cm
spacer along the bottom.
