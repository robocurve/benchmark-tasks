# Stand a flat box, too wide for one gripper, on edge in a narrow slot beside a riser

A flat, closed, handle-less cardboard box has to end up standing on edge in a narrow slot, like a file in a rack.
Lying flat, it is wider and deeper than a gripper opens, so no single gripper can pick it up, and it cannot be
turned on edge where it lies. Beside the slot stands a fixed riser exactly as tall as half the box's width: the box
has to be squeezed between both grippers, lifted onto the riser, and then tipped over the riser's edge, so that it
swings down about that edge and lands standing in the slot. Without the two-handed lift there is no way to get it
onto the riser, and without the riser there is nothing to tip it over. The whole task (scene, scoring and the
scripted demo) is one task file, [`box_on_edge_in_slot.py`](box_on_edge_in_slot.py), for the `yam_arena` embodiment.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task uses only boxes, so it needs no models from
Arena's object library.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` holds for 2 s (the box standing on edge in the slot and still),
otherwise `score()` at the end of the episode (0.5 when the box lies flat on top of the riser, else 0).
`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=box_on_edge_in_slot.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the flat white box into the slot between the wooden riser and the wall, standing on edge with its longest side upright." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/box-edge-dream
```

The scripted demo, which reaches full score (both closed grippers, turned sideways, press the box's left and right
faces with the outer faces of their fingers, lift it, carry it over the riser and set it down flat on top; the left
arm then moves clear, and the right gripper pushes the box left across the riser until it tips over the edge and
lands standing in the slot):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=box_on_edge_in_slot.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the flat white box into the slot between the wooden riser and the wall, standing on edge with its longest side upright." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/box-edge-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/box-edge-dream/*.json; do inspect-robots video "$log"; done
```

`BE_DEBUG=1` prints the box's position, how far its underside is above the table, how upright its long side is, and
whether it is on edge in the slot or flat on the riser, every 10 steps.

## Runtime per trial

On one L40S: about 5 minutes for a Dream-1 trial of 2,400 steps (279 s on the clock, with Dream answering in
0.77 s on average), and about 1 minute for the scripted demo (455 steps, 23 s of simulated time; 71 s while
rendering 1280 x 720 video). Isaac Sim's first start on a new machine adds a few minutes while it compiles its
shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A 12 x 20 x 9 cm closed carton weighted to 350 g, a 12 x 12 x 10 cm wooden block and a 14 x 1.5 x 9 cm wooden strip
screwed to the table 14.5 cm apart. A tape mark for the carton's start position makes the reset quick.
