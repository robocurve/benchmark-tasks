# Turn a cake out onto a plate: clamp plate and pan together with both hands, turn them over and lift the pan off

A round cake pan with a baked cake in it sits on a cake stand, and a flat plate lies upside down on the pan's rim. The
task is to turn the cake out onto the plate, as every home baker does: hold the plate and the pan together, turn the
two over as one so the cake drops onto the plate, set them down on the stand plate side down and lift the pan off. The
instruction states only the result (the cake upside down on the plate on the stand, the pan off). The
plate is 21 cm across, far wider than one gripper opens, and held from one side only the stack hangs off that grip and
comes apart, so the two arms clamp it from opposite sides and turn it together. The task is the task file
[`turn_cake_out_onto_plate.py`](turn_cake_out_onto_plate.py) for the `yam_arena` embodiment (scene, scoring and the
scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres. The pan (a floor, 24 wall panels, 24 rim panels and
two ears), the cake (with its crust) and the plate are loose rigid bodies built from several shapes, so `bodykit.py`
adds such bodies: importing it hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, and gives
`score()` each body's own pose. It also lets the demo's waypoints carry a full gripper rotation, for the side grips.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 2 s, otherwise the best `score()` reached during the
episode (0.5, 0.25 or 0). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial
starts from the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=turn_cake_out_onto_plate.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Turn the cake out of its pan onto the plate, so that it ends up upside down on the plate on the stand with the pan off it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cake-dream
```

The scripted demo, which reaches full score (both grippers come in level from the sides with their fingers upright,
clamp the plate and the pan's rim together, lift the stack, turn it over about the line between them, set it down on
the stand plate side down and let go; then each takes one of the pan's ears from above, and they lift the pan off the
cake, carry it towards the arms and set it down):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=turn_cake_out_onto_plate.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Turn the cake out of its pan onto the plate, so that it ends up upside down on the plate on the stand with the pan off it." \
  --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/cake-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/cake-dream/*.json; do inspect-robots video "$log"; done
```

`CAKE_DEBUG=1` prints, every 10 steps, where the pan, the cake and the plate are, how far each is turned over, and
each scoring condition.

## Runtime per trial

On one L40S: about 5 minutes for a Dream-1 trial of 2,400 steps (312 s on the clock, with Dream answering in
0.88 s on average), and about 3 minutes for the scripted demo while rendering 1280 x 720 video (650 steps, 32.5 s of
simulated time; 161 s). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A 14 cm round anodised-aluminium cake tin (about 2 mm sheet, 250 g) with a flat 2.5 cm rim and two small ears (or tabs
riveted to the rim), a flat white ABS plastic plate 21 cm across and 6 mm thick (220 g), a cake stand with a non-slip
silicone top (or any 10 cm pedestal about 13 cm across with a silicone mat on it), and a baked sponge cake 12.8 cm across
and 2.8 cm tall (or a 160 g foam disc) that fits loosely in the tin. In the simulation the friction is set per material:
pan and plate 0.6 static / 0.5 dynamic, cake 0.7 / 0.6, the silicone top and the gripper pads 1.2 / 1.0.
