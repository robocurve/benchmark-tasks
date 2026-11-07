# Plant a seedling: hold it upright in the hole with one hand while tipping the soil in with the other, then let it stand

A seedling stands in a small nursery cell on a potting bench, next to a square planting hole, and a tray of soil clods
lies at the hole's edge. The task is to plant the seedling the way every gardener does: stand it upright in the middle
of the hole, keep holding it while the soil goes in around it, and let go once it stands on its own. Its root plug has
a round bottom, so on the hole's flat floor the seedling falls over unless it is held, and soil tipped in afterwards
fixes it leaning: one hand holds it upright while the other tips the soil in. The task is the task file
[`plant_seedling_holding_it_upright.py`](plant_seedling_holding_it_upright.py) for the `yam_arena` embodiment (scene,
scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres. The seedling (a sphere and a cylinder for the root
plug, a stem and three leaves), the tray (a floor, three walls and a handle post) and the soil clods are loose rigid
bodies built from several shapes, so `bodykit.py` adds such bodies: importing it hooks `yam_arena.scene` when the
simulator starts, without changing yam_arena, and gives `score()` each body's own pose. Its step hook also lets the
task file compute both grippers' positions from the arms' joint positions (yam_arena's own URDF kinematics), which the
scoring uses to check that the seedling stands on its own; and it lets the demo's waypoints carry a full gripper
rotation, for the side grip on the stem.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 2 s, otherwise the best `score()` reached during the
episode (0.5, 0.25 or 0). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial
starts from the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=plant_seedling_holding_it_upright.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Plant the seedling: take it out of its little cell, stand it upright in the middle of the planting hole and keep holding it while you tip the tray of soil into the hole around it, then let go so that it stands upright on its own." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/plant-dream
```

The scripted demo, which reaches full score (the left gripper takes the seedling's stem from its own side, lifts it
out of the cell, carries it over the hole and lowers it until the plug sits on the floor, then holds it upright; the
right gripper takes the tray's handle post from above and lifts the tray's back so that the tray turns about its front
edge and the clods slide into the hole around the plug; it lowers the tray and lets go; then the left lets go and
draws back, and the seedling stands):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=plant_seedling_holding_it_upright.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Plant the seedling: take it out of its little cell, stand it upright in the middle of the planting hole and keep holding it while you tip the tray of soil into the hole around it, then let go so that it stands upright on its own." \
  --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/plant-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/plant-dream/*.json; do inspect-robots video "$log"; done
```

`PLANT_DEBUG=1` prints, every 10 steps, where the seedling is and how far it leans, where the tray is and how far it is
tipped, how many clods are in the hole, both grippers' positions and the score so far.

## Runtime per trial

On one RTX PRO 6000 (Blackwell): about 6 minutes for a Dream-1 trial of 2,400 steps (334 s on the clock, with
Dream answering in 0.85 s on average), and about 1.5 minutes for the scripted demo while rendering 1280 x 720 video
(529 steps, 26.5 s of simulated time; 94 s). Isaac Sim's first start on a new machine adds a few minutes while it
compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A seedling in a 4 cm root plug (or a 4 cm ball of soil wrapped round its roots), a 5 cm block of wood or a raised bed
with a 10 x 10 cm hole 5 cm deep, a small square nursery pot, a shallow tray 8 cm wide open at one end with a handle at
the other, and about 130 g of soil clods or 1.4 cm wooden cubes.
