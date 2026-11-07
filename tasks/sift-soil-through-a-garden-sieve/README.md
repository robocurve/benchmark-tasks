# Sift potting soil through a two-handled garden sieve over a tub, keeping the pebbles in the sieve

A square garden sieve full of potting soil, with a few pebbles in it, rests on two battens laid across a garden tub.
The task is to sift the soil the way gardeners do: lift the sieve off the battens by its two side handles, shake it
level over the tub until the soil has gone through the mesh, keep the pebbles in the sieve, and set the sieve back on
the battens. The sieve is wider than a gripper can span, with a flat handle on each side; held by one handle it tips
and pours pebbles into the tub, and loaded it sits too heavily on the battens to be shaken by sliding it there, so it
is held up by both handles and shaken. The task is the task file
[`sift_soil_through_a_garden_sieve.py`](sift_soil_through_a_garden_sieve.py) for the `yam_arena` embodiment (scene,
scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres. The sieve (a frame, a mesh of bars and two handles)
and the soil crumbs are loose rigid bodies built from several shapes, so `bodykit.py` adds such bodies: importing it
hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, and gives `score()` each body's own pose.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 2 s, otherwise the best `score()` reached during the
episode (0.5, 0.25 or 0; a pebble in the tub sets it to 0 for the rest of the run). `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=sift_soil_through_a_garden_sieve.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Sift the soil: lift the garden sieve off the battens by both of its handles, shake it level over the tub until the soil has gone through, keeping the pebbles in the sieve, then set the sieve back on the battens." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/sift-dream
```

The scripted demo, which reaches full score (each gripper comes in level from its own side with its fingers upright,
slides onto its handle from the front and clamps it from above and below; together they lift the sieve off the
battens, shake it round in small circles, level, until the soil has gone through, set it back on the battens and let
go):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=sift_soil_through_a_garden_sieve.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Sift the soil: lift the garden sieve off the battens by both of its handles, shake it level over the tub until the soil has gone through, keeping the pebbles in the sieve, then set the sieve back on the battens." \
  --scorer task_score --max-steps 1300 --store-frames --log-dir ~/yam-eval/sift-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/sift-dream/*.json; do inspect-robots video "$log"; done
```

`SIEVE_DEBUG=1` prints, every 10 steps, where the sieve is and how far it is tilted, how many crumbs are in the tub,
whether a pebble has gone in, and the score so far.

## Runtime per trial

On one RTX PRO 6000 (Blackwell): about 6 minutes for a Dream-1 trial of 2,400 steps (373 s on the clock, with
Dream answering in 0.75 s on average), and about 4 minutes for the scripted demo while rendering 1280 x 720 video
(880 steps, 44 s of simulated time; 223 s). Isaac Sim's first start on a new machine adds a few minutes while it
compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A square garden sieve (riddle) 20 cm across inside with a 14 mm mesh and a flat handle on two opposite sides, a
garden tub or seed tray 28 cm across and 6 cm deep with two battens laid across it, about 220 g of damp potting
soil and six pebbles 2.5 to 3 cm across.
