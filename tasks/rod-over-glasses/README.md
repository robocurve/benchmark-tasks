# Lift a rod over a row of glasses onto two rests

Both arms lift a 40 cm wooden rod over three standing glasses and lay it across two wooden rests beyond
them. The whole task (scene, scoring and the scripted demo) is one task file,
[`rod_over_glasses.py`](rod_over_glasses.py), for the `yam_arena` embodiment.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's
Brev launchable, installs everything, including the Dream-1 policy. The task uses only boxes and cylinders, so
it needs no models from Arena's object library.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` holds for 3 s, otherwise `score()` at the end of the episode
(0.5 or 0). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial
starts from the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=rod_over_glasses.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lift the wooden rod over the glasses and lay it across the two wooden rests without knocking over any glass." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/rod-dream
```

The scripted demo, which reaches full score (both arms grip the rod 8 cm in from its ends, lift it to 16 cm,
carry it over the glasses and lower it onto the rests):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=rod_over_glasses.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lift the wooden rod over the glasses and lay it across the two wooden rests without knocking over any glass." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/rod-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/rod-dream/*.json; do inspect-robots video "$log"; done
```

## Runtime per trial

On one L40S: about 5 minutes for a Dream-1 trial of 2,400 steps (298 s on the clock, with Dream answering
in 0.73 s on average), and under 1 minute for the scripted demo (333 steps, 16.7 s of simulated time, 53 s on
the clock). Isaac Sim's first start on a new machine adds a few
minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. With a
nonzero `object_jitter`, the trial's seed sets how far the free objects (the rod and the glasses) shift.
