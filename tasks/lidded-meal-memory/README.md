# Lid three lunch boxes, then serve the gluten-free one, which no longer shows once lidded

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [lidded_meal_memory.py](lidded_meal_memory.py).

Three open lunch boxes stand in a row: rice in the middle one, pasta in the two outer ones. The robot is told to lid all three and then put the gluten-free one, the rice, on the tray. Once the identical lids are on, nothing in any camera image tells the boxes apart, so the robot has to remember which one held the rice. This is the object-permanence problem that memory benchmarks for robot policies (RoboMME, LIBERO-Mem) test with cubes and buttons, here as a kitchen job where serving the wrong covered meal to a guest with coeliac disease is a real error. Serving a box before all three are lidded, which would skip the memory step, scores 0, as food is covered before it is carried.

The tray stands beside the right-hand box, so serving the box nearest the tray serves pasta.

[partkit.py](partkit.py) is copied unchanged from [tasks/din-rail-mount](../din-rail-mount/partkit.py). It builds each box and lid as one rigid part from several boxes and gives the task a reset callback, which clears the record of an early move at the start of each trial.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=lidded_meal_memory.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put lids on the three lunch boxes, then put the gluten-free one on the green tray. The rice is gluten-free." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/lidded-meal-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 800`. The left arm lids the left and middle boxes and the right arm the right one, each lid lifted by its knob and lowered onto the rim; then the right arm carries the middle box, lid on, to the tray.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the boxes and lids end and on whether a box moved too early.

## Checks

Scripted variants of the same scene, each replacing only `demo()`:

| Variant | Score |
|---|---|
| Demo: lid all three, serve the middle (rice) box | 1.0 |
| Lid all three, serve the box nearest the tray (pasta) | 0.0 |
| Lid all three, serve the left (pasta) box | 0.0 |
| Serve the rice box without lidding | 0.0 |
| Lid two boxes, then serve the rice box | 0.0 |
| Lid all three, serve nothing | 0.25 |
| Nothing done | 0.0 |

The demo was also run for two trials in one process; both scored 1.0. A box that is lidded and then carried can shift its lid by a few millimetres in the lift; the early-move check only applies until all three boxes have been lidded at the same time.

## Runtime and seeds

- Demo: about 730 steps (37 s simulated), about two minutes on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 6 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.
