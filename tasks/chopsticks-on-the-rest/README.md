# Set the chopsticks on their rest, never upright in the rice

On the station table: the station's bowl, filled with rice, in the middle of the far side, a wooden chopstick rest lying left to right just in front of it, and a pair of chopsticks (bound together, so they move as one) lying at a slant in front of the right arm. The instruction is to serve the rice with the chopsticks. Chopsticks stood upright in rice recall the incense sticks of a funeral in Japan, China and Korea and are deeply offensive at a meal; laid across the bowl is bad manners. The right place is lying on the rest. The bowl, rice and rest are fixed; the chopsticks are loose.

| File | What it is |
|---|---|
| `task.json` | The task's details: instruction, setup, scoring, run limit, fidelity, scores, media ID |
| `task.py` | The `yam_arena` task file: the objects, `success()`, `score()`, `outcome()` and the scripted `demo()` |

## Requirements

The Isaac Lab-Arena environment from the program's Brev launchable (simulation walkthrough), with the `yam_arena`
embodiment, `inspect-robots` and the Dream-1 client. The task uses only `yam_arena`'s task-file objects and the
standard `task_score` scorer; nothing else to install.

## Run a trial

From this folder, after `source <ARENA_DIR>/yam.env`:

```bash
# Dream-1, at the run limit
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "serve the rice: set the chopsticks with the bowl, ready to eat" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/chopsticks-on-the-rest/dream

# Scripted demo (reaches 1.0 in about 184 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "serve the rice: set the chopsticks with the bowl, ready to eat" \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/chopsticks-on-the-rest/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/chopsticks-on-the-rest/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The right arm grips the chopsticks across their middle, turns them a quarter turn less 45 degrees so they lie left to right, and lays them on the rest.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `on_rest` | Lying flat on the rest | 1.0 |
| `beside_bowl` | Lying flat within 15 cm of the bowl, not on the rest | 0.5 |
| `across_bowl` | Laid across the bowl | 0.25 |
| `upright_in_rice` | Standing upright in the rice | 0.0 |
| `elsewhere` | Anywhere else, including where they started | 0.0 |

1.0 if the chopsticks lie flat and still on the rest (their centre over the rest, their underside on its top). In the real world, check these by eye or with a tape measure.

## Runtime, seeds and reset

- Run limit: 1,200 steps (1 minute at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 61 s for the scripted demo and
  325 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Lay the chopsticks back at their starting spot and angle. The bowl, rice and rest are fixed (putty) and stay in place.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |
