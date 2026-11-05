# Quench two hot aluminium fittings within 6 seconds of the furnace, and air-cool the glowing steel one

A furnace tray has just come out of the solution furnace with three hot fittings on it, and the clock starts with the
trial. The two bare aluminium fittings have to be under water in the quench tank within 6 s: the time from furnace to
water (the quench delay) is what keeps the alloy's strength, and thin parts lose it within seconds in air. The glowing
steel fitting is being normalised: it has to cool in still air on the cooling rack and must never go in the water (it
would crack). Hot parts are never set down on the bench. Carried one at a time, the second aluminium fitting would
reach the water about 10 s after the start; only both grippers working at once, one fitting each, make the deadline.
The whole task (scene, scoring and the scripted demo) is one task file,
[`quench_aluminium_fittings.py`](quench_aluminium_fittings.py), for the `yam_arena` embodiment.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task uses only boxes, so it needs no models from
Arena's object library.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` holds for 2 s, otherwise `score()` at the end of the episode
(0.5 or 0). The quench clock is counted from the poses at every control step (20 Hz). The "at no time" rules (an
aluminium fitting not under water by 6.0 s, or taken out of the water again; the steel fitting in the tank; a fitting
set down on the bench or on another fitting) are latched in the task file's module state: once broken, the trial
scores 0. Run one trial per process (as below) so this state starts fresh. `-E object_jitter=0` keeps every object
exactly where the task file puts it, so every trial starts from the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=quench_aluminium_fittings.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The furnace tray has just come out: put both bare aluminium fittings into the blue water tank within 6 seconds, and put the glowing steel fitting on the grey cooling rack. The steel fitting must never go in the water, and hot fittings may only be set down on the tray, in the tank or on the rack." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/quench-dream
```

The scripted demo, which reaches full score (both grippers, turned a quarter turn, take an aluminium fitting each at
the same moment, lift it over the tank wall and lower it into the water straight ahead, both under water about 5 s
after the start; then the right gripper carries the steel fitting to the cooling rack while the left one moves out of
its way):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=quench_aluminium_fittings.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The furnace tray has just come out: put both bare aluminium fittings into the blue water tank within 6 seconds, and put the glowing steel fitting on the grey cooling rack. The steel fitting must never go in the water, and hot fittings may only be set down on the tray, in the tank or on the rack." \
  --scorer task_score --max-steps 400 --store-frames --log-dir ~/yam-eval/quench-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/quench-dream/*.json; do inspect-robots video "$log"; done
```

`QA_DEBUG=1` prints where the three fittings are, when each aluminium fitting went under water and which rule, if
any, has been broken, every 10 steps.

## Runtime per trial

On one L40S: about 5 minutes for a Dream-1 trial of 2,400 steps (293 s on the clock, with Dream answering in
0.73 s on average), and under 1 minute for the scripted demo (278 steps, 14 s of simulated time; 51 s while
rendering 1280 x 720 video). Isaac Sim's first start on a new machine adds a few minutes while it compiles its
shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

Three 4 x 7 x 2.5 cm blocks (two aluminium, one steel; painted or lit to read as glowing), a 36 cm steel plate as the
tray, a 14 x 36 cm tray of water with 6 cm sides as the tank and three slats as the cooling rack. Run it cold, with a
stopwatch on the overhead video for the 6 s.
