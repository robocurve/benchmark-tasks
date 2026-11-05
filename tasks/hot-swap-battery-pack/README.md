# Hot-swap a running station's failing battery pack without its bay ever staying empty more than 4 seconds

A running field base station has one battery bay, and its pack is failing (red tile). It has to be swapped for the
charged spare in the charger rack (green tile), not for the pack still charging (amber tile), and the failing pack
goes into the fire-safe return bin. The station runs with its bay empty only on its hold-up capacitor, for at most
4 s, and lithium packs are never set down loose: the failing pack may only be in the bay, the bin or a gripper, the
spare only in its cradle, the bay or a gripper. A single gripper would have to take the failing pack all the way to
the bin before it could fetch the spare, leaving the bay empty for far longer than 4 s. The swap is make-before-break:
one gripper holds the spare ready beside the bay while the other pulls the failing pack, and the spare goes in the
moment the bay is clear. The whole task (scene, scoring and the scripted demo) is one task file,
[`hot_swap_battery_pack.py`](hot_swap_battery_pack.py), for the `yam_arena` embodiment.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task uses only boxes, so it needs no models from
Arena's object library.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` holds for 2 s, otherwise `score()` at the end of the episode
(0.5 or 0). The empty-bay time is counted from the poses at every control step (20 Hz). The "at no time" rules (bay
empty for more than 4 s in a row, a pack set down where it may not be, a pack leaning more than 25 degrees, the
charging pack moved more than 1 cm) are latched in the task file's module state: once broken, the trial scores 0. Run
one trial per process (as below) so this state starts fresh. `-E object_jitter=0` keeps every object exactly where
the task file puts it, so every trial starts from the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=hot_swap_battery_pack.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Swap the failing battery pack in the running station (red tile) for the charged spare (green tile), not the one still charging (amber tile), and put the failing pack in the orange return bin. The station can run for at most 4 seconds with its bay empty, and packs may only be set down in the bay, their own charger or the return bin." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/hotswap-dream
```

The scripted demo, which reaches full score (the right gripper lifts the charged spare out of its cradle and holds it
just right of the bay; the left gripper grips the failing pack; then, on one timeline, the left gripper lifts the
failing pack clear of the bay walls, both grippers move left together and the right one lowers the spare into the
bay, about 2 s after the bay emptied; the left gripper then puts the failing pack in the bin):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=hot_swap_battery_pack.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Swap the failing battery pack in the running station (red tile) for the charged spare (green tile), not the one still charging (amber tile), and put the failing pack in the orange return bin. The station can run for at most 4 seconds with its bay empty, and packs may only be set down in the bay, their own charger or the return bin." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/hotswap-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/hotswap-dream/*.json; do inspect-robots video "$log"; done
```

`HS_DEBUG=1` prints where the spare and the failing pack are, how long the bay has been empty (and the longest so
far) and which rule, if any, has been broken, every 10 steps.

## Runtime per trial

On one L40S: about 5.5 minutes for a Dream-1 trial of 2,400 steps (332 s on the clock, with Dream answering in
0.80 s on average), and about 1 minute for the scripted demo (316 steps, 16 s of simulated time; 62 s while
rendering 1280 x 720 video). Isaac Sim's first start on a new machine adds a few minutes while it compiles its
shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

Three identical 7 x 6 x 8 cm dummy packs (400 g; 3D-printed shells with steel shot), a bay and two cradles printed or
cut as 3 cm high pockets with 1 cm clearance, a 24 cm wide housing block behind the bay, a 12 x 12 cm bin with 4 cm
walls, and coloured tape for the tiles. A timer on the overhead video checks the empty-bay time.
