# Turn a long parcel in place between two neighbours, then push it lengthwise into a taped bay

A long cardboard parcel lies across the table between two other boxes, with only about 2 cm to spare at each end. It
has to end up lying lengthwise in a loading bay taped out in red ahead of it, pushed up to the bay's stop bar,
without bumping either neighbour. Two bollards at the mouth of the bay stand closer together (37 cm) than the parcel
is long (42 cm), so it cannot go forward lying across: it has to be turned a quarter turn first, right where it lies.
Pushed at one end, a long box swings about its other end and slides sideways into a neighbour; turned about its
centre, its corners sweep a circle that just clears the neighbours and the bollards. That takes both arms pushing the
two ends in opposite directions at the same time; once turned, the parcel slides between the bollards into the bay.
The whole task (scene, scoring and the scripted demo) is one task file,
[`turn_parcel_into_bay.py`](turn_parcel_into_bay.py), for the `yam_arena` embodiment.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task uses only boxes, so it needs no models from
Arena's object library.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` holds for 2 s, otherwise `score()` at the end of the episode
(0.5 or 0). Two rules hold for the whole trial and are latched in the task file's module state: if the parcel's
centre ever moves more than 4 cm to either side of the centre line, or either white box moves more than 1 cm, the
trial scores 0. Run one trial per process (as below) so this state starts fresh. `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=turn_parcel_into_bay.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Get the long brown parcel into the red-taped bay, lying lengthwise with its long sides running front to back, pushed up to the stop bar, without bumping the white boxes on either side of it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/parcel-dream
```

The scripted demo, which reaches full score (both closed grippers push the parcel near its two ends in opposite
directions, in small steps along circles about its centre, with one regrasp halfway, so it turns a quarter turn in
place; then one gripper comes round behind it and pushes it straight forward between the bollards to the stop bar):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=turn_parcel_into_bay.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Get the long brown parcel into the red-taped bay, lying lengthwise with its long sides running front to back, pushed up to the stop bar, without bumping the white boxes on either side of it." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/parcel-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/parcel-dream/*.json; do inspect-robots video "$log"; done
```

`TP_DEBUG=1` prints the parcel's position and heading, whether it is inside the bay, how far it has strayed sideways
and how far the white boxes have moved, every 10 steps.

## Runtime per trial

On one L40S: about 5 minutes for a Dream-1 trial of 2,400 steps (301 s on the clock, with Dream answering in 0.79 s
on average), and about 1 minute for the scripted demo at 320 x 176 (633 steps, 32 s of simulated time; 1.7 minutes
when rendering 1280 x 720 video). Isaac Sim's first start on a new machine adds a few minutes while it compiles its
shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A 42 x 14 x 10 cm shipping carton weighted to 800 g, two 20 x 14 x 10 cm cartons weighted to 1.2 kg, two 3 x 3 cm
posts 8 cm tall screwed to the table, a 22 cm long bar as the stop, and red floor tape for the bay. Mark the three
cartons' start positions with tape so they can be reset quickly.
