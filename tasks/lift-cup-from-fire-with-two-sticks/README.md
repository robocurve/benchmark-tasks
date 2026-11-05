# Lift a tin cup out of a campfire that is out of reach, using two sticks together as tongs

A tin cup stands in a small campfire pit at the far end of the table, beyond the reach of either arm. Two long
sticks lie on the table, one in front of each arm, and a flat stone lies in front of the fire. The cup has to come out
of the fire and stand on the stone. One stick alone can only push the cup about inside the ring of stones, which is
taller than a stick can lift it over; the two sticks, one in each gripper, used together like tongs, can pinch the
cup, lift it over the stones and set it down. The whole task (scene, scoring and the scripted demo) is one task file,
[`lift_cup_from_fire_with_two_sticks.py`](lift_cup_from_fire_with_two_sticks.py), for the `yam_arena` embodiment.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task uses only boxes and a cylinder, so it needs
no models from Arena's object library.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` holds for 2 s, otherwise `score()` at the end of the episode
(0.5 or 0). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from
the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=lift_cup_from_fire_with_two_sticks.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The tin cup in the fire pit is out of reach. Hold one stick in each gripper and use the two sticks together like tongs to lift the cup out over the stones and stand it on the flat stone." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/fire-dream
```

The scripted demo, which reaches full score (each gripper picks up a stick and turns it inward so that the two sticks
point at the fire and converge like the halves of a pair of tongs; the stick ends go in on either side of the cup and
pinch it; both lift it over the stones, carry it back, set it down on the flat stone and move apart to let go):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=lift_cup_from_fire_with_two_sticks.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The tin cup in the fire pit is out of reach. Hold one stick in each gripper and use the two sticks together like tongs to lift the cup out over the stones and stand it on the flat stone." \
  --scorer task_score --max-steps 460 --store-frames --log-dir ~/yam-eval/fire-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/fire-dream/*.json; do inspect-robots video "$log"; done
```

`LC_DEBUG=1` prints where the cup and the two sticks are, the cup's tilt and both scoring conditions, every 10 steps.

## Runtime per trial

On one L40S: about 5 minutes for a Dream-1 trial of 2,400 steps (291 s on the clock, with Dream answering in
0.76 s on average), and about 1 minute for the scripted demo (417 steps, 20.9 s of simulated time; 70 s while
rendering 1280 x 720 video). Isaac Sim's first start on a new machine adds a few minutes while it compiles its
shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A ring of bricks or stones about 4.5 cm high round a 15 cm square of orange card (no real fire), an empty tin mug,
two 34 cm lengths of square dowel, and a flat tile in front of the ring.
