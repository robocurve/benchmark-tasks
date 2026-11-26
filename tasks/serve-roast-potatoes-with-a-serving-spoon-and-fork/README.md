# Serve roast potatoes with a serving spoon and fork

Two roast potatoes lie in a white serving dish, with a serving spoon and a two-tine serving fork resting in it. A dinner
plate stands to the right of the dish. The robot serves both potatoes onto the plate with the two tools, one in each hand,
without touching the food:

- The spoon is held tipped forward, its tip on the dish's floor, and the fork pushes a potato up into its bowl.
- The spoon lifts the potato, tipped back so it rests against the bowl's back rim, and carries it over the dish's rim.
- Over the plate the spoon tips forward and the potato slides off its open tip onto the plate.

The scene, scoring and scripted demo are in
[`serve_roast_potatoes_with_a_serving_spoon_and_fork.py`](serve_roast_potatoes_with_a_serving_spoon_and_fork.py), for
the `yam_arena` embodiment. It uses the helper [`bodykit.py`](bodykit.py), kept next to it, which builds the dish,
plate, potatoes, spoon and fork as rigid bodies made of boxes and cylinders.

## Requirements

- [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), tested at commit `0059d02`.
- Isaac Lab-Arena 0.3.1 and Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU.

The repository's `setup.sh`, or the program's Brev launchable, installs everything, including the Dream-1 policy. The
task uses only boxes and cylinders, so it needs no models from Arena's object library.

## Run and score a trial

The scorer is `task_score`. A trial scores 1.0 once `success()` holds for 1 s: both potatoes resting on the plate,
their middles within 11 cm of its middle. Otherwise the trial gets `score()` at the end of the run: 0.5 for one potato
on the plate, else 0. A gripper touching a potato, or a potato falling onto the bare table, scores 0. Use
`-E object_jitter=0` to keep every object where the task file puts it.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=serve_roast_potatoes_with_a_serving_spoon_and_fork.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Serve both roast potatoes onto the plate with the serving spoon and fork." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/serve-dream
```

The scripted demo:

1. The right hand takes the spoon's handle and the left hand takes the fork's, where they rest.
2. For each potato:
   1. The spoon's tip goes down on the dish's floor in front of the potato, tipped forward 8 degrees.
   2. The fork's tines go behind the potato, 12 mm up its side.
   3. The fork pushes the potato up into the bowl, then lifts out of the way.
3. The spoon lifts the potato, tipping back 15 degrees as it rises. It carries the potato 6 cm up over the dish's rim to
   the plate, then tips 35 degrees forward 2 cm above the plate, and the potato slides off onto it.
4. Both tools go back into the dish and the arms go back to where they started.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=serve_roast_potatoes_with_a_serving_spoon_and_fork.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Serve both roast potatoes onto the plate with the serving spoon and fork." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/serve-demo
```

Set `SRV_DEBUG=1` to print each potato's position relative to the plate, the hands' positions, and the spoon's and
fork's positions and tilt, once a second (`SRV_DEBUG_N=10` prints every 10 steps instead).

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/serve-dream/*.json; do inspect-robots video "$log"; done
```

## Runtime per trial

About 5 minutes for a 1,200-step Dream-1 trial on one NVIDIA L40S, about 1 minute of it starting Isaac Sim. The
scripted demo ends on success after about 650 steps and takes about 3 minutes.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. With a nonzero
`object_jitter`, the trial's seed sets how far the loose objects shift.

## The real trial

Use a white porcelain rectangular serving dish (26 x 18 cm outside, 2.5 cm deep), a 24 cm dinner plate, a 26 cm
stainless serving spoon and a 27 cm two-tine serving fork. Lay them out as in the setup and the screenshot, with the
potatoes on taped marks. For repeated trials, use two food-safe replicas of the potato chunks: 40 x 30 x 26 mm blocks
with the four long edges rounded to 5 mm, about 30 g, printed or carved. To score:

- Measure from the plate's middle to each potato's middle with a ruler.
- Check the video for a gripper touching a potato or a potato falling on the table.
