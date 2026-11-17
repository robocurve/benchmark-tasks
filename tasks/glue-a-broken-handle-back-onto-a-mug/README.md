# Mend a mug: hold it still on its side with one hand while the other glues its broken handle back on

A white mug lies on its side on a green cutting mat, its mouth to the left and its broken side toward the arms: two
small bare patches of unglazed clay show where its handle snapped off. The handle lies on the mat in front of it, next to
a small tube of superglue with its cap off. One hand holds the mug down so it cannot roll. The other picks up the glue
tube, touches its nozzle to both bare patches (a drop of glue appears on each), and lays the tube back. It then takes
the handle by its lower arm, lifts it level with the patches, presses its two broken ends onto them and holds it still
until the glue grabs. Both hands let go, the handle stays on the mug, and both arms go back to where they started. The
task is one task file, [`glue_a_broken_handle_back_onto_a_mug.py`](glue_a_broken_handle_back_onto_a_mug.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests: steadying an object that rolls while the other hand works on it; doing the steps in order (glue first,
then the handle); holding a part still in place for a while.

Why two hands: a mug lying on its side rolls as soon as it is pushed from the side, and both the nozzle and the handle
push on its side, and the glue grabs only while the handle is held still against a still mug. With
one hand (the right doing its part as in the demo, the left never moving) the demo scored 0.25: the nozzle put glue
on both patches, but pressing the handle on rolled the mug 4 cm away and turned it 52 degrees, so the glue never
grabbed, and the handle fell to the mat when the hand let go.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The mat is kinematic, fixed where it is put; the mug,
the handle and the glue tube are loose rigid bodies made of shapes from `bodykit.py`. The mug is a 48-sided glazed tube
with a base, lying on its side, so it rolls when pushed from the side.

- The glue: after every control step the task file checks the nozzle's tip against each bare patch; within 6 mm, that
  patch has glue, and a drop of glue (hidden until then) is shown on it.
- The bond: once both patches have glue, the task file checks the handle against where it belongs on the mug (within
  6 mm and 10 degrees), whether a gripper holds it (fingers closed, within 6 cm of it) and whether the mug is still
  (under 0.8 cm/s and 0.15 rad/s). When all of that has held for 3 s without a break, a fixed joint (a `Glue` from
  `bodykit.py`) switches on and holds the handle on the mug for good. The joint ties the handle to a hidden part glued
  to the mug, since a joint straight between the mug and the handle would stop them touching even while switched off.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the mug and its handle still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0;
see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts
from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to
320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=glue_a_broken_handle_back_onto_a_mug.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Glue the broken handle back onto the mug lying on its side: hold the mug still with one hand. With the other, take the glue tube and touch its nozzle to both bare patches where the handle broke off, then lay the tube back down. Take the handle and press its two broken ends onto the bare patches, and hold it there without moving until the glue grabs (3 seconds). Then let go of the handle and the mug, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/mug-dream
```

The scripted demo, which reaches full score: the left hand comes down over the mug near its mouth with its fingers half open, so its two fingertips press on the mug's flanks either side of the top (a V the mug cannot roll out of), leaning out so the wrists stay apart, and holds it down. The right takes the glue tube by its middle, touches the nozzle to the lower bare patch and then the upper one (a drop of glue appears on each), and lays the tube back. It takes the handle by its lower arm, turning its grip back 11.5 degrees about the arm as it lifts (the V-jaw rolls the handle that much), carries it level with the patches, presses its broken ends onto them and holds it still for 5 s; the glue grabs after 3 s. The right lets go and rises, the left lets go of the mug, the handle stays on (the mended mug rolls a little until its new handle rests on the mat), and both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=glue_a_broken_handle_back_onto_a_mug.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Glue the broken handle back onto the mug lying on its side: hold the mug still with one hand. With the other, take the glue tube and touch its nozzle to both bare patches where the handle broke off, then lay the tube back down. Take the handle and press its two broken ends onto the bare patches, and hold it there without moving until the glue grabs (3 seconds). Then let go of the handle and the mug, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/mug-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/mug-dream/*.json; do inspect-robots video "$log"; done
```

Set `MUG_DEBUG=1` to print what the score is built from four times a second: where the mug is and how fast it moves,
where the nozzle's tip is, which patches have glue, how far the handle is from where it belongs (distance and angle),
whether it is held, whether the mug is still, how long the hold has lasted, whether the glue has grabbed, and where
each TCP is.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 900 steps (174 s on the clock, with Dream answering in 0.77 s on average), and about 3 minutes for the scripted demo (680 steps, 34.0 s of simulated time, 154 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
