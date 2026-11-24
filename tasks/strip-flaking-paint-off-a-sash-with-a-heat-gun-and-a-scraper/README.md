# Strip three patches of old paint off a window sash: one hand keeps the heat gun on the paint while the other scrapes it

A wooden window sash lies flat on a dust sheet, its glass pane in the middle. It has been stripped to bare wood except
for three patches of old, cracked cream paint along its near rail. To the left an orange heat gun stands on its wire
stand, already running; to the right lies a paint scraper, its handle on a little wooden rest. One hand takes the heat
gun and aims it, nose down, at the first patch from a few centimetres away; once the paint blisters, the other hand sets
the scraper's blade down on the near side of the patch and pushes it through the hot paint toward the gun while the
gun keeps heating,
and the paint comes off in curls onto the dust sheet. Then the next patch, and the last. The heat gun goes back on its
stand, the scraper back on its rest, and both arms go back to where they started. The task is one task file,
[`strip_flaking_paint_off_a_sash_with_a_heat_gun_and_a_scraper.py`](strip_flaking_paint_off_a_sash_with_a_heat_gun_and_a_scraper.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports
from the same folder.

What it tests: using two tools on one spot at the same time (one heating, one scraping), keeping a hot tool aimed where
it should be and away from what it would damage, and putting both tools back.

Why two hands: old paint softens only while it is hot. In the task it blisters after about two seconds of heat and
hardens again about two seconds after the heat leaves it, so the scraper has to go through the patch while the gun is
still on it; a single hand cannot hold both tools. This is a rule standing in for the paint's heat, not contact
physics. With one hand (the right doing its part of the demo, the left never moving) the demo scored 0: the scraper
went through all three patches, but with the heat gun still on its stand the paint stayed hard and nothing came off.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The dust sheet, the sash with its glass, the stand and
the scraper's rest are kinematic, fixed where they are put; the heat gun and the scraper are loose rigid bodies made of
shapes from `bodykit.py`.

- Heat: after every control step the task file follows the line along the gun's nozzle from its tip. Where it first
  meets the rail's top (or, over the opening, the glass) within 10 cm of the tip is what the gun heats. Each patch
  keeps a heat value: it rises by 0.5 a second while the gun points at the patch (within 2 cm of it) and falls by 0.5 a
  second otherwise, between 0 and 1.6. From 1.0 the patch is blistered (the flat paint is swapped for blistered paint)
  until its heat falls below 0.6.
- Glass: pointed at the glass for a second in all, it cracks (crack lines appear on it) and the trial scores 0.
- Scraping: the scraper's blade edge, on the wood (from 4 mm below to 7 mm above the rail's top), over the rail's
  middle and nearly level across, takes the paint off wherever it passes through a blistered patch, in 2 mm strips.
  Once 24 of a patch's 30 strips are done, the patch is stripped: its paint disappears and curls of paint appear on
  the dust sheet in front of the rail.
- The patches, blisters, curls and cracks are drawn shapes that nothing collides with; they wait, hidden, under the
  glass until they are needed.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the heat gun and the scraper still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or
1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial
starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to
320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=strip_flaking_paint_off_a_sash_with_a_heat_gun_and_a_scraper.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Strip the three patches of old paint off the window sash's near rail with the heat gun and the scraper: with one hand hold the heat gun aimed at a patch, a few centimetres from it, until the paint blisters, and keep it there while the other hand pushes the scraper's blade along the rail through the patch to take the soft paint off; then do the next patch, and the last. Never point the heat gun at the glass. Then stand the heat gun back on its stand, lay the scraper back with its handle on its rest, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/strip-dream
```

The scripted demo, which reaches full score: the left hand comes down on the heat gun's housing, fingers closing across
it, lifts it off its stand and aims it nose down at the first patch from the far left, its nozzle 8 cm from the paint.
Meanwhile the right takes the scraper by its handle, 9 cm behind the blade. Once the paint has blistered, the right sets
the blade down on the near side of the patch and pushes it through the patch toward the gun while the gun keeps
heating; then the scraper moves on to the next patch, the gun is lifted and follows. After the last patch the gun goes back on its stand and the scraper
back with its handle on its rest, and both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=strip_flaking_paint_off_a_sash_with_a_heat_gun_and_a_scraper.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Strip the three patches of old paint off the window sash's near rail with the heat gun and the scraper: with one hand hold the heat gun aimed at a patch, a few centimetres from it, until the paint blisters, and keep it there while the other hand pushes the scraper's blade along the rail through the patch to take the soft paint off; then do the next patch, and the last. Never point the heat gun at the glass. Then stand the heat gun back on its stand, lay the scraper back with its handle on its rest, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/strip-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/strip-dream/*.json; do inspect-robots video "$log"; done
```

Set `STRIP_DEBUG=1` to print what the score is built from four times a second: what the gun points at and where, how
steeply and which way it points, where it sits in the hand, each patch's heat, whether it is blistered, how many of its
strips are done and whether it is stripped, how long the glass has been heated, where the scraper's blade edge is,
whether each tool is back, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (139 s on the clock, with Dream answering in 0.69 s on average), and about 2 minutes for the scripted demo (815 steps, 40.8 s of simulated time, 105 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
