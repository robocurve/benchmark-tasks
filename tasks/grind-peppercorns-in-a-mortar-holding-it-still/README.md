# Grind peppercorns in a marble mortar: hold the mortar still by a lug while the other hand grinds with the pestle

A white marble mortar with four rounded lugs at its rim stands on a polished black granite worktop, a small heap of black
peppercorns in its bowl. Its beech pestle stands on its broad end on a folded linen tea towel to the right. One hand
takes the mortar's back-left lug and holds the mortar still. The other takes the pestle, lifts it over the rim, lowers it
onto the bowl's floor and grinds round and round, pressing down, until every peppercorn is ground to powder. It then
lifts the pestle out and stands it back up on its tea towel, the first hand lets go of the lug, and both arms go back to
where they started. The task is one task file,
[`grind_peppercorns_in_a_mortar_holding_it_still.py`](grind_peppercorns_in_a_mortar_holding_it_still.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the
same folder.

What it tests: holding a light object still against a sustained, changing pull; using a hand tool in a steady circular
motion while pressing down; putting the tool back where it belongs.

Why two hands: the worktop is polished (friction 0.1), so the 0.45 kg mortar slides at about 0.4 N, while the pestle
pressed onto the bowl's floor and moved round drags it with a few newtons. Grinding counts only while the mortar keeps
still. With one hand (the right doing its part exactly as in the demo, the left never moving) the demo scored 0: the
mortar slid 4 cm and turned 16 degrees along with the pestle, and only 7.6 cm of rubbing counted out of the 47 cm needed.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The worktop and the tea towel are kinematic, fixed
where they are put; the mortar and the pestle are loose rigid bodies made of shapes from `bodykit.py`. The mortar's
bowl is a solid foot and a ring of 32 wall panels; the worktop's contacts take the lower of the two frictions (a
polished surface).

- Grinding: after every control step the task file looks at the pestle's end in the mortar's frame. While it is on the
  bowl's floor (within 8 mm of it, the pestle within 35 degrees of upright) and the mortar moved less than 1.5 mm and
  turned less than 0.75 degrees in that step, the distance the end moved across the floor counts as rubbing. Every
  9.4 cm of rubbing (one round 3 cm across) grinds a fifth of the peppercorns.
- What you see: the peppercorns are twelve drawn spheres on the bowl's floor and the ground pepper five drawn discs
  that do not collide. As the rubbing adds up, peppercorns are hidden and powder discs shown, until a brown layer of
  ground pepper covers the floor.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the mortar and the pestle still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0;
see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts
from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to
320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=grind_peppercorns_in_a_mortar_holding_it_still.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Grind the peppercorns in the marble mortar to powder: hold the mortar still by one of its lugs with one hand, and with the other take the wooden pestle and grind round and round on the bottom of the bowl until every peppercorn is ground. Then stand the pestle back up on its tea towel and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/mortar-dream
```

The scripted demo, which reaches full score: the left hand takes the mortar's back-left lug from above, its pads square
to the lug's sides and its wrist leaning out (back and to the left, clear of the right), and holds it. The right takes
the pestle by its handle just above the head, lifts it over the rim, lowers it onto the bowl's floor (aiming 2.5 mm into
it, so it presses down) and grinds eight rounds 3.6 cm across, about 2 s each. It lifts the pestle out and stands it
back up on its tea towel; the left lets go and both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=grind_peppercorns_in_a_mortar_holding_it_still.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Grind the peppercorns in the marble mortar to powder: hold the mortar still by one of its lugs with one hand, and with the other take the wooden pestle and grind round and round on the bottom of the bowl until every peppercorn is ground. Then stand the pestle back up on its tea towel and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/mortar-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/mortar-dream/*.json; do inspect-robots video "$log"; done
```

Set `MORTAR_DEBUG=1` to print what the score is built from four times a second: where the mortar is and how far it has
turned and tilted, where the pestle and its end are in the bowl, whether it is grinding, how much rubbing has counted
and how many rounds that makes, how far the mortar moved in the last step, whether the pestle is on its towel, the
finger openings and where each TCP is.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 900 steps (187 s on the clock, with Dream answering in 0.81 s on average), and about 4 minutes for the scripted demo (739 steps, 37.0 s of simulated time, 232 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
