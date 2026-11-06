# Feed a manikin a spoonful of soup with one hand while the other holds the bowl under its chin

A vinyl care-training manikin sits at the far side of the table with its mouth open; a steel feeding spoon with a
spoonful of soup (four beads) lies on a wooden rest, and a porcelain soup bowl with a long handle stands to the left. One hand lifts the bowl and holds
it under the manikin's chin; the other brings the spoon to the mouth, tips the soup in and brings the spoon out. The
task is one task file, [`feed_a_manikin_a_spoonful_over_a_held_bowl.py`](feed_a_manikin_a_spoonful_over_a_held_bowl.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports
from the same folder.

Why two hands: this is how carers are taught to feed someone, and the score asks for it. The bowl must be held under
the chin when the soup goes in, to catch what drips off the spoon; fed without it, a spoonful scores 0.5 at most, and
soup that drips onto the table counts against it.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The manikin, the spoon, its rest, the bowl and the
soup beads are rigid bodies made of shapes from `bodykit.py`; the manikin and the rest are kinematic, fixed where they
are put.

- The manikin's head is built of boxes round an open mouth: a cavity 5.5 cm deep behind the lips, whose floor drops
  1.5 cm behind the lower lip into a pit. Soup tipped off the spoon in the mouth falls into the pit and stays there;
  only soup in the pit counts as swallowed (not soup still on the spoon in the mouth).
- The soup is four beads 9 mm across, low friction and a little damped, in the spoon's bowl (4 x 2.2 cm, 4 mm rims).
- After every control step the task file notes where the bowl is at the moment the first bead is swallowed.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file look at the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the beads still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object
exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for
sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=feed_a_manikin_a_spoonful_over_a_held_bowl.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the soup bowl under the manikin's chin with one hand, and with the other bring the spoon to its mouth and tip the spoonful in, spilling none." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/feed-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the bowl's handle near its end,
lifts the bowl and holds it under the manikin's chin. The right hand comes straight down on the spoon's handle, lifts
the spoon level off its rest, carries it level to the mouth, puts it in and tips it 60 degrees about its handle, so the
soup runs off into the mouth; then it levels the spoon and brings it out:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=feed_a_manikin_a_spoonful_over_a_held_bowl.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the soup bowl under the manikin's chin with one hand, and with the other bring the spoon to its mouth and tip the spoonful in, spilling none." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/feed-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/feed-dream/*.json; do inspect-robots video "$log"; done
```

Set `FEED_DEBUG=1` to print what the score is built from (where the bowl is and whether it is under the chin, where
the spoon and each bead are, how many beads are swallowed and spilt, whether the bowl was under the chin when the
first went in, and where each gripper's TCP is) twice a second.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,200 steps (162 s on the clock, with Dream answering in 0.75 s on average), and about 1 minute for the scripted demo (401 steps, 20.1 s of simulated time, 61 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
