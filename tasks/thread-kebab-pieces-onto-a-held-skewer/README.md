# Hold a kebab skewer up with one hand and push four pieces onto it with the other

A metal kebab skewer lies across a small grill, its handle on one side wall and its point on the other. One hand
lifts it by the handle and holds it level; the other takes four kebab pieces off a cutting board one at a time, sets
each in the channel of a prep stand just past the skewer's point, hole toward the point, and pushes it onto the point
and up the skewer. Then the skewer goes back across the grill and both hands let go. The task is one task file,
[`thread_kebab_pieces_onto_a_held_skewer.py`](thread_kebab_pieces_onto_a_held_skewer.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

Why two hands: lying on the grill, the skewer rests on its point (flush with the outer face of the grill's wall), so no
piece can go on: it has to be lifted, and held level at the height of the stand's channel while each piece is pushed
on, which takes the other hand. One hand alone could only push pieces onto a skewer propped up on something at that
height, which the scene does not offer.

## How the pieces go on

The pieces are solid in the simulation, with the mouths of their holes printed on their two ends: the gripper's finger
hulls, pinching a hollow block, end up inside its hole. The skewer and the pieces never collide. The task file looks
after every control step for a piece whose hole has the skewer's point in its mouth (between 4 mm short of the end
face and 8 mm in, inside the hole's 12 x 16 mm outline, the two lined up within 15 degrees) and then switches on a
joint between them (a `Thread` from `bodykit.py`, here without a thread pitch): it keeps the piece on the blade with
3 mm and 12 degrees of play, free to turn about it and to slide along it against a slight drag, from the collar out to
the point. Pushed or pulled past the point, the piece comes off. Every reset takes all pieces off.

`yam_arena` task files place single boxes, cylinders and spheres. The grill, the skewer (handle, collar and blade),
the pieces and the stand are rigid bodies made of several shapes from `bodykit.py`; grill, board and stand are
kinematic, fixed where they are put. Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts,
without changing yam_arena, gives `score()` each body's own pose, and lets the task file look at the simulation after
every control step, where it puts pieces on the skewer and follows where the grippers are.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the skewer and every piece still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps
every object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at
1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=thread_kebab_pieces_onto_a_held_skewer.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the skewer level by its handle with one hand. With the other, set each kebab piece in the prep stand's channel, hole toward the skewer's point, and push it onto the point and up the skewer until all four pieces are on. Then lay the skewer back across the grill and let go." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/kebab-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the skewer's handle and lifts the
skewer level to the stand's height, its point at the channel. The right hand takes each piece straight down off the
board, sets it in the channel and lets go, then, its fingers half open round the blade, pushes the piece from behind
onto the point and up the skewer. Before each new piece the left hand tips the skewer up a little against the droop
that the pieces already on it cause. Then the left hand lays the skewer back across the grill, lets go and backs away:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=thread_kebab_pieces_onto_a_held_skewer.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the skewer level by its handle with one hand. With the other, set each kebab piece in the prep stand's channel, hole toward the skewer's point, and push it onto the point and up the skewer until all four pieces are on. Then lay the skewer back across the grill and let go." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/kebab-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/kebab-dream/*.json; do inspect-robots video "$log"; done
```

Set `KEBAB_DEBUG=1` to print what the score is built from (the skewer's axis and point, where each gripper's TCP is,
each piece's distance along and off the blade, which pieces are on, whether the skewer lies across the grill and
whether both hands have let go) twice a second.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,600 steps (176 s on the clock, with Dream answering in 0.56 s on average), and about 2 minutes for the scripted demo (1,160 steps, 58.0 s of simulated time, 150 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
