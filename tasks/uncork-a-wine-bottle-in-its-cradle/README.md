# Uncork a wine bottle in its cradle

A full 75 cl bottle of red wine lies in a wooden pouring cradle, tipped up 10 degrees, its neck pointing back toward the
right arm. Its natural cork sits flush in the neck and grips it as hard as a real one: it takes 240 N to start it. A
wing corkscrew (real size, 250 g) has been set on the mouth, wings down, its point on the middle of the cork. The right
hand takes the square knob on top and turns it clockwise, a quarter turn at a time (turn, let go, turn back, take it
again), screwing the worm 40 mm into the cork; the rack draws in and the two wings rise. Then each hand comes down
beside a wing's paddle and pushes it down along its arc, both together; the rack draws the worm and the cork out of the
neck. The right hand lifts off, the left takes both wings the rest of the way down, and the right lifts the corkscrew
off the bottle by its bell, with the cork on the worm, and lays it on the table. Both arms go home. The task is one task
file, [`uncork_a_wine_bottle_in_its_cradle.py`](uncork_a_wine_bottle_in_its_cradle.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: using a real lever tool the way people use it: turning a knob in regrip strokes while following it as it
draws in, then pressing two levers together through a long arc against a force that neither arm can supply alone
within its working push, and finishing the job by taking the tool and the cork off the bottle.

Why two hands: a natural cork takes 240 N to start. The corkscrew multiplies the push at each paddle by 112.5 / 17.5 =
6.4, so the cork needs 37 N at the paddles together, about 19 N from each arm; one arm can give one paddle at most about
20 N (the YAM's working push), about 130 N at the cork, and the cork does not move. With one hand (the right doing its
part exactly as in the demo, the left never moving) the demo scored 0.0: the simulated arm, its target running ahead
of the stuck paddle, pushed it with up to 39 N (more than the real arm's working push), which drew the rack with
229 N; the cork crept 9 mm but never started, and when the hand then tried to take the corkscrew off, it dragged the
bottle out of its cradle with the cork still in.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The cradle is a fixed body; the bottle, the cork and
the corkscrew's five parts (bell, rack, worm spindle and two wings) are loose rigid bodies made of shapes from
`bodykit.py`, at real size and mass, joined by PhysX joints authored in the task file.

- The cork's grip (the calibrated stand-in for a compressed cork's friction). The cork slides out along the neck on a
  slide joint whose drive is capped at the friction force and re-aimed after every control step so that it never pulls
  the cork back: a Coulomb friction. The force is 240 N while the whole 38 mm is in the neck and falls in proportion to
  the length still in it (friction on the contact length). 240 N is a real value for natural 38 x 24 mm corks in the
  standard 18.5 mm bore: the Australian Wine Research Institute's fact sheet "Applying cylindrical closures" gives
  230-400 N (McMahon 1994) and 120-350 N (CE.T.I.E. 2008) for that size, measured as in ISO 9727-5. Once its inner end
  has passed the lip the cork is free of the neck. The cork weighs 10 g in the simulation, not its real 3.5 g: at
  3.5 g, between the neck's 240 N hold and the worm's thread, PhysX's solver shared the load wrongly (the worm slid
  12 mm out of the cork at 240 N); at 10 g the cork starts at 239 N and the worm gives about 3 mm. The extra 6.5 g is
  nothing against the forces involved.
- The worm's thread. PhysX has no screw thread. A joint keeps the cork on the worm's axis and lets the worm turn in it
  against the thread's drag; its slide (the depth of the point in the cork) is held by a stiff drive. Resting on the
  cork's face, the point cannot be pushed in unturned but can be drawn back out (the drive's target follows it out
  only). Turned 45 degrees clockwise there, the thread has caught, and from then on the target follows the turns, 10 mm
  per turn of the spindle against the rack, the way the real worm draws in. Turned back past its point, or drawn 3 mm
  out of the cork before it caught, the worm is free of the cork. (This joint is on from the start: PhysX did not carry
  the cork's 240 N reliably through a joint switched on during the run.)
- The rack and the pinions. The rack slides in the bell; the worm's spindle turns in the rack (a real wing corkscrew's
  rack has ring teeth and turns with the worm; splitting it into a non-turning rack and a turning spindle is the same
  motion). Each wing turns on a pivot in the bell and its pinion (17.5 mm pitch radius) meshes with the rack. PhysX's
  rack-and-pinion joint cannot run in this GPU scene, so the mesh is built from drives: before every physics step the
  task file takes the mean of where the rack and each wing say the gear is, and aims the rack's slide drive and each
  wing's pivot drive at it (60 kN/m at the rack, the same stiffness seen through the pinion at each wing). A torque on a
  wing reaches the rack as that torque over 17.5 mm, and the cork's pull comes back to the paddles the same way: about
  2 mm of give in the mesh under the full 240 N.
- The paddles' push is read from the gear: each wing's drive torque over the paddle's 112.5 mm lever. In a debug run of
  the demo the cork started at 240.5 N on the rack, with 18.1 N and 19.6 N square to the two paddles (peaks 18.9 N and
  19.7 N); then the pull fell with the cork's grip as it came out.
- The bell's collar. The bell's skirt fits over the bead with 1 mm to spare and its inner ring sits on the lip. Contact
  between rings of boxes did not hold a 250 g corkscrew level on a near-horizontal bottle (it drooped 13 degrees), so
  the collar's fit is a joint: it keeps the bell on the neck's axis, free to slide off it and to turn on it against
  the collar's friction on the lip (0.17 N m s/rad), until the collar has slid 15 mm off the bead; from then on the
  corkscrew is free of the bottle.
- The square knob is gripped across its flats, so the turn is passed on by the flats, not by friction alone; the flats
  come round every 90 degrees, which is how far the wrist turns in each stroke.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step; the task file also hooks
the scene's write to the simulator, which runs before every physics step, for the gear.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the bottle, the cork and the corkscrew's bell still,
and both hands still and at least 10 cm clear of them) has held for 1 s, otherwise `score()` at the end of the episode
(0, 0.25, 0.5 or 1.0; see `task.json`).
`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same layout.
The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it,
so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=uncork_a_wine_bottle_in_its_cradle.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Open the bottle of wine without taking it out of its cradle, and leave the cork off it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cork-dream
```

The scripted demo, which reaches full score: the right hand comes onto the square knob from the side, grips it across
its flats and turns it clockwise in quarter-turn strokes (turn, let go, turn back, take it again), following the knob
in as the worm draws into the cork, until the worm is about 40 mm in and the wings are up. It backs off and comes down
beside the front wing's paddle while the left comes down beside the back one, and both push their paddles down along
their arcs together, the closed fingers turning with the paddles and aimed 4 cm ahead of them, until the wings are at
150 degrees and the cork is nearly out. The right lifts off; the left takes both wings the rest of the way down,
drawing the cork clear of the neck, and goes home. The right takes the corkscrew by its bell, draws it off the bottle
with the cork on its worm, lays it on the table to the right and goes home:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=uncork_a_wine_bottle_in_its_cradle.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Open the bottle of wine without taking it out of its cradle, and leave the cork off it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cork-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/cork-dream/*.json; do inspect-robots video "$log"; done
```

Set `CORK_DEBUG=1` to print what the score is built from four times a second: how far the cork has been drawn out and
its grip, the worm's depth in the cork (and how far off its axis and tilted), whether the thread has caught and the
turns since, the rack's slide and both wings' angles, the mesh's give, the push on each paddle and the pull on the rack,
how far the bottle has moved and turned, and where each TCP is against the paddles and the knob.

## Runtime per trial

On one RTX 5090: about 16 minutes for a Dream-1 trial of 2,400 steps (980 s on the clock, with Dream answering in 0.67 s on average), and about 21 minutes for the scripted demo (1,706 steps, 85.3 s of simulated time, 1284 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
