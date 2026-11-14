# Draw a 10 ml dose from a vented vial into a syringe without drawing air, then stand the syringe back in its rack

A vented 50 ml vial of training liquid (water dyed red) stands in a weighted holder, its rubber stopper up; beside it
an empty 20 ml syringe with a 38 mm blunt fill needle stands needle down in a syringe rack. The instruction states only
the goal: 10 ml in the syringe, no air. In the demo one hand takes the syringe out of the rack by its barrel, pushes
the needle down through the stopper, its tip well under the liquid, and holds the syringe there; the other pulls the
plunger up to the red 10 ml mark, and the liquid fills the clear barrel, red, below the plunger's tip. Then the first hand pulls the needle out, stands the
syringe back in its rack and lets go, and both arms go back to where they started. The task is one task file,
[`draw_10_ml_from_a_vial_into_a_syringe.py`](draw_10_ml_from_a_vial_into_a_syringe.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: steadying one part against the pull the other hand puts on it; drawing a measured amount (10 ml within
1 ml, read off the barrel's marks); keeping the needle's tip under the liquid's falling surface while doing it;
putting the tool away.

Why two hands: the needle holds in the stopper only lightly (about 0.25 N), and the plunger takes about 0.8 N to move,
so pulling the plunger of a syringe nobody holds draws the whole syringe off the needle. With one hand (the left putting
the needle in as in the demo, letting go of the barrel, then taking the plunger's rod and pulling it up) the syringe
came off the needle with the plunger, having drawn 0.07 ml (0.25); with the left doing just its part of the demo and the
right never moving, the syringe went back to its rack empty (0.25).

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The vial, its cap and the rack are kinematic, fixed
where they are put; the syringe (barrel, hub and needle) and its plunger are loose rigid bodies made of shapes from
`bodykit.py`.

- The plunger: a slider in the barrel (a `Slider` from `bodykit.py`). A spring holds it where it last stopped; after
  every control step, a plunger pushed more than 1 mm past its hold has given, and the task file moves the hold there,
  so it stays where it is left. That takes about 0.8 N.
- The needle in the stopper: a sliding joint (a `Thread` from `bodykit.py`, a PhysX D6 joint) between the vial and the
  syringe, switched on when the needle's tip goes into the stopper's exposed rubber with the syringe tilted no more
  than 15 degrees; it keeps the needle on the vial's axis (with 3 mm of play) and holds it with the same kind of
  stick-slip hold, about 0.25 N. It is switched off when the needle comes back out.
- The liquid: not a fluid. The vial holds 50 ml (inside 39 mm across, 11.9 cm2), its surface starting 14.6 mm
  below the stopper's top. After every control step the task file adds up how far the plunger has been drawn (3.18 mm
  per ml): as liquid while the needle is in the stopper with its tip below the liquid's current surface, otherwise as
  air. Each ml of liquid drawn takes the surface 0.84 mm lower; pushing the plunger back in empties liquid first (the
  needle points down), back into the vial if the needle is still in it. With the 38 mm needle in to its hub's stop the
  tip reaches 35 mm below the stopper's top, so it can stay under the surface (23.0 mm down after 10 ml) throughout the
  draw. Air pressure is not modelled: the real vial's cap has a filtered vent, so air replaces what is drawn. Inside
  the clear barrel (its collision cylinder is not drawn) a red cylinder shows below the plunger's tip.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step. This copy of `bodykit.py`
also lets a shape be collision only (`color=None`): not drawn, as the clear barrel is.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started,
with the syringe and plunger still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see
`task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from
the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176
before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=draw_10_ml_from_a_vial_into_a_syringe.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Draw 10 ml of the red liquid from the vial into the syringe without drawing in air, then stand the syringe back in its rack, let go of it, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/syringe-dream
```

The scripted demo, which reaches full score: the left hand comes in level from the left round the syringe's barrel,
takes it, lifts it out of the rack, carries it over the vial and pushes the needle straight down through the stopper,
33 mm deep (its tip under the liquid), and holds it. The right hand comes in level from the right round the plunger's rod, takes it and pulls
it up to the 10 ml mark, lets go and draws back. The left pulls the needle out, carries the syringe back, stands it in
its rack and lets go, and both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=draw_10_ml_from_a_vial_into_a_syringe.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Draw 10 ml of the red liquid from the vial into the syringe without drawing in air, then stand the syringe back in its rack, let go of it, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/syringe-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/syringe-dream/*.json; do inspect-robots video "$log"; done
```

Set `SYRINGE_DEBUG=1` to print what the score is built from twice a second: how deep the needle's tip is below the
stopper's top and how far it is off the vial's axis, whether the needle is caught in the stopper, how far the plunger
has been drawn (and in ml), how far the tip is under the liquid's surface and how much is left in the vial, how much
of the syringe's contents is liquid and how much air, whether the syringe stands in its rack,
whether both hands are clear, and where each TCP is from the barrel and the plunger's rod.

## Runtime per trial

On one RTX 5090: about 7 minutes for a Dream-1 trial of 1,200 steps (443 s on the clock, with Dream answering in 0.50 s on average), and about 4 minutes for the scripted demo (572 steps, 28.6 s of simulated time, 219 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
