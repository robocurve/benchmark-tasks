# Crack an egg into a bowl: hold it by both ends, tap it on the rim, pull the shell apart over the bowl, tray the shells

A brown hen's egg lies on its side on a small beech egg rest, a white mixing bowl stands in the middle of the table and
a steel scraps tray lies along the front. The egg's middle is tapped down on the bowl's rim to crack it; then, held
over the bowl with one hand on each end, the two halves of the shell are pulled apart so the yolk and the white drop
into the bowl. Both shell halves go into the scraps tray, none into the bowl, and both arms go back to where they
started. The task is one task file,
[`crack_an_egg_into_a_bowl_with_both_hands.py`](crack_an_egg_into_a_bowl_with_both_hands.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

What it tests: a two-handed kitchen job in order (crack, then open over the bowl), a measured tap on a rim, pulling one
object apart with one hand on each part, and putting the pieces in the right places.

Why two hands: a cracked shell still holds together, up to 4 N, until its halves are pulled 8 mm apart, and a hand
holding the egg by one end has nothing to pull the other end with. With one hand (the right doing its part as in the demo, the left never moving) the demo scored 0.25: the right took the egg by one end and knocked it on the rim, so it cracked, but the halves never came apart; the egg ended whole in the bowl, shell and all.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres; the bodies here are made of several shapes each, from
`bodykit.py`.

- The shell: two rigid halves, each a ring of panels narrowing toward its end, with a sloping inner membrane that keeps
  what is inside at the middle. They are 80 g each, heavier than a real shell, because the fingers' collision hulls
  sink into a very light body. Two prismatic joints (`Slider`s from `bodykit.py`) hold them in line, letting them part
  only along the egg's length: a stiff one (the whole shell) and a membrane that holds with at most 4 N.
- Cracking: after every control step the task file checks the egg's middle. Once it comes down onto the bowl's rim
  (within 2 cm of the rim's middle line, its underside within 8 mm of the rim's top, coming down at 3 cm/s or faster
  over the last 0.3 s), the egg is cracked, and half a second later the task file switches the stiff joint off. Once the
  halves are then 8 mm apart, it switches the membrane off and the halves are loose.
- The yolk is a 2.1 cm sphere and the white six 11 mm beads, inside the shell.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with the yolk, the white and both shell halves still) has
held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`).
`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before
sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=crack_an_egg_into_a_bowl_with_both_hands.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Crack the egg into the white bowl: tap its middle on the bowl's rim to crack it, then pull the two halves of the shell apart over the bowl so the yolk and the white drop in. Put both shell halves in the steel scraps tray, none in the bowl, and take both arms back to where they started." \
  --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/egg-dream
```

The scripted demo, which reaches full score: both hands, leaning out to their own sides, take the egg on its rest, one
on each half, lift it together and hold it over the bowl. Together they bring the egg's middle over the bowl's rim on
its right side and tap it down on the rim: it cracks. Back over the middle of the bowl, they pull the halves apart: the
yolk and the white drop into the bowl. Each hand sets its half down in the scraps tray, lets go, draws back sideways
and goes back to where it started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=crack_an_egg_into_a_bowl_with_both_hands.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Crack the egg into the white bowl: tap its middle on the bowl's rim to crack it, then pull the two halves of the shell apart over the bowl so the yolk and the white drop in. Put both shell halves in the steel scraps tray, none in the bowl, and take both arms back to where they started." \
  --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/egg-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/egg-dream/*.json; do inspect-robots video "$log"; done
```

Set `EGG_DEBUG=1` to print what the score is built from twice a second: where the egg's middle is, how far apart the
halves are, whether and when it cracked and opened, where the yolk is, whether it and how many drops of white are in the
bowl, how many shell halves are in the tray and whether one is in the bowl, whether both arms are back, where each TCP
is and each hand's finger joint.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 700 steps (126 s on the clock, with Dream answering in 1.04 s on average), and about 3 minutes for the scripted demo (498 steps, 24.9 s of simulated time, 162 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
