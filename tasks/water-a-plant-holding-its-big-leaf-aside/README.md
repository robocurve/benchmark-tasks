# Water a house plant at its roots without wetting its leaves

A house plant in a terracotta pot spreads its broad leaves low over the soil: three in the crown over the back and
sides, and one big leaf in front on a springy stalk, lying over the front of the soil. A blue watering jug with water
in it stands on a stool beside the pot. One hand pushes the big leaf round on its stalk, aside, and holds it there;
the other lifts the jug, carries it over the front of the pot and pours the water onto the soil the leaf covered,
without wetting any leaf. The jug goes back on its stool, the leaf is let go back over the soil, and both arms go back
to where they started. The task is one task file,
[`water_a_plant_holding_its_big_leaf_aside.py`](water_a_plant_holding_its_big_leaf_aside.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

What it tests: holding a sprung part out of the way while the other hand works where it was; pouring a light stream
into a target without spilling it on what is around it; putting everything back.

Why two hands: the big leaf springs back over the soil as soon as nothing holds it, and the soil it covers is the only
soil not under a leaf, so water poured with the leaf in place lands on the leaf. With one hand (the left pouring as in
the demo, the right never moving) the demo scored 0: all 12 beads landed on the big leaf, which lay over the
soil, and ran off it.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres; the bodies here are made of several shapes each, from
`bodykit.py`.

- The pot, its soil, the stem and the crown's three leaves are one fixed body. Each leaf is a thin rigid blade (seven
  boxes giving its oval outline) 15 cm long.
- The big front leaf is a loose rigid blade joined to the stem by a hinge about the vertical (`Hinge` from
  `bodykit.py`), its spring (0.0016 N m per degree) holding it over the front of the soil; pushed round 85 degrees it
  pushes back with 0.14 N m.
- The water is 12 rigid beads in the jug, which is a cup built from a floor and 16 wall panels.
- After every control step the task file checks each bead: within 4 mm of any leaf's blade, it has wetted that leaf;
  lying on the soil inside the pot without having touched a leaf, it has watered the soil.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with the jug, the big leaf and every bead still) has held
for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0`
keeps every object exactly where the task file puts it, so every trial starts from the same layout. The cameras render
at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=water_a_plant_holding_its_big_leaf_aside.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Water the plant at its roots, onto the soil, without wetting its leaves. Then put the jug back on its stool, leave the plant's leaves as they were and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/plant-dream
```

The scripted demo, which reaches full score: the right hand comes down beside the big leaf's left edge, fingers
closed, pushes the leaf round on its stalk 85 degrees, until it points to the right, clear of the front of the soil,
and holds it there. The left comes in from the side on the jug, fingers closing across it low down, lifts it, carries
it over the front of the pot and tips it out onto the soil the leaf covered. It sets the jug back on its stool, lets
go and goes back to where it started; the right lets the leaf go back round over the soil, rises and goes back to
where it started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=water_a_plant_holding_its_big_leaf_aside.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Water the plant at its roots, onto the soil, without wetting its leaves. Then put the jug back on its stool, leave the plant's leaves as they were and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/plant-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/plant-dream/*.json; do inspect-robots video "$log"; done
```

Set `PLANT_DEBUG=1` to print what the score is built from twice a second: how far round the big leaf is, how many
beads lie on the soil and how many have touched a leaf, how many are still in the jug, where the jug is and how far
it is tipped, whether the jug and the leaf are back, whether both hands are clear, and where each TCP is.

## A real trial

A peace lily (Spathiphyllum, about 25 cm tall) in a 17 cm terracotta pot on a saucer, soil just moist, three leaves
over the back and sides of the pot and one big lower leaf on its flexible stalk lying over the front of the soil; a
150 ml watering jug holding 120 ml of room-temperature water on a 6 cm stool beside it. Weigh the potted plant with
its saucer before and after the trial (1 g per ml) and blot every leaf with a pre-weighed paper towel: water on the
soil is the weight gain minus the blotted water. Full credit needs at least 90 ml on the soil and at most 10 ml on the
leaves, as the simulation's 9 and 1 of its 12 beads (10 ml each); 0.25 needs 40 ml on the soil. Reset by letting the
pot drain back to its starting weight, drying the leaves and refilling the jug.

## Runtime per trial

On one RTX 5090: about 6 minutes for a Dream-1 trial of 1,200 steps (367 s on the clock, with Dream answering in 0.59 s on average), and about 4 minutes for the scripted demo (607 steps, 30.4 s of simulated time, 234 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
