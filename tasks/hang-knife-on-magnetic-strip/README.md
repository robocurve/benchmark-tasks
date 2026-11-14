# Put a chef's knife back on the magnetic strip, blade flat

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [hang_knife_on_magnetic_strip.py](hang_knife_on_magnetic_strip.py), with the helper [trialkit.py](trialkit.py) next to it.

A restaurant prep station. A chef's knife lies flat on the cutting board; a magnetic knife strip, already holding two knives, runs along the tiled backsplash. A magnetic strip only holds a blade that lies flat against it, so the knife has to be lifted by its handle, rolled a quarter turn so the blade stands with its face toward the wall, and pressed flat onto the strip with the handle clear of it: laid along the strip with the handle beyond its end, or hung upright with the handle above or below it. A blade that meets the strip by its edge, or a handle pressed against the strip (which holds the blade off it), does not hold, and the knife falls.

## How it is modelled

- **Knife:** a 12 cm handle (2.4 x 1.8 cm) with a 20 cm, 2 mm steel blade, 200 g, lying flat on the board.
- **Magnet:** when the blade face is within 4 mm of the strip, flat against it (its face within about 8 degrees of the wall) and at least 4 cm2 of it lies over the strip's face, the task file joins the knife to the strip at its pose of that moment, as the magnet would hold it. Nothing else holds the knife. The area is the blade's outline seen face-on, clipped to the strip's face, so it works for a knife at any angle: about 43 cm2 laid along the strip, about 16 cm2 hung straight upright across it and more when it leans.

## Scoring

`score()` runs every control step and the trial is scored at the end:

- **0.25** once the knife has been lifted 5 cm off the board.
- **0.5** when the strip holds it.
- **1.0** when it hangs still on the strip with at least 12 cm2 of blade over the strip's face, laid along the strip or hung upright.
- **0** if the knife is never lifted, or is dropped: falling faster than 0.6 m/s (about a 2 cm fall) while neither a closed gripper within 6 cm of the handle nor the strip holds it. Setting the knife down on the board or counter and picking it up again is allowed.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=hang_knife_on_magnetic_strip.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the knife back on the knife rack." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/knife-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted`. The left arm grips the handle from above, lifts the knife and turns it with a quarter turn of its gripper so the knife stands upright, blade up, its face toward the wall, like the two knives already on the strip. It carries the knife to the strip with the handle below it, presses the blade flat across the strip, lets go and backs away. `KNIFE_DEBUG=1` prints the knife's pose, the blade's gap to the strip and the hold state.

## Checks

Scripted with `KNIFE_VARIANT`:

| Run | Result | Score |
|---|---|---|
| Demo (`upright`): hung upright, blade up across the strip, handle below it | held, 16 cm2 over the strip | 1.0 |
| `along`: laid along the strip, handle beyond its end | held, 43 cm2 over the strip | 1.0 |
| `tip`: laid along the strip with only the blade's tip over its end | held, 4.5 cm2 over the strip | 0.5 |
| `half_roll`: rolled about 60 degrees, blade meets the strip at an angle | not held, falls | 0 |
| `handle_on_strip`: laid along the strip with the handle pressed against it | the handle props the blade off; not held, falls | 0 |
| `drop`: let go above the board | dropped | 0 |
| `leave`: nothing done | never lifted | 0 |

## Real trials

A standard 40 cm magnetic knife strip screwed to a wall panel above the counter, and a 20 cm chef's knife. Score by photo: whether the knife hangs on the strip and how much of the blade lies over the strip's face (tape measure: the blade's length over the strip times the strip's height it covers). Handle the knife only by its handle; an operator stays clear of the blade's path.

## Runtime and seeds

- Demo: about 330 steps (16.5 s simulated).
- Dream-1: 1,200 steps (1 minute simulated).
- The layout is fixed; `object_jitter=0` keeps every position as listed.

## Licence and third-party material

This task is released under the MIT licence, including [trialkit.py](trialkit.py), a helper written for these tasks (compound parts, joints between objects and tilted gripper waypoints).

No third-party models, textures or code are used: every object is built from boxes and cylinders declared in the task file.
