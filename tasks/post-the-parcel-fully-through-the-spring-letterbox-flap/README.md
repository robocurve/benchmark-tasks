# Post a packet fully through a drop box's flap slot

A locked drop box stands on the table. Its front is a 20 mm panel with a slot and a brass-coloured flap that swings
inward. The robot posts a cardboard packet through the slot so that it drops inside and the flap falls shut behind it.

The gripper's fingers are about 26 mm wide (front to back) at the pads but about 68 mm wide higher up, so the hand
cannot enter the 60 mm slot: it stops with its centre about 34 mm in front of the panel. How far the packet can be
pushed in therefore depends on where it is held:

- held at its trailing end (the demo), the packet goes in far enough that its centre passes the slot's inner edge;
  released, it tips in, slides off the slot floor and drops onto the box floor, and the flap closes (1.0);
- held about 50 mm from its trailing end, it stops with its centre over the slot floor; released, it stays there,
  lying in the slot with its end sticking out (0; measured on the GPU, see below). Pushing that end again with the
  closed fingers can still send it in;
- held near its middle, its centre stays in front of the panel; released, it falls back out (0; from the geometry,
  not run on the GPU).

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment
for Isaac Lab-Arena), on the `station` scene. It uses `parts.py` (in this folder) for the multi-shape box with its
hinged flap and for the packet's friction. `near_misses.py` lists near-successes and wrong outcomes with the score
each must get; they are plain-Python checks of `score()` and `success()` and need no simulator. (In the code the
box is the object `letterbox`.)

## Building the flap

A solid brass plate of 210 x 66 x 3 mm would weigh about 350 g, seven times the simulated flap. The flap is built
light instead: cut a 210 x 66 mm piece of 3 mm cast acrylic sheet (about 1.19 g/cm³, so about 49.5 g) and cover its
outer face with brass-effect self-adhesive film (about 1.5 g more, so about 51 g). Hang it on two short pins through
its top corners into the box's side walls, so the hinge adds almost no mass to the flap, on a hinge line 4 mm above
the slot ceiling; it covers the slot and rests against the panel's outer face when shut. Weigh the flap with anything
fixed to it: it should be 50 g within 2 g (sand the back face if heavy; acrylic sheet varies by about 10% in
thickness). A 2 g difference changes the flap's weight torque by 4%. In `task.py` it is one uniform 50 g plate
(`FLAP`), so its weight acts at its middle, as the acrylic's does.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Post the brown packet into the drop box."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/dropbox-dream

# Scripted demo (reaches 1.0 in about 12 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/dropbox-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 30 s of simulated time (600 steps at 20 Hz); about 1 minute of wall time on an
  L40S once Isaac Sim has started.
- **Seeds:** none needed. Runs use `-E object_jitter=0`, so every object starts exactly where `task.py` puts it.
  With jitter on, seeded resets shift only the packet.

## How the simulator scores it

- "Inside" uses the packet's world-aligned bounding box, which is never smaller than the packet: its near side must
  be at or behind the panel's inner face (x 0.380 m), its far side in front of the back wall, its sides between the
  side walls and its top under the roof. A tilted packet leaning on the inside of the panel counts if its box
  clears the inner face.
- "Flap shut" is the hinge angle: lower edge at most 5 mm inside the face, i.e. at most asin(5/66) = 0.076 rad.
- "At rest" is moving under 1 cm/s and turning under 0.5 rad/s. Every limit is compared with a 1e-6 margin.
- The scoring is pass/fail: a packet stuck in the slot scores 0, so holding one pressed on the slot floor earns
  nothing.
- The top camera cannot see the box floor: from its height (430 mm) the 260 mm front panel hides everything inside.
  A photo for scoring is taken with the roof lifted.

## Scripted demo

`demo()` drives the left arm only. It grips the packet's trailing end from above (pads flush with the end, tips 3
mm above the table), lifts it to the slot's height (aimed 4 mm high to cancel the arm's measured sag while
carrying), lines it up 10 mm short of the flap, pushes it in level until the hand is 2 mm short of the panel,
opens the fingers and backs off. The packet's leading end opens the flap. On release its centre is about 7 mm past
the slot's inner edge; it tips in and drops onto the box floor, and the flap swings shut.

## GPU traces while building

- Slot 40 mm tall: the packet tipped and jammed diagonally between the slot ceiling's outer edge and the floor's
  inner edge (about 29 degrees). The slot was made 60 mm tall and the friction set to cardboard on painted wood.
- Slot floor 90 mm up: the packet came to rest as a ramp from the slot floor down to the box floor. The slot floor
  was raised to 140 mm and a smooth floor board added; the packet then slides off the slot and lies flat.
- Held 47 mm from its trailing end and released 7.5 mm above the slot floor: it stayed lying in the slot.
- Flap springs of 0.075 and 0.05 N m per rad (with the 50 g flap's weight, about 1.3 to 1.6 N on the packet's top):
  the flap pinned the packet's trailing end and it hung tilted about 48 degrees in the slot, so the demo did not
  reach 1.0. The task keeps 0.005 N m per rad.
