# Slice a peeled boiled egg in a hinged wire egg slicer, laid lengthwise in its cradle, and swing the frame back open

A hinged wire egg slicer (die-cast aluminium cradle 140 x 80 mm with ten stainless-steel cutting wires on a frame
hinged at one end, like the Westmark "Justus" egg slicer) stands on a non-slip mat with its frame swung open. A peeled
hard-boiled egg lies on a saucer beside it, crosswise to the cradle. The instruction asks for the egg to be sliced in
the slicer and the frame swung back open, the slices left in the slicer.

Slicing a boiled egg in a wire slicer is an everyday kitchen job (egg salad, sandwiches, salads, garnish) that most
people have done or watched. The maker describes the tool as cutting "the hard-boiled egg quickly and cleanly into
round or oval slices without breaking the yolk" ([Westmark Justus][justus]); for round slices the egg lies lengthwise
in the cradle's egg-shaped hollow. The usual way: lay the egg in the hollow, swing the wire frame over and press it
down through the egg in one stroke until it closes on the cradle, then swing the frame open again to free the slices
(they are then taken out or tipped onto a plate).

## The hand skill

- The peeled egg is slick and rounded. It lies across the robot's reach on the saucer and has to be picked up and
  turned a quarter turn, then set lengthwise and centred in a hollow only 6 mm deep. An egg not lying in the hollow
  when the wires reach it is crushed (score 0).
- The frame is taken by its tab (30 x 32 mm, 3 mm thick) and swung over through 95 deg along its arc, then driven
  down through the egg against the wires' cutting resistance, about 9 N at the tab, until it closes on the cradle;
  then it is swung back open past upright onto its stop, without knocking the eleven loose slices out of the
  cradle. The fingertips hold the tab by its two side edges, and the tab turns between them as the frame swings.

Two arms are not required (the demo uses the right arm for the egg and the left for the frame, because each is the one
that reaches its part comfortably).

The task is the task file
[`slice_a_boiled_egg_in_a_wire_egg_slicer.py`](slice_a_boiled_egg_in_a_wire_egg_slicer.py) for the `yam_arena`
embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac
Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable,
installs everything, including the Dream-1 policy. The task needs no models from Arena's object library and uses no
third-party assets: the slicer, the egg and the saucer are built in these files from boxes, cylinders and
triangle meshes computed in the task file.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them or put one on a joint. The
cradle (a fixed body whose top, with the hollow, is a triangle mesh), the wire frame (a linkage with one revolute
joint), the egg's eleven slices (each a convex mesh, joined by fixed joints that can break) and the saucer are built
with `bodykit.py`, which adds such bodies and joints: importing it hooks `yam_arena.scene` when the simulator starts,
without changing yam_arena, and gives the task file each body's pose and each joint's position. This copy of
`bodykit.py` lets a sprung joint cap its drive's force (`effort_limit`). Every control step the task file sets the hinge drive's force limit and target through Isaac Lab's
articulation API and, when the egg is cut, changes the slices' joints through USD, as described below.

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, z up from the table top.

- Egg slicer cradle, fixed (it stands on a non-slip mat): die-cast aluminium, 8 cm (x) by 14 cm (y), 2.2 cm tall,
  centred at x 21, y 4. In its top an egg-shaped hollow (the lower part of the egg, 1 mm clearance, 6 mm deep) at its
  centre, its long axis along y, and slots for the wires across the bed.
- Wire frame: hinged across the cradle's +y end (axis along x, 2.45 cm above the table, at y 10.4). Bars 4 mm square
  round an opening 8.4 cm wide and 12.6 cm long; a tab 3.0 cm wide, 3.2 cm long and 3 mm thick bent up at the free
  end; ten stainless-steel wires across it, 5.5 mm apart. Closed, the wires lie in the cradle's slots 1 mm below the
  hollow's floor, at y 4 + (-2.475, -1.925 ... 2.475) cm. 40 g. It starts swung open 95 deg, just past upright,
  resting on its stop. The hinge turns freely (0.03 N m s/rad of drag).
- Egg: peeled, hard-boiled, US large: 5.7 cm long, 4.3 cm across, 50 g. It lies on the saucer with its long axis
  along x, its centre at x 13, y -10.
- Saucer: 10 cm across, 6 mm thick, centred at x 13, y -10, loose.
- Both arms start at the ready pose, grippers open. The layout is the same in every trial.

What is modelled:

- The egg is eleven slices, cut at the planes where the wires pass through an egg lying centred in the hollow, held
  together by fixed joints (they part only at 400 N) until it is cut. Each slice is the convex hull of its part of the
  egg (an ellipsoid), its cut faces set back 0.15 mm. A slice collides with every other slice except its two
  neighbours (PhysX does not collide a jointed pair).
- Cutting (a stand-in for the wires going through the egg): the wires do not collide with the egg. Once the frame has
  come down so far that a wire over the egg reaches the egg's top, the hinge resists closing with 1.2 N m (a drive
  that holds the frame where the wires have got to, its force capped at 1.2 N m): about 9 N pressed at the tab, 13.5
  cm from the hinge, or 19 N on the wires at the egg's centre. Pressing harder moves the frame on; letting go leaves
  it where it is. When the frame is within 1.5 deg of closed, the egg is cut: the slices' joints become the light
  cling of moist cut faces (they part at 0.3 N or 0.8 mN m), so the slices stand together in the hollow, as a
  sliced egg does.
- Crushing: if the egg is not lying in the hollow when a wire first reaches it (its long axis more than 20 deg off the
  hollow's, its centre more than 6 mm off the hollow's centre, or more than 4 mm above its seat), the wires crush it
  rather than slice it: the score is 0 for the rest of the trial.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the frame's angle, each slice's pose and speed, and the points of each finger's
collision hulls. "In the cradle": a slice's centre over the cradle's 8 x 14 cm top and at most 4.5 cm above the table.

- 0.25: the egg has lain in the hollow, lengthwise and centred (its long axis within 20 deg of the hollow's, its
  centre within 6 mm of the hollow's centre and within 4 mm of its seat), at some time.
- 0.5: the egg has been cut into its 11 slices (the frame closed over the egg lying in the hollow).
- 1.0 (success, held 1 s): as 0.5, with the frame swung back open (85 deg or more), all 11 slices in the cradle, every
  finger point 1 cm or more from every slice, and all of them still (under 2 cm/s).

Once the egg has been crushed (the wires reached it while it was not lying in the hollow) the score is 0 for the rest
of the trial. Doing nothing scores 0. A real trial is scored from the video and a photo of the slicer at the end: the
egg lay lengthwise in the hollow (video); the frame closed fully on the cradle with the egg under it (video); at the
end the frame is open past upright and eleven clean round slices (the two smallest being the end caps) lie in the
cradle, none on the table (photo). A crushed egg does not give eleven clean slices.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=slice_a_boiled_egg_in_a_wire_egg_slicer.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Slice the peeled egg in the egg slicer, then swing the slicer's frame back open, leaving the slices in it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/eggslicer-dream
```

The scripted demo, which reaches full score (open loop from the known layout). The right gripper, straight down with
its fingers across the egg, picks the egg up off the saucer, turns it a quarter turn on the way and sets it in the
hollow, lengthwise, lets go and goes back to its ready pose. The left gripper, straight down with its fingers closing
along x, takes the frame's tab by its two side edges with its fingertips and swings the frame over along its arc (the
tab turns between the fingertips) onto the egg and on to 8 deg past closed, so that the arm presses the wires through;
it holds it closed 1.5 s, swings it back past upright (it stops at 95 deg on its stop), lets go and goes back. It
reaches success at about step 785 (39 s).

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=slice_a_boiled_egg_in_a_wire_egg_slicer.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Slice the peeled egg in the egg slicer, then swing the slicer's frame back open, leaving the slices in it." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/eggslicer-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/eggslicer-dream/*.json; do inspect-robots video "$log"; done
```

`EGG_DEBUG=1` prints, every 5 steps (`EGG_DEBUG_EVERY=1` for every step), the frame's opening angle, whether the hinge
is resisting (the wires in the egg), whether the egg lies in the hollow, whether it has been crushed or cut, the egg's
centre and how far its axis is off the hollow's, how many slices are in the cradle and how far they spread, how close
the nearest finger is to a slice, and where each gripper is and how far it is open; after the cut it also prints each
slice's position every 2 s.

`EGG_CROSS=1` runs the demo the wrong way: the egg is set into the cradle without turning it, so it lies across the
hollow. It cannot settle into the hollow (it rests 2 mm higher, turned 90 deg from it), so when the wires reach it they
crush it: the score is 0 from then on, although the same frame strokes follow.

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell Server Edition shared with other simulation jobs: debug runs of the scripted demo
without video (ending on success at about step 785, 39 s of simulated time) took about 2 minutes from start to finish,
the demo rendered at 1280 x 720 about 2.5 minutes, and a Dream-1 trial of 2,400 steps at 320 x 176 about 6 minutes. Isaac Sim's first start on a new machine adds a
few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A hinged wire egg slicer with an egg-shaped hollow and about ten wires (for example the Westmark "Justus", 140 x 80 x
30 mm, die-cast aluminium; any hinged wire slicer of this kind will do), set on a thin non-slip mat with its hinge
toward the robot's left, the hollow's centre 21 cm in front of the midpoint between the arm bases and 4 cm to the
left; a saucer about 10 cm across at x 13, y -10. Eggs: US
large, hard-boiled (10 minutes in boiling water, then into iced water for 5 minutes), peeled, kept in the fridge (4 C)
and used within the day; one egg per trial, laid on the saucer lengthwise along x just before the trial. The egg is
moist: thin food-safe nitrile covers on the finger pads keep the pads clean; wipe them between trials. The cutting
force can be checked by pressing the frame's tab down on an egg with a kitchen scale under the slicer (about 0.9 kg
shown at the tab, by our estimate). Before each trial clear the slices out, wipe the cradle and swing the frame open
past upright onto its stop.

[justus]: https://www.culinaris.de/en/Westmark-Egg-divider-Justus/31305
