# Squeeze a line of ketchup along a hot dog from an upside-down bottle, none on the bun, then stand the bottle back

A hot dog in its bun lies on a paper plate; beside it an upside-down ketchup squeeze bottle stands on its cap, the
flip lid open. The instruction asks for a line of ketchup along the sausage from end to end, kept on the sausage, and
the bottle stood back on its cap where it was. It does not say how.

What makes it hard is the hand skill. The full bottle (0.43 kg) is held by its soft walls, and the same walls are what
push the ketchup out: the hand has to hold the bottle firmly enough to carry it (about 4 N on the walls) without
opening the valve (5 N), then keep a steady squeeze (about 9.5 N) while it carries the nozzle 16 cm along the sausage
at a speed that matches the flow, 1 to 2 cm above the sausage and within 6 mm of its crest. Stopping, or going too
slowly, pools ketchup that runs off the sausage onto the bun; going too fast leaves gaps; drifting sideways runs the
line down the side of the sausage into the bun. Then the squeeze has to be let off before the far end and the bottle
set back on its narrow cap and let go. One arm does it; the other is not needed.

The task is the task file [`squeeze_ketchup_along_a_hot_dog.py`](squeeze_ketchup_along_a_hot_dog.py) for the
`yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Why it matters

Dressing hot dogs, burgers and sandwiches from a squeeze bottle is everyday work on fast-food lines and at concession
stands and hot-dog carts. O*NET lists it under the core tasks of the largest physical occupation in the US:
[35-3023.00 Fast Food and Counter Workers][onet1] (3.9 million): "Prepare daily food items, and cook simple foods and
beverages, such as sandwiches, salads, soups, pizza, or coffee, using proper safety precautions and sanitary
measures"; and [35-2011.00 Cooks, Fast Food][onet2]: "Prepare specialty foods, such as pizzas, fish and chips,
sandwiches, or tacos, following specific methods that usually require short preparation time." The National Hot Dog
and Sausage Council's etiquette ([reported by Food Republic][nhdsc]): "Always 'dress the dog,' not the bun." Most
people have also done it themselves at a barbecue or a ball game.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on
Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable, installs everything, including the
Dream-1 policy. The task needs no models from Arena's object library and uses no third-party assets: the bottle, the
hot dog and the plate are built in these files from measurements.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them. The bottle (a body with a cap
and two sprung walls on slide joints), the hot dog (bun and sausage as one body) and the plate are built with
`bodykit.py`, which adds such bodies and joints: importing it hooks `yam_arena.scene` when the simulator starts,
without changing yam_arena, and gives the task file each body's pose and each wall's displacement. The ketchup marks
are visual-only bodies that the task file lays on the food as ketchup lands (it moves only these marks, never a body
the robot handles).

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's
left, z up from the table top.

- Paper plate, loose: white, 9 inches (23 cm) across, 2 mm thick, 12 g; centred at x 19.0, y -6.0.
- Hot dog in its bun, loose, one rigid body of 0.11 kg on the plate, centred at x 19.0, y -6.0, its long axis along x:
  the frank 16 cm long and 2.2 cm across, its crest 4.0 cm above the plate, standing out 0.5 cm beyond the bun at
  each end; a split-top bun 15 cm long and 8.2 cm wide, its two halves 3.2 cm thick either side of the frank, their tops
  3.6 cm above the plate.
- Ketchup bottle, loose: a 14 oz (397 g) upside-down squeeze bottle (like Heinz Tomato Ketchup 14 oz upside-down),
  full, 0.43 kg. Body 7.6 cm (along x) by 4.8 cm (along y), 17.5 cm tall from the nozzle to the bottle's base. It
  stands on its cap (3.6 cm across, 2.8 cm tall, the valve's slit in the middle of its flat end) at x 17.0, y -21.5,
  the flip lid open. Its two wide faces (facing +y and -y) are the soft walls: each gives 2 N per mm pressed in (up
  to 8 mm).
- Both arms start at the ready pose, grippers open. The layout is the same in every trial.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run. Every
control step the task file reads the squeeze on the bottle (the mean of the two walls' spring forces, from how far the
fingers have pressed each wall in), the bottle's pose and where its nozzle is, the poses of the hot dog and the plate,
and the points of each finger's collision hulls.

Ketchup (a deposition rule, as accepted tasks model liquids with beads or rules; ketchup is not simulated as a
fluid):

- It flows only while the bottle is within 60 deg of tip-down and the squeeze is over 5 N (the valve opening), at
  0.15 ml/s for each newton above 5 N (0.68 ml/s at 9.5 N), and falls straight down from the nozzle to the first
  surface under it, in the hot dog's frame (the hot dog is loose).
- The frank is split into 16 cells of 1 cm. Ketchup landing on the frank within 6 mm of its crest goes into that cell.
  A cell holds at most 0.6 ml (the ridge a line can stand up to); more runs off onto the bun and is off target. A cell
  holding 0.12 ml (a bead about 4 mm wide along that centimetre) is covered.
- Ketchup landing on the frank's side (6 to 11 mm off its crest; it runs down into the bun), on the bun, the plate or
  the table is off target. 0.1 ml off target counts as a 0.4 cm^2 spot (2.5 mm thick).

The score:

- 0 if more than 2.0 cm^2 of ketchup is off target (0.5 ml).
- Otherwise 0.8 x min(1, L / 14 cm), where L is the longest unbroken run of covered cells, in cm.
- 1.0 (success, held 1 s): L at least 14 cm (of the frank's 16), at most 0.5 cm^2 off target, the bottle standing on
  its cap (within 15 deg of tip-down, the cap within 1 cm of the table), let go (squeeze under 1 N, every finger point
  2 cm or more from it) and still (moving less than 1 mm over 1 s), and the hot dog still on the plate (its middle
  within 6 cm of the plate's middle).

Doing nothing scores 0. A real trial is scored from an overhead photo taken at the end with a ruler laid beside the
hot dog: L is the longest length along the frank over which its top carries an unbroken line of ketchup at least 4 mm
wide; each spot of ketchup off the frank's top (on the bun, the plate, the paper) is measured length x width and the
areas added; the bottle standing on its cap, let go, is checked on the video.

The ketchup is shown in the cameras by red marks (visual only; nothing collides with them): one per covered cell,
laid along the frank where the ketchup landed, and one spot for each 0.1 ml off target, where it landed (or, for
ketchup that ran off a full cell, on the bun beside that cell).

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=squeeze_ketchup_along_a_hot_dog.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Put a line of ketchup along the sausage of the hot dog from end to end, keeping it on the sausage, then stand the ketchup bottle back on its cap where it was." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/ketchup-dream
```

The scripted demo, which reaches full score (open loop from the known layout; the left arm stays at the ready pose).
The right gripper comes straight down over the bottle, fingers closing across its 4.8 cm (along y), and holds it
lightly: 3.8 N on the walls, below the valve's 5 N. It lifts the bottle, carries it over the near end of the frank and
lowers the nozzle to about 1.3 cm above the crest (aimed 2.2 cm above; the arm sags under the load). It closes the
fingers further (the squeeze rises to 9.6 N), carries the bottle along the frank at 3 cm/s for 15 cm, lets the
squeeze off over the last centimetre, lifts, carries the bottle back, stands it on its cap, lets go and moves clear.
It lays 3.5 ml of ketchup: 15 cells covered in one unbroken run (0.21 to 0.26 ml each), none off target.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=squeeze_ketchup_along_a_hot_dog.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Put a line of ketchup along the sausage of the hot dog from end to end, keeping it on the sausage, then stand the ketchup bottle back on its cap where it was." \
  --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/ketchup-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/ketchup-dream/*.json; do inspect-robots video "$log"; done
```

`KETCHUP_DEBUG=1` prints, every 5 steps (`KETCHUP_DEBUG_EVERY=1` for every step), the squeeze and its peak, the
bottle's tilt from tip-down, the ketchup let out and off target (ml and cm^2), the longest covered run, the ketchup in
each cell, where ketchup would land and how far off the crest, the nozzle's height above that surface, where the
bottle is, how close the nearest finger is, whether the bottle stands, is still and the hot dog is on its plate, and
where each gripper is and how far open.

`KETCHUP_MODE=pool` runs the demo the wrong way: the same grip and height, but it squeezes for 6 s over the middle of
the frank without moving. That cell fills to its 0.6 ml and the rest (3.7 ml) runs off onto the bun: 15 cm^2 off
target. Score 0.

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell shared with other simulation jobs, a debug run of the scripted demo at 320 x 176
without video (800 steps) took about 2.5 minutes after Isaac Sim had started.
The scripted demo while rendering 1280 x 720 video took about 2.5 minutes from start to video (660 steps, ending on
success); a Dream-1 trial of 2,400 steps at 320 x 176 took about 12 minutes (0.29 s per step). Isaac Sim's first
start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A 14 oz upside-down ketchup squeeze bottle (Heinz Tomato Ketchup 14 oz or similar, about 3 USD), kept full by
refilling it to a mark from a larger container; a pack of all-beef franks (16 cm) and split-top buns, at room
temperature (20 to 24 C), a fresh frank and bun each trial; white 9-inch paper plates; paper to cover the table; a
30 cm ruler for the photo. Before each trial wipe the bottle's outside and nozzle, squeeze a short test line onto a
waste plate so the ketchup sits at the valve, and stand the bottle on its cap at the marked spot with the lid open.
The grippers touch only the bottle's dry outside.

[onet1]: https://www.onetonline.org/link/summary/35-3023.00
[onet2]: https://www.onetonline.org/link/summary/35-2011.00
[nhdsc]: https://www.foodrepublic.com/1828140/hot-dog-condiment-hack/
