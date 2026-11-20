# Take a cabinet door off its loose-pin hinges and lay it flat

A small wall cabinet hangs on a plywood wall panel at the right of the bench, its bottom 25 mm above the bench top. Its
door (30 x 20 cm, 18 mm painted pine, 0.55 kg) stands open 90 degrees on two 2 in. (51 mm) steel loose-pin butt hinges.
The bottom hinge's pin has already been taken out and lies in a steel parts dish; the top hinge's ball-tipped pin is
still in, so the door hangs on that one hinge. Two 1x2 pine battens lie flat on the bench in front of the cabinet. The
instruction asks for the door to be taken off its hinges and laid flat across the two battens, with the hinge pin in the
dish; it does not say how.

The procedure the task checks is the usual one for taking a door off loose-pin hinges ([Residence Supply, "How to
Remove a Door from Its Hinges"][rs]): "Working from the bottom up keeps the top hinge carrying the load until the very
end." "Remove the top pin last." "With the bottom (and middle) pins out, hold the door with one hand while tapping the
top pin free." Then the door is lifted off and laid down ([Hunker][hunker]: "Lay the door on a flat surface").
Laying it on two battens (or sawhorses) keeps the hinge leaves off the bench and leaves room to get the hands out from
under it.

Why it takes two hands, visible in the scene: with one pin left the door hangs on a single hinge, its weight 15 cm out
from the hinge axis, so that hinge's pin carries the door's turning load. Pulled with nobody holding the door, the pin
binds in the knuckles under that load and the ball tip slips out of the fingers. One hand has to take the door's weight
at its top edge, enough to free the pin, while the other draws the pin straight up out of the five knuckles (51 mm) by
its 11 mm ball tip. Once the pin is out, the door's knuckles only sit on the frame's knuckles: a door that is let go
swings down off level on them and drops onto the bench. The hands then lift the door clear of the knuckles together, stand its bottom
edge on the far batten, lower it face down onto both battens (turning their grips with it) and draw their fingers out
from under its edge.

The task is the task file [`lift_a_cabinet_door_off_its_hinge_pins.py`](lift_a_cabinet_door_off_its_hinge_pins.py) for
the `yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it. No
third-party assets are used: every body is built in the task file from boxes, cylinders and spheres.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `0059d02`) on Isaac Lab-Arena
0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable, installs
everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres, upright, and cannot join them. The cabinet (carcass,
wall panel, the frame leaves and knuckles of both hinges), the door (with its leaves and knuckles), the pins, the dish
and the battens are bodies built from several shapes, so `bodykit.py` adds such bodies: importing it hooks
`yam_arena.scene` when the simulator starts, without changing yam_arena, and gives `score()` each body's pose.

What is modelled and what is not: all parts are rigid and at true size. Each hinge has five knuckles, 8 mm across and
10 mm long; knuckles 1, 3 and 5 belong to the cabinet and 2 and 4 to the door. The bores are 4.4 mm square round a pin
whose shank is 3.8 mm square (shown round), 0.3 mm clearance a side, so that the shank bears on flat faces; steel on
steel and steel on the painted door are given a friction coefficient of about 0.4. Nothing else holds the door: no
joint, no hidden latch. Its weight hangs on the top hinge through the pin's contact with the knuckles, so the binding of
a loaded pin and the door's drop once the pin is out both come from those contacts. The hinge screws are not modelled
(the leaves are drawn on the door's edge and the cabinet's front edge, and the knuckles are part of the door and of the
cabinet). The cabinet is fixed; the battens are taped down.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run.

- 0 for the rest of the run once the door falls: its centre moves down faster than 0.30 m/s at any moment, or its
  lowest point hits the bench top moving down faster than 0.10 m/s. A door lowered by hand moves far slower and never
  touches the bench (it goes onto the battens); a door let go of once the pin is out swings down off level on its
  knuckles and its free corner hits the bench at about 0.3 m/s.
- Otherwise the best stage reached: 0.25 the top pin out of the hinge (its lower end above the top knuckle); 0.5 the door
  also off the hinges (its knuckles 15 mm or more from the hinge axis); 0.75 the door also lying flat across both
  battens (its face within 5 degrees of level, its underside within 3 mm of the battens' tops, overlapping each batten
  by 15 mm or more).
- 1.0: the door lying flat across both battens, both hinge pins in the dish, every finger point 2 cm or more from the
  door, and the door still (it moves under 1 mm in 1 s), held for 1 s.

Doing nothing scores 0. `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial
starts from the same layout. On a real cabinet the same checks are made from the overhead and wrist videos and a ruler:
the pin out and in the dish, the door off the knuckles, the door lying flat on both battens with no hand on it; a fall is
the door dropping onto the bench, or its centre dropping more than 10 mm between two frames of 30 fps video.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=lift_a_cabinet_door_off_its_hinge_pins.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Take the cabinet door off its hinges and lay it flat across the two battens, with the hinge pin in the dish." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cabinet-door-dream
```

The scripted demo, which reaches full score (the left gripper takes the door's top edge 21 cm from its hinge edge and
is raised 6 mm to take the door's weight off the pin; the right gripper, pitched 20 degrees, pinches the pin's ball tip across the hinge
corner, draws the pin 58 mm straight up, carries it to the dish and drops it in, then takes the door's top edge by the
hinge; together they lift the door 20 mm, draw it 16 mm off the hinge axis, carry it out over the far batten, stand its
bottom edge on that batten and lower it face down onto both battens in 7 s, turning their grips with it; they let go,
draw their fingers 4 cm out along the door, rise and close them):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=lift_a_cabinet_door_off_its_hinge_pins.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Take the cabinet door off its hinges and lay it flat across the two battens, with the hinge pin in the dish." \
  --scorer task_score --max-steps 1300 --store-frames --log-dir ~/yam-eval/cabinet-door-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/cabinet-door-dream/*.json; do inspect-robots video "$log"; done
```

`DOOR_DEBUG=1` prints, every 5 steps (`DOOR_DEBUG_EVERY=10` for every 10), the door centre's vertical speed, how far the
pin has risen, how far the door's knuckles are from the hinge axis, where the door is and how it is tilted, its lowest
point, whether it lies flat on the battens, how many pins are in the dish, how close the nearest finger is, where each
gripper is and how far open, and the stage reached.

Two wrong ways, each scoring 0 for a physical reason:

- `DOOR_MODE=nohold`: the left arm stays back and the right gripper pulls the pin with nobody holding the door. The pin
  binds in the knuckles under the door's weight, the ball tip slips out of the fingers and the pin never comes out: 0.
- `DOOR_MODE=letgo`: the demo until the pin is in the dish, then the left gripper lets go of the door before anyone has
  taken it off the knuckles. With no pin and no hand, the door swings down off level on its knuckles and its free corner
  hits the bench at 0.28 m/s: it fell, 0.

## Runtime per trial

On one RTX PRO 6000 (Blackwell), on a shared machine: about 5.5 minutes for the scripted demo while rendering 1280 x 720
video (1,245 steps, 62 s of simulated time), about 3.5 minutes for it at 320 x 176 without video, and about 11 minutes
for a Dream-1 trial of 2,400 steps (with some waiting on the policy server). Isaac Sim's first start on a new machine adds a few minutes while it compiles its
shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Building it for real

A small wall cabinet with a 30 x 20 cm door hung on two 2 in. loose-pin butt hinges with ball-tip pins (any hardware
store; the bottom pin taken out beforehand), screwed to a plywood panel clamped upright at the back of the bench so that
the cabinet's bottom is 25 mm above the bench; a small steel parts dish; two 1x2 battens taped to the bench 14 cm apart.
To reset, put the door back on its hinges, drop the top pin in, take the bottom pin out and put it in the dish.

[rs]: https://residencesupply.com/blogs/news/how-to-remove-a-door-from-its-hinges
[hunker]: https://www.hunker.com/13418637/how-to-take-a-door-off-its-hinges
