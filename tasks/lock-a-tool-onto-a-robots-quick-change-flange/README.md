# Fit the blue suction-cup tool to a robot's manual quick-change flange and lock it on

A robot arm, switched off for a tool change, reaches its wrist in low over the table, its tool flange facing the right
arm. It has just parked its gripper tool in the aluminium tool rack beside it; in the rack's other slot waits a blue suction-cup
tool. One hand takes the suction tool by its handle and holds its plate flat against the flange; the other presses the
locking ring's lever down, turning the ring until it locks the tool on and the light on the wrist turns from red to
green. Then both hands let go, the tool hanging on the robot, and both arms go back to where they started. The task is
one task file, [`lock_a_tool_onto_a_robots_quick_change_flange.py`](lock_a_tool_onto_a_robots_quick_change_flange.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests: holding a part in place against gravity with one hand while the other works a lock; picking the right
tool; finishing with both hands free.

Why two hands: nothing holds the tool up on the flange until it is locked, and the ring locks only a tool that is
already seated as it turns. With one hand (the right taking the tool and holding it on the flange as in the demo, the
left never moving) the demo scored 0.25: the tool sat on the flange with the ring open, and fell to the table as soon as
the hand let go.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The robot arm and the tool rack are kinematic, fixed
where they are put; the ring with its lever and the two tools are loose rigid bodies made of shapes from `bodykit.py`.

- The ring: a hinge about the flange's axis (a `Hinge` from `bodykit.py`), turned the locking way by pressing its lever
  down, up to 35 degrees. A spring holds it where it last stopped; after every control step, a ring pushed more than 2
  degrees past its hold has given, and the task file moves the hold there, so the ring stays where it is left.
- The lock: after every control step the task file checks each tool against the flange (its plate's face within 6 mm
  of the flange's face, its middle within 1 cm of the axis, its axis within 12 degrees, its handle up within 20
  degrees). A tool seated while the ring is within 8 degrees of open is locked on once the ring has turned 25 degrees:
  a fixed joint (a `Glue` from `bodykit.py`, to a hidden fixed body under the table) switches on and holds the tool
  square on the flange, and the green light is moved over the red one. Turning the ring back below 15 degrees switches
  the joint off.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Building it for a real trial

The changer is of the manual locking-ring type used on small robots:

- Robot side: a 64 mm steel flange on an ISO 9409-1-50-4-M6 wrist mount, with two 6 mm locating dowels in its face,
  and a steel locking ring 88 mm across and 18 mm deep turning on it behind the face. The ring carries three internal
  cam lugs at 120 degrees and the orange lever (11 cm toward the operator, then 6.3 cm along the flange's axis to a
  45 x 30 mm rubber pad). A friction detent (about 0.1 N m) keeps the ring where it is left; it turns up to 35 degrees.
- Tool side: a 60 mm, 8 mm thick steel plate with three lugs on its rim and two 6 mm holes for the dowels. With the
  plate on the dowels and the ring open, turning the ring 25 degrees hooks its cam lugs over the plate's lugs and draws
  the plate onto the face: the tool is locked and the wrist's light turns green. Turning it back below 15 degrees
  releases it.
- Tools: the blue suction-cup tool (120 g) and the grey two-finger gripper tool (140 g), each with a 16 mm square steel
  handle on top, in an aluminium rack whose 1 cm lips hold them lying with their plates toward the flange.

The simulation keeps the geometry and the lever operation but stands in for the dowels and cam lugs with a pose check
and a fixed joint (see `task.json`'s fidelity).

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the suction tool still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`).
`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before
sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=lock_a_tool_onto_a_robots_quick_change_flange.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Fit the blue suction-cup tool to the robot's tool flange and lock it on, leaving the gripper tool in the rack. Then let go of everything, with the tool hanging locked on the robot, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/flange-dream
```

The scripted demo, which reaches full score: the right hand comes straight down on the suction tool's handle, fingers
closing across it, lifts it out of the rack, carries it to the flange and holds its plate flat against the flange. The
left hand comes straight down on the lever's pad, fingers closed, and presses it down along its swing, 32 degrees: the
ring turns and locks the tool on, and the light turns green. The right lets go and draws back, the left rises, the tool
hanging on the robot, and both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=lock_a_tool_onto_a_robots_quick_change_flange.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Fit the blue suction-cup tool to the robot's tool flange and lock it on, leaving the gripper tool in the rack. Then let go of everything, with the tool hanging locked on the robot, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/flange-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/flange-dream/*.json; do inspect-robots video "$log"; done
```

Set `FLANGE_DEBUG=1` to print what the score is built from twice a second: how far the ring has turned, how far the
suction tool is from sitting on the flange (gap, offset, tilt, turn), whether it is seated with the ring open and
whether it is locked, whether the gripper tool is locked, whether both hands are clear, and where each TCP is from the
tool's handle and the lever's pad.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,200 steps (180 s on the clock, with Dream answering in 0.69 s on average), and about 1 minute for the scripted demo (394 steps, 19.7 s of simulated time, 69 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
