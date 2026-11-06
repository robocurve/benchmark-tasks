# Hold a nut behind a plate with one hand and turn a screw into it through the plate with the other

A steel plate stands upright between the two arms with a hole through it. One hand takes a long hex nut off its
stand and holds it against the plate over the hole; the other takes a hex-head screw out of its holder, puts it
through the hole from the other side and turns it into the nut until head and nut are snug against the plate, then
both let go. The task is one task file,
[`screw_into_a_nut_held_behind_a_plate.py`](screw_into_a_nut_held_behind_a_plate.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

Why two hands: the thread drags. Once it has caught, turning the screw turns a nut that nobody holds along with it,
so the screw does not go in; the nut has to be held still on the far side of the plate while the screw turns.
The same demo with the left hand letting go of the nut as soon as
the thread has caught shows it: the nut turns with the screw, the next four strokes (two turns of the hand) take the screw only
0.4 turns further in, head and nut stay 6 mm off the plate, and the trial scores 0.5 instead of 1.0.

## How the thread is modelled

PhysX has no screw thread, and the shank is a plain cylinder in the nut's plain hexagonal bore. The task file follows
the screw's turn against the nut after every control step and models the thread with a joint between them
(a `Thread` from `bodykit.py`, a PhysX D6 joint authored switched off):

- It catches when the screw's tip is at least 1 mm into the nut's bore, the two are lined up (axes within 10 degrees
  and 4 mm; the bore widens to 13 mm at each end as a lead-in), and the screw turns a quarter turn clockwise, seen from
  its head. The task file then switches the joint on.
- While it is on, the nut stays on the screw's axis, and turning the two against each other meets a friction torque of
  0.15 N m for each radian per second (so an unheld nut turns along). For every turn of the screw in the nut the task
  file moves the joint's slide target 5 mm toward the head, and a spring (2,000 N/m, at most 20 N) draws the nut along:
  turned in, head and nut close on the plate and then press it. Turned back far enough, the nut comes off the end.
- While it is on, the screw and the nut do not collide. Every reset switches it off again.

`yam_arena` task files place single boxes, cylinders and spheres. The plate (with its hole, foot and printing), the
screw's holder, the nut's stand, the screw and the nut are rigid bodies made of several shapes from `bodykit.py`; the
plate, holder and stand are kinematic, fixed where they are put. Importing `bodykit.py` hooks `yam_arena.scene` when the
simulator starts, without changing yam_arena, gives `score()` each body's own pose, and lets the task file look at the
simulation after every control step, where it follows the thread and where the grippers are.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the screw and the nut still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at
1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=screw_into_a_nut_held_behind_a_plate.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the nut flat against the plate over the hole with one hand, and with the other put the screw through the hole from the other side and turn it into the nut until both are snug against the plate, then let go." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/screw-dream
```

The scripted demo, which reaches full score: both hands come straight down, fingers closing front and back, the left
one round the middle of the nut and the right one on the screw's head. Each lifts its part and turns its gripper to
point at the plate. The left hand holds the nut on the plate's left face over the hole; the right hand pushes the
screw through the hole into the nut and turns it in strokes of half a turn (the wrist turns only so far): turn, let
go, turn back open, take the head again, five times, following the screw in. Then both let go and back away:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=screw_into_a_nut_held_behind_a_plate.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the nut flat against the plate over the hole with one hand, and with the other put the screw through the hole from the other side and turn it into the nut until both are snug against the plate, then let go." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/screw-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/screw-dream/*.json; do inspect-robots video "$log"; done
```

Set `SCREW_DEBUG=1` to print what the score is built from (where the nut is, where each gripper's TCP is from its part,
how far along the screw the nut is and how well they line up, the turn before the catch, the turns since, whether the
screw is through the hole, the play and whether both hands have let go) twice a second.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,600 steps (193 s on the clock, with Dream answering in 0.75 s on average), and about 2 minutes for the scripted demo (658 steps, 32.9 s of simulated time, 95 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
