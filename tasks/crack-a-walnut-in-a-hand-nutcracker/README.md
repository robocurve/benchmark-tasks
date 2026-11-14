# Crack a walnut with a hand nutcracker and sort the kernel from the shell

A steel hand nutcracker lies flat on a smooth worktop, its two long levers open in a V toward the arms, with an in-shell
walnut between its toothed jaws; a white bowl waits on the right and a blue saucer on the left. The walnut must be
cracked with the nutcracker, the kernel put in the bowl and every piece of shell on the saucer, and both arms brought
back to where they started. The task is one task file,
[`crack_a_walnut_in_a_hand_nutcracker.py`](crack_a_walnut_in_a_hand_nutcracker.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: two opposing pushes on one free tool, kept in line and built up until the jaws press the walnut with
40 N, so the tool closes instead of sliding away; then sorting a kernel and three pieces of shell into two containers.

Why two hands: the nutcracker's spring holds its jaws open with 0.3-0.7 N m (about 1-3 N on each handle), the walnut
needs 40 N from each jaw (about 13 N on each handle), and the 320 g tool slides on the worktop at about 1 N. Pushed on
one handle only, it just turns and slides on the worktop with the walnut, and the jaws never press it. With one hand
(the right pushing its handle as in the demo, the left never moving) the demo scored 0: the right hand pushed its
handle in, but the whole nutcracker turned on the worktop about its pivot and carried the walnut 1 cm sideways; the
levers stayed fully open and the jaws never pressed the walnut at all.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres; the bodies here are made of several shapes each, from
`bodykit.py`.

- The nutcracker: two loose steel levers (160 g each) joined at the pivot by a hinge (a `Hinge` from `bodykit.py`) with
  a spring (0.04 N m per degree) that holds them open at 30 degrees each side of the middle line, against a stop; they
  close to 20 degrees at most. Each lever runs straight out 20 cm from the pivot and then bends into a 9 cm red handle
  that runs along the middle line, so the two handles are pushed toward each other in line.
- The walnut: one rigid body, 12 g, 36 x 34 x 33 mm: an upright 34 mm cylinder that the jaws touch (unseen), inside a
  lumpy shell that is seen only. After every control step the task file reads the contact force on it from each lever
  (a PhysX rigid contact view, `contact_forces` in `bodykit.py`). Once both reach 40 N for two control steps running,
  with the walnut between the jaws and the levers pressed in, the walnut cracks: the task file moves it to a closed box
  out of sight above the scene and moves a 6 g kernel (28 x 25 x 24 mm) and three pieces of shell (half the shell, 3 g,
  34 x 32 x 17 mm; two quarters, 1.5 g each, 30 x 17 x 16 and 26 x 17 x 15 mm), waiting in that box, onto the worktop
  along the V's middle line, 10, 14.5, 18.8 and 22.6 cm out from the pivot.
- How hard that is: the jaws touch the walnut 6.4 cm out along each lever and the hands push the grips 24.7 cm out, a
  3.85:1 lever, so 40 N at the jaws takes about 13 N on each handle (with the spring). A real in-shell walnut takes
  about 200-400 N at the jaws, 54-106 N per handle with this lever: the simulated shell is 5-10 times weaker, so that the
  YAM arms can crack it (pushing the grips sideways with their fingertips, they press the jaws with about 50 N at most
  before the walnut or the nutcracker starts to slip).
- Why not a shell held together by breakable joints: at the real masses (pieces of 1.2-3 g), PhysX joints between the
  pieces stretched 1-8 mm and twisted 4-40 degrees under the demo's squeeze even when set never to break (fixed and
  revolute joints, 32 to 255 solver iterations), so the shell came apart wherever the solver gave way, not at a set
  force.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with the kernel, the three pieces of shell, the bowl and
the saucer still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5, 0.75 or 1.0; see
`task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the
same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before
sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=crack_a_walnut_in_a_hand_nutcracker.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Crack the walnut with the nutcracker. Put the kernel in the white bowl and every piece of shell on the blue saucer, then bring both arms back to where they started." \
  --scorer task_score --max-steps 1100 --store-frames --log-dir ~/yam-eval/nut-dream
```

The scripted demo, which reaches full score: each hand comes down beside the outside of one handle's grip, fingers
closed and leaning in, and the two push the grips toward each other. The soft arms push only in proportion to how far
they are held short of where they aim, so they aim well past the grips (about 5 mm per newton) until the shell cracks,
about 4 s into the push. They draw back and the spring opens the levers. The right hand takes the kernel to the bowl and
goes back to where it started; the left takes the three pieces of shell to the saucer, one by one, and goes back to
where it started too:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=crack_a_walnut_in_a_hand_nutcracker.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Crack the walnut with the nutcracker. Put the kernel in the white bowl and every piece of shell on the blue saucer, then bring both arms back to where they started." \
  --scorer task_score --max-steps 1100 --store-frames --log-dir ~/yam-eval/nut-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/nut-dream/*.json; do inspect-robots video "$log"; done
```

Set `NUT_DEBUG=1` to print what the score is built from: how far each lever is from the V's middle line, where the
walnut sits along and across the V, the force each jaw presses it with and the most both have pressed it with, whether
and when it cracked, where the pivot, the walnut and each piece are, whether the kernel is in the bowl, how many pieces
of shell are on the saucer and whether any is in the bowl, whether both arms are back, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 6 minutes for a Dream-1 trial of 1,100 steps (375 s on the clock, with Dream answering in 0.66 s on average), and about 6 minutes for the scripted demo (900 steps, 45.0 s of simulated time, 386 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-6W7S9D. Its review said:

> Scoring (not met): Scoring refers only to two shell halves even though real walnut shells commonly break into additional pieces.
>
> What to change: Revise the instruction and scoring to accommodate walnuts that break into more than two shell pieces: award full credit when the kernel is in the bowl and all shell fragments are on the saucer, and update the simulated outcome and demo accordingly. Document how the simulated cracking resistance compares with a real walnut.

What changed in response:

- **More than two shell pieces:** the walnut (36 x 34 x 33 mm, 12 g) now cracks into a 6 g kernel and three pieces of
  shell: half the shell (3 g) and two quarters (1.5 g each). It cracks only once both jaws press it with at least 40 N
  each (read from the simulator's contact forces, held for 0.1 s, with the walnut between the jaws).
- **Scoring:** full credit needs the kernel in the bowl, all three pieces of shell on the saucer and both arms back.
  Partial credit: 0.75 with everything sorted but an arm not back, 0.5 for the kernel in the bowl with no shell in it,
  0.25 for cracked only. A walnut broken outside the nutcracker never counts. In a real trial, the kernel (whole or in
  pieces) must be in the bowl and every piece of shell larger than about 5 mm on the saucer.
- **Demo:** it now moves all four pieces (run limit 1,100 steps) and still reaches full score.
- **Cracking resistance (fidelity):** 40 N at the jaws is about 13 N on each handle at this nutcracker's 3.85:1 lever.
  A real walnut takes about 200-400 N at the jaws (54-106 N per handle), so the simulated shell is 5-10 times weaker,
  within what the arms can push. Rigid bodies can't fracture, so a force-triggered swap of the whole walnut for its
  pieces stands in for the break. This is disclosed.
- **Instruction, title, category:** the instruction states only the goal and constraints, the title no longer names a
  method, and the category is Societal relatability.
