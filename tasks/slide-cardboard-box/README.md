# Slide a box up to the table's far edge without overhang

A risk-versus-reward task: the robot slides a large, empty cardboard box toward the table's far edge. The closer it
ends up, the higher the score, but any overhang past the edge scores 0. Scene, scoring and the scripted demo are
in [`slide_cardboard_box.py`](slide_cardboard_box.py), a `yam_arena` task file (Isaac Lab Arena 0.3.1,
Isaac Sim 6.1, Isaac Lab 3.0, inspect-robots 0.60.0, yam-isaac-lab-arena @0059d02).

## Run a trial

Run from this folder. The scorer is `task_score`, which gives the score at the last step.

Scripted demo (open loop, objects at their exact starting positions):

    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=slide_cardboard_box.py -E object_jitter=0 \
      --scorer task_score --grader none --max-steps 2400 \
      --instruction "Slide the cardboard box as close as you can to the far edge of the table without letting any part of it hang over the edge."

A model under test (Dream-1 or any other action policy at 20 Hz):

    inspect-robots run --embodiment yam_arena --policy <policy> \
      -E task=slide_cardboard_box.py -E control_hz=20 --seed <seed> \
      --scorer task_score --grader none --max-steps 2400 \
      --instruction "Slide the cardboard box as close as you can to the far edge of the table without letting any part of it hang over the edge."

## Scoring

The gap is the distance from the box's outermost point to the plane of the table's far edge (`x = +0.91 m`).
It's computed from the box's world-aligned bounding box, so a corner of a turned box also counts.

| End state                                                                       | Score |
|---------------------------------------------------------------------------------|-------|
| Any overhang, box tilted > 10°, bottom > 1 cm above the table top, or off the table | 0     |
| Gap ≤ 1 cm                                                                      | 1.0   |
| Gap ≤ 2 cm                                                                      | 0.75  |
| Gap ≤ 4 cm                                                                      | 0.5   |
| Gap ≤ 8 cm                                                                      | 0.25  |
| Gap > 8 cm (the box starts at 26 cm)                                            | 0     |

The task has no `success()` hook, so the episode never ends early: only the final position counts.
Reaching the edge and then pushing the box over it scores 0.

## Runtime per trial

Run limit: 2,400 control steps (2 minutes of simulated time at 20 Hz, set with `-E control_hz=20`; the embodiment's
default is 10 Hz). A model's trial runs the full 2,400 steps unless the policy stops earlier. The scripted demo
(both arms push together) is timed in seconds, not steps: it takes 14.5 s, holds its last pose for 3 s, then
stops itself, at any control rate (it was recorded at the default 10 Hz).

## Seeds

The demo runs with `-E object_jitter=0`, so the box starts exactly as specified. For model trials, `--seed` sets
the seeded reset. Each reset moves the box by up to 3 cm in x and y (`object_jitter`, default 0.03 m), so the
starting gap is 23 to 29 cm.
