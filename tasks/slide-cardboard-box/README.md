# Slide a box up to the table's far edge without overhang

A risk-versus-reward task: the robot slides a large, empty cardboard box toward the table's far edge. The closer it
ends up, the higher the score, but any overhang scores 0. Near the edge, the box hides the gap
from all three cameras (the top camera sees the table edge on each side of the box, not the box's front face), so
the robot must estimate the gap from the box's size and position. Scene, scoring and the scripted demo are
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
      -E task=slide_cardboard_box.py -E object_jitter=0 -E control_hz=20 \
      --scorer task_score --grader none --max-steps 2400 \
      --instruction "Slide the cardboard box as close as you can to the far edge of the table without letting any part of it hang over the edge."

## Scoring

The score uses only where the box is at the end of the episode.

**Score 0** if any of these is true:
- any part of the box hangs over any edge of the table;
- the box is no longer flat on the table (tilted more than 10°, or lifted more than 1 cm);
- the box is off the table.

**Otherwise,** measure the gap: the perpendicular distance from the table's far edge to each of the two box corners
nearest it, averaged. The score goes up linearly from 0 where the box started to 1.0 at a gap of 1 cm or less:

    score = (start gap - gap) / (start gap - 1 cm), clipped to 0..1

The box starts 26 cm from the edge, so the score is `(26 cm - gap) / 25 cm`. Each centimetre is worth 0.04:

| Gap | 26 cm or more | 21 cm | 16 cm | 11 cm | 6 cm | 2 cm | 1 cm or less |
|---|---|---|---|---|---|---|---|
| Score | 0 | 0.2 | 0.4 | 0.6 | 0.8 | 0.96 | 1.0 |

A turned box needs both corners near the edge. For example, a box turned 5° with one corner 0.5 cm from the
edge has an average gap of 2.2 cm and scores 0.95.

The episode does not end early when the box reaches the edge, so pushing it over the edge afterwards still
scores 0.

In the code, overhang uses the box's world-aligned bounding box, which includes the corners of a turned box. The
code uses the fixed start gap of 26 cm (see [Seeds](#seeds)).

## Runtime per trial

Run limit: 2,400 control steps (2 minutes of simulated time at 20 Hz, set with `-E control_hz=20`; the embodiment's
default is 10 Hz). A model's trial runs the full 2,400 steps unless the policy stops earlier. The scripted demo
(both arms push together) is timed in seconds, not steps: it takes 14.5 s, holds its last pose for 3 s, then
stops itself, at any control rate (it was recorded at the default 10 Hz).

## Seeds

One fixed layout: for simplicity, all trials run with `-E object_jitter=0`, so the box always starts 26 cm from
the edge. The scoring code uses this fixed start gap, so keep the jitter at 0. With jitter, the box could start up
to 3 cm nearer the edge and score up to 0.12 without moving. A real trial that starts at a different gap uses that
gap in the formula.
