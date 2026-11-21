# Cap a glue stick lying on the desk, winding its glue down first

Instruction: "Put the cap back on the glue stick."

A Pritt Stick 43 g lies on the table with its glue wound 12 mm out; its cap lies beside it. The cap only seats once
the knob has been turned about two and a half turns to wind the glue below the rim. In the scripted demo the stick
stays on the table: one hand pinches the barrel down while the other's closed fingertips roll the knob over its top,
about a quarter turn per stroke; then one hand holds the barrel, the other pushes the cap home against its snap fit and
presses the capped stick down while the first hand lets go.

## Files

- `glue_stick.py`: the scene, the screw (the glue's slide follows the knob's turn), the cap's snap fit, the scorer and
  the scripted demo.
- `layout.py`: every size, mass, force and threshold, with sources.
- `gluedemo.py`: the scripted demo's planner hook (plans each stretch from the simulator's poses).
- `dev/run.sh`: one trial on the GPU machine.

## Run and score a trial

From this folder on a machine with the `yam_arena` embodiment:

    bash dev/run.sh <name> <policy> <seed> 1800 [DEMO_VARIANT=careless|no_cap|nothing]

It runs `inspect-robots run --embodiment yam_arena -E task=glue_stick.py -E object_jitter=0 --scorer task_score
--max-steps 1800` with the instruction above, and prints the score. The layout is fixed; the seed changes nothing in
the scene. The scripted demo (`yam_scripted`) reads object poses from the simulator to plan its motions and moves the
parts only through the grippers, by contact.

Runtime per trial: about 7 to 9 minutes for the full scripted demo (1,462 steps) and about 18 minutes for a Dream-1 run
of 1,800 steps, on one L40S GPU shared with other jobs.

## Results

All runs on seed 0 with the same code. The scripted demo reads object poses from the simulator to plan its motions.

| Run | Policy | Predicted | Score | Steps |
| --- | --- | --- | --- | --- |
| Demo | `yam_scripted` | 1.0 | 1.0 | 1,462 |
| Demo, repeat (the video) | `yam_scripted` | 1.0 | 1.0 | 1,462 |
| Nothing done (`DEMO_VARIANT=nothing`) | `yam_scripted` | 0 | 0 | 81 |
| Careless: cap pushed straight on, glue not wound down (`DEMO_VARIANT=careless`) | `yam_scripted` | 0 | 0 | 835 |
| Part done: wound down, cap left lying (`DEMO_VARIANT=no_cap`) | `yam_scripted` | 0.5 | 0.5 | 989 |
| Dream-1 | `dream` | - | 0.0 | 1,800 |

In the demo the knob turns 2.5 times (the glue ends 3.7 mm below the rim), the cap clicks home without touching the
glue, and both hands end clear of the stick. In the careless run the cap's inner end meets the standing glue (score 0
from that moment). Dream-1 barely moved the knob and did not pick up the cap.
