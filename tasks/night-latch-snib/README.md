# Put a front door on the latch

Instruction: "Put the door on the latch, so that the bolt stays drawn back after you let go."

A Yale P77 night latch is fitted to a 30 x 30 cm section of a 44 mm door that stands on the table. The bolt is sprung
and stands 13 mm out of the door's edge. Turning the knob draws the bolt back against its spring; the snib (a 12 mm
button on the case) slides down into its hold notch only while the bolt is drawn back, and then keeps it back. Slid
down with the bolt out, the same snib deadlocks the bolt out. In the scripted demo the right hand grips the knob, turns
it 66 degrees and holds it; the left hand's closed fingertip pushes the snib down; then both let go and move away.

## Files

- `night_latch.py`: the scene, the latch (cam pin, sprung bolt, snib stop and notches), the scorer and the scripted demo.
- `latch_layout.py`: every size, mass, spring and threshold, with sources.
- `latchdemo.py`: steady hands for the scripted demo (it closes the soft arm joints' gap to the planned pose).
- `dev/run.sh`: one trial on the GPU machine. `dev/reach.py`: reach check without simulation.
- `runs/`: the logs of the runs below.

## Run and score a trial

From this folder on a machine with the `yam_arena` embodiment:

    bash dev/run.sh <name> <policy> <seed> 900 [DEMO_VARIANT=onehand|snibfirst|nothing] [LATCH_PROBE=1]

It runs `inspect-robots run --embodiment yam_arena -E task=night_latch.py -E object_jitter=0 --scorer task_score
--max-steps 900` with the instruction above, and prints the score. The layout is fixed; the seed changes nothing in
the scene. The scripted demo (`yam_scripted`) is a fixed plan for the fixed layout: it reads the arms' joint angles to
hold its planned poses and no object poses, and it moves the parts only through the grippers, by contact.
`LATCH_PROBE=1` runs the mechanism's truth table with a torque on the knob and a force on the snib instead of the arms.

Runtime per trial: about 4 minutes for the scripted demo (412 steps) and about 8 minutes for a Dream-1 run of 900
steps, on one L40S GPU shared with other jobs.

## Results

All runs on seed 0 with the same code.

| Run | Policy | Predicted | Score | Steps |
| --- | --- | --- | --- | --- |
| Demo (the video) | `yam_scripted` | 1.0 | 1.0 | 412 |
| Demo, repeat | `yam_scripted` | 1.0 | 1.0 | 412 |
| Nothing done (`DEMO_VARIANT=nothing`) | `yam_scripted` | 0 | 0 | 81 |
| One hand: knob turned, held 3 s, let go (`DEMO_VARIANT=onehand`) | `yam_scripted` | 0 | 0 | 350 |
| Wrong order: snib down first, then the knob (`DEMO_VARIANT=snibfirst`) | `yam_scripted` | 0 | 0 | 480 |
| Dream-1 | `dream` | - | 0.0 | 900 |

In the demo the knob is held at 66 degrees with the bolt flush, the snib goes down 6 mm, and after both hands let go
the bolt rests 1.2 mm out of the door's edge with the knob back at rest. In the one-hand run the bolt is back out at
13 mm as soon as the knob is let go. In the wrong-order run the snib deadlocks the bolt out: the gripper then turns the
knob only 22 degrees and the bolt gives 2.6 mm and returns. Dream-1 did not touch the latch: the knob, the bolt and the
snib never moved.

Mechanism probe (`LATCH_PROBE=1`, 0.30 N m on the knob, 3 N on the snib, no arms):

| Step | Knob (deg) | Bolt out (mm) | Snib down (mm) |
| --- | --- | --- | --- |
| Knob turned | 71.8 | -1.0 | 0.3 |
| Knob let go | 0.1 | 13.0 | 0.3 |
| Snib pushed, bolt out | 0.1 | 13.0 | 6.0 |
| Knob turned while deadlocked | 10.7 | 11.7 | 6.0 |
| Snib lifted | 0.1 | 13.0 | 0.0 |
| Knob held part way (0.085 N m), snib pushed | 16.7 | 10.2 | 0.5 |
| Knob turned and held, snib pushed | 71.8 | -1.0 | 6.0 |
| All let go | 0.0 | 1.2 | 6.0 |
| Snib lifted | 0.0 | 13.0 | 0.0 |
