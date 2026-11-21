# Verification record

## Offline checks

`python3 -m unittest discover -s tests -v`: 15 tests, all pass (the capping rule's engagement, turn counting, regrips,
wrap-around, the 10.5 mm cover depth, and every scoring tier and outcome).

## Scripted demo (GPU physics, yam_arena, fixed layout, no jitter)

| Run | Recording | task_score | Success | Steps |
|---|---|---|---|---|
| Final | 640 x 360 cameras, recorded (`demo.mp4`) | 1.0 | yes | 2,013 of 3,600 (101 s) |

Step for step:

- Pushed on: the copper 13.1 mm into the nut, 1.5 mm off its axis, 2.4 deg off its line.
- Turned in four strokes (177, 360, 521, 706 deg, then slowly to 722 deg). The nut bites at step 826, the copper 15.1 mm
  in (covered).
- After the folds the nut sits 20.8 mm onto the copper. Every point of both splices is inside the box, the frontmost
  20.7 mm behind the rim.
- The cover is closed on both screws at step 1994 (178.4 deg of its 180 deg swing, within the 2 deg tolerance); the
  score holds 1.0 for 1 s and the trial ends on success at step 2013.

Right-arm IK error at its worst: 4.8 mm, 1.7 deg (during the black fold); left arm: 0.1 mm.

## Dream-1

`dreamlabs/dream-1-beta` through inspect-robots (`-E object_jitter=0 -E control_hz=20`, 320 x 176 cameras, the
instruction above): task_score 0.0, the 3,600-step limit reached. The left hand reached toward the hanging cover and
the box again and again without swinging the cover closed; the right hand went to the parts tray and hovered over the
nuts without picking one up. No nut went onto the black splice.
