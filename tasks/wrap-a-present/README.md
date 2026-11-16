# Wrap a gift box with pre-creased paper

Fold the paper around the gift box, hold the overlapping flaps and secure them with the two tape strips. Leave the wrapped gift on the table and let go.

Gift wrapping requires shaping loose material and securing an overlap while keeping it in place. The paper is prepared in advance; cutting and making a bow are outside this task.

## Run and score

Use Python 3.12 and the Isaac 6.1 / YAM environment pinned in `ENVIRONMENT.json`,
including its public asset-root patch. An NVIDIA GPU renders the cameras.
From the repository root:

```sh
OMNI_KIT_ACCEPT_EULA=YES python tasks/wrap-a-present/run.py --mode demo --out /tmp/wrap-a-present-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/wrap-a-present/run.py --mode dream --out /tmp/wrap-a-present-dream
python tasks/wrap-a-present/rescore.py /tmp/wrap-a-present-demo
```

The layout is fixed; no random seed is used. Each action is an ordinary bounded
14-element joint/gripper command at 20 Hz. The scripted demo follows a saved
joint plan and reads only the robot's joint positions for tracking. It never
reads the task state or scorer. Dream receives the normal three camera views
and robot joint observations. Its key comes from its private credential store
or `DREAM_API_KEY`; run one Dream session per key at a time.

The material model reads measured hands, not action targets, stage labels or a
requested score. `run.json` gives the score and wall runtime. Outputs also
include complete measured traces, source hashes, three judging videos, a policy
camera video and initial/final photographs. `rescore.py` verifies frozen source
hashes and replays the measured hands to check the recorded result. Always
inspect the videos as well. `--cpu-preview` is an IK/material debugging aid and
is explicitly excluded from submission evidence.

## Setup and judging

A fixed 120 x 160 x 40 mm cardboard gift box rests on the centre of a blue cross-cut sheet of 100 gsm gift paper. Its central rectangle matches the box. The two 120 mm-wide flaps extend 140 mm beyond the long sides; the two 160 mm-wide flaps extend 120 mm beyond the short sides. Each flap is pre-creased 40 mm from its base; its outer panel initially rests about 17 degrees above the table, leaving the edge accessible. Two separate 40 x 12 mm paper-backed adhesive tape strips lie flat to the left of the box, at (22, 14) and (22, 24) cm in the scene’s table coordinates. The box holds the centre of the paper in place. Hands begin open; the setup photograph and scene coordinates fix all positions.

1.0 if all four sides and the top are covered, both opposite flap pairs are folded over the top and secured by separate tape strips bridging the upper flap edge with at least 8 mm on each side, and the hands have released the package for three seconds. 0.25 for each secured flap pair, up to 0.5 until the full release condition is met. Otherwise 0. A torn or overextended sheet scores 0. Judge coverage, tape placement and release from video and final photographs; no hidden target or special instrument is required.

Reset: Reset the paper to its original slightly raised creases and replace the two used tape strips. Restore the fixed starting layout.

## Model limits

The robot and fixed box use native PhysX. The pre-creased paper has eight passive fold hinges: panels preserve their shape between creases, with gravity, damping, plastic crease memory, finger-pad contact, box contact and measured-hand grasp forces. It is a folding-paper approximation, not arbitrary cloth or crumpling. Tape bonds after 0.4 seconds of flat contact across the exposed upper flap edge, with contact on both sides, then holds that pair of flaps. Grasp points cover the tape rectangle, including its accessible edges. Detailed adhesive traction, tape stretching and new creases are omitted. Closed side faces render 0.6 mm outside the box to avoid coincident-surface artifacts; this does not change the material state or score. The box and paper centre remain fixed; grasp capture and feedback forces are simplified.

## Recorded trials

- Demo: **1.0**, 1596 actions / 79.8 simulated seconds; 803.6 wall seconds including startup.
- Dream: **0.0**, 1600 actions / 80 simulated seconds; 920.7 wall seconds including startup.

Both trials use the same physics and starting layout. The demo adds a 0.6 mm rendering offset for closed paper sides. The baseline never closes a side far enough to enter that draw-only branch; its observations are unchanged. Exact source differences and trace checks are in VALIDATION.json. Independent
trace replay agrees with both scores. Videos were visually checked. Timings
include GPU startup and shared-host contention; they are not a speed benchmark.
`VALIDATION.json` records hashes and review details; compressed measured traces
and summaries are in `evidence/`. The uploaded videos preserve every native
frame at 20 fps, with two judging views placed side by side.
