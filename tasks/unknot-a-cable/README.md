# Untie a loose overhand knot

Untie the overhand knot in the cord. Leave the cord on the table without any crossings, then let go.

Undoing accidental knots is a common problem with household cords. The robot must feed an end back through the loop without cutting or breaking the cord.

## Run and score

Use Python 3.12 and the Isaac 6.1 / YAM environment pinned in `ENVIRONMENT.json`,
including its public asset-root patch. An NVIDIA GPU renders the cameras.
From the repository root:

```sh
OMNI_KIT_ACCEPT_EULA=YES python tasks/unknot-a-cable/run.py --mode demo --out /tmp/unknot-a-cable-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/unknot-a-cable/run.py --mode dream --out /tmp/unknot-a-cable-dream
python tasks/unknot-a-cable/rescore.py /tmp/unknot-a-cable-demo
# Re-score the included recordings on CPU, without Isaac:
python tasks/unknot-a-cable/replay_evidence.py
```

The layout is fixed; no random seed is used. The reset node geometry settles
under native gravity for one second before step 0 and the initial knot check. Each action is an ordinary bounded
14-element joint/gripper command at 20 Hz. The scripted demo follows a saved
joint plan and reads only the robot's joint positions for tracking. It never
reads the task state or scorer. Dream receives the normal three camera views
and robot joint observations. Its key comes from its private credential store
or `DREAM_API_KEY`; run one Dream session per key at a time.

The cord moves through native contact forces; its measured state supplies the score. `run.json` gives the score and wall runtime. Outputs also
include complete measured traces, source hashes, three judging videos, a policy
camera video and initial/final photographs. `rescore.py` verifies frozen source
hashes and recomputes topology from the measured native cord to check the recorded result. Always
inspect the videos as well. This task requires native PhysX; `--cpu-preview` cannot run its cord physics.

The public compressed traces retain every control step, action, measured hand
pose, cord centreline and maximum joint gap. Redundant rigid-link velocities
and poses are omitted to keep the submission small; the original raw trace
hashes are recorded in `VALIDATION.json`.

## Setup and judging

The wooden tabletop measures 1.10 x 1.30 m. One 1.075 m long, 10 mm diameter blue polyester cord, mass 80 g, lies on a wooden table in a loose overhand knot. Its two tips are marked yellow and red. The knot is centred about 28 cm in front of the robot; both free tips point towards the left-hand side of the table. Both grippers start open. The setup photograph and initial-nodes.json specify this single reproducible layout. No clamps, pins or hidden anchors hold the cord.

1.0 if the original overhand has been removed, the intact cord is entirely on the tabletop and has no crossings in an overhead view, the gap beneath the cord is no more than 2.5 cm anywhere, and both hands have released it for three seconds. 0.5 if the knot has been removed and the cord released for three seconds but it still has loose crossings or is not laid down. Otherwise 0. A broken cord or an invalid initial knot scores 0. Judge the knot and crossings from overhead and side video with a ruler; topology analysis supplies an additional simulation check.

Reset: Retie the same loose overhand and restore the photographed layout, or reset the scene. Nothing is consumed.

## Model limits

The cord is a native PhysX chain of 86 rounded capsules with bending constraints, gravity, friction, gripper contact and non-neighbour self-collision, simulated at 240 Hz. It is freely movable; no ideal grasp attachments, object teleports or scripted untying forces are used. A coarse link chain approximates bending, torsion and compression rather than individual fibres. Finite solver contacts can permit transient capsule overlap when squeezed; this is not a calibrated compression model. Static open-curve topology uses several closures and can be ambiguous; the final uncrossed, released cord and video remain the primary physical judging criteria.

## Recorded trials

- Demo: **1.0**, 1710 actions / 85.5 simulated seconds; 1071.0 wall seconds including startup.
- Dream: **0.0**, 1800 actions / 90 simulated seconds; 1415.5 wall seconds including startup.

Both trials use the same physics, cameras, instruction and starting layout. The scripted demonstration plan was updated after the baseline; Dream does not execute that plan or its offline authoring tools. Exact hashes and differences are recorded in VALIDATION.json. Independent
trace replay agrees with both scores. Videos were visually checked. Timings
include GPU startup and shared-host contention; they are not a speed benchmark.
`VALIDATION.json` records hashes and review details; compressed measured traces
and summaries are in `evidence/`. The uploaded videos preserve every native
frame at 20 fps, with two judging views placed side by side.

The final scorer adds a tabletop-support guard. Both native scores were rechecked
and remain unchanged. This score-only change does not affect physics or policy
observations; its exact patch is recorded in `VALIDATION.json`.
