# Give a measured dose with a medicine spoon

Pick up the filled medicine spoon, deliver its 5 ml dose through the training
manikin's mouth without spilling or hitting its face, and return the spoon.
Use coloured water in physical trials. This tests utensil control around a
recipient; it does not test swallowing, patient cooperation or clinical safety.

## Run

Use the pinned Isaac/YAM installation in `ENVIRONMENT.json`. From the repository
root, with that environment active:

```sh
python tasks/give-medicine-with-a-spoon/run.py --mode demo --out runs/spoon-demo
python tasks/give-medicine-with-a-spoon/rescore.py runs/spoon-demo
python tasks/give-medicine-with-a-spoon/run.py --mode dream --out runs/spoon-dream
python tasks/give-medicine-with-a-spoon/rescore.py runs/spoon-dream
```

Dream uses its normal SDK credential or the official local bridge. No credential
belongs in this folder. The scene is fixed, with no object jitter or random seed.
The limit is 1,000 actions at 20 Hz; the scripted demo uses 744 actions (37.2 s).
Recorded wall times below include simulator startup.

## Setup and judging

See `JUDGING.md` for dimensions, the fixed layout and the rubric. Fill the
adaptive medicine spoon with 5.0 ml of water tinted with food colouring, measured
with an oral dosing syringe. The scoop has a 5.73 ml geometric capacity, so it
is deliberately not filled to its rim. A dose is prepared before each trial;
the robot is not choosing a drug, measuring a dose or using a kitchen teaspoon.

## Simulation limits

The YAM arms, joint drives, table contact and cameras run in native Isaac.
The spoon uses a finite, measured-pose pinch capture and gravity when released;
finger friction and spoon inertia are simplified. Its fluid is a conserved
finite-volume free surface with gravity, filtered translational acceleration,
overflow and ballistic droplets. It can spill or miss the mouth. Surface
tension, viscous coating, rotational slosh and swallowing are not simulated.
Spilled liquid is counted at contact; surface wetting and puddle spreading are
not rendered.

The life-size manikin has a lip opening, curved face, nose, head and a drain to
a collector. Oral walls intercept fluid. Spoon geometry, measured approach
speed and every native robot collision shape are checked against the face;
violations invalidate all credit. The conservative robot clearance envelope
is a geometric collision check, not a calibrated tissue-force model. The
head is fixed like a training fixture; there are no moving lips or active
patient responses. Its collector is shown cut away so the delivered volume
can be seen. Material models receive measured hands, never the controller's
stage or a success target. All sources and measured states are frozen for replay.

## Relationship to earlier work

[PR 713](https://github.com/robocurve/benchmark-tasks/pull/713) was rejected for
its rectangular mouth pit, solid-bead soup and missing face-contact safeguards.
This task uses a life-size oral interface, spillable liquid, dose recovery and
latched contact/insertion/speed limits. The manikin scope is explicit: passing
this task does not establish that a robot can safely feed a person.

## Checks

```sh
python -m unittest discover -s tasks/give-medicine-with-a-spoon -p 'test_*.py'
```

These check conservation, spillage, mouth entry, contact invalidation and scoring.
They do not replace the native demonstration or its visual review.

## Recorded trials

- Demo: **1.0**, 744 controls / 37.2 simulated seconds; 363.7 wall seconds including startup.
- Dream: **0.0**, 1000 controls / 50 simulated seconds; 829.5 wall seconds including startup.

Dream used 121 requests across 1 server session(s). Request attempts and timing are supplied in `evidence/`.

Both trials independently replay to their recorded scores. One assessor-only
camera was reframed after Dream; its original definition is preserved in the
evidence. The instruction, policy cameras, physics and controller are identical.
The demo video shows front and clear side views; the Dream video retains its
original front view. Both preserve every recorded frame at 20 fps.

Published collider bounds are packed as float32 arrays to keep the submission
small. Every one of the 630 collider checks at every recorded step was compared
with the original float64 bounds; packing changed no head-contact decision.
The packing reports give the maximum error and original trace hashes. All hand,
liquid and score fields are unchanged. Full uncompressed traces are retained
by the submitter. Replay without Isaac, a GPU or a credential:

```sh
python tasks/give-medicine-with-a-spoon/replay_evidence.py demo
python tasks/give-medicine-with-a-spoon/replay_evidence.py dream
```
