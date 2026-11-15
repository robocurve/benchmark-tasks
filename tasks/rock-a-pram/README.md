# Rock a pram smoothly

Rock the pram containing a weighted training doll smoothly back and forth between the two floor marks ten times. Keep it straight, then stop near the starting mark and release the handle.

This tests controlled, repeated movement of a wheeled load through its handle. The occupant is a secured training doll; no live child is used.

## Run and score

Use Python 3.12 and the Isaac 6.1 / YAM environment pinned in `ENVIRONMENT.json`,
including its public asset-root patch. An NVIDIA GPU renders the cameras.
From the repository root:

```sh
OMNI_KIT_ACCEPT_EULA=YES python tasks/rock-a-pram/run.py --mode demo --out /tmp/rock-a-pram-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/rock-a-pram/run.py --mode dream --out /tmp/rock-a-pram-dream
python tasks/rock-a-pram/rescore.py /tmp/rock-a-pram-demo
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

A full-size aluminium-framed, four-wheel pram with a rigid polymer carrycot on a level, dry floor: 82.5 x 44.5 cm carrycot, 65 cm wheelbase, 51 cm wheel track, 24 cm rubber wheels and a 47 cm wide handle 102 cm above the floor. Total moving mass is 15 kg, including a secured 5 kg, approximately 56 cm vinyl training doll. A 70 x 36 cm foam mattress supports the doll. The robot faces the handle with both arms ready and grippers open. Two floor marks 20 cm apart align with the starting and forward positions of the rear axle. Use the fixed layout in the setup photograph.

0 if lateral drift exceeds 4 cm, yaw exceeds 10 degrees, speed exceeds 0.30 m/s, acceleration exceeds 1.5 m/s², or the pram leaves the 35 cm travel envelope. Otherwise, 0.5 for at least five complete rocks and 1.0 for at least ten, followed by at least three seconds at less than 1 cm/s translation and 0.02 rad/s yaw with the hands released and the rear axle within 5 cm of its starting position. A complete rock reaches at least 16.5 cm forward and returns to within 3.5 cm of the start. Fewer than five earns 0. Measure travel and timing from a ruler-marked floor and video; the trace supplies the same quantities.

Reset: Return the pram to the starting mark, with zero velocity and straight wheels; reset both arms. Nothing is consumed.

## Model limits

Native YAM arm dynamics and cameras drive a reduced planar pram model with 15 kg mass, yaw inertia, rolling resistance and bounded compliant handle grasps. The pram can translate and yaw. Native collision shells for the carrycot and doll follow that state and block the arms. Its wheels roll visually with travel. Vertical suspension, tipping, caster mechanics and doll motion are omitted. Grip capture and load reactions are approximated; the robot does not receive pram reaction forces. Smoothness thresholds define this benchmark, not infant-safety guidance.

## Recorded trials

- Demo: **1.0**, 940 actions / 47 simulated seconds; 483.9 wall seconds including startup.
- Dream: **0.0**, 1200 actions / 60 simulated seconds; 869.2 wall seconds including startup.

Both trials use the same runtime source and fixed starting layout. Independent
trace replay agrees with both scores. Videos were visually checked. Timings
include GPU startup and shared-host contention; they are not a speed benchmark.
`VALIDATION.json` records hashes and review details; compressed measured traces
and summaries are in `evidence/`. The uploaded videos preserve every native
frame at 20 fps, with two judging views placed side by side.
