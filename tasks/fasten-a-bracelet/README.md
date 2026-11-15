# Fasten a bracelet on a wrist form

Fasten the bracelet around the wrist form, then let go and leave it secure.

Helping someone fasten small jewellery requires holding a flexible band, aligning its clasp and closing it without dropping either end. This trial uses a rigid wrist form.

## Run and score

Use Python 3.12 and the Isaac 6.1 / YAM environment pinned in `ENVIRONMENT.json`,
including its public asset-root patch. An NVIDIA GPU renders the cameras.
From the repository root:

```sh
OMNI_KIT_ACCEPT_EULA=YES python tasks/fasten-a-bracelet/run.py --mode demo --out /tmp/fasten-a-bracelet-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/fasten-a-bracelet/run.py --mode dream --out /tmp/fasten-a-bracelet-dream
python tasks/fasten-a-bracelet/rescore.py /tmp/fasten-a-bracelet-demo
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

A 22 cm long, 54 mm diameter rigid silicone wrist form lies horizontally on two 25 x 80 x 25 mm plastic end supports. The middle remains clear underneath. A blue leather band, 210 x 10 x 2 mm, lies flat below its midpoint with the two clasp ends 21 cm apart. Its silver push-in pin clasp has two 10 x 7 mm housings and a 6 mm projecting pin. Both robot hands start open. Use the fixed layout and clasp orientations in the setup photograph.

1.0 if the clasp is closed, the intact bracelet surrounds the wrist form within 5 cm of its midpoint, and both hands have released it for three seconds. 0.5 if it is closed around the form but still held. 0.25 if the pin has entered the socket by at least 1 mm but has not caught. Otherwise 0. A broken, overstretched or off-wrist bracelet scores 0. Inspect the clasp and released bracelet with photographs and a ruler.

Reset: Open the clasp and restore the flat band, form and hands to the photographed layout. Nothing is consumed.

## Model limits

The robot, rigid wrist form, supports and cameras use native PhysX. The leather is a coarse flexible, length-constrained chain with gravity and wrist contact. Visible clasp grips are idealized captures. The pin and socket require geometric alignment and insertion before a simplified spring catch engages. Leather stretch failure, misalignment and release can prevent success. Detailed spring deformation, resolved finger friction and forces back onto the arms are omitted; this is not a calibrated jewellery model.

## Recorded trials

- Demo: **1.0**, 490 actions / 24.5 simulated seconds; 269.5 wall seconds including startup.
- Dream: **0.0**, 1000 actions / 50 simulated seconds; 614.8 wall seconds including startup.

Both trials use the same runtime source and fixed starting layout. Independent
trace replay agrees with both scores. Videos were visually checked. Timings
include GPU startup and shared-host contention; they are not a speed benchmark.
`VALIDATION.json` records hashes and review details; compressed measured traces
and summaries are in `evidence/`. The uploaded videos preserve every native
frame at 20 fps, with two judging views placed side by side.
