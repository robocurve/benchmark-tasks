# Apply and peel a cold-wax strip

Apply the wax strip to the top of the silicone leg form, smooth it while keeping the dry edge available, then peel it back in one quick pull. Leave the strip clear of the leg and release it.

This is the application-and-removal movement used with a cold-wax strip, practised on a rigid silicone leg form. It does not test hair removal or treatment on a person.

## Run and score

Use Python 3.12 and the Isaac 6.1 / YAM environment pinned in `ENVIRONMENT.json`,
including its public asset-root patch. An NVIDIA GPU renders the cameras.
From the repository root:

```sh
OMNI_KIT_ACCEPT_EULA=YES python tasks/apply-and-peel-a-wax-strip/run.py --mode demo --out /tmp/apply-and-peel-a-wax-strip-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/apply-and-peel-a-wax-strip/run.py --mode dream --out /tmp/apply-and-peel-a-wax-strip-dream
python tasks/apply-and-peel-a-wax-strip/rescore.py /tmp/apply-and-peel-a-wax-strip-demo
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

A rigid silicone leg form is a horizontal cylinder 340 mm long and 120 mm in diameter, supported on a 65 x 350 x 15 mm plastic rest. A flexible 45 x 160 mm yellow nonwoven cold-wax training strip lies flat on the table beside it, with a marked dry edge at one end. The strip has a tacky underside and approximately 1 mm total thickness. Both hands start open. All dimensions and the single starting layout are shown in the setup photograph and scene code.

0.25 for smoothing at least 70% of the strip flat against the form while holding its marked dry edge. 0.5 if it is then completely removed and left clear of the form, released for two seconds. 1.0 if the removal takes at most two seconds, measured from the held edge moving 2 cm until the last adhesive contact releases. Pull back close to the form: raising the held end more than 7 cm above a remaining bond at a steeper-than-55-degree pull, tearing or overstretching the strip earns 0. Outcomes that do not meet an application milestone score 0. Use video, a ruler and a stopwatch.

Reset: Use a fresh strip and restore it to the flat starting position. Reset the arms; the leg form stays fixed.

## Model limits

The YAM arms and rigid practice form use native PhysX. A coarse 5 x 17 flexible sheet uses gravity, stretch, shear, bending and cylindrical contact constraints. Tacky contact creates breakable bonds; finger-pad contact smooths and bonds the sheet, and local tension releases a peel front. Grips are idealized closing-finger captures. Individual hairs, skin deformation, pain and real wax chemistry are not modelled. It measures strip handling only, not hair-removal effectiveness or skin safety.

## Recorded trials

- Demo: **1.0**, 658 actions / 32.9 simulated seconds; 519.2 wall seconds including startup.
- Dream: **0.0**, 1000 actions / 50 simulated seconds; 705.6 wall seconds including startup.

Both trials use the same runtime source and fixed starting layout. Independent
trace replay agrees with both scores. Videos were visually checked. Timings
include GPU startup and shared-host contention; they are not a speed benchmark.
`VALIDATION.json` records hashes and review details; compressed measured traces
and summaries are in `evidence/`. The uploaded videos preserve every native
frame at 20 fps, with two judging views placed side by side.

The setup image pairs the initial front view with the initial top policy camera
(enlarged), which shows the unused strip beside the form.
