# Rethread a hoodie drawstring

Feed the blunt threader through the hood casing until at least 8 cm of drawstring extends from each opening. Leave the hood and threader on the table.

Replacing a lost drawstring means guiding a partly hidden tool through flexible fabric while keeping the trailing end outside. The cord is already attached to the threader; detaching it is outside this trial.

## Run and score

Use the Python 3.12 / Isaac 6.1 / YAM versions pinned in `ENVIRONMENT.json`.
An NVIDIA GPU is required for the native cameras. From the repository root:

```sh
OMNI_KIT_ACCEPT_EULA=YES python tasks/rethread-a-hoodie-drawstring/run.py --mode demo --out /tmp/rethread-a-hoodie-drawstring-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/rethread-a-hoodie-drawstring/run.py --mode dream --out /tmp/rethread-a-hoodie-drawstring-dream
python tasks/rethread-a-hoodie-drawstring/rescore.py /tmp/rethread-a-hoodie-drawstring-demo
```

The layout is fixed; there is no random seed. Limit: 1,200 actions (60 seconds at 20 Hz).
Both modes use identical material dynamics, observations and reset. Dream gets
the standard three RGB views and robot joint state through the normal adapter.
Its credential is read privately by the SDK. Run only one trial per key at once.

The demo uses a saved sequence of ordinary bounded joint/gripper commands and
joint feedback for tracking. The material model reads measured hand poses;
it does not read the demo stage, commanded target, policy mode or score.
`run.json` records status, wall time and result. The trace records every measured
hand pose, material state and applied action. `rescore.py` verifies the frozen
source hashes and replays the measured hands. `--cpu-preview` is a planning
check and is explicitly excluded from submission evidence.

## Setup

An adult hood lies flat with its 56.5 cm curved drawstring casing facing the robot. The casing is 16 mm wide; the adjoining cotton fabric panel is 10 cm deep. A 90 cm long, 4 mm diameter braided cord is attached to a blunt 40 x 3 mm steel bodkin. Its nose starts 15 mm inside the right opening; the loose tail lies in a broad U on the table. Both hands start open. model.py specifies the complete fixed layout. A 650 x 850 x 1 mm silicone workmat covers the tabletop.

The setup image and constants in `model.py` fix positions in metres. Restore
the photographed starting arrangement and open both hands before each trial.

For reset, arrange the casing as a semicircle of 180 mm radius, openings
360 mm apart and facing the robot. The adjoining 100 mm fabric panel lies
outside that curve and is not fixed to the mat. Attach the cord to the blunt
threader before the trial, insert its nose 15 mm into the right opening and
lay the unused cord in the photographed loose U. No previous threading
through the rest of the casing is provided.

## Judging

1.0: cord follows the existing casing, at least 8 cm extends from each opening, fabric and cord intact, and both hands released for 3 seconds. 0.75 for that result before release/hold. 0.5 if the threader has emerged at least 4 cm but either usable tail is not yet long enough. Otherwise 0. Losing the trailing end into the casing or damaging material overrides credit. Check with photographs and a ruler.

Scores describe the final result; later damage invalidates earlier progress.
The demo must satisfy the release hold in the recorded episode itself.

## Fidelity

Native YAM arms/cameras. Hood panel is a free flexible mass-point sheet with table friction. A reduced sliding guide follows its casing; tool alignment and friction govern advance and drag the fabric. Cord is a length-constrained chain. Pinching through fabric gives idealized tool capture. Fibre friction, finger reaction and full tunnel-wall contacts are simplified; forces are uncalibrated.

The material mesh is drawn from the current simulated state at every step.
These are reduced models, not calibrated material or force measurements.

## Recorded trials

- Demo: **1.0**, 856 actions / 42.8 simulated seconds; 349.2 wall seconds including startup.
- Dream: **0.0 observed state**, failed before its first inferred action. The recording contains only 32 observation-preroll controls / 1.6 simulated seconds; 53.0 wall seconds including startup. This is not a completed rollout or a measure of model task success.

The first model response failed the unmodified official adapter validation:

```text
ValueError: q_l_chunk gripper leaves 0..1: [-0.110, 0.967]
```

A fresh session produced the same error. Both attempt reports are supplied.
No invalid command was executed and no adapter guard was relaxed. The
[submission guide](https://tasks.robocurve.org/) allows partial or failed model
runs. The short actual recording is included alongside the full-score demo.
Use `audit_failed_attempt.py RUN --audit-failed-attempt` to replay only a
failed attempt's recorded prefix; it never marks the rollout completed.


Both runs use the same material physics, cameras, instruction and starting
layout. Independent measured-hand replay agrees with both results. The videos
were checked from multiple views. `VALIDATION.json` records source hashes and
any non-executed source differences. Native traces, summaries and Dream request
records are in `evidence/`; no server session IDs or credentials are published.

Dream used the official `BridgeServer` / `BridgeSession` over an SSH loopback
tunnel, keeping its credential on the client computer. The bridge and runtime
versions are recorded in `evidence/`. A local SDK credential also works with
the commands above.

The uploaded videos preserve every native frame at 20 fps, with the selected two
judging views side by side. The setup image shows the same two starting views.
For CPU-only replay of the supplied evidence, install NumPy and SciPy and run:

```sh
python tasks/rethread-a-hoodie-drawstring/replay_evidence.py demo
python tasks/rethread-a-hoodie-drawstring/replay_evidence.py dream
```
