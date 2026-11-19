# Secure a ponytail with a plain hair elastic

Gather the synthetic hair on the practice head into a ponytail and secure it with the plain elastic. Leave it held after both hands let go.

## Run and score

Use the Python 3.12 / Isaac Sim 6.1 / YAM runtime pinned in `ENVIRONMENT.json`.
An NVIDIA GPU is needed for native rendering. From the repository root:

```sh
OMNI_KIT_ACCEPT_EULA=YES python tasks/secure-a-ponytail/run.py --mode demo --out /tmp/secure-a-ponytail-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/secure-a-ponytail/run.py --mode dream --out /tmp/secure-a-ponytail-dream
python tasks/secure-a-ponytail/rescore_native_precision.py /tmp/secure-a-ponytail-demo
```

Dream receives the ordinary RGB cameras and joint state through Inspect Robots.
The SDK resolves its credential privately. Use one episode per key at a time.
The fixed layout has no random jitter or seed. The limit is 1,800
controls at 20 Hz. Demo and Dream use identical physics and hardware; the demo
commands only the robot joints and grippers.

Each run saves its source, complete state trace, camera recordings and report.
Rescoring checks source hashes and recomputes outcomes from the recorded native
states on CPU; it is not a new physics simulation. Camera strides above 1 are
for diagnostics and are excluded from submission evidence.

## Setup and reset

Use the adult synthetic-hair practice head and plain silicone elastic described
in `task.json`. The twelve loose locks weigh 3 g each; no lock is tied to another.
The head faces away from the robot. Its centre is (390, 0, 1052.5) mm in the
robot frame; the elastic starts at (270, 0, 810.5) mm on two secured wooden rests.
X points forward, Y left, and the table is 747.5 mm high. Restore the pictured
loose hair and separate band before each trial. Use intact hair and elastic.

The finger extensions have 40 mm working length, 24 mm width and 3 mm thickness.
Both arms use their ordinary mounting height and 40 N finger actuator limits.

## Scoring and material model

Count the locks passing through the band, measure the band's maximum width
and height, and check the released video. Its centre must be at least 55 mm
behind the head centre and 25–160 mm below it. Full credit needs at least eleven
locks, a band no wider than 50 mm at the specified ponytail height, and three
seconds retained after release. Nine locks earn 0.5; six earn 0.25, at that
height. Detached hair or a broken band scores zero. Passing the elastic only
over the hair tips does not complete the task.

Articulated 5 mm contact envelopes group fine hair fibres into flexible locks.
A native elastic surface models the band; its stiffness is approximate. A
geometry-gated native contact patch represents a centred finger pinch and
releases on opening, 8 mm slip or estimated 6 N local tension. This force
estimate is uncalibrated, not a measured native force limit. Local contacts at
a held patch are filtered to avoid conflicting constraints. No hair or band
pose is assigned during a trial.

The auxiliary pinch is selected within the central 8 mm of the 24 mm face.
It also holds adjacent elastic vertices already enclosed by that same real
face, representing friction over a finite contact patch. It does not attach
material outside the fingers. The estimated 6 N and 8 mm slip guards remain.

The tension guard estimates load at the exposed ends of the held patch,
using the current cord extension. It is checked at every control step.

The CPU audit wrapper restores the native float32 surface coordinates after
JSON loading, then runs the unchanged frozen scorer and tolerances. Without
that dtype restoration, the nearly circular band produces up to 0.000043 mm
of SVD measurement drift in the Dream trace. No score changes. The complete
recorded traces and original sources are preserved.

## Assets

All task geometry is original procedural work under the included MIT license.
The robot, table and runtime come from the pinned YAM / Isaac environment.
No third-party task meshes, scans or textures are used.

## Recorded trials

- Demo: **1**, 700 controls / 35 simulated seconds; 15.7 wall minutes including startup.
- Dream: **0**, 1800 controls / 90 simulated seconds; 48.7 wall minutes including startup.

Both completed recordings preserve every native frame at 20 fps. The uploaded
videos show two simultaneous judging views. CPU rescoring agrees with both
results, and the executed Python sources and demo plan match the submission.

Dream made 37 requests across 37 server sessions. The official adapter recovered expired sessions; this
is not an uninterrupted single-session run.

The instruction was clarified after recording the demo to identify the existing practice head and synthetic hair. Physics, actions, scoring and limits are unchanged; the frozen original metadata and explanation are preserved.

The model used the official loopback bridge over SSH; its credential remained
on the client computer. Adapter versions and the public request transcript
are in `evidence/`. No server session IDs or credentials are published.

Complete traces were audited locally and retained with their SHA-256
checksums in `evidence/trace-manifest.json`. They are omitted from the PR
to meet the 25 MB archive limit. A fresh run saves a full trace for `rescore.py`.
