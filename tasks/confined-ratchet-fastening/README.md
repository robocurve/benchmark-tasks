# Fasten a bracket bolt with a ratchet in confined access

Both arms start at their standard rest pose with open fingers. The ratchet
rests in its physical cradle, separate from the started bolt. One arm must
grasp and lift the ratchet, carry it to the bracket, fit the socket over the
bolt head and then seat the bolt with short drive and return strokes. The
reference demo uses the right arm while the left arm stays at rest.

The fixed layout contains an M8x16 bolt (1.25 mm pitch, 13 mm AF head, 14 g), a
3 mm bracket on a 93 mm receiver stand, a 203 mm ratchet and 13.4 mm AF
socket, a cradle, and two 162 mm tall walls. The walls limit the ratchet swing to
about ±16 degrees; the reference controller commands ±14 degrees (measured extrema −15.739/+15.171 degrees). The ratchet has
two small visible flat rubber grip pads, allowing the rigid fingertips to hold
its otherwise rounded grip. Dimensions and starting poses are in
`ratchet_task/config.py`, `asset_geometry.py` and `build_assets.py`.

## Run

Use the existing Isaac Lab Arena/YAM environment with Isaac Sim 6.1.0,
IsaacLab 17.0.2, Arena 0.3.0, Inspect Robots 0.60.0 and yam-arena 0.1. All
simulation and rendering require a verified live RTX runtime. The seating
revision below ran on the existing Brev L40S, with CPU PhysX and GPU rendering.
From this folder:

```sh
uv pip install --no-deps -e .
python -m unittest discover -p 'test_*.py'
TRIAL_OUTPUT=/absolute/path/outside/source/new-demo python diagnostics/demo.py
```

The full demo runner records native top and wrist H.264 videos, per-step
mechanism/scorer telemetry and `demo-result.json`. The output directory must
not already exist; the runner creates it to preserve prior evidence. The current
demo records native top and wrist cameras at 640×360 and 20 Hz. The registered
reference policy is `confined_ratchet_scripted`.

Dream evaluation uses the same physical scene, 480 Hz CPU PhysX, 20 Hz actions,
native 640×360 top and wrist cameras, `top_cam=demo`, zero object jitter and seed
17. The horizon is 4,000 actions (200 simulated seconds). With the private
Dream credentials configured outside this source directory:

```sh
inspect-robots run --embodiment yam_confined_ratchet \
  -E task=task.py -E top_cam=demo -E device=cpu -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 -E object_jitter=0 \
  --policy dream -P model=dreamlabs/dream-1-beta \
  --instruction "Pick up the ratchet from its cradle, fit the socket onto the started bracket bolt, then tighten it with short back-and-forth strokes between the walls until the bolt is seated." \
  --seed 17 --max-steps 4000 --scorer task_score \
  --store-frames --no-rerun --no-prompt --grader none --log-dir /absolute/path/run
```

## Contact, rotation and scoring

The socket and receiver use open compound collision geometry. The bolt head
contacts the metric socket; no grasp attachment or policy-driven pose changes
are used. The handle, rotor and socket have masses of 399.405 g, 28.355 g and
40 g. Handle friction is 0.9 static / 0.7 dynamic; other task bodies use 0.55 /
0.4. The native one-way hard limit has 5 degrees of take-up and passive return
drag. The thread uses a native prismatic/revolute pitch coupling. Head/bracket
contact supplies the seating load. An angular backup limit lies 0.1 mm behind
the physical seat; installing that limit does not qualify as seating. Thread
ridges are visual, and microscopic tooth impacts,
material deformation and fastener strength are omitted. The invisible 14 g
carrier adds translational inertia and has no contact geometry.

The bolt starts engaged in the receiver with a 2.5 mm under-head gap. Both
arms start at READY with open grippers, and the complete ratchet lies on its
cradle. A reset-only placement joint holds the bolt while its native thread is
created; it is removed before policy actions. The tool is supported by actual
cradle contacts. There are no tool-to-hand or socket-to-bolt attachment joints.
The native thread stays active while socket contact carries the applied torque.

The current-state score is 0.2 after a quarter clockwise turn, 0.4 after one
turn, 0.7 for verified head/bracket seating after at least 1.8 turns, and 1.0
after a stable seated hold. The initialized assembly earns zero. Seating uses
the exact `bolt/HeadContact` and `fixture/Bracket/Sector00`–`Sector47` collider
pair. Each accepted contact must lie on the head underside and bracket top,
inside the hex footprint and outside the bracket bore, with a normal within
5 degrees of both surface normals and at least 1 mN normal force from its
measured contact impulse. Socket, shank, receiver, wall and head-side contacts
cannot establish seating; friction anchors do not count as seating contacts.

Each shape in this seating pair has a 10 µm candidate-contact offset and zero
rest offset. Candidate contacts alone do not earn credit. Measured separation
must be between −50 and +2 µm, the head-centre gap between −50 and +2 µm,
and all underside vertices between −50 and +20 µm from the bracket top. The
2 µm positive allowance accommodates solver/pose precision; an impulse and the
surface checks remain mandatory. Penetration beyond 50 µm invalidates a trial.

Seating must persist for 0.5 seconds with the socket engaged and at least six
ratchet drive/return cycles, each with actual bolt advance. Every 20 Hz control
interval must end in accepted contact, contain accepted contact on at least
90% of its 480 Hz physics steps, and have at least 1 mN mean normal seating
force. No-contact steps count in the denominator; old contact samples are
cleared every control interval. Loss of the seated condition removes seating
credit and resets the hold. After qualification, the socket may disengage
while the physical seat persists. The scorer also rejects nonfinite states,
thread-constraint failure, fixture displacement above 2 mm and contact torque above 1 N m.
Contact torque is recorded from physical contact impulses; a calibrated torque
target is not required.

For a physical trial, mark the starting bolt angle and count drive/return
strokes from video; the known 1.25 mm pitch relates turns to axial travel.
Inspect the head/bracket interface from the side after withdrawing the socket
if it obscures the view. The required outcome is the head bearing on the
bracket, not a small visible clearance. A thin feeler gauge can reveal a
remaining gap; an obscured camera view alone does not prove contact. The
micrometre bounds and impulse thresholds above are simulator checks for this
physical condition, not a requirement to instrument a real trial at 480 Hz.
The task seats a started bolt; it does not certify fastening torque or preload.

## Assets

The ratchet visual is derived from NVIDIA SimReady, NVIDIA, CC-BY-4.0,
pinned revision `653d91e182011f357cf1644013cc94ddf194d349`:
https://huggingface.co/datasets/nvidia/simready-assets/tree/653d91e182011f357cf1644013cc94ddf194d349

Source meshes and UVs are preserved, source materials converted to portable
UsdPreviewSurface, and textures reduced to 1K. The source was located through
the NVIDIA SimReady asset search API. The metric socket, bolt, fixture, grip
pads and walls are original parametric geometry. Licenses, hashes and
derivation records are under `assets/`. The optional imperial socket visual is
reference material; the runtime uses the dimensioned metric socket.

Portable assets and textures are included. Rebuild with OpenUSD, NumPy, SciPy
and Pillow using `python fetch_assets.py --prepare` followed by
`python build_assets.py`. Original large vendor downloads are ignored by Git.

## Results

### Current seating revision — 2026-11-12

The full rest → pickup → placement → ratcheting demo (`seated-contact-001`)
completed with score **1.0** in **2,458 of 4,000 actions** (122.90 simulated
seconds; 511.45 wall seconds), seed 17. It completed 2.014725 clockwise turns
and 36 drive/return cycles. The final measured centre gap was −0.363 µm, with
underside vertex gaps from −0.759 to +0.033 µm. Final accepted contact
separations were −0.153 to −0.015 µm, with 12.522 N mean seating force and
contact in all 24 physics steps of the final control interval.

The qualifying 0.5 s hold occupied control steps 2448–2457 (zero-based): 237
of 240 physics steps had accepted seating contact, each interval had at least
22 of 24, and mean seating force ranged from 9.157 to 13.389 N. Maximum accepted
separation during the hold was +1.970 µm, within the explicit solver allowance.
The backup angular limit still had 23.50 degrees of travel remaining, so it
was not the source of the seated stop. Maximum pitch error was 22.024 µm;
maximum mean/peak socket torque was 0.03149/0.03988 N m. No integrity fault
was recorded. Pickup lift was 93.65 mm; socket engagement held throughout
ratcheting and the left arm stayed at rest.

All 41 offline checks passed. An independent saved-trace audit reproduced
every score, reconstructed underside gaps directly from the USD and recorded
poses, and found that all seating gates first qualified full credit on the
terminal step. Removing surface contact or force, restoring the old +0.196 mm
gap, or reducing contact duty below 90% limited credit to 0.4; deep penetration
invalidated the replay. Saved telemetry contains control-interval summaries,
not the raw contact manifold from every physics step. Selected camera frames
show pickup, placement and final hold; the socket occludes the seating interface,
so its quantitative contact evidence comes from the exact collision pair.
The sampled native-view review covered top-camera frames at 0, 18, 40 and
122 seconds and both final wrist-camera views. Pickup, socket placement and
hold were consistent with the trace; this was not an exhaustive review of
every video frame, and micrometre seating was established by telemetry.
The scripted controller
uses simulator pose feedback; this run is not a robot-only perception result.

Environment revision:
`dc35653bad5063a15ebb4f46ad30bb1ce66f23f62e4bf2580e5f627246bfdf37`.
Runtime: NVIDIA L40S, Isaac Sim 6.1.0.0, IsaacLab 17.0.2, Arena 0.3.0,
Inspect Robots 0.60.0, YAM commit
`0059d02a1a91bc6834c1d0846022891df0c78730`, Arena commit
`c7edf3a9d5d8276565208f72422f8f8a0806d6cf`.
The matching Dream-1 evaluation (`seated-dream-001`,
`dreamlabs/dream-1-beta`) completed normally with score **0.0** after all
4,000 actions (200 simulated seconds; 673.02 wall seconds), stopping at the
action limit with zero errored trials. It used the same environment revision,
seed 17, instruction, cameras and action rate as the corrected demo. Mean
inference latency was 0.566 seconds. The parent reviewed sampled native
top-camera frames at 0, 50, 100 and 199 seconds and both final wrist views.
The ratchet is moved out of its cradle and lies on the table by the final
sampled views; no completed seating is shown. This is the current model
outcome, distinct from the older run below. The review covered selected
frames, not every video frame. No new media has been uploaded; the existing
media ID remains historical pending an explicitly authorized replacement.

Current local evidence paths below are relative to the workspace's
`current/runs/confined-ratchet-fastening/` directory:

| Artifact | Path | SHA-256 |
| --- | --- | --- |
| Demo, native top camera | `seated-contact-001/remote/output/demo/top_cam.mp4` | `14b8416c9ef9ad7736e9302327e46c3d80d5ecfe8fb474bc163d69fe9c960113` |
| Dream, native top camera | `seated-dream-001/videos/scene-0-e0_top_cam.mp4` | `508713f1675dce950a6b89c72edf635b7249164308d3b8992ba01620ffccf02e` |
| Setup, initial native top camera | `seated-contact-001/remote/output/demo/initial-top_cam.png` | `f57f2b46aaba3942242b9d9997c360271b4fb133a2db70d9a76eb54d9ee3708b` |

The same demo directory contains `left_cam.mp4` and `right_cam.mp4`; the Dream
video directory contains `scene-0-e0_left_cam.mp4` and
`scene-0-e0_right_cam.mp4`. Sampled-review montages are
`seated-contact-001/remote/output/visual-review/review.jpg` and
`seated-dream-001/videos/review.jpg`. Complete paths, byte sizes and SHA-256
hashes for all six videos, the setup image and both montages are in
`validation.json` under `seated_contact_revision.media.current_artifacts`.
The parent independently reran the 41 offline checks successfully and reviewed
the complete diff and scorer before local commit review.

### Historical submitted revision — not current validation

The earlier full rest → pickup → placement → ratcheting demo (`rest-demo-002`)
completed with score **1.0** in 2,229 actions (111.45 simulated seconds;
418.03 wall seconds).
The tool was physically lifted 93.65 mm; the left arm stayed at rest. It
completed 32 drive/return cycles and 1.8452 clockwise turns, finishing with a
0.1961 mm gap and a reported 0.5 s hold under the former gap criterion. That
result did not establish physical seating. Maximum pitch error was 19.88 µm.
The socket stayed engaged throughout ratcheting. All three native camera views
were reviewed, and 26 offline checks passed.

Environment revision: `f25d8f934652375d062d4940f0c66cdb8c2ed70537427d6d1b82c0f37aa04d49`.
Dream-1 (`dream-rest-001`) completed all 4,000 actions (200 simulated
seconds; 727.94 wall seconds) with score **0.0**, stopping at the action limit.
The historical reviewed native views show no successful pickup or socket placement.
There were no errored trials. It used the same environment revision, seed 17,
instruction, camera resolution and action rate as the qualified demo. The
client made 83 inference requests with 30 automatic request retries. Private
credentials were removed at termination.

The earlier version started with the ratchet already held and fitted. Its
sources and results are preserved as an earlier revision; they do not qualify
the pickup-and-placement sequence documented here.

Media `M-KHDNNB` links the revised native top-camera Dream run (4,000 actions,
score 0), seated-contact scripted demo (2,458 actions, score 1) and setup
screenshot. Historical media `M-KGU6MH` remains recorded only as provenance.
Sources and detailed evaluation evidence are retained locally. The original fields in
`validation.json` preserve the historical validation unchanged; its appended
`seated_contact_revision` records the corrected demo and matching revised Dream.
