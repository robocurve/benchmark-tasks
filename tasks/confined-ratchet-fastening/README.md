# Fasten a bracket bolt with a ratchet in confined access

Both arms start at their standard rest pose with open fingers. The ratchet
rests in its physical cradle, separate from the started bolt. One arm must
grasp and lift the ratchet, carry it to the bracket, fit the socket over the
bolt head and then seat the bolt with short drive and return strokes. The
reference demo uses the right arm while the left arm stays at rest.

The fixed layout contains an M8x16 bolt (1.25 mm pitch, 13 mm AF head, 14 g), a
3 mm bracket on a 93 mm receiver stand, a 203 mm ratchet and 13.4 mm AF
socket, a cradle, and two 162 mm tall walls. The walls limit the ratchet swing to
about ±16 degrees; the reference controller commands ±14 degrees (measured extrema −15.735/+15.184 degrees). The ratchet has
two small visible flat rubber grip pads, allowing the rigid fingertips to hold
its otherwise rounded grip. Dimensions and starting poses are in
`ratchet_task/config.py`, `asset_geometry.py` and `build_assets.py`.

## Run

Use the existing Isaac Lab Arena/YAM environment with Isaac Sim 6.1.0,
IsaacLab 17.0.2, Arena 0.3.0, Inspect Robots 0.60.0 and yam-arena 0.1. All
cluster simulation and rendering must run on an allocated compute node.
From this folder:

```sh
uv pip install --no-deps -e .
python -m unittest discover -p 'test_*.py'
python diagnostics/demo.py /absolute/path/outside/source/demo
```

The full demo runner records native top and wrist H.264 videos, per-step
mechanism/scorer telemetry and `demo-result.json`. The review and handoff
videos `demo.mp4` and `dream.mp4` use only the original native top camera
(640×360 at 20 Hz). The registered reference policy is `confined_ratchet_scripted`.

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
drag. The thread uses a native prismatic/revolute pitch coupling, with a hard
terminal seat. Thread ridges are visual, and microscopic tooth impacts,
material deformation and fastener strength are omitted. The invisible 14 g
carrier adds translational inertia and has no contact geometry.

The bolt starts engaged in the receiver with a 2.5 mm under-head gap. Both
arms start at READY with open grippers, and the complete ratchet lies on its
cradle. A reset-only placement joint holds the bolt while its native thread is
created; it is removed before policy actions. The tool is supported by actual
cradle contacts. There are no tool-to-hand or socket-to-bolt attachment joints.
The native thread stays active while socket contact carries the applied torque.

The current-state score is 0.2 after a quarter clockwise turn, 0.4 after one
turn, 0.7 for seating within −0.3/+0.2 mm after at least 1.8 turns, and 1.0
after a stable seated hold. The initialized assembly earns zero.
Seating must hold for 0.5 seconds with the socket engaged and at least six
ratchet drive/return cycles, each with actual bolt advance. The scorer rejects nonfinite states, thread-constraint
failure, fixture displacement above 2 mm and contact torque above 1 N m.
Contact torque is recorded from physical contact impulses; a calibrated torque
target is not required.

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

The full rest → pickup → placement → ratcheting demo (`rest-demo-002`)
completed with score **1.0** in 2,229 actions (111.45 simulated seconds;
418.03 wall seconds).
The tool was physically lifted 93.65 mm; the left arm stayed at rest. It
completed 32 drive/return cycles and 1.8452 clockwise turns, finishing with a
0.1961 mm gap and a stable 0.5 s seated hold. Maximum pitch error was 19.88 µm.
The socket stayed engaged throughout ratcheting. All three native camera views
were reviewed, and 26 offline checks passed.

Environment revision: `f25d8f934652375d062d4940f0c66cdb8c2ed70537427d6d1b82c0f37aa04d49`.
Dream-1 (`dream-rest-001`) completed all 4,000 actions (200 simulated
seconds; 727.94 wall seconds) with score **0.0**, stopping at the action limit.
The reviewed native views show no successful pickup or socket placement.
There were no errored trials. It used the same environment revision, seed 17,
instruction, camera resolution and action rate as the qualified demo. The
client made 83 inference requests with 30 automatic request retries. Private
credentials were removed at termination.

The earlier version started with the ratchet already held and fitted. Its
sources and results are preserved as an earlier revision; they do not qualify
the pickup-and-placement sequence documented here.

Reviewed submission media are attached as `M-KGU6MH`: the native top-camera
Dream run, full scripted demo and setup screenshot. Sources and detailed
evaluation evidence are retained locally; `validation.json` records the original
validation state before submission media were uploaded.
