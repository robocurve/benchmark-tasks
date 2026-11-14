# Fasten a bracket bolt with a ratchet in confined access

Both arms start at their standard rest pose with open fingers. The ratchet
rests in its physical cradle, separate from the started bolt. One arm must
grasp and lift the ratchet, carry it to the bracket, fit the socket over the
bolt head and then seat the bolt with short drive and return strokes. The
reference demo uses the right arm while the left arm stays at rest.

The fixed layout contains an M8x16 bolt (1.25 mm pitch, 13 mm AF head, 14 g), a
3 mm bracket on a 93 mm receiver stand, a 203 mm ratchet and 13.4 mm AF
socket, a cradle, and two 162 mm tall walls. The walls limit the ratchet swing to
about ±16 degrees; the reference controller commands ±14 degrees. The ratchet has
two small visible flat rubber grip pads, allowing the rigid fingertips to hold
its otherwise rounded grip. Dimensions and starting poses are in
`ratchet_task/config.py`, `asset_geometry.py` and `build_assets.py`.

## Run

Use the existing Isaac Lab Arena/YAM environment with Isaac Sim 6.1.0,
IsaacLab 17.0.2, Arena 0.3.0, Inspect Robots 0.60.0 and yam-arena 0.1. All
simulation and rendering require a verified live RTX runtime. The observable-scoring revision uses the allocated L40S with CPU PhysX
and GPU rendering. Historical Brev results are recorded separately below.
From this folder:

```sh
uv pip install --no-deps -e .
python -m unittest discover -p 'test_*.py'
python diagnostics/demo.py /absolute/path/outside/source/new-demo
python diagnostics/audit.py /absolute/path/outside/source/new-demo/demo-trace.json
```

The full demo runner records native top and wrist H.264 videos, per-step
mechanism/scorer telemetry and `demo-result.json`. The output directory must
not already exist; the runner creates it to preserve prior evidence. The demo continues after the first qualifying score, withdraws the socket
through normal robot actions, records the exposed final assembly and checks a
live reset. Qualification, withdrawal and the final score must all pass within
4,000 actions. The current demo records native top and wrist cameras at 640×360 and 20 Hz. The registered
reference policy is `confined_ratchet_scripted`.

Dream evaluation uses the same physical scene, 480 Hz CPU PhysX, 20 Hz actions,
native 640×360 top and wrist cameras, `top_cam=demo`, zero object jitter and seed
17. The horizon is 4,000 actions (200 simulated seconds). With the private
Dream credentials configured outside this source directory:

```sh
RATCHET_TRACE=/absolute/path/run/mechanism.jsonl \
python diagnostics/record.py run --embodiment yam_confined_ratchet \
  -E task=task.py -E top_cam=demo -E device=cpu -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 -E object_jitter=0 \
  --policy dream -P model=dreamlabs/dream-1-beta \
  --instruction "Pick up the ratchet from its cradle, fit the socket onto the started bracket bolt, then tighten it with short back-and-forth strokes between the walls until the bolt is seated." \
  --seed 17 --max-steps 4000 --scorer task_score \
  --store-frames --no-rerun --no-prompt --grader none --log-dir /absolute/path/run
```

After Dream finishes, use its final `adhoc_*.json` result file (not the
`.live.json` progress file) to export the native streams and replay its telemetry:

```sh
inspect-robots video /absolute/path/run/adhoc_RUN_ID.json --out /absolute/path/run/videos
python diagnostics/audit.py /absolute/path/run/mechanism.jsonl
```

On the cluster, run the following **on the allocated L40S compute node** using the
existing Apptainer launcher. Keep the isolated task and output paths on a
filesystem bound into that container. `PYTHONPATH` selects this isolated task
without replacing another checkout's installed package:

```sh
RUNTIME=/path/to/robocurve-runtime
TASK=/path/to/revision/task
OUT=/path/to/revision/output/demo-A
cd "$TASK"
"$RUNTIME/cluster/run-arena.sh" env PYTHONPATH="$TASK" \
  python -m unittest discover -p 'test_*.py' -v
"$RUNTIME/cluster/run-arena.sh" env PYTHONPATH="$TASK" OMP_NUM_THREADS=8 \
  python diagnostics/demo.py "$OUT"
"$RUNTIME/cluster/run-arena.sh" env PYTHONPATH="$TASK" \
  python diagnostics/audit.py "$OUT/demo-trace.json"
```

Repeat with a new output directory. Only after reviewing a successful complete
demo, run the Dream command above through the same launcher and source path.
`diagnostics/record.py` records mechanism/scorer telemetry without changing
policy actions or scoring. Keep private Dream credentials outside the task and
all output artifacts.

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

The current-state score uses axial advance from the fixed **2.5 mm initial
under-head gap**: 0.2 at **0.3125 mm** advance (remaining gap ≤2.1875 mm),
0.4 at **1.25 mm** advance (gap ≤1.25 mm), 0.7 for actual head/bracket
seating, and 1.0 after a continuous **0.5-second** seated hold. The partial milestones equal a quarter turn and one turn at the
1.25 mm pitch. Scores can decrease when progress or seating is lost. Thread
engagement and integrity are required for every nonzero score; pushing an
unthreaded bolt down earns zero. Torque and productive cycle counts remain
recorded diagnostics and do not independently change the score. The confined
walls still require short physical strokes.

The numerical seating validator uses
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

Seating must persist for 0.5 seconds, whether the socket is engaged or withdrawn. Every 20 Hz control
interval must end in accepted contact, contain accepted contact on at least
90% of its 480 Hz physics steps, and have at least 1 mN mean normal seating
force. No-contact steps count in the denominator; old contact samples are
cleared every control interval. Loss of the seated condition removes seating
credit and resets the hold. Socket removal alone does not change the score. If extraction disturbs contact,
a fresh continuous 0.5-second seated hold is required to regain full credit. The scorer also rejects nonfinite states,
thread-constraint failure (including pitch error above 0.2 mm) and fixture
displacement above 2 mm. Contact torque is recorded from physical contact
impulses; no torque cap or calibrated torque target is part of scoring.

For a physical trial, measure the initial head-underside/bracket-top gap with
a caliper or feeler gauges and set it to 2.5 mm. At the end, measure the same
gap; subtract final from initial to obtain the axial advance for partial
credit. Confirm that the bolt remains threaded and the fixture remains fixed.
For seating credit, inspect the exposed interface from the side after
withdrawing the socket: the head must bear on the bracket without a remaining
clearance. A thin feeler gauge helps reveal a gap; an obscured camera view alone
does not prove contact. Video must show the continuous 0.5-second seated hold.
The simulator's micrometre bounds, contact duty and impulse thresholds are
numerical validation checks for this physical outcome, not extra sensors
required for a physical trial. The task seats a started bolt; it does not
certify service torque or clamp preload.

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

## Results and media provenance

The observable-scoring revision was validated on November 14, 2026 using an
allocated L40S, CPU PhysX at 480 Hz, 20 Hz control and seed 17. All 42 offline
scorer, contact, mechanism and asset tests passed.

| Current trial | Final score | Actions | Simulated time | Wall time |
| --- | ---: | ---: | ---: | ---: |
| Demo (`demo-022`) | 1.0 | 3,150 | 157.50 s | 962.99 s |
| Repeat (`demo-023`) | 1.0 | 3,150 | 157.50 s | 969.98 s |
| Dream-1 (`dream-001`) | 0.0 | 4,000 | 200.00 s | 683.39 s |

Both demos completed physical pickup, socket engagement, confined strokes,
withdrawal, final seating and a live reset. A short contact disturbance during
extraction reduced the score; full credit returned only after a fresh continuous
0.5-second contact hold, then remained through the exposed final state. The first
demo finished with a 0.054 µm measured centre gap,
100% accepted contact duty, and the tool 185.4 mm
away in the horizontal plane. These are numerical validation results, not a
claim of verified clamp preload.

Dream ran only after the successful demo review and repeat, from the same frozen
runtime source and assets. The ratchet was displaced from its cradle onto the
table beside the fixture. No socket engagement or seating occurred before the
action limit.

All trials report environment revision
`6d9758f67fe4ec79c2c222fa2be82daeb5416c3ef4b1296b010574c76435cb42`.
`validation.json` records the 48 runtime source/asset hashes, runtime versions,
per-trial scores and timings, failure-case checks, complete evidence hashes and
review scope. Runtime was Python 3.12.12, Isaac Sim 6.1.0.0, IsaacLab 17.0.2,
Arena 0.3.0, Inspect Robots 0.60.0, yam-arena 0.1.0 and dreamlabs-client 1.0.14;
the NVIDIA driver was 580.159.04. The adapter hard-codes `0.3.1` in its
environment label; the installed Arena distribution was verified as `0.3.0`.
Pinned source commits are in the validation record. The tests of the old gap,
missing/intermittent contact, penetration,
unthreaded pushing, fixture movement and diagnostic independence use unit tests
and recorded-trace counterfactuals; they are not separate physical fault trials.

Native top and wrist videos were decoded in full and visually sampled throughout
the complete recordings, with denser extraction review and native final frames.
This is not an assertion that every video frame was visually inspected. The
camera resolution cannot prove micrometre contact; independent geometry and
contact telemetry supply that check. Reproduction commands are above.

The `task.json` media ID **M-RPDV89** contains the current `demo-022` top-camera
video, matching `dream-001` top-camera video and authentic starting-layout
screenshot. These exact reviewed files were uploaded on November 14, 2026;
their SHA-256 hashes and upload provenance are recorded in `validation.json`.
The native wrist recordings, repeated demo and sanitized telemetry remain in
the local review-media archive, outside this Git task folder.

Historical results remain in `validation.json`. The November 12 seating
revision (`seated-contact-001`) scored 1.0 in 2,458 actions, with corrected
head/bracket contact but the former torque/cycle scoring gates and no socket
withdrawal. Its matching Dream (`seated-dream-001`) scored 0.0 in 4,000 actions;
these are the runs linked by M-KHDNNB. The older `rest-demo-002` scored 1.0
under the superseded clearance criterion at a 0.1961 mm gap, which did not
establish seating. Its Dream also scored 0.0; M-KGU6MH is the older media ID.
These runs are provenance, not evidence for the current implementation.

## Response to the revision points

- **Observable scoring:** partial credit is measured axial advancement from a
  reproducible 2.5 mm initial gap. Torque and cycle counts remain available for
  diagnosis, without a torque invalidation or minimum-cycle scoring gate. The
  physical procedure uses initial/final gap measurements and exposed side
  inspection; it makes no service-torque or clamp-preload claim.
- **Actual seating:** the exact head/bracket contact-pair filtering, contact
  duty, force, surface geometry and penetration checks remain in place. Full
  credit requires a continuous 0.5-second seated hold. Contact loss removes
  qualification; a fresh hold can qualify the exposed seat after withdrawal.
  Thread-integrity and fixture checks continue to reject invalid shortcuts.
- **Matching evidence:** the current demos record physical pickup, engagement,
  confined strokes and socket withdrawal. Their matching Dream trial uses the
  same frozen runtime source and assets. Current results and hashes are recorded
  separately from the historical hosted media. Media **M-RPDV89** supplies the
  matching current demo, Dream video and setup screenshot.
