# Transfer a loaded fixture pallet with a forked lifter

Insert a removable two-tine lifter beneath a loaded workholding pallet, lift it, transfer
it to two inspection rests, lower it, withdraw the fork and return the tool. The pallet
remains a separate free rigid body supported by contact and friction throughout the carry.
The loaded blank is a rigid part of the pallet; this task does not assemble or machine it.

The revised low-clearance simulator demonstration, `low-clearance-003`, scored
**1.0 in 997 actions** (49.85 simulated seconds), terminated with **success**,
and took **115.30 seconds wall time** on an L40S. Its maximum sampled pallet
bottom was **29.26 mm** above the table. During the 22–29 s carry, the sampled
bottom was 24.82–29.26 mm, at least **9.82 mm above the 15 mm rails**. The pallet
remained seated through fork withdrawal and tool return, with a 20-step success
hold. The 20 Hz samples do not exclude height excursions between samples.

All 998 reset/control records reproduce the current scorer with no mismatch;
the original scorer returns 0 throughout the same recording. Three native
H.264 camera videos are retained. The reviewed montage contains top views at
0/24/36/49 s and wrist views at left36/right49 s; the top samples are consistent
with transfer, placement and return, while the wrist samples are largely
occluded. This is sampled visual review, not a frame-by-frame video review or
real-world physical validation. The controller still uses privileged poses/FK.

The matching fresh Dream-1 run scored **0 in 1,800 actions** (`max_steps`), with
identical runtime/asset hashes. Its three videos have scoped parent montage
review, described below. The original
957-step demo and original Dream run remain historical; the unchanged linked
media use the original scorer. `validation.json` preserves those records and
adds separate revised evidence. All 20 CPU tests pass.


## Reproduce

From this folder in the configured YAM Arena RTX environment:

```bash
uv pip install -e . --no-deps
python -m unittest discover -s . -p 'test_*.py'
export TRIAL_OUTPUT=/path/outside/repository/pallet-low-clearance-demo
inspect-robots run --embodiment yam_pallet_lifter_validation --policy pallet_lifter_low_clearance \
  -E task="$PWD/task.py" -E telemetry_path="$TRIAL_OUTPUT/control-steps.jsonl" \
  -E object_jitter=0 -E device=cpu -E top_cam=demo \
  -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction 'Use the forked lifter to move the loaded pallet onto the two green inspection rests. Withdraw and return the lifter to its supports, then open both grippers.' \
  --scorer task_score --max-steps 1800 --seed 0 --store-frames \
  --log-dir "$TRIAL_OUTPUT/inspect" --no-prompt --grader none --no-rerun
```

Activate the configured Arena environment before installation. On Brev this is
`source ~/IsaacLab-Arena/yam.env`; CARC uses its documented allocation launcher.
Render the terminal JSON with `inspect-robots video <run.json>`. The model trial
uses the same validation embodiment, task, scene, seed, cameras and limit with
`--policy dream -P model=dreamlabs/dream-1-beta`, and a fresh `TRIAL_OUTPUT`.
The offline suite needs NumPy and `usd-core` (the `pxr` module), but does not launch
Isaac Sim or use a GPU. The original `yam_pallet_lifter` and
`pallet_lifter_scripted` entry points remain available for the historical
controller; revised results use the command above.

## Fixed setup and reset

World x points forward, y left, z up. Tabletop z=0.7475 m. `task.py` lists starting poses;
`build_assets.py` regenerates self-contained assets with usd-core. Lengths below are mm.

- The loaded pallet is one 120 g rigid body. A 120 × 160 × 6 plate carries a central
  40-diameter × 12 round blank, decorative grooves and fasteners. Four 18 × 14 × 20 feet
  are centered at local x=±45, y=±70, z=−13. Root starts at (0.40,0.08,0.7705) m,
  with feet resting on the table and 20 mm of underside access above the table.
  The reference centroid is 3.8 mm below the root due to asymmetric vertical bounds.
- The 80 g steel lifter has two 150 × 12 × 5 tines centered at local x=95, y=±30,
  z=2.5; a crossbar, handle and rigid 35 × 24 × 18 grip sleeve join them. The sleeve
  center is 100 behind the root and 16 above it. Root starts at (0.24,0.28,0.7825) m.
  Its local-bounds reference center is offset (17.5,0,12.5) from the root.
- Two fixed tool supports at (0.18,0.28) and (0.34,0.28) m have heights 40 and 35.
- Two fixed inspection rails are centered at x=0.37, y=−0.19 and −0.05 m. Each is
  150 × 18 × 15, with green visual seat marks and end fasteners. The open central
  space lets the fork lower clear and withdraw after the pallet's feet contact rails.

Reset restores the pallet, lifter and both arms, clears contact/failure history and
settles physics. Seed 0 and zero jitter reproduce the demonstrated arrangement.
Nothing is consumed. Masses and materials represent lightweight training fixtures.

## Numeric scoring

Use the final score. PhysX must report lifter–pallet contact; without that contact,
score zero. There is no minimum historical lift height for a valid seated outcome.
The `History.lifted` flag now reports observed rail clearance for diagnostics only;
it is not a prerequisite for seating or scoring.

A contact-verified pallet earns 0.75 when its reference xy is within 12 mm Euclidean distance of
(0.37,−0.12) m, its bottom is within 4 mm of tabletop+15 mm, full 3-D orientation is within
8° of identity, vertical extent is below 42 mm, and it rests (linear speed under 10 mm/s,
angular speed under 0.5 rad/s). Otherwise it earns 0.25 only while its entire world
AABB is strictly above tabletop+15 mm, and zero elsewhere. This partial-credit
clearance is the actual collision rail height, not an added lift requirement for
seating. A misplaced or flipped pallet at or below that height earns zero;
above it, only 0.25 is available unless all seating conditions hold.

The collision rails span local z=0 to 15 mm. Their visual marks and fasteners
reach 16.6 mm but have no colliders. Pallet feet extend to local z=−23 mm and
align with the rails at y=±70 mm; the plate underside is at z=−3 mm. A level carry
with feet 20 mm above the table therefore clears the rails by 5 mm, and the
5 mm tines fit the 20 mm underside channel. The CPU suite reads the checked-in
USD colliders and exercises low-clearance transfer/contact history, rail-height
seating with no recorded clearance peak, both sides of the seating tolerance,
the old high-lift outcome, and permanent invalid-contact/drop histories.

Full score additionally requires the lifter returned, both grippers over 75% open,
and both TCPs over 90 mm from both lifter and pallet reference centers. Parking means
lifter xy within 25 mm of (0.2575,0.28) m, bottom within 6 mm of tabletop+35 mm, vertical
extent below 35 mm and rest. Holding 1.0 for one second terminates successfully.

Any robot–pallet contact permanently invalidates, as does a lifter/pallet bottom below
tabletop minus 20 mm, nonfinite position, center outside x=(−0.1,0.95) m, or |y|>0.62 m.
Invalid trials score zero. No score maximum is retained. The tested paths include both
scene names and the actual `LeftArm`/`RightArm` PhysX actor paths.

## Physics and limitations

PhysX runs at 240 Hz with 20 Hz robot actions. The lifter and pallet use 32/4 solver
iterations, 0.5 mm contact offset, zero rest offset, and approximate station 1.2/1.0
static/dynamic friction. Robot colliders and drives are unchanged. Metal/polymer looks,
mass distribution and friction are stylized and uncalibrated; neither tool nor pallet
flexes. There are no attachments, welds, pose writes or scripted payload motion.

The demonstration uses privileged object poses and FK for grasp and payload feedback.
Measured payload slip corrects subsequent robot placement commands; lowering and fork
withdrawal preserve that correction. The historical controller uses fixed
+8 mm x/−10 mm y placement compensation. The low-clearance variant retains
the +8 mm x adjustment and removes the −10 mm y adjustment; scoring targets
remain fixed. The policy sends robot joint targets only. The
model receives ordinary images and joint state through the same contact-enabled embodiment.

YAM source `0059d02a1a91bc6834c1d0846022891df0c78730`, Arena source
`c7edf3a9d5d8276565208f72422f8f8a0806d6cf`; Python 3.12.12, Isaac Sim 6.1.0.0,
Isaac Lab 17.0.2, Arena 0.3.0, Inspect Robots 0.60.0, NumPy 2.3.1, Torch 2.11.0+cu128,
NVIDIA L40S. These are the historical run versions. Their source hashes and media
validation remain unchanged in `validation.json`; the separate revision entry
records the initial offline phase. `gpu_revision_2026_11_12` adds the revised
demo, scorer comparison and matching Dream result, preserving all older entries.
The revised Brev runtime uses Python 3.12.16, Isaac Sim 6.1.0.0, Isaac Lab
17.0.2, Arena 0.3.0 and Inspect Robots 0.60.0, with the same YAM/Arena source
commits listed above. Full task runtime/asset hashes match between the new demo
and Dream; metadata edits do not change the frozen runtime.

## Low-clearance demonstration and evidence scope

### Separate low-clearance controller

`pallet_lifter_low_clearance` retains the existing privileged simulator-pose/FK
feedback and changes only the two lift/carry TCP waypoint heights from 106 to
71 mm above the table. Level contact geometry predicts a 40 mm pallet bottom,
25 mm above the rails. These are nominal targets, not measured results.
Pickup, insertion, return and release waypoints are preserved. Following the
second diagnostic, destination y is shifted from -130 to -120 mm throughout
carry/lowering/withdrawal, and the fork-clear and withdrawal TCP targets are
23 mm instead of 31 mm. An additional 2-second stationary hold follows fork
lowering before horizontal withdrawal. Level tines nominally top out at 12 mm,
3 mm below the seated feet; actual loaded geometry can differ.
The separate `yam_pallet_lifter_validation` embodiment inherits the current
physics and scorer. The original policy and embodiment entry points still exist.
The module's prevalidation docstring and comments remain unchanged to preserve
the exact runtime bytes used in both completed runs. This README and the new
validation entry record the completed evidence and its limits.

CPU preparation: all 20 tests pass (14 existing scorer/geometry checks plus
six validation checks). Plan tests check the stationary low hold before
withdrawal, nominal tine/foot clearance, consistent destination y, preserved
pickup/insertion/return and the action budget. Synthetic states test telemetry,
contact history, score agreement, height excursions and incomplete records;
they do not validate physical behavior. Runtime code remains frozen; only
documentation/validation metadata is being updated.
The first parent-run GPU diagnostic used an earlier 56 mm TCP target and
failed: sampled pallet bottom at lift end was 14.08 mm, then it contacted a
15 mm rail at 24.30 s and tipped. The second diagnostic (71 mm carry) reached a maximum
sampled pallet bottom of 29.26 mm and briefly scored 0.75, then failed during
withdrawal. It ended at score 0 in 1,041 actions with `policy_stop`; tool parking
and open-clear passed, but the pallet was not seated. Rail support first
appeared at 33.75 s, and withdrawal began at 35 s with persistent lifter contact
and the tool centre 15.4 mm above its target. Contact during withdrawal displaced
and rotated the pallet off the rails. These observations motivate the y shift,
lower clearance target and hold. The third run is the qualified simulator
demonstration summarized above: first rail contact at 30.45 s, both rail
reports at 31.00 s, first seated score at 31.10 s, first full score at 48.90 s,
and success after 20 full-score actions at 49.85 s. All 141 carry samples
include lifter–pallet reports. None of the 81 withdrawal samples (37–41 s)
includes lifter–pallet reports. Final pallet xy error is 5.748 mm, bottom
15.000 mm, vertical extent 38.400 mm, orientation error 5.347°, linear speed
0.181 mm/s and angular speed 0.00180 rad/s; all meet the current seating checks.
Tool parking and runtime open-clear also pass, with no invalid history.

For a scheduled reproduction, use the command above from an installed immutable
task snapshot with a unique `TRIAL_OUTPUT`. After terminal completion, summarize
the sampled telemetry:

```bash
python -m pallet_lifter.telemetry "$TRIAL_OUTPUT/control-steps.jsonl" \
  > "$TRIAL_OUTPUT/telemetry-summary.json"
```

Use one process/episode and a new telemetry path for every trial; an existing
path is rejected. `device=cpu` preserves the existing PhysX configuration;
Isaac Sim cameras still require the scheduled RTX GPU. The parent launcher
handles installation and log capture; do not launch a second process independently.

The JSONL begins with input hashes and settings, then records reset and every
control step: object world bounds, bottom height, pose, velocities and rest;
pallet contact actor paths and report sizes received since the previous sample;
sticky history flags; current score and independent current-scorer evaluation;
seating, tool parking, gripper/TCP clearance history, success-hold count and
termination. CPU audit reconstructed contact/invalid/lift history and object
predicates from the recording, with zero score/history/rest/seating/parking
mismatches across 998 samples. Open-clear was taken from the recorded runtime
boolean because individual measured gripper openings and TCP distances are
not retained; those thresholds cannot be independently recomputed from this
recording. Contact reports are events, not a continuous support-force trace.
Heights are sampled at 20 Hz, not at each 240 Hz physics substep. A sampled
maximum below 60 mm cannot exclude an excursion between control samples.
The summary describes sampled evidence; it is not a physical-success verdict.
Review the matching terminal JSON, telemetry and videos together, including
actual rail clearance, contact-supported carry, stable seating, tool return
and release. Do not equate a score with proof of physical feasibility.

The recorded third-run states and contact events, recomputed through the original
submitted scorer, score zero at every sample because its 60 mm lift gate never
opens. This comparison uses the new low-clearance recording; it is not a rescore
of the historical videos. The current scorer returns the recorded final 1.0.
Runtime robot-contact and drop invalidation have CPU regressions, but were not
deliberately induced in these GPU trials. Contact actor identities and sampled
visual evidence do not establish continuous support forces or real-world safety.

The historical terminal JSON files contain aggregate scores and action-file
references, not per-step object bounds and contact events. They cannot be rescored
directly. Compare old/new scores if a complete state/contact recording can be
recovered. The comparison above uses the fresh low-clearance trajectory.
No revised-scoring score is claimed for old media.
The scripted policy still uses the privileged feedback described above.

## Dream-1 evaluation

The matching fresh `low-clearance-dream-001` used the same validation embodiment,
task runtime/assets, instruction, cameras, seed and limit as the new demo.
`dreamlabs/dream-1-beta` finished at score **0** after **1,800 actions**
(`max_steps`, 90 simulated seconds, **193.30 s wall time**). All 12 recorded
runtime input hashes match the demo. Its 1,801 reset/control records show no
lifter–pallet contact or invalid history; the pallet remains at its source.
Three native H.264/yuv420p, 640×360, 20 Hz videos, each 90.05 s long, are retained
with exact file hashes in `validation.json`. The parent reviewed top views at
0/30/60/89 s and both final wrist views at 89 s: the left arm approaches,
lifts and moves the fork from its supports, while the pallet stays at its
source and no transfer to the rails occurs. This is sampled montage review;
no frame-by-frame review of all video frames is claimed.
The scripted controller's privileged pose feedback is not an input to Dream.

Historically, `dreamlabs/dream-1-beta` completed with a final score of **0** in **1,800 steps**,
terminating with `max_steps`. Wall time including initialization and inference
was **204.61 seconds**. The run used seed 0, zero jitter, 20 Hz control, 640 × 360
saved camera frames, the `demo` top-camera configuration and the same runtime source
hashes and physics as the expert demonstration. The model received ordinary camera images
and joint state. All three complete camera videos were downloaded and visually reviewed.

The reviewed sequence shows the left arm reaching around the fork and loaded pallet, while the lifter remains on its supports and the pallet stays in its starting position. No contact-verified lift or transfer is completed. Final history has no lifter-pallet contact and no verified lift. The top sequence, final frame and both wrist views were inspected.

`validation.json` preserves the completed original result, runtime revision and
video hashes. The historical Dream video, demo and setup image remain linked
through the unchanged media ID in `task.json`; the new runs have not been uploaded.
