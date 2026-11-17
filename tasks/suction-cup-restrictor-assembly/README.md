# Install a plug-in vacuum cup and its flow restrictor through a mounting plate

Install the green suction cup through the plate and fit the white flow restrictor. Leave both installed and release them. Keep the arms clear of each other and the table.

Assembly of a vacuum robot end effector includes mechanically retaining its plug-in cups and flow restrictors. The compliant head must pass through the mounting hole and recover, while a separate ribbed core must seat from the opposite face without displacing the cup. This custom subassembly ends before vacuum commissioning.

These are dimensioned custom engineering parts. The robot demonstration is a
known-layout script using private object poses and ordinary original joint
commands. It does not teleport parts, change the scene during the episode, use
proximity latches, or apply scripted forces to the parts. Dream receives only the
original three RGB streams and joint state. No scripted controller or private
body-pose getter is installed for Dream.

## Reproduce

Use the original simulation walkthrough runtime and the exact source pins and
package versions in runtime-lock.json. `python runtime_check.py` verifies the
installed Python3.12 environment without starting a simulator or hosted client.
The task is portable within that pinned environment; no parent-workspace imports
or untracked external meshes are required. Do not change the original YAM rig,
robot drives, joint limits, cameras or action settings.

From this task folder, using the pinned runtime Python:

```bash
python runtime_check.py
python run.py --mode setup --steps 40 --gpu auto --log-dir /tmp/suction-cup-restrictor-assembly-setup
python run.py --mode demo --gpu auto --log-dir /tmp/suction-cup-restrictor-assembly-demo
python validate_run.py /tmp/suction-cup-restrictor-assembly-demo
python export_media.py /tmp/suction-cup-restrictor-assembly-demo /tmp/suction-cup-restrictor-assembly-media --key-frame 820
python run.py --mode dream --config-only --log-dir /tmp/suction-cup-restrictor-assembly-dream-config
```

The demo uses 1060 actions / 53 simulated seconds. The model limit is
2,400 actions /120 simulated seconds, leaving time for exploration/retries. One
control action is0.05s. Startup, rendering and inference latency add wall time. On the shared4090 the
full demo took about30 minutes wall time. Process limits are45 minutes for free
runs and120 minutes for Dream; action/simulation-time limits stay unchanged.
Use a new output directory each time. The wrapper validates the official report,
all-substep scorer replay, native constraints/travel, contact coverage, rendering
and conservative numerical-anomaly screens; CLI exit0 alone is insufficient.

`--config-only` does not load credentials, start a simulator or call a model.
A real model run is paid and transfers inference images/state; run it only with
account credit and explicit authorization:

```bash
python run.py --mode dream --gpu auto --log-dir /tmp/suction-cup-restrictor-assembly-dream
```

Dream uses `dreamlabs/dream-1-beta`,48 rows/request,32 preroll steps,record=false,
dry_run=false. It runs the same scene/scorer and120s ending-state budget; no demo
termination delay or target pose is provided to the model. record=false disables
extra cloud recording, not inference input transfer. The formal Dream-1 run scored 0.0 after all 2,400 actions / 120s, with 50 returned chunks and 99 SDK attempts (49 retry attempts). Three original streams contain 2,401 frames each. Archived-source scoring replay, contact coverage and native constraints pass. Its initial camera images match the full-score demo. This is one trial, not a success-rate estimate. The formal run used seed 0 with object_jitter=0 and took 55.1 minutes wall time on the shared server. The media ID is filled by the official uploader before publication.

The automatic GPU selector requires8GiB free by default; the shared-server free
checks may use `--min-free-mib 6144`. It never reserves GPUs or stops other work.
The runtime records all physics/contact samples and original camera frames. The
image-health guard stops grey/invalid streams. Never relabel a failed render as a
model failure.

## Scoring and physical reproduction

See SCORING.md for all tiers, measurement conventions, allowed symmetric poses,
release/support/retention and safety rules. PHYSICAL_SETUP.md specifies materials,
geometry, masses, passive joints, fixed placement and fidelity boundaries.
The primary outcome is installation; no particular arm, grasp, camera movement
or sequence is required. Free feasibility does not establish a visual policy or
material-strength rating. The part geometry is retained throughout the episode.

## Assets and licensing

All added parts are procedural geometry authored for this task. No Spot,
Schmalz or third-party product mesh is included or claimed. The pinned official
station and YAM assets remain upstream dependencies under their original license
files; this folder does not redistribute them. The project/source URLs and exact
commits are in runtime-lock.json. No purchased asset or external download is
required for the custom parts.

License inventory for upstream assets: I2RT YAM meshes/rig assets use MIT
(`yam_arena/assets/yam/LICENSE.i2rt`); IsaacLab-Arena uses Apache-2.0 and IsaacLab
BSD-3-Clause; inspect-robots-dream uses MIT. Isaac Sim is used under NVIDIA's
software license supplied by its official installation. No upstream meshes are
copied into this task. Preserve those upstream notices when installing the rig.

Conservative force/speed screens gate scripted feasibility evidence. For a model
run, a fast fall or impact alone is flagged for review and does not erase a valid
policy failure. Missing data, broken constraints or bad images still invalidate
the recording. Inspect flagged dynamics before claiming model full-score feasibility.

## Recorded validation

The complete scripted trajectory scores 1.0 in 1060 actions / 53s under the final scorer. Its original physical recording is preserved; the final scorer was replayed over every original physics substep. A copied-folder 40-action startup has identical initial images and physical arrays to that recording. This short reset check is not an additional full demonstration. Final geometry, robot controls and scripted plan match the recorded feasibility run. The later formal Dream run uses the final package implementation; the current outcome scorer was subsequently replayed on the complete unchanged physical and model recordings, as described below.

## Outcome and fabrication revision

The outcome now checks that installation, physical support and robot release
persist for three seconds, without nominal pose or microscopic pose-drift limits. Actual hook/shoulder/rib capture and solid-part fit determine installation. Native joint
integrity and anti-penetration guards remain part of simulation validation.
SCORING.md separates practical grading from numerical contact allowances.

The original complete full-score and Dream trajectories were regraded with this
revision. Geometry, reset, policy instruction, controls, cameras and scripted plan
are unchanged; this is not a new Dream trial. Manufacturing files describe a
workshop counterpart of the disclosed compliant surrogate. See fabrication/README.md
for dimensions, materials, connections, tolerances and qualification steps. They
are not OEM product CAD or measured material/load certification.

Offline rubric checks (no simulator/model): `python check_scoring.py`.
