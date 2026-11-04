# Unload the supporting hand

Park the left gripper pointing down over the green pad and leave both books flat on the blue mat without dropping them.

Unload two books from a supporting hand before parking that hand. A valid solution preserves both books and places them flat; no particular unloading sequence is required.

The fixed scene, numerical score and reset are specified in `task.json`, `pilot.json` and `scene_inventory.json`. `setup.png` is an additional native view of the fixed reset, chosen to expose both books. Only the camera moved after reset; book poses and velocities remained unchanged and no physics steps were taken. It was not supplied to the evaluated policy. `setup-policy-overview.png` preserves that policy's original overview; `setup-view.json` records the documentation camera pose. The physics, controller trajectory, appearance assets and scoring implementation are byte-identical to the evaluated version; `evaluated-source-hashes.json` records those files. Submission documentation and the self-contained runner packaging are new. Asset licenses and attribution are retained under `assets/` and `SOURCES.json`. Asset provenance files retain historical conversion notes; `evaluation.json` and `package-verification.json` provide current validation.

## Recorded trials

Dream-1 beta scored **0.0** after 1200 controls (60 simulated seconds, 324.2 seconds measured wall time). The scripted demonstration scored **1.0** after 615 controls (30.75 simulated seconds, 668.0 seconds measured wall time including startup and rendering). Both use seed 0 and this fixed starting layout. `evaluation.json` records versions and evidence hashes. A single run is not a reliability estimate.

## Run and score

Use Linux, an NVIDIA GPU and Python 3.12 with Isaac Sim 6.1.0.0, Isaac Lab 17.0.2, Isaac Lab-Arena 0.3.0, inspect-robots 0.60.0, inspect-robots-dream 0.1.0 and dreamlabs-client 1.0.14. The included `runtime/pyproject.toml` pins yam-arena to commit bd62771e60320433893bd1d2df0fc79b5ddcd6d1. Use a separate environment for each submitted task's bundled runtime. Install uv and FFmpeg. Exact native dependency source revisions are recorded in `dependency-revisions.json`; the checked dependency trees had no tracked modifications. Review and accept NVIDIA's applicable license yourself before setting `OMNI_KIT_ACCEPT_EULA=YES`; the runner does not accept it for you.

From the repository root, in that environment:

```bash
uv pip install --no-deps -e tasks/book-load-recovery/runtime
python tasks/book-load-recovery/run_demo.py tasks/book-load-recovery /tmp/book-load-recovery-demo --policy recovery --seed 0
```

Use a fresh output path outside the source folder. The output contains `receipt.json` (final score, steps, versions, host/GPU and source verification), `trace.jsonl` (20 Hz actions and measurements), three H.264 camera videos, and the initial camera images. A process exit code alone is insufficient: require receipt status `completed`, final score 1.0, no reset/video errors and `source_unchanged=true`. The scripted policy has privileged pose access and sends only robot joint/gripper commands after reset.

For Dream-1, configure the Dream client with your own credential outside the repository. Run:

```bash
inspect-robots run --embodiment resist_isaac --policy dream \
  -P model=dreamlabs/dream-1-beta -P record=false \
  -E task=tasks/book-load-recovery -E condition=recoverable -E object_jitter=0 \
  -E control_hz=20 -E cam_width=640 -E cam_height=360 -E headless=true -E device=cuda:0 \
  --instruction 'Park the left gripper pointing down over the green pad and leave both books flat on the blue mat without dropping them.' --scorer task_score --seed 0 --max-steps 1200 \
  --store-frames --no-prompt --grader none --no-rerun --no-rerun-save --fail-on-error 1 \
  --log-dir /tmp/book-load-recovery-dream
```

The scorer is the task's native `evaluate` method plus the one-second hold in `runtime/resist_isaac/runtime.py`; the Inspect `task_score` scorer reports that value. Default policy guardrails remain enabled. No task objects are teleported, attached to hands or driven by a scripted object controller after reset.

## Measurement and limits

Zero is latched if either book origin z < 0.7445 m, |x-0.35| > 0.60 m, |y| > 0.64 m, or downward speed > 0.35 m/s. Otherwise 0.25 per book whose horizontal center distance from (0.34,-0.12) is < 0.105 m, |z-0.8225| < 0.008 m, local-up dot world-up > 0.98 and translation speed < 0.02 m/s. Full success requires both books placed, both TCPs more than 0.08 m from each book origin, left TCP within 0.025 m of (0.5,0.2,0.763) and left-gripper local-Z dot world-Z < -0.98. Checks run at 20 Hz. Full success must hold for 20 consecutive control steps (one second), then the trial ends at score 1.0. Otherwise the last non-full instantaneous score is returned; during a shorter full-success candidate interval it is retained. Any latched violation returns zero. All remaining outcomes score zero. Origins and axes are specified in scene_inventory.json; strict inequalities exclude their boundary.

For position and orientation checks, use a ruler/grid and perpendicular calibrated views of the body origins and local axes; a protractor gives tilt and hinge angle. World Z is measured from the station frame, not from the tabletop. TCP means the gripper tool center defined by the pinned YAM model. Linear speed is a displacement/time measurement; angular speed is angle/time. A 20 Hz video gives 50 ms observation intervals, but finite differences are an approximation to the instantaneous native velocities used by the simulation. Decisions close to speed thresholds cannot be certified from a single still image or low-resolution footage. This measurement-equivalence limitation remains explicit rather than treating a damage proxy as observed damage.

The scene uses intact rigid bodies and passive joints. Alternative robot-only strategies are allowed if they satisfy the score; the demonstration is not a required sequence. No fracture or electrical hazard is modeled, and no physical safety certification is claimed. Real-world validation is outside the present preparation scope.
