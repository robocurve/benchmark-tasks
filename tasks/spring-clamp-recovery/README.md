# Release a spring clamp before pulling

Free the screwdriver and hold it over the green marker with the left gripper 15 cm above the tabletop, without dropping it or moving the clamp base.

Release a tool from a spring jaw while retaining the tool and keeping a loose fixture in place. Opening the jaw and counterholding are possible strategies, not mandatory action sequences.

The fixed scene, numerical score and reset are specified in `task.json`, `pilot.json` and `scene_inventory.json`. `setup.png` is the actual initial overview. The physics, controller trajectory, appearance assets and scoring implementation are byte-identical to the evaluated version; `evaluated-source-hashes.json` records those files. Submission documentation and the self-contained runner packaging are new. Asset licenses and attribution are retained under `assets/` and `SOURCES.json`.

## Recorded trials

Dream-1 beta scored **0.0** after 1200 controls (60 simulated seconds, 343.0 seconds measured wall time). The scripted demonstration scored **1.0** after 296 controls (14.80 simulated seconds, 115.6 seconds measured wall time including startup and rendering). Both use seed 0 and this fixed starting layout. `evaluation.json` records versions and evidence hashes. A single run is not a reliability estimate.

## Run and score

Use Linux, an NVIDIA GPU and Python 3.12 with Isaac Sim 6.1.0.0, Isaac Lab 17.0.2, Isaac Lab-Arena 0.3.0, inspect-robots 0.60.0, inspect-robots-dream 0.1.0 and dreamlabs-client 1.0.14. The included `runtime/pyproject.toml` pins yam-arena to commit bd62771e60320433893bd1d2df0fc79b5ddcd6d1. Use a separate environment for each submitted task's bundled runtime. Install uv and FFmpeg. Exact native dependency source revisions are recorded in `dependency-revisions.json`; the checked dependency trees had no tracked modifications. Review and accept NVIDIA's applicable license yourself before setting `OMNI_KIT_ACCEPT_EULA=YES`; the runner does not accept it for you.

From the repository root, in that environment:

```bash
uv pip install --no-deps -e tasks/spring-clamp-recovery/runtime
python tasks/spring-clamp-recovery/run_demo.py tasks/spring-clamp-recovery /tmp/spring-clamp-recovery-demo --policy recovery --seed 0
```

Use a fresh output path outside the source folder. The output contains `receipt.json` (final score, steps, versions, host/GPU and source verification), `trace.jsonl` (20 Hz actions and measurements), three H.264 camera videos, and the initial camera images. A process exit code alone is insufficient: require receipt status `completed`, final score 1.0, no reset/video errors and `source_unchanged=true`. The scripted policy has privileged pose access and sends only robot joint/gripper commands after reset.

For Dream-1, configure the Dream client with your own credential outside the repository. Run:

```bash
inspect-robots run --embodiment resist_isaac --policy dream \
  -P model=dreamlabs/dream-1-beta -P record=false \
  -E task=tasks/spring-clamp-recovery -E condition=recoverable -E object_jitter=0 \
  -E control_hz=20 -E cam_width=640 -E cam_height=360 -E headless=true -E device=cuda:0 \
  --instruction 'Free the screwdriver and hold it over the green marker with the left gripper 15 cm above the tabletop, without dropping it or moving the clamp base.' --scorer task_score --seed 0 --max-steps 1200 \
  --store-frames --no-prompt --grader none --no-rerun --no-rerun-save --fail-on-error 1 \
  --log-dir /tmp/spring-clamp-recovery-dream
```

The scorer is the task's native `evaluate` method plus the one-second hold in `runtime/resist_isaac/runtime.py`; the Inspect `task_score` scorer reports that value. Default policy guardrails remain enabled. No task objects are teleported, attached to hands or driven by a scripted object controller after reset.

## Measurement and limits

For each moving body origin, zero is latched if z < 0.7425 m, z > 1.4 m, |y| > 0.65 m, or x <= -0.19 m or x >= 0.91 m. Zero is also latched if clamp-base origin displacement from (0.35,-0.04,0.9) exceeds 0.025 m, base tilt exceeds 0.20 rad, tool downward speed exceeds 0.35 m/s, or tool origin z < 0.7725 m. Otherwise 0.5 when the tool origin is within 0.08 m of the left TCP and its complete conservative bounding box (tool-local X -0.068 to 0.140 m, Y and Z +/-0.018 m) clears the clamp: in clamp-base coordinates every corner has X < -0.065 m, or every corner has Y > 0.05 m, or every corner has Y < -0.05 m. Full success additionally requires tool translation speed < 0.02 m/s and left TCP within 0.025 m of (0.35,0.36,0.9). Checks run at 20 Hz. Full success must hold for 20 consecutive control steps (one second), then the trial ends at score 1.0. Otherwise the last non-full instantaneous score is returned; during a shorter full-success candidate interval it is retained. Any latched violation returns zero. All remaining outcomes score zero. Origins and axes are specified in scene_inventory.json; strict inequalities exclude their boundary.

For position and orientation checks, use a ruler/grid and perpendicular calibrated views of the body origins and local axes; a protractor gives tilt and hinge angle. World Z is measured from the station frame, not from the tabletop. TCP means the gripper tool center defined by the pinned YAM model. Linear speed is a displacement/time measurement; angular speed is angle/time. A 20 Hz video gives 50 ms observation intervals, but finite differences are an approximation to the instantaneous native velocities used by the simulation. Decisions close to speed thresholds cannot be certified from a single still image or low-resolution footage. This measurement-equivalence limitation remains explicit rather than treating a damage proxy as observed damage.

The scene uses intact rigid bodies and passive joints. Alternative robot-only strategies are allowed if they satisfy the score; the demonstration is not a required sequence. No fracture or electrical hazard is modeled, and no physical safety certification is claimed. Real-world validation is outside the present preparation scope.
