# Release a spring clamp before pulling

Free the screwdriver and hold it over the green marker with the left gripper 15 cm above the tabletop, without dropping it or moving the clamp base.

Category: **Impact on jobs**. Bench assembly and repair require releasing spring-loaded workholding while retaining a tool. Either hand may release the clamp; the screwdriver must finish in the left gripper. The task makes no claim of risk to people.

The fixed scene and numerical score are in `task.json`, `pilot.json` and `scene_inventory.json`. The reset image and standard calibrated YAM camera remain unchanged. Only robot joint/gripper commands act on the apparatus; grasp evidence is private to the evaluator.

## Revision status and evidence

**Revision prepared; fresh native validation is pending.** The earlier 80 mm tool-to-left-TCP proximity test was insufficient: a right-held tool could pass. Full credit now requires opposing loaded contacts with both exact left fingertips, with no other body supporting the screwdriver, throughout the one-second hold. A safely freed tool retains half credit regardless of hand.

`evaluation.json`, `evaluated-source-hashes.json`, `camera-verification.json` and `package-verification.json` describe the historical submission at `4fc7948`, before this scoring repair. Its Dream-1 beta trial scored 0.0 after 1200 controls; its scripted demo scored 1.0 after 296 controls. **Those are historical results, not validation of the revised scorer.** New evidence must include a successful left-hand recovery and a physical right-hand-near-left counterexample that cannot receive full credit. Missing native contact data is an invalid run, not a successful grasp.

## Run and score

Use Linux, an NVIDIA GPU and Python 3.12 with Isaac Sim 6.1.0.0, Isaac Lab 17.0.2, Isaac Lab-Arena 0.3.0, inspect-robots 0.60.0, inspect-robots-dream 0.1.0 and dreamlabs-client 1.0.14. The included `runtime/pyproject.toml` pins yam-arena to commit bd62771e60320433893bd1d2df0fc79b5ddcd6d1. Use a separate environment for each submitted task's bundled runtime. Install uv and FFmpeg. Exact native dependency source revisions are recorded in `dependency-revisions.json`; the checked dependency trees had no tracked modifications. Review and accept NVIDIA's applicable license yourself before setting `OMNI_KIT_ACCEPT_EULA=YES`; the runner does not accept it for you.

From the repository root, in that environment:

```bash
uv pip install --no-deps -e tasks/spring-clamp-recovery/runtime
python tasks/spring-clamp-recovery/run_demo.py tasks/spring-clamp-recovery /tmp/spring-clamp-recovery-demo --policy recovery --seed 0
```

Use a fresh output path outside the source folder. The output contains `receipt.json` (final score, steps, versions, host/GPU and source verification), `trace.jsonl` (20 Hz actions and measurements), three H.264 camera videos, and the initial camera images. A process exit code alone is insufficient: require receipt status `completed`, final score 1.0, no reset/video errors and `source_unchanged=true`. The fixed waypoint policy uses elapsed steps and robot URDF IK, and sends only robot joint/gripper commands after reset. `native-left-grasp.jsonl` records the actual contact actors, impulses, normals and separation for each physics frame.

For Dream-1, configure the Dream client with your own credential outside the repository. Run:

```bash
inspect-robots run --embodiment resist_isaac --policy dream \
  -P model=dreamlabs/dream-1-beta -P record=false \
  -E task=tasks/spring-clamp-recovery -E condition=recoverable -E object_jitter=0 \
  -E top_cam=calibrated -E control_hz=20 -E cam_width=640 -E cam_height=360 -E headless=true -E device=cuda:0 \
  --instruction 'Free the screwdriver and hold it over the green marker with the left gripper 15 cm above the tabletop, without dropping it or moving the clamp base.' --scorer task_score --seed 0 --max-steps 1200 \
  --store-frames --no-prompt --grader none --no-rerun --no-rerun-save --fail-on-error 1 \
  --log-dir /tmp/spring-clamp-recovery-dream
```

The scorer is the task's native `evaluate` method plus the one-second hold in `runtime/resist_isaac/runtime.py`; the Inspect `task_score` scorer reports that value. Default policy guardrails remain enabled. No task objects are teleported, attached to hands or driven by a scripted object controller after reset.

## Measurement and limits

Zero is latched if any moving-body origin leaves -0.19 < x < 0.91 m, |y| <= 0.65 m, 0.7425 <= z <= 1.4 m; clamp-base displacement from (0.35,-0.04,0.9) exceeds 25 mm or tilt exceeds 0.20 rad; or tool downward speed exceeds 0.35 m/s or origin z < 0.7725 m. Otherwise 0.5 for a freed tool, independent of hand: its complete tool-local bounding box X [-0.068,0.140] m, Y/Z +/-0.018 m must clear the jaw. In clamp-base coordinates all corners must have X < -0.065 m, or all Y > 0.05 m, or all Y < -0.05 m. Full score also requires left TCP within 25 mm of (0.35,0.36,0.9), tool speed < 0.02 m/s, and an isolated left-gripper pinch for 20 consecutive control steps (one second). At every 240 Hz physics sample, both exact left fingertip bodies must contact the tool with normal impulse magnitude > 1e-9 N s, separation <= 0 and an opposing normal pair with dot product <= -0.5; any contact row from another body disqualifies the pinch. Missing or invalid native data invalidates measurement. Right-held, one-finger, externally supported or merely nearby tools cannot earn full credit. A shorter full-score candidate interval retains the previous non-full score; otherwise report the current 0.5 or 0. Any latched violation overrides completion. Remaining outcomes score 0. Frame definitions are in scene_inventory.json.

For position and orientation checks, use a ruler/grid and perpendicular calibrated views of the body origins and local axes; a protractor gives tilt and hinge angle. World Z is measured from the station frame, not from the tabletop. TCP means the gripper tool center defined by the pinned YAM model. Linear speed is a displacement/time measurement; angular speed is angle/time. A 20 Hz video gives 50 ms observation intervals, but finite differences are an approximation to the instantaneous native velocities used by the simulation. Decisions close to speed thresholds cannot be certified from a single still image or low-resolution footage. This measurement-equivalence limitation remains explicit rather than treating a damage proxy as observed damage.

The scene uses intact rigid bodies and passive joints. Alternative robot-only strategies are allowed if they satisfy the score; the demonstration is not a required sequence. No fracture or electrical hazard is modeled, and no physical safety certification is claimed. Real-world validation is outside the present preparation scope.

## Submission camera

The submitted video and setup image use `yam_arena.station.top_camera("calibrated")`, the fixed standard YAM top camera. Wrist-camera recordings are supplementary and are not the uploaded main video. See `camera-verification.json` for the camera calibration and evidence association.

## Third-party assets and licenses

| Included asset | Source and modifications | License |
| --- | --- | --- |
| `assets/ycb044_handle.obj` | YCB **044 flat screwdriver** by Berk Calli, Arjun Singh, Aaron Walsman, Siddhartha Srinivasa, Pieter Abbeel and Aaron Dollar. The original wide blade was removed and handle coordinates/normals normalized; original texture coordinates remain. | [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/); [dataset license](https://ycb-benchmarks.s3.amazonaws.com/index.html), [local attribution](assets/LICENSE.ycb.md) |
| `assets/texture_map.png` | Original YCB 044 screwdriver texture, by the same authors, applied to the derived handle. | [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/); [local attribution](assets/LICENSE.ycb.md) |

The imported YAM robot meshes, convex collision hulls and URDF are I2RT assets under **MIT**, delivered by the pinned `yam-arena` dependency; see its [I2RT license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/LICENSE.i2rt). The upstream adapter itself declares no repository-wide license at this revision; the robot-model MIT license does not relicense that software. The station table, floor and camera stand are procedural dependency geometry, not additional downloaded object meshes.

These are the only third-party appearance files bundled with this task; `SOURCES.json` records their hashes. The narrow replacement blade, shaft, clamp, release paddle, support block and marker are authored nominal geometry under [MIT](LICENSE). The YAM robot and calibrated station are supplied by the pinned runtime dependency, not copied into this task's asset folder. `assets/UPSTREAM_PROVENANCE.json` is a historical parent-task manifest: its KiCad connector entries are not used or shipped here. No manufacturer dynamics or commercial clamp calibration is claimed.

In a physical replay, observe both left fingers pinching the handle and every other support clear during the full one-second hold. Simulation additionally checks actual opposing contact impulses at 240 Hz. A single still image or TCP proximity cannot establish this grasp, and video alone does not recover microscopic contact thresholds. The adversarial predicate tests (`python tasks/spring-clamp-recovery/test_grasp.py`) exercise scoring logic only; they are not native physics validation.
