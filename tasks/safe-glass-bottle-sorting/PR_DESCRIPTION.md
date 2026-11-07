Sort six bottles safely between occupied, vertically stacked shelf levels: red below, blue above. The narrow rack requires horizontal entry and preserves four neighboring bottles during the demonstrated staged swap. This tests multi-object reclassification, compared with retrieving one selected bottle into an empty tray.

Scoring uses an irreversible tilt/fall/workspace latch and numeric placement, gripper clearance and two-second position-drift criteria. Partial credit is 0 / 0.5 / 1; the starting arrangement scores zero. Force is telemetry only. Physical reproduction uses rigid plastic bottle surrogates, with a ruler/protractor and calibrated-video scoring guide; glass fracture is outside scope.

Validation on the GPU simulator, seed 0, zero pose jitter, 2,400-step limit:

- Scripted demo: 1.0, 1,239 steps, 61.95 simulated seconds, 431.8 seconds wall time; no errors or safety violations.
- Dream-1: 0.0, 2,400 steps, 120.00 simulated seconds, 744.7 seconds wall time; termination `max_steps`.
- Both recordings use the finalized observable scorer and exact submitted instruction.
- Every task.json section is at most 500 characters; detailed inventory and scoring remain in the included files.
- 18 regression tests pass; both arms' horizontal IK errors are below 0.1 mm and 0.02 degrees.
- Recordings fully decode as H.264/yuv420p, 640x360 at 20 fps, with faststart and no audio.

README.md supplies dependencies, commands, runtime, seed, layout, reset and fidelity. scene_inventory.json enumerates the scene. SUBMISSION_REVIEW.md records the authenticated board comparison and distinctness rationale. The evidence preflight checks source equivalence, scores, traces and media hashes. The official uploader returned media bundle `M-7UJQES`; strict submission preflight passes.
