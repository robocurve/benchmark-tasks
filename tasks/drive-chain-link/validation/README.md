# Current task validation

All current executable code and both robot-motion files are covered by source fingerprint `e416a5f7a88d638b0ce04c317eba00e26f4e66616e67f434f28df5ee2dd18de7`.

The reference policy reads elapsed time only. Its 1,180-command robot program was calibrated once during development with the standard Inspect guards. The calibration run itself is not the demonstration: the final native and registered runs independently re-execute those commands under native physics without object-state feedback. All executed commands in both final runs exactly match the frozen program. The environment has no controller-access registry.

- `reference-native.json`: actual three-camera native execution, 1.0 at 1,180 actions / 59 seconds; 14,160 per-physics-step workspace audits.
- `reference-inspect.json`: independent registered Inspect Robots execution with actual cameras and standard guards, 1.0 at 1,180 actions. Evaluation seed 0 derives trial seed 1415158438, also used by the native recipe.
- `reference-media.json`: all 1,181 initial/action frames per camera decoded and checked; 20fps H.264/yuv420p composite, every frame preserved. Initial, joining, keeper and final views were visually reviewed. No overlays, generated frames or temporal edits.
- `reference-controller.json` and `reference-roles.json`: policy-input audit and measured arm roles. The left hand travels over one metre while fitting the keeper and the right hand remains parked; arm roles are not scoring conditions.
- `negative-control.json`: omitting the retaining slide scores 0.4, with no violations, under the final source.
- `unit-tests.txt`, `lint.txt`, `usd-structure.json`: 37 tests, static checks and four asset structure checks pass.
- `model-native.json` and `model-media.json`: one matching Dream-1 episode, 0.0 at the 2,400-action limit, without execution errors. All 2,401 frames per camera and the composite video are validated.
- `preflight.json`: the completed post-model metadata, source, media and evidence checks. `files.sha256.json` records the task files, excluding itself.

The geometry, contact parameters, scoring and fixed setup are unchanged from the preceding scene. Superseded controller/receipts are archived outside the submission folder. The latest instruction and all final evidence use the same source fingerprint; no prompt-equivalence exception is needed.

The matching Dream-1 episode is complete and uses the exact frozen source and instruction above. No earlier model episode or media ID is reused. The real model score and video are ready alongside the 1.0 demonstration. Matching media are uploaded as `M-4UKEJ5` and bound explicitly in canonical `task.json`. `media-upload.json` records the exact uploaded file hashes.
