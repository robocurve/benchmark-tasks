# Submission preflight

The authoritative source is the [live Robocurve guide](https://tasks.robocurve.org/), including its five review criteria, and the [submission procedure](https://tasks.robocurve.org/submit).

| Criterion | Task evidence |
| --- | --- |
| Importance | Conveyor-chain joining is a specific machinery-maintenance reassembly operation. Manufacturer procedure and occupation references are in README/SOURCE.json. The complete task board and the six named benchmark catalogs were screened, including nearby articulated-link assembly tasks; no direct chain-connecting-link duplicate was found. |
| Construct validity | Eleven physical link bodies, ten passive hinges, a fixed guide, loose connector and retaining plate. Joining and retention depend on native contacts. Scoring accepts valid alternative arm assignments and sequences. Custom keeper geometry is described in the required fidelity field. |
| Scoring | Current-state 0 / 0.4 / 0.7 / 1.0; measured hole/slot engagement, guide seating, one-second stability and released hands. Numeric predicates and permanent invalid/workspace outcomes are documented. Adversarial tests cover missing ends, wrong keeper position, hovering/lifted parts, disassembly, clearance and nonfinite states. |
| Clarity and reproducibility | Fixed object poses, dimensions, materials, masses, colours, robot/camera calibration and reset procedure are in task.json, setup-layout.json and scene-inventory.json. The instruction states the goal and final conditions. README gives exact commands and seeds. |
| Feasibility | Fresh native and registered camera replays must independently reach 1.0 under the frozen clock-only program. Only fourteen normal robot commands actuate the task after reset. |

- [x] Goal-only prompt matches the environment; fidelity text is below 500 characters.
- [x] The reference policy reads elapsed time only. No environment registry, object-pose/contact feedback or score input.
- [x] All 37 tests pass, including full-program clock-only execution and bit-identical passage through the standard Inspect guards.
- [x] Four generated USD assets pass structure checks; code and custom assets have explicit licenses/provenance.
- [x] Omitting the retaining slide scores 0.4 under the final source, with no violations.
- [x] Native three-camera reference reaches 1.0; final receipt, frame counts and source hashes verified.
- [x] Registered Inspect reference with actual cameras reaches 1.0.
- [x] Fresh native footage visually reviewed; setup screenshot, all-camera decode, blind review, codecs and size limits verified.
- [x] Final local preflight passes; branch is prepared for fork synchronization.
- [x] One matching Dream-1 episode: 0.0 at 2,400 actions, no execution errors, all three cameras recorded and validated. No old model video is reused.
- [x] Matching Dream/demo videos and actual setup screenshot uploaded as M-4UKEJ5; canonical task.json contains the actual scores and media ID.

Submission uses one PR from the registered account. Its comments provide the task-board receipt.
