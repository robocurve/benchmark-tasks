# Thread a key onto a split ring

Thread the brass household key fully onto the split steel key ring and let it hang freely. A visible bench fixture holds the ring's lower coil. Its unpowered crank turns the ring, and its spring-return splitter blade can lift the free upper end. The robot must start the key's physical hole over the spring tip, feed the spiral through it, finish past the overlapping ends, and release the key with both turns captured.

Adding or sharing a house key is a familiar household action. Its distinctive difficulty is separating and following a steel spiral through a closed hole. The ring is not a closed rubber loop, and neither the key nor the ring receives a conditional attachment.

## Fixed setup

The supplied CAD is in metres. The 38.3 mm outside-diameter ring uses 1.3 mm round wire, 1.6 mm pitch and 705 degrees of winding. It is vertical, centred at `(0.400, 0.000, 0.9875)`. Its near free end points slightly up and toward the left arm. The 47.5 mm brass key is 2 mm thick, with a 7 mm hole, a 2 mm rim at the opening edge and an ordinary toothed blade. It starts flat on a small support at `(0.375, -0.140, 0.9675)`, completely separate from the ring.

The blue aluminum and steel bench holder has a 300 × 200 × 16 mm fixed base with a 70 × 210 mm side foot, an upright, a rotary bearing with constant 0.08 N·m·s/rad damping, and a balanced four-spoke handwheel of 85 mm radius. Four freely spinning orange knobs sit 100 mm behind the ring; their long pins project forward from the spokes. The demonstration physically releases and regrasps the next knob to keep the arms clear. Primitive volumes and material densities determine the handwheel mass, centre of mass and principal inertia. A visible 1.2 g trim slug balances the clamp and open spiral at rest. The fixture is fixed to the bench; one lower-coil node is permanently constrained in the small visible vise jaws. This models a tightened workholding clamp. A second orange grip on a 140 mm handle, with a 35 mm forward standoff, operates a 0–8 mm axial splitter slide on a 140 mm radial withdrawal carriage. A visible telescoping guide parks the blade and handle outside the crank sweep. The thin blade starts outside the ring, enters the inter-coil gap, raises the near coil when pulled toward the robot, and withdraws before the crank turns. Both slides return passively; the radial spring has 5 N/m stiffness, 1 N·s/m damping and 0.4 N preload at its outer stop. The splitter and rotary bearing have permanent springs/damping only. The key support, all jaws, rods, blades and crank parts have explicit collision shapes. The fixture is an original workholding aid, not a model of a specific retail product.

## Run locally

Use the pinned YAM/Arena environment documented in the repository's local installation guide: YAM adapter `0059d02`, Arena `release/0.3.1` at `c7edf3a`, Isaac Lab `91a0906`, Isaac Sim 6.1, Python 3.12. On the restricted workstation, prefix every simulator command with its physical-GPU-0 launcher.

From the repository root, after activating that environment:

```bash
python tasks/split-ring-key/build_assets.py
python tasks/split-ring-key/test_scoring.py
python tasks/split-ring-key/demo.py --out /tmp/split-ring-full --max-steps 2400 --reset-check
python tasks/split-ring-key/demo.py --variant partial --out /tmp/split-ring-partial --no-record --snapshots --width 320 --height 176
python tasks/split-ring-key/demo.py --variant direct --out /tmp/split-ring-direct --no-record --snapshots --width 320 --height 176
python tasks/split-ring-key/demo.py --variant wrong_hole --out /tmp/split-ring-offset --no-record --snapshots --width 320 --height 176
```

Output directories must be fresh. The seed is 0 and object jitter is disabled. Control runs at 20 Hz and physics at 1,000 Hz. The action limit is 2,400 steps (120 simulated seconds). The demonstration, physical controls, measured runtime and source hashes are reported in `VALIDATION.md`. `--no-record` skips continuous video; it does not change physics or actions. `--snapshots` preserves native engagement images for the controls.

A final model evaluation can use:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/split-ring-key/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Thread the brass house key fully onto the steel split ring. Use the orange spring-return splitter and unpowered crank as needed. Release the key and splitter, with both overlapping turns retained and both grippers clear of the key and ring. You may keep holding the crank." \
  --scorer task_score --max-steps 2400 --seed 0 --epochs 1 \
  --store-frames --no-prompt --fail-on-error 1 --log-dir /tmp/split-ring-dream
```

Dream credentials are required for reproduction. The recorded Dream-1 beta evaluation completed all 2,400 actions with native task_score 0.0, max_steps termination and zero errored trials. Its complete three-camera H.264 video is retained separately from the 1.0 scripted demonstration. DREAM_EVALUATION.json records the measured model result, reviewed runtime hashes and video hash. Task-board media `M-FNNFPU` contains the model recording, full-score scripted demonstration and setup screenshot; `task.json` records this uploaded media ID.

## Mechanics and observation boundary

The ring is a continuous native articulation of 20 curved wire pieces, each made of six small collision capsules. The permanent radial-axis torsional springs are a coarse elastic-rod approximation: 1.4 N·m/rad stiffness and 0.010 N·m·s/rad damping, with a 25-degree joint range. They preserve the rest spiral and permit the end to lift. There are no conditional joints, forces, motion callbacks, progress-driven latches or direct object-pose writes. The fixture's native joints and contacts transmit all robot motion.

The nominal surface gap between resting turns is 0.3 mm. Every authored collision shape has a uniform 30 µm positive rest offset and 80 µm contact offset. The combined contact envelope is 0.16 mm, below that resting coil gap; the combined 0.06 mm rest separation reduces numerical penetration at the key rim. These are numerical contact approximations, not a physical coating. Real split rings often touch or preload their turns. This small clearance and the discretized bending directions simplify contact and elasticity; the 2 mm key rim still requires substantial physical opening. Wire torsion, detailed plastic yield, surface scratches and microscopic contact are not modelled. Spring gains are an engineering approximation, not measured calibration of a commercial ring. Strong numerical bending damping controls release transients; it does not change the static spring rest shape or add an attraction to the key. Invalid separation or excessive deformation earns zero. The key uses a 9 g mass and a 4 mm torsional-friction contact patch; its inertia and contact coefficients are engineering approximations.

`demo.py:script` receives only two robot IK wrappers, a declared variant and a stage-label dictionary. It follows fixed dimensions and elapsed action indices. It never reads runtime object poses, contacts, score, segmentation or a hidden scene state to correct motion. The runner separately records privileged physics measurements for scoring and audit. Each trace row is an actual measurement from the simulation; each action row is the command sent through the robot's normal `Action` interface.

Native reset restores all root and articulation states. The key is never reset to an engaged configuration. A successful demonstration must remain physically retained after release; the left arm may continue holding the crank while staying clear of the key. Numerical scorer fixtures do not establish physical success.

## Geometry-based scoring

A fixed 960 × 540 native witness camera records a close view of the ring. Before any actions, it also takes one declared setup view of the key and returns to the fixed ring view. Both are real simulation renders. The witness camera is separate from the standard policy observations and never used by the controller.

The scorer transforms every actual collision capsule into the measured key frame and counts strand crossings through the empty hole. It checks exact support against all 32 authored polygon-wall halfspaces through the full 2 mm plate thickness, with an unchanged 0.05 mm numerical contact allowance. It also measures spring continuity, deformation, opening, distances from both free ends, key velocity and gripper clearance. A full result requires two distinct overlapping turns, passive closure, clear grippers and settled retention. Near placement and a key only started on one strand cannot earn full credit. Exactly one accepted crossing earns 0.25 when the closest free end is less than 10 mm from the hole centre, and 0.5 at distances of at least 10 mm. Exactly two crossings in the same direction, separated by 290–430 degrees along the material and at least 12 mm from both ends, earn 0.75. Other capture patterns, including two crossings still too close to the ends, earn zero. These partial scores do not imply release or closure. The complete geometric/motion hold earns 1.0 through the native TaskScore. Exact thresholds and counterexamples are in `task.py` and `test_scoring.py`.

## References and originality

[Beadalon FAQ archive, entry 201](https://www.beadalon.com/content/faq_archive_14.asp) discusses inserting a pointed tool between the turns, opening them slightly, starting the attachment and working it around the ring. The [split-ring pliers product page](https://ww.beadalon.com/products/tools_pliers_pliersbyfamily_splitring.asp) independently describes a separating tip that holds the spring turns apart while making an attachment. [Its findings catalogue](https://www.beadalon.com/images/Findings%2031%20New%20.pdf) identifies spring-tempered wire and the key-ring-like spiral construction. [The C.H. Hanson manufacturer catalogue, printed page 20](https://xrefs0.plumbersstock.com/product/8/8/884513_Q8resg.pdf) lists spring-tempered steel split key rings in several sizes, including 1.5 inch ID. These establish the construction and technique, not a calibration or endorsement of this fixture.

All task CAD, collision geometry and materials are generated by `build_assets.py`; no downloaded meshes, brand marks or product photographs are embedded. The standard YAM station is supplied by the upstream environment. One reviewed accepted comparison is [O-ring fitting #1576](https://github.com/robocurve/benchmark-tasks/pull/1576), which stretches an elastomer over a threaded plug. This task instead opens a steel spiral and physically threads a closed key hole through its overlapping turns. The public PR metadata screen found no direct split-key-ring match. The signed-in board check at 2026-11-17 04:06 UTC covered all 587 current active/planned entries: 207 accepted, 125 pending, 165 within their revision window and 90 planned. No direct match was identified. The accepted circlip task expands a ring radially over a shaft into its groove; this task separates spiral turns axially and feeds a closed key hole along the wire until both turns retain it. The key-cabinet task handles already assembled keys and rings. OVERLAP_REVIEW.json records these nearby comparisons and board links.

A scoped task-name and alias screen also found no direct match in BEHAVIOR-1K, LIBERO, RoboCasa, SimplerEnv, DataCenterBench or RoboDojo. The closest key tasks are [RoboDojo's lock insertion and turning](https://github.com/RoboDojo-Benchmark/RoboDojo/blob/main/task/RoboDojo/tasks/insert_key.py) and [DataCenterBench's turning of an already placed key](https://github.com/robocurve/datacenterbench#the-tasks). This was a metadata and focused alias screen, not an exhaustive novelty proof.

A supplementary Embodied First Exam review screened 2,404 task-mode directory names and eight nearby goal definitions. Its [cast-keyhole puzzle](https://github.com/mll-lab-nu/embodied-first-exam/blob/13bdc8481166ae873eb5e1eebcc51b1a81ce4e2d/tasks/roboarc/roboarc-cast-keyhole-01-i01-standard/instruction.md) separates rigid pieces, while its [cast-coil puzzle](https://github.com/mll-lab-nu/embodied-first-exam/blob/13bdc8481166ae873eb5e1eebcc51b1a81ce4e2d/tasks/roboarc/roboarc-cast-coil-01-i01-standard/instruction.md) assembles two rigid pieces into a flush cube. Those scoped comparisons did not identify this spring-opening and key-feeding task; other variants and all code were not audited.
