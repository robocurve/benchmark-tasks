# Thread a key onto a split ring

Thread the brass household key fully onto the split steel key ring and let it hang freely. A visible bench fixture holds the ring's lower coil. Its unpowered crank turns the ring, and its spring-return splitter blade can lift the free upper end. The robot must start the key's physical hole over the spring tip, feed the spiral through it, finish past the overlapping ends, and release the key with both turns captured.

Adding or sharing a house key is a familiar household action. Its distinctive difficulty is separating and following a steel spiral through a closed hole. The ring is not a closed rubber loop, and neither the key nor the ring receives a conditional attachment.

## Fixed setup

The supplied CAD is in metres. The 38.3 mm outside-diameter ring uses 1.3 mm round wire, 1.6 mm pitch and 705 degrees of winding. It is vertical, centred at `(0.400, 0.000, 0.9875)`. Its near free end points slightly up and toward the left arm. The 47.5 mm brass key is 2 mm thick, with a 7 mm hole, a 2 mm rim at the opening edge and an ordinary toothed blade. It starts flat on a small support at `(0.375, -0.140, 0.9675)`, completely separate from the ring.

The blue aluminum and steel bench holder has a 300 × 200 × 16 mm fixed base with a 70 × 210 mm side foot, an upright, a rotary bearing with constant 0.08 N·m·s/rad damping, and a balanced four-spoke handwheel of 85 mm radius. Four freely spinning orange knobs sit 100 mm behind the ring; their long pins project forward from the spokes. The demonstration physically releases and regrasps the next knob to keep the arms clear. Primitive volumes and material densities determine the handwheel mass, centre of mass and principal inertia. A visible 1.2 g trim slug balances the clamp and open spiral at rest. The fixture is fixed to the bench; one lower-coil node is permanently constrained in the small visible vise jaws. This models a tightened workholding clamp. A second orange grip on a 140 mm handle, with a 35 mm forward standoff, operates a 0–8 mm axial splitter slide on a 140 mm radial withdrawal carriage. A visible telescoping guide parks the blade and handle outside the crank sweep. The thin blade starts outside the ring, enters the inter-coil gap, raises the near coil when pulled toward the robot, and withdraws before the crank turns. Both slides return passively; the radial spring has 5 N/m stiffness, 1 N·s/m damping and 0.4 N preload at its outer stop. The splitter and rotary bearing have permanent springs/damping only. The key support, all jaws, rods, blades and crank parts have explicit collision shapes. The fixture is an original workholding aid, not a model of a specific retail product. A rebuildable specification and costed parts list are in **Building the fixture** below.

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

Dream credentials are required for reproduction. Runs of this revision (task.py SHA-256 prefix 1e04cf3ac2d6, the assets built by `build_assets.py` from this folder), seed 0, logs in `runs/`:

- Scripted demonstration (`demo.py --max-steps 2400 --reset-check`): **1.0**, success after 2,246 of 2,400 actions, with both turns captured, the ring closed (0.48 mm gap left), integrity held (peak turn spread 6.1 mm, ring outline within 0.02 mm of its radius) and both hands clear (`runs/demo_result.json`).
- Dream-1: **0.0** at the 2,400-action limit (`runs/dream_log.json`, per-step records trimmed for size).
- `test_scoring.py`: every case passes, including the 9 new integrity cases.

The shortcut controls were not rerun on this revision (each takes about 2 hours on the shared GPU); `VALIDATION.md` and `DREAM_EVALUATION.json` record the earlier code from pull request #2426, which used the joint-based integrity limits this revision replaces.

## Mechanics and observation boundary

The ring is a continuous native articulation of 20 curved wire pieces, each made of six small collision capsules. The permanent radial-axis torsional springs are a coarse elastic-rod approximation: 1.4 N·m/rad stiffness and 0.010 N·m·s/rad damping, with a 25-degree joint range. They preserve the rest spiral and permit the end to lift. There are no conditional joints, forces, motion callbacks, progress-driven latches or direct object-pose writes. The fixture's native joints and contacts transmit all robot motion.

The nominal surface gap between resting turns is 0.3 mm. Every authored collision shape has a uniform 30 µm positive rest offset and 80 µm contact offset. The combined contact envelope is 0.16 mm, below that resting coil gap; the combined 0.06 mm rest separation reduces numerical penetration at the key rim. These are numerical contact approximations, not a physical coating. Real split rings often touch or preload their turns. This small clearance and the discretized bending directions simplify contact and elasticity; the 2 mm key rim still requires substantial physical opening. Wire torsion, detailed plastic yield, surface scratches and microscopic contact are not modelled. Spring gains are an engineering approximation, not measured calibration of a commercial ring. Strong numerical bending damping controls release transients; it does not change the static spring rest shape or add an attraction to the key. Excessive turn spread or a distorted ring outline earns zero (see **Real-trial ring checks**). The key uses a 9 g mass and a 4 mm torsional-friction contact patch; its inertia and contact coefficients are engineering approximations.

`demo.py:script` receives only two robot IK wrappers, a declared variant and a stage-label dictionary. It follows fixed dimensions and elapsed action indices. It never reads runtime object poses, contacts, score, segmentation or a hidden scene state to correct motion. The runner separately records privileged physics measurements for scoring and audit. Each trace row is an actual measurement from the simulation; each action row is the command sent through the robot's normal `Action` interface.

Native reset restores all root and articulation states. The key is never reset to an engaged configuration. A successful demonstration must remain physically retained after release; the left arm may continue holding the crank while staying clear of the key. Numerical scorer fixtures do not establish physical success.

## Geometry-based scoring

A fixed 960 × 540 native witness camera records a close view of the ring. Before any actions, it also takes one declared setup view of the key and returns to the fixed ring view. Both are real simulation renders. The witness camera is separate from the standard policy observations and never used by the controller.

The scorer transforms every actual collision capsule into the measured key frame and counts strand crossings through the empty hole. It checks exact support against all 32 authored polygon-wall halfspaces through the full 2 mm plate thickness, with an unchanged 0.05 mm numerical contact allowance. It also measures the ring with the checks listed below, distances from both free ends, key velocity and gripper clearance. A full result requires two distinct overlapping turns, passive closure, clear grippers and settled retention. Near placement and a key only started on one strand cannot earn full credit. Exactly one accepted crossing earns 0.25 when the closest free end is less than 10 mm from the hole centre, and 0.5 at distances of at least 10 mm. Exactly two crossings in the same direction, separated by 290–430 degrees along the material and at least 12 mm from both ends, earn 0.75. Other capture patterns, including two crossings still too close to the ends, earn zero. These partial scores do not imply release or closure. The complete geometric/motion hold earns 1.0 through the native TaskScore. Exact thresholds and counterexamples are in `geometry.py`, `task.py` and `test_scoring.py`.

### Real-trial ring checks

Ring integrity uses only quantities that a real trial can measure with the fixed witness video, calipers or a feeler gauge. The simulator computes each from the measured poses of the ring's collision capsules, expressed in the crank's frame. `ring_measures()` in `task.py` compares each outer-turn centreline point with the inner turn at the same polar angle. Joint angles and joint endpoint gaps are still logged for audit, but they are not scored.

| Check | When | Limit (simulation) | Real-trial equivalent | Effect |
| --- | --- | --- | --- | --- |
| Peak turn spread | Every step, latched | Surface gap between overlapping turns at most 12 mm. The rest gap is 0.3 mm. | Fixed witness video, scaled by the known 38 mm OD: no turn ever lifts more than 12 mm, about six 1.98 mm wire diameters, off the turn beneath it | Exceeding it at any time scores 0 |
| Ring outline | Every step, latched | Every centreline point within 4 mm of the 18.5 mm nominal radius | Video outline, plus OD across three diameters with calipers after the trial: all within 30–46 mm (38 ± 8 mm) | Exceeding it scores 0 |
| Turn left spread | Final hold | Largest surface gap between turns at most 1.8 mm (0.3 mm rest + 1.5 mm) | After release, with the key hanging: a 1.5 mm feeler stack (1.0 + 0.5 mm leaves) must not enter between the turns anywhere | Needed for full score |
| Ring lies flat | Final hold | Axial centreline span at most 4.7 mm (relaxed 3.13 mm) | Ring thickness across both turns with calipers, at most the as-received thickness + 1.57 mm | Needed for full score |
| Permanent set | Real trials only | The modelled springs are elastic and cannot take a set, so the latched peak spread stands in for it | Remove the key after scoring and lay the bare ring on a flat plate. Repeat the feeler and caliper checks against the as-received values recorded before the trial | A failure scores 0 |

Real limits are stated relative to the as-received ring, because real turns touch at rest while the model keeps a 0.3 mm gap. Before each real trial, record the ring's thickness and confirm that a 0.05 mm leaf does not enter between the turns. The 12 mm spread limit exceeds the splitter's 8 mm axial stroke plus the rest gap. Using the full fixture stroke is therefore allowed, while forcing the coil open with the key head is not. This is a stated engineering limit, not a measured yield point for the purchased ring.

## Building the fixture

Positions are relative to the ring centre: +x points away from the robots along the ring axis, +z points up, and the ring centre is 240 mm above the table. Distances are in mm. Every part comes from `build_assets.py`, and the printable parts can be modelled directly from this text.

### Steel split ring (purchased)

**RS PRO split ring, RS stock no. 866-579**: carbon spring steel, nickel plated, 38 mm OD, 33.9 mm ID, 1.98 mm wire, 3.7 mm ring thickness, pack of 20. It is listed on RS Components sites in the UK, Europe, Australia, New Zealand and South Africa (about £13.64 ex VAT per pack of 20 in the UK). It matches the modelled 38.3 mm OD. Its wire is thicker than the modelled 1.3 mm, so it is stiffer than the model. No listing found specifies a 38 mm ring with 1.3 mm wire. This mismatch is disclosed, not calibrated. The modelled 7 mm key hole still passes both 1.98 mm turns with 3 mm to spare. Use one ring per trial, or reuse a ring only after it passes the permanent-set check.

### How the splitter opens touching turns

The simulated blade is a 0.18 mm plate that slides into the modelled 0.3 mm rest gap. A real ring's turns touch, with no gap. Its two round wires meet in a V-shaped groove on the ring's outer face. The real tooth is therefore a wedge, not a flat plate. It is 4.0 mm long radially and 1.0 mm wide. Its leading edge is at most 0.1 mm thick and tapers up to 0.6 mm at the neck, an included angle of about 14°. The robot pushes the carriage in radially. The edge enters the groove, and the taper forces the turns apart until the tooth sits between them with about 0.6 mm clearance. This is the same separating action as the pointed tip of split-ring pliers. Pulling the orange grip toward the robot then lifts the outer turn on the axial slide, up to its 8 mm stop, as in simulation. Grind the tooth from a 0.8 mm hardened spring-steel feeler-gauge leaf or shim, and solder or screw it to the arm.

### Dimensioned parts

| Part | Dimensions (mm) and position | Material / source |
| --- | --- | --- |
| Base plate | 300 × 200 × 16, centred at x = 130 on the table | 6061 aluminium plate, or 18 mm plywood |
| Side foot | 70 × 210 × 16, centred at x = 235, y = 168 | Same as base |
| Upright | 25 (x) × 30 (y) × 224 tall, at x = 250, from the base top to the bearing | Aluminium bar |
| Rotary bearing | Housing Ø30 × 34 long at x = 233–267, on the ring axis. Bore 28 H7 for **two 6001-2RS deep-groove ball bearings (12 × 28 × 8)** | Machined or printed housing |
| Spindle | Ø12 × 250, x = 15–265 | Steel rod |
| Handwheel spokes | Four, 85 long × 12 × 8, at x = 214, 90° apart | Aluminium flat bar |
| Knob pins and grips | Four Ø5 pins at 85 mm radius, x = 100–214. Grips Ø12 × 32 with a 5.2 mm bore, spinning freely | Steel dowel. Printed orange grips, 10 g each |
| Vise clamp | Ø4 stalk from the spindle end (x = 18) to the lower coil at 176° of the spiral. Jaws 5 × 5 × 3.7 (lower) and 3 × 4 × 1.7 (inner), pinching that coil with an M2 screw | Steel stalk. Printed or aluminium jaws |
| Trim slug | 1.2 g steel cube (5.3 mm), placed by the balance calculation in `build_assets.py` | Steel offcut or washer of equal mass |
| Radial withdrawal slide | 140 mm travel along the radius at −35° on the ring. Ø8 × 130 guide rail. Carriage block 25 × 20 × 10 with a Ø4 × 70 rod. Build it as an **MGN9 miniature rail, 200 mm, with one MGN9H carriage** | Steel |
| Axial splitter slide | 0–8 mm along the ring axis. Build it as an **MGN7 rail, 40 mm, with one MGN7C carriage**, with hard stops at 0 and 8 mm | Steel |
| Splitter arm | 136 × 6 × 3. Grip standoff Ø6 × 35. Grip 16 mm cube | Steel arm. Printed orange grip |
| Splitter tooth | Wedge 4.0 × 1.0, edge ≤ 0.1 to 0.6 thick, plus a 2.0 × 1.0 neck | Hardened spring steel |
| Splitter post, bridge, guide | Post 10 × 14 section, from the base to the guide. Bridge 14 × 10 section. Guide block 25 × 25 × 12 | Aluminium or printed |
| Key support | Base 75 × 60 × 12, column 14 × 12 × 208, rest 14 × 20 × 6. Rest top 220 mm above the table, 140 mm to the robot's right of the ring | Printed or aluminium |
| Key | 47.5 × 18 × 2 brass, Ø7 hole, 11 mm bow (2 mm rim), 9 g | Brass key blank. Drill the hole to 7 mm if needed |

### Springs and damping

| Element | Simulation | Real specification (design calculation, not a catalogue part) |
| --- | --- | --- |
| Radial return spring | 5 N/m, 0.4 N preload at the parked stop, 1.1 N fully inserted | Extension spring of 0.30 mm music wire, 6.0 mm OD, about 86 close-wound coils (26 mm body), rate ≈ 0.005 N/mm. Pre-stretch it 80 mm at the parked stop. Peak shear is about 630 MPa at 1.1 N. A constant-force spring of 0.4–1.1 N is an acceptable substitute |
| Axial splitter return spring | 35 N/m, no preload, 0.28 N at 8 mm | Compression spring of 0.20 mm music wire, 5.0 mm OD, 4 active coils, 14 mm free length, rate ≈ 0.036 N/mm |
| Slide damping | 1 N·s/m on each slide | Rail and carriage friction. Not tuned |
| Crank damping | 0.08 N·m·s/rad viscous | Nylon-tipped M4 set screw against the spindle. Adjust until a spring scale at a knob reads about 0.5–1 N while turning at about one turn per 6 s. This drag is Coulomb-like, not viscous, which is a disclosed difference |

### Costed parts list (approximate USD, November 2026)

| Item | Qty | Approx. cost |
| --- | ---: | ---: |
| RS PRO 866-579 split rings (pack of 20) | 1 | 18 |
| Aluminium plate and bar (base, foot, upright, spokes) | 1 lot | 45 |
| 6001-2RS bearings | 2 | 6 |
| Steel rod: Ø12, Ø8, Ø5, Ø4 lengths | 1 lot | 15 |
| MGN9 200 mm rail + MGN9H carriage | 1 | 20 |
| MGN7 40 mm rail + MGN7C carriage | 1 | 15 |
| Music wire (0.20 and 0.30 mm) or wound springs | 1 lot | 10 |
| Feeler-gauge set to 1.0 mm (tooth stock and the 1.5 mm stack) | 1 | 8 |
| M2/M3/M4 screws and set screws | 1 lot | 8 |
| Printed parts (grips, jaws, guide, housing, key support), about 120 g PLA | 1 lot | 3 |
| Brass key blank | 1 | 2 |
| 150 mm digital calipers (for the ring checks) | 1 | 15 |
| **Total** | | **≈ 165** |

Only the ring price was checked against a live listing. Other prices are typical retail estimates.

## Claimed revision

This task builds on pull request #2426, whose 48-hour revision hold lapsed without a revision. It answers each point of that review.

| Reviewer point | What changed |
| --- | --- |
| Rebuildable specification for the rotary vise and splitter (drawings or printable parts, bearings, springs, blade dimensions, parts list) | Added **Building the fixture**: dimensioned parts table with positions, 6001-2RS bearings, MGN9/MGN7 slides, spring wire/OD/coil specifications matching the modelled rates, blade dimensions and a costed parts list |
| Repeatably available steel ring, and how the splitter opens touching turns | Named the RS PRO 866-579 ring (38 mm OD, 1.98 mm wire) and disclosed its wire mismatch. Specified a wedge-ground tooth that separates touching turns before the axial lift |
| Replace simulator-only joint-angle and joint-gap integrity limits with observable checks | `task.py` now judges integrity by peak turn spread (12 mm) and ring outline (4 mm), both latched. Full score also needs a released gap of at most 1.8 mm and an axial span of at most 4.7 mm. Each check has a video, feeler-gauge or caliper equivalent, and real trials add a permanent-set check. Joint angles and gaps are logged only. `test_scoring.py` adds nine cases |

## Origin

This revision builds on the split-ring key-threading task first proposed in pull request #2426. It keeps that task's CAD, controller and crossing scorer.

## References and originality

[Beadalon FAQ archive, entry 201](https://www.beadalon.com/content/faq_archive_14.asp) discusses inserting a pointed tool between the turns, opening them slightly, starting the attachment and working it around the ring. The [split-ring pliers product page](https://ww.beadalon.com/products/tools_pliers_pliersbyfamily_splitring.asp) independently describes a separating tip that holds the spring turns apart while making an attachment. [Its findings catalogue](https://www.beadalon.com/images/Findings%2031%20New%20.pdf) identifies spring-tempered wire and the key-ring-like spiral construction. [RS PRO 866-579](https://uk.rs-online.com/web/p/key-accessories/0866579) lists the purchased ring's dimensions. [The C.H. Hanson manufacturer catalogue, printed page 20](https://xrefs0.plumbersstock.com/product/8/8/884513_Q8resg.pdf) lists spring-tempered steel split key rings in several sizes, including 1.5 inch ID. These establish the construction and technique, not a calibration or endorsement of this fixture.

All task CAD, collision geometry and materials are generated by `build_assets.py`; no downloaded meshes, brand marks or product photographs are embedded. The standard YAM station is supplied by the upstream environment. One reviewed accepted comparison is [O-ring fitting #1576](https://github.com/robocurve/benchmark-tasks/pull/1576), which stretches an elastomer over a threaded plug. This task instead opens a steel spiral and physically threads a closed key hole through its overlapping turns. The earlier submission's public PR metadata screen found no direct split-key-ring match. Its signed-in board check at 2026-11-17 04:06 UTC covered all 587 current active/planned entries: 207 accepted, 125 pending, 165 within their revision window and 90 planned. No direct match was identified. The accepted circlip task expands a ring radially over a shaft into its groove; this task separates spiral turns axially and feeds a closed key hole along the wire until both turns retain it. The key-cabinet task handles already assembled keys and rings. OVERLAP_REVIEW.json records these nearby comparisons and board links.

A scoped task-name and alias screen also found no direct match in BEHAVIOR-1K, LIBERO, RoboCasa, SimplerEnv, DataCenterBench or RoboDojo. The closest key tasks are [RoboDojo's lock insertion and turning](https://github.com/RoboDojo-Benchmark/RoboDojo/blob/main/task/RoboDojo/tasks/insert_key.py) and [DataCenterBench's turning of an already placed key](https://github.com/robocurve/datacenterbench#the-tasks). This was a metadata and focused alias screen, not an exhaustive novelty proof.

A supplementary Embodied First Exam review screened 2,404 task-mode directory names and eight nearby goal definitions. Its [cast-keyhole puzzle](https://github.com/mll-lab-nu/embodied-first-exam/blob/13bdc8481166ae873eb5e1eebcc51b1a81ce4e2d/tasks/roboarc/roboarc-cast-keyhole-01-i01-standard/instruction.md) separates rigid pieces, while its [cast-coil puzzle](https://github.com/mll-lab-nu/embodied-first-exam/blob/13bdc8481166ae873eb5e1eebcc51b1a81ce4e2d/tasks/roboarc/roboarc-cast-coil-01-i01-standard/instruction.md) assembles two rigid pieces into a flush cube. Those scoped comparisons did not identify this spring-opening and key-feeding task; other variants and all code were not audited.
