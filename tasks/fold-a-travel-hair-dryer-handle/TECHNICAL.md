# Physical scene and evidence contract

The policy-visible scene is a standard dual-arm YAM station with the stock robot, gains, torque limits, table and three RGB cameras. Policy inputs are three 320×176 RGB images and the 14 measured arm/jaw positions. Control is 20 Hz, physics 240 Hz, decimation 12. The declared budget is 1,000 actions. The extra 1280×704 side camera is review-only. The scripted runner makes no model call. The separate Dream-1 evaluator uses the same frozen task; its completed outcome is described below.

## Original asset and estimates

The physical dimensions and combined head/handle mass are anchored to Panasonic's EH-NA2C specification: 154×140×79 mm and 368 g excluding cord and nozzle. This is an original simplified rigid-body representation, not a digitized OEM product or a claim of measured hinge torque. The task uses a cool, unplugged device. Electrical, thermal and airflow behavior are outside this bounded folding operation.

There are nine dynamic bodies, in this exact native order: head, handle, cord_0 through cord_5, cord_bundle. The head is 285 g and the handle is 83 g. The head's 154 mm axis lies along local X: the front outlet changes radius 25→35 mm between X −77 and −60 mm, the middle barrel has radius 35 mm up to X +50 mm, and the rear motor housing has radius 39.5 mm up to X +77 mm. The 50 mm hinge offset and 50.5 mm handle reach yield the 140 mm open height. The handle grip measures 28×30×38.5 mm; it includes a strain relief and a visible switch. These allocations and internal dimensions are engineering estimates.

The folding axis is head-local Y, with its anchor at (35, 0, −50) mm. A real 7.5 mm radius pin passes through the handle's segmented annular eye, with 8 mm bore and 10 mm outer radius. The head has two 10 mm clevis cheeks centered 21 mm to either side of its hinge plane. Their inner faces are 32 mm apart, leaving 1 mm clearance on each side of the 30 mm grip. The 52 mm pin spans both cheeks. An earlier zero-clearance layout mechanically clamped the grip around 43 degrees; it was corrected as an assembly interference, while keeping parent/child collisions, friction, mass and the 90 degree joint stop. Parent/child body collision remains enabled. A native revolute joint enforces 0–90 degree stops and the coincident axis. It is excluded from articulation. The authored angular drive has zero stiffness, zero velocity target, damping 50 and maximum torque 0.025 N m, intended as a bounded dissipative hinge estimate; it has no angle target and is never changed. Authored drive fields are not themselves evidence of native retention: final evaluation measures actual released motion. Additional rigid-body angular damping is 0.10 s⁻¹. Geometry and material friction also remain fixed.

Six 50 mm, 2.5 mm radius capsules represent a 300 mm loose cord tail. Each is 1.25 g, with physically estimated capsule inertias. Seven native spherical joints join the handle outlet, links and coil. Cone limits are 179.9 degrees; exactly 180 is rejected by the native D6 parser and is not used. Only each adjacent ball-joint body pair is filtered from collision. All other contacts remain active.

The pre-tied coil/plug is a separate 68 g free body: annulus radii 30/40 mm and thickness 25 mm, a 40×25×18 mm bound plug, and visible binding straps. Its position is not fixed. The bound mass represents approximately 2.4 m of coiled cord plus the plug, so the total modeled cable is approximately 2.7 m; this length/mass is explicitly an engineering estimate, not a manufacturer specification. A tied remote cord is the initial storage precondition, not a robot-created invisible constraint.

Head inertia is (0.00023, 0.00063, 0.00063) kg m² with COM (8,0,−3) mm; handle inertia is (0.000025,0.000023,0.000013) kg m² with COM (0,0,−27) mm. Component estimates are declared in `layout.py` and the asset generator. All bodies are free, massive, nonkinematic, CCD-enabled; only native joints connect them. There is no guide, world anchor, fixed coil, proximity attachment, switched lock, object pose coupling or runtime modification of object mass/material/geometry.

Every convex mesh has at most 48 authored points. The annular eye and coil use separate eight-point convex sectors, preserving their open bore. Local mesh points are expressed at 1,000 times metre coordinates with a 0.001 inverse transform to avoid losing small physical geometry during convex cooking. Collision offsets are 0.15 mm with zero rest offset. Solver position/velocity iterations are 32/8, maximum depenetration velocity 0.5 m/s, and restitution zero. The body surface friction estimate is 0.4, cable 0.5.

## Reset and reference controller

A single deterministic scene and seed 0 are used. The unfolded device begins on its side and the loose cord naturally drops to the counter during the stock reset settling period. The adapter permits limited initial settling (40 mm for the device, 80 mm for loose cord bodies); the grading subsequently checks the actual hinge and every cable attachment rather than assuming authored poses. Transforms and velocities are set only at reset. After that the policy issues only native robot joint/jaw targets.

The reference may inspect actual body state and contacts. It grasps the barrel near its centre of mass, lifts it and turns it so gravity folds the freely hinged handle to approximately 78 degrees. It then turns the grip below the barrel while adding 30 degrees of yaw, lowers the device toward the counter and adjusts the commanded wrist orientation to the measured grasp with a five-degree pitch offset. Slow 0.5 mm robot-target advances let actual counter reaction fold the grip past 75 degrees; advancing stops at about 35 mm of extra target travel or when the measured net task-contact Z force reaches 8 N. That threshold is feedback to stop advancing, not a hard force clamp or an applied task-body force. The fingers open while withdrawing. Once the obstructing finger clears, actual counter force and the barrel's weight finish the fold against the real 90-degree stop. The folded grip bears the barrel and the released device settles. The actual native result, rather than the commanded pose or presumed drive behavior, determines success. Actual FK compensation only changes robot targets. It does not set the dryer or cord poses, apply task forces, switch joints or create attachments. Reference success therefore establishes fixed-start scripted feasibility, not vision-policy or hardware robustness.

## Independent grading

Every one of the 12 native substeps per action is recorded and graded. The body input is 9×7 pose, 9×6 spatial velocity, 9×9 body contact counts, 9×9×3 contact force vectors and 18×9 robot-contact counts. The grader is independent of the scripted phases.

Score the current hinge fold from its open position: below 45° earns 0, 45° or more earns 0.4, and the folded zone (80–93°, around the 90° storage stop) earns 0.7. Award 1.0 once the intact dryer rests entirely on the counter, both hands are clear of the dryer and cord, and the handle stays in that zone for one continuous second. During that second each visible barrel end must stay within 1 cm of where it began. Restart the clock on loss of fold, release or support, or from a new rest position if the barrel moves farther. Detachment, leaving the counter or invalid simulation input overrides all credit with 0.

Simulation derives fold from the relative rotations and support from the lowest transformed collision vertex over the counter. Native connector residuals reject separated components. These calculations implement the visible intact, supported state. The two barrel end centres are at ±77 mm on its local X axis. The first qualifying sample starts the timer; a full second spans at least 241 samples at 240 Hz.

## Portable evidence scope

Descriptions of formal archives, prior prototypes or retention diagnostics above
refer to the original recording history, which is retained separately. This
portable folder includes current validation, original-result hashes, explicitly
marked result summaries and audit metadata in `evidence/`; it does not claim that
omitted raw native traces can be replayed from those summaries. Use the commands
in README.md to run and score a fresh trial. The exact recorded protocol and
service/guard limitations are in [EVALUATION.md](EVALUATION.md).
