# Take kitchen shears apart for cleaning

Separate the loose full-size shear halves through their passive keyed pivot and leave both released and flat in the washing tray. Touch only the molded handles.

## Run a trial

Use an Isaac Lab environment with yam_arena, NumPy and the task files installed. Run from this directory:

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 --instruction "Separate the kitchen shears into their two halves and leave both flat in the washing tray, touching only their molded handles." --scorer task_score --max-steps 1800 --store-frames --log-dir logs/demo

For the model trial, replace `--policy yam_scripted` with `--policy dream` and use `--log-dir logs/model`. Score with `task_score`, not a distance proxy. Limit: 1800 control steps, 90 seconds at 20 Hz. The fixed starting layout uses zero object jitter and no random geometry; no seed-dependent task variation is used. The scripted waypoint clock lasts 80 seconds and early success termination is disabled, leaving the released finish recorded. A recorded setup/demo/video pipeline took about 6 minutes 49 seconds on one GPU; wall time varies with GPU load.

## Physics and demonstration

geometry.py defines two independent compound-convex halves retained by an elongated headed stem and through-slot. There is no pivot joint, attachment, force-based release or angle-triggered geometry change. The loop and metal shapes keep real-scale dimensions. Masses, friction and pivot clearances are engineering estimates rather than a measured commercial product twin. Cutting, blade preload and elastic handle deformation are omitted.

The scripted controller reads object poses that could be observed by a camera. It trims relative angle and centering by rotating the commanded hand about the observed pivot, requires 0.5 seconds of simultaneous alignment and stability before lifting, and starts the sequential tray transfer only after the ring is visibly clear of the head. It does not read scorer latches, contact forces or a supposed unlocked state. No scorer state changes object motion.

## Scoring and real-trial checks

scoring.py requires at least 5 mm between the complete overhead bounding rectangles of the separated halves. Every positive score requires independent support, all four fingertip points at least 80 mm from every object bounding envelope, all other hand volume at least 2 mm clear, and under 1 mm displacement of every mesh vertex between every pair of times in the final 21 samples spanning one second. Full credit additionally requires both halves flat and wholly on the tray floor. Supported separated halves outside the tray can receive partial credit. See task.json for all numeric conditions.

hands.py transforms a calibrated pad endpoint from each installed finger link's own world pose. Independent jaw travel is never mirrored. Overlapping spheres conservatively cover the rest of the gripper, using the farther actual endpoint to bound finger extent. This approximation can deny credit conservatively. Contact with metal outside the molded handles permanently scores zero under the handles-only safety constraint. Measured fracture or permanent deformation over 1 mm also latches zero. A temporary low pose alone is not evidence of damage or a drop and carries no historical penalty.

For a real trial, reproduce the intact geometry and fixed layout. Use a ruler/caliper for final gaps, tray margins, support height and hand clearance, a level or image for tilt, and a fixed 20 Hz camera for the final second. Compare identifiable extremal surface points. Inspect for deformation over 1 mm. Follow task.json reset notes and keep people outside the robots' reach.

## Validation and limitations

The offline suite contains 36 mechanism, scorer, controller and finger-link checks, including recovery after harmless low handling. Installed nominal IK and mesh-collider path screens checked reach and inter-arm/table clearance; these screens do not prove loaded contact. The scripted trial physically demonstrates full separation, both halves flat in the tray and both hands parked through the finish. The model trial leaves the halves assembled and scores zero. The model recording's final object poses also fail the separation requirement with hypothetical perfectly clear hands and no historical penalties, so its zero does not depend on the removed low-height proxy.
