# Set a carriage preload for rigidity and low drag

Slide the blue preload tab and orange lock toward the rail. Hold the green side-load cap with the carriage untouched, release it, then push the carriage to its amber end marker.

Run `bash run.sh settle`, `bash run.sh demo`, `bash run.sh ablation` and `bash run.sh dream` in the RoboCurve Isaac Lab Arena launchable. The layout is fixed, object jitter is zero and each run resets the simulator. The runner saves logs, frames and telemetry at 20 Hz.

The 357 x 261 x 8 mm aluminum base is centered at (304,-2,751.5) mm. The rail is centered at y=-51 mm. The black carriage starts at x=417 mm and has 108 mm of travel along a serial axial and lateral guide. The blue and orange tabs start at x=210 and 294 mm, y=98 mm, with 15 mm of stroke. The green cap is at (372,106,790) mm and has 7 mm of stroke. Exact sizes, colors, and reset positions are in task.py.

The serial prismatic joints move through contact and applied forces. Measured tab travel sets a calibrated equivalent V-wheel spring rate and rolling resistance. The cap applies 1 N to the physical lateral guide. A tool clearance check rejects bracing. No carriage pose is written during a trial.

0.25 for locking the preload. 0.50 for either a valid 1 N side test or a valid axial sweep. 1.0 when both pass at the same setting. Side deflection must be below 0.5 mm. Peak drag must be below 2 N over at least 100 mm. Bracing during the side test, fixture movement beyond the stated tolerances, overtravel, or a changed preload setting zeroes the trial.
