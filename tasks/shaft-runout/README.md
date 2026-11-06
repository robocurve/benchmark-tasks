# Align a coupled shaft and prove low runout

Slide the blue front and rear adjusters toward the spindle and slide the orange cradle lock. Press the green TEST cap and let the guarded spindle turn twice.

Run `bash run.sh settle`, `bash run.sh demo`, `bash run.sh ablation` and `bash run.sh dream` in the RoboCurve Isaac Lab Arena launchable. The layout is fixed, object jitter is zero and each run resets the simulator. The runner saves logs, frames and telemetry at 20 Hz.

A 356 x 255 x 8 mm aluminum base sits at (298,-2,751.5) mm on the YAM table. Front, rear and lock tabs start at x=211,291,372 mm, y=98 mm, z=798 mm with 15 mm of travel. The green cap at (270,-106,789) mm has 7 mm of travel. Guarded steel rotors turn about Z and X. Exact dimensions and reset geometry are specified in task.py.

Physical tabs set calibrated equivalent shaft alignment. The test cap enables an effort-controlled input. A compliant torsional model drives the actual guarded output shaft. Alignment and drag are equivalent calibrated bench quantities, with output turns measured from the joint.

0.25 for one aligned axis with the cradle locked. 0.50 for radial error <=0.5 mm and angular error <=0.5 degrees at the same locked setting. 0.75 after one output turn and 1.0 after two turns at <=0.25 N m peak coupling torque. Movement of the fixture, changed settings, or >0.50 N m torque zeroes the trial.
