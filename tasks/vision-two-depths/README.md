# Commission a camera at two inspection depths

Slide both blue lens setting tabs toward the chart rail, close the orange lens lock, then press green TEST and clear both arms.

Run `bash run.sh settle`, `bash run.sh demo`, `bash run.sh ablation` and `bash run.sh dream` in the RoboCurve Isaac Lab Arena launchable. The layout is fixed, object jitter is zero and each run resets the simulator. The runner saves logs, frames and telemetry at 20 Hz.

The base measures 366 x 266 x 8 mm at (302,-4,751.5) mm. The chart carriage starts at (249,-56,774) mm, with two physical bar targets at 180 and 205 mm working depth. The fixed inspection camera is at (300,-56,987.5) mm. Three blue/orange lens tabs start at y=98 mm and have 15 mm stroke. The green test cap is at (260,-115,789) mm with 7 mm stroke. The fixture parts and mounting details are in task.py.

Isaac renders physical bars with a dedicated inspection camera. A calibrated exposure and point spread model applies the measured lens settings to the rendered pixels. The scorer measures contrast and clipping from those images, blur from the optical model, and speed from the physical stage joint.

0.25 for locking the lens, 0.50 for one passing chart, 1.0 for both at the same settings. Each image must have contrast >=0.30, modeled blur <=1 pixel, clipped pixels <=1%, and measured carriage speed 95 to 105 mm/s. Changed settings, touching the chart during capture, or moving the fixture zeroes the trial.
