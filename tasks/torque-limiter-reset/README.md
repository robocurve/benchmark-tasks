# Prove an overload limiter releases and resets

Press blue RUN and let the silver output wheel turn at least once. Hold orange JAM until the output stops while the input keeps turning. Release JAM, press green RESET, and let the output turn once again.

Run `bash run.sh settle`, `bash run.sh demo`, `bash run.sh ablation`, or `bash run.sh dream` in the RoboCurve Isaac Lab Arena launchable. Each command starts a fixed-layout trial and records logs, camera frames and measured telemetry under `evidence/`. Trials use 20 Hz control with up to 2,400 steps. The demo uses a shorter cap. The scene has zero object jitter and no randomized starting settings.

The task measures overload protection and restoration of a stalled drive. The guarded fixture has two revolute shafts and three physical, spring-return caps. A RUN press enables the motor. Holding JAM applies a spring brake against the output shaft, and RESET reengages the clutch only after the brake is released. The 0.12 N·m load, compliant coupling, reflected inertia, and calibrated 0.23 N·m detent represent a bench model. The scorer reads the resulting rotation. The commands apply joint torques rather than changing a shaft pose.

Score 0.25 after the output completes a loaded turn, 0.5 after the jam releases the clutch, 0.75 after the reset restores engagement, and 1.0 after another loaded turn with RESET released. An out-of-sequence clutch trip or a moved fixture zeroes the trial. The no-RESET ablation checks that a release without recovery cannot earn full score.
