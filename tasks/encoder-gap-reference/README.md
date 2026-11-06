# Commission a linear encoder after a readhead adjustment

Dual YAM manipulation in the Isaac Lab Arena station. Use the blue tab to set the readhead gap, move the orange tab toward the rail to lock it, push the black carriage from its home stop to the far stop and back, then hold and release green CAL.

## Reproduce

With the installed Yam Arena and Inspect Robots environment, source its yam.env (the Brev launchable installs it at /data/IsaacLab-Arena/yam.env). Run bash run.sh demo or bash run.sh dream. The default Inspect Robots seed is 0. Object jitter is disabled and control is at 20 Hz. The demo is capped at 720 steps; Dream at 2,400. Each run writes an Inspect Robots JSON log, camera frames, and a measured fixture trace under evidence/. View the score printed as task_score or in the JSON, and export videos with inspect-robots video LOG.json.

## Fixture and reset

See task.py for complete collision geometry, materials by appearance, and positions in metres. The aluminum base is centered at (295,-1) mm on the standard tabletop (world z=747.5 mm). The rail is centered at y=-48 mm. Its carriage starts at x=420 mm; the far stop is x=170 mm and its orange reference mark is x=295 mm. The blue and orange tabs start at (229,98) and (327,98) mm respectively, each with 15 mm of travel toward the rail. The green spring-return CAL cap is at (346,-127) mm with 7 mm vertical travel. Clear the trial state and return each joint to its rest point on reset; the home and end guides and protected scale stay in the fixture. Nothing is consumed.

The four movable parts use prismatic joints and move through simulated contact and joint dynamics. The weak centering springs approximate detents on the two tabs and carriage. The optical readhead gap is an instrument reading inferred from blue tab position; optical interference, real electronics, tool wear, and deformation are outside the model. The scale is protected by fixed cover geometry. The orange lock tab's retained position is used as an indication of its lock state; screw torque and clamping force are not modeled.

## Scoring

The blue and orange tab travels must each reach at least 12 mm and stay there. This corresponds to a gap of approximately 2.03 to 2.10 mm in the model. With the carriage within 3 mm of home, this earns 0.25. After an outward pass across the 125 mm reference and reaching at least 247 mm from home, the score is 0.50. Return across the reference and within 3 mm of home for 0.75. Hold CAL at least 3.5 mm down for ten consecutive steps, then release it for forty steps with the tabs and carriage still valid for 1.0. Pressing CAL before a valid return, overtravel outside [-1,251] mm, fixture translation beyond 9 mm, or a fixture rotation beyond about 7.25 degrees zeroes the trial. Only the final score counts. All measurements are read from the physics state without adjusting it.
