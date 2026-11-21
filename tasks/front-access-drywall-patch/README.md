# Front-access drywall patch repair

Repair a square opening between studs from the front. The two timbers start loose and longer than the opening. Each must pass through the opening, turn inside the cavity and be held against the old board while it is fastened. Secure the new panel to the resulting backing and remove the lifting tool. The next trade operation is mesh and joint compound.

## Run

Use the Python interpreter from the official RoboCurve YAM walkthrough environment. The dependency revisions are in runtime-lock.json. The launcher checks them before starting. From this folder, supply a new output directory outside the task.

~~~bash
python run.py --mode setup --gpu auto --log-dir ../drywall-reset
python run.py --mode demo --gpu auto --log-dir ../drywall-demo
python verify.py ../drywall-demo --media ../drywall-demo-media
python run.py --mode dream --gpu auto --log-dir ../drywall-model
python verify.py ../drywall-model --media ../drywall-model-media
~~~

The last run uses Dream credentials configured with the official SDK. Configuration-only inspection is available with run.py --mode dream --config-only --log-dir ../drywall-config. One fixed reset, no object jitter. Physics runs at 240 Hz and the original dual YAM station operates at 20 Hz. The model sees calibrated overview, left wrist and right wrist RGB at 320 × 176, joint state and the instruction in task.json. Both policies get at most 2,400 actions (120 simulated seconds). The separate scripted demonstration uses known geometry, measured object states and joint targets. Actual wall loads, grips and released support come from the simulator.

The default repair begins with both strips, panel and driver on their rests. The base YAM scene loads a small build anchor; run.py attaches the workcell there before simulation. Each episode writes the source, native poses and contacts, action log, official score and original frames. verify.py rechecks the final score and can encode the full original episode. Setup and demonstration are local. Dream uses the official Inspect Robots adapter and disables additional cloud recording.

[Workcell construction, physical grading and reset](WORKCELL.md) · [Panel cutting template](panel-template.svg) · [Carrier drawing](carrier.svg) · [Exact runtime sources and versions](runtime-lock.json)

All task-specific geometry is generated from dimensions in repair_scene.py and rotary_driver.py. The calibrated station and robot are supplied by the original walkthrough distribution under its upstream licenses; no third-party models, scans or textures are copied into this task.
