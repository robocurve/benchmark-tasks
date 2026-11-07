# Secure a leaning panel with a foot stopper

A free balsa-wood panel leans against a wall. Seat a loose stopper in the socket at
its foot, withdraw both grippers, and pass a repeatable outward-pull test. The
stopper restrains the panel through contact with its foot and the socket's front
wall. No joint, weld, pose overwrite or conditional freezing secures the panel.
The scripted demo supports the panel with one arm while the other places the
stopper. Any safe strategy meeting the outcome conditions is valid.

## Run

Use the Robocurve YAM launchable: Isaac Lab-Arena 0.3.1, Isaac Sim 6.1,
`yam-arena` and Inspect Robots 0.60.0. The runner installs its changes only in the
current Python process. A direct `inspect-robots` invocation without `run.py`
does not apply the tilted spawn, materials, safety monitor or physical pull.

From the repository root on the GPU VM:

```bash
source ~/IsaacLab-Arena/yam.env
python tasks/stabilize-leaning-panel/run.py run \
  --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/stabilize-leaning-panel/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the leaning panel, seat the orange stopper in the green socket at its foot, then release and withdraw both hands." \
  --scorer task_score --max-steps 1200 --store-frames \
  --log-dir runs/stabilize-leaning-panel/demo
```

For Dream-1, replace `--policy yam_scripted` with `--policy dream` and choose a
fresh log directory. The fixed layout uses seed 0 and no object jitter. The
limit is 1,200 control steps (60 simulated seconds at 20 Hz). The measured demo
takes 520 steps (26 simulated seconds), 94.2 seconds wall time on an L40S
including startup. Dream-1 takes 1,200 steps (60 simulated seconds), 201.4 seconds
wall time on the same L40S, and scores 0.0. Both trials use seed 0 and no jitter.
Exact results are in `validation.json`; do not infer a
successful trial from the command's exit status alone.

`panel-trace.jsonl` contains the actual panel and stopper poses, safety latch,
release state, proof phase and score at each control step. The monitor samples
object safety at the 240 Hz physics rate. Read the Inspect Robots result JSON
for `status`, errors and the final `task_score`.

## Scoring and physical reproduction

All dimensions and initial poses are in `scene_inventory.json` and `task.json`.
The panel starts 20 degrees from vertical. Shallow side guides constrain sideways
motion but leave the foot free to slide toward the robot. The stopper is a
rectangular block, an allowed stopper variant of the proposed wedge task.

A seated stopper has its centre within 4 mm in x and y of its target, its bottom
within 3 mm of the base surface, and tilt and yaw within 5 degrees of the socket
axes. Translation must be below 15 mm/s and rotation below 0.15 rad/s.

Safety is irreversible within an episode: panel lean must remain from 10 to 32
degrees, its foot's horizontal displacement from its initial mark at most 18 mm,
and its foot height within 10 mm of the initial value. A moving object below the
table by more than 5 mm, outside x 5–50 cm or |y| 40 cm, or a nonfinite state
also fails the episode. Zero is retained even if the robot restores the layout.

Both grippers must be at least 80% open, with each TCP at least 8 cm outside both
objects' complete bounding boxes. Once the stopper is seated and both grippers
clear, the test applies a 0.6 N horizontal pull toward the robot at the centre
of the panel's bottom edge for 0.4 seconds. It then checks 2 seconds of stability:
panel and stopper position range at most 3 mm in each axis, panel translation
below 10 mm/s and rotation below 0.1 rad/s, with the same final speed and spin limits for the stopper. Returning a hand or losing the stopper's geometric seating during this test fails the episode. Brief motion under the pull is permitted; both objects must meet the final stillness check. The pull is a world-frame external
wrench, not a pose edit or scripted panel motion.

Score 0.5 for a seated stopper while all safety limits hold, awaiting completion
of the release test; 1.0 after the whole test passes; 0 otherwise. Untouched start
scores zero. Scoring uses the final state, not the best historical score.

For a tabletop reproduction use only a small, low-mass panel and a bounded
catch area. Match the dimensions, mass, measured friction and starting angle;
mark the foot location and stopper target. Record a side video against angle
and displacement marks, plus a top view of the socket. Apply the test pull at
the foot using a 0.6 N spring scale or a calibrated 61 g hanging mass with a
low-friction pulley, for 0.4 s. Use a ruler in the video for displacement and the
2 s stability interval. These proof-test tools are examiner equipment, not props
manipulated by the robot. This scale analogue does not establish full-size panel
handling safety.

## Verify

Offline (NumPy, pytest and the YAM package):

```bash
pytest -q tasks/stabilize-leaning-panel/test_task.py
ruff check tasks/stabilize-leaning-panel
```

On the GPU, run the paired contact control:

```bash
python tasks/stabilize-leaning-panel/probe.py
```

The probe applies exactly the same pull with no stopper, with a seated stopper,
and again without one after reset. Its test harness places the stopper directly
only to isolate physics; it is not a robot demonstration. It fails unless both
unbraced cases violate the displacement/tilt limits and the braced case remains
safe with its stopper seated. The real demonstration must grasp, move and
release the stopper using normal robot actions.

## Fidelity and current status

Rigid rectangular panel, wall, stopper and base; no wood flex, fracture, tool
fastening, injury model, or full-size construction dynamics. Panel/base static
friction is 0.22 and dynamic friction 0.176, combined with `min`; wall and shallow
foot-guide contacts are frictionless. Other objects retain the station's default friction. Gripper
clearance uses measured opening and TCP-to-bounding-box distance. It does not
measure every arm link's clearance. The standardized test disturbance is an
external force; its examiner cord/pulley is not rendered.

The scripted demo reaches 1.0 in 520 steps. Both unbraced physical controls
exceed the 18 mm foot-slip limit (24.66 mm); the braced control stays within it
(12.47 mm). Sixteen scorer/geometry regression checks and lint pass. The local
evidence includes the actual H.264 demo, setup image, raw trial log and safety
trace. Dream-1 scores 0.0 after its full 1,200-step limit. Both videos use the
same task code, instruction and starting layout. The official upload script
records their media ID in `task.json`.

## Preview

On the GPU VM, from the repository root after sourcing `yam.env`:

```bash
python tasks/stabilize-leaning-panel/view.py --demo
```

Omit `--demo` to inspect the starting layout. The panel has Pause / Resume,
Reset layout, Demo / Static and camera buttons. Scroll to zoom, Alt/Option-drag
to orbit, middle-drag to pan, or select an object and press F to focus. Use the
VM's noVNC desktop through the existing SSH/Brev port forward for port 6080.
Stop a foreground preview with Ctrl+C before running a recorded trial.
