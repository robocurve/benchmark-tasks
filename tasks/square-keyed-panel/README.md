# Assemble a four-piece keyed square panel

Fit four colored, rigid components into a flat square at the workbench center. Distinct
T-shaped mating profiles require the correct piece identities and orientations. Four orange
corner marks locate the target; they do not constrain or support the assembly.

The seeded scripted physics demonstration scored **1.0**, terminated with **success**, and
completed **3,507 control steps** (175.35 simulated seconds) within the 3,600-step limit.
Wall time including initialization was **600.35 seconds** on an NVIDIA L40S. The complete
640 × 360, 20 Hz top-camera video is **175.40 seconds**; all three camera videos were downloaded
and inspected, including the final interlocks in the right wrist view. `validation.json`
records the result and runtime source hashes. The completed model trial is recorded below.

## Reproduce

Use the configured Linux YAM Arena environment with RTX rendering. From this directory,
activate that environment and register the demonstration policy:

```bash
uv pip install -e . --no-deps
python -B -m unittest -v test_task
```

On the configured cluster, set `SIM_ROOT` to the installed runtime root, then run from a GPU
allocation. Both this task folder and its editable installation must be available there.

```bash
"$SIM_ROOT/cluster/run-arena.sh" inspect-robots run \
  --embodiment yam_arena --policy square_scripted \
  -E task="$PWD/task.py" -E top_cam=demo -E object_jitter=0 \
  -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction 'assemble the four keyed pieces into a square at the center of the table' \
  --scorer task_score --max-steps 3600 --seed 0 --store-frames \
  --log-dir /path/outside/the/repository/square-demo
```

Add `-P trace_path=/absolute/path/poses.jsonl` to record measured piece poses and controller
progress every ten actions. Render the terminal run with `inspect-robots video <run.json>`.
Keep logs, raw frames, screenshots and videos outside the public task directory.

The tested sources are YAM Arena commit `0059d02a1a91bc6834c1d0846022891df0c78730` and
Isaac Lab Arena commit `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`, with Inspect Robots 0.60.0
and Isaac Sim 6. The configured launcher exposes the node's NGX library for rendering.

## Fixed setup

World x points forward, y left and z up. The tabletop is at z = 0.7475 m. The nominal
assembly center is (0.36, 0) m on the 110 × 130 cm station workbench.

- Four loose components have 50 × 50 × 24 mm main bodies and assigned mass 80 g each. Their
  internal edges have 12 mm projecting T tabs or matching recesses: neck width 14 mm, head
  width 20 mm, distinct offsets −12, −9, −6 and −3 mm, with 2 mm nominal slot-width clearance.
- Their local-bounds reference centers start at (0.190, 0.180), (0.285, 0.205),
  (0.180, −0.180), and (0.285, −0.205) m. Initial yaw angles are 0, 0.40, −0.40 and 0.65 radians.
  Initial center height is 0.7595 m. Jitter is disabled.
- Four fixed orange 6 × 6 × 0.5 mm marks are centered at x = 0.307 or 0.413 m,
  y = ±0.053 m, z = 0.74775 m.

`geometry.py` and `task.py` specify every profile, color and pose. Reset restores the four
components and the robot's ready state. Nothing is consumed. The material is a stylized rigid
polymer approximation; mass, friction and compliance are not calibrated to manufactured parts.

## Scoring

Award 0.25 for each piece whose Arena reference centroid is within 4 mm horizontally and in
height of its specified target, with full 3-D rotation within 8° of zero, linear speed below
0.01 m/s and angular speed below 0.5 rad/s. Arena transforms the center of the object's local
geometry bounds into the world; this reference differs from recomputing a world AABB center.

| Piece | Target reference centroid (m) |
| --- | --- |
| 1, blue | (0.341, 0.019, 0.7595) |
| 2, orange | (0.385, 0.019, 0.7595) |
| 3, green | (0.341, −0.025, 0.7595) |
| 4, purple | (0.385, −0.025, 0.7595) |

When all four qualify individually, each seam's relative displacement must also match its
target displacement within 2 mm on every axis. Otherwise cap the score at 0.75. No qualifying
pieces scores zero; all other partial outcomes score 0.25 per qualifying piece. The final
score is used. Holding 1.0 for one second terminates the trial successfully.

## Physics and demonstration

The pieces are compound rigid bodies spawned at runtime with Isaac Lab primitive APIs;
there are no authored puzzle USD files. `procedural.py` scopes a spawner adapter to this task
and restores the stock object factory after construction. This uses pinned framework
internals and needs revalidation with a different framework version. PhysX runs at 240 Hz,
with 20 Hz control. Piece contact/rest offsets are 0.5/0 mm, position/velocity solver iterations
32/4, and static/dynamic friction 1.2/1.0. Robot colliders, drives and effort limits are unchanged.

`square_scripted` reads privileged simulator object poses and sends robot joint targets only.
It does not attach objects, write object poses, change the scorer, or bypass contacts. It
uses side-edge grasps, a −20 mm finger-joint hold target, a 70 mm TCP travel height, bounded
pose feedback and measured-error holds during descent. The right arm waits for the left
arm to clear the assembly. `demo_calibration.json` contains small, fixed waypoint adjustments
for measured release shifts; these have no effect on scoring targets.

Later placements follow the first component's measured frame with 0.5 mm nominal body
clearance. `closure.py` chooses the fourth piece's pose against all three measured neighbor
collider footprints, restricted to the original position and seam tolerances. The script
stops an insertion after excessive tilt or a sustained failure to align. The gate, clearance
planner and privileged feedback are demonstration aids, not additional scoring rules.

Eight offline checks cover profile dimensions/clearance, uniqueness on a 1 mm grid over
right-angle piece permutations, scoring, calibration independence, frame geometry, fingertip
contact regions, nominal timing, and the final-corner fit. The uniqueness check is limited to
its stated planar enumeration; it is not a proof over arbitrary 3-D configurations.

## Dream-1 evaluation

`dreamlabs/dream-1-beta` completed with a final score of **0** in **3,600 steps**,
terminating with `max_steps`. Wall time including initialization and inference
was **364.40 seconds**. The run used seed 0, zero jitter, 20 Hz control, 640 × 360
saved camera frames, the `demo` top-camera configuration and the same runtime source
hashes and physics as the expert demonstration. The model received ordinary camera images
and joint state. All three complete camera videos were downloaded and visually reviewed.

The reviewed sequence shows the right arm attempting to grasp and reposition the purple component. The parts remain separated at the end, and no square assembly is completed. The top sequence, final frame and a right-wrist interaction view were inspected.

`validation.json` records the completed result, runtime revision and video hashes.
The Dream video, full-score demo and setup image are linked through the media ID in `task.json`.
