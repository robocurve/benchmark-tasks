# Test two recessed spring plungers

Use a pin tool to press two recessed spring plungers through their specified stroke, then
release them to return. Park the tool and withdraw with open grippers. This represents a
mechanical stroke/return check in an unpowered service fixture, with no electrical test.
The 7 mm shaft fits into 15 mm bores; the gripper remains above the fixture.

The final scripted demonstration scored **1.0** in **724 steps** (36.2 simulated seconds),
terminated with **success**, and took **94.98 seconds wall time** on the L40S. All three
complete H.264 camera videos were downloaded. The top sequence, both left-wrist tool
insertions and final tool return were visually reviewed. Both pins achieved the required
stroke, returned, and the tool was parked; no invalid robot–pin contact was recorded.
Seven offline scorer checks pass, including actual `LeftArm`/`RightArm` actor paths.


## Reproduce

In the configured RTX YAM environment, from this folder:

```bash
uv pip install -e . --no-deps
python -m unittest discover -s . -p test_score.py
inspect-robots run --embodiment yam_plunger_gauge --policy plunger_gauge_scripted \
  -E task="$PWD/task.py" -E object_jitter=0 -E device=cpu -E top_cam=demo \
  -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction 'Use the pin tool to test both recessed spring plungers. Press each at least 8 mm, release both, return the tool to its stand, and open both grippers.' \
  --scorer task_score --max-steps 1800 --seed 0 --store-frames \
  --log-dir /path/outside/repository/plunger-demo
```

The configured cluster uses `$SIM_ROOT/cluster/run-arena.sh` inside the L40S allocation. Render the completed
JSON with `inspect-robots video <run.json>`. For Dream-1 use the same scene, embodiment,
seed and budget with `--policy dream -P model=dreamlabs/dream-1-beta`.

## Fixed setup and reset

World x points forward, y left and z up. The tabletop is at z=0.7475 m. Every mesh is
self-contained; `build_assets.py` regenerates it with usd-core. All lengths below are mm.

- The fixture has a 120 × 230 × 10 base at (0.35,0.09) m, side ribs, bolts and two steel
  guide wells centered at xy=(0.35,0.04) and (0.35,0.14) m. Their bores have radius 7.5,
  outer radius 22 and height 40, centered 30 above the table. Yellow mouth rings have
  outer radius 24 and top height 51. Circular visuals use 48 segments; 32 radial box
  sectors approximate each collision wall. These extend slightly beyond circular
  outer corners; their inner planes preserve the bore clearance.
- Each pin is a rigid 10-diameter × 12 cylinder with a thin orange contact-face visual,
  assigned mass 30 g and nominal root center 30 above the table. Its Z guide has limits
  −10 to 0, stiffness 500 N/m, damping 2 N s/m and a 20 N drive cap. Gravity causes about
  0.6 mm of initial spring deflection. The face starts roughly 15 mm below the mouth.
- The rigid pin tool is 80 g. Its steel shaft is 7 diameter × 80, centered 20 below its
  root; the tip is 60 below the root. An 18 × 24 × 30 rubber grip and 26-diameter cap
  are rigid parts of the same body. The tool starts at root (0.20,0.27,0.8175) m.
- Its fixed stand has a 70 × 70 × 10 base and an annular guide extending to height 55,
  with 17-diameter bore and 44-diameter outside. The shaft tip rests on the base and
  the grip rests on the guide rim. Guide clearance accommodates ordinary grasp error.

Reset restores both pins and the tool to these poses, returns both arms to ready, clears
history, and settles physics. The world joint anchors are authored in these coordinates:
zero jitter is required. The guide constraints and springs remain passive throughout a
trial. The controller never sets a pin pose, joint target or spring equilibrium.

## Numeric scoring

Use the final score. For each pin, PhysX must first report pin–tool contact. Its Arena
reference-center z must then be at most tabletop+22.1 mm for six consecutive 20 Hz steps
(0.3 s), corresponding to at least 8 mm from its nominal joint zero. Brief strokes do
not accumulate. A tested pin earns 0.4 only after it returns to within 1.2 mm of nominal
reference height tabletop+30.1 mm, with linear speed under 10 mm/s and angular speed
under 0.5 rad/s. This includes the approximately 0.6 mm gravity preload.

Add 0.2 only when both pins qualify and the tool is parked, resting, and both grippers
are at least 75% open with both TCPs over 90 mm from its reference center. Parking
requires xy within 6 mm of (0.20,0.27) m, bottom within 3 mm of tabletop+10 mm, vertical
extent under 84 mm and rest. Holding 1.0 for one second succeeds.

Direct robot–pin contact permanently invalidates. So does a tool bottom below tabletop
minus 20 mm, nonfinite tool position, center outside x=(−0.1,0.95) m or |y|>0.62 m.
Invalid trials score zero. Untested or unreturned pins earn zero; all other outcomes are
covered by those rules. The test history does not retain an intermediate score maximum.

## Physics and limitations

PhysX runs at 240 Hz, control at 20 Hz. The pins use maximal-coordinate prismatic joints
to world and ordinary rigid-object state reporting. Contact reports establish tool use;
there is no force sensor. Geometry and springs are training approximations, not calibrated
hardware. Materials use approximate station friction 1.2/1.0, 5 mm contact offset and zero
rest offset. The tool/fixture do not deform. The demonstration uses privileged pose feedback
and FK, sending only robot joint targets; the low press waypoint compensates for arm drive
compliance under spring load. Robot drives and collision geometry are unchanged.

Source pins: YAM `0059d02a1a91bc6834c1d0846022891df0c78730`, Arena
`c7edf3a9d5d8276565208f72422f8f8a0806d6cf`. Runtime: Python 3.12.12, Isaac Sim 6.1.0.0,
Isaac Lab 17.0.2, Arena 0.3.0, Inspect Robots 0.60.0, NumPy 2.3.1, Torch 2.11.0+cu128,
NVIDIA L40S. Contact-enabled `yam_plunger_gauge` is required for both demo and model trials.

## Dream-1 evaluation

`dreamlabs/dream-1-beta` completed with a final score of **0** in **1,800 steps**,
terminating with `max_steps`. Wall time including initialization and inference
was **236.06 seconds**. The run used seed 0, zero jitter, 20 Hz control, 640 × 360
saved camera frames, the `demo` top-camera configuration and the same runtime source
hashes and physics as the expert demonstration. The model received ordinary camera images
and joint state. All three complete camera videos were downloaded and visually reviewed.

The reviewed sequence shows the left arm approaching the tool stand and fixture, but the pin tool remains in its stand and neither recessed plunger completes a test stroke. Final contact history has no tool-pin contacts and no tested plungers. The top sequence, final frame and both wrist views were inspected.

`validation.json` records the completed result, runtime revision and video hashes.
The Dream video, full-score demo and setup image are linked through the media ID in `task.json`.
