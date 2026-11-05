# Transfer a loaded fixture pallet with a forked lifter

Insert a removable two-tine lifter beneath a loaded workholding pallet, lift it, transfer
it to two inspection rests, lower it, withdraw the fork and return the tool. The pallet
remains a separate free rigid body supported by contact and friction throughout the carry.
The loaded blank is a rigid part of the pallet; this task does not assemble or machine it.

The scripted physics demonstration scored **1.0** in **957 steps** (47.85 simulated
seconds), terminated with **success**, and took **103.52 seconds wall time** on an L40S.
All three complete H.264 videos were downloaded. The top sequence and final placement
were visually inspected: the loaded pallet rests on the green rails, the fork is withdrawn
and returned, and both hands are clear. Seven offline scorer checks pass, including
actual PhysX robot prim paths. The completed Dream-1 trial is recorded below.


## Reproduce

From this folder in the configured YAM Arena RTX environment:

```bash
uv pip install -e . --no-deps
python -m unittest discover -s . -p test_score.py
inspect-robots run --embodiment yam_pallet_lifter --policy pallet_lifter_scripted \
  -E task="$PWD/task.py" -E object_jitter=0 -E device=cpu -E top_cam=demo \
  -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction 'Use the forked lifter to move the loaded pallet onto the two green inspection rests. Withdraw and return the lifter to its supports, then open both grippers.' \
  --scorer task_score --max-steps 1800 --seed 0 --store-frames \
  --log-dir /path/outside/repository/pallet-demo
```

Use `$SIM_ROOT/cluster/run-arena.sh` inside the L40S allocation. Render the terminal
JSON with `inspect-robots video <run.json>`. The model trial uses the same embodiment,
scene, seed and limit with `--policy dream -P model=dreamlabs/dream-1-beta`.

## Fixed setup and reset

World x points forward, y left, z up. Tabletop z=0.7475 m. `task.py` lists starting poses;
`build_assets.py` regenerates self-contained assets with usd-core. Lengths below are mm.

- The loaded pallet is one 120 g rigid body. A 120 × 160 × 6 plate carries a central
  40-diameter × 12 round blank, decorative grooves and fasteners. Four 18 × 14 × 20 feet
  are centered at local x=±45, y=±70, z=−13. Root starts at (0.40,0.08,0.7705) m,
  with feet resting on the table and 20 mm of underside access above the table.
  The reference centroid is 3.8 mm below the root due to asymmetric vertical bounds.
- The 80 g steel lifter has two 150 × 12 × 5 tines centered at local x=95, y=±30,
  z=2.5; a crossbar, handle and rigid 35 × 24 × 18 grip sleeve join them. The sleeve
  center is 100 behind the root and 16 above it. Root starts at (0.24,0.28,0.7825) m.
  Its local-bounds reference center is offset (17.5,0,12.5) from the root.
- Two fixed tool supports at (0.18,0.28) and (0.34,0.28) m have heights 40 and 35.
- Two fixed inspection rails are centered at x=0.37, y=−0.19 and −0.05 m. Each is
  150 × 18 × 15, with green visual seat marks and end fasteners. The open central
  space lets the fork lower clear and withdraw after the pallet's feet contact rails.

Reset restores the pallet, lifter and both arms, clears contact/failure history and
settles physics. Seed 0 and zero jitter reproduce the demonstrated arrangement.
Nothing is consumed. Masses and materials represent lightweight training fixtures.

## Numeric scoring

Use the final score. PhysX must report lifter–pallet contact, then the pallet's entire
world AABB must lift above tabletop+60 mm at least once. Without that verified lift,
score zero. A verified pallet currently held above that clearance earns 0.25 if not seated.

A verified pallet earns 0.75 when its reference xy is within 12 mm Euclidean distance of
(0.37,−0.12) m, its bottom is within 4 mm of tabletop+15 mm, full 3-D orientation is within
8° of identity, vertical extent is below 42 mm, and it rests (linear speed under 10 mm/s,
angular speed under 0.5 rad/s). Otherwise it earns 0.25 only while fully above the lift
clearance, and zero elsewhere. This makes a misplaced or flipped pallet earn zero at rest.

Full score additionally requires the lifter returned, both grippers at least 75% open,
and both TCPs over 90 mm from both lifter and pallet reference centers. Parking means
lifter xy within 25 mm of (0.2575,0.28) m, bottom within 6 mm of tabletop+35 mm, vertical
extent below 35 mm and rest. Holding 1.0 for one second terminates successfully.

Any robot–pallet contact permanently invalidates, as does a lifter/pallet bottom below
tabletop minus 20 mm, nonfinite position, center outside x=(−0.1,0.95) m, or |y|>0.62 m.
Invalid trials score zero. No score maximum is retained. The tested paths include both
scene names and the actual `LeftArm`/`RightArm` PhysX actor paths.

## Physics and limitations

PhysX runs at 240 Hz with 20 Hz robot actions. The lifter and pallet use 32/4 solver
iterations, 0.5 mm contact offset, zero rest offset, and approximate station 1.2/1.0
static/dynamic friction. Robot colliders and drives are unchanged. Metal/polymer looks,
mass distribution and friction are stylized and uncalibrated; neither tool nor pallet
flexes. There are no attachments, welds, pose writes or scripted payload motion.

The demonstration uses privileged object poses and FK for grasp and payload feedback.
Measured payload slip corrects subsequent robot placement commands; lowering and fork
withdrawal preserve that correction. A fixed +8 mm x/−10 mm y waypoint adjustment compensates for the measured placement shift under changing load; scoring targets remain fixed. The policy sends robot joint targets only. The
model receives ordinary images and joint state through the same contact-enabled embodiment.

YAM source `0059d02a1a91bc6834c1d0846022891df0c78730`, Arena source
`c7edf3a9d5d8276565208f72422f8f8a0806d6cf`; Python 3.12.12, Isaac Sim 6.1.0.0,
Isaac Lab 17.0.2, Arena 0.3.0, Inspect Robots 0.60.0, NumPy 2.3.1, Torch 2.11.0+cu128,
NVIDIA L40S. Source hashes and media validation are recorded in `validation.json`.

## Dream-1 evaluation

`dreamlabs/dream-1-beta` completed with a final score of **0** in **1,800 steps**,
terminating with `max_steps`. Wall time including initialization and inference
was **204.61 seconds**. The run used seed 0, zero jitter, 20 Hz control, 640 × 360
saved camera frames, the `demo` top-camera configuration and the same runtime source
hashes and physics as the expert demonstration. The model received ordinary camera images
and joint state. All three complete camera videos were downloaded and visually reviewed.

The reviewed sequence shows the left arm reaching around the fork and loaded pallet, while the lifter remains on its supports and the pallet stays in its starting position. No contact-verified lift or transfer is completed. Final history has no lifter-pallet contact and no verified lift. The top sequence, final frame and both wrist views were inspected.

`validation.json` records the completed result, runtime revision and video hashes.
The Dream video, full-score demo and setup image are linked through the media ID in `task.json`.
