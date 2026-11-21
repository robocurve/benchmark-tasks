# Hold a Koch v1.1 robot arm up as its power is cut for a repair, and lay it limp in its cradle pose without dropping it

A Koch v1.1 follower arm (the open-source 3D-printed arm LeRobot drives as `koch_follower`) stands on the bench in its working pose, held by its servos. The robot closes one gripper on the arm's upper arm and the other on its wrist; one second after both have closed, the arm's torque is cut and its five joints go limp. The robot carries the limp arm, lays it forward into its cradle pose on three foam blocks, and lets go.

**Ability tested:** carrying a limp, gravity-loaded five-joint chain at two points through a power cut, and setting it down to a joint-space pose without a hard landing. The shoulder and elbow fold under the arm's own weight as soon as the torque goes, so both holds must already carry it, and the two hands must move together as it is lowered.

**Why it matters:** robot arms are repaired with their power off, and an arm built on back-drivable servos cannot hold itself up when powered down. Trossen warns that terminating control of its Dynamixel-driven WidowX arms "while the arms are not in their sleep configurations will cause them to collapse!". The Koch v1.1 uses servos of the same Dynamixel X series (XL430-W250 and XL330-M288), and LeRobot uses it to record robot-learning datasets. Unheld at the cut, the simulated Koch's forearm falls and a link passes 0.9 m/s (the `none` run below).

**Hands:** both grippers are asked for, and both are on the arm at the cut. The left holds the upper arm's 13 mm printed bracket 6 cm out from the shoulder lift axis (11.5 cm above the bench in the working pose) and carries the shoulder's load; the right holds the wrist's XL330 housing (15.5 cm up) and sets the elbow and wrist. Limit: in the simulator the right hand's pinch alone holds the light arm (0.22 kg moving) rigidly enough to lay it (the `onehand` run, held to 1/3 by the scoring because the left let go); a real pinch on a 22 mm housing may let the elbow buckle, but this is not shown here.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py`, `gatekit.py`, `taskspace.py` and `koch_model.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

**The Koch model:** `koch/follower_arm_v1.1.urdf` and the meshes in `koch/` are the Koch v1.1 follower arm's, from ManiSkill's robot assets (`mani_skill/assets/robots/koch`), which take them from [jess-moss/koch-v1-1](https://github.com/jess-moss/koch-v1-1) with fixed joint limits and axes. They are used under the Apache License 2.0 (Copyright 2024 The Hugging Face team), in `koch/LICENSE`. `koch/convert.py` converted each visual mesh (GLB or STL) to STL and clustered it to a 1.5 mm grid; `koch_model.py` reads the URDF's joint axes, origins, limits, link masses, centres of mass and inertias. One fix: the URDF puts `link_6`'s centre of mass 22 cm out along its own axis, outside the link, so its mass is kept and its centre is put at its meshes' centroid.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1800 \
  --instruction "One of this robot arm's servos has failed and stopped its servo bus, so the arm cannot be driven to a rest pose; its power will be cut for the repair and it will go limp. Hold it up so it does not drop: close both grippers on it, and one second later the power is cut. With both grippers still on it, lay it down in its cradle pose (shoulder lift 0.45, elbow flex 0.8, wrist flex -0.35 rad, the other joints 0) without dropping it, then let go."
```

## Scripted demo

From this folder:

```bash
python demokit.py task.py OUT --mode full                                  # full score, with a video
python demokit.py task.py OUT --no-record --mode none,onehand,wrong,gate     # the lazy routes and the unheld fall
```

The demo is a scripted reference. It reads the Koch's joint angles (its encoders report with torque off), which a policy would see through the cameras. Each hand comes from the YAMs' side, level, fingers along the arm's pitch axes: the left closes across the upper arm's bracket 6 cm out from the shoulder axis, the right across the wrist's servo housing. After the cut it lowers the arm to the cradle pose in 30 steps of 0.3 s, each hand turning with its link; each step it reads the joints and moves each hand's goal by half of the gap between where its link is and where it should be. The arm comes to rest on its three foam blocks. The right hand then opens where it is, draws 13.5 cm straight back along its fingers and returns to its ready pose; then the left, which opens only to 4 cm before it slides out.

## Building it for a real trial

- **Arm:** a Koch v1.1 follower built from [jess-moss/koch-v1-1](https://github.com/jess-moss/koch-v1-1): its bill of materials totals US$278 (two Dynamixel XL430-W250-T at US$50, four XL330-M288-T at US$24, idler and frame sets, a Waveshare serial bus servo board, a voltage reducer, a 12 V supply, wires, a table clamp, a screwdriver and USB cables), plus its 3D-printed links from the STLs in that repository (Apache-2.0). Its base is clamped to the bench at the stated position.
- **Power cut:** a script calls LeRobot's `DynamixelMotorsBus.disable_torque()` (the arm's `koch_follower` bus) when the operator presses a button once both YAM grippers have closed on the arm, and logs `Present_Position` with `sync_read` at 20 Hz throughout. The bus stays healthy, so the encoders keep reporting; the "failed servo" of the instruction is simulated.
- **Foam pad and blocks:** a 2 cm foam pad (polyurethane, any firm grade), 12 x 44 cm, centred at (32.5, -8) (its near edge at x = 26.5, clear of the hands as they lay the arm low), where an unheld arm lands; on it three blocks 6 cm wide in x at x = 30, cut to the arm's underside in the cradle pose: under the elbow, 2.5 cm long at y = 5.75, top 7.7 cm above the bench; under the forearm, 3.5 cm long at y = 0.25, top 5.6 cm; under the gripper, 4 cm long at y = -11, top 3.7 cm (all are listed at load in `task.py`).

Calibrate before trials:

| Value | In the simulator | How to measure it on the real arm |
|---|---|---|
| Static friction of each unpowered joint | 0.03 N m per XL430-W250 (shoulder pan and lift), 0.012 N m per XL330-M288; dynamic 70 % of it | **Estimate** (about 2 % of stall torque; ROBOTIS publishes no torque-off friction). Torque off, hang a spring scale on a link at a known radius and read the pull at which the joint starts to turn |
| Viscous damping | 0.02 N m s/rad per servo | **Estimate.** Let a link fall a few degrees and fit the speed from `Present_Position` |
| Rotor inertia seen at the joint | 0.005 kg m² per XL430, 0.002 per XL330 | **Estimate:** a small coreless rotor's inertia times the gear ratio squared (258.5:1 and 288.35:1, ROBOTIS e-Manual) |
| Powered hold | 10 N m/rad, 0.3 N m s/rad; torque limit 1.4 N m (XL430-W250 stall at 11.1 V), 0.52 N m (XL330-M288 stall at 5 V) | Only holds the start pose; any stiff position hold does |
| Arm mass | 0.29 kg with the base, 0.22 kg moving (the URDF) | Kitchen scale on the arm off its base |

## Scoring

See `scoring` in `task.json` and `Trial` in `task.py`. In a real trial, from the logged joint positions (link speeds from them through the arm's kinematics) and the video.

- **1/3, caught:** when the torque is cut, both grippers are closed on the arm (a point of each finger within 1.5 cm of its surface, the mean points of its two fingers under 6 cm apart).
- **2/3, laid:** also every joint within 10 degrees of the cradle pose at the last moment both grippers held it, and now, the arm still (no point faster than 1 cm/s) and resting on its foam. A joint that sags home under gravity after a hand opens does not count, and a hand that lets go before the arm is laid leaves 1/3.
- **1.0, let go:** then both whole grippers (a lattice 2 cm apart over the fingers, pads, palm, housing and 12 cm of the wrist) at least 5 cm from the arm's surface, the arm still laid, for 2 s.
- **A drop voids the run (0):** after the cut, any point of the arm faster than 0.5 m/s, or a link striking the bench, pad or foam faster than 0.25 m/s. Unheld, the arm passes 0.58 m/s within 0.1 s of the cut and peaks at 0.9 m/s (`none`); the demo's lowering peaks at 0.04 m/s.

Open hands hovering round the arm score nothing: the arm is carried only by closed pinches, so it falls at the cut and the drop voids the run. The two pinches and the 5 cm clearance are read from the arm's and grippers' geometry, and are visible in the front and overhead views of the film.

The speeds are of the arm's surface points (from the Koch meshes) computed from the joint readings through the URDF's kinematics, so a few degrees of drift in a light wrist joint is not a fall, and a dropped arm is.

## Results

| Run | What it does | Score |
|---|---|---|
| `none` | The arms wait; at 30 s the torque is cut and the unheld arm's forearm falls (a link at 0.58 m/s, peak 0.9 m/s) | 0.0 |
| `onehand` | Both hands hold through the cut; the left lets go and the right lays the arm alone by its wrist | 0.33 |
| `wrong` | Laid with the wrist flex 30 degrees off its cradle angle; the hands never set it within 10 degrees | 0.33 |
| `full` | Both hands carry the arm through the cut, lay it in the cradle pose, let go one at a time and return to their ready poses | 1.0 |
| Dream-1 (the model's run, 1,800 steps) | The model never closed its grippers on the arm; at 30 s the torque is cut unheld and the forearm falls (a link at 0.58 m/s: the drop voids the run); one session, no expiry | 0.0 |
