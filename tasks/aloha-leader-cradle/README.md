# Hold a Koch v1.1 robot arm up as its power is cut for a repair, and lay it limp in its cradle pose without dropping it

A Koch v1.1 follower arm (the open-source 3D-printed arm LeRobot drives as `koch_follower`) stands on the bench in its working pose, held by its servos. The robot closes one gripper on the arm's upper arm and the other on its wrist; one second after both grippers press on it, the arm's torque is cut and its five joints go limp. The robot carries the limp arm, lays it forward into its cradle pose on three foam blocks, and lets go.

**Ability tested:** carrying a limp, gravity-loaded five-joint chain at two points through a power cut, and setting it down to a joint-space pose without a hard landing. The elbow and shoulder fold under the arm's own weight within a fifth of a second of the cut, so both grippers must already be pressing on it, and the two hands must move together as it is lowered.

**Why it matters:** robot arms are repaired with their power off, and an arm built on back-drivable servos cannot hold itself up when powered down. Trossen warns that terminating control of its Dynamixel-driven WidowX arms "while the arms are not in their sleep configurations will cause them to collapse!". This is the case for the low-cost research and teleoperation arms without holding brakes (Koch, SO-100, ALOHA), which fall when their torque is cut. Industrial arms mostly carry axis holding brakes, so the claim here is only for brake-less arms. These arms are in wide use: LeRobot, which drives them, has 29,051 GitHub stars and 6,124 forks ([huggingface/lerobot](https://github.com/huggingface/lerobot), GitHub API, 23 Nov 2026), the Koch v1.1 repository has 737 stars and 69 forks ([jess-moss/koch-v1-1](https://github.com/jess-moss/koch-v1-1), same date), and the Hugging Face Hub search for datasets named 'so100' returns at least 1,000 results ([API query](https://huggingface.co/api/datasets?search=so100&limit=1000), same date; the API caps a page at 1,000). The Koch v1.1 uses Dynamixel X-series servos (XL430-W250 and XL330-M288) and is used with LeRobot to record robot-learning datasets.

**Hands:** both grippers are asked for, and both press on the arm at the cut. The left holds the upper arm's 13 mm printed bracket 6 cm out from the shoulder lift axis (11.5 cm above the bench in the working pose) and carries the shoulder's load; the right holds the wrist's XL330 housing (15.5 cm up) and sets the elbow and wrist. Limit: in the simulator the right hand's pinch alone holds the light arm (0.22 kg moving) rigidly enough to lay it (the `onehand` run, held to 1/3 by the scoring because the left let go); a real pinch on a 22 mm housing may let the elbow buckle, but this is not shown here.

## Torque-off behaviour: sources, simulation and its bound

Does the real Koch fall when unheld, and how fast? Published sources:

- LeRobot's Koch guide ([examples/7_get_started_with_real_robot.md](https://github.com/huggingface/lerobot/blob/8cfab3882480bdde38e42d93a9752de5ed42cae2/examples/7_get_started_with_real_robot.md), line 341): "When you're done playing, you can try to disable the torque, but make sure you hold your robot so that it doesn't fall"; line 352: "you can unplug the power cord, which will automatically disable torque and disconnect the motors". So the unheld Koch falls when its torque goes.
- ROBOTIS e-Manual, [XL430-W250](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/): gear ratio "258.5 : 1", stall torque "1.4 [N.m] (at 11.1 [V] ...)", Torque Enable "0(Default)"; [XL330-M288](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/): "288.4 : 1", stall "0.52 [N.m]". Neither page gives an unpowered holding or back-drive torque, so the joints' torque-off friction is an estimate here, bounded below.
- The bound: the arm falls only if each joint's back-drive friction is below the gravity torque on it. From the URDF's link masses, the forearm, wrist and gripper load the elbow with about 0.12 N m with the forearm level (the working pose), and the shoulder lift with about 0.27 N m with the arm level. So the accepted range is up to 0.2 N m per XL430 and 0.08 N m per XL330 (two thirds of those loads), down to near zero (0.005 and 0.002 N m).

The simulator was run at the default (0.03 N m per XL430, 0.012 N m per XL330, damping 0.02 N m s/rad) and at both ends of the range (`K_FRIC430`, `K_FRIC330`, `K_DAMPING`; `python demokit.py task.py OUT --no-record --mode full,none`):

| Joint friction (XL430 / XL330, N m), damping (N m s/rad) | Unheld (`none`): first link on the foam after the cut | Peak link speed unheld | `none` | `full` |
|---|---|---|---|---|
| 0.005 / 0.002, 0.005 (low end) | 0.15 s | 0.99 m/s | 0.0 | 1.0 |
| 0.03 / 0.012, 0.02 (default) | 0.20 s | 0.90 m/s | 0.0 | 1.0 |
| 0.2 / 0.08, 0.1 (high end) | 0.40 s, striking at 0.32 m/s | 0.32 m/s | 0.0 | 1.0 |

At every point of the range the unheld arm falls onto the foam in under half a second and the drop rule voids the run, and the scripted catch scores 1.0. A real arm is accepted for trials when its fall lies in that window (the bench check in the Real trial below).

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py`, `demokit.py`, `gatekit.py`, `taskspace.py` and `koch_model.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

**The Koch model:** `koch/follower_arm_v1.1.urdf` and the meshes in `koch/` are the Koch v1.1 follower arm's, from ManiSkill's robot assets (`mani_skill/assets/robots/koch`), which take them from [jess-moss/koch-v1-1](https://github.com/jess-moss/koch-v1-1) with fixed joint limits and axes. They are used under the Apache License 2.0 (copyright 2024 Hugging Face), in `koch/LICENSE`. `koch/convert.py` converted each visual mesh (GLB or STL) to STL and clustered it to a 1.5 mm grid; `koch_model.py` reads the URDF's joint axes, origins, limits, link masses, centres of mass and inertias. One fix: the URDF puts `link_6`'s centre of mass 22 cm out along its own axis, outside the link, so its mass is kept and its centre is put at its meshes' centroid.

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
python demokit.py task.py OUT --no-record --mode none,onehand,wrong,gate,push,offpad     # the lazy routes and the unheld fall
```

The demo is a scripted reference. It reads the Koch's joint angles (its encoders report with torque off), which a policy would see through the cameras. Each hand comes from the YAMs' side, level, fingers along the arm's pitch axes: the left closes across the upper arm's bracket 6 cm out from the shoulder axis, the right across the wrist's servo housing. After the cut it lowers the arm to the cradle pose in 30 steps of 0.3 s, each hand turning with its link; each step it reads the joints and moves each hand's goal by half of the gap between where its link is and where it should be. The arm comes to rest on its three foam blocks. The right hand then opens where it is, draws 13.5 cm straight back along its fingers and returns to its ready pose; then the left, which opens fully and slides out.

## Real items

- **Koch v1.1 follower arm**, [jess-moss/koch-v1-1](https://github.com/jess-moss/koch-v1-1) (bill of materials US$278 plus printed links from its STLs, Apache-2.0).
- **ROBOTIS Dynamixel XL430-W250-T** (shoulder pan and lift, [e-Manual](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/)) and **XL330-M288-T** (elbow, wrist flex, wrist roll and gripper, [e-Manual](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/)).
- **Interlink FSR 402** force-sensing resistors (12.7 mm round, about US$7 each, [maker page](https://www.interlinkelectronics.com/fsr-402)) for the hold check.

## Real trial

**(a) Setup.** Positions in cm from the midpoint between the two YAM arm bases: x forward, y to the robot's left, heights above the bench top.
- A Koch v1.1 follower built from [jess-moss/koch-v1-1](https://github.com/jess-moss/koch-v1-1) (two XL430-W250-T at US$50, four XL330-M288-T at US$24, idler and frame sets, a Waveshare serial bus servo board, a voltage reducer, a 12 V supply, wires, a table clamp, USB cables; links printed from its STLs). Its base is clamped with the shoulder pan axis at (30, 16), the arm pointing toward -y.
- Working pose, held by its servos: shoulder pan 0, shoulder lift 1.2, elbow flex 1.2, wrist flex 0, wrist roll 0 rad (upper arm 69 degrees up, forearm level, wrist 15.5 cm up at (28, 0.6)).
- A foam pad (McMaster-Carr 86375K-series quick-recovery open-cell polyurethane foam, the medium-hard 45 psi grade, [product page](https://www.mcmaster.com/products/quick-recovery-foam), two 3/8 in sheets stacked to about 2 cm; any firm polyurethane foam of that hardness serves) 12 x 44 x 2 cm centred at (32.5, -8), its near edge at x = 26.5. On it three foam blocks 6 cm wide in x at x = 30, cut to the arm's underside in the cradle pose: under the elbow, 2.5 cm long at y = 5.75, top 7.7 cm above the bench; under the forearm, 3.5 cm long at y = 0.25, top 5.6 cm; under the gripper, 4 cm long at y = -11, top 3.7 cm.
- Hold sensors: four FSR 402 discs taped with 0.1 mm Kapton tape to the arm's grip spots, one on each face the fingers close on: both faces of the upper arm's bracket 6 cm out from the shoulder lift axis, and both faces of the wrist-roll XL330 housing. Each is read by an Arduino Uno through a 10 kOhm divider at 50 Hz.
- Power cut: a script calls LeRobot's `DynamixelMotorsBus.disable_torque()` on the `koch_follower` bus 1 s after all four FSRs first read at least 1 N together (30 s after the start if they never do), and logs `Present_Position` with `sync_read` at 20 Hz throughout. The bus stays healthy, so the encoders keep reporting; the "failed servo" of the instruction is simulated.
- Camera: a phone at 60 fps on a tripod 60 cm to the side of the arm, level with the pad, a 30 cm steel rule standing on the pad in view as a scale.
- Gripper protection: none needed; the arm's printed links and the foam are softer than the YAM fingers.

**(b) Reset.** Power the Koch, drive it to the working pose with LeRobot's `send_action`, check the pad and blocks are at their marks (tape outlines on the bench), open both YAM grippers at their ready pose.

**(c) Trial steps.** Start the logging script and the camera. The policy runs for up to 90 s (1,800 steps at 20 Hz). The torque is cut by the script as in (a). The trial ends at 90 s.

**(d) Scoring by hand** (thresholds in `task.py`):
- **Held**, a gripper counts as holding the arm when both its FSRs read at least 1 N (`F_HOLD`); a finger near the arm without pressing on it does not count. The FSRs sit only on the grip spots, so a squeeze elsewhere on the arm is not read; the simulator is more lenient here: it counts finger contact force on any Koch link, which makes the hold easier to reach than on the real rig (a grip off the spots counts in the sim, not on the bench); a gripper counts only when both of its pads press its two FSR faces. Tool: the FSR log (about 0.2 N resolution after calibration).
- **1/3, caught:** at the cut, both grippers hold the arm (all four FSRs at least 1 N, from the log) and every joint is within 5.7 degrees (0.1 rad) of the working pose (encoder log); an arm driven to the cradle while powered and gripped afterwards does not count.
- **2/3, laid:** every joint within 10 degrees of the cradle pose (shoulder lift 0.45, elbow flex 0.8, wrist flex -0.35 rad, the others 0) at the last moment both grippers held it, and now the arm still (no joint moving, read from `Present_Position`) and resting on its foam. Tool: the encoder log (0.088 degrees per count).
- **1.0, let go:** then both grippers at least 5 cm from the arm for 2 s, the arm still laid. Tool: computed, not read off a picture: the YAM joint log through the YAM kinematics and the Koch encoder log through its kinematics give the gripper lattice and the arm surface in one frame, as `task.py` does (a minimum distance of 5 cm or more). Cross-check by video: two cameras at right angles (an overhead phone looking down gives the plan view, a side phone the height) with a 30 cm rule in view, so that no single view loses the depth axis.
- **Void (0), a drop:** after the cut, any point of the arm faster than 0.5 m/s, or a link striking the bench, pad or foam faster than 0.25 m/s. Read from the 20 Hz encoder log through the arm's kinematics (`arm_surface` in `task.py` takes the joint angles), checked against the 60 fps video with the rule as scale.

**(e) Forces, frictions and their bench checks.**

| Value | In the simulator | Accepted window | Bench check, and a unit outside the window |
|---|---|---|---|
| Unheld fall after the torque cut | first link on the foam 0.20 s after the cut (0.15 to 0.40 s across the friction range) | first contact with the foam 0.15 to 0.40 s after `disable_torque()` | Working pose, nobody touching the arm, call `disable_torque()`; time the cut (logged) to the first frame a link touches the foam in the 60 fps video (17 ms per frame); 3 tries. Slower than 0.40 s, or the arm stays up: a servo's gear train binds; find it with the friction check below and replace that servo. Faster than 0.15 s: a stripped gear; replace that servo. |
| Back-drive friction per joint, torque off | 0.03 N m per XL430-W250, 0.012 N m per XL330-M288 (estimates; dynamic 70 % of it) | 0.005 to 0.2 N m per XL430, 0.002 to 0.08 N m per XL330 | Torque off, the link horizontal and its weight taken off (the link held level by a hand under its centre); hook a 5 N pocket spring scale (0.05 N divisions, about US$8) 10 cm from the joint axis, square to the link, and read the pull at which the joint starts to turn: XL430 0.05 to 2 N, XL330 0.02 to 0.8 N. Outside: replace the servo. |
| Viscous damping | 0.02 N m s/rad (estimate) | 0.005 to 0.1 N m s/rad | Covered by the fall-time check above (both ends score 1.0 with a catch and 0 without one). |
| Rotor inertia seen at the joint | 0.005 kg m2 per XL430, 0.002 per XL330 (estimate: a small rotor's inertia times the gear ratio squared; ROBOTIS publishes the ratios 258.5:1 and 288.4:1 but not the rotor) | 0.002 to 0.02 per XL430, 0.0008 to 0.008 per XL330 (a rotor of 3e-8 to 3e-7 kg m2, the usual range for coreless and cored motors of this size, times the ratio squared) | Covered by the fall-time check: a unit whose unheld fall lands in 0.15 to 0.40 s is inside the window; outside it, replace the servo. |
| Hold force per finger | 1 N to count as holding (`F_HOLD`); the demo pinches at 20 to 28 N per finger, under the YAM gripper's about 40 N; the act itself needs about 5 N per finger (2.8 N weight over two grips at friction 0.3) | 1 N | Calibrate each FSR by setting 100 g and 500 g weights (0.98 N and 4.9 N) on it through a 10 mm rubber puck; the reading must be within 20 % of each. Outside: replace the FSR. |
| Contact friction, static / dynamic | 0.8 / 0.7 on the printed arm links, 1.0 / 0.9 on the foam pad and blocks (estimates; the foam and blocks are modelled rigid, 12 x 44 x 2 cm) | arm links 0.5 to 1.0, foam 0.7 to 1.2 | Tilt test: put a printed link on the foam and raise the foam until the link slides; the tangent of that angle is the static friction. A firm foam that does not compress under the 0.29 kg arm is the accepted pad; a soft foam is replaced. The gripper pinch (2 fingers x 40 N x 0.3 friction, 24 N) holds well above the arm's 2.8 N weight over this range. |
| Arm mass | 0.29 kg with its base, 0.22 kg moving (the URDF) | 0.25 to 0.33 kg with the base | Kitchen scale (1 g) on the arm off its clamp. |
| Powered hold before the cut | 10 N m/rad, 0.3 N m s/rad; torque limit 1.4 N m (XL430-W250 stall at 11.1 V), 0.52 N m (XL330-M288 stall at 5 V) | any stiff position hold | It only holds the start pose. |

**Whole-trial rules** (the drop void, and the hold at the cut) are recorded by the 20 Hz encoder log and the FSR log, both time-stamped by the same script that calls `disable_torque()`, and by the 60 fps video with the rule in view.

**Safety:** the arm weighs 0.29 kg and falls onto foam; keep hands clear of the YAMs while the policy runs.

## Scoring

See `scoring` in `task.json` and `Trial` in `task.py`. A gripper holds the arm when both its fingers press on the arm's links with at least 1 N each (PhysX contact reports between the YAM finger bodies and the Koch's links, the median of the last three steps); nearness alone never counts.

- **1/3, caught:** when the torque is cut, both grippers hold the arm and every joint is within 5.7 degrees (0.1 rad) of the working pose (an arm driven to the cradle while powered and gripped afterwards does not count).
- **2/3, laid:** also every joint within 10 degrees of the cradle pose at the last moment both grippers held it, and now, the arm still (no point faster than 1 cm/s) and resting on its foam. A joint that sags home under gravity after a hand opens does not count, and a hand that lets go before the arm is laid leaves 1/3.
- **1.0, let go:** then both whole grippers (a lattice 2 cm apart over the fingers, pads, palm, housing and 12 cm of the wrist) at least 5 cm from the arm's surface, the arm still laid, for 2 s.
- **A drop voids the run (0):** after the cut, any point of the arm faster than 0.5 m/s, or a link striking the bench, pad or foam faster than 0.25 m/s. Unheld, the arm passes 0.58 m/s within 0.1 s of the cut and peaks at 0.9 m/s (`none`); the demo's lowering peaks at 0.04 m/s.

The speeds are of the arm's surface points (from the Koch meshes) computed from the joint readings through the URDF's kinematics, so a few degrees of drift in a light wrist joint is not a fall, and a dropped arm is.

## Results

| Run | What it does | Score |
|---|---|---|
| `none` | The arms wait; at 30 s the torque is cut and the unheld arm's forearm falls (a link at 0.58 m/s, peak 0.9 m/s, on the foam 0.20 s after the cut) | 0.0 |
| `gate` | Both grippers press on the arm through the cut and open in place at once: the arm sags onto the open fingers (peak 0.28 m/s), never laid | 0.33 |
| `hover` (`full` with `CLOSE_L=0.08 CLOSE_R=0.09 K_AUTOCUT=300`) | The jaws close to just short of the arm, fingers on both sides of it but not pressing (0 N): never held, so the torque is cut at 15 s and the arm strikes the foam at 0.30 m/s | 0.0 |
| `onehand` | Both hands hold through the cut; the left lets go and the right lays the arm alone by its wrist | 0.33 |
| `wrong` | Laid with the wrist flex 30 degrees off its cradle angle; the hands never set it within 10 degrees | 0.33 |
| `push` (demo-only mode) | Both hands hold through the cut, then open; the right closes its jaws and pushes the arm down | 0.33 |
| `offpad` (demo-only mode) | The right hand closes on the forearm's middle instead of the wrist's grip spot: the pressing is read in the sim (any link counts), but the arm is not held at its working pose and a link strikes the foam at 0.33 m/s: the drop voids the run | 0.0 |
| `full` | Both hands carry the arm through the cut, lay it in the cradle pose, let go one at a time and return to their ready poses | 1.0 |
| `full` and `none` at both friction ends | See the torque-off table above | 1.0 and 0.0 |
| Dream-1 (the model's run, 1,800 steps) | The model never closed its grippers on the arm (no finger contact); at 30 s the torque is cut unheld and the forearm falls (a link at 0.58 m/s: the drop voids the run); one session, no expiry | 0.0 |
