# Deflate a retained catheter balloon before withdrawing the catheter

A balloon retains the shaft inside a clear bench chamber. One YAM gripper supports the free valve while the other seats an empty needle-free receiver. The receiver plunger returns under pressure and reduces the balloon envelope. Only then can the robot disconnect the receiver, withdraw the shaft through the neck, park both devices and release them. The trial uses an inert, needle-free fixture.

## Why this is a nursing task

Registered nurses carry out catheter removal during routine device care. Leeds Community Healthcare describes the nurse using a syringe to deflate the retention balloon before removing the device. This task isolates the contact, alignment and clearance change in that operation. The U.S. Bureau of Labor Statistics lists 3,465,400 registered nursing jobs in 2025. [Leeds Community Healthcare NHS Trust](https://leedscommunityhealthcare.nhs.uk/our-services-a-z/continence-urology-and-colorectal-cucs/trial-without-catheter-twoc/) · [BLS occupational profile](https://www.bls.gov/ooh/healthcare/registered-nurses.htm) · [O*NET registered nurses](https://www.onetonline.org/link/summary/29-1141.00)

## Lay out the station

Coordinates are metres in the calibrated bimanual YAM station. X points forward, Y left and Z up. The table is at z = 0.7475 and the draped support at z = 0.7615. Seed 0 starts both stock parallel grippers open in READY. The setup image is taken after 30 settling controls.

Chamber and retained shaft. A clear polymer cutaway chamber, with radii 75 × 65 × 45 mm, sits at (0.375, 0.125, 0.851) on a 183 × 185 × 9 mm base. Its 46 mm silicone neck has a 12 mm bore along Y. Its centre is (0.375, 0.039, 0.851), with inner and outer planes at y = 0.062 and 0.016. The 40 g rigid shaft has a silicone appearance and is 244 mm long and 6 mm in diameter. Its initial root is (0.375, −0.045, 0.851). Relative to this root, the balloon centre is at y = 132 mm and the distal tip at y = 168 mm. The inflated envelope has a 15 mm radius; the collapsed envelope has a 4 mm radius. The 30 mm long, 18 mm diameter polymer hub carries a 28 × 27 × 12 mm blue tab.

Valve and receiver. The 25 g valve begins at (0.339038, −0.115365, 0.851), yawed −35°. It has a 30 × 24 × 20 mm grip, a 16 mm deep socket and a freely rotating branch to the hub. The 100 g polymer receiver begins at (0.300, 0.300, 0.820) in two open polymer cradles on a 160 × 125 × 10 mm steel pad. Its barrel is 90 × 24 mm, its nozzle 16 × 8 mm, and its flange 43 × 18 mm. The 15 g plunger has a 32 × 25 mm thumb pad and 55 mm available travel. It starts fully forward.

Supports and collection. A polypropylene ledge under the valve tab and two narrow ledges under the hub hold the device at reset. All three are open on top, so the arms can lift the device. A stainless 180 × 380 mm receiving tray sits at (0.240, −0.341), with its floor at z = 0.829 and rim at z = 0.847. Two HDPE V rests inside the tray sit at x = 0.250 and y = −0.220 and −0.370. They support the shaft across the tray while the free valve settles. The storage datum is 30 mm forward of the geometric tray centre. The bundled layout files give each support's dimensions, orientation and mounting position.

For building the small staging supports, the valve top is 12 × 20 × 6 mm and shares the valve's −35° yaw. The two rear hub ledges are 2 × 6 × 8 mm. Their support tops are at z = 0.8535 m. The valve socket and shaft extraction corridor remain accessible. The tray has a 176 × 376 mm interior, a 1.5 mm floor and 2 mm walls. Each 20 × 28 × 62.3 mm receiving rest has a 90° V groove. The 35 mm pedestal gives the valve branch room to hang as the shaft settles. Use the bundled `layout.py`, `station-layout.usda`, `rest-layout.usda` and asset dimension files to reproduce the mount positions.

## Reset between trials

Start a fresh scene and scorer. Put the rigid shaft at (0.375, −0.045, 0.851) and the valve at (0.339038, −0.115365, 0.851), yaw −35°. Keep the hub and valve tabs on the open staging ledges. Put the receiver and plunger at (0.300, 0.300, 0.820) with identity rotation and stroke within 5 mm of zero. The recovery tray is empty. Restore the inflated collision envelope, clear the receiver connection and all score history, and leave both arms open in READY. Settle for 30 controls at 20 Hz before taking the setup photo or starting the score. The layout uses a new, independent initialization between trials.

## How the device behaves

PhysX reads the poses of the catheter, valve, receiver barrel and plunger. Sustained alignment at the specified insertion enables a prismatic connector at their measured relative pose. The detent has 80 N/m stiffness and a 1 N axial force limit. The virtual fluid path opens at 1 mm insertion and can close before full detachment. Reinserting the nozzle before separation reopens it.

A finite 0.35 N native piston drive approximates pressure. Measured plunger travel determines the drained fraction. At 99.5% return, both the visual and collision envelopes switch to the collapsed size. Reverse piston motion with the path open can re-expand the balloon. The neck's faceted free radius spans 6.000–6.029 mm. The valve socket spans 4.750–4.791 mm. The nozzle itself is represented visually, with coupling determined by pose and the axial connector.

The scripted controller observes device poses and sends joint and gripper targets. Native contacts lift and carry the devices. In the last placement, the shaft passes through three successive targets at x = 0.250 m, y = −0.330 m, and z = 0.930, 0.910 and 0.8875 m. The controller pauses for position and orientation tracking before release. The scorer evaluates the resulting motion independently of the controller phases.

## Score a trial

Grade the last valid 20 Hz observation. Read device pose and plunger travel relative to the valve, fingertip contacts against each device, neck crossings, and the final positions and speeds. At reset or if no milestone qualifies, score 0. Either hand may support the valve.

0.10 — Connect the empty receiver. Nozzle insertion must be 1–7 mm, lateral error at most 2 mm and axis error at most 10°. Alignment must persist for at least 0.15 s with both bodies below 0.10 m/s. The connection starts within 5 mm of zero plunger stroke. At connection and until first full deflation, both pads of one gripper must each register at least 0.05 N against the valve and both opposite pads at least 0.05 N against the barrel. Rest contact does not count.

0.40 — With the valve connected, measured receiver travel reaches at least 49.75 mm from its connection stroke, drained fraction at least 99.5% and balloon collision radius at most 5 mm. Reverse travel while the fluid path remains open recomputes the drained fraction. The 0.40 tier is lost if the balloon reinflates.

0.70 — After full deflation, the receiver pulls axially at least 12 mm from its connected position and the detent releases. Disconnection must be observed on an earlier control than the balloon's first entry into the neck at y = 0.062. Keep the balloon collapsed, pass through the neck within 3 mm of its centreline at both planes, and move the distal tip past y = 0.006.

1.00 — Maintain release for 20 consecutive controls (one second). The disconnected shaft midpoint lies within 25 mm in X and 35 mm in Y of (0.240, −0.311), at z = 0.868–0.888, within 10° of horizontal. The barrel lies within 25 mm horizontally of (0.300, 0.300), at z = 0.805–0.835. Both TCPs stay at least 60 mm from the sampled device points. Every tracked body moves at most 15 mm/s and 0.2 rad/s. Video also shows the fingers clear of the devices.

A missing opposing grasp during connection or first deflation, early disconnection, attached withdrawal, an inflated balloon entering the throat, a neck crossing outside the corridor, reinflation after extraction, or a body root below z = 0.710 permanently zeros the trial. Invalid start or missing/nonfinite measurements cannot establish success.

### Physical judging on an inert bench rig

For hardware scoring on the inert phantom, use four independently tared normal-force pads (one on each fingertip), with sensing faces flush to the normal jaw surfaces, a 0.01 N or finer readout, and synchronized 20 Hz force and video. Before each trial check each channel with a calibrated gauge at 0.05 N and 0.20 N and re-tare it unloaded. At the first aligned connection and each control through the first full deflation, use the side and top views to identify each finger's device. Both pads on one hand must read at least 0.05 N against the valve and both on the other hand at least 0.05 N against the barrel; table or rest contact does not qualify. Use the same sampled comparisons as the simulation. This is a specified hardware judging procedure, not a reported hardware measurement.

Use millimetre scales in the two camera views, index marks on the plunger and nozzle, and witness marks at the neck planes and parking regions. Compare consecutive calibrated frames to assess translation, rotation, gripper clearance and dwell. On the assembled bench, verify the two envelope diameters and neck bore with calipers. The measured normal forces in the submitted simulation establish hand contact with the devices. They do not establish clinically appropriate forces.

## Reproduce the run

The submitted runs use `yam-inspect-isaac:6.1.0`, seed 0 and one episode. The allowance is 3,600 14-dimensional controls at 20 Hz, or 180 seconds of simulated time. Output files for an episode include `result.json`, `trajectory.jsonl`, the setup image and camera MP4s. Dream-1 run: 0 after 129 steps (6.45 simulated seconds); termination: DreamSessionExpired:expired. Scripted/teleoperated demo: 1 after 3091 steps.

From this task folder inside the configured Isaac Lab/Arena image, run the supplied scripted controller. Give the image one GPU and use a writable output location outside the task folder.

```sh
python run.py --policy scripted --episodes 1 --seed 0 --output-dir /outputs --viz none
```

For an independent Dream-1 trial in the same image, supply the authorized Dream credentials and persistent budget ledger, then run the same command with `--policy dream`. Dream receives the task instruction, calibrated camera observations and robot state. Re-score either recording by substituting its actual episode directory below. The replay reads recorded measurements and checks the expected score.

```sh
episode='/outputs/balloon-catheter-removal/REPLACE_WITH_RUN_ID/episode-01'
python runtime/rescore.py --task task.py "$episode/trajectory.jsonl" --expect "$episode/result.json"
```

The package also includes the recorded actions and scorer inputs from **both submitted trials**. To audit the published outcomes without launching a new simulation, run the following in the pinned Python environment with NumPy and YAM Arena available.

```sh
for trial in demo dream; do
    gzip -dc "runtime/recorded-${trial}.jsonl.gz" > "/tmp/catheter-${trial}.jsonl"
    python runtime/rescore.py --task task.py "/tmp/catheter-${trial}.jsonl" --expect "runtime/recorded-${trial}-score.json"
done
```

The Python protocol tests exercise scoring and connector behaviour. From the task directory run `python -m unittest discover -s . -p 'test_*.py' -v` with NumPy installed. The physical demonstration and Dream result are reported separately in `task.json`.

## Fidelity, assets and licenses

The chamber has a cutaway view and a full hollow collision wall. The rigid shaft, spherical valve-branch joint and two balloon envelopes model the clearance change. The pressure drive approximates plunger return. The task does not simulate fluid dynamics, tissue contact, flexible catheter deformation, sterility or patient responses. Centre-to-sample distances act as a proxy for final hand clearance. Inspect both hands in the recorded video when judging release.

All USD references are relative to this task. The surgical station and supports include procedural construction and provenance. The YAM geometry carries its i2rt MIT notice in `assets/surgical-drape/LICENSE.i2rt`. The simulator image preserves source commit `120c3c81400171174604e503943f8d1ebc891058`. Isaac Lab/Arena has its own Apache-2.0 license and the Isaac Sim image is separately licensed.
