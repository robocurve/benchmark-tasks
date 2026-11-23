# Drill a shallow hole in a marked wood block

The right arm holds a cordless drill while the left arm operates its attached
spring-return trigger. The robot feeds the rotating 6 mm bit into the marked
wood, makes a 10–13 mm deep hole, withdraws the bit, and releases the trigger.
The task is in **Societal relatability**: drilling a marked hole for a shelf,
bracket or other home project is a familiar operation with a visible result.

`task.py` specifies setup and scoring. `cutting.py` is the complete reduced
material model; `runtime.py` applies its wrenches to the physical drill at each
240 Hz substep. `geometry.py` renders the persistent bore from that same state.
`demo.py` controls only the ordinary joint targets. Use `run.py` to load this
extension; no installed adapter files or other tasks are changed.

## Runtime and installation

Use Linux with an NVIDIA RTX GPU. If the YAM environment is already installed,
activate it and run this folder's `run.py`.

For a fresh installation, start in the benchmark repository root, using new
checkout directories:

```bash
TASK_DIR="$(pwd)/tasks/drill-wood-trigger"
ADAPTER_DIR="$HOME/yam-arena-wood-trigger"
ARENA_DIR="$HOME/IsaacLab-Arena-wood-trigger"
git clone https://github.com/kairosiann/yam-isaac-lab-arena.git "$ADAPTER_DIR"
git -C "$ADAPTER_DIR" checkout 0059d02a1a91bc6834c1d0846022891df0c78730
git clone --branch release/0.3.1 https://github.com/isaac-sim/IsaacLab-Arena.git "$ARENA_DIR"
git -C "$ARENA_DIR" checkout -B release/0.3.1 c7edf3a9d5d8276565208f72422f8f8a0806d6cf
git -C "$ARENA_DIR" apply --check "$TASK_DIR/patches/arena-production-assets.patch"
git -C "$ARENA_DIR" apply "$TASK_DIR/patches/arena-production-assets.patch"
ARENA_DIR="$ARENA_DIR" \
  DREAM_SPEC="git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339" \
  "$ADAPTER_DIR/setup.sh"
source "$ARENA_DIR/yam.env"
cd "$TASK_DIR"
```

The reference environment uses Python 3.12.16, Isaac Sim 6.1.0.0, the pinned
Arena release/0.3.1 source, `yam-arena` 0.1.0, Inspect Robots 1.0.0,
`inspect-robots-yam` 0.38.0 and `dreamlabs-client` 1.1.3. The pickup works with
the original upstream waypoint module. The included one-line Arena asset-root
patch selects the production asset mirror used by the recordings; it changes no
scene geometry, scoring or control logic.
Installation and the first asset load need network access. Simulation,
rendering and scripted control run locally. Dream inference uses the Dream Labs
API; configure its credentials outside the repository as in the adapter docs.

## Run and record

```bash
RUN_ROOT="$HOME/yam-eval/drill-wood-trigger"
python run.py demo --log-dir "$RUN_ROOT/demo"
python run.py dream --log-dir "$RUN_ROOT/dream"
```

Use fresh log directories. Add `--headless false` for the local Isaac Sim window.
Simulation and rendering run on the local GPU; Dream-1 inference uses the Dream
Labs API. Both trials use the same instruction and setup, seed 0, zero jitter,
20 Hz control, 240 Hz physics and 320 x 176 policy camera frames. The limit is
2,400 actions / 120 simulated seconds. The demo requests a stop at 110 s if needed.
The default output is `$YAM_LOG_ROOT/drill-wood-trigger/<mode>-<timestamp>` or
`~/yam-eval/drill-wood-trigger/<mode>-<timestamp>`.

```bash
inspect-robots video /path/to/adhoc_LOG_ID.json
```

The evaluation JSON stores the scorer result. `measurements.jsonl` records the
actual tip trajectory, contacts, spindle speed, material force/torque, permanent
hole geometry and qualifying depth. `source_manifest.json` records input hashes.
A reset starts with intact wood and an unpressed trigger; no demo stage, policy
identity or target-coordinate test enables material removal. The scripted demo
may read measured poses, contacts and depth for control, as permitted for demos.

## Shared station fixtures

The task adds only the wood, red marking, drill/probe and attached trigger listed
in `task.json`. There are no fruit, loose plates or other station props. The
wood's securing fixture is approximated by its kinematic body and is not rendered.
The initial layout is the uploaded setup image.

The floor is a fixed 200 x 200 x 0.02 m box with top Z=-0.0125 m. The overhead
camera is at (-0.166495, -0.009, 1.691821), with OpenGL XYZW quaternion
(0.183011, -0.183011, -0.683013, 0.683013). Its stand has a 110 x 110 x 16 mm
foot at (-0.236495, -0.009, 0.7555), a 12 mm radius mast with 928.321 mm spine,
a 9 mm radius support with 53.852 mm spine, and a 50 x 42 x 22 mm housing.
The calibrated wrist cameras are fixed to the grippers. Exact stand rotations,
lighting, camera intrinsics, robot meshes and wrist mounts come from
`station.py`, `scene.py` and the bundled robot assets at the pinned adapter commit.

## Physical task and scoring

The block and drill dimensions, materials, poses, target, ready joints and shared
station are specified in `task.json`. A fresh dry clear-pine coupon is
180 x 140 x 30 mm. Secure it on the bench and mark a 20 mm red disk. The simulation
approximates the workholding clamps by a fixed block; it does not render a clamp.
The steel bit is approximated by a 35 mm long, 6 mm diameter cylinder with a
rotating helical surface stripe. The drill has a spring-return trigger proxy
at its trigger location, 14 x 20 x 40 mm with 5 mm travel; that enlarged proxy
and its 150 N/m spring are approximations, not a claim about a measured switch.

The hole entry is the bit's actual first cutting position. It must be within
3 mm of the target centre, with its axis within 12 degrees of vertically down.
The bore depth must be between 10 and 13 mm. At least 10 mm of material advance
must happen while the right arm holds the handle and the left fingertips press
the attached trigger at least 3 mm. A hole made with a trigger propped against
another object cannot meet that two-arm cutting-history condition.

Finish holding the drill, bit at least 15 mm above the wood top, trigger released
(at most 1 mm stroke), for 0.5 seconds. Withdrawal happens before trigger release.
Primary physical scoring uses a ruler for hole-centre error, a marked straight
probe/depth gauge for hole depth, a protractor for direction, and trial video
for grasp, trigger operation, withdrawal and the final hold. The result can be
inspected after the drill is withdrawn; simulation-only force readings are not
required from the physical evaluator.

Scores cover every trial:

- 0: no grasp/no actions, or a bore at least 2 mm deep outside the location/angle
  limits, or any bore deeper than 13 mm.
- 0.25: hold the drill without enough qualifying cutting progress.
- 0.50: at least 2 mm of qualifying cutting at the correct position and angle.
- 0.75: complete the qualifying hole, without the final withdrawal/release hold.
- 1.0: complete the qualifying hole and the uninterrupted final 0.5 s hold.

The simulation's contact detectors require both right pads against the drill
above 0.05 N each, TCP within 55 mm of the handle, gap below 75 mm, and left
finger-to-trigger contact above 0.05 N. The attached trigger must have 3–7 mm
stroke, including solver allowance, and under 3 mm transverse error. These are
contact/attachment detectors, separate from the much larger drilling load.

## Reduced cutting model and fidelity

The normal rigid bit/wood pair is replaced by the material law in `cutting.py`.
The housing, chuck, fingers, trigger, table and wood retain ordinary PhysX
collisions. Each physics substep reads the real drill pose and attached trigger,
then applies reaction force at the bit tip plus reaction torque to the drill's
public wrench buffer. No policy writes a drill pose, trigger state or hole depth.

Trigger stroke drives a 0–900 rpm spindle with 0.12 s response time. With a
spinning bit (at least 300 rpm), at least 8 N axial material reaction, and an
actual bit advance into the current bore floor, material can be removed. Depth
advance is bounded by measured penetration and 3 mm/s; its force-dependent rate
is `0.0004 * (force_N - 8)` m/s. A stationary surface touch cannot accumulate a
deep hole. The hole stays after withdrawal and cannot grow with the motor off.

The material uses 16,000 N/m axial stiffness and 35 N s/m damping. Once a bore
exists, its walls apply 500 N/m lateral resistance with 0.5 N s/m damping beyond 0.7 mm clearance;
there is no guide before cutting. The bore starts at the actual entry pose,
including off-target entries, rather than being placed at the red target. It
models one top-entry bore per coupon, not arbitrary milling or multiple holes.
The bore floor and walls are rendered from the permanent material state; no
hole animation is triggered by an arm waypoint or a task success flag.

Cutting torque opposes the bit axis, `0.035 * (feed_force/12)` N m, capped at
0.12 N m. Axial and lateral force components are each capped at 55 N for numerical stability. Typical operating
loads must be reported from the run diagnostics. This softwood force law is an
engineering approximation **not calibrated against a physical drilling test**.
No claim of validated cutting-force transfer to hardware is made. There is no
fracture, chip transport, heat, motor stall model or detailed flute cutting.
Lateral reaming, bit bending and changes to bore diameter are omitted. The
rendered 6 mm bore therefore does not establish physical hole quality under
side loading. In the uploaded demo the largest transverse tip displacement
while at least 1 mm below the top is 4.61 mm during withdrawal; this is a known
limitation of the compliant single-bore model. The 1.0 result is the simulated
scorer result, and physical withdrawal/hole quality remain unvalidated.

For a physical trial, select the bit/drill/coupon combination and check the feed
and grip loads are within the rig's capacity before evaluations; preserve those
materials and settings across trials. Use a secured workpiece and backing board,
keep grippers clear of the rotating chuck/bit, and protect the shared rig from
chips. Each physical trial consumes a fresh matching coupon; simulator reset
restores the unbored material and clears cutting history and spindle state.

## Results

All three trials use the same six Python source files, instruction, seed 0 and fixed
layout, with a 2,400-step / 120-second limit.

| Policy | Score | Actions | Simulated time | Wall time |
| --- | --- | --- | --- | --- |
| Scripted demo with inspection (`wood_drill_demo`) | 1.0 | 1,562 | 78.10 s | 218.02 s |
| Inspect Robots reference (`wood_drill_demo`) | 1.0 | 1,579 | 78.95 s | 233.22 s |
| Dream-1 beta (`dreamlabs/dream-1-beta`) | 0.25 | 2,400 | 120.00 s | 270.21 s |

The submitted demo made an 11.57 mm deep hole, with entry error 1.34 mm and bore tilt
0.80 degrees. Cutting samples show 8.02–11.47 N axial reaction, mean 10.17 N.
It kept the right grasp, withdrew the bit, released the trigger, and met all
finish conditions for ten consecutive control steps. Dream-1 picked up and
held the drill but made no hole. Neither official trial had an execution error.
These are individual fixed-layout trials, not success-rate estimates.

`demo_result.json`, `reference_demo_result.json` and `dream_result.json` contain
the outcomes and source hashes. The uploaded demo is the inspection recording.
The withdrawn contact-only version's scores and media do not apply.

To make a full demonstration video with a four-second post-trial inspection:

```bash
python tools/inspection_demo.py --log-dir "$HOME/yam-eval/drill-wood-trigger/inspection"
```

This runs the same scripted control and scorer, then changes only the rendering
camera after successful termination. No additional physics steps or robot/material
state edits occur during the inspection. The resulting `demo-with-inspection.mp4`
shows the complete trial followed by a close view of the bore. Its score and
control limit apply to the trial before that camera-only inspection.

## CPU checks

```bash
uv pip install --python "$(command -v python)" "pytest==9.1.1"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests
```

The checks cover trigger-only and light-touch shortcuts, motor-off penetration,
stationary-bit depth, load/reaction and persistent depth, actual off-target entry,
cutting without the required arm contacts, reset state, visible bore geometry,
both gripping pads, bore-floor continuity during sidewall contact, re-entry into
intact wood, depth/angle/position limits, withdrawal, trigger release,
and the continuous final hold.

## Assets and licenses

- Cordless-drill mesh: NVIDIA Isaac Lab-Arena `power_drill`,
  `Arena/assets/object_library/power_drill_physics/power_drill_physics.usd`;
  [NVIDIA Isaac Sim Additional Software and Materials License](https://www.nvidia.com/en-us/agreements/enterprise-software/isaac-sim-additional-software-and-materials-license/).
  The asset is loaded from the official server rather than redistributed here.
- YAM robot meshes and wrist mounts: I2RT geometry bundled by the pinned
  YAM adapter, MIT License, with notice in `yam_arena/assets/yam/LICENSE.i2rt`.
- Station dimensions, primitive fixtures, lights and cameras: pinned YAM adapter
  and Isaac Lab-Arena sources; retain their repository license notices. Arena
  source is Apache-2.0.
- Wood, target, bit, helical stripe, bore and trigger: procedural geometry in
  this folder, with no additional third-party texture or mesh downloads.
