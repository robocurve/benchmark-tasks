# Fit an SO-101 arm's shoulder bracket over both horns of its fixed servo and line up its screw holes

Task for the bimanual YAM station in Isaac Lab-Arena (`yam_arena`, PhysX). Results (scripted demo, Dream-1 and
doing nothing through the official `inspect-robots run` path on the evaluation machine; negatives through
`isaac/demo.py`) are in `task.json` "scores".

## The task
The SO-101 is the open-source robot arm (TheRobotStudio / Hugging Face LeRobot) built in large numbers to collect
robot-learning data. Every joint is assembled the same way: a 3D-printed link with two arms (cheeks) straddles the
STS3215 servo, one cheek on each of the servo's two horns, then 4 M3 screws go in per side. This task is the first
joint (official assembly text, Joint 1: "Attach the shoulder part. Tighten the shoulder part with 4 M3x6mm screws on
top and 4 M3x6mm screws on the bottom"), from the point where the servo is already seated and screwed in the base and
both horns are on, up to the point where the screws go in. Driving the screws is the next, separate step and is not
part of this task. The servo is fixed, so the printed part moves onto it: unlike seating a servo in a housing, the
bracket must straddle both horn faces at once, with one of them out of sight, and then be turned to register its
holes.

Start (same every trial): the partly built arm (printed base + base motor holder + STS3215 servo, both horns fitted)
is held to the table by one C-clamp (about 5 cm jaw) on the base's 18 mm thick flange at its back edge, the side away
from the robot, clear of the bracket's path (in the sim the base is fixed and the clamp is not modelled); the printed
shoulder bracket stands on its lower arm on the table to the robot's right; the
top horn's hole pattern is 25 deg off the bracket's square-on position.

The robot must hold the bracket so that both cheeks stay level with the two horn faces (36.4 mm apart, line-to-line;
the lower cheek and lower horn are out of sight under the servo, 2 mm above the base floor), slide it on along the
one path its slots allow (the top horn's M3 screw head runs in the top cheek's 5.8 mm slot, the idler boss in the
bottom one) until the top horn meets the cheek's lip, with the bracket turned (either before it goes on, or on the
horns once seated) so that its four top holes are over the top horn's four holes closely enough for M3 screws to start
(each within 0.15 mm), then let go. The scripted reference reads the horn's hole angle, slides the bracket on at that
angle, turns it to the horn's holes from the measured poses, presses the top horn into the cheek's lip (the lip arc is
centred on the hole pattern, so this centres the holes), takes the push off and opens the hand at once. It then reads
the hole offset from the poses and grips and seats the bracket again if letting go moved it past 0.12 mm.

## Scoring (final state; `SCORE_OVER = "end"`)
| Level | Condition (code: `src/scoring.py`) | Person check |
|---|---|---|
| 0.25 | both cheeks straddle both horns: screw head at least 3 mm inside the top slot, within 1 mm of the slot line, bracket height within 1 mm, tilt <= 3 deg | side photo; screw head seen in the slot |
| 0.5 | home: horn axis within 0.5 mm of the cheeks' hole-pattern axis, midway between the cheeks (top horn on the lip), height within 0.5 mm, tilt <= 2 deg | horn rim on the lip, centre screw centred in the cheek's middle hole |
| 0.75 | home and registered: `hole_offset_mm` <= 0.15: for each of the four top holes, the distance between the hole's bore axis and the axis of the horn hole under it, taken at both ends of the 3.5 mm bore (the cheek's inner face and 3.5 mm above it) and the larger of the two kept, using the full bracket pose and the horn's actual angle; the largest of the four holes, for the best of the four 90-deg matchings (`HOLE_OFFSET_M`, `BORE_LEN_M`) | the check screw (an M3x6 from the kit, thread 2.90 mm across), put in each top hole in turn, starts in the horn's thread with fingers only and turns two full turns, the bracket not touched or held; taken out before the next hole (see below) |
| 1.0 | registered, every finger pad tip >= 20 mm from the bracket, bracket still (< 2 mm/s, < 2 deg/s) for 1 s | video frames |
| 0 | otherwise (elsewhere, jammed on a rim, one horn only, upside down, knocked over) | |
The episode ends as soon as 1.0 is reached. The bottom horn's holes are not scored: that horn turns freely on its
boss and is turned to its holes with the screw tip when the bottom screws go in (as in the real build).

Where the 0.15 mm comes from. Official CAD, measured on the bracket mesh: each top hole is a 3.2 mm round bore
(largest inscribed circle; the hole is a teardrop for printing, its point 2.3 mm from the centre) 3.5 mm long from the
cheek's inner face, under a counterbore for the screw head (CAD: 5.15 mm across on two holes, 5.35 mm on the other
two); the four holes sit on a 9.9 mm square about the cheeks' hole-pattern axis, which is also the centre of the lip
arc and the slots and lies 0.20 mm from the MJCF joint anchor (`HOLE_AXIS_B_M`); the top horn's four M3 threaded holes
sit on the same 9.9 mm square about the servo axis.
The real-trial check uses one fixed check screw: an M3x6 screw from the SO-101 kit whose thread measures 2.90 mm
across (2.88-2.92 mm with calipers; pick one from the kit) and whose head enters the counterbores. In the 3.2 mm bore
that screw can sit at most (3.2 - 2.90) / 2 = 0.15 mm off the bore's centre, and it starts only if it can lie on the
horn hole's thread axis along the whole bore. So the check passes only while each top hole's bore axis is within
0.15 mm of its horn hole's axis at both ends of the bore; that is the scored limit (`HOLE_OFFSET_M`, taken at the
cheek's inner face and 3.5 mm above it). 0.15 mm is this screw's limit; other screws differ: ISO 965 (6g) allows an
M3 thread of 2.874-2.980 mm across, so other kit screws have limits of 0.11-0.16 mm, which is why the check screw is
fixed. The setup's drill-bit test keeps each printed bore at 3.2 mm or more. In a real trial a starting screw can
shift the free bracket within its play (it is free in its slots and on the horns), so the person check can pass with
the holes more than 0.15 mm off; the simulator scores the geometry only, so it is the stricter of the two.

## Run and score a trial
Host: Isaac Sim 6.1 + Isaac Lab-Arena + yam_arena + Inspect Robots in a container with one GPU.
`isaac/run_frozen.sh <run> "<cmd>"` copies `isaac/ src/ assets/ task.json` into `$SO101_OUT/<run>/source`
(read-only, per-file and combined SHA-256) and runs `<cmd>` there; env vars: `ISAAC_LAUNCHER` (container launcher),
`GPU_SLOT_WRAPPER` (optional GPU slot wrapper), `ARENA_VENV_BIN` (Arena venv `bin/` inside the container), `SO101_OUT`.
```sh
I="$(python3 -c 'import json;print(json.load(open("task.json"))["instruction"])')"
# official Inspect Robots path (default guardrails, task_score), scripted reference policy registered by official_run.py
bash isaac/run_frozen.sh <run> "python official_run.py --stage demo -- run --embodiment yam_arena \
  --policy so101_joint_scripted -E task=task.py -E object_jitter=0 -E device=cpu -E control_hz=20 -E cam_width=320 \
  -E cam_height=176 --instruction \"$I\" --scorer task_score --max-steps 2400 --store-frames --no-prompt \
  --log-dir <out>/<run>"
# any model policy: same arguments with --policy <name>; --policy yam_scripted holds READY (do-nothing baseline)
# scripted demo directly (14 joint/gripper commands only): result.json, trace.jsonl, demo_<camera>.mp4, setup.png
bash isaac/run_frozen.sh <run> "python demo.py task.py <out>/<run> --stage demo --max-steps 2400"
# negative / partial cases (one per --stage): neg-partial | neg-unregistered | neg-offset | neg-held | neg-jammed |
# neg-beside | idle; python negcheck.py <out.json> <run dir>... checks each against its expected level
# several trials in one process with yam_arena's in-process reset (writes reset_check.json):
bash isaac/run_frozen.sh <run> "python demo.py task.py <out>/<run> --stage neg-partial,neg-unregistered,neg-held"
```
Development-only environment switches (unset in every demo, official and negative run): `SO101_HORN_JOINTS=articulation` (the rejected
horn version, for probe_horn / probe_mouth comparisons), `SO101_CONTACT_OFFSET`, `SO101_HULL_MIN_THICKNESS`,
`SO101_PHYSICS_DT`, `SO101_HORN_DAMPING`, `SO101_DEBUG_DROP` / `SO101_DROP_PIECES` (leave colliders out),
`SO101_CONTACT_REPORT`, `SO101_DEBUG_NOVIS`.
Engine-independent checks: `cd src && python check_scoring.py` (synthetic scorer checks, numpy only).
Geometry asset: `src/make_assets.py` rebuilds `assets/so101_joint1.npz` from the official CAD (see Assets).

## Files
| File | Role |
|---|---|
| `isaac/task.py` | yam_arena task file (SCENE, OBJECTS, score/success/distance) |
| `isaac/jointkit.py` | bench spawn (static clamped base, fixed top horn, bottom horn on a revolute joint, free bracket) + scorer hook |
| `isaac/demo.py` | scripted demo and negative variants (robot commands only; reads poses) |
| `isaac/official_run.py` | official `inspect-robots run` path with the scripted policy registered |
| `isaac/run_frozen.sh` | evidence runner on a read-only source snapshot |
| `isaac/probe.py` | bench probe (development) |
| `isaac/probe_horn.py` | horn friction-drive check of the earlier jointed horns: torque steps, holds / turns / stops (development; the top horn is now fixed) |
| `isaac/probe_fit.py` | fit check: bracket released at set slide positions, where it comes to rest (no arm) |
| `isaac/probe_mouth.py` | free-bracket push check: constant force along the slots, how far it gets, its speed |
| `isaac/negcheck.py` | stimulus + expected-level check of finished demo / negative runs (JSON only) |
| `src/layout.py`, `src/scoring.py`, `src/check_scoring.py` | layout, scorer, scorer checks |
| `src/make_assets.py` | official CAD -> visual meshes, static collider, convex slab pieces of the bracket |
| `assets/so101_joint1.npz`, `assets/LICENSE-SO101-Apache-2.0.txt` | derived geometry + its licence |

## Fidelity (full statement)
`task.json` "fidelity" is a short version of this statement.

Isaac Sim 6.1 / Isaac Lab-Arena yam_arena (PhysX on the CPU, rendering on the GPU), physics 960 Hz, control 20 Hz, YAM arms from their URDF with implicit joint drives at their real effort limits and 50 N fingers. Geometry is the official SO-101 CAD (TheRobotStudio STL files, Apache-2.0): visual meshes decimated; the base, base motor holder and servo case are one static triangle-mesh collider (fixed in the sim; clamped to the table in a real trial, clamp not modelled); the bracket's collider is its official mesh cut into 14 horizontal slabs at its feature levels (cheeks, slots, lip, web, upright), each cross-section split into convex prisms, so the slots (5.8 mm top, 6.2 mm bottom), the 1 mm lip and the cheek ends are the CAD's. The bracket's screw holes (including their edge chamfers, which break into the lower slot's walls by about 0.1 mm near the cheek face and would otherwise give the collider sharp-edged notches) and the motor-2 pocket are filled in the collider. Horns: 20.0 mm discs on the servo axis. The top horn is fixed in the sim at its start angle (a kinematic body): the unpowered 1:345 gearbox resists turning with about 0.6 N m (estimate; measure it with a spring scale on the horn), while the bracket can load the horn only through friction at the lip and faces (radius about 10 mm; even a 10 N push at friction 0.35 gives about 0.035 N m, over ten times less), so a real top horn does not turn during the task; a jointed horn (revolute joint with a torque-capped zero-velocity drive) was tried and turned by tens of degrees under solver contact noise, an engine artifact. The bottom horn is on a revolute joint with a stiff zero-velocity drive capped at its friction torque (PhysX's articulation-joint friction was tried and rejected: in this engine build its contacts with the bracket chattered and locked the bracket in its slots). The top horn carries its M3 screw head (5.0 mm; modelled 1.3 mm proud instead of 1.5 so that, with the face clearance below taken up by the bracket's weight, it keeps 0.2 mm under the roof of the 1.5 mm groove it runs in); the bottom horn turns freely (0.002 N m); the servo case's 6.0 mm idler boss under it (static) is modelled 5.7 mm across (0.25 mm a side in its 6.2 mm slot) and 0.2 mm longer than in the CAD, so that with the face clearance below (the bracket hangs 0.2 mm lower on the top horn than in the CAD) it still engages the bottom cheek's slot walls over the CAD's 0.6 mm. Fit: the CAD is line-to-line at the horn faces (cheek inner faces 36.4 mm apart = across the horns) with 0.11 mm at the lip (lip arc radius 10.11 mm about the hole-pattern axis on a 10.00 mm horn). PhysX needs a finite gap and the YAM wrist is soft, so the sim has 0.2 mm per side at the horn faces (a tilt of more than about 1 degree still pinches both horns) and 0.1 mm radial at the lip (the lip's inner face is a 64-gon of radius 10.10 mm about the hole-pattern axis; the lip slab is one chunky convex sector per 64-gon edge, and its two end cells are taken as their convex hulls, which adds 0.09 mm2 at the lip's two end corners, none of it nearer the horn, because a 0.1 mm wide sliver piece left at the lip's end was cooked by PhysX about 0.5 mm proud of its authored face and pushed a seated bracket 0.44 mm sideways); PhysX contact offset 0.1 mm per shape, below those slot clearances (with 0.15 mm or more the engine kept contacts on both walls of a slot at once and the bracket locked in its slots; with 0.1 mm the pins can be pressed up to about 0.2 mm into their slot walls under load, solver compliance); real printed parts come out line-to-line to a light press and the PLA cheeks flex slightly, which a real arm feels as a light push (hence the setup's fit test: 5 N or less to slide home, else a new print). Collider artifact: one thin convex piece of the bottom cheek's inner-face slab (a needle-shaped triangle 8.7 mm long and 0.9 mm wide) stands about 0.4 mm proud of its authored face once PhysX has cooked it, so near home it presses on the bottom horn's lower face and pinches the seated bracket between the two horns with about 25 N (no-arm probe, bracket placed at home: contact 0.3-0.46 mm above the authored face, 23-28 N). Its effect on the task: the last millimetre of the slide and turning the seated bracket are stiffer in the sim than the real part's 5 N fit, so the sim is harder here, not easier; the reference pushes through, and a seated, let-go bracket stays where it is. Each horn's outer rim has a 0.4 mm chamfer (estimate of the rounded rim of the kit horns), the only lead-in; the printed edges are sharp in the collider. Masses: bracket 35 g (estimate, weigh the print), horns 3 g; friction PLA on PLA/aluminium 0.35. No welds, snaps, proximity rules or teleports: the bracket goes on only by contact with the horns, the screw head and the boss in its slots. The scorer reads poses only (bracket, top horn, finger pad tips). The servo's cable and the controller board are not modelled (cable tucked in the base). A free bracket can turn slightly as it settles: placed at or near home with no arm touching it, it turned at most 0.45 deg as it settled on the fixed top horn, and pushed on its web with 1-3 N it turned up to 2.1 deg (no-arm probe). Registration is scored against the top horn's actual angle (constant, since the horn is fixed).

Checks and notes: `isaac/probe_horn.py` checked the friction drive of the earlier jointed horns (holds, turns, stops;
with the top horn now fixed it applies only to the bottom horn); with the horns as articulation links instead, their contacts with the bracket chattered and
locked it in its slots (`isaac/probe_mouth.py`). The scripted reference grips the bracket with about 13 N per finger and
pushes with a few newtons.

## Measurements to take before submission
These values are estimates from the CAD and data sheets. Measure each on one real print and one STS3215 with simple
tools, then put the number in the named fields. If a code constant changes, the evidence runs must be redone (they are
tied to the source SHA-256).
| Value (now) | How to measure | Feeds |
|---|---|---|
| Top-horn backdrive torque (0.6 N m, estimate) | servo unpowered; tie a thread to a screw in an outer horn hole (7.0 mm radius) or to a 10 cm lever screwed to the horn; pull tangentially with a spring scale and read the force at which the horn starts to turn; torque = force x radius; mean of 3 pulls | the fixed top horn in the sim (task.json "fidelity", README Fidelity): it holds while the torque stays over ten times the 0.035 N m that bracket friction can apply |
| Bracket mass (35 g, estimate) | kitchen scale (0.1 g) on the printed bracket | task.json "setup" and "fidelity", README Fidelity; `BRACKET_MASS` in `src/layout.py` |
| Slide-on push force (sim: 0.2 mm per side at the horn faces, 0.1 mm at the lip) | push-pull spring scale on the web, horn on its marker line, bracket square; force to slide from the slot mouths to home; mean of 3 | the 5 N limit of the fit test in task.json "setup"; sim clearances `HORN_TOP_Z`, `HORN_BOTTOM_Z` in `src/layout.py` |
| Bracket top-hole bore (3.2 mm, CAD) and the check screw's thread diameter (2.90 mm, 2.88-2.92 mm accepted; kit screws span 2.874-2.980 mm, ISO 6g) | drill bits or pin gauges in 0.05 mm steps: largest that passes each top hole; calipers across the thread of kit screws until one reads 2.88-2.92 mm | the 0.15 mm of the 0.75 check in task.json "scoring" and README Scoring = (bore - thread) / 2; `HOLE_OFFSET_M` in `src/scoring.py` |
| Horn mass (3 g) and rim chamfer (0.4 mm), PLA friction (0.35) | kitchen scale; calipers or a photo with a ruler; tilt-plane test (angle at which the bracket slides on a horn: friction = tan(angle)) | README Fidelity; `HORN_MASS`, `HORN_CHAMFER`, `BRACKET_FRICTION` in `src/layout.py` |
| Clamp position | clamp a real print as in the setup; check the bracket slides home and both arms reach it without touching the clamp | task.json "setup", README Start |

## Third-party assets and licenses
| Item | Source | License |
|---|---|---|
| SO-101 CAD: STL meshes (`base_so101_v2`, `base_motor_holder_so101_v1`, `sts3215_03a_v1`, `rotation_pitch_so101_v1`) and the MJCF transforms | TheRobotStudio SO-ARM100 / SO-101, as packaged in the Hugging Face repo `lerobot/so101-tabletop-mujoco` (`robotstudio_so101/`) | Apache-2.0 (`assets/LICENSE-SO101-Apache-2.0.txt`) |
| `assets/so101_joint1.npz` (decimated visual meshes, static collider, convex slab pieces) | derived from the SO-101 CAD by `src/make_assets.py` (SHA-256 of the sources in its `meta`) | Apache-2.0 (derived work) |
| Task code (`isaac/`, `src/`) | written for this task | MIT (`LICENSE`) |
| Task geometry generated in code (horn discs, top-horn screw head, idler boss) | written for this task (`isaac/jointkit.py`) | MIT (`LICENSE`) |
| YAM robot model (URDF, meshes) | i2rt, vendored by yam-isaac-lab-arena | MIT (i2rt) |
| Station scene (`yam_arena` station, IK, cameras, embodiment) | yam-isaac-lab-arena benchmark SDK, installed separately | see upstream (no licence file in the repository; used, not redistributed) |
| Isaac Lab-Arena, Isaac Lab, Inspect Robots | upstream, installed separately | see upstream |
| Isaac Sim 6.1 | NVIDIA, installed separately | see upstream |

## Runtime
Scripted demo: 701 control steps (35 s simulated, run limit 2400), about 2.4 min wall time through the official CLI
on the evaluation machine (one GPU, PhysX on the CPU, three 320x176 cameras) and 7-9 min on a slower development
machine. The built-in `yam_scripted` do-nothing policy stops itself after 61 steps (0.5 min). A model trial that runs
to the 2400-step limit takes about 8-9 min (Dream-1 on the evaluation machine). Seeds: none (layout fixed,
`object_jitter=0`); with PhysX on the CPU repeated runs of the same code in a fresh process are identical: every demo
run of this version ended after the same 701 steps in the same final pose (hole offset 0.087 mm after letting go,
inside the reference's own 0.12 mm check). In-process reset (several trials in one process, the multi-trial command
above): the bodies come back within 0.001 mm of the first trial's start, but the trials after a reset are not
bit-identical to a fresh process. The documented run `--stage neg-partial,neg-unregistered,neg-held` scored 0.25,
0.5 and 0.75 as expected; the scripted demo run again after an in-process reset (`--stage demo,demo`) scored 1.0, but
not on its first seat: its first let-go left the holes 0.165 mm off (0.5 at that moment), it gripped and seated the
bracket again (0.121 mm, over its own 0.12 mm check) and a third time (0.101 mm), and reached 1.0 after 1027 steps
instead of 701.

## Evidence media
Start state at 1280x704 (same frozen source, idle stage, 3 steps): `setup_top_cam_1280x704.png`,
`setup_left_cam_1280x704.png`, `setup_right_cam_1280x704.png` and the tiled `tiled_setup_hires.png` (top | left |
right).
Each demo / official run writes one H.264 video per camera (`demo_top_cam.mp4`, `demo_left_cam.mp4`,
`demo_right_cam.mp4`; the right arm's wrist camera is `right_cam`), `setup.png` and `final_<camera>.png`. Tiled views
(top | left | right, same timing): `tiled/tiled_top_left_right.mp4`, `tiled/tiled_setup.png`, `tiled/tiled_final.png`.
The final wrist-camera frame shows the bracket's top arm and its hole pattern from above at an angle: the trial ends
1 s after the hand lets go, while the hand is still swinging to its look-down pose.
