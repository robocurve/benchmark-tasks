# Spike a saline bag without touching the sterile spike or port, then prime the drip chamber

A task file for the `yam_arena` embodiment, using Isaac Lab-Arena 0.3.1 (`task.py`).

A sealed 500 mL saline bag hangs from an IV support, ports down. A sterile giving set stands upright in a presentation
cradle on a nonsterile steel tray, spike up, capped. The robot pulls the blue cap off the bag's spike port and the navy cap
off the spike, takes the set out of the cradle's clip, brings it under the bag, seats the spike in the port up to its collar,
squeezes the drip chamber until the saline stands 15 to 27 mm high, lets go and returns both arms to their starts. From the
moment either cap is off until the spike is locked in the port, that exposed sterile end must touch nothing nonsterile: no
gripper surface, the tray, the cradle, the IV support or the bag's outside. Any such touch scores 0 for good, as a nurse
would discard the set or bag.

| File | What |
|---|---|
| `task.py` | The task file: objects, the cap seals, the rack clip, the spike joint, the contamination checks, `score()`, `success()`, `demo()` |
| `asepsis.py` | The contamination latch: exposed port and spike against gripper surfaces, the tray, and every nonsterile obstacle (boxes and cylinders generated from the bodies' collision shapes) |
| `gripper_geometry.py` | Convex hulls of the YAM palm and finger collision meshes, read from the installed URDF |
| `priming_meter.py` | The drip-chamber liquid height from the measured compression of its squeezable wall |
| `bodykit.py` | Bodies, hinges (with break forces), sliders (the chamber wall's springs), the spike's sliding joint |
| `test_*.py` | CPU checks of the scorer, the contamination geometry and the meter |

## Run a trial

From this folder, in a shell with the `yam_arena` environment active.

```bash
INSTRUCTION="$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')"
```

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  -P rows_per_request=6 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir runs/dream
```

Scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir runs/demo
```

The run prints `task_score` at the end: 1.0 once `success()` holds (full score with bag and set at rest), otherwise
`score()` at the end of the run. The scripted demo scores 1.0 in 1,392 steps (liquid at 18.3 mm after three squeezes). `IVBAG_DEBUG=1` prints the
contamination state, its cause, the spike depth and the liquid height every half second.

## Real-world setup and reset

### Parts

Positions are in metres in the station frame: x forward and y to the robot's left from the midpoint between the arm bases, z up, with the table top at z = 0.7475 m.

- **Tray:** a flat stainless instrument tray, 34 x 54 cm, 4 mm thick, nonsterile. It covers x 0.16 to 0.50 m and y -0.30 to 0.24 m.
- **Saline bag (1 per trial):** a sealed 500 mL 0.9 % sodium chloride bag in a PVC container with a separate spike port
  and medication port at one end, about 13 x 20 x 2.6 cm and 530 g filled. The spike port is a 10 mm tube, 30 mm long,
  closed by a blue pull-off cap. Training bags of the same size behave the same.
- **Giving set (1 per trial):** a standard gravity IV administration set with a clear, squeezable PVC drip chamber about
  20 mm across and 45 mm long, a spike with a collar, a closed roller clamp, and its spike cap. Pressing the chamber's
  middle in by 4 mm a side takes a light fingertip squeeze (the simulation uses about 4 N a side, an estimate); on
  release it springs back and draws saline in.
- **Printed handling aids (reused):** three parts in clear or matching PLA, which move from one set and bag to the next:
  a pull strip, 12 x 12 x 110 mm, clipped to the bag-port cap and standing out square from the bag's face; a tab,
  12 x 12 x 40 mm, beside the spike cap; and two collars clipped round the drip chamber, each a 50 x 4 x 35 mm plate,
  their middles 18 mm above and below the chamber's middle. The collars leave the chamber's wall bare for 32 mm between
  them, where the fingers take and squeeze the chamber itself. The strip and tab let parallel-jaw fingers take the caps
  without reaching the sterile ends, as a nurse's fingertips do.
- **IV support:** a 60 cm steel rod, 18 mm across, standing on the table 7.5 cm beyond the bag's centre (away from the
  robot), carrying a 16 x 1.4 x 3 cm steel backplate 33 cm above the table. A binder clip on the backplate holds the bag by
  its top seal, so it hangs vertically and still, its centre at (0.39, -0.122, 0.99) m, 24 cm above the table; the spike port's mouth is 11 cm above the table.
- **Presentation cradle:** a printed or molded white stand, nonsterile: an 80 x 80 x 8 mm base and a 25 x 12 x 56 mm post
  25 mm to the robot's left of the set's axis. A small spring C-clip on the post
  holds the chamber's lower collar: the set stays put against its own weight and the spike cap's pull, and comes out
  when pulled with more than 8 N (check once with a luggage scale). The base's centre is at (0.36, -0.30) m; the chamber's centre stands 8.4 cm above the table.

### Reset after each trial

1. Remove the used bag and set. Move the printed strip, tab and collars onto a fresh bag and set.
2. Hang the bag in the binder clip, ports down. Close the roller clamp. Push the set into the cradle's clip, spike up and capped.
3. Put the arms at the ready pose.

Discard a bag or a set whose exposed end has touched anything, and follow the products' instructions and local aseptic policy.

### Scoring a real trial

- **Contamination:** from the video, any touch of the open port mouth or the bare spike on a gripper, the tray, the cradle,
  the IV support or the bag's outside, after its cap is off and before the spike is seated. Then the trial scores 0.
- **Caps off:** the port cap 5 mm or more, and the spike cap 6 mm or more, from where each sat.
- **Seated:** the spike's collar against the port mouth (23 to 25 mm in).
- **Liquid height:** read on the chamber with a ruler, vertically, from the outlet-end graduation (15 to 27 mm), with the
  chamber within 12 degrees of vertical. Above 34 mm is an overfill, scored 0.
- **Let go and home:** from the video, both grippers 4 cm or more from chamber and port, both arms back at their starts
  within 5 cm, and the bag and set still for 1 s.

## How the simulation judges each rule

- **Caps:** each is a body on a hinge that tears for good above its pull (the port cap at 3.5 N, the spike cap at 0.08 N).
  A cap counts as off once it is 6 mm (spike cap) or 5 mm (port cap) from where it sat.
- **Rack clip:** a joint between the cradle and the set that breaks for good above 8 N. It holds the set against its
  weight (0.2 N), the spike cap's pull and light bumps; the robot pulls the set out of it. Nothing releases it except that force.
- **Spike in the port:** once both caps are off, a spike tip within 7 mm of the port's axis and 12 degrees of its line
  engages a sliding joint along the port; the port grips it with about 1.5 N, and at 23 mm in it locks for good.
- **Contamination:** checked every control step while a sterile end is exposed and the spike is not yet locked. The port
  mouth (a 5 mm-radius disc) and the spike (a 3.2 mm-radius segment from its collar to its tip) are tested against the convex
  hulls of every collision mesh of both palms and all four fingers, against the tray's top (0.5 mm), and against every
  collidable shape of the cradle, the IV support and the bag (0.5 mm), except the spike port's own tube. The first touch
  latches the score at 0 and records its cause.
- **The squeezable chamber:** between the collars, the chamber's wall is two clear panels, one each side across the grip,
  each on a sliding joint with a spring (1 N/mm) that holds it flush with the chamber's surface; a squeeze pushes both in,
  up to a 4 mm stop each (about 4 N a side). Inside them is only a 3.5 mm rigid core, so a finger meeting the chamber there
  meets a panel. The panels' positions are read from the simulation every step.
- **Priming:** with the spike locked, the chamber within 12 degrees of vertical and its outlet end 10 mm or more below the
  port, each completed squeeze and release raises the liquid by up to 11 mm, in proportion to how far the panels were
  pushed in: their mean travel, counted only once *each* panel is in at least 1 mm, a full 4 mm giving 11 mm. A squeeze
  ends only when both panels are back within 0.5 mm of rest. Nothing moves the panels but contact: a gripper opening and
  closing near the chamber, or pressing one side only, adds nothing, and neither does a squeeze held through the spike's
  locking, one that is never released, or easing one side off while the other stays pressed.
- **Home and rest:** both TCPs within 5 cm of their ready-pose positions, 4 cm or more from chamber and port, and the bag
  and set at rest.

## Runtime and seeds

The run limit is 2,400 steps (2 minutes at 20 Hz); physics runs at 240 Hz. The fixed layout uses `object_jitter=0`; there
is no randomness to seed. The scripted demo's 1,392 steps took about 4.5 minutes (267 s) of wall time on one L40S GPU,
including start-up. The Dream-1 run of the full 2,400 steps took about 9 minutes (537 s), model responses included.

The scripted demo uses only joint-position commands at 20 Hz; it never moves an object or changes a joint, force or
material. The right arm pulls the port cap down by its strip, rises and crosses high over the cradle, pulls the spike cap up by its tab, takes the chamber between
its collars and pulls the set out of the clip, carries it upright under the bag, seats the spike, lets go, centres the
open fingers on the chamber's wall, squeezes and releases it three times with the same grasp, and goes home. The
left arm stays clear.

## Scoring checks

`python -m pytest -q` checks, without the simulator (the template's `yam_arena` package must be importable): the
contamination latch for each sterile end against grippers, tray, cradle, IV support and bag; that the spike port's own
tube is exempt over the whole insertion; box and cylinder distances; the breakable rack clip; the priming meter (no
liquid without compression of both panels, nothing from nearby gripper motion, from a one-sided push, or from rocking a
held squeeze); the chamber's geometry; and the gripper hulls.

## Assets and licences

| Item | Source | Licence |
|---|---|---|
| All shapes in the scene (tray, IV support, bag, caps, giving set, cradle) | Generated by `task.py` from plain boxes and cylinders; no third-party meshes, textures or images | This task's licence (below) |
| Scene layout, scripted demo, gripper-hull contamination check (`task.py`, `gripper_geometry.py`) | Adapted from pull request #2031 (see *Origin and credit*) | No licence stated in that pull request; reused with credit under the program's terms that submitted tasks are released open source |
| `bodykit.py` | The multi-shape body helper published in this repository with many accepted tasks (for example `tasks/car-battery-clamp/`) | No separate licence stated where it is published; reused unchanged |
| YAM arm and gripper URDF and meshes, read at run time from the installed `yam_arena` package (`yam_arena/assets/yam/`) | i2rt (`i2rt-robotics/i2rt`) | MIT (`LICENSE.i2rt` in that package); not copied here |
| The reused `station` scene: floor, lights, table, camera rig and calibration (`SCENE = "station"`), built from boxes at run time by `yam_arena` (`scene.py`, `station.py`) | `yam-isaac-lab-arena`, the program's launchable, ported from `YAM-inspect-Isaac`; its station numbers come from Dream Labs' MuJoCo station replica and the `ARISE-Initiative/yamlab` calibration | None stated: both repositories' READMEs say they have no licence yet. Run-time dependency; nothing copied here |
| YAM embodiment, scripted-demo helpers (`yam_arena.demo`, `yam_arena.ik`) | `yam-isaac-lab-arena` (as above) | None stated (as above); run-time dependency |
| Isaac Lab-Arena (0.3) | NVIDIA, `isaac-sim/IsaacLab-Arena` | Apache-2.0 |
| Isaac Lab | `isaac-sim/IsaacLab` | BSD-3-Clause (its `isaaclab_mimic` extension: Apache-2.0) |
| Isaac Sim (6.1) | NVIDIA | NVIDIA Omniverse License Agreement (proprietary); run-time dependency, not redistributed |
| Inspect Robots (`inspect-robots`) | The program's evaluation harness | MIT |
| Occupation codes in `task.json` (29-1141.00, 29-2061.00) | O*NET OnLine, U.S. Department of Labor, Employment and Training Administration | CC BY 4.0 |

This task's own code and text: MIT.

## Origin and credit

This task builds on two earlier tasks in this repository: **"Add spike-an-iv-bag-and-prime-its-drip-chamber task"**
([pull request #1096](https://github.com/robocurve/benchmark-tasks/pull/1096)) and **"Add IV bag spiking and drip-chamber
priming task"** ([pull request #2031](https://github.com/robocurve/benchmark-tasks/pull/2031)). Their reviews accepted the
work as routine for registered and licensed practical nurses; both were left unrevised past their 48-hour windows (last
reviews 12 and 16 November), so the idea is open. The scene, the gripper-hull contamination check, the priming meter and the
scripted demo are #2031's, reused with thanks under the repository's open-source terms. This version adds what #2031's last
review asked for and goes further:

- contamination now covers every reachable nonsterile surface, generated from the scene's own collision shapes: the
  cradle, the IV support and the bag's outside as well as the grippers and the tray, with the cause recorded;
- the cradle's latch is a physical spring clip that breaks above 8 N, instead of a scripted release when the fingers
  closed on the chamber;
- the drip chamber is squeezable: its wall is two sprung panels between the collars, and the liquid comes only from
  measured compression of both of them, squeezed and released (a gripper opening and closing nearby no longer counts);
- the cradle, IV support, bag clamp and every printed handling aid are specified with dimensions and materials;
- new tests, a rewritten setup, scoring and fidelity note, and new Dream-1 and scripted runs.
