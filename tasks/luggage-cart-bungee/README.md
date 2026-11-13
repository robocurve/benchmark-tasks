# Strap a carton onto a folding luggage cart with a bungee cord: hook each end round a frame tube, the second against the cord's pull

A folding steel luggage cart stands upright on the table facing the two arms, its nose plate flat and its handle
extended; a corrugated carton stands on the nose plate against the frame. A yellow bungee cord with two red
plastic-coated steel hooks lies across the table in front of the cart. The robot straps the carton to the cart: one
hook on each side tube of the frame, the cord stretched round the front of the carton at about half its height. Then
it lets go, and the scorer tilts the cart back on its wheels, rocks it from side to side, lifts it and tips it
forward, as a person checks a strapped load by the handle: the carton must stay on the nose plate.

What is not stated: a J-hook goes onto an upright tube from behind: it is reached past the tube, slid in behind it
and drawn forward into its bend. Once the first hook is on, the second has to be placed that way while the stretched
cord pulls it back with 8 to 12 N and the arm gives under the pull. Let go before the tube is inside its bend, it
flies back across the carton and pulls the first hook off with it. Each arm reaches only its own side tube, so both
arms are used, one per hook.

## Files

| File | What it is |
|---|---|
| `task.json` | The task as submitted (text fields). |
| `task.py` | The scene, the cord model, the scorer and its tip-back test (a yam_arena task file). |
| `partkit.py` | Helper: rigid parts made of several boxes and cylinders, a per-physics-step callback, gripper points. |
| `demo.py` | The scripted full-score demo and the cheap-strategy variants. |
| `LICENSE` | MIT. |

## Run a trial

In the Isaac Lab-Arena + YAM container, from this folder:

```
inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2000 --store-frames --log-dir logs \
  --instruction "$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')"
```

`--policy yam_scripted` plays the task file's empty `demo()`: both arms hold still (the do-nothing run). The episode
ends early when the scorer's tip-back test has passed (`success`).

## Scripted demo

```
python demo.py task.py demo_out            # records demo_out/demo.mp4 (left wrist, top, right wrist) and setup.png
DEMO_VARIANT=letgo_first python demo.py task.py out_letgo --no-record
```

It plans from the fixed start layout and looks only at the poses of the two hooks and the carton (what a camera
sees) and at the arms' own commanded and measured joint angles; its moves never use the cord's tension or any
scorer state (those are only printed in its log). It moves the hooks only with the grippers and never moves the cart. Each hook is held by its
crimp sleeve with the hand pitched 45 degrees back along the shank (straight down, the wrist camera hits the side
tube). Under the cord's pull the arm gives several centimetres; the demo makes up for it from the arm's own joint
readings (commanded minus measured angles, added back to the command) and then nudges the hook until the tube, at its
published position, sits between the hook's shank and tip, before it lets go.

## Scoring

Judged from the video and, at the end, a ruler, a phone level and a tape mark on the nose plate round the carton. A hook is home on a side tube when the tube passes through its bend (between the bend and the end of the tip) and the hook lies within 35 degrees of flat. The cord is round the carton when it bends round both of the carton's front vertical edges; it must cross each of them 6 to 18 cm above the nose plate. The carton stands on the nose plate when the middle of its bottom is on the plate, within 2 cm of it, and the carton leans less than 20 degrees from the frame's upright. 0.25: at least one hook home, the carton standing on the nose plate and the cord stretched round the carton's front (held or not). 0.5: both hooks home, one on each side tube, the cord round the front within that band, the carton on the plate, both grippers at least 3 cm from the hooks and the carton and 2 cm from the cord, and neither hook nor the carton moving more than 3 mm in 0.5 s. When all of this has held for 1 s, the test starts (in simulation the scorer moves the cart; by hand, take off the axle clamp and use the handle): tilt the cart back 45 degrees on its wheels, rock it 20 degrees to each side about the frame's long axis, stand it up, lift it 15 cm, tip it 30 degrees forward about the axle and back upright (20 s). 1.0: through the whole test the carton stays on the nose plate and moves less than 4 cm on it, both hooks are still home at the end, and the grippers stay as clear of the hooks, carton and cord as for 0.5; otherwise the trial stays at 0.5. Anything else scores 0.

In code: `evaluate()` and `_watch()` in `task.py` (the 0.25 and 0.5 conditions, the 1 s hold, the test and its verdict); `success()` is true only after the test passed, which ends the trial.

## Results (real runs of this folder's code, fixed start layout)

| Run (`DEMO_VARIANT`) | What it does | Score | Steps |
|---|---|---|---|
| Dream-1 (official CLI, `--policy dream`, the instruction above) | the grippers came near the hooks but no hook was set on a tube; both hooks lie on the table at the end | 0 | 2,000 |
| full demo | both hooks set round their tubes from behind, cord round the carton about 12.5 cm up; test passed (carton moved at most 2.2 cm) | **1.0** | 1,309 |
| `nothing` (and the official CLI with `--policy yam_scripted`) | arms stay still | 0 | 2,000 |
| `drop` | the right hook is let go while stretched, beside its tube but not round it | 0 (it flies back across the carton and pulls the left hook off) | 2,000 |
| `hold` | strapped correctly, but the left gripper keeps hold of its hook | 0.25 | 2,000 |
| `low` | both hooks aimed 5.5 cm above the table, near the bottom cross tube | 0 (the scripted moves failed near the wheels and the bottom cross tube, so neither hook went home; this is not a clean test of the 6 to 18 cm band) | 2,000 |
| `letgo_first` | the left hook is let go as soon as it is home, before the cord is stretched | 1.0 (the slack hook rests on the cross tube under it, so this is not a mistake on this cart) | 1,249 |
| `unstrapped` (measurement, not a strategy: the scorer's test is started by hand with no cord on) | the test alone | carton slides 22.4 cm, off the nose plate | 480 |
| strapped by placing the hooks directly (measurement) | the test alone | carton moves 2.5 cm, passes | 431 |

The last two rows are diagnostics that edit the scene (`seated_probe`, `unstrapped`); they show that the test separates
a strapped carton from a loose one.

## Runtime and seeds

One trial is at most 2000 control steps at 20 Hz. The start layout is fixed (`object_jitter=0`); there is no
randomness in the task. A demo run takes about 5 minutes on one NVIDIA L40S (simulator start-up included). The demo reaches full score in 1,309 steps; the run limit is 2,000.

## Fidelity

- The cart is rigid and is held still while the robot works: a person keeps a foot on the axle, and here a bracket
  over the axle does that job. For the test the scorer moves the cart (as a person tilting it by the handle would);
  the carton and hooks are free bodies throughout and move only by contact, gravity and the cord.
- The cord is a massless elastic line recomputed 240 times a second: from each hook's crimp sleeve, round the
  carton's front vertical edges whenever the straight line would pass through the carton (frictionless wrap), with its
  height changing evenly along its length; over the top if it would cross an edge above the carton's top. Its pull,
  112 N per metre beyond 50 cm, acts on both hooks along the cord and on the carton at every edge it bends round. It
  does not collide with the robot or the cart and is drawn as 24 yellow pieces that part as it stretches.
- 112 N/m beyond 50 cm is an estimate: an 8 mm, 24 in light-duty bungee is sold as needing 22 to 24 lbf (about
  100 N) to double its length; a 6 mm cord has about 56 % of that cross-section (56 N over 50 cm). Real shock cord is
  stiffer at small stretch than a linear spring. The pull follows the stretch with a 20 ms lag (the cord's mass and
  damping); each hook carries half the cord's 20 g.
- The test is done with the cart held up 15 cm at the end: in simulation the cart is moved by setting its pose, so
  friction from a tube moving down cannot carry the hooks down with it; setting the cart back down is therefore not
  part of the test.
- Corrugated board is rigid (no crushing under the cord); carton-on-steel friction 0.35 (estimate).

## Real-world build

- Cart: any folding steel luggage cart with two upright side tubes about 16 mm across and 34 cm apart, a nose plate
  about 38 x 26 cm and small wheels behind; kept from rolling or tipping while the robot works by a clamp over the
  axle (removed for the test).
- Carton: a single-wall shipping box 22 x 24 x 24 cm (depth x width x height) with about 1 kg of books inside (1.2 kg in all).
- Cord: a 24 in (61 cm) bungee cord, 6 mm, with plastic-coated steel J-hooks whose 20 mm opening takes the 16 mm tube.
- The test by hand: release the clamp; tilt the cart back by its handle to 45 degrees (phone level on the frame), rock
  it 20 degrees to each side and back, stand it up, lift it 15 cm, tip it 30 degrees forward and back upright; then
  look at the carton against a tape mark round it on the nose plate (moved less than 4 cm) and at both hooks.

## Third-party assets and licenses

The SDK is yam-isaac-lab-arena at revision 0059d02a1a91 (https://github.com/kairosiann/yam-isaac-lab-arena), the
program's official dependency, installed in the user's environment and loaded at run time. Nothing from it is
redistributed here. Licenses are given as each source states them.

| Asset | Used for | Source | License as stated by the source | Bundled |
|---|---|---|---|---|
| YAM arm and gripper (URDF, meshes, collision hulls, converted USD) | both arms | SDK `yam_arena/assets/yam/`, from i2rt (https://github.com/i2rt-robotics/i2rt, SOURCE_COMMIT 120c3c81400171174604e503943f8d1ebc891058) | MIT, Copyright (c) I2RT Robotics (SDK `LICENSE.i2rt`) | no |
| link2 / link3 meshes split into black and white parts | station arm colours | SDK `yam_arena/assets/yam/dream/`, from the dreamlabs 1.0.23 wheel | "MIT, see ../LICENSE.i2rt" (SDK `dream/SOURCE`) | no |
| Station floor and table | room | SDK `station.py` / `scene.py`: boxes with flat colours generated in code; no mesh or texture file | SDK README: "This repository has no license yet: most of it is ported from YAM-inspect-Isaac by Thomas Pierson, which has none." | no |
| Top-camera stand (foot, mast, arm, housing) | room | SDK `station.camera_stand`: boxes and capsules generated in code | as the row above | no |
| Dome light and lamp | lighting | SDK `scene.py`, Isaac Lab-Arena dome light and an Isaac Lab sphere light | as the row above | no |
| top_cam, left_cam, right_cam | policy RGB | SDK pinhole camera sensors at the calibrated poses | as the row above | no |
| Station fruit, bowl, plate (Arena object library) and water bottle | not used: this task's own objects replace them | - | - | no |
| Luggage cart, carton, bungee hooks and cord | task objects | this folder (`task.py`, `partkit.py`): boxes and cylinders generated in code | the authors' own geometry, MIT (this folder's `LICENSE`) | yes |

Isaac Lab-Arena (https://github.com/isaac-sim/IsaacLab-Arena, commit c7edf3a9d5d8) ships the Apache License 2.0.
Isaac Sim and Isaac Lab are runtime dependencies under their own licenses; none of them is redistributed here. No
third-party meshes or textures are used for the task objects. All task-specific code and geometry in this folder
(`task.py`, `partkit.py`, `demo.py`) is the authors' own work under the MIT License.
