# Carry a carton of cans whose untaped bottom flaps open unless it is held from below

A small shipping carton (6 x 6 x 6 inch regular slotted carton, single-wall corrugated board) stands on the table with
four full 330 ml soda cans in it. Its top flaps are folded out and down; its four bottom flaps are folded in but not
taped. A low white shelf stands to its right. The robot is told: "Put the carton of cans on the shelf, with all the
cans still in it."

The bottom holds only while the carton stands on something. Lifted by its walls or rim, the bottom flaps swing open and
the cans fall out, which ends the trial at 0. The robot has to keep the bottom shut from below for the whole move. One
way (the demo): tip the carton back on its far bottom edge, which keeps the flaps pressed shut against the table; slide
the closed fingers of the other gripper, held flat like a palm, under the raised near edge across the seam where the
two side flaps meet; carry the carton on that "palm" with the first hand steadying the rim; set it on the shelf with
its front over the board's edge so the palm can be pulled out; push it home.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: carton, cans and shelf, `success()` and `score()` |
| `hingekit.py` | Rigid parts built from boxes and cylinders with hinged links (the bottom flaps), for yam_arena task files |
| `demo.py` | The scripted full-score demo, and the cheap strategies (`DEMO_VARIANT`) |
| `LICENSE` | MIT License for everything in this folder |

`hingekit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task runs
with the standard `yam_arena` embodiment and `task_score` scorer. Every task shape is a primitive defined in
`task.py`; the robot and the station come from the SDK (see "Third-party assets and licenses").

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the carton of cans on the shelf, with all the cans still in it." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/open-carton-dream
```

`task_score` prints the trial's score: 1.0 on success (the trial ends there), otherwise `score()` at the end of the
episode. One trial is at most 1,200 control steps (1 minute of simulated time at 20 Hz); it takes about 4 minutes
of wall-clock time on one RTX A6000. The layout is fixed (`object_jitter=0`), so the seed does not change the scene.

## Scripted demo

```bash
python demo.py task.py demo_out                      # full score: demo_*_cam.mp4, setup.png, result.json
DEMO_VARIANT=rims python demo.py task.py out_rims    # a cheap strategy (see below)
python demo.py task.py --dry                         # offline: IK error of every planned pose (no simulator)
```

The demo plays open loop from the fixed published layout (the constants in `task.py`). At run time it reads only what
a policy could get: its own 14 joint positions, and three looks at the carton's pose (a large box in plain view of
the cameras): after tipping it, after setting it on the board and before pushing it home. It never reads the flap
angles, contact forces or any other internal state. The joint positions are used for one thing: the arms' joint
controllers sag under the 1.5 kg carton, so the demo adds the measured joint error back into its commands while an arm
carries load (an integral term), as a real controller would.

Result (the run in the demo video): **score 1.0, success at step 797** of 1,200.

## Cheap strategies

Measured with `DEMO_VARIANT=<name> python demo.py task.py out`:

| Strategy | What happens | Score |
|---|---|---|
| `nothing` | the arms stay in the ready pose | 0.0 |
| `rims` (the naive attempt) | pinch the left and right wall rims, lift, carry to the shelf: within 2.5 s of the lift all four bottom flaps hang open (90 to 113 degrees) and all four cans fall through | 0.0 |
| `pinch_only` | tip the carton as in the demo, then carry it by the front rim alone: the carton swings in the pinch, its side flaps open about 47 degrees, it slips out and lands on its back | 0.0 |
| `rims_low` | pinch the wall rims, lift 4 cm and hold: the side flaps swing open about 33 degrees and rest on the table, which holds the cans up | 0.0 |
| `rims_high` | pinch the wall rims, lift 9 cm and hold: the flaps open and the cans drop out | 0.0 |
| `push` | push the carton along the table toward the shelf with a closed gripper: it slides about 3 cm on the table before the arm runs out of reach; a carton slid along the table cannot get onto a board 12 cm above it | 0.0 |
| `side` | tip the carton right over onto its back wall with the front-rim pinch, then try to carry it that way: the cans stay inside, but the carton is never lifted (the arm cannot hold the pinch that far out) | 0.0 |
| `tip_only` | tip it and slide the palm under, then stop | 0.0 |
| full demo | tip, palm under, carry, set down, palm out, push home | 1.0 |

Through the official CLI (`inspect-robots run --embodiment yam_arena ... --scorer task_score --max-steps 1200`), a
policy that holds both arms in the ready pose for all 1,200 steps scores `task_score: 0`.

## How the hidden property can be observed

The bottom is not visible while the carton stands on the table, and nothing in the instruction says it is untaped.
Two cheap probes show it, both as visible displacements:

- Tip the carton back about its far edge (as in the demo, about 19 degrees): the underside of the near edge comes into
  view of the top and wrist cameras, and the seam where the flaps meet has no tape across it.
- Lift it by its rims (the `rims` strategy): as soon as the carton leaves the table its two side flaps swing down,
  then the end flaps, and the cans sink through the opening; within 2.5 s all four flaps hang open 90 to 113 degrees
  and the cans are on the table.

## Scoring

Measured with a ruler, a protractor and the video.

- 0 if any can leaves the carton at any moment (its centre outside the carton's inner walls, more than 1 cm below its bottom or more
  than 2 cm above the rim), even if it is put back.
- 1.0 if the carton rests on the shelf board (all four bottom corners within 12 mm of the board's top, its centre over
  the board), no part of its bottom more than 2 cm beyond any edge of the board, upright within 10 degrees, both
  grippers let go (every finger pad at least 3 cm from the carton) and the carton moving less than 2 mm, held for 1 s.
- 0.5 if it rests on the board within that 2 cm overhang limit with all cans inside, but the 1.0 conditions are not all
  met (still held, tilted or moving).
- 0.25 if at some moment the carton, with all cans inside, had all four bottom corners at least 8 cm above the table,
  but it is not on the board within the limit. 8 cm is more than a bottom flap's length (7.2 cm): lifted less, by its
  rims, the carton's side flaps swing open about 33 degrees and rest on the table, which then holds the cans up, so
  such a hover earns nothing.
- Otherwise 0.

In simulation the release check reads the finger pads' positions in an `on_step` hook, because `score()` does not see
the robot; if those positions cannot be read the carton counts as still held, so no credit is given for an unverified
release.

## Real sizes and masses

- Carton: 6 x 6 x 6 inch (15.2 cm) regular slotted carton, single-wall corrugated board (about 4 mm), 130 g empty; a
  stock shipping-box size.
- Cans: standard 330 ml aluminium cans, 6.6 cm x 11.5 cm, about 350 g full; four of them, 1.4 kg.
- Total lifted: 1.53 kg, shared between the two arms.
- Shelf: 12 cm high, a low step shelf; 25 x 30 cm board.

## Third-party assets and licenses

The SDK is yam-isaac-lab-arena at 0059d02a1a91bc6834c1d0846022891df0c78730, the program's official dependency,
installed by the user's environment. Nothing from it is redistributed here. Licenses are given as each source states
them.

| Asset | Used for | Source | License as stated by the source | Bundled |
|---|---|---|---|---|
| YAM arm and gripper (URDF, meshes, collision hulls, converted USD) | both arms | SDK `yam_arena/assets/yam/`, from i2rt (SOURCE_COMMIT 120c3c81400171174604e503943f8d1ebc891058) | MIT, Copyright (c) I2RT Robotics (SDK `LICENSE.i2rt`) | no |
| link2/link3 meshes split into black and white parts | station arm colours | SDK `yam_arena/assets/yam/dream/`, from the dreamlabs 1.0.23 wheel | "MIT, see ../LICENSE.i2rt" (SDK `dream/SOURCE`) | no |
| Station floor and table | room | SDK `station.py`/`scene.py`: boxes with flat colours generated in code; no mesh or texture file loaded | SDK README: "This repository has no license yet" (Placeholder) | no |
| Top-camera stand (foot, mast, arm, housing) | room | SDK `station.camera_stand`: boxes generated in code | as the row above | no |
| Sky dome light and lamp | lighting | SDK `scene.py`, using Isaac Lab-Arena's DomeLight and an Isaac Lab sphere light | as the row above | no |
| top_cam, left_cam, right_cam | policy RGB | SDK camera sensors at the calibrated poses; no camera mesh | as the row above | no |
| Station poses, sizes and colours | numbers in the SDK code | SDK README: "Dream Labs' calibrated YAM station, from Dream's MuJoCo replica (dreamlabs 1.0.23, `yam_mujoco`) and the calibration it follows (yamlab at `ec0455d`)" | license not stated by the SDK | no |
| Station fruit, bowl, plate (Arena object library) and water bottle | not used: a task file's `OBJECTS` replace them | - | - | no |
| Carton, bottom flaps, cans, shelf | task objects | this folder (`task.py`, `hingekit.py`): boxes and cylinders defined in code | authors' own, MIT License (`LICENSE`) | yes |

Isaac Lab-Arena release/0.3.1 (c7edf3a9d5d8276565208f72422f8f8a0806d6cf) ships the Apache License 2.0 in its
LICENSE.md. Isaac Sim and Isaac Lab are runtime dependencies under their own licenses. None of them is redistributed
here. All task-specific code and geometry in this folder is the authors' own, under the MIT License in `LICENSE`.
