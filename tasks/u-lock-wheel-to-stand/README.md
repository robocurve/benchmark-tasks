# U-lock a bike wheel to a stand

A real child's bike (12-inch wheels) is parked beside a full-size Sheffield cycle stand, one leg of the stand just in front of its
back tyre. A full-size U-lock lies on the table in two parts: the steel U (shackle) and its crossbar. The robot is
told only: "Lock the bike's back wheel to the bike stand with the U-lock."

To lock the wheel, the shackle has to go round the rim itself, with one leg between two spokes and the other outside
the tyre, and round the stand's leg; then the crossbar has to be pushed onto both leg ends at once until it catches,
on the far side of the wheel. A lock round the stand alone, or only through the spokes, leaves the wheel free; a
crossbar not pushed home does not lock. The stand's leg is 8 mm from the tyre and the gap between the two spokes is
25 mm, so the rigid 1.3 kg lock has to be carried level and threaded accurately while the other arm brings the
crossbar round the wheel and seats it on both legs.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: bike, stand and lock, the crossbar's catch, `success()` and `score()` |
| `lockkit.py` | Rigid parts built from boxes, cylinders and capsules, hinged links (the wheel) and switchable welds (the catch) |
| `demo.py` | The scripted full-score demo and the negative variants |
| `task.json` | The task card |

`lockkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task runs
with the standard `yam_arena` embodiment and the `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment with the YAM SDK installed, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lock the bike's back wheel to the bike stand with the U-lock." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir logs/u-lock-dream
```

`task_score` gives 1.0 on success, otherwise `score()` at the end of the episode. The same command with
`--policy yam_scripted -P hold_s=60` holds both arms still for the whole episode (the task's `demo()` is an empty plan):
the do-nothing check.

## Scripted demo

```bash
python demo.py task.py demo_out                         # full score; writes demo_*_cam.mp4, demo.mp4, setup.png, result.json
DEMO_VARIANT=no_crossbar python demo.py task.py out     # see the table below for the variants
```

1. The left arm picks the shackle up by its inner leg (on the sleeve, level with its centre of mass); the right arm
   picks the crossbar up round its middle.
2. The left arm carries the shackle level at hub height to just in front of the stand's leg, legs pointing at the
   wheel, and pushes it home: the inner leg between two spokes, the outer leg past the tyre, the stand's leg between
   them. Meanwhile the right arm carries the crossbar up past the stand's leg to behind the wheel.
3. The right arm lowers the crossbar in line with the leg ends, 3 cm beyond them, and pushes it onto both at once,
   6 mm past where it is fully home (the arm gives a little).
4. Both arms let go and back away; the lock settles hanging on the wheel and the stand.

The demo plans from the fixed published layout (the constants in `task.py`, run with `object_jitter=0`), from looks at
the poses of the two lock parts (objects in plain view of the cameras: it corrects the arm for how each part sits in the
hand and for the arm sagging under the load) and from its own arm commands. It never reads the catch state, contact
forces or any other internal state.

The demo reaches 1.0 in 800 steps (40 s); `run_limit` is 1,200 steps.

### Other strategies

Measured with the live scorer (the same `score()` as the model run):

| Strategy | What happens | Score |
|---|---|---|
| Do nothing (`inspect-robots run ... --policy yam_scripted -P hold_s=60`, 1,200 steps) | lock stays on the table | 0.0 |
| `nothing` (both arms still, demo harness) | lock stays on the table | 0.0 |
| `air`: close the lock in the air in front of the robot | locked round nothing; it drops onto the table | 0.0 |
| `no_crossbar`: thread the shackle, never bring the crossbar | shackle round the rim and the stand, open; it can be lifted off | 0.25 |
| `short`: push the crossbar on but stop 10 mm short of home | crossbar does not catch; lock open | 0.25 |
| `held`: lock correctly but keep both hands on the lock | locked round both, still held at the end | 0.5 |
| Full demo | locked round the rim and the stand, hands off, still | 1.0 at step 800 |

Not run as variants: a lock round only the stand (it cannot go round the stand's leg at hub height, where the leg is
8 mm from the tyre, but it fits round the leg above the tyre or round the top bar) and a lock round only the rim. Both
score 0 by the rules below: the scorer needs the lock round both.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`).

- The lock goes **round the rim** if the closed loop formed by the shackle's centre line and the straight line across
  its leg ends passes through the wheel's plane inside the rim's mid-line (93 mm from the hub) an odd number of times:
  the rim passes through the lock. In a real trial: the lock cannot be lifted off the wheel without opening it.
- It goes **round the stand** if that loop passes through the stand's opening (between its legs, under its top bar) an
  odd number of times.
- **Locked**: the crossbar has been pushed onto both leg ends until each is within 4 mm of its socket's bottom (16 mm
  in) and within 4 mm of the socket's centre line; from then on it stays locked.
- 1.0 (success, held for 1 s): locked, round both the rim and the stand, every finger pad of both grippers at least
  3 cm from both lock parts, and both parts moving less than 2 mm over the last second.
- 0.5: the same correct lock, closed round both the rim and the stand, but a gripper is still on it or it is still
  moving (the job is done except letting go).
- 0.25: not locked, but the shackle goes round both the rim and the stand (threaded correctly; crossbar missing or not
  pushed home).
- Otherwise 0, including a lock closed round only the rim or only the stand (it does not lock the wheel to the stand)
  and a shackle with both legs between the spokes, inside the rim (the loop then crosses the wheel's plane twice
  inside the rim, an even number).

## Runtime and seeds

About 2 minutes per trial for the full 1,200 steps on an A6000, plus about 1 minute for Isaac Sim to start. The layout
is fixed (`-E object_jitter=0`); there are no seeds to choose.

## Fidelity

All parts are rigid bodies built from capsules, boxes and cylinders. Sizes against real products: the 57-203 tyre
(31.7 cm across), ISO 203 rim and 16 spokes are those of a real 12-inch child's bike; the U-lock matches a standard
4 x 9 inch U-lock such as the Kryptonite KryptoLok Standard (13 mm shackle, about 1.3 kg); the stand matches a common
Sheffield stand (48 mm tube, 75 cm tall), though its legs are 70 cm apart where commercial stands are about 75 to 100 cm
long. The bike's wheelbase is 48 cm, shorter than the 58 to 60 cm of real 12-inch bikes, so that the whole bike fits on
the table. The bike's frame and front wheel are fixed (it is
parked on its kickstand and is not part of the job); its back wheel turns on its axle with light bearing drag. The
tyre is rigid. There is no key: the crossbar catches when both legs are 16 mm into its sockets, and from then on a
fixed joint holds it on the shackle. The shackle's vinyl sleeve and the crossbar's finish are given a friction of 1.0
against the robot's finger pads. No contact force cap is enforced in simulation.

## Real-world build

A child's bike with 12-inch spoked wheels on its kickstand, a Sheffield stand (48 mm tube) bolted down 8 mm from the
back tyre, and a standard U-lock (inside 102 x 229 mm, 12.7 mm shackle, about 1.3 kg) with a vinyl sleeve over the bend.
Use a push-to-lock crossbar or leave the key in the lock. Mark the outlines of the two lock parts and the two spokes on
either side of the lock point. Judge from a photo, then lift the lock by hand to check what passes through it.

## Third-party assets and licenses

The SDK is yam-isaac-lab-arena at 0059d02a1a91bc6834c1d0846022891df0c78730, the program's official dependency,
installed by the user's environment. Nothing from it is redistributed here. Licenses are given as each source states
them.

| Asset | Used for | Source | License as stated by the source | Bundled |
|---|---|---|---|---|
| YAM arm and gripper (URDF, meshes, collision hulls, converted USD) | both arms | SDK `yam_arena/assets/yam/`, from i2rt (SOURCE_COMMIT 120c3c81400171174604e503943f8d1ebc891058) | MIT, Copyright (c) I2RT Robotics | no |
| link2/link3 meshes split into black and white parts | station arm colours | SDK `yam_arena/assets/yam/dream/`, from the dreamlabs 1.0.23 wheel | "MIT, see ../LICENSE.i2rt" (SDK `dream/SOURCE`) | no |
| Station floor and table | room | SDK `station.py`/`scene.py`: boxes with flat colours generated in code | SDK README: "This repository has no license yet: most of it is ported from YAM-inspect-Isaac by Thomas Pierson, which has none." | no |
| Top-camera stand | room | SDK `station.camera_stand`: boxes and capsules generated in code | as the row above | no |
| Dome light and lamp | lighting | SDK `scene.py`, using Isaac Lab-Arena's DomeLight and an Isaac Lab sphere light | as the row above | no |
| top_cam, left_cam, right_cam | policy RGB | SDK pinhole camera sensors | as the row above | no |
| Station fruit, bowl, plate (Arena object library) and water bottle | not used: this task's own objects replace them | - | - | no |
| Bike, cycle stand, U-lock | task scene | this folder (`task.py`, `lockkit.py`): primitive shapes built in code | the authors' own, MIT License in `LICENSE` | yes |

Isaac Lab-Arena release 0.3.1 (c7edf3a9d5d8) ships the Apache License 2.0. Isaac Sim and Isaac Lab are runtime
dependencies under their own licenses. None of them is redistributed here. All task-specific code and geometry
(`task.py`, `lockkit.py`, `demo.py`) is the authors' own, under the MIT License in `LICENSE`.
