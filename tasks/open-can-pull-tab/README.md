# Open a beverage can with a 3D-printed tab lifter: one hand holds the can down, the other hooks the ring and lifts the tab

A full aluminium can stands on the table with its ring-pull tab shut. A 3D-printed tab lifter stands in a rack in front
of the right arm. The left hand takes the can by its body from the robot's side and holds it down; the right hand takes
the lifter by its handle, carries it over the can, lowers its thin plate through the hole of the ring, slides the
plate's tongue under the ring's side bar, and swings the lifter up about the tab's rivet until the score line tears and
the tab stands up past 45 degrees. The left hand then lets go of the can and the can is left standing.

Ring-pull cans can be hard to open with weak or stiff hands: the can has to be held down while the ring is pulled up.
Tab lifters, small hooks that go through the ring, are sold as an aid for exactly that. The task is to use one, with two arms.

## What is distinctive

**The task.** An everyday assistive-tool routine, not a pick-and-place: the pull on the ring is larger than the can's
weight, so the can has to be held while the tab is lifted, and the tab only moves once a hook is behind its ring.

**The skills it needs:**
1. *Tool use with a thin feature.* The tab lifter's plate (3 x 8 mm) goes through an 11.8 x 13 mm hole, to within about
   2 mm of the hole's edges, and its 1.8 mm tongue then has to slide under a 3.5 mm wide bar that stands 0.8 mm off the
   can's end, in a finger well 3 mm deep.
2. *Bimanual force balance.* The left hand holds the can down against the pull (a hold at 20 degrees of tilt, closing
   across the body), while the right hand pulls. A single hand cannot do both.
3. *Following an arc.* The ring turns about the rivet, so the lifter has to turn with it; the demo moves the hook along
   the arc in nine steps, tilting the grip up to 90 degrees.
4. *Letting go cleanly.* The left hand opens its fingers round the can and backs away without dragging it over.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, scoring (`score`, `success`), `self_test()` and the scripted `demo()` |
| `tabkit.py` | The tab's hinge: a stiff spring that gives way when the lift torque passes the score line's resistance (0.10 N m), then holds the torn tab at 0.03 N m |
| `scenekit.py`, `tiltkit.py`, `touch.py` | USD bodies for yam_arena task files (hooks yam_arena without changing it); the tilted grip used by the demo; which robot links touch which bodies (shared helpers) |
| `assets/can.usda` | The can and its tab (floating-base articulation, revolute `tab_rivet`): the USDCraft-10K meshes with rebuilt colliders |
| `assets/opener.usda`, `assets/opener_print.stl` | The tab lifter as a rigid body, and the same shape as a printable STL |
| `scripts/make_can.py`, `scripts/make_can_helpers.py`, `scripts/make_opener.py`, `scripts/make_opener_stl.py` | Build the two USD assets and the STL |
| `scripts/log_can.py`, `scripts/naive_can.py` | A wrapper that logs the state every 0.25 s and adds close-up cameras (needs a `latch_cam.py` close-up camera helper next to it); the wrong-strategy demo (the can is grabbed by its body and shaken, scores 0) |

Third-party assets: NVIDIA USDCraft-10K (huggingface.co/datasets/nvidia/USDCraft-10K), licence Apache-2.0, object
`ds_R043_01` ("Standard 330 ml Can": wall, end and ring-pull tab meshes). `scripts/make_can.py` copies its visual meshes,
presses the finger well into the end's mesh, darkens the tab's material so it shows on the lid, and replaces the colliders.

## Setup

Positions in cm from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the table top.

| Object | Where | What |
|---|---|---|
| Can | on its base at (30.0, -5.0), tab ring pointing +y (left) | Aluminium beverage can, 6.6 cm across, 12.0 cm tall, 0.35 kg (full, sealed); a stay-on ring-pull tab, 18.8 mm wide and 22.8 mm long, riveted at the end's centre, with an 11.8 x 13 mm hole, its underside 0.8 mm over the end panel; a finger well 3 mm deep under the ring |
| Tab lifter | standing in the rack, plate in the slot, tongue towards +x | 3D-printed PLA, 30 g: handle 16 x 16 x 90 mm; plate 3 x 8 mm, 30 mm long, hanging from the handle's bottom; tongue 5 x 8 x 1.8 mm at the plate's foot. The handle's centre is 3.75 mm towards the tongue from the plate, so the pull passes through the pinch |
| Rack | at (22.0, -17.0) | Grey block 3.8 x 3.4 x 2.8 cm with a 1.4 x 1.0 cm slot for the plate and tongue |
| Tray (real trials) | under the can | Shallow, to catch any spray |

Sizes: model extents of USDCraft-10K `ds_R043_01` (66 x 66 x 120 mm); the ring's hole and bars are measured from its tab
mesh. Retail cans differ by a few mm; in real trials measure the hole of the cans used (the plate must go through).

## Scoring

Scored on the final state, with the lifter allowed to stay hooked. The tab angle is read from a side photo or a
protractor (the angle between the tab and the end face).

| Score | Condition |
|---|---|
| 0 | tab lifted less than 20 degrees |
| 0.25 | at least 20 degrees |
| 0.5 | at least 35 degrees |
| 0.75 | at least 45 degrees, but the can is tipped more than 10 degrees from upright, off the table, or a gripper still touches the can or the tab |
| 1.0 | at least 45 degrees, the can standing upright on the table (axis within 10 degrees of vertical), no gripper touching the can or the tab |

## Testing in simulation

Run a model trial (Dream-1 or any policy) from this folder, in a shell with the yam_arena environment active:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the can: hold it down with one hand and use the tab lifter from the rack to pull the ring-pull tab up." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/open-can-pull-tab-dream
```

`task_score` prints the score (1.0 on success, otherwise `score()` at the end of the episode). Seed 0; the scene has no
random placement (`-E object_jitter=0`). About 5 minutes per trial including start-up on one GPU.

The scripted demo (`demo()` in `task.py`, which scores 1.0 at about 43 s, 870 steps):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -P hold_s=2 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 --instruction "x" --scorer task_score --max-steps 2400
```

`python task.py` runs the scoring rules on hand-built states (12 cases, no GPU).

## Reset

A new sealed can for every trial (a tab can be opened only once; a retail can costs about $1), set on its base on a marked
spot with the ring pointing left; put the lifter back in the rack; wipe the pads. A trial takes about 2 minutes plus the reset.

## Fidelity

The can and tab are rigid bodies (USDCraft-10K meshes; colliders: a cylinder body, the rim as slabs, the end panel with
the finger well, the ring as four boxes). The score line is not modelled: the tab's hinge is a spring that gives way when
the lift torque passes 0.10 N m (5 N at the ring's tip), then holds the torn tab at 0.03 N m. The can is a rigid 0.35 kg
body with no liquid.
