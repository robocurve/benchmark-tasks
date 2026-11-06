# Press the red button

A machine's reset button sits on a vertical control panel behind a clear guard. The guard is closed
on every side; its only opening is a 40 mm service port, and the gripper does not fit through it. A
rack beside the guard holds three bars. Only one of them is both long enough to reach the button and
thin enough to pass the port.

The instruction names neither the bar nor the method: **"Press the red button."**

## Environment

| | |
|---|---|
| `yam-isaac-lab-arena` | `0059d02` |
| Isaac Lab-Arena | 0.3.1 (installed: `isaaclab_arena` 0.3.0) |
| Isaac Sim / Isaac Lab | 6.1.0 / 3.0 |
| GPU used for the runs below | one RTX-class GPU, `cuda:0` |
| Control | 20 Hz, `object_jitter=0` |

Start every shell with `source <ARENA_DIR>/yam.env`.

## Files

| File | What |
|---|---|
| `press_button_through_hole.py` | the task file: `SCENE`, `OBJECTS`, `success`, `distance`, `score`, `demo` |
| `cage.py` | the scene's geometry and the measurements the scorer takes |
| `simkit.py` | the spawn helper: materials, glazing, compound bodies, the spring joint, smooth motion |
| `shortcuts.py` | six scripted shortcut policies |
| `check_reach.py` | offline checks: the port blocks the gripper, the demo's IK, clearances, the layout |
| `test_press_button_through_hole.py` | offline unit tests over every row of the scoring table |

No asset files ship with the task. Every body is built from Isaac Lab mesh primitives; the only
external references are NVIDIA's own MDL materials, fetched from the Isaac Sim asset bucket at scene
build time and cached by Omniverse afterwards.

## Running it

```bash
cd <this repo>

# the scripted demo (reaches full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/press-button-through-hole/press_button_through_hole.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Press the red button." --scorer task_score \
  --max-steps 1200 --store-frames --log-dir ~/yam-eval/pbth-demo

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/press-button-through-hole/press_button_through_hole.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Press the red button." --scorer task_score \
  --max-steps 1200 --store-frames --log-dir ~/yam-eval/pbth-dream

# one shortcut policy (hold | gripper | rule | baton | throw | shove)
PBTH_SHORTCUT=gripper inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -P hold_s=20 -E task=tasks/press-button-through-hole/shortcuts.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Press the red button." --scorer task_score \
  --max-steps 1200 --log-dir ~/yam-eval/pbth-sc-gripper

# offline, no GPU
cd tasks/press-button-through-hole
python check_reach.py
python test_press_button_through_hole.py        # or: pytest test_press_button_through_hole.py
```

`PBTH_TRACE=1` on any run prints the rod's ends, its tilt and the button's travel every tenth control
step; that is how the numbers below were measured.

**Seeds and determinism.** The layout is fixed: with `-E object_jitter=0` nothing is moved at reset,
so there is one starting state and no seed to set. The scripted demo is open loop, so it replays
identically. PhysX is not bit-deterministic (`deterministic=False` in Arena's default `PhysxCfg`), so
the button's final travel varies by a fraction of a millimetre between runs; the demo settles at
about 7 mm against a 5 mm threshold.

**Runtime per trial.** The demo plan is 19.8 s of motion, 396 control steps at 20 Hz, and the episode
ends on success about a second later. Wall-clock, on the machine above: roughly 1 minute per trial
once Isaac Sim is up, and 3 to 6 minutes for the first trial in a fresh process. A full 1,200-step
trial that never succeeds takes about 2 minutes.

## Results

| Policy | `task_score` | Notes |
|---|---|---|
| `yam_scripted` (the demo) | **1.0** | ends with `termination_reason="success"` after the 1 s hold |
| `dream` (Dream-1 beta) | **0.0** | ran the full 1,200 steps, `termination_reason="max_steps"` |
| shortcut `hold` | 0.0 | both arms hold the ready pose |
| shortcut `gripper` | 0.0 | wrist rolled 90 deg, closed gripper driven at the port |
| shortcut `rule` | 0.0 | the short rule taken and pushed at the port |
| shortcut `baton` | 0.0 | the thick baton taken and pushed at the port |
| shortcut `throw` | 0.25 | the rod lined up with the port and dropped instead of pushed |
| shortcut `shove` | 0.0 | each arm leans on its own side of the guard |

`throw` scores 0.25 rather than 0 because it does pick the right bar up, which is exactly what the
0.25 tier is for. The point of the run is the ceiling it hits: a dropped bar never reaches 0.5 (a bar
50 mm inside the guard) or 1.0. Every other shortcut scores 0.

## Why each shortcut fails: the measured baselines

**Reaching in with the gripper.** `check_reach.py` places the YAM's own URDF collision hulls by
forward kinematics with the fingers closed and measures the hand's cross-section perpendicular to the
approach axis, by depth behind the pad tips:

| Depth behind the pad tips | Clear square the hand needs | Through a 40 mm port |
|---|---|---|
| 5–15 mm | 15.5 mm | fits |
| 20–25 mm | 27.4 mm | fits |
| 30 mm | 35.6 mm | fits |
| **35–50 mm** | **40.1 mm** | **blocked** |
| 55 mm | 55.2 mm | blocked |
| 60 mm | 58.8 mm | blocked |

So the closed hand enters the port **30 mm** and stops. The button face is **130 mm** in: it is short
by **100 mm**. The port is 40 mm rather than 30 to leave lateral slop for the 16 mm bar — a gripper
cannot centre a bar to better than about a centimetre — and it costs nothing, because the hand's
width jumps from 35.6 mm to 40.1 mm right at that size.

**The wrong bars.** The oak rule is 140 mm long and reaches 70 mm past a grip at its middle; from the
nearest the arm can stand (x = 0.10 m) the button needs 401 mm of reach, so the rule is **331 mm
short**. The oak baton is 55 mm square against a 40 mm port: **15 mm too wide**. The reach rod's shaft
is 16 mm square, leaving 24 mm of clearance in the port.

**Throwing or abandoning the bar.** The button is sprung, so it returns as soon as nothing holds it,
and `success()` additionally requires the rod to be lying through the port at that moment: far end
inside the guard, near end still outside, within 12 degrees of horizontal. The guarded volume is
148 mm deep and the rod is 280 mm long, so a rod that reaches the button *must* be sticking out of
the port. The `throw` shortcut lines the rod up and lets go: it drops, and nothing is pressed.

**Moving the guard.** Every part of the guard, the control panel and the rack is spawned without a
mass, so each is a static collider. The `shove` shortcut leans on both sides and nothing moves.

**Reaching round or over the guard.** The guard has closed sides, a closed glazed roof and a closed
back (the control panel), so the only path in is the port. The bore through the control panel is
behind the button; a finger entering it from behind would push the button *out*, which is the wrong
direction and never scores.

## The task

**Instruction:** `Press the red button.`

**Scoring** (`--scorer task_score`, `SCORE_OVER = "best"`, so a trial keeps the furthest it got):

| Score | Condition |
|---|---|
| 1.0 | the button at least **5 mm** in, held for **1 s**, while the rod lies through the port: far end inside the guard and at least 50 mm past its front face, near end still outside, within 12 deg of horizontal |
| 0.5 | any bar's end at least **50 mm** inside the guard, the button not pressed |
| 0.25 | the reach rod's centre at least **30 mm** from where it sat on the rack |
| 0.0 | otherwise |

Full score comes only from `success()` holding for a second. `score()` itself stops at 0.5, so a
momentary touch cannot latch 1.0 through `SCORE_OVER = "best"` — an earlier version of this task had
exactly that hole, and a unit test now pins it shut.

Every condition is a length a judge can measure: the button's travel against the panel face with a
5 mm feeler or a depth gauge, how far the bar protrudes from the port (subtract from its known
280 mm), and whether the rod is off its rack.

**Run limit:** 1,200 steps, 60 s at 20 Hz — 3.0x the demonstrated solution's 396 steps. Deliberately
more than the 2x rule of thumb, because unlike the demo a policy has to find the port and choose
among three bars before it can start.

## Assets and materials

Nothing is downloaded as a task asset. Every body is an Isaac Lab **mesh primitive**
(`MeshCuboidCfg`, `MeshCylinderCfg`), so every collider is the exact shape of its visual mesh and the
collider-to-visual error is **0 mm by construction** — there is no decomposition to go wrong. Shapes
are spawned as meshes rather than analytic shapes only so that each one *can* carry a mesh
approximation if it ever needs one; here every body uses `convexHull`, which for a box or a cylinder
is exact.

**Visual materials** are NVIDIA's MDL library, referenced from the Isaac Sim 6.1 asset bucket:

| Part | MDL | Source |
|---|---|---|
| guard glazing | `Base/Glass/Clear_Glass.mdl` | NVIDIA Isaac Sim 6.1 asset library |
| rod | `vMaterials_2/Metal/Aluminum_Brushed.mdl` | NVIDIA Isaac Sim 6.1 asset library |
| rule, baton | `Base/Wood/Oak.mdl` | NVIDIA Isaac Sim 6.1 asset library |
| control panel | `Base/Metals/Steel_Stainless.mdl` | NVIDIA Isaac Sim 6.1 asset library |
| frame, rack posts | `Base/Metals/Steel_Carbon.mdl` | NVIDIA Isaac Sim 6.1 asset library |
| button | a tinted, self-lit preview surface | — |

These are NVIDIA's own materials, shipped with Isaac Sim and served from its public asset bucket;
they are used by reference and none is redistributed here.

**Glazing had to be measured, not guessed.** A `PreviewSurfaceCfg` with a low `opacity` does not
render as clear glass in this renderer — it renders as a milky sheet that hides what is behind it. Six
variants were put side by side in one scene, each with a red button 90 mm behind it, and photographed
through the top camera: `opacity` 0.05, 0.15, 0.35 and 0.60, plus `Clear_Glass` and a translucent
plastic MDL. Only `Clear_Glass` actually reads as clear. That is why the guard uses an MDL rather
than an opacity, and why the button is **illuminated** and the guard **framed** — see *Known
limitations*.

**Physics materials** are set per body, which the stock task-file path cannot do (it gives every
object one shared rubber-like material):

| Body | Static / dynamic friction | Restitution | Stands for |
|---|---|---|---|
| guard glazing, button | 0.40 / 0.33 | 0.25 | polycarbonate / ABS on steel |
| control panel, frame, rack posts | 0.60 / 0.50 | 0.45 | steel on steel |
| rod | 0.55 / 0.45 | 0.40 | aluminium on steel |
| rule, baton | 0.40 / 0.32 | 0.30 | oak on steel |

Contacts use Arena's own offsets (`contact_offset` 5 mm, `rest_offset` 0), matching
`yam_arena.scene`, so the thin finger pads register contact before they penetrate.

## What `simkit.py` patches, and why

A `yam_arena` task file may only list `Box`, upright `Cylinder`, `Sphere` and library `Asset`
objects: one rigid body, one primitive collider, one flat colour, one shared friction material, no
joints, no transparency. This task needs more, so `simkit.py` adds it **at import time**, without
editing `yam_arena` or `isaaclab_arena` — both stay at their pinned versions, and everything needed
ships in this folder.

| Patch | What it enables | How |
|---|---|---|
| `use_production_assets()` | MDL materials and library assets load at all | see below |
| `patch_scene()` | per-body MDL/opacity/emission/friction/restitution/collision, compound bodies, spring joints | a `sys.meta_path` finder wraps `yam_arena.scene`'s loader and replaces `task_object` the moment the embodiment imports that module — after Kit has started, so the task file itself still imports nothing from Isaac Sim |
| `patch_demo()` | a waypoint may carry a full 3x3 TCP rotation instead of only a yaw | wraps `yam_arena.demo.grip_rotation`, which `ArmPlan` calls by attribute, so a 3x3 matrix passes through and a float still reaches the original |
| `patch_arm_plan()` | the arm glides along a spline instead of turning square corners | replaces the `ArmPlan.target` *method* on the class; `yam_arena.scripted` binds the class object itself, so the policy sees the patched method and nothing is rebound by name |

`Part` subclasses `task_file.Box`, so `task_file.load`'s `isinstance` check and all of its validation
still apply unchanged.

### A bug this task had to work around

Arena 0.3.x rewrites its object-library root to the **`omniverse-content-staging`** bucket:

```python
# isaaclab_arena/assets/nucleus.py
# TODO(2026.07.14, Point Arena assets to the production bucket before release)
ARENA_NUCLEUS_DIR: str = ISAACLAB_NUCLEUS_DIR.replace("omniverse-content-production", "omniverse-content-staging")
```

That bucket no longer serves those files. Every library asset fails to load, and so does the stock
`station` scene itself:

```
error: EmbodimentFault: Failed to open layer
@https://omniverse-content-staging.s3-us-west-2.amazonaws.com/.../objects/objaverse/apple_01.usd@
```

The identical paths are served from `omniverse-content-production`, so
`simkit.use_production_assets()` rewrites the host when an asset is looked up, from inside the same
deferred scene patch. This task uses no library `Asset`, but keeps the fix because a reviewer running
a stock install will hit the same bucket.

## Fidelity: what the simulation simplifies

* **The spring is a joint drive, not a spring.** The button rides a world-anchored prismatic joint
  whose linear drive returns it at 220 N/m with 6 N·s/m of damping. A real button's return is not
  perfectly linear and has detent and stiction that are not modelled, and the "bore" has no
  clearance: the joint constrains the button exactly to its axis rather than letting it rattle.
* **Nothing deforms.** The polycarbonate, the steel panel, the oak bars and the aluminium tube are
  all rigid. A real reach rod cantilevered 240 mm from its handle would sag about a millimetre and
  flex when it met the button; here it does not.
* **The guard is clean glazing in a frame.** No fasteners, no haze, no scratches. The port has sharp
  square edges with no chamfer, so it offers no lead-in; a real service port usually has one.
* **The bars are perfect square stock on flat posts** with the frictions listed above — no knurling,
  no paint, no burrs. The rod cannot roll because it is square, and its handle is part of the same
  rigid body rather than a separate fitting.
* **Contact forces are not scored.** The embodiment runs without contact sensors, so "the guard was
  hit hard" cannot be measured, and the scoring table does not try to. The guard is a static
  collider, so a collision simply stops the arm.
* **One fixed layout**, no randomisation at all, which is what the screenshot shows and what the
  open-loop demo is written against.

## Reset between trials

Nothing is consumed. The button springs back out by itself as soon as the bar is withdrawn; check it
stands 18 mm proud of the panel again. Lay the three bars back across their posts, each along the
bench. In simulation a reset restores all of this exactly.

## Known limitations

* **The arm occludes the guard in the top camera during the final push**, because it has to be
  between its own base and the port. The wrist camera is filled by the rod at that moment, so the
  clearest view of the press is the top camera in the seconds before the arm covers it.
* **Visibility drove several design choices, and they are worth stating plainly.** With plain clear
  glazing and no frame, the guard is so transparent in the overhead view that it reads as an open
  box — which would make a reviewer think the task is cheatable from above. The dark steel frame, the
  roof border with two cross-ribs, the proud port bezel and the illuminated button are all there so
  that the barrier, the opening and the target are each legible at 320 x 176. They are also all
  realistic for a machine guard, but visibility is why they are there.
* **The `gripper` shortcut's plan reaches a worst IK error of about 8 mm**, because a wrist rolled 90
  degrees to point the fingers forward is near a joint limit at that reach. It still drives the hand
  into the port hard enough to show the port stops it; every other plan here solves to under 0.1 mm.
* **The rod has a weighted handle for a kinematic reason, not a cosmetic one.** The pads are only
  ~20 mm long, so the couple they can resist against a cantilevered bar runs out abruptly. Measured
  on this rig: a plain bar with 210 mm of overhang droops 0.4 degrees, 270 mm droops 8, and at 330 mm
  it falls out of the jaws altogether. Neither a lighter bar (88 g to 45 g) nor a harder squeeze
  moved that. A plain bar only behaved at 210 mm because the 150 mm *behind* the jaws counterweighted
  the overhang -- exactly what gripping the end throws away. The 80 x 28 mm handle puts that mass
  back 40 mm from the jaws instead of 150, and the tool then holds level to 0.7 degrees while being
  gripped by its end.
* **The arm cannot be moved far enough from the guard to stop occluding it, and raising the station
  makes it worse.** An 80 mm plinth was tried: the gripper rises with the target, so the button moved
  further *into* the arm's silhouette, not clear of it. The plinth was removed. Moving the guard
  laterally, so the camera's sight line passes beside the arm rather than through it, is the one
  option not yet tried.
