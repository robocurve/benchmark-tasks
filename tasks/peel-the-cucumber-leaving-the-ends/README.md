# Peel the cucumber, leaving the ends

A whole cucumber lies loose on a chopping board, and a swivel-blade Y-peeler lies on the table beside it. The robot
has to take the skin off the middle of the cucumber all the way round, leaving the last 2.6 cm at each end, and put
the peeler down. Skin comes off only where the blade's sharp lip is drawn along the cucumber, edge first, in light
contact (1 to 10 N). The underside lies on the board, so the cucumber has to be turned between strokes.

Built for yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), station scene. Three helper files
must sit next to `task.py`, which imports them:
- `parts.py`: the parts kit (multi-colour parts with joints), in a version extended for this task (see below).
- `lots.py`: layout lots; trial k of a run uses lot k mod 3 (A, B, C).
- `tilt.py`: lets the scripted demo lean a gripper; it affects only the scripted demo, never Dream-1 or scoring.

## Run and score a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Peel the cucumber, leaving the ends."

# Dream-1, all three lots (trial k uses lot k mod 3), averaged
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 4000 --epochs 3 --store-frames --log-dir logs/dream

# Scripted demo, all three lots
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 4000 --epochs 3 --store-frames --log-dir logs/demo
```

Put `TASK_LOT=B` (or `A`, `C`) in front of a command to run every trial in one lot. The videos and screenshot are
the first trial's, which is always lot A.

- **Runtime per trial:** at most 200 s of simulated time (4,000 steps at 20 Hz), about 25 to 30 minutes of wall
  time per trial on an L40S. The demo takes about 182 s.
- **Seeds:** submitted runs use `-E object_jitter=0`; the three lots fix the layouts.

## Objects

- **Board** (fixed): wood, 35 x 25 x 2 cm, centred 33 cm in front of the arm bases on the midline.
- **Cucumber** (loose, 300 g): 21.2 cm tip to tip, 4.2 cm across for its middle 17 cm, with rounded ends (spheres
  4.1 cm across centred 8.55 cm from the middle). The skin is dark green; under it the flesh is pale green. In the
  simulator the round body collides as a 24-sided prism (corners 21.2 mm, flats 21.0 mm from the axis), so it lies
  still where it is put: PhysX's round cylinders creep on a flat board. Friction 0.6 static, 0.5 dynamic.
  Rolling resistance: rigid-body angular damping 5 /s (a real cucumber is not perfectly round).
- **Y-peeler** (loose, 50 g, 15 cm): a black rubber handle 10.5 x 2.4 x 2.0 cm; a steel Y whose two arms (4.5 cm
  long) carry the blade on pins at its ends; the blade swivels on those pins through +-34 degrees with a light
  centring spring (0.005 N m/rad). The blade is a slotted strip: the lip on the handle side is the sharp edge (4.5 cm
  long, 2.5 mm deep, standing 0.5 mm proud of the other lip); the other lip is the guard. The edge is its own rigid
  body welded to the blade, so the simulator reports its contacts separately from the guard, the handle and the Y.
  Handle friction 1.2/1.0 (rubber); blade and edge 0.35/0.3 (polished steel).

## Scoring as implemented

`record_contact()` in `task.py` is fed every control step (20 Hz) by an `on_step` callback that asks PhysX for the
contact between the edge body and the cucumber (`create_rigid_contact_view(edge, filter=[cucumber])`):

- **Force:** the total normal force on the edge from the cucumber, `get_contact_force_matrix`.
- **Where:** the force-weighted mean of the reported contact points (`get_contact_data`), in the cucumber's own
  frame: `s` along it, `phi` round it.
- **A cut** is marked between this step's contact point and the last step's when both steps had an edge force from
  1 to 10 N, the edge's facing direction is within 45 degrees of the cucumber's length and it moved that way (edge
  first) at 1 cm/s or more along the body, the blade lay across the body (its length within about 45 degrees of
  square to it), and the contact moved less than 3 cm along and 15 degrees round. The cut is 16 mm wide (the width a
  real 5 cm Y-peeler takes on a 4 cm cucumber), centred on the contact.
- **A gouge**: any step with more than 10 N on the edge marks the skin within 2 mm along and 8 mm round as gouged.
- Nothing else changes the skin: no contact (a hover), contact on the guard, the blade's flat or the handle, a
  stationary press (a jab), a sideways wipe or a back-stroke.
- **Patches:** the middle 12 cm (|s| at most 60 mm) is 6 bands along x 8 strips round = 48 patches, tracked on a
  1 mm x 7.5 degree grid. A patch is peeled when at least 80 % of it is cut and none of it is gouged; it then turns
  pale in the video (its skin mesh is hidden). The 2 cm margins (60 to 80 mm) can be cut freely. The ends
  (|s| 80 mm or more, the last 2.6 cm) must never be cut.
- **Let go and still:** both arms' finger and gripper hulls (`yam_arena.analysis`) more than 5 mm from every shape of
  the cucumber and the peeler; both at rest (under 1 cm/s and 0.5 rad/s); the cucumber's lowest point within 3 mm of
  the board top. Success needs this for 1 s.
- **Whole trial, no credit:** an end cut; no finger or gripper links found for the blade-touch check (counted as a
  failed read); the blade or edge touching any finger or gripper link (more than 0.5 N on a
  contact view of blade and edge against both arms' `tip_left`, `tip_right` and `gripper`); the cucumber's lowest
  point ever more than 3 mm below the board top (off the board); any failed contact read; no reading yet (fail
  closed).
- **Tiers:** 1.0 = at least 44 peeled and none gouged, and let go and still (as above) for 20 control steps in a row
  (1 s), counted every step by the `on_step` callback (`still_steps`); 0.5 = at least 32 peeled and at most 2 gouged; 0.25 = at
  least 12 peeled (two strips: one stroke on the top is not enough); else 0. Trials are run on lots A, B and C and
  averaged.
- **History** is cleared only in the `on_reset` callback (a scene reset); nothing the robot does clears it.

`near_misses.py` drives `record_contact()` through each tier, the force limits on both sides, hover, jabs, wipes,
back-strokes, slow strokes, end cuts, gouges, a hand still on, fail-closed, a blade touching a gripper and a dropped
cucumber; `factory/check.py`-style CASES then check the end-state conditions.

## Lots

| lot | cucumber centre (x, y) | heading | roll (which strip is on top) |
|---|---|---|---|
| A | 31.5, 7.5 cm | 0 | 0.30 rad |
| B | 28.5, 9.0 cm | 0 | 2.40 rad |
| C | 30.0, 6.0 cm | -0.20 rad | 4.40 rad |

x is ahead of the line through the arm bases, y to the robot's left of the midline. The peeler starts in the same
place in every lot: hinge at (33.0, -17.0) cm, handle pointing away from the robot, lying flat on the table.

## The demo

`demo()` is open loop from the lot's published start pose (a coarse pose a camera can see): it reads only where the
cucumber starts and which way it points (`lot.pos("cucumber")` and the direction of its length from the lot's start
orientation), never the lot's name, its roll, or anything from the simulator. The right arm picks the peeler by its handle, leans it 36 degrees so the handle rises toward the hand
(`tilt.py`), lands the blade 6.3 cm behind the middle and draws it handle first along the top to 7 cm past the middle
in 3.8 s, commanded 24 mm below first contact (the arm's softness gives about 1.5 to 6 N on the edge). It then lifts and
parks the peeler to the right while the left arm's closed fingertips turn the cucumber by pushing its top
sideways, aiming at 30 degrees a turn, and the next stroke follows; twelve strokes in all. Measured on the GPU (traces
on the Isaac Sim station), a push turns the cucumber about three quarters of the pure-rolling angle for its length,
because the fingertips slip a little on the skin, so the demo pushes a third further than pure rolling would need.
Each turn still settles on one of the prism's 15-degree flats, so single turns vary; the strokes are spaced closer than
the 44-degree strip one stroke takes, and the overlap covers that. Then the peeler goes back on the table and both arms move away. The peeler turns a few
degrees in the fingers over the run, so later strokes land a few millimetres further back; the 2 cm margins absorb
this.

## Changes to the parts kit

`parts.py` here adds: joints that hang from another joint's link (`parent=`); a `Fixed` link welded to its parent
(own rigid body, for the edge); prism shapes collided as their convex hull; visual-only meshes a task can hide at run
time (`Visual`, `Sim.set_visible`, used for the skin patches); per-part contact offset (2 mm for the peeler),
contact reporting on chosen links (`PhysxContactReportAPI`, threshold 0), per-link friction, and rigid-body angular
damping. `lots.py` here adds a full start orientation per lot (`quat=`).

## Recorded runs

- Scripted demo, `--epochs 3`: 1.0 in each of lots A, B and C (task_score 1.0).
- Dream-1, `--epochs 3`: 0.0 in each lot (task_score 0.0); the arms moved but never took the peeler.

## Fidelity notes

- The cucumber is rigid and keeps its size; real peel takes off about 1 mm. No ribbons of peel are simulated.
- Peeling is a per-patch state change gated by edge contact force, direction and travel read from the simulator.
- Real trials: score from photos of the cucumber turned four ways, on the same 48-patch grid, a patch counting as
  peeled when at least 80 % of its skin is off; a scored grid can be drawn on with a food-safe marker.
- Lots stop a single fixed habit (one memorised layout), not a memorised replay of all three.

No third-party assets: every object is built from primitives in `task.py`.
