# Pick up the dog's ball with the ball launcher

A dog's ball lies on an outdoor mat. The robot must pick it up with a ball launcher (a plastic fetch stick with a cup on the end) without touching it, and hold it up off the mat.

The ball is a sphere, so it rolls. A cup pushed at it from the side shoves it across the mat. A cup pressed down off-centre lands a lip on the side of the ball and squirts it out. The ball is only caught when the cup comes down over the top of it, centred, and is pressed until the two lips spread past the middle of the ball and close under it. The robot holds the 0.36 m tool by its shaft, so it has to place the cup from 10 cm away, and the handle end of the tool has to be managed on the mat.

Ball launchers are sold for this pickup. The maker lists "hands-free pickup, no more touching slobbery balls" for the Chuckit! Sport launcher, and owners do it on every throw.

## Results

These are simulation results, not robot measurements. Both runs used the same task.py and instruction, with `object_jitter=0` and `--epochs 2`.

- **Scripted demo:** 1.0 in lot A and 1.0 in lot B. Each trial ended early on success.
- **Dream-1:** 0.0 in both lots, reaching the 600-step limit. It moved its arms but never picked up the launcher or touched the ball.

## Run it

From this folder (trial k uses lot k mod 2, so `--epochs 2` runs both layouts):

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 \
  --instruction "Pick the dog's ball up with the launcher without touching the ball, and hold it up off the mat." \
  --scorer task_score --max-steps 600 --epochs 2
```

Use `--policy dream` for Dream-1 with the same settings. `TASK_LOT=A` or `TASK_LOT=B` pins every trial to one layout, for debugging only. A trial takes about 30 s of simulated time (600 control steps at 20 Hz). Submissions run with `object_jitter=0`.

## The objects (true size)

World frame: x away from the arms, y to the robot's left, z up. Heights are above the mat top (table + 6 mm).

| Object | Size and material | Where |
|---|---|---|
| Outdoor mat | 0.30 x 0.60 m x 6 mm, green, fixed | x 0.10-0.40 m, y -0.20-0.40 m |
| Edging board | 18 mm thick, 0.60 m long, top 15 mm above the mat, brown, fixed | inner face at x 0.400 m |
| Dog ball | 50.8 mm (2 in) rubber ball, 45 g, orange | lot A: (0.3746, 0.040) m, touching the board; lot B: (0.270, 0.020) m |
| Ball launcher | 0.36 m, 141 g, lime plastic, blue handle | cup-down, shaft along +y, cup centre (0.160, 0.000) m, 10.5 mm above the mat |

Launcher, in its own frame (origin at the cup centre, x along the shaft towards the cup, z towards the cup's back):

- **Cup:** inner radius 26.5 mm (1.1 mm clearance on the ball), 3 mm wall. The back plate is 26.5 mm above the cup centre. The side walls are 40 mm long at 26.5-29.5 mm either side and reach down to the cup centre and no lower, so the sides of the cup are open below the ball's middle.
- **Lips:** two, one at each end of the cup along the shaft. Each is a 28 mm wide, 3 mm thick plate hinged at the back rim, 28 mm from the cup centre. It ends in a tooth 6.5-10.5 mm below the cup centre that reaches 3.5 mm inwards. The tooth edges are 23.0 mm from the cup axis, against the ball's 25.4 mm radius, so each lip must spread about 2.4 mm to let the ball's middle past. A lip opens outwards against a 2.0 N m/rad spring and is free between 0 and 0.5 rad.
- **Shaft and handle:** shaft 300 x 14 x 10 mm, centre line 16 mm above the cup centre, running from 36 to 336 mm behind the cup centre, joined to the cup's back by a short neck. Handle block 70 x 26 x 31.5 mm at 265-335 mm. Lying cup-down, the launcher rests on its two teeth and its handle.

Lot A puts the ball against the edging board; lot B puts it in the open. The launcher starts in the same place in both.

Sources for the sizes:
- Petmate's "About this launcher: Sport 14S" product image draws the launcher (14 in / 35.6 cm) and the ball (2 in / 5.1 cm) to one scale.
- Petmate's listing gives the 14S as 2.6 x 3.6 x 14 in and 0.31 lb (141 g).
- The lip layout was measured on that scaled image, using the ball as the scale (about 0.3 mm per pixel, +/- 3 mm). The cup holds the ball from behind. Two lips along the shaft axis wrap past the ball's middle, the shaft-side one by about 19 degrees. The sides of the cup are open. The opening faces at right angles to the shaft.
- No per-ball mass is published. The maker's small two-pack weighs 0.23 lb with packaging, so at most 52 g a ball; 45 g is an estimate.
- Lip stiffness is not published (see Fidelity).

## Scoring

Per trial:

- **1.0:** the ball is seated in the cup and lifted, and stays within 2 mm of where it was when the hold began, for 2 s in a row (world frame).
- **0.5:** the ball is seated in the cup at the end of the trial, on the mat or lifted.
- **0** otherwise.

**Seated:** in the launcher's frame, the ball's centre is within 4 mm of the cup axis along the shaft and within 6 mm across it. It is 1.1 +/- 3 mm above the cup centre, which means against the cup's back. Both lips are back within 0.12 rad of closed, so both teeth are under the ball. A ball pinched between the teeth above its middle, or wedged half-way in, is not seated (both happened in early runs and scored 0).

**Lifted:** the ball's bottom is at least 60 mm above the mat.

These rules are judged over the whole trial, and breaking any one scores 0:

- No gripper or fingertip surface ever comes within 5 mm of the ball. That counts as touching it. The hand's collision hulls are sampled over every face at 3 mm spacing, not just at their corners.
- The ball's centre never leaves the mat's area (x 0.10-0.40 m, y -0.20-0.40 m) or drops more than 5 mm below its resting height.

The score fails closed: an unknown or non-finite hand or object state at any step, or a skipped control step, scores 0 for the trial. The records reset only in the embodiment's reset hook, never from anything the robot does.

Trials alternate lots A and B, and the task score is their mean. A person can score a trial from video with a ruler. Check that the ball is up in the cup against its back with both lips closed under it, its bottom at least 60 mm above the mat, and that it does not visibly move for 2 s. Check that no finger touched it and that it stayed on the mat.

`near_misses.py` checks 29 cases: every tier, both sides of each limit, a lip left open, a ball pinched low, a cup beside the ball, a touch, an exit from the mat, unknown and stale state, and a played-back hold that drifts 1.9 mm (passes) or 2.1 mm (fails).

## How the catch works, and what it was checked against

Before building, the catch was checked in MuJoCo (CPU) with default contact settings. A force-limited hand (40 N cap) was welded to the shaft, with the lips on sprung hinges as here. With the built geometry and a 2.0 N m/rad lip:

- A centred press from above seats the ball with about 5 N, and a 60 mm lift holds it still.
- An offset of 28 mm or more along the shaft, or 32 mm across it, lands a lip on the side of the ball and squirts it out. It rolled 0.05 to 0.8 m.
- A cup pushed at the ball from the side, along or across the shaft, rolls it away.
- Unseating the held ball takes about 0.9 N of extra pull, about twice its weight.

These are probe results, not measurements. In the GPU simulation the demo presses with the handle end resting on the mat as a fulcrum. The fingers pinch across the shaft and cannot stop it turning about that axis, so the shaft pivots in them. The demo's press seats the ball with both teeth back under it, and the lifted ball stays inside 2 mm. In the last GPU trace before the final demo (same demo, before the 2 mm hold rule), the closest hand surface stayed 58-62 mm from the ball and the peak sampled fingertip force was 30 N.

## Fidelity

- **True size:** the launcher, cup and ball are built from the maker's scaled image and listing.
- **Shape:** the curved cup shell and lips are approximated by boxes, and the lips are rigid plates on sprung hinges rather than bending plastic. The shaft is straight; the real one curves. The ball is a rigid sphere.
- **Estimates:** lip stiffness (2.0 N m/rad), ball mass (45 g), frictions (ball 0.8/0.7, launcher 0.4/0.3, mat 0.9/0.8) and the neck and handle sizes are estimates, not calibrated to a real unit. The press force a real launcher needs was not measured.
- **The mat:** PhysX has no rolling resistance, so the mat's resistance to the ball rolling is stood in for by angular damping on the ball (4 per second).
- **Layouts:** the two lots stop a single fixed replay. They do not stop a policy that memorises both layouts.
- **The demo:** it is open loop from the published start layout and the ball's visible position in each lot. It reads no lot label, contact state, joint angle or hidden parameter.

## Licence

All geometry is generated from boxes and spheres in `task.py`; no third-party assets. Code MIT, text CC BY 4.0. `parts.py` (articulated parts, with sphere shapes and per-part angular damping added), `lots.py` (layout lots) and `observation/` (face-sampled hand surface and fingertip force readings) are helper modules that patch the yam_arena embodiment in the task's own process only; nothing installed is modified.
