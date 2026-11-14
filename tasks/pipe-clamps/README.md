# Fix a copper tube in two self-closing pipe clamps

The robot does steps 2 and 3 of Oatey's self-closing pipe clamp instructions on two clamps along one run: it lifts a
600 mm length of 1/2 in Type L copper tube out of two cradles, puts it into the mouths of two clamps screwed 360 mm
apart on a 2x6 plank, and presses it in until each clamp double-clicks shut. The first click holds the tube but lets it
move; the second locks it. Both clamps take the same rigid tube: it has to go into both mouths and be pushed through
both clamps' two detents, and it counts only where it runs right through a clamp's arms.

Files: `pipe_clamps.py` (the task), `mechanism.py` (the scene builder), `task.json`.

## Sources

| Number | Value | Source |
|---|---|---|
| Install steps | "1 Secure the self-closing clamp to the mounting surface. 2 Insert the pipe into the self-closing clamp. 3 Press the pipe inwards until you feel a double click. The first click holds pipe in place with the ability to position, the second click locks the pipe in place." | Oatey self-closing pipe clamp installation instructions (oatey.com PDF) |
| Clamp 34338 (1/2 in CTS) | tube 14-16 mm; B 1.1 in (27.9 mm, the base's width across the tube, front view); H 1.4 in (35.6 mm, closed height); h 0.94 in (23.9 mm, read here as the tube's axis over the base, the one reading that fits with H); load up to 134 lb. The side view gives the width along the tube only for 34344 (0.79 in) and 34345/6 (1.54 in) | Oatey self-closing pipe clamp specification sheet |
| Tube | 1/2 in Type L copper: OD 5/8 in (15.9 mm), wall 0.040 in, 0.285 lb/ft (0.254 kg for 600 mm) | Cooney Brothers copper tube Type L table |
| Jobs | Plumbers, Pipefitters, and Steamfitters: 510,600 jobs (2025); Core task "Attach pipes to walls, structures, or fixtures, such as radiators or tanks, using brackets, clamps, tools, or welding equipment." | O*NET 47-2152.00 |

Our choices (not from a source): the clamp's 28 mm length along the tube, its inner mechanism (Oatey does not publish one), the springs, the detent and
its two notch depths (Oatey publishes no click force), the 360 mm spacing, the plank, the cradles, and the frictions.
`task.json` `setup` lists every value; it is generated from the task file's constants.

## Sim results

yam_arena, scripted demo, one fixed start (`object_jitter=0`), 20 Hz, 2,400-step limit. The filmed demo scored 1.0 with success at step 288 (14.4 s) and took 115 s of wall time on a shared GPU. The one Dream-1 run scored 0 over 2,400 steps. Both runs hashed `pipe_clamps.py`, `mechanism.py` and `task.json` before starting; the hashes match the files here (`task.json` as it was before its `scores` line was filled in).

## Sim against real

- A real clamp's arms close through its own moulded linkage. Here the trigger pad under the mouth presses the arms'
  tails, so pushing the tube down onto the pad closes the arms round it; the arms never touch the tube on the way down.
- The clicks are a spring plunger in the clamp's foot riding into two V notches on a bar that hangs from the pad. The
  lock holds the pad down against its own and the arms' springs with about 5 N (`fidelity` in `task.json`); the real
  clamp is rated to 134 lb, so a hard upward pull would reopen this one.
- The tube is a solid cylinder with the tube's mass and outer diameter.
- The scorer reads joint positions, poses and gripper hulls, not forces.

## Third-party code

`mechanism.py` is adapted from the MIT-licensed `mechanism.py` of Robocurve PR #223, as used by earlier board tasks.
No meshes or other assets: every shape is a primitive or convex hull built in the task file.

