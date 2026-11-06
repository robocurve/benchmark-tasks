# Move both lanyard hooks along, never both unclipped

Three grey anchor cups are fixed in a row 36 cm in front of the robot (anchor 1 left, 2 centre, 3 right, numbered on
white cards). An orange harness block on the table front-right stands in for a climber's harness: two 6 mm lanyard
cords (red and blue) leave it and run straight into anchor 1, each ending in a black hook block lying in that cup.
The robot must move both hooks on to anchor 2 and then anchor 3, one at a time, so that at every moment at least
one hook is in an anchor cup, and then let go. This is the 100% tie-off rule for a twin-leg (Y) lanyard ("At least
one leg must be anchored at all times", FallTech MRES01.3 manual, sec. 3.5; Safewaze: connect "the free leg to the
next anchor point", then disconnect the other leg), relevant to tower work (O*NET 49-9052.00 Telecommunications Line
Installers and Repairers: "Participate in the construction or removal of telecommunication towers or associated
support structures."). It is a toy-scale mock-up of the clipping order: a hook block in an open cup (held or
resting) stands for a clipped snap hook, and no fall or load forces act. Starting and ending with both hooks in one
cup is a simplification: in real work only one leg stays clipped while working and the spare is parked on the
harness's lanyard keeper (Safewaze); FallTech keeps two-point tie-off to "the limited time needed to make the
transition to the new anchor point".

The naive plan (lift both hooks, or let both arms work at once) breaks the rule. The intended plan is a
leapfrog with a handover through anchor 2: one hook goes 1 -> 2 -> 3 while the other stays in anchor 1, then the
second follows while the first is in anchor 3.

**Two arms.** With a straight-down grip, the left arm reaches anchors 1 and 2 only and the right arm anchors 2 and
3 only (offline IK on the URDF, joint limits only: 15 test points in each cup's inner area; the far arm reaches 0 of
15 in anchor 1 or 3). So each hook has to be handed over through anchor 2. This is not strict: a position-only IK
check (any wrist tilt, no collision check) reaches across, so a single arm with a tilted gripper might do it all.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cord simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
    --instruction "Move both lanyard hooks from anchor 1 to anchor 2 and then to anchor 3, one hook at a time: at every moment at least one hook must be in an anchor cup. Finish with both hooks in anchor 3 and let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 2400
```

Closed loop (reads the live states): the left arm takes the red hook from anchor 1 (blue stays), sets it in
anchor 2 and backs off; the right arm takes it from anchor 2 into anchor 3 (blue still in anchor 1). Then the same
for blue, with red already in anchor 3. Each grasp follows the measured hook block and its heading, with the
fingers across the block and the wrist camera on the block's free end; hooks are lowered to 4 mm above the cup floor
before release. Success holds 1 s and ends the episode at step 883 (~44 s); task_score 1.0. `demo/` holds the videos,
`setup.png` and `result.json`.

Shortcut variants (test only, `LANYARD_CHEAT=<name> python cabledemo.py task.py out`), all score exactly 0:
`none` (do nothing), `one` (move only the red hook to anchor 3), `parallel` (both arms work at once: the left lifts
blue out of anchor 1 while the right carries red from anchor 2 to 3, so both hooks are out together), `hold` (the
full demo, but the right gripper closes again on the last hook in anchor 3 and keeps holding it).

## Scoring (`score()`, called every step; the end value counts; `success()` must hold 1 s)

- **0** if at any step neither hook block is in an anchor cup (latched for the episode); if at the end a hook is in
  anchor 3 without ever having been in anchor 2 (each hook's anchor-2 visit is remembered); or if at the end a hook is in no cup or a gripper centre (TCP)
  is within 3 cm of a hook block.
- A hook is **in a cup** when its block centre is inside the cup's inner walls (104 x 56 mm) and below its rim
  (2 cm). Held or resting does not matter for the rule; only the release check looks at the grippers. A hook
  carried over anchor 2 above the rim is not a visit; any step with it inside, below the rim, is (it need not be
  set down there).
- Otherwise **0.5** when both hooks are in anchor 2 or 3, **1.0** when both are in anchor 3. One hook moved
  ahead alone scores 0: the climber has not advanced until both legs have.
- `success()` also requires both cords to be at rest. `distance()`: sum of both hooks' distances to anchor 3 (1 m once
  the rule was broken). `progress()` reports each hook's anchor-2 visit and anchor-3 arrival, whether the rule held,
  and the release.

## How this differs from similar tasks

The invariant that at least one hook is always in an anchor cup is checked at every step while both hooks travel across three anchors, one hook at a time. Unlike tasks that open two gates in turn, the constraint must hold continuously across a sequence of alternating regrasps of hooks on flexible cords.

## Runtime and seeds

- Run limit 2,400 steps (2 min at 20 Hz); the demo needs 883 steps (about 15 min wall time on a shared local GPU at ~1 step/s).
- Layout fixed with `object_jitter=0`; the cords always start straight from the harness into anchor 1 and are not
  jittered, so every trial starts identically.
