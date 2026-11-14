# Free a stuck print-in-place hinge by rocking it

A print-in-place hinge comes off the printer fused solid, and a production operator frees it by rocking the leaf in small
pushes one way and the other. Forcing it round in one swing snaps the leaf instead. The task tests one thing: **loosening
a stuck joint with small alternating loads that stay under the part's breaking load**, not with force or with a steady push.

- **Stuck harder than the leaf can take.** The hinge's four layers hold it with up to 0.8 N m. The leaf snaps when they hold
  it with 0.49 N m (3.5 degrees of turn with four layers on, about 7.5 N at the grip). A hard push gets to the snap long
  before it moves the hinge. A steady push below it moves nothing, and nothing loosens.
- **Rocking loosens it.** Each rocking cycle switches one of the four layers off. After four, the hinge turns freely, and the
  leaf swings round to its stop at 130 degrees.

**The friction is a programmed rule, not the physics of fused plastic.** The layers are PhysX joints at the hinge, each a
torsion spring (2 N m/rad) capped at 0.2 N m, and the task file switches one off per rocking cycle. Nothing in the
simulation wears or heals the hinge.

| File | What it is |
|---|---|
| `free_printed_hinge.py` | The `yam_arena` task file: scene, rocking rule, scoring (`success`, `score`), the step watcher and the scripted `demo()` |
| `bonds.py` | `Hinged` (a revolute joint plus stuck layers that `unstick()` switches off, and `snap()` that breaks the box off), and the step and reset hooks |
| `colliders.py` | Makes a box collide as several smaller boxes (the base leaf as its plate only, the orange leaf as its plate and knuckle) |
| `looks.py` | Puts detailed meshes on a box at spawn and hides the plain box, which still collides |
| `looks/` | The meshes (`*.usdc`), their textures and `make_looks.py`, the Blender script that models and exports them |
| `test_free_printed_hinge.py` | Offline checks (no GPU): the numbers, angle, rocking counter, every scoring outcome, the swing's clearance, the demo's reach and arm clearance, the meshes in their boxes |
| `task.json` | The task's details for the board |

## The scene

Positions in metres in the station frame (x forward, y to the robot's left, from the midpoint of the arm bases), heights
above the table top. The hinge's axis is vertical at (0.34, -0.12), in the right arm's half.

| Part | Body | Detail |
|---|---|---|
| base leaf | fixed box, 6.9 x 1.3 x 4 cm, colliding as its 5 x 1 x 4 cm plate | dark teal, two barrels and a steel pin drawn at the axis |
| orange leaf | free box 8.9 x 1.4 x 4 cm, 50 g, `Hinged` on the base, colliding as a 7.5 x 1 x 4 cm plate and a 1.4 x 1.4 x 2.4 cm knuckle | orange, revolute at the axis, -12 to 130 degrees, damping 0.002 N m s/rad, four stuck layers |
| card | fixed box 28 x 17 x 0.2 cm | a printed protractor round the axis, 60 to 120 amber, 120 to 180 green |

The plates stand 1.5 mm over the card, so nothing drags on it. The leaf's mesh has a neck cut out round the hinge, where the
base leaf's barrels and bridges reach in. The hinge does not collide with the base leaf's plate through its joint.

### The layers, the rocking rule and the snap

| Part | Joint | Behaviour |
|---|---|---|
| layer (4) | a generic joint at the axis, its twist driven toward 0 by a torsion spring of 2 N m/rad, 0.05 N m s/rad, capped at 0.2 N m | together they hold the leaf at 0 with 8 N m/rad up to 0.8 N m |
| rocking rule | `on_step` | a push is the leaf turned 0.9 degrees or more to one side. The first push starts it, each later push to the other side is half a cycle, two halves are a cycle, and each cycle switches the next layer off (`unstick`) |
| snap | `on_step` | when the layers still on hold the leaf with 0.49 N m or more (their stiffness times its angle), or it is 8 mm off its hinge, its hinge and layers are switched off (`snap`): it is a free body, for the rest of the trial |

With four layers on, 0.49 N m is 3.5 degrees. With three it is 4.7, with two 7, with one 14, and with none the leaf is free.
The soft arm's push through a fingertip reaches 0.5 N m only when its hand is far ahead of the leaf, so a hard push snaps the
leaf, and a rocking push of about 0.25 N m (1.8 degrees with four layers on) does not.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active (`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 1,200 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=free_printed_hinge.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The orange leaf of the printed hinge is fused solid. Free the hinge so the orange leaf swings round at least 120 degrees, past the amber part of the card, and stays there. Do not snap the leaf off." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/free-printed-hinge-dream

# Scripted demo (right arm), 1,200 steps (it ends on success)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=free_printed_hinge.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The orange leaf of the printed hinge is fused solid. Free the hinge so the orange leaf swings round at least 120 degrees, past the amber part of the card, and stays there. Do not snap the leaf off." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/free-printed-hinge-demo
```

- The run prints `task_score` at the end. With `-E object_jitter=0` the layout is fixed and no seed is needed.
- The demo (right arm only) puts its fingers, open to a 17 mm gap, either side of the leaf's outer end (6.5 cm from the hinge)
  and rocks it five times: the hand goes 3 cm to one side, then 3 cm to the other, each push 1.3 s, so each pad takes up
  the gap and pushes the leaf. A grip closed on the leaf does not work: the soft wrist tilts instead of pushing. After the
  layers have let go (four cycles are needed, so one is in reserve) it lifts its hand, comes round behind the leaf with its
  closed fingertips and sweeps the leaf along its arc to the stop at 130 degrees, then backs off. The demo reads nothing
  from the simulation: it is a fixed plan from the fixed layout. It reaches 1.0 and ends on success at step 575 (29 s at
  20 Hz). The trials here shared one GPU with several other runs and took far longer, so the time alone on an L40S was not measured.
- Review of the demo's motion: the arms' closest link origins are 32 cm apart (the left arm holds READY). The leaf's peak
  speed is 0.05 m/s and 1.3 rad/s, nothing fell, nothing snapped, and the layers' holding torque peaks at 0.23 N m.
- **Ablations** (the same demo with one constant changed, played in the simulator by a probe because `task-ablate.sh` kept
  finding the shared GPU busy), each scoring 0:
  - `FORCE_S = 1.5`: instead of rocking, the hand goes round to 60 degrees in 1.5 s. The leaf snaps (the layers hold it
    with 0.57 N m, 4 degrees of turn) and scores 0.
  - `FORCE_S = 40`: the same push spread over 40 s. The pad takes the gap up and pushes harder and harder, and the leaf
    snaps at about 0.6 N m (4.5 degrees), no cycle ever counted. It scores 0.
  - `ROCKS = 0`: the demo's sweep on the still-stuck leaf, with no rocking. The fingertip push takes the leaf to 3.6 degrees and
    snaps it, so it scores 0.
- Dream-1 has not been run yet.
- Videos: `inspect-robots video ~/yam-eval/free-printed-hinge-demo/<log>.json` writes one MP4 per camera next to the frames.
- Offline checks: `python -m pytest -q test_free_printed_hinge.py`.

## Scoring

All positions are in metres in the station frame, and heights are above the table top.

- **Angle:** the orange leaf's turn about the hinge axis, counterclockwise from above, 0 where it starts (the base leaf's line).
- **Layers:** four at the start, one fewer after each rocking cycle (see above).
- **Snapped:** at any time the layers still on hold the leaf with 0.49 N m or more, or its centre is 8 mm or more off the
  hinge. Recorded in the episode: the trial scores 0 from then on. A reset clears it, and the layers.
- **0** if the state is not finite.
- **1.0 (success, held 1 s):** not snapped, angle 120 degrees or more, and the leaf still (under 10 mm/s and 0.5 rad/s). A leaf
  held still in a gripper counts.
- **0.5:** not snapped, angle 60 degrees or more at the end.
- **0** otherwise, including doing nothing.

In a real trial a protractor on the hinge gives the angle, and a snapped leaf shows.
