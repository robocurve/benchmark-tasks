# Regrasp a dropped terminal block into its mounting grip by handing it over in the air

A grey DIN-rail feed-through terminal block (77.2 x 49.8 x 12.1 mm, 38 g) lies flat on the table in front of the
left arm, as if it had dropped. Lying flat, it can only be picked up across its long top and bottom edges, which is
the wrong grip for mounting. The left arm picks it up and hands it over in the air to the right arm, which takes it
by its two narrow ends and holds it upright, latch foot down, ready to clip onto a rail. It must not be set down
again once lifted. Scoring checks the block's pose and both grippers: the right gripper alone must end up holding it
by its ends, after both grippers held it at the same time.

The task is a [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) task file (`task.py`) in
Dream's `station` scene, with the `yam_arena` embodiment and the official `dream` policy. It was built and run at
yam-isaac-lab-arena commit `0059d02` (the Brev launchable's pin), Isaac Lab-Arena 0.3.1, Isaac Sim 6.1.

## Files

| File | What it is |
|---|---|
| `task.py` | The task file: the block, scoring (`success`, `score`) and the scripted demo (`demo`) |
| `task.json` | The submission's details |
| `assets/terminal_flat.usd` | The block as one rigid body, lying flat (55 KB) |
| `tools/make_terminal_usd.py` | Builds `terminal_flat.usd` from a two-part (housing + latch foot) USD of the block |

## What `task.py` adds to yam_arena

The one-file task format covers boxes, cylinders, spheres and models from Arena's object library, scored from those
objects alone. This task needs three things beyond that, all inside `task.py`; nothing in yam_arena is edited.

1. **Its own model.** The task file registers `assets/terminal_flat.usd` in Arena's asset registry
   (`isaaclab_arena.assets.registries.AssetRegistry`, plain Python) under `terminal_block_flat`, so an ordinary
   `Asset(...)` spawns it.
2. **The grippers in scoring.** A handover is only a handover if both hands held the part and the second one ends up
   with it, so scoring needs the grippers. On the first scored step, `task.py` wraps
   `yam_arena.scene.object_states` so each step's state also holds `left_gripper` and `right_gripper`: the point
   midway between the finger pads (`gripper` link + yam_arena's `TCP_OFFSET`) and the pad opening (the two finger
   joints). Which is which follows the arm's base, not where the gripper is. The terminal's own state is unchanged.
3. **Tilted grips in the demo only.** yam_arena's scripted waypoints point the gripper straight down. The demo hands
   a full rotation to `yam_arena.demo.ArmPlan` through the waypoint's `yaw` (`Orientation`); only `demo()`
   installs this, so policies under evaluation are unaffected.

## Run and score a trial

On the launchable, in Arena's venv (`source ~/IsaacLab-Arena/yam.env`), from the repository root.

Dream-1 (20 Hz, 320x176 frames, as in the walkthrough):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/terminal-handover/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pick up the dropped terminal block with the left arm and hand it over in the air to the right arm. The right arm takes it by its two narrow ends and holds it upright, latch foot down, ready to clip onto a DIN rail. Do not set it down again." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/terminal-handover-dream
```

The scripted demo, which reaches full score:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/terminal-handover/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pick up the dropped terminal block with the left arm and hand it over in the air to the right arm. The right arm takes it by its two narrow ends and holds it upright, latch foot down, ready to clip onto a DIN rail. Do not set it down again." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/terminal-handover-demo
```

The run prints `task_score` at the end (1.0 for success, otherwise the partial credit in `task.json`). Add
`-E headless=false` to watch it on the desktop, and `TERMINAL_TRACE=1` in front of the command to print the scoring
state every 10 steps.

**Runtime per trial** on one L40S: the demo succeeds after about 400 steps (20 s simulated), about 1 minute of wall
time including startup; a Dream-1 trial that runs the full 2,400 steps takes about 5 minutes (0.6 s per inference).

**Seeds.** The start is fixed: `-E object_jitter=0`, and nothing else in the scene is randomised. Runs above used the
default seed. Scoring keeps a small history per trial (lifted, set down, held by both) and resets it when the block
lies untouched at its start; two trials in one run (`--epochs 2`) both scored 1.0.

## The demo

`demo()` (for `--policy yam_scripted`) plays open-loop waypoints with IK on the URDF:

1. Left arm, gripper pointing down, fingers closing front to back: grips the flat block across its height 24 mm
   from one end, where the housing is solid from top to bottom (its middle has wire-entry openings), and lifts it.
2. It turns the block upright in the air (gripper now pointing forward), latch foot down.
3. Right arm, pointing down, fingers closing left to right: comes down over the block and closes on its two narrow
   ends. Its finger target is 72 mm, 5 mm inside the block's length, so the squeeze is limited the way a
   force-limited gripper's would be; at full close a pad slips into a recess in an end face.
4. Left arm lets go, backs off and moves clear. Only then does the right arm carry the block 12 cm to the right and
   hold it there; the run ends after it has held still for 3 s.

Two constants come from runs, not from the model: `GRIP_SLIP` (the block settles 20.5 mm off-centre in the left
grip) and the handover point, chosen where both arms' IK is exact.

## Simplifications

- The block is one rigid body: the latch foot's spring is not modelled. A handover never loads it.
- Colliders are convex decompositions of the CAD mesh; mass is volume x 1,190 kg/m3.
- Pad friction is the station's rubber-like default (1.2 static, 1.0 dynamic), as for Dream's own objects.
- Contact jitter in a grip makes the simulator's velocities unreliable for a held part, so "still" is judged from
  the block's position over the last 0.5 s.

## Rebuilding the model

`assets/terminal_flat.usd` was built from the manufacturer's STEP model (Phoenix Contact XTV 16, 1329672), first
converted to a USD with the housing and the latch foot as separate meshes (`/root/Gehaeuse/Gehaeuse`,
`/root/Rastfuss/Rastfuss`), then:

```bash
python tasks/terminal-handover/tools/make_terminal_usd.py XTV-16_1329672.usd
```

It merges both meshes into one rigid body with convex-decomposition colliders and lays the block flat. It needs only
`pxr` (usd-core) and does not start Isaac Sim.
