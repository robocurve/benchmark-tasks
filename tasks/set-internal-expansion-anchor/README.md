# Set a drop-in expansion anchor

A 3/8 in drop-in anchor (Simpson Strong-Tie DIAB37) sits unset in a hole drilled in a concrete block, with its hand setting tool (DIABST37, called the punch below) standing in it. Hold the punch, strike it with the hammer until its shoulder meets the top of the anchor (the internal plug has then spread the sleeve against the hole), lift the punch out, and put the punch on its V rest and the hammer on its cradle.

## Hardware

Both parts are stock catalogue items sold by fastener and tool retailers:

| Part | Catalogue data | Price seen at US retailers |
|---|---|---|
| Simpson Strong-Tie **DIAB37** drop-in internally threaded anchor | 1/2 in drill bit, 1-9/16 in long, 3/8 in rod, 16 threads per inch, 5/8 in thread length, zinc-plated carbon steel, preassembled expansion plug, box of 50 | $14.00 to $28.26 per box of 50, $0.28 to $0.57 each |
| Simpson Strong-Tie **DIABST37** hand setting tool for DIAB37 and DIABL37 | carbon steel, listed weight 0.206 lb (93 g) | $4.41 to $5.09 each |

The maker's installation instruction is the task's setting check: drive the expander plug toward the bottom of the anchor until the shoulder of the setting tool makes contact with the top of the anchor.

A set anchor stays expanded in its hole, so every trial uses a fresh DIAB37 from the box in a fresh hole. Drill the hole with a 1/2 in carbide bit, 1-9/16 in deep, so the anchor sits on the bottom with its top flush with the surface. A block takes several holes: drill the next one at least 50 mm from a used one and clamp the block so that the new hole is at x=0.300, y=0.000 m. The setting tool and hammer are reused.

What the simulation copies from the catalogue: hole diameter and depth, anchor length and the 3/8 in bore. The modelled tool weighs 97 g against 93 g listed. What the maker does not publish and is modelled here: the anchor's outside diameter (12.4 mm), the number and length of the sleeve slits (four, 18 mm), the plug (8 mm long cone, 7.4 to 9.0 mm), and the tool's diameters and pin length (12 mm body, 6 mm × 28.5 mm pin, 131.5 mm overall). On hardware the tool is used as supplied; the V rest takes any round tool of about 12 to 13 mm. The four marks the real tool leaves on a set anchor's top face are not modelled. The real parts' starting gap between tool shoulder and anchor top was not measured for this task; it is 1.8 mm in the simulation. Measure it at setup on hardware and use 78% of it in place of the scorer's 1.4 mm plug travel.

## Setup and fabrication

`geometry/geometry.json` holds every task solid's convex vertices, pose, friction, mass and inertia, and the four sleeve-root hinges. `SETUP-INVENTORY.json` lists the bounds, mass and starting pose of every body. Units are metres and kilograms. No external asset is needed.

- Rig: the stock two-arm station, arm bases at x=0, y=±0.305 m on the table edge; table top at z=0.7475 m.
- Concrete block, 120×120×80 mm, with a 12.7 mm (1/2 in) hole 39.69 mm (1-9/16 in) deep at x=0.300, y=0.000 m; in the simulation the hole is at the block's centre. Clamp the block to the table.
- Anchor in the bore: a fresh DIAB37, modelled as steel, 12.4 mm outside diameter, 39.69 mm long, lower 18 mm split into four petals, with a loose steel cone plug (8 mm long, 7.4 to 9.0 mm diameter) resting inside the petals. Its top is flush with the concrete.
- Setting punch: the DIABST37, modelled as steel, 97 g, with a 6 mm × 28.5 mm pin and a 12 mm × 103 mm body. It starts standing in the anchor on the plug, shoulder 1.8 mm above the anchor top.
- Club hammer, 413 g: steel head 60×30×30 mm, ash handle 280×20×14 mm. It lies flat on a two-block wooden cradle, handle end at x=0.140, head at x=0.39 to 0.42, y=−0.210 m.
- Punch rest: two wooden V blocks at x=0.200 and 0.315 m, y=0.250 m, which hold the punch level along x.

Nothing is powered or pressurised. The anchor is unset at the start: the plug is up and the petals are slack in the bore.

Reset: put a fresh DIAB37 in a fresh hole as described under Hardware, stand the punch in it and put the hammer on the cradle at the listed poses. There is no reset during a trial.

## Run and score

Install Isaac Sim 6.1, Isaac Lab/Arena and the YAM Inspect Robots embodiment from the runtime repository named in `SOURCE-LICENSE.md`, with NumPy and SciPy in the same environment. One fixed starting layout, seed 0, no object jitter. The task sets 960 Hz physics and 20 Hz control (48 physics steps per action) in `native/task_runtime.py`; robot gains, limits and the gripper are stock.

Scripted demo, from this folder:

```sh
ROBOCURVE_TRACE_DIR="$PWD/run-demo/trace" inspect-robots run \
  --embodiment yam_arena --policy yam_scripted \
  -E task="$PWD/native/full_entry.py" -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction 'Set the drop-in anchor in the concrete hole with its setting tool and the hammer, then lay the setting tool on its V rest and the hammer on its cradle.' \
  --scorer task_score --max-steps 1440 --seed 0 --no-prompt \
  --store-frames --log-dir "$PWD/run-demo/inspect"
```

Dream-1: the same command with `--policy dream` and `-E task="$PWD/native/model_entry.py"`. `model_entry.py` loads the same scene and scorer and nothing of the scripted demo.

The score is printed at the end of the run and stored in the run's JSON under `--log-dir`. To recompute it from the recorded poses without physics:

```sh
python native/score_trace.py run-demo/trace/observations.jsonl --geometry geometry/geometry.json
```

Wrong-path checks play the same demo with one part left out; scene and scorer are unchanged:

```sh
ANCHOR_REFERENCE=no_strike   inspect-robots run ... -E task="$PWD/native/full_entry.py" ...   # every motion except the blows: 0.0
ANCHOR_REFERENCE=leave_punch inspect-robots run ... -E task="$PWD/native/full_entry.py" ...   # anchor set, hammer returned, punch left in the anchor: 0.5
```

Offline self-test (no simulator; checks the plan against joint limits, speeds and the run limit, the scorer on synthetic traces for every stage and wrong path, the sleeve material law and the demo's held-tool check):

```sh
python native/full_entry.py
```

The run limit is 1,440 actions (72 s at 20 Hz). The demo uses 1,013 actions (50.65 s). One trial takes about 30 to 35 minutes of wall time for the demo (about 49 minutes for a full 1,440-action run) on one RTX 3090, most of it the 960 Hz contact physics and per-step trace recording.

## Scoring

The scorer (`native/primary_score.py`) reads actual object poses, contacts and robot link poses each control step. It never reads the demo's phases or commands.

- **0**: anything less than a set anchor, including the untouched start, a plug pushed down without the hammer, an anchor pushed or pulled more than 0.8 mm or tilted more than 2° from its start, or tools put away without setting.
- **0.5**: the anchor is set. The punch shoulder has been seen flush on the anchor top (gap between −0.20 and +0.15 mm, centred within 0.4 mm, axes within 2°) while the plug was at least 1.4 mm below its start and all four petals were in contact with the concrete; at least 0.7 mm of that plug travel was made in control steps in which the hammer pressed on the punch with 10 N or more (largest contact force over the step's physics steps); and the anchor stays in place with the petals in contact.
- **0.75**: also the whole punch is at least 10 mm above the anchor top (or entirely beside the bore), with anchor and plug at rest for 0.55 s (11 steps).
- **1.0**: also the punch lies on its V rest (origin within x 0.170 to 0.200, y 0.230 to 0.270, z 0.780 to 0.800 m, axis within 5.7° of x), the hammer is back on its cradle within 10 mm and 5° of its start pose, every robot link is at least 20 mm from the punch, hammer and anchor, and tools, anchor and plug are at rest for 1.05 s (21 steps).

"At rest" means a speed below 5 mm/s and 0.1 rad/s, taken from the pose change over each 0.05 s control step. The simulator's own velocity readback is recorded but not used: for the set sleeve and plug, jammed in the hole, it stays at several rad/s while their poses do not change.

On real hardware the same outcome is checked with simple tools: a feeler gauge between punch shoulder and anchor top before the punch is lifted (the maker's own check, backed by the four marks the tool leaves on the anchor), a depth gauge on the plug through the anchor's bore after the punch is lifted, video for the hammer blows, and a ruler and photo for the stowed tools and arm clearance. Petal contact with the hole is read directly in simulation; on hardware a set anchor cannot be pulled out of the hole by hand. No pull-out or proof-load test is part of the score.

Each run writes `observations.jsonl` (poses, velocities, contacts, robot links and the score every control step), `impact-contacts.jsonl` (hammer/punch, punch/plug and punch/anchor contacts every physics step, since a 20 Hz sample can miss a blow), `material.jsonl` (petal root angles and plastic set every physics step) and the demo's own controller and alignment logs to `ROBOCURVE_TRACE_DIR`.

## How the demo works

`native/full_entry.py` replays the joint targets in `geometry/full_plan.json` (built by `native/plan_reference.py` from the arm kinematics alone) through the ordinary 14-D joint action.

1. The left hand takes the punch's upper section in a level side grasp and keeps holding it.
2. The right hand takes the hammer handle from above, 40 mm behind the head centre, lifts it and rolls it a quarter turn about the handle. The striking face then points down and the two pads sit above and below the handle, so a blow's reaction is carried by a pad face. In a grasp with the pads on the sides of the handle the same reaction turns the hammer in the hand.
3. Before each blow both hands correct from measured poses: the left centres the punch on the anchor, the right levels the hammer face 100 mm above the punch top (the arms sag a few millimetres under the tools).
4. Each blow is a 100 mm downward stroke in 0.4 s. Blows stop when the shoulder gap is measured at 0.08 mm or less; at most five are used.
5. The hammer is rolled back, brought over its cradle from its measured pose, lowered and released. The punch is lifted 60 mm, turned level, lowered onto the V rest and released. Both hands back off.

The demo never writes an object pose, applies a force or attaches a tool to the gripper. It stops early only if a tool has really left the grasp (more than 15 mm or 30° of slip for three controls, or no pad contact for six).

## Fidelity and limits

The anchor is full size and copies the DIAB37's catalogue dimensions; its internals are modelled as listed under Hardware, not copied from the maker's design. The threaded upper bore is a smooth 9.5 mm bore. The sleeve is four rigid petals on root hinges that follow an elastic, perfectly plastic law from A36-class steel values (7850 kg/m³, 200 GPa, 250 MPa yield): 250 N·m/rad stiffness and 1.25 N·m yield moment. The plastic rest angle changes only in response to measured strain, so the sleeve stays spread after the punch is lifted; nothing expands the sleeve on an event. Concrete is rigid, with no cracking or spalling. Friction (steel on steel 0.25 static, 0.20 kinetic; steel on concrete 0.50, 0.40) is estimated. The force needed to set the anchor is not calibrated against the real part and is far lower than it: the simulated anchor sets with one light blow, where the real one takes several hammer hits. No load rating is implied. Masses and inertias come from the solid volumes and densities.
