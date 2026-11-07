# Open the lever, push the bare wire into the port, close the lever

Splicing a wire into a lever connector, the way an electrician or a maker wiring a project does it. The connector's
near port is shut by its orange lever: the lever has to be lifted before the wire goes in, the bare copper end has to
go in up to its insulation and no further, and the lever has to be pressed back down to clamp it. Pushed in until it
stops, the wire puts its insulation in the clamp, the real wiring fault, and that scores 0.

| File | What it is |
|---|---|
| `lever_connector_wire.py` | The `yam_arena` task file: scene, scoring (`success`, `score`) and the scripted `demo()` |
| `bonds.py` | `Hinged`: a task-file box on a revolute joint with a spring drive whose target the task moves at run time |
| `shell.py` | Gives a task-file box several box colliders, all one rigid body (the lever's clamp tooth, the wire's thinner bare end) |
| `looks.py` | Puts a detailed mesh on a box at spawn and hides the plain box, which still collides |
| `looks/` | The meshes (`*.usdc`) and their texture, and `make_looks.py`, the Blender script that models and exports them |
| `simready.py` | Loads an NVIDIA SimReady prop, pinned to one revision of the dataset |
| `test_lever_connector_wire.py` | Offline checks: the file loads, the port and the clamp, the lever's swing, every scoring outcome, the demo's order, heights and reach (no GPU) |
| `task.json` | The task's details for the board |

## Scale

The connector and the wires are built at twice the size of a real 2.5 mm2 inline lever connector and its wire, close
to a 10 mm2 lever connector: the loose wire is 7 mm thick with a 4 mm bare end stripped 22 mm, and the port is 11 mm
square. At real size a 2 mm copper end in a 2.5 mm port is below what the parallel gripper and PhysX (5 mm contact
offset) handle. Every length below is the scaled one, and the scoring's depths are twice the real ones.

## The connector

All its parts are boxes. The board and the housing are fixed. Two parts move:

| Part | How it is built | Behaviour |
|---|---|---|
| near lever | a 3.3 x 1.4 x 0.8 cm orange box on a revolute joint across the housing at the port's face, 0 to 60 degrees, with a 6 x 9 mm clamp tooth under it 1.2 to 1.8 cm behind the hinge (one rigid body, `shell.py`), 20 g | bistable: a spring drive aims 60 degrees past the shut stop while the lever is under 30 degrees, and 60 degrees past the open stop above (a step hook flips the target), so it springs shut or open with 0.024 N m at either stop |
| port | an 11 mm square chamber 3 cm deep that opens at the board's surface, with a tinned contact bar on its floor, 1.5 mm tall, from 1.1 to 2.0 cm in, and an open window in its roof under the lever | shut, the clamp's foot is 3.7 mm above the bar: the 4 mm bare end can't pass it. On a copper end lying on the bar, the lever stops about 1 degree open and presses it down |
| loose wire | one 14.2 cm rigid body (`shell.py`): 12 cm of 7 mm square insulation and a 22 mm long, 4 mm square bare end, 25 g | rigid, it does not bend. In the port it lies level, its insulation on the board, its copper on the bar |

The board reaches 9 cm in front of the port, so a wire let go in the port stays level. Everything else is
`yam_arena` boxes, plus the multimeter.

## What it looks like

Each box keeps its collider, and `looks.py` draws a mesh in its place:

| Box | Mesh |
|---|---|
| `board` | the plywood board, the connector's grey housing with its two ports, contact bars, hinge bosses, mounting screws and strip gauge, the far orange lever, the far brown wire and the grey cable it comes from, with its blue and yellow wires capped |
| `lever` | the near orange lever with its lip and grip ridges, and the steel clamp under it |
| `wire` | the brown insulation and the bare copper end, inside the wire's box (a test checks it) |

The multimeter is NVIDIA SimReady `testingTool_multimeter_digital_b01` (CC-BY 4.0), pinned to the dataset's revision
`9d4556bfe808` in `simready.py`, fixed in place. To change the meshes, edit `looks/make_looks.py` and run it in
Blender 4.2 or later: `blender -b -P looks/make_looks.py`.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=lever_connector_wire.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lift the near orange lever of the connector, push the bare end of the loose brown wire into the empty port under it up to its insulation, then press the lever back down." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/lever-connector-wire-dream

# Scripted demo (right arm), 600 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=lever_connector_wire.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lift the near orange lever of the connector, push the bare end of the loose brown wire into the empty port under it up to its insulation, then press the lever back down." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/lever-connector-wire-demo
```

- The run prints `task_score` at the end. With `-E object_jitter=0` the layout is fixed and no seed is needed. Keep
  it at 0: jitter would move the lever off its hinge.
- The first run downloads the multimeter from Hugging Face into `~/.cache/simready`.
- The demo, right arm only, fingers closing along y: it pinches the lever's sides lightly (the pads sent 1.5 mm
  inside them, about 7 N each) 2.4 cm behind the hinge and carries the pinch round the hinge to 42 degrees, the wrist
  tilting forward with the lever (`wrist.py`'s `TiltWaypoint`), then lets go and the spring takes the lever to its
  60-degree stop. Straight down again, it takes the wire 6 cm behind its tip, lines the tip up 1.5 cm in front of the
  port, pushes it in to 21 mm and lets go. It comes in behind the open lever tilted 60 degrees, pinches it again,
  carries it back down to 3 degrees and lets go: the spring shuts it on the copper. The left arm stays at its ready
  pose. It reaches 1.0 and ends on success at step 474 (23.7 s at 20 Hz). A trial takes about 2 minutes on the
  instance, including startup.
- The same demo pushing the wire to 34 mm instead of 21 mm (`PUSH_TO = 0.034`, the wire stops at the chamber's back
  wall) scores 0: the insulation goes 8 mm in, under the clamp, and the lever still shuts on the copper.
- A full 50 N pinch holds the lever so hard that it can't turn between the pads, and a wrist that stays straight down
  stalls the swing, so the demo pinches lightly and tilts the wrist with the lever.
- Offline checks: `python -m pytest -q test_lever_connector_wire.py`.

## Scoring

All positions are in metres in the station frame (x forward, y to the robot's left, from the midpoint of the arm
bases). The port's face is at x = 0.33 and its axis at y = -0.22, 1.55 cm above the table. The depth is how far the
bare end's tip is past the port's face along the port's axis, read from the wire's centre and orientation, and 0 when
the tip is not inside the port's 11 mm square. The lever's angle is its turn about its hinge from shut.

- **0** if the wire has fallen off the table (centre more than 2 cm below the top) or the state is not finite.
- **0** if the tip is 26 mm or more in: 4 mm or more of insulation is in the clamp. Open or shut.
- **1.0 (success, held 1 s):** the tip 18 to 24 mm in (at most 4 mm of copper shows, at most 2 mm of insulation is
  in), the lever within 5 degrees of shut, the wire still (under 1 cm/s and 0.5 rad/s).
- **0.5:** the tip 18 to 24 mm in, the lever more than 5 degrees open.
- **0.25:** the lever more than 5 degrees open and the tip 6 mm or more in, outside 18 to 24 mm.
- **0** otherwise, including doing nothing, opening the lever only, and a lever shut on a wire less than 18 mm in.

In a real trial, a ruler against the port's face gives how much copper shows or how much insulation went in, and a
protractor or a photo from the side gives the lever's angle.
