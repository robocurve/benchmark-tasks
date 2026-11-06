# Plug the multimeter's test leads into COM and V ohm, not the 10 A jack

Setting up a handheld multimeter to measure voltage, the first thing an electronics technician does at the bench:
the black lead goes into COM and the red one into V ohm. The 4 mm banana pins go into 4.3 mm jacks 2 cm apart, the
colour decides the jack, and the jack beside COM is the 10 A current input. A lead left in it is the wrong set-up
for a voltage measurement, the classic way to blow a meter's fuse or short what is being measured.

| File | What it is |
|---|---|
| `multimeter_test_leads.py` | The `yam_arena` task file: scene, the meter's colliders (`meter_boxes`), scoring (`success`, `score`) and the scripted `demo()` |
| `leads.py` | `Lead`: a test lead as one rigid body, a plug box with box colliders for the pin and the stub of cable |
| `simready.py` | Loads the multimeter, an NVIDIA SimReady prop (CC-BY 4.0), pinned to one dataset revision. Its `boxes` option replaces the mesh's collider with boxes |
| `bonds.py` | Hooks a callback on every task-file box as it spawns (`on_spawn`), which `leads.py` and `looks.py` use |
| `looks.py` | Puts a detailed mesh on a box at spawn and hides the plain box, which keeps its bounds |
| `looks/` | The meshes (`*.usdc`) and their textures, and `make_looks.py`, the Blender script that models and exports them |
| `wrist.py` | `TiltWaypoint`: demo waypoints whose grip tilts forward, for the pin-down grip |
| `fingers.py` | Points on the gripper's collision meshes, for the test that the demo's gripper clears the supply and the meter |
| `test_multimeter_test_leads.py` | Offline checks: the file loads, every bore is open and chamfered, the boxes stay in the mesh, every scoring outcome with the meter moved and turned, the demo's reach and clearances (no GPU) |
| `task.json` | The task's details for the board |

## The meter and its jacks

The meter is SimReady's `testingTool_multimeter_c01`. Its mesh shows three jacks on the face at the jack end (A,
COM and V ohm, 2.04 cm apart), but they are 9 mm deep dimples and its collider (a signed distance field of the mesh)
has no usable holes. `simready.py`'s `boxes` option turns the mesh's collider off and gives the meter's one rigid
body 43 invisible box colliders instead, inside the mesh's bounds (`meter_boxes()` in the task file, a test checks
them): the body up to the face (36.9 mm above its back), and at each jack a square bore 4.3 mm wide (0.15 mm of play each side of the pin) and 20 mm deep
whose mouth is chamfered 1 mm at 45 degrees on all four sides (four square bars turned 45 degrees, a pyramid funnel).
The plug body (1 cm across) sits on the face around the bore, so the pin goes 18 mm in, short of the bore's floor.

A lead (`leads.py`) is one rigid body: the plug body is the task-file box (4 x 1 x 1 cm), and box colliders add a
4 mm square pin 18 mm long (its last 2 mm narrowed to 3 mm, a blunt point) and a 5 mm square stub of cable 10 cm
long. The pin slides on the jack at friction 0.3, everything else grips at 1.0.

The bores are a real 4 mm jack's 4.3 mm, so the 4 mm pin has 0.15 mm of play each side. What it took to get pins in
at that fit, measured in the simulator:

- A 4.6 mm bore and a 4.3 mm bore both take the pin when it arrives within about 0.5 mm of the axis and upright.
- The arm, holding the pin down, droops: the plug arrives tilted about 2.4 degrees across and 1.3 degrees along x.
  Pushed in at that tilt, a pin held firmly wedges at the mouth and the plug turns in the fingers. The demo sends the
  grip turned that much the other way (`TRIM_ROLL`, `TRIM_PITCH`), which leaves under 1.5 degrees.
- Held with 25 N, a pin that lands 0.2 to 0.5 mm off the axis sits on the chamfer and stops 3 mm in: the stiff grip
  and arm don't let it slide over. Held with 5 N, it pauses there for about 0.3 s, the plug lines itself up in the
  fingers, and it slides in. So the demo carries each lead with 25 N (with 15 N one plug turned 10 degrees in the
  fingers on the way) and eases the grip to 5 N before pushing it in, as a person holds a plug loosely to find a jack.
- A pointed pin made of two crossed chisel-shaped boxes didn't help (the union of two chisels is still full width at
  the end), so the pin keeps a stepped 3 mm tip.

## Assets

| Object | Source |
|---|---|
| multimeter | SimReady `testingTool_multimeter_c01` (CC-BY 4.0), revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, 17.7 x 8.5 x 4.4 cm, 120 g, free. Downloaded on first use into `~/.cache/simready`. Box colliders replace its mesh collider (above) |
| power supply | A fixed task-file box, drawn by a mesh modelled in Blender (`looks/make_looks.py`): a dark grey case with a recessed front panel, two green displays, two knobs, a power switch and three brass output posts |
| black and red leads | `leads.py` bodies, drawn by meshes modelled in Blender: a nickel pin with a pointed tip, a shrouded plug body with grip rings and a strain relief, and a silicone cable stub |
| mat | A fixed task-file box, blue-grey |

SimReady has no test leads and no bench power supply. The leads' meshes stay inside their colliders' bounds and the
supply's inside its box (a test and the export check it). To change the meshes, edit `looks/make_looks.py` and run
it in Blender 4.2 or later: `blender -b -P looks/make_looks.py`.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=multimeter_test_leads.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Plug the black test lead into the COM jack and the red test lead into the V ohm jack of the yellow multimeter." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/multimeter-test-leads-dream

# Scripted demo (right arm), 800 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=multimeter_test_leads.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Plug the black test lead into the COM jack and the red test lead into the V ohm jack of the yellow multimeter." \
  --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/multimeter-test-leads-demo
```

- The run prints `task_score` at the end. With `-E object_jitter=0` the layout is fixed and no seed is needed.
- The layout: everything is in front of the right arm (y < 0), and the left arm stays at its ready pose. The meter
  lies on the supply so its jacks face up at 13.7 cm: a pin-down grip (the gripper pointing forward, along +x)
  reaches them there, and not at table height. The jack row runs along y, so the open fingers (closing along y) stay
  clear of the plug already in COM.
- The demo, right arm only: for each lead, a straight-down grip across the plug body 1 cm from its back, the pad tips
  1 mm above the mat, closed to a 25 N squeeze. It lifts the lead, tilts the grip forward 90 degrees (the pin turns
  from pointing at the robot to pointing down), goes over the jack, eases the grip to 5 N, lowers the pin in and on
  down 4 mm past seated (the fingers slide down the body), opens the fingers 2 mm past the body and backs off. Black
  into COM first, then red into V ohm: the fingers, opened 2 mm, stay clear of the black plug beside it. It reaches
  1.0 and ends on success at step 648 (32.4 s at 20 Hz). A trial takes about 90 s on an L40S, including startup.
- The trap, the same demo with the black lead plugged into the A jack instead of COM (`scripts/task-ablate.sh
  multimeter-test-leads trap BLACK_JACK='"a"'`), scores 0: the red lead fully in V ohm earns nothing while a lead
  is in A.
- Videos: `inspect-robots video ~/yam-eval/multimeter-test-leads-demo/<log>.json` writes one MP4 per camera next to
  the frames.
- Offline checks: `python -m pytest -q test_multimeter_test_leads.py`.
