# Open a heavy laser cutter lid with both hands until its gas struts hold it

The first step of every job on a desktop CO2 laser cutter, in a fab lab or a sign shop. The lid is a steel hood with
a glass window, hinged at the back, and its two gas struts only help once it is open past 30 degrees. Below that the
whole 6 kg hangs on the handle: it takes both hands, one at each end of the handle. Let go too early and it slams
shut on the machine's glass and optics, so a trial in which the lid falls back scores 0.

| File | What it is |
|---|---|
| `open_laser_cutter_lid.py` | The `yam_arena` task file: scene, struts, scoring (`success`, `score`) and the scripted `demo()` |
| `cutter.py` | The laser cutter's sizes, collider boxes and lid geometry, shared with the Blender script (the same file is in the laser-scrap-sheet-lift task) |
| `bonds.py` | `Hinged`: a box on a revolute joint with limits, and hooks into `yam_arena` (`on_spawn`, `on_reset`, `on_step`) |
| `plate.py` | `collider()`: gives a box a collider made of box pieces (the lid's panel, flap and handle). `surface()`: another friction for a box |
| `wrist.py` | `PinchWaypoint`: a grip that points forward and tilts, its jaws closing up and down, for the demo |
| `looks.py` | Puts a detailed mesh on a box at spawn and hides the plain box, which keeps its bounds |
| `looks/` | The meshes (`*.usdc`), their textures, `make_looks.py` (the Blender script) and `cutter_looks.py` (the machine's modelling, shared with the laser-scrap-sheet-lift task) |
| `test_open_laser_cutter_lid.py` | Offline checks: the file loads, the lid's pieces, the struts' balance, the lid angle, every scoring outcome with the trial's memory, the demo's reach and clearances (no GPU) |
| `task.json` | The task's details for the board |

## The machine and the lid

The laser cutter is 52 cm wide, 40 cm deep and 22 cm tall, with no brand: a charcoal body whose front wall is 7 cm
tall, side walls with vent slots, a back block the full height (the laser tube, the control panel with a screen,
buttons and an emergency stop, and a round exhaust duct out of the back), an aluminium honeycomb bed inside, and the
laser head on its gantry parked at the back. Every fixed part is a box collider, so an arm cannot pass through it.

The lid is a hood: a top panel with a tinted window and a 15 cm front flap that closes the front above the low wall,
36 cm from the handle to the hinge. A black handle bar, 16 mm square, runs along the flap's lower edge, from 21 cm
right to 21 cm left of the centre line, 18 mm out from the flap on two posts. The lid is one rigid body of 6 kg
(`LID_MASS`) on a revolute joint across the top of the back block, from shut (0) to its open stop at 80 degrees. Its
collider is five boxes: panel, flap, bar and posts. Its weight pulls it shut with 12.7 N m at 0 degrees, 35 N at the
handle.

The struts are a torque about the hinge, set every control step from the lid's angle and speed: none below 30
degrees (their line passes close to the hinge), rising to 14.3 N m at 38 degrees and staying there, damped by 6 N m s
per rad once they push. They balance the lid's weight at 36.4 degrees: above that they lift it to the open stop by
themselves and hold it there.

Why 6 kg: a probe (the demo's motion with one arm and with two, for several masses) measured that one arm lifts a 2
or 4 kg lid past the struts' balance by itself, but stalls at 12 degrees with a 6 kg lid, which then slips out of the
gripper and slams shut. Both arms lift the 6 kg lid to 44 degrees, lagging the plan by about 5 degrees.

## Assets

| Object | Source |
|---|---|
| Laser cutter body, bed, gantry, head, lid | Task-file boxes (the lid's collider as above), drawn by meshes modelled in Blender (`looks/make_looks.py`, `looks/cutter_looks.py`): `cutter_body.usdc` on the front wall's box, `cutter_lid.usdc` on the lid's box (inside it) |

SimReady has no laser cutter. The textures (honeycomb, vents, screen) are drawn by the script. To change the meshes,
edit the scripts and run Blender 4.2 or later: `blender -b -P looks/make_looks.py`.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 1,200 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=open_laser_cutter_lid.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the laser cutter's lid with both hands on the two ends of its black handle, and keep lifting until the gas struts take over and hold it fully open. Do not let it fall back." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/open-laser-cutter-lid-dream

# Scripted demo (both arms), 600 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=open_laser_cutter_lid.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the laser cutter's lid with both hands on the two ends of its black handle, and keep lifting until the gas struts take over and hold it fully open. Do not let it fall back." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/open-laser-cutter-lid-demo
```

- The run prints `task_score` at the end. With `-E object_jitter=0` the layout is fixed and no seed is needed.
- The layout: the machine is centred on the robot's centre line, its front face 36 cm in front of the arm bases. Each
  arm works on its own end of the handle, 33 cm apart.
- The demo: each gripper points forward with its jaws closing up and down (`wrist.PinchWaypoint`), comes in round the
  bar 4.5 cm inside its post, closes and lifts the lid along its arc in 4 degree steps, turning with it, to 44
  degrees in 5.5 s. It lets go, backs off down and away, and the struts lift the lid on to its stop. It reaches 1.0
  and ends on success at step 268 (13.4 s at 20 Hz). A trial takes about 1.5 minutes on an L40S, including startup.
- Shortcuts, each the same demo with one constant changed (`scripts/task-ablate.sh`):
  - One hand only (`HANDS = 1`, the left arm stays at its ready pose): scores 0: the right arm alone lifts the 6 kg lid to 12 degrees and stalls, the handle slips out of its
    fingers and the lid slams shut.
  - Letting go before the struts take over (`LIFT_TO = 24.0`): scores 0: the lid falls back from 24 degrees and slams shut, which the trial records.
- Videos: `inspect-robots video ~/yam-eval/open-laser-cutter-lid-demo/<log>.json` writes one MP4 per camera next to
  the frames.
- Offline checks: `python -m pytest -q test_open_laser_cutter_lid.py`.

## Scoring

The lid's angle is how far it is open, in degrees, from shut (0) to its open stop (80): its turn about the hinge,
read from its orientation. In a real trial, a protractor or a phone's level app on the lid gives it.

- **0** if, at any time in the trial, the lid fell more than 20 degrees below the highest angle it had reached, or hit
  its closed stop (within 3 degrees of shut) closing faster than 60 degrees per second (a slam). The trial
  remembers both, so lifting the lid again afterwards does not help.
- **1.0:** the lid open 75 degrees or more and still (turning under 3 degrees per second). The episode ends in
  success once that has held for 1 s.
- **0.5:** the lid open 37 degrees or more at the end (past where the struts lift it by themselves).
- **0.25:** open 10 degrees or more at the end.
- **0** otherwise, including doing nothing.
