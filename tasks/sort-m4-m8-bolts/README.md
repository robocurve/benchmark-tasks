# Sort loose M8 and M4 bolts into a two-compartment organizer

Eight loose hex bolts lie on a cutting mat: four M8 x 30 (bright zinc, silver) and four M4 x 20 (black oxide). The
robot puts the M8 bolts in the left compartment of a blue parts organizer and the M4 bolts in the right one. Each bolt
is a few millimetres thick and lies flat, so it has to be pinched across its shank from above with the fingertips, and
each one goes to a compartment chosen by its size.

| File | What it is |
|---|---|
| `sort_m4_m8_bolts.py` | The `yam_arena` task file: scene, scoring (`success`, `score`) and the scripted `demo()` |
| `bolts.py` | A hex bolt as one rigid body of two box colliders (head and shank), with damped spin |
| `bonds.py` | Hooks into yam_arena's spawning, used by `bolts.py` and `looks.py` (only `on_spawn` is used here) |
| `looks.py` | Puts the render-only Blender meshes on the task file's boxes |
| `looks/make_looks.py` | Models the bolts, the organizer and the mat in Blender and exports `looks/*.usdc` |
| `test_sort_m4_m8_bolts.py` | Offline checks: every scoring outcome, the looks inside their boxes, the demo's reach |
| `task.json` | The task's details for the board |

## Scene

Positions are centres in metres from the midpoint between the two arm bases: x forward, y to the robot's left.
Heights are above the table top. Left and right are the robot's.

- **Cutting mat:** green self-healing mat, 22 x 30 x 0.2 cm, 1 cm grid, at (0.355, -0.20), fixed.
- **Bolts:** ISO 4017 hex bolts lying on a flat of the head, their axes at the angles below (degrees from +x,
  counterclockwise, head to tip):
  - M8 x 30, bright zinc plated (silver), 13 mm hex, 35.3 mm long overall, 15 g: at (0.29, -0.15) 30 deg,
    (0.28, -0.27) 80 deg, (0.36, -0.245) -35 deg and (0.41, -0.21) 50 deg.
  - M4 x 20, black oxide (black), 7 mm hex, 22.8 mm long overall, 3 g: at (0.30, -0.22) -60 deg, (0.35, -0.17)
    10 deg, (0.40, -0.14) 70 deg and (0.34, -0.29) -15 deg.
- **Organizer:** blue polypropylene, 9.8 x 19.2 x 3 cm outside, two 9 x 9 cm compartments side by side along y
  (4 mm walls and divider, 3 mm floor), centred at (0.18, -0.02), fixed. Left compartment centre (0.18, 0.027), right
  compartment centre (0.18, -0.067).

## Physics and looks

- Each bolt is one rigid body with two box colliders: the head (k x s x s) and the shank (l x d x d), so it rests
  tilted on its head and shank tip like a real bolt, and cannot roll. Contacts start 2 mm out, spin is damped at
  2 /s and the body is solved with 16 position iterations, so the light parts settle still.
- The bolts, the organizer and the mat wear meshes modelled in Blender (`looks/make_looks.py`): threaded hex bolts
  (1.25 and 0.7 mm pitch, chamfered heads), a moulded organizer and a gridded mat. The meshes are render-only: the
  task file's boxes still collide and give the scorer its bounds, and each bolt's mesh is inside its box (tested).
- No SimReady package fits. The SimReady bolt and screw packages (`factory_bolt_m04_loose`, `factory_bolt_m8_loose`,
  `screw_high_res`, `hardware_screw_*`) have no root USD at revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, and
  the Factory bolts are short socket-head cap screws, not M4 x 20 and M8 x 30 hex bolts.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 3,000 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=sort_m4_m8_bolts.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the four large silver M8 bolts in the left compartment of the blue organizer and the four small black M4 bolts in the right compartment." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/sort-bolts-dream

# Scripted demo (right arm), 1,300 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=sort_m4_m8_bolts.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the four large silver M8 bolts in the left compartment of the blue organizer and the four small black M4 bolts in the right compartment." \
  --scorer task_score --max-steps 1300 --store-frames --log-dir ~/yam-eval/sort-bolts-demo
```

- The run prints `task_score` at the end. With `-E object_jitter=0` the layout is fixed and no seed is needed.
- The demo (right arm) reaches 1.0 and ends on success at step 1091 (54.6 s at 20 Hz). A trial takes about
  3.5 minutes on an L40S, including startup.
- Videos: `inspect-robots video ~/yam-eval/sort-bolts-demo/<log>.json` writes one MP4 per camera next to the frames.
- Offline checks: `python -m pytest -q test_sort_m4_m8_bolts.py`. To rebuild the looks:
  `blender -b -P looks/make_looks.py`.

The demo pinches each bolt across its shank from above (the M4's 4 mm shank with the pad tips 1.2 mm above the mat),
with the fingers sent only 1 mm past each side of the shank, so they squeeze it with about 5 N instead of the
gripper's full 50 N. It lets each bolt go low inside its compartment, on its own spot of a 2 x 2 grid.

## Scoring

All positions are centres, in metres, in the station frame, and heights are above the table top. The score is read
at the end of the run (the run ends early once success has held for 1 s).

- A bolt is **in a compartment** when its centre is over that compartment's 9 x 9 cm floor (the inside of its walls)
  and less than 3 cm above the table (below the walls' top). A bolt held above the organizer, or lying across a wall
  or the divider, is in neither.
- Each M8 bolt in the left compartment and each M4 bolt in the right one, lying still (under 1 cm/s and 0.5 rad/s),
  adds **1/8**.
- Each bolt in the other compartment takes **1/8** off, so tipping all eight bolts into one compartment, or sorting
  them the wrong way round, scores 0. The score is never below 0.
- **Success (1.0, held 1 s):** all eight bolts lying still in their own compartments.
- **0** if any bolt has fallen off the table (its centre more than 2 cm below the table top), or the state is not
  finite.

Shortcut check: the scripted demo with every bolt taken to the left compartment (`DEMO_TO = {"m8": "left", "m4":
"left"}` in the task file) ends with all eight bolts lying in that compartment and scores 0.0.
