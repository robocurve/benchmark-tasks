# Lift the scrap sheet out of the laser cutter with both hands, leaving the cut parts

Unloading a laser cutter, the way a fab lab or a sign shop does it many times a day. The laser has cut four parts out
of a clear acrylic sheet, and they sit loose in their holes on the machine's honeycomb bed. The scrap sheet has to
come off first, straight up and level: lifted by its near edge first, it pivots on its far edge, which slides toward
the hands, and the walls of the holes drag the parts along. Then the sheet goes over the machine's side into the
scrap bin, and the parts stay on the bed for the next step.

| File | What it is |
|---|---|
| `laser_scrap_sheet_lift.py` | The `yam_arena` task file: scene, scoring (`success`, `score`) and the scripted `demo()` |
| `layout.py` | Every size and place in the scene, shared by the task file and the Blender script |
| `cutter.py` | The laser cutter's sizes, collider boxes and lid geometry (the same file is in the open-laser-cutter-lid task) |
| `plate.py` | `Plate`: a sheet with holes as one rigid body of box colliders. `Prism`: a part with a convex outline. `collider()`: box pieces as any box's collider (the open lid's, pitched). `surface()`: another friction for a box |
| `bonds.py` | Hooks a callback on every task-file box as it spawns (`on_spawn`), which `plate.py` and `looks.py` use |
| `wrist.py` | `PinchWaypoint`: a grip that points forward with its jaws closing up and down, for the demo |
| `looks.py` | Puts a detailed mesh on a box at spawn and hides the plain box, which keeps its bounds |
| `looks/` | The meshes (`*.usdc`), their textures, `make_looks.py` (the Blender script) and `cutter_looks.py` (the machine's modelling, shared with the open-laser-cutter-lid task) |
| `test_laser_scrap_sheet_lift.py` | Offline checks: the file loads, the sheet's collider and the play round every part, the machine's clearances, the tilt trap's geometry, every scoring outcome, the demo's pinch, clearances and reach (no GPU) |
| `task.json` | The task's details for the board |

## The laser cutter

A desktop CO2 laser cutter, 52 cm wide, 40 cm deep and 22 cm tall, with no brand: a charcoal body whose front wall is
7 cm tall, side walls with vent slots, a back block the full height (the laser tube, the control panel with a screen,
buttons and an emergency stop, and a round exhaust duct out of the back), an aluminium honeycomb bed inside with its
top 14.5 cm up, and the laser head on its gantry parked at the back. Its lid, a steel hood with a tinted window and a
front flap, stands open at its 80 degree stop on two gas struts. Every part is a box collider (the open lid's are
turned with it), so an arm cannot pass through the machine.

The low front wall is what lets a hand pinch a sheet on the bed: with the jaws closing up and down the grippers point
forward, and the lower jaw and the gripper body reach 5.5 cm below the pads, which clears the 7 cm wall with the bed
at 14.5 cm. The machine stands right of centre with its left side on the robot's centre line, the bin 1 cm left of it,
so the carry is 27.7 cm: the right arm ends 4.7 cm past the centre line.

## The sheet and the parts

The sheet is clear acrylic, 22 x 20 cm and 5 mm thick (219 g, 1190 kg/m3). `plate.py` keeps its spawned box for its
bounds (so the scorer reads the sheet's centre) and replaces its collider with box pieces, all shapes of the same
rigid body: a ring of 6 mm boxes along every side of each hole, so the walls of the holes are exact, and boxes merged
from a 2.5 mm grid over the rest of the sheet. The parts are separate bodies of the same acrylic, each in its own hole
with 0.5 mm of play all round:

| Part | Size | Where | Mass | Collider |
|---|---|---|---|---|
| disc | 6 cm across | 1.2 cm from the far edge | 16.8 g | 32-sided prism (the hole is 24-sided, 0.5 mm outside the circle) |
| rect_wide | 3 x 5 cm, long side along y | 1.2 cm from the far edge | 8.9 g | box |
| rect_long | 5 x 3 cm, long side along x | middle | 8.9 g | box |
| triangle | equilateral, 4 cm sides | middle | 4.1 g | triangular prism |

Acrylic slides on acrylic and on the bed with friction 0.4. Lifted level, the sheet rises off the parts, which stay
put. Lifted by its near edge with the far edge on the bed, it pivots there, and the far edge slides toward the hands
by L (1 - cos a), 2 cm at 25 degrees. The far holes' walls hold the far parts until they have risen past the parts'
5 mm tops, which at 1.2 cm from the far edge takes 24.6 degrees of tilt, so a lift tilted 15 degrees or more drags
the far parts by more than the 6 mm a part may move (20 degrees measured, below). Slid along the bed, the walls of the holes push every part.

## Assets

| Object | Source |
|---|---|
| Laser cutter, open lid, bin, sheet, parts | Task-file boxes (the sheet's, the parts' and the open lid's colliders as above), drawn by meshes modelled in Blender (`looks/make_looks.py`, `looks/cutter_looks.py`): the machine's body with its bed, gantry, head, control panel and duct (`cutter_body.usdc`), the open lid with its struts (`cutter_lid_open.usdc`), the black plastic bin with a rolled rim, the clear, light blue tinted sheet with the four cut-outs, and the parts with orange masking paper on top |

SimReady has no laser cutter, cut acrylic sheet or scrap bin of the right size. The sheet's and the parts' meshes fit
inside their boxes (the script checks it on export). To change the meshes, edit the scripts and run Blender 4.2 or
later: `blender -b -P looks/make_looks.py`.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=laser_scrap_sheet_lift.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lift the clear scrap sheet straight up off the laser cutter's bed with both hands so the four orange cut parts stay on the bed, then put the sheet in the black scrap bin." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/laser-scrap-sheet-lift-dream

# Scripted demo (both arms), 600 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=laser_scrap_sheet_lift.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lift the clear scrap sheet straight up off the laser cutter's bed with both hands so the four orange cut parts stay on the bed, then put the sheet in the black scrap bin." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/laser-scrap-sheet-lift-demo
```

- The run prints `task_score` at the end. With `-E object_jitter=0` the layout is fixed and no seed is needed.
- The demo: both grippers point forward along x with their jaws closing up and down (`wrist.PinchWaypoint`) and come
  in over the front wall at the sheet's height, 8 cm either side of its centre, until the pads are 2.3 cm into the
  overhang, one under the sheet and one on it. They close together, lift it 5 cm level in 2 s, raise it over the
  machine's side wall (the grippers 28 cm up), carry it 27.7 cm left over the bin in 3.5 s, turn their fingers 30
  degrees down so the sheet's far edge dips into the bin, let go and back out. It reaches 1.0 and ends on success at
  step 346 (17.3 s at 20 Hz). A trial takes about 2 minutes on an L40S, including startup.
- Shortcuts, each the same demo with one constant changed (`scripts/task-ablate.sh`):
  - A tilted lift (`LIFT_TILT = 0.35`, `LIFT = 0.075`: the near edge up 7.5 cm while the far edge stays on the bed,
    20 degrees) scores 0.5: the sheet pivots on its far edge, which slides toward the hands and drags
    the disc and the 3 x 5 cm rectangle more than 6 mm along the bed, and the sheet still ends in the bin.
  - One hand only (`HANDS = 1`, the left arm stays at its ready pose) also scores 1.0: one pinch at a near corner
    holds the stiff 219 g sheet level, as a person's hand would, so it lifts as cleanly as two. The instruction asks
    for both hands, the way a person unloads a laser, but the score counts only where the sheet and the parts end up.
    A sheet that sags when held by one corner would need a flexible or much larger sheet, beyond the arms' reach.
- Videos: `inspect-robots video ~/yam-eval/laser-scrap-sheet-lift-demo/<log>.json` writes one MP4 per camera next to
  the frames.
- Offline checks: `python -m pytest -q test_laser_scrap_sheet_lift.py`.

## Scoring

All positions are centres, in metres, in the station frame (x forward, y to the robot's left, from the midpoint of
the arm bases), and heights are above the table top.

- A part is **left on the bed** when its centre is within 6 mm of where it was cut, measured along the bed, at most
  3 mm higher and at most 5 mm lower, and it lies flat (tilted 10 degrees or less). A part carried off with the
  sheet, dropped in the bin, dragged or flipped up on an edge is not.
- The sheet is **in the bin** when its centre is over the bin's inside (28.8 x 22.2 cm around (0.425, 0.127)) and
  below its rim (10 cm up).
- **0** if the sheet fell off the table (centre more than 2 cm below the top) or the state is not finite.
- **1.0:** the sheet in the bin and all four parts left. The episode ends in success once the sheet has also been still
  (under 1 cm/s and 0.5 rad/s) for 1 s.
- **0.75:** the sheet in the bin and three parts left. **0.5:** two parts left. **0:** one or none.
- **0.25:** the sheet off the bed (its centre not over the bed, or 2 cm or more above where it lay) but not in the bin,
  with three or more parts left: held up, dropped on the table, or lying across the bin's rim.
- **0** otherwise, including doing nothing and a sheet slid along the bed (it drags its parts with it).

In a real trial, a ruler from marks on the bed gives how far each part moved, and a look gives where the sheet is.
