# Hold the acrylic sheet down and peel its protective film off by the tab

Peeling the masking film off a cut acrylic sheet, as every laser-cutting shop and makerspace does many times a day.
The film peels when its edge is lifted, strip by strip, while the hand comes back over the sheet: dragged along the
sheet, the tab tears off before the glue lets go, and a torn tab scores 0. The sheet is light and loose, so the first
pull tips it up with the film unless the other hand holds it down, and a lifted sheet scores 0. One hand holds the
sheet, the other peels the film and drops it in a basket.

| File | What it is |
|---|---|
| `peel_acrylic_film.py` | The `yam_arena` task file: scene, scoring (`success`, `score`) and the scripted `demo()` |
| `bonds.py` | PhysX joints between task-file boxes, authored when they spawn: `Joint` (fixed, revolute or ball, breakable), `tune` (solver settings), and step and reset hooks |
| `simready.py` | Loads the SimReady basket, pinned to one dataset revision |
| `looks.py` | Puts a detailed mesh on a box at spawn and hides the plain box, which still collides |
| `looks/` | The meshes (`*.usdc`) and their textures, and `make_looks.py`, the Blender script that models and exports them |
| `test_peel_acrylic_film.py` | Offline checks: the file loads, the glue, hinges and tab, every scoring outcome, the demo's order, arc, arm clearance and reach, and the meshes inside their boxes (no GPU) |
| `task.json` | The task's details for the board |

## The film

The film is a chain of four rigid paper strips, 3.75 x 10 cm and 1.5 mm thick, on the sheet's right half. `bonds.py`
authors these PhysX joints when the boxes spawn:

| Part | Joint | Behaviour |
|---|---|---|
| glue | a fixed joint between each strip (its first body) and the sheet, under the strip's edge toward the robot, `physics:breakTorque` 0.06 N m, `physics:breakForce` 8 N | pulled up at its far edge, a strip twists its glue past 0.06 N m and comes unstuck. Pulled along the sheet, it holds to 8 N |
| joints between strips | a ball joint (a PhysX D6 joint with its translations locked) at the middle of each cut, free, lightly damped | the peeled strips swing, fold and twist like paper, and the peeled part pulls up the next strip's edge. No turning passes from one strip to the next, so only a pull up at a strip's edge breaks its glue |
| fold | a ball joint between the tab and the first strip, with a weak spring that keeps the tab upright, `physics:breakForce` 4 N | the tab turns on its fold. Pulled with 4 N, it tears off |

How hard the peel pulls was measured in the simulator, with the scripted demo and the left hand lifted 6 cm off the
sheet: with the glue at 0.02 N m the strips came off before the loose sheet moved, at 0.04 N m the sheet tipped up 62
degrees, at 0.06 N m it tipped 14 degrees and slid 2.7 cm. With the left hand pressing the near left corner (pad tips
sent 3 cm into the sheet), the sheet did not move at 0.06 N m. Revolute hinges between the strips let a hand that
twists the tab break the glue (the first versions came off without the sheet ever tipping), hence the ball joints.

A step hook records each strip that has come unstuck (its centre 1 cm or more from its place, measured on the sheet)
a torn tab (its centre 2 cm or more from its fold on the first strip), and a lifted or dragged sheet (its centre more
than 2 cm from its start, or tilted more than 10 degrees).

## What it looks like

Each box keeps its collider, and `looks.py` draws a mesh in its place:

| Box | Mesh |
|---|---|
| `mat` | a green self-healing cutting mat with its 1 cm grid, 5 cm lines, 45 degree guides and knife scores |
| `sheet` | glossy opal white acrylic |
| `strip_1` to `strip_4` | brown kraft masking paper, printed "PROTECTIVE FILM" across the four strips as one piece |
| `tab` | the same paper, with a printed arrow |

The basket is the SimReady `misc_basket_plastic_a01` (revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, CC-BY 4.0),
scaled to 0.6, fixed, colliding through its own signed distance field. `simready.py` downloads it into
`~/.cache/simready` the first time the scene is built. The scored meshes fit inside their boxes (a test checks it).
To change them, edit `looks/make_looks.py` and run it in Blender 4.2 or later: `blender -b -P looks/make_looks.py`.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=peel_acrylic_film.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the white acrylic sheet down with your left hand. With your right hand, pinch the brown paper tab on the far edge of the film and peel the film off by lifting it up and back toward you, then drop the film in the blue basket on the right." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/peel-acrylic-film-dream

# Scripted demo (both arms), 600 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=peel_acrylic_film.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the white acrylic sheet down with your left hand. With your right hand, pinch the brown paper tab on the far edge of the film and peel the film off by lifting it up and back toward you, then drop the film in the blue basket on the right." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/peel-acrylic-film-demo
```

- The run prints `task_score` at the end. With `-E object_jitter=0` the layout is fixed and no seed is needed.
  Keep it at 0: jitter would move the strips away from their glue and joints.
- Each run is one episode: a glue joint or tab that breaks stays broken until the scene is built again.
- The layout keeps the arms apart: the film is on the sheet's right half, so the right arm peels on the right while
  the left arm holds the bare left half by its near corner.
- The demo: the left arm, fingers closed, presses the bare acrylic 2 cm from the near left corner (the pad tips are sent
  3 cm below its top, so it leans on it) and holds it until the film is off. The right arm, turned so its fingers
  close across the tab, pinches the tab's top 1.4 cm (the pad tips 11 mm above the film), then peels in one 6 s
  stroke up and back toward the robot, from the far edge to 19.5 cm above the near edge: the strips come up one after another and the film leans up between
  the hand and the peel line. It carries the hanging film over the basket, turning the hand so the fingers open along
  the basket's long side, lowers the film onto the basket's floor and lets go. It reaches 1.0 and ends on success at
  step 348 (17.4 s at 20 Hz). A trial takes about 2.5 minutes on an L40S, including startup.
- The same demo with the left hand stopping 6 cm above the sheet (`PRESS_DEPTH = -0.06`) scores 0: the first pull
  tips the loose sheet up with the film, and the scorer records it as lifted.
- Dream-1 scored 0.0 in its one run: both hands reach over the sheet and the right one hovers by the tab, but it
  never pinches the tab and no strip comes off.
- Videos: `inspect-robots video ~/yam-eval/peel-acrylic-film-demo/<log>.json` writes one MP4 per camera next to the
  frames.
- Offline checks: `python -m pytest -q test_peel_acrylic_film.py`.

## Scoring

All positions are centres, in metres, in the station frame (x forward, y to the robot's left, from the midpoint of
the arm bases), and heights are above the table top.

- **Peeled:** a strip whose centre has been 1 cm or more from its place on the sheet, measured in the sheet's own frame
  (so a sheet dragged with its film still stuck peels nothing). The scorer remembers: a peeled strip stays peeled,
  even if it falls back onto the sheet.
- **Torn:** the tab's centre 2 cm or more from its fold on the first strip (turning on its fold, it stays 1.25 cm from
  it). The scorer remembers: the trial scores 0 from then on.
- **Lifted:** the sheet's centre more than 2 cm from its start, or the sheet tilted more than 10 degrees, at any control
  step (lifted with the film, or dragged). The scorer remembers: the trial scores 0 from then on.
- **0** if the sheet is off the table or the state is not finite.
- **1.0 (success, held 1 s):** all four strips peeled and the centre of every strip and of the tab within 6 cm along x
  and 8 cm along y of the basket's centre (0.235, -0.265), inside its walls, and lower than its 6.9 cm rim, the film
  still (under 2 cm/s and 1 rad/s).
- **0.5:** all four strips peeled, the film not all in the basket (held, dropped on the table, or hanging over the rim).
- **0.25:** two or three strips peeled.
- **0** otherwise, including doing nothing or peeling one strip.

In a real trial, a tape measure gives the sheet's place and the film's place in the basket, a look at the sheet tells
which strips have come off, and the tab is checked for a tear.
