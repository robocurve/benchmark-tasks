# Press the slot latch and lift the graphics card out

Pulling a graphics card from a dead mainboard, the way an e-waste recycler or an IT asset disposal technician does it.
The PCIe slot's retention latch locks the card: pulled against the latch, the card does not come out until the latch
breaks, and a broken latch scores 0 for the rest of the trial. The slot's rails let the card out straight up only. One
hand presses the latch down, the other lifts the card out and puts it in a padded tray.

| File | What it is |
|---|---|
| `latch_and_lift_graphics_card.py` | The `yam_arena` task file: scene, scoring (`success`, `score`) and the scripted `demo()` |
| `bonds.py` | `Bonded`: a task-file box held by a bond that breaks at a set pull and twist, released on cue. `Sliding`: a box on a sprung prismatic joint |
| `looks.py` | Puts a detailed mesh on a box at spawn and hides the plain box, which still collides |
| `looks/` | The meshes (`*.usdc`) and their textures, and `make_looks.py`, the Blender script that models and exports them |
| `test_latch_and_lift_graphics_card.py` | Offline checks: the file loads, the trap is specified, every scoring outcome, the demo's order, depths and reach (no GPU) |
| `task.json` | The task's details for the board |

## The lock and the latch

Both are PhysX joints authored by `bonds.py` when the boxes spawn:

| Part | Joint | Behaviour |
|---|---|---|
| lock | fixed joint between the card and the bench, `physics:breakForce` 60 N | holds the card in the slot. Pulled harder than 60 N it breaks, which the scorer sees as the card moving before the latch was pressed |
| latch | prismatic joint along z between the latch and the bench, 4 mm of travel, a 500 N/m spring back to the top (2 N at full travel) | a sprung button. Once it has been 3 mm or more down at any control step, a step hook disables the lock's joint for the rest of the episode |

Everything else is `yam_arena` boxes. No assets are downloaded.

## What it looks like

Each box keeps its collider, and `looks.py` draws a mesh in its place:

| Box | Mesh |
|---|---|
| `card` | a dual-fan graphics card: shroud, fans, fin stack, heat pipes, backplate, gold edge connector, steel bracket with ports |
| `latch` | the orange button, with ridges and a hook toward the card |
| `mainboard` | the textured board, the PCIe slot under the card, the rails, the latch housing, the socket, memory slots and small parts |
| `mat`, `tray_floor` | the anti-static mat, and the tray with its walls and pick-and-pluck foam floor |

The card's and the latch's meshes fit inside their boxes (a test checks it), so what the cameras see is what
collides. The board's parts are drawn only: none is taller than 1.2 cm, and all are at least 1.5 cm from the card and the latch.
To change them, edit `looks/make_looks.py` and run it in Blender 4.2 or later:
`blender -b -P looks/make_looks.py`.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=latch_and_lift_graphics_card.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Press and hold the orange latch with the right hand, then lift the graphics card straight up with the left hand and put it in the tray on the left." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/latch-and-lift-graphics-card-dream

# Scripted demo (both arms), 600 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=latch_and_lift_graphics_card.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Press and hold the orange latch with the right hand, then lift the graphics card straight up with the left hand and put it in the tray on the left." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/latch-and-lift-graphics-card-demo
```

- The run prints `task_score` at the end. With `-E object_jitter=0` the layout is fixed and no seed is needed.
  Keep it at 0: jitter would move the card and the latch away from their joints.
- The layout keeps the arms apart: the slot runs along y, with the latch on the right arm's side and the card reaching
  toward the left arm, so each arm works in its own half of the table.
- The demo: the right arm, fingers closed, comes down on the latch and leans on it (the pad tips are sent 12 mm below its top,
  the button bottoms out at 4 mm). The left arm closes on the top 2 cm of the card above its centre once the latch is
  down, lifts the card straight up out of the slot, turns it a quarter turn while carrying it over the tray, lowers it
  and lets go. It reaches 1.0 and ends on success at
  step 206 (10.3 s at 20 Hz). A trial takes about 50 s on an L40S, including startup.
- The same demo with the right arm stopping above the latch (`PRESS_DEPTH = -0.02`) scores 0: the card stays locked
  in the slot, or the lock breaks and the trial is scored 0 from then on.
- Videos: `inspect-robots video ~/yam-eval/latch-and-lift-graphics-card-demo/<log>.json` writes one MP4 per camera
  next to the frames.
- Offline checks: `python -m pytest -q test_latch_and_lift_graphics_card.py`.

## Scoring

All positions are centres, in metres, in the station frame (x forward, y to the robot's left, from the midpoint of
the arm bases), and heights are above the table top.

- **Pressed:** the latch 3 mm or more below its rest. The scorer remembers: once pressed at any control step, the
  card is free for the rest of the trial, whether or not the latch is still held.
- **Broken:** the card's centre 5 mm or more above its start, or 3 cm or more sideways from it, at a control step
  before the latch had been pressed. The scorer remembers: the trial scores 0 from then on, even if the card is then
  put in the tray.
- **0** if the card has fallen off the table (centre more than 2 cm below the top) or the state is not finite.
- **1.0 (success, held 1 s):** the card's centre within 9 cm along x and 5 cm along y of the tray's centre
  (0.24, 0.26) and less than 7 cm above the tray floor (1 cm up), lying flat or standing in the tray, the card still
  (under 1 cm/s and 0.5 rad/s), and the latch not broken.
- **0.5:** the card out of the slot: its centre 1.2 cm or more above its start (its bottom edge clears the rails) or
  3 cm or more sideways, held in the air or set down anywhere out of the tray.
- **0.25:** the latch pressed and the card raised 5 mm or more, but still between the rails.
- **0** otherwise, including doing nothing, pressing the latch without lifting the card, or pulling the card without
  pressing the latch.

In a real trial, a tape measure gives the card's place in the tray and its height in the slot, and the latch is
checked for damage by hand.
