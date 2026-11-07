# Hold the LED flush while the soldering iron touches its pad

Through-hole soldering at an electronics bench, the way a repair technician or a maker tacks a part in place. A loose
LED backs out of its holes and sits crooked unless it is held, so one hand presses it flush against the board from the
components side while the other touches the soldering iron's tip to its pad on the solder side, holds it there long
enough to melt the solder, and then puts the iron back in its stand. The iron is a hot tool next to a hand: its tip
must not touch the LED, the other parts' joints, the table or a gripper. Real trials run with the iron cold.

| File | What it is |
|---|---|
| `hold_led_solder_pad.py` | The `yam_arena` task file: scene, scoring (`success`, `score`) and the scripted `demo()` |
| `bonds.py` | `Sliding`: a task-file box on a PhysX prismatic joint (here the LED's legs in their holes, breakable), and the `on_step`, `on_reset` and `on_spawn` hooks |
| `compound.py` | A task-file box that collides as several smaller boxes in one rigid body: the iron's handle, barrel and tip, the LED's body |
| `fingers.py` | Where the grippers' fingers are: points on their collision meshes, from the robot's `tip_left` and `tip_right` bodies |
| `looks.py` | Puts a detailed mesh on a box at spawn and hides the plain box, which still collides |
| `looks/` | The meshes (`*.usdc`) and their textures, and `make_looks.py`, the Blender script that models and exports them |
| `test_hold_led_solder_pad.py` | Offline checks: the file loads, the layout, every scoring outcome, the 2 s timer and its memory, the failure rules, the one-handed shortcut, the demo's order and reach (no GPU) |
| `task.json` | The task's details for the board |

## The scene

Every part is a `yam_arena` box. No assets are downloaded: the SimReady collection has no soldering iron, iron stand,
circuit board or LED, so the parts are modelled in Blender and drawn on the boxes.

The board, its holder and the iron's stand are turned 45 degrees: the board's components side faces forward and to the
robot's left, its solder side back and to the robot's right, and the iron lies in its stand pointing the same way as
the components side faces. So the left arm presses the LED from the far side of the board and the right arm brings the
iron to the near side, each in its own half, without reaching round the board.

| Part | Collider | Physics |
|---|---|---|
| board | one 8 x 0.16 x 6 cm box, fixed | clamped upright in the holder |
| holder | a 12 x 7 x 1.2 cm base and two 10 x 0.8 x 2.2 cm jaws, fixed | |
| LED | one box over its body and legs (5.8 x 13.2 x 5.8 mm, 5 g, free), colliding as its body only (`compound.py`) | a prismatic joint along its legs (`bonds.py`): free to slide from flush to 12 mm out, a little damping, breaks at 8 N or 0.05 N m. Starts 3 mm out |
| iron | one 2 x 22 x 2 cm box (60 g, free), colliding as a 2 cm handle, an 8 mm barrel and a 4 mm tip | its centre of mass in the handle, 4.5 cm behind the box's centre |
| stand | a 7 x 19 x 1.2 cm base, a square tube for the spiral (2.4 cm inside, 3 mm walls, 11 cm long), a rest with two lips under the handle, fixed | |

| Box | Mesh |
|---|---|
| `led` | the red epoxy body with its dome and flange, and its two legs |
| `iron` | the blue rubber grip with its rings, the steel barrel and nut, the tinned conical tip |
| `board` | both sides textured (solder mask, traces, pads, outlines), the resistor, capacitor, pin header and chip, and their solder joints |
| `holder_base` | the anodised base and the aluminium jaws with their thumb screws |
| `stand_base` | the cast base, the chrome spiral, the handle's rest and a damp sponge in its tray |

The LED's and the iron's meshes fit inside their boxes (a test checks it), so their centres, which the scorer reads,
are their boxes' centres. To change the meshes, edit `looks/make_looks.py` and run it in Blender 4.2 or later:
`blender -b -P looks/make_looks.py`.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 1,200 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=hold_led_solder_pad.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Press the red LED flush against the green circuit board and keep holding it with one hand while you touch the tip of the blue-handled soldering iron to one of the LED's two pads on the back of the board for two seconds with the other hand. Then put the iron back in its stand, tip first into the spiral." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/hold-led-solder-pad-dream

# Scripted demo (both arms), 700 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=hold_led_solder_pad.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Press the red LED flush against the green circuit board and keep holding it with one hand while you touch the tip of the blue-handled soldering iron to one of the LED's two pads on the back of the board for two seconds with the other hand. Then put the iron back in its stand, tip first into the spiral." \
  --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/hold-led-solder-pad-demo
```

- The run prints `task_score` at the end. With `-E object_jitter=0` the layout is fixed and no seed is needed.
  Keep it at 0: jitter would move the LED off its joint.
- The demo. Left arm, fingers closed and turned to close along the LED's axis: it comes down on the components side
  3 cm off the LED, then the outer face of its right finger pushes the LED the 3 mm back in, flush, and leans on it
  until the iron has left the pad. Right arm, gripper turned across the iron: it grips the handle over the iron's
  centre of mass, lifts it 11 mm, pulls it 12.5 cm back out of the spiral along its length, lifts it over the spiral,
  carries it to the solder side, sets the tip's end on the near pad and holds it there 3 s, then goes back the same way
  and slides it into the spiral and onto the rest. The iron keeps the same turn the whole way. The wrist is sent 4
  degrees off (`YAW_TRIM`) because it lags that much about the vertical, and the aim is raised 1.7 mm (`AIM`) for the
  held iron's sag. The tip's point stays within 1 mm of a pad's centre through the hold, the contact counts from
  about step 252, and the demo reaches 1.0 and ends on success at step 509 (25.5 s at 20 Hz). A trial takes about 2
  minutes on an L40S with the window open (`-E headless=false`), including startup.
- The one-handed shortcut scores 0: the same demo with the left arm staying back (`HOLD_LED = False`) touches the pad
  with the LED still 3 mm out, and scores 0 to the end of the trial.
- Videos: `inspect-robots video ~/yam-eval/hold-led-solder-pad-demo/<log>.json` writes one MP4 per camera next to the
  frames.
- Offline checks: `python -m pytest -q test_hold_led_solder_pad.py`.

## Scoring

All positions are in metres, in the station frame (x forward, y to the robot's left, from the midpoint of the arm
bases), and heights are above the table top. The iron's tip point (the centre of its tip's end) and the LED's body are
worked out from the iron's and the LED's centres and turns, so they follow the parts as they move. The pads, the other
parts and the stand are fixed.

- **Tip on a pad:** the tip's point within 2 mm of the centre of either of the LED's two pads.
- **LED flush:** the LED within 1 mm of flush against the board, along its legs (it starts 3 mm out).
- **LED held:** a point of a gripper finger within 4 mm of the LED's body. The fingers are points on the robot's
  finger collision meshes (`fingers.py`), 2 mm apart.
- **Soldered:** the tip on a pad, the LED flush and the LED held, at every control step for 2 s without a break (40
  steps at 20 Hz). The scorer remembers it for the rest of the trial.
- **Failed** (remembered: the trial scores 0 from then on), at any time:
  - the tip's point within 2 mm of the LED's body on the components side (the iron touched the LED),
  - the tip's point within 1 cm of another part's solder joint on the solder side, or of another part's body (the
    no-touch zone round the resistor, the capacitor, the header and the chip),
  - the tip's point within 2 mm of the table top,
  - the tip's point within 3 mm of a gripper finger,
  - the LED out of its holes: pulled 10 mm or more out of flush (its joint then lets go and it drops), 2 mm or more off
    its holes' axis, or turned 10 degrees or more (a knock of 8 N or 0.05 N m breaks its joint).
- **1.0 (success, held 1 s):** soldered, then the iron lies still (under 1 cm/s and 0.5 rad/s) in its stand: the tip's
  point inside the spiral (within 1.2 cm of its axis sideways and up or down, between its two ends) and the handle's underside within 1.5 cm of
  the rest's top.
- **0.5:** soldered, the iron not lying still in its stand.
- **0.25:** the tip touched a pad while the LED was held flush, but not for 2 s without a break.
- **0** otherwise, including doing nothing, touching the pad without holding the LED flush, or breaking a failure rule.

In a real trial, with the iron cold, a stopwatch and a video show the 2 s of contact and the hand on the LED, a feeler
gauge or a photo from the side shows the LED flush, and the failure rules and the iron's place in its stand are judged
from the video and a photo.
