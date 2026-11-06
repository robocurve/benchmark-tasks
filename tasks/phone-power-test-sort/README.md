# Test each phone's power button and sort the ones that turn on

Triage at an e-waste or phone refurbishing bench: each phone is switched on by holding its power button, the ones that
come on go on to refurbishment (green tray) and the dead ones to recycling (grey tray). Which phone works can't be
seen until it has been tested: all three screens start dark, and only the working phone's screen turns white once its
button has been held for 2.5 s. A phone sorted without that test scores nothing, even in the right tray.

| File | What it is |
|---|---|
| `phone_power_test_sort.py` | The `yam_arena` task file: scene, the hold timer, scoring (`success`, `score`) and the scripted `demo()` |
| `bonds.py` | `Sliding`: a task-file box on a sprung prismatic joint (here each power button, jointed to its phone), and the `on_step`/`on_reset` hooks |
| `looks.py` | Puts a detailed mesh on a box at spawn and hides the plain box, which still collides. `show()` turns a part of a mesh on or off at run time (the lit screen) |
| `looks/` | The meshes (`*.usdc`) and `make_looks.py`, the Blender script that models and exports them |
| `test_phone_power_test_sort.py` | Offline checks: the file loads, the buttons and layout, the hold timer and its memory, every scoring outcome, the demo's order and reach (no GPU) |
| `task.json` | The task's details for the board |

## The buttons and the screens

| Part | How it works |
|---|---|
| power button | an orange 3 x 0.35 x 0.5 cm box on the phone's right edge, on a prismatic joint to the phone across it: 2.5 mm of travel, a 1,200 N/m spring back out (3 N at full travel). The button and its phone don't collide, so it sinks into the phone's edge |
| hold timer | a step hook measures each button's depth in its phone's own frame after every control step. 1.5 mm or more in counts as held. Held for 2.5 s without a break (50 steps at 20 Hz), the phone is tested, for the rest of the trial |
| screen | each phone's mesh has a dark screen and a white glowing one 0.1 mm above it, hidden at reset. When the working phone (`phone_2`) is tested, the hook shows its white screen, and it stays on. The dead phones never light |

Everything that collides is a `yam_arena` box. No assets are downloaded. The NVIDIA SimReady collection has no
smartphone (its only phone is a desk phone) and no tray of this size and colour, so the phones and trays are modelled
in Blender as render-only meshes on the boxes.

## What it looks like

| Box | Mesh |
|---|---|
| `phone_1`, `phone_2`, `phone_3` | a phone with rounded corners: metal frame (graphite, silver, dark blue), black glass front, a dark screen, the white screen (hidden until lit), a punch-hole camera and an earpiece slit |
| `tray_green`, `tray_grey` | the tray, with its walls, a lip and a ribbed floor (the walls' own boxes are hidden) |

The phones' meshes fit inside their boxes (a test checks it), so what the cameras see is what collides and is scored.
To change them, edit `looks/make_looks.py` and run it in Blender 4.2 or later: `blender -b -P looks/make_looks.py`.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 3,600 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=phone_power_test_sort.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold each phone's orange power button for three seconds. Put the phones whose screens light up in the green tray and the others in the grey tray." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ~/yam-eval/phone-power-test-sort-dream

# Scripted demo (right arm), 900 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=phone_power_test_sort.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold each phone's orange power button for three seconds. Put the phones whose screens light up in the green tray and the others in the grey tray." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/phone-power-test-sort-demo
```

- The run prints `task_score` at the end. With `-E object_jitter=0` the layout is fixed and no seed is needed.
  Keep it at 0: jitter would move the phones away from their buttons' joints.
- The layout keeps the arms apart: the phones, the blocks and both trays are in the right arm's half of the table
  (y < -0.02), and the left arm's half is empty.
- The demo uses the right arm alone. It closes the gripper across each phone where the button is, so one pad squeezes
  the button in while the other holds the phone's far edge, the way a person holds a phone and presses its side
  button with one hand. It keeps squeezing for 3 s on the foam block, lifts the phone and carries it, still squeezed,
  to its tray, lowers it to 12 mm above the floor and lets go. Phone 3 goes to the grey tray, phone 2 (lit) to the
  green tray and phone 1 to the grey tray, on top of phone 3. It reaches 1.0 and ends on success at step 666
  (33.3 s at 20 Hz). A trial takes about 2 minutes on an L40S, including startup.
- Shortcuts score what the scoring says. The same demo gripping each phone 4 cm from its centre at the other end
  (`GRIP_DX = -0.04`), off the button, sorts all three phones without testing them: phone 2 stays dark and the trial
  scores 0. The demo that tests all three but sorts as if phone 1 worked (`DEMO_LIT = 0`) puts only phone 3 in its
  right tray and scores 1/3.
- Videos: `inspect-robots video ~/yam-eval/phone-power-test-sort-demo/<log>.json` writes one MP4 per camera next to
  the frames.
- Offline checks: `python -m pytest -q test_phone_power_test_sort.py`.

## Scoring

All positions are centres, in metres, in the station frame (x forward, y to the robot's left, from the midpoint of
the arm bases), and heights are above the table top.

- **Held:** a phone's button pushed 1.5 mm or more into the phone's edge (of its 2.5 mm travel), measured in the
  phone's own frame, so it counts however the phone is held.
- **Tested:** the button held for 2.5 s without a break. The scorer counts control steps and remembers: once tested,
  a phone stays tested, and the working phone's screen stays white.
- **Entered a tray:** the phone's centre over a tray's 18 x 12 cm outline and less than 5 cm above the table. The
  scorer remembers the phones that entered a tray before they were tested: they can no longer count.
- **In its right tray:** the phone's centre within 8.4 cm along x and 5.4 cm along y of the tray's centre (over its
  inner floor) and less than 4 cm above the tray floor, still (under 1 cm/s and 0.5 rad/s). The right tray is the
  green one for the working phone and the grey one for a dead phone. Stacked phones count.
- **Score:** 1/3 for each phone that was tested before it entered a tray and lies in its right tray. **1.0
  (success, held 1 s)** for all three.
- **0** for a phone never tested, tested only after it entered a tray, or in the wrong tray. **0 overall** if any
  phone has left the table (centre more than 2 cm below the table top) or the state is not finite.

In a real trial, a stopwatch times each press, a look at the screen tells lit from dark, and a tape measure (or a
photo) shows where each phone lies.

## Simplifications

- The power button's travel is 2.5 mm, more than the 0.5 to 1 mm of a real phone, so a press by a position-controlled
  gripper can be told apart from a touch.
- The phones lie on foam blocks narrower than they are, so a parallel gripper can close across a phone's edges and
  the button hangs free beside the block.
- The working phone is always the middle one, `phone_2`. Its screen switches straight to white, with no boot sequence.
