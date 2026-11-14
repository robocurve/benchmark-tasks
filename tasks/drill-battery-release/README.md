# Squeeze both release buttons and slide the battery off the drill

Taking the battery pack off a cordless drill, the way an e-waste recycler does before the tool goes to the shredder
(lithium packs must come out first). The pack is locked on the handle's foot by two release buttons, one on each
face across the pack, and lets go only while both are pressed at once. A parallel gripper closing across the pack
presses both, the way a person squeezes them between thumb and finger. The pack then slides off along the handle
against the friction of its rails, so the other hand has to hold the drill or it slides along with the pack. The
pack goes in a tray.

| File | What it is |
|---|---|
| `drill_battery_release.py` | The `yam_arena` task file: scene, scoring (`success`, `score`), the lock's step hook and the scripted `demo()` |
| `bonds.py` | `Slotted`: a box that slides on another object against a set friction, with a lock that can be switched off and on. `Sliding`: a box on a sprung prismatic joint (the buttons) |
| `fingers.py` | Where the grippers' fingers are: points on their collision meshes, from the robot's `tip_left` and `tip_right` bodies, for the "let go" check |
| `simready.py` | Loads the NVIDIA SimReady drill (pinned revision), makes its five bodies one, and removes its moulded-on battery |
| `looks.py` | Puts a detailed mesh on a box at spawn and hides the plain box, which still collides |
| `looks/` | The meshes (`battery.usdc`, `tray.usdc`) and `make_looks.py`, the Blender script that models and exports them |
| `test_drill_battery_release.py` | Offline checks: the file loads, the mechanism is specified, every scoring outcome (and the pack held still above the tray's floor), the demo's order, depths, arm spacing and reach (no GPU) |
| `task.json` | The task's details for the board |

## Assets

- **Drill:** NVIDIA SimReady `powerTool_drill_a01`, dataset `nvidia/simready-assets` at revision
  `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd` (CC-BY 4.0), downloaded on first use into `~/.cache/simready`. Its five
  bodies (body, trigger, direction switch, selector, chuck) are merged into one rigid body with a mass of 1.2 kg. Its
  battery is part of the body's mesh: `simready.py` deletes the battery's 15 shells (four point ranges of that mesh)
  when the drill spawns, so they are neither drawn nor collide. The drill collides as a signed distance field of its
  mesh.
- **Battery pack and tray:** modelled in Blender by `looks/make_looks.py` and drawn on the task file's boxes. The
  pack's mesh fits inside its box (a test checks it).

## The mechanism

All joints are PhysX joints authored by `bonds.py` when the boxes spawn.

| Part | Joint | Behaviour |
|---|---|---|
| slide | prismatic joint between the pack and the drill, along the handle | the pack slides off its foot toward the robot. A velocity drive with a 10 N force limit holds it back like friction: a drill that is not held is dragged along |
| lock | fixed joint between the pack and the drill, at the seat | holds the pack on. A step hook switches it off while both buttons are 1.5 mm or more in, and back on if they are let go while the pack is within 2 mm of its seat |
| buttons | two prismatic joints between each button and the pack, across the pack, 3 mm of travel, 1,667 N/m springs (5 N at full travel) | sprung buttons standing 3 mm proud of the pack's faces, flush when pressed in fully |

Once the pack has slid 4 cm it is off the foot: the step hook switches the slide off and the pack is free for the
rest of the trial.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=drill_battery_release.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the drill with the right hand. With the left hand, squeeze both release buttons on the orange battery, slide the battery off the drill's handle and put it in the blue tray." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/drill-battery-release-dream

# Scripted demo (both arms), 600 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=drill_battery_release.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the drill with the right hand. With the left hand, squeeze both release buttons on the orange battery, slide the battery off the drill's handle and put it in the blue tray." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/drill-battery-release-demo
```

- The run prints `task_score` at the end. With `-E object_jitter=0` the layout is fixed and no seed is needed. Keep it
  at 0: jitter would move the pack and its buttons away from their joints.
- The layout keeps each arm in its own half: the drill lies with its handle toward the robot and its chuck to the
  right, so the right arm holds the clutch ring (just behind the chuck) at y = -0.015 and the left arm takes the pack at
  y = 0.056 and carries it to the tray on the left. The two grippers stay 19 cm or more apart.
- The demo: the right arm comes straight down with its fingers closing along x round the drill's clutch ring and
  holds it. The left arm comes down over the pack with its fingers closing across it, which presses both buttons
  (2.4 to 2.6 mm in), pulls 7 cm along the handle toward the robot (the pack comes off its foot after 4 cm), lifts it,
  carries it to the tray, lowers it and lets go. The right arm lets go of the drill once the pack is up. It reaches 1.0
  and ends on success at step 258 (12.9 s at 20 Hz), with every finger at least 3 mm from the pack and the pack flat on
  the tray's floor. The drill moves about 2 mm. A trial takes about 90 s on an L40S,
  including startup.
- The same demo with the right arm staying up instead of holding the drill (`HOLD = False`) scores 0: the pack's
  friction on the handle drags the loose drill along, the left arm lifts the drill with the pack still on it and
  sets both down on the tray, and the pack never slides off the handle.
- Videos: `inspect-robots video ~/yam-eval/drill-battery-release-demo/<log>.json` writes one MP4 per camera next to
  the frames.
- Offline checks: `python -m pytest -q test_drill_battery_release.py`.

## Scoring

All positions are centres, in metres, in the station frame (x forward, y to the robot's left, from the midpoint of
the arm bases), and heights are above the table top. The pack's seat is where its centre sits on the handle's foot,
taken with the drill as it now lies (the drill's start, moved and turned the way the drill has moved and turned).

- **Squeezed:** both buttons 1.5 mm or more into the pack at the same control step. The scorer remembers it for the
  rest of the trial. Every score above 0 needs it.
- **0** if the drill or the pack has fallen off the table (centre more than 2 cm below the top) or the state is not
  finite.
- **Let go:** no point of any gripper finger within 3 mm of the pack or its buttons (the fingers are points on the
  robot's finger collision meshes, `fingers.py`). It is measured every control step, and nothing counts as let go
  before the first step.
- **Placed:** the pack lies on the tray's floor inside its walls. All eight corners of the pack are within the tray's
  inner walls (x within 9.2 cm and y within 6.7 cm of the tray's centre (0.19, 0.25)), and its four lowest corners are
  within 1.5 mm of the floor (6 mm above the table top), so one flat face is down. A pack held up, leaning on a wall,
  or resting on a wall's top is not placed.
- **1.0 (success, held 1 s):** the pack off the drill (its centre 5 cm or more from its seat), let go, placed, and
  still (under 1 cm/s and 0.5 rad/s), and the drill's centre within 10 cm of its start in the plane of the table,
  all of it holding without a break for 1 s (20 control steps). The second is timed in the task file: the step hook
  (`work_the_lock`, every control step) adds the step to `STATE["placed_s"]` while every condition holds and sets it
  back to 0 the moment one fails (a finger back on the pack, the pack moving, sliding off the floor). `success()`
  and `score()` return 1.0 only once that second is complete; `SUCCESS_HOLD_S` is 0, so the run ends on that step.
- **0.5:** the pack off the drill but not let go, placed and still for 1 s (held in the hand, held in the air over
  the tray, set down on the table, leaning, still moving, or placed for less than 1 s), or placed with the drill
  moved 10 cm or more.
- **0.25:** the pack slid 1 cm or more along the handle, away from its seat, but still on the drill.
- **0** otherwise, including doing nothing, squeezing without sliding, pulling the pack without squeezing (the lock
  holds), and dragging the drill along with its pack (the pack hasn't slid on the handle).

In a real trial, a tape measure gives the pack's distance from the handle's foot and the drill's distance from its
start. Placed is checked by hand after the run: the pack lies flat on the tray's floor (a 1.5 mm feeler gauge does not
slide under any of its four bottom corners) and no corner is over or on a wall. Let go and still are read from the
video (30 fps or more): the gripper opens and moves away, leaving at least a 3 mm gap between each pad and the pack,
and the pack does not move for 1 s (30 frames) afterwards. The squeeze is seen in the video (both buttons in at once).

## Revision notes

The first review asked: "Revise the full-score predicate so the pack must be released, resting on the tray floor,
and fully inside the tray walls; retain partial credit for a removed pack that has not been placed successfully."

- Released: every point of every gripper finger at least 3 mm from the pack and its buttons, measured each control
  step from the finger collision meshes (`fingers.py`, `finger_gap`, `work_the_lock`).
- Resting on the tray floor: the pack's four lowest corners within 1.5 mm of the floor, so a flat face is down and a
  pack held up, leaning on a wall or resting on a wall's top is not placed (`in_tray`).
- Fully inside the walls: all eight corners of the pack inside the tray's inner walls (`in_tray`).
- Still for 1 s: the released, placed, still state must hold without a break for 1 s, timed every control step in the
  task file (`tick`, `success`); any lapse restarts the second, so `score()` cannot return 1.0 earlier.
- Partial credit kept: a removed pack that is not placed (held, in the air, on the table, leaning, moving, or placed
  for less than 1 s) scores 0.5; a pack slid 1 cm along the handle scores 0.25.
- Tests: `test_still_above_the_floor_is_not_full_credit` (the reviewers' exploit), `test_in_tray_but_still_gripped_half`,
  `test_full_score_needs_one_second_without_a_break` and `test_no_full_score_while_any_condition_is_missing` (each
  full-success condition missing in turn, held 5 s: never 1.0).

## Sizes

The pack is 7 cm across its buttons, so the open gripper (94 mm between the pads) closes over it and its buttons with
9 mm to spare on each side, and 5.8 cm thick, as thick as the drill, so both rest flat on the table. Its buttons are on
the two faces that point left and right as the drill lies, which a gripper coming from above can squeeze together.
