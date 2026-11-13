# Close the push-to-open cabinet doors that are open, without pressing the closed ones

Three identical cabinet units stand side by side, each with a handleless push-to-open flap door. One or two doors,
drawn at random each trial, stand ajar; the others are latched shut. The robot must close the open doors and leave
the closed ones alone: a push-to-open latch pops a closed door open when it is pressed.

**Instruction:** `Close any cabinet door that is open. Do not press the doors that are already closed.`

## Why this task

Handleless cabinets with push-to-open latches are common in kitchens, bathrooms and bedrooms. The latch toggles:
pressing an open door shut latches it, and pressing a latched door pops it open. A robot that "makes sure" by pressing
every door undoes the doors that were already shut. The task tests acting only where action is needed, with the
instruction stated as a negation, and checking each door's state before touching it. Which doors are open changes
every trial, so the choice cannot be memorised.

## Design

| Object | Size | Mass | Notes |
|---|---|---|---|
| Cabinet units (3) | 20 cm wide, 30 cm deep, 25 cm tall, at y 20, 0 and -20 cm | heavy (25 kg in simulation) | White melamine-faced chipboard; they do not slide when a door is pressed |
| Flap doors (3) | 18 cm wide, 22 cm tall, 1.8 cm thick; hinged along the bottom edge 2 cm above the table; outer face at x 34 cm | 0.6 kg | White, handleless, push-to-open latch |
| Open doors | one or two, drawn at random (die) | | Tilted 6 degrees open: 2.3 cm at the top |

Positions are from the midpoint between the two arm bases (x forward, y left, z up).

- **The latch:** pressed 1.2 degrees past shut (4.6 mm at the top), an open door latches; a latched door pops open to 20
  degrees. It acts again only after the door has come back under 0.6 degrees (the press is released), as a real
  push-push latch needs a release between presses.
- **Why flaps:** the doors are hinged at the bottom (drop-front flaps) instead of at the side. The latch, the press and
  the toggle are the same; the simulation kit's hinges turn about a horizontal axis.

## Scoring

Evaluated every control step; the score at the end of the episode counts.

| Condition | Score |
|---|---|
| A door that started closed was ever popped open | 0 |
| All the doors that started open are latched shut (success, held 1 s) | 1.0 |
| Two doors started open and one of them is latched shut | 0.5 |
| Otherwise | 0 |

In a real trial: a latched door is flush and stays shut when let go; a popped door swings open, which the video shows.

Unintended strategies and what stops them:
- Pressing every door: the closed ones pop open (0).
- Doing nothing: 0.
- Memorising which doors to press: the open doors are drawn at random each trial.
- Pressing a door shut and then again: the second press pops it open, and it no longer counts as latched.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Close any cabinet door that is open. Do not press the doors that are already closed." \
  --scorer task_score \
  --max-steps 1200 --store-frames --log-dir ~/yam-eval/push-latch-cabinet-doors-demo
```

Each open door is pressed shut with the closed fingertips, pointing forward and 30 degrees down, 16 cm above the
hinge. The press is commanded 4 cm into the door; the arm's compliance and the door's stop limit the actual press.
The left arm closes the left door, the right arm the right door; the middle door goes to whichever arm has no door of
its own. The arms take turns: the left arm closes its door and moves away, then the right arm closes its own. The
demo presses only the doors that stand open in that trial (`AJAR=0,1` fixes the draw: 0 left, 1 middle, 2 right;
unset, one or two are drawn at random).

**Press-every-door contrast (expected score 0):**

```bash
AJAR=0,2 DEMO_ALL=1 inspect-robots run ...   # same arguments as above
```

The right arm presses the closed middle door first; it pops open.

`CAB_DEBUG=1` prints the door angles and latch states every 0.25 s.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: cabinets, doors, the latch emulation, `score()` / `success()`, and `demo()` |
| `jointkit.py` | Rigid parts made of boxes with one spring-driven hinge each, and per-physics-step and reset hooks (hooks yam_arena without modifying it) |
| `test_push_latch_cabinet_doors.py` | The task file loads, the latch thresholds, the demo presses only the open doors for every draw, the arms take turns, the demo's reach |
| `task.json` | Submission details |

## Runtime and seeds

A demo run takes about 3 minutes of wall time on an RTX workstation. Run limit: 1,200 steps (1 minute at 20 Hz).
Pass `-E object_jitter=0`. The open doors are drawn when the task file loads, from the operating system's random
source; set `AJAR` to fix them.

## Tests

```bash
python -m pytest -q      # in this folder, in the yam_arena environment; no GPU needed
```

## Verified in simulation

Isaac Lab-Arena (Isaac Lab 3.0, Isaac Sim 6.1), with the commands above.

| Run | `task_score` | What the log and video show |
|---|---|---|
| Scripted demo, left and middle doors open (`AJAR=0,1`) | **1.0** | The left arm presses the left door shut and moves away; the right arm presses the middle door shut. The right door is never touched. Success at step 253 |
| Scripted demo, middle and right doors open (`AJAR=1,2`) | **1.0** | The left arm closes the middle door, then the right arm the right door. Success at step 252 |
| `AJAR=0,2 DEMO_ALL=1` | **0** | The right arm presses the closed middle door, which pops open |
| Dream-1 (left and middle doors open, drawn at random) | **0** | 1,200 steps (step limit). Both arms draw back toward their bases and stay there; no door is touched |
| `python -m pytest -q` | 5 passed | |

## Real-world build

- **Cabinets:** three small flat-pack cabinet units (or one carcass with three compartments) with drop-front doors on
  hinges along the bottom edge, and one push-to-open latch per door, set so a press of about 5 mm at the top latches
  or releases it. Weigh the units down so they do not slide.
- **Setting open doors:** a 2.3 cm spacer at the top edge sets the 6 degree gap; remove it before the trial.
- **Safety:** the doors are light and the press force small; a popped door swings out by about 7 cm.

## Still approximate or unverified

- The latch is emulated from the door's angle, not modelled as a mechanism.
- Bottom-hinged flaps stand in for the more common side-hinged doors.
- One Dream-1 run (1,200 steps, left and middle doors open) scored 0: neither arm reached a door.
