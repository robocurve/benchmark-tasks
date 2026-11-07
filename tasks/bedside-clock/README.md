# Set the bedside clock to the time on the analog clock

After a power cut, a bedside clock shows 1:25. The instruction is: **"Set the bedside clock to the time on the analog
clock."**

The analog clock runs on a battery, so it kept time through the power cut: it reads 4:40. To set the bedside clock,
the robot has to:
- read the hands: the hour hand is two-thirds of the way from 4 to 5, the minute hand is on 8;
- read the bedside clock's display;
- work out the presses: HOUR 3 times and MIN 15 times;
- keep count while it presses.

Each press adds one hour or one minute, and the buttons only count up. Minutes roll from 59 to 00 without changing the
hour, so going past 4:41 costs nearly a full lap of the minutes.

Resetting clocks after a power cut or a switch between summer and winter time is an everyday chore, including in home
care. Reading an analog clock face is a known weak spot of vision-language models, and here the reading has to drive a
counted physical action. Doing nothing leaves 1:25, which scores 0.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, the monitor (which counts presses and turns the number drums), `success()` and `score()` |
| `geometry.py` | Dimensions and positions, the drums' numbering, the start and target times |
| `build_assets.py` | Writes the textures and `assets/bedside_clock.usda` and `assets/analog_clock.usda` (needs `pxr`, PIL and the DejaVu fonts) |
| `assets/` | The bedside clock (case, two spring-loaded buttons, three number drums), the analog clock on its stand, and their textures |
| `scenekit.py` | USD bodies, articulations with per-joint drives, and reset hooks for yam_arena task files |
| `demokit.py` | Helpers for the scripted demo |
| `demo.py` | The scripted full-score demo, and the misread variant |

`scenekit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer. Tested with yam-isaac-lab-arena at `bd62771`
(Isaac Lab-Arena 0.3.1).

## Run a trial

From this folder, in a shell with the yam_arena environment active. New shells on the walkthrough's Brev instance
have it; elsewhere, `source <ARENA_DIR>/yam.env`.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set the bedside clock to the time on the analog clock." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/clock-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

- **Seeds:** there is one fixed layout and no randomisation (`object_jitter=0`), so the seed has no effect.
- **Runtime:** a full 2,400-step Dream-1 trial took 3 min 0 s on an RTX 5090 (mean inference latency 0.58 s),
  plus about a minute to start Isaac Sim. A successful trial ends earlier: the scripted demo stops after 457 steps.

Dream-1 scored 0: its right arm reached over the bedside clock several times, but neither button moved from its
rest position, so the clock still showed 1:25. The run used one continuous Dream-1 session (50 requests, no retries).

## Scripted demo

```bash
python demo.py task.py demo_out              # full score: writes demo_*_cam.mp4, setup.png, result.json
python demo.py task.py misread_out --misread # reads the hour hand as 5: ends at 5:40, score 0.5
```

1. **Hours.** The left arm closes its gripper and presses HOUR three times with its fingertips, then goes back to its
   start pose with the gripper open. Each press goes 4 mm past the button's travel, dwells 0.15 s and comes back up.
2. **Minutes.** Then the right arm presses MIN fifteen times the same way and goes back. One arm works at a time, so
   the two stay clear of each other over the clock.
3. **Check.** The script reads the clock's state and presses again for any press that did not count. In the recorded
   demo, none was needed.

The demo succeeds (1.0) in 457 steps (23 s). The misread variant presses HOUR four times, ends at 5:40
and scores 0.5.

## Scoring

Everything is judged from a photo of the bedside clock's display at the end, and the video.

| Score | When |
|---|---|
| 1.0 (success) | The bedside clock shows 4:39, 4:40 or 4:41 for 4 s, which ends the trial. Pressing on past 4:41 within those 4 s does not count as success |
| 0.5 | The minutes show 39 to 41 and the hour is one off (3 or 5): the hour hand misread |
| 0 | Anything else. This includes the right hour with wrong minutes (three HOUR presses get the hour without reading the clock), doing nothing (1:25 stays), and the bedside clock knocked over or moved more than 1 cm. In simulation the clock is fixed, so it cannot be moved |

- **A press** counts when a button is pushed in more than 3.5 mm of its 5 mm travel. It must come back to less than
  1.5 mm before the next press counts. These are the clock's own switch points: in the real build the clock counts
  the presses, and in simulation the task does.
- **The tolerance** of one minute allows for reading the minute hand to the nearest minute.
- **Only the display counts.** The instruction does not ask the robot to move away, and holding a button does not
  change the display, so a gripper still on a button at the end does not matter.

## Real-world build

All positions are in cm from the midpoint between the arm bases: x forward, y to the robot's left. Heights are above
the table top. Both clocks are taped down.

- **Bedside clock.** A black ABS case, 22 cm wide, 15 cm deep and 10 cm tall, from x 20 to 35, centred on the
  midline.
  - **Front:** it slopes at 45 degrees from 3 cm up at x 20 to the top at x 27.
  - **Display:** on the slope, a 3.2 cm tall window between two black bezel bars, each 17 cm wide, 2 cm tall and
    standing 8 mm proud. Through it, white-on-black digits about 1.8 cm tall show the time as H:MM, with hours 1 to 12
    and a white colon.
    - The window is centred 3.5 cm back from the case's front edge, at x 23.5, and 6.5 cm up.
    - In simulation the digits are printed on three number drums that roll behind the window. A real build can use
      any white-on-black display there, for example a negative-mode LCD, driven by a microcontroller that reads the
      two buttons.
  - **Buttons:** two grey 2.4 cm square caps on the flat top, standing 8 mm above it, at x 30.5, y 7 and y −7, 14 cm
    apart. HOUR is on the left and MIN on the right, each name printed in white in front of its button. They travel
    5 mm and take about 1 N at the bottom.
  - **Behaviour:** each press advances the display by one. HOUR goes from 12 to 1. MIN goes from 59 to 00 and does
    not change the hour. Holding a button does not repeat.
  - **No timekeeping.** The display changes only when a button is pressed, so every trial starts at 1:25.
- **Analog clock.** A battery analog clock with a 22 cm white face, black numerals 1 to 12, minute ticks and black
  hands, in a dark grey rim 24 cm across and 3.5 cm deep.
  - Take the battery out and set the hands to 4:40, so every trial has the same target.
  - Its centre is at (58, 18), 17 cm up, face toward the robot, leaning back 25 degrees.
  - **Stand:** a wooden easel. A 14 × 16 cm foot, 1.2 cm thick, centred at (60, 18). A 6 cm wide back board leans back
    against the rim's back. A 10 cm wide ledge holds the rim's bottom, about 5 cm above the table.

## Simulation fidelity

- Rigid bodies, 240 Hz physics, 20 Hz control.
- **Buttons:** the two buttons are sliding joints with a 200 N/m spring and 5 mm of travel. The monitor reads them
  after every control step, so a tap shorter than one step (50 ms) can be missed.
- **Drums:** the task code turns the number drums, not a mechanism. After a counted press, the drum rolls on to the
  next number over 2 control steps (0.1 s), as fast as presses can come, so the display never falls behind. The drums
  have no colliders.
- **Analog clock:** a fixed body, with its numerals and hands printed on the face texture.
- **Resets:** before each reset the monitor puts the buttons up and the drums back to 1:25. The reset moves the clock
  only by its root, so trials can run back to back in one process.

## Reset between trials

Set the bedside clock back to 1:25:
- press HOUR until it shows 1;
- press MIN until it shows 25.

Then check that the analog clock still reads 4:40 and that both clocks are on their marks.
