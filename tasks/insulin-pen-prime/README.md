# Do an insulin pen's 2-unit safety shot with the needle up, then dial a 12-unit dose by its window

A fluid-filled training insulin pen with a pen needle on lies in its tray. Its instructions for use (Lilly's Humalog KwikPen IFU, https://uspl.lilly.com/humalog/humalog.html#section-18) start every injection the same way: pull off the outer and inner needle shields, turn the dose knob to 2 units, hold the pen with the needle pointing up, push the knob in until it stops and 0 shows in the dose window, hold it in while counting to 5, and "You should see insulin at the tip of the Needle". Then the dose is dialled by the window and corrected by turning the knob either way. The robot does this and dials 12 units: one hand picks the pen up, stands it needle up and holds it; the other pulls the shield, dials, pushes the knob home from below and holds it, and dials the dose; the pen goes back in its tray.

**What this task adds.** It is a sequence on one device, each step through the pen's own mechanism: a shield drawn off along the needle, a knob that clicks once a unit and screws out as it is dialled, read through a window that shows even numbers and a line for each odd one, and a push along the pen's axis from below that counts the units back to 0 and must be held there. The dial is turned in short strokes with a regrip between them, so a stroke that comes up a unit short or over must be seen in the window and corrected. The safety shot only counts with the needle up, and the dose only counts after it.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 3000 \
  --instruction "Get the insulin pen ready for a 12-unit dose: take off the needle shield and put it in its tray; do the 2-unit safety shot: needle up, push the dose knob in until it stops and hold it there 5 s so a drop shows at the needle tip; then set 12 units by the dose window and leave the pen in its tray."
```

## Scripted demo

```bash
python demokit.py task.py OUT --mode full                                                  # the full-score reference, filmed
python demokit.py task.py OUT --no-record --mode noshot,short_hold,over,shieldon,shieldloose,holdend --hold 2.5 # failure checks, below
python check_score.py                                                                       # the scoring table on hand-built records, no simulator
```

The demo is a scripted reference: it reads the pen's pose and the window's count from the simulator, and a policy gets only the cameras, in which the window's number shows. The left hand picks the pen at its middle from above, lifts it and turns it a quarter turn so the needle points up and the window faces the right arm, and holds it 15.5 cm up. The right hand, fingers level, pinches the shield and draws it off upward, then drops it in the shield tray. The left hand lifts the pen to 22.5 cm. The right hand turns the knob in strokes of up to 50 degrees about the pen's axis, letting go and coming back between strokes, until the window reads 2. With its fingers closed it comes up under the knob's end face and pushes the knob home, then holds it there 5.5 s; the left hand holds the pen's place against the push. Then the right hand dials 12 by the window, the left hand turns the pen back down and lays it in its tray, and both hands withdraw.

**Result** (scripted, GPU physics on an NVIDIA GeForce RTX 4070 Ti SUPER, 20 Hz, filmed at 640 x 360): 1.0. The shield in its tray; 2 dialled; the knob pushed home against a constant 9.2 N with the needle within 13 degrees of up, the window counting down to 0, the drop shown at the needle tip, held 5 s; 12 dialled by the window (one stroke over to 13, then back); the pen in its tray. 1324 steps (66.2 s of simulated time). No seed changes the scene.

**Model run** (Dream-1, once, on the revised task and instruction, 12 rows per request, seed 1): 0.0 in 3,000 steps (150 s of simulated time), one session of 248 requests, none refused. The policy lifted the pen from its tray, up to 29 cm, and turned it about, but never brought the needle within 30 degrees of up, never took the shield off and never turned the dose knob. At 97 s it set the pen down on the table outside its tray, which scores 0. The film is `model_run.mp4`. (On the first version of this task, the provider refused Dream-1's request 104 as unsafe, at 63.4 s.)

| Failure check | What it does | Score |
|---|---|---|
| `noshot` | dials 2, then goes straight on to 12 without the safety shot | 0.2 in 1149 steps: the shield in its tray; 12 set and the pen in its tray, but the dose counts only after a safety shot with a drop |
| `short_hold` | pushes the knob home but holds it 2 s, not 5 | 0.2 in 1287 steps: the count went to 0 needle up and the drop showed, but held 2 s the shot does not count, so neither does the dose |
| `over` | does the safety shot, then leaves the dose at 13 | 0.6 in 1319 steps: the shield and the safety shot with its drop; the window reads 13, not 12 |
| `shieldon` | leaves the needle shield on and does the safety shot and the dose as in the demo | 0.0 in 1150 steps: with the shield on, nothing flows out of the needle, so no drop shows and the shot does not count; the shield is not in its tray |
| `shieldloose` | draws the shield 4 mm and lets go, leaving it on the needle, then does the shot and the dose | 0.0 in 1318 steps: the shield was not off the needle at the shot, so no drop counted, and it never reached its tray |
| `holdend` | as the demo, but the left hand, letting go of the pen in its tray, moves straight to its knob end and pinches it | 0.6 in 1322 steps: everything done, but a hand stays within 5 cm of the pen, so the 1.0 is never reached |

## Building it for a real trial

- **Pen:** a fluid-filled training pen of the KwikPen kind (0 to 60 units, 1 unit a click; a demonstration pen whose cartridge holds water, refilled through its cartridge holder when low), 17 mm across the body, dose knob 18 mm across, with a 4 mm pen needle and its outer shield on (166 mm long in all), about 20 g. Take each new needle's inner shield off before the trial (the IFU: "Pull off the Inner Needle Shield and throw it away"); with it on, no drop could show. The safety shot must push a visible drop out of the needle, as on a real pen. KwikPen and Humalog are named for identification only; this task is not affiliated with Lilly.
- **Trays:** a white plastic pen tray 50 x 210 mm with 3 mm rails, centred at (24, 12) cm, the pen along y with the needle toward -y and the window facing down; a blue plastic shield tray 70 x 70 mm with 16 mm walls at (30, -11) cm. Both fixed to the table.

| Value | In the simulator | How to measure it on the real pen |
|---|---|---|
| Knob turn and rise per unit | 18 degrees, 0.33 mm | Dial 20 units: count the turns and measure the knob's rise with calipers |
| Click hold | 0.04 N m | A torque screwdriver bit on the knob |
| Push to deliver | a constant 9.2 N over the stroke: a standard KwikPen's plateau injection force at 6 U/s, 9.17 N, its maximum 11.95 N (van der Burg 2011, https://pmc.ncbi.nlm.nih.gov/articles/PMC3045231/); a Junior KwikPen's maximum is 9.34 N at 4.6 U/s (Kappes et al. 2018, https://pmc.ncbi.nlm.nih.gov/articles/PMC5851229/). The YAM's nominal payload is 2 kg, about 20 N (https://www.roboticscenter.ai/store/product/i2rt-yam-6dof/); in the demo the arm pushes the knob home from below against the constant 9.2 N and holds it 5 s | **Not measured on the training pen.** A kitchen scale under the knob's end while pushing it home slowly |
| Shield pull-off | released once a pinch draws it 3 mm | A spring scale on the shield |
| Digits in the window | 3.6 mm tall | Calipers |

## Safety for a real trial

Use the fluid-filled training pen only. Fit a fresh pen needle for each trial, its inner shield off and its outer shield on; never put a shield back on a used needle by hand, and drop used needles in a sharps container. During the push the bare needle points up and away from people. Keep people clear of the arms' reach and of the needle while the robot runs, and keep an emergency stop in reach.

## Scoring

See `scoring` in `task.json`, and `Pen`, `_tier`, `score` and `success` in `task.py`. The scorer reads the knob's count, the pen's pose, the shield's place and the fingertips. The safety shot counts once the shield is off the needle (its middle 3 cm or more from the tip) and the count, dialled to exactly 2, is pushed to 0 against the pen's 9.2 N with a fingertip under the knob's end face, the needle within 30 degrees of up on every step of the push and the hold, a drop shows at the needle tip, and the knob is held home 5 s; the window counts down as the knob goes in. The dose counts once the counter is at exactly 12 at the end, the pen lying in its tray, and every fingertip of both hands is more than 5 cm from points along the whole pen. The drop is the flow check: in the simulator it is shown once the 2 units are pushed out with the needle up and the shield off the needle (no liquid is simulated); pushed with the needle down, or with the shield on, there is no drop and the shot does not count. A push is a fingertip under the knob's end face moving the knob in; the count then follows the knob down, as the pen delivers. The 2 s hold applies only to the full 1.0: a run meeting every part scores 0.6 until it has held 2 s, and lower totals count at once.

A real trial is read from video and a photo: the right wrist and top cameras show the window at 2 before the push, the needle within 30 degrees of up, and the knob held home 5 s; a close camera on the needle shows the drop at its tip (no drop, no dose credited); the window is read at 12 on the right wrist camera before the pen is laid down; a photo at the end shows the pen in its tray and the shield in its tray. The drop is about 3 mm across: in the simulator it is drawn at its real size and read by the scorer, and at 320 x 176 it is a pixel or two in the station cameras. Between trials, turn the knob back to 0 without pushing (nothing delivered) and fit a fresh needle, its inner shield off and its outer shield on.

## Assets

All geometry is built in code from cylinders and boxes; the window's digits are seven-segment bars on the number sleeve. No third-party assets.

## Licence

MIT
