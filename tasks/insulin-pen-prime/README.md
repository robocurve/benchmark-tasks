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

**Result** (scripted, GPU physics on an NVIDIA GeForce RTX 4070 Ti SUPER, 20 Hz, filmed at 640 x 360): 1.0. The shield drawn off the needle and into its tray; 2 dialled; the knob pushed home against a constant 9.2 N with the needle within 14 degrees of up, the window counting down to 0, the drop shown at the needle tip, held 5 s; 12 dialled by the window in six strokes (the first left it at 0, then 2, 4, 7, 9, 12); the pen in its tray. 1207 steps (60.35 s of simulated time). No seed changes the scene.

**Model run** (Dream-1, 12 rows per request, seed 1, on this task as submitted): 0.0 in 3,000 steps (150 s of simulated time), one session, none refused. The policy lifted the pen from its tray, up to 25 cm, and turned it about, but never brought the needle within 30 degrees of up, never took the shield off and never turned the dose knob; it laid the pen back in its tray. The film is `model_run.mp4`.

| Failure check | What it does | Score |
|---|---|---|
| `noshot` | dials 2, then goes straight on to 12 without the safety shot | 0.2 in 992 steps: the shield in its tray; 12 set and the pen in its tray, but the dose counts only after a safety shot with a drop |
| `short_hold` | pushes the knob home but holds it 2 s, not 5 | 0.2 in 1328 steps: the count went to 0 needle up and the drop showed, but held 1.8 s the shot does not count, so neither does the dose |
| `over` | does the safety shot, then leaves the dose at 13 | 0.6 in 1332 steps: the shield and the safety shot with its drop; the window reads 13, not 12 |
| `shieldon` | leaves the needle shield on and does the safety shot and the dose as in the demo | 0.0 in 1123 steps: with the shield on, nothing flows out of the needle, so no drop shows and the shot does not count; the shield is not in its tray |
| `shieldloose` | draws the shield 4 mm and lets go, leaving it on the needle, then does the shot and the dose | 0.0 in 1295 steps: the shield stayed on the needle (it comes off only once drawn 16 mm), so no drop showed at the shot, and it never reached its tray |
| `holdend` | as the demo, but the left hand, letting go of the pen in its tray, moves straight to its knob end and pinches it | 0.6 in 1190 steps: everything done, but a hand stays within 5 cm of the pen, so the 1.0 is never reached |

## Building it for a real trial

- **Pen:** a fluid-filled training pen of the KwikPen kind (0 to 60 units, 1 unit a click; a demonstration pen whose cartridge holds water, refilled through its cartridge holder when low), 17 mm across the body, dose knob 18 mm across, with a 4 mm pen needle and its outer shield on (166 mm long in all), about 20 g. Take each new needle's inner shield off before the trial (the IFU: "Pull off the Inner Needle Shield and throw it away"); with it on, no drop could show. The safety shot must push a visible drop out of the needle, as on a real pen. KwikPen and Humalog are named for identification only; this task is not affiliated with Lilly.
- **Trays:** a white plastic pen tray 50 x 210 mm with 3 mm rails, centred at (24, 12) cm, the pen along y with the needle toward -y and the window facing down; a blue plastic shield tray 70 x 70 mm with 16 mm walls at (30, -11) cm. Both fixed to the table.

| Value | In the simulator | How to measure it on the real pen |
|---|---|---|
| Knob turn and rise per unit | 18 degrees, 0.33 mm | Dial 20 units: count the turns and measure the knob's rise with calipers |
| Click hold | 0.04 N m | A torque screwdriver bit on the knob |
| Push to deliver | a constant 9.2 N over the stroke: a standard KwikPen's plateau injection force at 6 U/s, 9.17 N, its maximum 11.95 N (van der Burg 2011, https://pmc.ncbi.nlm.nih.gov/articles/PMC3045231/); a Junior KwikPen's maximum is 9.34 N at 4.6 U/s (Kappes et al. 2018, https://pmc.ncbi.nlm.nih.gov/articles/PMC5851229/). In the demo the arm pushes the knob home from below against the constant 9.2 N and holds it 5 s | The published measurement above; before the trials the training pen's own push is read with a force gauge, and the pen is used only if it needs 9.2 N or less: see "The push: its measured force, and the arm pushing against it" below |
| Shield pull-off | held on the hub and needle, sliding along the axis with a pinch, until drawn 16 mm (its open end past the needle's tip); let go part way, it stays on the needle | A spring scale on the shield |
| Digits in the window | 3.6 mm tall | Calipers |

### The push: its measured force, and the arm pushing against it

**The pen's push resistance is a published measurement.** van der Burg (2011, https://pmc.ncbi.nlm.nih.gov/articles/PMC3045231/) measured a standard KwikPen's injection force at 6 units a second: a plateau of 9.17 N, at most 11.95 N; a Junior KwikPen's maximum is 9.34 N at 4.6 units a second (Kappes et al. 2018, https://pmc.ncbi.nlm.nih.gov/articles/PMC5851229/). The simulated pen meets the plateau, a constant 9.2 N, over the whole stroke, and the arm pushes the knob home against it and holds it 5 s.

**The real trial reads its own training pen with a force gauge before the trials.** A pen of this kind is, for example, the Wallcur Practi-Insulin Pen Trainer (1520PN: 0 to 60 units, a dose window, filled with distilled water; named for identification only, and this task is not affiliated with its maker); check its size against the table above. Clamp the pen needle up in a soft-jawed vice, with a new needle on, its inner shield off and water in the cartridge. Dial 2 units. Push the dose knob's end straight along the pen's axis with a digital push-pull force gauge (50 N range, 0.05 N resolution or better, flat tip), at about 1 unit a second, as the demo does, until the knob stops, and keep pushing 5 s. Record the peak force and the force while held. Do this five times, and once over a 12-unit stroke, and report the readings with the trial results. A trial uses the pen only if every peak is 9.2 N or less, the force the arm is shown pushing against below; a harder pen is reported and not used.

**What the YAM was shown to do, in this task's set-up.** In the simulator, with the simulator's joint torque limits for the YAM (27 N m on joints 1 to 3, its DM4340 motors; 7 N m on joints 4 to 6, the DM4310's peak), the demo's right arm pushes the knob home from below and holds it 5 s against 9.2 N, the run filmed above. The demo logs both arms' joint torques, gravity included, just before the push and while the knob is held home (`[pen] torques` in its output).

| Pen's force | Result | Right arm's joint torques halfway through the hold (joints 1 to 6, N m) |
|---|---|---|
| 0 (just before the push) | | -0.19, -6.71, 3.35, 1.22, 0.01, 0.07 |
| 9.2 N (the task) | pushed home, held 5.35 s, the shot counted; 1.0 in 1207 steps (the film) | 1.26, -9.9, 7.89, 2.97, -0.83, -0.07 |

The push adds about 0.35 N m per newton at joint 2, 0.49 at joint 3 and 0.19 at joint 4 (the 9.2 N row less the row before the push). At the published maximum, 11.95 N, that gives about 10.9 N m at joint 2 and 9.2 N m at joint 3, 40 and 34 percent of the 27 N m limit on those joints, and 3.5 N m at joint 4, above the DM4310's rated 3 N m and within its 7 N m peak, for the 5 s hold. At 20 N joint 4 would need about 5.0 N m, still within its peak. So the motors have the torque for the whole published range. The real trial's pen is limited to 9.2 N, the force the push is shown to complete. The motors are from i2rt's YAM configuration (https://github.com/i2rt-robotics/i2rt/blob/main/i2rt/robots/config/yam_v1.yml: DM4340 on joints 1 to 3, DM4310 on joints 4 to 6). The DM4310's ratings are from DAMIAO's DM-J4310-2EC manual (https://gitee.com/kit-miao/DM-J4310-2EC: rated 3 N m, peak 7 N m). The 27 N m on joints 1 to 3 is the simulator's limit for the DM4340 (joint 2 holds 9.9 N m at 9.2 N, 6.7 N m of it the arm's own weight).

## Safety for a real trial

Use the fluid-filled training pen only. Fit a fresh pen needle for each trial, its inner shield off and its outer shield on; never put a shield back on a used needle by hand, and drop used needles in a sharps container. During the push the bare needle points up and away from people. Keep people clear of the arms' reach and of the needle while the robot runs, and keep an emergency stop in reach.

## Scoring

See `scoring` in `task.json`, and `Pen`, `_tier`, `score` and `success` in `task.py`. The scorer reads the knob's count, the pen's pose, the shield's place and the fingertips. The safety shot counts once the shield is off the needle (its middle 3 cm or more from the tip) and the count, dialled to exactly 2, is pushed to 0 against the pen's 9.2 N with a fingertip under the knob's end face, the needle within 30 degrees of up on every step of the push and the hold, a drop shows at the needle tip, and the knob is held home 5 s; the window counts down as the knob goes in. The dose counts once the counter is at exactly 12 at the end, the pen lying in its tray, and every fingertip of both hands is more than 5 cm from points along the whole pen. The drop is the flow check: in the simulator it is shown once the 2 units are pushed out with the needle up and the shield off the needle (no liquid is simulated); pushed with the needle down, or with the shield on, there is no drop and the shot does not count. A push is a fingertip under the knob's end face moving the knob in; the count then follows the knob down, as the pen delivers. The 2 s hold applies only to the full 1.0: a run meeting every part scores 0.6 until it has held 2 s, and lower totals count at once.

A real trial is read from video and a photo: the right wrist and top cameras show the window at 2 before the push, the needle within 30 degrees of up, and the knob held home 5 s; a close camera on the needle shows the drop at its tip (no drop, no dose credited); the window is read at 12 on the right wrist camera before the pen is laid down; a photo at the end shows the pen in its tray and the shield in its tray. The drop is about 3 mm across: in the simulator it is drawn at its real size and read by the scorer, and at 320 x 176 it is a pixel or two in the station cameras. Between trials, turn the knob back to 0 without pushing (nothing delivered) and fit a fresh needle, its inner shield off and its outer shield on.

## Assets

All geometry is built in code from cylinders and boxes; the window's digits are seven-segment bars on the number sleeve. No third-party assets.

## Licence

MIT
