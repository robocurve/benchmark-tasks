# Send a tote down a roller conveyor to the far station without slamming the end stop

Instruction given to the policy: `Send the tote down the conveyor to the packing station at the far end.`

Files: `task.json`, `coast_tote_to_far_stop.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()`, the
sim control `control_walk()`, and an offline self-test with the reach check), `mechkit.py` (parts and per-step rules),
`README.md`.

## Response to review

**TK-8YTHZ5: "End position of the tote should be unreachable by pushing without momentum".** Changed the layout and
added a scorer gate:

- **The station is now beyond reach.** The farthest any robot point gets along the lane is x 0.791 m. That is a pad
  tip of the right arm, stretched out, at the tote's wall heights; the left arm gets to 0.659 m. In the old layout
  the station line (x 0.80) was only 1 cm past that. Now the station line is at x 0.92, 13 cm beyond reach, and the
  end stop is at x 1.10. A tote pushed by hand with no momentum stops where the hand stops: its rear face is at
  0.79 m or less, which is 13 cm short and scores 0.
- **Reach gate (new latch).** No robot point of either arm may come within 25 cm (plan view) of the station's
  footprint at any step, or the trial scores 0. The robot points are the gripper body, the TCP, both pad tips and
  both finger middles. In the lane, that means no point past x 0.67. So the hand's last contact is at least 25 cm
  before the station, and the tote must still be doing at least 0.31 m/s there to coast in (0.30-0.33 m/s across the
  calibrated drag band). The demo's hand never comes nearer than 38.7 cm (its viewing pose). On a real rig the gate is
  computed from the arms' logged joint positions through the URDF kinematics ("Measuring the latches on a real rig").
- **The gate fires in a control run.** In sim control `control_walk()`, the robot tries to push the tote all the way
  by hand. It walks the tote slowly (no momentum), first with the hand pointing down, then pointing forward along the
  lane, to TCP x 0.72, 3.5 cm inside the arm's reach there. In sim the gate latched when the right pad tip reached
  x 0.673, and the trial scored 0. The tote, held at the arm's reach, ended with its rear at 0.734, 19 cm short of
  the station ("Sim controls").
- **No-slam kept, now with a scored speed limit.** The tote meeting the end stop faster than 0.40 m/s now latches 0
  by itself (new latch). 0.40 m/s is the bottle's tip-over speed when the tote stops dead (below). The bottle falling
  still latches 0 too. In sim the VARIANT "hard" control (a shove to 0.75 m/s) fired this latch at 0.545 m/s, and the
  bottle fell two steps later. The demo meets the stop at 0.26 m/s. On a real rig two light gates at x 105 and 109,
  which stay in place, measure this speed.
- The numbers are in "Reach and distances" below.

**TK-84WE5K (rollers and the tote's deceleration).** This was answered in the previous revision and is kept here:
16 mm zinc-plated steel tube rollers at 40 mm centres on sealed ball bearings (now 25 of them, as the conveyor is
longer), and a three-light-gate coast-down that measures and sets the loaded tote's deceleration to 0.196 m/s^2
+/- 10 % ("Real-rig calibration").

## Nearby tasks

Unlike #1322 (strike a ball at the angle that sends another past a line) and #720 (pull a peel out from under a
pizza), the crux here is the release speed: too slow stops short, too fast meets the end stop above 0.40 m/s and tips
the load.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/coast-tote-to-far-stop/coast_tote_to_far_stop.py
```

It asserts the scoring rules on hand-built states, replays every arm's demo plan and the `control_walk()` plan
through the kinematics at 20 Hz (every joint inside the URDF limits, IK error under 0.5 mm), and checks how near
each plan's robot points come to the station (the demo must stay clear of the 25 cm gate by over 10 cm; the control
must cross it). Add `--reach` to recompute the arms' reach along the lane (`max_reach()`, needs SciPy, about a minute):
it prints `right arm 0.791 m, left arm 0.659 m`. Expected output:

```
   0.00  tote rear stopped at x 0.175  | start
   1.00  tote rear stopped at x 0.960  | demo push to 0.54 m/s: rolls to rest at the station
   1.00  tote rear stopped at x 0.960  | slow end of the window (0.52 m/s)
   1.00  tote rear stopped at x 0.960  | fast end of the window (0.60 m/s): meets the stop at 0.36 m/s, bottle stands
   0.00  tote rear stopped at x 0.457  | too soft (0.30 m/s): stops in the lane
   0.25  tote rear stopped at x 0.850  | a bit soft (0.46 m/s): within 12 cm, 0.25
   0.50  tote rear stopped at x 0.899  | nearly (0.48 m/s): within 5 cm, 0.5
   0.00  tote rear stopped at x 0.960  | too hard (0.62 m/s): meets the stop at 0.41 m/s, bottle falls
   0.00  tote rear stopped at x 0.960  | too hard (0.62 m/s), bottle stays up: the end-stop speed latch alone
   0.00  tote rear stopped at x 0.960  | shoved (0.85 m/s): slams the stop, bottle falls
   0.00  tote rear stopped at x 0.960  | shoved hard from a standing start (0.70 m/s)
   0.00  tote rear stopped at x 0.390  | walked by the down-pointing hand to rear 0.39, let go
   0.00  tote rear stopped at x 0.671  | walked by hand only up to the gate (hand to x 0.665, 25.5 cm from the station), let go: 25 cm short
   0.00  tote rear stopped at x 0.791  | walked by hand, pointing forward, to its reach limit (rear 0.79), let go: the gate
   0.00  tote rear stopped at x 0.791  | walked by hand to the reach limit and held there
   0.00  tote rear stopped at x 0.940  | tote lifted and carried to the station
   0.00  tote rear stopped at x 0.940  | bottle lifted out and set back in the tote, then the tote sent to the station
   0.00  tote rear stopped at x 0.940  | bottle knocked over at the start, set upright again, then the tote sent to the station
   0.00  tote rear stopped at x 0.960  | demo, but a gripper is left near the tote
   0.00  tote rear stopped at x 0.960  | demo, but the hand reached out over the lane to x 0.68 for one step mid-coast (touching nothing)
   1.00  tote rear stopped at x 0.960  | demo, the hand reaching out only to x 0.66 (26 cm from the station) mid-coast
   0.00  tote rear stopped at x 0.940  | tote still rolling at the last step
scoring: 23 cases pass; release-speed window 0.477-0.635 m/s, demo 0.54 m/s
left: 14.3 s plan, worst IK 0.090 mm / 0.013 deg
right: 14.3 s plan, worst IK 0.092 mm / 0.015 deg
control_walk left: 26.7 s plan, worst IK 0.090 mm / 0.013 deg
control_walk right: 26.7 s plan, worst IK 0.096 mm / 0.015 deg
nearest robot point to the station: demo 38.7 cm, control_walk 18.8 cm (gate 25 cm)
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/coast-tote-to-far-stop/coast_tote_to_far_stop.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Send the tote down the conveyor to the packing station at the far end." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/coast-tote-to-far-stop-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/coast-tote-to-far-stop/coast_tote_to_far_stop.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Send the tote down the conveyor to the packing station at the far end." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/coast-tote-to-far-stop-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 14.3 s of plan (the tote is at rest at the
station after about 10 s; with the 1 s success hold the episode ends well inside the run limit of 1,200 steps).

## Sim controls: every new latch fires

Each control runs the same command as the demo on a copy of the task file. For the gate, append
`demo = control_walk` to the copy. For the end-stop speed latch, set `VARIANT = "hard"` in the copy. Setting
`DEBUG = True` prints the trace. The reach-gate and end-stop speed latch lines (`LATCH ...`) print in sim whether or
not DEBUG is on; the other trace lines (the end-stop speed, `BOTTLE FELL`, the per-step state) print only with DEBUG.

| Control | What the robot does | Result in sim (Isaac, L40S) |
|---|---|---|
| `control_walk()`: the reach gate | Tries to push the tote all the way by hand, with no momentum (tote never above 90 mm/s). It walks the tote with the hand pointing down to TCP x 0.38, then pointing forward along the lane to TCP x 0.72. | `LATCH reach-gate: robot point 9 (right arm, right pad tip) at (0.673, -0.201, 0.070 above the table) is 24.7 cm from the station (gate 25 cm); tote rear at 0.674` at step 399. The tote ended at rear 0.734, 19 cm short of the station, held by the hand at the arm's reach. task_score 0. |
| VARIANT "hard": the end-stop speed limit | Shoves the tote at 2.5 m/s^2 to 0.75 m/s and lets it go. | `LATCH slam: the tote met the end stop at 545 mm/s (limit 400)` at step 160. Two steps later: `BOTTLE FELL (21.3 deg) with the tote rear at 0.960`. task_score 0. |
| `demo()` | Pushes at 1 m/s^2 to 0.54 m/s and lets go, then rises to the viewing pose. | The tote left the pads at 0.56 m/s, met the stop at 0.26 m/s, rebounded 1.8 cm and rested with its rear at 0.942, past the station line. The bottle stood. The hand reached the viewing pose by step 190, and the nearest robot point stayed 38.7 cm from the station; no latch fired. task_score 1. |

## Recorded runs (this revision)

- **Demo**, recorded with `--store-frames`: task_score 1.0. The episode ended on success after 207 frames (10.35 s).
  The demo video is the top camera and the right wrist camera side by side, from the same run. After the push the
  right hand rises to a viewing pose behind the gate (TCP x 0.52, 30 cm up, 38.7 cm from the station), and its
  wrist camera looks down the lane. The last frame shows the tote resting past the yellow station line with the
  bottle upright. The top camera alone shows the station only at its top edge.
- **Dream-1** (1,200 steps): task_score 0. It moved the tote a short way along the lane, and the tote stopped well
  short of the station.

## Videos and screenshot

```bash
for log in ~/yam-eval/coast-tote-to-far-stop-*/*.json; do inspect-robots video "$log"; done
d=$(ls -d ~/yam-eval/coast-tote-to-far-stop-demo/frames/*/)
ffmpeg -y -i "$d"*top_cam.mp4 -frames:v 1 setup.png
ffmpeg -y -i "$d"*top_cam.mp4 -i "$d"*right_cam.mp4 -filter_complex "[0:v][1:v]hstack=inputs=2" \
  -c:v libx264 -pix_fmt yuv420p -movflags +faststart demo.mp4   # top camera | right wrist camera
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## The conveyor (from the task file)

Positions in cm from the midpoint between the arm bases (x forward, y left).

| Part | Where and what | In `coast_tote_to_far_stop.py` |
|---|---|---|
| Kind | A non-powered gravity roller conveyor of the light-duty miniature kind used for small totes at bench stations, level, along y -20. | `LY` |
| Rollers | 25 zinc-plated steel tube rollers, 16 mm in diameter, at 40 mm centres from x 12.5 to x 108.5, 13 cm long between the side frames, on sealed ball bearings on fixed steel axles. The 14 cm tote always rides on 3 or 4 of them. In sim they are drawn only (no collision). | `ROLL_PITCH`, `ROLL_W` |
| Roller bed | Roller tops 3 cm above the table. In sim one flat bed with friction 0.02, combined by minimum, so the loaded tote slows at 0.02 x 9.81 = 0.196 m/s^2 (also measured in sim: 0.196 m/s^2). | `BED_H`, `MU_ROLL` |
| Frame | Steel, x 10.5 to 111.5 (101 cm), running past the table's edge (x 91) onto a steel leg. Side rails 1.2 cm wide, 13 cm apart, 2 cm above the roller tops. | `X_IN`, `X_END`, `LANE_W`, `RAIL_W`, `RAIL_UP` |
| End stop | Red steel, face at x 110, 4.5 cm above the roller tops. | `STOP_X`, `STOP_UP` |
| Station | The last 18 cm before the stop: the tote's rear face past x 92 (yellow line across the rollers). | `ZONE_REAR` |
| Reach | The farthest any robot point gets along the lane at the tote's wall heights: x 79.1 (right arm). | `REACH_X` |
| Gate | No robot point within 25 cm of the station (plan view): none past x 67 in the lane. | `GATE` |
| Load | Polypropylene tote 14 x 12 x 6 cm, 350 g, with a 500 mL square HDPE bottle 6 x 6 x 16 cm, 500 g, standing loose in its middle: 850 g. | `TOTE`, `TOTE_KG`, `BOTTLE`, `BOTTLE_KG` |

## Reach and distances

All in metres along the lane (x from the arm bases), from the task file's constants and the URDF's kinematics.

| Quantity | Value | Where from |
|---|---|---|
| Robot's max reach along the conveyor | 0.791 (right arm), 0.659 (left arm) | `max_reach()`: the farthest x of any robot point in the lane (y -0.265 to -0.135) at the tote's wall heights (3-9 cm above the table), with every link origin above the table. It uses 600 random joint sets, each refined by L-BFGS. The farthest point is a pad tip. At any height up to 30 cm the right arm gets 0.792. |
| Station line (tote's rear face) | 0.920, 13 cm beyond reach | `ZONE_REAR` |
| Far stop (end-stop face) | 1.100: 31 cm beyond reach, 78.5 cm of travel for the tote's front face from its start (0.315) | `STOP_X` |
| Tote against the stop (rear face) | 0.960 | `REAR_MAX` |
| Gate line | 0.670 (25 cm before the station) | `GATE` |
| Demo hand's farthest point | pad tip 0.533 in the viewing pose (38.7 cm from the station); 0.364 during the push | offline plan check |

**Coast distance from the roller drag.** A tote let go at speed v coasts d = v^2 / (2a) on the rollers. Here a is
the loaded tote's measured deceleration on the real rig, set to 0.196 m/s^2 with a band of 0.176-0.216 ("Real-rig
calibration"): a rolling resistance of 2 % of the tote's weight. In sim it is 0.196 m/s^2 (measured).

| Case | a = 0.176 | a = 0.196 (sim) | a = 0.216 |
|---|---|---|---|
| Speed needed at the reach limit (0.791) to coast to the station line | 0.213 m/s | 0.225 m/s | 0.236 m/s |
| Speed needed at the gate line (0.67) to coast to the station line | 0.297 m/s | 0.313 m/s | 0.329 m/s |
| Coast of a 0.1 m/s nudge | 2.8 cm | 2.6 cm | 2.3 cm |
| Demo: let go at 0.54 m/s with the rear at 0.34, coasts | 0.83 m | 0.74 m | 0.68 m |
| Demo: speed at the end stop (limit 0.40 m/s) | 0.27 m/s | 0.22 m/s | 0.15 m/s |
| Release-speed window at the demo's release point (to the station, under 0.40 m/s at the stop) | 0.45-0.62 m/s | 0.48-0.64 m/s | 0.50-0.65 m/s |

A hand that stops with the tote stops it within millimetres: a nudge coasts about 2.5 cm, and the tote's rear can
be no further than the reach limit, 13 cm short. Only a tote launched at about 0.48 m/s or more from the demo's
release point gets there.

**The end-stop speed limit (0.40 m/s).** When the tote stops dead at the end stop, the bottle (6 x 6 x 16 cm,
uniform, friction 1.0 on the tote's floor) cannot slide: stopping it by sliding would take 9.8 m/s^2, while it tips
at g (w/2)/(h/2) = 3.7 m/s^2. So it pivots on its front bottom edge. It goes over if its kinetic energy about that
edge, (1/2) I w^2 with I = m (w^2 + h^2) / 3 and w = v (h/2) / I, exceeds the work to lift its centre over the edge,
m g (sqrt((w/2)^2 + (h/2)^2) - h/2) = m g x 5.4 mm. That gives v = 0.40 m/s. In the earlier layout's sim runs the
bottle stood after meeting the stop at 0.14 m/s and fell at 0.42 m/s. The scorer latches 0 above 0.40 m/s, whether
or not the bottle happens to fall.

## Real-rig calibration: the loaded tote's deceleration

The release-speed window depends on how fast the rollers slow the tote. A real roller bed's resistance comes from
bearing and seal drag and from how the tote's floor beds on the rollers, and it depends on the load. So the rig does
not take it from the roller spec: it measures the loaded tote's deceleration and sets it.

**Target and acceptance band:** 0.196 m/s^2, accepted from 0.176 to 0.216 m/s^2 (+/- 10 %, a rolling-resistance
coefficient of 0.018 to 0.022). Across the band, the release-speed window from the task file's model (`window()`)
moves from 0.45-0.62 m/s to 0.50-0.65 m/s and stays about 0.16 m/s wide (0.48-0.64 m/s at 0.196). In the same
model, the demo's 0.54 m/s release still brings the tote to the station and meets the stop at 0.15-0.27 m/s.

1. **Light gates.** Fit three through-beam photoelectric gates across the lane, with beams 3 cm above the roller tops
   so they see only the tote's wall. Place them where the tote's front edge crosses x 45, 65 and 85, so the spacing
   is d = 20.0 cm. Set the spacing to +/- 0.5 mm with a steel rule. Log the moment each beam is first broken with a
   timer of 1 ms resolution or better (a photogate timer or a microcontroller). Mount these three gates on removable posts
   outside the rails and take them off for trials. (The two end-stop gates at x 105 and 109, "Measuring the
   latches on a real rig" below, stay in place for every trial; they sit beyond these three and do not affect the
   coast-down.)
2. **Load.** Use the tote as in the task, with the bottle standing in its middle. The tote meets the end stop below
   0.40 m/s in these runs, so the bottle stays up.
3. **Coast-down.** Push the tote along the lane by hand and let go before its front edge reaches the first gate, so
   it passes the first gate at 0.45 to 0.55 m/s (u1 below). Let it roll on to rest.
4. **Formula.** Let t1, t2 and t3 be the times the front edge breaks each beam. Then:

   ```
   u1 = d / (t2 - t1)        u2 = d / (t3 - t2)        a = 2 (u1 - u2) / (t3 - t1)
   ```

   u1 and u2 are the mean speeds over the two spans. Under a constant deceleration, each one is the speed at the
   middle of its span's time, and those two moments are (t3 - t1) / 2 apart, so the formula is exact. For example,
   at 0.196 m/s^2 and 0.50 m/s on release, t2 - t1 = 0.448 s and t3 - t2 = 0.579 s give u1 = 0.446 m/s,
   u2 = 0.345 m/s and a = 0.197 m/s^2. A 1 ms error at the middle gate moves a by about 1.6 %.
5. **Repeat** five times: two runs with u1 near 0.45 m/s, two near 0.55 m/s and one between. Accept when the mean is
   within 0.176-0.216 m/s^2 and every run is within 0.02 m/s^2 of the mean. If the slow runs and the fast runs differ
   by more than 0.02 m/s^2, the drag depends on speed (a sticking or braked roller): find that roller and replace it.
6. **Setting it.** Do not use speed-controlled brake rollers. Their drag rises with speed, while the sim's
   deceleration is the same at every speed.
   - First spin every roller by hand. Each should turn freely and coast down smoothly. Replace any that drags or
     stops well short of its neighbours.
   - Then set the conveyor's grade, uniform along its whole 101 cm frame, by shimming its feet. Each 1 mm of rise
     at the far end adds 9.81 x 0.001 / 1.01 = about 0.0097 m/s^2, and each 1 mm of fall takes the same off.
   - If the tote is too free (below 0.176 m/s^2), raise the far end. If it is too stiff (above 0.216 m/s^2), lower
     it. Then repeat step 5 and record the final grade.
   - If it needs more than 4 mm either way (about 0.04 m/s^2, a slope of 0.23 deg), service the rollers instead of
     tilting further. At that slope, the tote and bottle stay far inside the scorer's 8 and 15 deg limits.
7. **When.** Calibrate before a session's first trial and after any roller change. Check with one coast-down every 10
   trials; it must fall inside the band.

## Measuring the latches on a real rig

- **End-stop speed (limit 0.40 m/s).** Two through-beam light gates on posts outside the rails, beams 3 cm above the
  roller tops (they see only the tote's wall), placed where the tote's front edge crosses x 105 and x 109 cm: 4.0 cm
  apart, the second 1 cm before the stop face. Log the moment each beam is first broken (1 ms resolution or better).
  Speed at the stop = 4 cm / (t2 - t1). That is the mean over the 4 cm, centred 3 cm before the stop face, so near
  0.40 m/s it reads about 0.015 m/s above the speed at the face (0.196 m/s^2 x 0.03 m / 0.40 m/s): it errs toward
  calling a slam. 1 ms of timing error is 1 % of the speed. The gates stay in place for every trial; the tote never
  touches them.
- **Reach gate (25 cm).** Not a sensor in the cell (the tote and bottle would break any beam before the station on
  every success): log both arms' joint positions at the control rate, and compute the same 6 points per arm as the
  sim (gripper link origin, TCP, both pad tips, both finger middles) through the URDF's forward kinematics, with each
  finger's travel from its gripper joint. The trial scores 0 if any point comes within 25 cm of the station's
  footprint (x 92-110, the lane's 13 cm) in plan view, at any logged step.

## Reset between trials

Roll the tote back to the near end of the conveyor (rear face 17.5 cm ahead of the arm bases) and stand the bottle upright in its middle.
