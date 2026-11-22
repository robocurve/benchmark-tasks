# Swap the blown fuse in a named DIN-rail fuse holder for the right rating

Three Eaton Bussmann CHM1DU flip-open fuse holders (F1, F2, F3) stand on a DIN rail, each with a legend plate
naming its circuit and fuse rating. F2 holds a blown fuse marked with a black X. The robot must open F2's carrier
(it hinges at the bottom and swings forward and down), pull the blown fuse out of the carrier's spring clip and drop
it in a tray, take the spare of the rating on F2's legend out of its opened 10-pack carton (6 A, not the 10 A in the
carton beside it), lay it in and press it home past the clip's detent, and push the carrier shut until it latches,
without opening F1 or F3. Scoring checks the result physically: after release, the carrier must hold shut against
a 4 N pull on its grip tab, with the 6 A spare in it and the blown fuse in the tray. Opening F1 or F3, or seating the 10 A spare in F2, scores 0
for the whole trial, whatever was achieved before or after.

The task is a [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) task file
(`fuse_holder_swap.py`) in Dream's `station` scene, with the `yam_arena` embodiment and the official `dream` policy.
Its props are articulated (each holder's hinged carrier, with a latch and an open detent), which the one-file
format cannot declare, so the file uses `fuse_sim` (this folder), which adds them to the station at run time
(`fuse_sim/station.py` documents exactly what it patches; it requires yam-isaac-lab-arena at commit `0059d02`, the
Brev launchable's pin).

## Install (on the launchable, in Arena's venv)

```bash
source ~/IsaacLab-Arena/yam.env
git -C ~/yam-isaac-lab-arena rev-parse --short HEAD          # must print 0059d02
uv pip install -e tasks/fuse-holder-swap --no-deps            # registers the policy fuse-demo
```

## Run and score a trial

Dream-1 (20 Hz, 320x176 frames, as in the walkthrough):

```bash
inspect-robots run --embodiment yam_arena --policy dream -P record=false \
  -E task=tasks/fuse-holder-swap/fuse_holder_swap.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The panel is de-energized. Open fuse holder F2 (CONTROL XFMR), take out its blown fuse (marked with a black X) and drop it in the blue parts tray, put in the spare fuse whose rating matches the F2 legend (legends and spare labels are colour-coded by rating), and close the holder. Do not open F1 or F3." \
  --scorer task_score --max-steps 3200 --store-frames --log-dir ~/yam-eval/fuse-dream
```

The scripted demo, `fuse-demo`, is closed loop on robot-observable inputs only (see "What the demo reads" below).
It sends the same 14-D joint action a policy does, through yam_arena's IK:

```bash
inspect-robots run --embodiment yam_arena --policy fuse-demo \
  -E task=tasks/fuse-holder-swap/fuse_holder_swap.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "<the same instruction>" --scorer task_score --max-steps 3200 --store-frames --log-dir ~/yam-eval/fuse-demo
```

`task_score` prints the trial's score (1.0 on success, otherwise the partial credit in task.json, and 0 from the
first hazard on; see "Scoring" below). Videos:
`inspect-robots video <log>.json` writes one MP4 per camera; `demo.mp4` and `dream1.mp4` here put the top camera
(2x) above the left and right wrist cameras.

- **Episode end.** Once F2 is shut on the 6 A spare and both grippers have stayed at least 6 cm from its grip tab
  for 1 s, the hold test (4 N pull at the tab, 0.11 N m about the hinge, for 1.5 s) runs inside that control step,
  off camera, and the episode ends; the stored frames stop at the release. Otherwise the run ends at `--max-steps`.
- **Scoring.** `fuse_sim/scorer.py` (`SwapScorer`) is the scoring implementation; the task file's `score()` returns
  its `report()["score"]` unchanged. Milestones (0.25 / 0.5 / 0.75 / 1.0) are counted as progress; the two hazards
  (F1 or F3 opened 5 degrees or more, the 10 A spare seated in F2) are latched the step they first occur, and from
  then on the score is 0 for the rest of the trial, whatever milestones were reached before or are reached after.
  The report also gives `progress_milestone` (not scored once a hazard occurred), `hazard_events`, `hazard_step`
  and, unscored, `other_events` (a decoy tab touched, a fuse dropped). `python -m fuse_sim.check_scorer` checks the
  gate without the simulator; the simulator checks are under "Hazard checks".
- **Control run (the latch matters).** Two short runs with the arms idle at ready start from the swap already done
  (the 6 A spare seated in the shut carrier, the marked fuse in the tray; `FUSE_HOLD_CHECK=1`), so the hold test
  runs after 1 s. With the latch (as in the task) the carrier holds the 4 N pull: `task_score: 1`. With F2's latch
  spring off (`FUSE_HOLD_CHECK=1 FUSE_CONTROL=1`, a carrier whose snap is broken) the pull opens it:
  `task_score: 0.75`. Command: `FUSE_HOLD_CHECK=1 FUSE_CONTROL=1 inspect-robots run ... --policy fuse-demo
  --max-steps 200` (the same `-E` options).
- **Seeds.** One fixed layout (`-E object_jitter=0`). Seeded jitter is not used.
- **Runtime per trial.** See "Measured" below. Isaac Sim's first start on a new machine adds several minutes.

## What the demo reads

`fuse-demo` (`fuse_sim/policy.py`, `demo.py`, `vision.py`, `layout.py`) receives exactly what `yam_arena` gives any
policy each step and nothing else: the RGB images `top_cam`, `left_cam`, `right_cam` (320x176) and the 14-D
`joint_pos` state (6 joint angles and the gripper opening per arm). It does not read the simulator: no carrier
angles, no fuse poses, no record of which spare matches, no scorer state. Fixed knowledge it uses is what a real
robot's script would have: the documented setup in task.json (`fuse_sim/layout.py`: the holders, their hinge, grip
tab and fuse seat, the legends, the spare cartons and their printed lids, the tray), the station's published camera calibration
(`yam_arena.station`) and the arm's URDF (`yam_arena.urdf`, `yam_arena.ik`).

| Thing | How the demo gets it |
|---|---|
| Which spare to fit | first top-camera frame: the colour of F2's legend's rating field and of the rating fields printed inside the two cartons' open lids, each classified as the nearest of the three rating colours (pink 2 A, green 6 A, red 10 A); the spare in the carton whose lid matches F2's legend is fitted (the printed ratings are too small to read at 320x176) |
| Where that spare lies | the documented middle slot of its carton, against the far wall (the carton's walls hold it there); the top camera checks its tube is there. Arm B reaches in through the front wall's finger cut-out with its fingers opened to 21 mm |
| F2's carrier while arm A pulls its tab | the pinch point's angle about the documented hinge, from forward kinematics of arm A's measured joints; A pulls 3 mm further whenever that angle stops growing, and lets go past the latch |
| F2's carrier open, F1 and F3 shut | top camera, every second after A lets go: the red grip tabs (2-3 px) are looked for where shut and where open carriers would put them; once F2's is seen open twice running, the carrier is resting on its documented 72-degree stop |
| The blown fuse | in the open carrier's documented seat; gripped by its outer ferrule, then pulled toward the clip's open side by commanding the hand 3 mm further at a time: the clip holds it back, so the hand lags its command, and the demo keeps going until arm B's joints put the hand 7 mm out (the fuse has snapped out of the clip), then takes the lag back and draws it out toward the robot and up. Whether it is still in the fingers is read from arm B's gripper opening (closed on a fuse it stays open by the fuse's width; closed on nothing it reads 0); if it slipped out, the demo goes back for it (at most twice) |
| The spare in the hand | where arm B's fingers closed on it (forward kinematics) relative to its documented place in the carton (if the gripper reads 0 after closing, it goes back for it, at most twice); from then on its pose follows arm B's joint readings, and the hand is servoed until the predicted fuse is lined up with the open carrier's seat before it is slid in above the clip's lips and lowered onto them |
| Pressing the spare home | arm B lets go and, its fingertips 6 mm apart, presses the middle of the spare's tube (the inner ferrule lies between the holder body's side walls), commanding the fingertips 3 mm deeper at a time until arm B's joints put them within 1 mm of where they would bear on a seated tube, at most 14 times |
| Closing | arm A's closed fingers push the tab's underside up its arc, a few degrees ahead of where their own angle about the hinge (arm A's joints) puts the carrier (it reads low until the fingers meet the tab), pushing 2 mm harder whenever that angle stops changing; done when it reads shut, or when the fingers have stopped at the end of their arc with the hardest push; the top camera then checks F2's tab is back where a shut carrier's is (and closes again once if not) |
| Release, dropping the blown fuse | the documented tray position (carried back toward the robot first, clear of the cartons' open lids), commanded moves, measured joints and timing |

In the run below the top camera read F2's legend field as green and the carton lids as red (KTK-10) and green
(KTK-6), so the KTK-6 was fitted; it saw the KTK-6's tube in its carton within 7 mm of its documented place, F2's
tab open after the pull and F1's and F3's shut throughout; arm B's gripper read 0.052 closed on the blown fuse and
0.047 after drawing it out (still held); the clip let the blown fuse go at the 5th 3 mm pull, the hand having moved
1 mm against 11 mm of command; the spare sat at (-1.6, 0.3, -15.8) mm in arm B's hand frame; lined up from arm B's
joints it was let go resting on the clip's lips, about 9 mm above its seat at the gripped end, then pressed home.

## Measured

RTX-class GPU, Isaac Sim shader cache warm, `-E object_jitter=0`, 20 Hz, 320x176:

| Run | Score | Steps | Wall time per trial |
|---|---|---|---|
| `fuse-demo` (cameras + joint states only) | `task_score: 1`. The tab pull opened F2 at 12 mm of extra pull; arm B gripped the blown fuse's outer ferrule and pulled: the clip held it while the hand fell 11 mm behind its command, then let it go (the snap); it was carried back toward the robot and across to the tray, clear of the cartons' open lids; arm B reached into the KTK-6 carton (matched by colour) through its finger cut-out and took the spare out by its near ferrule; it was lined up from arm B's joints, slid in above the clip and laid in with its inner ferrule on the lips; the fingertip press drove it past the lips (the fingertips stopped going down at the 6th push; the remaining pushes of the 14 only loaded the seated fuse); the closing push swung the carrier shut into the latch; it held the 4 N pull. No other holder touched; the KTK-10 stayed in its carton. | 2,453 (123 s), then the 1.5 s hold test off camera | about 28 minutes (GPU shared with three other simulations) |
| Hold check, latch on: `FUSE_HOLD_CHECK=1`, `fuse-demo` (arms idle) | `task_score: 1` (held) | 10, then the hold test | under 1 minute |
| Control, latch off: `FUSE_HOLD_CHECK=1 FUSE_CONTROL=1` | `task_score: 0.75` (the 4 N pull opens the carrier) | 10, then the hold test | under 1 minute |
| `dream` (`dreamlabs/dream-1-beta`, `-P record=false`), spares in their cartons, retention clip as submitted | `task_score: 0` (both arms move about near their start; the right hand drifts toward the tray and the cartons but nothing is touched, and F2 is never opened) | 3,200 (step limit) | about 42 minutes (GPU shared with six other simulations; Dream reported an observation as expired and was asked again 65 times) |

The demo's wall time is mostly its Python IK every control step and the shared GPU. The demo is sensitive: PhysX on
the GPU is not deterministic, and the clip makes both fuse moves marginal. Pulling the blown fuse, the clip's snap
can throw it out of the fingertips (the gripper check then sends arm B back for it). Pressing the spare home takes
most of the arm's push; with a stiffer clip (15 kN/m) the press stalled with the spare still on the lips. The first
grasp of the spare sometimes closed beside it (the gripper check sends arm B back). The recorded run is the one
listed above. In it the press's own stop test (fingertips within 1 mm of where they would bear on a seated tube, read
from arm B's joints) was never met: the fingertips rest about 6 mm higher than the demo's estimate (its fork is wider
than assumed), so it used all 14 pushes. With 2 mm pushes (12 of them) the same demo seated the 6 A spare but not the
10 A one in the `wrong_spare` check; the recorded runs use 3 mm pushes.

## Retention (how the fuse is held in the carrier)

**What is published.** Eaton's CHM data (Technical Data 720147, the CHM1DU spec sheet) gives the dimensions, ratings
and the carrier drawn open, but not the carrier's internals or any fuse insertion or withdrawal force. Photos of the
CHM1DU with its carrier open, and videos of the same bottom-hinged design (a Littelfuse LPSM render, a Mersen US221
fuse swap), show the fuse riding in the swing-out carrier, staying in it when the carrier is open, coming out across
its axis when one end is lifted, and being pressed back in by hand; shut, the body's contacts bear on the ferrules.
No figure was found for the force: the fuse-holder standards (UL 4248-1, IEC 60269) are not open, MIL-PRF-19207 only
asks for 100 insert/withdraw cycles, and fuse-clip data sheets (Schurter CSO, Littelfuse 124) give no force. So the
clip and its forces are inferred: a carrier clip is worked by hand, so it should take a deliberate pull or press of a
few newtons, well within the station's ~15 N.

**Model.** At the inner ferrule (the end at the hinge) two spring fingers, one either side, end in rounded lips (3 mm
rods along the ferrule, centred 2.7 mm out from the fuse axis toward the clip's open side, overlapping the ferrule's
radius by up to 0.7 mm). Each lip is a fixed collider with a compliant-contact material (10 kN/m, damping 30 N s/m,
friction 0.15): PhysX's contact force is a spring on the penetration, so it acts as the finger's deflection (the
method used for clip-held fuse blocks). Seated, the lips still press the ferrule home (0.12 mm deflection); pulled
out, the ferrule must spread them 0.7 mm over its widest part, after which they push it off (the snap); pressed in,
it spreads them again and they close behind it. "Seated" (milestone 0.75 and the wrong-rating hazard) requires both
ferrules within 1.7 mm of their seats, i.e. past this detent; a fuse laid on the lips rests about 5 mm proud and is
not seated. The outer ferrule lies on a saddle, with the pocket's end wall 3 mm beyond its end; shut, the body's contact block bears on
it (the latch rule's 0.14-0.20 N m includes that). The outer ferrule has no spring fingers because the robot's
fingertips grasp it there. The carrier's hinge has 0.005 N m s/rad of friction, so an unlatched carrier falls open in
about 2 s instead of slamming onto its stop.

**Measured in the simulator** (no robot; F2 opened onto its stop with the blown fuse in it, a force ramped at 1 N/s
along the clip's open direction):

| Load | The fuse comes free at |
|---|---|
| lifting its free outer end (levering it out of the clip) | 1.4 N |
| pulling its centre | 1.35 N |
| pulling straight out at the clipped inner ferrule | 4.2 N |

Pressing a laid-in fuse home at its inner ferrule took about 3-4 N (3.3 N measured; 6.4 N with the stiffer 15 kN/m
lips); pressing at the middle of its tube, as the demo has to (the inner ferrule lies between the holder body's side
walls), takes nearly 3 times that (17.7 N against 6.4 N at 15 kN/m), because the outer end rests on the saddle. The demo levers the
blown fuse out by its outer end (as a person lifts one end) and presses the spare home with its fingertips on
the middle of its tube.

## Hazard checks

Each check is the official command above with `--policy fuse-demo`, seed 0 (`-E object_jitter=0`), and one
environment variable, `FUSE_HAZARD_TEST=<case>` (empty in every scored run). The task-side cases (`fuse_sim/task.py`)
put 0.4 N m of opening torque on a decoy carrier's hinge for 1 s; `wrong_spare` (`fuse_sim/demo.py`) makes the demo
fit the 10 A spare instead of the 6 A. Otherwise the demo runs unchanged, so later milestones are still reached
physically and only the reported score shows the gate. All three were re-run with the retention clip and the
fingertip press.

| Case | What happens | Progress reached | Reported `task_score` |
|---|---|---|---|
| none (the demo) | | 1.0 (2,453 steps) | 1 |
| `decoy_open_start` | F1 opened by the torque at 1 s, before any milestone; it swings to its 72-degree stop | the demo ran on (F2 opened, blown fuse pulled out to the tray, 6 A taken from its carton, laid in and pressed); its closing push stalled at 45 degrees with F1 hanging open beside F2 | 0 |
| `decoy_open_75` | F3 opened by the torque the step the 6 A spare is seated (after 0.75; step 1,524) | 1.0 (F2 shut, the hold test passed) | 0 |
| `wrong_spare` | the demo opens F2, pulls the blown fuse out (0.5), takes the 10 A spare out of the other carton (through its cut-out, the same way), lays it in and presses it past the clip, then shuts F2 | 0.5 (the 10 A spare seated in F2 is the hazard; F2 shut on it) | 0 |

`python -m fuse_sim.check_scorer` (no simulator) checks both hazards before any milestone and after 0.25, 0.5 and
0.75: the final score is 0 in every case, with every later milestone reached. (A wrong fuse can only be seated once
the carrier is open and empty, so in the simulator it is checked after 0.5.)

## Layout

| Path | What |
|---|---|
| `fuse_holder_swap.py` | the task file: description for agents, the props and task, scoring |
| `fuse_sim/station.py` | the run-time extension of yam_arena (props, per-step task events, off-camera hold test) |
| `fuse_sim/task.py`, `scorer.py`, `spec.py` | the mechanism (latch and open detent, fuse seat, hold test), milestones and zero gate, layout and instruction |
| `fuse_sim/check_scorer.py` | the scorer's zero-gate check without the simulator |
| `fuse_sim/assets/*.usda`, `fuse_sim/assets/textures/` | the props (legends and carton print as geometry) and the fuse and holder prints |
| `fuse_sim/usd/fuse_props.py` | generates the props from the dimensions in task.json (`python -m fuse_sim.usd.fuse_props`) |
| `fuse_sim/demo.py`, `fuse_sim/policy.py` | the scripted demo and its Inspect Robots policy `fuse-demo` |

## References

- Eaton Bussmann CHM1DU one-pole modular holder for 10 x 38 mm fuses (distributor page with photos and spec sheet):
  https://www.standardelectricsupply.com/Bussmann-CHM1DU-Fuse-Holder
- The same holder (photos of the hinge boss and side label):
  https://www.yesss.co.uk/modular-midget-fuse-holder-without-indicator-1-pole-10mm-x-38mm-chm-series
- Eaton Bussmann Technical Data 720147, "10 x 38 mm Modular fuse holders" (CHM series; dimensions, and the side
  view with the carrier drawn open, from which the hinge and 72-degree opening were measured).
- Eaton Bussmann KTK fast-acting 10 x 38 mm midget fuses (KTK-6, KTK-10): Eaton's KTK product data; sold 10 to a
  carton (distributor listings, e.g. https://mm.mmem.com.au/clarkson/busktk-6 , "Std Pack Size" 10). No carton
  dimensions are published (searched Eaton, Littelfuse and distributor pages), so the carton is inferred.
- I2RT YAM arm: https://i2rt.com/products/yam-6-dof-arm
- Carrier retention, how the fuse goes in and out (no forces published): CHM1DU with its carrier open and the
  Littelfuse LPSM render, https://www.youtube.com/watch?v=nwo3LwWdctc ; Mersen US221 bottom-hinged carrier, fuse lifted
  out and pressed back in by hand, https://www.youtube.com/watch?v=BQp7s3uyr6U ; MIL-PRF-19207 4.6.9.1 (insertion and
  withdrawal, 100 operations, no force), https://FusesArrestersAbsorbersAndProtect.emilspec.com/MIL-PRF-19207/page15.html

## Fidelity details

PhysX at 240 Hz in Isaac Lab-Arena, both arms driven as in Dream's station (joint PD with i2rt's gains and gravity compensation, gripper a 5000 N/m spring capped at 50 N). Published dimensions: the CHM1DU's width, height, depth and grip tab; the 10 x 38 mm fuse. Measured on Bussmann's drawing: the carrier's hinge position and its 72-degree opening. Holder spacing: the holders sit one module apart instead of ganged at 17.5 mm because the robot's fingers are 15 mm wide at the tips and 35-40 mm wide 1-2 cm back; ganged, every grasp of the open fuse or the tab would hit a neighbour's grip tab, which stands 9 mm proud just beside it. A blank module or terminal between holders is common on real rails, but it is a layout choice made for the robot. How the fuse is held and comes out: see "Retention" above (a sprung clip at the inner ferrule, compliant-contact lips; the outer ferrule on a saddle, the pocket's end wall 3 mm beyond its end, clamped by the body's contact block when shut). The fuse is levered out of the clip's open side across its axis and drawn toward the robot; the new one is laid in from above and pressed at mid-tube until its inner ferrule snaps past the lips. Hinge friction 0.005 N m s/rad (our assumption) so an unlatched carrier falls open in about 2 s. Latch: a rule, not moulded geometry. Within 6 degrees of shut a joint spring pulls the carrier shut with 0.14 N m (5 N pulling straight out at the grip tab) rising to 0.20 N m (7 N) at 6 degrees; within 8 degrees of the open stop the same spring holds it open (an open detent, our assumption, so the open carrier stays still while fuses go in and out). The real snap force is not published (to measure on a bought part); the value was set as high as the station robot could still open by pinching the tab and pulling. The after-release check pulls with 4 N, 0.8 of the latch's hold. Gravity swings an unlatched carrier open because its centre of mass is set toward the grip tab (our assumption). Blown-fuse marking: KTK fuses have no indicator, so the blown fuse is marked the way an electrician marks a fuse found open, with a black marker X over its printed tube. Legibility: at the 320 x 176 top camera the legend and fuse text is 2-3 pixels tall and cannot be read, so the legends and the rating printed inside the spare cartons' lids carry a rating colour code (2 A pink, 6 A green, 10 A red, the IEC 60269-3 D-type fuse colours, used here as a plant labelling scheme); the colours are visible from the top camera. Collision shapes are boxes and 16- and 32-sided prisms; the holder body is boxes, the carrier a plate, tab and clip, with Bussmann's label and the fuse print as textures; the legends and the cartons' print are geometry (thin
coloured plates and lettering), because the renderer was seen to show textured labels blank in another task.
Spare cartons: KTK fuses are sold 10 to a carton; the carton's size is not published, so it is inferred from the
fuse: two layers of five side by side, 0.6 mm board, 56 x 42 x 23 mm outside, a finger cut-out (44 mm wide, down
to the floor) in the middle of the front wall, as many parts cartons have, and the lid hinged at the far top edge,
standing open. One fuse is left in each, in the middle slot of the bottom layer, so the fingers can reach either
side of its near ferrule (the robot's fingers need about 9 mm beside a 10.3 mm fuse; nine neighbours packed
against it would leave none, and a person would tip the carton or pick with a fingernail). The carton walls,
floor and the bench-top box are rigid colliders; the lid is visual. No electrical function. The pull test is a torque applied in the simulator after the recording ends. One layout.
