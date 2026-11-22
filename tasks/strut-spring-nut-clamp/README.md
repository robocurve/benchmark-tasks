# Fasten a strut fitting with a spring nut: start the bolt square by hand, nut edge-on into the slot, bolt hand-snug

A 2 ft length of Unistrut P1000 strut channel (1-5/8 x 1-5/8 in, 12 ga, Perma-Green) is bolted flat to the bench top,
open side up, running left to right in front of the arms, as strut is surface-mounted on floors, walls and equipment
benches. A strip of red tape across its near wall and lip marks where a fitting has to go. In front of it, to the
right, stand the installer's parts, not assembled, on two wooden parts blocks: a Unistrut P1010 1/2-13 channel spring
nut standing on a post (its spring hanging in a pocket in the post's top), and a Unistrut P1064 flat plate fitting
(1-5/8 in square, 1/4 in thick, 9/16 in hole) lying on a block with a 1/2-13 x 15/16 in hex head cap screw dropped
through its hole (head on the fitting, shank down in a hole in the block). **The bolt is not in the nut: the robot
starts it.** The instruction gives only the goal and the constraints: fasten the fitting to the strut at the red mark
with its spring nut and bolt, leave the bolt hand-snug (not torqued), do not drop any part, let go when done. It does
not say how.

The way installers do it: hold the spring nut by its body, lay the fitting on it, put the bolt through the fitting's
hole and start it into the nut by hand, square to the nut (a crooked start cross-threads: the bolt binds after a
turn or so and the nut and bolt are scrap), two or more turns so that the nut hangs on the bolt; then, as Unistrut's
own instructions put it (GEC17 p7 and p10: "Insert the spring nut anywhere along the continuous slotted channel"; "A
90 degree clockwise turn aligns the grooves in the nut with the inturned edges of the channel"; the spring "holds the
nut in position while the connection is completed"), the nut goes edge-on into the slot (it is 19 mm wide, the slot
22.2 mm; 34.5 mm long, so it only fits lengthwise), is pushed down past the lips against its spring, turned a quarter
turn clockwise so its two grooves come under the lips' turned-down edges, and let up: the spring presses the grooves
onto the lips. Then it is slid along the channel to where the fitting goes, and the bolt is run down until the
fitting is held. **This task ends there: the bolt run down hand-snug, the fitting seated square at the mark, ready
for the installer's torque wrench.** The final tightening (50 ft-lb, 68 N m, for 1/2 in, Unistrut and B-Line
catalogs) is a separate wrench step far beyond the station; the task does not scale it down or call the bolt tight.

The bolt starts dropped through the fitting's hole but not in any nut, as fittings and their bolts often lie on a
bench; passing it through the hole first would add a 0.8 mm-clearance placement but no new skill, while the thread
start is unchanged: the robot starts the bolt into the nut itself.

The hard part is bimanual and contact-rich: one hand holds the nut by its ends (below the fitting, which rests on its
top face), the other lowers the fitting and bolt onto it so the bolt's end finds the nut's hole, then grips the head
and turns it in strokes, square to the nut: tilted more than 5 degrees when the thread catches, the bolt cross-threads
(a hazard). Then the nut, hanging on the bolt, must go into the slot edge-on (1.6 mm clearance a side), be pushed past
the lips against its spring (about 6 N) and turned with the bolt while it is down; it turns only clockwise and only
while it is below the lips; let up too early or not turned, it rises back out of the slot. Then the assembly has to
be slid along the channel to within 3 mm of the mark against the spring's friction, the fitting squared to the
channel (it can turn on the nut while the bolt is being started), and the bolt run down about three more turns in
regrasped strokes until the thread resists.

The task is a [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) task file
(`strut_spring_nut.py`) in Dream's `station` scene, with the `yam_arena` embodiment and the official `dream` policy.
Its nut, spring and bolt are one articulation (the bolt joined to the nut by a free six-axis chain whose drives are
off until the thread rule says the thread has started; see "Fidelity details"), which the one-file format cannot
declare, so the file uses `strutnut_sim` (this folder), which adds it to the station at run time
(`strutnut_sim/station.py` documents exactly what it patches; it requires yam-isaac-lab-arena at commit `0059d02`,
the Brev launchable's pin). The task runs PhysX at 480 Hz (the policy still acts at 20 Hz).

## Install (on the launchable, in Arena's venv)

```bash
source ~/IsaacLab-Arena/yam.env
git -C ~/yam-isaac-lab-arena rev-parse --short HEAD          # must print 0059d02
uv pip install -e tasks/strut-spring-nut-clamp --no-deps     # registers the policy strut-nut-demo
```

## Run and score a trial

Dream-1 (20 Hz, 320x176 frames, as in the walkthrough):

```bash
inspect-robots run --embodiment yam_arena --policy dream -P record=false \
  -E task=tasks/strut-spring-nut-clamp/strut_spring_nut.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fasten the square plate fitting to the strut at the red mark with its spring nut and bolt. Leave the bolt hand-snug, not torqued. Do not drop any of the parts, and let go of them when you are done." \
  --scorer task_score --max-steps 4800 --store-frames --log-dir ~/yam-eval/strut-nut-dream
```

The scripted demo, `strut-nut-demo` (closed loop, robot-observable inputs only; see "What the demo reads"):

```bash
inspect-robots run --embodiment yam_arena --policy strut-nut-demo \
  -E task=tasks/strut-spring-nut-clamp/strut_spring_nut.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "<the same instruction>" --scorer task_score --max-steps 4800 --store-frames --log-dir ~/yam-eval/strut-nut-demo
```

`task_score` prints the trial's score (1.0 on success, otherwise the partial credit below, or 0 after a hazard).
Videos: `inspect-robots video <log>.json` writes one MP4 per camera; `demo.mp4` and `dream1.mp4` here put the top
camera (2x) above the left and right wrist cameras.

- **Episode end.** Once 0.8 is reached and every finger pad has stayed at least 3 cm from the fitting and the bolt
  for 1 s without a break (the count restarts whenever a pad comes closer), the after-release pull check (2 s of
  simulated time) runs inside that control step, off camera, and the episode ends; the stored frames stop at the
  release. Otherwise the run ends at `--max-steps`.
- **Scoring.** `strutnut_sim/scorer.py` (`Scorer`); the task file's `score()` returns its score unchanged.
  Milestones, each needing the ones before it at the same step: 0.2 the bolt started by the robot at least 2 turns
  into the nut (its thread caught square, see below); 0.4 also the nut seated under the lips (turned across the
  channel within 10 degrees, its top within 1.2 mm of the seated height, centred within 3 mm); 0.6 also the bolt's
  axis within 3 mm of the mark's centre line; 0.8 also the bolt engaged at least 5 turns; 1.0 the after-release check
  passed (at its start the 0.8 conditions, the fitting flat on the lips within 0.6 mm and 2 degrees, square within
  3 degrees, the head down on it within 0.5 mm; then 20 N along the channel on the fitting moves it less than 1 mm).
  Hazards, each 0 for the whole trial from the step it happens, whatever was reached before or after: a part dropped
  (fitting, nut or bolt on the table off the channel); the nut loose in the channel (inside it with the bolt not
  started at least half a turn in it: dropped in, or run off the bolt; in a real channel it is lost); the bolt
  **cross-threaded** (its thread caught while its axis was more than 5 degrees off the nut's). Cross-threading zeroes
  the score rather than capping it: a cross-threaded 1/2-13 bolt has damaged both threads and the connection is not
  fit to torque, which is the point of the task. `python -m strutnut_sim.check_scorer` checks the gate without the
  simulator.
- **Control run.** `-P skip_turn=1` with `--policy strut-nut-demo` and `SNC_CONTROL=200`: the demo does everything
  as in the demo run (it starts the bolt) but leaves out the quarter turn, so the nut stays edge-on in the slot (not
  under the lips) while the bolt is slid to the mark and run down; `SNC_CONTROL` starts the after-release pull check
  at that time whatever the robot has done (otherwise a trial that never reaches 0.8 never starts it). It must score
  below 1.
- **Seeds.** One fixed layout (`-E object_jitter=0`). Seeded jitter is not used.

## What the demo reads

`strut-nut-demo` (`strutnut_sim/policy.py`, `demo.py`, `vision.py`) receives exactly what `yam_arena` gives any
policy each step and nothing else: the RGB images `top_cam`, `left_cam`, `right_cam` (320x176) and the 14-D
`joint_pos` state. It does not read the simulator: no nut, bolt or fitting pose, no thread or spring joint, no
scorer state. Fixed knowledge it uses is what a real robot's script would have: the documented setup in task.json
(where the channel, the mark and the parts blocks are; the channel section, nut, bolt and fitting sizes; 13 threads
per inch), the station's published camera calibration (`yam_arena.station`) and the arm's URDF (`yam_arena.urdf`,
`yam_arena.ik`).

| Thing | How the demo gets it |
|---|---|
| The mark | first top-camera frame: the red tape's pixels on the lip, back-projected onto the lip's top plane, give its centre along the channel; once the hand holds the kit over the channel, the right wrist camera sees the same tape larger and its estimate replaces the top camera's |
| The parts | in their documented places on the blocks: the nut is taken by its two ends (arm A, fingers level and forward, below its top face), the fitting by two edges (arm B, fingers down), the bolt's head across two flats (arm B) |
| The held nut | arm A's joints (forward kinematics, including the hand's measured rotation) and the grasp offset give the nut's top-face centre; the fitting and bolt are lowered onto that point |
| Starting the bolt | the bolt's end lands in the nut's hole as the fitting is lowered to 3 mm above the nut and let go; arm B grips the head and turns it in strokes of a third of a turn, each grip 1 mm below the head's expected height so the hand presses lightly; the wrist's measured roll counts the turns; the head's height follows from the turns (the first eighth of a turn finds the thread's start) and 13 threads per inch; seven strokes (2.3 turns of the wrist, about 2.5 turns of thread); each stroke also lowers the hand a third of a pitch, following the thread, so the hand never holds the bolt up against it (bolt tension would raise the thread's friction and turn the nut in arm A's fingers) |
| Nut pushed below the lips | the nut hangs below the head by the bolt length less the engaged length counted above; the hand is lowered until arm B's joints put the head where the nut's top is 3 mm below the lips, against the spring |
| The quarter turn | a third of a turn of the wrist (joint 6) clockwise while pushed; the nut turns with the bolt until its corners stop (the rest turns the bolt in it) |
| Bolt at the mark | arm B's joints give the head's position while it holds it; the hand slides along the channel until that is on the mark's centre line |
| Fitting squared | after the slide, the open fingers come down beside the channel's walls at the bolt's position (arm B's joints) and close across the channel on the fitting's two edges; the fitting is as wide as the channel, so the jaws line it up with the walls (it can turn on the held nut while the bolt is started) |
| Turns run down, hand-snug | before each grip the hand waits until its joints put it within 0.6 mm of the head's position (so closing does not push the kit along); the wrist's measured roll during each stroke while the fingers hold the head (turns counted), the head's height from the counted turns and the 13 threads per inch, and the wrist's lag behind its command at the end of a stroke (the thread resisting) |

In the demo run below the top camera put the mark's centre at y = -38.7 mm (4 px; the layout says -40.0) and the right
wrist camera, held over the channel, at -39.9 mm (81 px on the lip), which the demo used. Arm A took the nut off its
post and held it 5.5 cm up; arm B lowered the fitting, the bolt hanging in its hole, onto it (the bolt's axis 0.1 mm
from the nut's by the two arms' joints) and let go: the bolt's end sat in the nut's hole. Arm B took the head; the
thread caught in the first stroke with the bolt 2.5 degrees off the nut's axis (under the 5 degree limit) and its end
2.0 mm off centre; seven strokes (2.30 turns of the wrist) left the bolt 2.52 turns into the nut. Meanwhile the
fitting turned 29 degrees on the held nut. Arm A let go and backed out; arm B pushed the nut into the slot (its top
3 mm below the lips), gave the quarter turn and eased off (nut seated), slid it to the mark (its joints put the bolt
1.6 mm from the mark; 0.6 mm in the end), squared the fitting (21 degrees to 0.4) and ran the bolt down in eleven
strokes; the wrist lagged its command by 2.0 degrees while the bolt ran free and by 3.4 degrees on the stroke that
brought the head down (3.61 turns counted against 3.53 expected), and the demo stopped there.

## Measured

RTX PRO 6000 class GPU shared with up to seven other Isaac Sim processes (this task's own runs, in parallel), shader
cache warm, `-E object_jitter=0`, 20 Hz, 320x176 frames, PhysX at 480 Hz:

| Run | Score | Steps | Wall time per trial |
|---|---|---|---|
| `strut-nut-demo` (cameras + joint states only) | `task_score: 1`. Nut taken off its post by its ends and held up; fitting, the bolt hanging in it, lowered onto it and let go, the bolt's end in the nut's hole; thread caught in the first stroke, the bolt 2.5 degrees off the nut's axis; 7 strokes: 2.52 turns engaged (0.2, step 1250); nut pushed into the slot, a third of a turn, seated (0.4, step 2010); slid to the mark (0.6, step 2130); fitting squared (21 to 0.4 degrees); 11 strokes ran the bolt to 6.1 turns, head down on the fitting (0.8, step 3190), bolt tension 53 N; released; pull check 20 N: the fitting moved 0.069 mm; square within 0.5 degrees | 3,577 (179 s) | about 85 minutes (up to seven other simulations running) |
| Control: `-P skip_turn=1`, `SNC_CONTROL=200` (no quarter turn) | `task_score: 0.2`. The bolt was started as in the demo (0.2); the nut was pushed into the slot but not turned, so eased it rose back out edge-on under the fitting; slid and run down as in the demo (no wrist lag ever showed snug; stopped after 4.6 turns counted); never seated, so no 0.4; the forced pull check slid the whole kit 40 mm along the channel | 3,991 (forced check at 200 s) | about 93 minutes (up to seven other simulations running) |
| `dream` (`dreamlabs/dream-1-beta`, `-P record=false`, the instruction above) | `task_score: 0` (the arms move toward the channel and the nut's post; no part is picked up or moved; no hazard). Run once on this revision, before the last changes to the thread-start rule's internals (the nut's hole collider, the bolt shank's sides, the entry stop); Dream-1 never touched a part, so they cannot have changed its score | 4,800 (step limit) | about 2 hours 15 minutes (up to seven other simulations running; Dream reported an observation as expired and was asked again 99 times; mean inference latency 0.48 s) |

The demo's wall time is mostly the doubled physics rate, its Python IK every control step and the shared GPU.

## Hazard checks

Each check is the official demo command above with `--policy strut-nut-demo`, seed 0 (`-E object_jitter=0`). The
cross-thread check is physical: `-P tilt_deg=8` makes the demo hold the bolt tilted 8 degrees (about the channel's
direction, round its end) while it lowers it into the nut and gives the first stroke. The others set one environment
variable, `SNC_HAZARD_TEST=<case>`, read by `strutnut_sim/task.py` (empty in every scored run): the test harness causes
the hazard by writing a state (a stand-in for a hand or a slip); the demo otherwise runs unchanged, so later
milestones are reached physically where the disturbance allows, and only the reported score shows the gate.
`--max-steps` 1500 for the cross-thread check and 1200 for `drop_start` (both hazards come early), 4800 otherwise.

| Case | What happens | Progress reached | Reported `task_score` |
|---|---|---|---|
| none (the demo) | | 1.0 | 1 |
| `-P tilt_deg=8` (cross-thread) | the demo holds the bolt tilted 8 degrees while it presses its end into the nut's hole and turns it; the thread catches with the bolt 7.8 degrees off the nut's axis (over the 5 degree limit): cross-threaded, the bolt jams; the demo lets go | 0 (the thread never started, so no milestone) | 0 |
| `SNC_HAZARD_TEST=drop_start` | the fitting (with the bolt through it) is put on the table in front of the blocks at 1 s (step 20), before any milestone, and left there; the bolt drops into its block's hole; the demo carries on without them | 0 | 0 |
| `SNC_HAZARD_TEST=nutoff_60` | the bolt is backed out of the nut past its start the step 0.6 is reached (step 2124), so the thread rule frees it and the nut, seated in the channel, is no longer on the bolt (loose in the channel); the demo carries on turning a free bolt | 0.6 | 0 |
| `SNC_HAZARD_TEST=drop_80` | the fitting is put on the table the step 0.8 is reached (step 3183) and put back where it was the next step; the demo then snugs the bolt and lets go | 1.0 (the after-release pull check passed: the fitting moved 0.087 mm) | 0 |

`python -m strutnut_sim.check_scorer` (no simulator) checks the same gate for each hazard (dropped, nut loose,
cross-threaded) raised before any milestone and after 0.2, 0.4, 0.6, 0.8 and 1.0 (final score 0 each time, with
every later milestone still reached), and 1.0 without a hazard.

## Real-trial checks

- **Bolt started (0.2):** on the video, the head turns with the hand and descends (the hex's flats, 13 threads per
  inch: 2 turns = 3.9 mm); after the trial, the engaged length below.
- **Cross-threading (hazard):** before the start, a square against the nut's held face and the bolt (5 degrees = 2 mm
  over 24 mm of shank); after the trial, back the bolt out by hand: a cross-threaded start binds and shows galled
  flanks on the bolt's first threads.
- **Nut seated (0.4):** after the robot lets go, look into the slot: the nut lies across the channel, its grooves on
  both lips; pulled up by its bolt with fingers it does not come out. Photo from above.
- **At the mark (0.6):** a rule from the mark's centre line (the tape's middle) to the bolt's centre, along the
  channel: 3 mm or less.
- **Turns (0.8):** count the head's turns on the video (the hex's flats) from the start; or after the trial, measure
  the bolt's end below the nut through the slot: engaged length = 15/16 in less the gap from the head to the nut's
  top; 5 turns = 9.8 mm.
- **Seated, square, head down (at the check's start):** feeler gauge under the fitting at its four corners (0.6 mm
  does not enter), a square against the channel's side and the fitting's edge (3 degrees = 2 mm over 41 mm), feeler
  under the head (0.5 mm does not enter).
- **Hand-snug pull check (1.0):** with the robot clear, hook a spring scale on the fitting and pull along the channel
  to 20 N (2 kg on the scale) for 2 s: the fitting must not slide (less than 1 mm, a pencil line beside it). This
  checks the bolt is run down **hand-snug**, holding the fitting by the nut's grip on the lips; it is not a torque
  check and the bolt is not torqued (that would be 68 N m with a wrench).
- **Hazards:** from the video: any part on the bench off the channel; the nut inside the channel without the bolt in
  it (it drops inside the channel).

## Closest tasks on the board (checked 2026-11-19)

Searched for strut, Unistrut, channel, spring nut, nut, thread, clamp, bolt and screw. Nothing installs a channel
spring nut or works in a strut channel's slot. Closest:
- "Seat a strut across a keep-out" (accepted): moves a grey bar between saddles; no channel, nut or fastening.
- "Fit a cage nut into a server cabinet's front rail from inside" (pending): a different captive nut (a sprung cage
  clipped into a square hole by hooking its lips with a screwdriver); no quarter turn, no slot, no bolt.
- "Fix a hook tight to a hollow wall with its toggle bolt" (pending): a fastener whose wings open behind a board
  through a round hole; tightened against the wall, no slide to a mark.
- "Bolt a coupling nut to an upright plate through its hole with an M10 screw" (accepted): threads a screw into a
  free nut held by the other hand, as this task's first part does; here the start must be square (cross-threading
  is scored as a hazard), and the nut, then hanging on the bolt, goes into the channel under its lips to a mark.
- "Close off a terminal strip: fit and tighten an end bracket on a loose DIN rail" (accepted): a clip-on bracket on
  a rail tightened by a screw; no nut in a slot.
What is distinct here: starting a bolt square into a spring nut held in the other hand (with a cross-threading
hazard), then putting the nut into a lipped channel edge-on, pushing it past the lips against its spring and
turning it under them (it can only be turned while held down, and only one way), then positioning it inside the
channel to a mark and running the bolt down to hand-snug in regrasped strokes.

## Layout

| Path | What |
|---|---|
| `strut_spring_nut.py` | the task file: description for agents, the props and task, scoring |
| `strutnut_sim/station.py` | the run-time extension of yam_arena (props, per-step task events, physics rate) |
| `strutnut_sim/task.py`, `scorer.py`, `spec.py` | props, screw rule, measurements, after-release check, hazards, layout, instruction |
| `strutnut_sim/check_scorer.py` | the scorer's zero-gate check without the simulator |
| `strutnut_sim/usd/props.py`, `strutnut_sim/assets/` | the props, generated with `python -m strutnut_sim.usd.props strutnut_sim/assets` |
| `strutnut_sim/demo.py`, `vision.py`, `policy.py`, `kit.py` | the scripted demo and its Inspect Robots policy `strut-nut-demo` |

## References

- Unistrut General Engineering Catalog No. 17 (2011): https://acomee.com.mx/catalogos/catalogosu/UNISTRUT%20GENERAL%20ENGINEERING%20CATALOG.pdf
  - p7 "Unistrut connection" steps (insert the spring nut anywhere along the slot; a 90 degree clockwise turn aligns
    the grooves with the inturned edges; insert the bolt through the fitting into the nut; tighten with a wrench).
  - p10 materials: nuts rectangular with ends shaped to permit a quarter turn clockwise; two toothed grooves engage
    the inturned edges.
  - p24 P1000 section: 1-5/8 x 1-5/8 in, 12 ga (0.105 in), slot 7/8 in, lips 3/8 in, return 9/32 in; 189 lb/100 ft.
  - p71 fittings hardware ("All 9/16 in diameter holes use 1/2 x 15/16 in hex head cap screws and 1/2 in nuts -
    P1010") and design bolt torques; p75 flat plate fittings (P1064: 1/2 in bolt, 9/16 in hole, 1/4 in thick,
    17 lb/100); standard fitting dimensions.
- Unistrut Nuts & Hardware section (P1006-P1010 weights, bolt torque 50 ft-lb for 1/2 in, pull-out and slip loads):
  https://cdn.gescan.com/PIM_Docs/Docs/STEP_ASSETS_PDF/13505483.pdf
- Unistrut P1006-P1010 channel nut with spring, submittal sheet: https://static.chipdip.ru/lib/614/DOC028614241.pdf
- Eaton B-Line channel nuts and hardware catalog section (N225 1/2-13 spring nut 1/2 in thick; recommended torque
  50 ft-lb, 68 N m): https://lavitekco.com/Upload/files/Strut%20systems-acc/channel-nuts-and-hardware-catalog-section___.pdf
- Eaton N225HDG spec sheet (2 x 1.31 x 0.85 in, 0.116 lb): https://www.elliottelectric.com/Media/N225HDG-BLI-1021-3-Attachment.pdf
- Hex cap screws, ASME B18.2.1 dimensions (1/2-13: 3/4 in across flats, 5/16 in head): https://www.tannerbolt.com/media/akeneo_connector/CapScrewgr2-ss.pdf
- BLS Occupational Outlook Handbook: electricians https://www.bls.gov/ooh/construction-and-extraction/electricians.htm ,
  plumbers, pipefitters and steamfitters https://www.bls.gov/ooh/construction-and-extraction/plumbers-pipefitters-and-steamfitters.htm ,
  HVAC mechanics and installers https://www.bls.gov/ooh/installation-maintenance-and-repair/heating-air-conditioning-and-refrigeration-mechanics-and-installers.htm ;
  OEWS May 2025 employment (BLS API series OEUN000000000000047211101, ...47215201, ...49902101).
- I2RT YAM arm: https://i2rt.com/products/yam-6-dof-arm

## Fidelity details

PhysX at 480 Hz in Isaac Lab-Arena (the station's 240 Hz doubled, per task), both arms driven as in Dream's station
(joint PD with i2rt's gains and gravity compensation). Rigid bodies; collision shapes are boxes and convex hulls.

**Channel.** Unistrut P1000 as GEC17 p24 draws it: 41.3 mm square, 2.67 mm (12 ga) walls and back, top flanges 9.5 mm
wide, returns turned down to 7.1 mm below the top face at the 22.2 mm slot, the bend at the slot edge a 1.8 mm
45 degree bevel (the drawing shows a rounded bend; radius inferred). Fixed colliders, painted steel, friction 0.3. It
is bolted flat to the bench, back down (surface mounting; four anchor bolts and square washers are visual): a raised
stand was tried first and put the work above what the YAM reaches with its fingers pointing down (wrist joint 4 at its
limit above about 16 cm at 30 cm out).

**Spring nut.** P1010 size (19.05 x 34.5 x 12.7 mm; 54 g, Unistrut's 12 lb per 100). Its outline is the rectangle with
the two leading corners for a clockwise turn rounded to 17.6 mm about the centre ("ends shaped to permit a quarter
turn clockwise", GEC17 p10): inside the channel (35.96 mm between the walls) it turns clockwise only, and its two
square corners stop it 4-5 degrees past a quarter turn; anticlockwise it jams at once. Two grooves 4.2 mm wide and
1.6 mm deep across its top (inferred; the catalogs show two toothed grooves, the teeth are not modelled) sit under the
returns; the bottom long edges are bevelled 1.5 mm. The nut fits through the slot only edge-on (19 mm in 22.2 mm; its
34.5 mm length does not). The spring is a prismatic foot under the nut with a spring drive: 31.8 mm free (the
P1010's listed 1-3/4 in height with spring, less the nut), 440 N/m (inferred: about 5 N holds the seated nut up, about
6 N to push it 3 mm below the lips); its foot stands on the channel's back (friction 0.3).

**Bolt and thread.** 1/2-13 x 15/16 in hex head cap screw (ASME B18.2.1: 19.05 mm across flats, 7.9 mm head),
16-sided shank 12.5 mm across flats with its end chamfered to 10.5 mm over 1.2 mm (a pilot point). The nut has a
13.4 mm square hole through it, its mouth countersunk 0.8 mm at 45 degrees (nuts are countersunk). The bolt and the
nut are one articulation (rooted at the nut), joined by a chain of six joints (x, y and z slides, two tilts, the turn
about the bolt's axis) through light links with no colliders; while the bolt is not started every drive of the
chain is off (no stiffness, no damping), so the bolt moves as a free body, colliding with the nut, its spring and the
fitting. A thread rule (`strutnut_sim/task.py`, `_screw`) runs every control step:
- *Not started.* While the bolt's end is over the nut's hole (within 1.45 mm of its axis, tilted less than 20
  degrees), it can sit 0.6 mm into the hole's mouth and no deeper (the first thread has not caught: the thread's
  major diameter does not pass the nut's minor diameter): the z slide's drive holds it there once it is within 0.3 mm
  of that depth (or of the distance it would fall in 1.5 control steps), and keeps it there while its end stays
  within 3.95 mm of the axis (in the countersunk mouth).
- *The catch.* Turned clockwise (seen from the head) 45 degrees relative to the nut while sitting there, the first
  thread catches. If the bolt's axis is then within 5 degrees of the nut's (inferred: a 1/2-13 thread's flanks and
  the chamfer tolerate a few degrees, more and the flanks ride over each other), the thread is started: the lateral and
  tilt drives hold the bolt on the nut's axis and square to it (100 kN/m, 60 N m/rad), and the screw coupling below
  takes over. Tilted more, it **cross-threads**: a hazard (score 0 for the trial); the bolt is then jammed where it is
  (it takes 2 N m to turn further).
- *Started.* The screw is a rule, as in a lead screw: every control step the z slide's drive target is set to the
  catch position less 1.954 mm per turn of the revolute (clockwise from the head runs the bolt in), and its stiff
  drive (200 kN/m) carries the bolt's tension once the head bears on the fitting (F = k x the shortfall). Thread
  friction is a Coulomb drive on the revolute: it holds until the torque exceeds 0.03 N m running (inferred) plus
  0.2 x d x F (the bolt formula, nut factor 0.2), so the free nut below the lips turns with the bolt, and the bolt
  turns in the nut once the nut is stopped; the tighter it is, the harder it turns, and it never turns back by
  itself. Backed out past its start the bolt is free again.

**Fitting.** P1064 square washer, 41.3 mm square, 6.35 mm thick, 77 g (17 lb per 100), with a square 14.3 mm hole
round the 12.7 mm shank (the round 9/16 in hole drawn as a square of the same width). Free rigid body, held on the bolt
only by the head and the nut; friction 0.3 (zinc on paint).

**After-release check.** A 20 N force along the channel, applied at the fitting's centre (ramped over 0.5 s, held to
2 s), as a spring scale would pull it. A seated nut and a bolt run down to hand-snug hold the fitting by friction under
the head, under the fitting and between the nut's grooves and the lips (in the demo the bolt's tension at the end is
about 74 N, about 0.22 N m at the head by the bolt formula); a fitting on an unseated nut or a loose bolt slides.

**What is not modelled or simplified.** The nut's teeth and the lips' bite (they matter at wrench torque, not at
hand-snug); thread geometry (the thread rule above: the catch, the alignment limit and the screw coupling are rules,
not thread flanks in contact); the spring's coils (one sprung foot); a P1026-style fitting with an upright leg: its
leg stands 24 mm from the bolt, closer than the YAM's fingers (26 mm wide at the tips, 40-57 mm a little higher) can
turn a head, so the flat P1064 is used. A loose nut put into the slot by hand and turned with a thumb is the other
common way; the YAM's fingertips (26 mm) do not fit the 22.2 mm slot, so the robot starts the bolt in the nut first
(the bolt is then its handle for the nut), as installers also do when offering a fitting up. The bolt starts dropped
through the fitting's hole rather than separate from it. A dropped bolt cannot lie quite flat (the chain's tilt
joints stop at 80 degrees; it is a hazard by then anyway). One layout.
