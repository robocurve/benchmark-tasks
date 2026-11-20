# Find a hidden coolant leak with an inspection mirror and mark the spot above it

A machine housing stands 60 mm from a wall. Coolant has leaked onto the housing's face inside that gap. Nobody can
see that face directly: from the open end it is seen edge-on, and the gap is covered on top. A maintenance worker
slides a hand inspection mirror into the gap, finds the leak in the reflection, and marks its place on the cover
above it so the repair goes to the right spot.

**Done by:** general maintenance and repair workers, about 1.6 million US jobs, and automotive service technicians
and mechanics, 805,600 jobs (BLS Occupational Outlook Handbook, 2024 figures;
[49-9071](https://www.bls.gov/ooh/installation-maintenance-and-repair/general-maintenance-and-repair-workers.htm),
[49-3023](https://www.bls.gov/ooh/installation-maintenance-and-repair/automotive-service-technicians-and-mechanics.htm)).
O*NET duty lines:
- 49-9071.00: "Inspect, operate, or test machinery or equipment to diagnose machine malfunctions."
- 49-3023.00: "Inspect vehicles for damage and record findings so that necessary repairs can be made."

The task is a task-local plugin for the stock `yam_arena` adapter (Isaac Lab-Arena 0.3.1, Isaac Lab 3.0, Isaac
Sim 6.1, the calibrated `station` scene). It registers the embodiment `yam_inspection_mirror` (the stock robot,
cameras and 14-D joint action and observation, unchanged, plus the housing, mirror, marker and leak) and the policy
`inspection_mirror_scripted` (a scripted demo and the check runs below).

## What is hard, and why a guess does not pay
- **The mirror in a tight gap.** The off-the-shelf Ullman HTC-2LT lighted mirror has a 60 mm head in a battery case.
  Set at 45 degrees it is about 48 mm wide in a 60 mm gap: about 6 mm each side. The arm must hold it level, centred and at the right height up to 16 cm into the
  gap, while the hanging head sags toward the face. If anything touches the housing face anywhere (any part of the
  mirror, including the head's rim at the gap's mouth, the marker, or any part of the robot), the trial scores 0,
  at any moment of the trial: on a real leak, the touch would smear the only evidence of where the leak is.
- **Turning what is seen into a place.** The trace shows only in the mirror. The robot must relate where the head was
  inside the gap to a spot on the cover strip outside it, and set the marker tip within 6 mm of it with the other arm.
- **The read itself is trivial.** One bright green trace on a pale grey face. There are no codes or symbols.
- **No direct view.** The five places are 7.5 to 15.5 cm in from the open end, where the inside of the gap is dark.
  A no-mirror control (an earlier round, below) held the empty hand at 45 poses at the entrance and saw no trace.
- **The answer changes every trial.** The trace is at one of five places 20 mm apart, drawn from the episode seed.
  Full credit ends 6 mm from the trace and zero starts at 14 mm, so a neighbouring place earns nothing. Marking several
  places scores 0, because the lowest mark counts.
- **A mark needs a real look first.** A pause is the mirror head in the gap, held still (under 1 cm/s and 0.1 rad/s)
  for 0.5 s or more. A mark scores only if the last pause before it ended with at least 0.5 s in which a wrist camera
  saw the trace's middle in the mirror's glass. That is checked by reflection geometry from the camera's measured pose
  and lens: the glass side must face both the camera and the trace, the reflection must fall inside the glass and the
  image, and nothing (the housing, the tools, the robot) may block either leg of the view.
  - A blind mark (no mirror) scores 0 at every place.
  - A sweep past every place that ignores the image and marks the middle place scores 0 at every place: its last
    pause is at the far end, where the trace is not (unless it is there, and then its mark is 40 mm off).
  - A pause beside the trace, a pass that does not stop, a wrong-facing mirror or a blocked view earns nothing.
  - A policy can score only by marking the place its last still look showed the trace at. A policy that always ends
    its looks at one fixed place and marks it scores there when the trace is there, because it saw the trace.

## Setup
Coordinates: x forward from the line between the arm bases, y left from their midpoint; heights above the table top.

| Part | Description | Position |
|---|---|---|
| housing and wall | grey steel housing side and dark wall, 18 x 13 cm, 5 mm steel, 60 mm apart; open at the near end, closed at the far end and the bottom; fixed | x 22.5 to 40.5 cm (18 cm deep), gap y -3 to +3 cm |
| hidden face | the housing side's face inside the gap, painted light grey; before each trial the whole face, patch included, gets a light coat of white marking chalk (IRWIN STRAIT-LINE, white) as a contact witness: any touch wipes it and leaves chalk on what touched it | x 22.5 to 40.5 cm, table to strip |
| cover strip | grey steel, 5 mm thick, 70 mm wide, over the housing side, the gap and the wall; on two locating pins, held by two M5 thumb screws, one at each end | top 13.5 cm up |
| leak trace | a dry, printed green patch: 22 x 50 mm cut from a removable matte white paper label sheet (Avery 6465, about 0.1 mm thick) printed solid green on a laser printer, stuck to the hidden face; no liquid; 22 mm wide and 50 mm tall in the simulation too | centre 7.5 cm up, at x 30, 32, 34, 36 or 38 cm, 7.5 to 15.5 cm in from the open end (one per trial) |
| inspection mirror | Ullman HTC-2LT lighted telescoping inspection mirror, off the shelf. Maker's product page (ullmandevices.com, opened 2026-11-19): 2-3/8 in (60 mm) round glass mirror, built-in LED powered by two CR2016 cells in a black plastic case behind the glass, telescopes 10 to 37 in (254 to 940 mm), 4.5 oz (128 g). Light on; head set at 45 degrees on its joint, hanging below the shaft, glass facing the robot and the hidden face; black cushioned grip 100 mm long, 12.7 mm across (modelled square in section); steel shaft out 120 mm past the grip | grip on two support blocks at (12.5, -13) and (22.5, -13) cm, its free end (x 11.5 cm) toward the robot |
| support blocks | two 6061 aluminium bar blocks, 12 x 30 x 94 mm, grey anodised, held to the table with double-sided foam tape; fixed | (12.5, -13) and (22.5, -13) cm |
| paint marker | Sharpie oil-based paint marker, medium point, about 142 x 16 mm, 22 g, yellow paint | standing tip down in its cup |
| pen cup | steel, 34 mm square, 60 mm tall, 18 mm bore, on a 50 mm steel stand; fixed | (19, 9) cm |

- **Start:** both grippers open at the ready pose, pads clean (reset step 7).
- **Hidden state:** the leak place, from the episode seed. `-E leak=<0..4>` fixes it for checks.

### Reset between trials
1. Loosen the two thumb screws and lift the cover strip straight up off its pins. The gap is now open from above.
2. Off the rig, wipe the strip's paint marks off with a rag and mineral spirits and let it dry.
3. Brush the old chalk off the face with a dry, soft 25 mm brush and wipe it with a dry microfibre cloth. No liquid
   goes on the rig.
4. Peel off the old patch and stick a fresh one at the place drawn for the trial: centred at that x and 7.5 cm up.
   Five scribe lines on the housing's top edge, hidden under the strip, mark the places. The patch is dry, so there is
   no time limit between reset and trial.
5. Lay a sheet of paper on the table under the gap. Dust the whole face, patch included, with a light coat of the
   chalk from the brush. Tap the housing once to drop loose dust, then take the paper away.
6. Photograph the face square-on (the reference photo; the green must show clearly). Put the strip back on its pins
   and tighten the screws.
7. Gripper pads, mirror and marker:
   - Kept clear: chalk is only on the face inside the gap, put on with the strip off and the robot at its ready pose,
     so a pad meets chalk only by touching the face, which scores 0. The patch is dry; the only wet paint is the
     marker's tip and its marks, and the pads hold the barrel about 75 mm above the tip.
   - Cleaned before every trial: wipe each pad, the mirror and the marker barrel with a dry microfibre cloth, then
     with a lint-free wipe damp with 70% isopropyl alcohol. Let them dry for 1 minute.
   - Checked: press each pad on a black card. No white (chalk) or yellow (paint) mark may show; if one does, clean
     again.
8. Lay the mirror back on its two blocks (grip toward the robot, head set at 45 degrees, light on). Stand the marker
   tip down in its cup.

## Scoring
Measured at 20 Hz from the bodies' poses, the robot's link poses (with its collision shapes) and the wrist cameras'
poses and lenses. The same rules apply on a real bench; the right-hand column of the table says how to check each.

- **Contact with the hidden face:** if anything touches the face anywhere on it (x 22.5 to 40.5 cm, from the table
  to the strip): any part of the mirror (the head's glass, back and rim, its joint, shaft or grip, wherever the
  head's centre is), the marker, or any part of the robot. Touching means within 0.25 mm: the simulator holds a resting
  contact 0.06 to 0.10 mm apart. The score is then 0 for the rest of the trial, even after an earlier scored rest.
- **Paint mark:** where the marker tip touches the cover strip's top face (within 0.25 mm, as above). A tip held
  1 mm above the strip makes no mark. A touch more than 3 mm from every earlier mark makes a new mark. A dot is drawn
  there.
- **Pause:** the mirror head's centre in the gap and the mirror held still: its head moves under 1 cm/s and it turns
  under 0.1 rad/s (under 5 mm and 3 degrees in 0.5 s), without a break for 0.5 s or more.
- **Seen:** a wrist camera sees the trace's middle (16 x 30 mm: its centre, and points 8 mm to each side and 15 mm
  above and below) in the mirror's glass. Each point's reflection must fall at least 1.5 mm inside the glass's edge
  and inside the camera image, with the glass facing both the camera and the trace, and nothing (the housing, the
  mirror's grip and shaft, the marker, the robot's wrist and fingers) between the camera and the glass or between the
  glass and the trace.
- **Look:** a mark counts only if the last pause before it ended with the trace seen for 0.5 s without a break. A
  mark with no pause before it, or after a last pause that did not show the trace, earns 0. A look after the mark does
  not help.
- **Per mark:** 0 without a look. Else 1.0 if it is within 6 mm of the trace's centre along the gap, falling linearly to
  0 at 14 mm.
- **Value:** the lowest mark credit. 0 with no mark.
- **Put down:** each tool still (under 1 cm/s and 0.1 rad/s), released (no part of the robot within 5 mm of it, so
  only the table, its blocks or its cup can hold it still), and not on the housing; both grippers at least 80% open.
- **When it counts:** the score takes the value when the put-down state has held for 1 s. A new mark lowers the score
  at once, to the new value if that is lower. 1.0 ends the trial.

| Score | Condition | How to check on a real bench |
|---|---|---|
| 1.0 | one or more marks, each after a look, all within 6 mm of the trace; tools put down; no contact | tape measure from the mark to a plumb line from the trace; footage and photos as below |
| 0 to 1 | the worst mark 6 to 14 mm off, each after a look | same, linear |
| 0 | no mark, or a mark 14 mm or more off | photo of the strip |
| 0 | contact with the hidden face | lift the strip after the trial and photograph the face: any wipe in the chalk (on the face or on the patch) that the reference photo does not show, or chalk on the mirror, the marker or the robot's finger pads (pressed on a black card), is a contact |
| 0 | no look: in the wrist camera footage, the last time the mirror was held still in the gap before the mark, the trace's middle was not inside the mirror's glass for 0.5 s | play back the wrist footage up to the mark; held still: the trace's image in the mirror shifts by less than a quarter of its own 22 mm width over that 0.5 s |
| 0 | the tools not put down | top camera footage: each tool lies still for 1 s, off the housing, with no part of the robot touching it |

## Run a trial
```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e tasks/inspection-mirror
cd tasks/inspection-mirror
python dev/mirror_reach.py                          # every scripted waypoint through the stock IK, no GPU
python -m pytest -q test_scoring.py                 # scoring checks, no GPU
bash dev/queue.sh dev/all_runs.txt                  # every scripted check run, one at a time
bash dev/run.sh trial 3600 --policy inspection_mirror_scripted --epochs 5   # seeded: places 3, 1, 3, 0, 1
FRAMES=1 bash dev/run.sh dream 3600 --policy dream -P server_url=<Dream-1 server> --seed 0   # the Dream-1 run, with video
```
- `dev/all_runs.txt` lists every scripted run in the table: the demo at each place (`-E leak=0..4`), the two margin
  runs (`-P aim_mm=2` and `-2`), `touch`, `edge`, `hover`, `idle`, `guess` and `sweepguess` at each place, and the
  seeded run.
- **Seeds:** the leak place comes from the trial seed Inspect Robots derives from `--seed` (default 0) and the epoch.
  With the defaults, epochs 0 to 4 get places 3, 1, 3, 0 and 1; a single default run always gets place 3. Run several
  epochs (or vary `--seed`) so a fixed answer cannot score.
- `dev/run.sh` runs `inspect-robots run --embodiment yam_inspection_mirror -E object_jitter=0 -E control_hz=20
  -E cam_width=320 -E cam_height=176 --scorer task_score` with the instruction from `task.json`; `FRAMES=1` adds
  `--store-frames` and builds the video.
  The telemetry file it writes logs, per step, every contact clearance, the pause and seen flags, the tools' poses,
  the wrist cameras' poses and the put-down state.
- **The demo** finds the leak from the right wrist camera image only, plus the joint angles and the fixed setup
  positions. It never reads the leak place. Its one read of the simulator is the marker's pose, over the cup before the
  put-back (see below). The right hand takes the mirror by its grip from behind, moves it back and across to line it
  up with the gap (never past the housing's near corner), slides the head in and stops it at each of the five places.
  At each stop it counts bright green pixels in the wrist image; the place with the most is the leak. It goes back to
  that place and holds the mirror still on the trace for 1.5 s, then withdraws it and lays it back on its blocks.
  Meanwhile the left hand takes the marker out of its cup from the side, at its middle. Once the mirror is back on its
  blocks, it touches the tip onto the strip above the place (aimed 1 mm into it). To put the marker back, it holds it
  8 mm above the cup's rim, reads the marker's pose from the simulator and turns and shifts the hand so the barrel
  stands upright on the bore's centre line (twice). Then it lowers it straight down and lets go 4 mm above the floor.
  The grasp holds the marker 2 to 4 degrees off upright, and the 18 mm bore leaves 1 mm round the 16 mm barrel.
  Without this step the barrel landed on the rim (on seed 2 it fell over).
- **Check runs:** `peek` holds the empty hand (no mirror) at 45 poses at the gap's entrance and counts any green in
  every camera; `guess` skips the mirror and marks the middle place; `sweepguess` sweeps the mirror past every place
  (stopping at each), ignores the image, withdraws and marks the middle place; `touch` sweeps with the head 7 mm
  toward the housing face; `edge` first presses the head's rim onto the face's near edge at the gap's mouth, its
  centre still outside the gap, then runs the demo; `hover` runs the demo but aims the marker tip 3 mm above the
  strip instead of touching it; `idle` holds both arms; `aim_mm` shifts the mirror path toward the face and the mark
  along the gap.

## Results
Measured 2026-11-20 UTC on the shared L40S with the settings above, one episode each, all on the final code. Five
other simulations shared the GPU, so wall times are about three times the unloaded ones. Every run's log records
environment revision afe6a9019e63, the hash of the submitted task file. "Read" is the demo's green pixel count at
the five stops. "Mark" is the paint mark's distance from the trace's centre along the gap. "Closest" is the nearest
approach of anything (mirror, marker, robot) to the hidden face.

| Run | Seed / leak place | Score | Steps | Wall | Mark | Closest | What happened |
|---|---|---|---|---|---|---|---|
| demo | seed 2 / 1 | 1.0 | 1,283 | 13 min | +0.1 mm | 1.9 mm | read 86 / 437 / 147 / 0 / 0; looked at place 1, marked, put both tools back (marker seated in its cup) |
| demo | seed 0 / 3 | 1.0 | 1,283 | 14 min | +0.2 mm | 1.9 mm | read 0 / 0 / 89 / 440 / 92 |
| demo | seed 1 / 2 | 1.0 | 1,283 | 12 min (frames stored) | 0.0 mm | 1.9 mm | read 0 / 87 / 447 / 142 / 0 |
| `touch` (head 7 mm toward the face) | place 2 | 0 | 1,398 | 13 min | -39.9 mm | touched (mirror) | the head rubbed the face: 0 from then on |
| `guess` (no mirror, mark the middle place) | place 2 | 0 | 737 | 7 min | +0.3 mm | 25.2 mm (marker) | the mark is on the trace and the tools were put down, but there was no look: 0 |
| `sweepguess` (sweep, ignore the image, mark the middle place) | place 4 | 0 | 1,315 | 12 min | -40.0 mm | 1.9 mm | looked at every place, but the mark is 40 mm from the trace: 0 |
| Dream-1 (the model policy, full run limit) | seed 0 | 0 | 3,600 | 79 min | - | - | never looked at the face or marked it; the tools were not put down |

Earlier rounds ran the other checks (the demo at every place and on five seeds, the two 2 mm margin runs, `edge`,
`hover`, `guess` and `sweepguess` at every place, `peek` and `idle`) on the same scorer with the plain HTC-2 mirror
(57 mm head, 90 g) in a 55 mm gap: every demo and margin run scored 1.0 and every other run 0. This round changed
only the mirror's size and mass, the gap's width (to keep about 6 mm each side), the trace's material and the demo's
dab and marker put-back (above). The scorer is unchanged.

- Media: `demo.mp4` (seed 1, place 2: top camera and right wrist camera side by side; the wrist view shows the trace in
  the mirror at the stop and again at the held look before the mark), `model_run.mp4` (Dream-1, seed 0: top and right wrist cameras),
  `setup.png`.

## How the parts are modelled
- Housing, wall, cover strip, floor and pen cup are one fixed body. The support blocks are fixed boxes. The leak trace
  is a thin visual film (22 x 50 mm) moved onto the face at reset. The mirror and marker are free rigid bodies held
  only by finger contact (torsional contact patch on).
- The mirror head is an ideal flat reflector drawn by the path tracer on its glass side only; its back (the
  black battery case) is matte, so a head turned the wrong way shows no reflection. Its joint is fixed at 45 degrees,
  as a worker sets it before going in. The HTC-2LT's built-in LED lights the face and is hidden from direct view.
- Paint is a yellow dot placed where the tip touched the strip; the tip itself is a rigid felt cylinder.
- Contact and release use exact distances between whole convex solids, not sampled points: the mirror as a 60.3 x 8 mm
  head cylinder, a shaft cylinder, a grip box and a joint box; the marker as two cylinders; each collision piece of
  the robot's forearm, wrist, gripper and fingers as its bounding box; and the bounded hidden face as a rectangle. So a
  flat finger face under a tool, or the head's rim at the gap's mouth, is measured where it really is closest.
- The look check uses each wrist camera's pose (its mount on the gripper link) and lens, and treats the robot's
  wrist, gripper and fingers as their collision pieces' bounding boxes when checking for a blocked view.

## Known limits
- The joint cannot be re-set during a trial.
- The grip is modelled square in section: a round one rolled under the closing fingers in simulation.
- The mirror's mass, head size and lengths are the maker's published figures. The head's thickness with its battery
  case (8 mm), the grip and shaft diameters and the marker's mass are estimates.
- The housing and wall are fixed to the table.
- The look check is geometry, not image analysis: it does not model glare, focus or the LED's falloff. Its predicted
  trace position matched the rendered wrist image to within about 4 mm (checked on a stored frame).
- With the mirror's light on, a 3-pixel-wide green sliver (the trace's edge) shows directly past the head; it does
  not show where along the gap the trace is, and it never counts as a look.
- The deepest place's hand pose is 0.03 mm and 0.11 degrees outside the stock IK gate; it scored 1.0 in every run.

## Fidelity (full)

The mirror is an ideal flat reflector drawn by the path tracer on its glass side only (the back, its battery case, is matte); its joint is fixed at 45 degrees; the HTC-2LT's built-in LED lights the face. The grip is modelled square in section. The trace is a 22 x 50 mm green film, as the dry printed patch is. Contact and release are checked as exact distances between whole convex solids (the mirror's head, shaft, grip and swivel, the marker, and each robot collision piece as its bounding box) and the bounded hidden face, touching meaning within 0.25 mm (the simulator holds resting contacts 0.06 to 0.10 mm apart); on a real bench the chalk witness shows any touch. The look is checked by reflection geometry from the wrist cameras' measured poses and lenses (no glare or focus model). Paint is a dot drawn where the tip touched the strip. Housing, wall, blocks and cup are fixed. The mirror's mass is the maker's published 4.5 oz; the head's case thickness and the marker's mass are estimates.
