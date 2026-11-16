# Find a hidden coolant leak with an inspection mirror and mark the spot above it

A machine housing stands 55 mm from a wall. Coolant has leaked onto the housing's face inside that gap. Nobody can
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
- **The mirror in a tight gap.** The real Ullman HTC-2 head is 57 mm across. Set at 45 degrees it is 43 mm wide in a
  55 mm gap: about 6 mm each side. The arm must hold it level, centred and at the right height up to 16 cm into the
  gap, while the hanging head sags 3 to 6 mm toward the face. If the head touches the housing face it smears the leak trace, and the trial scores 0.
- **Turning what is seen into a place.** The trace shows only in the mirror. The robot must relate where the head was
  inside the gap to a spot on the cover strip outside it, and set the marker tip within 6 mm of it with the other arm.
- **The read itself is trivial.** One bright green trace on a pale grey face. There are no codes or symbols.
- **No direct view.** The five places are 7.5 to 15.5 cm in from the open end, where the inside of the gap is dark.
  A no-mirror control (below) held the empty hand at 45 poses at the entrance and saw no trace.
- **The answer changes every trial.** The trace is at one of five places 20 mm apart, drawn from the episode seed.
  Full credit ends 6 mm from the trace and zero starts at 14 mm, so a neighbouring place earns nothing. One blind mark
  averages 0.2 (measured below). Marking several places scores 0, because the lowest mark counts.

This answers the two points that closed an earlier mirror task (robocurve/benchmark-tasks #1671):
there the hard part was matching a digit and placing a numbered box, and the answer was always the same.

## Setup
Coordinates: x forward from the line between the arm bases, y left from their midpoint; heights above the table top.

| Part | Description | Position |
|---|---|---|
| housing and wall | grey steel housing side and dark wall, 18 x 13 cm, 5 mm steel, 55 mm apart; open at the near end, closed at the far end and the bottom; fixed | x 22.5 to 40.5 cm (18 cm deep), gap y -2.75 to +2.75 cm |
| cover strip | grey steel, 5 mm thick, 65 mm wide, over the housing side, the gap and the wall | top 13.5 cm up |
| leak trace | green coolant film, 22 mm wide, 50 mm tall with a rounded drip end, on the housing's face inside the gap | centre 7.5 cm up, at x 30, 32, 34, 36 or 38 cm, 7.5 to 15.5 cm in from the open end (one per trial) |
| inspection mirror | Ullman HTC-2: 57 mm (2-1/4 in) round glass head on a rim swivel, set at 45 degrees, hanging below the shaft; black vinyl grip 100 mm long, 12.7 mm across (modelled square in section); steel shaft telescoped 120 mm past it (the HTC-2 runs 165 to 924 mm); a small LED added at the head, as on Ullman's lighted HTC-2LT; 90 g | grip on two grey blocks (12 x 30 x 94 mm) at (12.5, -13) and (22.5, -13) cm, its free end (x 11.5 cm) toward the robot |
| paint marker | Sharpie oil-based paint marker, medium point, about 142 x 16 mm, 22 g, yellow paint | standing tip down in its cup |
| pen cup | steel, 34 mm square, 60 mm tall, 18 mm bore, on a 50 mm steel stand; fixed | (19, 9) cm |

- **Start:** both grippers open at the ready pose.
- **Hidden state:** the leak place, from the episode seed. `-E leak=<0..4>` fixes it for checks.

## Scoring
Measured at 20 Hz from the bodies' poses and the robot's measured joints.

- **Paint mark:** where the marker tip comes within 1.5 mm of the cover strip's top face. A dot is drawn there.
- **Per mark:** 1.0 if it is within 6 mm of the trace's centre along the gap, falling linearly to 0 at 14 mm.
- **Value:** the lowest mark credit. 0 with no mark. 0 if the mirror head has touched the housing face inside the gap
  (within 0.5 mm of it), because the head wipes the trace.
- **When it counts:** both grippers at least 80% open, and the mirror and marker still, for 1 s. The latest such state
  counts, so a stray mark made after a full score still lowers it. 1.0 ends the trial.

| Score | Condition | How to check on a real bench |
|---|---|---|
| 1.0 | one or more marks, all within 6 mm of the trace | tape measure from the mark to a plumb line from the trace |
| 0 to 1 | the worst mark 6 to 14 mm off | same, linear |
| 0 | no mark, a mark 14 mm or more off, or the trace smeared | photo of the strip; look for a wiped trace |

## Run a trial
```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e tasks/inspection-mirror
cd tasks/inspection-mirror
python dev/mirror_reach.py                          # every scripted waypoint through the stock IK, no GPU
python -m pytest -q test_scoring.py                 # scoring checks, no GPU
bash dev/queue.sh dev/all_runs.txt                  # the runs in the Results table, one at a time
bash dev/run.sh trial 3600 --policy inspection_mirror_scripted --epochs 5   # seeded: places 3, 1, 3, 0, 1
```
- `dev/all_runs.txt` lists every run in the table: the demo at each place (`-E leak=0..4`), the two margin runs
  (`-P aim_mm=2` and `-2`), `touch`, `idle`, `guess` at each place, and Dream-1.
- **Seeds:** the leak place comes from the trial seed Inspect Robots derives from `--seed` (default 0) and the epoch.
  With the defaults, epochs 0 to 4 get places 3, 1, 3, 0 and 1; a single default run always gets place 3. Run several
  epochs (or vary `--seed`) so a fixed answer cannot score.
- `dev/run.sh` runs `inspect-robots run --embodiment yam_inspection_mirror -E object_jitter=0 -E control_hz=20
  -E cam_width=320 -E cam_height=176 --scorer task_score --store-frames` with the instruction from `task.json`.
- **The demo** reads only the right wrist camera image and the joint angles, plus the fixed setup positions. It never
  reads the leak place. The right hand takes the mirror by its grip from behind, slides the head into the gap and stops
  it at each of the five places. At each stop it counts bright green pixels in the wrist image; the place with the
  most is the leak. Meanwhile the left hand takes the marker out of its cup from the side, at its middle. Once the
  mirror is back on its blocks, it touches the tip onto the strip above that place and puts the marker back.
- **Check runs:** `peek` holds the empty hand (no mirror) at 45 poses at the gap's entrance and counts any green in
  every camera; `guess` skips the mirror and marks the middle place; `touch` sweeps with the head 7 mm toward the
  housing face; `idle` holds both arms; `aim_mm` shifts the mirror path toward the face and the mark along the gap.

## Results
Measured 2026-11-16 UTC on the shared L40S with the settings above, one episode each (the seeded run: five).
Wall time is start to finish, simulator start-up included. "Mark" is the paint mark's distance from the trace's centre
along the gap; "closest" is the mirror head's nearest approach to the hidden face.

| Run | Leak place | Score | Steps (sim time) | Wall | Mark | Closest | What happened |
|---|---|---|---|---|---|---|---|
| demo | 0 | 1.0 | 979 (49 s) | 2.3 min | -0.1 mm | 2.9 mm | green read per place 406 / 92 / 0 / 0 / 0 |
| demo | 1 | 1.0 | 979 (49 s) | 2.3 min | +0.1 mm | 2.9 mm | 154 / 438 / 102 / 0 / 0 |
| demo | 2 | 1.0 | 978 (49 s) | 2.3 min | 0.0 mm | 2.9 mm | 0 / 173 / 441 / 78 / 0 |
| demo | 3 | 1.0 | 981 (49 s) | 2.3 min | +0.1 mm | 2.9 mm | 0 / 0 / 169 / 437 / 40 |
| demo | 4 | 1.0 | 981 (49 s) | 2.3 min | 0.0 mm | 2.9 mm | 0 / 0 / 0 / 169 / 292 |
| demo, default seeds, 5 epochs | 3, 1, 3, 0, 1 | 1.0 x 5 | 966 to 981 each | 10.4 min | -0.8 to +0.1 mm | 2.6 to 3.0 mm | every place read correctly |
| margin: mirror 2 mm toward the face, mark +2 mm | 4 (deepest) | 1.0 | 980 | 2.3 min | +2.0 mm | 0.9 mm | |
| margin: mirror 2 mm toward the wall, mark -2 mm | 4 | 1.0 | 980 | 2.2 min | -2.0 mm | 4.9 mm | |
| `touch` (head 7 mm toward the face) | 2 | 0 | 1,081 | 2.5 min | 0.0 mm | touched | read and marked the right place, but the head rubbed the face: trace smeared |
| `peek` (no mirror, 45 hand poses at the entrance) | 0 (shallowest) | 0 | 2,112 | 5.4 min | none | never in the gap | no trace in any frame; loose green count at most 16 dim pixels (top and left cameras 0), against 290 to 440 bright through the mirror |
| `idle` | 3 (seed) | 0 | 3,600 | 5.9 min | none | never in the gap | nothing done |
| `guess` (no mirror, mark the middle place) | 0 / 1 / 2 / 3 / 4 | 0 / 0 / 1.0 / 0 / 0 | 500 to 599 | 1.2 to 1.4 min | +40 / +20 / 0 / -20 / -40 mm | never in the gap | **mean 0.2** |
| Dream-1 | 3 (seed) | 0 | 3,600 (180 s) | 6.7 min | none | never in the gap | moved both arms and disturbed the mirror on its blocks; no mark |

- Media: `demo.mp4` (top camera, place 2), `demo_wrist.mp4` (right wrist camera: the trace in the mirror),
  `dream_run.mp4`, `setup.png`.

## How the parts are modelled
- Housing, wall, cover strip, floor and pen cup are one fixed body. The leak trace is a thin visual film moved onto the face at
  reset. The mirror and marker are free rigid bodies held only by finger contact (torsional contact patch on).
- The mirror head is an ideal flat reflector drawn by the path tracer. Its swivel is fixed at 45 degrees, as a worker
  sets it before going in. The added LED lights the face and is hidden from direct view (the plain HTC-2 has no
  light; the lighted HTC-2LT has a 60 mm head).
- Paint is a yellow dot placed where the tip met the strip; the tip itself is a rigid felt cylinder.

## Known limits
- The swivel cannot be re-set during a trial.
- The grip is modelled square in section: a round one rolled under the closing fingers in simulation.
- Mirror and marker masses, the grip and shaft diameters are estimates (the product listings give the head size and
  lengths only).
- The housing and wall are fixed to the table.
- With the mirror's light on, a 3-pixel-wide green sliver (the trace's edge) shows directly past the head; it does
  not show where along the gap the trace is.
- The deepest place's hand pose is 0.13 mm and 0.5 degrees outside the stock IK gate; it scored 1.0 in every run.
