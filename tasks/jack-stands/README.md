# Jack up a heavy beam, set it on two stands, then reach under

A mechanic who must work under a car lifts it with a floor jack, sets jack stands under the support points, lowers
the car onto the stands, and only then goes under it. Quoted from the owner's manual of Harbor Freight's 2.5-ton
floor jack, item 64543 (2021,
[64543-792363645430.pdf](https://manuals.harborfreight.com/manuals/64000-64999/64543-792363645430.pdf)):

> "Carefully position the Jack's Saddle under the vehicle manufacturer's recommended lifting point."
> "Set the jack stands to the same height according to the manufacturer's instructions, making sure they lock
> securely into position." "Slowly twist the Handle counterclockwise to lower the vehicle onto the jack stands'
> saddles." "Never work on, under or around a load supported only by this device."

Done by: automotive service technicians and mechanics (O*NET 49-3023.00).

Here the car is a tabletop trainer: a 23 kg solid-steel beam (an axle stand-in) on two end blocks, far too heavy for
an arm. A small floor jack is fixed under its lift point. One arm pumps the jack lever until the beam clears the
stands. Both arms slide a stand each under the support marks. The other arm turns the release knob, and the beam
settles onto the stands. Then the first arm pulls the wrench out from under the beam. The safety rule is scored, not
just the end state: a hand near or under the beam, or the wrench disturbed, while the beam is not on both stands,
zeroes the trial.

The task is a task-local plugin for the stock `yam_arena` adapter (Isaac Lab-Arena 0.3.1, Isaac Lab 3.0, Isaac
Sim 6.1, the calibrated `station` scene). It registers:

- the embodiment `yam_jack_stands`: the stock robot, cameras and 14-D joint action and observation, unchanged, plus
  the trainer's bodies and joints, scored from the measured part poses;
- the policy `jack_stands_scripted`: a privileged scripted demonstration, plus the failure paths below.

## Setup
Coordinates: x forward from the line between the arm bases, y left from their midpoint; heights above the table top.

| Part | Description | Position |
|---|---|---|
| beam (the load) | grey solid steel, 10 x 40 x 6 cm, with two dark end blocks 10 x 5 x 5 cm under its ends; one rigid body, 23 kg | centre (36, 0) cm; underside 5 cm high at the start |
| marks | yellow lift point (2 cm wide) at y 0; cyan support points (3 cm wide) at y +-12 cm; on the top and the near face | on the beam |
| jack body | blue painted steel, 12 x 9 x 3 cm, fixed | (36, 0) cm, under the lift point |
| jack saddle | black steel plate 8 x 8 x 1.5 cm, 0.5 kg, on a chrome ram 3 cm across (visual only: it shows as the saddle rises); travel 8.5 cm | top 3 mm below the beam at the start |
| pump lever | dark steel bar 8 x 1.4 x 1 cm, 0.08 kg, on a pivot between two fixed blue steel cheeks (4.5 x 0.6 x 4.8 cm, at y +-1.2 cm, from x 25.5 cm to the jack body), green rubber-faced pad 3 x 4 x 1.2 cm at the tip; spring holds it up; 30 degrees of stroke, 5.6 N at the pad fully pressed | pivot (26.5, 0) cm, 3.8 cm high; pad about (20, 0) cm, top 7.9 cm high at rest, 3.4 cm outside the crush zone |
| release knob | yellow plastic, 0.05 kg: wing 4.5 x 1 x 2.5 cm on a hub 2.4 cm across and 0.8 cm tall (wing top 7.9 cm high), on a fixed dark steel post (2.4 cm across, 4.5 cm tall); turns counterclockwise from 0 to 90 degrees | (20, -7) cm |
| 2 jack stands | red steel, 1.2 kg, fixed height (top 8 cm): 19 x 4 x 1 cm skid, 2.5 x 2.5 cm column, 6 x 3.5 cm top plate, dark-red grip block 3 x 2 x 4 cm on the skid's near end, 14 cm in front of the column | columns at (26, +-12) cm, 10 cm in front of the support marks |
| wrench (the work) | orange steel, 0.32 kg: 3 x 2 x 4 cm grip block and a 9 x 1.6 x 1.2 cm bar | grip block at (26.5, 6.5) cm, 4.5 cm in front of the beam's near edge; the bar under the beam |

- **The jack:** each full stroke of the lever (down past 0 degrees, then back up past 20) raises the saddle 7 mm,
  ramped in over a quarter second. The release knob opens the valve from 10 degrees and fully at 50; the saddle then
  lowers at up to 12 mm/s, and pumping does nothing. 8 strokes take the beam from 5 cm to about 10 cm, 2 cm above
  the stands' tops.
- **Heights:** the stands (8 cm) can't go under the beam at the start (5 cm); on the stands the end blocks hang 3 cm
  above the table and the saddle drops 3.3 cm clear.
- **Friction** (static/dynamic; PhysX averages the two surfaces in contact): beam and wrench 0.6/0.5, stands 0.4/0.35,
  saddle, lever and knob 1.0/0.9, the fixed jack body, cheeks and post 1.2/1.0 (the task-file default), table
  0.5/0.5 (no material set: PhysX's default).
- **Start:** both grippers open at the ready pose. One fixed layout; nothing is randomized.

## Scoring
Measured from part poses at 20 Hz (on a real table: a ruler against the beam, the stands and the wrench, or a
photo from the side and from above).

| Score | Condition | How to check on a real table |
|---|---|---|
| 0.25 | the jack holds the beam raised: its underside at least 5 mm above the stands' tops, the saddle within 4 mm of the underside, the beam at rest (under 1 cm/s) | side photo with a ruler: underside height |
| 0.5 | both stands upright (under 5 degrees, base on the table), each top within 2 cm of the beam's centre line and 1.5 cm of its cyan mark, while the jack still holds the beam as at 0.25; beam and stands at rest (under 1 cm/s) | top photo: each stand column on its mark |
| 0.75 | the beam resting on both stands (underside within 1.5 mm of each stand's top), the saddle at least 5 mm below it, the beam level within 2 degrees, at rest for 0.5 s | side photo: gaps at both stands and at the saddle |
| 1.0, success, the trial ends | then the wrench fully out (every corner at least 1 cm outside the beam's footprint) and still, the beam still on both stands, for 1 s | top photo: the wrench clear of the beam |
| 0, latched for the trial | while the beam is not resting on both stands (held by the jack even 2 mm above a stand counts as not resting): a gripper's tool point, finger tip or wrist point in the crush zone (the beam's footprint grown by 6 cm on every side, below its underside), or the wrench moved more than 2 cm from where it was when the beam last rested on both stands (from its start before that); at any time: the beam tilted over 10 degrees, falling faster than 0.25 m/s, or pushed more than 3 cm sideways; or a non-finite state | video: the hand near the beam, or the wrench moving, before both stands carry it |
| otherwise | the best stage reached | |

- **Every outcome:** nothing done, 0; beam raised, 0.25; stands set under it, 0.5; lowered onto them, 0.75; then the
  wrench out, 1.0. Each stage counts only at rest, never while passing through a band.
- **No credit for touching:** every stage is a measured position of the beam, the stands and the saddle. Pressing the
  lever, touching a stand or turning the knob earns nothing by itself.
- **Unintended strategies:** the beam is 23 kg and deeper (10 cm) than the gripper opens (9.4 cm), so it can't be
  gripped; lifting or propping it means getting a hand under it, which is in the crush zone and scores 0. Pressing
  or pushing it from above or from the side stays outside the zone but earns nothing: no stage moves the beam by
  hand, and pushing it more than 3 cm, or tipping it past 10 degrees off the saddle, scores 0. Pulling the wrench before the
  beam is on the stands (even at the start, with the beam on its end blocks) scores 0. Lowering the beam without the
  stands keeps only the 0.25 (the `nostands` run). Stands knocked over or left off their marks never reach 0.75.
- **The crush zone:** the hand rule is measured from four points per gripper: the tool point, two finger tips (the
  tool point moved 13 mm along the fingers, then half the measured finger gap either way) and a wrist point 9 cm back
  along the fingers (the housing's top is 7 cm back), so a tilted gripper can't get its wrist under the edge. The zone reaches 6 cm past the
  beam's edges, where the beam would land if it slipped. The stand grips (9 cm out), the lever pad and the knob are
  outside it; the wrench's grip block is inside it, so it can be taken only once the stands carry the beam.
- **The wrench rule** counts only movement while the beam is off the stands: it measures the wrench from where it
  was when the beam last rested on both stands. A wrench taken out correctly is never held against a later
  re-pump or a stand nudged loose.
- The scorer is pure Python, in the package's scoring module; the jack's stroke logic is in its design module.

## Run a trial
Install the pinned adapter, which installs Isaac Lab-Arena and Inspect Robots, then this package into the same
environment. From the repository root:

```bash
mkdir -p .deps
git clone https://github.com/kairosiann/yam-isaac-lab-arena.git .deps/yam-isaac-lab-arena
git -C .deps/yam-isaac-lab-arena checkout 0059d02a1a91bc6834c1d0846022891df0c78730
ARENA_DIR="$PWD/.deps/IsaacLab-Arena" .deps/yam-isaac-lab-arena/setup.sh
source .deps/IsaacLab-Arena/yam.env
uv pip install --no-deps -e tasks/jack-stands
```

On the Robocurve launchable the environment already exists: `source /data/IsaacLab-Arena/yam.env`, then
`uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e tasks/jack-stands`.

Run from `tasks/jack-stands`. Every run below uses the same settings; only `--policy` and `-P` change:

```bash
cd tasks/jack-stands
INSTR="$(python3 -c 'import json; print(json.load(open("task.json"))["instruction"])')"
COMMON=(--embodiment yam_jack_stands -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176
        --instruction "$INSTR" --scorer task_score --max-steps 2400 --store-frames)
# add -E telemetry=runs/<name>/telemetry.jsonl to any run for the per-step trace below

# The scripted demonstration (full score)
inspect-robots run "${COMMON[@]}" --policy jack_stands_scripted --log-dir runs/demo
# ... and two episodes with a reset between them
inspect-robots run "${COMMON[@]}" --policy jack_stands_scripted --epochs 2 --log-dir runs/epochs2

# Dream-1 (needs a Dream API key in ~/.dreamlabs/credentials)
inspect-robots run "${COMMON[@]}" --policy dream --log-dir runs/dream

# Failure paths
inspect-robots run "${COMMON[@]}" --policy jack_stands_scripted -P mode=reach --log-dir runs/reach
inspect-robots run "${COMMON[@]}" --policy jack_stands_scripted -P mode=nostands --log-dir runs/nostands

# One MP4 per camera, next to each run's frames
for log in runs/*/*.json; do inspect-robots video "$log"; done
```

- The embodiment loads its own task file (`jack_stands_task.py`: the fixed jack body, lever cheeks and knob post) and
  always uses a fixed layout.
- `-E telemetry=<file.jsonl>` writes a per-step trace: lever and knob angles, jack target, underside height, saddle
  and stand gaps, stand offsets from their marks, wrench clearance, hand-in-zone flags and score.
- The run prints `task_score` at the end: 1.0 on success, otherwise the best stage (0 after a violation).
- Policies: `-P mode=reach` pumps the beam up, then reaches for the wrench while the beam is on the jack only;
  `-P mode=nostands` pumps it up, then opens the release without setting the stands; `-P mode=idle` holds both arms
  still.
- `python tools/replay_scorer.py runs/<run>/telemetry.jsonl` (CPU only, same environment) replays a run's trace
  through the scorer and checks two probe states: the jack holding the beam 3 mm above both stands with a hand in
  the zone (must score 0), and a wrench taken out on the stands before the beam is lifted off them (keeps 0.75).

## Results
Final scene and scorer, measured on 2026-11-06 (UTC), one L40S shared with other builders' simulations, the settings
and instruction above:

| Run | Score | Steps | What happened |
|---|---|---|---|
| scripted demo, episode 1 | 1.0 | 1,020 (51.0 s) | 9 strokes: underside 106.8 mm, 26.8 mm above the stands; stands 0.9 and 1.3 mm across, 2.4 and 1.3 mm along the beam from their marks; beam on both stands, level within 0.02 degrees, saddle 33 mm clear; wrench 3.6 cm clear |
| scripted demo, episode 2 (after a reset) | 1.0 | 1,038 (51.9 s) | stands 0.1 and 1.1 mm across, 2.3 and 1.3 mm along; otherwise the same |
| `-P mode=reach` | 0.0 | 608 | beam raised on the jack only (0.25 reached), then the left gripper went in for the wrench: the hand rule fired at 19.8 s, before the wrench moved |
| `-P mode=nostands` | 0.25 | 664 | beam raised, then lowered back onto its end blocks without stands: only the 0.25 is kept |
| Dream-1 | 0.0 | 2,400 (the limit) | nothing was operated: the lever stayed within 0.02 degrees of rest, the knob at 0, the wrench moved under 1 micrometre; no gripper entered the crush zone |

- **Runtime per trial:** 51 s of simulated time for the demo, 120 s for a full 2,400-step run. Wall time on the
  shared L40S, including Isaac Sim's start: the two-episode demo 7 min 1 s, `reach` 1 min 40 s, `nostands`
  1 min 29 s, Dream-1 4 min 35 s.
- **Seeds:** one fixed layout; the seed changes nothing.
- **Reset:** a two-episode demo run (`--epochs 2`) scored 1.0 in both episodes; every part's start pose after the
  second reset matched the first within 0.08 mm.
- **Why the demo corrects its slide:** the arm sags behind a dragged stand (about 3 cm along, 1 cm sideways), so the
  demo adds an integral of the stand's measured position error to its command. In the first runs (2026-11-05),
  without it and with a light grip, the stands stopped 38 to 65 mm short, the beam came down on their edges and
  rolled off: scored 0 (beam tipped), as it should. With the correction along the slide only, they ended 7.5 to
  9.3 mm sideways from their marks (the beam itself stayed within 0.02 mm); the sideways correction brought that
  to 2.4 mm.
- **Scorer replay:** every run recorded for this task (16 episodes, including those on the earlier lever position
  and scorer) replays through the final scorer to the score it logged. Probes: the jack holding the beam 3 mm above
  both stands with a hand in the zone scores 0 (the resting tolerance is 1.5 mm); a wrench taken out on the stands
  and then the beam lifted 2 mm off them keeps 0.75, with no violation.
- **Not probed:** a two-arm attempt to lift the beam by hand. The beam is 10 cm deep, more than the gripper opens
  (9.4 cm), and its end blocks can only be reached from under the beam, so any such attempt puts a finger in the
  crush zone and scores 0.

## Simulation notes and limits
- **Joints:** the saddle (prismatic, with a stiff position drive: 3e5 N/m, so the 23 kg beam sags it about 1 mm),
  the lever (revolute, limits -5 to 25 degrees, a spring drive) and the knob (revolute, limits 0 to 90 degrees, a
  little damping) are native PhysX joints outside the robot's articulation. They are disabled while a reset writes
  the start layout and enabled afterwards; the saddle's drive target is set back to the bottom before that.
- **Hydraulics in code:** the saddle's target height comes from the measured lever and knob angles (strokes counted
  only on full travel both ways). The beam rides on the saddle and on the stands by contact only; nothing is
  teleported or locked in software.
- **Simplified:** the jack is fixed under the lift point (the "position the saddle" step is done), the stands are
  fixed height with no lock, rigid primitives, nominal masses, the frictions listed above. Physics at 240 Hz, control and scoring at
  20 Hz.
- **The demonstration is privileged:** it reads the measured underside height to know when to stop pumping, and the
  stand gaps to know when to let go of the knob, the way a mechanic watches the car.

## Real build and safety
- **A scaled trainer:** every part is a tabletop stand-in (beam 40 cm, stands 8 cm, jack travel 8.5 cm), not a car
  part, and can be cut from steel bar and plate; the jack can be a small hydraulic bottle jack with a lever and a
  release screw, or a screw lift.
- **On real arms:** a 23 kg steel beam over a working gripper is a real crush hazard. Run real trials with a lighter
  beam (aluminium or a hollow section, a few kg) marked with the same lift and support points, or inside a guarded
  fixture whose fixed end stops keep the beam from falling more than a few millimetres if a stand is missed. The
  scored behaviour (order of work, hands out of the zone) does not depend on the mass.
