# Set a wind-up metronome to 60 beats per minute

A mechanical pyramid metronome (22.5 cm tall, 500 g, front cover off, spring wound) stands on the table in front of the
arms with its scale facing them. Its pendulum is out of its storage holder and stands at rest, upright. The brass sliding weight on the pendulum rod is on
the 120 line of the 40-208 Maelzel scale. The instruction states only the goal: *Set the metronome to 60 beats per
minute and start it ticking.*

Setting a metronome before practice is routine for music students and teachers. Several things make it hard for a
robot:

- **The weight sits on a free pendulum.** As on a real Maelzel metronome, the rod is notched at every mark, and the
  weight's sprung tooth clicks from notch to notch only when it is pushed along the rod with about 3 N. A push with any sideways component swings the pendulum instead. The hand has to push the weight along the
  rod while keeping the rod upright, for example by pinching the weight across its sides.
- **Millimetre placement.** Near 60 the scale lines are 1-2 mm apart (58, 60 and 63 are at 117.6, 116.5 and 114.9 mm
  above the pivot). The weight has to click into the 60 notch, not the 58 or 63 one next to it, after a
  slide of 34 mm over 16 notches.
- **A clean start.** The pendulum has to be swung well to one side (every swing in the scored 5 s must reach
  15 degrees) and let go. The hand must then stay clear: nothing may come within 5 cm of the metronome for the 5 s over
  which the tempo is measured. Knocking the
  metronome off its place, or tipping it, scores 0.

The tempo is not a scripted number. It comes from the pendulum's physics: gravity on a jointed rod with a fixed
weight below the pivot and the sliding weight above it. Moving the weight up slows the pendulum. The scale lines are
drawn where the simulated pendulum ticks at each tempo.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the metronome, its pivot and clip joints, the scale computed from the pendulum's physics, `success()` and `score()` |
| `metkit.py` | Rigid bodies made of several shapes with explicit mass properties, and D6 joints between them (adapted from the bodykit module used by other tasks in this repository) |
| `demo.py` | The scripted demo |
| `demo.mp4`, `dream.mp4`, `setup.png` | The scripted demo and the Dream-1 run (top camera), and the starting layout |
| `test_scoring.py` | Offline checks (no GPU): the scale against an integration of the pendulum, every scoring outcome, attempts to score without doing the task, the runtime's ending, the demo's reach, and that `task.json` states the numbers in the code |

`metkit.py` hooks `yam_arena.scene` and the embodiment's reset and step without modifying yam_arena. The task
therefore runs with the standard `yam_arena` embodiment and the `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
INSTRUCTION=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/metronome-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. The layout is
fixed (`object_jitter=0`) and there are no seeds. A trial is at most 1,600 control steps (80 s). Set `MET_DEBUG=1` to
print the following every 5 steps: the swing angle, the weight's top edge, the hands' clearance, the case's tilt and
slide, and the centre passes, smallest swing and measured tempo over the window.

## Scripted demo

```bash
python demo.py task.py demo_out --max-steps 1600 --cam 640x352   # demo_*_cam.mp4, setup.png, result.json
DEMO_CALIBRATE=1 python demo.py task.py cal_out --no-record      # prints the CAL numbers (see below)
PYTHONDONTWRITEBYTECODE=1 python -m pytest -p no:cacheprovider test_scoring.py   # offline checks
```

1. The left arm points its open fingers at the scale and pinches the sliding weight across its sides, level with its
   middle. The rod runs between the pads.
2. It slides the weight straight up the rod from the 120 notch, clicking it over the ridges, and stops a notch or so
   short of 60. The pinch also holds the pendulum upright while the weight is pushed.
3. It eases the hand back to where the weight is, so that the compliant arm stops pulling on it. Then it reads the
   scale. While the weight is not in the 60 notch, it pushes it slowly toward 60. As soon as the weight passes a
   ridge's peak, it pulls the hand back, so that the arm's stored push does not carry the weight over the next ridge
   too. In the recorded demo this took one click, from 63 to 60.
4. It lets go, draws back and closes its fingers.
5. It puts the outside of its fingertips against the weight's left side and swings the pendulum 40 degrees to the
   right, along the pendulum's arc.
6. It pulls the hand straight back toward the arms, out of the pendulum's plane, so the pendulum swings on its own.
   Then it goes back to its ready pose.

The demo is open loop except for step 3. There it reads where the weight is from the simulator, as a person reads the
scale. The arm is compliant: it takes about 9 mm of commanded travel to push with the 3 N click force. So whether an
open-loop slide stops in the 60 notch or in the next one (1.1-1.6 mm away) is too sensitive to script. The open-loop
slide's overshoot (`CAL` in `demo.py`) is measured with `DEMO_CALIBRATE=1`.
`result.json` reports `score()` on the last step as the trial's score.

## Scoring

Evaluated every control step. The score at the end of the episode counts (`SCORE_OVER = "end"`).

- **Ticking on its own**: over the last 5 s (100 control steps), no vertex of either arm's hand or wrist collision hulls
  (links 4 and 5, the gripper and both fingers) came within 5 cm of the metronome (its case's bounding box, its rod and
  its sliding weight). In that time the pendulum passed the centre at least four times, and every swing between two
  passes reached at least 15 degrees from upright.
- **At 60**: the weight is in the 60 notch, with its top edge on the 60 line. In simulation this means the 60 line is
  the line nearest the weight's top edge; a weight let go between notches clicks into the nearest one.
- **Near 60**: the weight is in a notch from 52 to 69.
- **1.0**: at 60 and ticking on its own, at a tempo of 56-64 per minute measured from the times of those centre passes
  (interpolated between steps).
- **0.75**: ticking on its own with the weight near 60 (and not 1.0).
- **0.5**: at 60 and not ticking on its own.
- **0.25**: ticking on its own with the weight anywhere else, or near 60 and not ticking on its own.
- **0**: anything else. The score is also 0 at any step where the case is tilted over 5 degrees, slid over 2 cm or turned
  over 10 degrees. It stays 0 for the rest of the episode once the case has tilted past 20 degrees.

`success()` returns true only on steps where `score()` is 1.0. The runtime ends the episode once it has stayed true for
1 s, so an episode can only end with success at 1.0.

**In a real trial**, film the metronome with a phone on a stand in front, level with the 60 line, and a second phone
above the table; outline its base in tape.

- **The weight's line**: read on a still frame from the front phone. Its top edge is about 17 mm in front of the scale
  plate, so a camera 10 degrees off level shifts the reading by about 3 mm; keep the camera level with it.
- **The swing**: on the frames at the ends of the swings, measure the angle between the rod and the case's centre line.
- **The tempo**: time the first and last centre passes in the last 5 s on the video frames (or count the ticks).
  Four consecutive passes, three beats, 2.81-3.21 s apart is 56-64 per minute.
- **The 5 cm clearance**: judged on the phone above the table, against the 13.4 cm wide plinth.

## How it is modelled

**Parts.** Everything is a rigid body at real size.

- **The case** is free on the table (410 g, rubber-like friction 0.6). It can be pushed, turned or tipped.
- **The pendulum** is one body: a 4 x 0.8 mm steel strip from 45 mm below the pivot to 150 mm above it, plus a 70 g
  brass weight 40 mm below the pivot. Its mass properties are set explicitly from those parts.
- **The sliding weight** is 20 g, 22 x 18 x 12 mm. Its front block sits in front of the rod, with cheeks either side of
  the rod and a strap behind it.

**Joints** (PhysX D6, `Joint` in `metkit.py`):

- **The pivot**: the pendulum turns freely about the case's front-to-back axis. All other motion is locked.
- **The clip**: the weight slides along the rod between stops 6 mm below the 208 line and 6 mm above the 40 line. It
  cannot turn on the rod or move across it. A soft spring drive (500 N/m) pulls it back onto the rod with 2.8 N, as a
  real clip's leaf spring does. Its force changes by under 0.7 N over the tallest ridge.

**The notches** (real contact geometry, nothing scripted):

- **The ridges.** The rod's front face carries one low steel ridge between every two adjacent marks, and one beyond
  each end mark. Each ridge is two thin plates along 30 degree flanks that meet halfway between the marks. Between two
  ridges, at every mark, is a V notch whose bottom is on the rod's face.
- **The tooth.** The weight's tooth is a rounded steel edge (0.6 mm radius, 3 mm wide) on a short stem. The spring
  presses it into the rod, and contact friction is 0.3.
- **Seating.** Friction (0.3) is below the flanks' slope (tan 30 degrees = 0.58), so a tooth let go on a flank slides
  down into the notch. Both flanks of a notch have the same slope and the tooth touches them within 0.3 mm of the
  bottom, so it seats with the weight's top edge exactly on that mark's line.
- **Moving it.** To move the weight, the tooth has to ride up a flank and over the next ridge, against the spring and
  friction. That takes 2.8 N x (tan 30 + 0.3) / (1 - 0.3 tan 30) = 3.0 N (an estimate of a real metronome's click).
- **Sizes.** The ridges are 0.3-0.5 mm high around 60, where the marks are 1.1-1.6 mm apart, and up to 1.3 mm high near 208,
  where they are 4.6 mm apart. The weight's body stands 2 mm in front of the rod's face, clear of them.

**The tempo.** The pendulum is a compound pendulum. Its period is
T = 2π √(I / k) · (2/π) K(sin²(A/2)), where:

- I is its moment of inertia about the pivot;
- k = g (m_bob d - m_weight s - the rod's net moment) is gravity's restoring torque per radian;
- K(sin²(A/2)) is the large-swing correction for a swing of A to each side.

It ticks twice per period. `test_scoring.py` checks the formula against a direct integration of the pendulum's
equation of motion. The scale's lines are placed by solving this for each Maelzel tempo with A = 25 degrees. In the
simulation they run from 35.2 mm (208) to 126.4 mm (40) above the pivot, a 91 mm scale, close to a real metronome's.
With the weight on the 60 line, a swing of 15 to 35 degrees ticks at 60.5 to 59.3 per minute, inside the 56-64 tempo
window.

In a smoke test the simulated pendulum ticked at 121.9 per minute where the formula gives 122.0 (weight on 120). In
a test run of the demo, with the weight clicked into the 60 notch (0.01 mm from the line), it ticked at 59.7 per minute.

**No escapement.** A wound metronome's escapement gives the pendulum a small push at each pass of the centre. That push
replaces what the swing loses to pivot friction and air, and it makes the tick. The escapement is not modelled: there
is no scripted force or torque on the pendulum. The simulated pendulum swings freely after it is let go, and loses a
little each swing to the joint solver and the case's contact with the table. Released at about 43 degrees, its swing fell
to about 25 degrees 7 s later. The score therefore judges the swing over the last 5 s only. The robot's start alone
has to give it enough swing (every swing in those 5 s at 15 degrees or more). A real wound metronome would keep its
swing up, so it is at least as easy to score.

**Simplifications.**

- **The notches** are idealised: V notches with 30 degree flanks cut exactly at the marks, and a rounded tooth.
- **No escapement or spring drive** (above). There is no tick sound; the tempo is counted from centre passes.
- **The clockwork inside the case is not modelled.** The case is a solid wooden shape with the right outline and mass.
  The pendulum and weight do not collide with the case (they swing in front of the scale plate, as on the real one).
- **Estimates, to be measured on the metronome used.**
  - The masses of the fixed weight (70 g) and the sliding weight (20 g), the rod's section, and the pivot's height.
  - The click-over force (about 3 N) and the notches' shape. Measure the force by pushing the weight up the rod with a
    small spring scale.

  A real metronome's own printed scale is what the real trial reads, so the scoring does not depend on these
  estimates.

## Results

- **Scripted demo** (`demo.mp4`, top camera, 640 x 352): score 1.0. It first reached full score at step 651.
  Success ended the episode at step 670 (33.5 s). The open-loop slide stopped in the 63 notch, and one slow push
  clicked the weight into the 60 notch (0.01 mm from the line). The pendulum, released at about 43 degrees, swung
  freely. Over the scored 5 s the measured tempo was 59.7 per minute, and the smallest swing was 25 degrees.
- **Dream-1** (`dream.mp4`, top camera, 320 x 176, the command above): score 0.0. The right arm reached toward the
  metronome but did not set or start it, and the trial ran to the 1,600-step limit (about 5 minutes of wall time on
  one GPU, including startup).
- **Screenshot** (`setup.png`): on the left, the top camera at the start. On the right, the left wrist camera's view of
  the untouched metronome 3 s into the demo: the weight in the 120 notch, the notched rod, and the numbered scale to the
  right of the rod.
