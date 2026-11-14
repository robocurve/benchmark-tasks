# Spin washed lettuce dry in a hand-crank salad spinner, holding the bowl steady while turning the crank

A 2-quart hand-crank salad spinner stands on the worktop with washed lettuce in its basket and its lid on. The robot
has to spin the lettuce dry: turn the crank on the lid at least five full turns clockwise, then let go.

**Instruction:** "Spin the washed lettuce dry in the salad spinner: give its crank at least five full turns clockwise,
then let go of it."

A hand-crank salad spinner is in most home kitchens and in many restaurant prep kitchens. Its crank turns the basket
through gears (here 1:5) and a one-way clutch, so the basket spins up while the crank is turned and coasts on when it
stops. Doing it takes two things the instruction does not spell out:

1. **Turning a crank round and round.** The hand holds a small knob that has to travel round a 5.5 cm circle, many
   times over, without losing it. The knob is fixed to the crank, so the hand cannot choose its own path: it has to
   go where the crank lets it, and keep pulling it on.
2. **Holding the bowl.** The gears push back on the crank, most of all while the basket is spinning up, and that push
   goes into the lid and the bowl. A light plastic bowl on a smooth worktop turns and slides with the crank unless the
   other hand holds it by its side handle. Then the crank barely turns relative to the lid, and the lettuce does not
   spin.

## Why this task is different

**Sustained motion under a constraint, not a reach to a pose.** Almost every task on the board, and in the task sets
it is compared with, is a sequence of point-to-point moves: reach, grasp, carry, place, press. Here the hand has to
follow a path that the object imposes, continuously, for six revolutions. It is the same skill as winding a hand
crank, turning a valve handwheel, cranking a window or an awning, winding a hose reel or a pencil sharpener, or opening
a door along its arc. A policy that plans the hand's path in advance and ignores what the crank is doing loses the
knob (see the development runs below). One that follows the crank keeps it.

**The second hand is needed by physics.** The instruction mentions only the crank. Holding the bowl is a precondition
the robot has to work out: without it, the reaction of the gears goes into the loose spinner instead of the crank. In
our control run without the holding hand the cranking hand dragged the bowl round and tipped it, and the crank stalled
at half a turn (0.25); in an earlier development run the unheld bowl turned 35 degrees and slid 12 cm across the table. This is the everyday version of bracing a part against
the reaction of a tool, as when holding a jar while unscrewing its lid or a workpiece while turning a wrench.

**A moving part with its own dynamics.** The basket spins up through a clutch and keeps spinning after the crank
stops (about 4 seconds to slow down). A policy has to let go of a knob attached to a mechanism that is still moving.

**Nothing like it in the benchmarks we checked.** We searched all task titles and descriptions on the task board
(over 2,000 pull requests), the accepted-task list, the reviewers' comments, RoboCasa's 365 kitchen tasks and the
1,018 activity definitions of BEHAVIOR-1K. None has a salad spinner or a geared crank in the kitchen. RoboCasa's
kitchen tasks open and close appliances, turn knobs and move food between containers. BEHAVIOR-1K's salad and cooking
activities are symbolic state changes (diced, cooked) with no mechanism.

## Where this comes up

- **Home kitchens.** Washing and spinning salad greens, herbs and berries is a routine part of cooking at home, and
  hand-crank spinners are among the most common kitchen gadgets. People with limited grip strength or tremor find
  the crank hard to keep going while steadying the bowl, which makes it a natural task for assistive robots.
- **Food preparation work.** Washing and drying produce is a core task of food preparation workers (O*NET
  35-2021.00, nearly 900,000 US workers) and restaurant cooks (O*NET 35-2014.00). Small kitchens use the same hand
  spinners; larger ones use the same mechanism at bigger scale.
- **The same skill elsewhere.** Hand cranks drive pencil sharpeners, egg beaters, pasta machines, hose and cable
  reels, awnings, manual winches, hand drills and valve handwheels. Each needs a hand to follow the crank's circle
  while something holds the base.

## Potential impact

- **Constrained, continuous manipulation.** Measures whether a policy can keep hold of a moving handle and go
  where the mechanism goes, over many cycles, rather than planning a path in free space.
- **Using feel, not just sight.** The knob's path is fixed, but small slips in the fingers add up within a turn or
  two. Keeping hold means following where the hand is actually being pulled: a closed loop through the robot's own
  sense of its position.
- **Bimanual bracing that the instruction leaves unsaid.** The policy has to infer that the bowl must be held.
- **Graded progress.** Partial credit separates a policy that never takes the crank (0), one that turns it a little
  (0.25), one that keeps it going for a while (0.5) and one that finishes and lets go (1.0).

## Files

| File | What it is |
|---|---|
| `task.py` | The task file: the spinner and its layout, the instruction, what happened during the trial, scoring |
| `spinnerkit.py` | Builds the salad spinner (one articulation: the bowl with its lid, the crank, the crank's knob and the basket) and its gearing (`gear()`, run every control step) |
| `demo.py` | The scripted demo, and the control runs (`--mode nohold`, `--mode still`) |
| `test_scoring.py` | Offline tests of the gearing, the scoring rule and the spinner's geometry (no simulator) |
| `task.json` | The submission's details |

## Requirements

The yam_arena embodiment and Isaac Lab-Arena, as set up by the program's GPU instance image (an NVIDIA RTX GPU is
needed). `test_scoring.py` needs only Python with numpy, pytest and the `yam_arena` package.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Spin the washed lettuce dry in the salad spinner: give its crank at least five full turns clockwise, then let go of it." \
  --scorer task_score --max-steps 1200
```

Run limit: 1,200 steps (60 s at 20 Hz). The demo reaches full score at step 709 (35 s), so the limit leaves room for
a slower policy.

## Scripted demo

```bash
python demo.py task.py out_dir                  # the demo
python demo.py task.py out_dir --mode nohold    # control: the same cranking with the left hand kept away
python demo.py task.py out_dir --mode still     # control: the arms do not move
```

1. The left hand comes straight down on the bowl's side handle and holds it between its fingers.
2. The right hand comes straight down on the crank's knob and takes it between its fingers.
3. It turns the crank clockwise six times. Every control step it aims a little further round the crank's axis (0.25
   rad) from where it measures its own hand to be, at the distance from the axis it measures. So it pulls the knob
   round without fighting the crank's circle, and a small slip in the fingers cannot add up. The knob turns freely in
   its fingers as it goes round. If its fingers ever close on nothing (the knob has slipped out), it opens them and
   takes the knob again where it last held it. It stops once its own measured position has gone six times round.
4. It lets go of the knob and rises clear; then the left hand lets go of the handle.

The demo uses only the layout constants in `task.py`, the spinner's geometry in `spinnerkit.py` and the robot's own
measured hand positions and finger openings (proprioception). It never reads the spinner's state.

## Scoring

Tools: the trial video, a red mark on the lid beside the crank's path (for counting turns), and a tape mark round the
bowl's base on the worktop.

- **Crank turns** are the crank's net clockwise turns relative to the lid: count the times the crank arm passes the
  red mark on the lid going clockwise, less the times it passes going back. Turns of the whole spinner (the bowl
  turning with the crank) do not count, because they do not turn the crank on the lid.
- **Knocked about:** at any time, the spinner's base moves 8 cm or more from its place, or the spinner tilts 15
  degrees or more.
- **Let go:** at the end, both grippers' tool points are 4 cm or more from the crank's knob.

| Outcome | Score |
|---|---|
| Knocked about at any time | **0** for the whole trial |
| At the end: 5 or more crank turns, and let go | **1.0** |
| At the end: 2 or more crank turns | **0.5** |
| At the end: half a turn or more | **0.25** |
| Anything else | **0** |

Every outcome falls into exactly one row. Five crank turns spin the basket 25 times or more, and it coasts on
after the crank stops. In code: `_watch()`, `score_from()` and `score()` in `task.py`; the crank's turns are
counted in `_mechanism()` in `spinnerkit.py`.

## Verification

| Run | Score | Notes |
|---|---|---|
| Scripted demo | 1.0 | 6.07 crank turns (the basket 44 turns), both hands let go; full score at step 709 (35 s); the bowl moved 0.9 cm |
| No holding hand (control) | 0.25 | Same cranking with the left hand kept away: the hand dragged the loose bowl (it tipped 1.4 degrees and lifted on one side) instead of turning the crank, which stalled at 0.53 turns on the lid |
| Arms still (control) | 0.0 | Nothing moves; the crank stays at 0 turns |
| Dream-1, 1,200 steps | 0.0 | The right gripper went to the crank's knob and hovered over it, then drew back; the left arm stayed back; the crank was not turned |
| `pytest test_scoring.py` | 11 passed | The gearing (forward cranking drives the basket and is resisted, backward cranking does not drive it, the basket's spin-up and coasting), the score tiers, the knocked-about gate, the geometry |

What we learned building the demo, which shows what the task asks of a policy:

| Development run | Crank turns (of 6) | What happened |
|---|---|---|
| Hand follows a pre-planned circle, light pinch | 2.7 | The knob slipped along the finger pads and the crank fell behind the hand, then the knob came out |
| Pre-planned circle, firm pinch | 0.6 to 0.9 | Any mismatch between the hand's circle and the crank's became a radial pull that dragged the whole spinner 5 cm |
| Pushing the knob round with closed fingers | 1.4 | The fingers lost contact with the knob where its path turned across them |
| Following the hand's measured position (final) | 6.1 | The hand stays on the crank's circle and keeps the knob for all six turns |

## Runtime and seeds

About 3.5 minutes per demo run and about 5 minutes per Dream-1 run on one L40S, including about a minute of
start-up. `object_jitter=0`; the layout is fixed, and there is no randomness in the task.

## Setup

Positions in cm from the midpoint between the two arm bases, on the table (x forward, away from the arms; y to the
left).

- **The salad spinner:** a 2-quart (about 1.9 L) hand-crank salad spinner, base centred at x 32, y -7.
  - The bowl: clear plastic, 20 cm across and 11 cm tall, 0.30 kg with its lid and gears. Its base is plain plastic
    on the worktop (friction about 0.25). A green side handle stands out 6 cm from the bowl on its left (+y) side,
    5 mm thick and 2.5 cm tall, its middle 5 cm above the table.
  - The lid: clear, 20.6 cm across and 1 cm thick, on the rim. A green gear housing (4 cm across) in its middle, and
    a red mark near its edge.
  - The crank: a grey arm 6.6 cm long on top of the housing, turning about the spinner's axis, 13.5 cm above the
    table at its top. At its end, 5.5 cm from the axis, a green knob 2.2 cm across and 3.4 cm tall that turns freely
    on the arm. It starts pointing forward (+x).
  - The basket: white, 17.6 cm across, inside the bowl, with 150 g of washed lettuce (0.27 kg in all) and a red
    stripe on its rim.
  - The gearing: one clockwise turn of the crank turns the basket 5 times through a one-way clutch. Turned
    backward, the crank does not drive the basket.
- **Robot:** both arms start in the raised ready pose with their grippers open.

## Fidelity

- **Rigid parts and a rule for the gears.** The bowl, lid, crank, knob and basket are rigid; the crank, knob and
  basket turn on joints about the spinner's axis (the knob about its own). The gears and clutch are a rule applied
  each control step (`gear()` in `spinnerkit.py`):
  - while the crank, geared up 1:5, would spin the basket faster than it is going, the clutch drives the basket
    toward that speed, up to 0.01 N m, so it spins up in about 1.5 s;
  - otherwise the basket coasts against the lettuce's air drag (2.2 x 10^-4 N m s/rad, slowing over about 4 s);
  - the crank feels the clutch's torque times 5, plus the gears' friction (0.02 N m s/rad): about 0.1 N m (2 N at
    the knob) when cranking at a steady pace;
  - the same torques act on the lid and bowl, as a real gearbox's do on its housing.
- **No water or loose leaves.** The lettuce is drawn as leaves fixed in the basket, and its mass is in the basket.
  Water is not simulated. The score counts crank turns, which is what the instruction asks for and what dries the
  lettuce on a real spinner.
- **The bowl's collision shape** is a solid cylinder: nothing has to reach inside it.

## Real-world build

- **The spinner:** any 2-quart hand-crank salad spinner with a crank on the lid and a handle or rim the gripper can
  hold (or a small handle added to the bowl). Put a red mark on the lid beside the crank's path and a tape mark
  round the bowl's base.
- **Each trial:** about 150 g of washed lettuce in the basket, lid on, crank pointing away from the robot.
- **Scoring:** count crank turns past the mark on video; check the bowl against its tape mark and the grippers'
  distance from the knob at the end.

## Third-party assets

None. Every part is built from primitive shapes in `spinnerkit.py`.
