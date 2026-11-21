# Fit a VerseGrip stylus to a MinVerse haptic controller

A Haply MinVerse desktop haptic controller stands on its folding feet with its stylus off. The robot must fit the
VerseGrip stylus back onto the controller's gimbal: the 9 mm steel ball on the stylus's front end goes into the round
cutout in the gimbal's forked tip and snaps past a sprung tine, so that the ball stays captive in the gimbal on its
own. Then both hands let go and move clear. The gimbal turns freely about its own axis and the controller's arm is
unpowered, so the arm gives under a press and the gimbal turns unless something holds them.

## Why this matters

- **What the device is for.** A haptic controller is a hand-held input for driving a robot: the operator moves the
  stylus and a follower robot copies it, and the session is recorded. Haply's HARP software connects its handles to a
  follower robot and logs the follower's data
  ([HARP command screen](https://docs.haply.co/harp/screen-command/)), and the maker lists the MinVerse among its
  controllers for collecting robot AI training data ([exhibitor statement, 2026](https://www.ngen.ca/hubfs/HM26_ExhibitorList.pdf)).
  Demonstrations collected by teleoperation are a main source of training data for robot manipulation policies.
- **Why the stylus comes off.** The VerseGrip stylus is a wireless, battery-powered handle with its own USB-C charging
  port. The maker's guide has it detached to pack the controller away (fold the head, close the feet, put it in its
  travel case) and refitted before use
  ([MinVerse quick-start guide](https://docs.haply.co/docs/quick-start-minverse/), "Attaching and detaching the
  VerseGrip Stylus" and "Putting away your Minverse"). So it is refitted at the start of a session whenever the
  controller has been packed or the stylus charged.
- **How the maker says to fit it.** The same guide: "With one hand, firmly hold the arm of the Minverse." Then turn the
  gimbal until its circular cutout faces up and hold it there, set the ball on the cutout and turn the stylus down
  firmly until the ball snaps in. In the simulation a straight press on the ball opens the sprung tine as well as that
  lever motion does; the score asks only for the fitted result, not either motion.
- **Who does it, and why a robot.** Teleoperation stations are set up, packed and recharged between sessions by
  people today; every station a robot can set up itself is operator time freed for collecting demonstrations. This is
  an argument for relevance, not a measured bottleneck.
- **Occupation.** O*NET 17-3024.00 (Electro-Mechanical and Mechatronics Technologists and Technicians) lists "Align,
  fit, or assemble component parts, using hand or power tools, fixtures, templates, or microscopes." No worker count
  or measured frequency is claimed for this step.
- **Why it is hard for a robot.** The target is on a free mechanism: the arm gives under a push and the gimbal turns
  freely about its own axis. The gimbal's cutout has to be turned straight up (it starts turned 40 degrees in lot A)
  and kept there while a 9 mm ball is brought onto a cutout 10 mm across and pressed past the sprung tine. The stylus
  is a long, round handle held in a parallel gripper, so the press reaches the ball through a grip 85 mm away, and the
  stylus must then be let go without pulling the ball back out. Holding the gimbal with one hand while the other
  presses is the maker's method; the score does not require it (see the one-hand check below).

## Scene

Coordinates are metres on the desk: x away from the two bench (YAM) arms, y to their left, z up from the desk top.
The layout is fixed apart from the lot; object jitter 0.

- **Haply MinVerse** ([product page](https://www.haply.co/minverse): folding design, 240 x 120 x 40 mm, 0 to 4 N
  haptic output; 1.2 kg and about US$1,500 at launch per the maker's launch coverage; the current page asks for a
  quote). A real trial uses an original MinVerse with its stylus, as found in a lab that collects teleoperation data;
  no stand-in is proposed. Built from the maker's photographs at its published 240 mm width; every other size below
  is an estimate.
  It stands with its head lying flat (status light away from the bench arms) and the arm reaching toward the bench
  arms, as the guide sets it up, and does not move on the desk.
  - Base bar 240 x 30 x 20 mm (black) centred at (0.523, 0.030); folding feet 120 x 20 x 6 mm at y = 0.134 and
    -0.074; side posts 30 x 16 x 70 mm; head 200 x 95 x 32 mm (grey) centred at height 0.096 with a white idle status
    light on its far edge; the inkwell (calibration holder, 26 x 26 x 60 mm) at the right end.
  - Passive arm: a vertical swing axis and a horizontal shoulder at (0.465, 0.030, 0.088); upper arm 53 mm to the
    elbow at (0.418, 0.030, 0.112); forearm 64 mm down to the gimbal's bearing at (0.370, 0.030, 0.070). Black links
    with silver parallel rods.
  - Gimbal: a black tube 14 mm across and 63 mm long, from x 0.370 to 0.307, its axis level and pointing at the bench
    arms; it turns freely about that axis. Its tip is a fork: a pocket 9.6 mm square and 7.7 mm deep with a floor,
    a back wall, a fixed tine, two front posts 4.8 mm apart (the slot the stylus's neck sits in) and, on the other
    side, a sprung tine with a lip that closes over the ball. The cutout's mouth is chamfered at 45 degrees, about
    16 x 17 mm across at its top. The ball's seat (where the ball's centre sits when fitted) is on the turning axis at
    (0.300, 0.030, 0.070).
  - Start: arm still in the pose above; gimbal turned 40 degrees from cutout-up toward the bench arms' left (lot A)
    or cutout straight up (lot B); sprung tine closed.
- **Haply VerseGrip Stylus** (black, 180 mm long overall, 21 mm body, 9 mm steel ball on a 4 mm neck, 80 g; all
  estimates from the maker's photographs and its battery and electronics). It lies on the desk, buttons up:
  lot A with its ball at (0.290, -0.150) and its body along -x (ball pointing away from the bench arms); lot B with
  its ball at (0.200, -0.095) and its body along -y (ball pointing to the left).
- **Bench arms**: both at their ready pose, grippers open.

Trials alternate lots A and B (trial k uses lot k mod 2); the screenshot is lot A.

## The mechanism in simulation, and the estimates behind it

| Part | Model | Basis | One-minute check on a real unit |
|---|---|---|---|
| Arm joints (swing, shoulder, elbow) | each sticks until pushed past 0.05, 0.20 and 0.12 N m, then turns and stays where it is left | estimate of the unpowered motors' and capstans' drag; holds the arm's own weight and the fitted stylus | hang a spring scale on the gimbal tube |
| Gimbal turn | free, sticks below 0.004 N m | estimate | turn the tube with a torque screwdriver |
| Sprung tine | slides 0 to 3.5 mm sideways, pulled shut with 0.60 N rising to 0.78 N fully open (50 N/m) | Haply's patent US12681577B2 holds the ball between two socket halves, one on a magnet-biased arm; the force is an estimate | push the tine open with a spring scale |
| Fork and ball | rigid; 0.3 mm clearance round the seated ball; a press of roughly 1 to 2.5 N on the ball opens the tine, and the seated ball holds against about 4 N pulled straight out of the cutout and 1 N pushed sideways into the tine | estimate | press the ball in on a kitchen scale; pull it out with a spring scale |
| Stylus | rigid 80 g body; friction 0.6 | estimate | weigh it |

The controller's electronics, haptic forces and status lights are not simulated; it is unpowered throughout.

## Scoring

Score 0 if, at any moment, the stylus falls off the desk, or the gimbal is pushed down to within 5 mm of the desk
(braced on the desk instead of held; side video against a mm scale), 0 for the rest of the trial; also 0 if the
simulation's trial record is incomplete. Otherwise the row the end of the trial reaches:

| Score | End state | How it is read in a real trial |
|---|---|---|
| 1.0 | The ball is snapped in: its centre within 1.5 mm of the seat and the sprung tine shut (within 0.5 mm), and for the last 1 s both grippers stay 30 mm or more from the stylus, the gimbal and the controller's arm while neither the stylus nor the gimbal moves more than 1 mm or 2 degrees | close side photo against a mm scale: the ball down in the fork below the tine's lip, the sprung tine back against the ball with no gap at its stop; top and side video stepped frame by frame with a steel rule |
| 0.75 | Snapped in and still for 1 s with no gripper within 5 mm, but a gripper within 30 mm | same video |
| 0.5 | Snapped in at the end, but still held, touched or moving | same photo and video |
| 0.25 | Not snapped in, but the ball is in the cutout's mouth (resting or held): its centre within 3.5 mm of the cutout's centre line and at most 10 mm above the seat | side photo against a mm scale |
| 0 | Anything else: the ball elsewhere, or the stylus off the desk at any time | |

Trials are run on each of lots A and B and averaged.

Simulation scorer (`task.py`): every control step (20 Hz) the step hook reads the ball's centre in the gimbal's frame
(`ball_in_gimbal`), the tine's slide (`seated`, `in_mouth`), the gimbal's lowest point (`gimbal_low`), and points on
every collision face of both grippers, their fingers and wrist links, 3 mm apart (`hand_points`), measured against the
stylus, the gimbal and the controller's two arm links as rounded segments (`hand_gap`). The 1 s hold counts only
while the ball is snapped in, the hands are clear and the stylus and gimbal move under 1 mm and 2 degrees from the
hold's first frame. An unknown or stale state scores 0.

### Shortcuts and near misses (`near_misses.py`, all pass)

| Shortcut or near miss | Score |
|---|---|
| Stylus left on the desk | 0 |
| Ball laid beside the gimbal, 12 mm off the cutout | 0 |
| Ball resting in the cutout's mouth, not snapped in, let go | 0.25 |
| Ball pressed to the seat with the tine held open 0.6 mm | 0.25 |
| Snapped in but still held at the end | 0.5 |
| Snapped in, released, stylus still swinging | 0.5 |
| Snapped in, hands clear for only 0.9 s | 0.5 |
| Snapped in, a hand left 20 mm away | 0.75 |
| Snapped in, released, but a gripper still on the controller's forearm | 0.5 |
| Ball 3.7 mm off the cutout's centre line (2.6 mm each way) | 0 |
| Gimbal pushed down onto the desk to brace it, then the ball snapped in | 0 |
| Snapped in, then pulled back out and laid down | 0 |
| Stylus dropped off the desk, then fitted | 0 |
| Snapped in, released, hands clear and still for 1 s | 1.0 |

## Demo, one-hand check and Dream-1

- **Scripted demo: 1.0 in lot A and 1.0 in lot B** (`--epochs 2`, about 400 control steps each). The left arm grips
  the gimbal tube 50 mm behind the seat, leaned to match the cutout's start turn, and stands its grip up to turn the
  cutout toward straight up (it reached about 6 degrees from straight up in lot A), then holds the gimbal. The right
  arm picks the stylus off the desk, holds it leaned 20 degrees, sets the ball over the cutout and presses 20 mm past
  the seat. In the traces the ball stopped 4 to 5 mm above the seat with the tine 2.2 mm open while pressed, and
  snapped in as the press eased; under the press the gimbal turned about 10 degrees in the left gripper and the arm
  gave a few millimetres. Both arms let go and move away; the stylus stays in the gimbal with its tail resting on the
  desk, about 17 degrees down.
- **One-hand check** (`VERSEGRIP_DEMO=onehand`: the same stylus motions with the gimbal not held): in lot B, where the
  cutout already faces up, the arm gave about 35 mm under the press and the ball still snapped in (1.0). So a second
  hand is not required once the cutout faces up and the press goes through the gimbal's axis; the maker's two-handed
  method is not scored. In lot A, where the cutout starts turned 40 degrees, the same one-hand motions slid the ball
  off the side of the fork and the stylus dropped onto the desk (0). A policy that never turns the gimbal therefore
  averages at most about 0.5; turning the cutout up is part of the task.
- **Dream-1** (same instruction, `--epochs 2`, 1,200-step limit): 0.0 in lot A and 0.0 in lot B, no refusal. In
  lot A's video the right arm moved round the stylus on the desk and nudged it without lifting it; the left arm stayed
  near its start and the gimbal was not touched.

## How this differs from close tasks

- **TK-CS4JW9** (accepted): spread a delta robot's sprung rod pair with both arms and clip its four cups onto fixed
  ball studs. Here a single ball goes into a socket that is itself on a free, unpowered arm and a freely turning
  gimbal that must be turned cutout-up and kept there (the maker has the second hand hold it), not onto fixed studs.
- **TK-AE2X9X** (pending): flip a car mirror's night tab while steadying the housing so its aim does not move. Here
  the steadied part must also be turned to a set orientation first, and the other hand places and presses a loose part
  into it.
- **TK-TUMETS** (accepted): attach a magnetic stylus to a tablet's flat magnetic side. Here the stylus's ball must be
  snapped into a sprung socket on a moving mechanism.
- **TK-WXVQHE** (accepted): turn a humanoid's upper body round on a swivel hook, keep it still and press its back
  cover on. There the free part is one swivel and the work is a cover; here the receiver hangs on four joints (three
  arm joints that each give under about 1 N at the gimbal, and the gimbal's own free turn), it must first be turned so
  its cutout faces up, and the part pressed in is a 9 mm ball that must pass a lip with 0.3 mm clearance.
- **TK-M6GCBU** and **TK-3SHVWE** (accepted): hold one loose part still (an M.2 drive, a proximity sensor) while the
  other hand works a latch or coupling ring on it. Here the held part is the receiver and the other hand brings a
  separate part to it; the hold is the maker's own instruction, not an added constraint.

## Reproduce

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=$PWD/task.py -E object_jitter=0 \
  -E control_hz=20 --instruction "Fit the VerseGrip stylus onto the MinVerse's gimbal, then let go and move both hands clear." \
  --scorer task_score --max-steps 1200 --epochs 2 --store-frames
```

Use `--policy dream` for the model run. `--epochs 2` runs lots A and B; `TASK_LOT=A` or `TASK_LOT=B` in front pins one
lot. Run limit 1,200 control steps (60 s) per lot. The scripted demo (`demo()`) plays fixed waypoints computed from the
published layout (where the stylus lies and how far the gimbal's cutout is turned, both visible); it reads no
simulator state. It uses `tilt.py` (a leaned grip for the scripted demo only) and `lots.py` (layout lots); `parts.py`
builds the jointed controller. None of them changes Dream-1 or the scoring.

## Reset

1. Detach the stylus (turn it firmly in a lever-like motion until the ball snaps out, as the guide says) and lay it on
   the desk at the lot's position, buttons up.
2. Put the arm back in the start pose (gimbal tube level at the heights above) and turn the gimbal to the lot's start
   (lot A: cutout 40 degrees toward the bench arms' left, set against a printed protractor held on the tube's end;
   lot B: cutout straight up).
3. Bench arms to their ready pose. Nothing is consumed.

## Fidelity

See `fidelity` in task.json. The controller is a reconstruction from photographs, not the maker's CAD; it does not
move on the desk. Drag, tine force, stylus mass and all sizes other than the 240 mm width are estimates, listed above
with how to check them. The scripted press is a position command 20 mm past the seat, not a force; the contact force
was not measured in simulation. The YAM arms give under load, which limits it to a few newtons (estimate), within
what the maker's "firmly" suggests a person applies.

Assets: none; the controller and stylus are built from boxes, prisms and spheres in `task.py` (MIT). Product names
identify the real equipment the task describes; no maker images or files are included.
