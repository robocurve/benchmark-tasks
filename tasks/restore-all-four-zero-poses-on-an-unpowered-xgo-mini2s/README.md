# Pose an unpowered XGO-lite robot dog for joint calibration

An XGO-lite desktop robot dog stands on a development table with its servos switched off, so all twelve leg joints
move against their gearboxes and stay where they are left. Its rear legs are in the maker's calibration posture;
its two front lower legs are turned toe-up out of it, as they can be after the front knee servos are fitted and
before the legs are set. The robot must bring every leg into that posture (shoulders square to the body, thighs
vertical, lower legs level on the table) without disturbing the rear legs, and leave the robot standing by
itself, with both hands clear. The task ends before the zeros are stored.

## Why this matters

Robot building: a legged robot's joint zeros are set by posing its legs by hand after it is assembled or after a
leg servo is fitted. A robot that can do this takes one hands-on step of building and commissioning robots off
people.

- **The kit's procedure.** The XGO Robot Kit V2 instructions recalibrate from the XGO app: after Calibrate the
  servos "will no longer output torque, allowing you to rotate all joints", the robot is placed in the illustrated
  posture ("the shoulder of each leg is perpendicular to the body, the thigh is perpendicular to the ground, and the
  calf is perpendicular to the thigh"; jaws open, arm folded on the back), and Complete Calibration stores it. The
  same instructions replace a leg servo by refitting the lower leg at a right angle to the thigh and end "switch on
  and re-calibrate it" ([ElecFreaks XGO Robot Kit V2 FAQ](https://wiki.elecfreaks.com/en/microbit/robot/xgo-robot-kit-v2/FAQ)),
  so the calibration is the last step of fitting a joint. The robot maker's guide for the same frame gives the same
  step and warns that confirming overwrites the factory settings
  ([XGO-lite3 user guide](https://www.xgorobot.com/en/manual-lite3.html)).
- **Who does it.** Whoever builds or rebuilds the robot: a student, a lab technician or a robotics technician. No
  figure is claimed for how much time this saves or how it changes AI progress.
- **Why it is hard for a robot.** The joints are not free hinges and not motorized: each one holds until pushed past
  its gearbox's breakaway torque, then slips and stays. All joints hold about the same torque, so turning a lower
  leg by its toe also turns the thigh above it unless the thigh is kept still, and the robot's weight rests on its
  legs. Only the front legs start out, but all twelve joints are scored at the end: a rear lower leg or rear thigh
  knocked 8 degrees scores 0. The robot must end standing on
  its own, level and still, and the hands must leave without disturbing it. Single legs on a bench or a stand would
  remove exactly these couplings.

## The robot, and the real-trial route

- **ElecFreaks XGO Robot Kit V2 for micro:bit** (the XGO-lite V2): US$449 at the
  [ElecFreaks store](https://shop.elecfreaks.com/products/elecfreaks-xgo-robot-dog-kit-v2-for-micro-bit) (November
  2026, a sale price; list US$641), shipped as a complete machine
  ([unboxing list](https://wiki.elecfreaks.com/en/microbit/robot/xgo-robot-kit-v2/open-box-inspection)); 500 g;
  250 x 145 x 170 mm in its default stance; 1 mm aviation-aluminium body, ABS legs; 2.3 kg cm (0.23 N m) bus servos
  with a 0 to 300 degree range ([product parameters](https://wiki.elecfreaks.com/en/microbit/robot/xgo-robot-kit-v2/products-parameters));
  15 degrees of freedom with a robotic arm and gripper, three servos per leg
  ([introduction](https://wiki.elecfreaks.com/en/microbit/robot/xgo-robot-kit-v2/xgo-lite-v2-products-instruction),
  [hardware](https://wiki.elecfreaks.com/en/microbit/robot/xgo-robot-kit-v2/hardware-introduction)). The kit needs a
  BBC micro:bit, sold separately (US$23.99 in the same store). The tools are a protractor, a square, a steel rule, a
  phone camera and a 0-10 N force gauge for the one-off backdrive check (tens of dollars together). A real-trial
  setup costs about US$500-550.
- **Simulation model.** Luwu Dynamics, who make the XGO robots, publish an open MuJoCo model of the XGO-lite3
  (luwu_mjlab, Apache-2.0, revision 77d63dfb; file hashes in assets/SOURCES.json, licence in
  assets/LICENSE-XGO.txt). The XGO-lite3 has the V2's published 250 x 145 x 170 mm stance, aluminium body, ABS legs
  and 2.3 kg cm 300-degree servos, with a computer and screen head that make it 610 g
  ([user guide](https://www.xgorobot.com/en/manual-lite3.html), [model comparison](https://wiki.xgorobot.com/xgo/compare)).
  The task uses the model's geometry, joint axes, limits and link masses unscaled, with the V2's 500 g total; the
  V2's own link geometry has not been measured separately.
- Per leg: a shoulder (roll), a thigh (pitch) and a lower leg (pitch). The lower leg is an aluminium bracket under
  the knee and an ABS bar ending in a round 18 mm toe pad 68 mm ahead of the knee. The arm is folded on the back
  with its jaws open, as the calibration instructions say; the model has the arm's two joints, passive like the legs,
  with the jaws fixed open (the gripper servo is not modelled).

## Scene

Coordinates are metres from the midpoint between the two YAM arm bases: x forward, y to the left, z up from the
tabletop.

- **Table**: the station's level 1.10 x 1.30 m laminate development table. Nothing else is in the scene; no jig,
  stand, added grip or stop.
- **Start (lot A)**: head toward the left arm (84 degrees from +x, counterclockwise seen from above), the midpoint
  between the four shoulder axes over (0.310, -0.060). Both rear legs are in the illustrated posture, lying level on
  the table. Both front shoulders and thighs are in it too; the front lower legs are turned toe-up from it by
  22 degrees (front left) and 26 degrees (front right), resting on the heels under the knees with the toe pads
  well clear of the table. Two legs are complete and two are not.
- **Lot B**: the same start with the head at 102 degrees from +x, over (0.330, -0.045).
- Trials alternate lots A and B (trial k uses lot k mod 2); the screenshot shows lot A. Use `object_jitter=0`.

## The illustrated posture and how angles are read

The maker's picture shows each shoulder square to the body, each thigh vertical and each lower leg square to its
thigh, lying level on the table from the heel under the knee to the toe pad. In the maker's model the lower-leg
joint's zero is not that posture: the leg lies level at model angle -12.16 degrees (`python checks/flat_axis.py`
finds it from the meshes; fixture/flat-axis-check.json). The shoulder and thigh posture is model angle 0.
fixture/neutral-reference.svg is the reference drawing and fixture/axis-reference.json gives four points per leg in
the posture, in millimetres in the body frame: H, the shoulder shaft centre; T, the thigh shaft centre; C, the knee
shaft centre; F, the centre of the toe pad (the maker model's foot point).

Each leg's three angles are read from neighbouring rigid parts, not from a silhouette: the shoulder angle from H to
T about the shoulder axis in the body frame, the thigh angle from T to C in the shoulder's frame, and the lower-leg
angle from C to F in the thigh's frame. In a real trial these are protractor readings on photos taken square to
each axis (front or rear for the shoulders, left and right sides for thighs and lower legs, overhead for the body),
against the reference drawing and a square and steel rule on the table. A leg with a hidden or unresolved point
earns no credit. In simulation `observed_angles()` in task.py rebuilds the twelve angles from the link poses the
same way; if they ever differ from the joint readback by more than 1 degree the trial scores 0.

## The passive joints in simulation, and the estimates behind them

| Item | Model | Basis | One-minute check on a real unit |
|---|---|---|---|
| Gearbox hold, each of the 14 passive joints | Holds up to 0.06 N m, springs 2 N m/rad (damping 0.04 N m s/rad) within 1.7 degrees, then slips and keeps the new angle | Estimate: the maker gives no torque-off figure; 0.06 N m is 26% of the 0.23 N m powered torque | Force gauge at the toe pad (68 mm lever), servos off: the force at which the leg starts to turn |
| Reference of that hold | The last pose the joint was left in, never the calibration posture | No pull toward the goal | |
| Mass | Maker model's 345 g of links; the missing 155 g of the 500 g total added to the body | Product parameters | Kitchen scale |
| Friction | ABS and aluminium 0.6 static / 0.5 sliding against laminate | Estimates for these materials | Tilt the robot on the table until it slides |
| Lower-leg contact | Three convex pieces cut from the original mesh: heel bracket, bar, toe pad; other links use convex hulls | Avoids a filled hull over the leg's open L shape | |
| Self-collision | Off (the maker export's links overlap at rest) | | |

These numbers are estimates and are stated as such. Offline MuJoCo assays (`python checks/passive_retention.py`,
summary in fixture/passive-model-assays.json) show what they imply: with the declared hold the posture stays within
0.7 degrees for 10 s and the front legs of the start pose stay out of it; the maker model's own motors-off export
lets the legs fold 70 degrees; holding the chassis a few millimetres up while turning each front lower leg about its
knee ends with all four legs within 0.7 degrees; pressing each front toe pad straight down with nothing held turns
the thighs as well (7.8 degrees) and leaves the lower legs 9.4 degrees off, outside the 7-degree band. These are
model tests, not measurements of a real unit.
fixture/physical-validation-procedure.md gives the force-gauge and hands-off checks a real unit must pass before its
trials are scored.

## Scoring

Score 0 if, at any moment of the trial, any part of the robot goes past a table edge or more than 10 mm below the
tabletop (the zero stays for the rest of the trial). Otherwise the score is read over the last 1 s of the trial,
and every row above 0 needs all of these for that whole second:

- both hands, including gripper housings and fingers, at least 30 mm from every robot surface, and no other arm
  link touching the robot (video and steel rule);
- the body's lowest surface at least 15 mm above the table and the body within 10 degrees of level (steel rule,
  side photos); the robot's own arm at least 10 mm above the table;
- the whole robot still: no point on it moves more than 2 mm across the second (video against a steel rule).

A leg is complete when its shoulder, thigh and lower-leg angles are each within 7 degrees of the illustrated
posture and its thigh is within 7 degrees of vertical, for the whole second. Every row above 0 also needs both
rear legs complete: they start set and must still be set.

| Score | Condition | Real-trial reading |
|---|---|---|
| 1.0 | Both front legs complete (all four legs complete) | protractor overlays on front/rear, left/right and overhead photos against the reference drawing |
| 0.5 | One front leg complete | as above |
| 0 | Neither front leg complete, a rear leg out of the posture, or any condition in the list above fails | |

Simulation: `step()` in task.py runs every control step (20 Hz). It latches the table-edge zero, keeps a rolling
1 s window that is restarted by any motion beyond 2 mm, and counts a leg only if it was complete at every step of
the window; still means no corner of any link's bounding box moves more than 2 mm within the window (no point of a
rigid link moves farther than its farthest corner). Hands are checked with points sampled 3 mm apart over every
collision face of both grippers and fingers against conservative boxes round every robot link (`released()`); a
native contact view rejects any force between any arm link and the robot (`observation.arm_support_force()`). Any
unknown state (missing hand data, angles that disagree with the joint readback, a record that does not match the
state being scored) scores 0. A trial stops once full score has held.

### Shortcuts and near misses

`python near_misses.py` and `python checks/temporal.py` replay these through the same scorer (all pass);
`python checks/passive_retention.py` runs the press-only attempt in MuJoCo.

| Attempt | Score |
|---|---|
| Leave the start alone | 0 |
| Set both front legs but knock a rear lower leg 8 degrees; or set one front leg and knock a rear thigh 8 degrees | 0 |
| Press each front toe pad straight down, nothing else held (MuJoCo) | 0: the thighs turn 7.8 degrees and the lower legs stop 9.4 degrees off |
| A front leg left as that press leaves it (thigh 7.8, lower leg 9.4 degrees off) | that leg not counted |
| Shoulder and thigh each 6.5 degrees off (thigh 9.2 degrees from vertical) | that leg not counted |
| One shoulder, thigh or lower leg 7.5 degrees off | that leg not counted |
| Every thigh and lower leg 6.5 degrees off | 1.0 |
| The robot's own arm propped on the table (scorer replay; not reachable from a standing pose) | 0 |
| Lying on its belly with the legs folded | 0 |
| Body rolled 11 degrees | 0 |
| The robot creeping 2.5 mm across the second (1 mm still counts as still) | 0 |
| Hold the robot with either hand, or a hand face 0.5 mm from the body | 0 |
| An arm link resting on the robot | 0 |
| Off the table edge, or 12 mm below the tabletop, and back | 0 |
| Released for less than 1 s, or drifting 6 mm per second | 0 |
| One or both front legs complete, rear legs still set, released and still | 0.5, 1.0 |

The robot cannot be taken apart or replaced, and nothing else in the scene can stand in for a leg. The two layouts
are fixed, so they do not rule out a memorized motion.

## How this differs from close tasks

- **TK-ZW8U3V** (accepted, hand-guide a Franka arm to its service pose): one serial arm moved through its enabling
  device and taught. Here there is no enabling device or teach step; four coupled, load-bearing legs on a free body
  must end in a posture the robot holds by itself.
- **TK-6YUJWA** (pending, catch a power-cut Koch arm and lower it into a cradle): catching a falling arm onto a
  support. Here nothing falls and no cradle or support may stay.
- **TK-MT5CX5** (accepted, aim a spring-balanced desk lamp): one balanced chain scored by where its beam lands. Here
  twelve joints are scored against a reference posture and the robot's weight rests on the legs being set.
- **TK-E4FFSD** (accepted, unfold a laptop stand and seat its rods): rods captured in slots. No slot or catch holds
  anything here.
- **TK-9QFHJE** and **TK-NKM84Q** (a servo horn onto its spline on the zero mark): one part pressed on at a mark.
  Here the zero is set by posing a whole robot.

## Real trial: what it needs

The XGO-lite V2 in calibration mode (servos without torque; do not confirm the calibration or store zeros), the
reference drawing printed so its 100 mm scale bars measure 100 mm, a protractor, a square, a steel rule, a 0-10 N
force gauge and four camera views (front or rear, both sides, overhead). Before scoring trials, run the backdrive and hands-off checks in
fixture/physical-validation-procedure.md on the unit; none has been run on a real unit for this task.

## Run

Use yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 \
      -E control_hz=20 --instruction "Set every leg of this unpowered XGO-lite robot dog to its calibration posture (shoulders square to the body, thighs vertical, lower legs level on the table), then move both hands clear." \
      --scorer task_score --max-steps 3600 --epochs 2

Use `--policy dream` for the Dream-1 run with the same instruction and limits. In every run, scripted or model,
the task caps each YAM finger's force at 40 N. `--epochs 2` runs lot A then lot B and averages them; `TASK_LOT=A`
or `TASK_LOT=B` runs one lot. No seed changes the layout.

Files: `task.py` (scene, passive joints, scoring, demo), `observation/` (hand and arm contact readers),
`near_misses.py`, `checks/` (offline scorer and model checks), `assets/` (the maker's meshes and model with
sources; native.json is the same model exported for the simulator), `fixture/` (reference drawing, axis points,
assays, real-unit procedure), and three helpers that patch the stock embodiment inside the task's own process
only: `parts.py` (jointed parts), `lots.py` (per-trial layouts) and `tilt.py` (tilted grips for the scripted demo;
it does not affect Dream-1 or scoring).

## Demo and Dream-1

The scripted demo is open loop from the declared layout and reads nothing of the robot dog. The right hand closes
on the sides of the chassis over its centre of mass, leaning back 30 degrees so its fingers clear the folded arm,
and lifts the robot about 30 mm, so the body cannot move and no foot touches the table. The hand sags about 18 mm
under the robot's weight and draws the body a few millimetres toward it; the plan aims at the hanging pose measured
in a development run of the same motion (`HELD` in task.py). The left hand then takes each front leg in turn: it
pinches the toe pad across its two flat faces with the fingers leaning 22 degrees out ahead of the head, moves the
pad's centre along its arc about the knee to 13 degrees past the posture (the held robot dips and the hand lags
under the knee's resistance, and the joint springs back up to 1.7 degrees), opens fully and rises clear. The right
hand then lowers the robot until the front legs are about to touch, opens slowly, waits 1 s open and lifts straight
up; both hands park. Each scripted arm also corrects its own steady tracking error from its joint readings while it
waits or lines up, and holds that correction through every contact. The rear legs are never touched.

Scripted demo: 1.0 in lot A and 1.0 in lot B (970 and 977 control steps; each trial stopped once full score had
held). In a diagnostic recording of the same demo motion (made while the scorer still used a 10-degree band), which
ended at the same steps, every leg ends complete: in lot A the front legs end within 4.6 degrees of the posture (front right thigh
3.4 degrees, lower leg 4.6 degrees) and in lot B within 1.1 degrees; the rear legs stay within 0.7 degrees; the
body's lowest surface is 56 mm above the table and the robot's own arm 102 mm; the peak finger force is 22 N.

Dream-1, given the same instruction: 0.0 in lot A and 0.0 in lot B, both at the 3,600-step limit. In the recorded
lot A video both arms lower near their bases with their cameras toward the robot and never touch it; the robot stays
in its start pose.

## Reset

Keep the servos without torque; do not confirm the calibration or overwrite zeros. Fold the arm on the back with
its jaws open. Set every leg to the illustrated posture, then turn the front lower legs toe-up from it by 22 degrees
(front left) and 26 degrees (front right), using the reference drawing and a protractor. Place the robot at the
lot's position and heading and check that both rear legs are in the posture and neither front leg is. Nothing is
consumed.
