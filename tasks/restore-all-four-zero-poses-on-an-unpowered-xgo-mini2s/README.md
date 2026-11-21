# Pose an unpowered XGO-mini2S for joint calibration

A Luwu XGO-mini2S desktop quadruped stands on a development table with its servos switched off, so all twelve leg
joints move freely against their gearboxes. Its legs are out of the maker's calibration posture. The robot must
bring every leg into that posture (shoulders square to the body, thighs vertical, lower-leg shafts level) and leave
the robot standing by itself with all four toes on the table, with both hands clear. The task ends before the zeros
are stored.

## Why this matters

Robot upkeep: a small legged robot whose stance has gone wrong is re-zeroed by a person who poses its unpowered legs
by hand. A robot that can do this takes one hands-on step of keeping robots running off people.

- **The maker's procedure.** The XGO-mini2S manual's "Initial Position Calibration" step is for a robot whose
  standing posture is wrong, with its four legs not all touching the ground together
  ([maker calibration manual](https://www.xgorobot.com/en/manual-xgo2.html)). The servos "stop producing torque",
  the operator moves every leg by hand until each shoulder is square to the body, each thigh is vertical and each
  lower leg is square to its thigh, and only then confirms, which overwrites the stored joint zeros. The manual notes
  that this alignment sets the robot's operating posture.
- **Who does it.** The robot's owner, a lab technician or a robotics technician. O*NET 17-3024.01 (Robotics
  Technicians) lists the core task "Perform preventive or corrective maintenance on robotic systems or components"
  ([O*NET](https://www.onetonline.org/link/details/17-3024.01)). The maker sells the XGO-mini2S for robot
  development and ROS research ([maker product page](https://www.xgorobot.com/en/mini2S.html)). No figure is
  claimed for how much time this saves or how it changes AI progress.
- **Why it is hard for a robot.** The joints are not free hinges and not motorized: each one holds until pushed past
  its gearbox's breakaway torque, then slips and stays where it is left. The robot's own 915 g weight loads every
  leg, so moving the body moves all four legs at once, and a leg folded too far ends with its toe lifted off the
  table and only its heel down. The robot must end up standing on its own, with every toe down, and the hands must
  leave without jolting it. Single legs on a bench, a jig or a stand would remove exactly these couplings.

## Scene

Coordinates are metres from the midpoint between the two YAM arm bases: x forward, y to the left, z up from the
tabletop.

- **Luwu Dynamics XGO-mini2S**, complete, with its arm and head parked: 915 g, 16 degrees of freedom of which 12
  are in the legs, servos of 4.5 kg cm (0.44 N m) ([maker product page](https://www.xgorobot.com/en/mini2S.html)).
  The model has the 12 leg joints and 3 arm joints; the claw is left out. RobotShop US lists an "XGO Mini2s with
  Radar Desktop AI Robot Dog" at US$1,799 (November 2026); the robot is the object of the task, not a tool. Per
  leg: a shoulder (roll, limits plus or minus 34 degrees), a thigh (pitch, -70 to 109 degrees) and a lower leg
  (pitch, -65 to 60 degrees), ending in a round toe. Geometry, joint axes, limits and link masses come from the
  maker's open model (Luwu Dynamics luwu_mjlab, Apache-2.0, revision 77d63dfb; file hashes in assets/SOURCES.json,
  licence in assets/LICENSE-XGO.txt), unscaled: 266.85 x 144.13 x 176.30 mm. Body and legs are taken as aluminium and ABS
  and the toe tips as silicone for friction (estimates).
- **Table**: the station's level 1.10 x 1.30 m laminate development table. Nothing else is in the scene; no jig,
  stand, added grip or stop.
- **Start (lot A)**: head toward +x, the robot set down with the midpoint of its four shoulder axles over (0.274, 0)
  (model root at (0.300, 0, 0.140) before settling). Shoulders and thighs at the illustrated posture; all four
  lower legs turned 21 degrees past it (toe down, knee up; 36 degrees in the maker model's own joint coordinate,
  whose zero is not the illustrated posture). The robot's own arm at the maker model's zero (all three arm joints
  at 0), as in the screenshot. Under its own weight the robot settles nose-up by about 7 degrees and about 30 mm
  back (shoulder-axle midpoint over (0.243, 0)): the front lower legs stay about 20 degrees from the posture, the
  rear lower legs sag to 7 to 10 degrees from it, and the rear thighs lean 11 to 14 degrees from vertical, so no
  leg is complete.
- **Lot B**: the same start turned 180 degrees about the vertical, head toward -x, set down over (0.246, 0) (model
  root (0.220, 0, 0.140)); it settles over (0.283, 0).
- Trials alternate lots A and B (trial k uses lot k mod 2); the screenshot shows lot A. Use `object_jitter=0`.

## The illustrated posture and how angles are read

The maker's illustration shows the shoulders square to the body, the thighs vertical and the long shaft of each
lower leg level. In the maker's simulation model the lower-leg joint's zero is not that posture: cross-sections of
the original lower-leg mesh put the shaft 14.4 to 15.5 degrees above level at model zero, so the illustrated
posture is model angle 15 degrees (`python checks/shaft_axis.py` reproduces this from the meshes;
fixture/shaft-axis-check.json has the sections). fixture/neutral-reference.svg is the reference drawing and
fixture/axis-reference.json gives four points per leg in the posture, in millimetres in the body frame: H, the
shoulder shaft centre; T, the thigh shaft centre; C, the knee shaft centre; F, the centre of the round toe.

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
| Gearbox hold, each of the 12 joints | Holds up to 0.12 N m, springs 4 N m/rad (damping 0.08 N m s/rad) within 0.03 rad, then slips and keeps the new angle | Estimate: the maker gives no torque-off figure; 0.12 N m is 27% of the 0.44 N m powered stall torque | Force gauge at the toe (about 75 mm lever), servos off: the force at which the leg starts to turn |
| Reference of that hold | The last pose the joint was left in, never the calibration zero | No pull toward the goal | |
| Mass | Maker model's 748.8 g of links; the missing 166.2 g of the 915 g total added to the body | Product page | Kitchen scale |
| Friction | ABS 0.6 static / 0.5 sliding; silicone toe 1.2 / 0.9 against laminate | Estimates for these materials | Tilt the robot on the table until it slides |
| Lower-leg contact | Three convex pieces cut from the original mesh: heel, shaft, toe; other links use convex hulls | Avoids a filled hull over the leg's open shape | |
| Self-collision | Off (the maker export's links overlap at rest) | | |

These numbers are estimates and are stated as such. With them, the robot cannot stand on its toes alone: a toe
carrying a quarter of the weight 80 mm ahead of the knee makes about 0.18 N m, more than the hold, so each lower leg
folds under load until its heel also touches. At rest a loaded leg stands on toe and heel together, about 11.5 degrees
from the knee's model zero with the thigh vertical, which is within 5 degrees of the illustrated posture. Offline
MuJoCo assays (`python checks/passive_retention.py`, summary in fixture/passive-model-assays.json) compare this hold
with the maker model's motors-off export (the legs collapse, 70 degrees off after 30 s) and a plain 0.12 N m Coulomb
friction (26 degrees off); the declared hold keeps an aligned robot within 2 degrees. These are model tests, not
measurements of a real unit. fixture/physical-validation-procedure.md gives the force-gauge and hands-off retention
checks a real unit must pass before its trials are scored; a unit that cannot stand in the posture without support
does not qualify, and no added support is allowed.

## Scoring

Score 0 if, at any moment of the trial, any part of the robot goes past a table edge or more than 10 mm below the
tabletop (the zero stays for the rest of the trial). Otherwise the score is read over the last 1 s of the trial,
and every row above 0 needs all of these for that whole second:

- both hands, including gripper housings and fingers, at least 30 mm from every robot surface, and no other arm
  link touching the robot (video and steel rule);
- all four silicone toes on the table: a 0.1 mm feeler leaf (or a strip of 80 g/m2 paper) cannot be slid under any
  toe (side close-ups); a heel may touch too, but heel or shaft contact alone does not count;
- the body's lowest surface at least 15 mm above the table and the body within 10 degrees of level (steel rule,
  side photos); the robot's own arm and head at least 10 mm above the table (from any scoring pose it cannot reach
  lower than about 43 mm, so it cannot prop the body);
- the whole robot still: no point on it moves more than 2 mm across the second (video against a steel rule).

A leg is complete when its shoulder, thigh and lower-leg angles are each within 10 degrees of the illustrated
posture and its thigh is within 10 degrees of vertical, for the whole second.

| Score | Condition | Real-trial reading |
|---|---|---|
| 1.0 | All four legs complete | protractor overlays on front/rear, left/right and overhead photos against the reference drawing |
| 0.75 | Three legs complete | as above |
| 0.5 | Two legs complete | as above |
| 0.25 | One leg complete | as above |
| 0 | No leg complete, or any condition in the list above fails | |

Simulation: `step()` in task.py runs every control step (20 Hz). It latches the table-edge zero, keeps a rolling
1 s window that is restarted by any motion beyond 2 mm, and counts a leg only if it was complete at every step of
the window; still means no corner of any link's bounding box moves more than 2 mm within the window (no point of a
rigid link moves farther than its farthest corner). Hands are checked with points sampled 3 mm apart over every
collision face of both grippers and fingers against conservative boxes round every robot link (`released()`); a
native contact view rejects
any force between any arm link and the robot (`observation.arm_support_force()`). A toe is down when PhysX reports
a contact point that lies on that leg's toe piece, within 1 mm of the tabletop plane, with a separation of 0.1 mm
or less or a load (`observation.distal_supports()`); contact on the heel or shaft pieces never counts. Any unknown
state (missing hand data, a contact buffer that may be truncated, angles that disagree with the joint readback, a
record that does not match the state being scored) scores 0. A trial stops once full score has held.

### Shortcuts and near misses

`python near_misses.py`, `python checks/temporal.py` and `python checks/distal_contacts.py` replay these through the
same scorer (all pass); the heel-only stance was also run in the simulator.

| Attempt | Score |
|---|---|
| Leave the start alone | 0 |
| All four legs in the posture but standing on heels, toes 0.8 to 1.1 mm up (simulator control run) | 0 |
| The same with toes 1.5 to 2.2 mm up (scorer replay) | 0 |
| One toe hovering 0.15 mm or 4 mm | 0 |
| The robot's own arm propped on the table (scorer replay; not reachable from a scoring pose) | 0 |
| The robot creeping 2.5 mm across the second (1 mm still counts as still) | 0 |
| Hold an aligned robot with either hand | 0 |
| Fingertips clear but the gripper housing 0.5 mm from the body | 0 |
| A forearm resting on the robot | 0 |
| Belly on the table | 0 |
| Off the table edge and back | 0 |
| Released for less than 1 s, or drifting 10 mm over the second | 0 |
| One hidden joint 11 degrees off | that leg not counted |
| One, two, three or four complete legs, released and still | 0.25, 0.5, 0.75, 1.0 |

The robot cannot be taken apart or replaced, and nothing else in the scene can stand in for a leg. One hand is
enough; the demo uses one. Recovering a leg that was already right is allowed. The two layouts are fixed, so they do
not rule out a memorized motion.

## How this differs from close tasks

- **TK-ZW8U3V** (accepted, hand-guide a Franka arm to its service pose): one serial arm moved through its enabling
  device and taught. Here there is no enabling device or teach step; four coupled, load-bearing legs on a free body
  must end in a posture the robot holds by itself.
- **TK-6YUJWA** (pending, catch a power-cut Koch arm and lower it into a cradle): catching a falling arm onto a
  support. Here nothing falls and no cradle or support may stay.
- **TK-MT5CX5** (accepted, aim a spring-balanced desk lamp): one balanced chain scored by where its beam lands. Here
  twelve joints are scored against a reference posture and the robot's own weight works against it.
- **TK-E4FFSD** (accepted, unfold a laptop stand and seat its rods): rods captured in slots. No slot or catch holds
  anything here.
- **TK-9QFHJE** and **TK-NKM84Q** (a servo horn onto its spline on the zero mark): one part pressed on at a mark.
  Here the zero is set by posing a whole robot.

## Real trial: what it needs

The XGO-mini2S in calibration mode (servos without torque; do not confirm the calibration or store zeros), the
reference drawing, a square, a steel rule, a protractor, a 0.1 mm feeler leaf and four cameras (front or rear, both
sides, overhead). Before scoring trials, run the backdrive and hands-off retention checks in
fixture/physical-validation-procedure.md on the unit; none has been run on a real unit for this task.

## Run

Use yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 \
      -E control_hz=20 --instruction "Set every leg of this unpowered XGO-mini2S to its calibration posture (shoulders square to the body, thighs vertical, lower legs level), with all four toes on the table, then move both hands clear." \
      --scorer task_score --max-steps 3600 --epochs 2

Use `--policy dream` for the Dream-1 run with the same instruction and limits. In every run, scripted or model,
the task caps each YAM finger's force at 40 N. `--epochs 2` runs lot A then lot B and averages them; `TASK_LOT=A`
or `TASK_LOT=B` runs one lot. No seed changes the layout.

Files: `task.py` (scene, passive joints, scoring, demo), `observation/` (hand, arm and toe contact readers),
`near_misses.py`, `checks/` (offline scorer and model checks), `assets/` (the maker's meshes and model with
sources), `fixture/` (reference drawing, axis points, assays, real-unit procedure), and three helpers that patch the
stock embodiment inside the task's own process only: `parts.py` (jointed parts), `lots.py` (per-trial layouts) and
`tilt.py` (tilted grips for the scripted demo; it does not affect Dream-1 or scoring).

## Demo and Dream-1

The scripted demo uses one arm and is open loop. After the reset it reads the body's position once from the
simulator state, rounded to 5 mm (standing in for the overhead view); from then on it reads no joint angle,
contact, force or score. The left hand comes down from 100 mm above the rear of the body, closes on the 68 mm
chassis, guides the body 40 mm forward and 10 mm down over 4 s (the body follows about 20 mm while the toes slide
and the legs fold), holds 2 s, opens over 2.5 s, waits 1 s open, lifts straight up 100 mm and moves aside; the right arm stays parked.
Under the guidance each lower leg folds until its heel and toe both touch. In development runs a quick or sideways
exit jolted the body and left one toe 0.15 to 0.6 mm up, which scores 0; the slow vertical exit leaves all four
toes down.

Scripted demo: 1.0 in lot A and 1.0 in lot B (421 and 428 control steps; each trial stopped once full score had
held). At the end of both trials (read from a diagnostic recording of the same source and motion, which ended at
the same steps) every leg is complete and all four toes are down (most heels touch too), the
body's lowest surface is about 64 mm above the table and level within 0.5 degrees, and both hands are parked. The
margin is thin in lot A: one rear thigh ends 9.6 degrees from the posture (limit 10) and the front lower legs end
12 and 19 degrees from model zero (posture 15).

Dream-1, given the same instruction: 0.0 in lot A and 0.0 in lot B, both at the 3,600-step limit. In the recorded
lot A video both arms lower and fold near their bases with their cameras toward the robot and never touch it.

## Reset

Keep the servos without torque; do not confirm the calibration or overwrite zeros. Park the arm and head as in the
reference. Set the shoulders and thighs to the illustrated posture and turn each lower leg 21 degrees toe-down from
it, using the reference drawing and a protractor. Place the robot at the lot's position and heading, let it settle,
and check that no leg is in the posture. Nothing is consumed.
