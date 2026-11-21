# Re-seat a stopped UR3e's tripped CSR50 crash sensor

A Zimmer CSR50 pneumatic collision sensor sits between a stopped UR3e's tool flange and its end tool, which has been
taken off after the crash to be checked. The crash has tripped the sensor: its compensating flange stands 12.5 mm out
of the housing and is turned 75 degrees off its mark. The robot must put the flange back home by hand, as the maker's
manual says, so that the sensor reports ready and the arm can be restarted: turn the loose flange the short way until
its triangle meets the housing's triangle, push it in square until it seats, let the air hold it, then let go and move
clear.

## Why this matters

- **What trips it.** A collision sensor (crash protector, breakaway) is fitted between a robot's wrist and its tool so
  that a collision deflects the tool instead of breaking the tool, the arm or the fixture. On the CSR the deflection
  vents its cylinder and a pressure switch stops the machine: "The signal from the integrated proximity switch can be
  used as an EMERGENCY STOP signal for the machine" (Zimmer installation and operating instructions DDOC00267,
  section 6). Collisions are most common while a cell is being programmed or commissioned and while new programs or
  learned policies are first tried on a real arm. A survey of robot learning on real hardware: "collisions with
  obstacles may damage the robot and its surroundings, which will require costly repairs and manual interventions"
  (Ibarz et al., "How to Train Your Robot with Deep Reinforcement Learning", IJRR 2021, section 4.11).
- **Who resets it, and how.** The CSR does not reset itself. The same manual: "After activation, the compensating
  flange must be put into the home position by hand. The home position is reached once the triangles applied on the
  housing and the compensating flange align with each other again." The datasheet gives the reason: "Manual reset -
  To ensure that the machine and the reason for the emergency stop are examined, the manual reset is suitable." The
  reset is done by the robot's operator or a robotics technician; O*NET 17-3024.01 (Robotics Technicians) lists as core
  tasks "Perform preventive or corrective maintenance on robotic systems or components" and "Install, program, or
  repair programmable controllers, robot controllers, end-of-arm tools, or conveyors".
- **How often.** Once for every trip: the arm stays stopped until someone resets the flange. No published rate
  exists, and none is claimed here; it depends on the cell. Trips are most frequent where robots are being developed:
  new paths, new tools and learned policies on real arms. Each one needs a person at the cell, which is one of the
  manual interventions that keep data collection and policy testing from running unattended.
- **Why it is hard for a robot.** It is not a button press. The flange is loose on a captured joint (it slides 12.5 mm,
  tips 12.5 degrees and turns freely); only 12 mm of it shows below a housing 16 mm wider; it is on a robot's wrist
  that limits where a hand can go; it must be turned the short way by reading two small marks; and it must go in
  square and on its mark, or it stops on its seat 2.5 mm out and the sensor stays tripped.

## Scene

Coordinates are metres from the bench top: x away from the two bench (YAM) arms, y to their left. The YAM arms are the
station's; the cell's own robot is the UR3e.

- **Universal Robots UR3e** (11.2 kg, 500 mm reach, ISO 9409-1-50-4-M6 tool flange). Base centre at (0.660, -0.040).
  Stopped and braked at joint angles base -35.9, shoulder -81.8, elbow 133.5, wrist 1 -61.7, wrist 2 92.4, wrist 3
  0.0 degrees (enter them on the pendant). Its tool flange is then at (0.365, 0.017, 0.158) with its axis pointing back
  toward the left bench arm and 10 degrees up. In simulation it is built from round bodies placed with UR's published
  DH parameters (d1 0.15185, a2 -0.24355, a3 -0.2132, d4 0.13105, d5 0.08535, d6 0.0921 m); the body diameters (base
  128 mm, shoulder 90 mm, elbow 75 mm, wrists 63 mm, upper arm 64 mm, forearm 54 mm) and the upper-arm and forearm
  offsets (120 and 93 mm) are estimates. It never moves.
- **Zimmer CSR50** collision protection
  ([product page](https://www.zimmer-group.com/en/products/components/robotics/collision-protection/series-csr/products/csr50),
  [datasheet](https://www.zimmer-group.com/fileadmin/pim/MER/GD/PG/MER_GD_PG_CSR50__SEN__APD__V1.pdf),
  [manual DDOC00267](https://www.zimmer-group.com/fileadmin/pim/SOM/DOK/MON/SOM_DOK_MON_DDOC00267-CSR__SALL__APD__V3.pdf)).
  Datasheet: EN ISO 9409-1 TK 50 both sides, installation height 71 mm, 12.5 mm axial travel, 12.5 degrees angular
  travel, 360 degrees about its axis, 1 to 6 bar, 0.66 kg, repeatability 0.05 mm. Drawing sizes used: housing 79 mm
  across, compensating flange 63 mm across with 12 mm of it below the housing. Its face carries the ISO 9409-1-50-4-M6
  pattern (four M6 holes on a 50 mm circle, a 6 mm dowel hole, a 31.5 mm centring recess). The M8 proximity switch sits
  in its holder on the underside; air comes in through an M5 fitting and a blue 4 mm hose. The housing is mounted with
  its triangle uppermost. At home the flange's face centre is at (0.310, 0.060, 0.170).
- **Triangle marks**: black, 8 mm wide and 7 mm tall (estimate), one on the housing's rim and one on the flange's rim;
  at home they point at each other tip to tip.
- **Tripped start**: the flange 12.5 mm out of the housing (its stop), square to the housing, turned 75 degrees
  counter-clockwise from the mark as seen facing the flange in lot A and 75 degrees clockwise in lot B. Air is on at
  1 bar; vented, the flange stays where it is left. The end tool was taken off after the crash to be checked; it is
  not part of the task.
- **Bench arms**: both at their ready pose with the grippers open.

The layout is fixed apart from the lot. Trials alternate lots A and B (trial k uses lot k mod 2); the screenshot is
lot A. Use `object_jitter=0`.

## The sensor's mechanism in simulation, and the estimates behind it

The housing is the fixed base of a jointed part; the flange (with the piston, 0.26 kg moving in all, an estimate from
the 0.66 kg unit) hangs from it on a slide along the axis (0 to 12.5 mm), two tilts (plus or minus 12.5 degrees) and a
turn (plus or minus 175 degrees). The flange collides with the housing (0.1 mm gap at home) and with the bench arms,
so a tilted flange meets the seat with its edge. A step hook applies the sensor's states as joint drives
(`mechanism()` in `task.py`):

| State | Rule | Basis | One-minute check on a real unit |
|---|---|---|---|
| Vented, loose | Each joint sticks until pushed past 1.5 N along the axis or 0.05 N m in turn or tilt, then follows | estimate of seal friction | push the vented flange with a spring scale |
| Seat lands | Off the mark by more than 5 degrees, the flange stops 2.5 mm out against pushes up to 100 N (40 N/mm) | estimate: the seat closes only on its mark, as the manual's triangles imply | push home 30 degrees off the mark; feeler gauge |
| Lead-in | Within 5 degrees and 3 mm, a light centring (0.5 N m/rad) on turn and tilt | the seat's grooves | |
| Pressure back | Seated within 0.8 mm all round and within 2.5 degrees of the mark for 1.0 s: ready; the flange is then held home (drives capped at 125 N and 4 N m) | estimate; the 125 N cap is the datasheet's trip load at 1 bar | time from seating to the switch input |
| Trips again | A ready flange forced 1.5 mm, 2 degrees or 4 degrees off | | |

No air flow, membrane or switch is simulated; these rules stand in for them and are stated here and in `fidelity`.

## Scoring

Score 0 if the flange's triangle never comes within 5 degrees of the housing's (tip to tip 2.7 mm or more on a close
photo with a steel rule), for example a flange pushed straight in off the mark (it stops on its seat 2.5 mm out) or
turned the long way to its stop; also 0 if the simulation's trial record is incomplete. Otherwise the highest row
reached:

| Score | Condition | Real-trial reading |
|---|---|---|
| 1.0 | At the end the sensor is ready, the flange is home, and for the last 1 s both bench grippers, their fingers and their last wrist links stay 20 mm or more from the sensor and the flange does not move | the switch input on the robot controller's I/O screen (or an LED on the switch's M8 cable with a 24 V supply); a 0.5 mm feeler leaf enters the joint nowhere round it; close photo with a steel rule: triangle tips within 1.4 mm (2.5 degrees); video for the hands and the hold |
| 0.75 | Ready at the end, but a gripper is within 20 mm or that hold is shorter than 1 s | as above |
| 0.5 | At some moment the flange was flush (the largest gap round the joint 1 mm or less) with the triangle tips within 1.4 mm, but it is not ready at the end | top and side video against a mm scale |
| 0.25 | At some moment the triangle tips were within 2.7 mm (5 degrees), however far out the flange stood | close video of the marks |

Simulation: the record is kept every control step (20 Hz) in the step hook (`observe()`), from the flange's four joint
values (the gap round the joint is the distance out along the axis plus 31.5 mm times the sine of the tilt) and from
points on every collision face of both grippers, their fingers and their last wrist links, sampled 3 mm apart. Still
means the joints move under 0.3 mm and 0.3 degrees. Any unknown state (a missing hand pose, a non-finite joint, a
record that does not match the state being scored) scores 0. The record is cleared only when a trial starts. Trials
are run on each of lots A and B and averaged.

### Shortcuts and near misses

`python near_misses.py` replays each case through the same record at 20 Hz (all pass):

| Attempt | Score |
|---|---|
| Untouched | 0 |
| Pushed straight in, 75 degrees off the mark (stops on the seat), let go | 0 |
| Turned the long way to the stop, then pushed | 0 |
| Turned to 5.1 degrees only | 0 |
| Turned to exactly 5 degrees, or to the mark, never pushed | 0.25 |
| On the mark but held 1.1 mm out | 0.25 |
| Turned 3 degrees off and pushed onto the seat (not seated) | 0.25 |
| Flush on the mark for 0.6 s, then pulled back out before ready | 0.5 |
| On the mark, held 0.9 mm out, or home but tilted 1.7 degrees (0.93 mm at the rim), never ready | 0.5 |
| Home on the mark tilted 1.2 degrees (0.66 mm at the rim): seats, ready, released | 1.0 |
| Ready, gripper left on the flange to the end | 0.75 |
| Ready, hands clear for only 0.9 s | 0.75 |
| Ready, released, hands clear and still 1 s | 1.0 |
| Full reset, but one step's hand state unknown | 0 |
| Record from another state (stale) | 0 |

The flange is captured, so it cannot be taken off and refitted; the UR3e is braked and fixed, so it cannot be moved;
nothing else in the scene can stand in for the flange. Touching or bracing on the braked UR3e is allowed and is not
scored (in simulation it is rigid). One hand is enough for the reset; the second bench arm only has to stay clear.

## How this differs from close tasks

- **TK-8CR9FX** (accepted, seat an oil-feed sleeve with its radial passage registered): a loose sleeve placed into a
  bore. Here the part is captured on a four-way compliant joint on a robot's wrist, is turned in place by reading two
  marks, and seats only on its mark; success is the device's own ready state.
- **TK-CS4JW9** (accepted, refit a delta robot's sprung rod pair after a crash): the same family of crash recovery on
  a robot; there a free sprung part is spread and clipped on balls, here a captured flange is turned and pushed home.
- **TK-ZW8U3V** (accepted, hand-guide a Franka arm to a service pose): moves a whole arm through its enabling device;
  here the arm stays braked and only the sensor's flange moves.
- **TK-NNL3TS** (lock a tool onto a quick-change flange): a detached tool plate onto dowels and a locking ring. Here
  nothing is detached and no tool is fitted.
- **TK-MA8U68** (prove an overload limiter releases and resets): a button sequence on a fixture. Here the reset is
  physical and the robot is not told which way to turn.
- **TK-9QFHJE** and **TK-NKM84Q** (a servo horn onto a spline on its zero mark): a loose horn pressed onto a spline
  on a bench. Here the part is captured, the work is on a robot in its cell, and the press is into a pneumatic seat
  that only closes on its mark.

## Real trial: what it needs

The UR3e only holds the sensor and is stopped throughout; any lab arm with an ISO 9409-1-50 flange can stand in for it
at the same flange pose. The CSR50 is the one purchase (Zimmer quotes it on request; no price is published), with an
M5 fitting, 4 mm hose and a regulator set to 1 bar. Without a robot controller, a 24 V supply and an M8 cable with an
LED show the switch state. A lab without such an arm can bolt the CSR50 to a rigid mount that copies the UR3e's last
two wrist bodies (63 mm round, 92 and 85 mm long) at the stated pose, so that the same access limits apply.

## Run

Use yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 \
      -E control_hz=20 --instruction "The collision sensor on the stopped UR3e's wrist has tripped. Reset it by hand: put its flange back home, where the triangle on the flange's rim meets the triangle on the housing, then let go and move your hands clear." \
      --scorer task_score --max-steps 1200 --epochs 2

Use `--policy dream` for the Dream-1 run with the same instruction and limits. `--epochs 2` runs lot A then lot B and
averages them; set `TASK_LOT=A` or `TASK_LOT=B` to run one lot. The demo takes about 16 s of simulated time per lot.

Files: `task.py` (scene, mechanism, scoring, demo), `near_misses.py`, and three helpers that patch the stock
embodiment inside the task's own process only: `parts.py` (jointed multi-shape parts; this copy adds chained joints,
run-time joint stiffness and position writes, and inertia for internal carrier links), `lots.py` (per-trial layouts)
and `tilt.py` (tilted grips for the scripted demo; it does not affect Dream-1 or scoring). Code is MIT licensed; no
third-party meshes are used.

## Demo and Dream-1

The scripted demo is open loop from the published start of each lot: the left arm lines its gripper up on the
sensor's axis (80 degrees off straight down), slides over the flange, grips its rim, turns it 75 degrees back to the
mark, pushes it home 8 mm past flush to load against the arm's give, holds it 2 s, lets go, backs off along the axis
and parks; the right arm stands clear. It scored 1.0 in lot A and in lot B (300 and 301 steps). Dream-1, given
the same instruction, lowered both arms toward the bench near their bases, never reached the sensor and scored 0 in
both lots at the 1,200-step limit.

## Reset

Shut off the air, pull the flange out to its stop square to the housing, turn it to the lot's angle against a printed
75-degree scale held on the housing, and turn the air back on at 1 bar; the flange stays out and the sensor stays
tripped. Check the UR3e is braked at the stated joint angles. Acknowledge the controller's stop only after scoring.
Nothing is consumed.
