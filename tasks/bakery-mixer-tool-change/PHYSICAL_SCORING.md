# Observable scoring protocol

Use the same four stages in simulation and on the physical apparatus. Judge the
assembled appliance and its visible motion. A hidden coupling contact, hub yaw,
spring compression, exact drive ratio or submillimetre measurement earns no credit.
The trial budget is 180 seconds, including all observation holds.

## Preparation

The bowl is empty, mounted and clamped at its lower stop. The motor is off. The
beater and removable guard start on their separate draining supports, as shown in
`setup.png`. Only the robot may manipulate the apparatus during the trial.

Use a ruler, removable reference marks, a phone/video camera and its timestamps.
Do not reach into the apparatus to measure it during a trial. Establish these
references with power disconnected before the trial:

- Mark the centre of the exposed rectangular beater neck, 40 mm below the beater
  origin in `geometry.json`. Put a contrasting witness on the exposed carrier
  directly above the shaft axis. With a correctly mounted beater and power isolated, photograph the
  neck relative to this witness and a ruler in their plane. The neck must remain
  within 5 mm horizontally and 10 mm vertically of that mounted relationship,
  and within 10 degrees of upright. During operation the reference follows the
  visible carrier witness around its 20 mm-radius orbit; a fixed 5 mm box in the
  overview is not the reference. A side view must show both exterior references.
  If either is hidden, reposition the measurement camera before the trial or use
  an external pointer on the carrier that does not touch the beater. No
  measurement is made inside the hub.
- Mark the bowl's lower and upper stops, 75 mm apart. A bowl position within
  5 mm of the relevant stop passes. A ruler fixed to the stationary column makes
  the lift observable without touching it.
- Mark the guard centre and its front/back orientation on the bowl rim. Record
  the guard's correctly seated height: its rim origin is 144.5 mm above the bowl
  origin. A seated guard is within 5 mm horizontally and vertically, within
  5 degrees of level and within 10 degrees of its orientation mark. Use the three
  locating feet and the rear slot as visible alignment references for these bands;
  no concealed contact state is inspected.
- Put a small contrasting mark on a visible beater blade and on the carrier.
  Marks must be visible through the wire guard; use the side/wrist view if the
  overview is occluded. The stationary frame supplies the angular reference.
- Mark both gripper centres. For released-beater observations each centre must
  be more than 75 mm from the beater centreline at the hub and at points 40, 105
  and 170 mm below it. For a released guard both centres must also be more than
  190 mm from its centre. These distances are operational scoring boundaries.
- Mark the work envelope: x strictly between -180 and 900 mm, y from -650 to
  650 mm, and z from 727.5 to 1500 mm, in the task coordinates. The beater and
  guard origins must stay inside it. Mark the rotating-bowl exclusion cylinder:
  145 mm radius about the bowl centre, from 20 mm below its origin to world
  z = 1097.5 mm. A gripper centre inside that cylinder during powered/coasting
  motion faster than 5 rpm invalidates the trial.

The setup overview retains the standard calibrated YAM station camera angle.
Additional side/wrist recording is measurement evidence; it does not replace the
standard overview. Record continuously with timestamps and enough resolution to
read the external marks. If an observation is obscured, award only a lower stage
that the recording actually establishes; do not infer a concealed fit.

## Award the current stage

| Score | Observable test | How the judge checks it |
| --- | --- | --- |
| 0.00 | No qualifying retention test, or an invalidating event. | Apply the rules below. |
| 0.25 | The released beater hangs in its working band for one continuous second, with the bowl lowered and the guard still at least 350 mm from the hub. | Timed side/overview video shows both grippers clear, the guard away, the bowl at its lower mark, the beater upright and its exposed neck inside the external band. The bowl floor is at least 50 mm below the beater throughout these reference bands, so it cannot support the beater. |
| 0.50 | The retention test has passed; the beater remains in its working band; the guard is seated and released for one continuous second; the bowl is at its upper mark. | Check the guard rim/feet and reference marks, the raised-bowl scale, and the gripper-clear distances. |
| 0.75 | The preceding assembly conditions remain true while the motor visibly drives the beater clockwise for five uninterrupted seconds, at one revolution per second or faster, with both grippers clear. | Count at least one complete clockwise blade-mark revolution in each of five consecutive one-second windows, with no pause or reversal. The motor has been switched on, rather than the tool being turned by a gripper. Exact shaft speed and the internal gear ratio are irrelevant. |
| 1.00 | After that powered interval, the robot presses STOP and leaves the assembly retained, the guard seated and the bowl raised; the motor is off and both visible rotating marks remain stationary for two uninterrupted seconds with both grippers clear. | Video shows STOP followed by the two-second endpoint. Each mark may travel at most 5 degrees in total during the hold. The simulation uses a conservative maximum of 0.4 rpm for the visible beater and carrier at each sample: at most 4.8 degrees over two seconds. |

The score is the highest **currently valid** row. The initial released-hang test
is remembered while the beater stays in its external working band. Removing or
dropping it cancels that proof; repeat the unguarded, lowered-bowl release test.
Removing the guard or lowering the bowl cancels a completed powered interval;
reassemble and repeat the five-second run. An interrupted five-second run restarts
its timer. Merely lifting the bowl earns no separate stage. Retention before the
guard is installed prevents using the guard as a beater prop.

The physical run check uses continuous video. A turn count alone cannot hide a
pause or reversal. The simulated scorer conservatively checks clockwise speed
at each 20 Hz sample; it does not require the concealed shaft to match the tool.
The stop check uses absolute angular travel, so forward/backward motion cannot
cancel out. Do not use motor sound alone as evidence of beater motion or stopping.

## Invalid outcomes

Award zero for a beater/guard leaving the marked work envelope, a gripper centre
entering the marked bowl cylinder during powered/coasting motion faster than
5 rpm, or guard/bowl disengagement during that motion. Altering, breaking,
wedging, taping or adding supports to the apparatus or its measurement marks
also invalidates a physical trial. The permitted objects are the complete setup
inventory; no props or tools are added during a run.

Otherwise, an unsuccessful attempt receives the stage currently supported by the
evidence, including zero. A control or recording failure is reported as an invalid
trial, with its cause, rather than as a successful demonstration. Photograph and
measure only with power isolated; perform reset after all motion has stopped.

## What the simulator measures

`task.py` implements these coarse external bands and timed stages. Its original
precision guard/lift predicates still model the internal START interlock. They
are part of the apparatus behaviour, not score thresholds. Native contact flags,
hub-fit diagnostics, spring travel and hidden shaft speed remain available for
engineering diagnosis and do not decide any score stage.

`verify_trial.py --rescore` applies the revised rubric to saved native poses and
recorded motor/STOP events without editing the original receipt or trace. It
labels that result as rescoring historical native evidence. It is not a new
simulation, a new Dream-1 inference or a physical hardware demonstration.
