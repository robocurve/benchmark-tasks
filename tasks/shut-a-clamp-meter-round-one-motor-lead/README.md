# Shut a clamp meter's sprung jaw round the middle of three motor leads, one arm holding it, one on the trigger

**What the task is.** Electricians and maintenance workers find a failing motor by taking each leg's current at the
starter with a clamp meter. The robot must do that for the red middle lead (T2) of three: hold the meter in one gripper,
squeeze its trigger with the other so the sprung jaw opens (a person does this one-handed with thumb and fingers; the
gripper has no thumb), pass the open jaw between the neighbouring leads, shut it fully round T2 alone, keep the meter
still for 3 s, then reopen it, draw it off and lay it back on its tray. A jaw left resting on the lead, or shut round
a neighbour, is the real error (the meter reads wrong) and never earns the reading. Not tested: reading the display,
the range dial, live voltage.

**Parts, real size (nothing is scaled).**

- Clamp meter, 240 g, loose: the class of AC/DC Hall-effect clamp meter with a 26 mm jaw, on its DC A range. Safety
  yellow ABS body 40 mm thick: handle 66 mm wide x 115 mm long, neck 46 mm wide x 56 mm long, 215 mm overall; dark
  display and black range dial on top. Black jaw ring: 26 mm window, section 6.5 x 24 mm, 39 mm across. The left 210
  degrees of the ring is fixed; the right 150 degrees is the moving half (25 g), hinged on a vertical pin at its base
  and sprung shut (0.15 N m/rad, no preload). The trigger lever is part of the moving half: a black lever with a
  12 mm round pad on the neck's right side, 22 mm out from the neck with the jaw shut, flush with the neck at the
  open stop (0.70 rad, 3.3 N at the pad), where the jaw tips are 22 mm apart.
- Motor leads: three stiff 2 AWG THHN copper leads, 10.5 mm over the insulation, 40 mm between centres (real
  load-side leads on a small starter are this close), vertical, 30 cm of free run between the terminal strip and the
  contactor. T1 black, T2 red, T3 blue.
- Starter, fixed: light grey steel backplate 300 x 460 x 6 mm; black contactor 60 x 140 x 80 mm, 36 to 44 cm above the
  table; mid grey terminal strip 70 x 140 x 60 mm on the table, white lugs.
- Tray: dark grey ABS, 240 x 90 x 6 mm. The meter starts on it, jaw toward the leads.
- Clearances: the open jaw passes T2 between its tips with 4 mm or more to every lead all the way in; shut round T2 it
  clears T2 by 7.75 mm and the neighbours by 12 mm.

**What the scripted demo does.** Both arms, closed loop (`demo_script`, about 70 s, 1,406 steps; the run limit of 3,000
is 2.1 times that). The right gripper takes the meter off the tray by its handle, lifts it level to 11 cm and holds
the shut jaw 2.8 cm short of the leads, 3 mm left of T2's axis. The left gripper comes down over the neck and closes
on the trigger pad and the neck's far face until the jaw reads open. Both arms feed the open jaw straight in so T2
passes between the tips, then move 3 mm to centre it. The left gripper opens (the jaw springs shut round T2), rises
and moves away; the right arm keeps the meter still for 3.6 s. The left gripper returns and reopens the jaw, both back
the meter out, the left lets go and parks, and the right lays the meter on its tray, opens, rises 12 cm and parks. The
demo reads the meter's live pose and its jaw hinge angle from the simulator on every step, plus its joint encoders and
gripper openings; it uses them to trim the holding hand and to stop the squeeze. It never moves, teleports or edits a part.

**Fidelity.** Leads are fixed rigid rods, as stiff 2 AWG is under light contact. Trigger and moving jaw half are one
rigid bell crank on a linear spring with no preload and no hold; a real meter has a torsion spring. The simulator does
not collide the moving half with the meter's own body, so the lever passes through the neck where a real one has a
slot. Mass is uniform through the body plus a battery block in the handle (centre of mass 124 mm behind the window);
there is no iron core. No display, current or circuit is simulated: the simulated reading is the jaw shut round T2
alone. The simulator scores the last step without opening the grippers, so a meter still held scores 0.

## Files

- `shut_a_clamp_meter_round_one_motor_lead.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, the closed-loop `demo_script`).
- `mechkit.py`, `belimits.py`, `closedloop.py`: shared helpers for multi-part objects, geometry checks and the
  closed-loop scripted demo, kept beside the task file.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Take the red middle lead's current: shut the clamp meter's jaw round it for 3 s, then put the meter back on its tray."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=shut_a_clamp_meter_round_one_motor_lead.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3000 --store-frames --log-dir ~/yam-eval/clampmeter-dream

# Scripted demo (full score): right arm holds the meter, left arm works the trigger
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=shut_a_clamp_meter_round_one_motor_lead.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3000 --store-frames --log-dir ~/yam-eval/clampmeter-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps every part exactly where the task file puts it.
- **Runtime per trial:** the demo takes about 70 s of simulated time (1,406 steps) and the run ends within 8 s of the
  release; a Dream-1 trial runs up to 3,000 steps (150 s simulated).
- Wall-clock time on one L40S, including Isaac Sim start-up: about 300 s for the demo and about 410 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` has held for 8 s, otherwise `score()` at the end of the run.
  The long hold lets the demo open its gripper and park before the run ends.

## Scoring

Judged over the whole run. In a real trial use the top camera and a fixed close camera 30 cm in front of the leads at
jaw height that shows the jaw and the display through the hold; the reading is taken from that video. In simulation
the same quantities come from the meter's pose and jaw hinge.

1. **Shut:** the jaw hinge is at 0.05 rad or less (photo: the tips touch).
2. **In:** the window centre is within 9 mm of a lead's axis, on the lead's free run (9 to 33 cm above the table),
   with the window axis within 20 degrees of the lead.
3. **Reading taken:** shut and In on T2, no finger within 2 cm of the trigger, the meter moving under 5 mm and
   turning under 3 degrees, for 3.0 s unbroken. In a real trial the display must also show 4.5 to 5.5 A.
4. **Wrong lead:** shut and In on T1 or T3 at any time. **Fell:** the meter moving down at 0.4 m/s or more at any time.
5. **Released:** at the end every finger point is 3 cm or more from the meter.

Score, first row that fits: wrong lead, fell, or not released, **0**; reading taken, then the meter flat on its tray
(centre within 2 cm of its start, resting within 6 mm of its rest height, tilt 10 degrees or less, yaw 20 degrees or
less), jaw shut, still 1 s, **1.0**; reading taken, meter left anywhere else (hanging on the lead, off its spot, jaw
open), **0.6**; T2 was In with the jaw open 0.3 rad or more at some time but no reading was taken, **0.3**; otherwise
**0**. Doing nothing, pushing the meter along the table, or holding a shut jaw against the lead scores 0. Closing the
jaw by pushing the moving half directly instead of releasing the trigger is allowed.

The contactor and the terminal strip carry the lugs, the live parts of a starter, so an electrician keeps hands off
them and measures on the insulated run. The task file lists them in `ARM_KEEP_OUT`; the demo's arms stay outside.
This is not scored.

## Real-world reset

De-energise the panel. Join the three lead ends at the motor end and feed T2 from a bench supply limited to 5 A at
12 V or less, so 5 A goes out on T2 and 2.5 A returns on each of T1 and T3: only T2 alone reads 5 A, and a jaw round
a neighbour or round two leads reads 2.5 A. Zero the meter's DC A range with the jaw empty, then lay it on the tray
outline, jaw shut and toward the leads.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
