# Refit a gas burner after cleaning: turn the head until its slot drops round the igniter, then seat the cap

Everyone who cooks on gas lifts the burner heads and caps off to wipe up a spill and then has to put them back. The
head sits flat on its base only when the slot in its rim is turned round the igniter. Put down turned wrong, it
rides on the igniter and rocks, tilted 12 to 15 degrees, and the burner then clicks without lighting or lights
unevenly; a cap left off centre makes it burn yellow. Here the pan grate is off, the washed head and cap lie on a
drainer beside the hob, and the head starts with its slot pointing 150 degrees from +x, well away from the igniter's
side, so the slot has to be found by looking and the head turned in the hand before it goes down. The cap then nests
over the head's lip and must end centred and flat. Both are released fits, judged by height, tilt and offset.

## Parts (all full size)

- **Burner base** (fixed, dark grey cast aluminium): flange 84 mm across and 4 mm thick, whose top is the seat;
  locating spigot 52 mm across and 10 mm tall; white porcelain igniter 12 mm across, standing 16 mm above the seat
  with its axis 36 mm from the burner axis, with a steel tip.
- **Burner head** (loose, silver cast aluminium, 160 g): rim 75 mm across, 58 mm inside, 19 mm tall, with one 38 deg
  slot through its full height, which leaves 5.7 mm each side of the igniter, so the head goes down only within about
  9 degrees of the right turn; a crown closing the top over a 13 mm deep cavity (3 mm round the spigot and 3 mm above
  it); a lip 56 mm across and 6 mm tall on top; 18 flame ports round the rim.
- **Burner cap** (loose, matt black enamelled cast iron, 350 g): 76 mm across, 17 mm tall, slightly domed, with a
  recess 62 mm across and 8 mm deep that goes over the lip with 3 mm per side while its skirt rests on the rim.
- **Hob** (fixed): the front corner of a full-size built-in gas hob, brushed steel deck 4.5 cm high running out of
  the work area, two black knobs 34 mm across, and the next burner, assembled, 24 cm along.
- **Drainer** (fixed, pale blue plastic): tray 11 x 18 cm with five ribs 3.5 cm high.

## The demo and what it reads

The left arm does the whole job; the right arm stays parked with its gripper open. The left gripper comes straight
down on the head with its jaw line 90 degrees from the slot, so neither pad lands on the slot, closes on the 75 mm
rim and confirms the grasp from its finger gap (one retry 3 mm lower). It lifts the head, carries it at 5 cm/s to the
burner while turning the wrist 60 degrees clockwise so the slot points at the igniter, trims its position, lowers
the head at 1 cm/s until the rim is 2 mm above the seat and opens. It then checks that the head sat flat; if not, it
lifts it clear of the igniter, turns it by the slot error and sets it down again, once. It takes the 76 mm cap the
same way, carries it over the burner, lowers it over the lip until the skirt is 2 mm above the rim, opens, checks
that the cap is centred and flat (one retry) and parks at the ready pose with the gripper open.

The demo reads the live poses of the burner head and the burner cap from the simulator (the slot's direction at the
pick, each part's offset and height in the hand over the burner, and the two checks after release), plus the left
arm's joint encoders and its finger gap. It never moves, sets or edits a part.

## Fidelity

Every part is at real size and mass (masses follow from the modelled volumes at 2.7 and 7.2 g/cm3). Round parts are
24-sided prisms in collision; clearances are measured on those prisms. Gas, flame and spark are not simulated. The
hob, both burner bases and the second burner are fixed bodies and the pan grate is lifted off, not in the scene. The
head and cap are rigid; the flame ports are drawn as dark notches, not cut. Friction is 0.4 for metal on metal and on
plastic and 0.3 for the enamelled cap, with default contact offsets. The simulator scores the final step without
opening the grippers, so a part still held, within 3 cm of a gripper, or more than 1 cm above every surface scores 0.
Offline replays cannot see the fingers and use the height test alone.

## Files

- `refit_a_gas_burner_head_and_cap.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, `demo_script()`).
- `task.json`: the task's details.
- `mechkit.py`: multi-part objects and procedural round geometry (the slotted rim, the lip, the cap).
- `closedloop.py`: the closed-loop demo controller (encoder feedback, grasp checks).
- `belimits.py`: geometry helpers for the scorer (lowest point, tilt, resting height).

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Put the burner head back on its base so it sits flat round the igniter, then put the cap on it, centred."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=refit_a_gas_burner_head_and_cap.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/burner-dream

# Scripted demo (full score): the left arm refits the head and then the cap; the right arm stays still
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=refit_a_gas_burner_head_and_cap.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/burner-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps the head and cap exactly where the task file puts
  them. The scene has no random element, so no seed is needed.
- **Runtime per trial:** the demo moves for about 46 s of simulated time (930 steps) in the offline replay and about
  52 s with contact, where the centring rounds add a few seconds; the episode ends about 8 s after the cap is
  released; a Dream-1 trial runs up to 2,400 steps (2 minutes simulated).
  Wall-clock time on one L40S, including Isaac Sim start-up: about 210 s for the demo and about 340 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` has held for 8 s (head and cap seated, released, at rest),
  otherwise `score()` at the end of the run.

## Scoring

Heights are measured from the seat, the flange's top (4.9 cm above the table).

- **Head seated:** tilt under 2 deg, lowest point within 1.5 mm of the seat (1 mm of solver sink is allowed), axis
  within 3.5 mm of the burner axis, and the slot's centre line within 12 deg of the line from the head's axis to the
  igniter (the igniter stands in the slot).
- **Cap seated:** right way up, tilt under 3 deg, axis within 4 mm of the burner axis, lowest point 18 to 21.5 mm
  above the seat (it rests on the head's rim, 19 mm).
- **D (drop), latched over the whole run:** the head or the cap ever moves faster than 0.8 m/s, which is a free fall
  of about 3 cm or more, or a throw.

At the end:

1. **Score 0** unless both grippers are at least 3 cm clear of both parts, both parts have been at rest (under
   1 cm/s) for 1 s, and neither is more than 1 cm above every surface it could rest on.
2. **1.0:** head and cap seated, no D.
3. **0.5:** head seated, cap not (still on the drainer, caught on the lip, tilted, upside down), no D.
4. **0.25:** head seated, with D.
5. **0:** anything else. A head riding the igniter, on the deck or left on the drainer scores 0 whatever the cap
   does, so partial credit never rewards the refit mistake.

Doing nothing scores 0. Check on a real trial with a top photo (igniter tip inside the slot), a ruler from the cap's
edge to the flange's edge at four points (each 4 mm when centred) and a straightedge across the cap. Rest is measured from each part's position,
one control step to the next (0.5 mm or less per step at 20 Hz), the way a ruler on two video frames would.

Additional documentation, not a scored rule: the task file declares `ARM_KEEP_OUT`, a 30 mm square round the
igniter below 9 cm above the table, and the demo keeps every arm part out of it, because a gripper pressing on the
porcelain electrode cracks it. Pad contact with the igniter is not scored.

## Real-world reset

Lift the cap and the head off the burner and stand both back on the drainer at their marks: head at x 26, y 13 cm
with its slot pointing 150 deg from +x, cap top up at x 26, y 22 cm. The hob, base and drainer stay fixed. Use a
disconnected hob or a spare burner set; no gas is involved.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
