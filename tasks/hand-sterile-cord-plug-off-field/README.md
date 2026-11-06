# Pass a sterile device cord off the field to the console

Operating-room mock-up (no patient). The left half of the table holds a raised sterile field: a 5 cm tall box under a
blue drape whose right-hand edge lies on the table's midline. A white 8 mm, 45 cm device cord leaves a holster fixed
on the drape and lies straight across it, ending in a black 80 x 16 x 14 mm plug. A grey cord holder sits at the
field's right edge, on a line aimed at the console: an open-top slot on the drape (8.5-12.5 cm in from the edge, measured along that line)
and a flat ledge that reaches 8.5 cm beyond the edge. A grey device console stands on the right half; its jack faces the
holder, 16 cm beyond the field edge along the same line. The console is a plain mock-up box, not a real device.

The left arm plays the scrubbed (sterile) team member and must stay on the field side of the edge; the right arm
plays the non-sterile circulator and must never cross over the field. The left arm lays the cord in the holder slot
and leaves the plug on the ledge, off the field. The right arm takes the plug from the ledge and pushes it fully into
the console jack. The plug must never touch the table, and both arms let go at the end. The instruction gives only the
goal and says which arm is sterile; the zone, hand-off and table rules are tested, and stated under Scoring.

Why two arms: the zone rule keeps each arm on its own side of the edge, and reach backs it up. Along the hand-off
line, with the gripper pointing down, the left arm reaches only about 7 cm past the field edge (offline IK check), so even with the plug
pinched at its very end its tip gets at most about 14.5 cm out, short of the 17.5 cm needed to seat it in the jack.
The right arm cannot reach the plug where it starts on the field (offline IK: about 9 cm short). The ledge at the
field edge is the hand-off point, instead of a direct hand-to-hand pass.

Grounding: O*NET 29-2055.00 Surgical Technologists, task "Maintain a proper sterile field during surgical
procedures." A procedure guide ("Surgical asepsis: Maintaining a sterile field", revised May 19, 2025, citing AORN guidelines)
says to "Secure all cables and tubing for equipment to the sterile field with nonperforating devices". It also says
the scrubbed person can set a solution receptacle "to the edge of the waterproof draped table for the unsterile
person to pour so that the unsterile person doesn't have to reach across the sterile field". AfPP Standards and Recommendations for Safe Perioperative Practice
(Fifth Edition, aseptic technique extract), 5.5.18: "non-sterile staff should not lean over the sterile field". 5.5.21: "Scrubbed staff must keep their arms and
hands well within the sterile field at all times." This is a tabletop
mock-up with no patient; it borrows only these placement and reach rules.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cord simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Plug the sterile white cord into the grey console's jack, with the cord lying in the grey holder's slot. The left arm and the blue drape are sterile, the right arm is not. Let go with both arms at the end."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

The left arm pinches the plug 10 mm from its cord end and turns it to point along the hand-off line. It brings the
plug in along the holder line from behind the slot, so the cord trailing behind it lies in the slot. It sets the plug
on the ledge with the tip off the field while the gripper stays over the field, then moves back. The right arm
pinches the plug on the ledge 40 mm from its cord end, with its wrist camera towards the jack. It slides the plug
along the line into the jack (closed loop on the live plug pose), lets go and backs off. Test-only variants:
`STERILE_CHEAT=nothing | drop | scrub_carry | noclip | hold | hold_cord | reach_over` (all score 0; `hold_cord` lets
go of the seated plug and closes on the cord 5 cm behind it). `tier03` stops with the plug on the ledge and both arms
clear (scores 0.3). `drop` does not isolate the
table rule: its re-grasp also fails to seat the plug, though the log shows the table rule latching when the plug lands.

## Scoring

Tiers by end state (cord in the holder slot and both arms let go in each case): 1.0 plug tip at least 15 mm inside the jack, lined up; 0.3 plug resting on the ledge with its centre beyond the field edge, not in the jack; otherwise 0.

`score()` checks every control step, and the result stays latched until reset (zones use the gripper centres only):
- the left gripper centre must stay at y >= 0 (the field side of the edge);
- the right gripper centre must stay at y <= 0;
- the plug's lowest point must stay more than 2 mm above the table.
- when the right gripper centre first comes within 3 cm of the plug centre, the plug centre must already be off the
  field (y < 0): the circulator takes the plug only off the field.

At the end, it gives 0 if any of these was broken, if no cord segment lies inside the holder slot, or if a gripper
centre is within 4.5 cm of the plug's centre line (end to end) or 3 cm of the cord. Otherwise it gives 1.0 when the plug tip is at least 15 mm inside the
jack and lined up with it (centre within 8 mm of the jack axis, pointing within about 25 deg of it). It gives 0.3 when the plug lies on
the ledge with its centre beyond the field edge (the left arm's half done), and 0 otherwise. The episode ends early
with success once 1.0 has held for 1 s with the cord at rest.

## How this differs from similar tasks

Unlike tasks where a part is simply handed from one arm to the other, a boundary splits the arms by role: the left arm may never leave the draped area and the right arm may never reach over it, so the plug is set on a ledge past the field edge and the right arm takes it from there. The cord must also end in a holder slot and the plug must never touch the table. This tests asymmetric, constraint-defined arm roles with a cord, not a free handover.

## Runtime and seeds

Run limit 1,200 steps (60 s at 20 Hz). The scripted demo scored 1.0 and ended in 491 steps. The layout is fixed with
`object_jitter=0`, and the cord is not jittered. `VARIATION` in task.py lists the ranges for a seeded variation
runner.
