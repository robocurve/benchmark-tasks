# Rinse a molar inside a true-size mouth while the other hand's saliva ejector catches the water

## Credit

This task rebuilds an idea first submitted by SophiaZhuge1 as pull request #1160, "Dental hygienist assistant:
rinse, clear the mouth and return both tools" (TK-U5JBG2, revised as TK-9JD4EJ and TK-G8UHHR). Its final review
said the task "leaves out the mouth", and invited a rebuild "with real-size teeth inside an enclosed mouth (for
example a typodont in a printed shell), with real-trial checks a person can make". This version is a new
implementation written from scratch. The original's code and assets were not used.

How the review points are addressed:
- **The mouth.** True-size typodont teeth sit inside an enclosed shell: lips with a 56 x 60 mm opening, cheeks,
  palate and upper teeth, tongue, floor of the mouth, pockets and throat. The tools enter only through the lips.
- **Real-trial checks.** Every scored quantity is a volume a person can measure with cups and a scale (see Real
  rig). The debris is a visible smear that washes off.
- **Earlier rounds.** The category is Impact on jobs, as round 2 asked. The action is called rinsing, not plaque
  removal, as round 1 asked. The equipment is standard and affordable.

## Run

Simulator: yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), station scene,
`-E object_jitter=0`.

    inspect-robots run ... -E task=tasks/rinse-the-molar-while-suction-catches-the-water/task.py --epochs 2

Run it with `--epochs 2`. Trial k uses lot k mod 2 (A, then B). `TASK_LOT=A` or `TASK_LOT=B` pins every trial to one
lot. The run limit is 1,200 steps (60 s at 20 Hz), and the demo needs about 49 s. Set `TASK_DEBUG=1` to print the
water state once a second.

Files: `task.py` (scene, water model, scoring, demo), `near_misses.py` (offline cases), and `parts.py`, `lots.py`
and `tilt.py` (helper kit: parts with hinges and visual meshes, layout lots, tilted grips). `tilt.py` is imported by
`lots.py` but is not used by this demo.

## Scene

Dimensions are typical adult values; the exact numbers are in `task.py`.

- **Head.** A dental training head (typodont in a printed shell), loose, 1.8 kg, cut down to the face around the mouth (88 mm wide, 110 mm deep, 143 mm tall), lying low with its mouth toward
  the robot. This is the posture for lower-arch work on a reclined patient: chin down, lower biting plane level,
  75 mm above the table.
  - **Teeth.** 28 true-size crowns. The lower first molar is 11 x 10.5 x 7.5 mm, and the lower first molars are
    46 mm apart centre to centre. The front teeth are 50 mm apart, a wide-open adult mouth.
  - **Lips.** 10 mm thick, with an opening 56 mm wide and 60 mm tall.
  - **Inside.** The cheeks are 72 mm apart. Above are the palate and upper teeth. The tongue is 17 mm wide. The
    floor of the mouth is 16 mm below the biting plane.
  - **Pockets.** Behind each last lower molar is a pocket 16 x 22 mm and 8 mm deep. Its back lip is only 4 mm high,
    so it holds 1.4 mL; past that, water runs into the throat.
- **Saliva ejector.** 30 g, on a three-post rest on the robot's left. A 6.3 mm tube: the last 31 mm is bent down,
  then 75 mm runs back to a 70-degree bend toward the robot's left, then 35 mm of tube and an 80 x 13.9 mm valve
  handle.
- **Air/water syringe.** 90 g, on a two-post rest on the robot's right. A 110 x 13.9 mm handle with an 18 mm square hose coupling on its back end, a 16 mm head and a
  5.4 mm tip: 55 mm straight, then the last 25 mm bent 45 degrees down. Its water runs at 1 mL/s from 20 s to 30 s.
- **Lots.** A: the head on the centre line, square to the robot. B: 5 mm nearer, 2.5 cm to the right, turned
  8 degrees clockwise. Lots stop a replay of one fixed set of tool poses, not a memorised replay of both lots.

## Water model (what the score reads)

- **The jet.** Each control step while the water runs, the jet is a straight line from the nozzle along the bent
  tip.
  - If the nozzle is outside the mouth and the line does not pass through the lip opening, the water lands outside
    (spilled).
  - Otherwise it lands on the first lower tooth it meets, or on the tongue, floor or throat.
  - Water landing on the target molar counts toward rinsing it. 3 mL clears the debris, drawn as a smear that
    shrinks and then disappears.
- **Draining.** Water landing in the mouth drains to the pocket on that side of the tongue, with a 0.4 s lag. What a
  pocket cannot hold (1.4 mL) runs over its back lip into the throat.
- **Suction.** While the ejector's opening is inside a pocket and no higher than its back lip, it draws up to
  2 mL/s from that pocket.
- **Fail rules, judged over the whole trial.**
  - More than 1 mL in the throat.
  - More than 1 mL spilled.
  - Either tool's tip past the pockets and below the floor of the mouth (in the throat).
- The score never reads the robot's commands; it reads only the tools' and the head's poses.

## Scoring

- **0** if any fail rule has tripped, or if the hand state is unknown. The release check fails closed.
- **0.25** when the debris is rinsed off (3 mL on the molar).
- **0.5** when, in addition, the water has stopped and at most 0.2 mL is left in the mouth.
- **1.0** when, in addition, both tools are back on their rests and released, and the hold has lasted.
  - On its rest: each tool within 10 mm and 10 degrees of its start, within 3 mm of its start height, and tilted at
    most 8 degrees.
  - Released: every point on the finger and gripper collision surfaces is more than 5 mm from both tools. Each hull
    face is sampled about every 3 mm, not just its vertices.
  - The hold: both tools still for 1 s.

Without suction, the water overflows the pocket on the target's side into the throat before the debris is clear. The
same happens with the ejector in the other pocket, in the throat, or held above the water. `near_misses.py` checks
each of these.

## Real rig and checks

- **Head.** A commercial adult typodont, set in a 3D-printed head shell with the dimensions above. The shell has a
  drain hole at the back of the throat over a small collection cup, and the head stands loose on its marks, weighted to 1.8 kg with shot in its base.
- **Syringe.** A stainless air/water syringe handle with a disposable tip, fed by a 12 V peristaltic pump (about
  60 mL/min) through a timer relay set to run from 20 s to 30 s. It stands in for the thumb button, which a parallel
  gripper cannot press while holding the syringe.
- **Ejector.** A disposable saliva ejector on a portable suction unit with a valve handle.
- **Debris.** A 9 mm spot of water-soluble disclosing gel on the molar. Check from the video that it is gone.
- **Measurements.**
  - Throat: weigh the throat cup.
  - Spill: weigh the bib.
  - Left in the mouth: wick it out with a pre-weighed swab.
  - Tips: the video shows whether either tip went into the throat.
  - Rests: measure each tool's position on its rest with a ruler against marks.

The rig has not been built. The thresholds are this task's choices, not clinical standards:
- 1 mL in the throat is about a fifth of a teaspoon, enough to make a reclined patient cough.
- 0.2 mL left is about what a swab picks up from a dried mouth.

The water is plain tap water, 10 mL a trial. A drip tray under the syringe's rest catches it if the syringe is never picked up. The grippers hold only the handles, outside the mouth, and the bent tip aims the jet down and away from them.

## Demo

The demo is open loop. It uses the head's pose as the top camera shows it; it never reads the lot id, the water
state or exact simulator poses.
1. The left hand lifts the ejector off its rest and brings it level through the lips, high enough for the bent tip
   to clear the lower front teeth.
2. It lowers the tip into the pocket behind the right lower molars and holds it there.
3. The right hand waits until the ejector is down, then brings the syringe in. Its nozzle sits 25 mm short of the
   molar and as high above it, so the 45-degree jet lands on the biting surface. It aims 3 mm high to allow for the arm sagging under the syringe's weight.
4. Both hold while the water runs. The syringe comes out first, then the ejector, once the pocket is dry.
5. Both go back on their rests, and the hands let go.

## Licences

There are no third-party assets. All geometry is built from primitives in `task.py`.
