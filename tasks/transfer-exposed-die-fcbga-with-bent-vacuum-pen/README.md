# Transfer an exposed-die FCBGA chip from a waffle pack to a test socket with a bent vacuum pen

Lidless flip-chip packages (an AMD/Xilinx SBG484-class 14 x 14 mm bare silicon die on a 19 x 19 mm substrate with a 3.5 mm
45-degree pin-1 corner chamfer, as used on AI accelerators and FPGAs) are moved in bring-up and test labs with a handheld
bent-probe vacuum pen, not tweezers or grippers: the chip lies 0.8 mm below the waffle-pack rim with no edge to hold, metal
touching the die chips it, and a gripper would crush the solder balls under the substrate. A pen with a bellows button needs a
set order: squeeze the button before the cup touches down (this expels the air), bring the cup into flat centred contact on
the die with a seal-forming press (>= 0.6 N for >= 0.20 s), release the button to draw the vacuum, lift the chip clear of the
pocket walls, transfer it under suction over the keyed test socket and lower it to hover 4 mm above the floor with pin-1
corners aligned, squeeze again so the vacuum breaks and the chip drops flat into the locating cavity, then put the pen back on
its bench cradle. Here one arm holds the pen pitched 45 degrees and the other squeezes its side button, which is what a person
does with two fingers of one hand (on a real pen the index finger holds the button down while the cup is set on the part, then lifts off to draw the vacuum; the second arm stands in for that finger). The instruction is "Use the bent vacuum pen to transfer the chip from the waffle pack into
the test socket with pin-1 corners aligned, then return the pen."

## The parts, at real size

| Part | Real product (maker, number) | Size and material in the scene |
|---|---|---|
| Vacuum pen | Virtual Industries PEN-VAC V8901-A-ESD, with bent probe V3950-B (18 ga, 12.7 mm bent) and bellows-free conductive silicone cup V9038-ESD (Virtual Industries datasheet) | Aluminium barrel 12.7 mm OD x 146 mm (12-sided collider, silver), 15 mm nose cone, steel probe 8 mm straight then 18 mm at 45 degrees (the catalogue's 12.7 mm bent length plus the cup's push-on socket; 3.0 mm collider, the real 18 ga tube is 1.27 mm), black silicone cup 9.53 mm OD x 5 mm, face tilted so it is horizontal when the barrel is pitched 45 degrees. 40 g: base 36 g, button 4 g. Side button 14 x 10 x 4.5 mm, 3.5 mm travel, 714 N/m spring (2.5 N at full travel), 33 mm ahead of the barrel centre. |
| Waffle pack | Entegris H44-999-62C02, 4 in STAT-PRO conductive 3 x 3 tray | Charcoal, 101.6 x 101.6 x 6.5 mm, 3.2 mm floor, 3.3 mm walls, nine 25.32 mm square pockets at 30.8 mm pitch (5.48 mm between pockets, 7.34 mm border). |
| Chip | AMD/Xilinx SBG484 lidless FCBGA (19 x 19 mm, 0.8 mm pitch, 45-deg pin-1 chamfer) | 19 x 19 x 1.75 mm emerald substrate with a 3.5 mm 45-degree pin-1 corner chamfer at (-x, -y) plus a 14 x 14 x 0.75 mm silver bare die (2.5 mm in all) and gold pin-1 mark on top, 5 g, lying in the centre pocket with the die top 0.8 mm below the rim (3.16 mm clearance per side). Solder balls are not modelled. |
| Test socket | Open-top 19 mm BGA nest of the Enplas OTB-484 (38 mm body, 0.1-0.2 mm/side nest clearance, pin-1 chamfer) / Ironwood SG-BGA class; a printed stand-in from `fabricate.py` matched to those families' 19 mm locating cavity and pin-1 key, not a measured copy of one part number | Ivory PEEK, 38 x 38 x 10 mm, 19.5 x 19.5 mm locating pocket (0.25 mm/side clearance), 2.5 mm vertical locating walls with a 3.65 mm 45-degree pin-1 corner key and gold bezel mark at (-x, -y), then a 45 degree lead-in funnel to a 26.5 x 26.5 mm mouth. |
| Pen cradle | Printed bench cradle (`fabricate.py`, sized to PEN-VAC V8901-A-ESD 12.7 mm x 146 mm barrel) | Slate, 110 x 40 x 3 mm base, two 10 mm V saddles 100 mm apart (90 mm clear), barrel axis 41 mm above the mat, rear stop 4 mm behind the barrel end. |
| ESD mat | Any 4 mm static-dissipative bench mat cut to size | Blue-grey, 22 x 36 x 0.4 cm. |

Layout (m from the midpoint between the arm bases; x forward, y left): mat centre (0.270, -0.045); waffle pack and chip
(0.305, +0.045), pin-1 chamfer at (-x, -y); test socket (0.305, -0.065), pin-1 key at (-x, -y); pen cradle and pen
(0.245, -0.175), barrel axis 4.5 cm above the table top.

## What the scripted demo does and what it reads

The right arm takes the pen out of the cradle from the -y side, with the pads closing across the barrel 22 mm behind its
centre, lifts it, rolls it 45 degrees nose down and carries it over the die so the cup hovers 25 mm above it. The cup is
centred over the die using the pen's live pose. The left arm comes in from +y, straddles the barrel at the button and squeezes
it (the lower pad stays on the barrel's underside, the upper pad presses the button 3 mm), so the button is down before
touchdown. Both arms lower together 27.0 mm at 1.2 cm/s until the cup makes flat centred contact with the die and presses
2.0 mm past initial contact for 0.5 s (>= 0.6 N); the left arm opens, the button rebounds and the vacuum is drawn, and it
backs away. The right arm re-centres the chip, lifts 28 mm (clear of the pocket walls), carries it 110 mm under suction to the
keyed socket at under 5 cm/s and lowers it to hover 4 mm above the floor. From the chip's live pose it then turns the pen about the
socket centre until the chip lies flat (within 0.5 degrees) and square to the socket key (pin-1 yaw within 0.3 degrees: the
19.5 mm pocket admits the 19 mm chip only within about 1.5 degrees) and centres it within 0.25 mm. The left arm squeezes the button to 1.8 mm
(still sealed) and pauses while the right arm walks out the squeeze load and re-squares and re-levels the chip, then squeezes on to 3 mm: the
vacuum breaks and the chip drops 4 mm into the 19.5 x 19.5 mm keyed pocket. The right arm
lifts the pen clear while the button is still held (so the cup cannot re-seal), the left arm opens and goes to its ready pose,
and the right arm brings the pen back, unrolls it to level, sets it on the saddles, opens and goes to its ready pose. It then
waits 2.5 s with both grippers well clear.
The demo reads the live poses of the pen and the chip (position, tilt and yaw) and the button's travel from the simulator (to centre
the cup, to level, square and centre the chip over the socket and to stop the squeeze at 1.8 mm and 3 mm), plus its joint encoders and
gripper openings. It never moves an object or writes a joint.

## Fidelity note

The vacuum is a scorer-side hold, not a fluid model. Suction engages only if the button was squeezed 2.5 mm while the cup was
at least 5 mm above the die, the cup then makes actual contact (cup-to-die collider gap <= 0.3 mm, `dz` in -1.5 to +0.3 mm,
radial offset <= 2.0 mm so the 9.53 mm cup sits wholly on the 14 x 14 mm die, tilt <= 4 degrees) with a sustained seal-forming
press (>= 0.6 N measured by a PhysX contact view between `vac_pen/base` and `chip/base` for >= 0.20 s) while primed, and the
button returns within 0.6 mm of rest while still in contact. Once sealed, the chip is written onto the cup at its flush offset
before every physics step (240 Hz) until the button is squeezed 2.5 mm again. The real V9038-ESD cup (a bellows-free silicone
cup on a squeeze-bulb or button pen) pulls about 6 in.Hg and lifts 150 g; the 5 g chip is far inside that. The bellows is a
spring button (2.5 N), the cup and probe are rigid (press compliance comes from arm/grip compliance rather than cup skirt
flex), the probe collider is 3.0 mm (real 1.27 mm), the chip underside is flat (no balls; alignment is carried by the 3.5 mm
pin-1 chamfer and socket corner key), and a held chip passes through geometry (a sideways drag below the rim is scored as a
violation). A held chip never scores better than a released one and the demo never moves the chip itself. Pad force on the pen
is the gripper's closing stiffness times about 6 mm (about 32 N per finger); the button needs 2.5 N, so nothing the real action
needs is near 40 N.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Use the bent vacuum pen to transfer the chip from the waffle pack into the test socket with pin-1 corners aligned, then return the pen."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=transfer_exposed_die_fcbga_with_bent_vacuum_pen.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3000 --store-frames --log-dir ~/yam-eval/fcbga-dream

# Scripted demo (full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=transfer_exposed_die_fcbga_with_bent_vacuum_pen.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 3000 --store-frames --log-dir ~/yam-eval/fcbga-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: every part stays where the task file puts it.
- **Runtime per trial:** the demo takes up to about 105 s of simulated time (2,050 to 2,100 steps at 20 Hz) plus the 8 s success hold; a
  Dream-1 trial runs up to 3,000 steps (2.5 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 420 s for the demo and about 490 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 8 s (full score, both grippers 10 cm clear), otherwise `score()`
  at the end of the run.

## Scoring

At the end of the run, in this order:

1. **Score 0 for the whole run** if: a gripper pad touches the chip for 0.1 s (2 steps); the chip falls under -0.55 m/s for 0.1 s;
   it rests on the table or the pack rim (underside under 6 mm above the table top outside every pocket and the nest mouth);
   it is released (button squeezed while sealed) away from a pocket or the nest, or more than 15 mm above the floor under it; it
   is carried sealed below the pocket rim with any part of it outside its pocket; or the pen falls more than 5 cm below the table top.
2. **Suction** forms only when the button goes from 2.5 mm squeezed to within 0.6 mm of rest after: (a) the squeeze began with
   the cup at least 5 mm above the die (or > 10 mm off-centre); (b) the cup made actual contact (`gap <= 0.3 mm`, `dz` within
   -1.5 to +0.3 mm, radial offset <= 2.0 mm, cup tilt <= 4 degrees) with a sustained seal-forming press (`>= 0.6 N` for
   `>= 0.20 s`) while primed; and (c) the cup is still in flat, centred contact when the button returns to rest. Releasing in
   mid-air, across an air gap, without a press, or tilted/off-centre draws nothing.
3. **Tiers**, with the chip released (vacuum off), still (under 5 mm/s) and both grippers at least 3 cm clear for the last 2 s:
   **0.25** the chip was lifted by suction at least 10 mm above the pocket rim (underside above TOP + 20.5 mm) and now rests in
   a pocket or in the nest; **0.60** the chip completed that suction lift and transfer into the socket (checked in the trial
   video) and is seated flat on the keyed nest floor (centre within 1.0 mm, underside within 1.0 mm of the floor, tilt within
   3 degrees) with pin-1 corners aligned (yaw within 8 degrees); **1.00** also the pen is back on the cradle (centre within
   15 mm, barrel within 10 degrees of level), still and both grippers 3 cm from it. Anything else is 0.

Doing nothing scores 0. Pushing or placing the chip by any non-suction method scores 0 (both 0.60 and 1.00 require the verified
suction lift and transfer, checked in the trial video). A chip left on the pen scores 0.

## Real-world build and trial

**Parts list (USD, as listed in 2026-11; re-check before ordering):** Virtual Industries PEN-VAC V8901-A-ESD with V3950-B probe and
V9038-ESD cup, 83.60 (distributor price; list 113.20); spare V9038-ESD cup, 2.50; waffle pack: surplus Entegris H44-999-62C02, about
10, or the printed `waffle_pack.stl` in ESD-PETG, about 3; chip: a dead or dummy SBG484 about 10, or the printed `chip_dummy.stl`
(green PETG with 3.5 mm pin-1 chamfer, glue a 14 x 14 x 0.75 mm silicon offcut or aluminium shim on top, steel shim to 5.0 g),
about 2; keyed test-socket nest `test_socket.stl` reproducing the Enplas OTB-484 / Ironwood SG-BGA 19 mm cavity (19.5 x 19.5 mm
locating pocket, pin-1 corner key; commercial Enplas / Ironwood sockets cost 150 to 400), about 4 of PETG; bench cradle
`pen_cradle.stl` (sized to PEN-VAC V8901-A-ESD), about 2; ESD mat, about 15. **Total about 112 USD** with the printed route (the
pen is the only part over 20). Print at 0.2 mm layers, 20 % infill; every size is the scene's (`python3 fabricate.py out_dir`
writes ASCII STL in mm; nothing is written on import).

**Assembly:** lay the mat flat; put the waffle pack, keyed nest and cradle on it at the layout above (mark their outlines with
tape, matching the socket's pin-1 corner key at front right `(-x, -y)`); lay the pen level in the saddles, button up, nose
toward +x, with the rear cap 4 mm from the stop; drop the chip die-up in the centre pocket with its pin-1 chamfer at `(-x, -y)`.
Use a wrist strap and ground the mat.

**Real-trial scoring** (photo, ruler and one overhead video): the squeeze before touchdown is seen as the button down before
the cup meets and presses onto the die; suction lift (>= 10 mm above the rim) and transfer into the socket are verified in the
trial video; the nest seating is checked with a ruler and straightedge (<= 1.0 mm centre/height, flat, pin-1 chamfer matched to
the socket key); 'vacuum off' is the chip lying free in the nest with the pen lifted away from it.

**Reset and cleaning:** put the chip back in its pocket, die up, pin-1 chamfer at `(-x, -y)`; set the pen on the cradle with
its button released; wipe the cup with isopropyl alcohol and lint-free wipe between trials, and replace it (2.50) if it sticks
or marks the die. Nothing is consumed.

## Files

- `transfer_exposed_die_fcbga_with_bent_vacuum_pen.py`: the task file for the `yam_arena` embodiment (scene, `score()`, `success()`,
  the suction hold and the demo).
- `fabricate.py`: print files for the keyed nest, cradle, pack and chamfered dummy chip.
- `mechkit.py`, `closedloop.py`, `belimits.py`: the shared kit the task imports.
- `task.json`: the task's details.

## Licence

All geometry is procedural, generated by mechkit.py in this folder; no third-party assets.
