# Dose 10 mL from a bottle-top dispenser, stop to stop

**Instruction:** "Dispense one 10 mL dose of the solution from the bottle-top dispenser into the empty 50 mL tube, then
stand the tube back in the rack and let go."

A fixed-volume 10 mL bottle-top dispenser stands on a 500 mL reagent bottle. Its discharge tube points to the robot's
right and turns down at its end. An empty 50 mL tube waits in a rack beside it. The maker's operating manual
(BRAND Dispensette S, "Dispensing", steps c-e and the caution under them) gives the method:

1. hold the discharge-tube orifice on the inner wall of the receiving vessel;
2. lift the piston gently until the upper stop;
3. press it slowly and steadily, with minimal force, until the lower stop;
4. after use, always leave the piston pressed down to the lower stop (its parking position).

The stated accuracy holds only when both stops are approached gently and reached. The maker also warns that a stream
fired from the spout splashes.

**Why it is hard.** A tube standing in the rack can't reach the orifice: its rim is 90 mm below it. So one hand has to
hold the light open tube up around the 7 mm tip, the orifice 3-12 mm inside a 27 mm mouth. The other hand drags the
40 mm sleeve 40 mm up against 4 N of friction to a hard stop, then 40 mm back down to the other stop, without pulling
the bottle or the tube off the tip. Afterwards the tube goes back into a 36 mm hole. The natural shortcuts each fail:

- pumping into the racked tube fires the dose from outside the tube, so it counts as spilled;
- a part lift doses short;
- an extra stroke doses too much.

## Run and score a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=dispense_from_a_bottle_top_dispenser_stop_to_stop.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Dispense one 10 mL dose of the solution from the bottle-top dispenser into the empty 50 mL tube, then stand the tube back in the rack and let go." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir logs/dispenser-dream

# The scripted two-arm demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=dispense_from_a_bottle_top_dispenser_stop_to_stop.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Dispense one 10 mL dose of the solution from the bottle-top dispenser into the empty 50 mL tube, then stand the tube back in the rack and let go." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir logs/dispenser-demo
```

The run prints `task_score`, read at the end (latched zeros aside). Runtime per trial: at most 1,800 steps (90 s
simulated). The demo's plan lasts 45.3 s and reaches 1.0 at step 823 in the MuJoCo check. The layout is fixed
(`object_jitter=0`) and there are no seeds. Tests: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q`.

## Scoring (`grade()` in the task file)

A stroke is a lift of at least 5 mm and then a press of at least 5 mm. A stroke is clean when all three hold:

- its lift reached the upper stop (within 1 mm);
- the whole press was made with the orifice inside the tube (at least 3 mm below the rim, within the mouth);
- it ended at the lower stop.

Liquid pressed out while the orifice is outside the tube counts as spilled.

| Score | State at the end |
|---|---|
| 0 | under 2.0 mL in the tube, or a latched zero |
| 0.25 | at least 2.0 mL in the tube |
| 0.5 | at least 9.0 mL in the tube, and the tube stands in a rack hole within 10 deg of vertical |
| 0.75 | exactly one stroke, and it was clean; 9.90-10.10 mL in the tube; at most 0.3 mL spilled; tube in a hole within 10 deg |
| 1.0 | all of 0.75, plus: the piston within 2 mm of its lower stop; the bottle upright within 5 deg and within 20 mm of its cross; both grippers at least 30 mm from the bottle, dispenser, tube and rack; nothing moving faster than 5 mm/s for 1 s |

Two or more strokes, or more than 0.05 mL pressed out with the orifice outside the tube, cap the score at 0.5. (0.05 mL is 0.2 mm of travel; the solver's micron jitter at a stop stays far below it). The `in_rack`
variant presses a whole dose with the orifice outside: 10 mL spilled, latched 0. A tube tipped past 90 deg
pours its water out, which counts as spilled. Three things latch 0 for the trial:

- the bottle tips past 15 deg or moves 100 mm off its cross;
- the tube falls outside the tray;
- more than 3.0 mL is spilled.

**Measuring it in a real trial.**

- Tare the tube on a 0.01 g balance before the trial and weigh it after (1 g = 1 mL).
- Weigh the tray's towel before and after for the spill.
- Count strokes on the side video, and check there that the sleeve reached the upper stop and that the orifice was
  below the tube's rim for every press.
- Read the sleeve's height with a steel rule.
- Check the tube's and the bottle's tilt with a protractor, and measure the bottle's offset from its cross with a rule.
- Judge release and stillness from the video.

## Layout (x forward from the arm bases, y left; heights above the table)

| Object | Type and material | Size | Mass | Pose |
|---|---|---|---|---|
| catch tray | PP, fixed | 600 x 400 mm, 15 mm lips | - | centred x 250, y -50 mm |
| rubber mat | rubber, fixed | 200 x 200 x 3 mm | - | centred on the bottle's cross |
| media bottle | DURAN 500 mL GL45, borosilicate, 400 mL water | 86 mm OD x 176 mm | 1.2 kg with dispenser | cross at x 200, y 0 mm; free |
| bottle-top dispenser | Dispensette S fixed 10 mL (Cat. 4600 240): PP, borosilicate, PTFE | 50 mm valve block; 40 mm sleeve, 60 mm tall | (in the bottle's mass) | on the neck; spout to the right |
| discharge tube | PTFE, 7 mm | horizontal run at 228 mm above the mat, a bend 85 mm from the bottle axis, then 20 mm straight down | - | orifice 208 mm above the mat |
| piston (sleeve) | PP | 40 mm travel, stops at 0 and 40 mm, 4 N friction | 0.10 kg | parked at 0 (top 300 mm above the mat) |
| tube rack | PP | 170 x 45 x 60 mm, four 36 mm holes on a 40 mm pitch | 0.12 kg | centred x 200, y -150 mm, long side front to back; free |
| 50 mL tube | conical PP centrifuge tube, uncapped | 30 mm OD, 27 mm ID, 115 mm | 13 g | in the hole at x 220 mm; free |

**Real parts:** a BRAND Dispensette S fixed-volume 10 mL (Cat. 4600 240, about 450 USD); a DURAN 500 mL GL45 media
bottle; a 4-place rack for 50 mL tubes; Falcon-type 50 mL conical tubes; a PP catch tray; a rubber bench mat;
a 0.01 g balance.

## Simulation and the real case

- Everything is real size and built from boxes: the round parts are rings of flat plates.
- The bottle, rack and tube are free rigid bodies. The sleeve is a prismatic joint on the bottle, with hard stops at
  0 and 40 mm and 4 N of dry friction each way (static = sliding). Its weight is carried by the piston seals (gravity
  off for the sleeve), so it stays wherever it is left, as a real one does. Set the friction to a spring-scale pull on a real
  sleeve.
- The liquid is a declared analytic model (`Dose` in the task file). The cylinder holds 2.50 cm2 x the piston's
  travel: drawn up on the lift and delivered at the orifice on the press. It counts as in the tube only while the
  orifice is inside it, and as spilled otherwise (the maker's splash warning).
  - The ledger reads the travel clamped to the stops (the solver lets a stop give by a fraction of a mm).
  - Nothing is moved, latched or held by code.
- Simplified: no valves, wetting, drops or splash physics. The tube's 10 g of water is not added to its mass, and
  the solution is water.
- **Real-trial reset:** empty and dry the tube and return it to the same hole. Top the bottle back to 400 mL and
  stand it on its cross. Push the piston to its lower stop. Replace the tray towel.
- **Safety:** use water only, with the dispenser primed and its screw cap off. Run behind the cell's guard with the
  e-stop in reach. A fall breaks nothing worse than a 500 mL glass bottle of water standing in a catch tray.

## The demo (`demo()` in the task file)

**Right hand** (tilted 45 deg toward the other arm, from the tube's right):

1. Grips the tube 35 mm below its rim and lifts it 72 mm out of its hole.
2. Carries it under the tip, rim 18 mm below the orifice, and centres it on the tip's live position.
3. Raises the mouth until the orifice is 7 mm inside and holds it there.

**Left hand** (the mirror, from the bottle's left):

4. Grips the sleeve.
5. Eases it up at 20 mm/s, sent 10 mm past the upper stop. It then corrects from the sleeve's live height until the hand
   presses 8 mm past the stop, because the arm lags under the friction.
6. Eases it down the same way to the lower stop, corrects again, lets go and backs away.

**Right hand again:**

7. Lowers the tube off the tip and carries it back over its hole.
8. Centres the tube's tip over the hole three times from its live pose, dips into the lead-in, centres again and sets
   the tube down.
9. Lets go, backs away and goes home.

## Failure variants (MuJoCo check, `KIT_DEMO=<mode>`)

| Mode | What it does | Score |
|---|---|---|
| demo | the method above | 1.0 (Isaac Lab on the GPU: 1.0) |
| none | both arms stay home | 0.0 |
| in_rack | strokes the piston fully with the tube left in its rack | 0.0 (10 mL spilled, latched) |
| half_lift | lifts the piston only 20 mm, then presses it to the stop | 0.25 (5 mL) |
| held | a clean dose, the tube stood in its hole but never let go | 0.75 |

## Results

Isaac Lab (GPU), current code: demo 1.0, Dream-1 0.0 (it reached toward the rack and tube but never dosed). A first GPU run
scored 0.25: the lift stalled about 6 mm short of the upper stop under the friction. The demo now corrects from the
sleeve's live height until it presses on each stop.

## Licences

All code and geometry in this folder are original and built from boxes in code (MIT). There are no third-party
meshes or images.
