# Press a contact plate flat on a surface for 10 s

**Instruction:** "Take a surface sample inside the marked circle with the contact plate, then put the plate back in its
lid on the tray and let go."

Cleanrooms, hospital pharmacies and food plants check their surfaces for microbes every shift with agar contact
plates. The plate's agar stands 1.5 mm proud of its rim as a convex disc. It is pressed onto the surface, then the
plate is closed and incubated. The plate maker's instructions for use (Biokar/Solabia contact plates, after NF EN ISO
18593) give the method: open the plate, apply the agar directly to the surface and keep a uniform pressure, for
example 500 g for 10 s, then close the plate. Applicators sold for the standard (for example Thermo Scientific EM2001A)
are a 500 g weight with a 10 s timer.

**Why it is hard.** The plate is stored the usual way, agar down in its lid, so the hand must:

1. lift the 25 g base off its lid by the 12 mm of wall above the lid's rim;
2. carry it agar-down to the circle without touching the agar to anything;
3. hold a 3-7 N press (the agar 0.6-1.4 mm in) flat and still for at least 9 s, through an arm that lags by seconds
   under load;
4. lift it straight off and set it back on its lid's ledge.

The natural shortcuts each fail:

- a quick tap gives an incomplete imprint;
- a hard push brings the rim down and crushes the agar;
- a rocked or dragged press smears the imprint;
- agar touched to anything else contaminates the sample.

## Run and score a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=press_a_contact_plate_flat_for_10_s.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take a surface sample inside the marked circle with the contact plate, then put the plate back in its lid on the tray and let go." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/contact-plate-dream

# The scripted one-arm demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=press_a_contact_plate_flat_for_10_s.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Take a surface sample inside the marked circle with the contact plate, then put the plate back in its lid on the tray and let go." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/contact-plate-demo
```

The run prints `task_score`, read at the end (latched zeros aside). Runtime per trial: at most 2,400 steps (120 s
simulated). The demo's plan lasts 75.9 s and reaches 1.0 at step 1410 in the MuJoCo check. The layout is fixed
(`object_jitter=0`) and there are no seeds. Tests: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q`.

## Scoring (`grade()` in the task file)

A **touch** is the agar on the surface at 0.5 N or more. A **press** counts while all of these hold:

- the force is 3-7 N (300-700 g on the scale);
- the plate's centre is within 10 mm of the circle's centre;
- the plate is within 3 deg of flat.

| Score | State |
|---|---|
| 0 | anything else, including the untouched start, or a latched 0 |
| 0.25 | the agar pressed on the surface at 3 N or more, plate centre within 10 mm of the circle's centre |
| 0.5 | one continuous press held for at least 9 s |
| 0.75 | also: only one touch; no slide over 2 mm during it; never over 7 N (the rim never came down); the base seated back in its lid (centred within 3 mm, rim on the ledge), agar down |
| 1.0 | all of 0.75, plus the closed plate on the tray, both grippers at least 30 mm from the plate, nothing moving more than 2 mm for 1 s |

**Latched 0:** the agar touches anything other than the sampling plate (the lid floor, the tray, the table or a
gripper), or the plate falls off the table or tips over.

**Measuring it in a real trial.**

- **Force:** the sampling plate sits on a tared bench scale (0.1 g) with its display in the side camera's view, so the
  press reads directly in grams (100 g = 0.98 N) for every frame of the trial video.
- **Time:** from the video timer.
- **Flatness and slide:** from the agar imprint of the fluorescent tracer dusted in the circle. Under a UV torch, a flat
  still press leaves one crisp 57 mm disc; a slide leaves a streak and a rock leaves a crescent.
- **Centre offset:** measured on the imprint with a ruler.
- **Seating:** checked by eye (rim on the ledge, no gap).

## Layout (x forward from the arm bases, y left; heights above the table)

| Object | Type and material | Size | Mass | Pose |
|---|---|---|---|---|
| sample tray | PP, fixed | 120 x 120 x 3 mm | - | centred x 200, y -190 mm |
| plate lid | polystyrene, open side up, 5 mm inner ledge | 70 mm OD x 9 mm | 10 g | on the tray; free |
| plate base | polystyrene with agar, agar down on the lid's ledge | 65 mm OD x 16 mm; agar 57 mm, 1.5 mm proud | 25 g | in the lid; free |
| bench scale | ABS, fixed | 200 x 200 x 30 mm | - | centred x 260, y 20 mm |
| sampling plate | stainless, on the scale, 85 mm yellow tape circle | 200 x 200 x 2 mm, top 32 mm up | - | on the scale; fixed |

**Real parts:**

- a box of Biokar/Solabia Ø 65 mm pre-poured contact plates (about 1 USD each);
- a 200 x 200 mm stainless plate on a 3 kg x 0.1 g bench scale;
- a PP sample tray;
- yellow tape;
- fluorescent tracer powder and a UV torch.

## Simulation and the real case

- Everything is real size and built from boxes; the round walls are rings of flat plates.
- The base and lid are free rigid bodies.
- The agar is a disc on a slider in the base: a 5 N/mm spring over its 1.5 mm travel, after which the plate's rim meets
  the surface. Real agar is stiffer, so the real hand travels less for the same force; the real arm holds the band on its
  own compliance (it lags by seconds under load), which the demo's servo already does, reading the force from the scale.
- The scorer reads that spring's force, the base's pose and the lid's pose each step. Nothing is moved, latched or held
  by code.
- No microbes or tracer are simulated.
- **Tracer prep (2 min):** with a soft brush, dust a pinch (about 0.1 g) of fluorescent tracer powder (for example Glo
  Germ) evenly inside the 85 mm circle. Check under the UV torch that the whole circle glows; after a press the imprint
  shows as a dark disc in it.
- **Low-cost rig:** the 500 g applicator (Thermo EM2001A) is replaced by any 3 kg x 0.1 g digital table scale (for example
  the Steinberg basic 3 kg / 0.1 g table scale, about GBP 40,
  https://www.expondo.co.uk/steinberg-basic-digital-table-scale-3-kg-0-1-g-10030144), a UV torch and a box of plates.
- **Real-trial reset:** use a fresh plate each trial, stored agar down on its lid on the tray. Wipe the sampling plate
  with 70% ethanol, re-dust the tracer in the circle and tare the scale.
- **Safety:** sterile plates and a clean steel plate only, with no culture handled. Run behind the cell's guard with
  the e-stop in reach.

## The demo (`demo()` in the task file)

The right hand does all of it:

1. Comes down round the base, its pad tips above the lid's rim, and pinches the 65 mm wall.
2. Lifts the base 30 mm and carries it agar-down over the circle, correcting from the base's live pose.
3. Descends at 5 mm/s to the first touch.
4. Holds the press on a damped integral servo on the agar's compression (target 0.9 mm, 4.5 N). The command rises
   with the compression's rate, acts on the compression expected 2 s ahead (the arm lags by seconds under load) and
   backs off at full speed when that heads over 6 N, so the lagging arm does not overshoot into the rim. The hand keeps its own clock, like a 10 s timer: it holds until it has had 11 s in a row at 3.3-6.7 N.
5. Raises its command until the agar is free, then 15 mm more (the arm lags, so it would settle back onto the plate),
   and lifts straight up.
6. Carries the base over the lid, corrects from the lid's live pose and lowers it onto the ledge. The press slides the base
   up in the grip by a few mm, so the hand stops wherever its pad tips stay 3 mm above the lid rim. It centres the base
   on the lid once more and lets it drop the last few mm onto the ledge.
7. Opens, rises straight clear and goes home.

## Failure variants (MuJoCo check, `KIT_DEMO=<mode>`)

| Mode | What it does | Score |
|---|---|---|
| demo | the method above | 1.0 (step 1410) |
| none | the arm stays home | 0.0 |
| tap | presses for 1 s and lifts | 0.0 (never reaches 3 N in 1 s) |
| hard | pushes straight down at 4 mm/s until the rim is on the surface, for 13 s (the test file drives this variant: it patches the demo's servo in a scratch copy, scored by the unchanged scorer) | 0 (it passes 0.5 in the band, then ends at 0) |
| slide | a good press, the hand dragged 24 mm sideways during it | 0.5 |
| held | a good press, plate back in its lid, never let go | 0.5 (the hand still presses the base off its ledge) |

## Robustness (MuJoCo, one factor at a time)

1.0 at agar spring x2 and base mass x0.75 and x1.25. 0.5 at agar spring x0.5: at 2.5 N/mm the rim comes down at 3.75 N, so
the 3-7 N band is all but closed.

## Licences

All code and geometry in this folder are original and built from boxes in code (MIT). There are no third-party
meshes or images.
