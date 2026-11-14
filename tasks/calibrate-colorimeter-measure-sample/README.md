# Calibrate a colorimeter on the blank and measure a sealed sample

The blank sets zero absorbance for the cuvette walls and solvent. A wrong reference or ridged faces on the beam gives a plausible but wrong concentration with no warning. The requested wavelength comes from the card. Done by: lab technicians, students. This tests reference-dependent analytical measurement, beyond setting a device value or moving laboratory vessels.

Category: Impact on jobs, because correct reference-dependent measurements are routine analytical laboratory work. The two-arm split is the demonstration's choice: the left operates the panel and the right handles the hood and cuvettes. All assets are locally authored primitives and the included kits; no external assets or licences.

## Revision: earlier review feedback and what changed

The earlier version of this task was not accepted. Its review said: "Flags: The setup screenshot and Dream-1 frames show a 470 card, contradicting the claimed layout 12 and 635 card. Scoring (not met): A sealed cuvette briefly resting at more than 30 degrees permanently zeros the score even if the subsequent measurement and return are correct. What to change: Remove the permanent zero for a released sealed cuvette exceeding 30 degrees of tilt; score actual damage or loss instead, so a correct completed measurement is not invalidated by harmless temporary handling. Correct the stated screenshot and Dream-1 layout or provide media matching the stated 635 nm layout."

Changes in this version, and nothing else:
1. Scoring: the 30 degree tilt zero is removed from the scorer, its tests, task.json scoring and the Scoring section below. Only loss zeros a trial: a cuvette whose lowest body corner goes more than 5 mm below the table top has gone over the table edge. A cuvette that tips over or lies on the table costs nothing by itself, so a correct measurement and return afterwards still score 1.0; new tests cover a tip-over before and after the measurement record.
2. Layout statement: setup.png, the Dream-1 run and the scripted demo were recorded again for this version, all at CLI `--seed 0`, and all show layout 6 (card 470). task.json setup and this README now say so.

The review before that asked for the sample's dye identity, concentration and preparation, or a batch procedure for its targets; both are in Sample, unchanged.

## Run and score

With the official simulator installed, run from this folder with zero object jitter and 20 Hz:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=calibrate_colorimeter_measure_sample.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --seed 0 \
  --instruction "Calibrate the colorimeter on the blank and measure the sample at the wavelength on the card, then return both cuvettes to the rack." \
  --scorer task_score --max-steps 5200 --store-frames --log-dir ./demo-run
```

This demo command uses `--seed 0`, which selects layout 6 (card 470). The Dream-1 run used the same command with `--policy dream`, and setup.png is the first frame of a 2-step run of the same command, both at the same seed, so all three show layout 6. The harness reports task_score at the end. Encode recorded cameras with `inspect-robots video <run.json>`. Scores are in task.json. CPU scorer checks: `python -m pytest -q -p no:cacheprovider` with yam_arena installed.

## Instrument

The wavelength selection is 430, 470, 565 or 635 nm, clamped at both ends, with no wrap. Button travel >=2 mm produces a press edge; <=1 mm rearms it. Edges in a single observation process in the order down-arrow, up-arrow, CAL. The up-arrow points along +x toward longer wavelengths; the down-arrow points along -x toward shorter wavelengths, matching the lamp column. Every wavelength change clears the reference; a press at a clamped end does nothing.

Detector intensity is I=I0(w)*10**(-A_content(w))*F. I0=(1,.94,1.06,.98); blank A=0, sample A=(.12,.22,.20,.78) AU, the nominal values for a 10 mm path (see Sample). F=1 with the clear-face axis within 10 degrees of the x beam, otherwise 10**(-.60). In a 14.5 mm throat the square body can yaw only about 10.2 degrees before its corners meet the walls; the 10 degree optical gate includes the physically aligned state. Other fixture orientations use the frosted-face attenuation. A CAL edge with exactly one seated cuvette and closed hood stores (w,I), without identifying the liquid. An invalid CAL preserves the previous reference. A failed CAL flashes red for 2 s; otherwise a valid reference lights CAL green, or the lamp is off. A successful CAL ends a failure flash.

With one seated cuvette, closed hood and reference for the current wavelength, display A=log10(Iref/I); otherwise dashes. Four labelled lamps show the current wavelength. The sign and three seven-segment digits display X.XX. Sample CAL followed by sample measurement gives 0.00. Ridged blank CAL shifts the sample reading down by .60 AU; a ridged sample shifts it up by .60 AU. Wavelength changes invalidate the reference. Throughout the 1 s measurement window, the sample's clear-face axis must be within 10 degrees of the beam axis and the current reference must be a clear-face blank reference at the requested wavelength: log10(I0/Iref) within .02 AU of 0.

The model uses physical button travel, hood travel and cuvette pose and markings; it reads no contact forces, task score or display prims.

## Scoring

| Score | Current state, unless marked persistent |
|---|---|
| 0 | No satisfied tier, untouched start in every layout, or a cuvette lost off the table (persistent) |
| .25 | Requested wavelength and blank-equivalent clear reference, or a measurement record |
| .50 | Measurement record (persistent) |
| .75 | Record and sample currently in a rack pocket (tilt <=9 degrees), released and still |
| 1.00 | Record and both cuvettes currently in different rack pockets, released and still for 1 s |

Released: every gripper TCP >=30 mm from the cuvette bounding box. Still: origin displacement <.5 mm and orientation change <.5 degrees against every observed pose in the preceding .5 s, linear speed <.01 m/s; lid position change <.5 mm over .5 s. Seated: bottom centre <3 mm from the well axis on each lateral axis; lowest bottom corner between well floor-2 mm and floor+2 mm; tilt <=3 degrees. Closed lid: slide within 2 mm of 60 mm. Rack pocket: bottom centre <3.5 mm from pocket centre on each lateral axis, lowest bottom corner between pocket floor-2 mm and floor+2 mm, tilt <=9 degrees (the 12.5 mm body bottomed in the 15.5 mm pocket can lean on a 20 mm wall up to about 8.8 degrees, and that counts as returned); at most one cuvette per pocket, either pocket. Loss: a cuvette whose lowest body corner goes >5 mm below the table top has gone over the table edge (fallen off) and is lost: score 0 for the rest of the trial. Tipping over or lying on the table is not a loss and is not penalised by itself; a tipped cuvette only fails the seated and rack-pocket conditions while it is tipped, so a later correct measurement and return still score in full. Measurement record: sample seated, released and still, clear-face axis within 10 degrees of the beam axis, current reference at the requested wavelength with log10(I0/Iref) within .02 AU of 0, lid closed and still, valid reading at requested wavelength within .02 AU of the sample batch's measured target for that wavelength (README, Sample; the simulation's targets are the nominal .12,.22,.20,.78 AU at 430,470,565,635 nm), continuously 1 s; record persists. 0.25: current wavelength is requested and current reference's log10(I0/Iref) is within .02 AU of 0 for that wavelength, or record exists. In a real trial, the reference condition (here and in the record) means: the last CAL press was made at the requested wavelength with only the blank seated, its clear faces within 10 degrees of the light path and the hood closed, checked on the trial video. 0.50: record exists. 0.75: record exists and sample is currently in a rack pocket, released and still. 1.00: record exists and both cuvettes are currently in distinct rack pockets, released and still continuously 1 s. Otherwise 0. Tier conditions are current except the measurement record and the loss zero; highest satisfied tier applies. Success: 1.00 for a further 1 s.

A record captures the valid instrument reading rather than credit for merely putting a vessel in a region. The reference tier is current: selecting another wavelength or calibrating again can remove it until a measurement has been recorded. Return tiers can fall again when a cuvette is lifted. A cuvette that tips over on the table and is set upright again scores normally; a cuvette that has gone over the table edge stays at zero. Pose histories reject both slow tipping and excursions that return to their starting point; solver angular-velocity chatter alone does not reject a stationary light cuvette. Offline single-state fixtures assume their supplied poses have been stationary, while full-rate checks exercise the .5 s and 1 s windows. A ruler, instrument display and timed images check a physical trial's positions, readings, release and stillness.

## Physical setup

Positions are on the table top, measured forward from the line through the two arm bases and left (+) or right (-) of the midline between them; the arm bases are 610 mm apart. Nothing else is on the table.

1. **Colorimeter:** a teal ABS plastic housing, 110 mm front to back, 160 mm wide and 65 mm tall, fixed to the table with its centre 168 mm forward and 5 mm right of the midline. Its top has a square sample well, 14.5 mm across and 35 mm deep (floor 30 mm above the table), centred 190 mm forward and 45 mm right. The well mouth is ringed by four 10 mm-diameter rods centred on the housing top, forming a rounded lip whose crown is 70 mm above the table. Two blue printed triangles, 25 mm in front of and behind the well, mark the light path, which runs front to back.
2. **Hood:** a navy ABS plastic cover, 55 g: a 32 x 32 mm roof with two 28 mm side skirts, open at the front and back, and a rubber-coated 24 x 18 x 20 mm grip tab on top (tab top 131 mm above the table). It slides front to back on two 4 mm steel rails, 100 mm long and 92 mm above the table, which run 10 mm and 80 mm right of the midline on small steel posts. The slide has 0.8 N s/m damping and 0.25 static / 0.18 dynamic friction. The hood starts open, its centre 130 mm forward and 45 mm right; sliding it 60 mm forward to the end stop closes it over the well.
3. **Buttons:** three 16 x 16 x 12 mm polymer push-button caps, 8 g each, on the housing top 55 mm left of the midline: down arrow (shorter wavelength) at 132 mm forward, up arrow (longer wavelength) at 168 mm, CAL at 204 mm. Each is sprung (250 N/m; 0.75 N at rest, 1.25 N at 2 mm), travels 5 mm, and registers a press at 2 mm of travel and rearms at 1 mm.
4. **Indicators,** printed or lit on the housing top: four green wavelength lamps labelled 430, 470, 565 and 635 nm, 26 mm left of the midline at 141, 159, 177 and 195 mm forward; a CAL lamp 204 mm forward, 38 mm left; and a sign and three-digit absorbance display with 10 mm seven-segment digits, centred 128 mm forward and 13 mm left.
5. **Rack:** a steel cuvette rack, 44 x 164 mm and 24 mm tall, fixed to the table with its centre 190 mm forward and 250 mm right. It has two 15.5 mm square pockets with floors 4 mm above the table, each ringed by the same 10 mm rods centred on the rack top (crown 29 mm above the table): pocket 0 at 190 mm forward, 310 mm right (in front of the right arm), and pocket 1 at 190 mm forward, 190 mm right.
6. **Cuvettes:** two sealed polystyrene plastic cuvettes, 18 g each: a 12.5 x 12.5 x 45 mm body, a 6 mm square stopper stem and a rubber-coated 14 x 14 x 12 mm stopper head, 63 mm tall overall. They are physically identical; the blank holds deionized water and has a white stopper; the sample has a blue stopper and holds a blue water solution of 5.00 mg/L Brilliant Blue FCF (FD&C Blue No. 1) and 6.00 mg/L Allura Red AC (FD&C Red No. 40). The Sample section gives its preparation and the batch check that measures its scored absorbance at each wavelength. On each, two opposite faces are clear and the other two are frosted grey with dark horizontal ridges. Cuvette A stands upright in pocket 0 and cuvette B in pocket 1, square to the table edges, 1 mm above the pocket floors.
7. **Card:** a white paper (cellulose) card, 120 x 80 x 1 mm, taped flat with its centre 180 mm forward and 225 mm left. It has a blue 6 x 60 mm stripe along its far edge and a three-digit wavelength in 24 mm seven-segment digits, readable from the robot's side.

All loose and moving parts have 1.2 static / 1.0 dynamic friction except the hood. The physical objects never move between the 16 layouts: only the number printed on the card, the lamp lit at the start, which cuvette is the blank and which faces are frosted change (see Layouts). setup.png, the Dream-1 run and the scripted demo all show layout 6 (CLI `--seed 0`).

For exact reconstruction, all local coordinates below are in mm. Housing outer bounds x=+/-55,y=+/-80,z=0..65; bottom slab centre (0,0,15), size (110,160,30). Upper x strips fill x=-55..14.75 and 29.25..55, full y, z=30..65. The two remaining strips fill x=14.75..29.25 and y=-80..-47.25 or -32.75..80. Lead cylinders are centred at (well x +/-12.25,well y,65), axes y, or (well x,well y +/-12.25,65), axes x. Rack x strips fill x=-22..-7.75 and +7.75..22, y=+/-82,z=4..24. Remaining strips fill x=+/-7.75 and y=-82..-67.75,-52.25..52.25,67.75..82. Pocket lead cylinder axes are 12.75 mm from each pocket centre. These floor/wall unions make real openings. Cup bodies use the exact same three collider boxes, friction and contact offset for both identities.

The attached hood slides on steel rails through its runners; its open ends along x prevent a skirt from crossing the cap. The housing is 110x160 mm to contain the 60 mm hood slide and the panel, with the well at world (.190,-.045) m and the open hood at (.130,-.045) m. Closed fingers fit between button caps with at least 8 mm clearance to either neighbour. Every cap is over 50 mm from the well's cuvette box. Button preload is .75 N, force at the 2 mm switch is 1.25 N. At .1 mm contact offsets the lead-ins have 1 mm well and 1.5 mm pocket side clearance. R5 mm surfaces guide released bodies inward, with conservative capture 5.9/6.4 mm per axis, including diagonal offsets.

The printed card digits are 24 mm high with 3.6 mm strokes, with the header band above them along +x. At the left wrist look pose TCP (0.217895,0.225330,0.8975), yaw 0, their calibrated projected glyph heights are 17.292, 17.291 and 17.300 px at 320x176; in the top camera they are 4.462, 4.456 and 4.451 px. Heights are the shorter projected side of each digit, including camera rotation. The demo uses the wrist view. Every printed glyph on the card and instrument has text up along world +x and reading direction along world -y. At the card and panel viewing poses, each authored glyph's projected up and right axes are within 10 degrees of image up and image right.

## Sample

The sample is a blue solution of two food dyes in deionized water, by pure-dye mass: 5.00 mg/L Brilliant Blue FCF (FD&C Blue No. 1, E133, CAS 3844-45-9, 792.84 g/mol) and 6.00 mg/L Allura Red AC (FD&C Red No. 40, E129, CAS 25956-17-6, 496.42 g/mol). Brilliant Blue alone would absorb only about 0.01 AU at 470 nm and 0.02 AU at 430 nm, so calibrating on such a sample would also pass there. The added Allura Red raises every scored absorbance to at least 0.12 AU, so a sample-referenced reading (0.00) always misses its target by at least 0.10 AU. The solution stays blue; commercial blue food colouring uses the same two dyes.

Nominal absorbances for a 10 mm path (Beer-Lambert, A = a x c x 1 cm x the dye's relative spectrum):

| Wavelength (nm) | 430 | 470 | 565 | 635 |
|---|---|---|---|---|
| Brilliant Blue FCF, 5.00 mg/L | 0.023 | 0.008 | 0.142 | 0.774 |
| Allura Red AC, 6.00 mg/L | 0.099 | 0.212 | 0.059 | 0.001 |
| Sum | 0.122 | 0.220 | 0.201 | 0.775 |
| Simulated target (rounded) | 0.12 | 0.22 | 0.20 | 0.78 |

The peak absorptivities are the JECFA specification values, 164 L/(g cm) at 629 nm for Brilliant Blue FCF (2017) and 54.0 L/(g cm) at 501 nm for Allura Red AC (2016), which match EU Regulation 231/2012 (E1%1cm 1630 at about 630 nm and 540 at about 504 nm). The relative spectra at the four wavelengths come from published reference absorption spectra of both dyes (manufacturer application note FL53216 and a second published teaching-lab spectrum), which agree to within about 0.01 AU.

Preparation:
1. Use dyes with a stated lot dye content p (for example Sigma-Aldrich 861146 and 458848, with p from the certificate of analysis), deionized water, an analytical balance (0.1 mg), class A 100 mL and 500 mL volumetric flasks and a class A 5 mL volumetric pipette.
2. Stock: weigh 50.0 mg / p of the Brilliant Blue product and 60.0 mg / p of the Allura Red product, each within 0.3 mg (for example 58.8 mg at p = 0.85 and 75.0 mg at p = 0.80), into the 100 mL flask; dissolve and make up to the mark (500 and 600 mg/L).
3. Working solution: pipette 5.00 mL of stock into the 500 mL flask, make up to the mark with deionized water and mix by inversion (5.00 and 6.00 mg/L).
4. Fill a clean 12.5 x 12.5 x 45 mm polystyrene cuvette (10 mm path, usable 340-800 nm) with 3.0 mL of working solution, about 30 mm deep, and close it with the blue stopper. The blank is 3.0 mL of the same water in an identical cuvette with the white stopper.
5. Store the solutions in amber glass and the cuvettes in a closed box at room temperature, away from light.

Batch check, which sets the scored targets. On the colorimeter used for trials, after its warm-up, at each of 430, 470, 565 and 635 nm, repeat five times: seat the blank with its clear faces on the light path, close the hood, press CAL and remove it; seat the sample with its clear faces on the light path, close the hood, record the settled reading and remove it. The batch target at each wavelength is the mean of its five readings. Use the batch only if (a) the five readings at each wavelength span at most 0.015 AU, (b) every target is at least 0.08 AU, and (c) the targets lie within 430: 0.10-0.16, 470: 0.18-0.26, 565: 0.18-0.25 and 635: 0.49-0.87 AU, or a 10 mm spectrophotometer with a bandwidth of 5 nm or less reads the working solution at 0.78-0.86 AU at 629 nm and 0.32-0.35 AU at 501 nm. Repeat the check at the start of every session, and remake the sample if a target moves by more than 0.02 AU from its first value or the liquid turns cloudy.

A measurement record needs a reading within 0.02 AU of the batch target at the requested wavelength. Targets are measured rather than taken from the recipe because an LED colorimeter's reading depends on its LED's centre wavelength and bandwidth, which are not specified. For this sample, a 20 nm-wide LED reads about 0.10 AU lower at 635 nm than a spectrophotometer, and each nm of LED offset changes the 635 nm reading by about 0.015 AU. Measuring each batch on the trial instrument removes these terms and any dye-content error. Rule (a) bounds what remains (re-seating, drift and display rounding) to a standard deviation of about 0.008 AU, so a correctly made reading falls within 0.02 AU of its target about 99 % of the time. The simulated instrument is ideal, so its targets are the rounded nominal values in the table.

## Layouts

The physical objects never move between the 16 layouts; only the printed card, lamp and cuvette markings change. The media show one of them: setup.png, the Dream-1 run and the scripted demo all show layout 6 (CLI `--seed 0`: card 470, lamp 565, neither cuvette turned, blank with its white stopper in pocket 1).

Requested and starting wavelengths are visible on the card and lamps. Blank/sample identity and the clear-face axis are visible on the stoppers and face markings. Both physical cuvettes start at yaw zero, cuvette_a in pocket 0 and cuvette_b in pocket 1. A turned marking needs a net -90 degree rotation into the well and +90 degrees on return; an unturned marking needs zero net rotation. Either pocket is accepted at the end.

| Layout | Start nm | Requested nm | Signed presses (up-arrow positive) | Blank net turn in deg | Sample net turn in deg | Blank pocket |
|---|---|---|---|---|---|---|
| 0 | 430 | 430 | +0 | 0 | 0 | 0 |
| 1 | 470 | 430 | -1 | 0 | -90 | 1 |
| 2 | 565 | 430 | -2 | -90 | 0 | 0 |
| 3 | 635 | 430 | -3 | -90 | -90 | 1 |
| 4 | 470 | 470 | +0 | 0 | -90 | 0 |
| 5 | 430 | 470 | +1 | -90 | 0 | 1 |
| 6 | 565 | 470 | -1 | 0 | 0 | 1 |
| 7 | 635 | 470 | -2 | -90 | -90 | 0 |
| 8 | 565 | 565 | +0 | -90 | 0 | 0 |
| 9 | 635 | 565 | -1 | 0 | -90 | 1 |
| 10 | 470 | 565 | +1 | -90 | -90 | 0 |
| 11 | 430 | 565 | +2 | 0 | 0 | 1 |
| 12 | 635 | 635 | +0 | -90 | -90 | 1 |
| 13 | 565 | 635 | +1 | -90 | 0 | 0 |
| 14 | 470 | 635 | +2 | 0 | -90 | 1 |
| 15 | 430 | 635 | +3 | 0 | 0 | 0 |

Each wavelength occurs four times. Four layouts need zero presses; inactive press slots park safely and never touch a button. Both cuvettes are turned in eight layouts, their flags differ in eight, and each pocket holds the blank in eight. Inner wavelengths include both press directions; the endpoints permit only one direction.

The harness derives `zlib.crc32(f"{eval_seed}:{scene_seed}:{epoch}".encode()) & 0xFFFFFFFF`; reset uses this value modulo 16. The real derive_seed function gives the following table for scene_seed=None and epoch=0:

| CLI --seed | Layout |
|---|---|
| 0 | 6 |
| 1 | 8 |
| 2 | 11 |
| 3 | 5 |
| 4 | 12 |
| 5 | 2 |
| 6 | 1 |
| 7 | 15 |
| 8 | 3 |
| 9 | 13 |
| 10 | 0 |
| 11 | 14 |
| 12 | 13 |
| 13 | 3 |
| 14 | 10 |
| 15 | 4 |
| 16 | 7 |
| 17 | 9 |
| omitted (defaults to 0) | 6 |

A direct reset with seed=None selects layout 0. The CLI's `--seed` defaults to 0, so a CLI run without `--seed` selects layout 6, like `--seed 0`. Reset selects the layout before the original reset's first observation, then restores physical poses/joints and clears instrument reference, record, violation and histories. Visual variants are pre-authored at spawn; subsequent changes only toggle visibility.

## Demonstration and runtime

The script reads the known layout tuple: the requested wavelength printed on the card, the starting wavelength shown by the lamps, and the stopper/face markings. Those are observable scene properties. It reads no live object state, scorer result, instrument memory or hidden event. It is an existence proof of the motion, with image interpretation left to a camera policy. The plan resolves its pocket choices and wrist angles lazily after reset, so all sixteen layouts share the same timing.

The hood starts parked open. Left looks at the card and sets the wavelength while right goes straight to the blank, centres it and lifts it at the rack. Left parks before right travels to the well. Right closes the hood and parks before left presses CAL and parks. Right opens the hood, returns the blank, inserts the sample, closes the hood, parks and waits 2.5 s, opens again, returns the sample and parks for a 3 s tail. Left stages at TCP (.180,.150,.9125), outside the 120 mm housing zone, during the last 1.5 s of the sample lid-close sequence. Once right is parked, left moves in .6 s to the display view TCP (.150,.050,.8825), yaw 0, holds 1.9 s, then returns to park in 1 s, before right reaches the hood. The reading stays visible with right parked throughout the view hold: 1.9 s nominal and 1.52 s at .8 pace. The 10 mm display digits project to 10.108, 10.134 and 10.165 px at 320x176. This view includes all four wavelength lamps and the CAL lamp. The hood may remain open at completion. The arms use fixed-duration waits between their instrument operations. Each hood slide commands the tab 4 mm beyond the physical end stop: closing TCP x=.194 m, opening x=.126 m, y=-.045 m. The joint limits keep the hood at 60 mm closed or 0 mm open while the pinched rubber tab absorbs the overtravel. Sliding takes 2.5 s and the stop hold .6 s; at .8 pace these remain 2.0 s and .48 s. The fingers open at the overdriven target before rising vertically; finger/palm clearance to the housing, rails, posts and non-gripped hood parts remains at least 4 mm with the hood at either stop.

At the hover settle the fingers narrow to 23.5 mm before descending. Every pick centres the cap in two axes: close, hold .4 s, open to 23.5 mm over 1 s, turn the wrist 90 degrees at fixed height, close, hold .4 s, lift. Inward picks use wrist 0 to +90 degrees; outward picks +90 to 0. Carried face rotations then land the clear faces on the x beam. Targets use zero aim correction. Travel TCP is table+.165 m, with 1 s settling before lowering after a long move. A held cuvette rides 3.5 mm low. Release stops its bottom 1 mm above the lead cylinder crowns, settles .65 s, opens gently over one 1.9 s waypoint, and dwells .6 s at the release height with fingers open before rising; gravity alone seats the free body before the fingers rise past the cap. At .8 pace, regrasp opening and aim settling each take .8 s, drop settling .52 s, release opening 1.52 s and the released dwell .48 s. These retain the required .8/.8/.5/1.5 s opening and settling minima. The cap retains at least 10 mm pad overlap at 0..4 mm sag. CAL presses allow the measured 1.4 mm arm shortfall: expected 3.6 mm travel, beyond the 2 mm switch.

Both plans last 141.2 simulated seconds; including the additional success hold, 142.2 s is 54.69% of the 5200-step / 260 s limit. A timing variant scales every waypoint by .8 to 112.96 s; with the same 1 s success hold, 113.96 s is 43.83% of the limit, including the same physical clearances. At .17 wall seconds per control step, the nominal full plan plus hold takes about 483 wall seconds after startup; the full limit about 884 s. Every target is within .34 m planar of its arm base. The measured maximum target/interpolated planar reach is .324401 m. TCP separation is at least .18 m; both TCPs are never simultaneously within .12 m of the housing centre. Offline geometry checks cover all sixteen plans, joint-6 limits and the complete swept gripper clearances. Sight-line checks cover the entire rectangular display (sign, digits and decimal) and all five lamp faces at every control step of the view hold in all sixteen plans. They use the complete closed-hood, housing, rails and button geometry, both arms' URDF visual meshes bounded down to individual triangles, and both hands' finger/palm boxes. All regions fit inside the calibrated left wrist image with no intersecting viewing volume. These are offline geometric visibility checks; rendered legibility and motion tracking require simulator trials. Kinematic replay cannot move passive joints or reproduce gravity-driven seating.

## Fidelity

Beer-Lambert response uses the seated cuvette, its faces and lid; no optical simulation. Displays are visual. The cuvettes' face orientation and pocket order are randomised as markings; physically equivalent to the cuvettes having been placed turned or swapped before the trial (the collision shapes are 4-fold symmetric and identical). Cuvettes are rigid sealed bodies. The simulated detector is monochromatic, so its targets are the nominal Beer-Lambert values; a real LED colorimeter reads this sample differently, mostly lower at 635 nm, so real targets are measured per batch (see Sample).
