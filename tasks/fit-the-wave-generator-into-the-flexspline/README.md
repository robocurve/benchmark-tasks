# Fit the wave generator into a robot joint's mounted HSS-14 strain-wave gearbox

**Instruction:** "Fit the wave generator into the gear until it is fully home, then let go."

**Category:** Impact on AI timelines.

## Changes in this revision

The review of November 23, 2026 (TK-MF6E4K) is quoted point by point, each followed by the change made. The runout flag is answered with a derivation for this specimen and a baseline check on every enrolled unit; what has and has not been measured is set out under "Eccentric-mesh check".

1. *Flag: "The 0.3 mm shaft-runout check is unvalidated as a diagnostic for eccentric mesh; verify it against the
   chosen physical specimen."* The check is now tied to the chosen specimen (StepperOnline HSS-14-50-I-D8) and has an
   open derivation from its size and ratio (section "Eccentric-mesh check"): a one-tooth jump offsets the generator by
   about 0.35-0.7 mm, a shaft runout of roughly 0.7-1.4 mm; the 0.3 mm cutoff sits under half the low end. The proposed enrolment check
   tests only the correctly seated baseline on each physical specimen: a correctly fitted generator must read 0.10 mm or
   less, so the cutoff is at least three times the specimen's own baseline, or the specimen is not enrolled. No
   eccentric mesh has been produced or measured on a physical specimen, so the cutoff
   remains a derived estimate, and forcing one on purpose is not part of enrolment (it risks the flexspline damage
   Harmonic Drive's catalogue warns about).
2. *"The supplied housing drawing lacks a working bearing seat ..."* The printed housing is gone. The specimen is
   now a housed gearbox whose output runs on its own cross-roller bearing ("highly rigid, full-roller cross bearings
   that can be used directly to support external loads", product page). No printed part carries a bearing.
3. *"... and circular-spline bolt support ..."* The circular spline is mounted in the gearbox's own metal housing by
   the maker. The printed cradle only carries the gearbox: its 73 mm flange is bolted to the cradle's riser with
   8 x M4 screws through the flange's 8 x 4.5 mm holes on the 65 mm circle (maker's drawing), and the riser's
   56.5 mm bore centres the 56 h7 pilot.
4. *"... and the output flange is unspecified."* The output flange is the gearbox's own, specified from the maker's
   drawing: 6 x M4 on a 23 mm circle, 11 H7 centre bore, 56 h7 pilot, output face 34 mm below the input face. It
   turns freely inside the cradle bore, clear of the bench; nothing is attached to it in this task.
5. *"The only proposed component set has a $1,000 allowance, without a lower-cost route ..."* The specimen is now one
   StepperOnline HSS-14-50-I-D8 at $222.09 (product page read November 23, 2026). It is sold now but listed as
   pre-order (made or procured to order, ships from China, 2-4 week lead time obtained from the maker's technical support, Nov 2026), so it is orderable rather than held
   in stock. The original about-$400 kit allowance is superseded by the sourced partial BOM below. Tray, mounting hardware and consumables are unpriced (below).
6. *"... and the physical fixture cannot yet be built as specified."* `fixture/cradle.scad` and `fixture/cradle.stl`
   (with `fixture/make_stl.py`, which writes the STL from the same numbers) give a one-piece printed cradle; the
   handle is `fixture/handle.stl`. Print settings, fasteners and assembly steps are listed below. Export checks give
   248.524 cm3 for the cradle and 7.590 cm3 for the handle; the nearest bolt-hole wall is 2.60 mm from the bore. No custom bearing seat or output flange needs fabrication.
7. *"Please complete the housing and cradle design, including the bearing seat, circular-spline bolt support and
   output flange, and provide buildable files or an off-the-shelf fixture."* Done as in items 2-4 and 6: bearing,
   circular-spline mounting and output flange come off the shelf in the gearbox; the cradle and handle are buildable
   files.
8. *"Also specify a readily available, lower-cost matched gear or equivalent setup costing no more than a few hundred
   dollars that preserves the same loaded rotating insertion, and update the dimensions and scoring for it."* The
   HSS-14-50-I-D8 is a matched set sold as one unit. Its drawing gives the wave generator its own detail views (two
   interchangeable variants, A and B) and a toleranced installed dimension, 8.5 +/- 0.2 mm for generator A, read here
   as the generator's inner (hub) end below the input face. Separability is inferred from the drawing, and the maker's technical support (Nov 2026) confirmed that it is a separate part inserted while slowly rotating/rocking. That statement covers the insertion, not repeat removal; enrolment admits only a unit whose generator comes out without dismantling. Scene, mesh rule, scoring heights and demo were revised for the new geometry,
   and the recorded runs below use it; the full-score end state is generator A at its drawn installed
   depth, 8.5 +/- 0.2 mm.

## Why it matters

Robot-joint assembly technicians fit wave generators into mounted spline pairs when building strain-wave joints.
In a housed gearbox such as the HSS-14-50-I-D8 the circular spline and the cup-shaped flexspline arrive mounted on
the output bearing; the drawing dimensions the wave generator separately, with an installed position, which
indicates that it is fitted on the input shaft and pushed into the flexspline at joint assembly (an inference from the drawing). The separate-part and slowly rotating/rocking insertion statements were also obtained from the maker's technical support (Nov 2026). The
elliptical generator has to spread the thin flexspline into the circular spline evenly; pushed in straight or tilted,
the teeth on one side can jump a tooth and leave an eccentric (dedoidal) mesh even though the generator sits at the
right depth. Motor fitting, cover fitting and powered joint testing follow outside this task.

The generator is clamped on an 8 mm shaft with a printed knob: a stand-in for the 8 mm motor shaft it is clamped to in
the finished joint, so the insertion can be done, scored and reset without the motor. In the joint the same turning
insertion is made with the generator on the motor shaft; carrying the motor's mass and its register fit is not part
of this task.

Harmonic Drive's component-set catalogue ([CSF/CSG catalogue, printed page 29](https://www.harmonicdrive.net/_hd/content/catalogs/pdf/csf-csg.pdf))
states the general practice for strain-wave gears: "Insert Wave Generator as you rotate it." and "Avoid assembling
with excessive force on Wave Generator bearing." Those sentences are Harmonic Drive's, not StepperOnline's; the
HSS-14 drawing gives a separate generator detail and installed dimension (drawing-based inference); the separate-part statement also comes from the maker's technical support, as above. That statement does not cover repeat removal. The maker's installation manual is discussed under source access below.

The manipulation is aligned rotation under a 10-30 N axial load throughout the insertion: a rigid proxy in simulation
(below), the real flexible mesh in a physical trial. Final depth alone earns no credit for an eccentric mesh. The demo
uses the right hand; the bolted cradle holds the gearbox and the left hand stays clear.

## Scene (lot A is the screenshot)

Millimetres; positions from the midpoint between the arm bases (x forward, y to the robot's left), heights above
the bench. Sizes and their sources are in `fixture/dimensions.json`.

| Object | What it is | Where |
|---|---|---|
| Gearbox | StepperOnline HSS-14-50-I-D8 housed strain-wave gearbox, 50:1, cup flexspline, cross-roller output bearing (product page). Drawing: 73 flange, 34 from output face to input face, 38 h7 input register, 56 h7 output pilot, output 6 x M4 on a 23 circle. Inside the input bore, the dark circular spline and the bright flexspline rim (bore about 31.0 before insertion, obtained from the maker's technical support (Nov 2026); 1.0 lead-in and open end level with input face: estimates) | input side up, input face 60 above the bench, axis at x 290, y -90 |
| Assembly cradle | printed black PLA, one piece: 120 x 120 x 10 base plate and an 84 riser, 56.5 bore (through the plate; the simulated plate is solid), top 53 up; the flange is bolted on with 8 x M4 x 16 | under the gearbox; rigid housing/cradle body 0.775 kg (maker whole unit 0.51 kg, less 0.035 kg generator estimate, plus 0.30 kg cradle estimate); simulation adds a 0.050 kg hidden stop proxy, total 0.825 kg |
| Wave generator on its shaft | generator A: 8 mm keyed bore (drawing); 14 hub with set screws as dimensioned in the drawing's generator-B detail (assumed the same for A), 2 below a steel plug 30.4 across, 8.0 tall, 1.2 chamfer, red index mark (estimates); clamped by its set screws on an 8 mm stainless shaft, hub end flush with the shaft end; a printed black handle on the shaft: 30 x 3 disc 30 above the plug's bottom face and a 20 x 20 x 16 square grip knob, knob top 49.0 above the plug's bottom face; 64 g in all (estimate) | standing on its hub on the tray floor, shaft up |
| Parts tray | aluminium, 140 x 100 x 15, 4 floor, 0.40 kg | lot A centre x 195, y -225; lot B x 385, y -215 (generator turned 90 degrees) |
| Stepper motor | NEMA 23, 57 x 57 x 56, 8 mm shaft, on a printed 70 x 70 x 8 adapter plate (fixed, background) | x 540, y 20 |
| Joint cover | printed, 80 x 20 (fixed, background) | x 540, y -90 |

The gearbox and cradle are never moved between lots. Trials alternate lots A and B and are averaged.

## Scoring

Tools for a real trial: overhead and side video at 30 fps, the proposed 80 SPS load-cell logger under the cradle (tared), a caliper
with a depth rod (0.01 mm), a dial indicator (0.01 mm) on a stand, a small square, a 10x loupe and tape marks round
the cradle. Convert calibrated load-cell kilograms to newtons with N = kg x 9.81; 30 N is about 3.06 kg. Heights below are the
knob top above the gearbox's input face; the knob top is 49.0 above the plug's bottom face and 51.0 above the hub end.

**Score 0 for the whole trial if, at any moment:**

1. the measured axial generator/gear load exceeds 30 N as a rolling 0.25 s mean (use timestamped load-cell samples, as specified below; video frame rate does not determine sensor sample rate;
   simulation: a time-weighted 0.25 s window of physics-step axial normal and friction forces, summed across generator–stop and generator–housing/rim contacts, at any offset or tilt); 30 N is the benchmark overload threshold, not a published bearing rating;
2. a gripper enters the input bore below the input face (it would mark the teeth; fresh marks on the rim under a 10x
   loupe after the trial, and the side video; simulation: any gripper point within 17.7 mm of the axis and 0.5 mm or
   more below the input face);
3. the generator falls more than about 8 mm with no gripper on it anywhere but inside the gear, for example back onto
   the tray or the bench (side video; simulation: falling faster than 0.4 m/s, about an 8 mm free fall, with no
   gripper within 5 mm);
4. the cradle moves over 5 mm, tilts over 5 degrees or turns over 5 degrees about its axis (tape marks, side video).

**Score 0 at the end if the mesh is eccentric:** after the trial the dial indicator (proposed contact force about 0.5 N, unmeasured; indicator contact force not specified by the supplier) is set
on the shaft 10 mm above the input face and the shaft is turned slowly by the knob through two turns, fingertips on
the knob's top and side with no sideways push; a total reading over 0.3 mm assigns the eccentric-mesh outcome. In
simulation an unvalidated eccentricity proxy is set only on touching generator–stop contact: a measured axial force of at least 20 N continuously for 0.25 s while turning slower than 0.3 rad/s, or measured axial force of at least 8 N while tilted over 3 degrees. These benchmark triggers are not known HSS tooth-jump or damage thresholds. Lifting
the plug out of the rim lets the flexspline spring back round and clears it, so a generator that is taken out and
fitted again properly can still score in full.

**Otherwise**, with the generator's axis within 1 mm of the gear's and within 30 degrees of upright (home also needs
it within 3 degrees of the gear axis: the shaft square to a small square on the input face):

| Score | End state | How it is read |
|---|---|---|
| 1.0 | home: generator A at its installed depth, hub end 8.5 +/- 0.2 below the input face (knob top 42.5 +/- 0.2 above it), even mesh, both grippers at least 20 mm from the generator, shaft and handle, nothing moving for 1 s | depth rod; overhead video |
| 0.75 | home and even, but still gripped or not still for a full 1 s | depth rod; video |
| 0.5 | plug at least half way into the teeth zone: knob top 45.0 or less above the face | depth rod |
| 0.25 | plug at least 1 mm into the teeth zone: knob top 46.5 or less above the face | depth rod |
| 0 | anything else, including a generator resting on the rim or lifted back out | |

The simulation uses these observable proxies: `observe()` in `task.py` records the mesh front, the plug's depth,
offset and tilt, actual generator–stop contact, axial normal force from PhysX impulses at every physics step, the turn, the hands (face-sampled gripper hulls) and
the hold; `score()` applies the table. A trial ends at 2,400 steps or at full score. Home uses the drawing's +/- 0.2
mm. The partial tiers also require the mesh front to have reached each credited depth and allow 0.5 mm of plug-depth
tolerance, so their tolerated limits are 45.5 and 47.0 mm. For the final one-second hold, generator and housing
marker motion must stay within 1 mm, with linear speed below 20 mm/s and angular speed below 0.5 rad/s.

### The declared mesh rule (what the simulator cannot do)

PhysX bodies are rigid, so the flexspline cannot bend round the elliptical plug. The teeth zone is a stop under the
plug that the rule moves:

- the hub passes freely; the plug's chamfer and the rim's lead-in go in freely for 1.5 mm; there the plug rests on
  the teeth zone;
- while the plug presses on it at 10 N or more, square (tilt 3 degrees or less) and on the axis, each turn of the
  generator about its own axis, either way, lets it 4 mm deeper, up to home with the plug's bottom face 6.5 mm below
  the input face (hub end at the drawing's 8.5 mm);
- a continuous 0.25 s contact press of at least 20 N while turning under 0.3 rad/s, or contact press of at least 8 N while tilted over 3 degrees, sets the benchmark eccentricity proxy; the stop target moves to home. This is an unvalidated simulation outcome, not a measured HSS failure transition;
- withdrawing until the plug bottom is less than 0.5 mm **below** the input face resets the stop to the mouth and clears the proxy. Straight-push dwell clears on withdrawal, unloading, lost contact, or any other interruption of its qualifying force/spin condition;
- a passive contact-friction brake replaces omitted mesh turning resistance. The plug underside and stop use
  coefficient 0.4 (a designed proxy, not oiled-steel friction). The stop inner radius is 11 mm from the gear axis:
  at the 10 N feed threshold, the nominal coaxial sliding capacity is 0.4 x 10 x 0.011 = 0.044 N m.
  This nominal calculation exceeds the 0.041 N m maximum no-load starting torque of a CSF size-14 ratio-50 housed unit (Harmonic
  Drive catalogue printed page 46, Table 38), used as a sibling estimate because no HSS-14 input starting torque
  was found. Resistance increases with contact load. This is not a measured HSS insertion torque. PhysX solves
  the contact impulses with equal reaction on the stop and the generator, including passive stopping on release;
  no external torque is held across a control interval. `drag_est` in a trace is a diagnostic sliding-capacity
  estimate for coaxial contact, not a measured moment, applied force or guaranteed bound at every offset/tilt. At the allowed 1 mm axis offset the inner contact radius measured from the generator axis can be only 10 mm before tilt effects, giving a corresponding nominal capacity of 0.040 N m at 10 N. A minimum turning requirement is not certified by the 0.044 N m calculation.

Contact measurement uses the actual generator–stop body pair. Only reported solver points separated by at most
0.05 mm and carrying positive normal impulse count as touching; the axial projection of impulse divided by the
physics timestep supplies the load. The 0.05 mm tolerance is a numerical allowance, one sixth of the nominal 0.3 mm
radial clearance. Collision offsets are 0.20 mm; this speculative margin never earns feed without the narrower
touch test. Stop displacement, velocity or a nearby generator cannot substitute for a measured contact. The overload
window includes magnitudes of axial normal and friction components from both gear links (stop and housing/rim), including off-axis,
tilting and impacts; it is not cleared on withdrawal. Initial missing history is zero-filled to a full 0.25 s. The trace fields `static_estimate` and `dynamic_balance` are exploratory spring/velocity estimates, not scoring measurements; agreement with them is not assumed. Only detailed solver impulses supply scored force.

The press acts on the stop through contact and the stop pushes back equally on the generator; the rule only moves
the stop's position. The 10 N feed minimum uses the lower end of the 10-30 N insertion range obtained from the maker's technical support (Nov 2026). The 8 and 20 N eccentricity triggers, 30 N overload cutoff and 4 mm/turn feed remain benchmark estimates; that range is not a bearing damage rating. Because the tilted-press trigger is 8 N and feed needs 10 N, a press tilted over 3 degrees reaches the eccentricity trigger before it can feed. A physical trial reads force and runout directly.

### Eccentric-mesh check (verification on the specimen)

What is known: Harmonic Drive's catalogue links eccentric tooth mesh to noise, vibration and early failure, and describes an off-centre flexspline after ratcheting. No model-specific numeric runout cutoff is available in the supplied sources.

Derivation of the 0.3 mm cutoff for this specimen (an estimate, not a measurement): a 50:1 cup-type gear has a
flexspline with 2 x 50 = 100 teeth (assuming the usual two-tooth difference). With a pitch diameter of about 35 mm
(estimate; the circular spline sits inside the 38 mm register), the module is about 0.35 mm. A one-tooth jump on one side moves the generator off the gear axis by between about one module and one tooth height, 0.35-0.7 mm, so the shaft turned through two turns runs out by roughly 0.7-1.4 mm (how closely the unsupported shaft follows the offset is not known). The 0.3 mm cutoff is under half the low end of that range. A correctly fitted generator should run out by far less: an 8 H7 bore on a ground 8 mm
shaft leaves a few hundredths of a millimetre, and the set screws close it.

Verification on the chosen specimen (part of enrolment): fit the generator properly by hand, turning, and record the
two-turn runout three times, turning and holding the shaft as in the scoring check. Enrol the specimen only if every
reading is 0.10 mm or less, so that 0.3 mm is at least three times that specimen's own baseline (this also shows
that the hand-held shaft can be read steadily enough). Also record whether the shaft turns smoothly or with noise
and vibration (the sign Harmonic Drive names); this is noted, not scored.

What is not verified: no eccentric mesh has been produced on a physical HSS-14 and measured, so it is not shown that a
real dedoidal mesh in this gearbox exceeds 0.3 mm. Producing one deliberately is not part of enrolment because it
risks the damage Harmonic Drive's catalogue warns about. The cutoff stays a derived estimate until such a reading
exists. The enrolment readings of each specimen are reported with its first trial results.

## Bypasses and near misses

`near_misses.py` checks 4 fixed end states with no history (all 0) and replays trials through the scorer with 49
checks (`python near_misses.py`), including:

| Strategy | Score |
|---|---|
| pushed straight down without turning (25 N) | 0 (eccentric mesh), even though it ends home |
| pushed firmly (15 N) without turning | 0: stalls on the rim |
| turned without pressing (1 N) | 0: no progress |
| pressed while tilted 4 degrees | jams; at 9 N, 0 (eccentric mesh) |
| pressed at 28 N while turning | allowed (under the 30 N limit) |
| pressed at 35 N | 0 for the trial (benchmark overload threshold), even after a proper finish |
| fingertip into the input bore on the teeth | 0 for the trial |
| generator dropped on the way, or dropped 30 mm back onto the tray and then fitted | 0 for the trial |
| lowered fast (0.6 m/s) while gripped | not a drop: 1.0 |
| cradle knocked 8 mm, or turned 8 degrees | 0 for the trial |
| plug 1.5 mm off the axis on the rim | 0: no progress |
| generator upside down, shaft first | 0 |
| eccentric mesh, then lifted out and fitted properly | 1.0 |
| home but still gripped | 0.75 |
| plug held 0.35 mm short of the installed depth | 0.5 (outside the +/- 0.2 mm) |
| released 0.9 / 1.1 / 2.4 / 2.6 mm into the teeth zone | 0 / 0.25 / 0.25 / 0.5 |

## How this differs from close tasks

The comparisons concern the manipulation, not the product name or starting layout. Board searches and
public PR verdicts were checked on November 23, 2026.

- **Strain-wave flexspline unit, [PR 1890](https://github.com/robocurve/benchmark-tasks/pull/1890), final:**
  the reviewer found that turning a toothed unit to find phase while seating repeats TK-YNPRML; the rigid ten-tooth
  cup also failed construct validity. That task moves the flexspline unit into the circular spline. Here both splines
  are already mounted in the gearbox and the separate wave generator goes inside them, with continued loaded turning and a
  recoverable home-but-eccentric failure. This is the claimed procedural difference. The rigid proxy used for the flexible tooth engagement is declared under Fidelity.
- **Three-gear train, TK-YNPRML / [PR 893](https://github.com/robocurve/benchmark-tasks/pull/893), accepted:**
  find the phase of rigid spur gears and lower them onto shafts. This task requires accumulated rotation under
  axial load throughout the insertion depth in its declared rule, with a separate eccentric-mesh failure even when
  the generator is home. Physical flexspline engagement has not been validated.
  [PR 719](https://github.com/robocurve/benchmark-tasks/pull/719) received an overlap final for another spur-gear
  phase-and-seat task; merely changing the gear would not distinguish the skill.
- **Robot gearbox cartridge stack, [PR 3278](https://github.com/robocurve/benchmark-tasks/pull/3278), final for
  overlap:** search for an input-spline angle, then square the housing's separate register. Here there is no final
  angular register: loaded turning advances the mesh through its depth, and a forced insertion can be fully home
  yet eccentric. Hidden alignment alone is not the claimed difference.
- **Servo horn, [PR 1864](https://github.com/robocurve/benchmark-tasks/pull/1864), revisable:** choose a spline
  tooth at a visible zero mark, then press home. Here rotation continues during seating rather than selecting a
  final tooth angle. [PR 2708](https://github.com/robocurve/benchmark-tasks/pull/2708) received an oversimplification
  final and [PR 3112](https://github.com/robocurve/benchmark-tasks/pull/3112) a duplicate final for servo-horn tasks.
- **Jaw coupling, TK-V7QS9F / [PR 2484](https://github.com/robocurve/benchmark-tasks/pull/2484), final on
  November 23 for failing to establish an occupation-scale core task:** turn jaws into the spider's pockets,
  then slide the hub to its gap. Here continuing loaded rotation, rather than one phase match followed by sliding, controls insertion.

The difference from these tasks is procedural: the generator is not set to one phase and then pushed; it must be kept turning under a 10-30 N axial load through the whole teeth zone, and a straight or tilted push can leave it fully home but eccentric, which only the runout check shows. The rigid mesh proxy that stands in for this engagement is declared under Fidelity.

## Real-trial kit, fixture and specimen enrolment

The specimen is one ready-made StepperOnline HSS-14-50-I-D8: circular spline, flexspline and output bearing in one
housing, with its wave generator (drawing-based separability inference; separate-part confirmation also obtained from the maker's technical support, Nov 2026). Buy it whole; enrol only if the
generator is removable without dismantling the spline/bearing assembly. Do not mix generators between units.

Prices in USD, opened November 24, 2026 unless noted. These are **partial procurement costs**, not a complete
kit quote. The gearbox is pre-order, not in stock; the maker's technical support gave 2–4 weeks. No order, delivered specimen,
reset trial or delivery commitment exists. The earlier about-$400 total was only an allowance and is withdrawn.

| Item and quantity | Published cost | Supplier |
|---|---:|---|
| One HSS-14-50-I-D8 matched housed gearbox | $222.09 | [Maker product page](https://www.omc-stepperonline.com/hss-series-harmonic-drive-gearbox-73mm-ratio-50-1-backlash-20arcsec-8mm-input-diameter-hss-14-50-i-d8), pre-order; original browser read Nov 23, rechecked Nov 24 |
| One 1 kg spool PolyLite PLA black, cradle and adapter/cover material | $19.99 | [MatterHackers](https://www.matterhackers.com/store/l/polymaker-polylite-pla-3d-printer-filament/sk/M3RRH23H) |
| One 1 kg spool PolyLite PETG black, handle material | $19.99 | [MatterHackers](https://www.matterhackers.com/store/l/polymaker-polylite-petg-3d-printer-filament/sk/MAW5MH42) |
| One goBILDA 2100-0008-0100 stainless 8 mm shaft, 100 mm; cut to 51 mm | $3.49 | [goBILDA](https://www.gobilda.com/8mm-shaft-stainless-steel-100mm-length/) |
| One 25-pack M4 x 16 socket screws, use eight | $3.79 | [goBILDA 2800-0004-0016](https://www.gobilda.com/m4-x-0-7mm-zinc-plated-socket-head-screw-16mm-length/?sku=2800-0004-0016) |
| One unpowered 23HS22-2804D motor, 57 x 57 x 56, 8 mm shaft; background | $17.68 | [Maker](https://www.omc-stepperonline.com/dual-shaft-nema-23-bipolar-1-2nm-169-9oz-in-2-8a-57x57x56mm-4-wires-23hs22-2804d); rear shaft omitted in simulation |
| One Adafruit 4541 5 kg load cell | $3.95 | [Adafruit](https://www.adafruit.com/product/4541) |
| One Adafruit 5974 HX711 interface, 80 SPS selector | $9.95 | [Adafruit](https://www.adafruit.com/product/5974) |
| One CanaKit Pico H starter kit, board/USB cable/breadboard/jumpers | $28.95 | [CanaKit](https://www.canakit.com/raspberry-pi-pico.html?cid=usd&src=raspberrypi) |
| One Pittsburgh 63731 depth caliper, 0.01 mm display, +/-0.03 mm accuracy | $24.99 | [Harbor Freight](https://www.harborfreight.com/6-in-digital-caliper-with-sae-and-metric-fractional-readings-62569.html) |
| One 0-10 mm dial indicator, 0.01 mm divisions, with articulated magnetic base | $54.99 | [ARVONELLE](https://arvonelle.com/products/dial-indicator-magnetic-base), price displayed; stock not checked |
| One Shars 303-6320 hardened steel square, 2 x 1.5 in | $12.75 | [Shars](https://www.shars.com/2-x-1-1-2-machinists-work-shop-squares), in stock |
| One Carson LL-10 10x LumiLoupe | $8.99 | [B&H](https://www.bhphotovideo.com/c/product/437227-REG/Carson_LL_10_LL_10_LumiLoupe_Craft_Loupe.html/overview), in stock |
| **Sourced subtotal** | **$431.60** | Whole packs/spools, before shipping and tax |

Still needed: tray to the specified size, tape/wipes, glue, a steel support for the indicator's magnetic base,
load-cell mounting plates/fasteners and any required replacement pads/grease. These items are unpriced; they are standard workshop tools.
The shared robot, cameras, logging computer, printer and workshop equipment are excluded. The $431.60 subtotal is a sourced partial total; no complete delivered-kit total is given. Suppliers may change availability.

**Force acquisition proposal:** replace the unspecified kitchen scale with the named 5 kg cell/HX711/Pico path.
Set the HX711 switch to **80 SPS**, gain 128, channel A only; do not average on the microcontroller. Log each
ADC-ready conversion with its device timestamp and raw value over USB. Calibrate zero and 1/2/3 kg known loads
with the gearbox/cradle in place, then convert the tare-corrected kilograms to N using 9.81. The mounting must carry
all vertical load through the cell, without a stop, cable, clamp or plate bypass. Characterize its step response,
hysteresis and off-centre response before use; no bandwidth or calibration measurement has been made here.
For uniform 80 SPS samples the 0.25 s mean is 20 consecutive ADC conversions; with timestamp jitter use
the time-weighted integral over exactly 0.25 s, interpolating the oldest interval. Reject missing-sample gaps or a
stream that fails to sustain 80 SPS. A 30 fps video of a display is only a synchronized record, not 80 force samples/s.
The simulation uses physics-step contact impulses instead; matching real transient response remains unverified.

**Build the fixture once (not part of a reset):**

1. Print `cradle.stl` in PLA and `handle.stl` in PETG: 0.2 mm layers, 4 perimeters, 40 % gyroid infill, no supports
   (about 5 hours in all on a typical desktop printer; estimate). `cradle.scad` is the same part, editable.
2. Tap the cradle's eight 3.3 mm holes (65 mm circle, 12 deep) M4, or use thread-forming M4 screws.
3. Stand the gearbox input side up, its 56 h7 output pilot in the riser's 56.5 bore, line up the flange's 4.5 mm holes
   with the tapped holes and fit 8 x M4 x 16. The 56.5 bore allows for printed holes coming out small; if the pilot
   still binds, ease the bore with abrasive paper until it slides in.
4. Hacksaw the shaft to 51.0 mm and file a flat for the set screws. Fit and glue the handle onto the bare shaft first,
   with the shaft top flush with the grip top. Then fit the generator on the other end, its hub end flush with the
   shaft end, and tighten its set screws. This puts the disc underside 30 mm above the plug bottom and the grip top
   49.0 mm above it. Fit the handle before attaching the generator so the bearing never carries the handle-fitting force.
5. Tape marks round the cradle base; zero the depth rod on the input face.

**Proposed enrolment (5-10 minutes per specimen, unmeasured planning estimate):** record the model and lot, measure the drawing's
flange, register and pilot diameters with the caliper, fit the generator properly by hand, turning, and check it
reaches the installed depth (knob top 42.5 +/- 0.2 above the input face); record the two-turn runout three times
(each 0.10 mm or less, as above); withdraw it and record the force trace and heights.

**Reset between trials (planned, about 2-3 minutes; untimed on a specimen):** Read the runout first, lift the generator out by its
knob, look for fresh tooth marks, bearing roughness or looseness, check the output turns freely, return the generator
standing on its hub at the next lot's tray centre with its red mark toward +y (A) or -x (B), check the tape marks,
zero the depth rod and tare and check the force logger. The gearbox stays bolted to the cradle. Nothing is cast, cured or printed per
trial, and nothing is powered.

**Gripper pads:** before a physical trial, store the shared rig's pads and fit a dedicated replacement set with the
standard profile and material; inspect it for cuts, embedded metal or grease at each reset. Wipe any grease off the
knob and shaft.

## Fidelity

The simulated generator uses a round 24-sided envelope collider; the bearing does not deform elliptically and no flexible teeth are simulated. The generator bore, keyway and set screws are physical mounting specifications; the simulated generator, shaft and handle form one rigid compound without detachable fasteners.

Housing, register, pilot, flange holes, output flange, the generator's bore and installed depth are from the maker's
drawing; the hub size comes from the drawing's generator-B detail. The flange thickness, plug, rim and other spline
sizes, separate-part masses and frictions are estimates; the about 31 mm flexspline bore was obtained from the maker's technical support (Nov 2026); the maker's full datasheet gives 0.51 kg for the whole unit. Bodies are rigid; the flexspline's bending and tooth mesh are the
declared rule above, with a passive friction-brake proxy derived from the 0.041 N m CSF sibling starting torque:
coefficient 0.4 and the 11 mm stop inner radius give nominal coaxial sliding capacity 0.044 N m at 10 N. This is not a guaranteed minimum across allowed offsets/tilts or a measured HSS insertion requirement.
This is an artificial brake coefficient, not oiled-steel friction. The output's six M4 holes and 11 H7 bore are physical product specifications; the simulated output face is a plain cylinder. Output rotation is not modelled;
the gearbox and cradle share one rigid body, with an additional 0.050 kg prismatic mesh-stop proxy (0.825 kg combined). No physical trial has been run.

## Run

From the task folder in the Isaac Lab-Arena environment:

```
inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py \
  -E object_jitter=0 -E control_hz=20 --instruction "Fit the wave generator into the gear until it is fully home, then let go." \
  --scorer task_score --max-steps 2400 --epochs 2
```

- Simulator: yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena). `<policy>` is `yam_scripted`
  for the scripted demo or `dream` for Dream-1.
- `--epochs 2` runs lot A then lot B (trial k uses lot k mod 2) and averages them. `TASK_LOT=A` or `TASK_LOT=B` pins
  one lot. `-E object_jitter=0`: the layout is fixed; no seed changes the scene.
- `DEMO_ROUTE=straight` makes the scripted demo push the generator straight down without turning (the naive route).

## Recorded simulation scores

The final contact-force scorer (axial normal plus friction overload) reached scripted demo **A1.0/B1.0**.
Dream-1 reached **A0.0/B0.0** with the unchanged instruction and a 2,400-step limit per lot
(4800 executed steps total; termination reasons: max_steps, max_steps; recorded abstentions: {}).
These are simulated results, not measured robot or specimen results. Both runs used identical task.py SHA256
`ef48737263a0517ac367d79da9782488f4de1d3af3db9d0d37eee7334e2663ac` and calibration.

- Demo recording: `demo-20261124T033635`.
- Dream-1 recording: `dream-20261124T041132`.

The earlier normal-only demo and an interrupted Dream-1 start are superseded; the media for this revision are the two recordings above.

## Source access and remaining physical checks

The [maker's full datasheet](https://files.omc-stepperonline.com/HSS-14-50-I-D8_Full_Datasheet.pdf) was read via a
web PDF reader on November 23, 2026; its whole-unit weight is 0.51 kg. The [installation manual](https://files.omc-stepperonline.com/Harmonic_Gearbox_User_Manual.pdf)
was also text-readable, but extracted column order prevents assigning every instruction to HSS rather than HHT.
It describes lubrication, shaft-mounted generators, and symmetric engagement; it supplies no numeric runout
threshold or insertion load. The drawing-based separability statement is an inference; the maker's technical support separately confirmed a separate part. Model-specific repeated removal/reset remains unverified.
The proposed physical enrolment and runout checks have not been performed. A low-runout correctly seated baseline
alone would not validate detection of an eccentric mesh. Check current lead time before planning a physical trial.

Product figures obtained from the maker's technical support (Nov 2026): push while slowly rotating/rocking at about 10-30 N; installed hub depth about 8.5 mm; flexspline bore about 31 mm; generator supplied coated with harmonic-drive grease; lead time 2-4 weeks. These figures supplement the drawing. Separability is inferred from the drawing and separately confirmed by the maker's technical support; repeat removal/reset was not supplied.
