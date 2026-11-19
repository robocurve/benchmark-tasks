# barbell-spring-collar-slide-and-seat — build deviations (revision-1)

Each entry states what the simulation does, why a rigid-body engine needs it, and what the real
stock hardware does instead. Every item is stock-parts only: no custom machining, no fixture.

1. **Crossed prismatic spring-wire stirrups instead of helical wire torsion.** The 0.320 kg collar is a
   250 g three-turn 8-gon coil barrel (68 mm centreline, 4.4 mm wall, bore 59.5–63.6 mm) with two 35 g
   crossed `slide` stirrup jaws: 2200 N/m per crossed leg, 4.0 mm preload inside the 24.5 mm sleeve radius
   at rest, 12.5 mm stroke, 0.4 N joint friction. A real collar's three-turn coil is a torsion spring; a
   rigid-body engine cannot bend wire, so the same squeeze-open-and-clamp law is modelled as two
   counter-sliding shoes whose rest position is an interference fit. Consequence, stated plainly: at the
   seat the shoes sit a few mm into the sleeve and the spring's own force there is small (order 10 N), so
   the *grip* is a friction contact, not a hard lock. The scored retention evidence does not read the
   jaw joints; it reads the collar's end pose after release (see 5).

2. **1.20 kg loadable dumbbell instead of a full barbell.** A 68 mm collar cannot pass the plate stack of
   a full bar inside a 300 mm-wide bench reach, and the original 58 mm "retention shoulder lug ring" was a
   custom part the review refused. The dumbbell gives the same real product (49 mm plain round chrome
   loading sleeve, plain round, no key flats) with the collar in clear air over its free end. Both sleeves
   are built mirrored about the knurled handle; the stand grips only the 28 mm centre handle, so nothing
   stands between the collar and the sleeve.

3. **Scene turned onto world +Y (BAR_YAW = 90 deg).** The bar, stand, tray and collar are authored along
   their local x and turned onto world +Y so the right arm threads the collar straight away from its own
   base and the left arm clamps the far plate on its own side 60 mm off the axis. This removes an
   arm-to-arm link contact that existed when both arms reached the same world x. It is a layout choice, not
   an aid: no geometry is added to guide the collar.

4. **Two-step 1.5 mm sleeve end chamfer.** A stock loading sleeve has a chamfered end; without it the collar
   bore caught a 90-degree edge at the thread entry. Two 2 mm steps at 0.75 mm and 1.5 mm model it.

5. **Felt-lined timber holster and a vinyl-grip collar.** The collar stands upright on its coil in a
   felt-lined slot (38 mm walls to the coil equator, 2 mm / 2.5 mm clearance) 150 mm off the sleeve axis.
   The holster is a bench accessory, like a dumbbell rack, not a guiding aid: it holds the collar where a
   person set it down and never touches the bar. The squeeze handles are 11 mm wide x 34 mm tall vinyl
   loops 53 mm above the sleeve axis; the gripper pads are 9 mm thick black jaw shoes, so a 20 N pad load
   cannot plough the pad through the handle face.

6. **Retention is read from pose, not from a joint.** The review refused a joint-displacement threshold.
   This revision scores the collar's released end pose (front-face-to-plate separation and coaxial grip)
   with both grippers out of contact. The scripted retention phase still applies a light friction-limited
   pinch on the coil and pulls 20 mm (the spring-scale tug of a real trial), but the pinch is an actuator
   command, never a scored quantity. Power of the check is proved one-variable: with the wire spring live
   the collar settles at a 1.3 mm gap, radial 6.4 mm, 8.62 N in the wire (1.0); with the actuator force zeroed
   partway through the plan's own retention pull (kill instant re-derived on this tree and printed by the check:
   21.40 s, the pull starting at 20.90 s)
   the same plan leaves it at a 14.0 mm gap with the radial error growing to 7.4 mm and no
   force in the jaws (0.25). The ablation is a reviewer-side check run against this task file, and its kill instant is derived from the plan, so it cannot go stale.

7. **The flush gap is read at the collar's foremost plate-side surface, not at the coil.** On this stock collar
   the two black plastic squeeze handles stand 3.0 mm proud of the 27 mm coil face (collision hull 33.0 mm along
   the sleeve: handles at +-16.5 mm, coil at +-13.5 mm), so the handles are the first thing to touch the plate;
   the listing photos for the identified collar show the same, the plastic grips closing on the plate in front of
   the coil. The gate is therefore "no visible gap between collar and plate", 3 mm, measured from that foremost
   surface -- what a feeler gauge slid in at the bench would find. What changed, stated plainly: earlier revisions
   quoted the same 3 mm but described it as a coil-to-plate reading, and the modelled 3.0 mm handle offset then
   turned a seated collar into an apparent 3 mm miss. The offset is not added back anywhere and the tolerance is
   not widened. Engine note on the same measurement, both numbers read from end poses of this tree: MuJoCo
   (4.0.1) settles the handles at a 1.3 mm gap, and Isaac/PhysX with the arena's 5 mm contact offset ends the
   same plan at 0.6 mm. The two engines differ by well under the 3 mm gate, and neither stops short of it. An
   earlier revision quoted 4.7 mm for the stiffer engine; that reading was the cocked arrival -- the collar
   landing tilted about 4.8 deg so one corner of the handle face bore on the plate -- and not a contact
   compliance difference, so it is withdrawn rather than restated.
8. **Plan press depth and dwell are measured, not tuned by feel.** `SEAT_PRESS_X` 15 mm held `SEAT_DWELL_S` 1.0 s
   before release. At 14 mm / 0.8 s the collar released at 1.66 mm from the plate and the retention tug walked it
   back out to 3.02 mm, a 0.02 mm miss of the flush gate on one of five start jitters; the longer, slightly deeper
   press lets the released jaws settle onto the sleeve before the tug, and every jitter and both friction scales
   end inside 3 mm (worst 2.42 mm). Deeper presses were measured and rejected: at 16-18 mm other seeds slip to
   3.3-4.2 mm and at 18 mm with friction scaled 30% down the bar itself is pushed 12.5 mm along its cradle.
9. **Release happens under load, and the retract is a single move.** The right gripper opens its fingers over a full
   second WHILE it still holds the collar pressed against the plate, then retracts in one move clear of the ring.
   That is what a person does: let go with the collar home, not while pulling away. It matters because the released
   wire jaws are the only thing holding the collar; any motion that drags the ring before the shoes bite moves it
   off the plate. Measured on this plan: opening in place for 1.0 s passes the demo and all five start jitters and
   both friction scales (worst end gap 2.65 mm), while a 0.4 s open followed by a separate settle and a short
   square-up nudge applied after the release drops the demo to 0.50 and two jitters to 3.3-3.4 mm. Doing less, for longer, seats it. The left
   arm's hold on the far plate is derived from the right arm's own waypoint schedule, so it always covers the
   retention pull rather than running out early when the plan changes.
10. **The wrist squares the collar as it lands (plan deviation, stated so a reader finds it).** What: a
   `SQUARE_UP_DEG` 4.0 deg counter-roll about the gripper's own x axis, composed into the thread / land / drive /
   hold-squeeze waypoints (`r_sq = r_right @ R_x(4 deg)`). Why: the collar arrives cocked -- in the stiffer engine
   it lands tilted about 4.8 deg, one corner of the handle face bearing on the plate, so the press drives against
   an edge instead of a face. A person squares a collar by feel the same way, rolling the wrist as the ring meets
   the plate. Measured on the Isaac/PhysX traces, one variable (the same plan, the counter-roll on and off): tilt
   at contact **4.8 -> 0.5 deg**; the collar stalls at a **4.8 mm** gap (peak 4.9) and **ends at 4.0 mm** without
   the roll, against **0.6 mm** with it. MuJoCo cannot certify this lever because it
   self-recovers, so the number quoted is the probe trace, not the offline check. The direction and sign of the
   roll were sized from a one-variable sweep over the offline pre-check (+4 deg about x: peak tilt 2.72 -> 2.34 deg,
   end gap 2.21 -> 1.33 mm; -4 deg about x and about z both regress), not guessed.
