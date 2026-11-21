# Fingerstick a trainer hand with a contact-activated lancet

A fingerstick trainer lies palm up on a plywood board clamped to the table: a palm block on a post, with an index
finger, a thumb and a ring finger, the fingers sloping down 20 deg from the knuckles to the tips. The ring finger rides
on a light vertical slide behind its knuckle and gives way when pressed unless it is held. A BD Microtainer
contact-activated lancet (366577, 21 G x 1.8 mm) stands tip down in a stand whose socket holds its protective tab, and a
BD 305635 sharps container stands beside. The robot twists the tab off, holds the ring finger with one gripper ("Grasp
the finger", a hospital procedure for this lancet), places the lancet square on a side of the ring fingertip's pad with
the other, presses until it fires, and drops the used lancet into the container (the tab, the press and the discard
are the steps of BD's guide).

The pose is BD's, read from its guide (Sources). Step 4: "Have patient hold their hand in a downward position, allowing
gravity to help increase blood supply to the hand." The trainer's fingers slope down 20 deg toward their tips; the 20
deg is a choice, not BD's number. Palm up is an inference from steps 7 and 9B, not a quote: step 7's two figures show
the lancet pressed down onto the fingertip's pad, and step 9B, after the puncture, says "Turn the patient's hand palm
down.", so the hand was palm up for the puncture [inference].

Holding the finger is what scores, and the scorer's held rule is what makes a one-handed fire score 0. Rule: from the
lancet's first touch until 0.5 s after the click, a gripper must be closed on the ring finger with both pads on it at
every step, and the ring finger must stay within 2 mm of its top stop, read at its slide and at its fingertip. A fire
with no gripper closed on the ring finger is a fault ("fired with no gripper holding the ring finger"), whatever the
finger did. Physics backs the rule only in part: the unheld ring finger gives 2 mm at 1.20 N straight down, and with no
friction every press the scorer allows pushes it down that far before the lancet fires at 8 N (by 1.29 N or more). With
friction 0.25 or more that guarantee fails: a press tilted off the skin's normal can push the finger along or up its
slide instead (The model, the yield sweep). So the slide alone does not score every one-handed press 0; the held rule
does. The demo's hold lifts the finger against its top stop, so the hold pushes up against the press; a grip that does
not lift scores 0 (stem g00).

The trainer's place and turn are drawn from the seed (Seeds), so a replayed path misses the site: a plan made for one
seed's layout, replayed on another's, failed in all 20 seed pairs tried, and the unshifted plan failed at all 8 corners
of the layout box (`test_open_loop_plan_fails_other_layouts`). Two seeds whose layouts lie within a few mm of each other
may share a plan; the test does not claim otherwise.

Files: `lancet_fingerstick.py` (the task), `mechanism.py` (articulated objects, adapted from benchmark-tasks #223, MIT),
`test_lancet_fingerstick.py` (offline tests), `task.json`, `rig.scad` (the printable real-trial rig: the base with the
fixed fingers and the post, the seat pillar for the springs, the ring finger with its web, the springs' shims, the board's
outline, the stand and its cup), `tools/` (`gen_taskjson.py` writes task.json; `yieldsweep.py` the one-arm yield sweep;
`ikcheck.py` reach; `clearance.py` the sharps container; `grippers.py` the two arms' meshes against each other, the
trainer, the bench and the table; `rig_scad.py` writes `rig.scad` from the task's numbers), `sim/` (run scripts and the
sim stems).

## Review response to TK-GQ487R

1. "the robot must stabilize the punctured finger itself during the press ... rather than immobilizing a rigid finger by
   holding the wrist". The punctured finger, the ring, is the only part of the trainer that moves: it is on a sprung
   slide behind its knuckle. A fire with no gripper closed on the ring finger scores 0 ("fired with no gripper holding
   the ring finger"), and so does the ring finger moving more than 2 mm from first touch to 0.5 s after the click. That
   rule, not the slide, is what makes a one-handed fire score 0: the slide gives way under every allowed press only if
   the skin has no friction (The model, the yield sweep). A gripper on the index finger, the palm or the base holds
   nothing that counts (`test_finger_held_rule`); there is no index-finger stem, because the left comes in from the
   ring finger's side of the hand and the index lies beyond the ring from it. The slide is a real part of the rig,
   dimensioned in `rig.scad` (Real trials, the trainer).
2. "no socket dimensions or printable design". The stand, its post and its cup are parts `stand` and `cup` of
   `rig.scad`, with every size under Real trials, the stand.
3. "Specify a real-trial measurement for the 5 N partial-credit threshold". The placed rung is gone. A real trial reads
   the site from the patch and the square from the lean at the click, so a placement that never fires cannot be scored
   there, and no rung rests on it. The ladder is 0.25 tab off and out, 0.5 fired well, 0.75 in the container and let
   go, 1.0 held 1 s, each read by a named instrument (Real trials, scoring).
4. "assess the lancet angle against the local skin normal at each allowed puncture site rather than assuming every site
   faces vertically". Square is the lancet's axis within 10 deg of the skin's inward normal at the point it touches,
   read by `tilt_of` against `site_of`'s normal, never against the vertical
   (`test_square_is_against_the_local_normal_at_every_site`, every allowed site on both sides of the pad). With the
   hand palm up and pitched 20 deg, the site's normals lean 25 to 62 deg from vertical, so no vertical lancet is square
   anywhere on the site; the demo tilts the lancet 48 deg to meet its site. The real trial reads the same angle from
   two cameras (`test_real_square_check_matches_the_scorer`).

## Sources

Saved copies were read; every quote here is from them (the vendor and catalog pages for the slide included).

- BD, "Successful Specimen Collection: Fingersticks" (VS7688-1), the guide for this lancet. Step 4: "The patient should
  be sitting or lying down.", "Have patient hold their hand in a downward position, allowing gravity to help increase
  blood supply to the hand.", "Select finger: middle or ring finger is preferable. The fifth finger must not be
  punctured because the tissue depth is insufficient to prevent bone injury." and "Identify the desired puncture site
  (shaded area)." Step 6: "Twist off the tab to break the seal and discard." Step 7: "Position the safety lancet firmly
  against the puncture site as illustrated. Hold lancet between fingers." and "To activate, press safety lancet firmly
  against the puncture site. Do not remove the device from the site until an audible click is heard." Step 8: "Discard
  used safety lancet into a sharps container according to your facility's established procedures". Step 9B: "Hold hand
  below elbow level to obtain the required blood volume." and, after the first drop is wiped away, "Turn the patient's
  hand palm down." The CAUTION panel: "Handle all biologic samples and blood collection "sharps" (lancets, needles, luer
  adapters, and blood collection sets) in accordance with the policies and procedures of your facility." Figures: step
  4's lower figure shades both sides of the fingertip's pad and shows the collector's hand holding the patient's finger,
  the collector's thumb across the middle of the finger, below the shaded site; step 7's two figures show the lancet pressed down onto the fingertip's pad, the
  shaded crescent beside it; step 6's figure shows the tab twisted with curved arrows and no stated direction. An
  earlier revision of this README said the guide shows no figure of the finger during the press. That was wrong: the
  two step 7 figures are that figure.
- Akron Children's Hospital, laboratory procedure "Performing a Capillary Puncture" (Version 7), which names this
  lancet's family ("BD Microtainer Contact-Activated Lancet 2.0 mm (blue)"): "Grasp the finger, and using a sterile
  lancet, press firmly against the finger to make a puncture." It does not say how long the grasp lasts; that it lasts
  through the press is our reading of the sentence [inference].
- BD, FDA 510(k) K223243 (BD Microtainer Contact-Activated Lancet): "The BD Microtainer® Contact-Activated Lancet is
  activated by applying pressure against the puncture site upon contact; after skin puncture, the needle/blade returns
  into the device." and "Once activated, the device is auto-disabled". Its table gives the materials: needle carrier
  with protective tab, polystyrene; main housing, tab cap and rear cap, polypropylene; shield, polyethylene. It gives
  no activation force and no body size.
- FDA GUDID / openFDA: 366577 "LANCET CONT-ACT PINK 21G X 1.8MM US", "In Commercial Distribution", K223243. The older
  366593 is "Not in Commercial Distribution", so the task names 366577.
- BD patent US 9,005,233 ("Contact activated lancet device"): the shield pushed against the skin trips the drive
  spring; opposed sides carry "finger grip indentations". The patent allows the tab to break by twist or by pull; the
  task scores the twist BD's guide gives ("Twist off the tab"), and a pull that breaks the tab is not modelled.
- Owen Mumford, FDA 510(k) K222111 (Unistik Touch contact-activated safety lancets), bench test: "The force to activate
  the safety feature: 4 - 15N".
- Facet Technologies, FDA 510(k) K221433 (Facet 28G Universal Lancet): "Cap Twist-off torque" "< 4.9 N-cm".
- BD sharps collectors catalog: 305635 "1 qt with needle ports", "4.9 x 4.3 x 6.5" (in, L x W x H); "2.5 × 1.2 open
  (305635)" (in).
- O*NET 31-9097.00 Phlebotomists: Core task "Draw blood from capillaries by dermal puncture, such as heel or finger
  stick methods."; "143,900 employees" (2025).
- CMA (AAMA) certification exam content outline, Specimen Collection Techniques, Blood: "Capillary/Dermal puncture
  (finger, heel, ear)".
- NCCT, National Certified Medical Assistant detailed test plan (effective January 2024, from a national job analysis):
  "Perform capillary punctures (i.e., fingerstick, heelstick)."
- O*NET 31-9092.00 Medical Assistants: Core task "Collect blood, tissue, or other laboratory specimens, log the
  specimens, and prepare them for testing."; "Employment (2025)" "833,900 employees". O*NET's own definition of Core:
  relevance of at least 67% of incumbents and mean importance of at least 3.0.
- O*NET 29-2061.00 Licensed Practical and Licensed Vocational Nurses: Core task "Collect samples, such as blood, urine,
  or sputum from patients, and perform routine laboratory tests on samples."; "666,900 employees" (2025).
- McKesson item 1251387, Wallcur REM-LANCE-KIT-01D, "Blood Glucose Test Trainer Lance Finger": "Simulated finger stick
  and blood glucose trainer" and "Finger fits over the finger of a standard patient or Mannequin". It gives no size,
  mass or wall thickness.
- LILY Bearing, product page "MGN7C1R40Z0CM | Miniature Linear Guideway - HIWIN" ("Brand: LILY", "Interchangeable
  With: HIWIN"): the slide's sizes and ratings (Real trials, the slide), read 2026-11-19.
- THK, Linear Bushing catalog page (LM model): "When using the Linear Bushing on a single shaft, use two or more units
  (instead of one unit) on the same shaft to avoid a moment load".
- benchmark-tasks #1837 (merged): the trainer hand's sizes, "ANSUR II median adult male (hand length 193 mm, breadth 88
  mm)", its finger radii and spacing.

Not sourced, and set in the code with [EST] or [CHOICE]: the lancet's body size (13 x 10 x 37 mm) and mass (4 g); the
fire force, 8 N, chosen inside Unistik's 4 to 15 N because BD publishes none; the tab's break torque, 3 N-cm, under
Facet's 4.9 N-cm limit, and its break angle, 45 deg; the hand's pose (palm up is an inference from BD's steps 7 and 9B;
the 20 deg slope down is our number for BD's "downward position"); the site's bounds, read from BD's shaded figure (8
to 15 mm from the apex, 15 to 60 deg round on either side); the square, slide, held and still limits; the ring
finger's slide (its 25 g, 100 N/m spring, 1 N preload and 10 mm travel); the half-gap a gripper closed on the ring
finger reads (7.90 mm, read in the sim's PhysX traces, not on the real gripper); the rig's board, clamps, printed parts, fits
and bought parts (`rig.scad`; the linear guide's sizes and ratings are its vendor's page). None has been measured yet;
each one the real trial depends on is measured before the trials (Real trials, checks 1 to 8).

## The model

- The lancet is a loose tool of two links: the housing and its contact shield, which stands 3 mm proud on a spring
  (4 N/mm, 1 N preload). It fires at 8 N on the shield, 1.75 mm of travel, and only once its tab is off and it is out
  of the stand. Firing is read from the shield, so it fires against anything pressed hard enough (skin, table, a
  gripper pad), as the real one does. After firing it is spent. In the sim the housing weighs 20 g and the shield
  2 g, more than the real 4 g [CHOICE]: a 0.5 g shield sank into the fingertip's collider instead of compressing, and a 2 g housing slid 1 to
  2.6 mm in the right's grip when lifted out of the stand and at 4.6 to 6 N of press, far under the grip's 45 N of
  friction [ARITH: 2 pads x 1.2 x 19 N]; we read it as the solver's slip on a body 2 g against the fingers, an
  inference not checked in PhysX's documentation. At 20 g it held to 0.1 mm [RUN: PhysX, 2026-11-20]. The extra 0.2 N of
  weight is 2% of the press.
- The tab: the stand's socket is keyed to the lancet's front and turns on a spring, up to 100 deg either way; the tab
  breaks at 45 deg of turn either way, where the spring's torque is 3 N-cm. WHY either way: BD's step 6 figure shows
  the twist with no direction (`test_tab_breaks_either_way`, stem c00). In the real stand a fixed post holds the tab
  cap and a cup keyed to the housing turns on it, so the tab's own plastic gives the torque. BD says to discard the
  tab; here it stays in the post's slot and is taken out at reset, so it is not a step the robot does. Lifted out more
  than 10 mm before the break, the lancet still has its tab on: a fault. The sim's spring keeps its torque past the
  break, where the real broken tab has none, so the demo turns the lancet back to 0 deg before it lifts it out:
  lifted out at the twist, the socket sprang back while the housing was still in it and dragged the housing up to
  2.6 mm across the pads, and the press then landed up to 19 deg round from its aim [RUN: PhysX, 2026-11-20].
- The trainer: a base fixed to the table, and the ring finger on a slide. The base is a 6 mm plywood board under the
  hand with a bar running back between the arm bases, where two C-clamps hold it to the table (in the sim, the base
  link is fixed to the world); on the board a post carries the palm block, which carries an index finger on a rod
  and a thumb. The fingers slope down 20 deg from the knuckles to the tips, palm up, the ring's apex 100 mm over the
  table. The ring finger (25 g: the print, the guide's block and its screws) rides a vertical prismatic joint behind
  its knuckle, its web 106 mm behind the apex (level). A spring (100 N/m) holds it up on its top stop with 1 N to spare
  over its weight; it stops 10 mm down. The middle and little fingers are left off: a gripper closed round the ring
  finger needs its pads 7.8 to 14.2 mm either side of the finger's axis, where the neighbours' surfaces lie (12.2 and
  12.7 mm off it). Practice on a lone finger is what the commercial trainer the rig uses already is: Wallcur's Lance
  "fits over the finger of a standard patient or Mannequin" (Sources). The slide lumps a real finger's give (its
  knuckle, wrist and arm) into one vertical give behind the knuckle; the finger is rigid on it, so its tip gives what
  its knuckle gives. That a real finger pressed by a lancet and not held moves away is our inference, not measured and
  not quoted from a source; the sources say to grasp the finger (Akron) and show it held while the site is chosen (BD
  step 4).
- The site: the pad of the ring fingertip, 8 to 15 mm back from the apex and 15 to 60 deg round from the pad's middle
  line, on either side. The shield's face within 1.5 mm of the skin is on it. WHY either side, and off the middle: BD's
  figure shades a crescent on each side of the pad. From 8 mm, past the pad's 7.3 mm radius, the skin's normal has no
  lean along the finger. With the hand pitched 20 deg, the site's normals lean 25 to 62 deg from vertical. The demo
  aims 11.5 mm back from the ring's apex, 45 deg round on the thumb's side, where the lancet comes in 48 deg from
  vertical.
- Square: the lancet's axis within 10 deg of the skin's inward normal where the shield's face touches (`tilt_of` with
  `site_of`'s normal), at each allowed site, not against the vertical.
- The touch: the shield's face within 0.3 mm of the ring finger's skin (`TOUCH_D`), or the shield off its stop with its
  face on the ring finger. It is contact as a camera sees it, not a force: a real trial reads the same event, the first
  side-camera frame (60 fps) where the shield's face meets the ring finger. A touch on the table, the board or another
  finger does not count (`test_a_touch_off_the_ring_finger_does_not_start_the_window`). In the previous revision's
  traced demo the face sat 0.03 to 0.29 mm off the skin for 0.5 s before the shield left its stop [RUN], so a force
  threshold would have started the window later than the camera does.
- The press: from the shield leaving its stop to firing, the face moving more than 1.0 mm on the skin is a slide. Lifting
  off (the shield back on its stop) ends a placement, so the lancet may be lifted and placed again.
- Held: a gripper closed on the ring finger as it is with both pads on it: its
  half-gap 6.40 to 8.20 mm and its TCP within 15 mm of the ring finger's axis between its distal joint (26.6 mm back
  from the apex) and its base (99.0 mm back), at most 4 mm off the axis along the gripper's closing direction, and its
  pad tips (13 mm past the TCP along its approach) past the axis, in the ring finger's own frame. The half-gap window
  runs from 1.5 mm under to 0.30 mm over 7.90 mm, the left's reading closed on the 7.8 mm finger [RUN: PhysX traces of
  the demo at five layouts, 7.84 to 7.98 mm from the touch until 0.5 s after the click; 0.1 mm over the radius, so the
  reading is not the pads' distance from the axis]. The top had been 0.15 mm over 7.93 mm, which left the widest
  reading 0.1 mm inside it. Wider than 8.20, the pads are off the skin: 8.3, 8.6 and 8.7 mm do not count
  (`test_finger_held_rule`). A gripper that also reads as holding the lancet still counts: the two
  windows overlap from 6.40 to 7.0 mm, and a hold beside the press can read as both
  (`test_a_hold_near_the_press_that_also_reads_as_holding_the_lancet_counts`). A gripper closed on air 4 mm beside the finger, or stopped short with its tips before the axis, is not holding it
  (`test_finger_held_rule`). The hold must be there at every step from the lancet's first touch until 0.5 s after the
  click: a step without it is a fault ("not held from first touch to 0.5 s after the click"), and a fire with no such
  gripper is a fault.
- The model reads the hold and the finger at 20 Hz, the control rate, so a jab between two steps is not seen by the
  drop check. It does not need to be: the hold must be contact-like and present at every step of the window, so a jab
  on an unheld finger scores 0 however it is timed, and only a gripper really closed on the finger can score.
- Still: from the lancet's first touch (the touch above, tab off, out of the stand) until 0.5 s after the click, the
  ring finger pushed more than 2 mm down is a fault, whatever the force at the time. The window, and the hold it
  needs, start at the first touch, so a brush of the shield on the finger before a gripper holds it is a fault by design
  (`test_a_brush_before_the_hold_is_a_fault`): a real trial cannot tell a brush from the start of a press. The drop is read at
  the slide and at a fingertip mark 11.5 mm back from the apex on its axis, against the base, and the larger counts;
  the real trial reads the same two points. WHY past the click: a jab fast enough to fire on the finger's inertia
  leaves the finger moving down; from 1 m/s the net force on it, the 1 N preload plus 100 N/m times its drop x, stops
  25 g only after 8.7 mm (0.5 x 0.025 x 1^2 = 1.0 x + 50 x^2), past 2 mm [ARITH, 1-D; the 1 m/s is EST]. A finger
  pinned on its lower stop is 10 mm down, so pressing it there is a fault too (`test_ring_pinned_at_its_lower_stop`).
- The yield sweep (`tools/yieldsweep.py`): a press is allowed when the shield's face is on the site and the lancet's
  axis d is within 10 deg of the skin's inward normal n. The shield's spring carries the axial load, so the contact
  force C on the skin has C . d = F, the force on the shield. Friction (coefficient up to mu) lets C lean up to atan
  mu off d (the shield's flat face on the skin) or off n (its rim on the skin), so C = F c / (c . d) for such a
  direction c, and its downward part, which the slide feels, is F k with k = -c_z / (c . d). Over the site on both
  sides (every 0.5 mm and 2.5 deg), 73 lancet directions and 146 contact directions each, the least k [RUN]:

  | friction mu | least k | gives way (2 mm) at, N on the shield | margin at the 8 N fire |
  |---|---|---|---|
  | 0 | 0.3106 | 3.86 | +1.29 N |
  | 0.25 | 0.0733 | 16.38 | -0.61 N |
  | 0.5 | -0.1643 | never (pushed up) | -2.51 N |
  | 1.0 | -0.6395 | never (pushed up) | -6.32 N |

  The worst site is 9.0 mm back, 60 deg round, on either side (the band's far edge, where the normal leans 62 deg).
  The friction limit, plainly: the slide makes every allowed one-handed press give the finger way before the fire
  only if the skin has no friction. With friction 0.25 or more, a press 62 deg from vertical tilted 10 deg further
  can push the finger along or up its slide, and the lancet can fire on a finger that has not moved 2 mm. That fire
  still scores 0, because no gripper held the ring finger: the held rule is what makes every one-handed fire score 0
  (`test_fire_without_holding_the_finger_latches`). The test asks for a margin of at least 1.0 N with no friction and
  checks that it is negative from friction 0.25 up (`test_unheld_finger_gives_before_fire`), and shows the check can
  fail: a spring with 5 N to spare would not give way under some allowed presses even with no friction
  (`test_the_sweep_can_fail`).
- The demo's hold: the left comes in from the hand's far side from the thumb, its approach 30 deg below the palm's
  plane, and closes on the ring finger 86.0 mm back from its apex, 74.5 mm behind the press, its pad tips 8 mm past the
  finger's axis. It aims 25 mm over the finger, lifting it against its top stop with about 12.5 N [EST: the arm's 0.5
  N/mm at the TCP]. The press comes in 48 deg from vertical, so about 0.66 of its force pushes the finger down its
  slide: the demo's 9 N press (`PRESS_F`) puts about 5.9 N down, under the 13.5 N the hold and the preload hold up. The hold
  pushes the finger up, against the press: that is what holds it still. The top stop stands for the finger held
  straight, the way a collector's grasp holds it; it does not hold the finger for the robot. A grip on the finger that
  does not lift it (stem g00) leaves the press to meet the slide's spring and the arm's stiffness; its offline replay
  scores 0, not held still (`test_sim_stems`). A jab past the hold (stem y00, 60 mm past contact along the lancet's
  axis in one step, about 27 N [EST]) also scores 0, not held still.
- The demo's press. The right's arm is a joint-space PD, so the lancet's face gives under the push by the arm's
  compliance, J K^-1 J^T (J the face's Jacobian, K the joint stiffnesses, 80 N m/rad at joints 1 to 3 and 10 at 4 to
  6): 1.14 to 1.55 mm/N back along the axis over the layout's corners, 0.49 to 0.77 of that sideways, and a wrist turn
  of 0.27 to 0.31 deg/N [ARITH: `_press_give`]. PhysX agreed on seed 0: 1.34 mm/N settled against 1.39 computed, and
  2.4 to 2.6 deg of turn from the touch to the click at 8 N [RUN]. The plan allows for all three, so the push lands
  along the axis, square to the skin, at a set force: it drives along `press_dir` (the axis plus the sideways give per
  N over the axial give), aims at `press_depth` (the depth for the force at that layout's give), and turns the TCP
  ahead by the wrist's give (`press_rot`). It closes the fingers along the ring finger (`PRESS_ROLL`): the wrist's
  larger turn is then about the axis the pads hold by their normal force, not the line through both pads, which only
  their torsional friction holds. It presses in two stages, to 5 N, a 0.5 s settle, then to 9 N, midway between the
  8 N fire and the shield bottoming at 9.8 N. What each change was for, from the traces of earlier plans [RUN,
  PhysX, 2026-11-20]: pushed along the axis alone the arm pulled the face 0.8 to 4 mm sideways on the skin before the
  click; closing across the finger let the lancet rock 2.5 mm in the grip at 7 to 8.4 N; a single push 20 mm past
  contact bottomed the shield and pushed the finger 1.4 mm down after the click.
- Out of the stand: the lancet's frame more than 4 mm across or 10 mm up from where it stood. This latches: a lancet
  put back in the stand still counts as out.
- Fired in the stand: with its tab off and still in the stand, the lancet pressed down to 8 N on its shield fires into
  the stand's post, as the real one would. It is then spent, and the trial scores 0.
- Held (the lancet): a gripper's finger half-gap 3.0 to 7.0 mm, from 2 mm inside the housing's narrower side (10 mm)
  to 0.5 mm outside its wider side (13 mm), and its TCP within 15 mm of the lancet's axis, anywhere from 10 mm beyond
  its face to 53 mm up it (`test_slipped_or_deep_grasp_in_the_container_is_still_held`); a gripper closed on nothing
  does not hold it.
- Let go: a gripper's half-gap over 7.0 mm, or its TCP more than 60 mm from the lancet's axis. The 0.75 and 1.0
  rungs need both grippers to have let go.
- The score, at the last step, 0 once any fault has latched: 0.25 the tab off and the lancet out of the stand; 0.5
  and fired square on the site, the ring finger held and still, without a slide; 0.75 and the used lancet inside the
  sharps container, let go by both grippers; 1.0 that held for 1 s. WHY no placed rung: see Review response, point 3.
- Dropped: the lancet out of the stand, held by neither gripper and not in the container: loose for 0.5 s.
- The trainer's base may turn or tip 2 deg and move 5 mm off its layout pose; more is a fault (dragged, lifted or
  pushed down, turned or tipped). It is clamped, so this checks that the sim placed it where the layout says.

## Design

Where the hand lies and which arm does what came from an offline search, run before this revision and not shipped with
the task (its scripts import this task's model and `tools/grippers.py`'s meshes) [RUN]. It posed the trainer at 1,152
hand poses: 8 turns on the table (every 45 deg), pitch 0 and 20 deg, 4 rolls about the fingers (every 90 deg), the
ring's apex at 9 places on the table and 2 heights. At each it tried every press (the site's angle round the pad, the
lancet's tilt, the jaw's turn) for either arm with every hold (along the finger, round it) for the other, by static IK
at the contact poses with yam_arena's arm, and the full link meshes (link1 to link5, the gripper, both tips, the
lancet): floors 10 mm arm to arm, 3 mm arm to trainer and to the table. 30,839 press and hold pairs passed, in 83 of
the 84 families (turn, pitch, roll, pressing arm) that had any; 335 were refined (the press sagged by 10 N, the hold's
approach) and 40 checked at the layout's 8 corners. The design here, F1, is the family with the fingers pointing away
from the arm bases, palm up, the right pressing and the left holding from the finger's far side from the thumb. A
pitch sweep of F1 (-20, -10, 0, 10 and 20 deg) passed at the corners at every pitch; -20 was taken for BD's step 4. The
search checks contact poses only. The full paths of the design here (park, approach, twist, press, sharps) are checked
by `tools/ikcheck.py`, `tools/grippers.py` and `tools/clearance.py` (Two arms on one finger). Two parameters moved in
those path checks: the apex came down from 140 to 100 mm, where the right's pose 45 mm back from the site is in reach
at every corner (at 140 it was out by up to 35 mm), and the hold moved from 90 to 86 mm back from the apex, where the
left's shelled tips, passing under the finger, clear the slide's web by 5.5 mm (2.6 at 90).

The demo needs a tilted grip, and yam_arena's scripted demo plays only a straight-down one turned by a yaw. So the task
file patches the scripted policy, and only it: after each reset, `_replan_demo_after_reset` swaps yam_scripted's two
plans for `TiltPlan`s, yam_arena's `ArmPlan` (the same smoothstep in position, slerp in rotation and IK) with each
waypoint's full TCP rotation from `TWaypoint.rot`. Like the demo reading the seed's layout, this is a privileged demo
helper. It changes nothing a model under test sees or does: Dream-1 and every other policy send their own actions,
and the scorer reads the scene, not the plan.

Two arms on one finger. `tools/grippers.py` poses each arm's collision meshes (link1 to link5, the gripper and both
tips of the YAM URDF) by forward kinematics along both plans, every 0.1 s, the left's gripper, tips and link5 with the
3 mm shells of the real trial (Rig protection), and reports the least distance between the arms, from each arm to the
trainer's fixed parts and its moving ring, to the stand and the container, and to the table, and the lancet's too.
Over seeds 0, 2 and 4, the layout's eight corners and every stem, the arms stay at least 30 mm apart (the check's cap: nowhere closer), each arm at least 4.7 mm from the trainer (stem a00; 5.5 mm in the demo, the left's shelled tips passing under the ring finger near the slide's web), at least 7.8 mm from the stand and the container (the lancet over the container's opening) and at least 45.7 mm from the table [RUN: tools/grippers.py, 2026-11-20]
(floors 10 mm between the arms, 3 mm to everything else [CHOICE]). Where the left's pads close on the ring finger by
design, the rest of the left must still clear it. The arms' lag on a descent (2 to 10 mm) is not in that check. To keep
the arms apart the left takes the finger while the right waits clear at (0.22, -0.28), its gripper turned to yaw 0 so
the turn to the tilted press stays in reach. After the press the right backs out to that same spot, and only once the
left has let go and parked does the right go to the container.

## Seeds

Each trial draws the trainer's layout from `random.Random(seed)`, the seed inspect-robots passes to the reset (the
CRC-32 of "<--seed>:None:0", its eval seed, scene seed and epoch): the base shifted in x and in y, each uniform within
+-12 mm, and turned uniform within +-6 deg about a point 140 mm behind the ring's apex along the hand (the palm's
middle). Before its shift the ring's apex is at (0.26, -0.12), and the hand is turned 180 deg right of +x: its fingers
point +x, away from the arm bases, its thumb toward the right arm. The bounds are where both arms' reach
(`tools/ikcheck.py`), the sharps clearance (`tools/clearance.py`) and the arms' clearance (`tools/grippers.py`) pass at
all 8 corners of the box; the demo replay scores 1.0 at all 8 corners, on `--seed` 0 to 19 and on every seed below
(tests). Some seeds:

| `--seed` | shift x, y, mm | turn, deg | |
|---|---|---|---|
| 0 | -7.0, 0.1 | -2.71 | the filmed demo and the stems (Sim results); replayed offline |
| 2 | -9.7, -5.5 | -4.11 | replayed offline |
| 4 | -0.2, -2.2 | 2.67 | replayed offline |
| 26385 | -11.9, -11.7 | -5.98 | corner -x, -y, -turn; PhysX demo with a 0.05 s start delay |
| 84277 | -11.6, -11.9 | 5.93 | corner -x, -y, +turn; PhysX demo |
| 5410 | -12.0, 11.9 | -5.90 | corner -x, +y, -turn; PhysX demo |
| 55059 | -11.6, 11.6 | 5.85 | corner -x, +y, +turn; PhysX demo |
| 126748 | 11.8, -11.8 | -5.96 | corner +x, -y, -turn; PhysX demo |
| 5673 | 11.7, -11.7 | 5.83 | corner +x, -y, +turn; PhysX demo |
| 58369 | 11.7, 11.7 | -5.81 | corner +x, +y, -turn; PhysX demo |
| 100825 | 11.5, 11.7 | 5.98 | corner +x, +y, +turn; PhysX demo |

The scripted demo reads the seed's layout and plans from it, as a privileged script may. It shows the task can be done.

## Real trials

Never on a person. The trial uses the trainer only, with nobody in the arms' reach while they move.

Parts and instruments (the printed parts are in `rig.scad`, which `tools/rig_scad.py` writes from the task's own
numbers; `test_rig_scad_is_current` checks it is current):

- BD Microtainer contact-activated lancets 366577 (21 G x 1.8 mm), one per trial; a BD 305635 sharps collector.
- Wallcur Lance fingerstick trainer fingers (REM-LANCE-KIT-01D; the kit lists "Empty Fingers"), a hollow sleeve that
  "fits over the finger of a standard patient or Mannequin"; its distributor's page gives no material [the sleeve is
  soft and skin-like: EST, not sourced]. One is fitted over the printed ring finger, a fresh one each trial, taped round
  its base so it cannot turn or slide. The Lance finger takes the puncture; check 6b shows where the blade stops. The
  ring's distal pad is printed thinner by the Lance finger's wall (`lance_wall`, check 3b).
- The trainer (`rig.scad`; units mm, z over the table, x level along the hand from the ring's apex toward the wrist, y
  toward the thumb; the hand lies palm up, its fingers sloping down 20 deg toward their tips, the ring's apex
  100 mm over the table):
  - The board, cut from 6 mm plywood (part `board`, a 2D outline): a plate x 80 to 200, y -40 to 110, under the hand and a bar x 200 to 420, y -20 to 50,
    running back between the arm bases, held to the table by two C-clamps (throat 60 mm or more) over the bar at x
    400 [CHOICE], and drilled for the post's and the pillar's M3 screws. In the sim the base is a fixed link; the
    clamps are what make that true in the rig. Its corners stay at least 14 mm inside the table's rear edge and 13 mm
    clear of the top camera stand's foot (`test_board_on_the_table`).
  - Part `base`, one print: the post, the palm block on it, the index finger on a rod into the palm block, and the
    thumb. The part is 171 by 125 by 168 mm, so it needs a bed of at least 181 by 135 mm and 173 mm of height (from the model's shapes; not
    rendered). The post: x 120 to 165, 70 mm across, z 6 to 146.221, with two M3 heat-set inserts in its bottom for the
    screws up through the board, and three M2 inserts in its -x face at z 84.581, 99.581 and 114.581 for the rail's screws.
  - Part `seat`, one print: a pillar x 104 to 114, 38 mm across, z 6 to 76.581, held by two M3 screws up through
    the board into inserts in its bottom; the spring pins pressed 10 mm into its top; on it the lower stop post, x
    106 to 112, 6 mm across, its top at z 87.081.
  - Part `ring`: the ring finger (distal radius 7.3 mm less the Lance wall, proximal 7.8 mm), its base end over a web
    x 106 to 112, fused to it, 17 mm wide up to z 124.581 and 10 mm wide above (2.3 mm inside the holding pads), down to a
    foot z 97.081 to 102.081 that spans the two spring pins. The guide's block is bolted to the web's +x face by four M2 screws
    (holes 12 by 8 mm apart, at z 104.331 and 112.331). The foot lands on the stop post after 10 mm: the lower stop. The
    ring's base end, square to its pitched axis, clears the rail's face by 3.2 mm.
  - The slide, bought: a miniature linear guide, LILY MGN7C1R40Z0CM, listed as interchangeable with HIWIN's MGN7C
    [SOURCE: LILY Bearing's product page, read 2026-11-19]: rail 40 mm long, 7 wide, 4.8 high, holes for M2 x 6 at a 15
    mm pitch, 5 mm from each end; block 22.5 long, 17 wide, 8 mm high over the rail's base, M2 holes 2.5 mm deep, 12 by
    8 mm apart; very light preload (Z0); basic static load 1.24 kN; static moments MR 4.7, MP 2.84, MY 2.84 N m; block 8
    g. The block's screws are M2 x 6 in 2.2 mm counterbores in the 6 mm web, so 2.2 mm of each enters the block's 2.5 mm
    thread; the rail's are M2 x 6 into the post's inserts. The rail runs z 79.581 to 119.581 on the post; the block, z 97.081 to 119.581
    at rest, stays on it over the 10 mm travel. The press at the demo's site is 107.1 mm from the block's middle, so a
    15 N press (check H's) puts at most 1.61 N m of moment on it, under MP's 2.84 N m [ARITH]. WHY a rolling guide:
    plain pins in printed holes jam when the load is more than L / (2 mu) ahead of them (16 mm for 8 mm of engagement
    at mu 0.25), and the press is about 100 mm ahead; a ball block has no such wedge [EST: its friction is not modelled;
    check R measures the give with it]. WHY not LM4UU bushings: THK's LM catalog says "When using the Linear Bushing on
    a single shaft, use two or more units" to avoid a moment load, and a guide carries the pitch moment in one part.
  - Bought, for the springs: two 4 x 45 mm steel dowel pins pressed 10 mm into the pillar's top at x 109, 15
    mm either side of the axis, z 66.581 to 111.581; two 4 mm bore shaft collars (10 mm OD, 5 mm wide [EST]) clamped on the pins
    on the foot's top, z 102.081 to 107.081: the top stop. Two compression springs, one on each pin between its seat and the
    foot, each music wire 0.32 mm on a 4.8 mm mean diameter (OD 5.12, ID 4.48), 18.8 active coils and 20.8 in all, ends closed and ground, 0.05 N/mm, free length 31.0 mm, 18.5 mm long at rest (0.623 N each, 1.25 N in all: the 1 N preload plus the finger's weight), 8.5 mm at the lower stop, solid 6.66 mm, peak stress 458 MPa [ARITH: `tools/rig_scad.py` from k = G d^4 / (8 D^3 n) with G 79.3 GPa, a
    handbook value, EST]. The spring index is 15, above the 4 to 12 a maker prefers [EST]; the pin inside keeps it from
    buckling. This is a full order spec for a custom spring (no catalog part was checked against it); a catalog spring
    within 10% of rate and free length passes if check R passes. The collars are any 4 mm bore set-screw shaft collar
    of that size. The springs sit at z 78.581 on two printed 1 mm shims each on the pillar (part `shims`); a shim
    taken out lowers each spring's force by 0.05 N.
- The stand (parts `stand` and `cup`) and the sharps container, each fixed to the table with double-sided carpet tape
  under its base, at the places task.json's setup gives:

  | part | size, mm |
  |---|---|
  | block | 40 by 40, 25 high, with a recess 24.4 mm across and 6 deep for the cup |
  | post | 10 diameter, fixed in the block's middle, its top 27 over the table, where the shield's face rests; a slot in its top cut to the tab cap's measured size (`tab_w`, `tab_t`, `tab_h`, check 3) holds the cap still |
  | cup | 24 diameter, turning on the post (10.4 bore), from z 19 to its rim at z 35; its pocket keyed to the housing, 13.8 by 10.8, 8 deep, 0.4 each side, as the model's socket |
  | stop | a 3 mm pin in the recess's floor, 9 from the axis, in an arc slot under the cup that stops it at +-100 deg, the model's TAB_STOP |
  | tab slot | `tab_w` 6 by `tab_t` 3, `tab_h` 4 deep [EST until check 3 measures the tab cap] |

  The tab slot's 6 x 3 x 4 mm is an estimate [EST]: the real tab cap has not been measured. Check 3 measures it, and
  the slot is cut to that size before the stand is printed.

  Turning the housing turns the cup while the post holds the tab still, so the tab's own plastic gives the twist's
  resistance.
- A push-pull force gauge (0.1 N readability) with a flat tip; a cap torque gauge (0.1 N-cm); digital calipers (0.01
  mm); a digital angle gauge (0.1 deg); a 0.01 g scale; a protractor; a 10x loupe; washable ink and a paper template of
  the site zone (1 mm and 5 deg lines, both sides of the pad); two steel rules (0.5 mm divisions), held upright by hand,
  one beside the web, which carries a scribed line, and one beside the fingertip, where an ink mark on the Lance
  finger's side at its axis sits 11.5 mm back from the apex (the model's TIP_MARK). Nothing is fixed to the board: the
  rules are in the cell only for check R and for a scale frame before the arms go live, then taken out, so the board
  carries only the parts in the sim and in `tools/grippers.py` (the left's shelled tips pass the web 5.5 mm off). The
  scale frame: the side camera, fixed, films both rules in place, which gives its mm per pixel at the plane of each
  mark; with the arms live, the marks are read in pixels against that frame. A side camera (60 fps) on both marks and
  the site; a second side camera at 90 deg to it; an overhead camera; a microphone on the same
  recording.

The rig file has not been rendered or printed: OpenSCAD is not installed where it was written, and the offline test
checks only that the file is current, its brackets balance and its numbers are the model's. A render would add that
each part is one closed solid a slicer accepts and that the web and the block clear the post, the rail, the pillar and
the palm over their travel.

Checks before any trial, each with its pass criterion (no trial runs until all pass):

1. Fire force: five lancets pressed by hand, shield first, onto the gauge's tip through a 3 mm silicone pad; the peak at
   the click. Pass: every one from 4 to 15 N (Unistik's range). If the mean is more than 1 N off the sim's 8 N,
   `F_FIRE` is set to it and the offline tests are re-run. No lower bound comes from the yield sweep: with friction the
   slide does not promise the give (The model), and the held rule scores a one-handed fire 0 whatever the force.
2. Tab: ten tabs twisted off in the stand by the torque gauge on the housing, five each way. Pass: every peak at or
   under 4.9 N-cm (Facet's limit) and every break at or under 60 deg (protractor), under the demo's 70 deg twist.
3. Body size and tab cap: five lancets with calipers. Pass: each side within 1 mm of the sim's 13 x 10 x 37 mm, or the
   sim is updated and its tests re-run. The tab cap's width, thickness and height set `tab_w`, `tab_t` and `tab_h`.
3b. Ring finger: the Lance finger's wall, read with calipers on a cut spare, sets `lance_wall` before the ring is
   printed. After printing, the ring fingertip's radius with the Lance finger on is read with calipers. Pass: within
   0.3 mm of the sim's 7.3 mm; else the ring is reprinted. The ring with the block bolted on weighs 25 g plus or minus
   5 g (0.01 g scale). The Lance finger's length is read with the rule. If it reaches past the distal joint's line
   (26.6 mm from the apex), it is trimmed at the line, so the proximal segment, where the hold closes, keeps its
   7.8 mm radius and the held reading matches the sim's; uncut, its wall would add about 1 mm to the half-gap the
   gripper reads there [EST: the 1.0 mm wall]. The real held window is set from check H's reading on this finger, so it
   would still hold, but the sim's 7.90 mm would no longer describe it.
R. Slide: the gauge straight down on the ring's pad, on its middle line 11.5 mm back from the apex, read at both
   points: the web's line and the fingertip mark on their rules, held beside them by hand (no arm is live). Pass, five times: the force at 2 mm of drop at the
   fingertip is 1.0 to 1.4 N (the sim's 1.20 N plus or minus 0.2 N); the two drops agree within 0.5 mm at 2 mm (the
   block does not pitch); the force at the web's first visible motion (0.25 mm) is logged as R0; and the finger returns to within 0.25 mm of its top stop at both points when let go. At 1.4
   N, with no friction, every allowed press still gives the unheld finger way by 1.085 N at 8 N (0.31063 x 8 - 1.4);
   with friction it need not, and the held rule scores that press 0. If it fails, in order, each followed by check R
   again: over 1.4 N, a shim out under each spring (0.1 N less); no return, the block wiped and re-oiled; still no
   return, or the drops differ by more than 0.5 mm, the block replaced on the same rail; if it still fails, no trial
   runs.
H. Hold: the left arm runs the demo's hold on the ring finger, which lifts the finger up against its top stop. First
   the hold's force: the gauge pressed straight down on the top of the ring's printed base over the web, behind the
   pads, until the web's line first moves (0.25 mm, side camera against the scale frame); that reading less R0 is the
   hold's lift. Pass: at most 15 N. WHY 15 N: at 15 N the lift, 28.2 mm ahead of the web's middle, bends the web's
   narrow 10 by 6 mm section to 7.3 MPa, about a seventh of PETG's tensile strength (about 50 MPa [EST: a typical
   datasheet figure, not checked for this filament], and printed layers can be weaker), and puts 0.53 N m on the
   guide's block, under a fifth of its 2.84 N m pitch rating [ARITH: `rig_scad.numbers()`]. Over 15 N, the hold's aim
   over the finger (`HOLD_UP`, 25 mm) is lowered in 5 mm steps, the offline tests and the stems re-run, and check H
   repeated. Then the gauge pressed at the demo's site along the demo's press direction (48 deg from vertical, set
   with the angle gauge) to 15 N. Pass: the web's line and the fingertip mark each move under 1 mm (side camera against
   the scale frame). The left's gripper reading, closed on the finger, is logged here: it is the setup reading the
   trial's held check uses.
5. Square: a 2 mm rod held square to the skin at the demo's site, 11.5 mm back from the apex and 45 deg round on the
   thumb's side, reads 48 deg from vertical within 3 deg (angle gauge): the pad's normal there, acos(cos 20 x cos 45).
6. Patch: the site zone of a spare Lance finger, on the ring, is coated with washable ink and left to dry. Five
   lancets are fired by hand, square and without sliding, the finger held, each on a fresh inked spot; where the
   shield's face pressed, the ink lifts and leaves a bare patch. Pass: a patch is seen under the loupe every time. Its
   longest length (calipers), largest of the five, is L6, the no-slide patch.
6b. Blade: a spare Lance finger over a dowel of the ring fingertip's radius wrapped in carbon paper on white paper;
   five lancets fired by hand through it. Pass: no mark on the paper under the loupe every time. This is expected to
   fail on the estimates (the blade's 1.8 mm against the Lance wall's 1.0 mm [EST]). If it fails, the ring's pad is
   printed thinner by a further 1.0 mm and wrapped in 1.0 mm self-adhesive silicone tape [CHOICE] under the Lance
   finger, so the radius stays the model's and the blade ends in the tape; then check 6b is re-run, then check 3b.
6c. Slide: five lancets fired by hand on fresh inked spots, each slid 2 mm along the finger on the skin before the
   click, the slide set by a stop on a guide. Pass: every patch is at least L6 + 1.0 mm long.
7. Grip: with the right's bare pads, the lancet held in the gripper at the demo's target and pressed on the gauge to
   15 N does not slip in the grip (no change in the camera over 2 s).
8. Covers: five lancets fired by hand, shield on a spare 3 mm polycarbonate finger shell (the left's, Rig protection) laid over carbon paper on white
   card. Pass: no
   mark on the card under the loupe every time.

Scoring a trial, each threshold with its instrument (before the trial the site zone of the Lance finger is inked as in
check 6; nobody reaches into the cell once the arms are live):

- Tab off before the lift: the overhead video shows the lancet turned in the cup before it rises 10 mm (the mm scale
  on the stand), and the tab is left in the post's slot. A click while the lancet is still in the cup scores 0 (fired
  in the stand).
- Fired: the click on the audio track. Spent: after the trial the lancet is pressed on the gauge to 15 N with no second
  click (BD: auto-disabled).
- Touch: the first frame of the side camera (60 fps) where the shield's face meets the ring finger, the sim's touch
  (The model, the touch). A touch on the table, the board or another finger does not start the window.
- Held: in every frame from the touch until 0.5 s after the click, the overhead and side videos show a gripper closed
  on the ring finger with both pads on it (no light between pad and finger in the side view), its pad tips past the
  finger's axis line (side view), and the middle of its pads (the line midway between them along the finger, where
  the sim's TCP is) between the distal joint's line (inked 26.6 mm from the apex) and the knuckle's line (inked at
  99 mm); and its logged gripper reading is from 1.5 mm under to at most 0.5 mm over that gripper's reading closed on the
  finger in check H (wider, the pads are off the skin; 0.5 mm, not the sim's 0.15, because the real gripper's reading
  noise has not been measured [CHOICE]).
  These are the sim's held rule (The model, Held) in what the cameras and the arm's log show. A frame without such a
  gripper scores 0.
- Still: from the side camera at 60 fps, the web's line and the fingertip mark each stay within 2 mm of where they sat
  at rest, read in pixels against the scale frame (to 0.25 mm [EST]), from the touch's frame until 0.5 s after the click (the audio on the
  same recording).
- The patch scored: where the lancet was placed more than once, the inked finger shows a patch for each placement;
  the one scored is the patch under the press that gave the click, picked on the overhead video.
- Site: that patch's centre lies inside the zone on the template laid on the ring fingertip (8 to 15 mm back from the
  apex, 15 to 60 deg round from the pad's middle line, either side), checked with the loupe. The template reads its s
  (mm back from the apex) and its theta (deg round from the pad's middle line, signed: positive toward the thumb).
- Slide: the patch's longest length is at most L6 plus 1.0 mm (calipers).
- Square, against the skin's local normal at the patch: from the two side cameras, set to the trial's turn of the base
  (from its seed) with a protractor against the board's edge, one looking across the finger and one along it: at the
  click, the housing top's lean a toward the fingertip (the across view) and b toward the thumb (the along view),
  protractor overlay, each signed (negative away). In the rig's frame (x level toward the wrist, y toward the thumb, z
  up) the skin's outward normal at the patch is n = (sin P cos theta, sin theta, cos P cos theta), P = -20 deg the
  hand's pitch (every site is 8 mm or more back, past the pad's radius, so the normal has no lean along the finger
  beyond the pitch), and the housing's axis is d = (-tan a, tan b, 1). The lancet's angle off the normal is acos(n . d /
  (|n| |d|)). Pass: at most 10 deg. These are the scorer's own quantities: `test_real_square_check_matches_the_scorer`
  checks this reading against `tilt_of` at 400 random sites on both sides and leans, and
  `test_real_square_check_both_sides_of_theta` at both ends of the band on both sides with leans of both signs.
- In the container: the overhead video shows the whole lancet pass below the lid's opening (the lid's edge in view),
  and the container, weighed before and after the trial on the 0.01 g scale (outside the cell), gains the lancet's mass
  (weighed in check 3) within 0.1 g; the weight alone cannot tell a lancet on the lid from one inside. The video shows
  each gripper open (fingers more than 14 mm apart) or its TCP more than 60 mm from the lancet; no lancet loose
  elsewhere for 0.5 s.
- The base: within 5 mm of its marked outline on the table, turned under 2 deg (a protractor against the outline,
  before and after) and tipped under 2 deg (the angle gauge on its board). Clamped, it should not move; this reads it.

Rig protection and sharps handling (for the trial, not part of the task):

- The blade's exposure: BD's 510(k) says "after skin puncture, the needle/blade returns into the device" and "Once
  activated, the device is auto-disabled". So the blade is out for one stroke of at most 1.8 mm past the shield's face,
  only where the face is pressed at the fire force, and never again.
- The left arm, beside the press: each finger of its gripper is capped over its tip, outer face and grip face with a
  3 mm polycarbonate shell (the grip face's under thin grip tape), and its gripper housing and link5 are wrapped in
  3 mm polycarbonate sheet [CHOICE]. Foam is not used: it compresses under the press, and the blade would reach the
  pad. Check 8 proves the shell. `tools/grippers.py` counts these 3 mm on every point of those links (Two arms on one
  finger).
- The right arm is not shelled. It holds the lancet face down along its own approach by its housing, so the face is
  past its own fingers and housing; its parts can meet the face only if the lancet is handed to the left. That hand-off
  is legal, in the sim and in the real trial alike: the instruction assigns no gripper, and there is no E-stop for it.
  The cost is accepted: a policy that hands over the armed lancet can fire it once on the right's bare parts, a single
  stroke of at most 1.8 mm, and such a fire is off the site, so it scores 0 in both. Once fired, the lancet is spent
  and cannot cut again. WHY not both: with 3 mm on every link of both arms, the right came 0.9 mm from the stand at corner -12, -12 mm, -6 deg [RUN: `tools/grippers.py cover=3`], under the 3 mm floor. Check 7 is run with the right's bare pads, as the trial is.
- Every rig part: a policy is not the demo, so the lancet may be pressed on anything under it. The table around the
  trainer's board, the stand and the container is covered with a 3 mm polycarbonate sheet, cut out round their
  footprints so they stand on the table itself and every height in task.json holds. Every exposed cable within 300 mm
  of the work area is in split loom [CHOICE]. The printed parts are plastic and cheap to reprint; the cameras and the
  microphone stand outside the arms' reach.
- Sharps: the operator wears puncture-resistant gloves and loads a new lancet (tab on, sealed) into the cup by hand.
  A used lancet is never touched by hand: one that was dropped is picked up with forceps into the container. The tab
  is taken out of the post's slot with tweezers. The container is closed and replaced at its fill line. Handling
  follows the BD CAUTION above.
- E-stop: the operator holds it; the arms are stopped before anyone reaches into the cell.

## Run

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=lancet_fingerstick.py -E object_jitter=0 \
  -E control_hz=20 --instruction "$(python3 -c "import json;print(json.load(open('task.json'))['instruction'])")" \
  --scorer task_score --seed 0 --max-steps 1200
```

The run limit is 1,200 steps (60 s). The demo's plans take 41.0 s (right) and 36.1 s (left); the slowest stem, k00, 44.4 s. Every run below
uses the same 1,200 steps. The scripted demo plays its plans as `TiltPlan`s (Design), a demo helper only.

## Sim results

The trainer's pose, the board, the site and the hold changed after the earlier PhysX runs, so none of those is
quoted. One traced run of the previous revision is, below, for what it measured. The film and the runs in the table
were run on the files here, in the camera harness (`sim/film.sh`, `sim/run.sh`, 2026-11-20) [RUN].

yam_arena, `--seed 0` (the first row under Seeds), `object_jitter=0`, 20 Hz, 1,200-step limit. The film scored 1.0, success at step
801 (40.05 s), in 305 s of wall time [RUN].
Dream-1 scored 0 over the 1,200 steps (stopped at the step limit), in 992 s of wall time [RUN]. Both runs hash `lancet_fingerstick.py`, `mechanism.py` and `task.json` before starting (`sim/film.sh`,
`sim/dream.sh`); both hashes match the files here (`task.json` as it was before its `scores` line was filled in).
Expected for the film: 1.0 inside the 1,200 steps; the left's half-gap (`lf` in the trace) inside 6.40 to 8.20 mm
from the touch until 0.5 s after the click; and the base's height (`Hz` in the trace) steady while the left lifts.

A traced PhysX run of the previous revision (the t00 copy, `--seed 0`) scored 1.0 at step 735, fired at step 472
[RUN]. That revision differs from these files in the held window and the touch rule only, and its trace set both.

- The left's reading closed on the finger: 7.89 to 7.94 mm from the touch until 0.5 s after the click, against the
  7.2 mm planned. `HOLD_GAP_ON` is now 7.90 mm, and the window's top is 0.30 mm over it (The model, Held).
- The touch: the face within 0.3 mm of the skin at step 450, the shield off its stop at step 460, 0.5 s later. Under
  the rule here, the window starts at step 450; the left read 7.93 mm from step 450 on, so the hold covers it.
- The base: its root at the layout pose (0.2529, -0.1265, 0.8475) at every traced step, equal to `layout()` within
  the trace's 0.1 mm rounding, and the ring link moved only along its slide (0.6 mm in all, 0.5 mm the most in the
  window). The trace does not log the base's yaw; the base-turned check (2 deg) did not latch, and the press landed
  11.8 mm back and 45.9 deg round against the aim's 11.5 mm and 45 deg, which a misplaced yaw would shift.
- The right arm's lag: its joints follow their targets about 2 control steps (0.1 s) behind; the largest gap was
  0.14 rad on joint 6 during the twist and at most 0.044 rad from the touch until 0.5 s after the click. The press
  still landed 2.4 deg from square with a 0.43 mm slide, so no plan was changed for it.

PhysX runs of the next revision (the held window and touch rule here, the lancet's housing still at the plastic's
0.5/0.4 friction) [RUN]: the demo at corners 126748, 84277 and 100825 scored 0, 0 and 1.0; the stems o00, j00, x00,
w00, a00, g00 and y00 scored 0, all "not held still", latched before the fire, as their rows expect; k00 0.5 as
expected; f00 1.0 and c00 0, both against their rows. The traces give one cause for all four misses: the right's grip
let the housing slide up its pads at 5 to 6 N, under the 8 N fire. In the stand (f00) the press peaked at 5.26 N while the housing slid 3.5 mm
up the pads (run rr_1120_0020, `trace_f00_1415158438.csv`, steps 176 to 194), so it never fired there, and the stem went on to finish the demo. On the skin the housing slid 1 to 10 mm up the pads
and the lancet rocked (up to 10 deg in c00), so the face moved 1.0 to 4.7 mm on the skin before the click, a slide by
the scorer's 1.0 mm rule. The held window and the touch were not involved: the left read 7.89 to 7.94 mm and no
fault was "not held". The fix is in the lancet, not the scorer: its housing now has yam_arena's grasped-object
friction, 1.2/1.0 (`MATERIALS["housing"]`), which the real grip must match anyway (check 7: 15 N with no slip). The
three corners, f00, c00 and k00 are re-run on the files here; the other stems latched before the grip slipped and
are not.

PhysX re-runs of that fix (the housing at 1.2/1.0) [RUN]: corner 84277 scored 1.0; corners 126748 and 100825 and
stem f00 scored 0 (100825 had scored 1.0 the run before: pass and fail moved between corners from run to run). The
traces: at 126748 the press slid 1.1 mm before the click; at 100825 the lancet jumped 14 mm across the pads at 7 N,
and it fired on the ring finger 28.6 mm back, off the fingertip, 33 deg from square; f00 slid 1.2 mm and fired on
the ring. All three were the lancet moving in the grip near the fire, so the demo had no margin. The changes, each
from a traced cause in PhysX runs of the scripted demo (2026-11-20): the press allows for the arm's give
(`press_dir`, `press_depth`, `press_rot`, The model), closes the fingers along the ring finger (`PRESS_ROLL`), and
goes in two stages to 9 N; the lancet is turned back in the socket before it is lifted out (The model, the tab); the
housing weighs 20 g in the sim (The model, the lancet); and the last 4 mm comes in slowly (2.5 s, 1.0 s steady,
1.6 s on), where the faster approach let the face swing 2 to 8 deg round the finger. The held window's top moved
from 0.15 to 0.3 mm over the left's reading, from the traces (Held). On the files here the demo at `--seed` 0, 1, 2
and corners 126748 and 100825 scored 1.0 each, success at step 801, in each of two fastruns (fr8 and fr9, one
result.json per seed; the two gave the same numbers; the copy differs from these files in comments only), at every
one with: the slide 0.28 to
0.30 mm (limit 1.0), the tilt at the click 0.2 to 4.1 deg (limit 10), the finger's drop 0.0 mm (limit 2.0), the site
10.4 to 11.4 mm back (8 to 15) and 41.7 to 46.9 deg round (15 to 60), the left's half-gap 7.82 to 7.96 mm (6.40 to
8.20), and the housing's slip in the grip 0.1 mm.


More PhysX runs of the scripted policy, each on a copy that `sim/mk.sh` makes of this `lancet_fingerstick.py` with a
per-step trace appended (`sim/log_snippet.py`, which writes `sim/out/trace_<copy>_<seed>.csv`), and for the stems
`sim/stems.py` too. Each stem's offline replay gives the expected score (`test_sim_stems`):

| Run | `--seed` | Copy | Expected | PhysX score, steps, fault |
|---|---|---|---|---|
| demo at the 7 other layout corners | 126748, 84277, 100825, 5410, 55059, 5673, 58369 | t00 | 1.0, success | 1.0 at 126748, 84277 and 100825 (copies t00, t00b, t00c), success at step 801 each; the other four not run on these files |
| demo with a 0.05 s start delay, corner -x, -y, -turn | 26385 | t05 | 1.0, success | 1.0, success at step 802 |
| one arm: left parked, the demo's press | 0 | o00 | 0, not held still (or fired with no gripper holding the ring finger) | 0 at step 881, not held still (gave 2.2 mm under the press) |
| one arm: left parked, a 0.15 s jab | 0 | j00 | 0, not held still (or fired with no gripper holding the ring finger) | 0 at step 864, not held still (2.4 mm) |
| one arm: left parked, a jab 30 mm past contact in one control step (0.05 s) | 0 | x00 | 0, not held still (or fired with no gripper holding the ring finger) | 0 at step 862, not held still (6.5 mm) |
| one arm: left parked, a press 8.5 mm from the apex driven 10 deg toward the knuckle | 0 | w00 | 0, not held still (or slid, or fired with no gripper holding the ring finger) | 0 at step 881, not held still (2.2 mm) |
| both arms: the lancet pressed down in the stand after the twist | 0 | f00 | 1.0, success (in the sim the press in the stand cannot fire; below) | 1.0, success at step 843; in the stand the shield peaked at 3.99 N, 0.75 mm (run rr_1120_0646, `trace_f00_1415158438.csv`, step 192) |
| both arms: after firing, the lancet held inside the container, never let go | 0 | k00 | 0.5 | 0.5 at step 949 (fired well, never let go) |
| both arms: the tab twisted the other way (-70 deg) | 0 | c00 | 1.0, success | 1.0, success at step 801 |
| both arms: the left closed to 5 mm on air 26 mm above the ring finger's axis, and a jab 30 mm past contact in one step | 0 | a00 | 0, not held still (or fired with no gripper holding the ring finger) | 0 at step 862, not held still (6.5 mm) |
| both arms: the left closes on the ring finger but does not lift it | 0 | g00 | 0, not held still | 0 at step 881, not held still (2.1 mm) |
| both arms: the demo's hold, and a jab 60 mm past contact in one step | 0 | y00 | 0, not held still | 0 at step 862, not held still (2.5 mm) |

The stems' plans pass the same reach check as the demo's (`tools/ikcheck.py`). The offline replay is quasi-static, so
x00 replays as a slow press; whether a PhysX jab fires before the finger moves is what x00 shows, and either way the
window scores it 0. For the one-arm stems (o00, j00, x00, w00) either fault scores 0: "not held still" if the finger
gives way before the fire, "fired with no gripper holding the ring finger" if it does not (the press 48 deg from
vertical, with friction, need not push it down; The model, the yield sweep). Whichever PhysX gives is reported as
found. g00 and y00 test the hold itself in PhysX: the offline replay holds the finger as a joint with the arm's
assumed stiffness, so whether PhysX lets the press through a grip that does not lift, or a 60 mm jab through the
demo's hold, is what those two runs show: neither did.

f00 does not fire in the stand in the sim, so it finishes the demo; this is a limit of the sim, not of the scorer.
The filmed copy's press (20 mm down over 1.5 s) peaked at 3.99 N, 0.75 mm (run rr_1120_0646, step 192). Three debug
fastruns of f00 logged every body's height (a logger not shipped; the task files unchanged) [RUN, 2026-11-20]: the
filmed press; one 28.6 mm deep, aimed at 12 N and held 0.5 s; and a 30 mm jab in one control step. The deeper two
stopped at the same place: the shield at 1.05 mm, 5.2 N; the shield's face 1.9 mm down into the 2 mm tab cap it
stands on; and the housing's front 0.05 mm above the cap's top, where it bears the rest of the push. With the face
that deep, the housing's 3 mm stand off leaves the shield about 1 mm of travel, under the 1.75 mm the fire needs.
Proven by those heights: the housing's front meets the tab cap. Not proven: why the face sinks into the cap (the
contact between the 2 g shield and the 4 g socket in PhysX; not checked). The rule itself (a lancet, its tab off,
fired in the stand scores 0) is checked by `test_fired_in_the_stand_latches`, which puts 8.1 N on the shield in the
stand.

## Sim against real

- The needle, the skin's puncture and the blood are not simulated; the shield's spring stands in for the drive and
  fires at a set force. The real lancet's click and its spent state are read from audio and a re-press.
- A real finger yields at its joints, its wrist and its arm; the trainer lumps that into one vertical sprung slide
  behind the ring finger's knuckle, in the sim and in the rig. That an unheld real finger withdraws under a lancet is
  our inference (The model, the trainer).
- The real slide is a rolling linear guide, its pitch moment rated 2.84 N m against at most 1.61 N m from a 15 N
  press [ARITH]; its friction is not modelled. Check R measures the give at the fingertip and at the web, friction and
  any pitch included.
- The board, the post, the seat pillar and the palm block are separate parts in the rig, screwed and clamped; in the
  sim the base is one rigid link fixed to the world, and the rail, the pillar, the stop post, the block and the web are
  drawn only.
- The yield sweep is quasi-static. Its guarantee holds only for frictionless skin; a real Lance finger's silicone on
  the shield's polyethylene has not been measured. The held rule, read by camera in the real trial, does not depend
  on it.
- The left's 3 mm shells are not in the sim's gripper: closed on the finger at the sim's command, the real left
  squeezes 3 mm more each side [EST: the YAM gripper's own force limit governs that squeeze; not measured]. The real
  held check uses the shelled gripper's own reading (check H), not the sim's.
- The real slide check reads a patch that also records motion after the click (check 6c); the sim stops counting at
  the fire.
- BD's step 7 figures show the lancet straight down (the arrow is vertical) on the side of the pad, so the collector
  turns the finger to bring the site under a vertical press [inference from the figures]. Here the trainer's roll is
  fixed, and the robot tilts the lancet 48 deg from vertical to meet the site square instead.
- BD has the patient's hand held downward "allowing gravity to help increase blood supply to the hand"; the trainer's
  fingers slope down 20 deg for the pose, but it has no blood, so the slope does nothing for blood flow here.
- Out of scope, from BD's guide: warming the site, the alcohol wipe and air dry, wiping the first drop, collecting the
  specimen and pressing on the site after. Those need blood, a wipe or a tube on a fingertip the trainer cannot bleed
  from, and none of them is done with the lancet.

## Tests

`python3 test_lancet_fingerstick.py` (with yam_arena on PYTHONPATH) runs 75 offline tests: the shield and the site
(the demo's site and its 48 deg press; the pad's middle, too far round, past the pad or short of 8 mm, the index), the
tab either way, a good press, no fire under 8 N, a placement needing the finger held (a trace field, not a rung), a
touch off the ring finger not starting the window, the touch read 0.3 mm off the skin, a brush before the hold, a hold
beside the press that also reads as holding the lancet, either gripper carrying the lancet, a fire with no hold, a slide, a small creep and a fresh placement, a
tilted lancet, the finger giving way, the jab window past the click and from first touch, the held rule (index, palm,
open, on nothing, past the distal joint, behind the knuckle, on air beside and above the finger, short of the axis, wider than its half-gap window),
squareness against the local normal at every site on both sides, fires off the hand and in the stand, a drop and the
container, a slipped or deep grasp still held, the housing's grip friction, the yield sweep (its friction limit, and a design that fails it), the
scorer (a model that stops, the base dragged, lifted, turned or tipped, the ladder in order along the demo), both arms'
demo replayed at the 8 corners of the layout box, on `--seed` 0 to 19 and on every seed in the table, the layout's
draw, a plan replayed on another seed's layout failing, the base clear of the stand and the container, the board on the
table and clear of the camera stand, the lazy routes below and the sim stems, the geometry (palm up with the fingertips
lowered, the slide's parts), these numbers against the code, the real trial's two-camera squareness reading against the
scorer's tilt, and `rig.scad` current and at the model's sizes. `tools/ikcheck.py` (no argument: the 8 corners,
`--seed` 0, 2, 4 and the stems) checks both arms' reach and joint6; `tools/clearance.py` every arm link against the
sharps container; `tools/grippers.py` the arms against each other, the trainer, the bench and the table.

### Lazy routes

| Shortcut | Score | Why |
|---|---|---|
| Press without holding the ring finger | 0 | the held rule: "fired with no gripper holding the ring finger"; with no friction the finger also gives way first ("the ring finger was not held still") |
| Jab so fast it fires before the finger moves | 0 | no gripper held it; the finger may also give within 0.5 s of the click |
| Hold the index finger, the palm or the base instead | 0 | none holds the ring finger; "fired with no gripper holding the ring finger" or "not held still" |
| Close a gripper on air beside or over the ring finger | 0 | its gap, its offset along the closing direction or its tips short of the axis say it is not on the finger (stem a00) |
| Grip the ring finger, fire, then let go within 0.5 s of the click | 0 | "not held from first touch to 0.5 s after the click" |
| Grip the ring finger without lifting it | 0 | the press meets only the spring and the arm's stiffness; "not held still" (stem g00) |
| Push the ring finger down to its lower stop, then press | 0 | it is 10 mm down; "not held still" |
| Pull the lancet straight out, no twist | 0 | "lifted out of the stand with its tab still on" |
| Twist, then press the lancet down in the stand | 0 if it fires | "fired in the stand" (`test_fired_in_the_stand_latches`); in the sim the housing meets the tab cap first and it does not fire (stem f00, Sim results) |
| Press on the index fingertip | 0 | "not the side of the ring fingertip" |
| Press on the pad's middle line | 0 | "on the middle of the ring's pad, not its side" |
| Hand the armed lancet to the left and fire it on the right's gripper | 0 | off the hand: "fired not on the hand" |
| Press straight down anywhere on the site | 0 | the site's normals lean 25 deg or more, past the 10 deg square rule; "fired tilted" |
| Press 30 mm back, past the pad | 0 | "not the fingertip" |
| Slide the lancet while pressing | 0 | "the press slid" |
| Drop the used lancet on the table | 0 | "lancet dropped outside the sharps container" |
| Fire, then keep holding it over the container | 0.5 | never let go into the container |
| Fire, then hold it inside the container, at any grasp | 0.5 | held: never let go |
| Do nothing | 0 | no rung |

## Closest tasks

- #2259 (merged): loading a lancet into a pen lancing device, its tab twisted off. That loads a home device and never
  punctures; here a clinical contact-activated lancet is fired on a fingertip held still by the other hand. The tab
  twist is shared and is only the 0.25 rung here. #2259's review flag ("Protect the gripper pads or use suitable blunt
  training lancets") is answered under Rig protection.
- #2599 (open): an intramuscular deltoid injection with a retractable safety syringe. That is an injection into a
  training arm: a needle pushed in and a plunger pressed to deliver a dose. Here nothing is delivered: a capillary
  lancet fires itself on contact, on a fingertip the other hand must hold still, and the site is a fingertip's side.
- #1837 (merged): clipping a pulse oximeter on a trainer's finger while steadying the hand. Its hand sizes are used
  here. It has no sharp, no firing force, and no site or squareness rule.
- #1846 (final): a pulse oximeter clip, a duplicate of #1837.
- #1941 (merged): changing the blood tube on a needle already in a training arm's vein. Venipuncture with the needle
  in place; nothing is fired and no site is chosen.
- #2422 (not accepted, open) and #2403 (open), from their titles only: a glucose meter's control check, which uses
  control solution rather than a puncture, and drawing a splinter out of a fingertip with tweezers.
- #2039 (final, duplicate of an earlier task): filling blood tubes from a needle holder in a training arm. Its review
  says "Blood collection is core work for medical assistants and phlebotomists". That is venipuncture; this is
  capillary puncture, a different device and site.
- #862 (final): vaccinating a training arm with an inert pen, rejected because "The pen has no needle, activation
  mechanism or dose delivery". Here the lancet is a named product whose activation is modelled from its shield, and
  firing is read from the lancet itself.
- #2049, #2082, #1861 (merged): putting used needles in sharps. Here the discard is the last step, not the skill.
- #612 (final, blood tubes in order of draw) and #2242 (open, scanning a wristband): other clinical steps, no sharps.

Category: capillary puncture by finger is the O*NET Core task of Phlebotomists (143,900): "Draw blood from capillaries
by dermal puncture, such as heel or finger stick methods." Both national medical assistant exams test it: the CMA
(AAMA) outline lists "Capillary/Dermal puncture (finger, heel, ear)" and NCCT's plan "Perform capillary punctures (i.e.,
fingerstick, heelstick)." It falls under the Core tasks of Medical Assistants (833,900), "Collect blood, tissue, or
other laboratory specimens, log the specimens, and prepare them for testing.", and of Licensed Practical and Licensed
Vocational Nurses (666,900), "Collect samples, such as blood, urine, or sputum from patients, and perform routine
laboratory tests on samples." The accepted #1951 rests on one Medical Assistants Core task, and #2049
on tasks of three occupations; this follows #2049.

## Third-party code

`mechanism.py` is adapted from benchmark-tasks pull request #223 (MIT License; the licence is kept in its header).
