# apple-wedger-press-through-to-board: revision notes, deviations and known limits

What this revision changed against the first submitted version, and why. The review asked for (1) no cutting credit
from a pressed core, (2) a score a real apple can produce, and (3) a repeatable specimen.

1. **The core slide and its yield law are gone.** The first version scored `cut_depth = -fruit.joints["cut"]`: a 48 mm
   prismatic core anvil under an 18 N / 4.5 mm yield law, advanced from an `@on_step` hook with
   `set_joint_position_target`. The anvil stood 1.5 mm above the crown and its 15 mm disc overlapped the 9.2-11.5 mm
   corer annulus, so the wedger's own tube (or any finger on the crown) earned the cut. Removed: `cut_joint`,
   `CUT_FORCE_N`, `CUT_YIELD_M`, `CUT_K`, `_advance_cut_yield`, `cut_depth()`. No `set_joint_position_target` or
   `write_joint_stiffness_to_sim` remains in the file.
2. **The apple is eight free wedges plus a core plug, each glued to the world on its own seam line** (`mechkit`
   bonds `seam_0..7`, `seam_core`). The first version's wedges were pre-scored geometry with 5.34 mm of open slot per
   blade plane; here a seam is released by the cut rule only when a blade is within 2.5 mm and 6 deg of that seam
   plane, with the blade bottoms driven through to the board (`THROUGH = APPLE_H - 0.002`, i.e. the ring bottoming on
   the deck), the wedger level within 5 deg and every blade within 5 mm of its seam. The core plug's glue releases by
   the same rule when the corer reaches the board. The cut therefore grows exactly where a blade is.
3. **Scorer from observable geometry only:** separated seams, each wedge within 12 mm of its seat, ring within 1.5 mm
   of the deck (the first version's 4 mm tolerance let a demo ending 3.5 mm above the board score 1.0), tilt within
   5 deg, concentric within 10 mm, both TCPs more than 120 mm clear, 3 s hold. `SUCCESS_HOLD_S = 3.0` (first version: 0.5 s
   in code vs 3 s in text). Tiers 1.0 / 0.75 / 0.50 / 0.0 as in `task.json`: the 0.25 tier is gone because a seam
   not cut through is not an observable separation, so there is no partial-cut credit between "nothing" and a board
   state.
4. **Specimen:** Granny Smith, 64-67 mm band (66 mm modelled), 54 mm tall, 150 g, 20 +- 2 C, one apple per trial,
   wiped dry; stand-in a machinable wax cylinder 0.93 g/cm^3, 66 x 54 mm with 8 pre-scored 6 mm kerfs. The first
   version's "USDA ~182 g" figure had no citation and is dropped. The real wedger is the OXO Good Grips 32681
   (ASIN B00004OCKT): 114 mm blade span, 80 g, 2.1 x 4.5 in. The first version's 210 g / 160 mm span was not that part.
5. **Plain board, no cradle pocket.** The first version's routed 77 mm pocket and its dead ASIN are dropped; the apple
   stands on a plain 203 x 305 x 19 mm maple board. Board moved to x = 0.360 m so its near edge clears the staged
   ring by 16 mm (at 0.345 m the edge sat 10.5 mm inside the ring and the pick flung the wedger 0.3 m).
6. **Blade geometry bug found and fixed during the build (2026-11-19).** An early draft of this revision placed the eight
   radial blades and corer flats at `k * pi / 8` (22.5 deg pitch over a half circle, i.e. on the wedge midplanes)
   while the seams and the scorer's `_blade_dirs` use `k * 2 pi / 8`. The ring rolled 60-90 deg on crown contact in
   every one of 18 press-knob cells; the contact trace put blade boxes on the middle of `wedge_0` and `wedge_3`. With
   both loops on the 45 deg grid the scripted demo reaches 1.0 in 326 steps with 1.2 deg residual tilt.
   `test_blades_sit_on_seam_planes` and `test_seams_and_blades_are_antiphase` pin it.
7. **Demo knobs:** wrist splay 14 deg (9 deg let the link-4 housings push each other at 19.7 N during the pinch;
   14 deg: 3.9 N), forward aim 5.5 mm against wrist compliance, press overdrive 20 mm.
8. **Approach rewritten for PhysX (2026-11-19, measured over nine local Isaac 6.0.1 probes).** (a) A full-force
   pinch (finger target 0, 13 mm of error at 5000 N/m, 50 N cap) yawed the wedger 4.6 deg by the crown in Isaac, because
   the two finger carriages are offset across the closing axis and are driven independently there: the outer blades
   then missed the kerfs. The pinch now stops 2.5 mm past grip contact (~12 N per pad): yaw 0.4 deg. (b) The register
   move used to end 4 mm into the crown at 2-4 mm per step; in PhysX that first contact swung the ring about the pinch
   axis and the pads lost the grip. It now ends 4 mm above the crown and the 4 s drive crosses it at 0.9 mm per step.
   (c) The release opens the fingers 12 mm past the grip faces instead of fully: fully open, the inboard pads of the
   two grippers meet at the midline (436 N in the MuJoCo stand-in). Tried and withdrawn: pinching the grip ends along
   x (the two TCPs walked 50 mm apart), 2 mm entry chamfers on the kerf edges (PhysX ejects the wedger: a rigid slope
   under a 50 N press is an impulse that the MuJoCo stand-in's soft contacts hide). (d) A further probe confirmed the
   slow drive crosses the crown cleanly, but 1.2 mm blades in a 6 mm kerf still sit inside PhysX's 5 mm contact offset,
   so the blades and corer are now visual-only (item 10). With that change the scripted demo scored 1.0 in Isaac three
   times: ring bottomed 0.0 mm on the board, dx 0.9 / dy -0.3 mm, yaw 0.6 deg, tilt 0.0, all 8 seams released by step
   160, a 114-step hold at the end of every run, task_score 1.
9. **Negative-case note (hover).** An early version of the offline hover case first held the wedger at the register
   pose (4 mm into the crown) and then released it over the apple; both variants cut and scored above zero on the tree
   then in use (a partial-cut tier that has since been removed), which is a press, not a hover. The case now holds
   30 mm above the crown and lifts away without ever releasing over the fruit: 0.0.

10. **Pass-through cutting blades (PhysX, 2026-11-19).** The first probing attempt blew up: 1.2 mm blades inside 6 mm
   kerfs sit inside PhysX's 5 mm contact offset, and a rigid slope under a 50 N press is an impulse. The blades and the
   coring tube are therefore visual-only (`mechkit box(..., collide=False)`): they spawn without a collider in Isaac and
   pass through the fruit without collision, and cutting force is not modelled. This follows the accepted #699
   `slice-a-loaf-holding-it-with-a-claw-grip` convention ("the knife passes through bread"), and the cut is left as a
   geometry-gated rule on the wedger pose. The 6 mm pre-scored kerf is kept so a MuJoCo pre-check that treats the blades
   as colliding also scores the demo 1.0; a collide-aware private copy of that pre-check scored the same 1.0.

Known limits, measured on this revision and stated rather than hidden:
- A MuJoCo robustness probe of our own (not an official score): 1.0 at start-jitter seeds 0-3 and at both friction
  scales, 0.0 at seed 4 (+-5 mm, +-3 deg). The open-loop demo's grip approach is calibrated to the one fixed layout.
- **The one-hand press reaches 1.0 in the MuJoCo stand-in** (an offline one-hand variant of the demo): the stated,
  accepted consequence of the pass-through construct (item 10), not a scoring bug.
  In this tree nothing resists the blades physically: they pass through the 6 mm kerf with 2.4 mm of clearance and
  the seam glue is released by the cut rule, so a single grip carries the 80 g wedger level through the fruit. The
  "one grip rolls the ring" claim in `task.json` is therefore a claim about the real apple, not about this sim.
  Tried and measured: a 45 deg then 75 deg skin lip on each seam face
  at the crown, reaching 0.5 mm into the blade footprint, with `break_force` 1.5-4 N on each glue line and the
  wedges hung 1.2 mm above the deck. Result in the MuJoCo stand-in: the lips do resist (20-46 N on the ring, the
  one-hand press cocks to 33 deg and scores 0.0) but no glue line ever breaks, because the stand-in's bond is a soft
  weld (it sags 0.4 mm under a wedge's own 0.17 N, ~0.4 N/mm), so the load goes into board friction and the two-hand
  demo stalls at 2-3 mm depth (0.0, 18-24 deg). PhysX models a bond as a rigid fixed joint with a real break force, so
  the lip variant is the right thing to try in Isaac; it is kept as an unapplied patch because the demo must reach 1.0
  in the stand-in first.
- The MuJoCo stand-in approximates PhysX; `mechkit` bonds are modelled as soft breakable welds there.
- **Seam release is the #699 convention, adopted deliberately.** PhysX has no cutting, so the cut is a rule: a seam's
  glue is released only when a blade is at that seam line and the wedger has descended level into the flesh. Accepted
  #699 (`slice-a-loaf-holding-it-with-a-claw-grip`) states the same construct in its own fidelity text: "the task file
  deepens a cut along a joint only as the blade is sawn ... and frees the slice once the cut is through". Here the
  equivalent is: a seam gives only as its blade is driven through it, and the fruit falls open once all eight have.
  Consequence stated plainly: the rule reads blade pose, not blade load, so a one-hand press is NOT excluded by this
  sim (it reaches 1.0 in the stand-in); the crown press and the hover score 0 because no blade is at any seam line.

## Nearest neighbour, disclosed
#2720 `core-a-whole-apple-with-a-tube-corer` (OPEN, another team) shares the apple and the coring action. It differs in
the ability: a single handheld tube corer driven along the stem axis with plug retention, versus an eight-blade wedger
held by two side grips and pressed level through the fruit. No shared mechanism, geometry or scoring signal; listed here
so a reviewer can judge the distance.
