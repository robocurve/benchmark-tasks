# Revision — complete specimen and sustained-opening grading

This revision addresses the requested complete carton blank, assembly and
pre-creasing procedure, repeatable fresh specimens and load preparation, and
the question about measuring 1 mm / 1° stillness on obscured panels.

## Complete specimen and repeat preparation

The task includes full-size A3 and tiled A4 PDFs, a vector SVG, all panel/tab
coordinates, cut/crease/glue markings, assembly views, a closed-to-start pre-fold
sequence, material acceptance checks and a secured dry-ballast recipe. Four
fin relief slits match the simulated unconnected edges; the central pouring
crease stays intact. This is a specified custom carton with a 24 mm inner lip,
not a claim that an unspecified commercial carton has those dimensions.

Digital checks cover nine moving facets, ten stationary sheet panels, all
twelve joined crease edges, one connected nonoverlapping flat net, the initial
vertices and 302 rigid-fold configurations. Geometry discrepancies are below
1e-10 mm, which is numerical consistency, not manufacturing accuracy. Tabs
are fabrication features, not extra manipulation aids.

Mass is clarified without altering physics: 520 g is stationary body/contents;
moving paper adds 2.701032 g. The whole specimen target is 522.7 ± 0.5 g, with
batch paper mass recorded and stationary contribution checked at 520.0 ± 0.7 g.

PHYSICAL_SETUP.md separates batch cutting, assembly, adhesive setting and
weighing from per-trial selection, one outer-fold spread, start checks and
placement. Each scored physical trial uses fresh paper from matched prepared
stock. Used or repeatedly softened heads are rejected. The few-minute turnover
is an intended procedure, not a measured claim.

## Practical outcome scoring

The submitted all-panel 1 mm / 1° stillness rule is removed. The final rubric,
`sustained-opening-v3`, has **no micro-motion or panel-angle test**. Full credit
requires a mouth at least 30 mm wide, both apex and upper lip at least 8 mm
beyond the front wall, upper lip at least 35 mm above the shoulder, and body
tilt at most 5°, continuously while released and table-supported for the final
three seconds. Settling that preserves these functional conditions is allowed.
A loss of opening, release, support or uprightness restarts the hold; later
closure loses full credit. Partial scores and permanent integrity/data guards
remain explicit.

A ruler, square and short front/side/overall recording can check the specified
outcome. GRADING_PROTOCOL.md names visible material landmarks and explains
occlusion and uncertain threshold measurements. It no longer requires temporal
edge tracking, calibrated high-resolution multi-view overlays, or hidden-panel
orientation reconstruction. Independent grading views are not policy inputs.
The simulator's remaining 1 mm joint-separation guard detects a failed ideal
crease; it is not a real-paper motion tolerance.

## Evidence preserved and regraded

- 37 CPU checks pass. They include incomplete hold, re-grasp and support loss,
  opening below threshold then recovery, closure after success, tipping,
  overfolding, detached joins, malformed data, and malformed/disconnected blanks.
  Usable-opening motion and irrelevant outer-fin motion are explicitly allowed.
- The complete original full-score record (14,520 physics samples / 1,200
  actions) regrades to 1.0. Its first full-score sample remains 10,130.
- The complete original Dream record (28,920 samples / 2,400 actions) regrades
  to 0.0. No additional Dream inference was performed.
- The no-action 120-second stability record and failed-opening/contact probe
  remain 0.0 under the revised rubric.
- Geometry, crease dynamics, instruction, controller, robot, cameras, timing,
  raw evidence and published media are unchanged. The original videos are
  retained episodes, not newly executed runs of the revised scorer.
- README now lists procedural scene assets and inherited I2RT robot asset
  provenance; the complete MIT asset notice is included separately.

The existing simulator validates physical contact manipulation of the rigid-fold
surrogate, not measured paper forces. Self-contact, buckling, tearing, glue and
liquid behavior remain simplified. The complete blank and preparation procedure
are **digitally verified only**: no physical specimen, fabrication yield, timed
reset or real-world grading trial has been tested. Those boundaries are stated
for review; they are not presented as completed physical validation.
