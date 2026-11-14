# Judging

Use a top photo, a low side photo and a millimetre ruler for the broad shape
measurements. Judge the finished icing, not the nozzle path or number of squeezes.
The native demo scores 1.0 and the completed Dream baseline scores 0.0.

For ridge depth, use the calibrated projected-section procedure below. This is
an explicit judging procedure for a physical trial; it has not been tested on
real icing in this project. Native surface profiles independently verify the
simulation result. An unresolved photograph must be retaken before assigning
credit for its ridge depth.

The marked section is 90 mm long. Its permitted strip extends 9 mm on either
side and 2 mm beyond each end mark. A shell has a head 6–16 mm wide and 2.5–8 mm
high. Within 4–22 mm after that head, its tail must include a section 1 mm or more
wide, at least 0.5 mm high, at most 3 mm high, and no wider or higher than 65% of
the head. Its leading edge to qualifying tail must span at least 1.25 times its head width;
a round mound does not count. A finer point beyond this section is allowed. Head-to-tail motion can vary; shells have no fixed
centres, but distinct heads must be at least 6 mm apart. A low shoulder below
the minimum head height is part of a tail, not another head. Oversized or
over-height mounds remain obstructions; a shell cannot use a tail beyond them. For full credit, each counted shell needs at least two ridges at least
1 mm high, separated by 1.5 mm or more, with a dip of at least 0.3 mm between
them. Both ridges must continue for at least 2 mm on the shell, including its
tapered shoulder; an isolated bump does not count. Top and close side photographs show their length and separation. The saved
surface profiles measure depth; use the calibrated photographs below for a physical trial. Measure head height within 2 mm before
to 4 mm after its widest point, where width is at least 75% of the maximum.

Measure the total marked length occupied by valid head-and-tail shapes:

- Below 20 mm: 0.
- At least 20 mm: 0.25.
- At least 40 mm: 0.5.
- At least 60 mm: 0.75.
- At least 80 mm, at least five valid ridged shells, no gap over 2 mm, and the
  dispenser released clear of the border: 1.0. For the final second, the
  dispenser must remain at least 5 mm from every robot part and 20 mm from
  the decorated strip, moving no more than 2 mm.

Score zero if the trial is incomplete, the metal tool gouges more than 1 mm into
the cake, or visible icing at least 0.5 mm thick covers over 20 mm² outside the
permitted strip. Lost material outside the recorded scene makes evidence
incomplete. Unmodeled robot/icing interpenetration also invalidates evidence.
The simple footprint test replaces an earlier, unsuitable spilled-volume rule.

`scoring.py` computes a draft analogue from the final surface. It finds actual
peaks and tapered tails rather than requiring six exact bins. It defaults to
incomplete evidence and cannot certify completion from a material file alone.
Negative shape controls and native surface replay pass. Use the matching close-up photographs and measured profiles to inspect exact
ridge dimensions. A repeated
row of smooth blobs, a plain line or a drawn texture is not a shell border.

## Calibrated depth photographs

Use a fixed macro camera (a phone with a macro attachment is sufficient if it
passes calibration), a thin projected line, a millimetre reference frame and a
0.30 mm feeler-gauge step. The light line lies in a vertical plane across the
shell. Four marks outside the icing, in that same plane, are at (−10,0), (10,0),
(10,10), (−10,10) mm; zero height is the cake surface. Photograph obliquely so
both ridge crests and their intervening valley are visible. Perspective-correct
the four marks to their known rectangle in an image editor, then measure the
illuminated cross-section in millimetres. No hidden surface is inferred.

Recalibrate after any camera, focus, reference-frame or light-plane movement.
Before measuring icing, photograph the independent 0.30 mm step three times
with unchanged camera, focus and light-plane settings, covering the relevant
image positions and height range. Do not use the step to fit the coordinate map.
Each recovered step must be 0.30 ± 0.05 mm; the largest and smallest readings
must differ by no more than 0.05 mm. These are required evidence-quality checks,
not a claim of measured real-world accuracy. If they fail, improve the view,
lighting or magnification and repeat calibration.

Reject clipped or saturated stripes, hidden valleys, occluded crests and ambiguous
pixel correspondences. Include the stripe half-width and annotation uncertainty
in each height measurement. If the resulting depth interval straddles 0.3 mm,
retake the photograph; do not count that ridge as resolved. Retain top-view
photographs showing both ridges continuing for at least 2 mm, with transverse
profiles at several positions spanning that length. A single cross-section does
not establish continuity.

`photo_profile.py` optionally performs the coordinate arithmetic from manually
annotated visible points. It accepts a JSON object with:

- `reference_pixels`: the four `[u,v]` image coordinates in the order above.
- `calibration_steps_pixels`: three pairs of step endpoints, using the unchanged
  camera and reference frame.
- `pixel_uncertainty`: annotation half-width in pixels, including half the stripe
  width and at least 0.5 pixel.
- `mapping_uncertainty_mm`: a conservative additional height-error allowance
  for reference placement/annotation, calibration-step annotation and optical
  distortion, supported by the reference photographs; at least 0.05 mm. This
  is an explicit measurement assumption, not accuracy certified by the helper.
  If it cannot be supported, the result remains unresolved.
- `profiles`: objects with `name` and three `pixels` pairs in crest/valley/crest
  order.

```bash
python tasks/pipe-a-shell-border/photo_profile.py annotations.json --out profile-measurements.json
```

The helper reports reference-step errors, spread, heights, separation and depth
intervals conditional on that mapping allowance. It never fills occlusions or assigns a task score. The same arithmetic
can be done with a perspective-corrected photograph and ruler in an image editor.
Its tests use synthetic perspective geometry, not real-camera evidence.
