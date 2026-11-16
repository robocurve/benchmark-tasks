# Judging

Use a top photo, a low side photo and a millimetre ruler for the broad shape
measurements. Judge the finished icing, not the nozzle path or number of squeezes.
The native demo scores 1.0 and the completed Dream baseline scores 0.0.

Take a top photo and two low oblique close-ups from opposite sides, with a
millimetre ruler alongside the border. Use an ordinary phone camera in focus
and diffuse room light; side lighting may be used to make raised ridges clear.
At full resolution the 90 mm border should occupy at least 900 pixels. Retake
an out-of-focus or occluded view. No projected stripe, feeler gauge, depth
calibration or macro attachment is required.

The marked section is 90 mm long. Its permitted strip extends 9 mm on either
side and 2 mm beyond each end mark. A shell has a head 6–16 mm wide and 2.5–8 mm
high. Within 4–22 mm after that head, its tail must include a section 1 mm or more
wide, at least 0.5 mm high, at most 3 mm high, and no wider or higher than 65% of
the head. Its leading edge to qualifying tail must span at least 1.25 times its head width;
a round mound does not count. A finer point beyond this section is allowed. Head-to-tail motion can vary; shells have no fixed
centres, but distinct heads must be at least 6 mm apart. A low shoulder below
the minimum head height is part of a tail, not another head. Oversized or
over-height mounds remain obstructions; a shell cannot use a tail beyond them. For full credit, each counted shell needs two **separately visible raised
ridges**. A crest and the groove separating it from its neighbour must both
be traceable in the close photographs: two colour lines on a flat surface do
not count. The two crests must be at least 1.5 mm apart and each must continue
for at least 2 mm along the shell, including its tapered shoulder. Use the ruler
in the photo for those lengths; **do not measure groove depth or ridge height**.
A smooth mound or isolated bump fails this check. A ridge hidden in one view
may be judged in the other. If no sharp unoccluded view shows both, the shell
does not meet the full-credit ridge criterion.

Measure head height within 2 mm before
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

`scoring.py` measures connected heads and tails from the final geometry.
Its ridge check finds two crests separated by an actual valley and tracks them
along the shell; the tiny numerical epsilon only rejects a flat plateau and is
not a physical depth requirement. Check agreement against the native close-ups
before reporting full score. A row of smooth blobs, a plain line or painted
texture is not a ridged shell border.

The old calibrated projected-section procedure and 0.3 mm dip criterion are
retired. Historical evidence files retain their original measurements but do
not define the current rubric. The revised demo and Dream audits are in
`evidence/revision-2026-11-16/`.
