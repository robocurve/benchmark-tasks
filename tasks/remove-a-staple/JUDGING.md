# Judging

Photograph both sides of both fresh sheets before and after the trial. Identify
the original staple holes; inspect with a ruler and, if helpful, a magnifier.

- 0.5: at least one whole staple leg is clear of both sheets.
- 1.0: both legs are clear, the intact staple is at least 10 mm from either
  sheet, and staple and papers remain released for three seconds. The staple
  may rest on a remover that has also been released onto the work surface.
- 0: a new tear extends over 2 mm beyond an original hole; any new cut or
  puncture appears elsewhere; the staple breaks; sheets are altered; evidence
  is missing; or no qualifying outcome occurs. Creases alone are not tears.

Fill the `staple-judging-v4` record and run `scoring.py`. In this simulation,
both faces are checked through the recorded native sheet geometry and damage
state, alongside unobstructed top/side photos. The ideal wire law is declared
in README; `ordinary_contact_only` means no external edits or assisted robot
motion beyond that shared material model. Mesh-hinge residuals are recorded
separately and are not fibre-tear measurements.
