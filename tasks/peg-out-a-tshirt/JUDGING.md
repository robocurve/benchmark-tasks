# Judging the hanging T-shirt

Use the final five seconds of video and close-up photographs of each jaw. A
ruler or printed scale supplies distances. Inspect the actual fabric captured
between each peg's jaws together with the line; a peg nearby, closed in air or
resting on top of the fabric does not count.

- **0:** no valid retained fastening; the robot, laundry support or another object supports the
  shirt; any part is within 20 mm of the table or floor; or integrity fails.
- **0.5:** one valid peg retains the shirt, supported only by the line and pegs for five
  seconds, with at least 20 mm clearance. Two valid pegs less than 100 mm apart
  receive the same partial credit.
- **1:** both pegs capture the line and separate portions of the garment,
  their centres are at least 100 mm apart, and the shirt remains supported only by
  the line and pegs with at least 20 mm clearance for five seconds.

All score levels require an intact shirt, line and pegs. Torn cloth, broken
pegs, undisclosed support, visible cloth passing through itself or a solid, and
any unsupported geometry manipulation invalidate the run. If damage or contact
integrity cannot be assessed, leave the result unscored until reviewed. A
native strain diagnostic alone cannot certify the absence of tears.

The run ends at its declared action limit; earlier temporary success does not
override a dropped shirt. The hold must occur within the limit. A valid peg may capture the hem or another intact fabric region. Judge the
outcome rather than an arm-by-arm procedure: either arm and any physically
valid order of operations may be used.

`scoring.score(record)` consumes the reviewed values. Its evidence digest map
must include the released video and both jaw photographs. Synthetic records in
unit tests are test fixtures only. None is a demonstration result.

The disclosed ideal static-grip constraints represent actual material enclosed
by fingers or peg jaws. They do not establish a pass: verify the visible jaw
capture, released hold and clearance independently.

Incidental brushing against a vertical rack post is allowed. The garment may not
rest on a post top, foot, laundry board, table or robot. The two pegs must still
independently retain fabric and line; post contact never substitutes for a peg.
