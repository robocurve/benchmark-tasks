# Fold a printed letter in thirds and put it in a #10 window envelope with the address in the window

Instruction: `Put the letter in the envelope so the address shows in the window.`

A desk. A printed one-page letter lies face down, the way it came out of the printer, its top edge away from the
robot. Beyond it lies an empty #10 window envelope, front up, its flap open toward the robot. Folding a letter into
a window envelope is how bills are paid by check, forms are returned and printed letters are mailed, at home and in
every office.

The instruction says only where the letter goes and that the address must show. The robot has to work out the rest:

- **The fold.** A 27.9 cm sheet goes into a 10.5 cm envelope only folded in thirds (in half it is still 14 cm tall).
- **The fold order.** The address is printed on the top third. If the bottom third is folded up first and the top
  third brought down over it, the top third ends up on the outside, print facing out. Fold the top third first and
  the address ends up inside the packet: the letter fits, but the window shows the blank back of the bottom third.
  Since the sheet lies face down, the address panel cannot be seen until the second fold turns it over.
- **Which way in.** The address sits near the top crease of the packet. Slid in that crease first, as the packet
  naturally lies after folding, the address lands behind the window, which is near the envelope's bottom edge.
- **Two hands.** Each third is lifted by both corners and carried over in an arc so it lands square on the fold line;
  lifted by one corner, it twists and the folded packet comes out wider than the envelope or skewed. The packet is
  then pinched by both side edges and slid through the 24.1 cm mouth level, and pushed flush.

## Scene

Positions in cm from the desk point below the midpoint between the arm bases (x forward, y left), heights above the
desk.

| object | type | size | material (real part) | position |
|---|---|---|---|---|
| letter | paper sheet | 21.6 x 27.9 (US letter, 8.5 x 11 in), 0.1 mm | 80 g/m2 (20 lb) white copy paper, laser printed on one side: blue letterhead at the top, a three-line address block 6-7.3 cm below the top edge and 2.5-10 cm from the left edge, body text below | face down, long side along x, bottom edge at x 14, top edge at x 41.9, centred on y 0 |
| envelope | fixed (taped down at its bottom corners) | 10.5 x 24.1 (#10, 4-1/8 x 9-1/2 in) | white wove #10 window envelope, tinted security lining, flap open; window 2.9 x 11.4 cm (1-1/8 x 4-1/2 in) placed 2.2 cm (7/8 in) from the left edge and 1.3 cm (1/2 in) from the bottom edge | front up, long side along y, centred on y 0; mouth (top edge) along x 44.5 facing the robot, bottom edge at x 55; flap (4.5 cm deep) lying open flat on the desk in front of the mouth, its edge under the letter's top edge |

Sizes: US letter is 8.5 x 11 in; the #10 envelope is 4-1/8 x 9-1/2 in, and its standard window is 1-1/8 x 4-1/2 in,
7/8 in from the left and 1/2 in from the bottom edge (the layout window-envelope letter templates are printed to).
The address block on the letter is placed for that window when the letter is folded in thirds, bottom third first.

Reach: the near edge of the letter is 14 cm in front of the arm bases; the furthest grip is at the mouth, 44.5 cm out. All
grips are straight down at desk height.

## Reset

Unfold the letter, flatten it (a fresh print is easiest) and lay it face down, top edge away, at the marked
position; slide the old letter out of the envelope and leave the flap open on the desk. The envelope stays taped.

## Scoring (real trial)

From a photo of the envelope front and a ruler held at the mouth, taken at the end, after the grippers have been
open and at least 3 cm from the paper for 2 s.

Score 0 if the letter is torn, a gripper still holds it, or the grippers are not both open (2 cm gap or more) and
3 cm clear of the paper for the last 2 s. Otherwise the score is the highest tier below whose conditions all hold
through the whole of those final 2 s:

- 0.25: the letter is folded in thirds: exactly two creases running across its length, each within 1.5 cm of a
  one-third line (9.3 cm from each end), no other fold, crumple or fold across its width, and the packet is at most
  10.5 x 24.1 cm (checked by unfolding the letter after the photo and measuring the creases);
- 0.5: also inside: no part more than 2 mm out of the mouth or past the side or bottom edges, and no part lying on
  top of the front panel (paper bunched at the mouth's own edge, within 3 mm of it, still counts as in the mouth);
- 0.75: also at least half of the address block shows in the window: inside the window outline, print facing out,
  and no paper over it;
- 1.0: also all of it does.

The sim checks the address block at 32 points (4 rows of 8), the fold from the paper's centre line along its length
(it must run out, back and out again, turning within 1.5 cm of each one-third line, and stay 20 cm or more wide; a ripple that runs back less than 1 cm is not counted as a fold), and
the paper's whole surface for the other rules.

Every outcome is covered: doing nothing scores 0 (not folded); crumpling it small, folding it in half or in quarters, or folding it one-handed (the third twists and lands
skewed) scores 0; folding without inserting scores 0.25; folding
the top third first and inserting it scores 0.5 (blank paper in the window); leaving the packet sticking out of the
mouth, or still holding it, scores 0.25 or 0.

## Simulation notes

The letter is a reduced position-based sheet stepped with the rigid simulation, not a full cloth solver. A jaw grips
it only when its pads close to 1.2 mm (sheet thickness + 1 mm of pad give) across a paper edge inside their footprint;
a jaw that is already shut presses or pushes but picks nothing up. Paper lying
directly on the desk holds there by friction while the letter is being folded (two pinches); after that, a pinched,
folded packet slides freely over the desk and inside the envelope. The envelope is rigid and fixed, with 5 mm of
room inside; its front panel is paper, and within 3 cm of the mouth it yields, so pushed paper and fingertips slide
under it rather than over it. Paper
does not collide with itself: each new fold lies on top of the earlier ones in fold order, which is also how the sim
decides whether paper covers the address. The paper pushes nothing back on the fingers. Both simplifications fit
this task: 80 g paper is far too light to resist the arms, and every fold here lays one flat panel over another.

## Requirements

yam-isaac-lab-arena with Isaac Lab-Arena 0.3.1 (the official YAM launchable). `letter_sheet.py` (the paper sheet,
needs `warp`) and `letter_look.py` (its print and the envelope's look) are in this folder.
