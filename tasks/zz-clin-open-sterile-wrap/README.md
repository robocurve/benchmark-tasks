# Open a wrapped sterile pack: far flap first, then each side, near flap last, by the edges only

Instruction: `Open the sterile pack by its border: open the far flap first, each side flap with its same-side hand, then the near flap, without touching or passing over the exposed sterile field; release the wrap and move both hands clear.`

## Who does this

- Registered nurses (BLS OEWS May 2025: 3,379,720) and licensed practical nurses (648,410) open wrapped sterile
  packs and kits at the bedside for dressing changes, urinary catheter insertion and other sterile procedures.
  "Opening a sterile package" is a standard checked skill in nursing fundamentals (OpenStax Fundamentals of Nursing
  10.3; Lecturio "How to open a sterile package").
- Medical assistants (833,900) set up wrapped trays for minor office procedures; surgical technologists open wrapped
  instrument sets for every case.

## The rule (what the robot must do)

The outside of the pack and the outer 2.5 cm (1 in) border of the wrap are not sterile; the inside face of the wrap
and the contents are (OpenStax 10.3). The pack lies with its last-folded flap on top. The nurse:

1. opens the top (far) flap first, away from herself, reaching round the pack rather than over it;
2. opens each side flap with the hand on that side, out to that side. The two side flaps may be opened in either
   order (in practice the one lying on top comes first; here that is the right flap);
3. opens the near flap toward herself last;
4. touches the wrap only on its outside or its border, never the tray, lets each flap lie flat so it does not spring
   back onto the contents, and never passes a hand over the opened field.

## What makes it hard (visible in the motion)

- Each flap's tab must be pinched at its tip and carried in a wide arc up, over and out so the flap unfolds and lies
  flat on the table, without dragging the plies beneath it. Released too early or too high, a flap falls back onto
  the tray.
- The far flap must be laid out past the tray, about 60 cm in front of the robot, near the edge of the arm's reach,
  and the hand must then leave sideways round the flap, not back over it.
- Left flap with the left hand, right with the right, each drawn outward; then the near flap toward the robot, with
  the hand finishing at the tip and leaving to the side.

## Scene (station table = the bedside tray table; cm from the arm-base midpoint, x forward, y left)

Arm bases at (0, +-30.5), 75 cm above the floor; table top 74.75 cm. All heights below are above the table top.
The left arm is at positive y and the right arm is at negative y.

| Item | Product | Size | Place |
|---|---|---|---|
| Wrap | Halyard H300 sequential sterilization wrap, one sheet (cat. 10818) | 45 x 45 cm (18 x 18 in), blue SMS polypropylene | diamond, centre under the tray, corners pointing at the robot, away, left, right |
| Tray | empty stainless instrument-tray surrogate: a smooth rectangular box with flat top and straight sides, no rim | 15 (x) x 20 (y) x 4 cm, 0.35 kg empty | centre (30, 0), resting on the wrap |

### Fold and reset template (the envelope fold, as a nurse wraps it)

With the sheet as a diamond and the tray centred on it (long side across), each corner goes up its tray face and
over the top; where its tip runs past the crease line, the tip is folded back on itself into a tab:

| Order | Flap | Crease of the tab | Tab length | Tab tip ends at |
|---|---|---|---|---|
| 1 | near corner, over toward the far side | x 36 (1.5 cm inside the far edge) | 6 cm | about (30, 0) |
| 2 | left corner, over to the right | y -3 | 3.5 cm | about (30, 0) |
| 3 | right corner, over to the left, on the left flap | y +3 | 3 cm | about (30, 0) |
| 4 | far corner, over toward the robot, on top | x 24 (1.5 cm inside the near edge) | 4.5 cm | about (28, 0) |

So all four tabs end near the middle of the pack top, 4.5-5.5 cm above the table, layered near (bottom), left,
right, far (top), and the two corners' "dog ears" at each tray corner are tucked in by the next flap. Reset: unfold,
smooth the sheet flat, and re-wrap in the same order with the same creases (a card template with the four crease
lines marked makes this repeatable). For each real trial use a fresh H300 sheet so prior use does not change its
creases; the simulated reset restores the same folded sheet state and crease memory.

To make the physical folding card, mark the tray's 15 x 20 cm footprint at (30, 0) on a flat card, draw the x = 24
and x = 36 cm tab-crease lines and the y = -3 and y = +3 cm tab-crease lines, then transfer the four tab lengths
from the table above onto the wrap. The card is a reset aid and is removed before the trial.

## Scoring (from the trial video and sim state)

- Score 0 if a gripper touches the sterile field: the inner part (more than 2.5 cm in from the wrap's edge) of any
  flap that has been lifted or lies open, or the bare tray (with no ply of the wrap between). While a hand holds a
  flap, its tab (the tip folded back, 3-6 cm) is the handle; touching the outside of the pack while it is still
  folded is allowed (the outside is not sterile).
- Score 0 if any part of either robot passes vertically over the field: gripper geometry and finger pads within
  1.2 cm horizontally, arm links (sampled every 2 cm along the arm) within 4.2 cm. The field here is every opened,
  let-go flap's inner part and any part of the tray top that no ply of the wrap covers any more (checked on a 9 x 9
  grid of points, from the first lift on). Once a flap has lain open and been released, its inner part remains field
  even if it moves later. A flap still held by the hand laying it down is not yet field.
- Score 0 if a side flap is pinched by the hand of the other side, a flap opens out of order (the far flap first,
  the near flap after both side flaps), or the tray is pushed more than 3 cm. Order is recorded when a flap first
  meets the open criteria below.
- Otherwise 0.2 for each flap lying open: its tip on the table (under 3 cm up) at least 5 cm outside the tray, and no
  more than 3% of the flap over the tray (more than 1 cm in from the tray's edges; the fold line itself runs along the
  tray's bottom edge). In an overhead photo, trace each flap from its crease to its corner and compare its area inside
  the inset tray outline with its total area; an overlay with equal-area squares can show the 3% limit.
- 1.0 when all four flaps lie open, nothing of the wrap is left on the tray top, both jaws are open with nothing
  held, and both hands are at least 10 cm from the tray and the wrap and still (under 2 cm/s) for 0.5 s.
  Full-surface flatness and rebound are not separately scored; the tip and overlap limits above define an open flap.

For a real trial, record synchronized overhead and side views throughout. Use the overhead view to trace flap and
tray outlines, measure the tip and hand clearances, and spot a vertical projection over the field; use the side view
to check that the robot is above the field, the tip height, jaw opening, and the final still hold. A scale in the scene
sets the distances; review the full motion for the contact and order rules.

## Sim model and fidelity

- Tray: one rigid box (free, 0.35 kg). The instruments inside are not modelled.
- Wrap: a reduced position-based sheet (1.5 cm lattice, 961 points), tension-only stretch and shear (nonwoven SMS),
  weak bending with crease memory that eases over about 1.5 s (a creased wrap holds its folds, and an opened flap
  relaxes flat), ply-on-ply contact and friction, friction on the table and tray, uncalibrated. Rendered as a mesh
  every control step. Tearing is not modeled.
- A pinch is contact-triggered: cloth nodes under the closed pads are pinned to the pads; released on opening
  or if the cloth is pulled off the pads. Only border nodes of the top ply in the pad contact patch are pinned. Gripper parts push the cloth and
  carry it by friction.
- "Open", "over the field" and "touch" are computed geometrically from the sheet points and the gripper geometry.

## Demo

A scripted demo plans from the simulator state (tab positions) and drives the arms joint by joint. Left hand: pinches
the far flap's tab, carries it up, over and out, lays the tip on the table past the tray, lets go and leaves outward
past the tip and round the side. Right hand: pinches the right flap's tab, draws it back over its crease so the tab
unfolds, then carries the flap out to the right and lays the tip down. Left hand: the same for the left flap. Right
hand: comes in from the robot's side (round the opened right flap, not over it), stands the near tab up, and draws
the near flap toward the robot. Each hand lays its tip on the table and leaves outward, never back over the flap;
both arms then park clear. Scores: see task.json.

## Sources

- OpenStax Fundamentals of Nursing, 10.3 Sterile Technique (1 in border unsterile; top flap toward you; open away
  first, then sides, then the near flap): https://openstax.org/books/fundamentals-nursing/pages/10-3-sterile-technique
- Lecturio, How to open a sterile package (reach around, not over; first flap away; nearest flap last):
  https://articles.lecturio.com/nursing/free-cheat-sheet/how-to-open-a-sterile-package/
- Halyard H300 sequential sterilization wrap, 18 x 18 in (45 x 45 cm), cat. 10818:
  https://www.thomassci.com/p/sequential-h300-sterilization-wrap
- BLS OEWS May 2025 national estimates: https://www.bls.gov/oes/
