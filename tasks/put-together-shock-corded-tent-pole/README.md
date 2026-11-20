# Put a shock-corded tent pole together

Put the tent pole together: a folded three-section shock-corded aluminium tent pole lies on the table; the robot
unfolds it, lets the cord pull both joints home and leaves it straight on the table. This is the full-size named
retail pole (Six Moon Designs 45 in Ultralight Tent Pole, aluminium, 3-section) with its own cord, not a model of one.

## Scoring (judged at the end of the trial)

Tools: the top-camera video, a photograph, a steel rule and a 1.2 m straight edge (or a taut string). Only the end
result counts, not how the robot moved.

- An insert has **entered** when its tip is inside the next tube's mouth. The **gap** g of a joint is the length of
  insert still showing between the two tube ends, measured with the rule.
- **Score 0** if neither insert has entered (doing nothing scores 0), or if any part of the pole has left the table
  (off its edge or fallen to the floor) at any time.
- Otherwise the score is the **lowest** of these three numbers:

| Item | 1.0 | 0.75 | 0.5 | 0.25 |
|---|---|---|---|---|
| Joints home | both entered, both g <= 1 mm | both entered, both g <= 5 mm, not both <= 1 mm | both entered, a gap over 5 mm | exactly one entered |
| Straight: d = largest sideways distance from the straight edge laid tip to tip to the pole's centre line | d <= 5 mm | - | 5 mm < d <= 20 mm | d > 20 mm, or a joint not entered |
| Let go and still | both grippers' centre points at least 3 cm from the pole and no point of the pole moves more than 3 mm over the last 1 s | - | let go (3 cm clear) but some point moves more than 3 mm in the last 1 s | a gripper within 3 cm of the pole (holding or touching it) |

Every band has exact limits; together they cover every case. A pole held in the gripper scores 0.25 at most;
one joint made scores 0.25; the starting fold scores 0. The trial ends early once 1.0 has held for 1 s.

In the simulation (`score()`): entered = the insert's tip more than 1 mm past the mouth's face and within the bore
(4.55 mm of the tube's axis); g = distance along the receiving tube's axis from the insert's tube end face to the
mouth's face (a seated joint reads 0, up to 2 mm of contact overlap tolerated); d from the six tube end centres; the
3 cm clearance from each gripper's centre point to every tube's surface; stillness from positions over 1 s; off the
table = a tube end outside x -0.19..0.91 m, y -0.65..0.65 m or below the table top.

## Demonstration (scripted, `demo()`)

The scripted demonstration works the way a camper puts a pole together on the ground. Every section stays on the
table. The hands grip and turn the tubes where they lie and never lift one: the cord's pull on a lifted section would
twist it in a two-finger pinch. Only the tubes are touched; the cord is never gripped.

1. **Right joint (0-12 s).** The left hand pinches the middle and left sections together (one pinch across both,
   about 10 N). The right hand comes down over the right section where it now lies (its pose is read as the hand
   arrives, after the cord has settled it) and pinches it 14 cm from its insert's tip. It swings the section about
   168 degrees round through the open table, keeping the insert's end at the middle section's mouth. It then lines
   the insert up with the mouth, correcting each control step by 15% of the measured side and angle error. It takes
   the tip 8 mm in and opens to 20 mm, and the cord draws the section home through the open fingers. The right hand
   closes on the section again where it now lies.
2. **Left joint (12-28 s).** The left hand lets go. It presses its closed fingertips on top of the left and middle
   sections and rolls the left section out about 15 degrees, so that a finger fits between them. It pinches the
   left section 7 cm from its open end and swings it about 165 degrees round the other way. It lines the mouth up
   with the middle section's insert, slides it 8 mm over and opens, and the cord draws the section home. Throughout,
   the right hand holds the joined right and middle sections against the cord's pull.
3. **Release.** Both hands open, rise and move away, and the pole lies straight and still.

**Runs.** Scripted demonstration: 1.0, full score from 28.9 s: both joints home (insert showing 0.0 mm), the pole straight within 0.9 mm, both hands over 3 cm clear and the pole still. Earlier runs with seeded 1 cm shifts (before the last change, which plans against each tube's heading and ignores its roll about its own axis) scored 1.0 in 2 of 3. In the third, the middle section had rolled about 50 degrees on its axis and the left section's mouth stopped on the insert's tip (0.25). Letting go part-way round is not always wrong, as with a real shock-corded pole. Let go about 45 degrees short of line with its mouth at the insert's tip, the left section was swung in by the cord and seated within half a second (1.0). Let go about 95 degrees out of line, it stayed there: one joint, 0.25.

## Setup

Positions in metres from the midpoint between the two arm bases (bases at x 0, y +-0.305), x forward, y to the robot's left; heights above the table top. One Six Moon Designs 45 in Ultralight Tent Pole, aluminium, 3-section (maker: 114 cm assembled, 43 cm folded, 85 g, 3/8 in tube; USD 15): three dark grey anodised aluminium tubes, each 380 mm long, 9.5 mm across with an 8.1 mm bore, 28 g (bore and mass split: estimates). The right end section A and the middle section M each carry a bare aluminium insert 7.9 mm across (estimate) at the same end, showing 50 mm, its last 3.5 mm tapered; M's other end and the left end section B's near end are open. A black shock cord runs inside all three, knotted in the two outer ends (pull 3.0 N assembled, 4.0 N with both joints folded: estimates). Start: folded and laid flat on the bare table, the tubes side by side. M along y at x 0.150, its open end at y -0.190, its insert from y 0.190 to 0.240. B beside M at x 0.160, its open end at y 0.240 against M's insert, its free end at y -0.140. A outermost: its insert's tip at (0.159, -0.190) against M's open end, the tube running at 12 degrees off the y axis to its free end at (0.249, 0.230), just clear of B's free end. Tape marks on the table at the four ends of M and A. Nothing else on the table; x up to 0.60 is kept clear for the swings. Arms at the ready pose, grippers open. Assembled, the pole lies along y near x 0.15 from y -0.57 to 0.57 (the table is 1.30 m wide, y -0.65 to 0.65).

## Product and sources

- Six Moon Designs 45" Ultralight Tent Pole, Aluminum, 3-section, Single, USD 15.00:
  https://www.sixmoondesigns.com/products/45-inch-ultralight-tent-pole (fetched Nov 2026): "45" - 114 cm",
  3-section "folded 17" - 43 cm", aluminum 3-section "3 oz - 85 g", diameter "3/8" - 10 mm". The maker's photo shows
  dark grey (gunmetal) aluminium sections with narrower inserts at their ends.
- Derived: 114 / 3 = 38 cm sections; 43 - 38 = 5 cm of insert showing; a folded length of 43 cm (not 48) means the
  first two sections each carry their insert at the same end of the pole.
- Estimates (not published; how to check one in a minute): tube 9.5 mm across (3/8 in) with an 8.1 mm bore and
  insert 7.9 mm (calipers or a drill-bit gauge); insert tip tapered over its last 3.5 mm (look); 28 g a section
  (kitchen scale: whole pole 85 g); cord pull 3.0 N assembled and 4.0 N with both joints folded (hang the folded pole's
  end section from a luggage scale and pull the joint open: reads about 0.4 kg); aluminium friction 0.35 / 0.30.

## Fidelity

Three rigid 12-sided aluminium tubes at true size and mass; no tube flex. The shock cord is not a body: it is a tension line (8.5 N/m; 3.0 N assembled, 4.0 N folded: estimates) from one outer end through each insert tip and tube mouth to the other, recomputed 240 times a second. It pulls the joints together, has no contact, weight or friction, cannot be gripped and is drawn only where it shows at an open joint. Insert in bore: rigid, 0.2-0.5 mm clearance (estimate).

## Reset (about 20 s)

About 20 s. Pull each joint apart by its tubes against the cord, fold the end sections back beside the middle one and lay the pole on its tape marks: M on its line, B against M with its open end beside M's insert, A outermost with its insert's tip against M's open end and its free end on its mark. Check that the cord is inside the tubes and not frayed at the joints and that no tube or insert is bent; every 50 trials hang the folded pole's end section from a luggage scale and replace the pole if a joint opens under 2 N (cord gone slack). No consumables. Simulation reset restores the same folded layout with the cord at its folded pull.

## Files

- `put_together_shock_corded_tent_pole.py`: the scene, the scorer (`score()`, `success()`) and the scripted demo.
- `corded_pole.py`: the tube sections (convex pieces) and the cord's route through the joints.
- `elastic2_kit.py`: the elastic line (a massless Hooke line recomputed every physics step, applied as a force at
  each end and guide; drawn as pieces without colliders).
- `yam_parts.py`, `yam_mesh.py`, `yam_joints.py`: the harness's compound-part, mesh and joint kits, copied unchanged.
- Every shape is built in these files from boxes and convex polygon meshes; no third-party assets.

## Running

    inspect-robots run --embodiment yam_arena -E task=put_together_shock_corded_tent_pole.py \
        --policy yam_scripted --instruction "Put the tent pole together."

Run limit 1,200 steps (60 s at 20 steps per second); seeded resets shift the whole folded pole by up to 1 cm
(`-E object_jitter=0.01`).
