# Design: empty an opened bag of potting mix into a nursery pot

Category: **Societal relatability** (home gardening and repotting). The same move is everyday work at a nursery
or greenhouse potting bench when small pots are filled by hand.

## The real job

An opened bag of potting mix stands on a potting bench beside an empty 1-gallon nursery pot. A person picks the
bag up with one hand at a front corner near its cut mouth and the other at the opposite front corner, low on the
filled part. They carry the mouth over the pot and turn the bag over. They turn the filled end further over and
shake it to free the mix that hangs up on the film. Then they put the empty bag down. Nothing is weighed in the job
itself, but the pot should end up holding the mix, the bench should stay clean, and the bag should not end up in or
over the pot.

## Real-world protocol (reproducible)

| Item | Specification |
|---|---|
| Bench | Any flat table at the station. A sheet of kraft paper about 90 x 60 cm under the scene catches spills for a clean reset. |
| Bag | A stand-up quad-seal (block-bottom) bag of about 0.1 mm PE or PE laminate. Its front and back panels are 160 mm wide and its side gussets 80 mm deep, with four vertical corner heat seals and a flat bottom. Filled, it bulges to a rounded oval about 190 x 100 mm. Cut the top off square 150 mm above the bottom; the top stays open. The two front corner seals face the arms. |
| Mix | 2.0 L of a peat-based all-purpose potting mix with perlite, measured loose in a 2 L jug. Weigh it and bring it to 0.80 kg (0.78 to 0.82 kg) by misting water in and mixing, or by letting it air-dry. Pour it into the bag loosely without pressing. It fills the bag about 115 mm deep, so it shows about 35 mm below the cut edge. |
| Pot | A black polypropylene 1-gallon (trade size) nursery pot: 165 mm outside diameter at the top, 135 mm at the base, 160 mm tall, about 2.7 L. It stands free on the bench. |
| Layout | Distances are forward from the line between the arm bases and sideways from the midpoint between them. Bag: floor centre 40 cm forward and 12.5 cm left, front panel facing the arms. Pot: base centre 40 cm forward and 13.5 cm right. |
| Scale | Kitchen scale, 1 g resolution. |

**Measuring a trial.**
- Before the trial, weigh the filled bag and the empty pot.
- After the trial, weigh the pot again: its gain is the mix transferred.
- Weigh the bag. That weight minus the empty bag's weight is the mix left in it. Weigh the empty bag once, after
  the first trial.
- The mix spilled is 0.80 kg minus both. Spilled mix includes anything on the bench, the floor, the pot's rim or
  outside, or the robot.
- In the video, check that both grippers end open with neither touching the bag. Check that no part of the bag is
  inside the pot's rim circle (plus 10 mm) less than 20 mm above the rim.

**Reset.** Pour the transferred mix back into the jug together with the spill, and top it up to 0.80 kg with
fresh mix. Refill the bag, or a fresh identical bag, and stand it in place. Empty and wipe the pot.

**Safety.** The arms carry under 1 kg. Damp potting mix is not hazardous and makes little dust. The kraft paper
catches spills.

## Scoring (simulation and real trial alike, by mass)

Fractions are of the 0.80 kg starting mix and are judged at the end of the trial.

| Score | Condition |
|---|---|
| 1.0 | At least 95 % in the pot, at most 3 % spilled, the pot upright, both grippers open (at least 20 mm) and the bag clear of the pot |
| 0.5 | At least 70 % in the pot and at most 8 % spilled, the pot upright |
| 0.25 | At least 30 % in the pot and at most 15 % spilled, the pot upright |
| 0 | Anything else, including the pot tilted 20 degrees or more |

- **Mix in the bag** counts as neither transferred nor spilled. Dropping the whole bag into the pot therefore
  scores zero, because that mix is in the bag, not in the pot.
- **The pot moving** is allowed, as long as it ends upright.
- **In simulation**, every grain has its mass and is placed by its centre:
  - in the bag if it is inside the film, the film's triangle mesh closed by a lid across the mouth;
  - otherwise in the pot if it is inside the pot's wall below its rim, in the pot's own frame;
  - otherwise spilled.

  "Clear of the pot" checks every film vertex and both tabs' corners.

## Simulation model and its simplifications

- **Bag film.** One PhysX surface deformable (thin cloth-like film): a triangle mesh of 2,169 vertices and 4,260
  triangles with 5 to 14 mm edges (`clothbag.py`).
  - Material: stretch stiffness of 0.1 mm polyethylene (E = 200 MPa) and almost no bending stiffness, so it folds
    where it is empty and narrows as it empties.
  - Settings: self-collision on, 64 solver iterations, friction 0.35 against the mix.
  - **Simplification: mass and thickness.** The film weighs 50 g; a real bag of the same area weighs about
    8-12 g. Its contact layer is 3 mm thick. With true film mass, PhysX lets the 0.8 kg of grains push through the
    floor film (a mass-ratio limit).
- **Gripped corners.** Where a hand pinches a front corner seal, the seal is a small rigid tab, 40 x 12 x 30 mm and
  20 g, turned to face forward. PhysX deformable attachments fix it to the 15 to 18 film vertices it encloses.
  - Mouth tab: 25 to 55 mm below the cut edge.
  - Bottom tab: 50 to 85 mm above the floor.

  A hand pinching a bag corner gathers several folds of film. More importantly, the simulated YAM gripper is
  position-controlled at 5,000 N/m per finger, so a thin seal would be squeezed only weakly.
- **Mix.** 2,300 angular rigid grains (convex icosahedra, 9 to 12 mm across), density 925 kg/m^3, 0.80 kg in
  2.0 L bulk (0.40 kg/L, typical for damp peat mix).
  - Real particles are 0.1 to 10 mm, so this is a coarse stand-in.
  - Grain-on-grain friction is 1.0, so the mix stands at about 45 degrees.
  - There is no moisture or cohesion model. Mix that stays in the bag is held only by friction on the shallow
    lower walls of the turned-over bag.
- **Starting fill.** `prepare_bag.py` fills the bag once, as a person fills a bag held open:
  1. The film's cut edge is held open, and a thin rigid sleeve stands inside it.
  2. The grains drop into the sleeve and settle.
  3. The sleeve is drawn out, the edge is let go, and the film settles onto the mix.

  The settled film (`bag_start.npz`) and grains (`mix_start.npz`) are put back at every reset.
- **Pot.** One free rigid body, 70 g: a floor, 24 tapered wall plates and a rolled rim.

## The demo (`bagplan.plan()`)

1. **Grip.** Both hands line up 10 cm in front of the two front corner tabs, hold still, slide on and pinch.
   - The right (mouth) hand approaches level.
   - The left (bottom) hand approaches 15 degrees down.
   - The bag is lifted 22 cm. The filled bottom hangs about 15 cm below the hands and must clear the pot's rim.
2. **Tip.** The mouth travels toward the pot while the bag turns only 30 % of the way over, so no mix leaves yet.
   Over the pot it comes down while the bag turns the rest of the way, to 118 degrees about the forward axis. That
   is the most the mouth hand's wrist roll allows (120 degrees). The mouth's lip ends 45 mm above the rim and 60 mm
   toward the bag's side of the pot's axis, because the mix leaves the lip moving away from the bag.
3. **Turn and shake.** The bottom hand turns the filled end further over, to 140 degrees.
   - On the way it swings outward so the two grippers stay at least 3 cm apart.
   - The film between the hands slackens and folds.
   - The limp bag pivots as its bottom turns over and would carry its mouth about 5 cm off the pot's axis, so
     both hands move together 4.5 cm forward and 2.5 cm toward the pot's far side while the bottom is turned
     over.
   - The hands never move further apart than 128 mm; a taut film would pull the tab out of the hand.
   - The bottom hand then shakes 22 times: it swings 5 cm up across the line between the hands and jerks back.
4. **Put the bag down.** The bag turns back upright, is carried over the bench beside the pot and stood down, and
   both hands let go and back off.

The mouth hand's IK targets are exact. The bottom hand's are weighted, position first, because it holds limp film.
The policy plays the joint targets and corrects for the arms' sag from the observed joint positions.

## Assets

NVIDIA SimReady and YCB have no nursery pot or soil bag, and the bag has to be a deformable film anyway. Every
object is authored procedurally in this folder.
