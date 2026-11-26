# Design: take one tortilla out of a resealable bag

Category: **Societal relatability.** Taking a tortilla out of its bag is something most people do every week:
press-to-close bags of flour tortillas sit in most fridges and pantries, and a wrap, a quesadilla or a taco starts
with opening the bag and sliding one tortilla out of the stack without dragging the rest out with it.

## The real job

A bag of tortillas lies flat on a cutting board, zip closed, mouth toward the person. They pull the zip's two lips
apart to open it, lift the front film off the stack, slide the top tortilla toward the mouth with their fingertips
until its edge hangs out, take that edge and draw the tortilla out, and lay it flat on a plate. The other tortillas
stay in the bag. What makes it hard for a robot: the bag and the tortillas are thin, floppy sheets; the two lips lie
on each other and must each be taken alone; the zip opens only when the lips are pulled apart near each other (pull
one lip alone and the whole bag front lifts); the top tortilla must be moved without dragging the one under it; and
a floppy tortilla hanging from one edge must be laid down flat, not folded.

## Real-world protocol

| Item | Specification |
|---|---|
| Board | End-grain maple cutting board, 12 x 14 in, 1 1/2 in thick (305 x 356 x 38 mm), its 12 in side toward the arms, near edge 360 mm in front of the line through the arm bases, centred between the arms. |
| Bag | The resealable press-to-close bag of an 8 in flour tortilla pack (clear LDPE, about 0.06 mm), opened once (tear strip removed), with 6 tortillas left in it. Inside 240 mm across the mouth, 285 mm from the zip to the bottom seal; side seals 8 mm, bottom seal 10 mm. The lips above the zip: the front lip 22 mm, the back lip 34 mm (trim the lips with scissors to these if they are longer). The bag lies flat, mouth toward the arms, the zip 20 mm in from the board's near edge, centred; the back lip hangs over the board's edge. |
| Tortillas | 6 shelf-stable 8 in (203 mm) flour tortillas, about 2 mm thick, about 50 g each, at room temperature, stacked flat in the bag, the stack's near edge 15 mm behind the zip. |
| Plate | White ceramic dinner plate, 10.5 in (267 mm), on the counter, centre 330 mm in front of the arm bases and 320 mm to the left of the centre line (left of the board). |
| Zip | Closed along its whole length (press it shut from end to end before each trial). |

**Reset.** Put the tortilla back on top of the stack (replace it if torn), lay the bag flat in its marked place,
press the zip shut end to end, fold the front lip down onto the back lip, and put the plate back on its mark. A pack
serves a day of trials (tortillas are shelf-stable); start each day with a fresh pack.

**Safety.** Nothing sharp, hot or heavy; dry food only (no residue on the pads).

## Scoring (the same in simulation and in a real trial)

Judged at the end of the trial. A tortilla is *out* of the bag where it is past the zip (toward the arms), beside the
bag or above it; *in* otherwise.

| Score | Condition | Real-trial check |
|---|---|---|
| 1.0 | One tortilla all the way out, lying flat on the plate (at least 90 % of it inside the plate's rim, not folded over: its footprint at least 85 % of its area), at rest; the other five still in the bag (no more than a quarter of any of them past the zip); both hands open, holding nothing, at least 10 cm from it | photo of the end state; video for the hands |
| 0.75 | One tortilla all the way out and the others in, but it is not lying flat on the plate (or a hand still holds it) | photo |
| 0.5 | One tortilla at least half out of the bag past the zip, the others in | photo |
| 0.25 | The zip open along at least 150 mm of its 240 mm | ruler along the zip |
| 0 | Anything else; two or more tortillas at least half out at the end scores 0 | photo |

In the real trial, a torn tortilla (a tear longer than 20 mm) scores 0 (the simulation cannot tear).

## Simulation model

- **Sheets.** The bag's two films and the six tortillas are PhysX surface deformables (FEM shells): `clothkit.py`.
  Each film is a 6.4 mm triangle grid (about 2,100 vertices), 12 g, collision thickness 1 mm; the films are welded
  along the side and bottom seals (vertex-vertex attachments). The front film's lip is set curled up, rising 55° from the zip
  (its rest shape's bend angles), as an opened bag's lip is. Each tortilla is a 7.5 mm disc mesh (639
  vertices), 50 g, 2 mm thick; bending stiffness is set so a tortilla held 80 mm out over an edge droops about
  36 mm. Friction: tortilla on tortilla 0.3, film 0.35.
- **Zip.** 37 cells of 6.4 mm. Each cell is a pair of small rigid ribs, one fixed to each film's zip line, held by a
  PhysX fixed joint that breaks at 0.15 N (and 0.0006 N m): pulled apart harder, PhysX breaks it for good and the cell
  is open (`zipkit.py`). The force is low because the films are simulated as light, very flexible sheets; it is set so
  the zip holds through handling and opens only when the two lips are pulled apart near each other, as a real zip
  does. Pulling one lip alone lifts the bag's front and opens nothing.
- **What the pads hold (`gripkit.py`).** PhysX cloth friction scales with the mass of the touched vertices, so a
  rubber pad cannot hold a 50 g tortilla or a film by friction in simulation. Two rules stand in for pad friction,
  in the task code (for any policy, the model as much as the demo), checked every control step from the arms' joint
  positions:
  - *Pinch:* as the pads close, if the gap between them comes down to the thickness of the sheets between them plus
    1.5 mm, the sheet vertices between the pads (15 x 30 mm) are fixed to the finger until the pads open 3 mm more.
    A sheet that runs across the gap at a slant (one of its mesh edges crosses the pads' mid-plane within the pad
    area) is pinched once the closing pads come within its thickness plus 6 mm, as they would trap it there.
  - *Press:* with the fingers closed, if the tips press into a sheet for two control steps, the top sheet's vertices
    under the tips (10 mm around the contact) are fixed to the finger until the tips lift 3 mm or the fingers open.
    Clear film lying on a tortilla is slick: pressing it holds neither.
  The robot still has to put the pads on the sheet: a pinch needs the lower pad under the lip or the hanging edge,
  and a press needs the tips on the tortilla itself, not on the film over it.
- **Rigid bodies.** Board 2.6 kg, plate 0.6 kg (a well and a sloped rim), both loose on the counter.

## The demo (`plan.py`, played by `run_demo.py`)

Each phase is planned when it starts from where the lips and the top tortilla are (a scripted demo may read the
simulation), as fingertip targets solved by IK.

1. The right hand pinches the curled front lip, the left the back lip where it hangs over the board's edge, 25 mm
   either side of the middle, each coming in on a diagonal so the wrists stay apart.
2. They pull the lips apart (front up, back down) and spread them along the zip: the zip peels open.
3. The right hand keeps the front lip and lifts it, holding the front film up off the stack like a tent; the left
   lets go of the back lip.
4. The left hand comes in under the lifted film, closes its fingers, presses its tips on the top tortilla 30 mm in
   from its edge and slides it 85 mm toward the arms, so its edge hangs out over the board's edge; then lifts off.
5. It pinches the hanging edge, draws the tortilla out of the bag, carries it hanging over the plate's far side and
   lays it down while drawing the hand back toward the near side (the tortilla leaves the pads past their tips, so
   it is laid out ahead of the hand without folding), then lets go.
6. The right lays the front film back down and lets go; both arms go back to the start.

## Assets

All geometry is authored in this folder; the textures (`textures/`, from `make_textures.py`) are procedural. No
third-party assets.
