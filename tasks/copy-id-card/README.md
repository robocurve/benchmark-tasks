# Copy an ID card on a home printer-scanner

A home all-in-one printer stands on the table, its scanner lid shut, and an ID card lies face up on top of the lid.
The instruction asks for one black-and-white copy of the card's face. That takes five things in the right order, each
a different kind of contact:
1. Take the card off the lid. Lifting the lid with the card on it slides the card off.
2. Lift the lid by its front edge.
3. Turn the card over in the hand.
4. Lay it on the glass and push it against the front and right guides at the marked corner.
5. Close the lid without knocking the card, and press Black Copy.

A copy pressed with the card face up, out of the corner, or with the lid open is a wasted sheet. Both hands are
needed: one holds the lid up while the other places the card.

## The real step
HP's DeskJet user guide (2700 series, "Load an original on the scanner glass" and "Copy from the scanner glass"):
1. Lift the scanner lid.
2. Load the original print-side down on the scanner glass, aligned to the indicated corner and the engraved guides
   along the edges of the glass.
3. Close the scanner lid.
4. Press the Black Copy or Color Copy button.

Done by: nearly anyone with a home printer, and office workers. What goes wrong: a copy of the card's back (card face
up), a skewed or clipped copy (card off the corner), or a wasted sheet (Copy with the lid open). The printed copy, or a
photo of the glass, shows it.

## The printer, the card, and what was simplified
| Size | Value used | Source |
|---|---|---|
| printer body | 42.5 x 30.4 x 15.4 cm, 3.42 kg | HP's spec pages for the DeskJet 2810e and 2742e, the same body as the 2855e on sale (retailers list 14.9-15.3 cm and 3.42-3.97 kg) |
| scan area | 21.6 x 29.7 cm | HP DeskJet user guide, Specifications |
| lid | 31.9 x 35.3 x 1.4 cm, 0.45 kg, overhanging the front by 1.5 cm, hinged 0.7 cm from the back, opening to 95 degrees | **estimate**: to be measured on a printer |
| glass | 1.5 mm below the frame, whose edges are the guides | **estimate** |
| buttons | Black Copy and Color Copy, 1.2 cm square, 2.5 mm travel, to the left of the lid | **estimate**: position and travel to be measured |
| card | 8.56 x 5.40 x 0.076 cm (ISO/IEC 7810 ID-1), 5 g | size: the standard; mass: **estimate** |

How the sim models it:
- The printer is a fixed articulation, built with `mechanism.py` (MIT, from robocurve#223):
  - fixed boxes for the body, the frame around the glass, the glass and the control panel;
  - the lid on a revolute joint with light damping, and no spring or friction hold, so it falls if let go;
  - each button on a sprung prismatic joint.
- The card is one loose rigid box at its true size and mass. Its photo is drawn on its face but has no collider.
- The glass's material combines friction with "min", so the card slides on it at 0.3 / 0.25. Every loose object in
  the task-file format gets 1.2 / 1.0, which PhysX's default averaging would turn into a grippy 0.75.

What we simplified:
- The lid, hinge, glass recess and buttons are estimated until a printer is measured.
- The lid has no foam pad and doesn't touch the card when shut (it closes 0.7 mm above the card's top).
- Nothing is scanned or printed: a press is scored from the scene at that moment.
- The printer's other parts (paper trays, output tray, cartridge door) are left out.

## Setup
Positions in cm from the midpoint between the arm bases: x forward, y to the robot's left, heights above the table.

| Object | Description | Position |
|---|---|---|
| printer | white all-in-one, front towards the arms; fixed | front face at x 22, centred on y 0 |
| scanner lid | light grey, hinged at the back | covers x 20.5-52.4, y -21.25 to 14 |
| glass | dark | scan area x 24.5-46.1, y -18 to 11.7: front guide at x 24.5, right guide at y -18; orange 8 mm mark at the corner |
| Black Copy, Color Copy | black and cyan, 1.2 cm square | (24.5, 16.5) and (24.5, 19), on the panel left of the lid |
| ID card | light blue, a dark photo on its face; loose | face up on the lid, long side along y, centred at (20.7, -11), its front 2.5 cm over the lid's front edge |

Both grippers start open at the ready pose. One fixed layout: `-E object_jitter=0`. For real trials: the printer on
with paper loaded, the lid shut, the card placed as in the setup photo.

## Scoring
Every press of either Copy button is recorded. The copy asked for is a **Black Copy** press made when all of these
hold:
- the lid shut, within 2 degrees;
- the card face down, within 10 degrees, lying on the glass with all four corners inside the scan area;
- its front and right edges within 4 mm of the guides, and square to them within 3 degrees.

The card can lie either way round, long side along or across.

**One copy only.** Full credit needs exactly one press in the whole trial, and it must be that Black Copy. **A wrong or
extra copy scores 0**, whatever happens after it:
- any Color Copy press (a colour copy wasn't asked for, and spends colour ink);
- a Black Copy press with the card out of place or the lid open (a wasted sheet);
- a second press of either button (an extra sheet).

**Hands off** means no point along either gripper's finger pads, and neither fingertip's end, within 2 cm of the card.

| Score | When |
|---|---|
| 1.0 | exactly one copy made, the Black Copy asked for, and the card still placed so, both hands off, everything still; held for 1 s, which ends the trial |
| 0.75 | the card placed so under the shut lid, but no copy yet, or a hand still near the card |
| 0.5 | the card face down inside the scan area, not aligned, or the lid not shut; no copy yet |
| 0.25 | the lid opened past 60 degrees and the card on the glass face up; no copy yet |
| 0 | anything else, and any wrong or extra copy (a Color Copy press, a Black Copy press with the card out of place or the lid open, or a second copy) |

In a real trial, the printer's output tray shows how many sheets came out and whether each is black-and-white and
good; a ruler on the glass gives the edges.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder. `mechanism.py` must sit beside the task file, which
imports it:

```bash
INSTR="Make one black-and-white copy of the ID card's face: take the card off the printer's lid, lift the lid, lay the card face down on the glass against the front and right guides at the marked corner, close the lid, and press Black Copy."
COMMON=(--embodiment yam_arena -E task=copy_id_card.py -E object_jitter=0 -E control_hz=20 -E cam_width=320
        -E cam_height=176 --instruction "$INSTR" --scorer task_score --max-steps 1800 --store-frames)

# Dream-1
inspect-robots run "${COMMON[@]}" --policy dream --log-dir ~/yam-eval/copy-id-card-dream

# The scripted demo, which reaches full score
inspect-robots run "${COMMON[@]}" --policy yam_scripted --log-dir ~/yam-eval/copy-id-card-demo

# A wrong way: the card laid face up (not turned over), then copied
COPY_DEMO=faceup inspect-robots run "${COMMON[@]}" --policy yam_scripted --log-dir ~/yam-eval/copy-id-card-faceup

# Two more wrong ways: the copy made with Color Copy, and Black Copy pressed twice
COPY_DEMO=color inspect-robots run "${COMMON[@]}" --policy yam_scripted --log-dir ~/yam-eval/copy-id-card-color
COPY_DEMO=twice inspect-robots run "${COMMON[@]}" --policy yam_scripted --log-dir ~/yam-eval/copy-id-card-twice

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/copy-id-card-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. With `COPY_DEBUG=<file>` set, each control step adds a line to that file:
- where the card is, which way up, how far each guided edge is from its guide, and how square it is;
- the lid's angle and both buttons' travel;
- the presses so far and whether a good copy was made;
- where the pad tips are.

## The scripted demo
It plays a fixed plan from the fixed layout with the arm's own inverse kinematics, and reads nothing from the
simulator.

1. The right hand turns to point at the printer, with its fingers closing up and down. It comes down beside the card,
   slides in sideways so one finger is above the card's overhang and one below, and pinches it.
2. It lifts the card off the lid and carries it out past the lid's right side. There it turns the card face down in
   two quarter turns of the wrist.
3. Meanwhile the left hand, its closed fingers under the lid's overhanging front edge, lifts the lid round its hinge
   in 5-degree steps to 65 degrees, and holds it there.
4. The right hand tips the card forward, sets its far edge on the glass 15 mm behind and left of the corner, and opens
   a little. It draws back, so the card slides off the lower finger and falls flat; drawing back drags it about 12 mm
   towards the front.
5. Pointing down, it presses its fingertips on the card and pushes it 24 mm towards the right guide, then 8 mm forward.
   The card stops against both guides and the fingertips slide on over it.
6. The right hand moves off. The left hand lowers the lid round its hinge to shut, comes out from under its edge, and
   presses Black Copy.

## Runtime and seeds
- **Official runs (2026-11-20, this version):** the scripted demo scored 1.0, ending on success at step 691 (35 s).
  The card ended face down against both guides, 0.0 mm from each and square within 0.1 degree, under the shut lid, with
  one Copy press.
  - Earlier versions failed, and each failure changed the plan: the card was knocked off, the lid tipped the card off,
    the lid was lifted again on the way to the button, the card was pushed into the front guide only, the fingertips
    only rested on the card.
  - The wrong way (`COPY_DEMO=faceup`: the card laid face up, slid to the corner, lid shut, Black Copy) scored 0: a
    wasted copy of the card's back. Two earlier demo versions pressed Copy with the card 12.1 and 10.7 mm from the right
    guide, and scored 0 the same way.
- **Official runs (2026-11-21, after the review: one copy only, and only Black Copy):**
  - The scripted demo, unchanged, scored 1.0 again, ending on success at step 691 with exactly one press, of Black Copy.
  - `COPY_DEMO=color` (the same, but the copy made with Color Copy) scored 0: one press, of Color Copy, with the card
    placed perfectly.
  - `COPY_DEMO=twice` (the same, but Black Copy pressed a second time 0.6 s after the first, before the good copy's
    1 s hold could end the trial) scored 0: the second press registered at step 686.
  - An earlier `twice` probe pressed again 1.6 s later and scored 1.0, because the trial had already ended on the
    first, good copy. Once success has held for 1 s the trial ends, so a press after that can't happen.
- **Offline checks:** the scoring and reach checks run without the simulator: `python -m pytest -q test_copy_id_card.py`
  (27 tests, including a Color Copy press and a second press, each scoring 0).
- **Seeds:** one fixed layout (`-E object_jitter=0`), so the seed doesn't change the start.
- **`tools/check_reach.py`:** the offline reach scan used to place the printer.
