# SCENE -- ims-cassette-load-and-latch

Station coordinates: metres, +X forward, +Y left, origin midway between the arm bases, z = 0 on the bench top (TOP).
Real size everywhere; nothing is enlarged.

| object | kind | real product / source | dimensions (mm) | mass | material | position |
|---|---|---|---|---|---|---|
| exam cassette, 5 instruments | fixed base (in a holder) + lid hinge + clip hinge | stainless instrument cassette; size and layout from the maker's IMS catalog 2020 p.6 ("76 mm x 203 mm x 32 mm ... Holds 5 instruments"), latch p.5 ("Ergonomic latch with positive locking") | 76 x 203 x 32 closed; base rim 17, lid 15; walls 1.5 (real ones perforated; modelled solid) | fixed | brushed stainless, blue silicone rails, red clip | centre (0.300, 0.000), long side toward the robot |
| rails | part of the base, + 10 sprung lips (slide joints, one per rail at every notch) | two silicone rails across the width (catalog photo p.6) | 8 thick, 120 apart (estimate); 5 notches 10.5 wide at 12.7 pitch (estimate), floor 4 above the cassette floor, tooth tops 16 (rounded); lips 2 mm round, 2.5 mm into the mouths of notches 2 and 4, 3 kN/m (estimate) | -- | blue silicone | y = +-0.060 |
| lid | hinge about Y on the back long edge | catalog p.6 photo | 76 x 203 x 15, one press pad between the rails 14.3 above the floor (0.8 over a seated handle) | 0.12 kg (estimate) | stainless sheet (real one perforated; modelled solid) | open 120 deg (leaning back on its stop), closed = 0 |
| latch clip | hinge about Y on the lid's front top edge | catalog p.5 Figure 2 (pivoting clip, middle of the front edge, "positive locking") | 23 x 12 x 2 with a 1.5 x 1.2 hook tab at its tip (estimate) | 4 g | red | open = leaning back over the lid at 125 deg (over centre); locked = hanging at -90 deg, tab 0.9 mm under the keeper lip |
| keeper lip | fixed (separate body on the base's front wall) | the cassette's latch catch | 1.2 x 24 x 1.2, 4 mm under the rim | -- | steel | the clip's hook tab catches under it |
| instruments 1, 3, 5 | loose (free bodies) in slots 1, 3, 5, held only by the rail lips | single-ended exam instruments | 165 long, 130 x 9.5 handle (estimate) | -- | satin stainless, colour bands | down in their notches |
| instrument 2 (mouth mirror) | loose | same handle | 165 long, 9.5 handle | 20 g (estimate) | satin stainless | on the tray at x 0.240, y 0, handle along y |
| instrument 4 (explorer), misloaded | loose | same handle | 165 long, 9.5 handle | 20 g (estimate) | satin stainless | across the cassette, handle along x, centre (0.270, 0.047), on the front rim and over the seated handles (clear of the open lid's pads, which swing back over the hinge) |
| holder, tray | fixed | shop-made bench holder (6 mm plate, corner stops) and a resin instrument tray (no product) | holder 100 x 227 x 6; tray 64 x 190 x 6 with a back lip and cradle ridges (1.5 x 24 x 3, 5.5 mm either side of each handle near its ends) | fixed | black anodised; white resin | under the cassette; tray centre x 0.222 |

Misload (the error to correct): the explorer lying across the cassette holds the lid about 1 cm open (lid 6.8 deg on it
in the control run) and the clip cannot hook; it has to be lifted out, turned 90 deg and seated in a free slot.

Mechanism: a handle set on a notch's mouth rests on the rail lips and the rounded tooth cap (centre 16.3 mm up);
pressed down (the demo pushes with up to 13 N in MuJoCo) it pushes its lip back (one lip per rail on the notch's +x side; about 1.3-1.5 mm by the geometry, peak 0.56-0.59 mm
logged in the MuJoCo pre-check, where contact softness takes the rest) and drops to the notch floor (centre 8.75 mm), and
the lip snaps back over it. Seated, its top is 0.8 mm under the lid's pads, so the lid closes flat on its own weight.
The clip, leaning back over centre, stays open until pushed forward past upright; it then swings down and its hook
tab ends under the keeper lip, a separate fixed steel body on the front wall (links of one articulation do not
collide, so a lip on the base itself could not catch the clip). All of this is collision geometry and joints, not
scripted state.

Reach (offline tests, official IK, worst_ik <= 0.1 mm): the left arm grips the misloaded explorer with wrist 0 deg
(jaws across a handle along x) and turns to -90 deg in the air. Jaws across a handle along y need wrist yaw +90 deg on the right arm
and -90 deg on the left (the other sign is outside each wrist's range at these heights: 11-23 deg error). The left arm
reaches behind the open lid by coming round the cassette's +Y end at 0.085 m (straight over the lid's top edge is out
of reach). Parks: right (0.16, -0.32, +0.20), left (0.20, +0.26, +0.20).
