# SCENE -- mortise-latchbolt-handing-flip

Station coordinates: metres, +X forward, +Y left, origin midway between the arm bases, z = 0 on the bench top (TOP).
Real size everywhere; nothing is enlarged.

| object | kind | real product / source | dimensions (mm) | mass | material | position |
|---|---|---|---|---|---|---|
| mortise lock case | fixed, task body | grade-1 commercial mortise lock case; size from the maker's L/LV9000 installation instructions 108367 p.1 ("4-7/16 x 6-1/16 x 1", 113 x 154 x 25 mm"); features (steel case, anti-friction tongue, field-reversible handing) from the maker's L-Series catalogue 102899 p.9 | 113 deep x 154 long x 25 thick | fixed (no mass) | plated steel case, sintered hub with 8 mm square spindle hole at 2-3/4 in backset | standing on its back edge, faceplate up, centre (0.320, -0.150), long axis along Y |
| armored front | part of the case | the maker's catalogue p.31 (armored fronts, 8 in, stainless) | 203 x 32 x 3.2, opening 29.5 x 17.5 | -- | brushed stainless (630), two countersunk Phillips screws | faceplate top 128.2 above the bench |
| latchbolt | fixed base + slide + hinge | same lock; 3/4 in (19 mm) throw (installation instructions 108367 p.1); head size and key engagement estimated (not published) | head 28 x 16, 19 proud, 6.5 inside the opening, pulls out 13 more; flat bevel from 2 mm to a 3.5 mm land; anti-friction tongue in the bevel | 65 g (estimated) | satin chrome (626) brass head, black-oxide tongue | centred in the opening, bevel facing +X at start |
| jamb with strike | fixed guide base + slide along X | jamb section with a standard 4-7/8 x 1-1/4 in mortise-lock strike (ANSI/BHMA A115.1 frame preparation); opening, lip and strike box estimated | jamb 70 (X) x 260 (Y) x 25; strike plate 124 x 55 incl. lip, 1.6 thick; opening 20 x 32 into a 22 mm deep strike box; push block 30 x 50 x 20 on the back end | 0.6 kg (estimated) | oak (satin), brushed stainless strike, dark strike box | face 3 mm above the faceplate; closed = strike opening over the latch (stop), starts open 90 mm toward -X; return spring 12 N/m (~1 N at closed) |
| linear guide | fixed, part of the jamb body | shop-made aluminium guide (no product) | rail 200 x 20, foot 220 x 60 x 8, rubber stop | fixed | anodised aluminium | under the jamb's +Y end, rail centre y = +0.020 |
| bench holder | part of the lockcase body | shop-made padded holder (no product): aluminium base plate + two cheeks with rubber pads | base 190 x 80 x 12, cheeks 130 x 14 x 45 | fixed | anodised aluminium, black rubber | under the case, centre (0.320, -0.150) |
| end-cutting pliers | visual-only prop | NVIDIA SimReady `tool_pliers_a01` (CC-BY-4.0) | 192 long | none | model PBR | (0.170, -0.060), lying flat |
| claw hammer | visual-only prop | NVIDIA SimReady `tool_hammer_a01` (CC-BY-4.0) | 304 long | none | model PBR | (0.625, +0.175), lying flat |

Mechanism (the ability under test, 1:1): the head is a 16 x 28 mm rectangle in a 17.5 x 29.5 mm opening, so it only
turns once pulled out more than 6.5 mm past its rated throw; released while turned off-square it rests on the
faceplate (held out on the shoulder); square at 0 or 180 deg it seats back at the 19 mm throw under its spring.
Strike: closing the jamb (+X) brings the strike lip (3 mm above the faceplate) onto the latch's -X face. Square back
toward the lip (handing as delivered): the lip stops on it, 42 mm short of the stop, and the return spring carries the
jamb back. Bevel toward the lip (reversed): the lip cams the latch in to 3 mm, slides over it, and at the stop the
latch springs 19 mm out into the strike opening; its square back then holds the jamb against the return spring
(rests 1 mm off the stop). All of this is collision geometry, not scripted state.

Reach (screen.py and the offline tests, official IK): left arm pushes the jamb's back end at y = +0.020 with closed
fingers, TCP 0.152 m above the bench, x 0.157 -> 0.317 (commanded 41 mm past the end face at the stop, worst_ik 0.1 mm);
right arm parks at (0.16, -0.32, +0.20), outside the hands-off box.
Right arm: grip / pulled / hover TCP 0.152 / 0.169 / 0.191 m above the bench at yaw 0, +90 and
-90 deg, worst_ik 0.08 mm. The spec's spot near the station centre (0.33, -0.02) caps at ~0.155 m and yaw -90 deg is
infeasible there, so the lock sits on the right arm's side at (0.32, -0.15) (still near the image centre).
