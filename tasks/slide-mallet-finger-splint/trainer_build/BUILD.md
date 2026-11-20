# Building the trainer hand for the mallet-finger task

This is a reference build of the one custom mechanism in the task: an adult index finger whose last joint (the DIP, 26.6 mm from the fingertip apex) is a hinge that droops 40 degrees at rest and resists straightening with a weak spring, as in a mallet finger. Everything else (hand, wedge, splint) is a printed or cast part. No single off-the-shelf hinged-finger trainer is claimed; the files below let another lab build and reset an equivalent finger.

Status of this design: the three parts were generated from the simulation's dimensions (`make_parts.py`), are watertight, and were checked by a collision sweep of the hinge through -5 to 70 degrees (no overlap). A physical print has **not** been made or tested by us. The spring is a design value; step 5 tunes it against the measurement table, which is what makes the reset repeatable across labs.

## Files

| File | What it is |
|---|---|
| `make_parts.py` | Parametric generator (needs `trimesh`, `numpy`, `manifold3d`). Prints the collision sweep. |
| `fingertip.stl` | Distal segment: 18 x 15 mm section, rounded apex 26.6 mm from the hinge axis, 6 mm hinge tongue, 1 mm blind hole for the spring's moving leg. |
| `finger_stub.stl` | Middle and proximal segment, 19 x 16 mm, 68 mm long: clevis with an 11 mm slot, 4 mm arms, 3.2 mm hinge bore, arc slot for the spring leg (world angles 17 to 98 degrees), 1 mm anchor hole for the fixed leg, two 3.4 mm holes to bolt the stub to the hand's finger bed. |
| `splint.stl` | Stack-type splint, 23.5 x 20 x 47.5 mm outside, 20.5 x 17 mm inside, 1.5 mm wall, closed hood over the first 31 mm, floor plus 8.5 mm side walls beyond it. |

Frame for all parts: x across the finger, y along it toward the fingertip, z up (nail side); the DIP axis is the x axis through the origin.

## Bill of materials

| Item | Spec |
|---|---|
| Hinge pin | M3 x 25 mm socket-head screw with a nylon-insert nut (bore is 3.2 mm) |
| Spring | Steel music-wire torsion spring, wire 0.5 mm, mean coil diameter 5 mm, about 2.0 active coils, both legs bent parallel to the axis (about 12 mm long). Target rate 0.35 N mm per degree (0.02 N m/rad) |
| Hand | Any adult-size hand form (about 86 mm across the knuckles, 26 mm palm thickness) whose index finger can be replaced by `finger_stub.stl` bolted to the finger bed with two M3 screws; silicone skin 2 mm over the printed parts and the hand's foam core |
| Skin | Platinum-cure silicone, Shore 00-30 or softer, brushed or dipped 2 mm thick; leave the hinge slot clear |
| Printing | PETG or PLA, 0.2 mm layers, 3 perimeters, 30% infill; splint in clear PETG so the fingertip apex and the closed end are visible for photo checks |
| Markers | Black ink dot at the DIP axis on each side of the finger stub; a white line on the fingertip 3 mm from the apex; a line on the splint 1.5 mm inside the closed end (the "seated" mark) |

## Assembly

1. Print the three parts. Ream the hinge bores with a 3.2 mm drill if the print is tight; the tip must turn by hand without binding.
2. Fit the fingertip's tongue into the stub's slot (tongue against the left arm). Push the M3 screw through the right arm, the tongue and the left arm and fit the nut so the joint turns freely with 0.2 mm of side play.
3. Slide the spring over the screw outside the right arm. Hook the fixed leg into the anchor hole on the right arm's outer face (at 143 degrees, radius 5.5 mm). Pass the moving leg through the arc slot into the blind hole in the tongue. The spring must be unloaded at 37 degrees of flexion.
4. Bolt the stub into the hand's finger bed so the fingertip points up the wedge slope when the palm lies on the wedge.
5. Tune (next section), then cast or brush the silicone skin over the finger stub and the fingertip, keeping the slot and the spring covered by a thin flexible sleeve.

## Setting and checking the resting angle and the spring resistance

Flexion is 0 when the fingertip is straight and positive when it bends toward the palm.

| Check | How | Target |
|---|---|---|
| Resting droop | Lay the hand palm down on the 20 degree wedge, let the fingertip hang, hold a phone inclinometer on the fingertip's nail face (or a protractor against the two ink dots) | 40 degrees of flexion, plus or minus 3 degrees. The spring's unloaded angle is 37 degrees; the tip's own weight (8 g) adds about 3 degrees. If outside the range, move the anchor leg by one hole position or re-form the leg. |
| Spring resistance | Hold the stub fixed, push the fingertip apex perpendicular to the finger with a kitchen scale (0.01 N, or a 50 g fish scale) and read the force as the finger reaches each angle | See table below, plus or minus 0.1 N |
| Range | Move the fingertip by hand | Free from 5 degrees past straight (-5) to 70 degrees flexion, no sticking |

Force at the apex needed to hold the fingertip at each flexion (spring rate 0.02 N m/rad, rest 37 degrees, lever 26.6 mm):

| Flexion (degrees) | 0 | 10 | 20 | 30 | 37 |
|---|---|---|---|---|---|
| Apex force (N) | 0.49 | 0.35 | 0.22 | 0.09 | 0.00 |

If the force at 0 degrees reads high, remove a quarter coil; if low, add a quarter coil (the rate is inversely proportional to the number of active coils). Re-check the droop after changing the spring.

## Reset between trials

Remove the splint, let the fingertip droop, confirm 40 degrees plus or minus 3 on the inclinometer, and put the hand back on its outline on the wedge. Repeat the spring check once per session or after 50 trials, whichever comes first.
