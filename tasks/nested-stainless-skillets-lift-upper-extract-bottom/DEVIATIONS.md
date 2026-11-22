# Physical approximations

The full-scale cookware references are the 8-inch and 10-inch uncoated stainless skillets linked in the README. The listing's catalog data reports 4.41 lb (2.00 kg) for the package and 18.16 × 10.59 × 5.25 in for the shipping envelope. Individual skillet masses and wall profiles are absent from that catalog data. The mat listing publishes its 24 × 18 in footprint, silicone rubber material and 7 oz package weight.

| Dimension or material | Representation |
|---|---|
| Rim diameter | Nominal retail diameters rounded to 203 and 254 mm |
| Flat sole diameter and wall height | Modeled at 155 × 42 mm and 195 × 46 mm respectively, using the task brief |
| Wall and sole thickness | Estimated 2.7 mm wall and 5.5 or 6.5 mm laminated sole |
| Mass | Estimated 0.80 and 1.10 kg, consistent with a 2.00 kg gross package |
| Handle | Estimated bar section 24 × 16 mm, with riveted shoulders at the bowl wall. The distance from bowl center to handle end is 267 mm or 310 mm. Rivet heads are visual detail within the compound envelope. |
| Silicone work surface | Exact listed footprint, estimated 1.4 mm thickness and a simplified white and turquoise finish. The virtual mat adds two placement guides in the texture. Use the README’s ruler coordinates with the retail mat, which needs no added parts. |

The physics scene resolves the pan wells with flat stepped soles and sloped wall facets. The skillet and mat meshes follow the collider envelopes. Rigid contact represents the countertop-backed silicone and steel. The cool simulated trial represents staging cookware above a clean surface without heat or oil dynamics.

Seating allows tilt up to 3.0°, with both the lowest collider point and the sole-center origin required within −0.5 to +3.0 mm relative to the mat top. The negative allowance covers compliant contact, not visible sinking. Separation uses rim disks and yaw-oriented shoulder and grip rectangles with a minimum horizontal clearance of 2 mm.

CPU replay measurements during the final 2.0 s hold (rounded to six decimals; each tuple is lowest collider offset in mm, sole-center offset in mm, tilt in degrees):

| Cases with matching rounded readings | Upper | Lower |
|---|---|---|
| Nominal, jitter0–4, friction ±30%, fric0.8, fric1.25, fric1.5 | (−0.005507, −0.002820, 0.001986) | (−0.004662, −0.002820, 0.001082) |
| solref0.011 | (−0.009222, −0.004723, 0.003326) | (−0.007808, −0.004723, 0.001813) |

The soft-contact case stays within the −0.5 mm collider bound, so no wider penetration allowance is needed.

## Scripted demo carry (revision 1)

The first scripted demo dragged the larger skillet across the mat by its handle. The pan turned a few degrees about the grip on the way, and the turn differed between the two simulators (final heading about -112 degrees in the local GPU simulator, about -117 degrees in offline MuJoCo), so the pan landed about 23 mm apart in the two. The demo now lifts the larger skillet 60 mm by its handle, carries it to its spot and sets it down, so the grip holds its heading (offline MuJoCo: -115.5 degrees at the end, start -115). The scorer and the spots are unchanged; only the demo's motion changed.

The larger skillet's staging spot moved 8 mm away from the mat's near edge (README table: Y -6 -> 2 mm). At the old spot its handle end cleared the 10 mm mat margin by only 3.9 mm, so a 5 mm shift of the set-down pushed it over the margin (offline jitter runs 2 and 3 and +30 % friction scored 0.0). Two scorer tests that build poses from the spot geometry follow it: the touching-handles pose uses a 0.25 m offset (was 0.2) and the clear-handles yaw boundary is -92/-90 degrees (was -94/-92).
