# Spring-detent clasp: fabrication specification, revision A

This is a build specification, **not a claim that this hardware has been made
or calibrated**. The supplied native trials validate the disclosed reduced
simulation. Check a fabricated clasp as described below before physical trials;
report measured engagement and holding behaviour rather than treating the
simulation's 30 N break setting as a measured specification.

`clasp.scad` is the editable part geometry (millimetres). `clasp-drawing.svg`
shows the catch in section and the spring blank. Export each named part with
OpenSCAD, for example:

```sh
openscad -D 'part="female_base"' -o female-base.stl clasp.scad
```

The mesh represents drill diameters, not helical threads. Tap the holes as
specified below. Do not print the springs in plastic. Nothing is pre-latched,
magnetic, or attached to a robot hand.

## Parts and manufacturing

| Qty | Part | Material / specification |
| --- | --- | --- |
| 1 each | Female base and cover | C360 brass, CNC mill or wire-EDM from supplied geometry; split plane z = 1.25 mm |
| 1 | Male housing, collar and pin | One-piece C360 brass, CNC/EDM; retaining shoulder and groove integral, no glued-on pin |
| 2 | Opposed leaf springs | 0.0508 mm (0.002 inch) **full-hard 302 stainless**; 2.00 mm wide; blank/profile below |
| 2 | Spring anchor screws | M1.2 × 0.25 × 2.5 mm under-head length; head diameter ≤2.3 mm, height ≤0.9 mm |
| 4 | Female cover screws | M1.6 × 0.35 × 4 mm under-head length; head diameter ≤2.8 mm, height ≤1.4 mm |
| 4 | Band fastening screws | M1.2 × 0.25 × 4 mm under-head length; head diameter ≤2.3 mm, height ≤1.2 mm |
| 1 | Band | Leather, 10 × 2 mm section; 195 mm **free** length plus 2 mm hidden in each housing (199 mm cut length) |

Spring stock is available as Precision Brand **22L2 / UPC 22150**, 0.002 inch
stainless shim stock. Its manufacturer's [stainless shim listing](https://www.precisionbrand.com/product-category/stainless-steel-shim/)
identifies full-hard 302 material. This is a material source, not a sourced
finished clasp. Request material certification; do not substitute annealed foil.
Use commercial miniature metric screws meeting the dimensions above; the fit
does not depend on a particular screw-head drive.

General tolerance ±0.05 mm; pin, groove, spring tip position and mouth ±0.03 mm.
Spring thickness: supplied 0.0508 mm nominal, verify within 0.045–0.056 mm.
Deburr every edge. Retaining shoulder corner radius ≤0.05 mm; all other handled
edges R0.1–0.2 mm. Finish contact faces smooth (Ra ≤1.6 µm). Machine the square
mouth by EDM or broach/file the internal corners; do not leave a cutter radius
that obstructs a 4 mm square gauge. Do not round the pin's retaining shoulder.

## Female part coordinates and threads

The entrance plane is x=0, rear face x=10. Outside y=±6, z=±3.5. The mouth is
4 × 4 mm. The bore ends at x=7.8; the rear band slot starts at x=8, with a solid
0.2 mm partition. The band slot is 10.2 × 2.2 mm. Springs occupy the bore sides;
“4 mm bore” refers to the housing opening, not an empty passage through a catch.

Spring anchor holes have axes parallel to y, at x=7, z=0. Tap **M1.2 × 0.25**,
2.5 mm minimum full thread, from each inner side wall outwards. The screw heads
are inside the bore, behind the maximum 3 mm pin insertion; install them before
the cover. Each screw passes through a spring's 1.3 mm hole and clamps its entire
straight root against the wall at |y|=2.0508. No adhesive fixes a spring.

Cover screw centres are (x,y)=(1.6,±4.5),(6,±4.5). Tap the base M1.6 × 0.35;
cover clearance Ø1.8, head counterbore Ø2.9 from z=2 upwards. Screws seat at z=2
and end at z=−2. The band screws are at (8.75,±3) on the female, (−8.75,±3) on the
male. Tap below the slot M1.2 × 0.25; Ø1.3 clearance above and through leather;
head counterbore Ø2.4, seat z=2.2, screw end z=−1.8. Drill the leather through
these holes after seating its 2 mm tabs. The screws clamp and positively retain
the tabs. Recess all heads within the stated housing envelope.

## Spring blank and forming

Cut **6.373 × 2.00 mm** rectangles from the specified foil. Measure flat length
from the free end. Punch/laser a Ø1.30 hole at (length,width)=(5.573,1.00).
Deburr and reject a cracked edge. The two bend lines are at lengths 0.500 and
4.773 mm. Form opposite bends of **16.31°**, with an inside bend radius of
0.15 mm, to give the offset profile below. Use a smooth forming jig; no heating
or post-form heat treatment. The flat lengths are nominal neutral-axis lengths;
trim/adjust the jig to the **final profile**, which controls engagement.

Final contact-face coordinates in the female part, before loading:

- Free tongue: x=1.60 to 2.10, inner face |y|=0.80, z=−1 to +1.
- Active sloping leaf: from (x,|y|)=(2.10,0.80) to (6.20,2.00).
- Flat clamped root: x=6.20 to 7.80, inner face |y|=2.00; hole x=7.00, z=0.
- Stock lies on the **outward** side of those faces. Make a mirrored second leaf.

The generous clearance behind the mouth lets the leaves flex outward as the
head enters. The nominal straight beam is 4.273 mm long. The thin spring is
chosen for elastic insertion travel, not to supply a 30 N spring preload:
retention comes from a positive shoulder. A spring that takes a permanent set
or has a tip outside the stated position tolerance must be replaced.

## Male pin and engagement

The male housing is x=−10 to 0. Its 5 × 5 mm collar extends x=0 to 3. The exposed
pin extends x=3 to 6 (3 mm): 2.2 mm square root to x=4.2; Ø1.6 neck from x=4.2
to 5.6; 2.2 mm square head from x=5.6 to 5.8; a 0.2 mm lead chamfer ending in a
1.6 mm square tip at x=6. The shoulders are integral with the housing.

At 2.5 mm insertion, the back of the head clears both leaf tongues at bore
depth 2.1 mm. They return into the neck and block withdrawal. At 3 mm insertion
the collar stops against the female face; the 1.4 mm long neck still spans both
tongues. A light separating load seats the head against the leaves at 2.5 mm
insertion: housing front faces 3.5 mm apart, centres 13.5 mm apart. Contact of
the housing exteriors cannot engage this catch.

## Assembly, reset and bench qualification

1. Fit the two spring roots, align their tips to the specified positions using
   a magnified photograph and scale, then snug the anchor screws without
   distorting the foil. Check both free tongues move independently and return.
2. Screw on the cover. Fit the leather tabs and the four band screws. Mark each
   screw head with a witness line. Nothing projects beyond the gripping faces.
3. Close by aligned axial insertion. Photograph the collar-to-socket gap:
   0–0.5 mm after engagement, approximately 0.5 mm under a light separating load.
   Withdrawals before 2.5 mm insertion must not latch. Test 0°, 45° and 90°
   relative axial orientation, without forcing a jammed pin.
4. Make **five complete closures**, each holding a **100 g hanging mass for
   three seconds**. Inspect for leaf set, loose screws, damaged leather or a
   pin that withdraws. Reject/rework any failure and restart the five checks.
   This is a practical qualification requirement; no measured results are
   supplied or invented here. Record stock batch, dimensions and observations.
5. For reset, remove the four cover screws, use a fine hook to spread both leaf
   tongues away from the pin neck, and withdraw the male end. Release the leaves,
   inspect their return positions, and refit the cover. This manual service
   operation is outside the robot trial; it consumes no part. Recheck the
   witness marks and restore the photographed unlatched layout.

The simulated pin omits the small groove/chamfer and internal leaves. The
insertion-gated catch represents their retention, axial rotation and limited
swing. Housing envelopes, bore, exposed pin length, collar and engagement depth
match the native trials. The 195 mm simulated leather is the exposed span;
the hidden mounting tabs add no reach. These fabrication files specify a
repeatable mechanism; they do not establish physical force calibration or
prove every simulated catch angle on real hardware.
