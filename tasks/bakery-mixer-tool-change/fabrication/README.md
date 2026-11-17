# Manufacturing files

These are original construction files for the empty-bowl apparatus in
[PHYSICAL_BUILD.md](../PHYSICAL_BUILD.md). They are not OEM parts or a record
of manufactured hardware. All STL coordinates and drawing dimensions are
**millimetres**. The native scene still comes from `geometry.json`; exporting
these files does not modify or execute the simulator.

## Reproduce and inspect

From the repository root, in a separate CAD environment:

```bash
python3 -m venv /tmp/mixer-cad
/tmp/mixer-cad/bin/python -m pip install -r tasks/bakery-mixer-tool-change/fabrication/requirements.txt
/tmp/mixer-cad/bin/python tasks/bakery-mixer-tool-change/export_fabrication.py --output /tmp/mixer-manufacturing
```

The generator performs solid unions, exports STL, reloads each STL and checks
one connected, positively oriented watertight solid and unchanged bounds. It
also checks the actual hub/pin meshes through insertion and twist, interference
against straight downward removal, the open shaft bore, and matched lift teeth
at 43 sample angles. `mesh-validation.json` records versions, source/generator
hashes, dimensions, density estimates and the results. These are static CAD
checks. They establish neither contact dynamics nor print strength, motor
margin, friction, force, stopping time or physical safety.

STLs have been shifted in z to sit on a slicer bed; x/y retain their part
origins. Recover assembly coordinates by subtracting each
`print_translation_mm` in the manifest. Do not interpret the slicer origin as
the beater hub origin. `mesh-preview.png` is a CAD illustration, separate from
the native setup image and videos.

| File | Quantity | Material / finishing |
| --- | ---: | --- |
| `beater.stl` | 1 | PETG; ream 12.7 mm bore and finish the L-slot |
| `guard.stl` | 1 | PETG; remove supports and round every open end |
| `bowl.stl` | 1 | PETG; cradle/clamps are separate stock parts |
| `lift_pinion.stl`, `lift_rack.stl` | 1 each | PETG; matched pitch 3.125 mm |
| `drive_pod.stl` | 1 | PETG; fixed nose cover, mounting holes drilled to brackets |
| `spindle_bearing_housing.stl` | 1 | PETG; 26 mm OD, 18 mm long |
| `orbit_bearing_housing.stl` | 1 | PETG; 26 mm OD, 28 mm long, 38 mm flange |
| `spin_motor_pulley.stl`, `spin_spindle_pulley.stl` | 1 each | PETG; 24 mm pitch diameter, 2 mm round belt |
| `spring_mandrel.stl` | 1 coupon jig | PETG; 17.2 mm OD starting mandrel; measure springback |
| `metal_ballast_large.stl` | 2 | **Cut/drill 6 mm mild steel; do not print** |
| `metal_ballast_small.stl` | 1 | **Cut/drill 6 mm mild steel; do not print** |

Use tough PETG, 0.2 mm layers, at least six perimeters and solid infill for the
core pieces. Use a printer with at least a 250 x 250 x 200 mm build volume.
Rotate the guard approximately 45 degrees about z to fit its 261 mm long
bounding box on the square bed; confirm the sliced bounds before printing.
Supports are required for the guard, hub overhangs and cover flange. Remove
all supports; ream mating bores, finish the slot and test the fit with power
isolated. Raw mesh validity does not guarantee a usable as-printed tolerance.
Reject delamination, cracks or a flexible joint that loses its locating shape.

Both bearing housings have 20.96 mm nominal bearing pockets. Fit/ream them
to the actual 21 mm outer races. The central 12.4 mm passage is clearance,
not a shaft bearing. Retention uses the shoulders, shaft collars/spacers and
the flange attachment specified in the build. The pulleys require the
post-print set screw and collar described there. Buy ordinary 12T/36T GT2
pulleys and a 180 mm belt for the orbit transmission.

## Drawings and ballast

[coupling-dimensions.svg](coupling-dimensions.svg) defines the shaft, collar,
spring seat, guides and slot section. [carrier-plates.svg](carrier-plates.svg)
defines the two stock aluminium plates and steel webs. The plan views are
1:1 at 100% print scale; check the printed 50 mm reference before drilling.
The editable SVGs are the drawing sources. The bridge, chassis, racks and
cradle are dimensioned stock cut lists in the build document; do not print
the solid native draining platforms.

The bare beater mesh is approximately 67 g at 1.25 g/cm3 PETG density.
Two large steel webs, one short web and fasteners give a nominal 251 g tool.
Fit them at the stated locations and **weigh the complete tool to 250 +/-10 g**.
Ballast and capped fasteners remain below the grasp, within the blade's
rotating outline. They cannot provide outside support. The physical tool's
centre of mass is about 110 mm below its hub versus the native lumped 80 mm;
this difference and the approximate inertia are disclosed.

Add captive guard ballast to 180 +/-10 g and bowl/cradle ballast to
1.20 +/-0.05 kg, following the build. Check all rotating clearances and
retention after adding ballast. No weight may enter the bowl cavity or fall
free. Parts/calibration/waste allowances are included in the
[costed BOM](costed-bom.json): $353.01 estimate, $400 planning allowance,
before tax/shipping, labour and already-owned workshop tools.

The physical specimen remains unbuilt. Complete the force, clearances,
interlock, ten-cycle run/stop and observable-scoring commissioning record
before any real robot trial.
