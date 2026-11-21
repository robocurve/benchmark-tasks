# Custom printer-style cassette: construction package (revision FC1)

This packet defines an original, custom benchmark cassette. It is not an HP part or an OEM-compatible replacement. Status: **complete drawings, profiles, BOM and sources; unbuilt and uncalibrated**. No prototype has been built from them, and no physical result, spring rate, mass, drag curve or holding strength has been measured. A built cassette is used for trials only after it passes the acceptance tests in CALIBRATION.md.

Units are millimetres. U is across the paper, V increases from the feed edge toward the rear, Z is above the underside of the 3 mm tray floor. The floor underside is 35 mm above the supporting bench; plinth Z=-35..0. The floor is U±152,V−12..405,Z0..3. The fixed removable handle adds V−22..−12. Overall body including handle is 304×427 mm, 100 mm above the bench to the front-panel top. Paper-contact and latch coordinates are relative to the floor, so bench heights add 35 mm.

Read ASSEMBLY.md, HINGE.md, PAWL.md, AXLES.md, SCALE.md, SPRINGS.md and CALIBRATION.md together. All parts, holes, tolerances and secondary machining are defined by these documents, generated cad/*.dxf and parts.csv. Dimension labels and machining instructions govern; engineering drawings are not to scale. Profiles are editable 2D manufacturing sources. `parts.scad` is an optional unassembled extrusion source; no assembled STEP, verified mesh, CAM or full assembly simulation is claimed.

Regenerate with Python 3 and Pillow:

```
python3 fabrication/build_package.py
```

The generator reads adjacent `task.py` when present, without importing simulator code. Otherwise it uses `design-config.json` and labels the resulting contract as a design configuration, with no current source hash. It requires no private repository path, network, paid service or simulator. `validation.json` records numerical/profile checks and hashes, not physical validation. SVG and PNG share one drawing source. `drawings/compression-scale.svg` is a separate 100×25 mm actual-size print sheet.

The mechanism preserves squeeze, slide, square loose A4 sheets, ease compression, and release. The paired width guides use opposed racks and an idler pinion. A bidirectional square pawl engages a 0.5 mm rack only inside its ±0.05 mm tooth-phase window; off-phase release leaves the guide unlatched until a small slide of the carriage brings the pawl over a tooth, which it then catches. No automatic centering is implied. Dry sliding friction is adjusted by split bushes; it is not a viscous damper. Direct 0–5 mm pad scales measure pad compression, independently of guide position.

All original code, profiles, artwork and text are MIT licensed. SOURCES.md lists material and custom-manufacture routes. No supplier stock or lead-time guarantee, purchase, quote or contact is asserted.
