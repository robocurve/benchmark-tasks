# Material-response boundary

Paste starts as a finite120µL bead on the loose stencil. Each aperture is subdivided into volume elements with the real125µm capacity. Native blade/foil load, edge position, relative motion and local foil/PCB gap drive filling. The nominal yield line load is20N/m; the characteristic filling time is30ms at8N over80mm. These are explicit engineering coefficients, not a calibrated commercial-paste fit.

After all occupied apertures clear the board, a geometry-dependent adhesion split leaves paste on the PCB and retains a remainder on the foil. Deposit locations use actual near-board cavity positions, with no snapping to targets. Later scraping can move deposits. Full rheological slump, flux chemistry, reflow and foil flexure are omitted. The independent native150µm-sphere gate confirms that apertures are physically open; the full model does not claim all paste is a native sphere simulation.

No material rule moves a rigid body. Model input remains the three original RGB streams and joint state; private contact/pose information is used only for environment evolution and scoring.

- [Adafruit Motor Shield v2 PCB sources](https://github.com/adafruit/Adafruit-Motor-Shield-V2-PCB)
- Original design, licence, exact commit and pad extraction are included in `reference/` and `board-geometry.json`.
