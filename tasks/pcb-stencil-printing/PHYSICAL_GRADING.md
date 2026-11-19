# Physical grading

Weigh the clean, dry PCB on a balance with 1 mg resolution. Weigh the
same board after the print and subtract its tare. Exclude paste left on the
bed or liner from this mass. With integer milligram readings, 13 mg is the
first displayed gain that meets the 12.5 mg band. Grade the mass gain
against the bands in [SCORING.md](SCORING.md). Photograph the board and any visible surrounding deposit
at sufficient resolution to see each pad and the gaps between pads. Align
the supplied pad layout to the photograph. Count the union of visible paste
footprints on each pad. For the on-pad area fraction, divide the area on
pads by the whole visible deposited footprint. Any continuous visible trace
that wets two distinct pads is a bridge, even if the wetted ends are small.
Overlapping deposits count once. The model uses a nominal density of
4.2 g/mL and resolves deposits at least 10 µm thick. Keep the original
photograph and both balance readings.

## Tool clearance during the final second

Keep a plan view of the board outline and a side view with a millimetre scale
at the board. A squeegee parked wholly outside the outline clears at any
height, including on its original low rests. If part of the tool projects
over the board, use the side view to check the lower edge of that part
against a mark 2 mm above the bare PCB face. Check both ends of the blade
and the handle when tilted. Contact with the board or the 2 mm zone interrupts
the hold. Place the mark beside the board to leave the print undisturbed.
Use the same second of footage for stencil support, release and stillness.

## Paste and reset for each physical print

Use Chip Quik SMD291SNL10T4, the premixed syringe containing SAC305 with
Type 4 powder (20–38 µm) and REL0 no-clean flux. Keep the capped syringe at
3–8 °C. Condition it closed for four hours at 20–25 °C before dispensing.
Use the syringe as supplied without stirring or thinning. Prime the tip onto
a liner until a uniform, continuous bead emerges. Reject visibly dried or
clumped paste. Use the same product and lot across compared trials and
record its lot, conditioning time and room temperature.
[Manufacturer's datasheet](https://www.chipquik.com/datasheets/SMD291SNL10T4.pdf)

With the foil level on its starting supports, dispense **120 µL** in one
straight, approximately uniform **76 mm** bead parallel to the blade.
Measure with a primed positive-displacement paste dispenser set to 0.120 mL.
Use the foil centre as the origin and the setup axes shown by the pad layout.
Start at x = −38 mm, end at x = +38 mm, and keep the line at y = −24 mm
(on the side toward the PCB's starting position). Start the trial within
60 seconds of dispensing. Keep the frame and handle dry. Before each trial
use fresh paste, remove residue from both foil faces, all apertures, PCB and
blade, let them dry, and verify the clean PCB tare and open apertures.
The physical dose is volumetric. The simulator's nominal density is used
only in its material model.
