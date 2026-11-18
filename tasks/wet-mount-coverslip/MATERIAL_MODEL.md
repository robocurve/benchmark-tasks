# Material-response boundary

The water model uses a0.5mm finite-volume grid,10µL total liquid, and a20µm unresolved film gap. Local excess moves to neighbouring cells while conserving volume. Dry cells enclosed by liquid beneath a close cover become trapped air; physically reopening a gap vents them. Flat placement is allowed to succeed. No tilt angle, arm role, elapsed phase or demo waypoint gives credit.

The current model is one-way: liquid does not exert capillary reaction forces on the rigid glass. It is an engineering approximation, not a validated fluid/optical model. Blue tracer makes the finite state visible in the original RGB cameras. No tissue, microbes, incubation or clinical diagnosis is simulated.

No material rule moves a rigid body. Model input remains the three original RGB streams and joint state; private contact/pose information is used only for environment evolution and scoring.

- [AmScope coverslip sizes](https://amscope.com/products/cs-s22-200)
- [Microscopy technique](https://amscope.co.uk/pages/using-a-microscope)
- [Wet-mount procedure](https://marclos.github.io/SOPs/lab_sops/SOP-L601-wet-mount-microscopy.html)

Visible wet cells and the numeric wet-area criterion share the same12µm
resolved-film cutoff. Subthreshold liquid is still conserved, but is not drawn
as an opaque blue wet patch; this is a visibility approximation, not calibrated optics.
