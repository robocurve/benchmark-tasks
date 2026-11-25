# Balloon catheter trainer

Original procedural training props created on 2026-11-10. No external anatomical
meshes, photographs, textures or commercial device branding are included.
`geometry.py` reuses the original helper code of the local closed-suction-drain
trainer; the new scene geometry is defined by `build_asset.py` in this package.

Context: the retention balloon must be deflated before the catheter is removed.
Reference, without copying imagery or instructional text:
https://leedscommunityhealthcare.nhs.uk/our-services-a-z/continence-urology-and-colorectal-cucs/trial-without-catheter-twoc/

This is a benchtop engineering phantom, not a clinical device or complete procedure
model. Dimensions and rigid proxies are chosen for visibility and contact
inspection. The main catheter is free; the valve branch has a spherical joint. No forceful aspiration or withdrawal procedure is prescribed. Task-local runtime implements a measured pressure/volume surrogate with a finite-force
receiver drive and two measured inflated/collapsed render and collision states. No fluid solver is claimed.

## Version 2 — generic annular-sector transform correction (2026-11-14)

Original author construction and dimensions are retained. Convex annular boxes now
compose translate/rotate/scale, so nonuniform radial/tangential dimensions rotate
with each sector. Version 1 composed translate/scale/rotate and produced a 12mm
minimum but noncircular neck (top clearance 15.6mm). Version 2 restores the intended
12mm minimum bore with a 32-facet approximation (6.000–6.029mm free radius). The valve socket uses the same
correction; its intended 9.5mm minimum bore uses a 24-facet approximation (4.750–4.791mm free radius). The syringe sectors have
equal transverse scales, so their geometry is unaffected. No threshold, force,
mass, friction, collision filter or runtime actuation is changed. The v1 package
is preserved; this reusable generic correction is a new package.

## Version 3 — physical dish supports (2026-11-14)

Enable native collision on the two existing 20 x 28 x 8 mm polymer dish rests.
Their dimensions, transforms and visible geometry remain unchanged; the rests
now provide the physical support described in the task setup. Both previous
packages are preserved. No device geometry, body mass, material, pressure,
connector, scorer or robot actuator parameter changes.
