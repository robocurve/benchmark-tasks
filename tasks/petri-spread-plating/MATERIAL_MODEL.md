# Material-response boundary

The liquid occupies a1mm grid over the86.8mm agar surface.100µL is predispensed. A contacting moving bar exchanges liquid with a finite10µL tool film; an empty tool cannot paint, hovering cannot spread and stationary touching is not a sweep. The model conserves liquid on agar, tool and lost-volume accounts. A reduced gravity flux drains tilted dishes; an inverted dish loses its unbound liquid.

The rigid agar collision surface has native compliant contact2500N/m and4.6Ns/m. Bar indentation of0.3mm marks a damage witness; this is not calibrated gel tearing. Only the central80mm circle is graded, leaving a3.4mm rim margin. Damage anywhere in the full surface is recorded. The trial tests a physical nonliving spreading procedure, not biological success or sterility.

No material rule moves a rigid body. Model input remains the three original RGB streams and joint state; private contact/pose information is used only for environment evolution and scoring.

- [GVS standard cell spreaders](https://www.gvs.com/en/catalog/cell-spreader)
- [Spread/lawn plate technique](https://practicalbiology.org/standard-techniques/making-a-spread-or-lawn-plate.html)
- [University of Otago lab procedures](https://blogs.otago.ac.nz/classprep/files/2024/02/MicrobiologyLaboratoryProceduresHandbook2024.pdf)

Visible wet patches and numeric coverage share the same5µm resolved-film cutoff.
Thinner liquid remains conserved but is not rendered as an opaque wet patch.
This is an optical visibility approximation. The round lid has no keyed yaw;
only its face normals, support and closure matter.

The damping is an engineering critical-damping reference:2√(2500N/m×0.0021kg)≈4.58Ns/m. It is not an agar measurement. The earlier underdamped contact trial and unsuccessful control trajectories remain in internal validation records.

The short-duration model does not simulate evaporation, absorption into agar, microbial growth or colony counts. Physical grading uses the visible nonliving tracer distribution, not biological viability.


Damage monitoring also covers native contacts from all robot links, the lid and
the complete spreader, including its shaft. The base-root pair filter supplies
contact points, normals and signed penetration. Points on the upper gel disk
(within43.4mm radius, local height3–6.2mm and normal within60degrees of the
plate normal) with at least0.3mm penetration mark their1mm grading cell.
This avoids treating safe side-wall, rim or table-support contact as gel damage.
The native point witness supplements the T-bar footprint indentation model;
both remain approximations of visible gouges rather than calibrated gel tearing.
Sparse damaging contact events are archived and independently replayed. Buffer
overflow or nonfinite contact evidence terminates the run instead of omitting
contacts. No contact sensor or private pose is sent to Dream.
