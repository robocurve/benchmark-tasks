# Printable custom construction bricks

These are custom 16 mm-pitch bricks generated from `../geometry.py`, not
certified LEGO/Duplo parts. The STL files are in **millimetres**. Their origin is
the centre of the body, matching the SCAD and simulator geometry; a slicer can
translate the lowest point to its build plate.

| STL | Overall size, including studs | Material volume | Solid PLA estimate |
| --- | --- | --- | --- |
| `brick_1x1.stl` | 15.8 × 15.8 × 24.0 mm | 3.1511 cm³ | 3.907 g |
| `brick_2x2.stl` | 31.8 × 31.8 × 24.0 mm | 9.7665 cm³ | 12.110 g |

Mass estimates use an assumed density of **1.24 g/cm³** for all modelled
material. Actual sliced infill, supports, extrusion and material density affect
the printed mass. The models already contain their hollow sockets; these
estimates do not fill those cavities.

Start with the **open socket mouths toward the build plate and studs upward**.
The roof bridges the socket bores, so inspect a trial print for bridging and
mouth distortion before testing a pair. The nominal radial interference is
0.10 mm. Printer tolerances and real material elasticity determine whether
that produces a usable fit; insertion force and clutch/retention are uncalibrated.
The simulator's compliant contact parameters are not measured PLA properties.

In simulation the socket meshes remain rigid: spring-damper contact permits
overlap to approximate local compliance. It does not model plastic bending,
yielding or fracture. The 500 N/m contact coefficient is not a whole-brick
insertion stiffness or a material's Young's modulus. A watertight printable
shape and a successful simulated join therefore do not establish that a PLA
print of this geometry will press together or retain the same load. Treat these
as fit-test prototypes until a printed pair has been checked. See NVIDIA's
[compliant-contact model](https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/RigidBodyDynamics.html#compliant-contacts)
for the distinction between contact springs and deformable material simulation.

The exported surface is a Boolean union of the actual roof, walls, studs and
socket-sector primitives returned by `brick_geometry_specs()`. It is not a
collection of overlapping, disconnected shells. Studs have 96 facets, matching
the SCAD export; socket rings retain the source's 16 sectors. Before union,
coordinates are rounded to 10⁻⁸ mm solely to reconcile intended shared faces
that otherwise differ by floating-point arithmetic. No joining material or
fit adjustment is added.

Both written STL files were reimported and validated as one connected,
watertight, consistently wound, positive-volume manifold. Maximum bounding-box
error from STL float precision is below 0.000001 mm. The exact source/output
hashes, dependency versions and checks are in `stl_validation.json`.
This geometric validation does not establish printed strength, fit or clutch.

Regenerate from this task's directory (use your Python environment):

```bash
python -m pip install manifold3d==3.5.4 trimesh==5.2.0
python cad/export_stl.py
```

The exporter leaves the existing `.scad` sources unchanged. To create a
separate fit-test variant, use `--output-dir` and `--interference` (in metres),
then use the same interference value when generating its simulator geometry.
